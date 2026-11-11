#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <functional>
#include <limits>
#include <memory>
#include <stdexcept>
#include <vector>

#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>

using geometry_msgs::msg::Twist;
using nav_msgs::msg::Odometry;
using sensor_msgs::msg::LaserScan;

namespace {
constexpr double pi = 3.14159265358979323846;
constexpr double rad(double degrees) { return degrees * pi / 180.0; }
constexpr double padded_radius = 0.18 + 0.10;
constexpr double waypoint_step = 0.75;
constexpr double waypoint_tolerance = 0.08;
constexpr double release_distance = 0.75;
constexpr double stop_distance = 0.25;
constexpr double max_turn = rad(65);
constexpr double planning_margin = 0.04;
constexpr double infinity = std::numeric_limits<double>::infinity();

double wrap(double angle) { return std::atan2(std::sin(angle), std::cos(angle)); }

struct Point { double x, y; };

// Distance the center can travel before its padded disk touches a scan point.
// A positive curvature turns left. Distances are measured along the path.
double contact_distance(Point p, double curvature) {
  const double range_sq = p.x * p.x + p.y * p.y;
  if (range_sq <= padded_radius * padded_radius) return 0.0;
  if (std::abs(curvature) < 1e-6) {
    if (p.x <= 0.0 || std::abs(p.y) > padded_radius) return infinity;
    return p.x - std::sqrt(padded_radius * padded_radius - p.y * p.y);
  }

  // Reflect right turns into left turns. The center of rotation is at (0, r).
  const double r = 1.0 / std::abs(curvature);
  const double y = curvature > 0.0 ? p.y : -p.y;
  const double d = std::hypot(p.x, y - r);
  if (d == 0.0 || std::abs(d - r) > padded_radius) return infinity;
  const double nearest_angle = std::atan2(p.x, r - y);
  const double cosine = (r * r + d * d - padded_radius * padded_radius) / (2.0 * r * d);
  double entry = nearest_angle - std::acos(std::clamp(cosine, -1.0, 1.0));
  if (entry < 0.0) entry += 2.0 * pi;
  return r * entry;
}


struct CoverageBin {
  int total = 0;
  int usable = 0;
  bool covered() const { return usable > 0 && usable * 2 >= total; }
};
}  // namespace

class Lab3Node : public rclcpp::Node {
public:
  Lab3Node() : Node("lab3_node") {
    goal_distance_ = declare_parameter<double>("goal_distance", 1.5);
    goal_lateral_ = declare_parameter<double>("goal_lateral", 0.0);
    kv_ = declare_parameter<double>("kv", 0.5);
    kp_ = declare_parameter<double>("kp", 1.5);
    if (!std::isfinite(goal_distance_) || !std::isfinite(goal_lateral_) ||
        !std::isfinite(kv_) || kv_ <= 0.0 || !std::isfinite(kp_) || kp_ <= 0.0)
      throw std::invalid_argument("Goal offsets must be finite and gains finite and positive");
    using std::placeholders::_1;
    sub_odometry_ = create_subscription<Odometry>("/TTB06/odom", rclcpp::SensorDataQoS(),
        std::bind(&Lab3Node::odometry_callback, this, _1));
    sub_laser_scan_ = create_subscription<LaserScan>("/TTB06/scan", rclcpp::SensorDataQoS(),
        std::bind(&Lab3Node::laser_scan_callback, this, _1));
    pub_twist_ = create_publisher<Twist>("/TTB06/cmd_vel_unstamped", 10);
  }

private:
  rclcpp::Subscription<Odometry>::SharedPtr sub_odometry_;
  rclcpp::Subscription<LaserScan>::SharedPtr sub_laser_scan_;
  rclcpp::Publisher<Twist>::SharedPtr pub_twist_;
  Point pos_{}, goal_{}, waypoint_{};
  double goal_distance_, goal_lateral_, kv_, kp_;
  double current_angle_ = 0.0;
  bool goal_initialized_ = false;
  bool goal_reached_ = false;
  bool odom_received_ = false;
  bool scan_received_ = false;
  std::vector<Point> obstacles_;
  std::array<CoverageBin, 72> coverage_{};  // 5-degree sectors from -180 to +180
  bool too_close_ = false;
  double sensor_horizon_ = 0.0;
  double speed_command_ = 0.0, steering_command_ = 0.0;
  int avoidance_side_ = 0;  // A committed side relative to the direction to the final goal.
  bool following_waypoint_ = false;
  int clear_scans_ = 0;

  void stop() {
    speed_command_ = 0.0;
    steering_command_ = 0.0;
    pub_twist_->publish(Twist{});
  }

  double path_clearance(double curvature) const {
    double clearance = infinity;
    for (Point p : obstacles_) {
      clearance = std::min(clearance, contact_distance(p, curvature));
    }
    return clearance;
  }

  // Cover the forward half of the footprint and the directions it sweeps
  // through during the requested arc. Only the leading half gains new area.
  bool path_covered(double curvature, double travel, double bearing = 0.0) const {
    const double sweep = std::clamp(curvature * travel, -2.0 * pi, 2.0 * pi);
    const double low = bearing + std::min(0.0, sweep) - pi / 2.0;
    const double high = bearing + std::max(0.0, sweep) + pi / 2.0;
    const double width = 2.0 * pi / coverage_.size();
    for (std::size_t i = 0; i < coverage_.size(); ++i) {
      if (coverage_[i].covered()) continue;
      const double start = -pi + i * width;
      for (int shift = -1; shift <= 1; ++shift) {
        const double a = start + shift * 2.0 * pi;
        if (a < high - 1e-9 && a + width > low + 1e-9) return false;
      }
    }
    return true;
  }

  void odometry_callback(const Odometry &msg) {
    const auto &pose = msg.pose.pose;
    const auto &q = pose.orientation;
    const double norm_sq = q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w;
    if (!std::isfinite(pose.position.x) || !std::isfinite(pose.position.y) ||
        !std::isfinite(norm_sq) || norm_sq < 1e-12) return;
    const double yaw = std::atan2(2.0 * (q.w*q.z + q.x*q.y),
                                  norm_sq - 2.0 * (q.y*q.y + q.z*q.z));
    pos_.x = pose.position.x;
    pos_.y = pose.position.y;
    current_angle_ = yaw;
    odom_received_ = true;
    if (!goal_initialized_) {
      goal_.x = pos_.x + goal_distance_ * std::cos(yaw) - goal_lateral_ * std::sin(yaw);
      goal_.y = pos_.y + goal_distance_ * std::sin(yaw) + goal_lateral_ * std::cos(yaw);
      goal_initialized_ = true;
      RCLCPP_INFO(get_logger(), "Start=(%.2f %.2f) goal=(%.2f %.2f)",
                  pos_.x, pos_.y, goal_.x, goal_.y);
    }
    if (std::hypot(goal_.x - pos_.x, goal_.y - pos_.y) < 0.05) goal_reached_ = true;
  }

  void laser_scan_callback(const LaserScan &msg) {
    obstacles_.clear();
    coverage_.fill({});
    too_close_ = false;
    // +Inf only certifies open space as far as the sensor can measure.
    if (msg.ranges.empty() || !std::isfinite(msg.angle_min) ||
        !std::isfinite(msg.angle_increment) || msg.angle_increment == 0.0f ||
        !std::isfinite(msg.range_min) || msg.range_min < 0.0f ||
        !std::isfinite(msg.range_max) || msg.range_max <= msg.range_min ||
        msg.range_max < padded_radius + release_distance) {
      scan_received_ = false;
      stop();
      return;
    }

    sensor_horizon_ = msg.range_max - padded_radius - planning_margin;
    for (std::size_t i = 0; i < msg.ranges.size(); ++i) {
      // As in the original node, the scanner is treated as centered and forward-aligned.
      const double angle = wrap(msg.angle_min + static_cast<double>(i) * msg.angle_increment);
      const double range = msg.ranges[i];
      const int index = std::clamp(static_cast<int>((angle + pi) / rad(5)), 0, 71);
      auto &bin = coverage_[index];
      ++bin.total;
      if (range == -infinity) {
        too_close_ = true;
        continue;
      }
      const bool open = range == infinity;
      if (!open && (!std::isfinite(range) || range < msg.range_min || range > msg.range_max))
        continue;
      ++bin.usable;
      if (!open) obstacles_.push_back({range * std::cos(angle), range * std::sin(angle)});
    }
    scan_received_ = true;
    command_loop_function();
  }

  double distance_to(Point target) const {
    return std::hypot(target.x - pos_.x, target.y - pos_.y);
  }

  double bearing_to(Point target) const {
    return wrap(std::atan2(target.y - pos_.y, target.x - pos_.x) - current_angle_);
  }

  // Minimum distance between a scan point and the finite robot-center segment.
  // The endpoint matters: an obstacle well beyond the target cannot block it.
  double segment_gap(double bearing, double length) const {
    const double c = std::cos(bearing), s = std::sin(bearing);
    double best_sq = infinity;
    for (Point p : obstacles_) {
      const double along = std::clamp(p.x*c + p.y*s, 0.0, length);
      const double dx = p.x - along*c, dy = p.y - along*s;
      best_sq = std::min(best_sq, dx*dx + dy*dy);
    }
    return std::sqrt(best_sq) - padded_radius;
  }

  bool select_waypoint(double goal_bearing, double goal_range) {
    // Search for an observed detour on the committed side. Prefer the smallest
    // departure from the goal direction; for equal progress prefer more clearance.
    const double step = std::min({waypoint_step, sensor_horizon_, goal_range});
    for (int degrees = 10; degrees <= 90; degrees += 5) {
      int best_side = 0;
      double best_bearing = 0.0, best_gap = planning_margin;
      for (int side : {1, -1}) {
        if (avoidance_side_ != 0 && side != avoidance_side_) continue;
        const double bearing = wrap(goal_bearing + side * rad(degrees));
        if (std::abs(bearing) > max_turn || !path_covered(0.0, step, bearing)) continue;
        const double gap = segment_gap(bearing, step);
        if (gap > best_gap) {
          best_gap = gap;
          best_side = side;
          best_bearing = bearing;
        }
      }
      if (best_side != 0) {
        const double heading = current_angle_ + best_bearing;
        waypoint_ = {pos_.x + step * std::cos(heading), pos_.y + step * std::sin(heading)};
        avoidance_side_ = best_side;
        following_waypoint_ = true;
        RCLCPP_INFO(get_logger(), "Detour side=%d waypoint=(%.2f %.2f)",
                    best_side, waypoint_.x, waypoint_.y);
        return true;
      }
    }
    return false;
  }

  // One iteration of TTBLab2Node::execute_go_to_goal, using its pid=false
  // distance branch. Lab 3 applies motion limits and publishes the result.
  Twist go_to_goal(Point target) const {
    Twist cmd;
    cmd.linear.x = kv_ * distance_to(target);
    cmd.angular.z = kp_ * bearing_to(target);
    return cmd;
  }

  void command_loop_function() {
    if (goal_reached_ || !goal_initialized_ || !odom_received_ || !scan_received_ ||
        too_close_ || !path_covered(0.0, release_distance)) {
      clear_scans_ = 0;
      stop();
      return;
    }
    const double goal_range = distance_to(goal_);
    const double goal_bearing = bearing_to(goal_);
    const double lookahead = std::min(goal_range, sensor_horizon_);
    const bool direct_observed = path_covered(0.0, lookahead, goal_bearing);
    const double direct_gap = segment_gap(goal_bearing, lookahead);
    const bool direct_clear = direct_observed && direct_gap > 0.0;

    // Recheck the final-goal path once per laser scan.
    if (avoidance_side_ != 0 && direct_clear)
      clear_scans_ = std::min(clear_scans_ + 1, 3);
    else
      clear_scans_ = 0;
    if (clear_scans_ == 3) {
      following_waypoint_ = false;
      avoidance_side_ = 0;
      clear_scans_ = 0;
    }

    if (following_waypoint_) {
      const double remaining = distance_to(waypoint_);
      const double bearing = bearing_to(waypoint_);
      if (remaining <= waypoint_tolerance || !path_covered(0.0, remaining, bearing) ||
          segment_gap(bearing, remaining) <= 0.0)
        following_waypoint_ = false;
    }
    if (!following_waypoint_ && (avoidance_side_ != 0 || !direct_clear)) {
      // Wait for confirmation if the route has just reopened. A scan with
      // insufficient coverage alone is never evidence of an obstacle.
      if (!direct_observed || direct_clear || !select_waypoint(goal_bearing, goal_range)) {
        stop();
        return;
      }
    }

    const Point target = following_waypoint_ ? waypoint_ : goal_;
    const Twist desired = go_to_goal(target);
    const double heading_error = bearing_to(target);
    const double target_w = std::clamp(desired.angular.z, -0.9, 0.9);
    steering_command_ = 0.45 * target_w + 0.55 * steering_command_;

    // Pivot toward a new waypoint before translating to avoid cutting a corner.
    const double remaining = distance_to(target);
    const double travel = std::min(stop_distance, remaining);
    if (std::abs(heading_error) > rad(15) || path_clearance(0.0) <= travel) {
      speed_command_ = 0.0;
    } else {
      const double target_v = std::min({desired.linear.x, 0.15, 0.5 * goal_range});
      speed_command_ = std::min(0.40 * target_v + 0.60 * speed_command_, target_v);
      if (speed_command_ > 0.0) {
        const double curvature = steering_command_ / speed_command_;
        if (!path_covered(curvature, travel)) {
          stop();
          return;
        }
        if (path_clearance(curvature) <= travel) speed_command_ = 0.0;
      }
    }

    Twist cmd;
    cmd.linear.x = speed_command_;
    cmd.angular.z = steering_command_;
    pub_twist_->publish(cmd);
    RCLCPP_INFO(get_logger(), "target=%s dist=%.2f goal_gap=%.2f side=%d v=%.2f w=%.2f",
        following_waypoint_ ? "waypoint" : "goal", goal_range, direct_gap, avoidance_side_,
        cmd.linear.x, cmd.angular.z);
  }
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<Lab3Node>();
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}

