#include <cmath>
#include <array>
#include <chrono>
#include <functional>
#include <memory>
#include <algorithm>
#include <limits>
#include <vector>
#include <stdexcept>

#include <rclcpp/rclcpp.hpp>

#include <sensor_msgs/msg/laser_scan.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/vector3.hpp>

using namespace std::chrono_literals;
using namespace std::placeholders;

using namespace sensor_msgs::msg;
using namespace geometry_msgs::msg;
using namespace nav_msgs::msg;

class Lab3Node : public rclcpp::Node {
private:
  // ROS 2 Handles
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Subscription<Odometry>::SharedPtr sub_odometry_;
  rclcpp::Subscription<LaserScan>::SharedPtr sub_laser_scan_;
  rclcpp::Publisher<Twist>::SharedPtr pub_twist_;

  // Vehicle State (Relative Origin Frame)
  geometry_msgs::msg::Vector3 pos_;
  geometry_msgs::msg::Vector3 goal_;
  double current_angle_ = 0.0;

  bool odom_received_ = false;
  bool scan_received_ = false;

  // Origin Pose Offset
  double origin_x_ = 0.0;
  double origin_y_ = 0.0;
  double origin_yaw_ = 0.0;
  bool origin_rendered_ = false;

  struct Point { double x, y; };
  std::vector<Point> hits_;                  // LiDAR returns in robot coordinates
  struct Sector { int usable = 0, total = 0; };
  std::array<Sector, 36> coverage_{};        // 10-degree sectors around the robot
  using Clock = std::chrono::steady_clock;
  Clock::time_point last_scan_, last_odom_;
  bool too_close_ = false;
  int avoid_side_ = 0;                      // +1 left of the goal, -1 right
  bool have_waypoint_ = false;
  Point waypoint_{};                        // Fixed in the starting pose frame
  double scan_yaw_offset_ = 0.0;

  // Controller Tuning Parameters
  double robot_radius_ = 0.20;       // Physical radius buffer (m)
  double obstacle_threshold_ = 0.70;  // Distance threshold to trigger obstacle avoidance (m)
  double goal_tolerance_ = 0.10;     // Goal reach radius (m)

  double kg_ = 1.2;                  // Go-to-Goal heading gain
  double margin_ = 0.07;               // Additional lateral clearance (m)
  double max_vel_ = 0.25;            // Max linear speed (m/s)
  double max_ang_vel_ = M_PI / 4.0;  // Max angular speed (rad/s)

public:
  explicit Lab3Node() : rclcpp::Node("lab3_node") {

    scan_yaw_offset_ = declare_parameter<double>("scan_yaw_offset_deg", 0.0) * M_PI / 180.0;

    if (!std::isfinite(scan_yaw_offset_)) throw std::invalid_argument("scan_yaw_offset_deg must be finite");

    // Main control loop running at 10 Hz (100 ms)
    timer_ = this->create_wall_timer(
        100ms, std::bind(&Lab3Node::command_loop_function, this));

    // Subscriptions using sensor data QoS
    sub_odometry_ = this->create_subscription<Odometry>(
        "/TTB06/odom",
        rclcpp::SensorDataQoS(),
        std::bind(&Lab3Node::odometry_callback, this, _1));

    sub_laser_scan_ = this->create_subscription<LaserScan>(
        "/TTB06/scan",
        rclcpp::SensorDataQoS(),
        std::bind(&Lab3Node::laser_scan_callback, this, _1));

    // Command Velocity Publisher
    pub_twist_ = this->create_publisher<Twist>("/TTB06/cmd_vel", 10);

    // Initial goal position relative to start frame
    pos_.x = 0.0;
    pos_.y = 0.0;
    goal_.x = 1.5;
    goal_.y = 0.0;
  }

private:
  static double wrap(double a) { return std::atan2(std::sin(a), std::cos(a)); }

  static int sector_index(double a) {
    return std::clamp(static_cast<int>((wrap(a) + M_PI) * 18.0 / M_PI), 0, 35);
  }

  bool observed(double bearing) const {
    const int center = sector_index(bearing);
    for (int offset = -1; offset <= 1; ++offset) {
      const auto &bin = coverage_[(center + offset + 36) % 36];
      if (bin.usable == 0 || 2 * bin.usable < bin.total) return false;
    }
    return true;
  }

  // Smallest centerline-to-hit distance along a finite straight segment.
  double segment_distance(double bearing, double length) const {
    const double ux = std::cos(bearing), uy = std::sin(bearing);
    double best_sq = std::numeric_limits<double>::infinity();
    for (const auto &p : hits_) {
      const double along = std::clamp(p.x * ux + p.y * uy, 0.0, length);
      const double dx = p.x - along * ux, dy = p.y - along * uy;
      best_sq = std::min(best_sq, dx * dx + dy * dy);
    }
    return std::sqrt(best_sq);
  }

  bool choose_detour(double goal_error, double length, int allowed_side) {
    // Search increasing departures from the goal. At equal angles choose
    // the side with more measured room; retain that choice across scans.
    const double radius = robot_radius_ + margin_;
    // Every segment includes the current pose. If already within the extra
    // planning margin, allow a departure that maintains current clearance.
    const double minimum_gap = std::clamp(segment_distance(0.0, 0.0) - 1e-6, radius, radius + 0.03);
    for (int angle = 10; angle <= 85; angle += 5) {
      int chosen = 0;
      double best_gap = minimum_gap, best_heading = 0.0;
      for (int side : {1, -1}) {
        if (allowed_side != 0 && allowed_side != side) continue;
        const double h = wrap(goal_error + side * angle * M_PI / 180.0);
        if (std::abs(h) > M_PI / 2 || !observed(h)) continue;
        const double gap = segment_distance(h, length);
        if (gap > best_gap) {
          chosen = side;
          best_gap = gap;
          best_heading = h;
        }
      }
      if (chosen) {
        const double yaw = current_angle_ + best_heading;
        waypoint_ = {pos_.x + length * std::cos(yaw), pos_.y + length * std::sin(yaw)};
        avoid_side_ = chosen;
        have_waypoint_ = true;
        return true;
      }
    }
    return false;
  }

  void stop(const char *reason) {
    pub_twist_->publish(Twist{});
    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000, "Stopped: %s", reason);
  }

  void command_loop_function() {
    const auto now = Clock::now();
    if (!odom_received_ || now - last_odom_ > 500ms) return stop("waiting for fresh odometry");
    if (!scan_received_ || now - last_scan_ > 500ms) return stop("waiting for a usable LiDAR scan");
    if (too_close_) return stop("LiDAR reports a point inside the robot footprint");
    if (!observed(0.0)) return stop("insufficient forward LiDAR coverage");

    const double dx = goal_.x - pos_.x, dy = goal_.y - pos_.y;
    const double distance = std::hypot(dx, dy);
    if (distance <= goal_tolerance_) return stop("goal reached");
    const double goal_error = wrap(std::atan2(dy, dx) - current_angle_);
    const double lookahead = std::min(distance, obstacle_threshold_);
    const double radius = robot_radius_ + margin_;
    // Extra clearance before leaving avoidance prevents chatter at a corner.
    if (observed(goal_error) &&
        segment_distance(goal_error, lookahead) > radius + (avoid_side_ ? 0.04 : 0.0)) {
      have_waypoint_ = false;
      avoid_side_ = 0;
    } else {
      if (have_waypoint_) {
        const double wx = waypoint_.x - pos_.x, wy = waypoint_.y - pos_.y;
        const double remain = std::hypot(wx, wy);
        const double bearing = wrap(std::atan2(wy, wx) - current_angle_);
        have_waypoint_ = remain > 0.08 && observed(bearing) &&
                         segment_distance(bearing, remain) > radius;
      }
      if (!have_waypoint_) {
        for (double length : {lookahead, std::min(0.35, lookahead)}) {
          if (choose_detour(goal_error, length, avoid_side_) ||
              (avoid_side_ && choose_detour(goal_error, length, -avoid_side_))) break;
          if (length <= 0.35) break;
        }
        if (!have_waypoint_) return stop("no observed bypass with enough clearance");
      }
    }

    const double heading = have_waypoint_
        ? wrap(std::atan2(waypoint_.y-pos_.y, waypoint_.x-pos_.x) - current_angle_)
        : goal_error;
    Twist cmd;
    cmd.angular.z = std::clamp(kg_ * heading, -max_ang_vel_, max_ang_vel_);
    // Turn in place until aligned. While translating, drive straight and
    // re-evaluate at 10 Hz so the checked segment matches the command.
    const double travel = std::min(0.20, distance);
    if (std::abs(heading) <= 8.0 * M_PI / 180.0 &&
        segment_distance(0.0, travel) > radius) {
      cmd.angular.z = 0.0;
      cmd.linear.x = std::min(0.6 * distance, have_waypoint_ ? 0.10 : max_vel_);
    }
    pub_twist_->publish(cmd);
    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 500,
        "target=%s side=%d heading=%.0fdeg v=%.2f w=%.2f",
        have_waypoint_ ? "bypass" : "goal", avoid_side_, heading * 180.0 / M_PI,
        cmd.linear.x, cmd.angular.z);
  }

  void odometry_callback(const Odometry &msg) {
    double raw_x = msg.pose.pose.position.x;
    double raw_y = msg.pose.pose.position.y;

    // Quaternion Yaw Angle Extraction
    double qx = msg.pose.pose.orientation.x;
    double qy = msg.pose.pose.orientation.y;
    double qz = msg.pose.pose.orientation.z;
    double qw = msg.pose.pose.orientation.w;
    double raw_yaw = std::atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz));

    if (!std::isfinite(raw_x) || !std::isfinite(raw_y) || !std::isfinite(raw_yaw) ||
        !(qx*qx + qy*qy + qz*qz + qw*qw > 1e-12)) {
      odom_received_ = false;
      return;
    }
    // Lock initial pose as origin (0,0,0)
    if (!origin_rendered_) {
      origin_x_ = raw_x;
      origin_y_ = raw_y;
      origin_yaw_ = raw_yaw;
      origin_rendered_ = true;
    }

    // Transform position relative to starting origin
    double dx = raw_x - origin_x_;
    double dy = raw_y - origin_y_;
    pos_.x = dx * std::cos(origin_yaw_) + dy * std::sin(origin_yaw_);
    pos_.y = -dx * std::sin(origin_yaw_) + dy * std::cos(origin_yaw_);
    current_angle_ = std::atan2(std::sin(raw_yaw - origin_yaw_), std::cos(raw_yaw - origin_yaw_));

    last_odom_ = Clock::now();
    odom_received_ = true;
  }

  void laser_scan_callback(const LaserScan &msg) {
    scan_received_ = false;
    hits_.clear();
    coverage_.fill({});
    too_close_ = false;
    if (msg.ranges.empty() || !std::isfinite(msg.angle_min) ||
        !std::isfinite(msg.angle_increment) || msg.angle_increment == 0 ||
        !std::isfinite(msg.range_min) || msg.range_min < 0 ||
        !std::isfinite(msg.range_max) || msg.range_max <= msg.range_min ||
        msg.range_max < obstacle_threshold_ + robot_radius_ + margin_) return;
    double front_range = std::numeric_limits<double>::infinity();
    for (size_t i = 0; i < msg.ranges.size(); ++i) {
      const double r = msg.ranges[i];
      const double a = msg.angle_min + i * static_cast<double>(msg.angle_increment) + scan_yaw_offset_;
      auto &bin = coverage_[sector_index(a)];
      ++bin.total;
      if (r == -std::numeric_limits<double>::infinity()) { too_close_ = true; continue; }
      if (r == std::numeric_limits<double>::infinity()) { ++bin.usable; continue; }
      if (!std::isfinite(r) || r < msg.range_min || r > msg.range_max) continue;
      ++bin.usable;
      if (std::abs(wrap(a)) <= 15.0 * M_PI / 180.0) front_range = std::min(front_range, r);
      hits_.push_back({r * std::cos(a), r * std::sin(a)});
      if (r <= robot_radius_) too_close_ = true;
    }
    last_scan_ = Clock::now();
    scan_received_ = true;
    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000,
        "LiDAR front +/-15 deg: %.2f m (scanner range)", front_range);
  }
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);

  auto control_node = std::make_shared<Lab3Node>();
  rclcpp::executors::SingleThreadedExecutor exec{};
  exec.add_node(control_node);
  exec.spin();

  rclcpp::shutdown();
  return 0;
}
