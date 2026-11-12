#include <algorithm>
#include <cmath>
#include <functional>
#include <memory>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <rmw/qos_profiles.h>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/twist.hpp>

using namespace std::chrono_literals;
using namespace std::placeholders;
using namespace sensor_msgs::msg;
using namespace geometry_msgs::msg;
using namespace nav_msgs::msg;

static double wrap(double a) { return std::atan2(std::sin(a), std::cos(a)); }

class Lab3Node : public rclcpp::Node {
private:
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Subscription<Odometry>::SharedPtr sub_odometry_;
  rclcpp::Subscription<LaserScan>::SharedPtr sub_laser_scan_;
  rclcpp::Publisher<Twist>::SharedPtr pub_twist_;

  // Pose in the start-relative frame
  double x_ = 0.0, y_ = 0.0, yaw_ = 0.0;
  double goal_x_ = 1.5, goal_y_ = 0.0;
  double goal_tolerance_ = 0.10;

  double origin_x_ = 0.0, origin_y_ = 0.0, origin_yaw_ = 0.0;
  bool origin_set_ = false;

  // Scan data (robot frame)
  std::vector<double> angles_;
  std::vector<double> clearance_;   // windowed min range per beam
  double front_clear_ = 0.0;        // min range in the cone dead ahead
  bool have_scan_ = false;

  // State machine
  enum class Mode { DRIVE, ROTATE };
  Mode mode_ = Mode::DRIVE;
  double turn_dir_ = 0.0;           // committed spin direction (+1 left, -1 right)

  // Tunables
  double max_range_ = 3.0;
  double max_vel_ = 0.3;
  double min_vel_ = 0.08;           // always creep forward while driving
  double max_ang_vel_ = M_PI / 2;
  double rot_vel_ = M_PI / 3;       // in-place spin speed
  double window_half_angle_ = 0.40; // rad, how wide a gap must be (robot width)
  double front_half_angle_ = 0.35;  // rad, emergency cone
  double stop_dist_ = 0.35;         // m, enter ROTATE below this
  double resume_dist_ = 0.60;       // m, front must clear this to leave ROTATE
  double open_dist_ = 0.80;         // m, clearance needed to be a rotate target
  double slow_dist_ = 1.00;         // m, start slowing below this
  double w_goal_ = 1.0, w_clear_ = 1.5, w_prev_ = 0.3;
  double k_turn_ = 1.5;
  double prev_angle_ = 0.0;

public:
  Lab3Node() : rclcpp::Node("lab3_node") {
    timer_ = create_wall_timer(20ms, std::bind(&Lab3Node::command_loop, this));

    auto qos = rclcpp::QoS(
        rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_sensor_data),
        rmw_qos_profile_sensor_data);

    sub_odometry_ = create_subscription<Odometry>(
        "/TTB06/odom", qos, std::bind(&Lab3Node::odometry_callback, this, _1));
    sub_laser_scan_ = create_subscription<LaserScan>(
        "/TTB06/scan", qos, std::bind(&Lab3Node::laser_scan_callback, this, _1));
    pub_twist_ = create_publisher<Twist>("/TTB06/cmd_vel", 10);
  }

  void command_loop() {
    Twist cmd;  // zeros by default = stop
    if (!origin_set_ || !have_scan_) { pub_twist_->publish(cmd); return; }

    double dist = std::hypot(goal_x_ - x_, goal_y_ - y_);
    if (dist < goal_tolerance_) { pub_twist_->publish(cmd); return; }

    // Goal direction relative to the robot (same frame as the laser angles)
    double goal_rel = wrap(std::atan2(goal_y_ - y_, goal_x_ - x_) - yaw_);

    if (mode_ == Mode::DRIVE && front_clear_ < stop_dist_) {
      mode_ = Mode::ROTATE;
      turn_dir_ = 0.0;
    }

    if (mode_ == Mode::ROTATE) {
      // Target = open direction closest to the goal heading
      double target = 0.0, best_err = 1e9;
      bool found = false;
      for (size_t i = 0; i < angles_.size(); ++i) {
        if (clearance_[i] < open_dist_) continue;
        double err = std::fabs(wrap(angles_[i] - goal_rel));
        if (err < best_err) { best_err = err; target = angles_[i]; found = true; }
      }
      if (!found) {  // boxed in: spin toward the most open direction
        double best_clear = -1.0;
        for (size_t i = 0; i < angles_.size(); ++i)
          if (clearance_[i] > best_clear) { best_clear = clearance_[i]; target = angles_[i]; }
      }

      // Commit to a spin direction when the target is nearly behind us
      double w;
      if (std::fabs(target) > 2.6) {
        if (turn_dir_ == 0.0) turn_dir_ = (target >= 0.0) ? 1.0 : -1.0;
        w = turn_dir_ * rot_vel_;
      } else {
        turn_dir_ = 0.0;
        w = std::clamp(k_turn_ * target, -rot_vel_, rot_vel_);
      }

      if (front_clear_ > resume_dist_ && std::fabs(target) < 0.35) {
        mode_ = Mode::DRIVE;  // facing open space toward the goal: go
      } else {
        cmd.angular.z = w;    // linear stays 0
        pub_twist_->publish(cmd);
        return;
      }
    }

    // DRIVE: goal-weighted direction choice, always moving forward
    double best_cost = 1e9, best_angle = 0.0;
    for (size_t i = 0; i < angles_.size(); ++i) {
      double a = angles_[i];
      double cost = w_goal_ * std::fabs(wrap(a - goal_rel))
                  + w_clear_ * (max_range_ - clearance_[i]) / max_range_ * M_PI
                  + w_prev_ * std::fabs(wrap(a - prev_angle_));
      if (cost < best_cost) { best_cost = cost; best_angle = a; }
    }
    prev_angle_ = best_angle;

    double w = std::clamp(k_turn_ * best_angle, -max_ang_vel_, max_ang_vel_);
    double speed_scale = std::clamp(
        (front_clear_ - stop_dist_) / (slow_dist_ - stop_dist_), 0.0, 1.0);
    double align = std::max(0.3, std::cos(best_angle));  // floor so it never stalls
    double v = std::max(min_vel_, max_vel_ * speed_scale * align);
    v = std::min(v, dist);  // don't overshoot the goal

    cmd.linear.x = v;
    cmd.angular.z = w;
    pub_twist_->publish(cmd);
  }

  void odometry_callback(const Odometry &msg) {
    double px = msg.pose.pose.position.x;
    double py = msg.pose.pose.position.y;
    const auto &q = msg.pose.pose.orientation;
    double yaw = std::atan2(2.0 * (q.w * q.z + q.x * q.y),
                            1.0 - 2.0 * (q.y * q.y + q.z * q.z));

    if (!origin_set_) {
      origin_x_ = px; origin_y_ = py; origin_yaw_ = yaw;
      origin_set_ = true;
    }
    double dx = px - origin_x_, dy = py - origin_y_;
    x_ =  dx * std::cos(origin_yaw_) + dy * std::sin(origin_yaw_);
    y_ = -dx * std::sin(origin_yaw_) + dy * std::cos(origin_yaw_);
    yaw_ = wrap(yaw - origin_yaw_);
  }

  void laser_scan_callback(const LaserScan &msg) {
    const size_t n = msg.ranges.size();
    if (n == 0) return;

    std::vector<double> r(n);
    std::vector<double> ang(n);
    for (size_t i = 0; i < n; ++i) {
      double range = msg.ranges[i];
      // inf = no return (free); nan/below range_min = invalid, treat as free
      if (!std::isfinite(range) || range < msg.range_min) range = max_range_;
      r[i] = std::min<double>(range, max_range_);
      ang[i] = wrap(msg.angle_min + msg.angle_increment * i);
    }

    // Windowed min: a direction is only "open" if the robot fits through it
    std::vector<double> clr(n, max_range_);
    double front = max_range_;
    for (size_t i = 0; i < n; ++i) {
      double m = max_range_;
      for (size_t j = 0; j < n; ++j) {
        if (std::fabs(wrap(ang[j] - ang[i])) <= window_half_angle_)
          m = std::min(m, r[j]);
      }
      clr[i] = m;
      if (std::fabs(ang[i]) <= front_half_angle_)
        front = std::min(front, r[i]);
    }

    angles_ = std::move(ang);
    clearance_ = std::move(clr);
    front_clear_ = front;
    have_scan_ = true;
  }
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<Lab3Node>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
