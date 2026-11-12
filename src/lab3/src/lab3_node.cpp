#include <algorithm>
#include <cmath>
#include <functional>
#include <memory>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/executors/single_threaded_executor.hpp>
#include <rclcpp/qos.hpp>

#include <rmw/qos_profiles.h>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/vector3.hpp>

using namespace std::chrono_literals;
using namespace std::placeholders;

using namespace sensor_msgs::msg;
using namespace geometry_msgs::msg;
using namespace nav_msgs::msg;

// One lidar return in the robot body frame
struct ScanPoint {
  double angle;  // rad, 0 = robot forward, +CCW
  double range;  // m
};

class Lab3Node : public rclcpp::Node {
private:
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Subscription<Odometry>::SharedPtr sub_odometry_;
  rclcpp::Subscription<LaserScan>::SharedPtr sub_laser_scan_;
  rclcpp::Publisher<Twist>::SharedPtr pub_twist_;

  // Vehicle state (origin frame: +x = heading at start, same convention as lab2)
  Vector3 pos_;
  Vector3 goal_;
  double current_angle_ = 0.0;
  bool odom_received_ = false;
  bool scan_received_ = false;

  double origin_x_ = 0.0;
  double origin_y_ = 0.0;
  double origin_yaw_ = 0.0;
  bool origin_set_ = false;

  // Live lidar data only
  std::vector<ScanPoint> obstacles_;
  bool lidar_ok_ = false;            // scan had at least one non-NaN ray (all-inf = open space, still OK)
  rclcpp::Time last_scan_time_{0, 0, RCL_ROS_TIME};

  // Geometry / safety
  double goal_distance_ = 10.0;      // goal: this far ahead of the start pose (m)
  double goal_margin_ = 0.10;        // lab2 "margin"
  double robot_radius_ = 0.20;       // disk abstraction (m)
  double side_margin_ = 0.03;        // sideways margin on top of the radius for the travel tube (m)
  double front_buffer_ = 0.10;       // stop/back-off distance beyond the radius, straight ahead (m)
  double clear_distance_ = 1.0;      // a heading is "open" if this much room is free along it (m)
  double lookahead_ = 2.0;           // lidar distance considered (m)
  double lidar_yaw_offset_ = 0.0;    // rad; set if scan angle 0 is NOT the robot's front

  // Controller (lab2 style)
  double kv_ = 0.5;                  // velocity gain on distance
  double kp_ = 1.2;                  // heading gain
  double max_vel_ = 0.35;            // m/s
  double max_ang_vel_ = M_PI / 3.0;  // rad/s
  double reverse_vel_ = 0.10;        // m/s, used only when an obstacle is inside the stop distance

  double wrap(double angle) {
    return std::atan2(std::sin(angle), std::cos(angle));
  }

public:
  explicit Lab3Node() : rclcpp::Node("lab3_node") {
    timer_ = this->create_wall_timer(
        100ms, std::bind(&Lab3Node::command_loop_function, this));

    sub_odometry_ = this->create_subscription<Odometry>(
        "/TTB06/odom",
        rclcpp::QoS(
            rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_sensor_data),
            rmw_qos_profile_sensor_data),
        std::bind(&Lab3Node::odometry_callback, this, _1));

    sub_laser_scan_ = this->create_subscription<LaserScan>(
        "/TTB06/scan",
        rclcpp::QoS(
            rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_sensor_data),
            rmw_qos_profile_sensor_data),
        std::bind(&Lab3Node::laser_scan_callback, this, _1));

    pub_twist_ = this->create_publisher<Twist>("/TTB06/cmd_vel", 10);

    pos_.x = 0.0;
    pos_.y = 0.0;
    goal_.x = goal_distance_;
    goal_.y = 0.0;
  }

private:
  // How far the robot (a tube of half-width half_width) can travel along `heading`
  // before an obstacle point is inside the tube. Capped at lookahead_.
  double free_distance(double heading, double half_width) {
    double free = lookahead_;
    for (const auto &p : obstacles_) {
      double rel = wrap(p.angle - heading);
      double forward = p.range * std::cos(rel);
      double lateral = std::abs(p.range * std::sin(rel));
      if (forward > 0.0 && lateral < half_width && forward < free) {
        free = forward;
      }
    }
    return free;
  }

  // Heading (body frame) for when the goal direction is blocked: of all headings that are open
  // (>= clear_distance_ free), take the one closest to the goal direction. If none is open,
  // take the heading with the most free room.
  double choose_heading(double goal_heading_body, double half_width) {
    double open_heading = goal_heading_body, open_err = 1e9;
    double best_heading = goal_heading_body, best_free = -1.0;

    for (double h = -M_PI / 2.0; h <= M_PI / 2.0 + 1e-9; h += 5.0 * M_PI / 180.0) {
      double free = free_distance(h, half_width);
      double err = std::abs(wrap(h - goal_heading_body));

      if (free >= clear_distance_ && err < open_err) {
        open_err = err;
        open_heading = h;
      }
      if (free > best_free + 1e-6) {
        best_free = free;
        best_heading = h;
      }
    }
    return (open_err < 1e9) ? open_heading : best_heading;
  }

  void command_loop_function() {
    if (!odom_received_ || !scan_received_) {
      return;
    }

    // Never drive on stale or empty lidar data
    if ((this->now() - last_scan_time_).seconds() > 0.5 || !lidar_ok_) {
      pub_twist_->publish(Twist());
      return;
    }

    double half_width = robot_radius_ + side_margin_;     // travel tube: only what the body would hit
    double stop_distance = robot_radius_ + front_buffer_; // never drive forward inside this

    // --- go_to_goal (lab2) ---
    double distance = std::hypot(goal_.x - pos_.x, goal_.y - pos_.y);
    if (distance < goal_margin_) {
      pub_twist_->publish(Twist());
      RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "Goal Reached!");
      return;
    }

    double goal_heading_body = wrap(std::atan2(goal_.y - pos_.y, goal_.x - pos_.x) - current_angle_);

    // --- obstacle avoidance: steer toward the clearest heading ---
    // If the path to the goal is open, go straight to the goal; otherwise use the clearest heading.
    double heading_error = goal_heading_body;
    if (free_distance(goal_heading_body, half_width) < std::min(distance, clear_distance_)) {
      heading_error = choose_heading(goal_heading_body, half_width);
    }
    double steering = kp_ * heading_error;

    // --- velocity: kv * distance, limited by the free space straight ahead ---
    double ahead = free_distance(0.0, half_width);
    double velocity = 0.0;
    if (ahead > stop_distance) {
      velocity = std::clamp(kv_ * std::min(distance, ahead - stop_distance), 0.0, max_vel_);
      velocity *= std::max(0.2, std::cos(heading_error));   // slow in sharp turns, keep arcing
    } else if (free_distance(M_PI, robot_radius_) > 0.30) {
      velocity = -reverse_vel_;                             // too close: back away while steering
    }                                                       // else: stay still, never drive into it

    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
        "pos(%.2f, %.2f) goal_dist %.2f | goal_hdg %.0f deg, chosen %.0f deg | ahead_free %.2f | v %.2f",
        pos_.x, pos_.y, distance, goal_heading_body * 180.0 / M_PI, heading_error * 180.0 / M_PI,
        ahead, velocity);

    Twist cmd;
    cmd.linear.x = velocity;
    cmd.angular.z = std::clamp(steering, -max_ang_vel_, max_ang_vel_);
    pub_twist_->publish(cmd);
  }

  void odometry_callback(const Odometry &msg) {
    double raw_x = msg.pose.pose.position.x;
    double raw_y = msg.pose.pose.position.y;

    double qx = msg.pose.pose.orientation.x;
    double qy = msg.pose.pose.orientation.y;
    double qz = msg.pose.pose.orientation.z;
    double qw = msg.pose.pose.orientation.w;
    double raw_yaw = std::atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz));

    // First message defines the start frame (+x = start heading). Goal = (goal_distance_, 0) in it,
    // i.e. exactly goal_distance_ metres straight ahead of the start position.
    if (!origin_set_) {
      origin_x_ = raw_x;
      origin_y_ = raw_y;
      origin_yaw_ = raw_yaw;
      origin_set_ = true;
      goal_.x = goal_distance_;
      goal_.y = 0.0;
      RCLCPP_INFO(this->get_logger(), "Start (%.2f, %.2f, yaw %.0f deg). Goal = %.1f m ahead -> odom (%.2f, %.2f)",
                  raw_x, raw_y, raw_yaw * 180.0 / M_PI, goal_distance_,
                  raw_x + goal_distance_ * std::cos(raw_yaw), raw_y + goal_distance_ * std::sin(raw_yaw));
    }

    double dx = raw_x - origin_x_;
    double dy = raw_y - origin_y_;
    pos_.x = dx * std::cos(origin_yaw_) + dy * std::sin(origin_yaw_);
    pos_.y = -dx * std::sin(origin_yaw_) + dy * std::cos(origin_yaw_);
    current_angle_ = wrap(raw_yaw - origin_yaw_);

    odom_received_ = true;
  }

  void laser_scan_callback(const LaserScan &msg) {
    obstacles_.clear();
    lidar_ok_ = false;
    for (size_t i = 0; i < msg.ranges.size(); ++i) {
      double r = msg.ranges[i];
      if (!std::isnan(r)) {
        lidar_ok_ = true;
      }
      // NaN / inf / out-of-range = no obstacle (inf is open space, not an error)
      if (std::isnan(r) || std::isinf(r) || r < msg.range_min || r >= msg.range_max) {
        continue;
      }
      double angle = wrap(msg.angle_min + static_cast<double>(i) * msg.angle_increment + lidar_yaw_offset_);
      obstacles_.push_back({angle, r});
    }
    last_scan_time_ = this->now();
    scan_received_ = true;
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
