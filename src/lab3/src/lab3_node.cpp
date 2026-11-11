#include <cmath>
#include <cstdlib>
#include <functional>
#include <memory>
#include <algorithm>
#include <limits>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/create_timer.hpp>
#include <rclcpp/executors.hpp>
#include <rclcpp/executors/single_threaded_executor.hpp>
#include <rclcpp/publisher_base.hpp>
#include <rclcpp/qos.hpp>
#include <rclcpp/subscription.hpp>
#include <rclcpp/utilities.hpp>

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

  // Obstacle Telemetry
  double min_obstacle_dist_ = std::numeric_limits<double>::infinity();
  double min_obstacle_angle_ = 0.0;

  // Controller Tuning Parameters
  double robot_radius_ = 0.20;       // Physical radius buffer (m)
  double obstacle_threshold_ = 0.70;  // Distance threshold to trigger obstacle avoidance (m)
  double goal_tolerance_ = 0.10;     // Goal reach radius (m)

  double kg_ = 1.2;                  // Go-to-Goal heading gain
  double ka_ = 0.8;                  // Reactive obstacle steering gain
  double max_vel_ = 0.25;            // Max linear speed (m/s)
  double max_ang_vel_ = M_PI / 4.0;  // Max angular speed (rad/s)

public:
  explicit Lab3Node() : rclcpp::Node("lab3_node") {

    // Main control loop running at 10 Hz (100 ms)
    timer_ = this->create_wall_timer(
        100ms, std::bind(&Lab3Node::command_loop_function, this));

    // Subscriptions using sensor data QoS
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

    // Command Velocity Publisher
    pub_twist_ = this->create_publisher<Twist>("/TTB06/cmd_vel", 10);

    // Initial goal position relative to start frame
    pos_.x = 0.0;
    pos_.y = 0.0;
    goal_.x = 1.5;
    goal_.y = 0.0;
  }

private:
  void command_loop_function() {
    if (!odom_received_ || !scan_received_) {
      return;
    }

    Twist cmd;
    double dist_to_goal = std::hypot(goal_.x - pos_.x, goal_.y - pos_.y);

    // 1. Goal Reached Check
    if (dist_to_goal <= goal_tolerance_) {
      cmd.linear.x = 0.0;
      cmd.angular.z = 0.0;
      pub_twist_->publish(cmd);
      RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "Goal Reached!");
      return;
    }

    // 2. Go-To-Goal Heading Error Calculation
    double goal_heading = std::atan2(goal_.y - pos_.y, goal_.x - pos_.x);
    double heading_error = std::atan2(std::sin(goal_heading - current_angle_),
                                      std::cos(goal_heading - current_angle_));

    double steering_cmd = 0.0;

    // 3. Reactive Obstacle Avoidance Logic (Lecture-Based Direct Nudge)
    if (min_obstacle_dist_ < obstacle_threshold_) {
      // Turn away from the nearest obstacle (sign flipped)
      double avoid_direction = (min_obstacle_angle_ >= 0.0) ? 1.0 : -1.0;
      double avoidance_steering = avoid_direction * (obstacle_threshold_ - min_obstacle_dist_);

      // Blend Go-To-Goal heading with Reactive Obstacle Avoidance Nudge
      steering_cmd = (kg_ * heading_error) + (ka_ * avoidance_steering);

      // Reduce linear velocity proportionally when approaching obstacles
      double speed_factor = std::clamp(min_obstacle_dist_ / obstacle_threshold_, 0.1, 1.0);
      cmd.linear.x = max_vel_ * speed_factor * std::max(0.0, std::cos(heading_error));
    } else {
      // Pure Go-To-Goal Steering
      steering_cmd = kg_ * heading_error;
      cmd.linear.x = max_vel_ * std::max(0.0, std::cos(heading_error));
    }

    // Enforce Angular Velocity Limits
    cmd.angular.z = std::clamp(steering_cmd, -max_ang_vel_, max_ang_vel_);

    pub_twist_->publish(cmd);
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

    odom_received_ = true;
  }

  void laser_scan_callback(const LaserScan &msg) {
    double min_dist = std::numeric_limits<double>::infinity();
    double min_angle = 0.0;

    for (size_t i = 0; i < msg.ranges.size(); ++i) {
      double r = msg.ranges.at(i);

      if (std::isnan(r) || std::isinf(r) || r < msg.range_min || r > msg.range_max) {
        continue;
      }

      // Ray angle in robot body frame wrapped to [-pi, pi]
      double angle = msg.angle_min + (msg.angle_increment * static_cast<double>(i));
      angle = std::atan2(std::sin(angle), std::cos(angle));

      // Subtract physical robot radius buffer for effective clearance
      double d_eff = std::max(0.001, r - robot_radius_);

      if (d_eff < min_dist) {
        min_dist = d_eff;
        min_angle = angle;
      }
    }

    min_obstacle_dist_ = min_dist;
    min_obstacle_angle_ = min_angle;
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
