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

  // Robot State (Relative Origin Frame)
  geometry_msgs::msg::Vector3 pos_;
  geometry_msgs::msg::Vector3 goal_;
  double current_angle_ = 0.0;

  bool odom_received_ = false;
  bool scan_received_ = false;

  // Origin Frame Calibration
  double origin_x_ = 0.0;
  double origin_y_ = 0.0;
  double origin_yaw_ = 0.0;
  bool origin_rendered_ = false;

  // Obstacle Telemetry
  double min_obstacle_dist_ = std::numeric_limits<double>::infinity();
  double min_obstacle_angle_ = 0.0;

  // Motion & Avoidance Parameters
  double robot_radius_ = 0.20;          // Physical robot radius (m)
  double obstacle_threshold_ = 0.85;     // Distance to trigger avoidance steering (m)
  double stop_distance_ = 0.28;          // Critical distance to stop linear motion (m)
  double goal_tolerance_ = 0.10;         // Distance to consider goal reached (m)

  double target_speed_ = 0.50;          // Constant linear target speed (m/s)
  double max_ang_vel_ = M_PI / 2.0;      // Max angular speed limit (rad/s)

  double kg_ = 1.0;                      // Go-To-Goal heading gain
  double ka_ = 2.2;                      // Stronger obstacle avoidance gain

  // TOGGLE THIS TO true IF YOUR SENSOR STEERS TOWARD OBSTACLES INSTEAD OF AWAY
  bool flip_steering_ = false;

public:
  explicit Lab3Node() : rclcpp::Node("lab3_node") {

    // Main control loop running at 10 Hz (100 ms)
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

    // Initial goal position (relative to start frame)
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
      RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "Goal Target Reached!");
      return;
    }

    // 2. Go-To-Goal Heading Error
    double goal_heading = std::atan2(goal_.y - pos_.y, goal_.x - pos_.x);
    double heading_error = std::atan2(std::sin(goal_heading - current_angle_),
                                      std::cos(goal_heading - current_angle_));

    double linear_cmd = target_speed_;
    double steering_cmd = 0.0;

    // 3. Obstacle Avoidance & Speed Decision
    if (min_obstacle_dist_ < obstacle_threshold_) {

      // Do not slow down dynamically. Only stop linear motion within emergency distance.
      if (min_obstacle_dist_ <= stop_distance_) {
        linear_cmd = 0.0; 
      } else {
        linear_cmd = target_speed_; // Maintain 0.5 m/s speed while steering away
      }

      // Determine raw steering direction away from obstacle
      double avoid_dir = 0.0;
      if (std::abs(min_obstacle_angle_) < 0.1) {
        // Obstacle is straight ahead: force turn right to break deadlock
        avoid_dir = -1.0;
      } else {
        // Obstacle is on left (angle > 0) -> turn right (-1.0)
        // Obstacle is on right (angle < 0) -> turn left (+1.0)
        avoid_dir = (min_obstacle_angle_ > 0.0) ? -1.0 : 1.0;
      }

      if (flip_steering_) {
        avoid_dir *= -1.0;
      }

      // Proximity weight increases as robot gets closer to obstacle
      double proximity_weight = (obstacle_threshold_ - min_obstacle_dist_) / obstacle_threshold_;
      double avoidance_steering = avoid_dir * ka_ * proximity_weight;

      // Blend Go-To-Goal heading with strong Obstacle Avoidance offset
      steering_cmd = (kg_ * heading_error) + avoidance_steering;

    } else {
      // Path is clear: full speed ahead towards goal
      linear_cmd = target_speed_;
      steering_cmd = kg_ * heading_error;
    }

    // Assign commands and apply angular velocity limits
    cmd.linear.x = linear_cmd;
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

    if (!origin_rendered_) {
      origin_x_ = raw_x;
      origin_y_ = raw_y;
      origin_yaw_ = raw_yaw;
      origin_rendered_ = true;
    }

    // Frame transformation relative to startup pose
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

      // Angle in robot body frame wrapped to [-pi, pi]
      double angle = msg.angle_min + (msg.angle_increment * static_cast<double>(i));
      angle = std::atan2(std::sin(angle), std::cos(angle));

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
