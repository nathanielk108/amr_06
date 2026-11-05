#include <cmath>
#include <cstdlib>
#include <functional>
#include <memory>
#include <algorithm>
#include <limits>

#include <rclcpp/create_timer.hpp>
#include <rclcpp/executors.hpp>
#include <rclcpp/executors/single_threaded_executor.hpp>
#include <rclcpp/publisher_base.hpp>
#include <rclcpp/qos.hpp>
#include <rclcpp/subscription.hpp>
#include <rclcpp/utilities.hpp>

#include <rmw/qos_profiles.h>
#include <sensor_msgs/msg/joy.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <irobot_create_msgs/msg/ir_intensity_vector.hpp>

using namespace std::chrono_literals;
using namespace std::placeholders;

using namespace sensor_msgs::msg;
using namespace geometry_msgs::msg;
using namespace nav_msgs::msg;
using namespace irobot_create_msgs::msg;

class Lab3Node : public rclcpp::Node {

private:
  // Timers
  rclcpp::TimerBase::SharedPtr timer_;

  // Subscriptions
  rclcpp::Subscription<Joy>::SharedPtr sub_joy_;
  rclcpp::Subscription<Odometry>::SharedPtr sub_odometry_;
  rclcpp::Subscription<LaserScan>::SharedPtr sub_laser_scan_;

  // Publishers
  rclcpp::Publisher<Twist>::SharedPtr pub_twist_;

  // Vehicle State
  Vector3 pos_;
  Vector3 vel_;
  Vector3 goal_;

  double current_angle_ = 0.0;
  double laser_steering_ = 0.0;
  double obstacle_strength_ = 0.0;
  double closest_obstacle_ = std::numeric_limits<double>::infinity();

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

    pos_.x = 0;
    pos_.y = 0;
    pos_.z = 0;

    goal_.x = 1.5;
    goal_.y = 0;
  }

  void command_loop_function() {
    double kg = 0.5;
    double ka = 0.5;

    double distance = std::hypot(goal_.x - pos_.x, goal_.y - pos_.y);
    double desiredHeading = std::atan2(goal_.y - pos_.y, goal_.x - pos_.x);
    double headingError = std::atan2(std::sin(desiredHeading - current_angle_), std::cos(desiredHeading - current_angle_));

    double steering = (kg * headingError) + (ka * laser_steering_);

    steering = std::clamp(steering, -1.0, 1.0);

    double speed = 0.2 * (1.0 - 0.8 * obstacle_strength_);
    speed = std::clamp(speed, 0.04, 0.2);

    Twist cmd;

    if (distance < 0.05) {
      cmd.linear.set__x(0.0);
      cmd.angular.set__z(0.0);
    } else {
      cmd.linear.set__x(speed);
      cmd.angular.set__z(steering);
    }

    pub_twist_->publish(cmd);

    RCLCPP_INFO_THROTTLE(
        this->get_logger(),
        *this->get_clock(),
        500,
        "pos=(%.2f, %.2f) dist=%.2f head_err=%.2f obstacle=%.2f closest=%.2f speed=%.2f steer=%.2f",
        pos_.x,
        pos_.y,
        distance,
        headingError,
        obstacle_strength_,
        closest_obstacle_,
        cmd.linear.x,
        cmd.angular.z);
  }

  void odometry_callback(const Odometry &msg) {
    // Global Frame
    pos_.x = msg.pose.pose.position.x;
    pos_.y = msg.pose.pose.position.y;

    vel_.x = msg.twist.twist.linear.x;
    vel_.y = msg.twist.twist.linear.y;

    // Robot Frame
    double qx = msg.pose.pose.orientation.x;
    double qy = msg.pose.pose.orientation.y;
    double qz = msg.pose.pose.orientation.z;
    double qw = msg.pose.pose.orientation.w;

    current_angle_ = std::atan2(
        2.0 * (qw * qz + qx * qy),
        1.0 - 2.0 * (qy * qy + qz * qz));
  }

  void laser_scan_callback(const LaserScan &msg) {
    double obstacle_range = 0.8;

    double repulsive_x = 0.0;
    double repulsive_y = 0.0;

    laser_steering_ = 0.0;
    obstacle_strength_ = 0.0;
    closest_obstacle_ = std::numeric_limits<double>::infinity();

    for (size_t i = 0; i < msg.ranges.size(); ++i) {
      double range = msg.ranges.at(i);
      double angle = msg.angle_min + (msg.angle_increment * i);

      // Only use valid forward-facing measurements
      if (!std::isfinite(range)) {
        continue;
      }

      // Disregard angles outside forward 180 degrees
      if (std::abs(angle) > M_PI / 2.0) {
        continue;
      }

      if (range <= 0.0 || range > obstacle_range) {
        continue;
      }

      double strength = (obstacle_range - range) / obstacle_range;

      // Reverse the measured obstacle vector
      repulsive_x -= strength * std::cos(angle);
      repulsive_y -= strength * std::sin(angle);

      obstacle_strength_ = std::max(obstacle_strength_, strength);
      closest_obstacle_ = std::min(closest_obstacle_, range);
    }

    if (obstacle_strength_ > 0.0) {
      laser_steering_ = obstacle_strength_ * std::atan2(repulsive_y, repulsive_x);
    }

    RCLCPP_INFO_THROTTLE(
        this->get_logger(),
        *this->get_clock(),
        500,
        "laser: strength=%.2f closest=%.2f repulse=(%.2f, %.2f) avoidance=%.2f",
        obstacle_strength_,
        closest_obstacle_,
        repulsive_x,
        repulsive_y,
        laser_steering_);
  }
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);

  Lab3Node::SharedPtr control_node = std::make_shared<Lab3Node>();
  rclcpp::executors::SingleThreadedExecutor exec{};
  exec.add_node(control_node);
  exec.spin();

  rclcpp::shutdown();
  return 0;
}
