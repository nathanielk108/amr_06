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

  bool goal_initialized_ = false;

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

    vel_.x = 0;
    vel_.y = 0;
    vel_.z = 0;

    goal_.x = 0;
    goal_.y = 0;
    goal_.z = 0;
  }

  void command_loop_function() {

    if (!goal_initialized_) {
      return;
    }

    double kg = 0.4;
    double ka = 2.0;

    double distance =
        std::hypot(goal_.x - pos_.x, goal_.y - pos_.y);

    double desiredHeading =
        std::atan2(goal_.y - pos_.y, goal_.x - pos_.x);

    double headingError =
        std::atan2(
            std::sin(desiredHeading - current_angle_),
            std::cos(desiredHeading - current_angle_));

    double goalSteering = kg * headingError;
    double obstacleSteering = ka * laser_steering_;

    double steering = goalSteering + obstacleSteering;

    steering = std::clamp(steering, -1.0, 1.0);

    // Slow down as obstacle strength increases
    double speed =
        0.2 * (1.0 - 0.9 * obstacle_strength_);

    speed = std::clamp(speed, 0.02, 0.2);

    Twist cmd;

    if (distance < 0.05) {
      cmd.linear.set__x(0.0);
      cmd.angular.set__z(0.0);
    }
    else {
      cmd.linear.set__x(speed);
      cmd.angular.set__z(steering);
    }

    pub_twist_->publish(cmd);

    RCLCPP_INFO_THROTTLE(
        this->get_logger(),
        *this->get_clock(),
        500,
        "pos=(%.2f, %.2f) goal=(%.2f, %.2f) dist=%.2f heading_err=%.2f goal_turn=%.2f avoid_turn=%.2f obstacle=%.2f closest=%.2f -> v=%.2f w=%.2f",
        pos_.x,
        pos_.y,
        goal_.x,
        goal_.y,
        distance,
        headingError,
        goalSteering,
        obstacleSteering,
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

    // Set goal 1.5 m in the direction the robot starts facing
    if (!goal_initialized_) {

      goal_.x =
          pos_.x + 1.5 * std::cos(current_angle_);

      goal_.y =
          pos_.y + 1.5 * std::sin(current_angle_);

      goal_initialized_ = true;

      RCLCPP_INFO(
          this->get_logger(),
          "Start=(%.2f, %.2f) heading=%.2f rad goal=(%.2f, %.2f)",
          pos_.x,
          pos_.y,
          current_angle_,
          goal_.x,
          goal_.y);
    }
  }

  void laser_scan_callback(const LaserScan &msg) {

    double obstacle_range = 1.5;

    double steering_sum = 0.0;
    double strength_sum = 0.0;

    laser_steering_ = 0.0;
    obstacle_strength_ = 0.0;
    closest_obstacle_ =
        std::numeric_limits<double>::infinity();

    int forward_count = 0;
    int obstacle_count = 0;

    for (size_t i = 0; i < msg.ranges.size(); ++i) {

      double range = msg.ranges.at(i);

      double angle =
          msg.angle_min +
          (msg.angle_increment * i);

      // Normalize scan angle to [-pi, pi]
      angle =
          std::atan2(std::sin(angle), std::cos(angle));

      if (!std::isfinite(range)) {
        continue;
      }

      // Only use forward-facing 180 degrees
      if (std::abs(angle) > M_PI / 2.0) {
        continue;
      }

      forward_count++;

      // Only obstacles inside influence range matter
      if (range <= msg.range_min ||
          range > obstacle_range) {
        continue;
      }

      obstacle_count++;

      double strength =
          (obstacle_range - range) /
          obstacle_range;

      /*
       * Obstacle on left:
       * angle > 0
       * steering contribution < 0
       *
       * Obstacle on right:
       * angle < 0
       * steering contribution > 0
       */
      steering_sum +=
          -strength * std::sin(angle);

      strength_sum += strength;

      obstacle_strength_ =
          std::max(
              obstacle_strength_,
              strength);

      closest_obstacle_ =
          std::min(
              closest_obstacle_,
              range);
    }

    if (strength_sum > 0.0) {
      laser_steering_ =
          steering_sum / strength_sum;
    }

    /*
     * A box directly in front can produce equal left and right
     * repulsive forces. Give the robot a direction to break the
     * symmetry instead of driving directly into the local minimum.
     */
    if (obstacle_strength_ > 0.2 &&
        std::abs(laser_steering_) < 0.05) {

      laser_steering_ = 0.5;
    }

    RCLCPP_INFO_THROTTLE(
        this->get_logger(),
        *this->get_clock(),
        500,
        "scan=%zu forward=%d obstacles=%d closest=%.2f strength=%.2f avoidance=%.2f",
        msg.ranges.size(),
        forward_count,
        obstacle_count,
        closest_obstacle_,
        obstacle_strength_,
        laser_steering_);
  }
};

int main(int argc, char **argv) {

  rclcpp::init(argc, argv);

  Lab3Node::SharedPtr control_node =
      std::make_shared<Lab3Node>();

  rclcpp::executors::SingleThreadedExecutor exec{};

  exec.add_node(control_node);
  exec.spin();

  rclcpp::shutdown();

  return 0;
}
