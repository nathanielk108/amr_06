#include <cmath>
#include <functional>
#include <memory>
#include <algorithm>
#include <limits>

#include <rclcpp/rclcpp.hpp>

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
  rclcpp::TimerBase::SharedPtr timer_;

  rclcpp::Subscription<Odometry>::SharedPtr sub_odometry_;
  rclcpp::Subscription<LaserScan>::SharedPtr sub_laser_scan_;

  rclcpp::Publisher<Twist>::SharedPtr pub_twist_;

  Vector3 pos_;
  Vector3 goal_;

  double current_angle_ = 0.0;

  double front_ = 10.0;
  double left_ = 10.0;
  double right_ = 10.0;

  bool goal_initialized_ = false;

  // Change to 0 if lidar angle 0 already points forward.
  double lidar_offset_ = M_PI / 2.0;

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

    pub_twist_ = this->create_publisher<Twist>(
        "/TTB06/cmd_vel", 10);

    pos_.x = 0.0;
    pos_.y = 0.0;

    goal_.x = 0.0;
    goal_.y = 0.0;
  }

  void command_loop_function() {

    if (!goal_initialized_) {
      return;
    }

    double distance =
        std::hypot(goal_.x - pos_.x,
                   goal_.y - pos_.y);

    double desired_heading =
        std::atan2(goal_.y - pos_.y,
                   goal_.x - pos_.x);

    double heading_error =
        std::atan2(
            std::sin(desired_heading - current_angle_),
            std::cos(desired_heading - current_angle_));

    double speed = 0.2;
    double steering = 0.8 * heading_error;

    // Obstacle ahead
    if (front_ < 0.65) {

      speed = 0.08;

      // Turn toward the side with more space
      if (left_ > right_) {
        steering = 0.8;
      }
      else {
        steering = -0.8;
      }
    }

    // Very close obstacle
    if (front_ < 0.30) {

      speed = 0.0;

      if (left_ > right_) {
        steering = 1.0;
      }
      else {
        steering = -1.0;
      }
    }

    steering = std::clamp(steering, -1.0, 1.0);

    Twist cmd;

    if (distance < 0.05) {
      cmd.linear.x = 0.0;
      cmd.angular.z = 0.0;
    }
    else {
      cmd.linear.x = speed;
      cmd.angular.z = steering;
    }

    pub_twist_->publish(cmd);

    RCLCPP_INFO_THROTTLE(
        this->get_logger(),
        *this->get_clock(),
        500,
        "dist=%.2f front=%.2f left=%.2f right=%.2f v=%.2f w=%.2f",
        distance,
        front_,
        left_,
        right_,
        cmd.linear.x,
        cmd.angular.z);
  }

  void odometry_callback(const Odometry &msg) {

    pos_.x = msg.pose.pose.position.x;
    pos_.y = msg.pose.pose.position.y;

    double qx = msg.pose.pose.orientation.x;
    double qy = msg.pose.pose.orientation.y;
    double qz = msg.pose.pose.orientation.z;
    double qw = msg.pose.pose.orientation.w;

    current_angle_ =
        std::atan2(
            2.0 * (qw * qz + qx * qy),
            1.0 - 2.0 * (qy * qy + qz * qz));

    // Goal is 1.5 meters in front of starting pose
    if (!goal_initialized_) {

      goal_.x =
          pos_.x + 1.5 * std::cos(current_angle_);

      goal_.y =
          pos_.y + 1.5 * std::sin(current_angle_);

      goal_initialized_ = true;

      RCLCPP_INFO(
          this->get_logger(),
          "Start=(%.2f, %.2f) heading=%.2f goal=(%.2f, %.2f)",
          pos_.x,
          pos_.y,
          current_angle_,
          goal_.x,
          goal_.y);
    }
  }

  void laser_scan_callback(const LaserScan &msg) {

    front_ = 10.0;
    left_ = 10.0;
    right_ = 10.0;

    for (size_t i = 0; i < msg.ranges.size(); ++i) {

      double range = msg.ranges[i];

      if (!std::isfinite(range)) {
        continue;
      }

      if (range < msg.range_min ||
          range > msg.range_max) {
        continue;
      }

      double angle =
          msg.angle_min +
          msg.angle_increment * i;

      // Convert lidar frame to robot-forward frame
      angle += lidar_offset_;

      angle =
          std::atan2(
              std::sin(angle),
              std::cos(angle));

      // Front: +/- 30 degrees
      if (std::abs(angle) < M_PI / 6.0) {
        front_ = std::min(front_, range);
      }

      // Left: 30 to 90 degrees
      else if (angle >= M_PI / 6.0 &&
               angle < M_PI / 2.0) {
        left_ = std::min(left_, range);
      }

      // Right: -90 to -30 degrees
      else if (angle <= -M_PI / 6.0 &&
               angle > -M_PI / 2.0) {
        right_ = std::min(right_, range);
      }
    }
  }
};

int main(int argc, char **argv) {

  rclcpp::init(argc, argv);

  Lab3Node::SharedPtr control_node =
      std::make_shared<Lab3Node>();

  rclcpp::executors::SingleThreadedExecutor exec;

  exec.add_node(control_node);
  exec.spin();

  rclcpp::shutdown();

  return 0;
}
