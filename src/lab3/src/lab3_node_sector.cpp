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
#include <sensor_msgs/msg/laser_scan.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/twist.hpp>

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
  bool goal_initialized_ = false;

  double front_ = 10.0;
  double front_left_ = 10.0;
  double left_ = 10.0;
  double back_left_ = 10.0;
  double back_ = 10.0;
  double back_right_ = 10.0;
  double right_ = 10.0;
  double front_right_ = 10.0;

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
  }

  void command_loop_function() {

    if (!goal_initialized_) {
      return;
    }

    double obstacle_distance = 0.7;
    double stop_distance = 0.3;

    double distance =
        std::hypot(goal_.x - pos_.x, goal_.y - pos_.y);

    double desired_heading =
        std::atan2(goal_.y - pos_.y, goal_.x - pos_.x);

    double heading_error =
        std::atan2(
            std::sin(desired_heading - current_angle_),
            std::cos(desired_heading - current_angle_));

    double speed = 0.2;
    double steering = 0.5 * heading_error;

    /*
     * Obstacle directly ahead.
     * Turn toward whichever side has more room.
     */
    if (front_ < obstacle_distance) {

      speed = 0.08;

      if (front_left_ > front_right_) {
        steering = 0.8;
      }
      else {
        steering = -0.8;
      }
    }

    /*
     * Obstacle ahead-left.
     */
    else if (front_left_ < obstacle_distance) {

      speed = 0.12;
      steering = -0.6;
    }

    /*
     * Obstacle ahead-right.
     */
    else if (front_right_ < obstacle_distance) {

      speed = 0.12;
      steering = 0.6;
    }

    /*
     * Very close obstacle.
     */
    if (front_ < stop_distance) {
      speed = 0.0;

      if (left_ > right_) {
        steering = 1.0;
      }
      else {
        steering = -1.0;
      }
    }

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
        "F=%.2f FL=%.2f FR=%.2f L=%.2f R=%.2f dist=%.2f -> v=%.2f w=%.2f",
        front_,
        front_left_,
        front_right_,
        left_,
        right_,
        distance,
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

    current_angle_ = std::atan2(
        2.0 * (qw * qz + qx * qy),
        1.0 - 2.0 * (qy * qy + qz * qz));

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
    front_left_ = 10.0;
    left_ = 10.0;
    back_left_ = 10.0;
    back_ = 10.0;
    back_right_ = 10.0;
    right_ = 10.0;
    front_right_ = 10.0;

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

      angle =
          std::atan2(std::sin(angle), std::cos(angle));

      /*
       * ROS lidar angles:
       *
       *          +pi/2
       *            L
       *
       *      FL         BL
       *
       * F                  B
       *
       *      FR         BR
       *
       *            R
       *          -pi/2
       */

      if (angle >= -M_PI / 8.0 &&
          angle < M_PI / 8.0) {

        front_ = std::min(front_, range);
      }

      else if (angle >= M_PI / 8.0 &&
               angle < 3.0 * M_PI / 8.0) {

        front_left_ = std::min(front_left_, range);
      }

      else if (angle >= 3.0 * M_PI / 8.0 &&
               angle < 5.0 * M_PI / 8.0) {

        left_ = std::min(left_, range);
      }

      else if (angle >= 5.0 * M_PI / 8.0 &&
               angle < 7.0 * M_PI / 8.0) {

        back_left_ = std::min(back_left_, range);
      }

      else if (angle >= 7.0 * M_PI / 8.0 ||
               angle < -7.0 * M_PI / 8.0) {

        back_ = std::min(back_, range);
      }

      else if (angle >= -7.0 * M_PI / 8.0 &&
               angle < -5.0 * M_PI / 8.0) {

        back_right_ = std::min(back_right_, range);
      }

      else if (angle >= -5.0 * M_PI / 8.0 &&
               angle < -3.0 * M_PI / 8.0) {

        right_ = std::min(right_, range);
      }

      else if (angle >= -3.0 * M_PI / 8.0 &&
               angle < -M_PI / 8.0) {

        front_right_ = std::min(front_right_, range);
      }
    }
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
