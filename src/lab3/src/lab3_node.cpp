#include <cmath>
#include <cstdlib>
#include <functional>
#include <memory>
#include <algorithm>

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


using namespace std::chrono_literals;
using namespace std::placeholders;

using namespace sensor_msgs::msg;
using namespace geometry_msgs::msg;
using namespace nav_msgs::msg;

class Lab3Node : public rclcpp::Node {

private:
  // Timers
  rclcpp::TimerBase::SharedPtr timer_;

  // Subscriptions
  rclcpp::Subscription<Odometry>::SharedPtr sub_odometry_;
  rclcpp::Subscription<LaserScan>::SharedPtr sub_laser_scan_;

  // Publishers
  rclcpp::Publisher<Twist>::SharedPtr pub_twist_;

  // Vehicle State
  Vector3 pos_;
  Vector3 vel_;
  Vector3 goal_;

  double current_angle_ = 0.0;
  double last_angle_ = 0.0;
  double laser_steering_ = 0.0;
  double min_obstacle_dist_ = 100.0;
  double robot_radius_ = 0.2; 
  double max_range_ = 3.0;   
  double goal_tolerance_ = 0.10;  
  bool goal_rendered_ = false;

  double max_vel = 0.5;
  double max_ang_vel = M_PI/4;

  double origin_x_ = 0.0;
  double origin_y_ = 0.0;
  double origin_yaw_ = 0.0;
  bool origin_rendered_ = false;

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
    goal_.x = 1.5;
    goal_.y = 0;
  }

  void command_loop_function() {
    Twist cmd;
    double kg = 0.5;
    double ka = 0.1;
    double distance = std::hypot(goal_.x - pos_.x, goal_.y - pos_.y);
    if (distance > goal_tolerance_) {
      double goal_heading = std::atan2(goal_.y - pos_.y, goal_.x - pos_.x);
      double heading_error = std::atan2(std::sin(goal_heading - current_angle_), std::cos(goal_heading - current_angle_));
      double steering = (kg * heading_error) + (ka * laser_steering_);
    
      cmd.linear.set__x(0.5);
      cmd.angular.set__z(steering);
    }
    pub_twist_->publish(cmd);
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
    current_angle_ = std::atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz)); // quarternion yaw angle

    if (!origin_rendered_) {
      origin_x_ = pos_.x;
      origin_y_ = pos_.y;
      origin_yaw_ = current_angle_;
      origin_rendered_ = true;
    }
    
    double dx = pos_.x - origin_x_;
    double dy = pos_.y - origin_y_;
    pos_.x = dx * std::cos(origin_yaw_) + dy * std::sin(origin_yaw_);
    pos_.y = -dx * std::sin(origin_yaw_) + dy * std::cos(origin_yaw_);
    current_angle_ = std::atan2(std::sin(current_angle_ - origin_yaw_), std::cos(current_angle_ - origin_yaw_));

  }

  void laser_scan_callback(const LaserScan &msg) {

    double goal_heading = std::atan2(goal_.y - pos_.y, goal_.x - pos_.x);
    double heading_error = std::atan2(std::sin(goal_heading - current_angle_), std::cos(goal_heading - current_angle_));

    double max_dist = -1.0;
    double free_space_heading = heading_error;

    for (size_t i = 0; i < msg.ranges.size(); ++i) {
      double range = msg.ranges.at(i);
      if (range > max_range_) {
        range = max_range_;
      }
      double d_i = std::max(0.001, range - robot_radius_);
      double angle = msg.angle_min + (msg.angle_increment * i);
      angle = std::atan2(std::sin(angle), std::cos(angle));
      if (d_i > max_dist) {
        max_dist = d_i;
        free_space_heading = angle;
      }
    }
    laser_steering_ = free_space_heading;
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
