#include <algorithm>
#include <chrono>
#include <cmath>
#include <functional>
#include <limits>
#include <memory>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <rmw/qos_profiles.h>

#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>

using namespace std::chrono_literals;
using namespace std::placeholders;

using namespace sensor_msgs::msg;
using namespace geometry_msgs::msg;
using namespace nav_msgs::msg;

class Lab3Node : public rclcpp::Node {

private:
  struct Point {
    double x;
    double y;
  };

  enum State {
    GO_TO_GOAL,
    WALL_FOLLOW
  };

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Subscription<Odometry>::SharedPtr sub_odometry_;
  rclcpp::Subscription<LaserScan>::SharedPtr sub_laser_scan_;
  rclcpp::Publisher<Twist>::SharedPtr pub_twist_;

  Vector3 pos_;
  Vector3 goal_;
  std::vector<Point> obstacles_;

  State state_ = GO_TO_GOAL;

  double current_angle_ = 0.0;
  double goal_distance_ = 1.5;
  bool lidar_flip_ = true;

  double front_distance_ = std::numeric_limits<double>::infinity();
  double left_distance_ = std::numeric_limits<double>::infinity();
  double right_distance_ = std::numeric_limits<double>::infinity();
  double left_space_ = 0.0;
  double right_space_ = 0.0;

  double speed_command_ = 0.0;
  double steering_command_ = 0.0;

  double hit_goal_distance_ = 0.0;
  int avoidance_direction_ = 0;
  bool wall_acquired_ = false;
  int clear_scans_ = 0;
  unsigned long scan_sequence_ = 0;
  unsigned long last_checked_scan_ = 0;
  int log_counter_ = 0;

  bool goal_initialized_ = false;
  bool odom_received_ = false;
  bool scan_received_ = false;

public:
  explicit Lab3Node() : rclcpp::Node("lab3_node") {
    goal_distance_ = this->declare_parameter<double>("goal_distance", 1.5);
    lidar_flip_ = this->declare_parameter<bool>("lidar_flip", true);

    timer_ = this->create_wall_timer(
      50ms, std::bind(&Lab3Node::command_loop_function, this));

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
    goal_.x = 0.0;
    goal_.y = 0.0;
  }

private:
  static double wrap_angle(double angle) {
    return std::atan2(std::sin(angle), std::cos(angle));
  }

  double distance_to_goal() const {
    return std::hypot(goal_.x - pos_.x, goal_.y - pos_.y);
  }

  double bearing_to_goal() const {
    double angle = std::atan2(goal_.y - pos_.y, goal_.x - pos_.x);
    return wrap_angle(angle - current_angle_);
  }

  bool direct_path_clear(double goal_bearing, double goal_distance) const {
    const double robot_radius = 0.18;
    const double safety_margin = 0.10;
    const double corridor_radius = robot_radius + safety_margin;
    const double lookahead = std::min(goal_distance, 1.5);

    double c = std::cos(goal_bearing);
    double s = std::sin(goal_bearing);

    for (const Point &obstacle : obstacles_) {
      double along = obstacle.x * c + obstacle.y * s;

      if (along < 0.0 || along > lookahead) continue;

      double lateral = std::abs(-obstacle.x * s + obstacle.y * c);

      if (lateral < corridor_radius) return false;
    }

    return true;
  }

  void stop() {
    speed_command_ = 0.0;
    steering_command_ = 0.0;
    pub_twist_->publish(Twist());
  }

  void enter_wall_follow(double goal_distance) {
    hit_goal_distance_ = goal_distance;
    clear_scans_ = 0;
    wall_acquired_ = false;

    if (left_space_ > right_space_ + 0.10)
      avoidance_direction_ = 1;
    else if (right_space_ > left_space_ + 0.10)
      avoidance_direction_ = -1;
    else
      avoidance_direction_ = 1;

    state_ = WALL_FOLLOW;

    RCLCPP_INFO(
      this->get_logger(),
      "Obstacle hit: dist=%.2f left=%.2f right=%.2f direction=%s",
      goal_distance,
      left_space_,
      right_space_,
      avoidance_direction_ > 0 ? "LEFT" : "RIGHT");
  }

  void go_to_goal(double distance, double goal_bearing,
                  double &target_velocity, double &target_steering) {
    const double kv = 0.5;
    const double kp = 1.5;
    const double max_velocity = 0.15;
    const double max_turn = 0.9;

    target_velocity = std::min(kv * distance, max_velocity);
    target_steering = std::clamp(kp * goal_bearing, -max_turn, max_turn);

    if (std::abs(goal_bearing) > M_PI / 3.0)
      target_velocity = 0.0;
    else if (std::abs(goal_bearing) > M_PI / 6.0)
      target_velocity = std::min(target_velocity, 0.07);
  }

  void wall_follow(double &target_velocity, double &target_steering) {
    const double desired_wall_distance = 0.45;
    const double front_turn_distance = 0.50;
    const double front_stop_distance = 0.32;
    const double wall_kp = 2.0;
    const double max_turn = 0.9;
    const double wall_speed = 0.10;

    if (front_distance_ < front_turn_distance) {
      target_velocity = front_distance_ < front_stop_distance ? 0.0 : 0.04;
      target_steering = avoidance_direction_ * max_turn;
      return;
    }

    double wall_distance = avoidance_direction_ > 0 ? right_distance_ : left_distance_;

    if (!wall_acquired_) {
      if (std::isfinite(wall_distance) && wall_distance < 0.90) {
        wall_acquired_ = true;
      }
      else {
        target_velocity = 0.05;
        target_steering = avoidance_direction_ * 0.60;
        return;
      }
    }

    if (!std::isfinite(wall_distance) || wall_distance > 1.0) {
      target_velocity = 0.07;
      target_steering = -avoidance_direction_ * 0.35;
      return;
    }

    double wall_error = desired_wall_distance - wall_distance;

    target_velocity = wall_speed;
    target_steering = avoidance_direction_ * wall_kp * wall_error;
    target_steering = std::clamp(target_steering, -max_turn, max_turn);

    if (std::abs(target_steering) > 0.5)
      target_velocity = 0.07;
  }

  void command_loop_function() {
    if (!goal_initialized_ || !odom_received_ || !scan_received_) return;

    double distance = distance_to_goal();
    double goal_bearing = bearing_to_goal();

    if (distance < 0.10) {
      stop();
      return;
    }

    bool path_clear = direct_path_clear(goal_bearing, distance);

    if (state_ == GO_TO_GOAL && !path_clear)
      enter_wall_follow(distance);

    if (state_ == WALL_FOLLOW && scan_sequence_ != last_checked_scan_) {
      last_checked_scan_ = scan_sequence_;

      if (path_clear && distance < hit_goal_distance_ - 0.05)
        clear_scans_++;
      else
        clear_scans_ = 0;

      if (clear_scans_ >= 3) {
        state_ = GO_TO_GOAL;
        avoidance_direction_ = 0;
        wall_acquired_ = false;
        clear_scans_ = 0;

        RCLCPP_INFO(
          this->get_logger(),
          "Goal path clear: returning to go-to-goal");
      }
    }

    double target_velocity = 0.0;
    double target_steering = 0.0;

    if (state_ == GO_TO_GOAL)
      go_to_goal(distance, goal_bearing, target_velocity, target_steering);
    else
      wall_follow(target_velocity, target_steering);

    speed_command_ = 0.45 * target_velocity + 0.55 * speed_command_;
    steering_command_ = 0.50 * target_steering + 0.50 * steering_command_;

    if (front_distance_ < 0.25)
      speed_command_ = 0.0;

    Twist cmd;
    cmd.linear.x = speed_command_;
    cmd.angular.z = steering_command_;
    pub_twist_->publish(cmd);

    log_counter_++;
    if (log_counter_ >= 10) {
      RCLCPP_INFO(
        this->get_logger(),
        "state=%s dist=%.2f bearing=%.2f front=%.2f left=%.2f right=%.2f clear=%d dir=%d acquired=%d v=%.2f w=%.2f",
        state_ == GO_TO_GOAL ? "GOAL" : "WALL",
        distance,
        goal_bearing,
        front_distance_,
        left_distance_,
        right_distance_,
        path_clear,
        avoidance_direction_,
        wall_acquired_,
        cmd.linear.x,
        cmd.angular.z);

      log_counter_ = 0;
    }
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

    odom_received_ = true;

    if (!goal_initialized_) {
      goal_.x = pos_.x + goal_distance_ * std::cos(current_angle_);
      goal_.y = pos_.y + goal_distance_ * std::sin(current_angle_);
      goal_initialized_ = true;

      RCLCPP_INFO(
        this->get_logger(),
        "Start=(%.2f %.2f) heading=%.2f goal=(%.2f %.2f) lidar_flip=%d",
        pos_.x,
        pos_.y,
        current_angle_,
        goal_.x,
        goal_.y,
        lidar_flip_);
    }
  }

  void laser_scan_callback(const LaserScan &msg) {
    const double planning_range = 2.0;
    const double front_half_angle = 30.0 * M_PI / 180.0;
    const double side_min_angle = 55.0 * M_PI / 180.0;
    const double side_max_angle = 105.0 * M_PI / 180.0;
    const double space_min_angle = 20.0 * M_PI / 180.0;
    const double space_max_angle = 100.0 * M_PI / 180.0;
    const double max_space = 2.0;

    obstacles_.clear();

    front_distance_ = std::numeric_limits<double>::infinity();
    left_distance_ = std::numeric_limits<double>::infinity();
    right_distance_ = std::numeric_limits<double>::infinity();

    double left_sum = 0.0;
    double right_sum = 0.0;
    int left_count = 0;
    int right_count = 0;

    for (size_t i = 0; i < msg.ranges.size(); i++) {
      double angle = msg.angle_min + static_cast<double>(i) * msg.angle_increment;

      if (lidar_flip_)
        angle = -angle;

      angle = wrap_angle(angle);

      double range = msg.ranges[i];

      if (std::isinf(range) && range > 0.0) {
        if (angle >= space_min_angle && angle <= space_max_angle) {
          left_sum += max_space;
          left_count++;
        }

        if (angle <= -space_min_angle && angle >= -space_max_angle) {
          right_sum += max_space;
          right_count++;
        }

        continue;
      }

      if (!std::isfinite(range)) continue;
      if (range < msg.range_min || range > msg.range_max) continue;

      if (std::abs(angle) <= front_half_angle)
        front_distance_ = std::min(front_distance_, range);

      if (angle >= side_min_angle && angle <= side_max_angle)
        left_distance_ = std::min(left_distance_, range);

      if (angle <= -side_min_angle && angle >= -side_max_angle)
        right_distance_ = std::min(right_distance_, range);

      double space = std::min(range, max_space);

      if (angle >= space_min_angle && angle <= space_max_angle) {
        left_sum += space;
        left_count++;
      }

      if (angle <= -space_min_angle && angle >= -space_max_angle) {
        right_sum += space;
        right_count++;
      }

      if (range <= planning_range) {
        Point obstacle;
        obstacle.x = range * std::cos(angle);
        obstacle.y = range * std::sin(angle);
        obstacles_.push_back(obstacle);
      }
    }

    left_space_ = left_count > 0 ? left_sum / left_count : max_space;
    right_space_ = right_count > 0 ? right_sum / right_count : max_space;

    scan_received_ = true;
    scan_sequence_++;
  }
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);

  Lab3Node::SharedPtr control_node = std::make_shared<Lab3Node>();
  rclcpp::executors::SingleThreadedExecutor exec;

  exec.add_node(control_node);
  exec.spin();

  rclcpp::shutdown();
  return 0;
}

