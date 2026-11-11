#include <array>
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

  struct Candidate {
    double velocity = 0.0;
    double steering = 0.0;
    double score = -std::numeric_limits<double>::infinity();
    double clearance = 0.0;
    bool safe = false;
  };

  rclcpp::TimerBase::SharedPtr timer_;

  rclcpp::Subscription<Odometry>::SharedPtr sub_odometry_;
  rclcpp::Subscription<LaserScan>::SharedPtr sub_laser_scan_;

  rclcpp::Publisher<Twist>::SharedPtr pub_twist_;

  Vector3 pos_;
  Vector3 goal_;

  std::vector<Point> obstacles_;

  double current_angle_ = 0.0;
  double goal_distance_ = 1.5;

  double left_space_ = 0.0;
  double right_space_ = 0.0;
  double closest_front_ = std::numeric_limits<double>::infinity();

  double speed_command_ = 0.0;
  double steering_command_ = 0.0;
  double previous_steering_ = 0.0;

  int avoidance_side_ = 0;
  int direct_clear_scans_ = 0;

  bool goal_initialized_ = false;
  bool odom_received_ = false;
  bool scan_received_ = false;
  bool new_scan_ = false;
  bool flip_lidar_ = false;

public:
  explicit Lab3Node() : rclcpp::Node("lab3_node") {
    goal_distance_ = this->declare_parameter<double>("goal_distance", 1.5);
    flip_lidar_ = this->declare_parameter<bool>("flip_lidar", false);

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

    RCLCPP_INFO(
      this->get_logger(),
      "goal_distance=%.2f flip_lidar=%s",
      goal_distance_,
      flip_lidar_ ? "true" : "false");
  }

private:
  static double wrap_angle(double angle) {
    return std::atan2(std::sin(angle), std::cos(angle));
  }

  static int steering_side(double steering) {
    if (steering > 0.05) return 1;
    if (steering < -0.05) return -1;
    return 0;
  }

  void stop() {
    speed_command_ = 0.0;
    steering_command_ = 0.0;
    previous_steering_ = 0.0;
    avoidance_side_ = 0;
    direct_clear_scans_ = 0;
    pub_twist_->publish(Twist());
  }

  Candidate evaluate_candidate(double velocity, double steering,
                               double goal_x, double goal_y,
                               double goal_distance) const {
    const double robot_radius = 0.18;
    const double safety_margin = 0.08;
    const double collision_radius = robot_radius + safety_margin;
    const double collision_radius_sq = collision_radius * collision_radius;

    const double horizon = 3.0;
    const double dt = 0.05;
    const int steps = static_cast<int>(horizon / dt);

    double x = 0.0;
    double y = 0.0;
    double theta = 0.0;
    double min_distance_sq = std::numeric_limits<double>::infinity();

    Candidate candidate;
    candidate.velocity = velocity;
    candidate.steering = steering;
    candidate.safe = true;

    for (int step = 0; step < steps; step++) {
      double theta_mid = theta + 0.5 * steering * dt;

      x += velocity * std::cos(theta_mid) * dt;
      y += velocity * std::sin(theta_mid) * dt;
      theta = wrap_angle(theta + steering * dt);

      for (const Point &obstacle : obstacles_) {
        double dx = obstacle.x - x;
        double dy = obstacle.y - y;
        double distance_sq = dx * dx + dy * dy;

        min_distance_sq = std::min(min_distance_sq, distance_sq);

        if (distance_sq < collision_radius_sq) {
          candidate.safe = false;
          return candidate;
        }
      }
    }

    double final_dx = goal_x - x;
    double final_dy = goal_y - y;
    double final_distance = std::hypot(final_dx, final_dy);
    double final_goal_heading = wrap_angle(std::atan2(final_dy, final_dx) - theta);

    double progress = goal_distance - final_distance;
    double clearance = std::sqrt(min_distance_sq) - collision_radius;

    if (!std::isfinite(clearance)) clearance = 2.0;
    clearance = std::clamp(clearance, 0.0, 1.0);

    double side_switch_penalty = 0.0;
    int side = steering_side(steering);

    if (avoidance_side_ != 0 && side != 0 && side != avoidance_side_)
      side_switch_penalty = 3.0;

    candidate.clearance = clearance;
    candidate.score =
      8.0 * progress +
      1.2 * clearance -
      0.6 * std::abs(final_goal_heading) -
      0.15 * std::abs(steering) -
      0.20 * std::abs(steering - previous_steering_) -
      side_switch_penalty +
      0.5 * velocity;

    return candidate;
  }

  Candidate find_best_candidate(double goal_x, double goal_y, double distance) const {
    const std::array<double, 3> velocities = {0.07, 0.11, 0.15};
    const std::array<double, 9> steering_rates = {
      -0.9, -0.7, -0.5, -0.25, 0.0, 0.25, 0.5, 0.7, 0.9
    };

    Candidate best;

    for (double velocity : velocities) {
      for (double steering : steering_rates) {
        Candidate candidate = evaluate_candidate(
          velocity, steering, goal_x, goal_y, distance);

        if (candidate.safe && candidate.score > best.score)
          best = candidate;
      }
    }

    const std::array<double, 2> turn_rates = {-0.8, 0.8};

    for (double steering : turn_rates) {
      Candidate candidate = evaluate_candidate(
        0.0, steering, goal_x, goal_y, distance);

      candidate.score -= 0.5;

      if (candidate.safe && candidate.score > best.score)
        best = candidate;
    }

    return best;
  }

  void plan_command() {
    double goal_dx = goal_.x - pos_.x;
    double goal_dy = goal_.y - pos_.y;
    double distance = std::hypot(goal_dx, goal_dy);

    if (distance < 0.08) {
      stop();
      return;
    }

    double goal_angle = std::atan2(goal_dy, goal_dx);
    double goal_bearing = wrap_angle(goal_angle - current_angle_);

    double goal_x = distance * std::cos(goal_bearing);
    double goal_y = distance * std::sin(goal_bearing);

    Candidate straight = evaluate_candidate(
      0.15, 0.0, goal_x, goal_y, distance);

    bool direct_clear =
      straight.safe &&
      closest_front_ > 0.60;

    if (avoidance_side_ != 0) {
      if (direct_clear)
        direct_clear_scans_++;
      else
        direct_clear_scans_ = 0;

      if (direct_clear_scans_ >= 3) {
        avoidance_side_ = 0;
        direct_clear_scans_ = 0;
      }
    }

    Candidate best = find_best_candidate(goal_x, goal_y, distance);

    if (!best.safe) {
      speed_command_ = 0.0;

      if (avoidance_side_ == 0) {
        if (left_space_ > right_space_ + 0.10)
          avoidance_side_ = 1;
        else if (right_space_ > left_space_ + 0.10)
          avoidance_side_ = -1;
        else if (std::abs(previous_steering_) > 0.10)
          avoidance_side_ = steering_side(previous_steering_);
        else
          avoidance_side_ = goal_bearing >= 0.0 ? 1 : -1;
      }

      steering_command_ = 0.8 * avoidance_side_;
      previous_steering_ = steering_command_;
      return;
    }

    int best_side = steering_side(best.steering);

    if (avoidance_side_ == 0 &&
        !direct_clear &&
        best_side != 0) {
      avoidance_side_ = best_side;
      direct_clear_scans_ = 0;
    }

    double target_velocity = best.velocity;
    double target_steering = best.steering;

    const double hard_stop_distance = 0.30;

    if (closest_front_ < hard_stop_distance)
      target_velocity = 0.0;

    speed_command_ =
      0.50 * target_velocity +
      0.50 * speed_command_;

    steering_command_ =
      0.55 * target_steering +
      0.45 * steering_command_;

    if (closest_front_ < hard_stop_distance)
      speed_command_ = 0.0;

    previous_steering_ = target_steering;

    RCLCPP_INFO(
      this->get_logger(),
      "dist=%.2f front=%.2f left=%.2f right=%.2f side=%d best=(%.2f %.2f) score=%.2f",
      distance,
      closest_front_,
      left_space_,
      right_space_,
      avoidance_side_,
      best.velocity,
      best.steering,
      best.score);
  }

  void command_loop_function() {
    if (!goal_initialized_ || !odom_received_ || !scan_received_) return;

    if (new_scan_) {
      plan_command();
      new_scan_ = false;
    }

    Twist cmd;
    cmd.linear.x = speed_command_;
    cmd.angular.z = steering_command_;
    pub_twist_->publish(cmd);
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
        "Start=(%.2f %.2f) heading=%.2f goal=(%.2f %.2f)",
        pos_.x, pos_.y, current_angle_, goal_.x, goal_.y);
    }
  }

  void laser_scan_callback(const LaserScan &msg) {
    const double planning_range = 2.5;
    const double side_range = 2.0;
    const double front_half_angle = 60.0 * M_PI / 180.0;
    const double side_inner_angle = 20.0 * M_PI / 180.0;
    const double side_outer_angle = 100.0 * M_PI / 180.0;

    obstacles_.clear();
    closest_front_ = std::numeric_limits<double>::infinity();

    double left_sum = 0.0;
    double right_sum = 0.0;
    int left_count = 0;
    int right_count = 0;

    for (size_t i = 0; i < msg.ranges.size(); i++) {
      double angle = wrap_angle(
        msg.angle_min + static_cast<double>(i) * msg.angle_increment);

      if (flip_lidar_)
        angle = -angle;

      double range = msg.ranges[i];

      if (std::isinf(range) && range > 0.0) {
        if (angle >= side_inner_angle && angle <= side_outer_angle) {
          left_sum += side_range;
          left_count++;
        }

        if (angle <= -side_inner_angle && angle >= -side_outer_angle) {
          right_sum += side_range;
          right_count++;
        }

        continue;
      }

      if (!std::isfinite(range)) continue;
      if (range < msg.range_min || range > msg.range_max) continue;

      if (std::abs(angle) < front_half_angle)
        closest_front_ = std::min(closest_front_, range);

      double space = std::min(range, side_range);

      if (angle >= side_inner_angle && angle <= side_outer_angle) {
        left_sum += space;
        left_count++;
      }

      if (angle <= -side_inner_angle && angle >= -side_outer_angle) {
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

    left_space_ = left_count > 0 ? left_sum / left_count : 0.0;
    right_space_ = right_count > 0 ? right_sum / right_count : 0.0;

    scan_received_ = true;
    new_scan_ = true;
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

