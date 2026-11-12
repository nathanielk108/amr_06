#include <cmath>
#include <algorithm>
#include <chrono>
#include <functional>
#include <limits>
#include <memory>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/qos.hpp>
#include <rmw/qos_profiles.h>

#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <irobot_create_msgs/srv/reset_pose.hpp>

using namespace std::chrono_literals;
using namespace std::placeholders;

using geometry_msgs::msg::Twist;
using geometry_msgs::msg::Vector3;
using nav_msgs::msg::Odometry;
using sensor_msgs::msg::LaserScan;
using irobot_create_msgs::srv::ResetPose;

class Lab3Node : public rclcpp::Node {
private:
  // ROS 2 Communication Handles
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Subscription<Odometry>::SharedPtr sub_odometry_;
  rclcpp::Subscription<LaserScan>::SharedPtr sub_laser_scan_;
  rclcpp::Publisher<Twist>::SharedPtr pub_twist_;
  rclcpp::Client<ResetPose>::SharedPtr client_reset_pose_;

  // Vehicle State (in start frame)
  Vector3 pos_{};
  Vector3 goal_{};
  double current_angle_ = 0.0;

  // Odometry Calibration Flags & Raw Offsets
  bool odom_received_ = false;
  bool origin_rendered_ = false;
  double origin_x_ = 0.0;
  double origin_y_ = 0.0;
  double origin_yaw_ = -90.0;
  rclcpp::Time reset_trigger_time_{0, 0, RCL_ROS_TIME};

  // Laser Scan Storage
  LaserScan latest_scan_;
  bool scan_received_ = false;
  rclcpp::Time last_scan_time_{0, 0, RCL_ROS_TIME};

  // Detour Commitment (Hysteresis)
  double committed_heading_ = 0.0;
  bool has_committed_detour_ = false;
  rclcpp::Time detour_commit_time_{0, 0, RCL_ROS_TIME};

  // Configuration Parameters
  double robot_radius_ = 0.15;
  double side_margin_ = 0.05;
  double front_stop_ = 0.35;
  double clear_distance_ = 1.0;
  double repel_gain_ = 3.0;
  double kv_ = 0.5;
  double kp_ = 1.5;
  double margin_ = 0.10;
  double max_vel_ = 0.35;
  double max_ang_vel_ = 60.0 * M_PI / 180.0; // 60 deg/s in rad/s
  double lidar_yaw_offset_ = 0.0;

  // Helper functions
  static double wrap(double angle) {
    return std::atan2(std::sin(angle), std::cos(angle));
  }

public:
  explicit Lab3Node() : rclcpp::Node("lab3_node") {
    // Parameters
    robot_radius_ = declare_parameter<double>("robot_radius", 0.15);
    side_margin_ = declare_parameter<double>("side_margin", 0.05);
    front_stop_ = declare_parameter<double>("front_stop", 0.35);
    clear_distance_ = declare_parameter<double>("clear_distance", 1.0);
    repel_gain_ = declare_parameter<double>("repel_gain", 3.0);
    kv_ = declare_parameter<double>("kv", 0.5);
    kp_ = declare_parameter<double>("kp", 1.5);
    margin_ = declare_parameter<double>("margin", 0.10);
    max_vel_ = declare_parameter<double>("max_vel", 0.35);
    max_ang_vel_ = declare_parameter<double>("max_ang_vel", 60.0 * M_PI / 180.0);
    lidar_yaw_offset_ = declare_parameter<double>("lidar_yaw_offset", 0.0);

    // Subscriptions and Publishers
    sub_odometry_ = create_subscription<Odometry>(
        "/TTB06/odom",
        rclcpp::QoS(rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_sensor_data), rmw_qos_profile_sensor_data),
        std::bind(&Lab3Node::odometry_callback, this, _1));

    sub_laser_scan_ = create_subscription<LaserScan>(
        "/TTB06/scan",
        rclcpp::QoS(rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_sensor_data), rmw_qos_profile_sensor_data),
        std::bind(&Lab3Node::laser_scan_callback, this, _1));

    pub_twist_ = create_publisher<Twist>("/TTB06/cmd_vel", 10);
    client_reset_pose_ = create_client<ResetPose>("/TTB06/reset_pose");

    // 10 Hz Timer Loop
    timer_ = create_wall_timer(100ms, std::bind(&Lab3Node::command_loop_function, this));

    // Reset pose asynchronous call
    reset_trigger_time_ = this->now();
    reset_robot_pose();
  }

private:
  void reset_robot_pose() {
    if (!client_reset_pose_->wait_for_service(3s)) {
      RCLCPP_WARN(get_logger(), "ResetPose service unavailable after 3s; using current pose as origin.");
      return;
    }

    auto req = std::make_shared<ResetPose::Request>();
    req->pose.position.x = 0.0;
    req->pose.position.y = 0.0;
    req->pose.position.z = 0.0;
    req->pose.orientation.x = 0.0;
    req->pose.orientation.y = 0.0;
    req->pose.orientation.z = 0.0;
    req->pose.orientation.w = 1.0;

    client_reset_pose_->async_send_request(
        req, std::bind(&Lab3Node::reset_pose_response_callback, this, _1));
  }

  void reset_pose_response_callback(rclcpp::ClientGoalHandle<ResetPose>::SharedFuture future) {
    try {
      (void)future.get();
      RCLCPP_INFO(get_logger(), "Successfully called /TTB06/reset_pose.");
    } catch (const std::exception &e) {
      RCLCPP_WARN(get_logger(), "ResetPose service call failed: %s", e.what());
    }
  }

  void odometry_callback(const Odometry &msg) {
    const double stamp = rclcpp::Time(msg.header.stamp).seconds();
    const double trig_time = reset_trigger_time_.seconds();

    // Skip stale pre-reset odometry messages (up to ~1s post request)
    if (!origin_rendered_ && (stamp - trig_time < 0.8)) {
      return;
    }

    double raw_x = msg.pose.pose.position.x;
    double raw_y = msg.pose.pose.position.y;

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

      // Goal set to (10.0, 0.0) in the verified start frame
      goal_.x = 10.0;
      goal_.y = 0.0;

      RCLCPP_INFO(get_logger(), "Start odom (%.2f, %.2f, %.2f). Goal = odom (%.2f, %.2f)",
                  0.0, 0.0, 0.0, goal_.x, goal_.y);
    }

    double dx = raw_x - origin_x_;
    double dy = raw_y - origin_y_;
    pos_.x = dx * std::cos(origin_yaw_) + dy * std::sin(origin_yaw_);
    pos_.y = -dx * std::sin(origin_yaw_) + dy * std::cos(origin_yaw_);
    current_angle_ = wrap(raw_yaw - origin_yaw_);

    odom_received_ = true;
  }

  void laser_scan_callback(const LaserScan &msg) {
    latest_scan_ = msg;
    scan_received_ = true;
    last_scan_time_ = this->now();
  }

  // Pure functions for LIDAR calculations
  double free_distance(double heading_body, double half_width) const {
    if (!scan_received_ || latest_scan_.ranges.empty()) {
      return 0.0;
    }

    double max_lookahead = 2.0;
    double min_free = max_lookahead;

    for (size_t i = 0; i < latest_scan_.ranges.size(); ++i) {
      double r = latest_scan_.ranges[i];
      if (std::isnan(r) || r < latest_scan_.range_min) {
        continue; // Drop invalid readings
      }

      // inf or range >= range_max represents open space
      if (std::isinf(r) || r >= latest_scan_.range_max) {
        continue;
      }

      double raw_angle = latest_scan_.angle_min + static_cast<double>(i) * latest_scan_.angle_increment;
      double angle = wrap(raw_angle + lidar_yaw_offset_);

      double forward = r * std::cos(angle - heading_body);
      double lateral = std::abs(r * std::sin(angle - heading_body));

      // 0.05m threshold prevents side walls (forward ~ 0) from triggering blockages
      if (forward > 0.05 && lateral < half_width) {
        if (forward < min_free) {
          min_free = forward;
        }
      }
    }
    return min_free;
  }

  double compute_side_repulsion() const {
    if (!scan_received_ || latest_scan_.ranges.empty()) {
      return 0.0;
    }

    double repulsion = 0.0;
    const double influence = 0.6;
    const double max_angle_rad = 110.0 * M_PI / 180.0;

    for (size_t i = 0; i < latest_scan_.ranges.size(); ++i) {
      double r = latest_scan_.ranges[i];
      if (std::isnan(r) || std::isinf(r) || r < latest_scan_.range_min || r >= latest_scan_.range_max) {
        continue;
      }

      double raw_angle = latest_scan_.angle_min + static_cast<double>(i) * latest_scan_.angle_increment;
      double angle = wrap(raw_angle + lidar_yaw_offset_);

      if (r <= influence && std::abs(angle) <= max_angle_rad) {
        double w = (influence - r) / influence;
        // Point on right (angle < 0) pushes left (positive repulsion)
        repulsion += -std::sin(angle) * (w * w) * latest_scan_.angle_increment * repel_gain_;
      }
    }
    return repulsion;
  }

  bool is_lidar_dead() const {
    if (!scan_received_) return true;
    if ((this->now() - last_scan_time_).seconds() > 0.5) return true;

    size_t valid_rays = 0;
    for (double r : latest_scan_.ranges) {
      if (!std::isnan(r)) {
        valid_rays++;
      }
    }
    return (valid_rays == 0);
  }

  double choose_heading(double goal_heading_body, double dist_to_goal, double half_width) {
    double goal_free = free_distance(goal_heading_body, half_width);
    double target_clearance = std::min(dist_to_goal, clear_distance_);

    if (goal_free >= target_clearance) {
      has_committed_detour_ = false;
      return goal_heading_body;
    }

    // Check if committed detour remains valid (held >= 1.5s)
    if (has_committed_detour_) {
      double committed_body = wrap(committed_heading_ - current_angle_);
      if (free_distance(committed_body, half_width) >= clear_distance_) {
        return committed_body;
      }
      // Lost clearance; reset commitment
      has_committed_detour_ = false;
    }

    // Sweep +/- 90 degrees in 5-degree steps
    double best_heading = 0.0;
    double min_goal_diff = std::numeric_limits<double>::infinity();
    double max_open_dist = -1.0;
    bool found_clear_candidate = false;

    for (int deg = -90; deg <= 90; deg += 5) {
      double sweep_angle = wrap(goal_heading_body + (deg * M_PI / 180.0));
      double free_dist = free_distance(sweep_angle, half_width);

      if (free_dist >= clear_distance_) {
        found_clear_candidate = true;
        double diff = std::abs(wrap(sweep_angle - goal_heading_body));
        if (diff < min_goal_diff) {
          min_goal_diff = diff;
          best_heading = sweep_angle;
        }
      } else if (!found_clear_candidate) {
        if (free_dist > max_open_dist) {
          max_open_dist = free_dist;
          best_heading = sweep_angle;
        }
      }
    }

    // Commit to chosen detour angle in global frame
    committed_heading_ = wrap(best_heading + current_angle_);
    has_committed_detour_ = true;
    detour_commit_time_ = this->now();

    return best_heading;
  }

  void command_loop_function() {
    if (!odom_received_) {
      return;
    }

    Twist cmd{};

    // Handle dead LIDAR safety
    if (is_lidar_dead()) {
      pub_twist_->publish(cmd);
      return;
    }

    // Goal distance and heading calculations
    double dist = std::hypot(goal_.x - pos_.x, goal_.y - pos_.y);
    if (dist < margin_) {
      pub_twist_->publish(cmd);
      RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 2000, "Goal reached successfully!");
      return;
    }

    double goal_heading_global = std::atan2(goal_.y - pos_.y, goal_.x - pos_.x);
    double goal_heading_body = wrap(goal_heading_global - current_angle_);

    double half_width = robot_radius_ + side_margin_; // 0.20 m
    double desired_heading_body = choose_heading(goal_heading_body, dist, half_width);

    // Compute steering with side repulsion
    double repulsion = compute_side_repulsion();
    double raw_steering = (kp_ * desired_heading_body) + repulsion;
    double steering = std::clamp(raw_steering, -max_ang_vel_, max_ang_vel_);

    // Compute velocity over driven arc
    double free_straight = free_distance(0.0, half_width);
    double free_mid = free_distance(0.5 * desired_heading_body, half_width);
    double free_desired = free_distance(desired_heading_body, half_width);

    double path_free = std::min({free_straight, free_mid, free_desired});
    double velocity = 0.0;

    if (path_free > front_stop_) {
      double target_v = kv_ * std::min(dist, path_free - front_stop_);
      velocity = std::clamp(target_v, 0.0, max_vel_);
      velocity *= std::max(0.2, std::cos(desired_heading_body));
    } else if (free_distance(M_PI, half_width) > 0.30) {
      // Slow reverse if forward blocked and rear is clear
      velocity = -0.10;
    } else {
      velocity = 0.0;
    }

    cmd.linear.x = velocity;
    cmd.angular.z = steering;
    pub_twist_->publish(cmd);

    // Log telemetry at 1 Hz
    RCLCPP_INFO_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "Pos:(%.2f,%.2f) GoalDist:%.2fm GoalHead:%.2f rad DesHead:%.2f rad PathFree:%.2fm Repel:%.2f V:%.2fm/s",
        pos_.x, pos_.y, dist, goal_heading_body, desired_heading_body, path_free, repulsion, velocity);
  }
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<Lab3Node>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
