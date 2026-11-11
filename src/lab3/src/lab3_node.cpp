#include <cmath>
#include <cstdlib>
#include <ctime>
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

enum class RobotState {
  NAVIGATING,
  RECOVERY_ROTATE,
  RECOVERY_FORWARD
};

class Lab3Node : public rclcpp::Node {
private:
  // ROS 2 Handles
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Subscription<Odometry>::SharedPtr sub_odometry_;
  rclcpp::Subscription<LaserScan>::SharedPtr sub_laser_scan_;
  rclcpp::Publisher<Twist>::SharedPtr pub_twist_;

  // Vehicle State (Origin-Transformed Frame)
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

  // LiDAR Scan Data
  LaserScan latest_scan_;

  // Physical & Safety Parameters
  double robot_radius_ = 0.20;           // Disk abstraction radius (m)
  double buffer_clearance_ = 0.10;       // Safety buffer beyond disk abstraction (10 cm)
  double min_safe_distance_ = 0.30;      // Total clearance threshold (robot_radius + buffer)
  double max_lidar_range_ = 12.0;        // Max effective LiDAR range (m)
  double goal_tolerance_ = 0.10;         // Goal reach threshold (m)

  // Controller Tuning
  double max_vel_ = 0.35;                // Linear target speed (m/s)
  double max_ang_vel_ = M_PI / 3.0;      // Angular speed cap (rad/s)
  double kg_ = 1.2;                      // Heading tracking gain

  // Stuck Detection & Recovery State Machine
  RobotState state_ = RobotState::NAVIGATING;
  int stuck_ticks_ = 0;
  int recovery_spin_dir_ = 1;            // +1 for CCW, -1 for CW
  double recovery_start_x_ = 0.0;
  double recovery_start_y_ = 0.0;
  double last_progress_x_ = 0.0;
  double last_progress_y_ = 0.0;

  double wrap(double angle) {
    return std::atan2(std::sin(angle), std::cos(angle));
  }

public:
  explicit Lab3Node() : rclcpp::Node("lab3_node") {
    std::srand(static_cast<unsigned int>(std::time(nullptr)));

    // 10 Hz Control Loop (100 ms)
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

    // Default target location relative to start pose
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

    // 2. Recovery State Machine Logic
    if (state_ == RobotState::RECOVERY_ROTATE) {
      // Spin in place to find a safe corridor
      cmd.linear.x = 0.0;
      cmd.angular.z = recovery_spin_dir_ * max_ang_vel_;

      // Inspect forward wedge (+/- 30 degrees) for a 0.5m clearance corridor
      double forward_clearance = get_forward_clearance();
      if (forward_clearance >= (0.50 + min_safe_distance_)) {
        state_ = RobotState::RECOVERY_FORWARD;
        recovery_start_x_ = pos_.x;
        recovery_start_y_ = pos_.y;
        RCLCPP_INFO(this->get_logger(), "Safe corridor found (%.2fm). Driving forward 0.5m.", forward_clearance);
      }

      pub_twist_->publish(cmd);
      return;
    }

    if (state_ == RobotState::RECOVERY_FORWARD) {
      // Drive straight out of the trap
      cmd.linear.x = 0.20;
      cmd.angular.z = 0.0;

      double dist_traveled = std::hypot(pos_.x - recovery_start_x_, pos_.y - recovery_start_y_);
      double forward_clearance = get_forward_clearance();

      // Resume normal navigation once 0.5m is traversed or if an immediate obstacle blocks front
      if (dist_traveled >= 0.50 || forward_clearance < min_safe_distance_) {
        state_ = RobotState::NAVIGATING;
        stuck_ticks_ = 0;
        last_progress_x_ = pos_.x;
        last_progress_y_ = pos_.y;
        RCLCPP_INFO(this->get_logger(), "Recovery escape completed. Resuming navigation.");
      }

      pub_twist_->publish(cmd);
      return;
    }

    // 3. Normal Navigation: Check for Stuck Condition / Oscillations
    double move_progress = std::hypot(pos_.x - last_progress_x_, pos_.y - last_progress_y_);
    if (move_progress < 0.03) {
      stuck_ticks_++;
    } else {
      stuck_ticks_ = std::max(0, stuck_ticks_ - 1);
      last_progress_x_ = pos_.x;
      last_progress_y_ = pos_.y;
    }

    // Trigger recovery if trapped for > 2.5 seconds (25 ticks)
    if (stuck_ticks_ > 25) {
      state_ = RobotState::RECOVERY_ROTATE;
      recovery_spin_dir_ = (std::rand() % 2 == 0) ? 1 : -1;
      stuck_ticks_ = 0;
      RCLCPP_WARN(this->get_logger(), "Stuck in trap/oscillation! Initiating spin recovery.");
      return;
    }

    // 4. Furthest Distance & Goal-Weighted Scan Heading Selection
    double goal_heading_global = std::atan2(goal_.y - pos_.y, goal_.x - pos_.x);
    double goal_heading_body = wrap(goal_heading_global - current_angle_);

    double best_score = -1e9;
    double best_heading_angle = goal_heading_body;
    bool found_safe_ray = false;

    // Weighting coefficients
    const double w_dist = 0.55;  // Weight for open distance (up to 12m)
    const double w_goal = 0.45;  // Weight for goal heading alignment

    for (size_t i = 0; i < latest_scan_.ranges.size(); ++i) {
      double r = latest_scan_.ranges[i];

      if (std::isnan(r) || std::isinf(r) || r < latest_scan_.range_min) {
        continue;
      }

      double angle = wrap(latest_scan_.angle_min + static_cast<double>(i) * latest_scan_.angle_increment);
      double r_clamped = std::clamp(r, 0.0, max_lidar_range_);

      // Reject rays within 10cm buffer beyond disk abstraction (0.30 m)
      if (r_clamped < min_safe_distance_) {
        continue;
      }

      // Compute normalized clearance score [0.0, 1.0] and goal alignment score [-1.0, 1.0]
      double dist_score = r_clamped / max_lidar_range_;
      double goal_score = std::cos(angle - goal_heading_body);

      double score = (w_dist * dist_score) + (w_goal * goal_score);

      if (score > best_score) {
        best_score = score;
        best_heading_angle = angle;
        found_safe_ray = true;
      }
    }

    // If no safe ray is found (surrounded within 30cm), trigger immediate spin recovery
    if (!found_safe_ray) {
      state_ = RobotState::RECOVERY_ROTATE;
      recovery_spin_dir_ = (std::rand() % 2 == 0) ? 1 : -1;
      stuck_ticks_ = 0;
      RCLCPP_WARN(this->get_logger(), "No safe paths (>30cm) found. Entering recovery spin.");
      return;
    }

    // 5. Generate Output Motion Commands
    double steering = kg_ * best_heading_angle;
    double alignment_factor = std::max(0.0, std::cos(best_heading_angle));

    cmd.linear.x = max_vel_ * alignment_factor;
    cmd.angular.z = std::clamp(steering, -max_ang_vel_, max_ang_vel_);

    pub_twist_->publish(cmd);
  }

  double get_forward_clearance() {
    double min_front = std::numeric_limits<double>::infinity();

    for (size_t i = 0; i < latest_scan_.ranges.size(); ++i) {
      double r = latest_scan_.ranges[i];
      if (std::isnan(r) || std::isinf(r) || r < latest_scan_.range_min || r > latest_scan_.range_max) {
        continue;
      }

      double angle = wrap(latest_scan_.angle_min + static_cast<double>(i) * latest_scan_.angle_increment);

      // Forward wedge (+/- 30 degrees)
      if (std::abs(angle) <= (M_PI / 6.0)) {
        if (r < min_front) {
          min_front = r;
        }
      }
    }
    return min_front;
  }

  void odometry_callback(const Odometry &msg) {
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
      last_progress_x_ = 0.0;
      last_progress_y_ = 0.0;
    }

    // Map to relative starting origin
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
