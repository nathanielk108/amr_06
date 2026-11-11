#include <cmath>
#include <chrono>
#include <cstddef>
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
  std::chrono::steady_clock::time_point last_odom_time_{};
  std::chrono::steady_clock::time_point last_scan_time_{};
  const std::chrono::milliseconds sensor_timeout_{1000};
  bool logged_first_odom_ = false;
  bool logged_first_scan_ = false;

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

  const char *state_name() const {
    switch (state_) {
      case RobotState::NAVIGATING: return "NAVIGATING";
      case RobotState::RECOVERY_ROTATE: return "RECOVERY_ROTATE";
      case RobotState::RECOVERY_FORWARD: return "RECOVERY_FORWARD";
    }
