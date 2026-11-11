#include <algorithm>
#include <cmath>
#include <deque>
#include <functional>
#include <limits>
#include <memory>
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

// One valid lidar return, expressed in the robot body frame
struct ScanPoint {
  double angle;  // rad, 0 = robot forward, +CCW
  double range;  // m
};

class Lab3Node : public rclcpp::Node {
private:
  // ROS 2 Handles
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Subscription<Odometry>::SharedPtr sub_odometry_;
  rclcpp::Subscription<LaserScan>::SharedPtr sub_laser_scan_;
  rclcpp::Publisher<Twist>::SharedPtr pub_twist_;

  // Vehicle State (Origin-Transformed Frame, same convention as lab2: pos_ + current_angle_)
  Vector3 pos_;
  Vector3 goal_;
  double current_angle_ = 0.0;

  bool odom_received_ = false;
  bool scan_received_ = false;

  // Origin Frame Calibration
  double origin_x_ = 0.0;
  double origin_y_ = 0.0;
  double origin_yaw_ = 0.0;
  bool origin_rendered_ = false;

  // LiDAR Data (pre-processed in the callback)
  std::vector<ScanPoint> obstacles_;   // finite returns = real obstacles
  size_t valid_rays_ = 0;              // finite + "inf" (no return) rays; 0 => lidar is dead
  rclcpp::Time last_scan_time_{0, 0, RCL_ROS_TIME};

  // Physical & Safety Parameters
  double robot_radius_ = 0.20;           // Disk abstraction radius (m)
  double buffer_clearance_ = 0.10;       // Safety buffer beyond disk abstraction (m)
  double min_safe_distance_ = 0.30;      // Stop / reject distance straight ahead (radius + buffer)
  double corridor_half_width_ = 0.25;    // Half-width of the swept "tube" used to test a heading
  double lookahead_ = 2.0;               // Distance (m) at which a heading counts as fully open
  double min_heading_clear_ = 0.60;      // A heading needs this much free distance to be a candidate
  double goal_tolerance_ = 0.10;         // Goal reach threshold (m)  (lab2 "margin")
  double lidar_yaw_offset_ = 0.0;        // Set if scan angle 0 is NOT the robot's front (rad)
  double scan_timeout_ = 0.5;            // Stop if scan is older than this (s)

  // Controller Tuning (same roles as lab2: kv, kp)
  double max_vel_ = 0.35;                // Linear speed cap (m/s)
  double max_ang_vel_ = M_PI / 3.0;      // Angular speed cap (rad/s)
  double kv_ = 0.5;                      // Velocity gain on distance
  double kp_ = 1.2;                      // Heading tracking gain
  double min_vel_ = 0.05;                // Avoid crawling when path is clear but short

  // Heading selection
  double search_fov_ = 100.0 * M_PI / 180.0;  // candidates in +/- 100 deg (ignores robot's own rear posts)
  double search_step_ = 5.0 * M_PI / 180.0;
  double w_dist_ = 0.55;                 // Weight: free distance along heading
  double w_goal_ = 0.45;                 // Weight: alignment with goal
  double w_prev_ = 0.20;                 // Hysteresis: penalty for changing heading (stops L/R dithering)
  double prev_heading_ = 0.0;            // Last chosen heading (body frame)

  // Stuck Detection & Recovery State Machine
  RobotState state_ = RobotState::NAVIGATING;
  std::deque<Vector3> pose_history_;     // one entry per 100 ms tick
  size_t history_len_ = 60;              // 6 s window
  double stuck_displacement_ = 0.10;     // < 10 cm net motion in the window => stuck
  int recovery_spin_dir_ = 1;            // +1 CCW, -1 CW
  int recovery_ticks_ = 0;
  int recovery_rotate_timeout_ = 100;    // 10 s
  double recovery_clear_ = 0.80;         // free distance needed to leave RECOVERY_ROTATE
  double recovery_start_x_ = 0.0;
  double recovery_start_y_ = 0.0;

  double wrap(double angle) {
    return std::atan2(std::sin(angle), std::cos(angle));
  }

public:
  explicit Lab3Node() : rclcpp::Node("lab3_node") {
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

    pos_.x = 0.0;
    pos_.y = 0.0;
    pos_.z = 0.0;
    goal_.x = 1.5;   // goal relative to start pose
    goal_.y = 0.0;
    goal_.z = 0.0;
  }

private:
  // ---------------------------------------------------------------------------
  // Free distance along a heading (body frame): the robot is treated as a tube of
  // half-width corridor_half_width_. Any obstacle point inside that tube limits
  // how far the robot can travel. This replaces the single-ray test, which let
  // the robot aim at gaps/headings that were narrower than its body.
  // ---------------------------------------------------------------------------
  double free_distance(double heading) {
    double free = lookahead_;
    for (const auto &p : obstacles_) {
      double rel = wrap(p.angle - heading);
      double forward = p.range * std::cos(rel);
      double lateral = std::abs(p.range * std::sin(rel));
      if (forward > 0.0 && lateral < corridor_half_width_ && forward < free) {
        free = forward;
      }
    }
    return free;
  }

  void stop_robot() {
    pub_twist_->publish(Twist());
  }

  void start_recovery(const char *reason) {
    // Spin toward the most open direction instead of a random one
    double best_free = -1.0;
    double best_h = 0.0;
    for (double h = -M_PI; h <= M_PI; h += 10.0 * M_PI / 180.0) {
      double f = free_distance(h);
      if (f > best_free) {
        best_free = f;
        best_h = h;
      }
    }
    recovery_spin_dir_ = (best_h >= 0.0) ? 1 : -1;
    recovery_ticks_ = 0;
    state_ = RobotState::RECOVERY_ROTATE;
    RCLCPP_WARN(this->get_logger(), "%s Spinning %s toward most open heading.", reason,
                recovery_spin_dir_ > 0 ? "CCW" : "CW");
  }

  void command_loop_function() {
    if (!odom_received_ || !scan_received_) {
      return;
    }

    // Fail-safe: stale or empty lidar => do not drive blind
    double scan_age = (this->now() - last_scan_time_).seconds();
    if (scan_age > scan_timeout_ || valid_rays_ == 0) {
      stop_robot();
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "Lidar stale/empty. Stopping.");
      return;
    }

    Twist cmd;
    double dist_to_goal = std::hypot(goal_.x - pos_.x, goal_.y - pos_.y);

    // 1. Goal Reached Check
    if (dist_to_goal <= goal_tolerance_) {
      stop_robot();
      RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "Goal Target Reached!");
      return;
    }

    // 2. Recovery State Machine
    if (state_ == RobotState::RECOVERY_ROTATE) {
      cmd.linear.x = 0.0;
      cmd.angular.z = recovery_spin_dir_ * max_ang_vel_;
      recovery_ticks_++;

      double forward_clearance = free_distance(0.0);
      if (forward_clearance >= recovery_clear_) {
        state_ = RobotState::RECOVERY_FORWARD;
        recovery_start_x_ = pos_.x;
        recovery_start_y_ = pos_.y;
        RCLCPP_INFO(this->get_logger(), "Safe corridor found (%.2fm). Driving forward 0.5m.", forward_clearance);
      } else if (recovery_ticks_ > recovery_rotate_timeout_) {
        // Never spin forever: give normal navigation another try
        state_ = RobotState::NAVIGATING;
        pose_history_.clear();
        RCLCPP_WARN(this->get_logger(), "Recovery rotate timed out. Resuming navigation.");
        stop_robot();
        return;
      }

      pub_twist_->publish(cmd);
      return;
    }

    if (state_ == RobotState::RECOVERY_FORWARD) {
      cmd.linear.x = 0.20;
      cmd.angular.z = 0.0;

      double dist_traveled = std::hypot(pos_.x - recovery_start_x_, pos_.y - recovery_start_y_);
      double forward_clearance = free_distance(0.0);

      if (dist_traveled >= 0.50 || forward_clearance < min_safe_distance_) {
        state_ = RobotState::NAVIGATING;
        pose_history_.clear();
        prev_heading_ = 0.0;
        RCLCPP_INFO(this->get_logger(), "Recovery escape completed. Resuming navigation.");
        stop_robot();
        return;
      }

      pub_twist_->publish(cmd);
      return;
    }

    // 3. Stuck Detection: net displacement over a 6 s window (catches dithering too)
    pose_history_.push_back(pos_);
    if (pose_history_.size() > history_len_) {
      pose_history_.pop_front();
    }
    if (pose_history_.size() == history_len_) {
      double net = std::hypot(pos_.x - pose_history_.front().x, pos_.y - pose_history_.front().y);
      if (net < stuck_displacement_) {
        pose_history_.clear();
        start_recovery("Stuck in trap/oscillation!");
        stop_robot();
        return;
      }
    }

    // 4. Desired heading (lab2 structure: desiredHeading -> headingError -> steering)
    double goal_heading_global = std::atan2(goal_.y - pos_.y, goal_.x - pos_.x);
    double goal_heading_body = wrap(goal_heading_global - current_angle_);

    double desired_body = goal_heading_body;

    // 4a. Go-to-goal if the tube toward the goal is open up to the goal (or lookahead)
    double goal_free = free_distance(goal_heading_body);
    bool goal_is_clear = goal_free >= std::min(dist_to_goal + goal_tolerance_, lookahead_);

    if (goal_is_clear) {
      desired_body = goal_heading_body;
    } else {
      // 4b. Otherwise head for the clearest heading, still biased toward the goal
      double best_score = -1e9;
      bool found = false;

      for (double h = -search_fov_; h <= search_fov_ + 1e-9; h += search_step_) {
        double f = free_distance(h);
        if (f < min_heading_clear_) {
          continue;
        }
        double score = w_dist_ * (f / lookahead_)
                     + w_goal_ * std::cos(wrap(h - goal_heading_body))
                     - w_prev_ * std::abs(wrap(h - prev_heading_)) / M_PI;
        if (score > best_score) {
          best_score = score;
          desired_body = h;
          found = true;
        }
      }

      if (!found) {
        start_recovery("No heading with enough clearance.");
        stop_robot();
        return;
      }
    }
    prev_heading_ = desired_body;

    double desired_heading = wrap(current_angle_ + desired_body);
    double heading_error = wrap(desired_heading - current_angle_);
    double steering = kp_ * heading_error;

    // 5. Velocity: kv * distance (lab2), where "distance" is limited by the obstacle straight ahead
    double ahead_free = free_distance(0.0);
    double velocity = 0.0;
    if (ahead_free > min_safe_distance_) {
      double usable = std::min(dist_to_goal, ahead_free - min_safe_distance_);
      velocity = std::clamp(kv_ * usable, min_vel_, max_vel_);
      velocity *= std::max(0.0, std::cos(heading_error));  // turn first, drive when aligned
    }
    // else: obstacle inside safety distance ahead -> rotate in place only

    cmd.linear.x = velocity;
    cmd.angular.z = std::clamp(steering, -max_ang_vel_, max_ang_vel_);

    pub_twist_->publish(cmd);
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
    obstacles_.clear();
    valid_rays_ = 0;

    for (size_t i = 0; i < msg.ranges.size(); ++i) {
      double r = msg.ranges[i];
      if (std::isnan(r) || r < 0.0) {
        continue;                                  // invalid reading
      }
      if (std::isinf(r) || r >= msg.range_max) {
        valid_rays_++;                             // no return = open space, NOT an obstacle
        continue;
      }
      if (r < msg.range_min) {
        continue;                                  // too close / self hit: unreliable
      }
      valid_rays_++;
      double angle = wrap(msg.angle_min + static_cast<double>(i) * msg.angle_increment + lidar_yaw_offset_);
      obstacles_.push_back({angle, r});
    }

    last_scan_time_ = this->now();
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
