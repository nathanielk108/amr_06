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

/*
#include <algorithm>
#include <chrono>
#include <cmath>
#include <functional>
#include <memory>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/executors/single_threaded_executor.hpp>
#include <rclcpp/qos.hpp>

#include <rmw/qos_profiles.h>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <irobot_create_msgs/srv/reset_pose.hpp>

using namespace std::chrono_literals;
using namespace std::placeholders;

using namespace sensor_msgs::msg;
using namespace geometry_msgs::msg;
using namespace nav_msgs::msg;
using ResetPose = irobot_create_msgs::srv::ResetPose;

// One lidar return in the robot body frame
struct ScanPoint {
  double angle;  // rad, 0 = robot forward, +CCW (left)
  double range;  // m
};

class Lab3Node : public rclcpp::Node {
private:
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Subscription<Odometry>::SharedPtr sub_odometry_;
  rclcpp::Subscription<LaserScan>::SharedPtr sub_laser_scan_;
  rclcpp::Publisher<Twist>::SharedPtr pub_twist_;
  rclcpp::Client<ResetPose>::SharedPtr reset_client_;

  // Odom reset at start: odom is set to (0, 0, 0), then the goal is (10, 0) in that frame
  bool reset_sent_ = false;
  bool reset_done_ = false;          // reply received OR fallback taken
  int wait_ticks_ = 0;               // 100 ms ticks spent waiting for the reset service
  rclcpp::Time settle_until_{0, 0, RCL_ROS_TIME};  // skip stale pre-reset odom until this time

  // Vehicle state (start frame: +x = heading at start, same convention as lab2)
  Vector3 pos_;
  Vector3 goal_;
  double current_angle_ = 0.0;
  bool odom_received_ = false;
  bool scan_received_ = false;

  double origin_x_ = 0.0;
  double origin_y_ = 0.0;
  double origin_yaw_ = 0.0;
  bool origin_set_ = false;

  // Live lidar data only (no memory between scans)
  std::vector<ScanPoint> obstacles_;
  double ang_inc_ = 0.0175;          // rad between rays (updated from the scan)
  bool lidar_ok_ = false;
  rclcpp::Time last_scan_time_{0, 0, RCL_ROS_TIME};

  // Geometry / safety
  double goal_distance_ = 10.0;      // goal: this far ahead of the start pose (m)
  double goal_margin_ = 0.10;        // lab2 "margin" (m)
  double lidar_yaw_offset_ = 0.0;    // lidar mounting offset: rad added to every ray angle (0 = ray 0 is forward)
  double robot_radius_ = 0.15;       // disk abstraction (m)
  double side_margin_ = 0.05;        // extra sideways margin for the travel tube (m)
  double front_stop_ = 0.35;         // never drive forward with an obstacle closer than this (m, from lidar)
  double clear_distance_ = 1.0;      // a heading is "open" if this much room is free along it (m)
  double lookahead_ = 2.0;           // lidar distance considered (m)
  double influence_ = 0.6;           // obstacles closer than this push the robot away (m)
  double repel_gain_ = 3.0;          // strength of that push (rad/s)
  double stale_scan_ = 0.5;          // scan older than this = dead lidar (s)

  // Controller (lab2 style)
  double kv_ = 0.5;                  // velocity gain on distance
  double kp_ = 1.2;                  // heading gain
  double max_vel_ = 0.35;            // m/s
  double max_ang_vel_ = M_PI / 3.0;  // rad/s (60 deg/s)
  double reverse_vel_ = 0.10;        // m/s, only when driving forward would ram

  // Detour commitment (stops the heading flipping left/right every tick)
  double detour_heading_ = 0.0;      // global angle of the chosen detour
  int detour_ticks_ = 0;
  int detour_hold_ = 15;             // keep a detour at least 1.5 s

  double wrap(double angle) {
    return std::atan2(std::sin(angle), std::cos(angle));
  }

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
    reset_client_ = this->create_client<ResetPose>("/TTB06/reset_pose");

    pos_.x = 0.0;
    pos_.y = 0.0;
    goal_.x = goal_distance_;
    goal_.y = 0.0;
  }

private:
  // How far a tube of half-width `half_width` can travel along `heading` (body frame) before an
  // obstacle point is inside it. Capped at lookahead_.
  // Points with forward <= 0.05 m are beside the robot, not in its path (a wall next to the robot
  // must not freeze it: that is handled by repulsion()). Do NOT use forward > 0 here: a ray at
  // ~90 degrees gives forward ~1e-17 and froze the robot.
  double free_distance(double heading, double half_width) {
    double free = lookahead_;
    for (const auto &p : obstacles_) {
      double rel = wrap(p.angle - heading);
      double forward = p.range * std::cos(rel);
      double lateral = std::abs(p.range * std::sin(rel));
      if (forward > 0.05 && lateral < half_width - 1e-3 && forward < free) {   // 1 mm: float noise at the tube edge
        free = forward;
      }
    }
    return free;
  }

  // Sideways push away from nearby obstacles (steering, rad/s).
  // A point on the RIGHT (negative angle) gives a positive (left-turning) push, and vice versa.
  double repulsion() {
    double push = 0.0;
    for (const auto &p : obstacles_) {
      if (p.range >= influence_ || std::abs(p.angle) > 110.0 * M_PI / 180.0) {
        continue;
      }
      double w = (influence_ - p.range) / influence_;   // 0 (far) .. 1 (touching)
      push -= std::sin(p.angle) * w * w * ang_inc_;
    }
    return repel_gain_ * push;
  }

  // Goal direction blocked: of all open headings (>= clear_distance_ free) take the one closest
  // to the goal. If none is open, take the one with the most free room.
  double choose_heading(double goal_heading_body, double half_width) {
    double open_heading = goal_heading_body, open_err = 1e9;
    double best_heading = goal_heading_body, best_free = -1.0;

    for (int k = -18; k <= 18; ++k) {                    // -90 .. +90 deg in 5 deg steps
      double h = k * 5.0 * M_PI / 180.0;
      double free = free_distance(h, half_width);
      double err = std::abs(wrap(h - goal_heading_body));

      if (free >= clear_distance_ && err < open_err) {
        open_err = err;
        open_heading = h;
      }
      if (free > best_free + 1e-6) {
        best_free = free;
        best_heading = h;
      }
    }
    return (open_err < 1e9) ? open_heading : best_heading;
  }

  // Ask the Create 3 to reset its odometry to (0, 0, 0). Falls back to the software start
  // frame (current pose = 0, 0, 0) if the service is unavailable for 3 s or silent for 6 s.
  bool odom_reset_ready() {
    if (reset_done_) {
      return true;
    }
    wait_ticks_++;
    if (!reset_sent_) {
      if (reset_client_->service_is_ready()) {
        auto req = std::make_shared<ResetPose::Request>();
        req->pose.orientation.w = 1.0;   // position 0,0,0 and yaw 0
        reset_client_->async_send_request(req, [this](rclcpp::Client<ResetPose>::SharedFuture) {
          if (reset_done_) {
            return;                      // already fell back: a late reply must not re-zero the frame mid-run
          }
          origin_set_ = false;           // re-capture the start frame from the reset odom
          odom_received_ = false;
          settle_until_ = this->now() + rclcpp::Duration::from_seconds(1.0);
          reset_done_ = true;
          RCLCPP_INFO(this->get_logger(), "Odom reset to (0, 0, 0).");
        });
        reset_sent_ = true;
      } else if (wait_ticks_ > 30) {
        RCLCPP_WARN(this->get_logger(), "reset_pose service unavailable. Using current pose as (0, 0, 0).");
        reset_done_ = true;
      }
    } else if (wait_ticks_ > 60) {
      RCLCPP_WARN(this->get_logger(), "reset_pose got no reply. Using current pose as (0, 0, 0).");
      reset_done_ = true;
    }
    return reset_done_;
  }

  void command_loop_function() {
    if (!odom_reset_ready() || !odom_received_ || !scan_received_) {
      return;
    }

    // Never drive on stale or empty lidar data
    if ((this->now() - last_scan_time_).seconds() > stale_scan_ || !lidar_ok_) {
      pub_twist_->publish(Twist());
      return;
    }

    double half_width = robot_radius_ + side_margin_;   // travel tube

    // --- go_to_goal (lab2) ---
    double distance = std::hypot(goal_.x - pos_.x, goal_.y - pos_.y);
    if (distance < goal_margin_) {
      pub_twist_->publish(Twist());
      RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "Goal Reached!");
      return;
    }
    double goal_heading_body = wrap(std::atan2(goal_.y - pos_.y, goal_.x - pos_.x) - current_angle_);

    // --- heading: goal if its path is open, otherwise a committed detour toward open space ---
    double desired = goal_heading_body;
    bool goal_open = free_distance(goal_heading_body, half_width) >= std::min(distance, clear_distance_);

    double detour_body = wrap(detour_heading_ - current_angle_);
    bool detour_open = detour_ticks_ > 0 && std::abs(detour_body) <= M_PI / 2.0 &&
                       free_distance(detour_body, half_width) >= clear_distance_;

    if (detour_open) {
      desired = detour_body;                       // keep the detour, do not flip sides
      detour_ticks_ = goal_open ? detour_ticks_ - 1 : detour_hold_;
    } else if (!goal_open) {
      desired = choose_heading(goal_heading_body, half_width);
      detour_heading_ = wrap(current_angle_ + desired);   // stored as a GLOBAL angle
      detour_ticks_ = detour_hold_;
    } else {
      detour_ticks_ = 0;
    }

    // --- steering: toward the heading, pushed away from nearby obstacles on either side ---
    double repel = repulsion();
    double steering = kp_ * desired + repel;

    // --- velocity: kv * distance (lab2), limited by the room along the path actually driven ---
    // The robot arcs from straight ahead (0) toward `desired`, so check both and the middle.
    double path_free = std::min({free_distance(0.0, half_width),
                                 free_distance(0.5 * desired, half_width),
                                 free_distance(desired, half_width)});
    double velocity = 0.0;
    if (path_free > front_stop_) {
      velocity = std::clamp(kv_ * std::min(distance, path_free - front_stop_), 0.0, max_vel_);
      velocity *= std::max(0.2, std::cos(desired));   // slow in sharp turns, keep arcing
    } else if (free_distance(M_PI, half_width) > 0.30) {
      velocity = -reverse_vel_;                       // driving forward would ram: back off while steering
    }                                                 // else: stay still, never drive into it

    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
        "pos(%.2f, %.2f) goal_dist %.2f | goal_hdg %.0f, desired %.0f deg | path_free %.2f | repel %.2f | v %.2f",
        pos_.x, pos_.y, distance, goal_heading_body * 180.0 / M_PI, desired * 180.0 / M_PI,
        path_free, repel, velocity);

    Twist cmd;
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

    // After the reset reply, skip stale pre-reset messages (odom not yet back near 0,0,0) for at
    // most ~1 s; the software start frame below covers any leftover offset.
    if (!origin_set_ && this->now() < settle_until_ &&
        (std::hypot(raw_x, raw_y) > 0.05 || std::abs(raw_yaw) > 0.05)) {
      return;
    }

    // Start frame = odom after the reset (0, 0, 0). Goal = (goal_distance_, 0): 10 m along +x.
    if (!origin_set_) {
      origin_x_ = raw_x;
      origin_y_ = raw_y;
      origin_yaw_ = raw_yaw;
      origin_set_ = true;
      goal_.x = goal_distance_;
      goal_.y = 0.0;
      RCLCPP_INFO(this->get_logger(), "Start odom (%.2f, %.2f, yaw %.0f deg). Goal = odom (%.2f, %.2f)",
                  raw_x, raw_y, raw_yaw * 180.0 / M_PI,
                  raw_x + goal_distance_ * std::cos(raw_yaw), raw_y + goal_distance_ * std::sin(raw_yaw));
    }

    double dx = raw_x - origin_x_;
    double dy = raw_y - origin_y_;
    pos_.x = dx * std::cos(origin_yaw_) + dy * std::sin(origin_yaw_);
    pos_.y = -dx * std::sin(origin_yaw_) + dy * std::cos(origin_yaw_);
    current_angle_ = wrap(raw_yaw - origin_yaw_);

    odom_received_ = true;
  }

  void laser_scan_callback(const LaserScan &msg) {
    obstacles_.clear();
    lidar_ok_ = false;
    if (msg.angle_increment > 0.0) {
      ang_inc_ = msg.angle_increment;
    }
    for (size_t i = 0; i < msg.ranges.size(); ++i) {
      double r = msg.ranges[i];
      if (!std::isnan(r)) {
        lidar_ok_ = true;                // inf counts as alive: it means open space
      }
      // NaN / inf / out-of-range = no obstacle
      if (std::isnan(r) || std::isinf(r) || r < msg.range_min || r >= msg.range_max) {
        continue;
      }
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
*/



