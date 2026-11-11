#include <algorithm>
#include <cmath>
#include <deque>
#include <functional>
#include <limits>
#include <memory>
#include <string>
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
#include <irobot_create_msgs/msg/hazard_detection_vector.hpp>

using namespace std::chrono_literals;
using namespace std::placeholders;

using namespace sensor_msgs::msg;
using namespace geometry_msgs::msg;
using namespace nav_msgs::msg;

enum class RobotState {
  NAVIGATING,
  RECOVERY_BACKUP,  // reverse straight away from the closest scan point
  RECOVERY_ROTATE   // then spin away from the obstacle, then back to NAVIGATING (no blind forward drive)
};

// One valid lidar return, expressed in the robot body frame
struct ScanPoint {
  double angle;  // rad, 0 = robot forward, +CCW
  double range;  // m
};

// An obstacle point remembered in the odom/origin frame (survives leaving the lidar view
// or being closer than the lidar's range_min, e.g. a box we are touching)
struct MemoryPoint {
  double x;
  double y;
  rclcpp::Time stamp;
};

class Lab3Node : public rclcpp::Node {
private:
  // ROS 2 Handles
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Subscription<Odometry>::SharedPtr sub_odometry_;
  rclcpp::Subscription<LaserScan>::SharedPtr sub_laser_scan_;
  rclcpp::Subscription<irobot_create_msgs::msg::HazardDetectionVector>::SharedPtr sub_hazard_;
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
  std::vector<ScanPoint> obstacles_;        // live finite returns = real obstacles
  std::vector<ScanPoint> world_obstacles_;  // live + remembered obstacles (body frame), rebuilt every tick
  std::vector<MemoryPoint> memory_;         // obstacles we hit / got stuck on
  double memory_duration_ = 60.0;           // s a remembered obstacle stays active
  size_t memory_max_points_ = 400;

  // Bumper
  bool bump_pending_ = false;
  int bump_spin_dir_ = 0;                   // +1 CCW, -1 CW, 0 = decide from lidar
  size_t valid_rays_ = 0;              // finite + "inf" (no return) rays; 0 => lidar is dead
  rclcpp::Time last_scan_time_{0, 0, RCL_ROS_TIME};

  // Physical & Safety Parameters
  double robot_radius_ = 0.20;           // Disk abstraction radius (m)
  double buffer_clearance_ = 0.10;       // Safety buffer beyond disk abstraction (m)
  double min_safe_distance_ = 0.30;      // Stop / reject distance straight ahead (radius + buffer)
  double corridor_half_width_ = 0.30;    // Swept tube half-width = disk radius + safety buffer (min_safe_distance_)
  double brake_margin_ = 0.10;           // Extra stopping distance in front of min_safe_distance_
  double lookahead_ = 2.0;               // Distance (m) at which a heading counts as fully open
  double min_heading_clear_ = 0.60;      // A heading needs this much free distance to be a candidate
  double goal_distance_ = 10.0;          // Goal = this far straight ahead of the START pose/heading (m)
  double goal_tolerance_ = 0.10;         // Goal reach threshold (m)  (lab2 "margin")
  double lidar_yaw_offset_ = 0.0;        // Set if scan angle 0 is NOT the robot's front (rad)
  double scan_timeout_ = 0.5;            // Stop if scan is older than this (s)

  // Controller Tuning (same roles as lab2: kv, kp)
  double max_vel_ = 0.35;                // Linear speed cap (m/s)
  double max_ang_vel_ = M_PI / 3.0;      // Angular speed cap (rad/s)
  double kv_ = 0.5;                      // Velocity gain on distance
  double kp_ = 1.2;                      // Heading tracking gain

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
  int recovery_rotate_timeout_ = 70;     // 7 s (a bit more than one full revolution)
  double recovery_clear_ = 0.70;         // free distance needed (over a +/-20 deg wedge) to leave RECOVERY_ROTATE

  // Backing away from the closest scan point
  double backup_trigger_ = 0.45;         // back up if the closest point is nearer than this (m)
  double backup_clear_ = 0.50;           // stop backing once the closest point is this far (m)
  double backup_max_distance_ = 0.40;    // never reverse further than this per attempt (m)
  double backup_speed_ = 0.10;           // reverse speed (m/s)
  double backup_min_rear_ = 0.30;        // need this much free space behind (m) to reverse
  int backup_timeout_ = 50;              // 5 s
  double backup_start_x_ = 0.0;
  double backup_start_y_ = 0.0;
  bool backed_up_ = false;               // only one backup per recovery episode (no endless loops)
  double bump_bearing_ = 0.0;            // body-frame bearing of the bumper that fired (rad)
  double recovery_min_turn_ = 60.0 * M_PI / 180.0;  // always spin at least this far away before leaving
  double recovery_start_yaw_ = 0.0;

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

    sub_hazard_ = this->create_subscription<irobot_create_msgs::msg::HazardDetectionVector>(
        "/TTB06/hazard_detection",
        rclcpp::QoS(
            rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_sensor_data),
            rmw_qos_profile_sensor_data),
        std::bind(&Lab3Node::hazard_callback, this, _1));

    pub_twist_ = this->create_publisher<Twist>("/TTB06/cmd_vel", 10);

    pos_.x = 0.0;
    pos_.y = 0.0;
    pos_.z = 0.0;
    goal_.x = goal_distance_;   // goal 10 m straight ahead of the start pose (origin frame +x = start heading)
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
  double free_distance(double heading, double half_width = -1.0) {
    if (half_width < 0.0) {
      half_width = corridor_half_width_;
    }
    double free = lookahead_;
    for (const auto &p : world_obstacles_) {
      double rel = wrap(p.angle - heading);
      double forward = p.range * std::cos(rel);
      double lateral = std::abs(p.range * std::sin(rel));
      if (forward > 0.0 && lateral < half_width && forward < free) {
        free = forward;
      }
    }
    return free;
  }

  void stop_robot() {
    pub_twist_->publish(Twist());
  }

  // Store nearby live lidar points (global frame) so the box we hit stays "solid"
  // even when it is behind us, outside the FOV, or closer than the lidar's range_min.
  void remember_obstacles(double radius) {
    rclcpp::Time now = this->now();
    for (const auto &p : obstacles_) {
      if (p.range > radius) {
        continue;
      }
      double gx = pos_.x + p.range * std::cos(current_angle_ + p.angle);
      double gy = pos_.y + p.range * std::sin(current_angle_ + p.angle);

      bool duplicate = false;
      for (auto &m : memory_) {
        if (std::hypot(m.x - gx, m.y - gy) < 0.05) {
          m.stamp = now;   // refresh
          duplicate = true;
          break;
        }
      }
      if (!duplicate) {
        memory_.push_back({gx, gy, now});
      }
    }
    while (memory_.size() > memory_max_points_) {
      memory_.erase(memory_.begin());
    }
  }

  // Live + remembered obstacles, all expressed in the current body frame
  void build_world_obstacles() {
    world_obstacles_ = obstacles_;
    rclcpp::Time now = this->now();
    memory_.erase(
        std::remove_if(memory_.begin(), memory_.end(),
                       [&](const MemoryPoint &m) { return (now - m.stamp).seconds() > memory_duration_; }),
        memory_.end());

    double c = std::cos(current_angle_);
    double s = std::sin(current_angle_);
    for (const auto &m : memory_) {
      double dx = m.x - pos_.x;
      double dy = m.y - pos_.y;
      double bx = dx * c + dy * s;
      double by = -dx * s + dy * c;
      double r = std::hypot(bx, by);
      if (r < 1e-3 || r > lookahead_) {
        continue;
      }
      world_obstacles_.push_back({std::atan2(by, bx), r});
    }
  }

  // Spin AWAY from whatever is close in front: obstacle on the left -> spin CW, on the right -> CCW
  int away_spin_dir() {
    double side = 0.0;
    for (const auto &p : obstacles_) {
      if (p.range < 1.5 * min_safe_distance_ && std::abs(p.angle) <= 120.0 * M_PI / 180.0) {
        side += std::sin(p.angle) / p.range;   // closer points weigh more
      }
    }
    if (std::abs(side) > 1e-6) {
      return (side > 0.0) ? -1 : 1;
    }
    // Nothing close in front: fall back to the most open heading
    double best_free = -1.0;
    double best_h = 0.0;
    for (double h = -M_PI; h <= M_PI; h += 10.0 * M_PI / 180.0) {
      double f = free_distance(h);
      if (f > best_free) {
        best_free = f;
        best_h = h;
      }
    }
    return (best_h >= 0.0) ? 1 : -1;
  }

  // Closest point of the live + remembered obstacles (the thing we must back away from)
  bool closest_point(ScanPoint &closest) {
    if (world_obstacles_.empty()) {
      return false;
    }
    closest = world_obstacles_.front();
    for (const auto &p : world_obstacles_) {
      if (p.range < closest.range) {
        closest = p;
      }
    }
    return true;
  }

  // Reversing is only allowed if the closest point is ahead of us (so backing increases the gap)
  // and the space behind the robot body is free.
  bool should_back_up() {
    ScanPoint c;
    if (!closest_point(c)) {
      return false;
    }
    bool close_and_in_front = c.range < backup_trigger_ && std::abs(c.angle) <= 100.0 * M_PI / 180.0;
    bool rear_free = free_distance(M_PI, robot_radius_) > backup_min_rear_;
    return close_and_in_front && rear_free;
  }

  void begin_backup() {
    state_ = RobotState::RECOVERY_BACKUP;
    recovery_ticks_ = 0;
    backed_up_ = true;
    backup_start_x_ = pos_.x;
    backup_start_y_ = pos_.y;
    RCLCPP_WARN(this->get_logger(), "Backing away from closest obstacle.");
  }

  void begin_rotate() {
    state_ = RobotState::RECOVERY_ROTATE;
    recovery_ticks_ = 0;
    recovery_start_yaw_ = current_angle_;
    recovery_spin_dir_ = (bump_spin_dir_ != 0) ? bump_spin_dir_ : away_spin_dir();
    bump_spin_dir_ = 0;   // one-shot: only the episode that follows a bump uses the bumper side
    RCLCPP_WARN(this->get_logger(), "Spinning %s away from obstacle.", recovery_spin_dir_ > 0 ? "CCW" : "CW");
  }

  void start_recovery(const char *reason) {
    RCLCPP_WARN(this->get_logger(), "%s", reason);
    remember_obstacles(1.0);   // blacklist the box so the escape path never re-enters its inflated zone
    backed_up_ = false;
    if (should_back_up()) {
      begin_backup();          // first get away from the closest scan point, THEN spin
    } else {
      begin_rotate();
    }
  }

  // Clear enough over the whole forward wedge (+/- 20 deg), not just a single ray
  bool wedge_is_clear(double clearance) {
    for (double h = -20.0; h <= 20.0; h += 10.0) {
      if (free_distance(h * M_PI / 180.0) < clearance) {
        return false;
      }
    }
    return true;
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

    build_world_obstacles();

    Twist cmd;
    double dist_to_goal = std::hypot(goal_.x - pos_.x, goal_.y - pos_.y);

    // 1. Goal Reached Check
    if (dist_to_goal <= goal_tolerance_) {
      stop_robot();
      RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "Goal Target Reached!");
      return;
    }

    // 1b. Bumper contact: we already hit it. Stop, remember the box (even if it is closer than
    //     the lidar's range_min), then back off and spin away.
    if (bump_pending_) {
      bump_pending_ = false;
      stop_robot();
      // Virtual obstacle just in front of the bumper that fired
      memory_.push_back({pos_.x + 0.22 * std::cos(current_angle_ + bump_bearing_),
                         pos_.y + 0.22 * std::sin(current_angle_ + bump_bearing_), this->now()});
      build_world_obstacles();
      if (state_ == RobotState::NAVIGATING) {
        pose_history_.clear();
        start_recovery("Bumper triggered!");
      } else {
        remember_obstacles(1.0);   // already recovering: just refresh the box in memory
      }
      return;
    }

    // 2a. Recovery: reverse straight away from the closest scan point
    if (state_ == RobotState::RECOVERY_BACKUP) {
      recovery_ticks_++;
      double backed = std::hypot(pos_.x - backup_start_x_, pos_.y - backup_start_y_);
      ScanPoint c;
      double closest = closest_point(c) ? c.range : lookahead_;
      bool rear_free = free_distance(M_PI, robot_radius_) > backup_min_rear_;

      if (backed >= backup_max_distance_ || closest >= backup_clear_ || !rear_free ||
          recovery_ticks_ > backup_timeout_) {
        stop_robot();
        begin_rotate();
        return;
      }

      cmd.linear.x = -backup_speed_;
      cmd.angular.z = 0.0;
      pub_twist_->publish(cmd);
      return;
    }

    // 2b. Recovery: spin away from the box (no forced forward drive)
    if (state_ == RobotState::RECOVERY_ROTATE) {
      recovery_ticks_++;
      double turned = std::abs(wrap(current_angle_ - recovery_start_yaw_));

      // Leave only after turning well away AND the whole forward wedge is open
      // (remembered box included). Navigation then takes over; its speed limit and
      // tube test keep the disk + safety radius out of the box's keep-out zone.
      if (turned >= recovery_min_turn_ && wedge_is_clear(recovery_clear_)) {
        state_ = RobotState::NAVIGATING;
        pose_history_.clear();
        prev_heading_ = 0.0;
        RCLCPP_INFO(this->get_logger(), "Turned %.0f deg away, wedge clear. Resuming navigation.",
                    turned * 180.0 / M_PI);
        stop_robot();
        return;
      }
      if (recovery_ticks_ > recovery_rotate_timeout_) {
        // Spinning did not open a path (e.g. wedged in close): reverse from the closest point
        // once, otherwise give navigation another try instead of spinning forever.
        stop_robot();
        if (!backed_up_ && should_back_up()) {
          begin_backup();
        } else {
          state_ = RobotState::NAVIGATING;
          pose_history_.clear();
          prev_heading_ = 0.0;
          RCLCPP_WARN(this->get_logger(), "Recovery rotate timed out. Resuming navigation.");
        }
        return;
      }

      cmd.linear.x = 0.0;
      cmd.angular.z = recovery_spin_dir_ * max_ang_vel_;
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

    // 5. Velocity: kv * distance (lab2), limited by the obstacle straight ahead.
    //    Anything inside the disk + safety radius in the direction of travel => no forward motion.
    double ahead_free = free_distance(0.0);
    double stop_distance = min_safe_distance_ + brake_margin_;
    double velocity = 0.0;
    if (ahead_free > stop_distance) {
      double usable = std::min(dist_to_goal, ahead_free - stop_distance);
      velocity = std::clamp(kv_ * usable, 0.0, max_vel_);
      velocity *= std::max(0.0, std::cos(heading_error));  // turn first, drive when aligned
    }
    // else: rotate in place only

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
      // The origin frame is anchored to the robot's pose on the first odom message, with +x along
      // its heading at that moment, so "10 m ahead of the start" is simply (goal_distance_, 0).
      goal_.x = goal_distance_;
      goal_.y = 0.0;
      RCLCPP_INFO(this->get_logger(), "Start pose captured. Goal is %.1f m straight ahead.", goal_distance_);
    }

    // Map to relative starting origin
    double dx = raw_x - origin_x_;
    double dy = raw_y - origin_y_;
    pos_.x = dx * std::cos(origin_yaw_) + dy * std::sin(origin_yaw_);
    pos_.y = -dx * std::sin(origin_yaw_) + dy * std::cos(origin_yaw_);
    current_angle_ = wrap(raw_yaw - origin_yaw_);

    odom_received_ = true;
  }

  void hazard_callback(const irobot_create_msgs::msg::HazardDetectionVector &msg) {
    for (const auto &d : msg.detections) {
      if (d.type != irobot_create_msgs::msg::HazardDetection::BUMP) {
        continue;
      }
      bump_pending_ = true;
      // Spin away from the bumper that fired (frame id e.g. bump_left / bump_front_center / bump_right)
      const std::string &f = d.header.frame_id;
      if (f.find("left") != std::string::npos) {
        bump_spin_dir_ = -1;
        bump_bearing_ = 30.0 * M_PI / 180.0;
      } else if (f.find("right") != std::string::npos) {
        bump_spin_dir_ = 1;
        bump_bearing_ = -30.0 * M_PI / 180.0;
      } else {
        bump_spin_dir_ = 0;   // centre bump: decide from the lidar
        bump_bearing_ = 0.0;
      }
    }
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
