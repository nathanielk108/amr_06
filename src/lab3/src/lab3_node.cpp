#include <algorithm>
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

struct ScanPoint {
  double angle;
  double range;
};

class Lab3Node : public rclcpp::Node {
private:
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Subscription<Odometry>::SharedPtr sub_odometry_;
  rclcpp::Subscription<LaserScan>::SharedPtr sub_laser_scan_;
  rclcpp::Publisher<Twist>::SharedPtr pub_twist_;
  rclcpp::Client<ResetPose>::SharedPtr reset_client_;

  bool reset_sent_ = false;
  bool reset_done_ = false;         
  int wait_ticks_ = 0;               
  rclcpp::Time settle_until_{0, 0, RCL_ROS_TIME}; 

  Vector3 pos_;
  Vector3 goal_;
  double current_angle_ = 0.0;
  bool odom_received_ = false;
  bool scan_received_ = false;

  double origin_x_ = 0.0;
  double origin_y_ = 0.0;
  double origin_yaw_ = 0.0;
  bool origin_set_ = false;

  std::vector<ScanPoint> obstacles_;
  double ang_inc_ = 0.0175;         
  bool lidar_ok_ = false;
  rclcpp::Time last_scan_time_{0, 0, RCL_ROS_TIME};

  double goal_distance_ = 10.0;   
  double goal_margin_ = 0.10;        
  double lidar_yaw_offset_ = M_PI / 2.0;    // lidar mounting offset, was approximaetly 90 deg for rviz
  double robot_radius_ = 0.15;       // disk abstraction
  double side_margin_ = 0.05;     
  double front_stop_ = 0.35;      
  double clear_distance_ = 1.0;   
  double lookahead_ = 2.0;        
  double influence_ = 0.6;     
  double repel_gain_ = 3.0;        
  double stale_scan_ = 0.5;        

  double kv_ = 0.5;  // velocity gain 
  double kp_ = 1.2;   // heading gain
  double max_vel_ = 0.35;        
  double max_ang_vel_ = M_PI / 3.0;  
  double reverse_vel_ = 0.10;        

  double detour_heading_ = 0.0;  // global angle of  chosen detour
  int detour_ticks_ = 0;
  int detour_hold_ = 15;             

  double wrap(double angle) {
    return std::atan2(std::sin(angle), std::cos(angle));
  }

public:
  explicit Lab3Node() : rclcpp::Node("lab3_node") {
    timer_ = this->create_wall_timer(100ms, std::bind(&Lab3Node::command_loop_function, this));

    sub_odometry_ = this->create_subscription<Odometry>(
        "/TTB06/odom",
        rclcpp::QoS(rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_sensor_data), rmw_qos_profile_sensor_data),
        std::bind(&Lab3Node::odometry_callback, this, _1));

    sub_laser_scan_ = this->create_subscription<LaserScan>(
        "/TTB06/scan",
        rclcpp::QoS(rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_sensor_data), rmw_qos_profile_sensor_data),
        std::bind(&Lab3Node::laser_scan_callback, this, _1));

    pub_twist_ = this->create_publisher<Twist>("/TTB06/cmd_vel", 10);
    reset_client_ = this->create_client<ResetPose>("/TTB06/reset_pose");

    pos_.x = 0.0;
    pos_.y = 0.0;
    goal_.x = goal_distance_;
    goal_.y = 0.0;
  }

private:
  double free_distance(double heading, double half_width) {
    double free = lookahead_;
    for (const auto &p : obstacles_) {
      double rel = wrap(p.angle - heading);
      double forward = p.range * std::cos(rel);
      double lateral = std::abs(p.range * std::sin(rel));
      if (forward > 0.05 && lateral < half_width - 1e-3 && forward < free) {  
        free = forward;
      }
    }
    return free;
  }

  double repulsion() {
    double push = 0.0;
    for (const auto &p : obstacles_) {
      if (p.range >= influence_ || std::abs(p.angle) > 110.0 * M_PI / 180.0) {
        continue;
      }
      double w = (influence_ - p.range) / influence_;   // 0 to 1 for closeness
      push -= std::sin(p.angle) * w * w * ang_inc_;
    }
    return repel_gain_ * push;
  }

  double choose_heading(double goal_heading_body, double half_width) {
    double open_heading = goal_heading_body, open_err = 1e9;
    double best_heading = goal_heading_body, best_free = -1.0;

    for (int k = -18; k <= 18; ++k) {  // 5 deg steps from -90 to 90         
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

  bool odom_reset_ready() { // just resets geometry, goal seemed to hold old info and turn the robot away
    if (reset_done_) {
      return true;
    }
    wait_ticks_++;
    if (!reset_sent_) {
      if (reset_client_->service_is_ready()) {
        auto req = std::make_shared<ResetPose::Request>();
        req->pose.orientation.w = 1.0;   
        reset_client_->async_send_request(req, [this](rclcpp::Client<ResetPose>::SharedFuture) {
          if (reset_done_) {
            return;                      
          }
          origin_set_ = false;          
          odom_received_ = false;
          settle_until_ = this->now() + rclcpp::Duration::from_seconds(1.0);
          reset_done_ = true;
        });
        reset_sent_ = true;
      } else if (wait_ticks_ > 30) {
        reset_done_ = true;
      }
    } else if (wait_ticks_ > 60) {
      reset_done_ = true;
    }
    return reset_done_;
  }

  void command_loop_function() {
    if (!odom_reset_ready() || !odom_received_ || !scan_received_) {
      return;
    }

    if ((this->now() - last_scan_time_).seconds() > stale_scan_ || !lidar_ok_) {
      pub_twist_->publish(Twist());
      return;
    }

    double half_width = robot_radius_ + side_margin_; 

    double distance = std::hypot(goal_.x - pos_.x, goal_.y - pos_.y);
    if (distance < goal_margin_) {
      pub_twist_->publish(Twist());
      RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "Goal Reached!");
      return;
    }
    double goal_heading_body = wrap(std::atan2(goal_.y - pos_.y, goal_.x - pos_.x) - current_angle_);

    double desired = goal_heading_body;
    bool goal_open = free_distance(goal_heading_body, half_width) >= std::min(distance, clear_distance_);

    double detour_body = wrap(detour_heading_ - current_angle_);
    bool detour_open = detour_ticks_ > 0 && std::abs(detour_body) <= M_PI / 2.0 && free_distance(detour_body, half_width) >= clear_distance_;

    if (detour_open) {
      desired = detour_body;    // keep detour
      detour_ticks_ = goal_open ? detour_ticks_ - 1 : detour_hold_;
    } 
    else if (!goal_open) {
      desired = choose_heading(goal_heading_body, half_width);
      detour_heading_ = wrap(current_angle_ + desired);  
      detour_ticks_ = detour_hold_;
    } 
    else {
      detour_ticks_ = 0;
    }

    double repel = repulsion();
    double steering = kp_ * desired + repel; // mixing go_to_goal and obstacle repulsion

    double path_free = std::min({free_distance(0.0, half_width),
                                 free_distance(0.5 * desired, half_width),
                                 free_distance(desired, half_width)});

    double velocity = 0.0;

    if (path_free > front_stop_) {
      velocity = std::clamp(kv_ * std::min(distance, path_free - front_stop_), 0.0, max_vel_);
      velocity *= std::max(0.2, std::cos(desired));
    } 
    else if (free_distance(M_PI, half_width) > 0.30) {
      velocity = -reverse_vel_;  // back off from obstacle
    }                               

    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
        "pos(%.2f, %.2f) goal_dist %.2f | goal_hdg %.0f, desired %.0f deg | path_free %.2f | repel %.2f | v %.2f",
        pos_.x, pos_.y, distance, goal_heading_body * 180.0 / M_PI, desired * 180.0 / M_PI,
        path_free, repel, velocity); // big status statement for robot processing

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
    double raw_yaw = std::atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz)); // same as lab1 and lab2

    if (!origin_set_ && this->now() < settle_until_ && (std::hypot(raw_x, raw_y) > 0.05 || std::abs(raw_yaw) > 0.05)) {
      return;
    }

    if (!origin_set_) { // starting pose odom reset
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
        lidar_ok_ = true;  
      }

      if (std::isnan(r) || std::isinf(r) || r < msg.range_min || r >= msg.range_max) {
        continue; // we found that the lidar was throwing away readings, looked online and it seems like this will only take the right measurements out
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
