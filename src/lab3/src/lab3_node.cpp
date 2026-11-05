#include <cmath>
#include <functional>
#include <memory>
#include <algorithm>

#include <rclcpp/rclcpp.hpp>
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
  Vector3 goal_;

  double current_angle_ = 0.0;
  bool goal_initialized_ = false;

  // Lidar sectors
  double front_ = 10.0;
  double front_left_ = 10.0;
  double front_right_ = 10.0;
  double left_ = 10.0;
  double right_ = 10.0;

  // Adjust this if lidar angle 0 is not robot-forward
  double lidar_offset_ = M_PI / 2.0;

  enum AvoidMode {
    GO_TO_GOAL,
    AVOID_LEFT,
    AVOID_RIGHT
  };

  AvoidMode avoid_mode_ = GO_TO_GOAL;

  int clear_count_ = 0;

public:
  explicit Lab3Node() : rclcpp::Node("lab3_node") {

    timer_ = this->create_wall_timer(
        100ms,
        std::bind(&Lab3Node::command_loop_function, this));

    sub_odometry_ = this->create_subscription<Odometry>(
        "/TTB06/odom",
        rclcpp::QoS(
            rclcpp::QoSInitialization::from_rmw(
                rmw_qos_profile_sensor_data),
            rmw_qos_profile_sensor_data),
        std::bind(&Lab3Node::odometry_callback, this, _1));

    sub_laser_scan_ = this->create_subscription<LaserScan>(
        "/TTB06/scan",
        rclcpp::QoS(
            rclcpp::QoSInitialization::from_rmw(
                rmw_qos_profile_sensor_data),
            rmw_qos_profile_sensor_data),
        std::bind(&Lab3Node::laser_scan_callback, this, _1));

    pub_twist_ =
        this->create_publisher<Twist>(
            "/TTB06/cmd_vel", 10);

    pos_.x = 0.0;
    pos_.y = 0.0;

    goal_.x = 0.0;
    goal_.y = 0.0;
  }

  void command_loop_function() {

    if (!goal_initialized_) {
      return;
    }

    double distance =
        std::hypot(
            goal_.x - pos_.x,
            goal_.y - pos_.y);

    double desired_heading =
        std::atan2(
            goal_.y - pos_.y,
            goal_.x - pos_.x);

    double heading_error =
        std::atan2(
            std::sin(
                desired_heading - current_angle_),
            std::cos(
                desired_heading - current_angle_));

    // Approximate robot footprint
    double robot_radius = 0.18;
    double safety_margin = 0.12;

    double minimum_clearance =
        robot_radius + safety_margin;

    double detect_distance = 0.75;
    double exit_distance = 0.55;

    double speed = 0.2;
    double steering = 0.0;

    /*
     * Normal go-to-goal behavior
     */
    if (avoid_mode_ == GO_TO_GOAL) {

      steering = 0.8 * heading_error;

      bool obstacle_ahead =
          front_ < detect_distance ||
          front_left_ < minimum_clearance ||
          front_right_ < minimum_clearance;

      if (obstacle_ahead) {

        clear_count_ = 0;

        /*
         * Choose the side with more space.
         * Once chosen, stay in that avoidance mode.
         */
        if ((left_ + front_left_) >
            (right_ + front_right_)) {

          avoid_mode_ = AVOID_LEFT;

          RCLCPP_INFO(
              this->get_logger(),
              "Obstacle detected: avoiding LEFT");
        }
        else {

          avoid_mode_ = AVOID_RIGHT;

          RCLCPP_INFO(
              this->get_logger(),
              "Obstacle detected: avoiding RIGHT");
        }
      }
    }

    /*
     * Avoid obstacle by turning left
     */
    if (avoid_mode_ == AVOID_LEFT) {

      speed = 0.10;
      steering = 0.7;

      /*
       * If obstacle gets too close to the front
       * or right side, turn harder.
       */
      if (front_ < minimum_clearance ||
          front_right_ < minimum_clearance ||
          right_ < minimum_clearance) {

        speed = 0.05;
        steering = 1.0;
      }

      /*
       * When avoiding left, obstacle should pass
       * along the robot's right side.
       *
       * Do not return to goal behavior until the
       * front and right side stay clear.
       */
      if (front_ > exit_distance &&
          front_right_ > exit_distance &&
          right_ > exit_distance) {

        clear_count_++;
      }
      else {

        clear_count_ = 0;
      }

      if (clear_count_ > 8) {

        avoid_mode_ = GO_TO_GOAL;
        clear_count_ = 0;

        RCLCPP_INFO(
            this->get_logger(),
            "Obstacle cleared: returning to goal");
      }
    }

    /*
     * Avoid obstacle by turning right
     */
    else if (avoid_mode_ == AVOID_RIGHT) {

      speed = 0.10;
      steering = -0.7;

      /*
       * If obstacle gets too close to the front
       * or left side, turn harder.
       */
      if (front_ < minimum_clearance ||
          front_left_ < minimum_clearance ||
          left_ < minimum_clearance) {

        speed = 0.05;
        steering = -1.0;
      }

      /*
       * When avoiding right, obstacle should pass
       * along the robot's left side.
       */
      if (front_ > exit_distance &&
          front_left_ > exit_distance &&
          left_ > exit_distance) {

        clear_count_++;
      }
      else {

        clear_count_ = 0;
      }

      if (clear_count_ > 8) {

        avoid_mode_ = GO_TO_GOAL;
        clear_count_ = 0;

        RCLCPP_INFO(
            this->get_logger(),
            "Obstacle cleared: returning to goal");
      }
    }

    steering =
        std::clamp(
            steering,
            -1.0,
            1.0);

    Twist cmd;

    /*
     * Stop when goal is reached
     */
    if (distance < 0.05) {

      cmd.linear.x = 0.0;
      cmd.angular.z = 0.0;
    }
    else {

      cmd.linear.x = speed;
      cmd.angular.z = steering;
    }

    pub_twist_->publish(cmd);

    RCLCPP_INFO_THROTTLE(
        this->get_logger(),
        *this->get_clock(),
        500,
        "mode=%d F=%.2f FL=%.2f FR=%.2f L=%.2f R=%.2f dist=%.2f v=%.2f w=%.2f",
        avoid_mode_,
        front_,
        front_left_,
        front_right_,
        left_,
        right_,
        distance,
        cmd.linear.x,
        cmd.angular.z);
  }

  void odometry_callback(
      const Odometry &msg) {

    pos_.x =
        msg.pose.pose.position.x;

    pos_.y =
        msg.pose.pose.position.y;

    double qx =
        msg.pose.pose.orientation.x;

    double qy =
        msg.pose.pose.orientation.y;

    double qz =
        msg.pose.pose.orientation.z;

    double qw =
        msg.pose.pose.orientation.w;

    current_angle_ =
        std::atan2(
            2.0 * (qw * qz + qx * qy),
            1.0 -
            2.0 * (qy * qy + qz * qz));

    /*
     * Set goal once from initial position
     * and initial heading.
     */
    if (!goal_initialized_) {

      goal_.x =
          pos_.x +
          1.5 * std::cos(current_angle_);

      goal_.y =
          pos_.y +
          1.5 * std::sin(current_angle_);

      goal_initialized_ = true;

      RCLCPP_INFO(
          this->get_logger(),
          "Start=(%.2f, %.2f) heading=%.2f goal=(%.2f, %.2f)",
          pos_.x,
          pos_.y,
          current_angle_,
          goal_.x,
          goal_.y);
    }
  }

  void laser_scan_callback(
      const LaserScan &msg) {

    // Reset sectors for each scan
    front_ = 10.0;
    front_left_ = 10.0;
    front_right_ = 10.0;
    left_ = 10.0;
    right_ = 10.0;

    for (size_t i = 0;
         i < msg.ranges.size();
         ++i) {

      double range =
          msg.ranges[i];

      if (!std::isfinite(range)) {
        continue;
      }

      if (range < msg.range_min ||
          range > msg.range_max) {
        continue;
      }

      double angle =
          msg.angle_min +
          msg.angle_increment * i;

      /*
       * Convert lidar frame into robot-forward frame.
       */
      angle += lidar_offset_;

      angle =
          std::atan2(
              std::sin(angle),
              std::cos(angle));

      /*
       * Front:
       * -15 to +15 degrees
       */
      if (std::abs(angle) <
          M_PI / 12.0) {

        front_ =
            std::min(
                front_,
                range);
      }

      /*
       * Front Left:
       * +15 to +60 degrees
       */
      else if (
          angle >= M_PI / 12.0 &&
          angle < M_PI / 3.0) {

        front_left_ =
            std::min(
                front_left_,
                range);
      }

      /*
       * Front Right:
       * -60 to -15 degrees
       */
      else if (
          angle <= -M_PI / 12.0 &&
          angle > -M_PI / 3.0) {

        front_right_ =
            std::min(
                front_right_,
                range);
      }

      /*
       * Left:
       * +60 to +100 degrees
       */
      else if (
          angle >= M_PI / 3.0 &&
          angle <
          5.0 * M_PI / 9.0) {

        left_ =
            std::min(
                left_,
                range);
      }

      /*
       * Right:
       * -100 to -60 degrees
       */
      else if (
          angle <= -M_PI / 3.0 &&
          angle >
          -5.0 * M_PI / 9.0) {

        right_ =
            std::min(
                right_,
                range);
      }
    }
  }
};

int main(
    int argc,
    char **argv) {

  rclcpp::init(
      argc,
      argv);

  Lab3Node::SharedPtr control_node =
      std::make_shared<Lab3Node>();

  rclcpp::executors::SingleThreadedExecutor exec;

  exec.add_node(
      control_node);

  exec.spin();

  rclcpp::shutdown();

  return 0;
}
