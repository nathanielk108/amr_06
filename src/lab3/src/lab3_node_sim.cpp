#include <cmath>
#include <functional>
#include <memory>
#include <algorithm>
#include <limits>

#include <rclcpp/rclcpp.hpp>
#include <rmw/qos_profiles.h>

#include <sensor_msgs/msg/laser_scan.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <geometry_msgs/msg/vector3.hpp>

using namespace std::chrono_literals;
using namespace std::placeholders;

using namespace sensor_msgs::msg;
using namespace geometry_msgs::msg;
using namespace nav_msgs::msg;

class Lab3Node : public rclcpp::Node {

private:
  rclcpp::TimerBase::SharedPtr timer_;

  rclcpp::Subscription<Odometry>::SharedPtr sub_odometry_;
  rclcpp::Subscription<LaserScan>::SharedPtr sub_laser_scan_;

  rclcpp::Publisher<TwistStamped>::SharedPtr pub_twist_;

  Vector3 pos_;
  Vector3 goal_;

  double current_angle_ = 0.0;
  bool goal_initialized_ = false;

  // Repulsive field calculated from lidar
  double repulsive_x_ = 0.0;
  double repulsive_y_ = 0.0;

  double closest_obstacle_ =
      std::numeric_limits<double>::infinity();

  bool obstacle_present_ = false;

  // Set so corrected lidar angle 0 points forward
  double lidar_offset_ = M_PI / 2.0;

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
        this->create_publisher<TwistStamped>(
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

    double goal_dx =
        goal_.x - pos_.x;

    double goal_dy =
        goal_.y - pos_.y;

    double distance =
        std::hypot(goal_dx, goal_dy);

    TwistStamped cmd;

    if (distance < 0.05) {

      cmd.header.set__stamp(this->get_clock()->now());
      cmd.twist.linear.x = 0.0;
      cmd.twist.angular.z = 0.0;

      pub_twist_->publish(cmd);
      return;
    }

    /*
     * Attractive field
     *
     * First calculate direction to goal in global frame,
     * then express it in the robot frame.
     */
    double goal_angle =
        std::atan2(goal_dy, goal_dx);

    double relative_goal_angle =
        std::atan2(
            std::sin(goal_angle - current_angle_),
            std::cos(goal_angle - current_angle_));

    double attractive_gain = 1.0;

    double attractive_magnitude =
        attractive_gain * std::min(distance, 1.0);

    double attractive_x =
        attractive_magnitude *
        std::cos(relative_goal_angle);

    double attractive_y =
        attractive_magnitude *
        std::sin(relative_goal_angle);

    /*
     * Total artificial potential field
     */
    double field_x =
        attractive_x + repulsive_x_;

    double field_y =
        attractive_y + repulsive_y_;

    /*
     * Perfectly centered obstacles can produce a field
     * directly backwards. Give it a small lateral bias
     * to break the symmetry.
     */
    if (obstacle_present_ &&
        field_x < 0.0 &&
        std::abs(field_y) < 0.05) {

      field_y = 0.05;
    }

    double desired_heading =
        std::atan2(field_y, field_x);

    /*
     * Convert field direction into angular velocity.
     */
    double turn_gain = 1.5;

    double steering =
        turn_gain * desired_heading;

    steering =
        std::clamp(steering, -1.0, 1.0);

    /*
     * Forward speed.
     *
     * Slow down when:
     *  1. desired direction is not straight ahead
     *  2. obstacle is close
     */
    double max_speed = 0.15;

    double heading_scale =
        std::max(
            0.0,
            std::cos(desired_heading));

    double obstacle_scale = 1.0;

    double slow_distance = 0.60;

    if (closest_obstacle_ < slow_distance) {

      obstacle_scale =
          closest_obstacle_ / slow_distance;

      obstacle_scale =
          std::clamp(
              obstacle_scale,
              0.0,
              1.0);
    }

    double speed =
        max_speed *
        heading_scale *
        obstacle_scale;

    /*
     * If the field points mostly sideways/backwards,
     * rotate first instead of driving toward obstacle.
     */
    if (std::abs(desired_heading) > M_PI / 3.0) {
      speed = 0.0;
    }

    cmd.header.set__stamp(this->get_clock()->now());
    cmd.twist.linear.x = speed;
    cmd.twist.angular.z = steering;

    pub_twist_->publish(cmd);

    RCLCPP_INFO_THROTTLE(
        this->get_logger(),
        *this->get_clock(),
        500,
        "dist=%.2f att=(%.2f %.2f) rep=(%.2f %.2f) field=(%.2f %.2f) obs=%.2f v=%.2f w=%.2f",
        distance,
        attractive_x,
        attractive_y,
        repulsive_x_,
        repulsive_y_,
        field_x,
        field_y,
        closest_obstacle_,
        cmd.twist.linear.x,
        cmd.twist.angular.z);
  }

  void odometry_callback(const Odometry &msg) {

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
     * Goal is 1.5 m directly ahead of initial pose.
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
          "Start=(%.2f %.2f) heading=%.2f goal=(%.2f %.2f)",
          pos_.x,
          pos_.y,
          current_angle_,
          goal_.x,
          goal_.y);
    }
  }

  void laser_scan_callback(const LaserScan &msg) {

    /*
     * Repulsive field parameters
     */
    double robot_radius = 0.18;
    double safety_margin = 0.10;

    double influence_distance = 0.75;
    double repulsive_gain = 0.08;

    double minimum_clearance = 0.05;

    repulsive_x_ = 0.0;
    repulsive_y_ = 0.0;

    closest_obstacle_ =
        std::numeric_limits<double>::infinity();

    obstacle_present_ = false;

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
       * Transform lidar angle into robot frame.
       */
      angle += lidar_offset_;

      angle =
          std::atan2(
              std::sin(angle),
              std::cos(angle));

      /*
       * Clearance from the outside of the robot,
       * not from the lidar sensor itself.
       */
      double clearance =
          range -
          robot_radius -
          safety_margin;

      /*
       * Keep track of actual measured obstacle range
       * for speed control.
       */
      closest_obstacle_ =
          std::min(
              closest_obstacle_,
              clearance);

      if (clearance > influence_distance) {
        continue;
      }

      obstacle_present_ = true;

      clearance =
          std::max(
              clearance,
              minimum_clearance);

      /*
       * Standard artificial potential field
       * repulsive magnitude:
       *
       * eta * (1/rho - 1/rho0) / rho^2
       */
      double magnitude =
          repulsive_gain *
          ((1.0 / clearance) -
           (1.0 / influence_distance)) /
          (clearance * clearance);

      /*
       * Integrate over lidar angle.
       *
       * Multiplying by angle_increment keeps the result
       * from depending heavily on lidar resolution.
       */
      magnitude *=
          std::abs(msg.angle_increment);

      /*
       * Lidar ray points toward obstacle.
       * Repulsive force points opposite the ray.
       */
      repulsive_x_ -=
          magnitude * std::cos(angle);

      repulsive_y_ -=
          magnitude * std::sin(angle);
    }

    /*
     * Avoid huge commands if something is nearly touching.
     */
    double max_repulsive = 4.0;

    double repulsive_length =
        std::hypot(
            repulsive_x_,
            repulsive_y_);

    if (repulsive_length > max_repulsive) {

      repulsive_x_ *=
          max_repulsive /
          repulsive_length;

      repulsive_y_ *=
          max_repulsive /
          repulsive_length;
    }
  }
};

int main(int argc, char **argv) {

  rclcpp::init(argc, argv);

  Lab3Node::SharedPtr control_node =
      std::make_shared<Lab3Node>();

  rclcpp::executors::SingleThreadedExecutor exec;

  exec.add_node(control_node);

  exec.spin();

  rclcpp::shutdown();

  return 0;
}
