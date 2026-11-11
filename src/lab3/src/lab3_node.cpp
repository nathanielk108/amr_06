#include <cmath>
#include <functional>
#include <memory>
#include <algorithm>
#include <limits>

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
  rclcpp::TimerBase::SharedPtr timer_;

  rclcpp::Subscription<Odometry>::SharedPtr sub_odometry_;
  rclcpp::Subscription<LaserScan>::SharedPtr sub_laser_scan_;

  rclcpp::Publisher<Twist>::SharedPtr pub_twist_;

  Vector3 pos_;
  Vector3 goal_;

  double current_angle_ = 0.0;

  bool goal_initialized_ = false;
  bool scan_received_ = false;

  double goal_distance_ = 1.5;

  /*
   * Filtered repulsive field.
   */
  double repulsive_x_ = 0.0;
  double repulsive_y_ = 0.0;

  /*
   * Closest obstacle anywhere.
   */
  double closest_obstacle_ =
      std::numeric_limits<double>::infinity();

  /*
   * Closest obstacle in the wider forward sector.
   * Used mainly for speed control.
   */
  double closest_front_obstacle_ =
      std::numeric_limits<double>::infinity();

  /*
   * Closest obstacle directly in front of the robot.
   * Used to determine whether the path is actually blocked.
   */
  double closest_center_obstacle_ =
      std::numeric_limits<double>::infinity();

  bool obstacle_present_ = false;

  /*
   * Persistent avoidance direction.
   *
   * -1 = right
   *  0 = none
   * +1 = left
   */
  int avoidance_side_ = 0;

  /*
   * Smoothed angular velocity command.
   */
  double steering_command_ = 0.0;

public:
  explicit Lab3Node()
      : rclcpp::Node("lab3_node") {

    /*
     * Example:
     *
     * ros2 run <package> <executable> \
     *   --ros-args -p goal_distance:=2.0
     */
    this->declare_parameter<double>(
        "goal_distance",
        1.5);

    goal_distance_ =
        this->get_parameter(
            "goal_distance").as_double();

    timer_ =
        this->create_wall_timer(
            100ms,
            std::bind(
                &Lab3Node::command_loop_function,
                this));

    sub_odometry_ =
        this->create_subscription<Odometry>(
            "/TTB06/odom",
            rclcpp::QoS(
                rclcpp::QoSInitialization::from_rmw(
                    rmw_qos_profile_sensor_data),
                rmw_qos_profile_sensor_data),
            std::bind(
                &Lab3Node::odometry_callback,
                this,
                _1));

    sub_laser_scan_ =
        this->create_subscription<LaserScan>(
            "/TTB06/scan",
            rclcpp::QoS(
                rclcpp::QoSInitialization::from_rmw(
                    rmw_qos_profile_sensor_data),
                rmw_qos_profile_sensor_data),
            std::bind(
                &Lab3Node::laser_scan_callback,
                this,
                _1));

    pub_twist_ =
        this->create_publisher<Twist>(
            "/TTB06/cmd_vel",
            10);

    pos_.x = 0.0;
    pos_.y = 0.0;

    goal_.x = 0.0;
    goal_.y = 0.0;
  }

  void command_loop_function() {

    if (!goal_initialized_ ||
        !scan_received_) {
      return;
    }

    double goal_dx =
        goal_.x - pos_.x;

    double goal_dy =
        goal_.y - pos_.y;

    double distance =
        std::hypot(
            goal_dx,
            goal_dy);

    Twist cmd;

    /*
     * Goal reached.
     */
    if (distance < 0.05) {

      steering_command_ = 0.0;

      cmd.linear.x = 0.0;
      cmd.angular.z = 0.0;

      pub_twist_->publish(cmd);

      return;
    }

    /*
     * Direction to goal in global frame.
     */
    double goal_angle =
        std::atan2(
            goal_dy,
            goal_dx);

    /*
     * Goal direction relative to robot.
     */
    double relative_goal_angle =
        std::atan2(
            std::sin(
                goal_angle -
                current_angle_),
            std::cos(
                goal_angle -
                current_angle_));

    /*
     * Attractive potential field.
     */
    double attractive_gain = 1.0;

    double attractive_magnitude =
        attractive_gain *
        std::min(
            distance,
            1.0);

    double attractive_x =
        attractive_magnitude *
        std::cos(
            relative_goal_angle);

    double attractive_y =
        attractive_magnitude *
        std::sin(
            relative_goal_angle);

    /*
     * Combine attractive and repulsive fields.
     */
    double field_x =
        attractive_x +
        repulsive_x_;

    double field_y =
        attractive_y +
        repulsive_y_;

    /*
     * Use a relatively narrow central sector to determine
     * whether an obstacle is actually blocking the path.
     *
     * Corridor walls should not cause the avoidance state
     * to remain latched indefinitely.
     */
    double avoidance_start_distance = 0.55;
    double avoidance_release_distance = 0.70;

    /*
     * Select an avoidance side only when something is
     * directly blocking the robot.
     */
    if (avoidance_side_ == 0 &&
        closest_center_obstacle_ <
            avoidance_start_distance) {

      if (repulsive_y_ > 0.03) {

        avoidance_side_ = 1;

      } else if (
          repulsive_y_ < -0.03) {

        avoidance_side_ = -1;

      } else {

        /*
         * Symmetric obstacle directly ahead.
         * Pick left to break the tie.
         */
        avoidance_side_ = 1;
      }
    }

    /*
     * Release the persistent side as soon as the central
     * path is sufficiently clear.
     *
     * Do not wait for the entire repulsive field to vanish,
     * because corridor walls may always create some
     * repulsive force.
     */
    if (avoidance_side_ != 0 &&
        closest_center_obstacle_ >
            avoidance_release_distance) {

      avoidance_side_ = 0;
    }

    /*
     * Apply a lateral avoidance bias while an obstacle is
     * genuinely blocking the center of the path.
     */
    if (avoidance_side_ != 0) {

      double proximity =
          1.0 -
          std::clamp(
              closest_center_obstacle_ /
                  avoidance_start_distance,
              0.0,
              1.0);

      /*
       * Stronger lateral bias as obstacle gets closer.
       */
      double avoidance_bias =
          0.20 +
          0.50 * proximity;

      field_y +=
          avoidance_bias *
          avoidance_side_;

      /*
       * Prevent obstacle repulsion from producing a field
       * pointing directly behind the robot.
       */
      double minimum_forward_field =
          0.05;

      field_x =
          std::max(
              field_x,
              minimum_forward_field);
    }

    double desired_heading =
        std::atan2(
            field_y,
            field_x);

    /*
     * If the robot has rotated so far that the goal is
     * behind it, abandon the current avoidance maneuver
     * and deliberately recover toward the goal.
     */
    bool goal_recovery =
        std::abs(
            relative_goal_angle) >
        M_PI / 2.0;

    if (goal_recovery) {

      avoidance_side_ = 0;

      double max_recovery_heading =
          60.0 *
          M_PI /
          180.0;

      desired_heading =
          std::clamp(
              relative_goal_angle,
              -max_recovery_heading,
              max_recovery_heading);

    } else if (obstacle_present_) {

      /*
       * Normal obstacle avoidance is also restricted to
       * +/-60 degrees so the APF cannot command a near
       * 180-degree reversal.
       */
      double max_avoid_heading =
          60.0 *
          M_PI /
          180.0;

      desired_heading =
          std::clamp(
              desired_heading,
              -max_avoid_heading,
              max_avoid_heading);
    }

    /*
     * Angular controller.
     */
    double turn_gain = 1.5;

    double steering_target =
        turn_gain *
        desired_heading;

    steering_target =
        std::clamp(
            steering_target,
            -1.0,
            1.0);

    /*
     * Smooth angular command.
     */
    double steering_alpha =
        0.40;

    steering_command_ =
        steering_alpha *
            steering_target +
        (1.0 -
         steering_alpha) *
            steering_command_;

    /*
     * Remove very small angular commands.
     */
    if (std::abs(
            steering_command_) <
        0.02) {

      steering_command_ = 0.0;
    }

    /*
     * Forward speed controller.
     */
    double max_speed = 0.15;

    double minimum_avoidance_speed =
        0.03;

    double stop_distance =
        0.15;

    double slow_distance =
        0.70;

    double speed =
        max_speed;

    /*
     * Reduce speed based on obstacles in the wider
     * forward sector.
     */
    if (closest_front_obstacle_ <=
        stop_distance) {

      speed = 0.0;

    } else if (
        closest_front_obstacle_ <
        slow_distance) {

      double scale =
          (closest_front_obstacle_ -
           stop_distance) /
          (slow_distance -
           stop_distance);

      scale =
          std::clamp(
              scale,
              0.0,
              1.0);

      speed =
          minimum_avoidance_speed +
          scale *
              (max_speed -
               minimum_avoidance_speed);
    }

    /*
     * Do not completely stop just because a large
     * steering correction is required.
     *
     * A small amount of translation helps the robot escape
     * APF local minima at corridor bends.
     */
    double abs_heading =
        std::abs(
            desired_heading);

    if (abs_heading >
        50.0 *
        M_PI /
        180.0) {

      speed =
          std::min(
              speed,
              0.03);

    } else if (
        abs_heading >
        35.0 *
        M_PI /
        180.0) {

      speed =
          std::min(
              speed,
              0.06);
    }

    /*
     * If something is dangerously close directly ahead,
     * still allow a full stop.
     */
    if (closest_center_obstacle_ <=
        stop_distance) {

      speed = 0.0;
    }

    /*
     * During goal recovery, rotate toward the goal before
     * translating again.
     */
    if (goal_recovery) {

      speed = 0.0;
    }

    cmd.linear.x =
        speed;

    cmd.angular.z =
        steering_command_;

    pub_twist_->publish(cmd);

    RCLCPP_INFO_THROTTLE(
        this->get_logger(),
        *this->get_clock(),
        500,
        "dist=%.2f "
        "goalAng=%.2f "
        "att=(%.2f %.2f) "
        "rep=(%.2f %.2f) "
        "field=(%.2f %.2f) "
        "front=%.2f "
        "center=%.2f "
        "side=%d "
        "recover=%d "
        "heading=%.2f "
        "v=%.2f "
        "w=%.2f",
        distance,
        relative_goal_angle,
        attractive_x,
        attractive_y,
        repulsive_x_,
        repulsive_y_,
        field_x,
        field_y,
        closest_front_obstacle_,
        closest_center_obstacle_,
        avoidance_side_,
        goal_recovery,
        desired_heading,
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

    /*
     * Quaternion to yaw.
     */
    current_angle_ =
        std::atan2(
            2.0 *
                (qw * qz +
                 qx * qy),
            1.0 -
                2.0 *
                (qy * qy +
                 qz * qz));

    /*
     * Place goal directly ahead of the initial pose.
     */
    if (!goal_initialized_) {

      goal_.x =
          pos_.x +
          goal_distance_ *
              std::cos(
                  current_angle_);

      goal_.y =
          pos_.y +
          goal_distance_ *
              std::sin(
                  current_angle_);

      goal_initialized_ =
          true;

      RCLCPP_INFO(
          this->get_logger(),
          "Start=(%.2f %.2f) "
          "heading=%.2f "
          "goal=(%.2f %.2f) "
          "distance=%.2f",
          pos_.x,
          pos_.y,
          current_angle_,
          goal_.x,
          goal_.y,
          goal_distance_);
    }
  }

  void laser_scan_callback(
      const LaserScan &msg) {

    /*
     * APF parameters.
     */
    double robot_radius =
        0.18;

    double safety_margin =
        0.10;

    double influence_distance =
        0.75;

    double repulsive_gain =
        0.08;

    double minimum_clearance =
        0.05;

    double raw_repulsive_x =
        0.0;

    double raw_repulsive_y =
        0.0;

    closest_obstacle_ =
        std::numeric_limits<double>::
            infinity();

    closest_front_obstacle_ =
        std::numeric_limits<double>::
            infinity();

    closest_center_obstacle_ =
        std::numeric_limits<double>::
            infinity();

    obstacle_present_ =
        false;

    /*
     * Wide sector used for speed reduction.
     */
    double front_half_angle =
        80.0 *
        M_PI /
        180.0;

    /*
     * Narrow sector used to determine whether the
     * path itself is blocked.
     */
    double center_half_angle =
        30.0 *
        M_PI /
        180.0;

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

      /*
       * LaserScan angle zero is treated as forward.
       */
      double angle =
          msg.angle_min +
          msg.angle_increment *
              static_cast<double>(i);

      angle =
          std::atan2(
              std::sin(angle),
              std::cos(angle));

      /*
       * Clearance from the robot safety boundary.
       */
      double clearance =
          range -
          robot_radius -
          safety_margin;

      closest_obstacle_ =
          std::min(
              closest_obstacle_,
              clearance);

      /*
       * Wider front sector for speed control.
       */
      if (std::abs(angle) <
          front_half_angle) {

        closest_front_obstacle_ =
            std::min(
                closest_front_obstacle_,
                clearance);
      }

      /*
       * Narrow center sector for blocked-path logic.
       */
      if (std::abs(angle) <
          center_half_angle) {

        closest_center_obstacle_ =
            std::min(
                closest_center_obstacle_,
                clearance);
      }

      /*
       * Outside influence range:
       * no repulsive contribution.
       */
      if (clearance >
          influence_distance) {
        continue;
      }

      obstacle_present_ =
          true;

      clearance =
          std::max(
              clearance,
              minimum_clearance);

      /*
       * Standard APF repulsive force:
       *
       * eta * (1/rho - 1/rho0) / rho^2
       */
      double magnitude =
          repulsive_gain *
          ((1.0 / clearance) -
           (1.0 /
            influence_distance)) /
          (clearance *
           clearance);

      /*
       * Approximate angular integration.
       */
      magnitude *=
          std::abs(
              msg.angle_increment);

      /*
       * Force points opposite lidar ray.
       */
      raw_repulsive_x -=
          magnitude *
          std::cos(angle);

      raw_repulsive_y -=
          magnitude *
          std::sin(angle);
    }

    /*
     * Saturate repulsive field.
     */
    double max_repulsive =
        2.0;

    double repulsive_length =
        std::hypot(
            raw_repulsive_x,
            raw_repulsive_y);

    if (repulsive_length >
        max_repulsive) {

      raw_repulsive_x *=
          max_repulsive /
          repulsive_length;

      raw_repulsive_y *=
          max_repulsive /
          repulsive_length;
    }

    /*
     * Low-pass filter repulsive vector.
     */
    double repulsive_alpha =
        0.25;

    repulsive_x_ =
        repulsive_alpha *
            raw_repulsive_x +
        (1.0 -
         repulsive_alpha) *
            repulsive_x_;

    repulsive_y_ =
        repulsive_alpha *
            raw_repulsive_y +
        (1.0 -
         repulsive_alpha) *
            repulsive_y_;

    /*
     * Remove small residual filtered forces.
     */
    if (!obstacle_present_ &&
        std::hypot(
            repulsive_x_,
            repulsive_y_) <
            0.01) {

      repulsive_x_ = 0.0;
      repulsive_y_ = 0.0;
    }

    scan_received_ =
        true;
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

  rclcpp::executors::
      SingleThreadedExecutor exec;

  exec.add_node(
      control_node);

  exec.spin();

  rclcpp::shutdown();

  return 0;
}
