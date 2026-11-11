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
  double goal_distance_ = 1.5;

  bool goal_initialized_ = false;
  bool scan_received_ = false;

  /*
   * Filtered repulsive field in robot coordinates.
   */
  double repulsive_x_ = 0.0;
  double repulsive_y_ = 0.0;

  /*
   * Obstacle clearances measured from the robot's
   * safety boundary.
   */
  double closest_front_ =
      std::numeric_limits<double>::infinity();

  double closest_center_ =
      std::numeric_limits<double>::infinity();

  /*
   * Average free space to either side.
   */
  double left_space_ = 0.0;
  double right_space_ = 0.0;

  /*
   * -1 = avoid right
   *  0 = normal APF
   * +1 = avoid left
   */
  int avoidance_side_ = 0;

  /*
   * Smoothed velocity commands.
   */
  double steering_command_ = 0.0;
  double speed_command_ = 0.0;

public:
  Lab3Node()
      : rclcpp::Node("lab3_node") {

    /*
     * Example:
     *
     * ros2 run lab3 lab3_node \
     *   --ros-args -p goal_distance:=4.0
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
  }

  void command_loop_function() {

    if (!goal_initialized_ ||
        !scan_received_) {
      return;
    }

    Twist cmd;

    /*
     * --------------------------------------------------
     * Goal relative to robot
     * --------------------------------------------------
     */

    double goal_dx =
        goal_.x - pos_.x;

    double goal_dy =
        goal_.y - pos_.y;

    double distance =
        std::hypot(
            goal_dx,
            goal_dy);

    /*
     * Stop at goal.
     */
    if (distance < 0.05) {

      speed_command_ = 0.0;
      steering_command_ = 0.0;

      cmd.linear.x = 0.0;
      cmd.angular.z = 0.0;

      pub_twist_->publish(cmd);

      return;
    }

    double goal_angle =
        std::atan2(
            goal_dy,
            goal_dx);

    double relative_goal_angle =
        normalize_angle(
            goal_angle -
            current_angle_);

    /*
     * --------------------------------------------------
     * Attractive field
     * --------------------------------------------------
     */

    const double attractive_gain = 1.0;

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
     * --------------------------------------------------
     * Blocked-path detection
     * --------------------------------------------------
     *
     * Use both the center sector and wider forward
     * sector. This prevents an angled wall from being
     * ignored because it lies just outside the center.
     */

    const double center_block_distance = 0.55;
    const double front_block_distance = 0.40;
    const double release_distance = 0.75;

    bool path_blocked =
        closest_center_ <
            center_block_distance ||
        closest_front_ <
            front_block_distance;

    /*
     * Choose an avoidance side once.
     */
    if (avoidance_side_ == 0 &&
        path_blocked) {

      if (left_space_ >
          right_space_ + 0.05) {

        avoidance_side_ = 1;

      } else if (
          right_space_ >
          left_space_ + 0.05) {

        avoidance_side_ = -1;

      } else {

        /*
         * If both sides are about equally open,
         * use the repulsive field to break the tie.
         */
        if (repulsive_y_ >= 0.0) {
          avoidance_side_ = 1;
        } else {
          avoidance_side_ = -1;
        }
      }
    }

    /*
     * Keep the chosen side until both the center and
     * wider forward area are clearly open.
     */
    if (avoidance_side_ != 0 &&
        closest_center_ >
            release_distance &&
        closest_front_ >
            release_distance) {

      avoidance_side_ = 0;
    }

    bool avoiding =
        avoidance_side_ != 0;

    /*
     * --------------------------------------------------
     * Select desired heading
     * --------------------------------------------------
     */

    double desired_heading = 0.0;

    bool goal_behind =
        std::abs(
            relative_goal_angle) >
        85.0 *
        M_PI /
        180.0;

    if (avoiding) {

      /*
       * A blocked path gets an explicit strong turn.
       *
       * This avoids the local minimum where a centered
       * obstacle produces almost no lateral APF force.
       */
      const double avoid_angle =
          65.0 *
          M_PI /
          180.0;

      desired_heading =
          avoidance_side_ *
          avoid_angle;

    } else if (goal_behind) {

      /*
       * If the robot has rotated too far, point back
       * toward the goal before translating.
       */
      const double recovery_angle =
          60.0 *
          M_PI /
          180.0;

      desired_heading =
          std::clamp(
              relative_goal_angle,
              -recovery_angle,
              recovery_angle);

    } else {

      /*
       * Normal artificial potential field.
       */
      double field_x =
          attractive_x +
          repulsive_x_;

      double field_y =
          attractive_y +
          repulsive_y_;

      desired_heading =
          std::atan2(
              field_y,
              field_x);

      /*
       * Normal APF should not command a complete
       * reversal.
       */
      const double max_heading =
          60.0 *
          M_PI /
          180.0;

      desired_heading =
          std::clamp(
              desired_heading,
              -max_heading,
              max_heading);
    }

    /*
     * --------------------------------------------------
     * Angular velocity
     * --------------------------------------------------
     */

    const double turn_gain = 1.5;

    double steering_target =
        turn_gain *
        desired_heading;

    steering_target =
        std::clamp(
            steering_target,
            -0.9,
            0.9);

    /*
     * Smooth scan-to-scan steering changes.
     */
    const double steering_alpha = 0.45;

    steering_command_ =
        steering_alpha *
            steering_target +
        (1.0 -
         steering_alpha) *
            steering_command_;

    if (std::abs(
            steering_command_) <
        0.015) {

      steering_command_ = 0.0;
    }

    /*
     * --------------------------------------------------
     * Linear velocity
     * --------------------------------------------------
     */

    const double max_speed = 0.15;

    const double emergency_stop_distance = 0.25;
    const double slow_distance = 0.75;

    double speed_target =
        max_speed;

    /*
     * Goal behind:
     * rotate back toward it before moving.
     */
    if (goal_behind &&
        !avoiding) {

      speed_target = 0.0;

    /*
     * Explicit obstacle avoidance.
     */
    } else if (avoiding) {

      /*
       * Very close obstacle:
       * turn in place.
       */
      if (closest_front_ <
          emergency_stop_distance) {

        speed_target = 0.0;

      } else {

        /*
         * Otherwise creep forward while turning.
         * Translation helps change the geometry and
         * prevents getting stuck rotating forever.
         */
        speed_target = 0.025;
      }

    } else {

      /*
       * Normal speed reduction from forward clearance.
       */
      if (closest_front_ <
          emergency_stop_distance) {

        speed_target = 0.0;

      } else if (
          closest_front_ <
          slow_distance) {

        double scale =
            (closest_front_ -
             emergency_stop_distance) /
            (slow_distance -
             emergency_stop_distance);

        scale =
            std::clamp(
                scale,
                0.0,
                1.0);

        speed_target =
            0.04 +
            scale *
                (max_speed -
                 0.04);
      }

      /*
       * Slow down during substantial APF turns.
       */
      double abs_heading =
          std::abs(
              desired_heading);

      if (abs_heading >
          45.0 *
          M_PI /
          180.0) {

        speed_target =
            std::min(
                speed_target,
                0.035);

      } else if (
          abs_heading >
          25.0 *
          M_PI /
          180.0) {

        speed_target =
            std::min(
                speed_target,
                0.08);
      }
    }

    /*
     * Smooth normal acceleration and deceleration.
     */
    const double speed_alpha = 0.40;

    speed_command_ =
        speed_alpha *
            speed_target +
        (1.0 -
         speed_alpha) *
            speed_command_;

    /*
     * Safety stop is immediate, not filtered.
     */
    if (closest_front_ <
        emergency_stop_distance) {

      speed_command_ = 0.0;
    }

    cmd.linear.x =
        speed_command_;

    cmd.angular.z =
        steering_command_;

    pub_twist_->publish(cmd);

    /*
     * --------------------------------------------------
     * Debug output
     * --------------------------------------------------
     */

    RCLCPP_INFO_THROTTLE(
        this->get_logger(),
        *this->get_clock(),
        500,
        "dist=%.2f "
        "goalAng=%.2f "
        "rep=(%.2f %.2f) "
        "front=%.2f "
        "center=%.2f "
        "left=%.2f "
        "right=%.2f "
        "blocked=%d "
        "side=%d "
        "heading=%.2f "
        "v=%.2f "
        "w=%.2f",
        distance,
        relative_goal_angle,
        repulsive_x_,
        repulsive_y_,
        closest_front_,
        closest_center_,
        left_space_,
        right_space_,
        path_blocked,
        avoidance_side_,
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
     * Goal is placed goal_distance_ meters directly
     * ahead of the initial odometry pose.
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
     * --------------------------------------------------
     * Geometry
     * --------------------------------------------------
     */

    const double robot_radius = 0.18;
    const double safety_margin = 0.10;

    /*
     * Obstacle begins influencing APF when its clearance
     * from the robot safety boundary is below 1 m.
     */
    const double influence_distance = 1.0;

    const double minimum_clearance = 0.05;

    /*
     * Stronger than the previous version because the
     * repulsive sector is now correctly limited to
     * obstacles in front of the robot.
     */
    const double repulsive_gain = 0.10;

    /*
     * Only rays within +/-85 degrees contribute to
     * repulsion.
     *
     * cos(angle) is positive throughout this range, so
     * every obstacle produces a non-positive X
     * repulsive component.
     */
    const double repulsive_half_angle =
        85.0 *
        M_PI /
        180.0;

    /*
     * Wider sector used for collision/speed control.
     */
    const double front_half_angle =
        75.0 *
        M_PI /
        180.0;

    /*
     * Center sector used for blocked-path detection.
     */
    const double center_half_angle =
        40.0 *
        M_PI /
        180.0;

    /*
     * Side sectors used to determine the more open
     * avoidance direction.
     */
    const double side_inner_angle =
        20.0 *
        M_PI /
        180.0;

    const double side_outer_angle =
        90.0 *
        M_PI /
        180.0;

    double raw_repulsive_x = 0.0;
    double raw_repulsive_y = 0.0;

    closest_front_ =
        std::numeric_limits<double>::
            infinity();

    closest_center_ =
        std::numeric_limits<double>::
            infinity();

    double left_sum = 0.0;
    double right_sum = 0.0;

    int left_count = 0;
    int right_count = 0;

    /*
     * --------------------------------------------------
     * Process lidar
     * --------------------------------------------------
     */

    for (size_t i = 0;
         i < msg.ranges.size();
         ++i) {

      double angle =
          msg.angle_min +
          msg.angle_increment *
              static_cast<double>(i);

      angle =
          normalize_angle(
              angle);

      double range =
          msg.ranges[i];

      /*
       * An infinite return means nothing was detected
       * within the sensor range. For open-space scoring,
       * treat that direction as open.
       */
      bool no_return =
          std::isinf(range) &&
          range > 0.0;

      if (no_return) {

        double open_space = 1.5;

        if (angle >=
                side_inner_angle &&
            angle <=
                side_outer_angle) {

          left_sum +=
              open_space;

          ++left_count;
        }

        if (angle <=
                -side_inner_angle &&
            angle >=
                -side_outer_angle) {

          right_sum +=
              open_space;

          ++right_count;
        }

        continue;
      }

      if (!std::isfinite(range)) {
        continue;
      }

      if (range < msg.range_min ||
          range > msg.range_max) {
        continue;
      }

      /*
       * No lidar angle offset is used.
       * Testing showed angle zero is robot forward.
       */
      double clearance =
          range -
          robot_radius -
          safety_margin;

      /*
       * ------------------------------------------------
       * Clearance sectors
       * ------------------------------------------------
       */

      if (std::abs(angle) <
          front_half_angle) {

        closest_front_ =
            std::min(
                closest_front_,
                clearance);
      }

      if (std::abs(angle) <
          center_half_angle) {

        closest_center_ =
            std::min(
                closest_center_,
                clearance);
      }

      /*
       * ------------------------------------------------
       * Side openness
       * ------------------------------------------------
       */

      double space =
          std::clamp(
              clearance,
              0.0,
              1.5);

      if (angle >=
              side_inner_angle &&
          angle <=
              side_outer_angle) {

        left_sum +=
            space;

        ++left_count;
      }

      if (angle <=
              -side_inner_angle &&
          angle >=
              -side_outer_angle) {

        right_sum +=
            space;

        ++right_count;
      }

      /*
       * ------------------------------------------------
       * APF repulsion
       * ------------------------------------------------
       */

      if (std::abs(angle) >
          repulsive_half_angle) {
        continue;
      }

      if (clearance >
          influence_distance) {
        continue;
      }

      double rho =
          std::max(
              clearance,
              minimum_clearance);

      /*
       * Standard APF:
       *
       * eta * (1/rho - 1/rho0) / rho^2
       */
      double magnitude =
          repulsive_gain *
          ((1.0 / rho) -
           (1.0 /
            influence_distance)) /
          (rho * rho);

      /*
       * Account for angular lidar resolution.
       */
      magnitude *=
          std::abs(
              msg.angle_increment);

      /*
       * Lidar ray points toward obstacle.
       * Repulsion points away.
       */
      raw_repulsive_x -=
          magnitude *
          std::cos(
              angle);

      raw_repulsive_y -=
          magnitude *
          std::sin(
              angle);
    }

    /*
     * --------------------------------------------------
     * Side openness averages
     * --------------------------------------------------
     */

    if (left_count > 0) {

      left_space_ =
          left_sum /
          static_cast<double>(
              left_count);

    } else {

      left_space_ = 0.0;
    }

    if (right_count > 0) {

      right_space_ =
          right_sum /
          static_cast<double>(
              right_count);

    } else {

      right_space_ = 0.0;
    }

    /*
     * --------------------------------------------------
     * Limit repulsive field
     * --------------------------------------------------
     */

    const double max_repulsive = 2.0;

    double repulsive_length =
        std::hypot(
            raw_repulsive_x,
            raw_repulsive_y);

    if (repulsive_length >
        max_repulsive) {

      double scale =
          max_repulsive /
          repulsive_length;

      raw_repulsive_x *=
          scale;

      raw_repulsive_y *=
          scale;
    }

    /*
     * Low-pass filter the resulting field.
     */
    const double repulsive_alpha = 0.30;

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
     * Remove tiny residual values.
     */
    if (std::hypot(
            repulsive_x_,
            repulsive_y_) <
        0.005) {

      repulsive_x_ = 0.0;
      repulsive_y_ = 0.0;
    }

    scan_received_ = true;
  }

  static double normalize_angle(
      double angle) {

    return std::atan2(
        std::sin(angle),
        std::cos(angle));
  }
};

int main(
    int argc,
    char **argv) {

  rclcpp::init(
      argc,
      argv);

  auto node =
      std::make_shared<Lab3Node>();

  rclcpp::executors::
      SingleThreadedExecutor executor;

  executor.add_node(
      node);

  executor.spin();

  rclcpp::shutdown();

  return 0;
}
