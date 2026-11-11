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
   * Filtered APF repulsive vector in robot frame.
   */
  double repulsive_x_ = 0.0;
  double repulsive_y_ = 0.0;

  /*
   * Obstacle clearances measured from the robot safety
   * boundary rather than directly from the lidar.
   */
  double closest_front_ =
      std::numeric_limits<double>::infinity();

  double closest_center_ =
      std::numeric_limits<double>::infinity();

  /*
   * Average free space on each side of the robot.
   * Used only to choose an avoidance direction.
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
   * Filtered velocity commands.
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
     *     --ros-args -p goal_distance:=4.0
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
     * Goal position relative to robot
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
     *
     * Attractive magnitude is capped so repulsion can
     * still influence the robot far from the goal.
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
     * --------------------------------------------------
     * Determine whether center path is blocked
     * --------------------------------------------------
     */

    const double block_distance = 0.35;
    const double release_distance = 0.55;

    /*
     * Enter avoidance only when something is actually
     * blocking the narrow center sector.
     */
    if (avoidance_side_ == 0 &&
        closest_center_ <
            block_distance) {

      /*
       * Choose whichever side has more free space.
       */
      if (left_space_ >
          right_space_ + 0.05) {

        avoidance_side_ = 1;

      } else if (
          right_space_ >
          left_space_ + 0.05) {

        avoidance_side_ = -1;

      } else {

        /*
         * If both sides are about equal, use the
         * repulsive field to break the tie.
         */
        if (repulsive_y_ < 0.0) {
          avoidance_side_ = -1;
        } else {
          avoidance_side_ = 1;
        }
      }
    }

    /*
     * Keep the chosen direction until the center path
     * is clearly open again.
     */
    if (avoidance_side_ != 0 &&
        closest_center_ >
            release_distance) {

      avoidance_side_ = 0;
    }

    /*
     * --------------------------------------------------
     * Select desired heading
     * --------------------------------------------------
     */

    double desired_heading = 0.0;

    bool blocked =
        avoidance_side_ != 0;

    bool goal_behind =
        std::abs(
            relative_goal_angle) >
        85.0 * M_PI / 180.0;

    if (blocked) {

      /*
       * Explicit blocked-path behavior.
       *
       * Do not let a nearly symmetric APF decide to
       * oscillate around zero degrees. Turn decisively
       * toward the selected open side.
       */
      double avoid_angle =
          55.0 *
          M_PI /
          180.0;

      desired_heading =
          avoidance_side_ *
          avoid_angle;

    } else if (goal_behind) {

      /*
       * If an earlier avoidance maneuver caused the
       * robot to rotate too far, deliberately point
       * back toward the goal before moving again.
       */
      double recovery_angle =
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
       * Normal APF motion is restricted to the forward
       * hemisphere. This prevents a close obstacle from
       * telling the robot to completely turn around.
       */
      double max_heading =
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

    double turn_gain = 1.5;

    double steering_target =
        turn_gain *
        desired_heading;

    steering_target =
        std::clamp(
            steering_target,
            -0.9,
            0.9);

    /*
     * Smooth steering without making it overly sluggish.
     */
    double steering_alpha = 0.45;

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

    const double stop_distance = 0.12;
    const double slow_distance = 0.60;

    double speed_target =
        max_speed;

    /*
     * Goal behind:
     * rotate first.
     */
    if (goal_behind &&
        !blocked) {

      speed_target = 0.0;

    /*
     * Center blocked:
     * turn in place when very close, otherwise creep
     * forward while turning so the geometry changes.
     */
    } else if (blocked) {

      if (closest_center_ <
          stop_distance) {

        speed_target = 0.0;

      } else {

        speed_target = 0.025;
      }

    } else {

      /*
       * Normal speed reduction based on the wider
       * forward obstacle sector.
       */
      if (closest_front_ <=
          stop_distance) {

        speed_target = 0.0;

      } else if (
          closest_front_ <
          slow_distance) {

        double scale =
            (closest_front_ -
             stop_distance) /
            (slow_distance -
             stop_distance);

        scale =
            std::clamp(
                scale,
                0.0,
                1.0);

        /*
         * Maintain some useful translation rather than
         * collapsing almost immediately toward zero.
         */
        speed_target =
            0.04 +
            scale *
                (max_speed -
                 0.04);
      }

      /*
       * Large turns reduce speed but do not stop it.
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
     * Smooth acceleration/deceleration.
     */
    double speed_alpha = 0.40;

    speed_command_ =
        speed_alpha *
            speed_target +
        (1.0 -
         speed_alpha) *
            speed_command_;

    /*
     * Do not smooth through an emergency stop.
     */
    if (closest_center_ <
        stop_distance) {

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
     * Goal is created once, directly ahead of the
     * initial odometry pose.
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
     * Geometry and APF parameters
     * --------------------------------------------------
     */

    const double robot_radius = 0.18;
    const double safety_margin = 0.10;

    const double influence_distance = 0.80;
    const double minimum_clearance = 0.05;

    const double repulsive_gain = 0.05;

    /*
     * APF repulsion only considers obstacles roughly
     * ahead of the robot.
     *
     * A wall behind the robot must never create a
     * forward-pointing "repulsive" force.
     */
    const double repulsive_half_angle =
        100.0 *
        M_PI /
        180.0;

    /*
     * Wider sector used for speed control.
     */
    const double front_half_angle =
        70.0 *
        M_PI /
        180.0;

    /*
     * Narrow sector that determines whether the route
     * directly ahead is blocked.
     */
    const double center_half_angle =
        25.0 *
        M_PI /
        180.0;

    /*
     * Side sectors used to determine which direction
     * has more open space.
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
     * Process scan
     * --------------------------------------------------
     */

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
       * Testing showed angle zero is already robot
       * forward, so no lidar offset is applied.
       */
      double angle =
          msg.angle_min +
          msg.angle_increment *
              static_cast<double>(i);

      angle =
          normalize_angle(
              angle);

      /*
       * Clearance from our chosen safety boundary.
       */
      double clearance =
          range -
          robot_radius -
          safety_margin;

      /*
       * ------------------------------------------------
       * Clearance measurements
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
       * Open-space scores.
       *
       * Clamp large values because we only care which
       * side is locally more open.
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
       * Repulsive potential field
       * ------------------------------------------------
       *
       * Ignore obstacles behind the robot.
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
       * Standard APF repulsive field:
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
       * Account for lidar angular resolution.
       */
      magnitude *=
          std::abs(
              msg.angle_increment);

      /*
       * Ray points toward obstacle.
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
     * Calculate side openness
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
     * Limit and filter repulsive field
     * --------------------------------------------------
     */

    const double max_repulsive = 1.5;

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
     * Filtering the resulting vector is much less
     * expensive and simpler than filtering every
     * individual lidar beam.
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
     * Remove insignificant filter residue.
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
