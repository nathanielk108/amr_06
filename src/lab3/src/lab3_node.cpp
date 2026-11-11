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
   * Closest obstacle anywhere and in front of robot.
   */
  double closest_obstacle_ =
      std::numeric_limits<double>::infinity();

  double closest_front_obstacle_ =
      std::numeric_limits<double>::infinity();

  bool obstacle_present_ = false;

  /*
   * Persistent avoidance direction.
   *
   * -1 = pass obstacle on right
   *  0 = no preference
   * +1 = pass obstacle on left
   */
  int avoidance_side_ = 0;

  /*
   * Smoothed angular velocity command.
   */
  double steering_command_ = 0.0;

public:
  explicit Lab3Node() : rclcpp::Node("lab3_node") {

    /*
     * Goal distance can be set from command line:
     *
     * --ros-args -p goal_distance:=2.0
     */
    this->declare_parameter<double>("goal_distance", 1.5);
    goal_distance_ =
        this->get_parameter("goal_distance").as_double();

    timer_ = this->create_wall_timer(
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

    if (!goal_initialized_ || !scan_received_) {
      return;
    }

    double goal_dx =
        goal_.x - pos_.x;

    double goal_dy =
        goal_.y - pos_.y;

    double distance =
        std::hypot(goal_dx, goal_dy);

    Twist cmd;

    /*
     * Stop at goal.
     */
    if (distance < 0.05) {

      steering_command_ = 0.0;

      cmd.linear.x = 0.0;
      cmd.angular.z = 0.0;

      pub_twist_->publish(cmd);
      return;
    }

    /*
     * Attractive field.
     *
     * First find the goal direction in the global frame,
     * then express it relative to the robot heading.
     */
    double goal_angle =
        std::atan2(goal_dy, goal_dx);

    double relative_goal_angle =
        std::atan2(
            std::sin(
                goal_angle -
                current_angle_),
            std::cos(
                goal_angle -
                current_angle_));

    double attractive_gain = 1.0;

    double attractive_magnitude =
        attractive_gain *
        std::min(distance, 1.0);

    double attractive_x =
        attractive_magnitude *
        std::cos(relative_goal_angle);

    double attractive_y =
        attractive_magnitude *
        std::sin(relative_goal_angle);

    /*
     * Total artificial potential field.
     */
    double field_x =
        attractive_x +
        repulsive_x_;

    double field_y =
        attractive_y +
        repulsive_y_;

    /*
     * Avoidance hysteresis.
     *
     * Once the robot chooses a side around an obstacle,
     * keep that side until the obstacle is actually clear.
     */
    double avoidance_start_distance = 0.75;
    double avoidance_release_distance = 0.90;

    if (avoidance_side_ == 0 &&
        closest_front_obstacle_ <
            avoidance_start_distance) {

      /*
       * Positive robot-frame Y means turn left.
       * Negative means turn right.
       */
      if (repulsive_y_ > 0.03) {

        avoidance_side_ = 1;

      } else if (repulsive_y_ < -0.03) {

        avoidance_side_ = -1;

      } else {

        /*
         * Nearly centered obstacle.
         * Pick one side to break symmetry.
         */
        avoidance_side_ = 1;
      }
    }

    if (avoidance_side_ != 0) {

      double repulsive_length =
          std::hypot(
              repulsive_x_,
              repulsive_y_);

      /*
       * Do not release avoidance just because the
       * obstacle moves outside the front sector.
       */
      if (closest_front_obstacle_ >
              avoidance_release_distance &&
          repulsive_length < 0.15) {

        avoidance_side_ = 0;

      } else {

        /*
         * Increase lateral avoidance as the obstacle
         * gets closer.
         */
        double proximity =
            1.0 -
            std::clamp(
                closest_front_obstacle_ /
                    avoidance_start_distance,
                0.0,
                1.0);

        double avoidance_bias =
            0.20 +
            0.50 * proximity;

        field_y +=
            avoidance_bias *
            avoidance_side_;
      }
    }

    /*
     * While avoiding an obstacle, do not allow the
     * potential field to point behind the robot.
     *
     * Forward safety is handled independently by
     * the speed controller.
     */
    if (avoidance_side_ != 0) {

      double minimum_forward_field = 0.05;

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
     * An obstacle should never cause the robot to
     * turn completely around and leave the goal.
     *
     * Limit obstacle avoidance to the forward
     * hemisphere.
     */
    if (obstacle_present_) {

      double max_avoid_heading =
          80.0 *
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
     * Smooth the steering command to reduce lidar
     * scan-to-scan jitter.
     */
    double steering_alpha = 0.40;

    steering_command_ =
        steering_alpha *
            steering_target +
        (1.0 - steering_alpha) *
            steering_command_;

    /*
     * Ignore extremely small steering commands.
     */
    if (std::abs(steering_command_) < 0.02) {
      steering_command_ = 0.0;
    }

    /*
     * Forward speed controller.
     *
     * Only obstacles within the forward sector
     * directly reduce forward velocity.
     */
    double max_speed = 0.15;

    double minimum_avoidance_speed = 0.03;

    double stop_distance = 0.15;
    double slow_distance = 0.70;

    double speed = max_speed;

    if (closest_front_obstacle_ <=
        stop_distance) {

      speed = 0.0;

    } else if (
        closest_front_obstacle_ <
        slow_distance) {

      /*
       * Linearly reduce speed as obstacle
       * clearance decreases.
       */
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
     * Slow down when a large steering correction
     * is required.
     */
    double abs_heading =
        std::abs(
            desired_heading);

    if (abs_heading >
        70.0 *
        M_PI /
        180.0) {

      speed = 0.0;

    } else if (
        abs_heading >
        45.0 *
        M_PI /
        180.0) {

      speed =
          std::min(
              speed,
              0.04);
    }

    cmd.linear.x = speed;
    cmd.angular.z =
        steering_command_;

    pub_twist_->publish(cmd);

    RCLCPP_INFO_THROTTLE(
        this->get_logger(),
        *this->get_clock(),
        500,
        "dist=%.2f "
        "att=(%.2f %.2f) "
        "rep=(%.2f %.2f) "
        "field=(%.2f %.2f) "
        "front=%.2f "
        "side=%d "
        "heading=%.2f "
        "v=%.2f w=%.2f",
        distance,
        attractive_x,
        attractive_y,
        repulsive_x_,
        repulsive_y_,
        field_x,
        field_y,
        closest_front_obstacle_,
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
     * Place goal goal_distance_ meters directly
     * ahead of the robot's initial pose.
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

      goal_initialized_ = true;

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
     * Repulsive field parameters.
     */
    double robot_radius = 0.18;
    double safety_margin = 0.10;

    double influence_distance = 0.75;
    double repulsive_gain = 0.08;

    double minimum_clearance = 0.05;

    /*
     * Calculate a new raw field for this scan.
     * The stored field is filtered later.
     */
    double raw_repulsive_x = 0.0;
    double raw_repulsive_y = 0.0;

    closest_obstacle_ =
        std::numeric_limits<double>::infinity();

    closest_front_obstacle_ =
        std::numeric_limits<double>::infinity();

    obstacle_present_ = false;

    /*
     * Obstacles within +/-80 degrees are considered
     * forward obstacles for speed control.
     *
     * Obstacles outside this sector still contribute
     * repulsive force.
     */
    double front_half_angle =
        80.0 *
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
       * LaserScan angle 0 is treated as forward.
       * No lidar offset is applied.
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
       * Convert sensor range into clearance from the
       * robot's safety boundary.
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
       * Track front clearance separately for
       * forward speed control.
       */
      if (std::abs(angle) <
          front_half_angle) {

        closest_front_obstacle_ =
            std::min(
                closest_front_obstacle_,
                clearance);
      }

      /*
       * Obstacles outside the influence distance do
       * not produce a repulsive field.
       */
      if (clearance >
          influence_distance) {
        continue;
      }

      obstacle_present_ = true;

      /*
       * Prevent singularity when something is very
       * close to the robot.
       */
      clearance =
          std::max(
              clearance,
              minimum_clearance);

      /*
       * Standard APF repulsive magnitude:
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
       * Approximate integration over lidar angle so
       * field strength does not depend strongly on
       * lidar angular resolution.
       */
      magnitude *=
          std::abs(
              msg.angle_increment);

      /*
       * Laser ray points toward obstacle.
       * Repulsive force points away from it.
       */
      raw_repulsive_x -=
          magnitude *
          std::cos(angle);

      raw_repulsive_y -=
          magnitude *
          std::sin(angle);
    }

    /*
     * Limit the magnitude of the repulsive field.
     *
     * This allows strong avoidance without letting
     * a close obstacle create an arbitrarily large
     * field.
     */
    double max_repulsive = 2.0;

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
     * Low-pass filter the complete repulsive vector.
     */
    double repulsive_alpha = 0.25;

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
     * Remove tiny residual forces once the obstacle
     * has disappeared.
     */
    if (!obstacle_present_ &&
        std::hypot(
            repulsive_x_,
            repulsive_y_) <
            0.01) {

      repulsive_x_ = 0.0;
      repulsive_y_ = 0.0;
    }

    scan_received_ = true;
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
