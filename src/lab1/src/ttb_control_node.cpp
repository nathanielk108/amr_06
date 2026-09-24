#include <cmath>
#include <cstdlib>
#include <functional>
#include <memory>
#include <random>

#include <rclcpp/create_timer.hpp>
#include <rclcpp/executors.hpp>
#include <rclcpp/executors/single_threaded_executor.hpp>
#include <rclcpp/publisher_base.hpp>
#include <rclcpp/qos.hpp>
#include <rclcpp/subscription.hpp>
#include <rclcpp/utilities.hpp>

#include <geometry_msgs/msg/twist.hpp>
#include <irobot_create_msgs/msg/ir_intensity_vector.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rmw/qos_profiles.h>
#include <sensor_msgs/msg/joy.hpp>

using namespace std::chrono_literals;
using namespace std::placeholders;

using namespace sensor_msgs::msg;
using namespace geometry_msgs::msg;
using namespace nav_msgs::msg;
using namespace irobot_create_msgs::msg;

class TTBControlNode : public rclcpp::Node {

public:
  enum JoyAxes { L_X = 0, L_Y, L_TRIG, R_X, R_Y, R_TRIG, D_X, D_Y };

  enum JoyButtons {
    CROSS = 0,
    CIRCLE,
    TRIANGLE,
    SQUARE,
    L1,
    R1,
    L2,
    R2,
    SHARE,
    OPTIONS,
    HOME,
    L3,
    R3,
  };

  enum LabMode { JOY_TELEOP, AUTO_WANDERING, CRUISE_CONTROL };

  enum WanderMode { FORWARD, TURN };

private:
  rclcpp::TimerBase::SharedPtr timer_;

  rclcpp::Subscription<Joy>::SharedPtr sub_joy_;

  // TODO: Process odometry
  rclcpp::Subscription<Odometry>::SharedPtr sub_odometry_;

  rclcpp::Subscription<irobot_create_msgs::msg::IrIntensityVector>::SharedPtr
      sub_ir_;

  Twist twist_cmd;
  rclcpp::Publisher<Twist>::SharedPtr pub_twist_;

  // PID variables
  const double kp = 0.1;
  const double ki = 0.001;
  const double kd = 0.03;

  double target_speed = 0.3;

  double current_speed = 0.0;

  double previous_error = 0.0;
  double integral = 0.0;
  double output = 0.0;

  // Random angle target generation
  std::mt19937 mt;
  std::uniform_real_distribution<float> angle_dist;

  double target_angle = 0.0;
  double current_angle = 0.0;
  double current_ir = 1.0;
  double angle_speed = 0.5;

  LabMode current_mode;
  LabMode previous_mode;

  WanderMode wander_mode = FORWARD;

  bool r1_was_pressed = false;

public:
  explicit TTBControlNode() : rclcpp::Node("ttb_control_node") {

    current_mode = JOY_TELEOP;
    previous_mode = JOY_TELEOP;

    // Instantiate PRNG
    mt = std::mt19937();
    angle_dist = std::uniform_real_distribution<float>(-3.14, 3.14);

    timer_ = this->create_wall_timer(
        100ms, std::bind(&TTBControlNode::command_loop_function, this));

    sub_joy_ = this->create_subscription<Joy>(
        "/TTB06/joy",
        rclcpp::QoS(
            rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_sensor_data),
            rmw_qos_profile_sensor_data),
        std::bind(&TTBControlNode::joy_callback, this, _1));

    sub_ir_ = this->create_subscription<IrIntensityVector>(
        "/TTB06/ir_intensity",
        rclcpp::QoS(
            rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_sensor_data),
            rmw_qos_profile_sensor_data),
        std::bind(&TTBControlNode::ir_callback, this, _1));

    sub_odometry_ = this->create_subscription<Odometry>(
        "/TTB06/odom",
        rclcpp::QoS(
            rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_sensor_data),
            rmw_qos_profile_sensor_data),
        std::bind(&TTBControlNode::odometry_callback, this, _1));

    pub_twist_ = this->create_publisher<Twist>("/TTB06/cmd_vel", 10);
  }

private:
  void command_loop_function() {

    switch (current_mode) {
    case JOY_TELEOP:
      joy_teleop_function();
      break;

    case AUTO_WANDERING:
      auto_wandering_function();
      break;

    case CRUISE_CONTROL:
      cruise_control_function();
      break;

    default:
      break;
    }
  }

  void auto_wandering_function() {
    RCLCPP_INFO(this->get_logger(), "current ir: %f", current_ir);
    switch (wander_mode) {
    case FORWARD: {
      if (current_ir > 20.0) {
        RCLCPP_INFO(this->get_logger(), "FORWARD if");
        target_angle = angle_dist(mt);
        twist_cmd.linear.x = 0;
        twist_cmd.angular.z = 0;
        wander_mode = TURN;
      } else {
        RCLCPP_INFO(this->get_logger(), "FORWARD else");
        twist_cmd.linear.x = target_speed;
        twist_cmd.angular.z = 0;
      }
      break;
    }
    case TURN: {
      twist_cmd.linear.x = 0;
      double angle_error = target_angle - current_angle;
      angle_error = std::atan2(std::sin(angle_error), std::cos(angle_error)); // using atan2 from Lecture 3
      double abs_error = std::fabs(angle_error);

      if (abs_error > 0.1) {
        RCLCPP_INFO(this->get_logger(), "TURN if");
        if (angle_error > 0) {
          twist_cmd.angular.z = angle_speed;
          angle_error = target_angle - current_angle;
        } else if (angle_error < 0) {
          twist_cmd.angular.z = -(angle_speed);
          angle_error = target_angle - current_angle;
        }
      } else {
        RCLCPP_INFO(this->get_logger(), "TURN else");
        twist_cmd.angular.z = 0;
        twist_cmd.linear.x = target_speed;
        wander_mode = FORWARD;
      }
      break;
    }
    }
    pub_twist_->publish(twist_cmd);
  }

  void cruise_control_function() {

    double dt = 0.1;

    // error = setpoint - measured value
    double error = target_speed - current_speed;

    // integral = integral + error * dt
    integral = integral + error * dt;

    // derivative = (error - previous_error) / dt
    double derivative = (error - previous_error) / dt;

    // output = output + PID terms
    output = output + kp * error + ki * integral + kd * derivative;

    // Send PID output to robot
    twist_cmd.linear.x = output;
    twist_cmd.angular.z = 0.0;

    // Save error for next iteration
    previous_error = error;

    pub_twist_->publish(twist_cmd);
  }

  void joy_teleop_function() { pub_twist_->publish(twist_cmd); }

  void joy_callback(const Joy &msg) {

    bool r1_pressed = msg.buttons[JoyButtons::R1];

    if (r1_pressed) {
      RCLCPP_INFO(this->get_logger(), "MODE: Joystick Teleoperation");
      previous_mode = current_mode;
      current_mode = JOY_TELEOP;
      twist_cmd.linear.x = msg.axes[JoyAxes::L_Y];
      twist_cmd.angular.z = msg.axes[JoyAxes::L_X];
    } else if (msg.buttons[JoyButtons::L1] && !r1_pressed) {
      RCLCPP_INFO(this->get_logger(), "MODE: Auto Wandering");
      current_mode = AUTO_WANDERING;
    } else if (msg.buttons[JoyButtons::L2] && !r1_pressed) {
      RCLCPP_INFO(this->get_logger(), "MODE: Cruise Control");
      current_mode = CRUISE_CONTROL;
    } else if (!r1_pressed && r1_was_pressed) {
      RCLCPP_INFO(this->get_logger(), "Returning to previous mode");
      current_mode = previous_mode;
    }

    // Cruise control speed selection
    if (current_mode == CRUISE_CONTROL && !r1_pressed) {

      if (msg.buttons[JoyButtons::SQUARE]) {
        target_speed = 0.1;
      }

      if (msg.buttons[JoyButtons::TRIANGLE]) {
        target_speed = 0.2;
      }

      if (msg.buttons[JoyButtons::CIRCLE]) {
        target_speed = 0.4;
      }

      if (msg.buttons[JoyButtons::CROSS]) {
        target_speed = 0.0;
      }
    } else if (current_mode == AUTO_WANDERING) {
      target_speed = 0.3;
    }

    r1_was_pressed = r1_pressed;
  }

  void odometry_callback(const Odometry &msg) {

    current_speed = msg.twist.twist.linear.x;
    double qx = msg.pose.pose.orientation.x;
    double qy = msg.pose.pose.orientation.y;
    double qz = msg.pose.pose.orientation.z;
    double qw = msg.pose.pose.orientation.w;
    current_angle =
        std::atan2(2.0 * (qw * qz + qx * qy),
                   1.0 - 2.0 * (qy * qy + qz * qz)); // quarternion yaw angle
  }

  void ir_callback(const IrIntensityVector &msg) {
    double intensity = 0.0;
    for (const auto &reading : msg.readings) {
      if (reading.value > intensity) {
        intensity = reading.value;
      }
    }
    current_ir = intensity;
  }
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);

  TTBControlNode::SharedPtr control_node =
      std::make_shared<TTBControlNode>(); // Must keep reference to shared
                                          // pointer or fails to start
  rclcpp::executors::SingleThreadedExecutor exec{};
  exec.add_node(control_node);
  exec.spin();

  rclcpp::shutdown();
  return 0;
}
