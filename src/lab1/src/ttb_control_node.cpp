#include <cstdlib>
#include <memory>
#include <functional>
#include <random>

#include <rclcpp/create_timer.hpp>
#include <rclcpp/executors.hpp>
#include <rclcpp/executors/single_threaded_executor.hpp>
#include <rclcpp/publisher_base.hpp>
#include <rclcpp/qos.hpp>
#include <rclcpp/subscription.hpp>
#include <rclcpp/utilities.hpp>

#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rmw/qos_profiles.h>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/joy.hpp>

using namespace std::chrono_literals;
using namespace std::placeholders;

using namespace sensor_msgs::msg;
using namespace geometry_msgs::msg;
using namespace nav_msgs::msg;

class TTBControlNode : public rclcpp::Node {

public:
  enum JoyAxes {
    L_X = 0,
    L_Y,
    L_TRIG,
    R_X,
    R_Y,
    R_TRIG,
    D_X,
    D_Y
  };

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

  // TODO: Multiple Modes
  enum LabMode {
    JOY_TELEOP,
    AUTO_WANDERING,
    CRUISE_CONTROL
  };

private:
  rclcpp::TimerBase::SharedPtr timer_;

  rclcpp::Subscription<Joy>::SharedPtr sub_joy_;

  // TODO: Process Laser range and odometry
  // rclcpp::Subscription<LaserScan>::SharedPtr sub_laser_scan_;
  // rclcpp::Subscription<Odometry>::SharedPtr sub_odometry_;

  TwistStamped twist_cmd;
  rclcpp::Publisher<TwistStamped>::SharedPtr pub_twist_;

  // TODO: Random angle target generation
  std::mt19937 mt;
  std::uniform_real_distribution<float> angle_dist;

  LabMode current_mode;

public:
  explicit TTBControlNode() : rclcpp::Node("ttb_control_node") {

    current_mode = JOY_TELEOP;

    // Instantiate PRNG
    mt = std::mt19937();
    angle_dist = std::uniform_real_distribution<float>(-3.14, 3.14);

    timer_ = this->create_wall_timer(100ms, std::bind(&TTBControlNode::command_loop_function, this));

    sub_joy_ = this->create_subscription<Joy>(
      "/TTB06/joy", 
      rclcpp::QoS(rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_sensor_data), rmw_qos_profile_sensor_data), 
      std::bind(&TTBControlNode::joy_callback, this, _1)
    );

    // sub_laser_scan_ = this->create_subscription<LaserScan>(
    //   "/TTB06/scan", 
    //   rclcpp::QoS(rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_sensor_data), rmw_qos_profile_sensor_data), 
    //   std::bind(&TTBControlNode::laser_scan_callback, this, _1)
    // );

    // sub_odometry_ = this->create_subscription<Odometry>(
    //   "/TTB06/odom", 
    //   rclcpp::QoS(rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_sensor_data), rmw_qos_profile_sensor_data), 
    //   std::bind(&TTBControlNode::odometry_callback, this, _1)
    // );

    pub_twist_ = this->create_publisher<TwistStamped>(
      "/TTB06/cmd_vel",
      10
    );
  }

private:
  void command_loop_function() {
    // RCLCPP_INFO(this->get_logger(), "RANDOM: %f", angle_dist(mt));
    switch (current_mode) {
      case JOY_TELEOP:
        joy_teleop_function(); break;
      case AUTO_WANDERING:
        auto_wandering_function(); break;
      case CRUISE_CONTROL:
        cruise_control_function(); break;
      default:
        break;
    }
  }

  void auto_wandering_function() {
    return; // TODO: Auto Wandering
  }

  void cruise_control_function() {
    return; // TODO: Cruise Control
  }

  void joy_teleop_function() {
    pub_twist_->publish(twist_cmd);
  }

  void joy_callback(const Joy &msg) {
    if (msg.buttons[JoyButtons::R1]) {
      twist_cmd.twist.linear.set__x(msg.axes[JoyAxes::L_Y]);
      twist_cmd.twist.angular.set__z(msg.axes[JoyAxes::L_X]);
    }
  }

  // void laser_scan_callback(const LaserScan &msg) {
  //   if (current_mode != AUTO_WANDERING) return;
  //   if (std::any_of(msg.ranges.begin(), msg.ranges.end(), [](float val) { return val < 0.1; })) {
  //     twist_cmd = TwistStamped();
  //     pub_twist_->publish(twist_cmd);
  //     float new_angle = angle_dist(mt);
  //   }
  // }
  //
  // void odometry_callback(const Odometry &msg) {
  //   if (current_mode != CRUISE_CONTROL) return;
  //   pid_time = msg.header.stamp.nanosec;
  //   // angular_twist = msg.twist.twist.linear;
  //   // linear_twist = msg.twist.twist.angular;
  // }

};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);

  TTBControlNode::SharedPtr control_node = std::make_shared<TTBControlNode>(); // Must keep reference to shared pointer or fails to start
  rclcpp::executors::SingleThreadedExecutor exec {};
  exec.add_node(control_node);
  exec.spin();

  rclcpp::shutdown();
  return 0;
}
