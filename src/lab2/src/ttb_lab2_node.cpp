#include <cmath>
#include <cstdlib>
#include <functional>
#include <memory>
#include <mutex>
#include <thread>

#include <rclcpp/create_timer.hpp>
#include <rclcpp/executors.hpp>
#include <rclcpp/executors/single_threaded_executor.hpp>
#include <rclcpp/publisher_base.hpp>
#include <rclcpp/qos.hpp>
#include <rclcpp/subscription.hpp>
#include <rclcpp/utilities.hpp>

#include <rclcpp_action/rclcpp_action.hpp>
#include <lab2/action/go_to_goal.hpp>

#include <geometry_msgs/msg/twist.hpp>
#include <irobot_create_msgs/msg/ir_intensity_vector.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp_action/server.hpp>
#include <rmw/qos_profiles.h>
#include <sensor_msgs/msg/joy.hpp>

using namespace std::chrono_literals;
using namespace std::placeholders;

using namespace sensor_msgs::msg;
using namespace geometry_msgs::msg;
using namespace nav_msgs::msg;
using namespace irobot_create_msgs::msg;

class TTBLab2Node : public rclcpp::Node {

  using GoToGoal = lab2::action::GoToGoal;
  using GoToGoalHandle = rclcpp_action::ServerGoalHandle<GoToGoal>;

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

  enum Action {
    NONE,
    GO_TO_GOAL,
    MAKE_SQUARE,
    FOLLOW_CIRCLE,
    TRACK_SHAPE,
  };

private:
  rclcpp::TimerBase::SharedPtr timer_;

  // Subscriptions
  rclcpp::Subscription<Joy>::SharedPtr sub_joy_;
  rclcpp::Subscription<Odometry>::SharedPtr sub_odometry_;
  rclcpp::Subscription<irobot_create_msgs::msg::IrIntensityVector>::SharedPtr sub_ir_;

  // Publishers
  rclcpp::Publisher<Twist>::SharedPtr pub_twist_;

  // Actions
  rclcpp_action::Server<GoToGoal>::SharedPtr serv_go_to_goal_;

  // Commands
  Twist joy_cmd_;

  // Vehicle State
  Vector3 pos_;

  // Action State
  std::mutex goal_mtx_;
  bool goal_active_;

  double current_angle_;
  Vector3 goal_pos_;
  double goal_margin_;

public:
  explicit TTBLab2Node() : rclcpp::Node("ttb_lab2_node") {
    timer_ = this->create_wall_timer(
        100ms, std::bind(&TTBLab2Node::command_loop_function, this));

    sub_joy_ = this->create_subscription<Joy>(
        "/TTB06/joy",
        rclcpp::QoS(
            rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_sensor_data),
            rmw_qos_profile_sensor_data),
        std::bind(&TTBLab2Node::joy_callback, this, _1));

    sub_odometry_ = this->create_subscription<Odometry>(
        "/TTB06/odom",
        rclcpp::QoS(
            rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_sensor_data),
            rmw_qos_profile_sensor_data),
        std::bind(&TTBLab2Node::odometry_callback, this, _1));

    serv_go_to_goal_ = rclcpp_action::create_server<GoToGoal>(
      this,
      "/TTB06/go_to_goal",
      std::bind(&TTBLab2Node::handle_goal_go_to_goal, this, _1, _2),
      std::bind(&TTBLab2Node::handle_cancel_go_to_goal, this, _1),
      std::bind(&TTBLab2Node::handle_accepted_go_to_goal, this, _1)
    );

    pub_twist_ = this->create_publisher<Twist>("/TTB06/cmd_vel", 10);

    pos_.x = 0;
    pos_.y = 0;
    pos_.z = 0;
  }

private:
  void command_loop_function() {
    // pub_twist_->publish(joy_cmd_);
    return;
  }

  void joy_callback(const Joy &msg) {
    joy_cmd_.linear.x = msg.axes[L_Y];
    joy_cmd_.angular.z = msg.axes[L_X];
  }

  void odometry_callback(const Odometry &msg) {
    pos_.x = msg.pose.pose.position.x;
    pos_.y = msg.pose.pose.position.y;

    double qx = msg.pose.pose.orientation.x;
    double qy = msg.pose.pose.orientation.y;
    double qz = msg.pose.pose.orientation.z;
    double qw = msg.pose.pose.orientation.w;
    current_angle_ = std::atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz)); // quarternion yaw angle
  }

  rclcpp_action::GoalResponse handle_goal_go_to_goal(const rclcpp_action::GoalUUID &uuid, [[maybe_unused]] std::shared_ptr<const GoToGoal::Goal> goal) {
    std::string goal_name {rclcpp_action::to_string(uuid)};
    std::lock_guard<std::mutex> lock(goal_mtx_); // Locks mutex until end of scope
    if (goal_active_) {
      RCLCPP_INFO(this->get_logger(), "BUSY - Rejecting Action Request: %s", goal_name.c_str());
      return rclcpp_action::GoalResponse::REJECT;
    }

    goal_active_ = true;
    goal_pos_.set__x(pos_.x + goal->position[0]);
    goal_pos_.set__y(pos_.y + goal->position[1]);
    goal_margin_ = goal->margin;

    RCLCPP_INFO(this->get_logger(), "FREE - Accepting Action Request: %s - [%lf, %lf]", goal_name.c_str(), goal_pos_.x, goal_pos_.y);
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse handle_cancel_go_to_goal([[maybe_unused]] const std::shared_ptr<GoToGoalHandle> handle) {
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handle_accepted_go_to_goal(const std::shared_ptr<GoToGoalHandle> handle) {
    std::thread{std::bind(&TTBLab2Node::execute_go_to_goal, this, _1), handle}.detach();
  }

  void execute_go_to_goal(const std::shared_ptr<GoToGoalHandle> handle) {
    auto feedback = std::make_shared<GoToGoal::Feedback>();
    auto result = std::make_shared<GoToGoal::Result>();

    std::string goal_name {rclcpp_action::to_string(handle->get_goal_id())};

    double Kv = 0.1;
    double Kp = 0.1;
    Twist gtg_twist;

    rclcpp::WallRate rate(10); // 10 Hz loop
    while (rclcpp::ok()) {
      if (handle->is_canceling()) {
        break;
      }

      double distance = std::sqrt((goal_pos_.x - pos_.x)*(goal_pos_.x - pos_.x) + (goal_pos_.y - pos_.y)*(goal_pos_.y - pos_.y));
      double abs_distance = std::abs(distance);
      if (abs_distance < goal_margin_) {
        break;
      }

      double velocity = Kv * distance;
      double theta = std::atan2(goal_pos_.y - pos_.y, goal_pos_.x - pos_.x);
      double gamma = Kp * (std::atan2(std::sin(theta), std::cos(theta)));

      gtg_twist.linear.set__x(velocity);
      gtg_twist.angular.set__z(gamma);
      pub_twist_->publish(gtg_twist);

      feedback->set__velocity(velocity);
      feedback->set__gamma(gamma);
      feedback->set__distance(std::abs(distance));
      handle->publish_feedback(feedback);

      rate.sleep();
    }

    pub_twist_->publish(Twist());
    if (handle->is_canceling()) {
      RCLCPP_INFO(this->get_logger(), "%s Action Cancelled", goal_name.c_str());
      handle->canceled(result);
    }
    else {
      RCLCPP_INFO(this->get_logger(), "%s Action Succeeded", goal_name.c_str());
      handle->succeed(result);
    }

    std::lock_guard<std::mutex> lock(goal_mtx_);
    goal_active_ = false;
  }
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);

  TTBLab2Node::SharedPtr control_node = std::make_shared<TTBLab2Node>();
  rclcpp::executors::SingleThreadedExecutor exec{};
  exec.add_node(control_node);
  exec.spin();

  rclcpp::shutdown();
  return 0;
}
