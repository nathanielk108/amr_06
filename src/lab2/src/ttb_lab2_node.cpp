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
#include <lab2/action/make_square.hpp>

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

  using MakeSquare = lab2::action::MakeSquare;
  using MakeSquareHandle = rclcpp_action::ServerGoalHandle<MakeSquare>;


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
  rclcpp_action::Server<MakeSquare>::SharedPtr serv_make_square_;

  // Commands
  Twist joy_cmd_;

  // Vehicle State
  Vector3 pos_;
  Vector3 vel_;

  // Action State
  std::mutex goal_mtx_;
  bool goal_active_ = false;

  double current_angle_ = 0.0;

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

    serv_make_square_ = rclcpp_action::create_server<MakeSquare>(
    this,
    "/TTB06/make_square",
    std::bind(&TTBLab2Node::handle_goal_make_square, this, _1, _2),
    std::bind(&TTBLab2Node::handle_cancel_make_square, this, _1),
    std::bind(&TTBLab2Node::handle_accepted_make_square, this, _1)
    );

    pub_twist_ = this->create_publisher<Twist>("/TTB06/cmd_vel", 10);

    pos_.x = 0;
    pos_.y = 0;
    pos_.z = 0;
  }

private:
  void command_loop_function() {
    return;
  }

  void joy_callback(const Joy &msg) {
    joy_cmd_.linear.x = msg.axes[L_Y];
    joy_cmd_.angular.z = msg.axes[L_X];
  }

  void odometry_callback(const Odometry &msg) {
    // Global Frame
    pos_.x = msg.pose.pose.position.x;
    pos_.y = msg.pose.pose.position.y;

    vel_.x = msg.twist.twist.linear.x;
    vel_.y = msg.twist.twist.linear.y;

    // Robot Frame
    double qx = msg.pose.pose.orientation.x;
    double qy = msg.pose.pose.orientation.y;
    double qz = msg.pose.pose.orientation.z;
    double qw = msg.pose.pose.orientation.w;
    current_angle_ = std::atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz)); // quarternion yaw angle
  }

  rclcpp_action::GoalResponse handle_goal(const rclcpp_action::GoalUUID &uuid) {
    std::string goal_name {rclcpp_action::to_string(uuid)};
    std::lock_guard<std::mutex> lock(goal_mtx_); // Locks mutex until end of scope
    if (goal_active_) {
      RCLCPP_INFO(this->get_logger(), "BUSY - Rejecting Action Request: %s", goal_name.c_str());
      return rclcpp_action::GoalResponse::REJECT;
    }

    goal_active_ = true;
    RCLCPP_INFO(this->get_logger(), "FREE - Accepting Action Request: %s", goal_name.c_str());
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::GoalResponse handle_goal_go_to_goal(const rclcpp_action::GoalUUID &uuid, [[maybe_unused]] std::shared_ptr<const GoToGoal::Goal> goal) {
    return handle_goal(uuid);
  }

  rclcpp_action::CancelResponse handle_cancel_go_to_goal([[maybe_unused]] const std::shared_ptr<GoToGoalHandle> handle) {
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  rclcpp_action::GoalResponse handle_goal_make_square(const rclcpp_action::GoalUUID &uuid, [[maybe_unused]] std::shared_ptr<const MakeSquare::Goal> goal) {

    return handle_goal(uuid);
    }

rclcpp_action::CancelResponse handle_cancel_make_square([[maybe_unused]] const std::shared_ptr<MakeSquareHandle> handle) {
    return rclcpp_action::CancelResponse::ACCEPT;
    }

    void handle_accepted_make_square(const std::shared_ptr<MakeSquareHandle> handle) {

    std::thread{std::bind(&TTBLab2Node::execute_make_square, this, _1), handle}.detach();
}

  void handle_accepted_go_to_goal(const std::shared_ptr<GoToGoalHandle> handle) {
    auto goal = handle->get_goal();
    std::thread{std::bind(&TTBLab2Node::execute_go_to_goal, this, _1), handle}.detach();
  }

  void execute_go_to_goal(const std::shared_ptr<GoToGoalHandle> handle) {
    std::string goal_name {rclcpp_action::to_string(handle->get_goal_id())};
    auto goal = handle->get_goal();

    auto feedback = std::make_shared<GoToGoal::Feedback>();
    auto result = std::make_shared<GoToGoal::Result>();

    double goal_pos_x = pos_.x + (goal->position[0] * std::cos(current_angle_)) - (goal->position[1] * std::sin(current_angle_));
    double goal_pos_y = pos_.y + (goal->position[0] * std::sin(current_angle_)) + (goal->position[1] * std::cos(current_angle_));
    double kv = goal->kv;
    double kp = goal->kp;
    double margin = goal->margin;

    bool pid = goal->pid;
    double setpoint = goal->setpoint;
    double pid_kp = goal->pid_kp;
    double pid_ki = goal->pid_ki;
    double pid_kd = goal->pid_kd;

    double previous_error = 0;
    double integral = 0;

    RCLCPP_INFO(this->get_logger(), "GOAL: %f, %f", goal_pos_x, goal_pos_y);

    Twist gtg_twist;

    rclcpp::WallRate rate(10); // 10 Hz loop
    while (rclcpp::ok()) {
      if (handle->is_canceling()) {
        break;
      }

      double distance = std::sqrt((goal_pos_x - pos_.x)*(goal_pos_x - pos_.x) + (goal_pos_y - pos_.y)*(goal_pos_y - pos_.y));
      if (distance < margin) {
        break;
      }

      double velocity = 0;
      if (pid) {
        double dt = 0.1;
        double error = setpoint - vel_.x;
        integral += error * dt;
        double derivative = (error - previous_error)/dt;
        velocity += pid_kp*error + pid_ki*integral + pid_kd*derivative;
        previous_error = error;
      }
      else {
        velocity = kv * distance;
      }
      double desiredHeading = std::atan2(goal_pos_y - pos_.y, goal_pos_x - pos_.x);
      double headingError = std::atan2(std::sin(desiredHeading - current_angle_), std::cos(desiredHeading - current_angle_));
      double steering = kp * headingError;

      gtg_twist.linear.set__x(velocity);
      gtg_twist.angular.set__z(steering);
      pub_twist_->publish(gtg_twist);

      feedback->set__velocity(velocity);
      feedback->set__steering(steering);
      feedback->set__distance(distance);
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

  void execute_make_square(const std::shared_ptr<MakeSquareHandle> handle) {
  std::string goal_name {
    rclcpp_action::to_string(handle->get_goal_id())
  };

  auto goal = handle->get_goal();

  auto feedback = std::make_shared<MakeSquare::Feedback>();
  auto result = std::make_shared<MakeSquare::Result>();

  double side = goal->side_length;

  // Save where the square starts
  double start_x = pos_.x;
  double start_y = pos_.y;
  double start_angle = current_angle_;

  /*
   * Define the four corners in the robot's LOCAL starting frame:
   *
   * start ---- corner 1
   *   |            |
   *   |            |
   * corner 3 -- corner 2
   *
   * corner 4 is starting point.
   */
  double local_x[4] = {
    side,
    side,
    0.0,
    0.0
  };

  double local_y[4] = {
    0.0,
    side,
    side,
    0.0
  };

  rclcpp::WallRate rate(10);

  double total_distance = 0.0;
  double previous_x = pos_.x;
  double previous_y = pos_.y;

  for (int corner = 0; corner < 4 && rclcpp::ok(); corner++) {

    if (handle->is_canceling()) {
      break;
    }

    // Convert the corner from the robot's starting frame
    // into the global odometry frame.
    double goal_x =
        start_x
        + local_x[corner] * std::cos(start_angle)
        - local_y[corner] * std::sin(start_angle);

    double goal_y =
        start_y
        + local_x[corner] * std::sin(start_angle)
        + local_y[corner] * std::cos(start_angle);

    RCLCPP_INFO(
        this->get_logger(),
        "Corner %d: (%f, %f)",
        corner + 1,
        goal_x,
        goal_y
    );

    // STEP 1: TURN TOWARD CORNER

    while (rclcpp::ok()) {

      if (handle->is_canceling()) {
        break;
      }

      double desired_heading =
          std::atan2(goal_y - pos_.y, goal_x - pos_.x);

      double heading_error =
          std::atan2(
              std::sin(desired_heading - current_angle_),
              std::cos(desired_heading - current_angle_)
          );

      // About 3 degrees
      if (std::abs(heading_error) < 0.05) {
        break;
      }

      Twist turn_twist;

      if (goal->pid) {
        // Proportional heading control
        double kp_turn = 1.5;
        turn_twist.angular.z = kp_turn * heading_error;
      }
      else {
        // Bang-bang control, but based on sensor feedback
        turn_twist.angular.z =
            (heading_error > 0.0) ? 0.5 : -0.5;
      }

      pub_twist_->publish(turn_twist);

      rate.sleep();
    }

    // Fully stop before moving forward
    pub_twist_->publish(Twist());

    if (handle->is_canceling()) {
      break;
    }

    // STEP 2: DRIVE STRAIGHT TO CORNER

    while (rclcpp::ok()) {

      if (handle->is_canceling()) {
        break;
      }

      double dx = goal_x - pos_.x;
      double dy = goal_y - pos_.y;

      double distance = std::sqrt(dx * dx + dy * dy);

      // Corner reached
      if (distance < 0.05) {
        break;
      }

      double desired_heading = std::atan2(dy, dx);

      double heading_error =
          std::atan2(
              std::sin(desired_heading - current_angle_),
              std::cos(desired_heading - current_angle_)
          );

      Twist drive_twist;

      if (goal->pid) {
        // Similar to your GoToGoal controller
        double kv = 0.5;
        double kp = 1.5;

        drive_twist.linear.x = kv * distance;
        drive_twist.angular.z = kp * heading_error;

        // Don't let it get excessively fast
        if (drive_twist.linear.x > 0.3) {
          drive_twist.linear.x = 0.3;
        }
      }
      else {
        // Constant forward velocity with heading correction
        drive_twist.linear.x = 0.2;

        double kp = 1.0;
        drive_twist.angular.z = kp * heading_error;
      }

      pub_twist_->publish(drive_twist);

      // Distance traveled since the last loop
      double step_dx = pos_.x - previous_x;
      double step_dy = pos_.y - previous_y;

      total_distance +=
          std::sqrt(step_dx * step_dx + step_dy * step_dy);

      previous_x = pos_.x;
      previous_y = pos_.y;

      // Action feedback
    feedback->distance = static_cast<float>(distance);
    handle->publish_feedback(feedback);

      rate.sleep();
    }

    // Stop at each corner before beginning the next turn
    pub_twist_->publish(Twist());
  }

  // Make absolutely sure robot stops
  pub_twist_->publish(Twist());

  result->position = {
    static_cast<float>(pos_.x),
    static_cast<float>(pos_.y)
  };


  if (handle->is_canceling()) {
    RCLCPP_INFO(
        this->get_logger(),
        "%s MakeSquare Action Cancelled",
        goal_name.c_str()
    );

    handle->canceled(result);
  }
  else {
    RCLCPP_INFO(
        this->get_logger(),
        "%s MakeSquare Action Succeeded",
        goal_name.c_str()
    );

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