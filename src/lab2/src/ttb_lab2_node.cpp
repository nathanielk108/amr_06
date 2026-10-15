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
#include <lab2/action/pursue_path.hpp>

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

  using PursuePath = lab2::action::PursuePath;
  using PursuePathHandle = rclcpp_action::ServerGoalHandle<PursuePath>;


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

  enum Shape {
    CIRC, 
    STAR
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
  rclcpp_action::Server<PursuePath>::SharedPtr serv_pursue_path_;


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

    serv_pursue_path_ = rclcpp_action::create_server<PursuePath>(
    this,
    "/TTB06/pursue_path",
    std::bind(&TTBLab2Node::handle_goal_pursue_path, this, _1, _2),
    std::bind(&TTBLab2Node::handle_cancel_pursue_path, this, _1),
    std::bind(&TTBLab2Node::handle_accepted_pursue_path, this, _1)
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

  rclcpp_action::GoalResponse handle_goal_pursue_path(const rclcpp_action::GoalUUID &uuid, [[maybe_unused]] std::shared_ptr<const PursuePath::Goal> goal) {
    return handle_goal(uuid);
  }

  rclcpp_action::CancelResponse handle_cancel_pursue_path([[maybe_unused]] const std::shared_ptr<PursuePathHandle> handle) {
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handle_accepted_make_square(const std::shared_ptr<MakeSquareHandle> handle) {

    std::thread{std::bind(&TTBLab2Node::execute_make_square, this, _1), handle}.detach();
  }

  void handle_accepted_pursue_path(const std::shared_ptr<PursuePathHandle> handle) {

    std::thread{std::bind(&TTBLab2Node::execute_pursuit, this, _1), handle}.detach();
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
  bool smooth = goal->smooth;

  // Save starting pose
  double start_x = pos_.x;
  double start_y = pos_.y;
  double start_angle = current_angle_;

  /*
   * Four corners in the robot's LOCAL starting frame:
   *
   * start ---- corner 1
   *   |            |
   *   |            |
   * corner 3 -- corner 2
   *
   * corner 4 = starting point
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

  // Convert corners into global odometry coordinates
  double corner_x[4];
  double corner_y[4];

  for (int i = 0; i < 4; i++) {
    corner_x[i] =
        start_x
        + local_x[i] * std::cos(start_angle)
        - local_y[i] * std::sin(start_angle);

    corner_y[i] =
        start_y
        + local_x[i] * std::sin(start_angle)
        + local_y[i] * std::cos(start_angle);
  }

  rclcpp::WallRate rate(10);

  // =========================================================
  // SMOOTH MODE
  // Constant linear velocity through all four points
  // =========================================================
  if (smooth) {

    const double velocity = 0.2;
    const double kp = 1.5;
    const double corner_margin = 0.08;

    int corner = 0;

    while (rclcpp::ok() && corner < 4) {

      if (handle->is_canceling()) {
        break;
      }

      double dx = corner_x[corner] - pos_.x;
      double dy = corner_y[corner] - pos_.y;

      double distance = std::sqrt(dx * dx + dy * dy);

      // Close enough to this corner:
      // immediately begin steering toward the next one.
      if (distance < corner_margin) {

        RCLCPP_INFO(
            this->get_logger(),
            "Reached Corner %d",
            corner + 1
        );

        corner++;

        if (corner >= 4) {
          break;
        }

        // Recalculate using the next corner
        dx = corner_x[corner] - pos_.x;
        dy = corner_y[corner] - pos_.y;
        distance = std::sqrt(dx * dx + dy * dy);
      }

      // Desired heading toward current target
      double desired_heading =
          std::atan2(dy, dx);

      // Heading error based on odometry feedback
      double heading_error =
          std::atan2(
              std::sin(desired_heading - current_angle_),
              std::cos(desired_heading - current_angle_)
          );

      Twist square_twist;

      // Constant linear velocity
      square_twist.linear.x = velocity;

      // Continuously steer based on sensor feedback
      square_twist.angular.z = kp * heading_error;

      pub_twist_->publish(square_twist);

      feedback->distance = static_cast<float>(distance);
      handle->publish_feedback(feedback);

      rate.sleep();
    }
  }

  // =========================================================
  // STOP, TURN, AND GO MODE
  // =========================================================
  else {

    const double corner_margin = 0.05;
    const double heading_margin = 0.05;
    const double kp_turn = 1.5;
    const double kp_drive = 1.5;
    const double kv = 0.5;
    const double max_velocity = 0.3;

    for (int corner = 0; corner < 4 && rclcpp::ok(); corner++) {

      if (handle->is_canceling()) {
        break;
      }

      double goal_x = corner_x[corner];
      double goal_y = corner_y[corner];

      RCLCPP_INFO(
          this->get_logger(),
          "Moving to Corner %d: (%f, %f)",
          corner + 1,
          goal_x,
          goal_y
      );

      // -------------------------------------------------------
      // STEP 1: TURN TOWARD CORNER
      // -------------------------------------------------------
      while (rclcpp::ok()) {

        if (handle->is_canceling()) {
          break;
        }

        double desired_heading =
            std::atan2(
                goal_y - pos_.y,
                goal_x - pos_.x
            );

        double heading_error =
            std::atan2(
                std::sin(desired_heading - current_angle_),
                std::cos(desired_heading - current_angle_)
            );

        // Stop turning once aligned
        if (std::abs(heading_error) < heading_margin) {
          break;
        }

        Twist turn_twist;

        turn_twist.linear.x = 0.0;
        turn_twist.angular.z = kp_turn * heading_error;

        pub_twist_->publish(turn_twist);

        rate.sleep();
      }

      // Stop before moving forward
      pub_twist_->publish(Twist());

      if (handle->is_canceling()) {
        break;
      }

      // -------------------------------------------------------
      // STEP 2: DRIVE TO CORNER
      // -------------------------------------------------------
      while (rclcpp::ok()) {

        if (handle->is_canceling()) {
          break;
        }

        double dx = goal_x - pos_.x;
        double dy = goal_y - pos_.y;

        double distance = std::sqrt(dx * dx + dy * dy);

        // Corner reached
        if (distance < corner_margin) {
          break;
        }

        double desired_heading =
            std::atan2(dy, dx);

        double heading_error =
            std::atan2(
                std::sin(desired_heading - current_angle_),
                std::cos(desired_heading - current_angle_)
            );

        double velocity = kv * distance;

        if (velocity > max_velocity) {
          velocity = max_velocity;
        }

        Twist drive_twist;

        drive_twist.linear.x = velocity;
        drive_twist.angular.z = kp_drive * heading_error;

        pub_twist_->publish(drive_twist);

        feedback->distance = static_cast<float>(distance);
        handle->publish_feedback(feedback);

        rate.sleep();
      }

      // Stop at corner before turning toward next one
      pub_twist_->publish(Twist());
    }
  }

  // Stop after completing either mode
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

  void execute_pursuit(const std::shared_ptr<PursuePathHandle> handle) {
    std::string goal_name {rclcpp_action::to_string(handle->get_goal_id())};
    auto feedback = std::make_shared<PursuePath::Feedback>();
    auto result = std::make_shared<PursuePath::Result>();
    
    int shape = CIRC; // CIRC or STAR selection variable

    double radius = 1.0;
    int total_points = 30; // can change number of points per circle
    if (shape == STAR) {
      total_points = 5; // typical 5 point star
    }
    double deg_per_point = 360/total_points;
    double rad_per_point = (M_PI * deg_per_point) / 180; //convert from deg to rad
    std::vector<double> points_x; // establish list
    std::vector<double> points_y; // establish list
    int point_select = 0;
    for (int i = 0; i < total_points; i++) { // go through each of the points and establish cartesian coordinates for the circle
      double x_val_n;
      double y_val_n;
      if (shape == lab2::action::PursuePath::Goal::CIRC) {
        x_val_n = radius*std::cos(i*rad_per_point); 
        y_val_n = radius*std::sin(i*rad_per_point);
      }
      else if (shape == lab2::action::PursuePath::Goal::STAR) {
        x_val_n = radius*std::cos(2*i*rad_per_point); // making a double step so two full times around will hit all 5 points, 
        y_val_n = radius*std::sin(2*i*rad_per_point);
      }
      points_x.push_back(x_val_n);
      points_y.push_back(y_val_n);
    }

    // take current odometry, add the values of points to the current odom position as the new point and follow it dynamically
    // new points are x_d, y_d. Go to goal for [x_d, y_d]
    double x_d = pos_.x + points_x[point_select]; // [CHANGE #2] declared here so it compiles
    double y_d = pos_.y + points_y[point_select];

    double dt = 0.1;
    double integral = 0;
    double output = 0;
    double kp = 1.0; //tune
    double ki = 0.1; //tune
    double kh = 0.4; // tune
    double dist_d = .1; // distance d_d behind the pursuit point that the robot follows
    double follow_threshold = dist_d * 1.5; // give threshold before resetting the point, tunable

    Twist twist_cmd;


    rclcpp::WallRate rate(10); // 10 Hz loop
    while (rclcpp::ok()) {
      if (handle->is_canceling()) {
        break;
      }
      if (point_select > 2*total_points) { // two iterations of the shape
        break;
      }

    double x_error = (x_d - pos_.x); // x pulled from odometry
    double y_error = (y_d - pos_.y); // y pulled from odometry
    double x_error_2 = (x_d - pos_.x) * (x_d - pos_.x);
    double y_error_2 = (y_d - pos_.y) * (y_d - pos_.y);
    double follow_error = (std::sqrt(x_error_2 + y_error_2) - dist_d); // from lecture 3

    double slope_quotient = y_error/x_error;
    double theta_d = std::atan2(y_error, x_error); // use arctan for angle error

    double angle_error = theta_d - current_angle_; // current angle from odom quaternion to euler convert
    angle_error = std::atan2(std::sin(angle_error), std::cos(angle_error)); // using atan2 from Lecture 3
    double abs_error = std::fabs(angle_error);

    integral = integral + follow_error * dt; // PID
    output = output + kp * follow_error + ki * integral;
    if (output < 0) {
      output = 0;
    }

    twist_cmd.linear.x = output;
    twist_cmd.angular.z = kh * angle_error; // dynamic value based upon alignment with the path itself, should be v=r*theta related

    pub_twist_->publish(twist_cmd);

    // when follow error is less than a threshold, increment the follow point to the next step in [x_points, y_points]
    if (follow_error < follow_threshold) {
      point_select += 1;
      x_d = points_x[point_select];
      y_d = points_y[point_select];
    } // robot should then have the new point be more than .15 meters away and thus should direct towards the new point

    feedback->set__distance(follow_error);
    handle->publish_feedback(feedback);
    rate.sleep();
  }

  pub_twist_->publish(Twist());
  result->position = {static_cast<float>(pos_.x), static_cast<float>(pos_.y)};
    
  if (handle->is_canceling()) {
    RCLCPP_INFO(this->get_logger(), "%s PursuePath Action Cancelled", goal_name.c_str());
    handle->canceled(result);
  } 
  else {
    RCLCPP_INFO(this->get_logger(), "%s PursuePath Action Succeeded", goal_name.c_str());
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
