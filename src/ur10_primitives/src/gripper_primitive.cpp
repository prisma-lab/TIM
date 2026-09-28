#include "ur10_primitives/gripper_primitive.hpp"
#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <control_msgs/msg/joint_tolerance.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>
#include <pluginlib/class_list_macros.hpp>

namespace ur10_primitives
{
using primitive_manager::Status;

template<typename T>
T parameter(rclcpp::Node & node, const std::string & name, const T & value)
{
  if (!node.has_parameter(name)) {node.declare_parameter<T>(name, value);}
  return node.get_parameter(name).get_value<T>();
}

void GripperPrimitive::initialize(rclcpp::Node & node, const std::string & name)
{
  // Controller settings are shared; each named instance has its own target.
  const auto action = parameter<std::string>(node, "gripper.action_name",
    "/robotiq_gripper_controller/follow_joint_trajectory");
  const auto joints = parameter<std::string>(node, "gripper.joint_states_topic", "/joint_states");
  joint_name_ = parameter<std::string>(node, "gripper.joint_name", "robotiq_85_left_knuckle_joint");
  target_ = parameter(node, name + ".target_position", std::numeric_limits<double>::quiet_NaN());
  state_fact_ = parameter<std::string>(node, name + ".state_fact", "");
  tolerance_ = parameter(node, "gripper.position_tolerance", 0.02);
  motion_duration_ = parameter(node, "gripper.motion_duration", 2.0);
  server_timeout_ = parameter(node, "gripper.server_timeout", 10.0);
  execution_timeout_ = parameter(node, "gripper.execution_timeout", 20.0);
  feedback_timeout_ = parameter(node, "gripper.feedback_timeout", 2.0);
  if (!std::isfinite(target_) || target_ < 0 || target_ > 0.8 || state_fact_.empty()) {
    throw std::invalid_argument(name + " needs target_position in [0, 0.8] and a state_fact");
  }
  for (double value : {tolerance_, motion_duration_, server_timeout_, execution_timeout_, feedback_timeout_}) {
    if (!std::isfinite(value) || value <= 0) {throw std::invalid_argument("Gripper tolerances/timeouts must be positive and finite");}
  }
  client_ = rclcpp_action::create_client<Action>(&node, action);
  joint_sub_ = node.create_subscription<sensor_msgs::msg::JointState>(joints, rclcpp::SensorDataQoS(),
    [this](sensor_msgs::msg::JointState::ConstSharedPtr msg) {
      auto found = std::find(msg->name.begin(), msg->name.end(), joint_name_);
      if (found == msg->name.end()) {return;}
      auto index = static_cast<size_t>(std::distance(msg->name.begin(), found));
      if (index >= msg->position.size() || !std::isfinite(msg->position[index])) {return;}
      position_ = msg->position[index];
      have_position_ = true;
      joint_received_ = Clock::now();
      ++joint_sequence_;
    });
}

bool GripperPrimitive::fresh() const
{
  return have_position_ && std::chrono::duration<double>(Clock::now() - joint_received_).count() < feedback_timeout_;
}

bool GripperPrimitive::at_target() const
{
  return fresh() && std::abs(position_ - target_) <= tolerance_;
}

std::vector<primitive_manager::Observation> GripperPrimitive::observe() const
{
  return {{state_fact_, at_target()}};
}

void GripperPrimitive::deadline_after(double seconds)
{
  deadline_ = Clock::now() + std::chrono::duration_cast<Clock::duration>(std::chrono::duration<double>(seconds));
}

bool GripperPrimitive::execute(const std::vector<std::string> & args)
{
  if (busy()) {throw std::logic_error("Cannot execute a busy gripper plugin");}
  ++generation_;
  if (!args.empty()) {finish(Status::FAILED, "This gripper primitive takes no arguments"); return false;}
  if (at_target()) {finish(Status::SUCCEEDED, "Fresh joint feedback already satisfies the target"); return true;}
  status_ = Status::RUNNING;
  feedback_ = "Waiting for the controller and fresh joint feedback";
  phase_ = Phase::WAITING;
  deadline_after(server_timeout_);
  return true;
}

void GripperPrimitive::tick()
{
  if (!busy() || cancel_requested_) {return;}
  if (phase_ == Phase::VERIFYING && joint_sequence_ > verification_sequence_ && at_target()) {
    finish(Status::SUCCEEDED, "Target confirmed at " + std::to_string(position_) + " rad");
  } else if (Clock::now() >= deadline_) {
    if (phase_ == Phase::WAITING) {finish(Status::FAILED, "Controller or fresh joint feedback unavailable");}
    else if (phase_ == Phase::VERIFYING) {finish(Status::FAILED, "Controller succeeded but fresh joint feedback did not confirm the target");}
    else {
      timed_out_ = true;
      cancel_requested_ = true;
      status_ = Status::FAILED;
      feedback_ = "Execution timed out; awaiting cancellation/result";
      request_cancel();
    }
  } else if (phase_ == Phase::WAITING && client_->action_server_is_ready() && fresh()) {
    send_goal();
  }
}

void GripperPrimitive::send_goal()
{
  Action::Goal goal;
  goal.trajectory.joint_names = {joint_name_};
  trajectory_msgs::msg::JointTrajectoryPoint point;
  point.positions = {target_};
  point.velocities = {0.0};
  point.time_from_start = rclcpp::Duration::from_seconds(motion_duration_);
  goal.trajectory.points = {point};
  control_msgs::msg::JointTolerance tolerance;
  tolerance.name = joint_name_;
  tolerance.position = tolerance_;
  tolerance.velocity = -1.0;
  tolerance.acceleration = -1.0;
  goal.goal_tolerance = {tolerance};
  goal.goal_time_tolerance = rclcpp::Duration::from_seconds(2.0);

  const auto generation = generation_;
  rclcpp_action::Client<Action>::SendGoalOptions options;
  options.goal_response_callback = [this, generation](Handle::SharedPtr handle) {
    if (generation != generation_) {
      if (handle) {client_->async_cancel_goal(handle);}
      return;
    }
    if (!handle) {
      finish(timed_out_ ? Status::FAILED : cancel_requested_ ? Status::CANCELLED : Status::FAILED,
        "Controller rejected the trajectory");
      return;
    }
    handle_ = handle;
    phase_ = Phase::RUNNING;
    if (cancel_requested_) {request_cancel();}
    else {feedback_ = "Controller accepted the trajectory";}
  };
  options.result_callback = [this, generation](const Handle::WrappedResult & result) {
    if (generation != generation_) {return;}
    if (timed_out_) {finish(Status::FAILED, "Timed-out controller command has ended; reset is required");}
    else if (cancel_requested_) {
      // Success may race with cancellation; either terminal result confirms
      // that the controller ended the trajectory before the busy slot is freed.
      if (result.code == rclcpp_action::ResultCode::CANCELED || result.code == rclcpp_action::ResultCode::SUCCEEDED) {
        finish(Status::CANCELLED, "Controller command ended after cancellation request");
      } else {finish(Status::FAILED, "Controller aborted during cancellation");}
    } else if (result.code != rclcpp_action::ResultCode::SUCCEEDED || !result.result ||
      result.result->error_code != Action::Result::SUCCESSFUL) {
      finish(Status::FAILED, "Controller failed: " + (result.result ? result.result->error_string : "missing result"));
    } else {
      phase_ = Phase::VERIFYING;
      verification_sequence_ = joint_sequence_;
      deadline_after(feedback_timeout_);
      feedback_ = "Controller succeeded; checking measured joint position";
    }
  };
  phase_ = Phase::SENDING;
  deadline_after(execution_timeout_);
  try {client_->async_send_goal(goal, options);}
  catch (const std::exception & error) {
    // Transport failure leaves the controller state uncertain. Retain the busy
    // slot for recovery instead of allowing another trajectory to overlap it.
    timed_out_ = true;
    cancel_requested_ = true;
    status_ = Status::FAILED;
    feedback_ = std::string("Goal transport failed; recover controller and restart manager: ") + error.what();
  }
}

void GripperPrimitive::request_cancel()
{
  if (!handle_ || cancel_sent_) {return;}
  cancel_sent_ = true;
  try {client_->async_cancel_goal(handle_);}
  catch (const std::exception & error) {
    timed_out_ = true;
    status_ = Status::FAILED;
    feedback_ = std::string("Cancellation transport failed; awaiting controller result: ") + error.what();
  }
}

void GripperPrimitive::cancel()
{
  if (!busy() || cancel_requested_) {return;}
  if (phase_ == Phase::WAITING || phase_ == Phase::VERIFYING) {
    finish(Status::CANCELLED, "Cancelled with no controller trajectory pending");
    return;
  }
  cancel_requested_ = true;
  status_ = Status::CANCELLING;
  feedback_ = "Waiting for the controller command to end";
  request_cancel();
}

void GripperPrimitive::finish(Status status, const std::string & feedback)
{
  phase_ = Phase::IDLE;
  status_ = status;
  feedback_ = feedback;
  handle_.reset();
}

void GripperPrimitive::reset()
{
  if (busy()) {throw std::logic_error("Cannot reset before controller execution has ended");}
  status_ = Status::IDLE;
  feedback_.clear();
  handle_.reset();
  cancel_requested_ = timed_out_ = cancel_sent_ = false;
}
}  // namespace ur10_primitives

PLUGINLIB_EXPORT_CLASS(ur10_primitives::GripperPrimitive, primitive_manager::PrimitiveBase)
