#include "ur10_hardware_primitives/gripper_primitive.hpp"
#include <pluginlib/class_list_macros.hpp>

namespace ur10_hardware_primitives
{
using primitive_manager::Status;
void GripperPrimitive::initialize(rclcpp::Node & node, const std::string & name)
{
  state_.initialize(node);
  target_ = setting(node, name + ".target_position", -1.0);
  require_object_ = setting(node, name + ".require_object", false);
  fact_ = setting<std::string>(node, name + ".state_fact", "");
  timeout_ = setting(node, "gripper.execution_timeout", 15.0);
  tolerance_ = setting(node, "gripper.position_tolerance", 0.02);
  if (!std::isfinite(target_) || target_ < 0 || target_ > 0.695 ||
    !std::isfinite(timeout_) || timeout_ <= 0 || !std::isfinite(tolerance_) || tolerance_ <= 0 ||
    (require_object_ && target_ <= 0.02)) {throw std::invalid_argument("Invalid 2F-140 target or timeouts");}
  client_ = rclcpp_action::create_client<Action>(&node,
    setting<std::string>(node, "gripper.action_name", "/robotiq_gripper_controller/gripper_cmd"));
}

bool GripperPrimitive::at_target() const
{
  if (!state_.healthy()) {return false;}
  if (require_object_) {return state_.held();}
  if (target_ > 0.02 && state_.held()) {return true;}
  return state_.object == 3 && std::abs(state_.position - target_) <= tolerance_;
}

std::vector<primitive_manager::Observation> GripperPrimitive::observe() const
{return fact_.empty() ? std::vector<primitive_manager::Observation>{} :
  std::vector<primitive_manager::Observation>{{fact_, at_target()}};}

bool GripperPrimitive::execute(const std::vector<std::string> & args)
{
  if (active_) {throw std::logic_error("Gripper still busy");}
  if (!args.empty() || !state_.healthy() || !client_->action_server_is_ready()) {
    finish(Status::FAILED, "Gripper requires no arguments, healthy measured feedback, and its action server"); return false;
  }
  active_ = pending_ = true; stopping_ = false; status_ = Status::RUNNING;
  feedback_ = "Sending 2F-140 gripper command";
  deadline_ = Clock::now() + std::chrono::duration_cast<Clock::duration>(std::chrono::duration<double>(timeout_));
  sequence_ = state_.sequence;
  const auto generation = ++generation_;
  Action::Goal goal; goal.command.position = target_;
  // Humble's GripperActionController does not forward per-goal force to this
  // hardware. Force/speed are configured in the driver's xacro instead.
  goal.command.max_effort = 0.0;
  rclcpp_action::Client<Action>::SendGoalOptions options;
  options.goal_response_callback = [this, generation](Handle::SharedPtr handle) {
    if (generation != generation_) {if (handle) {client_->async_cancel_goal(handle);} return;}
    if (!handle) {finish(stopping_ ? final_status_ : Status::FAILED, "Gripper controller rejected command"); return;}
    handle_ = handle;
    if (stopping_) {request_cancel();}
  };
  options.result_callback = [this, generation](const Handle::WrappedResult & result) {
    if (generation != generation_) {return;}
    pending_ = false; handle_.reset();
    if (stopping_) {finish(final_status_, feedback_); return;}
    if (result.code != rclcpp_action::ResultCode::SUCCEEDED || !result.result) {
      finish(Status::FAILED, "Gripper controller aborted command"); return;
    }
    // Require a measured sample after the action ended, not just its success bit.
    sequence_ = state_.sequence;
    feedback_ = "Checking measured gripper result";
  };
  try {client_->async_send_goal(goal, options);}
  catch (const std::exception & e) {stop(Status::FAILED, std::string("Uncertain gripper transport: ") + e.what());}
  return true;
}

void GripperPrimitive::tick()
{
  if (!active_ || stopping_) {return;}
  if (!state_.healthy()) {stop(Status::FAILED, "Gripper feedback stale, disconnected, or faulted");}
  else if (Clock::now() >= deadline_) {stop(Status::FAILED, "Gripper timed out; waiting for controller to end");}
  else if (!pending_ && state_.sequence > sequence_) {
    if (at_target()) {finish(Status::SUCCEEDED, require_object_ ? "Object detected while closing" : "Gripper state confirmed");}
    else if (require_object_ && state_.object == 3) {finish(Status::FAILED, "Gripper reached its target without detecting an object");}
  }
}
void GripperPrimitive::request_cancel()
{
  if (!handle_) {return;}
  try {client_->async_cancel_goal(handle_);}
  catch (const std::exception & e) {status_ = Status::FAILED; final_status_ = Status::FAILED;
    feedback_ = std::string("Cancellation transport uncertain; await result/recover: ") + e.what();}
}
void GripperPrimitive::stop(Status status, const std::string & message)
{
  stopping_ = true; final_status_ = status; feedback_ = message;
  status_ = status == Status::FAILED ? Status::FAILED : Status::CANCELLING;
  if (pending_) {request_cancel();} else {finish(status, message);}
}
void GripperPrimitive::cancel()
{if (active_ && !stopping_) {stop(Status::CANCELLED, "Waiting for gripper command to end");}}
void GripperPrimitive::finish(Status status, const std::string & message)
{active_ = pending_ = false; status_ = status; feedback_ = message; handle_.reset();}
void GripperPrimitive::reset()
{
  if (active_) {throw std::logic_error("Cannot reset busy gripper");}
  stopping_ = false; status_ = Status::IDLE; feedback_.clear(); handle_.reset();
}
}
PLUGINLIB_EXPORT_CLASS(ur10_hardware_primitives::GripperPrimitive, primitive_manager::PrimitiveBase)
