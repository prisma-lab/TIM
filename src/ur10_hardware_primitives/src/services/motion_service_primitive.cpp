#include "ur10_hardware_primitives/services/motion_service_primitive.hpp"

#include <charconv>
#include <cmath>
#include <stdexcept>

namespace ur10_hardware_primitives::services
{
using primitive_manager::Status;

void MotionServicePrimitive::initialize(rclcpp::Node & node, const std::string & name)
{
  node_ = &node;
  name_ = name;
  response_timeout_ = parameter(node, "services.response_timeout", 5.0);
  execution_timeout_ = parameter(node, "services.execution_timeout", 180.0);
  for (double timeout : {response_timeout_, execution_timeout_}) {
    if (!std::isfinite(timeout) || timeout <= 0.0) {
      throw std::invalid_argument("Service timeouts must be positive and finite");
    }
  }
  const auto start_topic = parameter<std::string>(node, "services.start_topic", "/motion_start");
  const auto end_topic = parameter<std::string>(node, "services.end_topic", "/motion_end");
  // Subscribe before any request. Volatile events are associated by unique ID,
  // never by which topic happened to publish most recently.
  const auto qos = rclcpp::QoS(1000).reliable().durability_volatile();
  start_sub_ = node.create_subscription<std_msgs::msg::String>(start_topic, qos,
    [this](std_msgs::msg::String::ConstSharedPtr msg) {receive_event(msg->data, true);});
  end_sub_ = node.create_subscription<std_msgs::msg::String>(end_topic, qos,
    [this](std_msgs::msg::String::ConstSharedPtr msg) {receive_event(msg->data, false);});
  configure();
}

bool MotionServicePrimitive::execute(const std::vector<std::string> & args)
{
  if (active_) {throw std::logic_error("Cannot dispatch while a remote motion is pending");}
  try {
    dispatch(args);
    return true;
  } catch (const std::exception & error) {
    fail(error.what());
    return false;
  }
}

void MotionServicePrimitive::begin_request()
{
  if (start_sub_->get_publisher_count() == 0 || end_sub_->get_publisher_count() == 0) {
    throw std::runtime_error("Discover compatible motion_start/motion_end publishers before dispatch");
  }
  starts_.clear();
  ends_.clear();
  active_ = true;
  response_received_ = ids_known_ = faulted_ = false;
  requested_at_ = Clock::now();
  status_ = Status::RUNNING;
  feedback_ = "Waiting for service response and motion IDs";
}

void MotionServicePrimitive::receive_response(bool accepted, const std::vector<std::uint64_t> & ids)
{
  response_received_ = true;
  if (!accepted) {
    active_ = false;  // Contract: rejection schedules no motion.
    fail("Service rejected the request");
    return;
  }
  const std::set<std::uint64_t> unique(ids.begin(), ids.end());
  if (ids.empty() || unique.size() != ids.size()) {
    // It may have started. Do not release the manager's execution slot.
    fail("Accepted request returned empty or duplicate motion_ids; execution is unknown");
    return;
  }
  first_id_ = ids.front();
  last_id_ = ids.back();
  ids_known_ = true;
  const bool started = starts_.count(first_id_);
  const bool ended = ends_.count(last_id_);
  starts_.clear();
  ends_.clear();
  if (started) {starts_.insert(first_id_);}
  if (ended) {ends_.insert(last_id_);}
  update_execution();
}

void MotionServicePrimitive::receive_event(const std::string & text, bool start)
{
  if (!active_) {return;}
  std::uint64_t id;
  const auto parsed = std::from_chars(text.data(), text.data() + text.size(), id);
  if (parsed.ec != std::errc{} || parsed.ptr != text.data() + text.size()) {return;}
  if (ids_known_ && id != (start ? first_id_ : last_id_)) {return;}

  auto & events = start ? starts_ : ends_;
  // Bound the early-event cache if an unrelated publisher floods these topics.
  if (!events.count(id) && events.size() >= 4096) {
    fail("Too many events before the service response; execution is unknown");
    return;
  }
  events.insert(id);
  if (ids_known_) {update_execution();}
}

void MotionServicePrimitive::update_execution()
{
  if (ends_.count(last_id_)) {
    // Start/end topics can be delivered in either order. The final end alone is
    // sufficient under the agreed sequential, successful-execution contract.
    record_completion();
    active_ = false;
    if (!faulted_) {
      status_ = Status::SUCCEEDED;
      feedback_ = "Completed at /motion_end ID " + std::to_string(last_id_);
    }
  } else if (!faulted_) {
    feedback_ = starts_.count(first_id_) ?
      "Started at /motion_start ID " + std::to_string(first_id_) :
      "Accepted; waiting for /motion_start ID " + std::to_string(first_id_);
  }
}

void MotionServicePrimitive::tick()
{
  if (!active_ || faulted_) {return;}
  const double elapsed = std::chrono::duration<double>(Clock::now() - requested_at_).count();
  if (!response_received_ && elapsed >= response_timeout_) {
    fail("Service response timed out; remote execution is unknown, awaiting response/end");
  } else if (elapsed >= execution_timeout_) {
    fail("Final motion_end timed out; remote execution is unknown, awaiting end");
  }
}

void MotionServicePrimitive::cancel()
{
  if (active_) {
    fail("Remote cancellation is not provided; awaiting final motion_end (robot not stopped)");
  }
}

void MotionServicePrimitive::fail(const std::string & reason)
{
  faulted_ = true;
  status_ = Status::FAILED;
  feedback_ = reason;
  // active_ stays true if the server might still be executing.
}

void MotionServicePrimitive::reset()
{
  if (active_) {throw std::logic_error("Cannot reset before remote execution ends");}
  status_ = Status::IDLE;
  feedback_.clear();
  faulted_ = response_received_ = ids_known_ = false;
  starts_.clear();
  ends_.clear();
}
}  // namespace ur10_hardware_primitives::services
