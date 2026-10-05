#include "ur10_hardware_primitives/services/motion_service_primitive.hpp"

#include <algorithm>
#include <charconv>
#include <cmath>
#include <stdexcept>

namespace ur10_hardware_primitives::services
{
using primitive_manager::Status;

namespace
{
std::shared_ptr<TaskState> task_state_for(rclcpp::Node & node)
{
  // Plugins initialize and run on the manager's single-threaded executor.
  static std::map<rclcpp::Node *, std::weak_ptr<TaskState>> managers;
  auto state = managers[&node].lock();
  if (!state) {
    state = std::make_shared<TaskState>();
    managers[&node] = state;
  }
  return state;
}

void validate_names(const std::vector<std::string> & names, const std::string & parameter_name)
{
  std::set<std::string> unique;
  for (const auto & name : names) {
    const bool valid = !name.empty() && name.front() >= 'a' && name.front() <= 'z' &&
      name.find_first_not_of("abcdefghijklmnopqrstuvwxyz0123456789_") == std::string::npos;
    if (!valid || !unique.insert(name).second) {
      throw std::invalid_argument(parameter_name + " must contain unique symbolic names");
    }
  }
  if (names.empty()) {throw std::invalid_argument(parameter_name + " must not be empty");}
}
}  // namespace

void MotionServicePrimitive::initialize(rclcpp::Node & node, const std::string & name)
{
  node_ = &node;
  name_ = name;
  task_ = task_state_for(node);
  locations_ = parameter(node, "services.locations", std::vector<std::string>{});
  objects_ = parameter(node, "services.objects", std::vector<std::string>{"workpiece"});
  validate_names(locations_, "services.locations");
  validate_names(objects_, "services.objects");
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
  starts_.clear();
  ends_.clear();
  active_ = true;
  response_received_ = ids_known_ = faulted_ = false;
  requested_at_ = Clock::now();
  status_ = Status::RUNNING;
  feedback_ = "Waiting for service response and motion IDs";
}

void MotionServicePrimitive::receive_response(bool accepted, const std::vector<int64_t> & ids)
{
  response_received_ = true;
  if (!accepted) {
    active_ = false;  // Contract: rejection schedules no motion.
    fail("Service rejected the request");
    return;
  }
  const std::set<int64_t> unique(ids.begin(), ids.end());
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
  int64_t id;
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
  // Preserve completed task effects across the manager's per-command resets.
}

void MotionServicePrimitive::require_location(const std::string & location) const
{
  if (std::find(locations_.begin(), locations_.end(), location) == locations_.end()) {
    throw std::invalid_argument("Unknown location: " + location);
  }
}

void MotionServicePrimitive::require_object(const std::string & object) const
{
  if (std::find(objects_.begin(), objects_.end(), object) == objects_.end()) {
    throw std::invalid_argument("Unknown object: " + object);
  }
}

std::vector<primitive_manager::Observation> MotionServicePrimitive::observe() const
{
  std::vector<primitive_manager::Observation> facts;
  for (const auto & location : locations_) {
    facts.push_back({"arm.at(" + location + ")", task_->arm_location == location});
  }
  for (const auto & object : objects_) {
    facts.push_back({"object.held(" + object + ")", task_->held_object == object});
    const auto placed = task_->placements.find(object);
    for (const auto & location : locations_) {
      facts.push_back({"object.placed(" + object + "," + location + ")",
        placed != task_->placements.end() && placed->second == location});
    }
  }
  return facts;
}
}  // namespace ur10_hardware_primitives::services
