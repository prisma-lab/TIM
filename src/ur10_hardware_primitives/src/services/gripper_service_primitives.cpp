#include "ur10_hardware_primitives/services/motion_service_primitive.hpp"

#include <inverse_msgs/srv/enqueue_trigger.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <map>

namespace ur10_hardware_primitives::services
{
using EnqueueTrigger = inverse_msgs::srv::EnqueueTrigger;

namespace
{
// Completion of an open/close request, not a measurement or object detection.
// Sharing this value lets a completed place invalidate the earlier pick goal.
enum class GripperPosition {UNKNOWN, OPEN, CLOSED};

std::shared_ptr<GripperPosition> gripper_position_for(rclcpp::Node & node)
{
  static std::map<rclcpp::Node *, std::weak_ptr<GripperPosition>> managers;
  auto position = managers[&node].lock();
  if (!position) {
    position = std::make_shared<GripperPosition>(GripperPosition::UNKNOWN);
    managers[&node] = position;
  }
  return position;
}
}  // namespace

class PickServicePrimitive : public MotionServicePrimitive
{
public:
  std::vector<primitive_manager::Observation> observe() const override
  {
    return {{"gripper.closed", *position_ == GripperPosition::CLOSED}};
  }

private:
  void configure() override
  {
    position_ = gripper_position_for(*node_);
    const auto endpoint = parameter<std::string>(*node_, "pick.service", "/pick");
    client_ = node_->create_client<EnqueueTrigger>(endpoint);
  }

  void dispatch(const std::vector<std::string> & args) override
  {
    if (!args.empty()) {throw std::invalid_argument("pick takes no arguments");}
    auto request = std::make_shared<EnqueueTrigger::Request>();
    send_request<EnqueueTrigger>(client_, request);
    // Until this request completes, an interrupted gripper motion must not
    // restore the previous command's open/closed completion fact.
    *position_ = GripperPosition::UNKNOWN;
  }

  void record_completion() override {*position_ = GripperPosition::CLOSED;}

  std::shared_ptr<GripperPosition> position_;
  rclcpp::Client<EnqueueTrigger>::SharedPtr client_;
};

class PlaceServicePrimitive : public MotionServicePrimitive
{
public:
  std::vector<primitive_manager::Observation> observe() const override
  {
    return {{"gripper.open", *position_ == GripperPosition::OPEN}};
  }

private:
  void configure() override
  {
    position_ = gripper_position_for(*node_);
    const auto endpoint = parameter<std::string>(*node_, "place.service", "/place");
    client_ = node_->create_client<EnqueueTrigger>(endpoint);
  }

  void dispatch(const std::vector<std::string> & args) override
  {
    if (!args.empty()) {throw std::invalid_argument("place takes no arguments");}
    auto request = std::make_shared<EnqueueTrigger::Request>();
    send_request<EnqueueTrigger>(client_, request);
    *position_ = GripperPosition::UNKNOWN;
  }

  void record_completion() override {*position_ = GripperPosition::OPEN;}

  std::shared_ptr<GripperPosition> position_;
  rclcpp::Client<EnqueueTrigger>::SharedPtr client_;
};
}  // namespace ur10_hardware_primitives::services

PLUGINLIB_EXPORT_CLASS(ur10_hardware_primitives::services::PickServicePrimitive,
  primitive_manager::PrimitiveBase)
PLUGINLIB_EXPORT_CLASS(ur10_hardware_primitives::services::PlaceServicePrimitive,
  primitive_manager::PrimitiveBase)
