#include "ur10_hardware_primitives/services/motion_service_primitive.hpp"

#include <pluginlib/class_list_macros.hpp>

// The service provider owns these definitions. Keep the move adapter buildable
// before they arrive, without inventing production gripper interfaces.
#if __has_include(<inverse_msgs/srv/pick.hpp>) && __has_include(<inverse_msgs/srv/place.hpp>)
#include <inverse_msgs/srv/pick.hpp>
#include <inverse_msgs/srv/place.hpp>
#define TIM_HAS_GRIPPER_SERVICES 1
#else
#define TIM_HAS_GRIPPER_SERVICES 0
#endif

namespace ur10_hardware_primitives::services
{
// Only object/location bookkeeping is shared. No descent, retreat, MoveIt, or
// direct gripper-controller commands belong in these adapters.
class GripperServicePrimitive : public MotionServicePrimitive
{
protected:
  void select_target(const std::vector<std::string> & args)
  {
    if (args.size() != 2) {throw std::invalid_argument("Use " + name_ + "(Object,Location)");}
    require_object(args[0]);
    require_location(args[1]);
    if (task_->arm_location != args[1]) {
      throw std::runtime_error("Complete move_a_b(" + args[1] + ") before " + name_);
    }
    object_ = args[0];
    location_ = args[1];
  }

  std::string object_, location_;
};

class PickServicePrimitive : public GripperServicePrimitive
{
private:
  void configure() override
  {
#if TIM_HAS_GRIPPER_SERVICES
    const auto endpoint = parameter<std::string>(*node_, "pick.service", "/pick");
    client_ = node_->create_client<inverse_msgs::srv::Pick>(endpoint);
#else
    throw std::runtime_error("Install inverse_msgs/srv/Pick and Place from the provider, then rebuild "
      "ur10_hardware_primitives; use enable_gripper:=false for move-only operation");
#endif
  }

  void dispatch(const std::vector<std::string> & args) override
  {
    select_target(args);
    if (!task_->held_object.empty()) {throw std::runtime_error("Pick requires an empty gripper");}
#if TIM_HAS_GRIPPER_SERVICES
    send_request<inverse_msgs::srv::Pick>(client_, std::make_shared<inverse_msgs::srv::Pick::Request>());
#endif
  }

  void record_completion() override
  {
    task_->held_object = object_;
    task_->placements.erase(object_);
  }

#if TIM_HAS_GRIPPER_SERVICES
  rclcpp::Client<inverse_msgs::srv::Pick>::SharedPtr client_;
#endif
};

class PlaceServicePrimitive : public GripperServicePrimitive
{
private:
  void configure() override
  {
#if TIM_HAS_GRIPPER_SERVICES
    const auto endpoint = parameter<std::string>(*node_, "place.service", "/place");
    client_ = node_->create_client<inverse_msgs::srv::Place>(endpoint);
#else
    throw std::runtime_error("Install inverse_msgs/srv/Pick and Place from the provider, then rebuild "
      "ur10_hardware_primitives; use enable_gripper:=false for move-only operation");
#endif
  }

  void dispatch(const std::vector<std::string> & args) override
  {
    select_target(args);
    if (task_->held_object != object_) {
      throw std::runtime_error("Place requires the requested object to be held");
    }
#if TIM_HAS_GRIPPER_SERVICES
    send_request<inverse_msgs::srv::Place>(client_, std::make_shared<inverse_msgs::srv::Place::Request>());
#endif
  }

  void record_completion() override
  {
    task_->held_object.clear();
    task_->placements[object_] = location_;
  }

#if TIM_HAS_GRIPPER_SERVICES
  rclcpp::Client<inverse_msgs::srv::Place>::SharedPtr client_;
#endif
};
}  // namespace ur10_hardware_primitives::services

PLUGINLIB_EXPORT_CLASS(ur10_hardware_primitives::services::PickServicePrimitive,
  primitive_manager::PrimitiveBase)
PLUGINLIB_EXPORT_CLASS(ur10_hardware_primitives::services::PlaceServicePrimitive,
  primitive_manager::PrimitiveBase)
