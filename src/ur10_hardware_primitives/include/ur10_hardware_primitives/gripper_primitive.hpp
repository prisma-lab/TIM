#pragma once
#include <control_msgs/action/gripper_command.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <primitive_manager/primitive_base.hpp>
#include "ur10_hardware_primitives/gripper_state.hpp"

namespace ur10_hardware_primitives
{
class GripperPrimitive : public primitive_manager::PrimitiveBase
{
public:
  void initialize(rclcpp::Node & node, const std::string & name) override;
  bool execute(const std::vector<std::string> & args) override;
  void tick() override;
  void cancel() override;
  void reset() override;
  bool busy() const override {return active_;}
  std::vector<primitive_manager::Observation> observe() const override;
private:
  using Action = control_msgs::action::GripperCommand;
  using Handle = rclcpp_action::ClientGoalHandle<Action>;
  using Clock = std::chrono::steady_clock;
  bool at_target() const;
  void stop(primitive_manager::Status status, const std::string & message);
  void finish(primitive_manager::Status status, const std::string & message);
  void request_cancel();
  GripperState state_;
  rclcpp_action::Client<Action>::SharedPtr client_;
  Handle::SharedPtr handle_;
  double target_{0}, timeout_{15}, tolerance_{0.02};
  std::string fact_;
  bool require_object_{false}, active_{false}, pending_{false}, stopping_{false};
  primitive_manager::Status final_status_{primitive_manager::Status::CANCELLED};
  uint64_t sequence_{0}, generation_{0};
  Clock::time_point deadline_{};
};
}
