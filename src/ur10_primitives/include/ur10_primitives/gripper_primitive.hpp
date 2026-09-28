#pragma once
#include <chrono>
#include <cstdint>
#include <memory>
#include <string>
#include <control_msgs/action/follow_joint_trajectory.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <primitive_manager/primitive_base.hpp>

namespace ur10_primitives
{
class GripperPrimitive : public primitive_manager::PrimitiveBase
{
public:
  void initialize(rclcpp::Node & node, const std::string & name) override;
  bool execute(const std::vector<std::string> & args) override;
  void tick() override;
  void cancel() override;
  void reset() override;
  bool busy() const override {return phase_ != Phase::IDLE;}
  std::vector<primitive_manager::Observation> observe() const override;
private:
  using Action = control_msgs::action::FollowJointTrajectory;
  using Handle = rclcpp_action::ClientGoalHandle<Action>;
  using Clock = std::chrono::steady_clock;
  enum class Phase {IDLE, WAITING, SENDING, RUNNING, VERIFYING};
  void send_goal();
  void request_cancel();
  void finish(primitive_manager::Status status, const std::string & feedback);
  bool at_target() const;
  bool fresh() const;
  void deadline_after(double seconds);
  rclcpp_action::Client<Action>::SharedPtr client_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_sub_;
  Handle::SharedPtr handle_;
  std::string joint_name_, state_fact_;
  double target_{0.0}, position_{0.0}, tolerance_{0.02};
  double motion_duration_{2.0}, server_timeout_{10.0}, execution_timeout_{20.0}, feedback_timeout_{2.0};
  bool have_position_{false}, cancel_requested_{false}, timed_out_{false}, cancel_sent_{false};
  Phase phase_{Phase::IDLE};
  Clock::time_point joint_received_{}, deadline_{};
  uint64_t generation_{0}, joint_sequence_{0}, verification_sequence_{0};
};
}  // namespace ur10_primitives
