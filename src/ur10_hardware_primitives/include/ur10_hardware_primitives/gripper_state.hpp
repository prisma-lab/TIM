#pragma once
#include <algorithm>
#include <cmath>
#include <map>
#include <control_msgs/msg/dynamic_joint_state.hpp>
#include <rclcpp/rclcpp.hpp>

namespace ur10_hardware_primitives
{
template<typename T> T setting(rclcpp::Node & node, const std::string & key, const T & value)
{
  if (!node.has_parameter(key)) {node.declare_parameter<T>(key, value);}
  return node.get_parameter(key).get_value<T>();
}

// The pinned driver patch invalidates these fields when its serial link fails.
// Fresh ROS messages alone cannot establish that USB feedback is fresh.
class GripperState
{
public:
  void initialize(rclcpp::Node & node)
  {
    node_ = &node;
    timeout_ = setting(node, "gripper.feedback_timeout", 0.5);
    if (!std::isfinite(timeout_) || timeout_ <= 0) {throw std::invalid_argument("Invalid feedback timeout");}
    joint_ = setting<std::string>(node, "gripper.joint_name", "finger_joint");
    auto topic = setting<std::string>(node, "gripper.state_topic", "/dynamic_joint_states");
    sub_ = node.create_subscription<control_msgs::msg::DynamicJointState>(topic, rclcpp::SensorDataQoS(),
      [this](control_msgs::msg::DynamicJointState::ConstSharedPtr msg) {
        valid_ = false;
        auto it = std::find(msg->joint_names.begin(), msg->joint_names.end(), joint_);
        if (it == msg->joint_names.end()) {return;}
        auto index = size_t(it - msg->joint_names.begin());
        if (index >= msg->interface_values.size()) {return;}
        const auto & values = msg->interface_values[index];
        if (values.interface_names.size() != values.values.size()) {return;}
        std::map<std::string, double> data;
        for (size_t i = 0; i < values.values.size(); ++i) {data[values.interface_names[i]] = values.values[i];}
        for (const auto & key : {"position", "object_status", "gripper_fault"}) {
          if (!data.count(key) || !std::isfinite(data[key])) {return;}
        }
        const double dt = (node_->now() - rclcpp::Time(msg->header.stamp)).seconds();
        if (rclcpp::Time(msg->header.stamp).nanoseconds() == 0 || dt < -0.1 || dt > timeout_) {return;}
        position = data["position"]; object = data["object_status"]; fault = data["gripper_fault"];
        valid_ = position >= 0 && position <= 0.71 && object >= 0 && object <= 3 &&
          std::floor(object) == object && fault == 0;
        received_ = std::chrono::steady_clock::now();
        stamp_ = rclcpp::Time(msg->header.stamp);
        ++sequence;
      });
  }
  bool healthy() const
  {
    const double dt = (node_->now() - stamp_).seconds();
    return valid_ && dt >= -0.1 && dt < timeout_ &&
      std::chrono::duration<double>(std::chrono::steady_clock::now() - received_).count() < timeout_;
  }
  bool held() const {return healthy() && object == 2;}
  bool empty() const {return healthy() && object == 3 && (position <= 0.02 || position >= 0.675);}
  bool opened() const {return healthy() && object == 3 && position <= 0.02;}
  double position{0}, object{0}, fault{255};
  uint64_t sequence{0};
private:
  rclcpp::Node * node_{};
  std::string joint_;
  double timeout_{0.5};
  bool valid_{false};
  rclcpp::Time stamp_{0, 0, RCL_ROS_TIME};
  std::chrono::steady_clock::time_point received_{};
  rclcpp::Subscription<control_msgs::msg::DynamicJointState>::SharedPtr sub_;
};
}  // namespace ur10_hardware_primitives
