#include "ur10_hardware_primitives/services/motion_service_primitive.hpp"

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <inverse_msgs/msg/target_pose_array.hpp>
#include <inverse_msgs/srv/reach_position.hpp>
#include <pluginlib/class_list_macros.hpp>

#include <cmath>
#include <map>
#include <set>

namespace ur10_hardware_primitives::services
{
using Pose = geometry_msgs::msg::PoseStamped;
using TargetPoses = inverse_msgs::msg::TargetPoseArray;
using ReachPosition = inverse_msgs::srv::ReachPosition;

class MoveServicePrimitive : public MotionServicePrimitive
{
public:
  std::vector<primitive_manager::Observation> observe() const override
  {
    std::vector<primitive_manager::Observation> facts;
    for (const auto & location : known_locations_) {
      facts.push_back({"arm.at(" + location + ")", current_location_ == location});
    }
    return facts;
  }

private:
  void configure() override
  {
    max_velocity_ = parameter(*node_, "move.max_velocity", 0.05);
    pose_timeout_ = parameter(*node_, "move.pose_timeout", 2.0);
    for (double value : {max_velocity_, pose_timeout_}) {
      if (!std::isfinite(value) || value <= 0) {
        throw std::invalid_argument("Move velocity and pose timeout must be positive and finite");
      }
    }
    const auto topic = parameter<std::string>(*node_, "move.target_topic", "/target_poses");
    targets_sub_ = node_->create_subscription<TargetPoses>(topic, rclcpp::QoS(1).reliable(),
      [this](TargetPoses::ConstSharedPtr message) {receive_targets(*message);});
    const auto service = parameter<std::string>(*node_, "move.service", "/reach_position");
    client_ = node_->create_client<ReachPosition>(service);
  }

  void receive_targets(const TargetPoses & message)
  {
    // Each publication replaces the complete set. Never reuse an older set
    // after an invalid update, since it may refer to targets that were removed.
    targets_.clear();
    target_error_.clear();
    if (message.names.size() != message.poses.size()) {
      target_error_ = "TargetPoseArray names and poses must have equal lengths";
    } else {
      for (size_t i = 0; i < message.names.size(); ++i) {
        const auto & name = message.names[i];
        const bool valid_name = !name.empty() && name.front() >= 'a' && name.front() <= 'z' &&
          name.find_first_not_of("abcdefghijklmnopqrstuvwxyz0123456789_") == std::string::npos;
        if (!valid_name || !targets_.emplace(name, message.poses[i]).second) {
          target_error_ = "TargetPoseArray needs unique names using lowercase letters, digits and underscores";
          break;
        }
      }
    }
    if (!target_error_.empty()) {
      targets_.clear();
      RCLCPP_WARN(node_->get_logger(), "%s", target_error_.c_str());
      return;
    }
    // Remember old names too, so SEED can receive false for earlier locations.
    for (const auto & target : targets_) {known_locations_.insert(target.first);}
  }

  void validate_pose(const Pose & pose) const
  {
    if (pose.header.frame_id.empty()) {
      throw std::invalid_argument("Target pose needs a reference frame in header.frame_id");
    }
    const auto & p = pose.pose.position;
    const auto & q = pose.pose.orientation;
    for (double value : {p.x, p.y, p.z, q.x, q.y, q.z, q.w}) {
      if (!std::isfinite(value)) {throw std::invalid_argument("Pose contains a nonfinite value");}
    }
    if (std::abs(q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w - 1.0) > 0.01) {
      throw std::invalid_argument("Pose orientation must be a unit quaternion");
    }
    const rclcpp::Time stamp(pose.header.stamp);
    const double age = (node_->now() - stamp).seconds();
    if (stamp.nanoseconds() == 0 || age > pose_timeout_ || age < -0.1) {
      throw std::invalid_argument("Pose timestamp is missing, stale, or in the future");
    }
  }

  void dispatch(const std::vector<std::string> & args) override
  {
    if (args.size() != 1) {throw std::invalid_argument("Use move(Location)");}
    if (!target_error_.empty()) {throw std::runtime_error(target_error_);}
    const auto target = targets_.find(args[0]);
    if (target == targets_.end()) {throw std::runtime_error("No target pose named " + args[0]);}
    validate_pose(target->second);

    auto request = std::make_shared<ReachPosition::Request>();
    // The provider moves target_link. Keep the pose's reference frame and stamp
    // unchanged; updates to /target_poses cannot redirect this invocation.
    request->desired_pos = target->second;
    request->max_vel = max_velocity_;
    request->immediate_execution = false;
    destination_ = args[0];
    send_request<ReachPosition>(client_, request);
    current_location_.clear();
  }

  void record_completion() override {current_location_ = destination_;}

  double max_velocity_{0.05}, pose_timeout_{2.0};
  std::map<std::string, Pose> targets_;
  std::set<std::string> known_locations_;
  std::string destination_, current_location_, target_error_;
  rclcpp::Subscription<TargetPoses>::SharedPtr targets_sub_;
  rclcpp::Client<ReachPosition>::SharedPtr client_;
};
}  // namespace ur10_hardware_primitives::services

PLUGINLIB_EXPORT_CLASS(ur10_hardware_primitives::services::MoveServicePrimitive,
  primitive_manager::PrimitiveBase)
