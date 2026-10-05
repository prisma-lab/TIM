#include "ur10_hardware_primitives/services/motion_service_primitive.hpp"

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <inverse_msgs/srv/point_to_point_motion.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <cmath>
#include <optional>

namespace ur10_hardware_primitives::services
{
using Pose = geometry_msgs::msg::PoseStamped;
using PointToPoint = inverse_msgs::srv::PointToPointMotion;

class MoveServicePrimitive : public MotionServicePrimitive
{
private:
  void configure() override
  {
    tcp_frame_ = parameter<std::string>(*node_, "services.tcp_frame", "");
    if (tcp_frame_.empty() || tcp_frame_ == "base_link") {
      throw std::invalid_argument("Set services.tcp_frame to the TCP used by the external poses");
    }
    max_velocity_ = parameter(*node_, "move_a_b.max_velocity", 0.05);
    pose_timeout_ = parameter(*node_, "move_a_b.pose_timeout", 2.0);
    for (double value : {max_velocity_, pose_timeout_}) {
      if (!std::isfinite(value) || value <= 0) {
        throw std::invalid_argument("Move velocity and pose timeout must be positive and finite");
      }
    }
    buffer_ = std::make_unique<tf2_ros::Buffer>(node_->get_clock());
    listener_ = std::make_shared<tf2_ros::TransformListener>(*buffer_, node_, false);
    for (const auto & location : locations_) {
      targets_[location] = std::nullopt;
      const auto topic = parameter<std::string>(*node_, "move_a_b.target_topics." + location,
        "/ur10/hardware/targets/" + location);
      subscriptions_.push_back(node_->create_subscription<Pose>(topic, 10,
        [this, location](Pose::ConstSharedPtr pose) {targets_.at(location) = *pose;}));
    }
    const auto service = parameter<std::string>(*node_, "move_a_b.service", "/point_to_point_motion");
    client_ = node_->create_client<PointToPoint>(service);
  }

  void validate_pose(const Pose & pose) const
  {
    if (pose.header.frame_id != "base_link") {
      throw std::invalid_argument("Motion poses must use base_link");
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

  Pose current_pose() const
  {
    const auto transform = buffer_->lookupTransform("base_link", tcp_frame_, tf2::TimePointZero);
    Pose pose;
    pose.header = transform.header;
    pose.pose.position.x = transform.transform.translation.x;
    pose.pose.position.y = transform.transform.translation.y;
    pose.pose.position.z = transform.transform.translation.z;
    pose.pose.orientation = transform.transform.rotation;
    validate_pose(pose);
    return pose;
  }

  void dispatch(const std::vector<std::string> & args) override
  {
    if (args.size() != 1) {throw std::invalid_argument("Use move_a_b(Location)");}
    require_location(args[0]);
    const auto & target = targets_.at(args[0]);
    if (!target) {throw std::runtime_error("No external target for " + args[0]);}
    validate_pose(*target);

    auto request = std::make_shared<PointToPoint::Request>();
    request->y0 = current_pose();
    request->g = *target;  // Snapshot: later publications cannot redirect this call.
    request->max_vel = max_velocity_;
    request->plan_y0_motion = false;
    destination_ = args[0];
    send_request<PointToPoint>(client_, request);
    task_->arm_location.clear();
  }

  void record_completion() override {task_->arm_location = destination_;}

  std::string tcp_frame_, destination_;
  double max_velocity_{0.05}, pose_timeout_{2.0};
  std::map<std::string, std::optional<Pose>> targets_;
  std::vector<rclcpp::Subscription<Pose>::SharedPtr> subscriptions_;
  std::unique_ptr<tf2_ros::Buffer> buffer_;
  std::shared_ptr<tf2_ros::TransformListener> listener_;
  rclcpp::Client<PointToPoint>::SharedPtr client_;
};
}  // namespace ur10_hardware_primitives::services

PLUGINLIB_EXPORT_CLASS(ur10_hardware_primitives::services::MoveServicePrimitive,
  primitive_manager::PrimitiveBase)
