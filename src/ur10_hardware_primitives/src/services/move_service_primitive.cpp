#include "ur10_hardware_primitives/services/motion_service_primitive.hpp"

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <inverse_msgs/srv/reach_position.hpp>
#include <pluginlib/class_list_macros.hpp>

#include <cmath>
#include <set>
#include <stdexcept>

namespace ur10_hardware_primitives::services
{
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
    if (!std::isfinite(max_velocity_) || max_velocity_ <= 0.0) {
      throw std::invalid_argument("Move velocity must be positive and finite");
    }
    const auto service = parameter<std::string>(*node_, "move.service", "/reach_position");
    client_ = node_->create_client<ReachPosition>(service);
  }

  void dispatch(const std::vector<std::string> & args) override
  {
    if (args.size() != 1 || args.front().empty()) {
      throw std::invalid_argument("Use move(Frame), for example move(via(bus_bar))");
    }
    const auto & target_frame = args.front();

    // Request the origin of the named frame. The service provider resolves it.
    // Default construction leaves the timestamp and position at zero.
    geometry_msgs::msg::PoseStamped target_pose;
    target_pose.header.frame_id = target_frame;
    target_pose.pose.orientation.w = 1.0;

    auto request = std::make_shared<ReachPosition::Request>();
    request->desired_pos = target_pose;
    request->max_vel = max_velocity_;
    request->immediate_execution = false;

    destination_ = target_frame;
    known_locations_.insert(destination_);
    send_request<ReachPosition>(client_, request);
    current_location_.clear();
  }

  void record_completion() override {current_location_ = destination_;}

  double max_velocity_{0.05};
  std::set<std::string> known_locations_;
  std::string destination_;
  std::string current_location_;
  rclcpp::Client<ReachPosition>::SharedPtr client_;
};
}  // namespace ur10_hardware_primitives::services

PLUGINLIB_EXPORT_CLASS(ur10_hardware_primitives::services::MoveServicePrimitive,
  primitive_manager::PrimitiveBase)
