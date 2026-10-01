#pragma once

#include "ur10_primitives/gripper_primitive.hpp"
#include <primitive_manager/primitive_base.hpp>
#include <moveit_msgs/action/move_group.hpp>
#include <moveit_msgs/action/execute_trajectory.hpp>
#include <moveit_msgs/srv/get_cartesian_path.hpp>
#include <moveit_msgs/srv/apply_planning_scene.hpp>
#include <moveit_msgs/msg/attached_collision_object.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <cmath>
#include <optional>

#include <map>
namespace ur10_primitives
{
using primitive_manager::Status;
using Pose = geometry_msgs::msg::PoseStamped;
using Clock = std::chrono::steady_clock;
using Move = moveit_msgs::action::MoveGroup;
using MoveHandle = rclcpp_action::ClientGoalHandle<Move>;
using Scene = moveit_msgs::srv::ApplyPlanningScene;
using Grasp = std_srvs::srv::SetBool;
using Cartesian = moveit_msgs::srv::GetCartesianPath;
using Execute = moveit_msgs::action::ExecuteTrajectory;
using ExecuteHandle = rclcpp_action::ClientGoalHandle<Execute>;

// Shared ROS clients, feedback checks, and cancellation for the three skills.
class ParameterizedPrimitive : public primitive_manager::PrimitiveBase
{
protected:
  enum class Step {OPEN, ABOVE, DOWN, GRASP, ATTACH, ATTACH_SCENE, DETACH, DETACH_SCENE, RETREAT, VERIFY};
  virtual const char * operation() const = 0;
  virtual std::vector<Step> stages() const = 0;
public:
  void initialize(rclcpp::Node & node, const std::string & name) override;

  bool execute(const std::vector<std::string> & args) override;

  void tick() override;

  void cancel() override;
  bool busy() const override;
  void reset() override;
  std::vector<primitive_manager::Observation> observe() const override;

private:
  static double age(Clock::time_point point);
  static double distance(const Pose & a, const Pose & b);
  void validate(const Pose & pose) const;
  Pose world_target(const Pose & target) const;
  Pose current_tcp() const;
  bool at_approach(const Pose & target) const;
  void finish(Status status, const std::string & detail);
  void stop(Status final_status, const std::string & detail);
  void next();
  void start_step();
  void send_move(bool above, bool straight);
  void send_cartesian(const Pose & pose);
  void set_grasp(bool attach);
  void set_scene(bool attach);
  rclcpp::Node * node_{nullptr};
  std::string operation_, target_key_, frame_, group_, tool_;
  double tcp_offset_, approach_, target_timeout_, execution_timeout_, velocity_, position_tolerance_, orientation_tolerance_;
  bool active_{false}, stopping_{false};
  bool move_pending_{false}, service_pending_{false};
  Status stop_status_{Status::CANCELLED};
  uint64_t generation_{0};
  size_t index_{0};
  std::vector<Step> steps_;
  struct Target {std::optional<Pose> pose; Clock::time_point received{};};
  std::map<std::string, Target> targets_;
  struct Object
  {
    Pose pose;
    bool held{false};
    Clock::time_point pose_received{}, held_received{};
    std::vector<double> dimensions;
    std::shared_ptr<GripperPrimitive> grasp;
    rclcpp::Client<Grasp>::SharedPtr service;
  };
  std::map<std::string, Object> objects_;
  std::string object_key_;
  std::map<std::pair<std::string, std::string>, Pose> placements_;
  Pose snapshot_;
  Clock::time_point deadline_{};
  GripperPrimitive open_;
  GripperPrimitive * child_{nullptr};
  std::unique_ptr<tf2_ros::Buffer> buffer_;
  std::shared_ptr<tf2_ros::TransformListener> listener_;
  std::vector<rclcpp::Subscription<Pose>::SharedPtr> target_subs_;
  std::vector<rclcpp::Subscription<Pose>::SharedPtr> object_subs_;
  std::vector<rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr> held_subs_;
  rclcpp_action::Client<Move>::SharedPtr move_;
  MoveHandle::SharedPtr move_handle_;
  rclcpp::Client<Cartesian>::SharedPtr cartesian_;
  rclcpp_action::Client<Execute>::SharedPtr execute_;
  ExecuteHandle::SharedPtr execute_handle_;
  rclcpp::Client<Scene>::SharedPtr scene_service_;
};
}  // namespace ur10_primitives
