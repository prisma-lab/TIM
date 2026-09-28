#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo_ros/node.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <mutex>

namespace ur10_sim_support
{
// Physics mutations occur only in the world-update callback. The ROS service
// response is deferred until that mutation has actually completed.
class DemoGrasp : public gazebo::WorldPlugin
{
  using Service = std_srvs::srv::SetBool;
public:
  void Load(gazebo::physics::WorldPtr world, sdf::ElementPtr sdf) override
  {
    world_ = world;
    node_ = gazebo_ros::Node::Get(sdf);
    robot_model_ = sdf->Get<std::string>("robot_model");
    robot_link_ = sdf->Get<std::string>("robot_link");
    object_model_ = sdf->Get<std::string>("object_model");
    object_link_ = sdf->Get<std::string>("object_link");
    tcp_ = sdf->Get<ignition::math::Vector3d>("tcp_offset");
    distance_ = sdf->Get<double>("attach_distance");
    held_pub_ = node_->create_publisher<std_msgs::msg::Bool>("/ur10/demo/holding", rclcpp::QoS(1).transient_local());
    pose_pub_ = node_->create_publisher<geometry_msgs::msg::PoseStamped>("/ur10/demo/object_pose", 10);
    service_ = node_->create_service<Service>("/ur10/demo/set_grasp",
      [this](std::shared_ptr<rmw_request_id_t> header, std::shared_ptr<Service::Request> request) {
        std::lock_guard<std::mutex> guard(mutex_);
        if (pending_) {
          Service::Response response;
          response.message = "Another grasp request is pending";
          service_->send_response(*header, response);
          return;
        }
        pending_ = header;
        requested_ = request->data;
      });
    update_ = gazebo::event::Events::ConnectWorldUpdateBegin([this](const gazebo::common::UpdateInfo &) {update();});
  }
private:
  void update()
  {
    auto robot = world_->ModelByName(robot_model_);
    auto object = world_->ModelByName(object_model_);
    std::shared_ptr<rmw_request_id_t> pending;
    bool requested = false;
    {
      std::lock_guard<std::mutex> guard(mutex_);
      pending.swap(pending_);
      requested = requested_;
    }
    if (pending) {
      Service::Response result;
      auto parent = robot ? robot->GetLink(robot_link_) : nullptr;
      auto child = object ? object->GetLink(object_link_) : nullptr;
      if (!parent || !child) {result.message = "Robot or object link is unavailable";}
      else if (requested == static_cast<bool>(joint_)) {result.success = true; result.message = "Already in requested state";}
      else if (requested) {
        const auto tcp = parent->WorldPose().Pos() + parent->WorldPose().Rot().RotateVector(tcp_);
        if (tcp.Distance(child->WorldPose().Pos()) > distance_) {
          result.message = "Object is too far from the grasp TCP";
        } else {
          child->SetCollideMode("none");
          child->SetGravityMode(false);
          joint_ = world_->Physics()->CreateJoint("fixed", robot);
          joint_->Load(parent, child, ignition::math::Pose3d::Zero);
          joint_->Init();
          result.success = true;
          result.message = "Simulation attachment created";
        }
      } else {
        joint_->Detach();
        joint_.reset();
        child->SetCollideMode("all");
        child->SetGravityMode(true);
        object->SetLinearVel(ignition::math::Vector3d::Zero);
        object->SetAngularVel(ignition::math::Vector3d::Zero);
        result.success = true;
        result.message = "Simulation attachment released";
      }
      service_->send_response(*pending, result);
    }
    const double time = world_->SimTime().Double();
    if (time >= last_publish_ && time - last_publish_ < 0.05) {return;}
    last_publish_ = time;
    std_msgs::msg::Bool held;
    held.data = static_cast<bool>(joint_);
    held_pub_->publish(held);
    if (object) {
      auto pose = object->WorldPose();
      geometry_msgs::msg::PoseStamped msg;
      msg.header.frame_id = "world";
      msg.header.stamp = rclcpp::Time(world_->SimTime().sec, world_->SimTime().nsec);
      msg.pose.position.x = pose.Pos().X(); msg.pose.position.y = pose.Pos().Y(); msg.pose.position.z = pose.Pos().Z();
      msg.pose.orientation.x = pose.Rot().X(); msg.pose.orientation.y = pose.Rot().Y();
      msg.pose.orientation.z = pose.Rot().Z(); msg.pose.orientation.w = pose.Rot().W();
      pose_pub_->publish(msg);
    }
  }
  gazebo::physics::WorldPtr world_;
  gazebo::physics::JointPtr joint_;
  gazebo::event::ConnectionPtr update_;
  gazebo_ros::Node::SharedPtr node_;
  rclcpp::Service<Service>::SharedPtr service_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr held_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
  std::string robot_model_, robot_link_, object_model_, object_link_;
  ignition::math::Vector3d tcp_;
  double distance_{0.04}, last_publish_{-1};
  std::mutex mutex_;
  std::shared_ptr<rmw_request_id_t> pending_;
  bool requested_{false};
};
GZ_REGISTER_WORLD_PLUGIN(DemoGrasp)
}
