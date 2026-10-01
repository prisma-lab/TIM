#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo_ros/node.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <map>
#include <mutex>
#include <stdexcept>

namespace ur10_sim_support
{
// Separate from DemoGrasp: object-specific services, one physical grasp slot.
class TwoObjectGrasp : public gazebo::WorldPlugin
{
  using Service = std_srvs::srv::SetBool;
  struct Object
  {
    std::string model, link;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr holding;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose;
    rclcpp::Service<Service>::SharedPtr service;
  };
public:
  void Load(gazebo::physics::WorldPtr world, sdf::ElementPtr sdf) override
  {
    world_ = world;
    node_ = gazebo_ros::Node::Get(sdf);
    robot_model_ = sdf->Get<std::string>("robot_model");
    robot_link_ = sdf->Get<std::string>("robot_link");
    tcp_ = sdf->Get<ignition::math::Vector3d>("tcp_offset");
    distance_ = sdf->Get<double>("attach_distance");
    const auto prefix = sdf->Get<std::string>("topic_prefix");
    if (!sdf->HasElement("object")) {throw std::invalid_argument("Configure grasp objects");}
    for (auto entry = sdf->GetElement("object"); entry; entry = entry->GetNextElement("object")) {
      const auto id = entry->Get<std::string>("id");
      if (id.empty() || objects_.count(id)) {throw std::invalid_argument("Invalid/duplicate object ID");}
      auto & object = objects_[id];
      object.model = entry->Get<std::string>("model");
      object.link = entry->Get<std::string>("link");
      const auto topic = prefix + "/" + id;
      object.holding = node_->create_publisher<std_msgs::msg::Bool>(topic + "/holding", rclcpp::QoS(1).transient_local());
      object.pose = node_->create_publisher<geometry_msgs::msg::PoseStamped>(topic + "/pose", 10);
      object.service = node_->create_service<Service>(topic + "/set_grasp",
        [this, id](std::shared_ptr<rmw_request_id_t> header, std::shared_ptr<Service::Request> request) {
          std::lock_guard<std::mutex> lock(mutex_);
          if (pending_) {
            Service::Response response;
            response.message = "Another grasp request is pending";
            objects_.at(id).service->send_response(*header, response);
            return;
          }
          pending_ = header; requested_id_ = id; requested_attach_ = request->data;
        });
    }
    update_ = gazebo::event::Events::ConnectWorldUpdateBegin([this](const gazebo::common::UpdateInfo &) {update();});
  }
private:
  void update()
  {
    std::shared_ptr<rmw_request_id_t> pending;
    std::string id;
    bool attach = false;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      pending.swap(pending_); id = requested_id_; attach = requested_attach_;
    }
    if (pending) {
      Service::Response response;
      const auto & object_config = objects_.at(id);
      auto robot = world_->ModelByName(robot_model_);
      auto model = world_->ModelByName(object_config.model);
      auto parent = robot ? robot->GetLink(robot_link_) : nullptr;
      auto child = model ? model->GetLink(object_config.link) : nullptr;
      if (!parent || !child) {response.message = "Robot or requested object is unavailable";}
      else if (attach && joint_ && held_id_ != id) {response.message = "Another object is already held";}
      else if (!attach && joint_ && held_id_ != id) {response.message = "Cannot release a different object";}
      else if ((attach && held_id_ == id) || (!attach && !joint_)) {
        response.success = true; response.message = "Already in requested state";
      } else if (attach) {
        const auto tcp = parent->WorldPose().Pos() + parent->WorldPose().Rot().RotateVector(tcp_);
        if (tcp.Distance(child->WorldPose().Pos()) > distance_) {response.message = "Requested object is too far from TCP";}
        else {
          child->SetCollideMode("none"); child->SetGravityMode(false);
          joint_ = world_->Physics()->CreateJoint("fixed", robot);
          joint_->Load(parent, child, ignition::math::Pose3d::Zero); joint_->Init();
          held_id_ = id;
          response.success = true; response.message = "Attached " + id;
        }
      } else {
        joint_->Detach(); joint_.reset(); held_id_.clear();
        child->SetCollideMode("all"); child->SetGravityMode(true);
        model->SetLinearVel(ignition::math::Vector3d::Zero);
        model->SetAngularVel(ignition::math::Vector3d::Zero);
        response.success = true; response.message = "Released " + id;
      }
      objects_.at(id).service->send_response(*pending, response);
    }
    const double time = world_->SimTime().Double();
    if (time >= last_publish_ && time - last_publish_ < 0.05) {return;}
    last_publish_ = time;
    for (const auto & entry : objects_) {
      const auto & object = entry.second;
      auto model = world_->ModelByName(object.model);
      // Absence is unknown, not a freshly observed empty gripper/object.
      if (!model) {continue;}
      std_msgs::msg::Bool held; held.data = joint_ && held_id_ == entry.first;
      object.holding->publish(held);
      const auto pose = model->WorldPose();
      geometry_msgs::msg::PoseStamped msg;
      msg.header.frame_id = "world";
      msg.header.stamp = rclcpp::Time(world_->SimTime().sec, world_->SimTime().nsec);
      msg.pose.position.x = pose.Pos().X(); msg.pose.position.y = pose.Pos().Y(); msg.pose.position.z = pose.Pos().Z();
      msg.pose.orientation.x = pose.Rot().X(); msg.pose.orientation.y = pose.Rot().Y();
      msg.pose.orientation.z = pose.Rot().Z(); msg.pose.orientation.w = pose.Rot().W();
      object.pose->publish(msg);
    }
  }
  gazebo::physics::WorldPtr world_;
  gazebo::physics::JointPtr joint_;
  gazebo::event::ConnectionPtr update_;
  gazebo_ros::Node::SharedPtr node_;
  std::map<std::string, Object> objects_;
  std::string robot_model_, robot_link_, held_id_, requested_id_;
  ignition::math::Vector3d tcp_;
  double distance_{0.04}, last_publish_{-1};
  std::mutex mutex_;
  std::shared_ptr<rmw_request_id_t> pending_;
  bool requested_attach_{false};
};
GZ_REGISTER_WORLD_PLUGIN(TwoObjectGrasp)
}  // namespace ur10_sim_support
