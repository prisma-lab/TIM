#include "ur10_primitives/parameterized_primitive.hpp"

namespace ur10_primitives
{
namespace
{
template<typename T> T setting(rclcpp::Node & node, const std::string & name, const T & value)
{
  if (!node.has_parameter(name)) {node.declare_parameter<T>(name, value);}
  return node.get_parameter(name).get_value<T>();
}
}  // namespace

void ParameterizedPrimitive::initialize(rclcpp::Node & node, const std::string & /* name */)
{
  node_ = &node;
  operation_ = operation();
  const auto keys = setting(node, "manipulation.targets", std::vector<std::string>{});
  if (keys.empty()) {throw std::invalid_argument("Configure at least one move target");}
  for (const auto & key : keys) {
    if (key.empty() || targets_.count(key)) {throw std::invalid_argument("Invalid/duplicate target name");}
    targets_.emplace(key, Target{});
    const auto parameter = "manipulation.target_topics." + key;
    const auto topic = setting<std::string>(node, parameter, "/ur10/two_objects/targets/" + key);
    target_subs_.push_back(node.create_subscription<Pose>(topic, 10, [this, key](Pose::ConstSharedPtr msg) {
      targets_.at(key) = {*msg, Clock::now()};
    }));
  }
  frame_ = setting<std::string>(node, "manipulation.frame", "world");
  group_ = setting<std::string>(node, "manipulation.group", "ur_manipulator");
  tool_ = setting<std::string>(node, "manipulation.tool_link", "tool0");
  tcp_offset_ = setting(node, "manipulation.tcp_offset", 0.15);
  approach_ = setting(node, "manipulation.approach_height", 0.1);
  target_timeout_ = setting(node, "manipulation.target_timeout", 2.0);
  execution_timeout_ = setting(node, "manipulation.execution_timeout", 90.0);
  velocity_ = setting(node, "manipulation.velocity_scale", 0.25);
  position_tolerance_ = setting(node, "manipulation.position_tolerance", 0.02);
  orientation_tolerance_ = setting(node, "manipulation.orientation_tolerance", 0.08);
  for (double value : {tcp_offset_, approach_, target_timeout_, execution_timeout_, velocity_, position_tolerance_, orientation_tolerance_}) {
    if (!std::isfinite(value) || value <= 0) {throw std::invalid_argument("Manipulation parameters must be positive and finite");}
  }
  if (velocity_ > 1.0) {throw std::invalid_argument("Velocity scale must be <= 1");}
  buffer_ = std::make_unique<tf2_ros::Buffer>(node.get_clock());
  listener_ = std::make_shared<tf2_ros::TransformListener>(*buffer_, &node, false);
  // Every plugin watches every object, so identities cannot be confused.
  const auto ids = setting(node, "manipulation.objects", std::vector<std::string>{});
  if (ids.empty()) {throw std::invalid_argument("Configure manipulation.objects");}
  for (const auto & id : ids) {
    if (id.empty() || objects_.count(id)) {throw std::invalid_argument("Invalid/duplicate object ID");}
    auto & object = objects_[id];
    const auto prefix = setting<std::string>(node, "objects." + id + ".topic_prefix", "/ur10/two_objects/objects/" + id);
    object.dimensions = setting(node, "objects." + id + ".dimensions", std::vector<double>{});
    if (object.dimensions.size() != 3) {throw std::invalid_argument("Object dimensions need three values");}
    for (double value : object.dimensions) {
      if (!std::isfinite(value) || value <= 0) {throw std::invalid_argument("Invalid object dimensions");}
    }
    held_subs_.push_back(node.create_subscription<std_msgs::msg::Bool>(prefix + "/holding", rclcpp::QoS(1).transient_local(),
      [this, id](std_msgs::msg::Bool::ConstSharedPtr msg) {
        auto & object = objects_.at(id);
        object.held = msg->data; object.held_received = Clock::now();
        if (object.held) {
          for (auto it = placements_.begin(); it != placements_.end();) {
            if (it->first.first == id) {it = placements_.erase(it);} else {++it;}
          }
        }
      }));
    object_subs_.push_back(node.create_subscription<Pose>(prefix + "/pose", 10,
      [this, id](Pose::ConstSharedPtr msg) {
        try {
          auto transformed = world_target(*msg);
          auto & object = objects_.at(id);
          object.pose = transformed; object.pose_received = Clock::now();
        } catch (const std::exception &) {objects_.at(id).pose_received = Clock::time_point{};}
      }));
    object.service = node.create_client<Grasp>(prefix + "/set_grasp");
    if (operation_ == "pick") {
      object.grasp = std::make_shared<GripperPrimitive>();
      object.grasp->initialize(node, "objects." + id + ".grasp");
    }
  }
  if (operation_ == "move_a_b") {
    move_ = rclcpp_action::create_client<Move>(&node, "/move_action");

  } else {
    open_.initialize(node, "manipulation.open");
    cartesian_ = node.create_client<Cartesian>("/compute_cartesian_path");
    execute_ = rclcpp_action::create_client<Execute>(&node, "/execute_trajectory");
    scene_service_ = node.create_client<Scene>("/apply_planning_scene");
  }
}

bool ParameterizedPrimitive::execute(const std::vector<std::string> & args)
{
  if (active_) {throw std::logic_error("Cannot execute an active manipulation");}
  if (operation_ == "move_a_b") {
    if (args.size() != 1 || !targets_.count(args[0])) {
      finish(Status::FAILED, "Expected move_a_b(configured_location)"); return false;
    }
    target_key_ = args[0]; object_key_.clear();
  } else {
    if (args.size() != 2 || !objects_.count(args[0]) || !targets_.count(args[1])) {
      finish(Status::FAILED, "Expected " + operation_ + "(configured_object,configured_location)"); return false;
    }
    object_key_ = args[0]; target_key_ = args[1];
  }
  for (const auto & entry : objects_) {
    if (age(entry.second.held_received) > 2.0 || age(entry.second.pose_received) > 2.0) {
      finish(Status::FAILED, "Missing fresh observation for " + entry.first); return false;
    }
    if (operation_ == "pick" && entry.second.held) {
      finish(Status::FAILED, "Pick requires an empty gripper; held object is " + entry.first); return false;
    }
  }
  const auto & target = targets_.at(target_key_);
  if (!target.pose || age(target.received) > target_timeout_) {finish(Status::FAILED, "No fresh target pose"); return false;}
  try {
    snapshot_ = world_target(*target.pose);
    // Read measured feedback before dispatch; point A is the current TCP pose.
    current_tcp();
  } catch (const std::exception & error) {finish(Status::FAILED, std::string("Invalid target or robot feedback: ") + error.what()); return false;}
  if (operation_ == "move_a_b") {
    if (!move_->action_server_is_ready()) {finish(Status::FAILED, "MoveIt is not ready"); return false;}
  } else {
    const auto & object = objects_.at(object_key_);
    if (operation_ == "place" && !object.held) {
      finish(Status::FAILED, "Place requires the requested object to be held"); return false;
    }
    if (operation_ == "pick" && distance(object.pose, snapshot_) > 0.04) {
      finish(Status::FAILED, "Requested object is not at the selected pick location"); return false;
    }
    if (!at_approach(snapshot_)) {
      finish(Status::FAILED, "Run move_a_b(" + target_key_ + ") first: arm is not at the target approach pose"); return false;
    }
    if (!execute_->action_server_is_ready() || !cartesian_->service_is_ready() ||
      !objects_.at(object_key_).service->service_is_ready() || !scene_service_->service_is_ready()) {
      finish(Status::FAILED, "MoveIt or simulation grasp service is not ready"); return false;
    }
  }
  active_ = true;
  stopping_ = false;
  status_ = Status::RUNNING;
  feedback_ = "Target snapshotted from external pose source";
  if (operation_ == "pick") {
    for (auto it = placements_.begin(); it != placements_.end();) {
      if (it->first.first == object_key_) {it = placements_.erase(it);} else {++it;}
    }
  }
  ++generation_;
  steps_ = stages();
  index_ = 0;
  start_step();
  return true;
}

void ParameterizedPrimitive::tick()
{
  if (!active_) {return;}
  if (child_) {
    child_->tick();
    if (child_->status() == Status::FAILED && !stopping_) {stop(Status::FAILED, child_->feedback());}
    if (!child_->busy()) {
      auto status = child_->status();
      const auto detail = child_->feedback();
      child_->reset(); child_ = nullptr;
      if (stopping_) {finish(stop_status_, feedback_);}
      else if (status == Status::SUCCEEDED) {next();}
      else {finish(Status::FAILED, detail);}
    }
  }
  if (!active_) {return;}
  if (!stopping_ && Clock::now() >= deadline_) {stop(Status::FAILED, "Manipulation stage timed out; awaiting pending operation");}
  if (stopping_) {return;}
  if (steps_[index_] == Step::VERIFY && operation_ == "move_a_b") {
    if (at_approach(snapshot_)) {finish(Status::SUCCEEDED, "Arrival at " + target_key_ + " approach pose verified");}
    return;
  }
  if (steps_[index_] == Step::VERIFY) {
    const auto & object = objects_.at(object_key_);
    if (age(object.held_received) >= 2.0 || age(object.pose_received) >= 2.0) {return;}
    if (operation_ == "pick" && object.held && at_approach(snapshot_) &&
      object.pose.pose.position.z > snapshot_.pose.position.z + approach_ * 0.5) {
      finish(Status::SUCCEEDED, object_key_ + " attachment and lift verified");
    } else if (operation_ == "place" && !object.held && at_approach(snapshot_) && distance(object.pose, snapshot_) < 0.035) {
      placements_[{object_key_, target_key_}] = snapshot_;
      finish(Status::SUCCEEDED, object_key_ + " release at " + target_key_ + " verified");
    }
  }
}

void ParameterizedPrimitive::cancel()
{if (active_ && !stopping_) {stop(Status::CANCELLED, "Waiting for the current operation to end");}}

bool ParameterizedPrimitive::busy() const
{return active_;}

void ParameterizedPrimitive::reset()
{
  if (active_) {throw std::logic_error("Cannot reset active manipulation");}
  status_ = Status::IDLE; feedback_.clear(); stopping_ = false;
  move_handle_.reset(); execute_handle_.reset(); child_ = nullptr;
}

std::vector<primitive_manager::Observation> ParameterizedPrimitive::observe() const
{
  if (operation_ == "move_a_b") {
    std::vector<primitive_manager::Observation> facts;
    for (const auto & entry : targets_) {
      bool reached = false;
      if (entry.second.pose && age(entry.second.received) < target_timeout_) {
        try {reached = at_approach(world_target(*entry.second.pose));}
        catch (const std::exception &) {}  // Invalid/stale targets cannot be satisfied.
      }
      facts.push_back({"arm.at(" + entry.first + ")", reached});
    }
    return facts;
  }
  std::vector<primitive_manager::Observation> facts;
  for (const auto & entry : objects_) {
    const auto & object = entry.second;
    const bool fresh = age(object.held_received) < 2.0 && age(object.pose_received) < 2.0;
    if (operation_ == "pick") {
      facts.push_back({"object.held(" + entry.first + ")", fresh && object.held});
    } else {
      for (const auto & target : targets_) {
        const auto placed = placements_.find({entry.first, target.first});
        facts.push_back({"object.placed(" + entry.first + "," + target.first + ")",
          fresh && !object.held && placed != placements_.end() && distance(object.pose, placed->second) < 0.035});
      }
    }
  }
  return facts;
}

double ParameterizedPrimitive::age(Clock::time_point point)
{return std::chrono::duration<double>(Clock::now() - point).count();}

double ParameterizedPrimitive::distance(const Pose & a, const Pose & b)
{
  if (a.header.frame_id != b.header.frame_id) {return INFINITY;}
  return std::hypot(std::hypot(a.pose.position.x - b.pose.position.x, a.pose.position.y - b.pose.position.y), a.pose.position.z - b.pose.position.z);
}

void ParameterizedPrimitive::validate(const Pose & pose) const
{
  const auto & p = pose.pose.position; const auto & q = pose.pose.orientation;
  if (pose.header.frame_id.empty()) {throw std::invalid_argument("frame_id is required");}
  for (double value : {p.x,p.y,p.z,q.x,q.y,q.z,q.w}) {
    if (!std::isfinite(value)) {throw std::invalid_argument("Pose contains a nonfinite value");}
  }
  if (std::abs(q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w - 1.0) > 0.01) {throw std::invalid_argument("Orientation must be a unit quaternion");}
  const double age = (node_->now() - rclcpp::Time(pose.header.stamp)).seconds();
  if (age > target_timeout_ || age < -0.1 || rclcpp::Time(pose.header.stamp).nanoseconds() == 0) {
    throw std::invalid_argument("Target timestamp is stale, zero, or in the future");
  }
}

Pose ParameterizedPrimitive::world_target(const Pose & target) const
{
  validate(target);
  return target.header.frame_id == frame_ ? target : buffer_->transform(target, frame_);
}

Pose ParameterizedPrimitive::current_tcp() const
{
  const auto transform = buffer_->lookupTransform(frame_, tool_, tf2::TimePointZero);
  const double seconds = (node_->now() - rclcpp::Time(transform.header.stamp)).seconds();
  if (seconds > 2.0 || seconds < -0.1) {throw std::runtime_error("No fresh tool transform");}
  Pose pose; pose.header = transform.header;
  pose.pose.orientation = transform.transform.rotation;
  tf2::Quaternion q; tf2::fromMsg(pose.pose.orientation, q);
  const auto offset = tf2::quatRotate(q, tf2::Vector3(0, 0, tcp_offset_));
  pose.pose.position.x = transform.transform.translation.x + offset.x();
  pose.pose.position.y = transform.transform.translation.y + offset.y();
  pose.pose.position.z = transform.transform.translation.z + offset.z();
  return pose;
}

bool ParameterizedPrimitive::at_approach(const Pose & target) const
{
  try {
    auto above = target; above.pose.position.z += approach_;
    const auto measured = current_tcp();
    tf2::Quaternion actual, desired;
    tf2::fromMsg(measured.pose.orientation, actual); tf2::fromMsg(target.pose.orientation, desired);
    return distance(measured, above) <= position_tolerance_ &&
      actual.angleShortestPath(desired) <= orientation_tolerance_;
  } catch (const std::exception &) {return false;}
}

void ParameterizedPrimitive::finish(Status status, const std::string & detail)
{
  active_ = false; status_ = status; feedback_ = detail;
  move_pending_ = service_pending_ = false;
  move_handle_.reset();
  execute_handle_.reset();
}

void ParameterizedPrimitive::stop(Status final_status, const std::string & detail)
{
  stopping_ = true; stop_status_ = final_status;
  status_ = final_status == Status::FAILED ? Status::FAILED : Status::CANCELLING;
  feedback_ = detail;
  if (child_) {child_->cancel();}
  if (move_handle_) {move_->async_cancel_goal(move_handle_);}
  if (execute_handle_) {execute_->async_cancel_goal(execute_handle_);}
  if (!child_ && !move_pending_ && !service_pending_) {finish(final_status, detail);}
}

void ParameterizedPrimitive::next()
{++index_; start_step();}

void ParameterizedPrimitive::start_step()
{
  deadline_ = Clock::now() + std::chrono::duration_cast<Clock::duration>(std::chrono::duration<double>(execution_timeout_));
  const auto step = steps_.at(index_);
  RCLCPP_INFO(node_->get_logger(), "%s stage %zu/%zu", operation_.c_str(), index_ + 1, steps_.size());
  if (step == Step::OPEN || step == Step::GRASP) {
    child_ = step == Step::OPEN ? &open_ : objects_.at(object_key_).grasp.get();
    child_->reset(); child_->execute({});
  } else if (step == Step::ABOVE || step == Step::DOWN || step == Step::RETREAT) {
    send_move(step != Step::DOWN, step != Step::ABOVE);
  } else if (step == Step::ATTACH || step == Step::DETACH) {
    set_grasp(step == Step::ATTACH);
  } else if (step == Step::ATTACH_SCENE || step == Step::DETACH_SCENE) {
    set_scene(step == Step::ATTACH_SCENE);
  }
}

void ParameterizedPrimitive::send_move(bool above, bool straight)
{
  Pose pose = snapshot_;
  if (above) {pose.pose.position.z += approach_;}
  tf2::Quaternion q; tf2::fromMsg(pose.pose.orientation, q);
  auto offset = tf2::quatRotate(q, tf2::Vector3(0,0,tcp_offset_));
  pose.pose.position.x -= offset.x(); pose.pose.position.y -= offset.y(); pose.pose.position.z -= offset.z();
  if (straight) {send_cartesian(pose); return;}
  Move::Goal goal;
  goal.request.group_name = group_;
  goal.request.pipeline_id = "ompl";
  goal.request.num_planning_attempts = 5;
  goal.request.allowed_planning_time = 8.0;
  goal.request.max_velocity_scaling_factor = velocity_;
  goal.request.max_acceleration_scaling_factor = velocity_;
  goal.request.start_state.is_diff = true;
  goal.request.workspace_parameters.header.frame_id = frame_;
  goal.request.workspace_parameters.min_corner.x = goal.request.workspace_parameters.min_corner.y = -2.0;
  goal.request.workspace_parameters.min_corner.z = -0.1;
  goal.request.workspace_parameters.max_corner.x = goal.request.workspace_parameters.max_corner.y = goal.request.workspace_parameters.max_corner.z = 2.0;
  moveit_msgs::msg::Constraints constraints;
  moveit_msgs::msg::PositionConstraint position;
  position.header.frame_id = frame_; position.link_name = tool_; position.weight = 1.0;
  shape_msgs::msg::SolidPrimitive sphere; sphere.type = sphere.SPHERE; sphere.dimensions = {0.003};
  position.constraint_region.primitives = {sphere}; position.constraint_region.primitive_poses = {pose.pose};
  constraints.position_constraints = {position};
  moveit_msgs::msg::OrientationConstraint orientation;
  orientation.header.frame_id = frame_; orientation.link_name = tool_;
  orientation.orientation = pose.pose.orientation;
  orientation.absolute_x_axis_tolerance = orientation.absolute_y_axis_tolerance = orientation.absolute_z_axis_tolerance = 0.02;
  orientation.weight = 1.0;
  constraints.orientation_constraints = {orientation};
  goal.request.goal_constraints = {constraints};
  goal.planning_options.plan_only = false;
  goal.planning_options.planning_scene_diff.is_diff = true;
  goal.planning_options.planning_scene_diff.robot_state.is_diff = true;
  const auto generation = generation_;
  rclcpp_action::Client<Move>::SendGoalOptions options;
  options.goal_response_callback = [this, generation](MoveHandle::SharedPtr handle) {
    if (generation != generation_) {if (handle) {move_->async_cancel_goal(handle);} return;}
    if (!handle) {move_pending_ = false; finish(stopping_ ? stop_status_ : Status::FAILED, "MoveIt rejected the motion"); return;}
    move_handle_ = handle;
    if (stopping_) {move_->async_cancel_goal(handle);}
  };
  options.result_callback = [this, generation](const MoveHandle::WrappedResult & result) {
    if (generation != generation_) {return;}
    move_pending_ = false; move_handle_.reset();
    if (stopping_) {finish(stop_status_, feedback_);}
    else if (result.code == rclcpp_action::ResultCode::SUCCEEDED && result.result && result.result->error_code.val == 1) {next();}
    else {finish(Status::FAILED, "MoveIt motion failed, code=" + std::to_string(result.result ? result.result->error_code.val : 0));}
  };
  move_pending_ = true;
  move_->async_send_goal(goal, options);
}

void ParameterizedPrimitive::send_cartesian(const Pose & pose)
{
  auto request = std::make_shared<Cartesian::Request>();
  request->header.frame_id = frame_;
  request->start_state.is_diff = true;
  request->group_name = group_; request->link_name = tool_;
  request->waypoints = {pose.pose};
  request->max_step = 0.005;
  request->jump_threshold = 5.0;
  request->revolute_jump_threshold = 0.3;
  request->avoid_collisions = true;
  request->max_velocity_scaling_factor = velocity_;
  request->max_acceleration_scaling_factor = velocity_;
  service_pending_ = true;
  cartesian_->async_send_request(request, [this](rclcpp::Client<Cartesian>::SharedFuture future) {
    service_pending_ = false;
    if (stopping_) {finish(stop_status_, feedback_); return;}
    const auto result = future.get();
    if (result->error_code.val != 1 || !std::isfinite(result->fraction) ||
      result->fraction < 0.999 || result->solution.joint_trajectory.points.empty()) {
      finish(Status::FAILED, "No complete collision-free Cartesian path (fraction=" +
        std::to_string(result->fraction) + ")"); return;
    }
    Execute::Goal goal; goal.trajectory = result->solution;
    rclcpp_action::Client<Execute>::SendGoalOptions options;
    options.goal_response_callback = [this](ExecuteHandle::SharedPtr handle) {
      if (!handle) {move_pending_ = false; finish(stopping_ ? stop_status_ : Status::FAILED, "MoveIt rejected Cartesian execution"); return;}
      execute_handle_ = handle;
      if (stopping_) {execute_->async_cancel_goal(handle);}
    };
    options.result_callback = [this](const ExecuteHandle::WrappedResult & response) {
      move_pending_ = false; execute_handle_.reset();
      if (stopping_) {finish(stop_status_, feedback_);}
      else if (response.code == rclcpp_action::ResultCode::SUCCEEDED && response.result && response.result->error_code.val == 1) {next();}
      else {finish(Status::FAILED, "Cartesian trajectory execution failed");}
    };
    move_pending_ = true;
    execute_->async_send_goal(goal, options);
  });
}

void ParameterizedPrimitive::set_grasp(bool attach)
{
  auto request = std::make_shared<Grasp::Request>(); request->data = attach;
  service_pending_ = true;
  objects_.at(object_key_).service->async_send_request(request, [this, attach](rclcpp::Client<Grasp>::SharedFuture result) {
    service_pending_ = false;
    // A grasp mutation may complete after cancellation. Reconcile MoveIt's
    // attached-body model before releasing the manager's execution slot.
    if (stopping_ && result.get()->success) {set_scene(attach);}
    else if (stopping_) {finish(stop_status_, feedback_);}
    else if (result.get()->success) {next();}
    else {finish(Status::FAILED, result.get()->message);}
  });
}

void ParameterizedPrimitive::set_scene(bool attach)
{
  auto request = std::make_shared<Scene::Request>();
  request->scene.is_diff = true; request->scene.robot_state.is_diff = true;
  moveit_msgs::msg::AttachedCollisionObject object;
  object.link_name = tool_; object.object.id = object_key_;
  object.object.header.frame_id = tool_;
  object.object.operation = attach ? object.object.ADD : object.object.REMOVE;
  if (attach) {
    shape_msgs::msg::SolidPrimitive box; box.type = box.BOX; const auto & dimensions = objects_.at(object_key_).dimensions;
    box.dimensions.assign(dimensions.begin(), dimensions.end());
    geometry_msgs::msg::Pose pose; pose.orientation.w = 1.0; pose.position.z = tcp_offset_;
    object.object.primitives = {box}; object.object.primitive_poses = {pose};
    object.touch_links = {tool_, "wrist_3_link", "robotiq_85_base_link", "robotiq_85_left_finger_tip_link",
      "robotiq_85_right_finger_tip_link", "robotiq_85_left_finger_link", "robotiq_85_right_finger_link"};
  }
  request->scene.robot_state.attached_collision_objects = {object};
  // Remove the detached object's planning representation; the demo tracks its
  // physical position separately. Table and fixtures remain in the robot model.
  if (!attach) {
    moveit_msgs::msg::CollisionObject remove; remove.id = object_key_; remove.operation = remove.REMOVE;
    request->scene.world.collision_objects = {remove};
  }
  service_pending_ = true;
  scene_service_->async_send_request(request, [this](rclcpp::Client<Scene>::SharedFuture result) {
    service_pending_ = false;
    if (!result.get()->success) {finish(Status::FAILED, "MoveIt could not update the carried-object collision model");}
    else if (stopping_) {finish(stop_status_, feedback_);}
    else {next();}
  });
}

}  // namespace ur10_primitives
