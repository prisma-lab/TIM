#include "ur10_hardware_primitives/manipulation_primitive.hpp"

namespace ur10_hardware_primitives
{
void ManipulationPrimitive::initialize(rclcpp::Node & node, const std::string & name)
{
  node_ = &node;
  operation_ = operation();
  const auto keys = operation_ == "move_a_b" ?
    setting(node, name + ".targets", std::vector<std::string>{"pick", "place"}) :
    std::vector<std::string>{operation_};
  if (keys.empty()) {throw std::invalid_argument("Configure at least one move target");}
  for (const auto & key : keys) {
    if (key.empty() || targets_.count(key)) {throw std::invalid_argument("Invalid/duplicate target name");}
    targets_.emplace(key, Target{});
    const auto parameter = operation_ == "move_a_b" ? name + ".target_topics." + key : name + ".target_topic";
    const auto topic = setting<std::string>(node, parameter, "/ur10/targets/" + key);
    target_subs_.push_back(node.create_subscription<Pose>(topic, 10, [this, key](Pose::ConstSharedPtr msg) {
      targets_.at(key) = {*msg, Clock::now()};
    }));
  }
  frame_ = setting<std::string>(node, "manipulation.frame", "base_link");
  group_ = setting<std::string>(node, "manipulation.group", "ur_manipulator");
  tool_ = setting<std::string>(node, "manipulation.tool_link", "grasp_tcp");
  approach_ = setting(node, "manipulation.approach_height", 0.1);
  target_timeout_ = setting(node, "manipulation.target_timeout", 2.0);
  execution_timeout_ = setting(node, "manipulation.execution_timeout", 180.0);
  velocity_ = setting(node, "manipulation.velocity_scale", 0.1);
  position_tolerance_ = setting(node, "manipulation.position_tolerance", 0.005);
  orientation_tolerance_ = setting(node, "manipulation.orientation_tolerance", 0.08);
  for (double value : {approach_, target_timeout_, execution_timeout_, velocity_, position_tolerance_, orientation_tolerance_}) {
    if (!std::isfinite(value) || value <= 0) {throw std::invalid_argument("Manipulation parameters must be positive and finite");}
  }
  if (velocity_ > 1.0) {throw std::invalid_argument("Velocity scale must be <= 1");}
  buffer_ = std::make_unique<tf2_ros::Buffer>(node.get_clock());
  listener_ = std::make_shared<tf2_ros::TransformListener>(*buffer_, &node, false);
  gripper_.initialize(node);
  if (operation_ == "move_a_b") {
    move_ = rclcpp_action::create_client<Move>(&node, setting<std::string>(node, "manipulation.move_action", "/move_action"));
    return;  // Travel monitors gripper health but does not actuate it.
  }
  open_.initialize(node, "manipulation.open");
  if (operation_ == "pick") {grasp_.initialize(node, "manipulation.grasp");}
  cartesian_ = node.create_client<Cartesian>(setting<std::string>(node, "manipulation.cartesian_service", "/compute_cartesian_path"));
  execute_ = rclcpp_action::create_client<Execute>(&node, setting<std::string>(node, "manipulation.execute_action", "/execute_trajectory"));
  scene_service_ = node.create_client<Scene>(setting<std::string>(node, "manipulation.scene_service", "/apply_planning_scene"));
  scene_query_ = node.create_client<SceneQuery>(setting<std::string>(node, "manipulation.scene_query", "/get_planning_scene"));
  object_id_ = setting<std::string>(node, "manipulation.object_id", "workpiece");
  object_dimensions_ = setting(node, "manipulation.object_dimensions", std::vector<double>{});
  touch_links_ = setting(node, "manipulation.touch_links", std::vector<std::string>{});
  auto xyz = setting(node, "manipulation.object_offset_xyz", std::vector<double>{0., 0., 0.});
  auto rpy = setting(node, "manipulation.object_offset_rpy", std::vector<double>{0., 0., 0.});
  if (object_id_.empty() || object_dimensions_.size() != 3 || xyz.size() != 3 || rpy.size() != 3 || touch_links_.empty()) {
    throw std::invalid_argument("Configure workpiece box dimensions, tool-relative offset, and gripper touch links");
  }
  for (double v : object_dimensions_) {if (!std::isfinite(v) || v <= 0) {throw std::invalid_argument("Invalid object dimensions");}}
  for (double v : xyz) {if (!std::isfinite(v)) {throw std::invalid_argument("Invalid object offset");}}
  for (double v : rpy) {if (!std::isfinite(v)) {throw std::invalid_argument("Invalid object rotation");}}
  object_offset_.position.x = xyz[0]; object_offset_.position.y = xyz[1]; object_offset_.position.z = xyz[2];
  tf2::Quaternion q; q.setRPY(rpy[0], rpy[1], rpy[2]); object_offset_.orientation = tf2::toMsg(q);

}

bool ManipulationPrimitive::execute(const std::vector<std::string> & args)
{
  if (active_) {throw std::logic_error("Cannot execute an active manipulation");}
  if (operation_ == "move_a_b") {
    if (args.size() != 1 || !targets_.count(args[0])) {
      finish(Status::FAILED, "Use move_a_b(target_name), with a configured target such as pick or place"); return false;
    }
    target_key_ = args[0];
  } else {
    if (!args.empty()) {finish(Status::FAILED, "Pick/place take no arguments; publish a target pose"); return false;}
    target_key_ = operation_;
  }
  const auto & target = targets_.at(target_key_);
  if (!target.pose || age(target.received) > target_timeout_) {finish(Status::FAILED, "No fresh target pose"); return false;}
  try {
    snapshot_ = world_target(*target.pose);
    // Read measured feedback before dispatch; point A is the current TCP pose.
    current_tcp();
  } catch (const std::exception & error) {finish(Status::FAILED, std::string("Invalid target or robot feedback: ") + error.what()); return false;}
  if (!gripper_.healthy()) {finish(Status::FAILED, "No healthy measured gripper feedback"); return false;}
  if (!gripper_.held() && !gripper_.empty()) {
    finish(Status::FAILED, "Gripper must be fully open, fully closed empty, or detecting a held object"); return false;
  }
  carrying_ = gripper_.held();
  if (operation_ == "move_a_b") {
    if (!move_->action_server_is_ready()) {finish(Status::FAILED, "MoveIt is not ready"); return false;}
  } else {
    if ((operation_ == "pick" && gripper_.held()) || (operation_ == "place" && !gripper_.held())) {
      finish(Status::FAILED, operation_ == "pick" ? "Object is already held" : "Place requires a held object"); return false;
    }
    if (!at_approach(snapshot_)) {
      finish(Status::FAILED, "Run move_a_b(" + operation_ + ") first: arm is not at the target approach pose"); return false;
    }
    if (!execute_->action_server_is_ready() || !cartesian_->service_is_ready() ||
      !scene_query_->service_is_ready() || !scene_service_->service_is_ready()) {
      finish(Status::FAILED, "MoveIt motion or planning scene services are not ready"); return false;
    }
  }
  active_ = true;
  stopping_ = false; needs_reconcile_ = reconciling_ = false;
  status_ = Status::RUNNING;
  feedback_ = "Target snapshotted from external pose source";
  placement_confirmed_ = false;
  ++generation_;
  steps_ = stages();
  index_ = 0;
  try {start_step();}
  catch (const std::exception & e) {stop(Status::FAILED, std::string("Transport/state error: ") + e.what());}
  return true;
}

void ManipulationPrimitive::tick()
{
  if (!active_) {return;}
  if (!stopping_ && !gripper_.healthy()) {stop(Status::FAILED, "Gripper feedback stale or faulted during manipulation");}
  if (!stopping_ && carrying_ && !gripper_.held() &&
      !(operation_ == "place" && (steps_[index_] == Step::OPEN || steps_[index_] == Step::DETACH_SCENE ||
        steps_[index_] == Step::RETREAT || steps_[index_] == Step::VERIFY))) {
    stop(Status::FAILED, "Object detection lost during manipulation");
  }
  if (!stopping_ && operation_ == "move_a_b" && !carrying_ && !gripper_.empty()) {
    stop(Status::FAILED, "Unexpected gripper state during empty-hand transfer");
  }
  if (child_) {
    child_->tick();
    if (child_->status() == Status::FAILED && !stopping_) {stop(Status::FAILED, child_->feedback());}
    if (!child_->busy()) {
      auto status = child_->status();
      const auto detail = child_->feedback();
      child_->reset(); child_ = nullptr;
      if (stopping_) {settle_stop();}
      else if (status == Status::SUCCEEDED) {
        if (steps_[index_] == Step::GRASP) {carrying_ = true;}
        next();
      }
      else {finish(Status::FAILED, detail);}
    }
  }
  if (!active_) {return;}
  if (!stopping_ && Clock::now() >= deadline_) {stop(Status::FAILED, "Manipulation stage timed out; awaiting pending operation");}
  if (stopping_) {settle_stop(); return;}
  if (steps_[index_] == Step::VERIFY && operation_ == "move_a_b") {
    if (at_approach(snapshot_)) {finish(Status::SUCCEEDED, "Arrival at " + target_key_ + " approach pose verified");}
    return;
  }
  if (steps_[index_] == Step::VERIFY && at_approach(snapshot_)) {
    if (operation_ == "pick" && gripper_.held()) {
      finish(Status::SUCCEEDED, "Gripper object detection retained after retreat");
    } else if (operation_ == "place" && gripper_.opened()) {
      placement_confirmed_ = true; placed_target_ = snapshot_;
      finish(Status::SUCCEEDED, "Release at target and TCP retreat verified (no object localization sensor)");
    }
  }
}

void ManipulationPrimitive::cancel()
{if (active_ && !stopping_) {stop(Status::CANCELLED, "Waiting for the current operation to end");}}

bool ManipulationPrimitive::busy() const
{return active_;}

void ManipulationPrimitive::reset()
{
  if (active_) {throw std::logic_error("Cannot reset active manipulation");}
  status_ = Status::IDLE; feedback_.clear(); stopping_ = false;
  move_handle_.reset(); execute_handle_.reset(); child_ = nullptr;
}

std::vector<primitive_manager::Observation> ManipulationPrimitive::observe() const
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
  if (operation_ == "pick") {return {{"object.held", gripper_.held()}};}
  bool target_matches = false;
  const auto & target = targets_.at("place");
  if (target.pose && age(target.received) < target_timeout_) {
    try {target_matches = distance(world_target(*target.pose), placed_target_) < position_tolerance_;}
    catch (const std::exception &) {}
  }
  return {{"object.placed", placement_confirmed_ && gripper_.opened() && target_matches}};
}

double ManipulationPrimitive::age(Clock::time_point point)
{return std::chrono::duration<double>(Clock::now() - point).count();}

double ManipulationPrimitive::distance(const Pose & a, const Pose & b)
{
  if (a.header.frame_id != b.header.frame_id) {return INFINITY;}
  return std::hypot(std::hypot(a.pose.position.x - b.pose.position.x, a.pose.position.y - b.pose.position.y), a.pose.position.z - b.pose.position.z);
}

void ManipulationPrimitive::validate(const Pose & pose) const
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

Pose ManipulationPrimitive::world_target(const Pose & target) const
{
  validate(target);
  return target.header.frame_id == frame_ ? target : buffer_->transform(target, frame_);
}

Pose ManipulationPrimitive::current_tcp() const
{
  const auto transform = buffer_->lookupTransform(frame_, tool_, tf2::TimePointZero);
  const double seconds = (node_->now() - rclcpp::Time(transform.header.stamp)).seconds();
  if (seconds > 2.0 || seconds < -0.1) {throw std::runtime_error("No fresh tool transform");}
  Pose pose; pose.header = transform.header;
  pose.pose.orientation = transform.transform.rotation;
  pose.pose.position.x = transform.transform.translation.x;
  pose.pose.position.y = transform.transform.translation.y;
  pose.pose.position.z = transform.transform.translation.z;
  return pose;
}

bool ManipulationPrimitive::at_approach(const Pose & target) const
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

void ManipulationPrimitive::finish(Status status, const std::string & detail)
{
  active_ = false; status_ = status; feedback_ = detail;
  move_pending_ = service_pending_ = false;
  move_handle_.reset();
  execute_handle_.reset();
}

void ManipulationPrimitive::stop(Status final_status, const std::string & detail)
{
  stopping_ = true; stop_status_ = final_status;
  status_ = final_status == Status::FAILED ? Status::FAILED : Status::CANCELLING;
  feedback_ = detail;
  if (child_) {child_->cancel();}
  try {
    if (move_handle_) {move_->async_cancel_goal(move_handle_);}
    if (execute_handle_) {execute_->async_cancel_goal(execute_handle_);}
  } catch (const std::exception & e) {status_ = stop_status_ = Status::FAILED;
    feedback_ = std::string("Cancellation transport uncertain; await result/recover: ") + e.what();}
  settle_stop();
}

void ManipulationPrimitive::settle_stop()
{
  if (!active_ || child_ || move_pending_ || service_pending_) {return;}
  if (needs_reconcile_) {
    if (!gripper_.healthy() ||
      (operation_ == "place" && !gripper_.opened() && !gripper_.held()) ||
      (operation_ == "pick" && !gripper_.empty() && !gripper_.held())) {
      status_ = stop_status_ = Status::FAILED;
      feedback_ = "Grasp state uncertain after interruption; retain execution slot until feedback recovers";
      return;
    }
    const bool attach = operation_ == "pick" && gripper_.held();
    const bool detach = operation_ == "place" && gripper_.opened();
    if (attach || detach) {
      reconciling_ = true;
      try {set_scene(attach);}
      catch (const std::exception & e) {status_ = stop_status_ = Status::FAILED; feedback_ = e.what();}
      return;
    }
    needs_reconcile_ = false;
  }
  finish(stop_status_, feedback_);
}

void ManipulationPrimitive::next()
{++index_; try {start_step();} catch (const std::exception & e) {stop(Status::FAILED, std::string("Transport/state error: ") + e.what());}}

void ManipulationPrimitive::start_step()
{
  deadline_ = Clock::now() + std::chrono::duration_cast<Clock::duration>(std::chrono::duration<double>(execution_timeout_));
  const auto step = steps_.at(index_);
  RCLCPP_INFO(node_->get_logger(), "%s stage %zu/%zu", operation_.c_str(), index_ + 1, steps_.size());
  if (step == Step::PREPARE_SCENE) {set_scene(false, true);}
  else if (step == Step::OPEN || step == Step::GRASP) {
    if (step == Step::GRASP || operation_ == "place") {needs_reconcile_ = true;}
    child_ = step == Step::OPEN ? &open_ : &grasp_;
    child_->reset(); child_->execute({});
  } else if (step == Step::ABOVE || step == Step::DOWN || step == Step::RETREAT) {
    send_move(step != Step::DOWN, step != Step::ABOVE);
  } else if (step == Step::ATTACH_SCENE || step == Step::DETACH_SCENE) {
    set_scene(step == Step::ATTACH_SCENE);
  }
}

void ManipulationPrimitive::send_move(bool above, bool straight)
{
  Pose pose = snapshot_;
  if (above) {pose.pose.position.z += approach_;}
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
  goal.request.workspace_parameters.min_corner.z = -2.0;
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
    if (stopping_) {settle_stop();}
    else if (result.code == rclcpp_action::ResultCode::SUCCEEDED && result.result && result.result->error_code.val == 1) {next();}
    else {finish(Status::FAILED, "MoveIt motion failed, code=" + std::to_string(result.result ? result.result->error_code.val : 0));}
  };
  move_pending_ = true;
  move_->async_send_goal(goal, options);
}

void ManipulationPrimitive::send_cartesian(const Pose & pose)
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
      if (stopping_) {settle_stop();}
      else if (response.code == rclcpp_action::ResultCode::SUCCEEDED && response.result && response.result->error_code.val == 1) {next();}
      else {finish(Status::FAILED, "Cartesian trajectory execution failed");}
    };
    move_pending_ = true;
    execute_->async_send_goal(goal, options);
  });
}

void ManipulationPrimitive::set_scene(bool attach, bool prepare)
{
  auto request = std::make_shared<SceneQuery::Request>();
  request->components.components = moveit_msgs::msg::PlanningSceneComponents::ALLOWED_COLLISION_MATRIX;
  service_pending_ = true;
  scene_query_->async_send_request(request, [this, attach, prepare](rclcpp::Client<SceneQuery>::SharedFuture result) {
    service_pending_ = false;
    if (stopping_ && !reconciling_) {settle_stop(); return;}
    try {apply_scene(attach, prepare, result.get()->scene.allowed_collision_matrix);}
    catch (const std::exception & e) {stop(Status::FAILED, e.what());}
  });
}

void ManipulationPrimitive::apply_scene(bool attach, bool prepare, const moveit_msgs::msg::AllowedCollisionMatrix & matrix)
{
  auto request = std::make_shared<Scene::Request>();
  request->scene.is_diff = true; request->scene.robot_state.is_diff = true;
  // Preserve the robot's existing collision policy. Only workpiece/finger contact is allowed.
  auto & acm = request->scene.allowed_collision_matrix;
  acm = matrix;
  auto names = touch_links_; names.push_back(object_id_);
  for (const auto & name : names) {
    if (std::find(acm.entry_names.begin(), acm.entry_names.end(), name) == acm.entry_names.end()) {
      acm.entry_names.push_back(name);
    }
  }
  acm.entry_values.resize(acm.entry_names.size());
  for (auto & row : acm.entry_values) {row.enabled.resize(acm.entry_names.size(), false);}
  auto index = [&acm](const std::string & name) {return std::find(acm.entry_names.begin(), acm.entry_names.end(), name) - acm.entry_names.begin();};
  for (const auto & link : touch_links_) {
    acm.entry_values[index(object_id_)].enabled[index(link)] = true;
    acm.entry_values[index(link)].enabled[index(object_id_)] = true;
  }
  moveit_msgs::msg::AttachedCollisionObject attached;
  attached.link_name = tool_; attached.object.id = object_id_;
  attached.object.header.frame_id = tool_;
  attached.object.operation = attach ? attached.object.ADD : attached.object.REMOVE;
  attached.touch_links = touch_links_;
  shape_msgs::msg::SolidPrimitive box; box.type = box.BOX; box.dimensions.assign(object_dimensions_.begin(), object_dimensions_.end());
  if (attach) {
    attached.object.primitives = {box}; attached.object.primitive_poses = {object_offset_};
    moveit_msgs::msg::CollisionObject remove;
    remove.id = object_id_; remove.operation = remove.REMOVE;
    request->scene.world.collision_objects = {remove};
  } else {
    // Detaching keeps the released body in the world collision model.
    const auto tcp = prepare ? snapshot_ : current_tcp();
    tf2::Transform parent, offset; tf2::fromMsg(tcp.pose, parent); tf2::fromMsg(object_offset_, offset);
    const auto pose = parent * offset;
    geometry_msgs::msg::Pose world_pose;
    world_pose.position.x = pose.getOrigin().x(); world_pose.position.y = pose.getOrigin().y(); world_pose.position.z = pose.getOrigin().z();
    world_pose.orientation = tf2::toMsg(pose.getRotation());
    moveit_msgs::msg::CollisionObject world;
    world.id = object_id_; world.header.frame_id = frame_; world.operation = world.ADD;
    world.primitives = {box}; world.primitive_poses = {world_pose};
    request->scene.world.collision_objects = {world};
  }
  if (!prepare) {request->scene.robot_state.attached_collision_objects = {attached};}
  service_pending_ = true;
  scene_service_->async_send_request(request, [this](rclcpp::Client<Scene>::SharedFuture result) {
    service_pending_ = false;
    if (!result.get()->success) {finish(Status::FAILED, "MoveIt could not update workpiece geometry");}
    else {
      needs_reconcile_ = reconciling_ = false;
      if (stopping_) {settle_stop();} else {next();}
    }
  });
}
}  // namespace ur10_hardware_primitives
