/* 
 * Author: RC
 *
 * First created: 9-feb-2024 (SEED 7.0)
 * 
 */

#include "franka_behaviors.h"

using namespace seed; //this is not needed to compile, but most IDEs require it


/* 
*  *******************************************************************************
*                                   FRANKA BEHAVIOR
*  *******************************************************************************
*/

FrankaBehavior::FrankaBehavior(){
    //initialize buffer
    tf_buffer = std::make_unique<tf2_ros::Buffer>(nh->get_clock());
    //initialize listener
    tf_listener = std::make_shared<tf2_ros::TransformListener>(*tf_buffer);
}



/* 
*  *******************************************************************************
*                                   FRANKA MANAGER
*  *******************************************************************************
*/


// REGISTER this behavior into the Behavior-based System (BBS).
//  This is a 2 steps process...

// 1. PROVIDE a unique name to this behavior
//  the behavior will be identified by this name
std::string FrankaManagerBehavior::behavior_name = "FrankaManager";

// 2. ADD this behavior (identified by the behavior_name) to the BBS for possible activation
bool FrankaManagerBehavior::registered = BehaviorBasedSystem::add(behavior_name,&FrankaManagerBehavior::create);

// ...now your behavior has been registered, you may recall it by loading
//  a predicate in the form "<behavior_name>(<Arg1>, <Arg2>, ..., <ArgN>)" to the WM



// DEFINE the code of this behavior.
//  This is a 6 steps process...

// 1. DEFINE the constructor for this beahvior, it will be executed when this behavior is recalled.
//
//      instance: is the string containing the predicate that has been used to invoke ths behavior.
//                NOTE: if no paramaters (i.e., further args) are given then instance == behavior_name
FrankaManagerBehavior::FrankaManagerBehavior(std::string instance){
    // please set the instance before anything else
    setInstance(instance);

    robot_name = arg(1);

    current_action = "idle";

    // Create clients for services
    client_skill = nh->create_client<inverse_msgs::srv::ExecuteSkill>("TODO");
    client_p2p = nh->create_client<inverse_msgs::srv::PointToPointMotion>("TODO");
    client_stop = nh->create_client<std_srvs::srv::Trigger>("TODO");

    //TODO: initialize communication with robot

    // write CUSTOM construction code here...
    std::cout<<arg(0)<<": connection OPEN"<<std::endl;
}

// 2. DEFINE the create function.
//  this function MUST return the reference to the currente behavior.
Behavior *FrankaManagerBehavior::create(std::string instance){
    return new FrankaManagerBehavior(instance);
}


// now let's implement the CUSTOM core of our behavior (virtual functions).

// 3. DEFINE the start function.
//  this function executed only once at the beginning of the execution
void FrankaManagerBehavior::start(){
    // write CUSTOM code here...

}

// 4. DEFINE the perceptualSchema.
//  this function is executed at the specified frequency.
//      If TRUE is returned, the motorSchema may be executed
bool FrankaManagerBehavior::perceptualSchema(){
    // write CUSTOM code here...
    //std::cout<<arg(0)<<": perceptualSchema() executed "<<std::endl;

    this->setRate(20);

    wm_lock();

    //NOTE: here we will try the self-competition among behaviors
    new_action = wmv_get<std::string>("franka_"+robot_name+".action");

    wm_unlock();

    return true;
}

// 5. DEFINE the motorSchema.
//  this function is executed at the specified frequency (+ delay of perceptualSchema), 
//  but only if: 
//      - the perceptualSchema returns true
//      - all releasers along the WM branch are true
//      - all goals along the WM branch are false
//  if these conditions are not satisfied, only perceptualSchema is executed.
void FrankaManagerBehavior::motorSchema(){
    // write CUSTOM code here...

    // if action to be executed is changed, get the data about the new action
    if(new_action != current_action && current_action != ""){
        std::cout<<arg(0)<<": starting "<<new_action<<std::endl;

        if(current_action!="idle"){
            std::cout<<arg(0)<<": stopping "<<current_action<<std::endl;
            //stop execution of current action

            std::cout<<arg(0)<<": "<<current_action<<" stopped!"<<std::endl;
        }

        //start execution of current action

        std::vector<std::string> v = instance2vector(current_action);

        if(v[0] == "frankaGo"){
            geometry_msgs::msg::PoseStamped p_init, p_fin;
            if(v.size()>3){
                p_init = franka_subscribe_tf(v[2]);
                p_fin = franka_subscribe_tf(v[3]);
            }
            else{
                p_init = franka_subscribe_tf("TODO_FRANKA_CURRENT_POSE");
                p_fin = franka_subscribe_tf(v[2]);
            }
            if(!franka_move_p2p(p_init, p_fin)){
                return;
            }
        }
        else if(v[0] == "frankaExec"){
            geometry_msgs::msg::PoseStamped p_init, p_fin;

            p_init = franka_subscribe_tf(v[3]);
            p_fin = franka_subscribe_tf(v[4]);

            if(!franka_execute_skill(v[2], p_init, p_fin)){
                return;
            }
        }
        else if(v[0] == "frankaInsert"){

        }
        else if(v[0] == "frankaOpen"){

        }
        else if(v[0] == "frankaClose"){

        }
        else {
            std::cout<<arg(0)<<": action "<<current_action<<" does not exists"<<std::endl;
        }


        current_action = new_action;

        std::cout<<arg(0)<<": "<<current_action<<" started!"<<std::endl;
    }
}

// 6. DEFINE the exit function.
//  this function executed only once at the end of the execution
void FrankaManagerBehavior::exit(){
    // write CUSTOM code here...

    std::cout<<arg(0)<<": connection CLOSED"<<std::endl;
}


// FUNCTIONS FOR UNITN SERVICES

bool FrankaManagerBehavior::franka_stop()
{
    while (!client_stop->wait_for_service(std::chrono::seconds(1))) {
        if (!rclcpp::ok()) {
            std::cout<<"waiting for stop service"<<std::endl;
            return false;
        }
        std::cout<<"service on"<<std::endl;
    }

    auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
    auto result_future = client_stop->async_send_request(request);

    if (rclcpp::spin_until_future_complete(nh->get_node_base_interface(), result_future) == rclcpp::FutureReturnCode::SUCCESS) {
        auto response = result_future.get();
        if (response->success) {
            std::cout<<"franka stopped: "<<response->message<<std::endl;
        }
        else {
            std::cout<<"unable to stop franka: "<<response->message<<std::endl;
        }
        return response->success;
    }
    else {
        std::cout<<"failed to call service"<<std::endl;
        return false;
    }
}

bool FrankaManagerBehavior::franka_execute_skill(std::string skill_name, geometry_msgs::msg::PoseStamped init_p, geometry_msgs::msg::PoseStamped fin_p){
    while (!client_skill->wait_for_service(std::chrono::seconds(1))) {
        if (!rclcpp::ok()) {
            std::cout<<"waiting for skill service "<<std::endl;
            return false;
        }
        std::cout<<"service on"<<std::endl;
    }

    auto request = std::make_shared<inverse_msgs::srv::ExecuteSkill::Request>();
    request->skill_name = skill_name;
    request->initial_pose = init_p;
    request->final_pose = fin_p;
    //request->max_vel = 0.05;
    //request->use_learned_initial_pose = true;
    //request->use_learned_final_pose = true;

    auto result_future = client_skill->async_send_request(request);

    if (rclcpp::spin_until_future_complete(nh->get_node_base_interface(), result_future) == rclcpp::FutureReturnCode::SUCCESS) {
        auto response = result_future.get();
        if (response->success) {
            std::cout<<"franka execute_skill accepted"<<std::endl;
        }
        else {
            std::cout<<"unable to start execute_skill franka"<<std::endl;
        }
        return response->success;
    }
    else {
        std::cout<<"failed to call service"<<std::endl;
        return false;
    }
}

bool FrankaManagerBehavior::franka_move_p2p(geometry_msgs::msg::PoseStamped init_p, geometry_msgs::msg::PoseStamped fin_p){
    while (!client_p2p->wait_for_service(std::chrono::seconds(1))) {
        if (!rclcpp::ok()) {
            std::cout<<"waiting for skill service "<<std::endl;
            return false;
        }
        std::cout<<"service on"<<std::endl;
    }

    auto request = std::make_shared<inverse_msgs::srv::PointToPointMotion::Request>();
    request->y0 = init_p;
    request->g = fin_p;
    //request->max_vel = 0.25;
    //request->plan_y0_motion = false;
    auto result_future = client_p2p->async_send_request(request);

    if (rclcpp::spin_until_future_complete(nh->get_node_base_interface(), result_future) == rclcpp::FutureReturnCode::SUCCESS) {
        auto response = result_future.get();
        if (response->success) {
            std::cout<<"franka p2p accepted"<<std::endl;
        }
        else {
            std::cout<<"unable to start p2p franka: "<<std::endl;
        }
        return response->success;
    }
    else {
        std::cout<<"failed to call service"<<std::endl;
        return false;
    }
}




/* 
*  *******************************************************************************
*                                   FRANKA GO
*  *******************************************************************************
*/


// REGISTER this behavior into the Behavior-based System (BBS).
//  This is a 2 steps process...

// 1. PROVIDE a unique name to this behavior
//  the behavior will be identified by this name
std::string FrankaGoBehavior::behavior_name = "frankaGo";

// 2. ADD this behavior (identified by the behavior_name) to the BBS for possible activation
bool FrankaGoBehavior::registered = BehaviorBasedSystem::add(behavior_name,&FrankaGoBehavior::create);

// ...now your behavior has been registered, you may recall it by loading
//  a predicate in the form "<behavior_name>(<Arg1>, <Arg2>, ..., <ArgN>)" to the WM




// DEFINE the code of this behavior.
//  This is a 6 steps process...

// 1. DEFINE the constructor for this beahvior, it will be executed when this behavior is recalled.
//
//      instance: is the string containing the predicate that has been used to invoke ths behavior.
//                NOTE: if no paramaters (i.e., further args) are given then instance == behavior_name
FrankaGoBehavior::FrankaGoBehavior(std::string instance){
    // please set the instance before anything else
    setInstance(instance);

    // write CUSTOM construction code here...

    robot_name = arg(1);
    
    //if second argument is empty
    if(arg(2,false) != ""){
        start_pos = arg(2);
        end_pos = arg(3);
    }
    else
        end_pos = arg(2);

}

// 2. DEFINE the create function.
//  this function MUST return the reference to the currente behavior.
Behavior *FrankaGoBehavior::create(std::string instance){
    return new FrankaGoBehavior(instance);
}


// now let's implement the CUSTOM core of our behavior (virtual functions).

// 3. DEFINE the start function.
//  this function executed only once at the beginning of the execution
void FrankaGoBehavior::start(){
    // write CUSTOM code here...
    //std::cout<<arg(0)<<": start() executed "<<std::endl;
    
    //if(end_pos == "rnd"){
    ////select a random free slot on the ground (gnd1, gnd2, gnd3)
    //    //  for now it is fixed to gnd1
    //    end_pos = "gnd1";
    //}

}

// 4. DEFINE the perceptualSchema.
//  this function is executed at the specified frequency.
//      If TRUE is returned, the motorSchema may be executed
bool FrankaGoBehavior::perceptualSchema(){
    // write CUSTOM code here...
    //std::cout<<arg(0)<<": perceptualSchema() executed "<<std::endl;

    this->setRate(10);

    wm_lock();
    wmv_compete<std::string>("frankaManager("+robot_name+")", "franka_"+robot_name, this->getInstance());
    wm_unlock();

    return true;
}

// 5. DEFINE the motorSchema.
//  this function is executed at the specified frequency (+ delay of perceptualSchema), 
//  but only if: 
//      - the perceptualSchema returns true
//      - all releasers along the WM branch are true
//      - all goals along the WM branch are false
//  if these conditions are not satisfied, only perceptualSchema is executed.
void FrankaGoBehavior::motorSchema(){
    // write CUSTOM code here...
    //std::cout<<arg(0)<<": motorSchema() executed "<<std::endl;
    wm_lock();
    // SELF-SOLVING CONTENTION
    if(wmv_solve_once<std::string>("franka_"+robot_name) == this->getInstance()){
        // if here, I'm in control of IIWA
        wmv_set<std::string>("franka_"+robot_name+".action", this->getInstance());
    }
    wm_unlock();
}

// 6. DEFINE the exit function.
//  this function executed only once at the end of the execution
void FrankaGoBehavior::exit(){
    // write CUSTOM code here...
    std::cout<<arg(0)<<": exit() executed "<<std::endl;

    wm_lock();
    wmv_withdraw<std::string>("franka_"+robot_name);
    wmv_set<bool>(this->getInstance() + ".done", false);
    wm_unlock();
}



/* 
*  *******************************************************************************
*                                   FRANKA EXEC
*  *******************************************************************************
*/


// REGISTER this behavior into the Behavior-based System (BBS).
//  This is a 2 steps process...

// 1. PROVIDE a unique name to this behavior
//  the behavior will be identified by this name
std::string FrankaExecBehavior::behavior_name = "frankaExec";

// 2. ADD this behavior (identified by the behavior_name) to the BBS for possible activation
bool FrankaExecBehavior::registered = BehaviorBasedSystem::add(behavior_name,&FrankaExecBehavior::create);

// ...now your behavior has been registered, you may recall it by loading
//  a predicate in the form "<behavior_name>(<Arg1>, <Arg2>, ..., <ArgN>)" to the WM




// DEFINE the code of this behavior.
//  This is a 6 steps process...

// 1. DEFINE the constructor for this beahvior, it will be executed when this behavior is recalled.
//
//      instance: is the string containing the predicate that has been used to invoke ths behavior.
//                NOTE: if no paramaters (i.e., further args) are given then instance == behavior_name
FrankaExecBehavior::FrankaExecBehavior(std::string instance){
    // please set the instance before anything else
    setInstance(instance);

    // write CUSTOM construction code here...
    //std::cout<<arg(0)<<": Constructor() executed "<<std::endl;

    robot_name = arg(1);
    motion_name = arg(2);

}

// 2. DEFINE the create function.
//  this function MUST return the reference to the currente behavior.
Behavior *FrankaExecBehavior::create(std::string instance){
    return new FrankaExecBehavior(instance);
}


// now let's implement the CUSTOM core of our behavior (virtual functions).

// 3. DEFINE the start function.
//  this function executed only once at the beginning of the execution
void FrankaExecBehavior::start(){
    // write CUSTOM code here...
    //std::cout<<arg(0)<<": start() executed "<<std::endl;

    //wm_lock();
    //if(have_motion)
    //    wmv_set<bool>(motion_name+".known", true);
    //else
    //    wmv_set<bool>(motion_name+".known", false);
    //wm_unlock();

}

// 4. DEFINE the perceptualSchema.
//  this function is executed at the specified frequency.
//      If TRUE is returned, the motorSchema may be executed
bool FrankaExecBehavior::perceptualSchema(){
    // write CUSTOM code here...
    //std::cout<<arg(0)<<": perceptualSchema() executed "<<std::endl;

    this->setRate(10);

    wm_lock();
    if(wmv_get<bool>(motion_name+".known")){
        wmv_compete<std::string>("frankaManager("+robot_name+")", "franka_"+robot_name, this->getInstance());
    }
    wm_unlock();

    return true;
}

// 5. DEFINE the motorSchema.
//  this function is executed at the specified frequency (+ delay of perceptualSchema), 
//  but only if: 
//      - the perceptualSchema returns true
//      - all releasers along the WM branch are true
//      - all goals along the WM branch are false
//  if these conditions are not satisfied, only perceptualSchema is executed.
void FrankaExecBehavior::motorSchema(){
    // write CUSTOM code here...
    //std::cout<<arg(0)<<": motorSchema() executed "<<std::endl;
    wm_lock();
    // SELF-SOLVING CONTENTION
    if(wmv_solve_once<std::string>("franka_"+robot_name) == this->getInstance()){
        // if here, I'm in control of IIWA
        wmv_set<std::string>("franka_"+robot_name+".action", this->getInstance());
    }
    wm_unlock();
}

// 6. DEFINE the exit function.
//  this function executed only once at the end of the execution
void FrankaExecBehavior::exit(){
    // write CUSTOM code here...
    wm_lock();
    wmv_withdraw<std::string>("franka_"+robot_name);
    wmv_set<bool>(this->getInstance() + ".done", false);
    wm_unlock();
}



/* 
*  *******************************************************************************
*                                   FRANKA INSERT
*  *******************************************************************************
*/


// REGISTER this behavior into the Behavior-based System (BBS).
//  This is a 2 steps process...

// 1. PROVIDE a unique name to this behavior
//  the behavior will be identified by this name
std::string FrankaInsertBehavior::behavior_name = "frankaInsert";

// 2. ADD this behavior (identified by the behavior_name) to the BBS for possible activation
bool FrankaInsertBehavior::registered = BehaviorBasedSystem::add(behavior_name,&FrankaInsertBehavior::create);

// ...now your behavior has been registered, you may recall it by loading
//  a predicate in the form "<behavior_name>(<Arg1>, <Arg2>, ..., <ArgN>)" to the WM




// DEFINE the code of this behavior.
//  This is a 6 steps process...

// 1. DEFINE the constructor for this beahvior, it will be executed when this behavior is recalled.
//
//      instance: is the string containing the predicate that has been used to invoke ths behavior.
//                NOTE: if no paramaters (i.e., further args) are given then instance == behavior_name
FrankaInsertBehavior::FrankaInsertBehavior(std::string instance){
    // please set the instance before anything else
    setInstance(instance);

    // write CUSTOM construction code here...
    //std::cout<<arg(0)<<": Constructor() executed "<<std::endl;

    robot_name = arg(1);

    //get object to insert
    object_id = arg(2);

    //get object in which to insert
    slot_id = arg(3);
}

// 2. DEFINE the create function.
//  this function MUST return the reference to the currente behavior.
Behavior *FrankaInsertBehavior::create(std::string instance){
    return new FrankaInsertBehavior(instance);
}


// now let's implement the CUSTOM core of our behavior (virtual functions).

// 3. DEFINE the start function.
//  this function executed only once at the beginning of the execution
void FrankaInsertBehavior::start(){
    // write CUSTOM code here...
    //std::cout<<arg(0)<<": start() executed "<<std::endl;

}

// 4. DEFINE the perceptualSchema.
//  this function is executed at the specified frequency.
//      If TRUE is returned, the motorSchema may be executed
bool FrankaInsertBehavior::perceptualSchema(){
    // write CUSTOM code here...
    //std::cout<<arg(0)<<": perceptualSchema() executed "<<std::endl;

    this->setRate(10);

    //cartesian_target = iiwa_subscribe_tf(slot_id);
    //object_pose = iiwa_subscribe_tf(object_id);

    wm_lock();
    wmv_compete<std::string>("frankaManager("+robot_name+")", "franka_"+robot_name, this->getInstance());
    wm_unlock();

    return true;
}

// 5. DEFINE the motorSchema.
//  this function is executed at the specified frequency (+ delay of perceptualSchema), 
//  but only if: 
//      - the perceptualSchema returns true
//      - all releasers along the WM branch are true
//      - all goals along the WM branch are false
//  if these conditions are not satisfied, only perceptualSchema is executed.
void FrankaInsertBehavior::motorSchema(){
    // write CUSTOM code here...
    //std::cout<<arg(0)<<": motorSchema() executed "<<std::endl;
    wm_lock();
    // SELF-SOLVING CONTENTION
    if(wmv_solve_once<std::string>("franka_"+robot_name) == this->getInstance()){
        // if here, I'm in control of IIWA
        wmv_set<std::string>("franka_"+robot_name+".action", this->getInstance());
    }
    wm_unlock();
}

// 6. DEFINE the exit function.
//  this function executed only once at the end of the execution
void FrankaInsertBehavior::exit(){
    // write CUSTOM code here...
    std::cout<<arg(0)<<": exit() executed "<<std::endl;

    wm_lock();
    wmv_withdraw<std::string>("franka_"+robot_name);
    wmv_set<bool>(this->getInstance() + ".done", false);
    wm_unlock();
}




/* 
*  *******************************************************************************
*                                   FRANKA OPEN
*  *******************************************************************************
*/


// REGISTER this behavior into the Behavior-based System (BBS).
//  This is a 2 steps process...

// 1. PROVIDE a unique name to this behavior
//  the behavior will be identified by this name
std::string FrankaOpenBehavior::behavior_name = "frankaOpen";

// 2. ADD this behavior (identified by the behavior_name) to the BBS for possible activation
bool FrankaOpenBehavior::registered = BehaviorBasedSystem::add(behavior_name,&FrankaOpenBehavior::create);

// ...now your behavior has been registered, you may recall it by loading
//  a predicate in the form "<behavior_name>(<Arg1>, <Arg2>, ..., <ArgN>)" to the WM




// DEFINE the code of this behavior.
//  This is a 6 steps process...

// 1. DEFINE the constructor for this beahvior, it will be executed when this behavior is recalled.
//
//      instance: is the string containing the predicate that has been used to invoke ths behavior.
//                NOTE: if no paramaters (i.e., further args) are given then instance == behavior_name
FrankaOpenBehavior::FrankaOpenBehavior(std::string instance){
    // please set the instance before anything else
    setInstance(instance);

    // write CUSTOM construction code here...
    //std::cout<<arg(0)<<": Constructor() executed "<<std::endl;

    robot_name = arg(1);

    //get object to insert
    object_id = arg(2);

    //get object in which to insert
    slot_id = arg(3);
}

// 2. DEFINE the create function.
//  this function MUST return the reference to the currente behavior.
Behavior *FrankaOpenBehavior::create(std::string instance){
    return new FrankaOpenBehavior(instance);
}


// now let's implement the CUSTOM core of our behavior (virtual functions).

// 3. DEFINE the start function.
//  this function executed only once at the beginning of the execution
void FrankaOpenBehavior::start(){
    // write CUSTOM code here...
    //std::cout<<arg(0)<<": start() executed "<<std::endl;

}

// 4. DEFINE the perceptualSchema.
//  this function is executed at the specified frequency.
//      If TRUE is returned, the motorSchema may be executed
bool FrankaOpenBehavior::perceptualSchema(){
    // write CUSTOM code here...
    //std::cout<<arg(0)<<": perceptualSchema() executed "<<std::endl;

    this->setRate(10);

    //cartesian_target = iiwa_subscribe_tf(slot_id);
    //object_pose = iiwa_subscribe_tf(object_id);

    wm_lock();
    wmv_compete<std::string>("frankaManager("+robot_name+")", "franka_"+robot_name, this->getInstance());
    wm_unlock();

    return true;
}

// 5. DEFINE the motorSchema.
//  this function is executed at the specified frequency (+ delay of perceptualSchema), 
//  but only if: 
//      - the perceptualSchema returns true
//      - all releasers along the WM branch are true
//      - all goals along the WM branch are false
//  if these conditions are not satisfied, only perceptualSchema is executed.
void FrankaOpenBehavior::motorSchema(){
    // write CUSTOM code here...
    //std::cout<<arg(0)<<": motorSchema() executed "<<std::endl;
    wm_lock();
    // SELF-SOLVING CONTENTION
    if(wmv_solve_once<std::string>("franka_"+robot_name) == this->getInstance()){
        // if here, I'm in control of IIWA
        wmv_set<std::string>("franka_"+robot_name+".action", this->getInstance());
    }
    wm_unlock();
}

// 6. DEFINE the exit function.
//  this function executed only once at the end of the execution
void FrankaOpenBehavior::exit(){
    // write CUSTOM code here...
    std::cout<<arg(0)<<": exit() executed "<<std::endl;

    wm_lock();
    wmv_withdraw<std::string>("franka_"+robot_name);
    wmv_set<bool>(this->getInstance() + ".done", false);
    wm_unlock();
}






/* 
*  *******************************************************************************
*                                   FRANKA CLOSE
*  *******************************************************************************
*/


// REGISTER this behavior into the Behavior-based System (BBS).
//  This is a 2 steps process...

// 1. PROVIDE a unique name to this behavior
//  the behavior will be identified by this name
std::string FrankaCloseBehavior::behavior_name = "frankaClose";

// 2. ADD this behavior (identified by the behavior_name) to the BBS for possible activation
bool FrankaCloseBehavior::registered = BehaviorBasedSystem::add(behavior_name,&FrankaCloseBehavior::create);

// ...now your behavior has been registered, you may recall it by loading
//  a predicate in the form "<behavior_name>(<Arg1>, <Arg2>, ..., <ArgN>)" to the WM




// DEFINE the code of this behavior.
//  This is a 6 steps process...

// 1. DEFINE the constructor for this beahvior, it will be executed when this behavior is recalled.
//
//      instance: is the string containing the predicate that has been used to invoke ths behavior.
//                NOTE: if no paramaters (i.e., further args) are given then instance == behavior_name
FrankaCloseBehavior::FrankaCloseBehavior(std::string instance){
    // please set the instance before anything else
    setInstance(instance);

    // write CUSTOM construction code here...
    //std::cout<<arg(0)<<": Constructor() executed "<<std::endl;

    robot_name = arg(1);

    //get object to insert
    object_id = arg(2);

    //get object in which to insert
    slot_id = arg(3);
}

// 2. DEFINE the create function.
//  this function MUST return the reference to the currente behavior.
Behavior *FrankaCloseBehavior::create(std::string instance){
    return new FrankaCloseBehavior(instance);
}


// now let's implement the CUSTOM core of our behavior (virtual functions).

// 3. DEFINE the start function.
//  this function executed only once at the beginning of the execution
void FrankaCloseBehavior::start(){
    // write CUSTOM code here...
    //std::cout<<arg(0)<<": start() executed "<<std::endl;

}

// 4. DEFINE the perceptualSchema.
//  this function is executed at the specified frequency.
//      If TRUE is returned, the motorSchema may be executed
bool FrankaCloseBehavior::perceptualSchema(){
    // write CUSTOM code here...
    //std::cout<<arg(0)<<": perceptualSchema() executed "<<std::endl;

    this->setRate(10);

    //cartesian_target = iiwa_subscribe_tf(slot_id);
    //object_pose = iiwa_subscribe_tf(object_id);

    wm_lock();
    wmv_compete<std::string>("frankaManager("+robot_name+")", "franka_"+robot_name, this->getInstance());
    wm_unlock();

    return true;
}

// 5. DEFINE the motorSchema.
//  this function is executed at the specified frequency (+ delay of perceptualSchema), 
//  but only if: 
//      - the perceptualSchema returns true
//      - all releasers along the WM branch are true
//      - all goals along the WM branch are false
//  if these conditions are not satisfied, only perceptualSchema is executed.
void FrankaCloseBehavior::motorSchema(){
    // write CUSTOM code here...
    //std::cout<<arg(0)<<": motorSchema() executed "<<std::endl;

    wm_lock();
    // SELF-SOLVING CONTENTION
    if(wmv_solve_once<std::string>("franka_"+robot_name) == this->getInstance()){
        // if here, I'm in control of IIWA
        wmv_set<std::string>("franka_"+robot_name+".action", this->getInstance());
    }
    wm_unlock();
}

// 6. DEFINE the exit function.
//  this function executed only once at the end of the execution
void FrankaCloseBehavior::exit(){
    // write CUSTOM code here...
    std::cout<<arg(0)<<": exit() executed "<<std::endl;

    wm_lock();
    wmv_withdraw<std::string>("franka_"+robot_name);
    wmv_set<bool>(this->getInstance() + ".done", false);
    wm_unlock();
}
