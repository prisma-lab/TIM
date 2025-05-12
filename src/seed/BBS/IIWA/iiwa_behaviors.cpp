/* 
 * Author: RC
 *
 * First created: 9-feb-2024 (SEED 7.0)
 * 
 */

#include "iiwa_behaviors.h"

using namespace seed; //this is not needed to compile, but most IDEs require it


/* 
*  *******************************************************************************
*                                   IIWA MANAGER
*  *******************************************************************************
*/


// REGISTER this behavior into the Behavior-based System (BBS).
//  This is a 2 steps process...

// 1. PROVIDE a unique name to this behavior
//  the behavior will be identified by this name
std::string IIWAManagerBehavior::behavior_name = "iiwaManager";

// 2. ADD this behavior (identified by the behavior_name) to the BBS for possible activation
bool IIWAManagerBehavior::registered = BehaviorBasedSystem::add(behavior_name,&IIWAManagerBehavior::create);

// ...now your behavior has been registered, you may recall it by loading
//  a predicate in the form "<behavior_name>(<Arg1>, <Arg2>, ..., <ArgN>)" to the WM



// DEFINE the code of this behavior.
//  This is a 6 steps process...

// 1. DEFINE the constructor for this beahvior, it will be executed when this behavior is recalled.
//
//      instance: is the string containing the predicate that has been used to invoke ths behavior.
//                NOTE: if no paramaters (i.e., further args) are given then instance == behavior_name
IIWAManagerBehavior::IIWAManagerBehavior(std::string instance){
    // please set the instance before anything else
    setInstance(instance);

    // write CUSTOM construction code here...
    std::cout<<arg(0)<<": Constructor() executed "<<std::endl;
}

// 2. DEFINE the create function.
//  this function MUST return the reference to the currente behavior.
Behavior *IIWAManagerBehavior::create(std::string instance){
    return new IIWAManagerBehavior(instance);
}


// now let's implement the CUSTOM core of our behavior (virtual functions).

// 3. DEFINE the start function.
//  this function executed only once at the beginning of the execution
void IIWAManagerBehavior::start(){
    // write CUSTOM code here...
    std::cout<<arg(0)<<": Starting connection with IIWA..."<<std::endl;
    
    //open conncetion to IIWA
    iiwa_interface.open_connection(38095, 39095);

    current_state = iiwa_interface.getFullState();
    
    //set control mode to cartesian position
    iiwa_interface.setControlMode(IIWA_control_mode::CARTESIAN_POSITION);
    iiwa_interface.setCartesianPosition(current_state.cartesian_position);

    current_mode = "idle_cartesian_position";
    current_action = "idle";

    //init tf publisher
    tf_broadcaster = std::make_unique<tf2_ros::TransformBroadcaster>(nh);

    is_motion_continuous = false;

    std::cout<<arg(0)<<": ...connection OPEN, cartesian position control started."<<std::endl;
}

// 4. DEFINE the perceptualSchema.
//  this function is executed at the specified frequency.
//      If TRUE is returned, the motorSchema may be executed
bool IIWAManagerBehavior::perceptualSchema(){
    // write CUSTOM code here...
    //std::cout<<arg(0)<<": perceptualSchema() executed "<<std::endl;

    this->setRate(1000);

    //update IIWA STATE
    current_state = iiwa_interface.getFullState();

    wm_lock();

    iiwa_publish_tf();

    //export the whole state
    wmv_set<IIWA_state>("iiwa.state", current_state);

    // check the competition among actions
    // std::string new_action = wmv_solve<std::string>("iiwa.action");
    //NOTE: here we will try the self-competition among behaviors
    std::string new_action = wmv_get<std::string>("iiwa.action");


    // if action to be executed is changed, get the data about the new action
    if(new_action != current_action){
        std::cout<<arg(0)<<": executing "<<new_action<<std::endl;
        current_action = new_action;

        old_pose.clear();
        stop_counter = 0;

        // check if IIWA mode is changed
        std::string new_mode = wmv_get<std::string>("iiwa.mode");
        if(new_mode != current_mode){
            // if so, switch to the new control mode
            std::cout<<"\t switching from "<<current_mode<<" to "<<new_mode<<std::endl;

            //set control_mode and parameters depending on the new_mode
            if(new_mode=="execution_cartesian_impedance"){
                //THESE PARAMETERS WORK REALLY GOOD FOR EXECUTION (300 is MAX for angles!)
                std::vector<double> c_stiff_execution = {3500,3500,3500,300,300,300}; //this is able to write
                std::vector<double> c_dump_execution  = {1,1,1,1,1,1};
                
                iiwa_interface.setCartesianStiffness(c_stiff_execution);
                iiwa_interface.setCartesianDumping(c_dump_execution);

                iiwa_interface.setControlMode(IIWA_control_mode::CARTESIAN_IMPEDANCE);

                is_motion_continuous = false;
            }
            else if(new_mode=="insertion_cartesian_impedance"){
                //THESE PARAMETERS WORK REALLY GOOD FOR EXECUTION (300 is MAX for angles!)
                std::vector<double> c_stiff_insertion = {250,250,250,30,30,30}; //less force during insertion
                std::vector<double> c_dump_insertion  = {1,1,1,1,1,1};
                
                iiwa_interface.setCartesianStiffness(c_stiff_insertion);
                iiwa_interface.setCartesianDumping(c_dump_insertion);

                iiwa_interface.setControlMode(IIWA_control_mode::CARTESIAN_IMPEDANCE);

                is_motion_continuous = true;
            }
            else if(new_mode=="teaching_joint_impedance"){
                //THESE PARAMETERS WORK REALLY GOOD FOR TEACHING
                std::vector<double> j_stiff_gravity = {0.7,0.7,0.7,0.7,0.7,0.7,0.7}; //THIS WORKED VERY GOOD IN FIRST TESTS
                //std::vector<double> j_stiff_gravity = {50,50,50,50,50,50,50}; //SAFE SIDE
                std::vector<double> j_dump  = {1,1,1,1,1,1,1};
                
                iiwa_interface.setJointStiffness(j_stiff_gravity);
                iiwa_interface.setJointDumping(j_dump);

                iiwa_interface.setControlMode(IIWA_control_mode::JOINT_IMPEDANCE);
                steady_pose = current_state.joint_position; //fixed

                is_motion_continuous = true;
            }
            else{
                iiwa_interface.setControlMode(IIWA_control_mode::CARTESIAN_POSITION);
                iiwa_interface.setCartesianPosition(current_state.cartesian_position);

                is_motion_continuous = false;
            }

            current_mode = new_mode;
        }

        current_motion = wmv_get<std::vector<std::vector<double>>>("iiwa.motion");

        //plot_motion();
        //restart the motion
        iiwa_initialize_motion(50,15);
    }

    // update current action, wether it is used or not
    current_motion = wmv_get<std::vector<std::vector<double>>>("iiwa.motion");

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
void IIWAManagerBehavior::motorSchema(){
    // write CUSTOM code here...
    //std::cout<<arg(0)<<": motorSchema() executed "<<std::endl;

    // if IIWA is idle
    if(current_action == "idle" || current_action == ""){
        // stay in the current position
        //std::cout<<arg(0)<<": IIWA is IDLE! "<<std::endl;
        iiwa_interface.setCartesianPosition(current_state.cartesian_position);
    }
    else if(current_mode == "teaching_joint_impedance"){
        iiwa_interface.setJointPosition(steady_pose);
    }
    // otherwise
    else if(current_mode == "execution_cartesian_impedance" || 
            current_mode == "execution_cartesian_position" || 
            current_mode == "insertion_cartesian_impedance") {

        // execute current motion
        std::string execution_state;
        if(current_motion.size()>1)
            execution_state = iiwa_execute_cartesian_motion();
        else
            execution_state = iiwa_reach_cartesian_pose();

        //std::cout<<arg(0)<<": "<<current_action<<" is "<<execution_state<<std::endl;
        //update WMV
        if(execution_state != "RUNNING"){
            wm_lock();
            if(execution_state == "ACCOMPLISHED"){
                std::cout<<arg(0)<<": "<<current_action<<" ACCOMPLISHED"<<std::endl;
                wmv_set<bool>(current_action+".done",true);
            }
            else{
                std::cout<<arg(0)<<": "<<current_action<<" FAILED"<<std::endl;
                wmv_set<bool>(current_action+".fail",true);
            }
            //when over go to idle
            //  ISSUE: the robot stop working if switches from one impedance to another...
            //         We mast go through cartesian_position!
            wmv_set<std::string>("iiwa.action","idle");
            wmv_set<std::string>("iiwa.mode","idle_cartesian_position");
            current_mode = "idle_cartesian_position";
            current_action = "idle";

            iiwa_interface.setControlMode(IIWA_control_mode::CARTESIAN_POSITION);
            iiwa_interface.setCartesianPosition(current_state.cartesian_position);

            wm_unlock();

            //sleep(1); //wait 1 sec to be sure the robot goes in cartesian_position
        }
    }
}

// 6. DEFINE the exit function.
//  this function executed only once at the end of the execution
void IIWAManagerBehavior::exit(){
    // write CUSTOM code here...

    iiwa_interface.close_connection();
    std::cout<<arg(0)<<": connection CLOSED"<<std::endl;
}

std::string IIWAManagerBehavior::iiwa_reach_cartesian_pose(){

    

    //check if target pose is reached
    if(cartesian_pose_reached(current_motion[motion_i], current_state.cartesian_position)){
    //if(cartesian_pose_reached(current_motion[motion_i], current_state.cartesian_position, 5.0, 0.02)){

        iiwa_interface.setCartesianPosition(current_state.cartesian_position);
        std::cout<<arg(0)<<": pose is reached"<<std::endl;
        return "ACCOMPLISHED";

    }
    //check if the robot is still and the target pose is almost reached!
    //  NOTE: this is designed to finalize actions even if the robot is interacting with something
    else if(old_pose.size()!=0 && cartesian_pose_reached(old_pose, current_state.cartesian_position)
        && cartesian_pose_reached(current_motion[motion_i], current_state.cartesian_position, 10.0, 0.2)){

        if(stop_counter > 100){
            iiwa_interface.setCartesianPosition(current_state.cartesian_position);
            std::cout<<arg(0)<<": pose is almost reached, and robot is stopped!"<<std::endl;
            return "ACCOMPLISHED";
        }
        else{
            stop_counter++;
            //std::cout<<"action seems ended (counter is "<<stop_counter<<")..."<<std::endl;
        }

    }
    else
        stop_counter = 0;
    //Otherwise, set target pose again and wait for the robot to reach it

    // ISSUE: with impedance control the target is often not reached!
    //      let's try by adding a offset to the target pose -> not working
    std::vector<double> target_cartesian_position = current_motion[motion_i];

    old_pose = current_state.cartesian_position;

    iiwa_interface.setCartesianPosition(target_cartesian_position);
    //std::cout<<"running..."<<std::endl;
    return "RUNNING";
}

std::string IIWAManagerBehavior::iiwa_execute_cartesian_motion(){
    //std::cout<<"reaching waypoint "<<motion_i+1<<std::endl;

    //check if motion is completed
    if(motion_i >= current_motion.size()){
        //stay in the current position to be on the safe side
        iiwa_interface.setCartesianPosition(current_state.cartesian_position);
        std::cout<<arg(0)<<": motion is over"<<std::endl;
        return "ACCOMPLISHED";
    }

    //check if target pose is not reached
    if(is_motion_continuous || !cartesian_pose_reached(current_motion[motion_i], current_state.cartesian_position) && motion_n_loops < motion_n_max_loops){
        //if so, set target pose again and wait for the robot to reach it

        // ISSUE: with impedance control the target is often not reached!
        std::vector<double> target_cartesian_position = current_motion[motion_i];

        iiwa_interface.setCartesianPosition(target_cartesian_position);
        //avoid STUCK-checking on the first pose
        if(motion_i>0)
            motion_n_loops++;
        //std::cout<<"running..."<<std::endl;
        return "RUNNING";
    }
    //otherwise, or the pose has been reached or n_max_loop has been reached!
    
    // JUMPING PROCEDURE
    //      this has been created to prevent iiwa to get stuck due to impedance control
    if(motion_n_loops>=motion_n_max_loops){
        //the robot is stuck
        motion_n_loops = 0;
        if(motion_n_jumps <= motion_n_max_jumps){
            //we can try to jump to the next waypoint
            std::cout<<"ROBOT is STUCK, jump to next pose!"<<std::endl;
            plot_cartesian_difference(current_state.cartesian_position, current_motion[motion_i]);
            motion_i++;
            motion_n_jumps++;
            return "RUNNING";
        }
        else {
            //otherwise, abort!
            std::cout<<"ABORTED, maximum number of jumps reached!"<<std::endl;
            return "ABORTED";
        }
    }
    else
        motion_n_jumps = 0;

    //std::cout<<"wp reached..."<<std::endl;
    motion_i++;

    return "RUNNING";
}

void IIWAManagerBehavior::iiwa_publish_tf(std::string base_frame){

    geometry_msgs::msg::TransformStamped t;

    // Read message content and assign it to
    // corresponding tf variables
    t.header.stamp = nh->get_clock()->now();
    t.header.frame_id = base_frame;
    t.child_frame_id = "iiwa_ee";

    t.transform.translation.x = current_state.cartesian_position[0]/1000.0;
    t.transform.translation.y = current_state.cartesian_position[1]/1000.0;
    t.transform.translation.z = current_state.cartesian_position[2]/1000.0;

    // IIWA returns RPY
    tf2::Quaternion q;
    //q.setRPY(current_state.cartesian_position[3],
    //         current_state.cartesian_position[4], 
    //         current_state.cartesian_position[5]);
    
    //NOTE: IIWA EE seems to use YPR!
    q.setRPY(current_state.cartesian_position[5],
             current_state.cartesian_position[4], 
             current_state.cartesian_position[3]);

    t.transform.rotation.x = q.x();
    t.transform.rotation.y = q.y();
    t.transform.rotation.z = q.z();
    t.transform.rotation.w = q.w();

    // Send the transformation
    tf_broadcaster->sendTransform(t);
}



/* 
*  *******************************************************************************
*                                   IIWA GO
*  *******************************************************************************
*/


// REGISTER this behavior into the Behavior-based System (BBS).
//  This is a 2 steps process...

// 1. PROVIDE a unique name to this behavior
//  the behavior will be identified by this name
std::string IIWAGoBehavior::behavior_name = "iiwaGo";

// 2. ADD this behavior (identified by the behavior_name) to the BBS for possible activation
bool IIWAGoBehavior::registered = BehaviorBasedSystem::add(behavior_name,&IIWAGoBehavior::create);

// ...now your behavior has been registered, you may recall it by loading
//  a predicate in the form "<behavior_name>(<Arg1>, <Arg2>, ..., <ArgN>)" to the WM




// DEFINE the code of this behavior.
//  This is a 6 steps process...

// 1. DEFINE the constructor for this beahvior, it will be executed when this behavior is recalled.
//
//      instance: is the string containing the predicate that has been used to invoke ths behavior.
//                NOTE: if no paramaters (i.e., further args) are given then instance == behavior_name
IIWAGoBehavior::IIWAGoBehavior(std::string instance){
    // please set the instance before anything else
    setInstance(instance);

    // write CUSTOM construction code here...
    //std::cout<<arg(0)<<": Constructor() executed "<<std::endl;

    //init tf subscriber

    //initialize buffer
    tf_buffer = std::make_unique<tf2_ros::Buffer>(nh->get_clock());
    //initialize listener
    tf_listener = std::make_shared<tf2_ros::TransformListener>(*tf_buffer);

    have_goal = false;

    //if second argument is empty
    if(arg(2,false) != ""){
        start_pos = arg(1);
        end_pos = arg(2);
    }
    else
        end_pos = arg(1);

}

// 2. DEFINE the create function.
//  this function MUST return the reference to the currente behavior.
Behavior *IIWAGoBehavior::create(std::string instance){
    return new IIWAGoBehavior(instance);
}


// now let's implement the CUSTOM core of our behavior (virtual functions).

// 3. DEFINE the start function.
//  this function executed only once at the beginning of the execution
void IIWAGoBehavior::start(){
    // write CUSTOM code here...
    //std::cout<<arg(0)<<": start() executed "<<std::endl;

    //needed to give the time to subscribe!
    sleep(1);
    
    if(end_pos == "rnd"){
        //select a random free slot on the ground (gnd1, gnd2, gnd3)
        //  for now it is fixed to gnd1
        end_pos = "gnd1";
    }
    cartesian_goal.push_back( iiwa_subscribe_tf( end_pos ) );

    if(cartesian_goal[0].empty()){
        std::cout<<arg(0)<<": FRAME is not in TF, checking DB "<<std::endl;
        cartesian_goal.clear();

        std::ifstream infile (SEED_HOME_PATH + "/BBS/IIWA/DB/frames.txt");
        if (infile.is_open()) {
            std::string line;
            while (std::getline(infile, line)){
                std::istringstream ss(line);
                double fx, fy, fz, fa, fb, fc;
                if(ss>>frame_name>>fx>>fy>>fz>>fa>>fb>>fc && frame_name == end_pos){
                    std::vector<double> c_goal = {fx,fy,fz,deg2rad(fa),deg2rad(fb),deg2rad(fc)};
                    cartesian_goal.push_back(c_goal);
                    have_goal = true;
                    infile.close();

                    std::cout<<arg(0)<<": have goal (DB)! "<<std::endl;

                    return;
                }
            }
            std::cout<<arg(0)<<": FRAME does not exists "<<std::endl;
                
            infile.close();
        }
        else
            std::cout<<arg(0)<<": DB does not exists "<<std::endl;
    }
    else{
        have_goal = true;
        frame_name = end_pos;
        std::cout<<arg(0)<<": have goal (tf)! "<<std::endl;
    }
    
    if(have_goal){

        //check limits of the goal!
        cartesian_goal[0] = iiwa_safety_check(cartesian_goal[0]);

	    IIWA_state state = wmv_get<IIWA_state>("iiwa.state");
    
        std::cout<<arg(0)<<": FROM: "<<std::endl;
        std::cout<<"\t "<<state.cartesian_position[0]<<std::endl;
        std::cout<<"\t "<<state.cartesian_position[1]<<std::endl;
        std::cout<<"\t "<<state.cartesian_position[2]<<std::endl;
        std::cout<<"\t "<<state.cartesian_position[3]<<std::endl;
        std::cout<<"\t "<<state.cartesian_position[4]<<std::endl;
        std::cout<<"\t "<<state.cartesian_position[5]<<std::endl;

        std::cout<<arg(0)<<": TO: "<<std::endl;
        std::cout<<"\t "<<cartesian_goal[0][0]<<std::endl;
        std::cout<<"\t "<<cartesian_goal[0][1]<<std::endl;
        std::cout<<"\t "<<cartesian_goal[0][2]<<std::endl;
        std::cout<<"\t "<<cartesian_goal[0][3]<<std::endl;
        std::cout<<"\t "<<cartesian_goal[0][4]<<std::endl;
        std::cout<<"\t "<<cartesian_goal[0][5]<<std::endl;

        std::cout<<arg(0)<<": DIFF: "<<std::endl;
        plot_cartesian_difference(state.cartesian_position, cartesian_goal[0]);
    
    }

}

// 4. DEFINE the perceptualSchema.
//  this function is executed at the specified frequency.
//      If TRUE is returned, the motorSchema may be executed
bool IIWAGoBehavior::perceptualSchema(){
    // write CUSTOM code here...
    //std::cout<<arg(0)<<": perceptualSchema() executed "<<std::endl;

    wm_lock();
    if(have_goal){

        wmv_compete<std::string>("iiwaManager", "iiwa", this->getInstance());

        IIWA_state state = wmv_get<IIWA_state>("iiwa.state");

        if(cartesian_pose_reached(cartesian_goal[0], state.cartesian_position, 30.0, 10.0))
            wmv_set<bool>(frame_name+".near", true);
        else
            wmv_set<bool>(frame_name+".near", false);

        if(cartesian_pose_reached(cartesian_goal[0], state.cartesian_position))
        //if(cartesian_pose_reached(cartesian_goal[0], state.cartesian_position, 20.0, 0.2))
            wmv_set<bool>(frame_name+".reached", true);
        else
            wmv_set<bool>(frame_name+".reached", false);
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
void IIWAGoBehavior::motorSchema(){
    // write CUSTOM code here...
    //std::cout<<arg(0)<<": motorSchema() executed "<<std::endl;
    wm_lock();
    // SELF-SOLVING CONTENTION
    if(wmv_solve_once<std::string>("iiwa") == this->getInstance()){
        // if here, I'm in control of IIWA
        wmv_set<std::string>("iiwa.action", this->getInstance());
        wmv_set<std::string>("iiwa.mode", "execution_cartesian_impedance");
        //wmv_set<std::string>("iiwa.control_mode", "cartesian_position");
        wmv_set<std::vector<std::vector<double>>>("iiwa.motion", cartesian_goal);
    }
    wm_unlock();
}

// 6. DEFINE the exit function.
//  this function executed only once at the end of the execution
void IIWAGoBehavior::exit(){
    // write CUSTOM code here...
    std::cout<<arg(0)<<": exit() executed "<<std::endl;

    wm_lock();
    wmv_withdraw<std::string>("iiwa");
    wmv_set<bool>(this->getInstance() + ".done", false);
    wm_unlock();
}

std::vector<double> IIWAGoBehavior::iiwa_subscribe_tf(std::string target_frame){

        geometry_msgs::msg::TransformStamped t;

        std::string start_frame = "iiwa_base_link";
        
        try {
            //t = tf_buffer->lookupTransform(target_frame, start_frame, tf2::TimePointZero);
            t = tf_buffer->lookupTransform(start_frame, target_frame, tf2::TimePointZero);
            //t = tf_buffer->lookupTransform(target_frame, start_frame, nh->get_clock()->now(),rclcpp::Duration(1000000));
            std::cout<<arg(0)<<": TF FOUND"<<std::endl;

            tf2::Quaternion q(
                t.transform.rotation.x,
                t.transform.rotation.y,
                t.transform.rotation.z,
                t.transform.rotation.w);

            tf2::Matrix3x3 m(q);
            double roll, pitch, yaw;
            m.getRPY(roll, pitch, yaw);
            
            //NOTE: IIWA EE seems to use YPR!
            std::vector<double> iiwa_cartesian_goal{
                t.transform.translation.x*1000.0,
                t.transform.translation.y*1000.0,
                t.transform.translation.z*1000.0,
                //roll,
                //pitch,
                //yaw
                yaw,
                pitch,
                roll
            };

            return iiwa_cartesian_goal;
        } catch (const tf2::TransformException & ex) {
            std::cout<<arg(0)<<": unable to find transform from "<<start_frame<<" to "<<target_frame<<std::endl;
            return std::vector<double>();
        }
    }





/* 
*  *******************************************************************************
*                                   IIWA TEACH
*  *******************************************************************************
*/


// REGISTER this behavior into the Behavior-based System (BBS).
//  This is a 2 steps process...

// 1. PROVIDE a unique name to this behavior
//  the behavior will be identified by this name
std::string IIWATeachBehavior::behavior_name = "iiwaTeach";

// 2. ADD this behavior (identified by the behavior_name) to the BBS for possible activation
bool IIWATeachBehavior::registered = BehaviorBasedSystem::add(behavior_name,&IIWATeachBehavior::create);

// ...now your behavior has been registered, you may recall it by loading
//  a predicate in the form "<behavior_name>(<Arg1>, <Arg2>, ..., <ArgN>)" to the WM


// DEFINE the code of this behavior.
//  This is a 6 steps process...

// 1. DEFINE the constructor for this beahvior, it will be executed when this behavior is recalled.
//
//      instance: is the string containing the predicate that has been used to invoke ths behavior.
//                NOTE: if no paramaters (i.e., further args) are given then instance == behavior_name
IIWATeachBehavior::IIWATeachBehavior(std::string instance){
    // please set the instance before anything else
    setInstance(instance);

    // write CUSTOM construction code here...
    //std::cout<<arg(0)<<": Constructor() executed "<<std::endl;

    motion_name = arg(1);

    teaching_started = false;
    teaching_stopped = false;
}

// 2. DEFINE the create function.
//  this function MUST return the reference to the currente behavior.
Behavior *IIWATeachBehavior::create(std::string instance){
    return new IIWATeachBehavior(instance);
}


// now let's implement the CUSTOM core of our behavior (virtual functions).

// 3. DEFINE the start function.
//  this function executed only once at the beginning of the execution
void IIWATeachBehavior::start(){
    // write CUSTOM code here...
    //std::cout<<arg(0)<<": start() executed "<<std::endl;

}

// 4. DEFINE the perceptualSchema.
//  this function is executed at the specified frequency.
//      If TRUE is returned, the motorSchema may be executed
bool IIWATeachBehavior::perceptualSchema(){
    // write CUSTOM code here...
    //std::cout<<arg(0)<<": perceptualSchema() executed "<<std::endl;

    this->setRate(50);

    wm_lock();
    //if(!teaching_stopped)
    wmv_compete<std::string>("iiwaManager", "iiwa", this->getInstance());
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
void IIWATeachBehavior::motorSchema(){
    // write CUSTOM code here...
    //std::cout<<arg(0)<<": motorSchema() executed "<<std::endl;
    wm_lock();
    // SELF-SOLVING CONTENTION
    if(wmv_solve_once<std::string>("iiwa") == this->getInstance()){
        // I'm in control of the IIWA
        if(!teaching_started){
            // if here, I'm in control of IIWA
            wmv_set<std::string>("iiwa.action", this->getInstance());
            wmv_set<std::string>("iiwa.mode", "teaching_joint_impedance");
            teaching_started = true;

            initial_pose = wmv_get<IIWA_state>("iiwa.state").cartesian_position;
            //wmv_set<std::vector<std::vector<double>>>("iiwa.motion", cartesian_goals);
            std::cout<<"IIWA: please TEACH me how to do \""<<motion_name<<"\""<<std::endl;
        }
        else if(teaching_started && !teaching_stopped ){
            //here teaching is running...
            std::vector<double> current_pose = wmv_get<IIWA_state>("iiwa.state").cartesian_position;
            //if iiwa is away from initial pose (is moving)
            if(!cartesian_pose_reached(initial_pose, current_pose)){
                //record the pose
                cartesian_goals.push_back(current_pose);
                std::cout<<arg(0)<<": recorded pose "<<cartesian_goals.size()-1<<std::endl;

                //check if teach is over
                if(wmv_get<bool>("teaching.done")){
                    //here teaching is over!
                    teaching_stopped = true;

                    //remove the "k" node that advised me teaching is over
                    std::vector<WM_node *> k_nodes = WM->getNodesByInstance("k");
                    if(k_nodes.size()>0)
                        remove(k_nodes[0]);
                    
                    //set variables
                    wmv_set<bool>(this->getInstance() + ".done", true);
                    wmv_set<bool>(motion_name + ".known", true);

                    //save motion
                    std::ofstream outfile(SEED_HOME_PATH + "/BBS/IIWA/DB/motions/"+motion_name+".txt");
                    if (outfile) {
                        for(size_t i=0; i<cartesian_goals.size(); i++){
                            std::stringstream ss;
                            outfile<<cartesian_goals[i][0]<<" ";
                            outfile<<cartesian_goals[i][1]<<" ";
                            outfile<<cartesian_goals[i][2]<<" ";
                            outfile<<cartesian_goals[i][3]<<" ";
                            outfile<<cartesian_goals[i][4]<<" ";
                            outfile<<cartesian_goals[i][5]<<"\n";
                        }
                        outfile.close();
                        std::cout<<arg(0)<<": motion "<<motion_name<<" SAVED!"<<std::endl;
                    }
                    else
                        std::cout<<arg(0)<<": UNABLE to create FILE "<<std::endl;

                    //withdraw competition... this shoudl not be necessary
                    //wmv_withdraw<std::string>("iiwa");

                    //iiwa go idle
                    wmv_set<std::string>("iiwa.action", "idle");
                    wmv_set<std::string>("iiwa.mode", "idle_cartesian_position");
                }
            }
        }
    }
    wm_unlock();
}

// 6. DEFINE the exit function.
//  this function executed only once at the end of the execution
void IIWATeachBehavior::exit(){
    // write CUSTOM code here...
    wm_lock();
    wmv_withdraw<std::string>("iiwa");
    wmv_set<bool>(this->getInstance() + ".done", false);
    wm_unlock();
}




/* 
*  *******************************************************************************
*                                   IIWA EXE
*  *******************************************************************************
*/


// REGISTER this behavior into the Behavior-based System (BBS).
//  This is a 2 steps process...

// 1. PROVIDE a unique name to this behavior
//  the behavior will be identified by this name
std::string IIWAExeBehavior::behavior_name = "iiwaExe";

// 2. ADD this behavior (identified by the behavior_name) to the BBS for possible activation
bool IIWAExeBehavior::registered = BehaviorBasedSystem::add(behavior_name,&IIWAExeBehavior::create);

// ...now your behavior has been registered, you may recall it by loading
//  a predicate in the form "<behavior_name>(<Arg1>, <Arg2>, ..., <ArgN>)" to the WM




// DEFINE the code of this behavior.
//  This is a 6 steps process...

// 1. DEFINE the constructor for this beahvior, it will be executed when this behavior is recalled.
//
//      instance: is the string containing the predicate that has been used to invoke ths behavior.
//                NOTE: if no paramaters (i.e., further args) are given then instance == behavior_name
IIWAExeBehavior::IIWAExeBehavior(std::string instance){
    // please set the instance before anything else
    setInstance(instance);

    // write CUSTOM construction code here...
    //std::cout<<arg(0)<<": Constructor() executed "<<std::endl;

    have_motion = false;
    motion_delivered = false;

    motion_name = arg(1);

}

// 2. DEFINE the create function.
//  this function MUST return the reference to the currente behavior.
Behavior *IIWAExeBehavior::create(std::string instance){
    return new IIWAExeBehavior(instance);
}


// now let's implement the CUSTOM core of our behavior (virtual functions).

// 3. DEFINE the start function.
//  this function executed only once at the beginning of the execution
void IIWAExeBehavior::start(){
    // write CUSTOM code here...
    //std::cout<<arg(0)<<": start() executed "<<std::endl;

    load_motion();

    wm_lock();
    if(have_motion)
        wmv_set<bool>(motion_name+".known", true);
    else
        wmv_set<bool>(motion_name+".known", false);
    wm_unlock();

}

// 4. DEFINE the perceptualSchema.
//  this function is executed at the specified frequency.
//      If TRUE is returned, the motorSchema may be executed
bool IIWAExeBehavior::perceptualSchema(){
    // write CUSTOM code here...
    //std::cout<<arg(0)<<": perceptualSchema() executed "<<std::endl;

    wm_lock();

    if(!have_motion){
        //check if motion becomes available
        if(wmv_get<bool>(motion_name+".known")){
            load_motion();
        }
    }


    if(wmv_get<bool>(motion_name+".known")){
        wmv_compete<std::string>("iiwaManager", "iiwa", this->getInstance());
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
void IIWAExeBehavior::motorSchema(){
    // write CUSTOM code here...
    //std::cout<<arg(0)<<": motorSchema() executed "<<std::endl;
    wm_lock();
    // SELF-SOLVING CONTENTION
    if(wmv_solve_once<std::string>("iiwa") == this->getInstance()){
        // if here, I'm in control of IIWA
        if(!motion_delivered){
            //send motion to iiwa
            wmv_set<std::string>("iiwa.action", this->getInstance());
            wmv_set<std::string>("iiwa.mode", "execution_cartesian_impedance");
            //wmv_set<std::string>("iiwa.control_mode", "cartesian_position");
            wmv_set<std::vector<std::vector<double>>>("iiwa.motion", cartesian_motion);
        }
    }
    else
        motion_delivered = false;
    wm_unlock();
}

// 6. DEFINE the exit function.
//  this function executed only once at the end of the execution
void IIWAExeBehavior::exit(){
    // write CUSTOM code here...
    std::cout<<arg(0)<<": exit() executed "<<std::endl;
    wm_lock();
    wmv_withdraw<std::string>("iiwa");
    wmv_set<bool>(this->getInstance() + ".done", false);
    wm_unlock();
}


void IIWAExeBehavior::load_motion(){
    std::vector<double> f_teach = get_frame_from_db("teach");
    std::vector<double> f_target = get_frame_from_db(arg(2));

    if(f_teach.size()>0 && f_target.size()>0){

        double off_x, off_y, off_z;
        off_x = f_target[0] - f_teach[0];
        off_y = f_target[1] - f_teach[1];
        off_z = f_target[2] - f_teach[2];

        std::ifstream infile (SEED_HOME_PATH + "/BBS/IIWA/DB/motions/"+motion_name+".txt");
        if (infile.is_open()) {
            std::string line;
            have_motion = true;
            while (std::getline(infile, line)){
                std::istringstream ss(line);
                double fx, fy, fz, fa, fb, fc;
                if(ss>>fx>>fy>>fz>>fa>>fb>>fc){
                    std::vector<double> c_goal = {fx + off_x,fy + off_y,fz + off_z,fa,fb,fc};
                    cartesian_motion.push_back(c_goal);
                }
            }
            infile.close();
        }
        else
            std::cout<<arg(0)<<": MOTION "<<motion_name<<" does not exists "<<std::endl;
    }
    else
        std::cout<<arg(0)<<": a FRAME does not exists "<<std::endl;
}



/* 
*  *******************************************************************************
*                                   IIWA INSERT
*  *******************************************************************************
*/


// REGISTER this behavior into the Behavior-based System (BBS).
//  This is a 2 steps process...

// 1. PROVIDE a unique name to this behavior
//  the behavior will be identified by this name
std::string IIWAInsertBehavior::behavior_name = "iiwaInsert";

// 2. ADD this behavior (identified by the behavior_name) to the BBS for possible activation
bool IIWAInsertBehavior::registered = BehaviorBasedSystem::add(behavior_name,&IIWAInsertBehavior::create);

// ...now your behavior has been registered, you may recall it by loading
//  a predicate in the form "<behavior_name>(<Arg1>, <Arg2>, ..., <ArgN>)" to the WM




// DEFINE the code of this behavior.
//  This is a 6 steps process...

// 1. DEFINE the constructor for this beahvior, it will be executed when this behavior is recalled.
//
//      instance: is the string containing the predicate that has been used to invoke ths behavior.
//                NOTE: if no paramaters (i.e., further args) are given then instance == behavior_name
IIWAInsertBehavior::IIWAInsertBehavior(std::string instance){
    // please set the instance before anything else
    setInstance(instance);

    // write CUSTOM construction code here...
    //std::cout<<arg(0)<<": Constructor() executed "<<std::endl;

    //get object to insert
    object_id = arg(1);

    //get object in which to insert
    slot_id = arg(2);

    //init tf subscriber

    //initialize buffer
    tf_buffer = std::make_unique<tf2_ros::Buffer>(nh->get_clock());
    //initialize listener
    tf_listener = std::make_shared<tf2_ros::TransformListener>(*tf_buffer);

    have_goal = false;

}

// 2. DEFINE the create function.
//  this function MUST return the reference to the currente behavior.
Behavior *IIWAInsertBehavior::create(std::string instance){
    return new IIWAInsertBehavior(instance);
}


// now let's implement the CUSTOM core of our behavior (virtual functions).

// 3. DEFINE the start function.
//  this function executed only once at the beginning of the execution
void IIWAInsertBehavior::start(){
    // write CUSTOM code here...
    //std::cout<<arg(0)<<": start() executed "<<std::endl;

}

// 4. DEFINE the perceptualSchema.
//  this function is executed at the specified frequency.
//      If TRUE is returned, the motorSchema may be executed
bool IIWAInsertBehavior::perceptualSchema(){
    // write CUSTOM code here...
    //std::cout<<arg(0)<<": perceptualSchema() executed "<<std::endl;

    //this->setRate(1);
    this->setRate(2);

    cartesian_target = iiwa_subscribe_tf(slot_id);

    object_pose = iiwa_subscribe_tf(object_id);

    have_goal = false;
    if(cartesian_target.size() != 0 && object_pose.size()!=0){
        have_goal = true;
    }

    wm_lock();
    if(have_goal){
        wmv_compete<std::string>("iiwaManager", "iiwa", this->getInstance());
    }
    wm_unlock();

    return have_goal;
}

// 5. DEFINE the motorSchema.
//  this function is executed at the specified frequency (+ delay of perceptualSchema), 
//  but only if: 
//      - the perceptualSchema returns true
//      - all releasers along the WM branch are true
//      - all goals along the WM branch are false
//  if these conditions are not satisfied, only perceptualSchema is executed.
void IIWAInsertBehavior::motorSchema(){
    // write CUSTOM code here...
    //std::cout<<arg(0)<<": motorSchema() executed "<<std::endl;
    wm_lock();
    // SELF-SOLVING CONTENTION
    if(wmv_solve_once<std::string>("iiwa") == this->getInstance()){
        // if here, I'm in control of IIWA
        wmv_set<std::string>("iiwa.action", this->getInstance());
        wmv_set<std::string>("iiwa.mode", "insertion_cartesian_impedance");

        //send the cartesian target
        //  NOTE: here we have to implement a sort of peg-in-hole
        //      the target has to randomly change a bit to shake a little the robot

        double lin_limit = 15.0; //10.0;
        double yaw_limit = 0.1;
        double ang_limit = 0.02;

        //shakle the robot
        std::vector<double> noisy_cartesian_target;
        noisy_cartesian_target.push_back(cartesian_target[0] + rnd.real(-lin_limit,lin_limit) ); // 5mm
        noisy_cartesian_target.push_back(cartesian_target[1] + rnd.real(-lin_limit,lin_limit) );
        noisy_cartesian_target.push_back(cartesian_target[2] + rnd.real(-lin_limit,lin_limit) );
        noisy_cartesian_target.push_back(cartesian_target[3] + rnd.real(-yaw_limit,yaw_limit) ); // 10deg 
        noisy_cartesian_target.push_back(cartesian_target[4] + rnd.real(-ang_limit,ang_limit) );
        noisy_cartesian_target.push_back(cartesian_target[5] + rnd.real(-ang_limit,ang_limit) );

        std::vector<std::vector<double>> one_point_motion;
        one_point_motion.push_back(noisy_cartesian_target);

        wmv_set<std::vector<std::vector<double>>>("iiwa.motion", one_point_motion);

        IIWA_state state = wmv_get<IIWA_state>("iiwa.state");

        if(!cartesian_pose_reached(cartesian_target, state.cartesian_position,8,0.10)){
        //if(!cartesian_pose_reached(cartesian_target, object_pose)){
            wmv_set<bool>("inserted("+object_id+","+slot_id+")", false);
            std::cout<<"insertion running:"<<std::endl;
            plot_cartesian_difference(cartesian_target, state.cartesian_position);
        }
        else {
            std::cout<<"insertion accomplished!"<<std::endl;
            wmv_set<bool>("inserted("+object_id+","+slot_id+")", true);
        }


    }
    wm_unlock();
}

// 6. DEFINE the exit function.
//  this function executed only once at the end of the execution
void IIWAInsertBehavior::exit(){
    // write CUSTOM code here...
    std::cout<<arg(0)<<": exit() executed "<<std::endl;

    wm_lock();
    wmv_withdraw<std::string>("iiwa");
    wmv_set<bool>(this->getInstance() + ".done", false);
    wm_unlock();
}


std::vector<double> IIWAInsertBehavior::iiwa_subscribe_tf(std::string target_frame){

        geometry_msgs::msg::TransformStamped t;

        std::string start_frame = "iiwa_base_link";
        
        try {
            //t = tf_buffer->lookupTransform(target_frame, start_frame, tf2::TimePointZero);
            t = tf_buffer->lookupTransform(start_frame, target_frame, tf2::TimePointZero);
            //t = tf_buffer->lookupTransform(target_frame, start_frame, nh->get_clock()->now(),rclcpp::Duration(1000000));
            std::cout<<arg(0)<<": TF FOUND"<<std::endl;

            tf2::Quaternion q(
                t.transform.rotation.x,
                t.transform.rotation.y,
                t.transform.rotation.z,
                t.transform.rotation.w);

            tf2::Matrix3x3 m(q);
            double roll, pitch, yaw;
            m.getRPY(roll, pitch, yaw);
            
            //NOTE: IIWA EE seems to use YPR!
            std::vector<double> iiwa_cartesian_goal{
                t.transform.translation.x*1000.0,
                t.transform.translation.y*1000.0,
                t.transform.translation.z*1000.0,
                //roll,
                //pitch,
                //yaw
                yaw,
                pitch,
                roll
            };

            return iiwa_cartesian_goal;
        } catch (const tf2::TransformException & ex) {
            std::cout<<arg(0)<<": unable to find transform from "<<start_frame<<" to "<<target_frame<<std::endl;
            return std::vector<double>();
        }
    }






/* 
*  *******************************************************************************
*                                   IIWA EXTRACT
*  *******************************************************************************
*/


// REGISTER this behavior into the Behavior-based System (BBS).
//  This is a 2 steps process...

// 1. PROVIDE a unique name to this behavior
//  the behavior will be identified by this name
std::string IIWAExtractBehavior::behavior_name = "iiwaExtract";

// 2. ADD this behavior (identified by the behavior_name) to the BBS for possible activation
bool IIWAExtractBehavior::registered = BehaviorBasedSystem::add(behavior_name,&IIWAExtractBehavior::create);

// ...now your behavior has been registered, you may recall it by loading
//  a predicate in the form "<behavior_name>(<Arg1>, <Arg2>, ..., <ArgN>)" to the WM




// DEFINE the code of this behavior.
//  This is a 6 steps process...

// 1. DEFINE the constructor for this beahvior, it will be executed when this behavior is recalled.
//
//      instance: is the string containing the predicate that has been used to invoke ths behavior.
//                NOTE: if no paramaters (i.e., further args) are given then instance == behavior_name
IIWAExtractBehavior::IIWAExtractBehavior(std::string instance){
    // please set the instance before anything else
    setInstance(instance);

    // write CUSTOM construction code here...
    //std::cout<<arg(0)<<": Constructor() executed "<<std::endl;

    //get object to extract
    object_id = arg(1);

    //get object from which to extract
    slot_id = arg(2);

    //init tf subscriber

    //initialize buffer
    tf_buffer = std::make_unique<tf2_ros::Buffer>(nh->get_clock());
    //initialize listener
    tf_listener = std::make_shared<tf2_ros::TransformListener>(*tf_buffer);

    have_goal = false;

}

// 2. DEFINE the create function.
//  this function MUST return the reference to the currente behavior.
Behavior *IIWAExtractBehavior::create(std::string instance){
    return new IIWAExtractBehavior(instance);
}


// now let's implement the CUSTOM core of our behavior (virtual functions).

// 3. DEFINE the start function.
//  this function executed only once at the beginning of the execution
void IIWAExtractBehavior::start(){
    // write CUSTOM code here...
    //std::cout<<arg(0)<<": start() executed "<<std::endl;

    fixed_target_z = -1;

}

// 4. DEFINE the perceptualSchema.
//  this function is executed at the specified frequency.
//      If TRUE is returned, the motorSchema may be executed
bool IIWAExtractBehavior::perceptualSchema(){
    // write CUSTOM code here...
    //std::cout<<arg(0)<<": perceptualSchema() executed "<<std::endl;

    //this->setRate(1);
    //this->setRate(5);
    this->setRate(10); //shackle harder!

    cartesian_target = iiwa_subscribe_tf(slot_id);

    object_pose = iiwa_subscribe_tf(object_id);

    have_goal = false;
    if(cartesian_target.size() != 0 && object_pose.size()!=0){
        if(fixed_target_z < 0.0)
            fixed_target_z = cartesian_target[2] + 20.0; //add 2 cm to the .over tf (ONLY ONCE!)
        have_goal = true;
    }

    wm_lock();
    if(have_goal){
        wmv_compete<std::string>("iiwaManager", "iiwa", this->getInstance());
    }
    wm_unlock();

    return have_goal;
}

// 5. DEFINE the motorSchema.
//  this function is executed at the specified frequency (+ delay of perceptualSchema), 
//  but only if: 
//      - the perceptualSchema returns true
//      - all releasers along the WM branch are true
//      - all goals along the WM branch are false
//  if these conditions are not satisfied, only perceptualSchema is executed.
void IIWAExtractBehavior::motorSchema(){
    // write CUSTOM code here...
    //std::cout<<arg(0)<<": motorSchema() executed "<<std::endl;
    wm_lock();
    // SELF-SOLVING CONTENTION
    if(wmv_solve_once<std::string>("iiwa") == this->getInstance()){
        // if here, I'm in control of IIWA
        wmv_set<std::string>("iiwa.action", this->getInstance());
        wmv_set<std::string>("iiwa.mode", "insertion_cartesian_impedance");

        //send the cartesian target
        //  NOTE: here we have to implement a sort of peg-in-hole
        //      the target has to randomly change a bit to shake a little the robot

        double lin_limit = 10.0; //10.0;
        double yaw_limit = 0.2;
        double ang_limit = 0.1;

        //shakle the robot
        std::vector<double> noisy_cartesian_target;
        noisy_cartesian_target.push_back(cartesian_target[0] + rnd.real(-lin_limit,0) ); // 5mm
        noisy_cartesian_target.push_back(cartesian_target[1] + rnd.real(-lin_limit,0) );
        noisy_cartesian_target.push_back(fixed_target_z + rnd.real(-lin_limit,0) ); //NOTE: this is to avoid infinite following of bricks' Z
        noisy_cartesian_target.push_back(cartesian_target[3]); //+ rnd.real(-yaw_limit,yaw_limit) ); // 10deg 
        noisy_cartesian_target.push_back(cartesian_target[4]); //+ rnd.real(-ang_limit,ang_limit) );
        noisy_cartesian_target.push_back(cartesian_target[5]); //+ rnd.real(-ang_limit,ang_limit) );

        std::vector<std::vector<double>> one_point_motion;
        one_point_motion.push_back(noisy_cartesian_target);

        wmv_set<std::vector<std::vector<double>>>("iiwa.motion", one_point_motion);

        IIWA_state state = wmv_get<IIWA_state>("iiwa.state");

        if(!cartesian_pose_reached(cartesian_target, state.cartesian_position,7,0.10)){
        //if(!cartesian_pose_reached(cartesian_target, object_pose)){
            wmv_set<bool>("extracted("+object_id+","+slot_id+")", false);
            std::cout<<"extraction running:"<<std::endl;
            plot_cartesian_difference(cartesian_target, state.cartesian_position);
        }
        else {
            std::cout<<"extraction accomplished!"<<std::endl;
            wmv_set<bool>("extracted("+object_id+","+slot_id+")", true);
        }


    }
    wm_unlock();
}

// 6. DEFINE the exit function.
//  this function executed only once at the end of the execution
void IIWAExtractBehavior::exit(){
    // write CUSTOM code here...
    std::cout<<arg(0)<<": exit() executed "<<std::endl;

    wm_lock();
    wmv_withdraw<std::string>("iiwa");
    wmv_set<bool>(this->getInstance() + ".done", false);
    wm_unlock();
}


std::vector<double> IIWAExtractBehavior::iiwa_subscribe_tf(std::string target_frame){

        geometry_msgs::msg::TransformStamped t;

        std::string start_frame = "iiwa_base_link";
        
        try {
            //t = tf_buffer->lookupTransform(target_frame, start_frame, tf2::TimePointZero);
            t = tf_buffer->lookupTransform(start_frame, target_frame, tf2::TimePointZero);
            //t = tf_buffer->lookupTransform(target_frame, start_frame, nh->get_clock()->now(),rclcpp::Duration(1000000));
            std::cout<<arg(0)<<": TF FOUND"<<std::endl;

            tf2::Quaternion q(
                t.transform.rotation.x,
                t.transform.rotation.y,
                t.transform.rotation.z,
                t.transform.rotation.w);

            tf2::Matrix3x3 m(q);
            double roll, pitch, yaw;
            m.getRPY(roll, pitch, yaw);
            
            //NOTE: IIWA EE seems to use YPR!
            std::vector<double> iiwa_cartesian_goal{
                t.transform.translation.x*1000.0,
                t.transform.translation.y*1000.0,
                t.transform.translation.z*1000.0,
                //roll,
                //pitch,
                //yaw
                yaw,
                pitch,
                roll
            };

            return iiwa_cartesian_goal;
        } catch (const tf2::TransformException & ex) {
            std::cout<<arg(0)<<": unable to find transform from "<<start_frame<<" to "<<target_frame<<std::endl;
            return std::vector<double>();
        }
    }







/* 
*  *******************************************************************************
*                                   IIWA WRITE
*  *******************************************************************************
*/


// REGISTER this behavior into the Behavior-based System (BBS).
//  This is a 2 steps process...

// 1. PROVIDE a unique name to this behavior
//  the behavior will be identified by this name
std::string IIWAWriteBehavior::behavior_name = "iiwaWrite";

// 2. ADD this behavior (identified by the behavior_name) to the BBS for possible activation
bool IIWAWriteBehavior::registered = BehaviorBasedSystem::add(behavior_name,&IIWAWriteBehavior::create);

// ...now your behavior has been registered, you may recall it by loading
//  a predicate in the form "<behavior_name>(<Arg1>, <Arg2>, ..., <ArgN>)" to the WM




// DEFINE the code of this behavior.
//  This is a 6 steps process...

// 1. DEFINE the constructor for this beahvior, it will be executed when this behavior is recalled.
//
//      instance: is the string containing the predicate that has been used to invoke ths behavior.
//                NOTE: if no paramaters (i.e., further args) are given then instance == behavior_name
IIWAWriteBehavior::IIWAWriteBehavior(std::string instance){
    // please set the instance before anything else
    setInstance(instance);

    // write CUSTOM construction code here...
    //std::cout<<arg(0)<<": Constructor() executed "<<std::endl;

    //get characters
    std::string word = arg(1);

    for(int i=0; i<word.length(); i++){
        std::string str(1, word.at(i));
        std::cout<<arg(0)<<": got char: "<<str<<std::endl;
        characters.push_back(str);
    }

    current_c = 0;

    //get frame info
    std::vector<std::string> iv = instance2vector(arg(2));
    m_x = ston(iv[1]);
    m_y = ston(iv[2]);

}

// 2. DEFINE the create function.
//  this function MUST return the reference to the currente behavior.
Behavior *IIWAWriteBehavior::create(std::string instance){
    return new IIWAWriteBehavior(instance);
}


// now let's implement the CUSTOM core of our behavior (virtual functions).

// 3. DEFINE the start function.
//  this function executed only once at the beginning of the execution
void IIWAWriteBehavior::start(){
    // write CUSTOM code here...
    //std::cout<<arg(0)<<": start() executed "<<std::endl;

}

// 4. DEFINE the perceptualSchema.
//  this function is executed at the specified frequency.
//      If TRUE is returned, the motorSchema may be executed
bool IIWAWriteBehavior::perceptualSchema(){
    // write CUSTOM code here...
    //std::cout<<arg(0)<<": perceptualSchema() executed "<<std::endl;

    return true;
}

// 5. DEFINE the motorSchema.
//  this function is executed at the specified frequency (+ delay of perceptualSchema), 
//  but only if: 
//      - the perceptualSchema returns true
//      - all releasers along the WM branch are true
//      - all goals along the WM branch are false
//  if these conditions are not satisfied, only perceptualSchema is executed.
void IIWAWriteBehavior::motorSchema(){
    // write CUSTOM code here...
    //std::cout<<arg(0)<<": motorSchema() executed "<<std::endl;
    
    wm_lock();
    WM_node *me = WM->getNodesByInstance(this->getInstance())[0];
    //if son is already in execution
    if(me->son.size()>0){
        //if son is accomplished
        if(me->son[0]->goalStatus()){
            remove(me->son[0]);
            current_c++;
        }
    }
    //oth. load a new son
    else {
        if(current_c < characters.size()){
            std::stringstream ss;
            //ss<<"iiwaExe("<<characters[current_c]<<",m("<<m_x+current_c<<","<<m_y<<"))";
            //ss<<"iiwaTry("<<characters[current_c]<<",m("<<m_x+current_c<<","<<m_y<<"))";
            ss<<"iiwa try "<<characters[current_c]<<" m("<<m_x+current_c<<","<<m_y<<")";
            me->addSon(ss.str());
            std::cout<<arg(0)<<": new "<<ss.str()<<" action LOADED"<<std::endl;
        }
        else
            wmv_set<bool>(this->getInstance()+".done",true);
    }

    wm_unlock();
}

// 6. DEFINE the exit function.
//  this function executed only once at the end of the execution
void IIWAWriteBehavior::exit(){
    // write CUSTOM code here...
    std::cout<<arg(0)<<": exit() executed "<<std::endl;

    wm_lock();
    wmv_set<bool>(this->getInstance()+".done",false);
    wm_unlock();
}




// ...enjoy
