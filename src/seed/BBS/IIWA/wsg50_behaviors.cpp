/* 
 * Author: RC
 *
 * First created: 9-feb-2024 (SEED 7.0)
 * 
 */

#include "wsg50_behaviors.h"

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
std::string WSG50ManagerBehavior::behavior_name = "wsg50Manager";

// 2. ADD this behavior (identified by the behavior_name) to the BBS for possible activation
bool WSG50ManagerBehavior::registered = BehaviorBasedSystem::add(behavior_name,&WSG50ManagerBehavior::create);

// ...now your behavior has been registered, you may recall it by loading
//  a predicate in the form "<behavior_name>(<Arg1>, <Arg2>, ..., <ArgN>)" to the WM



// DEFINE the code of this behavior.
//  This is a 6 steps process...

// 1. DEFINE the constructor for this beahvior, it will be executed when this behavior is recalled.
//
//      instance: is the string containing the predicate that has been used to invoke ths behavior.
//                NOTE: if no paramaters (i.e., further args) are given then instance == behavior_name
WSG50ManagerBehavior::WSG50ManagerBehavior(std::string instance){
    // please set the instance before anything else
    setInstance(instance);

    wmv_set<bool>("gripper.ready",false);
    wmv_set<std::string>("gripper.holding","");

    current_action = "idle";
    have_new_action = false;

    // write CUSTOM construction code here...
    std::cout<<arg(0)<<": Constructor() executed "<<std::endl;
}

// 2. DEFINE the create function.
//  this function MUST return the reference to the currente behavior.
Behavior *WSG50ManagerBehavior::create(std::string instance){
    return new WSG50ManagerBehavior(instance);
}


// now let's implement the CUSTOM core of our behavior (virtual functions).

// 3. DEFINE the start function.
//  this function executed only once at the beginning of the execution
void WSG50ManagerBehavior::start(){
    // write CUSTOM code here...
    std::cout<<arg(0)<<": Starting connection with WSG50..."<<std::endl;
    
    //open conncetion to WSG50
    if (wsg50.open_connection(1000, "172.31.1.20")){
        std::cout<<arg(0)<<": ...WSG50 connection established"<<std::endl;

        //initialize gripper
        wsg50.go_home();

        //set min distance considering VANVITELLI fingers!
        wsg50.set_min_width(16.0); //to be sure, it is 15mm in reality

        wm_lock();
        wmv_set<bool>("gripper.ready",true);
        wmv_set<bool>("gripper.open",true);
        //if the gripper was holding something, remove it
        std::string previous_target = wmv_get<std::string>("gripper.holding");
        wmv_set<std::string>("gripper.holding","");
        if(previous_target != ""){
            wmv_set<bool>("gripper.hold("+previous_target+")",false);
        }
        wm_unlock();

    }
    else{
        std::cout<<arg(0)<<": ...unable to connecto to WSG50, exiting"<<std::endl;
        wm_lock();
        wm_remove_associated_nodes();
        wm_unlock();
    }

    
}

// 4. DEFINE the perceptualSchema.
//  this function is executed at the specified frequency.
//      If TRUE is returned, the motorSchema may be executed
bool WSG50ManagerBehavior::perceptualSchema(){
    // write CUSTOM code here...
    //std::cout<<arg(0)<<": perceptualSchema() executed "<<std::endl;

    this->setRate(10);

    wm_lock();

    // check the competition among actions
    //NOTE: here we will try the self-competition among behaviors
    std::string new_action = wmv_get<std::string>("gripper.action");

    //if a new action is detected
    if(new_action != "" && new_action != "idle" && new_action != current_action){
        //the gripper is not READY
        std::cout<<arg(0)<<": have new action "<<new_action<<std::endl;
        wmv_set<bool>("gripper.ready", false);
        //NOTE: this is a safety protocol: 
        //  since the motion of the gripper is blocking (i.e., it must be
        //  performed on unlocked semaphore) it is better to advise the
        //  rest of the system that the gripper is unavailable

        have_new_action = true;
        current_action = new_action;
        current_target = wmv_get<std::string>("gripper.target");
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
void WSG50ManagerBehavior::motorSchema(){
    // write CUSTOM code here...
    //std::cout<<arg(0)<<": motorSchema() executed "<<std::endl;

    if(have_new_action){
        bool action_executed = false;

        std::cout<<arg(0)<<": executing "<<current_action<<"..."<<std::endl;
        // if action to be executed is changed, get the data about the new action
        if(current_action == "open"){
            action_executed = wsg50.go_home(); //NOTE: maybe this can be made more gently
        }
        else if(current_action == "close"){
            
            //wsg50.set_grasping_force(5.0); //TOO LOW FOR THE BRICKS!
            wsg50.set_grasping_force(40.0);

            //seed::time::Clock clk;
            //std::cout<<"\t garsp start "<<std::endl;

            //NOTE: this needs to be object-specific, the size of the object must be PRECISE!!
            double obj_w = 25.0; //THIS IS THE LOWER LIMIT!
            if(current_target.find("brick") != std::string::npos)
                obj_w = 65.0;
            else if(current_target.find("bottle") != std::string::npos)
                obj_w = 45.0; //75.0;
            else if(current_target.find("ball") != std::string::npos)
                obj_w = 65.0; //real object w is 65.0
            else if(current_target.find("pen") != std::string::npos)
                obj_w = 25.0; //17.5; //real object w is 65.0

            action_executed = wsg50.do_grasp(obj_w, 50.0); //NOTE: this is sort of general purpose 
            //std::cout<<"\t garsp end "<<clk.toc()<<std::endl;
            
            //wsg50.set_grasping_force(0.0);

            action_executed = true;
            //if(!action_executed){
            //    action_executed = wsg50.send_ack(); //THIS ACK-SEND IS NOT WORKING!
            //}
        }

        wm_lock();

        if(action_executed){
            std::cout<<arg(0)<<": done "<<std::endl;
            if(current_action == "open"){
                //gripper is now open
                wmv_set<bool>("gripper.open",true);
                //if the gripper was holding an object, remove it
                std::string previous_target = wmv_get<std::string>("gripper.holding");
                wmv_set<std::string>("gripper.holding","");
                if(previous_target != ""){
                    wmv_set<bool>("gripper.hold("+previous_target+")",false);
                }
            }
            else if(current_action == "close"){
                //gripper is now closed
                wmv_set<bool>("gripper.open",false);
                wmv_set<std::string>("gripper.holding",current_target);
                if(current_target != ""){
                    wmv_set<bool>("gripper.hold("+current_target+")",true);
                }
            }
            wmv_set<bool>("gripper.ready", true);
        }
        else {
            std::cout<<arg(0)<<": execution ERROR, gripper is not ready anymore "<<std::endl;
        }
        have_new_action = false;
        wmv_set<std::string>("gripper.action", "idle");

        wm_unlock();
    }
}

// 6. DEFINE the exit function.
//  this function executed only once at the end of the execution
void WSG50ManagerBehavior::exit(){
    // write CUSTOM code here...

    wsg50.close_connection();

    //sleep(1);
    std::cout<<arg(0)<<": connection CLOSED"<<std::endl;
}


/* 
*  *******************************************************************************
*                                   WSG50 GRASP
*  *******************************************************************************
*/


// REGISTER this behavior into the Behavior-based System (BBS).
//  This is a 2 steps process...

// 1. PROVIDE a unique name to this behavior
//  the behavior will be identified by this name
std::string WSG50GraspBehavior::behavior_name = "wsg50Grasp";

// 2. ADD this behavior (identified by the behavior_name) to the BBS for possible activation
bool WSG50GraspBehavior::registered = BehaviorBasedSystem::add(behavior_name,&WSG50GraspBehavior::create);

// ...now your behavior has been registered, you may recall it by loading
//  a predicate in the form "<behavior_name>(<Arg1>, <Arg2>, ..., <ArgN>)" to the WM




// DEFINE the code of this behavior.
//  This is a 6 steps process...

// 1. DEFINE the constructor for this beahvior, it will be executed when this behavior is recalled.
//
//      instance: is the string containing the predicate that has been used to invoke ths behavior.
//                NOTE: if no paramaters (i.e., further args) are given then instance == behavior_name
WSG50GraspBehavior::WSG50GraspBehavior(std::string instance){
    // please set the instance before anything else
    setInstance(instance);

    // write CUSTOM construction code here...
    //std::cout<<arg(0)<<": Constructor() executed "<<std::endl;

    target = arg(1);
}

// 2. DEFINE the create function.
//  this function MUST return the reference to the currente behavior.
Behavior *WSG50GraspBehavior::create(std::string instance){
    return new WSG50GraspBehavior(instance);
}


// now let's implement the CUSTOM core of our behavior (virtual functions).

// 3. DEFINE the start function.
//  this function executed only once at the beginning of the execution
void WSG50GraspBehavior::start(){
    // write CUSTOM code here...
    //std::cout<<arg(0)<<": start() executed "<<std::endl;
}

// 4. DEFINE the perceptualSchema.
//  this function is executed at the specified frequency.
//      If TRUE is returned, the motorSchema may be executed
bool WSG50GraspBehavior::perceptualSchema(){
    // write CUSTOM code here...
    //std::cout<<arg(0)<<": perceptualSchema() executed "<<std::endl;

    wm_lock();
    //subscribe this node to the competition
    wmv_compete<std::string>("wsg50Manager", "wsg50", this->getInstance());
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
void WSG50GraspBehavior::motorSchema(){
    // write CUSTOM code here...
    //std::cout<<arg(0)<<": motorSchema() executed "<<std::endl;
    wm_lock();
    // SELF-SOLVING CONTENTION
    if(wmv_solve_once<std::string>("wsg50") == this->getInstance()){
        // if here, I'm in control of the GRIPPER
        wmv_set<std::string>("gripper.action", "close");
        wmv_set<std::string>("gripper.target", target);
    }
    wm_unlock();
}

// 6. DEFINE the exit function.
//  this function executed only once at the end of the execution
void WSG50GraspBehavior::exit(){
    // write CUSTOM code here...
    std::cout<<arg(0)<<": exit() executed "<<std::endl;

    wm_lock();
    wmv_withdraw<std::string>("wsg50");
    //wmv_set<bool>(this->getInstance() + ".done", false);
    wm_unlock();
}



/* 
*  *******************************************************************************
*                                   WSG50 RELEASE
*  *******************************************************************************
*/


// REGISTER this behavior into the Behavior-based System (BBS).
//  This is a 2 steps process...

// 1. PROVIDE a unique name to this behavior
//  the behavior will be identified by this name
std::string WSG50ReleaseBehavior::behavior_name = "wsg50Release";

// 2. ADD this behavior (identified by the behavior_name) to the BBS for possible activation
bool WSG50ReleaseBehavior::registered = BehaviorBasedSystem::add(behavior_name,&WSG50ReleaseBehavior::create);

// ...now your behavior has been registered, you may recall it by loading
//  a predicate in the form "<behavior_name>(<Arg1>, <Arg2>, ..., <ArgN>)" to the WM




// DEFINE the code of this behavior.
//  This is a 6 steps process...

// 1. DEFINE the constructor for this beahvior, it will be executed when this behavior is recalled.
//
//      instance: is the string containing the predicate that has been used to invoke ths behavior.
//                NOTE: if no paramaters (i.e., further args) are given then instance == behavior_name
WSG50ReleaseBehavior::WSG50ReleaseBehavior(std::string instance){
    // please set the instance before anything else
    setInstance(instance);

    // write CUSTOM construction code here...
    //std::cout<<arg(0)<<": Constructor() executed "<<std::endl;

    target = arg(1);
}

// 2. DEFINE the create function.
//  this function MUST return the reference to the currente behavior.
Behavior *WSG50ReleaseBehavior::create(std::string instance){
    return new WSG50ReleaseBehavior(instance);
}


// now let's implement the CUSTOM core of our behavior (virtual functions).

// 3. DEFINE the start function.
//  this function executed only once at the beginning of the execution
void WSG50ReleaseBehavior::start(){
    // write CUSTOM code here...
    //std::cout<<arg(0)<<": start() executed "<<std::endl;
}

// 4. DEFINE the perceptualSchema.
//  this function is executed at the specified frequency.
//      If TRUE is returned, the motorSchema may be executed
bool WSG50ReleaseBehavior::perceptualSchema(){
    // write CUSTOM code here...
    //std::cout<<arg(0)<<": perceptualSchema() executed "<<std::endl;

    wm_lock();
    //subscribe this node to the competition
    wmv_compete<std::string>("wsg50Manager", "wsg50", this->getInstance());
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
void WSG50ReleaseBehavior::motorSchema(){
    // write CUSTOM code here...
    //std::cout<<arg(0)<<": motorSchema() executed "<<std::endl;
    wm_lock();
    // SELF-SOLVING CONTENTION
    if(wmv_solve_once<std::string>("wsg50") == this->getInstance()){
        // if here, I'm in control of the GRIPPER
        wmv_set<std::string>("gripper.action", "open");
        wmv_set<std::string>("gripper.target", target);
    }
    wm_unlock();
}

// 6. DEFINE the exit function.
//  this function executed only once at the end of the execution
void WSG50ReleaseBehavior::exit(){
    // write CUSTOM code here...
    std::cout<<arg(0)<<": exit() executed "<<std::endl;

    wm_lock();
    wmv_withdraw<std::string>("wsg50");
    //wmv_set<bool>(this->getInstance() + ".done", false);
    wm_unlock();
}




// ...enjoy