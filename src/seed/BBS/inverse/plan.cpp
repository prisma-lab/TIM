/* 
 * Author: RC
 *
 * First created: 9-feb-2024 (SEED 7.0)
 * 
 */

#include "plan.h"

using namespace seed; //this is not needed to compile, but most IDEs require it


// REGISTER this behavior into the Behavior-based System (BBS).
//  This is a 2 steps process...

// 1. PROVIDE a unique name to this behavior
//  the behavior will be identified by this name
std::string InvPlanBehavior::behavior_name = "plan";

// 2. ADD this behavior (identified by the behavior_name) to the BBS for possible activation
bool InvPlanBehavior::registered = BehaviorBasedSystem::add(behavior_name,&InvPlanBehavior::create);

// ...now your behavior has been registered, you may recall it by loading
//  a predicate in the form "<behavior_name>(<Arg1>, <Arg2>, ..., <ArgN>)" to the WM



// DEFINE the code of this behavior.
//  This is a 6 steps process...

// 1. DEFINE the constructor for this beahvior, it will be executed when this behavior is recalled.
//
//      instance: is the string containing the predicate that has been used to invoke ths behavior.
//                NOTE: if no paramaters (i.e., further args) are given then instance == behavior_name
InvPlanBehavior::InvPlanBehavior(std::string instance){
    // please set the instance before anything else
    setInstance(instance);

    // write CUSTOM construction code here...

    //NOTE: These are strings for now, we must adjust them once the official type is selected
    domain_pub_ = nh->create_publisher<std_msgs::msg::String>("planning_domain", 10);
    problem_pub_ = nh->create_publisher<std_msgs::msg::String>("planning_problem", 10);

    // Subscriber to the plan
    plan_sub_ = nh->create_subscription<plansys2_msgs::msg::Plan>(
      "/plan", 10,
      std::bind(&InvPlanBehavior::plan_cb, this, std::placeholders::_1)
    );

    std::cout<<arg(0)<<": Constructor() executed "<<std::endl;
}

// 2. DEFINE the create function.
//  this function MUST return the reference to the currente behavior.
Behavior *InvPlanBehavior::create(std::string instance){
    return new InvPlanBehavior(instance);
}


// now let's implement the CUSTOM core of our behavior (virtual functions).

// 3. DEFINE the start function.
//  this function executed only once at the beginning of the execution
void InvPlanBehavior::start(){
    // write CUSTOM code here...
    std::cout<<arg(0)<<": start() executed "<<std::endl;
}

// 4. DEFINE the perceptualSchema.
//  this function is executed at the specified frequency.
//      If TRUE is returned, the motorSchema may be executed
bool InvPlanBehavior::perceptualSchema(){
    // write CUSTOM code here...

    if(have_planning_request){
        //send planning domain/problem

        auto domain_msg = std_msgs::msg::String();
        domain_msg.data = 
        "(define (domain demo)\n"
        "  (:requirements :strips :typing)\n"
        "  (:types robot location)\n"
        "  (:predicates\n"
        "    (at ?r - robot ?l - location)\n"
        "    (connected ?l1 - location ?l2 - location))\n"
        "  (:action move\n"
        "    :parameters (?r - robot ?from - location ?to - location)\n"
        "    :precondition (and (at ?r ?from) (connected ?from ?to))\n"
        "    :effect (and (not (at ?r ?from)) (at ?r ?to))))";

        auto problem_msg = std_msgs::msg::String();
        problem_msg.data = 
        "(define (problem move_robot)\n"
        "  (:domain demo)\n"
        "  (:objects robot1 - robot loc1 loc2 - location)\n"
        "  (:init (at robot1 loc1) (connected loc1 loc2) (connected loc2 loc1))\n"
        "  (:goal (at robot1 loc2)))";

        domain_pub_->publish(domain_msg);
        problem_pub_->publish(problem_msg);
    }

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
void InvPlanBehavior::motorSchema(){
    // write CUSTOM code here...
    //std::cout<<arg(0)<<": motorSchema() executed "<<std::endl;
}

// 6. DEFINE the exit function.
//  this function executed only once at the end of the execution
void InvPlanBehavior::exit(){
    // write CUSTOM code here...
    std::cout<<arg(0)<<": exit() executed "<<std::endl;
}

void InvPlanBehavior::plan_cb(const plansys2_msgs::msg::Plan::SharedPtr msg) {
    RCLCPP_INFO(nh->get_logger(), "Received plan with %zu actions:", msg->items.size());

    for (const auto & item : msg->items) {
      std::stringstream ss;
      ss << item.time << ":\t" << item.action << " [" << item.duration << "s]";
      RCLCPP_INFO(nh->get_logger(), "%s", ss.str().c_str());
    }
  }

// ...enjoy