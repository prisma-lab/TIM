#ifndef BEHAVIOR_INVPLAN_H
#define BEHAVIOR_INVPLAN_H

#include "seed.h"

#include "plansys2_msgs/msg/plan.hpp"
#include "plansys2_msgs/msg/plan_item.hpp"

using namespace seed; //this is not needed to compile, byt most IDEs require it

class InvPlanBehavior : public Behavior {
public:
    InvPlanBehavior(std::string instance);
    
    static Behavior *create(std::string instance);

    bool perceptualSchema();

    void motorSchema();

    void start();
    
    void exit();

protected:
    //NOTE: this variable is used to self-register the class into the BBS
    //inline static bool registered = BehaviorBasedSystem::add("template",&TemplateBehavior::create); //this should be done in the .cpp
    static std::string behavior_name;
    static bool registered;
    std::string msg;

    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr domain_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr problem_pub_;
    
    rclcpp::Subscription<plansys2_msgs::msg::Plan>::SharedPtr plan_sub_;

    bool have_planning_request;
    bool is_plan_running;

    void plan_cb(const plansys2_msgs::msg::Plan::SharedPtr msg);
};

#endif	/* BEHAVIOR_INVPLAN_H */

