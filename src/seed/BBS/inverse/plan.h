#ifndef BEHAVIOR_INVPLAN_H
#define BEHAVIOR_INVPLAN_H

#include "seed.h"

#include "plansys2_msgs/msg/plan.hpp"
#include "plansys2_msgs/msg/plan_item.hpp"

#include "task_planner_msgs/msg/PlanningRequest.hpp"

using namespace seed; //this is not needed to compile, but most IDEs require it

enum Status { PLANNING, PLAN_SUCCESS, PLAN_FAILURE, EXECUTING, EXEC_SUCCESS, EXEC_FAILURE, IDLE };

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

    //rclcpp::Publisher<std_msgs::msg::String>::SharedPtr domain_pub_;
    //rclcpp::Publisher<std_msgs::msg::String>::SharedPtr problem_pub_;

    rclcpp::Publisher<task_planner_msgs::msg::PlanningRequest>::SharedPtr plan_pub_;

    rclcpp::Subscription<plansys2_msgs::msg::Plan>::SharedPtr plan_sub_;

    bool have_planning_request;
    Status status;

    std::vector<std::string> plan;
    std::string current_plan_id;
    int n_plan;

    std::string exec_instance;
    std::vector<WM_node *> exec_nodes;

    void plan_cb(const plansys2_msgs::msg::Plan::SharedPtr msg);

    //convert planned action (item) into executive action (instance) if needed
    std::string plan2exec(std::string);

    //create the planning domain (from KB, file or context)
    std::string create_plan_domain();

    //create the planning problem (from user's reqest, file or context)
    std::string create_plan_problem();
};

#endif	/* BEHAVIOR_INVPLAN_H */

