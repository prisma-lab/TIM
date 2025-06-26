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
    //domain_pub_ = nh->create_publisher<std_msgs::msg::String>("planning_domain", 10);
    //problem_pub_ = nh->create_publisher<std_msgs::msg::String>("planning_problem", 10);
    plan_pub_ = this->create_publisher<task_planner_msgs::msg::PlanningRequest>("/plan_request", 10);

    // Subscriber to the plan
    plan_sub_ = nh->create_subscription<plansys2_msgs::msg::Plan>(
      "/plan_response", 10,
      std::bind(&InvPlanBehavior::plan_cb, this, std::placeholders::_1)
    );

    status = Status::IDLE;

    n_plan = 0;

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

    if(have_planning_request && status == Status::IDLE){
        //start new planning process
        
        //send planning domain/problem

        //auto domain_msg = std_msgs::msg::String();
        //domain_msg.data = create_plan_domain();

        //auto problem_msg = std_msgs::msg::String();
        //problem_msg.data = create_plan_problem();

        auto msg = task_planner_msgs::msg::PlanningRequest();

        // set header too
        msg.domain = create_plan_domain();
        msg.problem = create_plan_problem();

        std::cout<<arg(0)<<", sending domain: "<<msg.domain<<std::endl;
        std::cout<<arg(0)<<", sending problem: "<<msg.problem<<std::endl;

        //domain_pub_->publish(domain_msg);
        //problem_pub_->publish(problem_msg);

        plan_pub_->publish(msg);

        status = Status::PLANNING;
    }
    else if(status == Status::PLAN_SUCCESS){
        //start execution of plan

        n_plan++;
        current_plan_id = "plan" + std::to_string(n_plan);

        std::stringstream ss;
        ss<<"softSequence([";
        for(auto i=0; i<plan.size()-1; i++){
            ss<<plan[i]<<",";
        }
        ss<<plan[plan.size()-1]<<"],"<<current_plan_id<<")";

        exec_instance = ss.str();

        wm_lock();
        std::cout<<arg(0)<<", loading new plan: "<<std::endl;
        std::cout<<"\t "<<exec_instance<<std::endl;
        exec_nodes = wm_add_child_node(exec_instance);
        wm_unlock();

        status = Status::EXECUTING;
    }
    else if(status == Status::PLAN_FAILURE){
        //invoke teaching or learning process

        std::cout<<arg(0)<<", planning FAILED! learning/teaching phase should be invoked"<<std::endl;

        status = Status::IDLE;
    }
    else if(status == Status::EXECUTING){
        //monitor execution

        wm_lock();
        
        if(!exec_nodes[0]->isWorking()){
            if(exec_nodes[0]->goalStatus()){
                status = Status::EXEC_SUCCESS;
            }
            //otherwise, still executing
        }
        else{
            status = Status::EXEC_FAILURE;
        }

        wm_unlock();
    }
}

// 6. DEFINE the exit function.
//  this function executed only once at the end of the execution
void InvPlanBehavior::exit(){
    // write CUSTOM code here...
    std::cout<<arg(0)<<": exit() executed "<<std::endl;
}

void InvPlanBehavior::plan_cb(const plansys2_msgs::msg::Plan::SharedPtr msg) {
    RCLCPP_INFO(nh->get_logger(), "Received plan with %zu actions:", msg->items.size());

    if(status != Status::PLANNING)
        return;

    if( msg->items.size() == 0)
        status = Status::PLAN_FAILURE;
    else
        plan.clear();

    for (const auto & item : msg->items) {
      std::stringstream ss;
      ss << item.time << ":\t" << item.action << " [" << item.duration << "s]";
      RCLCPP_INFO(nh->get_logger(), "%s", ss.str().c_str());

      plan.push_back(plan2exec(item.action));

      status = Status::PLAN_SUCCESS;
    }
  }


std::string InvPlanBehavior::plan2exec(std::string plan_act){
    //TBD depending on the planning domain,
    //  for now just return the action assuming that plan and exec representations are compatible

    //transform PDDL operator into SEED behavior
    //  e.g. (pickup a) -> pickup(a)
    std::string s = "pddl" + plan_act;
    std::replace( s.begin(), s.end(), ' ', ',');
    std::vector<std::string> pddl_v = instance2vector(plan_act);
    std::stringstream ss;

    ss<<pddl_v[1];
    //if we have arguments
    if(pddl_v.size>2){
        ss<<"(";
        for(auto i=2; i<pddl_v.size();i++){
            ss<<pddl_v[i];
            if(i+1 == pddl_v.size())
                ss<<")";
            else
                ss<<",";
        }
    }

    return ss.str();
}


std::string InvPlanBehavior::create_plan_domain(){
    //TBD, for now I would take it from file
    std::ifstream file(SEED_HOME_PATH + "/BBS/inverse/domains/example_blocksworld_domain.pddl");
    
    if (!file) {
        std::cout<<"Unable to open DOMAIN file"<<std::endl;
    }

    std::ostringstream content;
    content << file.rdbuf();
    
    return content.str();
}


std::string InvPlanBehavior::create_plan_problem(){
    //TBD, for now I would take it from file

    std::ifstream file(SEED_HOME_PATH + "/BBS/inverse/domains/example_blocksworld_problem.pddl");
    
    if (!file) {
        std::cout<<"Unable to open PROBLEM file"<<std::endl;

    }

    std::ostringstream content;
    content << file.rdbuf();
    
    return content.str();
}

// ...enjoy