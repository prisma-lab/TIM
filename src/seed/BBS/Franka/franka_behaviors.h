#ifndef BEHAVIOR_FRANKA_H
#define BEHAVIOR_FRANKA_H

#include "seed.h"

// tf publisher
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/transform_broadcaster.h"

// tf subscriber
#include "tf2/exceptions.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"

//ROS services from UNITN to invoke Franka primitives
#include <chrono>
#include "inverse_msgs/srv/execute_skill.hpp"
#include "inverse_msgs/srv/point_to_point_motion.hpp"
#include "std_srvs/srv/trigger.hpp"  


//ROS actions for the franka gripper
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <franka_msgs/action/grasp.hpp>
#include <franka_msgs/action/move.hpp>

using namespace seed; //this is not needed to compile, byt most IDEs require it

using Grasp = franka_msgs::action::Grasp;
using Move = franka_msgs::action::Move;


// Function for conversion
inline double rad2deg(double radian) {
    double pi = 3.14159;
    return(radian * (180 / pi));
}

inline double deg2rad(double degree) {
    double pi = 3.14159;
    return(degree * pi / 180);
}

/* 
*  *******************************************************************************
*                                   FRANKA BEHAVIOR
*  *******************************************************************************
*/

class FrankaBehavior : public Behavior {
public:

    FrankaBehavior();

    inline geometry_msgs::msg::PoseStamped franka_subscribe_tf(std::string target_frame){

        geometry_msgs::msg::PoseStamped p;
        geometry_msgs::msg::TransformStamped t;

        //std::string start_frame = "iiwa_base_link";
        std::string start_frame = "franka_"+robot_name+"/base_link";
        
        try {
            //t = tf_buffer->lookupTransform(target_frame, start_frame, tf2::TimePointZero);
            t = tf_buffer->lookupTransform(start_frame, target_frame, tf2::TimePointZero);
            //t = tf_buffer->lookupTransform(target_frame, start_frame, nh->get_clock()->now(),rclcpp::Duration(1000000));
            std::cout<<arg(0)<<": TF FOUND"<<std::endl;

            p.header = t.header;
            p.pose.position.x = t.transform.translation.x;
            p.pose.position.y = t.transform.translation.y;
            p.pose.position.z = t.transform.translation.z;
            p.pose.orientation = t.transform.rotation;

            return p;
        } catch (const tf2::TransformException & ex) {
            std::cout<<arg(0)<<": unable to find transform from "<<start_frame<<" to "<<target_frame<<std::endl;
            return p;
        }
    }

protected:
    //Franka varaibles
    std::string robot_name;

    std::shared_ptr<tf2_ros::TransformListener> tf_listener{nullptr};
    std::unique_ptr<tf2_ros::Buffer> tf_buffer;
};

/* 
*  *******************************************************************************
*                                   FRANKA MANAGER
*  *******************************************************************************
*/

class FrankaManagerBehavior : public FrankaBehavior {
public:
    FrankaManagerBehavior(std::string instance);
    
    static Behavior *create(std::string instance);

    bool perceptualSchema();

    void motorSchema();

    void start();
    
    void exit();

    // functions invoking UNITN services
    bool franka_stop();

    bool franka_execute_skill(std::string skill_name, geometry_msgs::msg::PoseStamped init_p, geometry_msgs::msg::PoseStamped fin_p);

    bool franka_move_p2p(geometry_msgs::msg::PoseStamped init_p, geometry_msgs::msg::PoseStamped fin_p);

    //functions invoking Franka actions

    std::shared_future<bool> franka_open_gripper(double width, double velocity);

    std::shared_future<bool> franka_close_gripper(double width, double force, double velocity);

    inline void onGraspGoalResponse(
        const std::shared_ptr<std::promise<bool>> &promise,
        //std::shared_future<typename rclcpp_action::ClientGoalHandle<Grasp>::SharedPtr> future_handle) {
        std::shared_ptr<rclcpp_action::ClientGoalHandle<Grasp>> future_handle) {
        auto goal_handle = future_handle.get();
        if (!goal_handle)
        {
            promise->set_value(false);
            return;
        }
    }

    inline void onGraspResult(
        const std::shared_ptr<std::promise<bool>> &promise,
        //const typename rclcpp_action::ClientGoalHandle<Grasp>::WrappedResult &result) {
        rclcpp_action::ClientGoalHandle<Grasp>::WrappedResult result) {
        bool success = result.result->success;
        promise->set_value(success);
    }

    inline void onMoveGoalResponse(
        const std::shared_ptr<std::promise<bool>> &promise,
        //std::shared_future<typename rclcpp_action::ClientGoalHandle<Move>::SharedPtr> future_handle) {
        std::shared_ptr<rclcpp_action::ClientGoalHandle<Move>> future_handle) {
        auto goal_handle = future_handle.get();
        if (!goal_handle)
        {
            promise->set_value(false);
            return;
        }
    }

    inline void onMoveResult(
        const std::shared_ptr<std::promise<bool>> &promise,
        //const typename rclcpp_action::ClientGoalHandle<Move>::WrappedResult &result) {
        rclcpp_action::ClientGoalHandle<Move>::WrappedResult result) {
        bool success = result.result->success;
        promise->set_value(success);
    }

protected:
    //NOTE: this variable is used to self-register the class into the BBS
    //inline static bool registered = BehaviorBasedSystem::add("template",&TemplateBehavior::create); //this should be done in the .cpp
    static std::string behavior_name;
    static bool registered;
    std::string msg;

    std::string current_action;
    std::string new_action;

    //UNITN services
    rclcpp::Client<inverse_msgs::srv::ExecuteSkill>::SharedPtr client_skill;
    rclcpp::Client<inverse_msgs::srv::PointToPointMotion>::SharedPtr client_p2p;
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr client_stop;

    //Franka gripper actions
    rclcpp_action::Client<Grasp>::SharedPtr grasp_client_;
    rclcpp_action::Client<Move>::SharedPtr move_client_;
};


/* 
*  *******************************************************************************
*                                   FRANKA GO
*  *******************************************************************************
*/


class FrankaGoBehavior : public FrankaBehavior {
public:
    FrankaGoBehavior(std::string instance);
    
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

    
    std::string start_pos;
    std::string end_pos;

};



/* 
*  *******************************************************************************
*                                   FRANKA EXEC
*  *******************************************************************************
*/


class FrankaExecBehavior : public FrankaBehavior {
public:
    FrankaExecBehavior(std::string instance);
    
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

    std::string motion_name;
};



/* 
*  *******************************************************************************
*                                   FRANKA INSERT
*  *******************************************************************************
*/


class FrankaInsertBehavior : public FrankaBehavior {
public:
    FrankaInsertBehavior(std::string instance);
    
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

    std::string object_id, slot_id;
};






/* 
*  *******************************************************************************
*                                   FRANKA OPEN
*  *******************************************************************************
*/


class FrankaOpenBehavior : public FrankaBehavior {
public:
    FrankaOpenBehavior(std::string instance);
    
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

    std::string object_id, slot_id;
};




/* 
*  *******************************************************************************
*                                   FRANKA CLOSE
*  *******************************************************************************
*/


class FrankaCloseBehavior : public FrankaBehavior {
public:
    FrankaCloseBehavior(std::string instance);
    
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

    std::string object_id, slot_id;
};

#endif	/* BEHAVIOR_FRANKA_H */

