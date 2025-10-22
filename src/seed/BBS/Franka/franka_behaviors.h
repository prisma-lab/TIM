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

using namespace seed; //this is not needed to compile, byt most IDEs require it


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

FrankaBehavior(std::string instance);

inline std::vector<double> franka_subscribe_tf(std::string target_frame){

    geometry_msgs::msg::TransformStamped t;

    //std::string start_frame = "iiwa_base_link";
    std::string start_frame = "franka_"+robot_name+"/base_link";
    
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

protected:
    //NOTE: this variable is used to self-register the class into the BBS
    //inline static bool registered = BehaviorBasedSystem::add("template",&TemplateBehavior::create); //this should be done in the .cpp
    static std::string behavior_name;
    static bool registered;
    std::string msg;

    std::string current_action;
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

