#ifndef BEHAVIOR_VISION_H
#define BEHAVIOR_VISION_H

#include "seed.h"

#include <algorithm>

using namespace seed; //this is not needed to compile, byt most IDEs require it


/* 
*  *******************************************************************************
*                                   VISION STREAM
*  *******************************************************************************
*/

class VisionStreamBehavior : public Behavior {
public:
    VisionStreamBehavior(std::string instance);
    
    static Behavior *create(std::string instance);

    bool perceptualSchema();

    void motorSchema();

    void start();
    
    void exit();

    //custom functions

    inline void vision_callback(const std_msgs::msg::String::SharedPtr msg){
        vision_msg = msg->data;
    }

    std::string vision2schema(std::string v_str);


    std::vector<double> iiwa_subscribe_tf(std::string target_frame);

protected:
    //NOTE: this variable is used to self-register the class into the BBS
    static std::string behavior_name;
    static bool registered;
    std::string msg;

    //custom varaibles

    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sb_vision;


    std::string vision_msg;
    std::string old_vision_msg;
};

#endif	/* BEHAVIOR_VISION_H */

