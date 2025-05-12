/*
 * WSG 50 ETH API
 * Copyright (c) 2024, Riccardo Caccavale
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the author Riccardo Caccavale, nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 * \author Riccardo Caccavale (riccardo.caccavale@unina.it)
 * \brief WSG 50 ETH API.
 */


//------------------------------------------------------------------------
// Includes
//------------------------------------------------------------------------


#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <assert.h>
#include <thread>
#include <chrono>

#include <unistd.h>
#include <iostream>


#include "wsg_50/common.h"
#include "wsg_50/cmd.h"
#include "wsg_50/msg.h"
#include "wsg_50/functions.h"

//#include "wsg_50_common/Status.h"
//#include "wsg_50_common/Move.h"
//#include "wsg_50_common/Conf.h"
//#include "wsg_50_common/Incr.h"
//#include "wsg_50_common/Cmd.h"

// ROS1 functions
//#include <ros/ros.h>
//#include "std_msgs/String.h"
//#include "std_srvs/Empty.h"
//#include "sensor_msgs/JointState.h"
//#include "std_msgs/Float32.h"
//#include "std_msgs/Bool.h"



//------------------------------------------------------------------------
// Typedefs, enums, structs
//------------------------------------------------------------------------

#define GRIPPER_MAX_OPEN 110.0
#define GRIPPER_MIN_OPEN 0.0

//------------------------------------------------------------------------
// Class definition
//------------------------------------------------------------------------

class WSG50_eth{
public:
    WSG50_eth();

    bool open_connection(int in_port, std::string in_ip, bool use_udp=false);

    bool close_connection();
    
    bool go_absolute_position(double req_width, double req_speed, int &res_error);

    bool go_relative_position(std::string req_direction, double req_increment);

    bool do_release(double req_width, double req_speed, int &res_error);

    bool go_home();

    bool emergency_stop();

    bool set_max_accelleration(double req_acc);

    bool set_grasping_force(double req_force);

    bool send_ack();

    // additional functions (not provided in the original main.cpp file)

    bool tare();

    double get_current_force();

    double get_grasping_force();

    const char * get_state();

    void set_min_width(double in_w);

    bool do_grasp(double req_width, double req_speed);

    double get_position();

private:

    bool check_width(double req_w){
        if(wsg50_min_width < 0){
            printf("WSG50: ERROR, you must set_min_width before to move the gripper\n");
            return false;
        }
        else if(req_w < wsg50_min_width){
            printf("WSG50: ERROR, requested position (%f) is lower than min_width (%f)\n",req_w,wsg50_min_width);
            return false;
        }
        return true;
    };

    //float increment; //seems to be not used
    bool objectGraspped;

    // initialized on construction
    int g_timer_cnt;
    bool g_ismoving;
    bool g_mode_script;
    bool g_mode_periodic; 
    bool g_mode_polling;
    float g_goal_position; 
    float g_goal_speed; 
    float g_speed;

    //additional attributes
    double wsg50_min_width;
};


