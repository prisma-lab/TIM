/*
 * WSG 50 ROS NODE
 * Copyright (c) 2012, Robotnik Automation, SLL
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
 *     * Neither the name of the Robotnik Automation, SLL. nor the names of its
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
 * \author Marc Benetó (mbeneto@robotnik.es)
 * \brief WSG-50 ROS driver.
 */


//------------------------------------------------------------------------
// Includes
//------------------------------------------------------------------------

#include "wsg_50/wsg50_eth.h"

//------------------------------------------------------------------------
// Function implementation
//------------------------------------------------------------------------

WSG50_eth::WSG50_eth(){
    g_timer_cnt = 0;
    g_ismoving = false;
    g_mode_script = false;
    g_mode_periodic = false; 
    g_mode_polling = false;
    g_goal_position = 0; 
    g_goal_speed = 0; 
    g_speed = 10.0;

	wsg50_min_width = -1;
}

bool WSG50_eth::open_connection(int in_port, std::string in_ip, bool use_udp){
    std::string ip, protocol, com_mode;
    int port, local_port;
    double rate; 
    //double grasping_force; //not set by default

    // original default parameters:
    // nh.param("ip", ip, std::string("192.168.1.20"));
    // nh.param("port", port, 1000);
    // nh.param("local_port", local_port, 1501);
    // nh.param("protocol", protocol, std::string(""));
    // nh.param("com_mode", com_mode, std::string(""));
    // nh.param("rate", rate, 1.0); // With custom script, up to 30Hz are possible
    // nh.param("grasping_force", grasping_force, 0.0);

    ip = in_ip;
    port = in_port;
    local_port = 1501;
    protocol = "";
    com_mode = "";
    rate = 1.0;
    //grasping_force = 0.0;

    if (protocol == "udp")
        use_udp = true;
    else
        protocol = "tcp";
    if (com_mode == "script")
        g_mode_script = true;
    else if (com_mode == "auto_update")
        g_mode_periodic = true;
    else
    {
        com_mode = "polling";
        g_mode_polling = true;
    }

    printf("WSG50: connecting to %s:%d (%s); communication mode: %s ...\n", ip.c_str(), port, protocol.c_str(), com_mode.c_str());

    // Connect to device using TCP/USP
    int res_con;
    if (!use_udp)
        res_con = cmd_connect_tcp(ip.c_str(), port);
    else
        res_con = cmd_connect_udp(local_port, ip.c_str(), port);

    if(res_con==0)
        printf("WSG50: connection established\n");
    else
        printf("WSG50: connection refused (error code %d)\n", res_con);

    return (res_con==0);
}

bool WSG50_eth::close_connection(){
    printf("WSG50: closing connection\n");
    g_mode_periodic = false;
    g_mode_script = false;
    g_mode_polling = false;
    //sleep(1);
    cmd_disconnect();
    printf("WSG50: connection closed\n");
    return true;
}

//ex bool moveSrv(wsg_50_common::Move::Request &req, wsg_50_common::Move::Response &res)
bool WSG50_eth::go_absolute_position(double req_width, double req_speed, int &res_error) 
{
	if(!check_width(req_width)) return false;

	if ( (req_width >= 0.0 && req_width <= 110.0) && (req_speed > 0.0 && req_speed <= 420.0) ){
  		printf("Moving to %f position at %f mm/s.\n", req_width, req_speed);
		res_error = move(req_width, req_speed, false);
	}else if (req_width < 0.0 || req_width > 110.0){
		printf("Imposible to move to this position. (Width values: [0.0 - 110.0] \n");
		res_error = 255;
		return false;
	}else{
	    printf("Speed values are outside the gripper's physical limits ([0.1 - 420.0])  Using clamped values.\n");
		res_error = move(req_width, req_speed, false);
	}

	printf("Target position reached.\n");
  	return true;
}

//ex incrementSrv(wsg_50_common::Incr::Request &req, wsg_50_common::Incr::Response &res)
bool WSG50_eth::go_relative_position(std::string req_direction, double req_increment)
{
	if (req_direction == "open"){

		if (!objectGraspped){

			float currentWidth = getOpening();
			float nextWidth = currentWidth + req_increment;

			if(!check_width(nextWidth)) return false;

			if ( (currentWidth < GRIPPER_MAX_OPEN) && nextWidth < GRIPPER_MAX_OPEN ){
				//grasp(nextWidth, 1);
				move(nextWidth,20, true);
				currentWidth = nextWidth;
			}else if( nextWidth >= GRIPPER_MAX_OPEN){
				//grasp(GRIPPER_MAX_OPEN, 1);
				move(GRIPPER_MAX_OPEN,1, true);
				currentWidth = GRIPPER_MAX_OPEN;
			}
		}else{
			printf("Releasing object...\n");
			release(GRIPPER_MAX_OPEN, 20);
			objectGraspped = false;
		}
	}else if (req_direction == "close"){

		if (!objectGraspped){

			float currentWidth = getOpening();
			float nextWidth = currentWidth - req_increment;

			if ( (currentWidth > GRIPPER_MIN_OPEN) && nextWidth > GRIPPER_MIN_OPEN ){
				//grasp(nextWidth, 1);
				move(nextWidth,20, true);
				currentWidth = nextWidth;
			}else if( nextWidth <= GRIPPER_MIN_OPEN){
				//grasp(GRIPPER_MIN_OPEN, 1);
				move(GRIPPER_MIN_OPEN,1, true);
				currentWidth = GRIPPER_MIN_OPEN;
			}
		}
	}
	return true;
}

//ex releaseSrv(wsg_50_common::Move::Request &req, wsg_50_common::Move::Response &res)
bool WSG50_eth::do_release(double req_width, double req_speed, int &res_error)
{
	if ( (req_width >= 0.0 && req_width <= 110.0) && (req_speed > 0.0 && req_speed <= 420.0) ){
  		printf("Releasing to %f position at %f mm/s.\n", req_width, req_speed);
		res_error = release(req_width, req_speed);
	}else if (req_width < 0.0 || req_width > 110.0){
		printf("Imposible to move to this position. (Width values: [0.0 - 110.0] \n");
		res_error = 255;
		return false;
	}else{
	        printf("Speed or position values are outside the gripper's physical limits (Position: [0.0 - 110.0] / Speed: [0.1 - 420.0])  Using clamped values.\n");
		res_error = release(req_width, req_speed);
	}
	printf("Object released correctly.\n");
  	return true;
}

//ex homingSrv(std_srvs::Empty::Request &req, std_srvs::Empty::Request &res)
bool WSG50_eth::go_home()
{
	printf("Homing...\n");
	homing();
	printf("Home position reached.\n");
	return true;
}

//ex stopSrv(std_srvs::Empty::Request &req, std_srvs::Empty::Request &res)
bool WSG50_eth::emergency_stop()
{
	printf("Stop!\n");
	stop();
	printf("Stopped.\n");
	return true;
}

//ex setAccSrv(wsg_50_common::Conf::Request &req, wsg_50_common::Conf::Response &res)
bool WSG50_eth::set_max_accelleration(double req_acc)
{
	setAcceleration(req_acc);
	return true;
}

//ex setForceSrv(wsg_50_common::Conf::Request &req, wsg_50_common::Conf::Response &res)
// NOTE force can be 0 or in [5.0,80.0]
bool WSG50_eth::set_grasping_force(double req_force)
{

	int res = setGraspingForceLimit(req_force);

	return res>=0;
}

//ex ackSrv(std_srvs::Empty::Request &req, std_srvs::Empty::Request &res)
bool WSG50_eth::send_ack()
{
	ack_fault();
	return true;
}



// additional functions

bool WSG50_eth::tare()
{
    doTare();
	return true;
}

double WSG50_eth::get_current_force(){
    return getForce(0);
}

double WSG50_eth::get_grasping_force(){
    return getGraspingForceLimit();
}


const char * WSG50_eth::get_state(){
    return systemState();
}

void WSG50_eth::set_min_width(double in_w){
	wsg50_min_width = in_w;
}

bool WSG50_eth::do_grasp(double req_width, double req_speed){
	if(!check_width(req_width)) return false;

	return grasp(req_width,req_speed)>=0;
}

double WSG50_eth::get_position(){
	return getOpening(0);
}

/*
 NOTE:
  there are additional functions from the original ROS1 Driver that have not been ported
  you can find their original implementation in the main.cpp file. 
   Here a list:

    void position_cb(const wsg_50_common::Cmd::ConstPtr& msg)
    
    void speed_cb(const std_msgs::Float32::ConstPtr& msg)

    void timer_cb(const ros::TimerEvent& ev)

    void read_thread(int interval_ms)

    void sigint_handler(int sig)
*/