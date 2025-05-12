
// Server side implementation of UDP client-server model 
//#include <stdio.h>
#include <iostream> 
#include <sstream> 
#include <vector> 

#include <string> 

#include <stdlib.h> 
#include <unistd.h> 
#include <string.h> 
#include <sys/types.h> 
#include <sys/socket.h> 
#include <arpa/inet.h> 
#include <netinet/in.h> 

#include <signal.h>

#include <iostream>
#include <chrono>
#include <cmath>

#include <thread>
#include <mutex>

// seed namespace
//#include "../../../utils/seed_debug.h"
//seed::time namespace
//#include "../../../utils/seed_time.h"


/*
 * iiwa_ess_interface (check for compatibility with IIWA-side software)
 * 
 *  VERSION: 2.1
 * 
*/


class ServerSocket{
public:
    ServerSocket();

    bool connect(int in_port);

    bool disconnect();

    std::string read(int buff_size = 1024);

    int write(std::string str);

//private:

    int sd; 
    struct sockaddr_in servaddr, cliaddr;
    int cliaddr_len;

    int port;
};

enum class IIWA_control_mode{ NONE, JOINT_POSITION, CARTESIAN_POSITION, JOINT_IMPEDANCE, CARTESIAN_IMPEDANCE };
//NOTE: IIWA has no VELOCITY modes

typedef struct IIWA_state{
    std::vector<double> joint_position;
    std::vector<double> joint_torque;
    std::vector<double> cartesian_position;
    std::vector<double> cartesian_wrench;
} IIWA_state;


// IIWA EthernetSmartServo Interface
class IIWA_ess_interface {
public:
    IIWA_ess_interface();

    bool open_connection(int stat_port, int comm_port);

    inline bool open_connection(int port = 58080){
        return open_connection(port, port+1);
    }

    bool close_connection();

    //void th_communication(); //single thread is too slow!

    void th_command(); //out to IIWA

    void th_state(); //in from IIWA

    std::string pack_string(std::vector< std::vector<double> > in_vec);

    std::vector<double> unpack_string(std::string in_str);

    void update_state(std::vector<double> state_vec);


    inline bool isRunning(){
        bool app;
        lock();
        app = running;
        unlock();
        return app;
    }

    inline bool setControlMode(IIWA_control_mode cm){
        lock();
        control_mode = cm;
        unlock();
    };

    inline IIWA_control_mode getControlMode(){
        IIWA_control_mode app;
        lock();
        app = iiwa_control_mode;
        unlock();
        return app;
    }


    inline void setJointPosition(std::vector<double> target){
        lock();
        out_jp = target;
        unlock();
    };

    inline void setJointStiffness(std::vector<double> target){
        lock();
        out_js = target;
        unlock();
    };

    inline void setJointDumping(std::vector<double> target){
        lock();
        out_jd = target;
        unlock();
    };

    inline void setCartesianPosition(std::vector<double> target){
        lock();
        out_cp = target;
        unlock();
    };

    inline void setCartesianStiffness(std::vector<double> target){
        lock();
        out_cs = target;
        unlock();
    };

    inline void setCartesianDumping(std::vector<double> target){
        lock();
        out_cd = target;
        unlock();
    };

//    inline void setCartesianVelocity(std::vector<double> target){
//        lock();
//        out_cv = target;
//        unlock();
//    };

//    inline void setCartesianTorque(std::vector<double> target){
//        lock();
//        out_ct = target;
//        unlock();
//    };


    inline std::vector<double> getJointPosition(){
        return get_from_iiwa(in_jp);
    };


    inline std::vector<double> getJointTorque(){
        return get_from_iiwa(in_jt);
    };

    inline std::vector<double> getJointStiffness(){
        return get_from_iiwa(in_js);
    };

    inline std::vector<double> getJointDumping(){
        return get_from_iiwa(in_jd);
    };

    inline std::vector<double> getCartesianPosition(){
        return get_from_iiwa(in_cp);
        //reading_updated = false; //TO BE REMOVED - IS TO TEST THE SPEED OF COMMUNICATION
    };

    inline std::vector<double> getCartesianWrench(){
        return get_from_iiwa(in_cw);
    };

    inline std::vector<double> getCartesianStiffness(){
        return get_from_iiwa(in_cs);
    };

    inline std::vector<double> getCartesianDumping(){
        return get_from_iiwa(in_cd);
    };

    inline IIWA_state getFullState(){
        IIWA_state s;
        lock();
        have_request = true;
        while( have_request ){
            unlock();
            usleep(100); //be fast!
            lock();
        }
        s.joint_position = in_jp;
        s.joint_torque = in_jt;
        s.cartesian_position = in_cp;
        s.cartesian_wrench = in_cw;
        unlock();

        return s;
    };


    inline std::string plot(std::vector<double> vec){
        std::stringstream ss;
        for(auto i=0; i<vec.size(); i++)
            //ss<<" "<<roundec(vec[i],4);
	    ss<<" "<<vec[i];

        return ss.str();
    }

private:

    //this is a blocking function (CONDWAIT) asking IIWA for a single vector
    inline std::vector<double> get_from_iiwa(std::vector<double> &in_vec){
        std::vector<double> app;
        lock();
        have_request = true;
        in_vec = app;
        while( in_vec.empty() ){
            unlock();
            usleep(100); //be fast!
            lock();
        }
        app = in_vec;
        unlock();

        return app;
    }

    inline void lock(){
        this->m.lock();
        //std::cout<<name<<" locked"<<std::endl;
    }
    inline void unlock(){
        //std::cout<<name<<" unlocked"<<std::endl;
        this->m.unlock();
    }


    //thread for async IIWA communication
    std::mutex m;
    std::thread t_comm;
    std::thread t_stat;

    IIWA_control_mode control_mode = IIWA_control_mode::NONE;
    IIWA_control_mode iiwa_control_mode = IIWA_control_mode::NONE;

    bool running;

    bool connected;

    bool have_request = false;

    // --- socket stuff
    ServerSocket socket_comm;
    ServerSocket socket_stat;

    // ---
    

    //from USER to IIWA
    std::vector<double > out_jp; //joint pos
    std::vector<double > out_js; //joint stiffness
    std::vector<double > out_jd; //joint dumping

    std::vector<double > out_cp; //cart pos
    std::vector<double > out_cs; //cart stiffness
    std::vector<double > out_cd; //cart dumping


    //from IIWA to USER
    std::vector<double > in_jp; //joint pos
    std::vector<double > in_jt; //joint tor
    std::vector<double > in_js; //joint stiffness
    std::vector<double > in_jd; //joint dumping

    std::vector<double > in_cp; //cart pos
    std::vector<double > in_cw; //cart wrench
    std::vector<double > in_cs; //cart stiffness
    std::vector<double > in_cd; //cart dumping
};
