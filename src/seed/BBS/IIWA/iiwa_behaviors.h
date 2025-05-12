#ifndef BEHAVIOR_IIWA_H
#define BEHAVIOR_IIWA_H

#include "seed.h"
#include "iiwa_ess_interface/iiwa_ess_interface.h"

// tf publisher
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/transform_broadcaster.h"

// tf subscriber
#include "tf2/exceptions.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"

using namespace seed; //this is not needed to compile, byt most IDEs require it


//NOTE: 
//  lin_dist is in mm
//  ang_dist is in rad
inline bool cartesian_pose_reached(std::vector<double> cp, std::vector<double> tp, double desired_lin_dist = 5, double desired_ang_dist = 0.05){
    for(auto i=0; i<3; i++){
        if(std::abs(cp[i] - tp[i]) > desired_lin_dist)
            return false;
    }
    for(auto i=3; i<6; i++){
        //if(std::abs(radpi(cp[i]) - radpi(tp[i])) > desired_ang_dist) //this clearly needs to be a ring
        radpi ca = cp[i];
        radpi ta = tp[i];
        if(std::abs(ca - ta) > desired_ang_dist)
            return false;
    }
    return true;
}

inline bool joint_pose_reached(std::vector<double> cp, std::vector<double> tp, double desired_joint_dist = 0.01){
    for(auto i=0; i<7; i++){
        if(std::abs(cp[i] - tp[i]) > desired_joint_dist)
            return false;
    }
    return true;
}


inline void plot_cartesian_difference(std::vector<double> cp, std::vector<double> tp){
    std::cout<<"cp: [";
    for(auto i=0; i<6; i++)
        std::cout<<cp[i]<<" ";
    std::cout<<"]"<<std::endl;

    std::cout<<"tp: [";
    for(auto i=0; i<6; i++)
        std::cout<<tp[i]<<" ";
    std::cout<<"]"<<std::endl;

    std::cout<<"diff: [";
    for(auto i=0; i<6; i++)
        std::cout<<tp[i]-cp[i]<<" ";
    std::cout<<"]"<<std::endl;

    std::cout<<"diff (radpi): [";
    for(auto i=0; i<6; i++)
        if(i<3)
            std::cout<<tp[i]-cp[i]<<" ";
        else
            std::cout<<radpi(tp[i])-radpi(cp[i])<<" ";

    std::cout<<"diff (radpi, plotted as deg): [";
    for(auto i=0; i<6; i++)
        if(i<3)
            std::cout<<tp[i]-cp[i]<<" ";
        else
            std::cout<<deg180(radpi(tp[i])-radpi(cp[i]))<<" ";
    std::cout<<"]"<<std::endl;
}


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
*                                   IIWA MANAGER
*  *******************************************************************************
*/

class IIWAManagerBehavior : public Behavior {
public:
    IIWAManagerBehavior(std::string instance);
    
    static Behavior *create(std::string instance);

    bool perceptualSchema();

    void motorSchema();

    void start();
    
    void exit();

    //custom functions

    std::string iiwa_reach_cartesian_pose();

    std::string iiwa_execute_cartesian_motion();

    void iiwa_publish_tf(std::string base_frame = "iiwa_base_link");

    // n_max_loop: is the number of loops we can afford to wait the robot in reaching a target pose.
    //      If this number is reached, the "jumping procedure" is attempted, i.e., robot tries to reach the 
    //      NEXT pose in the trajectory and to skip the impasse
    // n_max_jumps: is the maximum number of jumps allowed.
    //      If this number is reached, the execution is ABORTED
    inline void iiwa_initialize_motion(int n_max_loops = 100, int n_max_jumps = 1){
        motion_i = 0;

        motion_n_max_jumps = n_max_jumps;
        motion_n_max_loops = n_max_loops; //1sec (it depends on rate!)
        motion_n_jumps = 0;
        motion_n_loops = 0;
    }

    //helping functions

    inline void plot_motion(){
        for(size_t i=0; i<current_motion.size(); i++){
            std::cout<<i<<":";
            for(size_t j=0; j<current_motion[i].size(); j++)
                std::cout<<current_motion[i][j]<<" ";
            std::cout<<std::endl;
        }
    }

    inline IIWA_control_mode string2control(std::string ctrl_string){
        if(ctrl_string == "cartesian_impedance")
            return IIWA_control_mode::CARTESIAN_IMPEDANCE;
        else if(ctrl_string == "joint_position")
            return IIWA_control_mode::JOINT_POSITION;
        else if(ctrl_string == "joint_impedance")
            return IIWA_control_mode::JOINT_IMPEDANCE;
        else
            return IIWA_control_mode::CARTESIAN_POSITION;
        }

    inline std::string control2string(IIWA_control_mode ctrl_mode){
        if(ctrl_mode == IIWA_control_mode::CARTESIAN_IMPEDANCE)
            return "cartesian_impedance";
        else if(ctrl_mode == IIWA_control_mode::JOINT_POSITION)
            return "joint_position";
        else if(ctrl_mode == IIWA_control_mode::JOINT_IMPEDANCE)
            return "joint_impedance";
        else
            return "cartesian_position";
    }

protected:
    //NOTE: this variable is used to self-register the class into the BBS
    //inline static bool registered = BehaviorBasedSystem::add("template",&TemplateBehavior::create); //this should be done in the .cpp
    static std::string behavior_name;
    static bool registered;
    std::string msg;

    //IIWA varaibles
    IIWA_ess_interface iiwa_interface;
    IIWA_state current_state;
    std::string current_action;
    std::string current_mode;
    std::vector<std::vector<double>> current_motion;
    std::vector<double> steady_pose;

    //IIWA check early stop
    int stop_counter;
    std::vector<double> old_pose;

    //IIWA motion variables

    int motion_i;
    //jumping procedure (created to prevent iiwa to stuck due to impedance)
    int motion_n_max_jumps;
    int motion_n_max_loops; //1sec
    int motion_n_jumps;
    int motion_n_loops;

    bool is_motion_continuous;

    //tf puiblisher
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster;
};


/* 
*  *******************************************************************************
*                                   IIWA GO
*  *******************************************************************************
*/


class IIWAGoBehavior : public Behavior {
public:
    IIWAGoBehavior(std::string instance);
    
    static Behavior *create(std::string instance);

    bool perceptualSchema();

    void motorSchema();

    void start();
    
    void exit();

    std::vector<double> iiwa_subscribe_tf(std::string target_frame);

    inline std::vector<double> iiwa_safety_check(std::vector<double> target_pose){
        std::vector<double> new_pose = target_pose;
/*
        //check if the orientation of the z axis is too high!
        if(std::abs(deg180(rtod(target_pose[3]))) < 90 && std::abs(deg180(rtod(target_pose[4]))) < 90){
            std::cout<<"IIWA-SAFETY CHECK: WARNING, Z-axis is too low"<<std::endl;
            //let's try adjusting just one component (minimum adjustment)
            if( std::abs(target_pose[3]) > std::abs(target_pose[4]) ){
                new_pose[3] = new_pose[3]/std::abs(new_pose[3]) * dtor(90);
                std::cout<<"\t adjusting roll from "<< deg180(rtod(target_pose[3])) <<" to "<<deg180(rtod(new_pose[3]))<<std::endl;
            }
            else{
                new_pose[4] = new_pose[4]/std::abs(new_pose[4]) * dtor(90);
                std::cout<<"\t adjusting pitch from "<< deg180(rtod(target_pose[4])) <<" to "<<deg180(rtod(new_pose[4]))<<std::endl;
            }
            
        }
    */

        tf2::Quaternion q1;
        //NOTE: IIWA uses YPR!
        q1.setRPY(target_pose[5], target_pose[4],  target_pose[3]);
        tf2::Transform t1(q1);

        tf2::Quaternion q2;
        q2.setRPY(0, 0, 0);
        tf2::Transform t2(q2);

        tf2::Vector3 zPoint(0,0,1);

        tf2::Vector3 otherZPoint = t1.inverseTimes(t2) * zPoint;

        float angle = acos(zPoint.dot(otherZPoint) );

        std::cout<<ansi::cyan<<"IIWA-GO, Z ANGLE IS "<<radpi(angle)<<ansi::end<<std::endl;

        if(std::abs(radpi(angle)) < 1.57){
            std::cout<<ansi::yellow<<"WARNING, ANGLE IS TOO HIGH!"<<ansi::end<<std::endl;

            //adjust frame so that the angle is 1.57

            double sgn = 1;
            if(angle != 0) 
                sgn = std::abs(angle)/angle;

            double ang_diff = sgn * (1.57 - std::abs(radpi(angle)));

            std::cout<<ansi::yellow<<"DIFF is "<<ang_diff<<ansi::end<<std::endl;

            // FROM ALE

            tf2::Vector3 v = otherZPoint.cross(zPoint);

            tf2::Quaternion qt;
            qt.setRotation(v,ang_diff);

            tf2::Quaternion q_new = q1 * qt;

            double nr,np,ny;

            tf2::Matrix3x3 m(q_new);
            m.getRPY(nr, np, ny);

/*
            tf2::Vector3 xyPoint(1,1,0);
            tf2::Vector3 xyAxis = t1.inverseTimes(t2) * xyPoint;

            //q1.setRotation(xyAxis,ang_diff);
            q1.setRotation(otherZPoint,ang_diff);

            double nr,np,ny;

            tf2::Matrix3x3 m(q1);
            m.getRPY(nr, np, ny);
*/

            std::cout<<ansi::yellow<<"OLD RPY: "<<target_pose[5]<<", "<< target_pose[4]<<", " << target_pose[3]<<ansi::end<<std::endl;
            std::cout<<ansi::yellow<<"NEW RPY: "<<nr<<", "<<np<<", " <<ny<<ansi::end<<std::endl;

            new_pose[5] = nr;
            new_pose[4] = np;
            new_pose[3] = ny;

        }

        return new_pose;

    };

protected:
    //NOTE: this variable is used to self-register the class into the BBS
    //inline static bool registered = BehaviorBasedSystem::add("template",&TemplateBehavior::create); //this should be done in the .cpp
    static std::string behavior_name;
    static bool registered;
    std::string msg;

    std::string start_pos;
    std::string end_pos;

    std::string frame_name;
    bool have_goal;
    std::vector<std::vector<double>> cartesian_goal;

    //tf subscriber
    std::shared_ptr<tf2_ros::TransformListener> tf_listener{nullptr};
    std::unique_ptr<tf2_ros::Buffer> tf_buffer;
};



/* 
*  *******************************************************************************
*                                   IIWA TEACH
*  *******************************************************************************
*/


class IIWATeachBehavior : public Behavior {
public:
    IIWATeachBehavior(std::string instance);
    
    static Behavior *create(std::string instance);

    bool perceptualSchema();

    void motorSchema();

    void start();
    
    void exit();

    // Function for conversion
    inline double rad2deg(double radian) {
        double pi = 3.14159;
        return(radian * (180 / pi));
    }

    inline double deg2rad(double degree) {
        double pi = 3.14159;
        return(degree * pi / 180);
    }

protected:
    //NOTE: this variable is used to self-register the class into the BBS
    //inline static bool registered = BehaviorBasedSystem::add("template",&TemplateBehavior::create); //this should be done in the .cpp
    static std::string behavior_name;
    static bool registered;
    std::string msg;

    std::string motion_name;
    bool teaching_started;
    bool teaching_stopped;
    std::vector<double> initial_pose;
    std::vector<std::vector<double>> cartesian_goals;
};



/* 
*  *******************************************************************************
*                                   IIWA EXE
*  *******************************************************************************
*/


class IIWAExeBehavior : public Behavior {
public:
    IIWAExeBehavior(std::string instance);
    
    static Behavior *create(std::string instance);

    bool perceptualSchema();

    void motorSchema();

    void start();
    
    void exit();

    void load_motion();

    inline std::vector<double> get_frame_from_db(std::string frame){
        std::ifstream file (SEED_HOME_PATH + "/BBS/IIWA/DB/frames.txt");
        std::string frame_name;
        if (file.is_open()) {
            std::string line;
            while (std::getline(file, line)){
                std::istringstream ss(line);
                double fx, fy, fz, fa, fb, fc;
                if(ss>>frame_name>>fx>>fy>>fz>>fa>>fb>>fc && frame_name == frame){
                    std::vector<double> c_goal = {fx,fy,fz,fa,fb,fc};
                    file.close();
                    return c_goal;
                }
            }
            file.close();
        }
        return std::vector<double>();
    }

protected:
    //NOTE: this variable is used to self-register the class into the BBS
    //inline static bool registered = BehaviorBasedSystem::add("template",&TemplateBehavior::create); //this should be done in the .cpp
    static std::string behavior_name;
    static bool registered;
    std::string msg;

    std::string motion_name;
    bool have_motion;
    std::vector<std::vector<double>> cartesian_motion;
    bool motion_delivered;
};



/* 
*  *******************************************************************************
*                                   IIWA INSERT
*  *******************************************************************************
*/


class IIWAInsertBehavior : public Behavior {
public:
    IIWAInsertBehavior(std::string instance);
    
    static Behavior *create(std::string instance);

    bool perceptualSchema();

    void motorSchema();

    void start();
    
    void exit();

    std::vector<double> iiwa_subscribe_tf(std::string target_frame);

protected:
    //NOTE: this variable is used to self-register the class into the BBS
    //inline static bool registered = BehaviorBasedSystem::add("template",&TemplateBehavior::create); //this should be done in the .cpp
    static std::string behavior_name;
    static bool registered;
    std::string msg;

    std::string object_id;
    std::string slot_id;

    std::vector<double> cartesian_target;
    std::vector<double> object_pose;
    Random rnd;

    bool have_goal;

    //tf subscriber
    std::shared_ptr<tf2_ros::TransformListener> tf_listener{nullptr};
    std::unique_ptr<tf2_ros::Buffer> tf_buffer;
};




/* 
*  *******************************************************************************
*                                   IIWA EXTRACT
*  *******************************************************************************
*/

class IIWAExtractBehavior : public Behavior {
public:
    IIWAExtractBehavior(std::string instance);
    
    static Behavior *create(std::string instance);

    bool perceptualSchema();

    void motorSchema();

    void start();
    
    void exit();

    std::vector<double> iiwa_subscribe_tf(std::string target_frame);

protected:
    //NOTE: this variable is used to self-register the class into the BBS
    //inline static bool registered = BehaviorBasedSystem::add("template",&TemplateBehavior::create); //this should be done in the .cpp
    static std::string behavior_name;
    static bool registered;
    std::string msg;

    std::string object_id;
    std::string slot_id;

    std::vector<double> cartesian_target;
    std::vector<double> object_pose;
    Random rnd;

    bool have_goal;

    double fixed_target_z;

    //tf subscriber
    std::shared_ptr<tf2_ros::TransformListener> tf_listener{nullptr};
    std::unique_ptr<tf2_ros::Buffer> tf_buffer;
};




/* 
*  *******************************************************************************
*                                   IIWA WRITE
*  *******************************************************************************
*/


class IIWAWriteBehavior : public Behavior {
public:
    IIWAWriteBehavior(std::string instance);
    
    static Behavior *create(std::string instance);

    bool perceptualSchema();

    void motorSchema();

    void start();
    
    void exit();

    std::vector<double> iiwa_subscribe_tf(std::string target_frame);

protected:
    //NOTE: this variable is used to self-register the class into the BBS
    //inline static bool registered = BehaviorBasedSystem::add("template",&TemplateBehavior::create); //this should be done in the .cpp
    static std::string behavior_name;
    static bool registered;
    std::string msg;

    std::vector<std::string> characters;
    int m_x;
    int m_y;
    int current_c;
};




#endif	/* BEHAVIOR_IIWA_H */

