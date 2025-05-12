#ifndef BEHAVIOR_WSG50_H
#define BEHAVIOR_WSG50_H

#include "seed.h"
#include "wsg_50/wsg50_eth.h"

using namespace seed; //this is not needed to compile, byt most IDEs require it



/* 
*  *******************************************************************************
*                                   WSG50 MANAGER
*  *******************************************************************************
*/

class WSG50ManagerBehavior : public Behavior {
public:
    WSG50ManagerBehavior(std::string instance);
    
    static Behavior *create(std::string instance);

    bool perceptualSchema();

    void motorSchema();

    void start();
    
    void exit();

    //custom functions


protected:
    //NOTE: this variable is used to self-register the class into the BBS
    //inline static bool registered = BehaviorBasedSystem::add("template",&TemplateBehavior::create); //this should be done in the .cpp
    static std::string behavior_name;
    static bool registered;
    std::string msg;

    //WSG50 varaibles
    WSG50_eth wsg50;

    std::string current_action;
    std::string current_target;
    bool have_new_action;



};

/* 
*  *******************************************************************************
*                                   WSG50 GRASP
*  *******************************************************************************
*/


class WSG50GraspBehavior : public Behavior {
public:
    WSG50GraspBehavior(std::string instance);
    
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

    std::string target;
};

/* 
*  *******************************************************************************
*                                   WSG50 RELEASE
*  *******************************************************************************
*/


class WSG50ReleaseBehavior : public Behavior {
public:
    WSG50ReleaseBehavior(std::string instance);
    
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

    std::string target;
};



#endif	/* BEHAVIOR_WSG50_H */

