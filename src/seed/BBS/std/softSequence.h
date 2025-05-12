#ifndef BEHAVIOR_SOFT_SEQUENCE_H
#define BEHAVIOR_SOFT_SEQUENCE_H

#include "seed.h"

using namespace seed; //this is not needed to compile, byt most IDEs require it

class SoftSequenceBehavior : public Behavior {
public:
    SoftSequenceBehavior(std::string instance);
    
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
    int sequence_step;

    std::vector<std::string> task_list;
    std::vector<std::string> seq_goal_list;
};

#endif	/* BEHAVIOR_SOFT_SEQUENCE_H */

