#ifndef BEHAVIOR_TEMPLATE_H
#define BEHAVIOR_TEMPLATE_H

#include "seed.h"

using namespace seed; //this is not needed to compile, byt most IDEs require it

class TemplateBehavior : public Behavior {
public:
    TemplateBehavior(std::string instance);
    
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
};

#endif	/* BEHAVIOR_TEMPLATE_H */

