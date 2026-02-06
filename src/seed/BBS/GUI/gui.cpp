/* 
 * Author: RC
 *
 * First created: 9-feb-2024 (SEED 7.0)
 * 
 */

#include "gui.h"

using namespace seed; //this is not needed to compile, but most IDEs require it


// REGISTER this behavior into the Behavior-based System (BBS).
//  This is a 2 steps process...

// 1. PROVIDE a unique name to this behavior
//  the behavior will be identified by this name
std::string GUIBehavior::behavior_name = "gui";

// 2. ADD this behavior (identified by the behavior_name) to the BBS for possible activation
bool GUIBehavior::registered = BehaviorBasedSystem::add(behavior_name,&GUIBehavior::create);

// ...now your behavior has been registered, you may recall it by loading
//  a predicate in the form "<behavior_name>(<Arg1>, <Arg2>, ..., <ArgN>)" to the WM



// DEFINE the code of this behavior.
//  This is a 6 steps process...

// 1. DEFINE the constructor for this beahvior, it will be executed when this behavior is recalled.
//
//      instance: is the string containing the predicate that has been used to invoke ths behavior.
//                NOTE: if no paramaters (i.e., further args) are given then instance == behavior_name
GUIBehavior::GUIBehavior(std::string instance){
    // please set the instance before anything else
    setInstance(instance);

    // write CUSTOM construction code here...
}

// 2. DEFINE the create function.
//  this function MUST return the reference to the currente behavior.
Behavior *GUIBehavior::create(std::string instance){

    return new GUIBehavior(instance);
}


// now let's implement the CUSTOM core of our behavior (virtual functions).

// 3. DEFINE the start function.
//  this function executed only once at the beginning of the execution
void GUIBehavior::start(){
    // write CUSTOM code here...

    pb = nh->create_publisher<std_msgs::msg::String>(SEED_NAME + "/wm", 1);
    
    std::cout<<arg(0)<<": started "<<std::endl;
}

// 4. DEFINE the perceptualSchema.
//  this function is executed at the specified frequency.
//      If TRUE is returned, the motorSchema may be executed
bool GUIBehavior::perceptualSchema(){
    // write CUSTOM code here...
    return true;
}

// 5. DEFINE the motorSchema.
//  this function is executed at the specified frequency (+ delay of perceptualSchema), 
//  but only if: 
//      - the perceptualSchema returns true
//      - all releasers along the WM branch are true
//      - all goals along the WM branch are false
//  if these conditions are not satisfied, only perceptualSchema is executed.
void GUIBehavior::motorSchema(){
    // write CUSTOM code here...

    std::string jstr = saveWMtoJSON(WM);

    std_msgs::msg::String msg;
    msg.data = jstr;
    pb->publish(msg);
}

// 6. DEFINE the exit function.
//  this function executed only once at the end of the execution
void GUIBehavior::exit(){
    // write CUSTOM code here...
    std::cout<<arg(0)<<": exited "<<std::endl;
}

// ...enjoy