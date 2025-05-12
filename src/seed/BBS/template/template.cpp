/* 
 * Author: RC
 *
 * First created: 9-feb-2024 (SEED 7.0)
 * 
 */

#include "template.h"

using namespace seed; //this is not needed to compile, but most IDEs require it


// REGISTER this behavior into the Behavior-based System (BBS).
//  This is a 2 steps process...

// 1. PROVIDE a unique name to this behavior
//  the behavior will be identified by this name
std::string TemplateBehavior::behavior_name = "template";

// 2. ADD this behavior (identified by the behavior_name) to the BBS for possible activation
bool TemplateBehavior::registered = BehaviorBasedSystem::add(behavior_name,&TemplateBehavior::create);

// ...now your behavior has been registered, you may recall it by loading
//  a predicate in the form "<behavior_name>(<Arg1>, <Arg2>, ..., <ArgN>)" to the WM



// DEFINE the code of this behavior.
//  This is a 6 steps process...

// 1. DEFINE the constructor for this beahvior, it will be executed when this behavior is recalled.
//
//      instance: is the string containing the predicate that has been used to invoke ths behavior.
//                NOTE: if no paramaters (i.e., further args) are given then instance == behavior_name
TemplateBehavior::TemplateBehavior(std::string instance){
    // please set the instance before anything else
    setInstance(instance);

    // write CUSTOM construction code here...
    std::cout<<arg(0)<<": Constructor() executed "<<std::endl;
}

// 2. DEFINE the create function.
//  this function MUST return the reference to the currente behavior.
Behavior *TemplateBehavior::create(std::string instance){
    return new TemplateBehavior(instance);
}


// now let's implement the CUSTOM core of our behavior (virtual functions).

// 3. DEFINE the start function.
//  this function executed only once at the beginning of the execution
void TemplateBehavior::start(){
    // write CUSTOM code here...
    std::cout<<arg(0)<<": start() executed "<<std::endl;
}

// 4. DEFINE the perceptualSchema.
//  this function is executed at the specified frequency.
//      If TRUE is returned, the motorSchema may be executed
bool TemplateBehavior::perceptualSchema(){
    // write CUSTOM code here...
    std::cout<<arg(0)<<": perceptualSchema() executed "<<std::endl;
    return true;
}

// 5. DEFINE the motorSchema.
//  this function is executed at the specified frequency (+ delay of perceptualSchema), 
//  but only if: 
//      - the perceptualSchema returns true
//      - all releasers along the WM branch are true
//      - all goals along the WM branch are false
//  if these conditions are not satisfied, only perceptualSchema is executed.
void TemplateBehavior::motorSchema(){
    // write CUSTOM code here...
    std::cout<<arg(0)<<": motorSchema() executed "<<std::endl;
}

// 6. DEFINE the exit function.
//  this function executed only once at the end of the execution
void TemplateBehavior::exit(){
    // write CUSTOM code here...
    std::cout<<arg(0)<<": exit() executed "<<std::endl;
}

// ...enjoy