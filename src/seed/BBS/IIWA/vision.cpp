/* 
 * Author: RC
 *
 * First created: 9-feb-2024 (SEED 7.0)
 * 
 */

#include "vision.h"

using namespace seed; //this is not needed to compile, but most IDEs require it


/* 
*  *******************************************************************************
*                                   VISION STREAM
*  *******************************************************************************
*/


// REGISTER this behavior into the Behavior-based System (BBS).
//  This is a 2 steps process...

// 1. PROVIDE a unique name to this behavior
//  the behavior will be identified by this name
std::string VisionStreamBehavior::behavior_name = "visionStream";

// 2. ADD this behavior (identified by the behavior_name) to the BBS for possible activation
bool VisionStreamBehavior::registered = BehaviorBasedSystem::add(behavior_name,&VisionStreamBehavior::create);

// ...now your behavior has been registered, you may recall it by loading
//  a predicate in the form "<behavior_name>(<Arg1>, <Arg2>, ..., <ArgN>)" to the WM



// DEFINE the code of this behavior.
//  This is a 6 steps process...

// 1. DEFINE the constructor for this beahvior, it will be executed when this behavior is recalled.
//
//      instance: is the string containing the predicate that has been used to invoke ths behavior.
//                NOTE: if no paramaters (i.e., further args) are given then instance == behavior_name
VisionStreamBehavior::VisionStreamBehavior(std::string instance){
    // please set the instance before anything else
    setInstance(instance);

    // write CUSTOM construction code here...

    sb_vision = nh->create_subscription<std_msgs::msg::String>("vision/learning", 0, std::bind(&VisionStreamBehavior::vision_callback, this, _1));

    vision_msg = "";
    old_vision_msg = "";

    std::cout<<arg(0)<<": Constructor() executed "<<std::endl;
}

// 2. DEFINE the create function.
//  this function MUST return the reference to the currente behavior.
Behavior *VisionStreamBehavior::create(std::string instance){
    return new VisionStreamBehavior(instance);
}


// now let's implement the CUSTOM core of our behavior (virtual functions).

// 3. DEFINE the start function.
//  this function executed only once at the beginning of the execution
void VisionStreamBehavior::start(){
    // write CUSTOM code here...
    std::cout<<arg(0)<<": started"<<std::endl;
}

// 4. DEFINE the perceptualSchema.
//  this function is executed at the specified frequency.
//      If TRUE is returned, the motorSchema may be executed
bool VisionStreamBehavior::perceptualSchema(){
    // write CUSTOM code here...
    //std::cout<<arg(0)<<": perceptualSchema() executed "<<std::endl;

    this->setRate(1);

    //do nothing

    return true;
}

// 5. DEFINE the motorSchema.
//  this function is executed at the specified frequency (+ delay of perceptualSchema), 
//  but only if: 
//      - the perceptualSchema returns true
//      - all releasers along the WM branch are true
//      - all goals along the WM branch are false
//  if these conditions are not satisfied, only perceptualSchema is executed.
void VisionStreamBehavior::motorSchema(){
    // write CUSTOM code here...
    //std::cout<<arg(0)<<": motorSchema() executed "<<std::endl;

    if(vision_msg != "" && vision_msg != old_vision_msg){
        std::cout<<arg(0)<<": new schema from vision!"<<std::endl;

        old_vision_msg = vision_msg;

        std::string new_schema = vision2schema(vision_msg);

        std::cout<<new_schema<<std::endl;


        wm_lock();

        //add the schema to the LTM

        std::ofstream outfile; 
        outfile.open(SEED_HOME_PATH + "/LTM/" + SEED_NAME + "_LTM.prolog", std::ios_base::app);
        if (outfile.is_open()) {

            outfile<<new_schema<<"\n\n";
                
            outfile.close();
        }
        else
            std::cout<<arg(0)<<": LTM not found "<<std::endl;

        std::cout<<"SEED: LTM updated! "<<std::endl;

        //reload the LTM
        if( !LTM->loadLTM(SEED_HOME_PATH + "/LTM/" + SEED_NAME + "_LTM.prolog") ){
            std::cout<<"SEED-ERROR: unable to RELOAD LTM for the name "<<SEED_NAME<<std::endl;
        }
        else
            std::cout<<"SEED: LTM reloaded! "<<std::endl;
        
        wm_unlock();

    }

}

// 6. DEFINE the exit function.
//  this function executed only once at the end of the execution
void VisionStreamBehavior::exit(){
    // write CUSTOM code here...

    std::cout<<arg(0)<<": connection CLOSED"<<std::endl;
}

std::string VisionStreamBehavior::vision2schema(std::string v_str){
    std::stringstream ss;

    //std::transform(vision_msg.begin(), vision_msg.end(), vision_msg.begin(), std::tolower);

    //remove spaces
    v_str.erase(std::remove(v_str.begin(), v_str.end(), ' '), v_str.end());

    //vision message is represented as a predicate P(X1,X2, ...Xn)
    //  get vector from the predicate
    std::vector<std::string> vv = instance2vector(v_str);

    //the name of the predicate is the name of the schema
    ss<<"schema("<<vv[0]<<",[";

    //here we assume this action to be a HARD SEQUENCE
    ss<<"[hardSequence([";

    //for each argument of the predicate
    for(size_t i=1; i< vv.size(); i++){
        //add the argument as sub-schema of the schema
        std::vector<std::string> sub_v = instance2vector(vv[i]);

        //check if it is a primitive action
        if(sub_v[0] == "REACH"){
            ss<<"iiwaGo("<<sub_v[1]<<")";
        }
        else if(sub_v[0] == "GRASP"){
            //for now let's use the name of the task as OBJECT
            ss<<"wsg50Grasp("<<vv[0]<<")";
        }
        else if(sub_v[0] == "WAIT"){
            //for now let's use the name of the task as EVENT
            ss<<"iiwaWait("<<vv[0]<<")"; //3 secs wait
        }
        else{
            //task is not primitive, add it directly
            ss<<sub_v[0];
        }

        //add comma if not the last
        if(i != vv.size()-1)
            ss<<",";
    }

    //close the hard sequence
    ss<<"]),0,[\"TRUE\"]]";

    //close the schema
    ss<<"],[],[]).";

    return ss.str();
}


// ...enjoy
