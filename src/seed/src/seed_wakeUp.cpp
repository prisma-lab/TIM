#define GUI_Behavior                 1       //include behavior for GUI visualization (needs gtk)
#define SHOW_Behavior                1       //include show behavior (needs GRAPHVIZ)
#define VOCAL_Behavior               0       //include vocal_stream behaviors (needs ESPEAK, has problem with HUMBLE)
#define LEARN_JOY_Behavior           0       //include learning behavior through JOY


//seed_header provide compilation flags too
#include "seed.h"


//---include default behavior
#include "std/std_behaviors.h"
#include "std/ltm_behaviors.h"
#include "std/wmv_behaviors.h"
#include "std/ros_behaviors.h"
#include "std/test.h"

#include "std/softSequence.h"
#include "std/hardSequence.h"
//---

//---include show behavior
#if SHOW_Behavior
  #include "std/show.h"
#endif
//---

//---include vocal behaviors
#if VOCAL_Behavior
  #include "vocal_stream.h"
#endif
//---


//---include GUI behavior
#if GUI_Behavior
    #include "GUI/gui.h"
#endif
//---

#if LEARN_JOY_Behavior
    //this should be adjcusted and moved to BBS
    #include "learn/joyStream.h" 
#endif


// try to implement self-registering classes for a factory design pattern
//	further reading: https://www.cppstories.com/2018/02/factory-selfregister/
#include "template/template.h"

namespace seed{

bool wakeUp(std::string behavior, std::string instance){
    Behavior *obj=NULL;

    if(behavior=="alive")
        return true;

    else if(behavior=="forget")
        obj=new ForgetBehavior(instance);
    else if(behavior=="remember")
        obj=new RememberBehavior(instance);
    else if(behavior=="test")
        obj=new TestBehavior(instance);
    else if(behavior=="inputStream")
        obj=new InputStreamBehavior(instance);
    else if(behavior=="listing")
        obj=new ListingBehavior(instance);

    else if(behavior=="set")
        obj=new SetBehavior(instance);
    else if(behavior=="get")
        obj=new GetBehavior(instance);
    else if(behavior=="timer")
        obj=new TimerBehavior(instance);
    else if(behavior=="compete")
        obj=new CompeteBehavior(instance);
    else if(behavior=="solve")
        obj=new SolveBehavior(instance);
    else if(behavior=="ltm")
        obj=new LTMBehavior(instance);

    else if(behavior=="rosStream")
        obj=new RosStreamBehavior(instance);
    else if(behavior=="rosSolve")
        obj=new RosSolveBehavior(instance);
    else if(behavior=="rosState")
        obj=new RosStateBehavior(instance);
    else if(behavior=="rosAct")
        obj=new RosActBehavior(instance);
    else if(behavior=="tfobserver")
        obj=new TfObserverBehavior(instance);

#if SHOW_Behavior
    else if(behavior=="show")
        obj=new ShowBehavior(instance);
#endif
    
#if VOCAL_Behavior
    else if(behavior=="vocalStream")
        obj=new VocalStreamBehavior(instance);
    else if(behavior=="say")
        obj=new SayBehavior(instance);
#endif

#if GUI_Behavior
    else if(behavior=="gui")
        obj=new GuiBehavior(instance);
#endif

    // SELF-REGISTERING BEHAVIORS (SINCE SEED 7.0)
    else if(BBS->exists(behavior)){
    	std::cout<<"LOADING NEW BEHAVIOR"<<std::endl;
        obj=BBS->load(behavior,instance);
    }

//    std::cout<<"waking: "<<instance<<"...\n";

    if(obj!=NULL){
        pthread_t thread_exe;
        pthread_create(&thread_exe, NULL,execution, (void*) obj);
        obj->setTid( thread_exe );
        seed_thread_list.push_back( thread_exe );
        pthread_detach(thread_exe); //detach thread to optimize memory consumption
        return true;
    }
    return false;
}


} //seed namespace
