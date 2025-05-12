/* 
 * Author: RC
 *
 * First created: 18-mar-2024 (SEED 7.0)
 * 
 */

#include "softSequence.h"

using namespace seed; //this is not needed to compile, but most IDEs require it

//bonus of emphasis for sequential behaviors
#define SEED_SEQUENTIAL_BONUS 2.0


// REGISTER this behavior into the Behavior-based System (BBS).
//  This is a 2 steps process...

// 1. PROVIDE a unique name to this behavior
//  the behavior will be identified by this name
std::string SoftSequenceBehavior::behavior_name = "softSequence";

// 2. ADD this behavior (identified by the behavior_name) to the BBS for possible activation
bool SoftSequenceBehavior::registered = BehaviorBasedSystem::add(behavior_name,&SoftSequenceBehavior::create);

// ...now your behavior has been registered, you may recall it by loading
//  a predicate in the form "<behavior_name>(<Arg1>, <Arg2>, ..., <ArgN>)" to the WM



// DEFINE the code of this behavior.
//  This is a 6 steps process...

// 1. DEFINE the constructor for this beahvior, it will be executed when this behavior is recalled.
//
//      instance: is the string containing the predicate that has been used to invoke ths behavior.
//                NOTE: if no paramaters (i.e., further args) are given then instance == behavior_name
SoftSequenceBehavior::SoftSequenceBehavior(std::string instance){
    // please set the instance before anything else
    setInstance(instance);

    // write CUSTOM construction code here...
    std::cout<<arg(0)<<": Constructor() executed "<<std::endl;

    sequence_step = 0;
}

// 2. DEFINE the create function.
//  this function MUST return the reference to the currente behavior.
Behavior *SoftSequenceBehavior::create(std::string instance){
    return new SoftSequenceBehavior(instance);
}


// now let's implement the CUSTOM core of our behavior (virtual functions).

// 3. DEFINE the start function.
//  this function executed only once at the beginning of the execution
void SoftSequenceBehavior::start(){
    // write CUSTOM code here...


    // get list by argument
    //convert prolog-list into vector of tasks
    std::string to_instance = arg(1);
    std::replace( to_instance.begin(), to_instance.begin()+1, '[', '(');
    std::replace( to_instance.end()-1, to_instance.end(), ']', ')');
    task_list = instance2vector(to_instance);
    task_list.erase(task_list.begin());

    //check sequence
    if(task_list.size() == 0){
        std::cout<<ansi::red<<arg(0)<<":\n\t ERROR, sequence is empty"<<ansi::end<<std::endl;
        wm_lock();
        wm_remove_associated_nodes();
        wm_unlock();
        return;
    }

    wm_lock();
    //add all nodes to the sequence
    std::cout<<ansi::cyan<<arg(0)<<":\n\t adding nodes: "<<ansi::end<<std::endl;
    for(size_t i=0; i<task_list.size(); i++){
        std::vector<WM_node *> new_son = wm_add_child_node(task_list[i]);
        std::cout<<"\t task-"<<i<<": "<<task_list[i]<<" added"<<std::endl;
        //set contribution BONUS to the initial node
        if(i==0){
            std::cout<<"\t focusing on node: "<<task_list[sequence_step]<<ansi::end<<std::endl;
            for(size_t j=0; j<new_son.size(); j++){
                new_son[j]->setContribution(this->getInstance(), SEED_SEQUENTIAL_BONUS);
            }
        }
    }
    wm_unlock();
}

// 4. DEFINE the perceptualSchema.
//  this function is executed at the specified frequency.
//      If TRUE is returned, the motorSchema may be executed
bool SoftSequenceBehavior::perceptualSchema(){
    // write CUSTOM code here...
    //std::cout<<arg(0)<<": perceptualSchema() executed "<<std::endl;

    // OLD SOFT SEQUENCE (goal-based)
    //SOFT SEQUENCE here we update seq_goals of sons depending on the sequence's releasers!
    //      DEF: the sequential_goal of a node i in the sequence is true iff. there is
    //      at least one node j in the sequence, with j>i, that is released. 
    // bool is_accomplished = false;
    // wm_lock();
    // for(int i=me->son.size()-1; i>=0; i--){

    //     std::string sequential_goal = this->getInstance()+".step"+std::to_string(i);
    //     // NOTE: when in a sequence the goal is overwritten!
    //     me->son[i]->goal.clear();
    //     me->son[i]->goal.push_back(sequential_goal);

    //     if(is_accomplished){
    //         wmv_set<double>(sequential_goal,1.0);
    //     }
    //     else{
    //         wmv_set<double>(sequential_goal,0.0);
    //         if(me->son[i]->releaserStatus()){
    //             is_accomplished = true;
    //         }
    //     }
    // }
    // wm_unlock();



    return true;
}

// 5. DEFINE the motorSchema.
//  this function is executed at the specified frequency (+ delay of perceptualSchema), 
//  but only if: 
//      - the perceptualSchema returns true
//      - all releasers along the WM branch are true
//      - all goals along the WM branch are false
//  if these conditions are not satisfied, only perceptualSchema is executed.
void SoftSequenceBehavior::motorSchema(){
    // write CUSTOM code here...
    //std::cout<<arg(0)<<": motorSchema() executed "<<std::endl;

    // NEW SOFT SEQUENCE (emphasis-based)
    //
    // SOFT SEQUENCE: here seuqencing is induced through emphasis. All nodes of the seuqence 
    //  are always in the attentional set (WM) but there is a bonus of emphasis going from 
    //  accomplished nodes to next ones (like a wave). 
    //
    //      NOTE: the bonus is a SOFT drive, there is nothing to prevent out-of-sequence nodes
    //          to be executed:
    //              - previously accomplished nodes may be executed again (Stereotypy)
    //              - future nodes may be executed early in the sequence (Anticipation)
    wm_lock();
    //if sequence is already over
    if(((size_t)sequence_step)>=task_list.size()){
        //return
        wm_unlock();
        return;
    }
    //otherwise, there are still nodes to execute (sequence is not over)

    //for all associated nodes
    std::vector<WM_node *> my_nodes = wm_get_associated_nodes();
    for(size_t i=0; i<my_nodes.size(); i++){
        //if one of the nodes of the sequence has been removed (by any chance)
        if(my_nodes[i]->son.size() != task_list.size()){
            //remove the seuqence
            std::cout<<ansi::red<<arg(0)<<":\n\t ERROR, a node has been removed for instance: "<<my_nodes[i]<<ansi::end<<std::endl;
            remove(my_nodes[i]);
        }
    }

    //get all child nodes
    std::vector<std::vector<WM_node *>> current = wm_get_child_nodes();
    //if list is empty
    if(current.size() == 0){
        //all nodes have been removed, we can return
        wm_unlock();
        return;
    }

    //check if the current node is accomplished
    if(current[0][sequence_step]->goalStatus()){
        std::cout<<ansi::cyan<<arg(0)<<":\n\t current node accomplished, focusing on next one"<<ansi::end<<std::endl;
        //going to next task
        sequence_step++;
        if(((size_t)sequence_step)>=task_list.size()){
            std::cout<<ansi::cyan<<"\t sequence accomplished!"<<ansi::end<<std::endl;
            wmv_set<bool>(this->getInstance()+".done",true);
        }
        //otherwise
        else {
            //update the bonus in all my instances
            for(size_t i=0; i<current.size(); i++){
                //remove bonus from the accomplished node
                current[i][sequence_step-1]->setContribution(this->getInstance(), 0.0);
                //add bonus to the current node
                std::cout<<"\t focusing on node: "<<task_list[sequence_step]<<ansi::end<<std::endl;
                current[i][sequence_step]->setContribution(this->getInstance(), SEED_SEQUENTIAL_BONUS);

            }
        }
    }
    
    wm_unlock();
}

// 6. DEFINE the exit function.
//  this function executed only once at the end of the execution
void SoftSequenceBehavior::exit(){
    // write CUSTOM code here...
    std::cout<<arg(0)<<": exit() executed "<<std::endl;

    wm_lock();
    wmv_set<bool>(this->getInstance()+".done",false);
    wm_unlock();
}

// ...enjoy