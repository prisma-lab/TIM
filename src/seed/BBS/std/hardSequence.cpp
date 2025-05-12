/* 
 * Author: RC
 *
 * First created: 9-apr-2024 (SEED 8.0)
 * 
 */

#include "hardSequence.h"

using namespace seed; //this is not needed to compile, but most IDEs require it


// REGISTER this behavior into the Behavior-based System (BBS).
//  This is a 2 steps process...

// 1. PROVIDE a unique name to this behavior
//  the behavior will be identified by this name
std::string HardSequenceBehavior::behavior_name = "hardSequence";

// 2. ADD this behavior (identified by the behavior_name) to the BBS for possible activation
bool HardSequenceBehavior::registered = BehaviorBasedSystem::add(behavior_name,&HardSequenceBehavior::create);

// ...now your behavior has been registered, you may recall it by loading
//  a predicate in the form "<behavior_name>(<Arg1>, <Arg2>, ..., <ArgN>)" to the WM



// DEFINE the code of this behavior.
//  This is a 6 steps process...

// 1. DEFINE the constructor for this beahvior, it will be executed when this behavior is recalled.
//
//      instance: is the string containing the predicate that has been used to invoke ths behavior.
//                NOTE: if no paramaters (i.e., further args) are given then instance == behavior_name
HardSequenceBehavior::HardSequenceBehavior(std::string instance){
    // please set the instance before anything else
    setInstance(instance);

    // write CUSTOM construction code here...
    std::cout<<arg(0)<<": Constructor() executed "<<std::endl;

    sequence_step = 0;
}

// 2. DEFINE the create function.
//  this function MUST return the reference to the currente behavior.
Behavior *HardSequenceBehavior::create(std::string instance){
    return new HardSequenceBehavior(instance);
}


// now let's implement the CUSTOM core of our behavior (virtual functions).

// 3. DEFINE the start function.
//  this function executed only once at the beginning of the execution
void HardSequenceBehavior::start(){
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

    //add the first task of the sequence
    wm_lock();
    std::vector<WM_node *> new_child = wm_add_child_node(task_list[sequence_step]);
    std::cout<<ansi::cyan<<arg(0)<<":\n\t new node running: "<<task_list[sequence_step]<<ansi::end<<std::endl;
    //if(new_child[0]->goal.size() == 0){
    //    std::cout<<ansi::yellow<<"\t WARNING, node "<<task_list[sequence_step]<<" has no goal, sequence stops on it"<<ansi::end<<std::endl;
    //}
    wm_unlock();
}

// 4. DEFINE the perceptualSchema.
//  this function is executed at the specified frequency.
//      If TRUE is returned, the motorSchema may be executed
bool HardSequenceBehavior::perceptualSchema(){
    // write CUSTOM code here...
    //std::cout<<arg(0)<<": perceptualSchema() executed "<<std::endl;

    // do nothing... by convention this node's activities ara managed in the motorSchema

    return true;
}

// 5. DEFINE the motorSchema.
//  this function is executed at the specified frequency (+ delay of perceptualSchema), 
//  but only if: 
//      - the perceptualSchema returns true
//      - all releasers along the WM branch are true
//      - all goals along the WM branch are false
//  if these conditions are not satisfied, only perceptualSchema is executed.
void HardSequenceBehavior::motorSchema(){
    // write CUSTOM code here...
    //std::cout<<arg(0)<<": motorSchema() executed "<<std::endl;

    // NEW HARD SEQUENCE (remember/forget-based)
    //
    // HARD SEQUENCE: nodes that are acomplished are forgotten (no attention on it)
    //  and the next ones in the sequence are remembered (pay attention on it)
    //
    //  NOTE: this mechanism is used to emulate classic AI sequences in which an accomplished task
    //      enables the next one but is never considered again. Following the attention model, this
    //      means that accomplished nodes are out of the attentional set (WM), while new ones are 
    //      added to the attentional set (WM).

    wm_lock();
    //if there are still nodes to execute (sequence is not over)
    std::vector<WM_node *> my_nodes = wm_get_associated_nodes();
    // if the current node has been forgotten, the sequence is removed from WM
    for(size_t i=0; i<my_nodes.size(); i++){
        if(my_nodes[i]->son.size() == 0){
            std::cout<<ansi::red<<arg(0)<<":\n\t ERROR, no running nodes for instance: "<<my_nodes[i]<<ansi::end<<std::endl;
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
    
    //if current node has been accomplished (goal is satisfied)
    //  NOTE: check the first node as they all must have the same goal
    if(current[0][0]->goalStatus()){
        std::cout<<ansi::cyan<<arg(0)<<":\n\t current node accomplished, going to next one"<<ansi::end<<std::endl;
        //forget the current node
        wm_remove_child_node(task_list[sequence_step]);
        //go to the next step of the sequence
        sequence_step++;
        //if no more tasks are available
        if(((size_t)sequence_step)>=task_list.size()){
            //the sequence is accomplished
            std::cout<<ansi::cyan<<"\t sequence accomplished!"<<ansi::end<<std::endl;
            wmv_set<bool>(this->getInstance()+".done",true);
        }
        //otherwise
        else{
            //remember the next node of the sequence
            std::vector<WM_node *> new_child = wm_add_child_node(task_list[sequence_step]);
            std::cout<<ansi::cyan<<"\t new node running: "<<task_list[sequence_step]<<ansi::end<<std::endl;
            //if(new_child[0]->goal.size() == 0){
            //    std::cout<<ansi::yellow<<"\t WARNING, node "<<task_list[sequence_step]<<" has no goal, sequence stops on it"<<ansi::end<<std::endl;
            //}
        }
    }
    wm_unlock();
}

// 6. DEFINE the exit function.
//  this function executed only once at the end of the execution
void HardSequenceBehavior::exit(){
    // write CUSTOM code here...
    std::cout<<arg(0)<<": exit() executed "<<std::endl;

    wm_lock();
    wmv_set<bool>(this->getInstance()+".done",false);
    wm_unlock();
}

// ...enjoy