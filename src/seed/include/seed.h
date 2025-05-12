/*
 * File:   seed.h
 * Author: hargalaten
 *
 * Created on 12 dicembre 2012, 15.45
 */

#ifndef SEED_H
#define	SEED_H

#include <iostream>
#include <pthread.h>
#include <string>
#include <math.h>
#include <time.h>
#include <fstream>

//-- ROS2 headers
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/float32.hpp"
//#include "std_msgs/msg/uint32.hpp"
#include "std_msgs/msg/bool.hpp"

#include "sensor_msgs/msg/joy.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>
//--

//#include "opencv2/opencv.hpp"
//#include "opencv2/ml/ml.hpp"
#include "boost/random.hpp"
#include <execinfo.h>
#include <signal.h>

#include "utils/seed_geometry.h"
#include "utils/seed_debug.h"
#include "utils/seed_time.h"

// added in SEED 8.0 to use simple/extended eval function (exdende used by default)
#ifndef SEED_USE_EVAL_SIMPLE
#define SEED_USE_EVAL_SIMPLE 0
#endif

// THESE ARE OUTDATED SINCE SEED 7.1
//QUIESCENCE and DEFALT_RTM are dangerous: if those are too high the competition can be affected by noise
//  on the transintion between not-released and released state of competitors
#define QUIESCENCE 0.1
#define SEED_DEFAULT_RTM 0.1

// 10 Hzis the default rete of behaviors
#define SEED_DEFAULT_RATE 10

#define MSECOND 1000000
#define PRECISION 10000

#define TELEOLOGY_EMP 1

#define SEED_DEFAULT_WEIGHT 1.0

//max lenght of the history for WMV, negative value is for infinite history
#define SEED_MAX_CONTENTIONS_TO_REMEMBER 5

using namespace std::placeholders; // needed by ROS2

namespace seed{

double rtod(double);
double dtor(double);
//double string_to_double(std::string string_to_convert, char separator = '.'); //replaced by ston (string to number) from seed_debug.h

extern std::string SYS_HOME_PATH;
extern std::string SEED_HOME_PATH;

extern std::string SEED_NAME;

extern pthread_mutex_t memMutex;
extern std::string agentName;

extern int seed_wm_id;

extern std::vector<pthread_t> seed_thread_list;

extern time::Clock seed_global_clk;

// define UNIQUE ROS-node for seed
extern rclcpp::Node::SharedPtr nh;

// **** **** **** **** WMV **** **** **** **** //


//Class containing informations about the value of a variable
class WM_value {
public:
    WM_value(){
        instance = "";
        time = seed::time::now();
        value = NULL;
    }
    
    //assign a new value, the previous value is deleted!
    template<class T> void assign(std::string in_instance, T in_val){
        //delete the old pointer
        delete (T*)value; //pointer cannot be deleted here, this value is stored in the history!
        
        //create a new pointer to the value
        T *point = new T();
        *point = in_val;
        
        //assign the new pointer
        value = (void *) point;
        instance = in_instance;
        time = seed::time::now();
        //std::cout<<ansi::green<<"NEW pointer "<<value<<ansi::end<<std::endl;
    }
    
    //assign a new wm_value by copy, the previous value is deleted!
    template<class T> void assign(WM_value new_val){
        //delete the old pointer
        delete (T*)value; //pointer cannot be deleted here, this value is stored in the history!
        
        //create a new pointer to the value
        T *point = new T();
        *point = new_val.value;
        
        //assign the new pointer
        value = (void *) point;
        instance = new_val.instance;
        time = new_val.time;
        //std::cout<<ansi::green<<"NEW pointer "<<value<<ansi::end<<std::endl;
    }
    
    //delete the pointer to the value, calling the destructor of the
    //  specified type T
    //      NOTE: VALUES MUST BE CLEARED AFTER USE!
    template<class T> void clear(){
        instance = "";

        delete (T*)value;
        //std::cout<<ansi::red<<"DELETE pointer "<<value<<ansi::end<<std::endl;
        value = NULL;
    }
    
    //copy the value allocating a new instance of the value (a new pointer is 
    //  created and have to be cleared after use)
    template<class T> WM_value copy(){
        
        WM_value copy_value;
        
        //create a new pointer to the value
        T *point = new T();
        *point = *( (T *) value );
        
        copy_value.instance = instance;
        copy_value.time = time;
        copy_value.value = point;
        
        return copy_value;
    }
    
    //instance of the node proposing this value
    std::string instance;
    //emphasis value of the node //seed 4.0: emphasis will be computed online to be NEWER. To be removed!
    //double emp;
    //instant of time when this value have been proposed
    seed::time::t_point time;
    //proposed value
    void *value;
};

//Class representing the competition to elect the winning value for a variable
//  among all the proposed values
class WM_variable {
public:
    WM_variable(){
        
        selected = false;
        set = false;
    }
    
    //copy the variable allocating a new instance of the value (a new pointer is 
    //  created and have to be cleared after use)
    template<class T> WM_variable copy(){
        
        WM_variable copy_var;
        
        copy_var.selected_value = selected_value.copy<T>();
        copy_var.selected = selected;
        copy_var.set = set;
        copy_var.selection_time = selection_time;
        
        for(size_t i=0; i< candidate_value.size(); i++)
            copy_var.candidate_value.push_back( candidate_value[i].copy<T>() );
        
        return copy_var;
    }
    
    //delete all the allocated pointers, calling the destructor of the
    //  specified type T
    template<class T> void clear(){
        //erase the winner
        selected_value.clear<T>();
        //erase all proposed values
        for(size_t i=0; i<candidate_value.size(); i++)
            candidate_value[i].clear<T>();
    }
    
    //true if a current value have been selected
    bool selected;
    //true if the value have been set directly (bypassing the competition)
    bool set;
    
    seed::time::t_point selection_time;
    WM_value selected_value;
    
    std::vector<WM_value> candidate_value;
};


//HashMap of the working memory variables
class WM_varmap {
public:
    WM_varmap() {
        
        //values_to_remember = SEED_MAX_CONTENTIONS_TO_REMEMBER < 2 ? 2 : SEED_MAX_CONTENTIONS_TO_REMEMBER; //at least two values
        
        if(SEED_MAX_CONTENTIONS_TO_REMEMBER == 0)
            values_to_remember = 2; //default
        else
            values_to_remember = SEED_MAX_CONTENTIONS_TO_REMEMBER;
        
        t_start = time::now();
    }
    
    //
    template<class T>
    void set(std::string instance_of_setter, std::string id, T value) {
        //std::cout<<instance <<", setting "<<id <<std::endl;
        
        //set the variable
        v[id].selected = true;
        v[id].set = true;
        v[id].selected_value.assign<T>(instance_of_setter, value);
        
        //if stored contentions exceed the limits
        if( h[id].size() == (size_t)values_to_remember ){
            //erase the last stored contention
            h[id].back().clear<T>();
            h[id].pop_back();
        }

        //push_front into the list
        h[id].push_front( v[id].copy<T>() );
        
        //std::cout<<ansi::yellow<<instance <<", set done, pointer is "<<v[id].selected_value.value<<ansi::end<<std::endl;
        
    }

    template<class T>
    T get(std::string instance_of_getter, std::string id) {
        (void) instance_of_getter; //unused
        //std::cout<<instance<<", getting "<<id<<std::endl;
        if (v.find(id) != v.end() && v[id].selected) {
            return *( (T *) v[id].selected_value.value );
        } else {
            return T();
        }
    }
    
    template<class T>
    T get(std::string id) {
        //std::cout<<"getting "<<id<<std::endl;
        if (v.find(id) != v.end() && v[id].selected) {
            //std::cout<<ansi::yellow<<"  "<<id<<" is selected, pointer is "<<v[id].selected_value.value<<ansi::end<<std::endl;
            return *( (T *) v[id].selected_value.value );
        } else {
            return T();
        }
    }
    
    template<class T>
    T get(std::string id, double seconds_ago) {
        //if the variable exists
        if( h.find(id) != h.end() ){
            //search the value into the history
            return *( (T *) search_back_in_time(h[id], seconds_ago).selected_value.value );
        }
        else return T();
    }
    
    template<class T>
    T get(std::string id, time::t_point time_at) {
        
        //if the variable exists
        if( h.find(id) != h.end() ){
            //search the value into the history
            return *( (T *) search_back_in_time(h[id], time_at).selected_value.value );
        }
        else return T();
    }
    
    WM_variable variable(std::string id){
        if( v.find(id) != v.end() ){
            return v[id];
        }
        else
            return WM_variable();
    }

    int history_size(std::string id){
        return h[id].size();
    }
    
    WM_variable history(std::string id, int index){
        int i = 0;
        if( (size_t)index<h[id].size() ){
            for(auto it = h[id].begin(); it != h[id].end(); it++) {
                if(i == index)
                    return *it;
            }
        }
        else
            return WM_variable();
    }

    //set winning value selecting a winner among candidate values
    //  -> "selected" variable becomes TRUE
    template<class T>
    void select(std::string id, int selected_candidate_index){
        
        v[id].selected_value = v[id].candidate_value[selected_candidate_index].copy<T>();
        
        v[id].selected = true;
        v[id].set = false;
        v[id].selection_time = seed::time::now();
        
        //store this value in history and start a new contention
        
        //if stored contentions exceed the limits
        if( (int) h[id].size() == values_to_remember ){
            //erase the last stored contention
            h[id].back().clear<T>();
            h[id].pop_back();
        }

        //push_front into the list
        h[id].push_front( v[id].copy<T>() );
        
        // modified in SEED 6.0 (updated to ROS2)
        // NOTE: here the competition was reset, now reset is made in the calling (wmv_solve) function.
        //       it has been made to avoid the atuomatic reset of the competition
    }

    //deselect winning value (ADDED IN SEED 6.0)
    //  -> "selected" variable becomes FALSE
    template<class T>
    void deselect(std::string id){
        
        v[id].selected_value = WM_value();
        
        v[id].selected = false;
        v[id].set = false;
        v[id].selection_time = seed::time::now();
    }

    //reset competition for the variable (i.e., all candidates are removed)
    template<class T>
    void reset(std::string id){
        //clear all the single values
        for(size_t i=0; i<v[id].candidate_value.size(); i++)
            v[id].candidate_value[i].clear<T>();
        //clear the vector of values
        v[id].candidate_value.clear(); 
    }

    // remove instance from competition
    template<class T> void remove(std::string instance, std::string id){
        size_t i = 0;
        while (i < v[id].candidate_value.size()){
            if(v[id].candidate_value[i].instance == instance){
                //clear the value
                v[id].candidate_value[i].clear<T>();
                //erase the element from the vector 
                v[id].candidate_value.erase(v[id].candidate_value.begin() + i);
            }
            else
                i++;
        }
    }
    
    template<class T> void compete(std::string instance, std::string id, T value){
        
        //if variable already exists
        if (v.find(id) != v.end()){
                
            //search if this instance already proposed a value
            int proposer_index = -1;
            for(size_t i=0; i<v[id].candidate_value.size(); i++){
                if( v[id].candidate_value[i].instance == instance){
                    proposer_index = i;
                    break;
                }
            }
            //if the value is the first one proposed by this instance
            if(proposer_index<0){
                //add a the new value
                WM_value new_val;
                new_val.assign<T>(instance,value);
                v[id].candidate_value.push_back(new_val);
            }
            //oth. update the existing value
            else{
                //v[id].candidate_value[proposer_index].clear<T>();
                v[id].candidate_value[proposer_index].assign<T>(instance,value);
            }
        }
        else{
            //create a new value
            WM_value new_val;
            new_val.assign<T>(instance, value);
            
//            //create a new variable with this value
//            WM_variable new_var;
//            new_var.candidate_value.push_back(new_val);
            
            v[id].candidate_value.push_back(new_val);
        }
    }
    
    /**
     *  This function checks if the hashing function of the map is colliding.
     *      NOTE: if there are collisions, it is maybe better to use map (which 
     *            is O(log(n)) on access) instead of unordered_map (which goes
     *            from O(1) without collisions to O(n)).
     */
    bool isColliding(){
        for(size_t bucket = 0; bucket < v.bucket_count(); bucket++) {
            if(v.bucket_size(bucket) > 1) {
                return true;
            }
        }
        return false;
    }
    
    std::string owner(std::string id){
        //if variable exists
        if (v.find(id) != v.end()){
            //if value is selected
            if(v[id].selected)
                //return the owner of that value
                return v[id].selected_value.instance;
            //oth. get the last variable
            else if( h[id].size() >= 1 )
                //return the owner of the last selected value
                return h[id].front().selected_value.instance;
        }
        
        return "NULL";
    }
    
    std::vector< WM_value > candidates(std::string id){
        //if variable exists
        if (v.find(id) != v.end()){
            //return the the candidate values
            return v[id].candidate_value;
        }
        
        return std::vector<WM_value>();
    }
    
    WM_value get_candidate_value(std::string id, std::string instance_of_candidate){
        //if variable exists
        if (v.find(id) != v.end()){
            //return the the candidate values
            for(size_t i=0; i<v[id].candidate_value.size(); i++)
                if(v[id].candidate_value[i].instance == instance_of_candidate)
                    return v[id].candidate_value[i];
        }
        
        return WM_value();
    }
    
    template<class T>
    void clear(std::string id) {
        
        v[id].clear<T>();
        v[id].selected_value.instance = "";
        v[id].candidate_value.clear();
        
        v[id].selected = false;
        v[id].set = false;
    }
    
    //check if a candidate value has been selected, solving the competition
    bool isSelected(std::string id){
        return v[id].selected;
    }

private:
    
    //  NOTE: seconds ago from the current time 
    WM_variable search_back_in_time(std::list< WM_variable > _list, double seconds){
        time::t_point t_current = time::now(); 
        //iterate from the newest element to the oldest one
        for(auto it = _list.begin(); it != _list.end(); it++) {
            //if the elapsed time is 0/negative the value is the most updated
            //  at the instant t_target
            if( time::elapsed(it->selection_time, t_current ) >= seconds )
                //return the TimedValue
                return *it;
        }
        //return null with the oldest time
        return WM_variable();
    }
    
    WM_variable search_back_in_time(std::list< WM_variable > _list, time::t_point t_target){
        //iterate from the newest element to the oldest one
        for(auto it = _list.begin(); it != _list.end(); it++) {
            //if the elapsed time is 0/negative the value is the most updated
            //  at the instant t_target
            if( time::elapsed(t_target, it->selection_time) <= 0 )
                //return the TimedValue
                return *it;
        }
        //return null with the oldest time
        return WM_variable();
    }
    
    /* unique map of variables*/
    std::unordered_map<std::string, WM_variable > v;
    /* history of the variables, storing all past selected/candidate values */
    std::unordered_map<std::string, std::list< WM_variable > > h;
    
    int values_to_remember;
    
    time::t_point t_start;
};



// **** **** **** **** WM **** **** **** **** //


//node of the Working Memory
class WM_node {
public:

    WM_node(std::string newInstance, WM_node* instanceFather);
    //id of the schema, unique into the WM
    int id;
    //name of the schema
    std::string name;
    //instance of schema (ie. name+parameters)
    std::string instance;
    //releaser: sequence of working memory variables in AND form
    std::vector<std::string> releaser;
    //contributions given by schemata
    std::unordered_map<std::string, double *> contribution;
//    //weights of the contributions
//    std::map<std::string, double *> weight;
    //emphasis of the node, given by the LTM (a priori)
    double ltm_emphasis;
    //number of times that the goal is reached
    double goalCount;
    //current rhythm of the associated behavior (only descriptive)
    double rtm;
    //list of the sons of the node (sub-nodes)
    std::vector< WM_node*> son;
    //pointer to the father node (sup-node)
    WM_node *father;
    //true if the node is abstract, false if concrete (have associated behavior)
    bool abstract;
    //true if the node has been expanded by the system
    bool expanded;
    //true if the node is goal-oriented (have a goal)
    bool teleological;
    //true if the node has been already amplified
    bool amplified;
    //goal: sequence of working memory variables in AND form
    std::vector<std::string> goal;
    //list of behaviors contending the node (FIFO), the head of the list wins
    std::vector<std::string> contenders;
    //fading value, when 0 the node is forgotten (UNSTABLE)
    double fading;
    //true if the node is freezed (ie. it cant activate motor-schema)
    bool freezed;
    //true if the internal released, of the associated behavior, is satisfied 
    bool internalReleaser;
    //true if the node is in background, hence variables are not overwrite
    bool background;
    
    //TRUE if the node is sequential, ie. the sons are executed in sequence
    bool is_sequential;
    //TRUE if the node is part of a sequence
    bool in_sequence;
    //TRUE if node is the current executing node of the sequence
    bool sequentialReleaser;

    void loadWeights();

    void saveWeights();

    /** function setting the node to background (along with the associated tree) 
     *
     * @return the list of all nodes in the rooted subtree
     */
    void setBackground(bool value);

    /** function transforming a subtree into a list of nodes 
     *
     * @return the list of all nodes in the rooted subtree
     */
    std::vector<WM_node *> tree2list();

    /** function transforming a subtree into a list of nodes (only concrete)
     *
     * @return the list of all concrete nodes in the rooted subtree
     */
    std::vector<WM_node *> tree2list_concrete();

    /** function transforming a subtree into a list of nodes (only active nodes)
     *      (ie. released and not accomplished)
     *
     * @return the list of all concrete nodes in the rooted subtree
     *
     *  NOTE: if the root is in a subtree that is not active, the function
     *        returns anyway the list of active nodes, starting from the root.
     *        To avoid this, it is suggested to use isBranchReleased() on the
     *        root-node before this function call.
     */
    std::vector<WM_node *> activetree2list(); //to be replaced with tree2list_active

    /** function computing the emphasis for that instance
     *
     * @param instanceToFind the instance whose emphasis are requested 
     * 
     * @return the total emphasis of all nodes having sharing the same instance
     */
    double getInstanceEmphasis(std::string instanceToFind);

    /** function computing the emphasis of the single node
     *
     * @param check = true, by default the function checks if the node is
     *                      and, if not, 0 is returned.
     * 
     * @return the emphasis of the node
     */
    double emphasis(bool check = true);

    //ADDED on SEED 8.0
    /** function evaluating if a formula is satisfied (for releaser and goal checking).
     *
     * @param formula the formula to be evaluated 
     *
     * @return TRUE if the formula is satisfied
     */
    bool eval(std::string formula);

    /** function controlling the goal for that node
     *
     * @return TRUE if the goal of this node is satisfied
     */
    bool goalStatus();

    /** function controlling the releaser for that node
     *
     * @return TRUE if the releaser of this node is satisfied
     */
    bool releaserStatus();

    /** function checking for the releasers for the whole branch (sup-nodes of 
     *  the current node)
     *
     * @return TRUE if all nodes along the branch are have satisfied releasers
     *
     *  NOTE: this function is combined with releaserStatus() and goalStatus()
     *        to compute the value of releasers and goals for each node of the
     *        branch.
   
     */
    bool isBranchReleased();

    /** function checking the releaser in the whole working memory
     *
     * @param toFind instance to find
     * @return TRUE if it exists at least one node sharing "toFind" as instance
     *              having a satisfied releaser and unsatisfied goal, 
     *              FALSE otherwise
     */
    bool isReleased(std::string toFind);

    //ADDED on SEED 7.2
    /** function that updates (set emphasis) the contribution of a node (the subtree will inherit this value)
     *
     * @param contribution_name instance of the schema that is contributing or generic id of a contribution
     * @param contribution_value value of the additional emphasis
     */
    void setContribution(std::string contribution_name, double contribution_value);

    //ADDED on SEED 7.2
    /** function that gets the contribution of a node
     *
     * @param contribution_name instance of the schema that is contributing or generic id of a contribution
     * @return value of the contribution as double* (if contribution is not set, NULL is retruned)
     */
    double * getContribution(std::string contribution_name);

    //REMOVED on SEED 7.2
    // /** function that updates (set emphasis) the contribution of a subtree to a certain value
    //  *
    //  * @param contribution_name instance of the schema that is contributing or generic id of a contribution
    //  * @param contribution_value value of the additional emphasis
    //  */
    // void updateContribution(std::string contribution_name, double contribution_value);

    // /** function that amplifies (add emphasis) a subtree of a certain factor
    //  *
    //  * @param contributingSchema instance of the schema that is contributing
    //  * @param factor value of the additional emphasis
    //  */
    // void amplify(std::string contributingSchema, double factor);

    // /** function that amplifies a subtree of a certain factor, in this case,
    //  *  also the hierarchical contributions of sup-nodes is considered
    //  *
    //  * @param contributingSchema instance of the schema that is contributing
    //  * @param factor value of the additional emphasis
    //  */
    // void amplify_hierarchical(std::string schemaInstance, double factor);

    /** function that searches and amplifies all nodes of the sub-tree that have 
     *  reached their goal
     */
    void amplifyNodes();
    
    /** function that searches and updates all sequences of the sub-tree, 
     *  if the current executing node of the sequence is accomplished, the
     *  next node is enabled.
     * 
     *      added 5/6/2020 in seed 3.0
     */
    void updateSequentialNodes();

    /** function that searches all faded nodes of the sub-tree and removes
     *  them
     */
    void forgetNodes();

    /** function that searches for a node that have to be expanded
     * 
     *  @return the reference to the first node found that is not expanded
     */
    WM_node* getExpandableNode();

    /** function allocating a new node as a new son of this node
     * 
     *  @param sonInstance the instance of the node to be added
     * 
     *  @return the reference the newly created node
     */
    WM_node* addSon(std::string sonInstance);

    /** function searching all nodes sharing a given name (not instance)
     * 
     *  @param name the name of the instances to be found e.g. instances
     *         foo(a) and foo(b) have the same name "foo"
     * 
     *  @return the vector containing found nodes
     */
    std::vector<WM_node *> getNodesByName(std::string name);

    /** function searching all nodes sharing the same instance 
     * 
     * @param name instance to be found
     * 
     * @return list of all nodes with the same instance
     */
    std::vector< WM_node*> getNodesByInstance(std::string name);
    
    /** function that seeks and returns the specific node given its ID
     *
     * @param id_to_find: number that identify the node in the WM
     * @return the node associated to the id
     */
    WM_node* getNodeById(int id_to_find);
    
    /** function that  seeks and returns the specific node following the instances in "path"
     *
     * @param path: vector where the first element is the target node and the
     *              last element is a son of the current node
     * @return the node whose instance is the first element of path
     */
    WM_node* getNodeByPath(std::vector<std::string> path);
    
    /** function that returns the path to the current node
     *
     * @return the vector containing all instances on the branch, where the
     *         first element is the current node and the last is a son of 
     *         "alive"
     * 
     *         Note: the function returns NULL if the path is wrong
     */
    std::vector<std::string> getPath();

    /** function checking if a behavior (concrete) with a specific name is 
     *  awake (running)
     * 
     * @param name instance to find
     * @return TRUE if it exists a behavior named with "name" that is running
     *              (awake)
     */
    bool isAwake(std::string name);

    //NO MORE USED after seed_learn inclusion
    void updateMagnitude();

    void tic();

    void ticBranch();

    /** function checking if a subtree has at least one node running (ie. that
     *  doing something)
     * 
     * @return TRUE if it exists an awake node in the subtree (ie. having 
     *              a satisfied releaser)
     */
    bool isWorking();
};



//LTM class
class LongTermMemory{
public:
    
    //LongTermMemory();
    
    virtual bool loadLTM(std::string) = 0;
    
    virtual bool loadNodeSemantics(WM_node *) = 0;
    
    virtual std::string query(std::string) = 0;
    
    //virtual WM_node *loadNode(std::string) = 0;
    
    //virtual WM_node *saveNode(WM_node *) = 0;
    
    virtual bool close() = 0;
    
};



//taken from seed.cpp

extern WM_varmap WMV;

extern WM_node *WM;

extern LongTermMemory *LTM;



//FUNZIONI GLOBALI

std::vector<std::string> instance2vector(std::string);

bool wakeUp(std::string, std::string);

void printWM(WM_node *);

void remove(WM_node * node);

void *execution(void *);

bool dead();

std::vector<WM_node *> sortNodes(std::vector<WM_node *>);



// **** **** **** **** Behavior **** **** **** **** //


class Behavior {
public:

    Behavior();
    //funzione di uscita del behavior
    virtual void exit() = 0;
    //funzione di ingresso del behavior
    virtual void start() = 0;
    //schema percettivo del behavior (ritorna lo stato del releaser interno)
    virtual bool perceptualSchema() = 0;
    //schema motorio del behavior
    virtual void motorSchema() = 0;

    //update del ritmo considerando affordance ed enfasi dalla WM
    void updateRtm(double affordance, double max, double min);
    //update del ritmo considerando la sola enfasi dalla WM - EX setDefaultRtm
    void updateRtm();
/*
    //funzione di aggiornamento del periodo con legge di weber e soglia
    // il max diventa la soglia oltre il quale lo stimolo viene percepito
    //ATTENZIONE: qui il ritmo non viene integrato con la magnitudine
    void updateRtm_weber2(double newStimulus, double maxStimulus, double minStimulus);

    //funzione di aggiornamento della frequenza con legge di Weber
    //  gli ultimi 2 parametri sono fittizi servono solo per evitare di modificare le chiamate a funzione
    //ATTENZIONE: qui il ritmo non viene integrato con la magnitudine
    void updateRtm_weber(double newStimulus, double fake1, double fake2);
 */

    void setSelfContribution(double c);

    void setContribution(std::string source, std::string target, double c);

    //remove contribution (reset value to NULL)
    void delContribution(std::string source, std::string target);

    //ritorna il nuovo contributo calcolato
    double addContribution(std::string source, std::string target, double c);

    double getRtm();

    void setRtm(double newRtm);

    //setta l'istanza del behavior (nome+parametri)
    void setInstance(std::string newInstance);

    //restituisce l'istanza del behavior, compreso di parametri
    std::string getInstance();

    void setReleaser(bool r);

    bool getReleaser();

    double getStimulus();

    double setStimulus(double s);

    bool isUpdated();

    bool isBackground();
    
    pthread_t getTid();

    void setTid(pthread_t t);

    // set the rate of the behavior in Hz
    //  i.e., the frequency of activation for perceptual/motor schemata
    //
    //  NOTE: this is soft real-time
    inline void setRate(double r){
        rate = r;
    }

    // get the rate of the behavior in Hz
    //  i.e., the frequency of activation for perceptual/motor schemata
    //
    //  NOTE: this is soft real-time
    inline double getRate(){
        return rate;
    }

    // user-friendly WM lock
    inline void wm_lock(){
        pthread_mutex_lock(&memMutex);
    }

    // user-friendly WM unlock
    inline void wm_unlock(){
        pthread_mutex_unlock(&memMutex);
    }

    /** user-friendly WM function used to retrieve all nodes associated to this
     *   behavior (having the same instance)
     *
     * @return: list of pointers to the associate WM nodes.
     *
     * NOTE: this function need WM to be locked!
    */
    inline std::vector<WM_node *> wm_get_associated_nodes(){
        return WM->getNodesByInstance(this->getInstance());
    }

    /** user-friendly WM function used to remove all nodes associated to this
     *   behavior. Since all nodes are removed, this behavior is also exited
     *
     * NOTE: this function need WM to be locked!
    */
    inline void wm_remove_associated_nodes(){
        std::vector<WM_node *> my_nodes = WM->getNodesByInstance(this->getInstance());
        for(size_t i=0; i<my_nodes.size(); i++)
            remove(my_nodes[i]);
    }

    /** user-friendly WM function used to retrieve children from the associated nodes.
     *   Since multiple associated nodes may be available, this function will return
     *   a matrix of nodes (associated x children)
     *
     * @return: list of pointers to the child WM nodes.
     *
     * NOTE: this function need WM to be locked!
    */
    inline std::vector<std::vector<WM_node *>> wm_get_child_nodes(){
        std::vector<std::vector<WM_node *>> children;
        std::vector<WM_node *> my_nodes = WM->getNodesByInstance(this->getInstance());
        for(size_t i=0; i< my_nodes.size(); i++){
            children.push_back(my_nodes[i]->son);
        }
        return children;
    }

    /** user-friendly WM function used to add new children to the associated nodes.
     *
     * @param child_instance: instance of the node to add as child of this node.
     *
     * @return: list of pointers to the newly created WM nodes.
     *
     * NOTE: this function need WM to be locked!
    */
    inline std::vector<WM_node *> wm_add_child_node(std::string child_instance){
        std::vector<WM_node *> new_nodes;
        std::vector<WM_node *> my_nodes = WM->getNodesByInstance(this->getInstance());
        for(size_t i=0; i< my_nodes.size(); i++)
            new_nodes.push_back(my_nodes[i]->addSon(child_instance));
        return new_nodes;
    }

    /** user-friendly WM function used to nemove children from the associated nodes.
     *
     * @param child_instance: instance of the child node to remove.
     *
     * NOTE: this function need WM to be locked!
    */
    inline void wm_remove_child_node(std::string child_instance){
        std::vector<WM_node *> possible_children = WM->getNodesByInstance(child_instance);
        for(size_t i=0; i< possible_children.size(); i++)
            if(possible_children[i]->father->instance == this->getInstance())
                remove(possible_children[i]);
    }

    // safely get the args of this behavior
    //  set warning to false to avoid online warnings for on existing arguments 
    inline std::string arg(int i, bool warning=true){
        if((size_t)i>=args.size()){
            if(warning) 
                std::cout<<ansi::yellow<<"SEED-WARNING: behavior "<<instance<<" has no argument number "<<i<<ansi::end<<std::endl;
        }
        else
            return args[i];
        return "";
    }

    /**
     *
     * @param instances vettore delle istanze da amplificare
     *
     */
    void amplifyAllInstances(std::string source, std::vector<WM_node *> instances);
    
    bool isFrozen();

    /** funzione di settaggio del releaser interno nei nodi della WM
     *
     * @param ir: releaser interno del processo, viene dato in output
     *      dallo schema percettivo.
     *
     * NB. prima di eseguire questa funzione va lockato il semaforo della
     *      WM.
     */
    void setNodeInternalReleaser(bool ir);

    /** funzione per la scrittura del file di debug (PROJ/debug/instance)
     *
     * @param s: stringa da scrivere nel file di debug
     */
    void debug_write(std::string filename ,std::string s);

    /** funzione di controllo del behavior in WM
     *
     * @return TRUE se il behavior è presente in WM
     *
     * inoltre, setta "instance" e "rtm" sulla base
     * del nodo eventualmente trovato
     */
    bool perceptWM();
    
    /* Interface functions for the WMVs */
    
    //NOTE: this is the updated version of the wmv_compete() function!
    /**
     * funzione di modifica delle variabili in ambito protetto
     * può essere utilizzata per modificare i valori degli attuatori
     * se la risorsa è disponibile il valore viene inviato (sommandolo
     * ad altri se necessario), altrimenti la funzione non ha effetto.
     *
     * @param receiver: istanza del behavior da contattare
     *  se NULL il sistema considera la variabile da modificare
     *  come non condivisa (modificandola direttamete) [CURRENTLY UNUSED!]
     * @param var: nome della variabile di sistema da modificare
     * @param value: valore da inserire nella variabile
     *
     * @return 1 se il valore è stato inviato, -1 se è stato parzialmente inviato
     *  (ie. non è stato il primo a scrivere), 0 altrimenti
     */
    template<class T>
    int wmv_compete(std::string receiver, std::string var, T value) {
        
        int out = 0;
        
        //seed4.0: receiver is NOT used now but it can be useful in the future
        (void) receiver;
//        //recupera il nodo che corrisponde al ricevitore
//        std::vector< WM_node*> recNode = WM->getNodesByInstance(receiver);
        
        //std::cout<<ansi::cyan<<this->getInstance()<<" compete for "<<var<<ansi::end<<std::endl;

        if(this->background && var[0]!= '#')
            var = "#" + var;

        if(WM == NULL)
            return out;
        
        
        WMV.compete<T>(this->getInstance(), var, value);
        
        if( WMV.owner(var) == this->getInstance() )
            out = 1;
        
        //std::cout<<ansi::cyan<<this->getInstance()<<" competed for "<<var<<" with result "<<out<<ansi::end<<std::endl;
        
        return out;
    }


    /**
     * This function removes the current behavior from competition. It is used in
     *  combination with wmv_solve_once function which solves the competition wthout
     *  resetting it.
     * 
     * @param var: name of the variable to be 
     *
     *  NOTE: added in SEED 6.0 (ROS2 version)
     */
    template<class T>
    void wmv_withdraw(std::string var) {
        
        //seed4.0: receiver is NOT used now but it can be useful in the future
//        //recupera il nodo che corrisponde al ricevitore
//        std::vector< WM_node*> recNode = WM->getNodesByInstance(receiver);

        WMV.remove<T>(this->getInstance(),var);
    }
    
    /**
     * Solve the competition for the variable and return the selected value.
     *
     * @param var: name of the variable whose competition have to be solved
     * @param persistence: if true, the old winner persists if no competitors
     *                     are found (TRUE by default).
     *
     * @return the value that is selected from the candidate ones
     * 
     * NOTE: after the Solve the competition is reset, hence all candidates are removed.
     *       To participate againt to the competition, candidates have to compete again.
     * 
     * NOTE: if persistence is true and no competitors are present in the 
     *       current row, this function returns the old winner if released. 
     *       Probably a watchdog should be used also.
     */
    template<class T>
    T wmv_solve(std::string var, bool persistence = true) {

        std::vector<WM_value> cand = WMV.candidates(var);
        int best_index = -1;
        double best_emph = -1;
        double app_emph;
        int ex_aequo_winners = 0;

        T final_value = T();
        std::stringstream log;
        
        log<<var<<", ";
        for(size_t i=0; i<cand.size(); i++){
            log<<"("<<cand[i].instance<<",";
            if( WM->isReleased(cand[i].instance) ){
                app_emph = WM->getInstanceEmphasis(cand[i].instance);
                log<<"true,"<<app_emph<<"), ";
                if( app_emph > best_emph ){
                    best_index = i;
                    best_emph = app_emph;
                    ex_aequo_winners = 0;
                }
                //check for ex-aequo, it could happen when integer emphasis (or zeros) are used
                else if( app_emph == best_emph ){
                    ex_aequo_winners++;
                }
            }
            else
                log<<"false,0), ";
        }

        
        if(persistence) {
            //check the old winner
            std::string old_winner = WMV.owner(var);
            std::vector<WM_node * > winVec = WM->getNodesByInstance(old_winner);

            //if the old_winner exists and is still the best
            if(old_winner != "" && WM->getInstanceEmphasis(old_winner) > best_emph){
                //return its value if it is still enabled!
                for(size_t i=0; i<winVec.size(); i++){
                    if( winVec[i]->internalReleaser && winVec[i]->isBranchReleased() ){
                    //if( winVec[i]->isBranchReleased() ){
                        final_value = WMV.get<T>(var); //return the value of the old winner
                        best_index = -1; //to skip the following if
                        log<<"OLD_WINNER("<<old_winner<<","<<WM->getInstanceEmphasis(old_winner)<<")";
                        break;
                    }
                }
            }
        }
        
        //else, if a new best exists
        if(best_index != -1 && ex_aequo_winners == 0){
            //select it!
            WMV.select<T>(var,best_index);
            WMV.reset<T>(var);
            final_value = WMV.get<T>(var);
            log<<"WINNER("<<cand[best_index].instance<<","<<best_emph<<")";
        }
        else
            log<<"NO-WINNER";
        //oth. there is no winner so far

        //WRITE LOG COMPETITION
        std::ofstream f;
        f.open(SEED_HOME_PATH + "/log/" + SEED_NAME + "_log.txt",std::ios::app);
        //f << log.str()<<", "<<ros::Time::now()<<"\n";
        f << log.str()<<"\n";
        f.close();

        return final_value;
        
    }


    /**
     * Solve ONCE the competition for the variable and return the selected value.
     *  Differently from Solve, here the competition is NOT reset, hence candidates 
     *  are NOT removed by default.
     *
     * @param var: name of the variable whose competition have to be solved
     *
     * @return the value that is selected from the candidate ones
     * 
     * NOTE: Candidates that are not in WM are automatically removed from the comeptition,
     *       otherwise, candidates can call "wmv_withdraw" to unsubscribe.
     */
    template<class T>
    T wmv_solve_once(std::string var) {

        std::vector<WM_value> cand = WMV.candidates(var);
        int best_index = -1;
        double best_emph = -1;
        double app_emph;
        int ex_aequo_winners = 0;

        T final_value = T();
        std::stringstream log;
        
        log<<var<<", ";
        //for(auto i=0; i<cand.size(); i++){
        size_t i=0;
        while(i<cand.size()) {
            log<<"("<<cand[i].instance<<",";
            if( WM->getNodesByInstance(cand[i].instance).empty() ) {
                // remove candidate from competition
                WMV.remove<T>(instance, var);
                cand.erase(cand.begin() + i);
                log<<"removed), ";
                i--;
            }
            else if( WM->isReleased(cand[i].instance) ){
                app_emph = WM->getInstanceEmphasis(cand[i].instance);
                log<<"true,"<<app_emph<<"), ";
                if( app_emph > best_emph ){
                    best_index = i;
                    best_emph = app_emph;
                    ex_aequo_winners = 0;
                }
                //check for ex-aequo, it could happen when integer emphasis (or zeros) are used
                else if( app_emph == best_emph ){
                    ex_aequo_winners++;
                }
            }
            else
                log<<"false,0), ";

            i++;
        }
        
        //else, if a new best exists
        if(best_index != -1 && ex_aequo_winners == 0){
            //select it!
            WMV.select<T>(var,best_index);
            final_value = WMV.get<T>(var);
            log<<"WINNER("<<cand[best_index].instance<<","<<best_emph<<")";
        }
        //oth. there is no winner so far
        else{
            WMV.deselect<T>(var);
            log<<"NO-WINNER";
        }

        //WRITE LOG COMPETITION
        std::ofstream f;
        f.open(SEED_HOME_PATH + "/log/" + SEED_NAME + "_log.txt",std::ios::app);
        //f << log.str()<<", "<<ros::Time::now()<<"\n";
        f << log.str()<<"\n";
        f.close();

        return final_value;
    }


    /**
     * Set the value of the variable directly, bypassing the competition
     *
     * @param var: name of the variable to be set
     * @param value: value to be assigned
     */
    template<class T>
    void wmv_set(std::string var, T value) {
        WMV.set<T>(this->getInstance(), var,value);
    }
    
    /**
     * Get the value of a variable (this is just an interface for wmv_get())
     *
     * @param var: name of the variable
     *
     * @return the current value of the variable
     */
    template<class T>
    T wmv_get(std::string var) {
        return WMV.get<T>(this->getInstance(), var);
    }
    
    /**
     * Wait until the variable reaches a specific value (condWait)
     *  WARNING: this is BLOCKING!! the behavior will be stuck on this line
     *           until "value" is reached
     * 
     * NOTE: mutex should be already locked before this call
     *
     * @param var: name of the variable
     * @param value: expected value
     *
     */
    template<class T>
    void wmv_wait(std::string var, T value) {
        
        std::cout<<this->getInstance()<<" condWaiting for "<<var<<": start"<<std::endl;
        
        while( WMV.get<T>(this->getInstance(), var) != value ){
            pthread_mutex_unlock(&memMutex);
            //release scheduler
            usleep(0.01 * MSECOND); //fixed for now
            
            pthread_mutex_lock(&memMutex);
        }
        
        std::cout<<this->getInstance()<<" condWaiting for "<<var<<": value REACHED"<<std::endl;
    }
    

private:
    //id of the executing thread
    pthread_t tid;

    //DEPRECATED: now name can only be retrieved through the getName function (which is class-specific)
    //std::string name;

    // instance of the behavior (name+arguments)
    std::string instance;

    // this variable regulates the frequency of percept/motor schemata (from SEED 7.1)
    //  rate is in Hz!
    double rate;

    bool releaser;
    //true se il rtm è stato aggiornato
    bool updated;
    //lista dei nodi associati al processo
    std::vector< WM_node * > myNodeList;

    //vero se tutti i nodi del behavior sono in background
    bool background;

    // arguments vector
    //  NOTE: args[0] has the name of the behavior 
    std::vector<std::string> args;

    // NOTE: all these attributes should be probably purged

    //rhythm of the behavior, ie. period of time between 2 activations (OLD unused from SEED 7.1)
    double rtm;
    //updateRtm Weber law
    double oldStimulus;
    double oldPeriod;
    double k_weber;
};


// **** **** **** **** BBS **** **** **** **** //

// FACTORY WITH SELF-REGISTERING TYPES:
//	This class implements the factory side of a factory pattern design.
//	In order to be used, behavior must be registered (i.e. added) to the
//	repository. This is performed inside behavior classes through the
//	"registered" static variable.
class BehaviorBasedSystem {
public:
    
    //BehaviorBasedSystem(); //use default constructor
    
    inline static bool add(std::string name, std::function<Behavior*(std::string)> creation_function){
        repository[name] = creation_function;
        return true;
    }
    
    inline Behavior * load(std::string name, std::string instance){
        return repository[name](instance);
    }
    
    inline bool exists(std::string name){
        if (auto it = repository.find(name); it == repository.end())
            return false;
        return true;
    }
    
private:

    inline static std::unordered_map<std::string, std::function<Behavior*(std::string)> > repository; //this should be done in the .cpp file
    
};

extern BehaviorBasedSystem *BBS;




class AliveBehavior : public Behavior {
public:

    AliveBehavior(std::string name) {
        setInstance(name);
        //setRtm(QUIESCENCE); //DEPRECATED since SEED 7.1
        setRate(100);
    }
    bool perceptualSchema() {
        return true;
    }

    void motorSchema() {
        WM_node* expNode;

        pthread_mutex_lock(&memMutex);

        if (dead())
            return;
        //per adesso si mette un ritmo di default
        //setRtm(0.01); //DEPRECATED since SEED 7.1
        //amplifica gli eventuali nodi
        WM->amplifyNodes();
        //update sequences into the WM (added 5/6/2020 in seed 3.0)
        WM->updateSequentialNodes();
        //se esiste un nodo da espandere
        if ((expNode = WM->getExpandableNode()) != NULL) {

            //espandi il nodo (motor schema)
                        std::cout<<"EXPAND: "<<expNode->instance<<"\n";
//            if (expNode->father != NULL) {
//                expNode->magnitude = ((expNode->father->magnitude)*(expNode->ltMagnitude)) / expNode->father->son.size();
//                //expNode->amplification=(expNode->father->amplification)/expNode->father->son.size();
//            }
            
            //se la risposta non è una definizione semantica
            if ( !LTM->loadNodeSemantics(expNode) ) {
                //rimuovi il nodo
                std::cout << "SYSTEM: no semantic for " << expNode->instance << "\n";
                remove(expNode);
            }//altrimenti espando il nodo
            else {
                //se non vi sono altre istanze sveglie
                if (!(WM->isAwake(expNode->instance))){
                    //prova ad allocare il thread
                    if (wakeUp(expNode->name, expNode->instance))
                        //se lo hai allocato allora è concreto
                        expNode->abstract = false;
                    else {
                        //std::cout<<"SYSTEM: "<<expNode->instance<<" ABSTRACT!\n";
                        //altrimenti è astratto
                        expNode->abstract = true;
                    }
                } else
                    //altrimenti (ie. esiste una istanza concreta sveglia) è concreto
                    expNode->abstract = false;

                //carica i pesi per il learning
                expNode->loadWeights();
                
                //assign node id
                expNode->id = seed_wm_id;
                seed_wm_id++;

//                //aggiungi l'istanza come contributo
//                std::vector<std::string> app_inst = instance2vector(expNode->instance);
//                std::stringstream app_ss;
//                for(int i=1; i<app_inst.size(); i++){
//                    app_ss<<i<<":"<<app_inst[i];
//                    //la somma dei contributi dall'istanza deve fare 1!
//                    addContribution(app_ss.str(),expNode->instance, 1/(app_inst.size()-1) );
//                    app_ss.str("");
//                }
//                //se il nodo non ha argomenti allora si potenzia di 1 per rendere la competizione fair
//                //  altrimenti i nodi con tanti argomenti vengono potenziati troppo!
//                if(app_inst.size()==1)
//                    addContribution(expNode->instance,expNode->instance, 1);

                expNode->expanded = true;
                //wmv_set<std::string>("alive", expNode->instance);
                wmv_set<std::string>("alive", expNode->instance);
            }

                        std::cout<<"...DONE\n";
        }

        //aggiorna la magnitudine dei nodi dopo l'inserimento
        //WM->updateMagnitude();
        pthread_mutex_unlock(&memMutex);
    }

    void start() {
        std::cout << "SYSTEM: hello world!\n";
    }

    void exit() {
        std::cout << "SYSTEM: alive removed, shutting down \n";
        //LTM->close(); //removed by the main thread!
    }
};

} //seed namespace

#endif	/* SEED_H */

