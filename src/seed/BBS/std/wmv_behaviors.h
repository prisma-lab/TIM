#ifndef WMV_BehaviorS_H
#define WMV_BehaviorS_H

#include "seed.h"

using namespace seed;

/*
*   This header collects behaviors for the management of the 
*   Working Memory Variables (WMVs)
*
*   Added in SEED 6.0 (ROS2 version)
*/

#include <ctype.h>

/*
*   Superclass for WMV behaviors
*/
class WMVBehavior : public Behavior
{
public:
    // void exit(){
    // }
    // void start(){
    //}
    std::string getStringType(std::string strVal)
    {
        if ( (isdigit(strVal[0]) || strVal[0] == '-') && strVal.find(".") != std::string::npos)
        {
            return "double";
        }
        else if (isdigit(strVal[0]) || strVal[0] == '-')
        {
            return "int";
        }
        else if (strVal == "TRUE" || strVal == "FALSE" || strVal == "true" || strVal == "false")
        {
            return "bool";
        }
        else
            return "string";
    }
    void cond_loop()
    {
        if (rate == 0){
            //remove(WM->getNodesByInstance(this->getInstance())[0]);
            wmv_set<bool>(this->getInstance() + ".done", true);
        }
        else
            setRtm(rate);
    }

protected:
    std::string varname, varvalue, vartype;
    double rate;
};

/*
 *   set a WMV to a specific value:
 *       set(varName,varValue,rtm=0)
 *           varName: name of the WMV to set
 *           varValue: desired value (bool string or double)
 *           rtm (optional): period-rate of setting (only once if 0)
 * 
 *      NOTE: the variable is set to a default (null) value on exit.
 */
class SetBehavior : public WMVBehavior
{
public:
    SetBehavior(std::string instance){
        setInstance(instance);
        setRtm(QUIESCENCE);

        varname = instance2vector(instance)[1];
        varvalue = instance2vector(instance)[2];

        if (instance2vector(instance).size() == 4)
            rate = ston(instance2vector(instance)[3]);
        else
            rate = 0;

        // guess var type
        vartype = getStringType(varvalue);
        std::cout << "vartype: " << vartype << "\n";
    }
    std::string getName(){
        return "set";
    }
    bool perceptualSchema()
    {
        return true;
    }
    void motorSchema()
    {
        pthread_mutex_lock(&memMutex);
        if (vartype == "double")
        {
            wmv_set<double>(varname, ston(varvalue));
        }
        else if (vartype == "int")
        {
            wmv_set<double>(varname, ston(varvalue));
        }
        else if (vartype == "bool")
        {
            wmv_set<bool>(varname, (varvalue == "TRUE" || varvalue == "true") ? true : false);
        }
        else
        {
            wmv_set<std::string>(varname, varvalue);
        }

        cond_loop();
        pthread_mutex_unlock(&memMutex);
    }
    void start()
    {
    }
    void exit()
    {
        // clean the variable
        pthread_mutex_lock(&memMutex);
        if (vartype == "double")
        {
            wmv_set<double>(varname, 0.0);
        }
        else if (vartype == "int")
        {
            wmv_set<double>(varname, 0);
        }
        else if (vartype == "bool")
        {
            wmv_set<bool>(varname, false);
        }
        else
        {
            wmv_set<std::string>(varname, varvalue);
        }
        wmv_set<bool>(this->getInstance() + ".done", false);
        pthread_mutex_unlock(&memMutex);
    }
};


/*
 *   get the value of a WMV (NO contention). The solution is plot on stdout as a blue string.
 *       get(varName,varType,period)
 *           varName: name of the WMV
 *           varType: type of the WMV (string, bool, int, double)
 *           period (optional): rtm of the behavior, frequence of plotting
 */
class GetBehavior : public WMVBehavior
{
public:
    GetBehavior(std::string instance){
        setInstance(instance);
        setRtm(QUIESCENCE);

        // varowner = instance2vector(instance)[1]; //this is not used anymore from SEED 4.0
        varowner = "none";
        varname = instance2vector(instance)[1];
        vartype = instance2vector(instance)[2];

        //nh = rclcpp::Node::make_shared("solve");
        //ex.add_node(nh); // does it work? NO

        if (instance2vector(instance).size() == 5){
            rate = ston(instance2vector(instance)[3]);
        }
        else{
            rate = 0;
        }
    }
    std::string getName(){
        return "get";
    }
    bool perceptualSchema()
    {
        return true;
    }
    void motorSchema()
    {
        pthread_mutex_lock(&memMutex);

        if(!wmv_get<bool>(this->getInstance() + ".done")){

            std::cout<<ansi::blue<<"GET "<<varname<<":"<<std::endl;

            if (vartype == "double")
            {
                std::cout<<"\t current value ("<<vartype<<") is "<<wmv_get<double>(varname);
            }
            else if (vartype == "int")
            {
                std::cout<<"\t current value ("<<vartype<<") is "<<wmv_get<double>(varname);
            }
            else if (vartype == "bool")
            {
                std::cout<<"\t current value ("<<vartype<<") is "<<wmv_get<bool>(varname);
            }
            else
            {
                std::cout<<"\t current value ("<<vartype<<") is "<<wmv_get<std::string>(varname);
            }

            std::cout<<ansi::end<<std::endl;
        }

        cond_loop();
        // ros::spinOnce();
        pthread_mutex_unlock(&memMutex);
    }
    void start()
    {
    }
    void exit()
    {
        pthread_mutex_lock(&memMutex);
        wmv_set<bool>(this->getInstance() + ".done", false);
        pthread_mutex_unlock(&memMutex);
    }

protected:
    std::string varowner;
    std::string topic;
};



/*
 *   timer to set a WMV to a specific value. The timer restarts every time the
 *   node becomes active (realeased and not accomplished):
 *       timer(varName,varValue,timer)
 *           varName: name of the WMV to set
 *           varValue: desired value (bool string or double)
 *           timer: waiting period, after which the variable is set (only once).
 * 
 *      NOTE: the variable is set to a default (null) value on exit.
 */
class TimerBehavior : public WMVBehavior
{
public:
    TimerBehavior(std::string instance){
        setInstance(instance);
        setRtm(QUIESCENCE);

        varname = instance2vector(instance)[1];
        varvalue = instance2vector(instance)[2];
        waiting_time = ston(instance2vector(instance)[3]);

        rate = 0.01; //default

        // guess var type
        vartype = getStringType(varvalue);
        std::cout << "vartype: " << vartype << "\n";

        timer_started = false;
        timer_stopped = false;
    }
    std::string getName(){
        return "timer";
    }
    bool perceptualSchema()
    {
        wm_lock();
        //if node is active
        if(WM->isReleased(this->getInstance())){
            //if timer is not started yet
            if(!timer_started){
                //restart timer
                timer.tic();
                timer_started = true;
            }
        }
        //otherwise, if node is not active
        else{
            //restart the timer, and wait for node to be active again
            timer_started = false;
            timer_stopped = false;
        }
        wm_unlock();

        return true;
    }
    void motorSchema()
    {
        //if timer has not been stopped (variable aready set) and timer elapsed
        if(!timer_stopped && timer.toc() >= waiting_time){
            //Set the variable
            wm_lock();
            if (vartype == "double")
            {
                wmv_set<double>(varname, ston(varvalue));
            }
            else if (vartype == "int")
            {
                wmv_set<double>(varname, ston(varvalue));
            }
            else if (vartype == "bool")
            {
                wmv_set<bool>(varname, (varvalue == "TRUE" || varvalue == "true") ? true : false);
            }
            else
            {
                wmv_set<std::string>(varname, varvalue);
            }
            cond_loop();
            wm_unlock();

            //stop the timer (no need to set variable again)
            timer_stopped = true;
        }
    }
    void start()
    {
    }
    void exit()
    {
        // clean the variable
        wm_lock();
        if (vartype == "double")
        {
            wmv_set<double>(varname, 0.0);
        }
        else if (vartype == "int")
        {
            wmv_set<double>(varname, 0);
        }
        else if (vartype == "bool")
        {
            wmv_set<bool>(varname, false);
        }
        else
        {
            wmv_set<std::string>(varname, varvalue);
        }
        wmv_set<bool>(this->getInstance() + ".done", false);
        wm_unlock();
    }

    double waiting_time;
    seed::time::Clock timer;
    bool timer_started;
    bool timer_stopped;
};



/*
 *   compete to set a value for a WMV (suffer of contention):
 *       set(varName,varType,varValue,period)
 *           varName: name of the WMV to contend
 *           varType: type of the WMV to contend (string, bool, int, double)
 *           varValue: desired value (bool string or double)
 *           period (optional): rtm of the behavior, i.e. the rate of the competition (rate of access to the variable)
 */
class CompeteBehavior : public WMVBehavior
{
public:
    CompeteBehavior(std::string instance){
        setInstance(instance);
        setRtm(QUIESCENCE);

        // varowner = instance2vector(instance)[1]; //this is not used anymore from SEED 4.0
        varowner = "none";
        varname = instance2vector(instance)[1];
        vartype = instance2vector(instance)[2];
        varvalue = instance2vector(instance)[3];

        if (instance2vector(instance).size() == 5)
            rate = ston(instance2vector(instance)[4]);
        else
            rate = 0;
    }
    std::string getName(){
        return "compete";
    }
    bool perceptualSchema()
    {

        // NOTE: competition MUST be made in the perceptualSchema, this allows the behavior to subscribe
        //       to the shared variables even if it cannot win. When an event occurs and the behavior is
        //       released it is already subscribed to the competition, avoiding other competeing behaviors
        //       (already released but less emphasized) to "sneak in" before the subscription.

        pthread_mutex_lock(&memMutex);

        if (vartype == "double")
        {
            wmv_compete<double>(varowner, varname, ston(varvalue));
        }
        else if (vartype == "int")
        {
            wmv_compete<double>(varowner, varname, ston(varvalue));
        }
        else if (vartype == "bool")
        {
            wmv_compete<bool>(varowner, varname, (varvalue == "TRUE" || varvalue == "true") ? true : false);
        }
        else
        {
            wmv_compete<std::string>(varowner, varname, varvalue);
        }

        // if(varname == "rover.command")
        //     std::cout<<ansi::magenta<<varvalue<<" competing: "<<seed_global_clk.toc()<<" with emph: "<<WM->getInstanceEmphasis(this->getInstance())<<ansi::end<<std::endl;

        cond_loop();
        // updateRtm(1,2,1); //fixed at maximum
        pthread_mutex_unlock(&memMutex);

        return true;
    }
    void motorSchema()
    {
    }
    void start()
    {
    }
    void exit()
    {
        pthread_mutex_lock(&memMutex);
        wmv_set<bool>(this->getInstance() + ".done", false);
        pthread_mutex_unlock(&memMutex);
    }

protected:
    std::string varowner;
};

/*
 *   solve the competition for a WMV (suffer of contention). The solution is plot on stdout as a blue string.
 *       solve(varName,varType,period)
 *           varName: name of the contended WMV
 *           varType: type of the contended WMV (string, bool, int, double)
 *           period: rtm of the behavior, i.e. duration of the competition (time to compete to that variable)
 *
 *       WARNING: less emphasized behaviors may "sneak in" if the behavior (which is supposed to win) subscribes
 *                late to the competition (e.g, if it is just been allocated)
 *
 *       NOTE: since the emphasis check is made by the solving behavior, to avoid the sneaking in problem a possible
 *             solution could be for competing behaviors to subscribe themselves to the competition ON-START and to
 *             unsubscribe themselves from the competition ON-EXIT. In fact, if competing behaviors are not released
 *             or not sufficiently emphasized they are ignored by the solving behavior anyway.
 */
class SolveBehavior : public WMVBehavior
{
public:
    SolveBehavior(std::string instance){
        setInstance(instance);
        setRtm(QUIESCENCE);

        // varowner = instance2vector(instance)[1]; //this is not used anymore from SEED 4.0
        varowner = "none";
        varname = instance2vector(instance)[1];
        vartype = instance2vector(instance)[2];

        //nh = rclcpp::Node::make_shared("solve");
        //ex.add_node(nh); // does it work? NO

        if (instance2vector(instance).size() == 5){
            rate = ston(instance2vector(instance)[3]);
        }
        else{
            rate = 0;
        }
    }
    std::string getName(){
        return "solve";
    }
    bool perceptualSchema()
    {
        return true;
    }
    void motorSchema()
    {
        pthread_mutex_lock(&memMutex);

        if(!wmv_get<bool>(this->getInstance() + ".done")){

            std::cout<<ansi::blue<<"SOLVE "<<varname<<" competition solved:"<<std::endl;

            if (vartype == "double")
            {
                std::cout<<"\t current value ("<<vartype<<") is "<<wmv_solve<double>(varname);
            }
            else if (vartype == "int")
            {
                std::cout<<"\t current value ("<<vartype<<") is "<<wmv_solve<double>(varname);
            }
            else if (vartype == "bool")
            {
                std::cout<<"\t current value ("<<vartype<<") is "<<wmv_solve<bool>(varname);
            }
            else
            {
                std::cout<<"\t current value ("<<vartype<<") is "<<wmv_solve<std::string>(varname);
            }

            std::cout<<ansi::end<<std::endl;

        }

        cond_loop();
        // ros::spinOnce();
        pthread_mutex_unlock(&memMutex);
    }
    void start()
    {
    }
    void exit()
    {
        pthread_mutex_lock(&memMutex);
        wmv_set<bool>(this->getInstance() + ".done", false);
        pthread_mutex_unlock(&memMutex);
    }

protected:
    std::string varowner;
    std::string topic;
};

#endif	/* WMV_BehaviorS_H */