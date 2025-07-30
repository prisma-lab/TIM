#ifndef ROS_BehaviorS_H
#define ROS_BehaviorS_H

//by default this variable is false
#ifndef SEED_INTERFACES
#define SEED_INTERFACES 0
#endif

#include "seed.h"

#if SEED_INTERFACES
#include "seed_interfaces/srv/set_wmv.hpp"
#include "seed_interfaces/srv/add_node.hpp"
#include "seed_interfaces/srv/del_node.hpp"

#include "seed_interfaces/srv/set_regulation_value.hpp"
#include "seed_interfaces/srv/add_regulated_nodes.hpp"
#include "seed_interfaces/srv/del_regulated_nodes.hpp"
#endif

#include "tf2/exceptions.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"

using namespace seed;

/*
*   This header collects behaviors based on ROS2 communication
*
*   Added in SEED 6.0 (ROS2 version)
*/

class RosStreamBehavior : public Behavior{
public:   
    RosStreamBehavior(std::string instance){
        setInstance(instance);
        setRtm(QUIESCENCE);

        sb_local_stream = nh->create_subscription<std_msgs::msg::String>(SEED_NAME + "/stream", 0, std::bind(&RosStreamBehavior::ros_input_callback, this, _1));
        sb_local_state = nh->create_subscription<std_msgs::msg::String>(SEED_NAME + "/state", 0, std::bind(&RosStreamBehavior::ros_state_callback, this, _1));
        sb_local_reg = nh->create_subscription<std_msgs::msg::String>(SEED_NAME + "/regulation", 0, std::bind(&RosStreamBehavior::ros_regulation_callback, this, _1));

        sb_global_stream = nh->create_subscription<std_msgs::msg::String>("seed/stream", 0, std::bind(&RosStreamBehavior::ros_input_callback, this, _1));
        sb_global_state = nh->create_subscription<std_msgs::msg::String>("seed/state", 0, std::bind(&RosStreamBehavior::ros_state_callback, this, _1));

#if SEED_INTERFACES
        ss_wmv = nh->create_service<seed_interfaces::srv::SetWmv>(SEED_NAME + "/set_wmv", std::bind(&RosStreamBehavior::service_set_variable, this, _1, _2));
        ss_add = nh->create_service<seed_interfaces::srv::AddNode>(SEED_NAME + "/add_node", std::bind(&RosStreamBehavior::service_add_node, this, _1, _2));
        ss_del = nh->create_service<seed_interfaces::srv::DelNode>(SEED_NAME + "/del_node", std::bind(&RosStreamBehavior::service_del_node, this, _1, _2));

        ss_set_reg = nh->create_service<seed_interfaces::srv::SetRegulationValue>(SEED_NAME + "/set_regulation_value", std::bind(&RosStreamBehavior::service_set_regulation_value, this, _1, _2));
        ss_add_reg = nh->create_service<seed_interfaces::srv::AddRegulatedNodes>(SEED_NAME + "/add_regulated_nodes", std::bind(&RosStreamBehavior::service_add_regulated_nodes, this, _1, _2));
        ss_del_reg = nh->create_service<seed_interfaces::srv::DelRegulatedNodes>(SEED_NAME + "/del_regulated_nodes", std::bind(&RosStreamBehavior::service_del_regulated_nodes, this, _1, _2));
#endif
    }
    std::string getName(){
        return "rosStream";
    }
    bool perceptualSchema(){
        //std::cin>>input;
        pthread_mutex_lock(&memMutex);
        //ros::spinOnce();
        if(dead())
            return false;

        if(!rclcpp::ok()){
            std::cout<<"SEED (ros_stream): ROS shutdown received..."<<std::endl;
            remove(WM);
            ros_input = "";
        }

        pthread_mutex_unlock(&memMutex);
        
        if(ros_input == "")
            return true;
        
        importance=0;
        while(ros_input[ros_input.size()-1]=='!'){
            ros_input.erase(ros_input.size()-1);
            importance++;
        }
        return true;
    }
    void motorSchema(){
        std::vector<WM_node *> reqs;

        if(ros_input == "")
            return;
        
        //std::cout<< "SYSTEM: " << input << "\n";
        
        pthread_mutex_lock(&memMutex);
        if(dead())
            return;

        reqs=WM->getNodesByInstance( "requestStream" );
        if(reqs.size()>0){
            //WM_node *son;
            //son=reqs[0]->addSon(input);
            reqs[0]->addSon(ros_input);
            //son->ltMagnitude+=importance;
        }
        else
            std::cout<<"SYSTEM: no requestStream!\n";
        
        pthread_mutex_unlock(&memMutex);
        ros_input="";
        
    }
    void exit(){
        //std::cout<<getName()<<" EXITED\n";
        std::cout<<"SEED (ros_stream): exited"<<std::endl;
    }
    void start(){
        //std::cout<<"SYSTEM: "<<getName()<<" on!\n";
    }

//callbacks:

    void ros_input_callback(const std_msgs::msg::String::SharedPtr msg){
        ros_input = msg->data;
    }

    void ros_state_callback(const std_msgs::msg::String::SharedPtr msg){
        ros_state = msg->data;
        if(ros_state == ""){
            std::cout<<ansi::cyan<<SEED_NAME<<", skipped empty msg"<<ansi::end<<std::endl;
            return; //skip empty messages
        } 
        std::cout<<ansi::cyan<<SEED_NAME<<", received: "<<ros_state<<ansi::end<<std::endl;
        if(ros_state[0] == '-'){
            char c;
            std::stringstream ss(ros_state);
            ss >> c>>ros_state;
            wmv_set<bool>(ros_state,false);
        }
        else{
            wmv_set<bool>(ros_state,true);
        }
        std::cout<<ansi::cyan<<"\t "<<ros_state<<": "<<wmv_get<bool>(ros_state)<<ansi::end<<std::endl;
    }

    void ros_regulation_callback(const std_msgs::msg::String::SharedPtr msg){
        std::stringstream ss(msg->data);
        std::string source;
        double value;

        while(ss>>source>>value){
            //std::cout<<"source: "<<source<<", value: "<<value<<std::endl;
            //WM->updateContribution(source,value);
            WM->setContribution(source,value);
        }
    }

//service:
#if SEED_INTERFACES
    bool service_set_variable(std::shared_ptr<seed_interfaces::srv::SetWmv::Request>  req, std::shared_ptr<seed_interfaces::srv::SetWmv::Response> res){
        wmv_set<double>(req->variable,req->value);
        res->result = true;
        return true;
    }

    bool service_add_node(std::shared_ptr<seed_interfaces::srv::AddNode::Request>  req, std::shared_ptr<seed_interfaces::srv::AddNode::Response> res){
        std::vector<WM_node *> reqs;

        reqs=WM->getNodesByInstance( "requestStream" );
        if(reqs.size()>0){
            reqs[0]->addSon(req->instance);
            //son->ltMagnitude+=importance;
            res->result = true;
        }
        else{
            std::cout<<"SYSTEM: no requestStream!\n";
            res->result = false;
        }

        return res->result;
    }

    bool service_del_node(std::shared_ptr<seed_interfaces::srv::DelNode::Request>  req, std::shared_ptr<seed_interfaces::srv::DelNode::Response> res){
        std::vector<WM_node *> reqs;

        reqs=WM->getNodesByInstance( req->instance );
        if(reqs.size()>0){
            remove(reqs[0]);
            res->result = true;
        }
        else{
            res->result = false;
        }

        return res->result;
    }


    bool service_set_regulation_value(std::shared_ptr<seed_interfaces::srv::SetRegulationValue::Request>  req, std::shared_ptr<seed_interfaces::srv::SetRegulationValue::Response> res){
        WM->updateContribution(req->source,req->value);
        res->result = true;
        return true;
    }

    bool service_add_regulated_nodes(std::shared_ptr<seed_interfaces::srv::AddRegulatedNodes::Request>  req, std::shared_ptr<seed_interfaces::srv::AddRegulatedNodes::Response> res){
        for(auto i=0; i<req->nodes.size(); i++)
            setContribution(req->source,req->nodes[i],req->value);
        res->result = true;
        return true;
    }

    bool service_del_regulated_nodes(std::shared_ptr<seed_interfaces::srv::DelRegulatedNodes::Request>  req, std::shared_ptr<seed_interfaces::srv::DelRegulatedNodes::Response> res){
        for(auto i=0; i<req->nodes.size(); i++)
            delContribution(req->source,req->nodes[i]);
        res->result = true;
        return true;
    }
#endif

protected:
    int importance;

    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sb_local_stream;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sb_local_state;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sb_local_reg;

    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sb_global_stream;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sb_global_state;

#if SEED_INTERFACES
    rclcpp::Service<seed_interfaces::srv::SetWmv>::SharedPtr ss_wmv;
    rclcpp::Service<seed_interfaces::srv::AddNode>::SharedPtr ss_add;
    rclcpp::Service<seed_interfaces::srv::DelNode>::SharedPtr ss_del;

    rclcpp::Service<seed_interfaces::srv::SetRegulationValue>::SharedPtr ss_set_reg;
    rclcpp::Service<seed_interfaces::srv::AddRegulatedNodes>::SharedPtr ss_add_reg;
    rclcpp::Service<seed_interfaces::srv::DelRegulatedNodes>::SharedPtr ss_del_reg;
#endif

    std::string ros_input;
    std::string ros_state;
};



/*
 *   solve the competition for a WMV (suffer of contention):
 *       solve(varName,topic,period)
 *           varName: name of the contended WMV
 *           varType: type of the contended WMV (string, bool, int, double)
 *           topic: name of the toipic to publish with the retreived value
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
class RosSolveBehavior : public WMVBehavior
{
public:
    RosSolveBehavior(std::string instance){
        setInstance(instance);
        setRtm(QUIESCENCE);

        // varowner = instance2vector(instance)[1]; //this is not used anymore from SEED 4.0
        varowner = "none";
        varname = instance2vector(instance)[1];
        vartype = instance2vector(instance)[2];
        topic = instance2vector(instance)[3];

        //nh = rclcpp::Node::make_shared("solve");
        //ex.add_node(nh); // does it work? NO

        if (instance2vector(instance).size() == 5){
            rate = ston(instance2vector(instance)[4]);
        }
        else{
            rate = 0;
        }

        if (vartype == "double")
        {
            // pb = nh.advertise<std_msgs::Float32>(topic, 1);
            pbf = nh->create_publisher<std_msgs::msg::Float32>(topic, 1);
        }
        else if (vartype == "int")
        {
            // pb = nh.advertise<std_msgs::UInt32>(topic, 1);
            pbi = nh->create_publisher<std_msgs::msg::Float32>(topic, 1);
        }
        else if (vartype == "bool")
        {
            // pb = nh.advertise<std_msgs::Bool>(topic, 1);
            pbb = nh->create_publisher<std_msgs::msg::Bool>(topic, 1);
        }
        else
        {
            // pb = nh.advertise<std_msgs::String>(topic, 1);
            pbs = nh->create_publisher<std_msgs::msg::String>(topic, 1);
        }
    }
    std::string getName(){
        return "rosSolve";
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
            std_msgs::msg::Float32 msg;
            msg.data = wmv_solve<double>(varname);
            pbf->publish(msg);
        }
        else if (vartype == "int")
        {
            std_msgs::msg::Float32 msg;
            msg.data = wmv_solve<double>(varname);
            pbi->publish(msg);
        }
        else if (vartype == "bool")
        {
            std_msgs::msg::Bool msg;
            msg.data = wmv_solve<bool>(varname);
            pbb->publish(msg);
        }
        else
        {
            std_msgs::msg::String msg;
            msg.data = wmv_solve<std::string>(varname);
            pbs->publish(msg);

            // if(varname == "rover.command")
            //     std::cout<<ansi::magenta<<"rover.command solved at "<<seed_global_clk.toc()<<", winner is "<<msg.data<<ansi::end<<std::endl;
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

    //rclcpp::Node::SharedPtr nh;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pbf;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pbi;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pbb;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pbs;
};

/*
 *   state a variable to be true or false and publish it on a specific topic.
 *   The topic is always of type String and the value is "varName" if this node is active
 *   (ie. released and not accomplished) and "-varName" otherwise.
 *       state(varName,topic)
 *           varName: the anme of the variable to be stated
 *           topic: name of the toipic to publish with the retreived value
 *       NOTE: The value is published only ONCE everytime the node is activated/deactivated
 */
class RosStateBehavior : public WMVBehavior{
public:
    RosStateBehavior(std::string instance){
        setInstance(instance);
        setRtm(QUIESCENCE);

        varname = instance2vector(instance)[1];
        topic = instance2vector(instance)[2];

        //nh = rclcpp::Node::make_shared("state");
        //ex.add_node(nh);

        // pb = nh.advertise<std_msgs::String>(topic, 1, true); //this is latched!!
        pb = nh->create_publisher<std_msgs::msg::String>(topic, 1);

        releaser_status = false;
    }
    std::string getName(){
        return "rosState";
    }
    bool perceptualSchema()
    {
        pthread_mutex_lock(&memMutex);

        if (getReleaser() != releaser_status)
        {
            std_msgs::msg::String msg;

            releaser_status = getReleaser();

            if (releaser_status)
                msg.data = varname;
            else
                msg.data = "-" + varname;

            pb->publish(msg);
        }

        pthread_mutex_unlock(&memMutex);

        // ros::spinOnce();

        return true;
    }
    void motorSchema()
    {
        // do nothing
    }
    void start()
    {
        // NOTE: this empty message is sent because the first message is somethimes skipped by ROS
        std_msgs::msg::String msg;
        msg.data = "";
        pb->publish(msg);
        // ros::spinOnce();
        //ex.spin_once();
        usleep(100000);
    }
    void exit()
    {
    }

protected:
    std::string varowner;
    std::string topic;

    bool releaser_status;

    //rclcpp::Node::SharedPtr nh;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pb;
};



// this performs contention and solves
class RosActBehavior : public WMVBehavior{
public:
    RosActBehavior(std::string instance){
        setInstance(instance);
        setRtm(QUIESCENCE);

        varowner = "none";
        varvalue = instance2vector(instance)[1];
        varname = instance2vector(instance)[2];
        topic = instance2vector(instance)[3];

        if (instance2vector(instance).size() == 5){
            rate = ston(instance2vector(instance)[4]);
        }
        else{
            rate = 0;
        }

        pbs = nh->create_publisher<std_msgs::msg::String>(topic, 1);
    }
    std::string getName(){
        return "rosAct";
    }
    bool perceptualSchema()
    {

        pthread_mutex_lock(&memMutex);

        wmv_compete<std::string>(varowner, varname, varvalue);

        cond_loop();
        // ros::spinOnce();
        pthread_mutex_unlock(&memMutex);

        return true;
    }
    void motorSchema()
    {
        pthread_mutex_lock(&memMutex);

        if(wmv_solve_once<std::string>(varname) == varvalue){
            std_msgs::msg::String msg;
            msg.data = varvalue;
            pbs->publish(msg);
        }
        //NOTE: if no one wins (e.g., ex-aequo) nothing is published!

        pthread_mutex_unlock(&memMutex);
    }
    void start()
    {
    }
    void exit()
    {
        pthread_mutex_lock(&memMutex);
        wmv_withdraw<std::string>(varname);
        wmv_set<bool>(this->getInstance() + ".done", false);
        pthread_mutex_unlock(&memMutex);
    }

protected:
    std::string varowner;
    std::string topic;

    //rclcpp::Node::SharedPtr nh;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pbs;
};



class TfObserverBehavior : public WMVBehavior{
public:
    TfObserverBehavior(std::string instance){
        setInstance(instance);
        setRtm(QUIESCENCE);

        //from observer/SEED_NAME.rules
        std::ifstream infile(SEED_HOME_PATH + "/observer/" + SEED_NAME + ".rules");
        if (infile.is_open()) {
            std::string line;
            while (std::getline(infile, line))
                str_rules.push_back(line);
            infile.close();
        }
        else
            std::cout<<ansi::cyan<<"parsing file does not exists! "<<std::endl<<
                SEED_HOME_PATH<<"/observer/"<<SEED_NAME<<".rules"<<ansi::end<<std::endl;

        //initialize buffer
        tf_buffer = std::make_unique<tf2_ros::Buffer>(nh->get_clock());
        //initialize listener
        tf_listener = std::make_shared<tf2_ros::TransformListener>(*tf_buffer);
        

        //parse rules
        for(size_t i=0; i<str_rules.size(); i++){
            if(str_rules[i][0] != '#')
                rules.push_back(ParsedRule(str_rules[i]));
        }
            
    }
    std::string getName(){
        return "tfobserver";
    }
    bool perceptualSchema()
    {
        
        //get values from ROS2 TF2
        results.clear();
        for(size_t i=0; i<rules.size(); i++){
            geometry_msgs::msg::TransformStamped t;
            KnownValue kv;
            if(get_tf2(rules[i].parameter1,rules[i].parameter2,t)){
                kv.known = true;

                if(rules[i].function == "xdiff")
                    kv.v = xdiff(t);
                else if(rules[i].function == "ydiff")
                    kv.v = ydiff(t);
                else if(rules[i].function == "zdiff")
                    kv.v = zdiff(t);
                else if(rules[i].function == "dist")
                    kv.v = eudist(t);
                else if(rules[i].function == "xydist")
                    kv.v = xydist(t);
                else if(rules[i].function == "adist"){
                    kv.v = adist(t);
                    //std::cout<<"adist "<<rules[i].parameter1<<" to "<<rules[i].parameter2<<": "<<kv.v<<std::endl;
                }
                else if(rules[i].function == "exists")
                    kv.v = 1.0;
                else
                    kv.known = false;
            }
            else if(rules[i].function == "exists"){
                kv.known = true;
                kv.v = 0.0;
            }
            else
                kv.known = false;

            results.push_back(kv);
        }
        //
        

        pthread_mutex_lock(&memMutex);

        for(size_t i=0; i<rules.size(); i++){
            if(!results[i].known){
                //std::cout<<ansi::cyan<<"rule "<<i<<" unknown:"<<ansi::end<<std::endl;
                //std::cout<<ansi::cyan<<"\t "<<str_rules[i]<<ansi::end<<std::endl;
                continue;
            }

            if(rules[i].type == "->" && rules[i].op == ">"){
                if(rules[i].value > results[i].v)
                    wmv_set<bool>(rules[i].target,true);
                else
                    wmv_set<bool>(rules[i].target,false);
            }
            else if(rules[i].type == "->" && rules[i].op == "<"){
                if(rules[i].value < results[i].v)
                    wmv_set<bool>(rules[i].target,true);
                else
                    wmv_set<bool>(rules[i].target,false);
            }
            else if(rules[i].type == "->" && rules[i].function == "exists"){
                if(rules[i].value > 0.5)
                    wmv_set<bool>(rules[i].target,true);
                else
                    wmv_set<bool>(rules[i].target,false);
            }
            else if(rules[i].type == "~>" && rules[i].op == "*"){
                //WM->updateContribution(rules[i].target, rules[i].value * results[i].v);
                WM->setContribution(rules[i].target, rules[i].value * results[i].v);
            }
            else if(rules[i].type == "~>" && rules[i].op == "/"){
                //WM->updateContribution(rules[i].target, rules[i].value / results[i].v);
                WM->setContribution(rules[i].target, rules[i].value / results[i].v);
            }
            else{
                std::cout<<ansi::cyan<<"unable to execute rule "<<i<<":"<<ansi::end<<std::endl;
                std::cout<<ansi::cyan<<"\t "<<str_rules[i]<<":"<<ansi::end<<std::endl;
            }
        }

        pthread_mutex_unlock(&memMutex);

        return true;
    }
    void motorSchema()
    {
        //empty
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

    class ParsedRule{
        public:
        ParsedRule(std::string str){
            parse(str);
        }

        //simple parser
        void parse(std::string str){
            std::istringstream ss1(str);
            std::istringstream ss2(str);
            //std::cout<<ansi::cyan<<"parsing: "<<str<<ansi::end<<std::endl;
            good = false;
            if(ss1 >> value >> op >> function >> type >> target){
                std::vector<std::string> fv = instance2vector(function);
                if(fv.size() == 3){
                    function = fv[0];
                    parameter1 = fv[1];
                    parameter2 = fv[2];
                    good = true;
                    //std::cout<<ansi::cyan<<"\t STRING IS GOOD"<<ansi::end<<std::endl;
                    //std::cout<<ansi::cyan<<"\t function: "<<function<<ansi::end<<std::endl;
                    //std::cout<<ansi::cyan<<"\t parameter1: "<<parameter1<<ansi::end<<std::endl;
                    //std::cout<<ansi::cyan<<"\t parameter2: "<<parameter2<<ansi::end<<std::endl;
                    //std::cout<<ansi::cyan<<"\t op: "<<op<<ansi::end<<std::endl;
                    //std::cout<<ansi::cyan<<"\t value: "<<value<<ansi::end<<std::endl;
                    //std::cout<<ansi::cyan<<"\t type: "<<type<<ansi::end<<std::endl;
                    //std::cout<<ansi::cyan<<"\t target: "<<target<<ansi::end<<std::endl;
                }
                else
                    std::cout<<ansi::cyan<<"TF-OBSERVER ERROR: BAD STRING err2"<<ansi::end<<std::endl;
            }
            else if(ss2 >> function >> type >> target){
                std::vector<std::string> fv = instance2vector(function);
                if(fv.size() == 3){
                    function = fv[0];
                    parameter1 = fv[1];
                    parameter2 = fv[2];
                    good = true;
                    //std::cout<<ansi::cyan<<"\t STRING IS GOOD"<<ansi::end<<std::endl;
                    //std::cout<<ansi::cyan<<"\t function: "<<function<<ansi::end<<std::endl;
                    //std::cout<<ansi::cyan<<"\t parameter1: "<<parameter1<<ansi::end<<std::endl;
                    //std::cout<<ansi::cyan<<"\t parameter2: "<<parameter2<<ansi::end<<std::endl;
                    //std::cout<<ansi::cyan<<"\t type: "<<type<<ansi::end<<std::endl;
                    //std::cout<<ansi::cyan<<"\t target: "<<target<<ansi::end<<std::endl;
                }
                else
                    std::cout<<ansi::cyan<<"TF-OBSERVER ERROR: BAD STRING err2"<<ansi::end<<std::endl;
            }
            else
                std::cout<<ansi::cyan<<"TF-OBSERVER ERROR: BAD STRING err1"<<ansi::end<<std::endl;
            
        }

        bool good;
        std::string function;
        std::string parameter1;
        std::string parameter2;
        std::string op;
        double value;
        std::string type;
        std::string target;
    };

    struct KnownValue{
        bool known;
        double v;
    };

    bool get_tf2(std::string start_frame, std::string target_frame, geometry_msgs::msg::TransformStamped &t){
        try {
            //t = tf_buffer->lookupTransform(target_frame, start_frame, tf2::TimePointZero);
            t = tf_buffer->lookupTransform(start_frame, target_frame, tf2::TimePointZero);
            
            //t = tf_buffer->lookupTransform(target_frame, start_frame, nh->get_clock()->now(),rclcpp::Duration(1000000));
            return true;
        } catch (const tf2::TransformException & ex) {
            return false;
        }
    }

    // difference along the Z axis
    inline double zdiff(geometry_msgs::msg::TransformStamped t){
        return t.transform.translation.z;
    }

    // difference along the X axis
    inline double xdiff(geometry_msgs::msg::TransformStamped t){
        return t.transform.translation.x;
    }

    // difference along the Y axis
    inline double ydiff(geometry_msgs::msg::TransformStamped t){
        return t.transform.translation.y;
    }

    // Euclidean distance
    inline double eudist(geometry_msgs::msg::TransformStamped t){
        return sqrt(pow(t.transform.translation.x,2)+pow(t.transform.translation.y,2)+pow(t.transform.translation.z,2));
    }

    // planar distance along X and Y axis
    inline double xydist(geometry_msgs::msg::TransformStamped t){
        return sqrt(pow(t.transform.translation.x,2)+pow(t.transform.translation.y,2));
    }

    // Angular distance
    inline double adist(geometry_msgs::msg::TransformStamped t){
        double vx,vy,vz;
        
        return sqrt(pow(t.transform.translation.y,2.0)+pow(t.transform.translation.z,2.0));
        
        double norm = sqrt(pow(t.transform.translation.x,2)+pow(t.transform.translation.y,2)+pow(t.transform.translation.z,2));
        vx = t.transform.translation.x / norm;
        vy = t.transform.translation.y / norm;
        vz = t.transform.translation.z / norm;

        double tetha = atan2(vy,vx);
        double phi = atan2(vx,vz);
        
        return (tetha + phi)/2;
    }

    std::string ego_frame;

    std::vector<std::string> str_rules;
    std::vector<ParsedRule> rules;
    std::vector<KnownValue> results;

    std::shared_ptr<tf2_ros::TransformListener> tf_listener{nullptr};
    std::unique_ptr<tf2_ros::Buffer> tf_buffer;

    //rclcpp::Node::SharedPtr nh;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pbs;
};

#endif	/* ROS_BehaviorS_H */
