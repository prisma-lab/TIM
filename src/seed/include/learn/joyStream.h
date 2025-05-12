#ifndef JOYSTREAM_H
#define JOYSTREAM_H

#include "../seed_header.h"

////left components
//double joy_Lx,joy_Ly;
////right components
//double joy_Rx,joy_Ry;
////buttons
//int joy_B2;
//int joy_Al;



std::string selected;

//void joy_input_callback(const sensor_msgs::Joy::ConstPtr& msg){
//    joy_Lx = -msg->axes[0];
//    joy_Ly = msg->axes[1];
//
//    //esper
//    joy_Rx = msg->axes[2];
//    joy_Ry = msg->axes[3];
//    
//    //esper buttons: 1 - 2 - 3 - 4 - L1 - R1 - L2 - R2 - select - start - analogL - analogR 
//    joy_B2 = msg->buttons[1];
//    
//    //esper arrows (+1/-1): l/r - u/d
//    joy_Al = -msg->axes[4];
//    
//    
//    //std::cout<<" arr: "<<msg->axes[4]<<" "<<msg->axes[5]<<" "<<msg->axes[6]<<" "<<msg->axes[7]<<std::endl;
//    
//    //X-BOX
//    //joy_Rx = -msg->axes[3];
//    //joy_Ry = msg->axes[4];
//
//    //std::cout<<"ROS::JOY, x: "<<joy_Lx<<", y: "<<joy_Ly<<"\n";
//
//}


std::unordered_map<std::string, double> jmap;

void joy_cb(const sensor_msgs::msg::Joy::ConstPtr& msg){

  //esper axes-vector: LeftX - LeftY - RightX - RightY - arrowLR (+1 left, -1 right) - arrowUD (+1 up, -1 down)
  jmap["Lx"] = -msg->axes[0];
  jmap["Ly"] = msg->axes[1];
  jmap["Rx"] = msg->axes[2];
  jmap["Ry"] = msg->axes[3];
  jmap["Alr"] = msg->axes[4];
  jmap["Aud"] = msg->axes[5];

  //esper buttons-vector: 1 - 2 - 3 - 4 - L1 - R1 - L2 - R2 - select - start - analogL - analogR
  jmap["B1"] = msg->buttons[0]; //triangle
  jmap["B2"] = msg->buttons[1]; //circle
  jmap["B3"] = msg->buttons[2]; //cross
  jmap["B4"] = msg->buttons[3]; //square

  jmap["L1"] = msg->buttons[4];
  jmap["R1"] = msg->buttons[5];
  jmap["L2"] = msg->buttons[6];
  jmap["R2"] = msg->buttons[7];

  jmap["select"] = msg->buttons[8];
  jmap["start"] = msg->buttons[9];
  jmap["analogL"] = msg->buttons[10];
  jmap["analogR"] = msg->buttons[11];

  //X-BOX
  //joy_Rx = -msg->axes[3];
  //joy_Ry = msg->axes[4];

  //std::cout<<"ROS::JOY, x: "<<joy_Lx<<", y: "<<joy_Ly<<"\n";
}

void vrep_selected_callback(const std_msgs::String::ConstPtr& msg){
  selected = msg->data;
}

class JoyStreamBehavior : public IOBehavior{
public:
    JoyStreamBehavior(std::string instance){
        setName(instance2vector(instance)[0]);
        setInstance(instance);
        setRtm(QUIESCENCE);

        //sb=nh.subscribe("joy", 0, joy_input_callback);
        sb=nh.subscribe("joy", 0, joy_cb);
        sb2=nh.subscribe("/vrep/selected_target", 0, vrep_selected_callback);
        
        pb = nh.advertise<std_msgs::Float32>("/vrep/selection_action", 1);

        wmv_set<bool>("joyStream",true);
        wmv_set<bool>("joyStream.command",false);

        wmv_set<std::time_t>("simulation.startTime",std::time(0));

        old_jB2 = 0;
        old_jAl = 0;
        
        selected = "";
        
        command_sent = false;
    }
    bool perceptualSchema(){
        //std::cin>>input;
        ros::spinOnce();

        jLx = jmap["Lx"];  //joy_Lx;
        jLy = jmap["Ly"];  //joy_Ly;
        jRx = jmap["Rx"];  //joy_Rx;
        jRy = jmap["Ry"];  //joy_Ry;
        jB2 = jmap["B2"];  //joy_B2;
        jAl = jmap["Alr"]; //joy_Al;
        
        jmap["Alr"] = 0; //joy_Al = 0;

        return true;
    }
    void motorSchema(){

        double max_fs = 0.4; //m/s
        double max_ls = 0.2; //m/s
        double max_ts = 30; //degr
        bool command = false;
        
        bool arm_command = false;

        //std::cout<< "SYSTEM: " << input << "\n";

        pthread_mutex_lock(&memMutex);
        
        if(dead())
            return;

        double ryaw = wmv_get<double>("engine.yaw");

        double jfs,jls,jts,jyaw=0;

        //change selected object
        std_msgs::Float32 selMsg;
        if(jAl != 0) {
            selMsg.data = jAl;
            //old_jAl = jAl;
            //std::cout<<" select: "<<jAl<<std::endl;
            jAl = 0;
            pb.publish(selMsg);
            
        }
        //--
        
        if(jB2 != old_jB2){
            old_jB2 = jB2;
            arm_command = true;
        }
        

        if(jRy>-0.2 && jRy<0.2){
            jfs=0;
        }
        else{
            jfs = max_fs*jRy; //*sqrt(jRx*jRx + jRy*jRy);
            command = true;
        }
        
        
        if(jRx>-0.2 && jRx<0.2){
            jls=0;
        }
        else{
            jls = max_ls*jRx; //*sqrt(jRx*jRx + jRy*jRy);
            command = true;
        }


        if (jfs > max_fs)
                jfs = max_fs;
            else if (jfs< -max_fs/2 )
                jfs = -max_fs/2;

        if( (jLy>0.05 || jLy<-0.05) || (jLx>0.05 || jLx<-0.05) ){
        //if(jLy!=0 || jLx!=0){
//            jyaw = rtod(atan2(jLy, jLx));
            jts = rtod(-jLx);
            command = true;
        }
        
//        jts = -ryaw + jyaw;
//
//        //controlla se sto girando nel verso corretto
//        if (jts > 180) jts = -1 * (360 - jts);
//        else if (jts<-180) jts = 360 + jts;

        //std::cout<<"JOY: "<<jyaw<<", "<<command<<"\n";

        //gira per un massimo di max_ts gradi
        if(jts > max_ts)
            jts=max_ts;
        else if(jts<-max_ts)
            jts=-max_ts;

        if(jts>-3 && jts<3)
            jts=0;

        this->setRtm(0.01);

//        if(command==true){
//            std::cout<<"JOY, jyaw: "<<jyaw<<", ryaw: "<<ryaw<<" -> ts: "<<jts<<"\n";
//            std::cout<<"\t\t jLx: "<< jLx <<" -> jLy:"<<jLy<<"\n";
//            std::cout<<"\t\t jRy: "<< jRy <<" -> fs:"<<jfs<<"\n";
//        }

        //INIZIATIVA "MISTA"
        if (WM->getNodesByInstance("avoidLearn").size() != 0) {
            bgNode = WM->getNodesByInstance("avoidLearn")[0];
        //if (WM->getNodesByInstance("take2").size() != 0) {
        //    bgNode = WM->getNodesByInstance("take2")[0];
        //if (WM->getNodesByInstance("requestStream").size() != 0) {
        //    bgNode = WM->getNodesByInstance("requestStream")[0];

            if (command) {
                wmv_set<bool>("joyStream.command",true); //per eventuali stampe

                bgNode->setBackground(true);
                
                command_sent = true;
                
//                wmv_set<double>("engine.fs.count", 0);
//                wmv_set<double>("engine.ts.count", 0);
//                wmv_set<double>("engine.fs", jfs);
//                wmv_set<double>("engine.ts", jts);

                wmv_compete<double>("", "engine.fs", jfs);
                wmv_compete<double>("", "engine.ls", jls);
                wmv_compete<double>("", "engine.ts", jts);
                
            } else {
                
                if(command_sent){
                    wmv_compete<double>("", "engine.fs", 0);
                    wmv_compete<double>("", "engine.ls", 0);
                    wmv_compete<double>("", "engine.ts", 0);
                }
                
                wmv_set<bool>("joyStream.command",false); //per eventuali stampe

                bgNode->setBackground(false);
                
                command_sent = false;
            }
        }
        else if (command){
//            wmv_set<double>("engine.fs.count", 0);
//            wmv_set<double>("engine.ts.count", 0);
//            wmv_set<double>("engine.fs", jfs);
//            wmv_set<double>("engine.ts", jts);
            
            wmv_compete<double>("", "engine.fs", jfs);
            wmv_compete<double>("", "engine.ls", jls);
            wmv_compete<double>("", "engine.ts", jts);
            
            command_sent = true;
            
            //std::cout<<"JOY, fs: "<<jfs<<", ls: "<<jls<<", ts: "<<jts<<std::endl;
        } else {
                
            if(command_sent){
                wmv_compete<double>("", "engine.fs", 0);
                wmv_compete<double>("", "engine.ls", 0);
                wmv_compete<double>("", "engine.ts", 0);
            }

            command_sent = false;
        }
        
        if(arm_command){
            if( jB2 == 1 ){
                wmv_compete<std::string>("", "arm.target", "move " + selected);
            }
            else{
                
                wmv_compete<std::string>("", "arm.target", "");
            }
        }

        pthread_mutex_unlock(&memMutex);


    }
protected:
    int importance;
    ros::NodeHandle nh;
    ros::Subscriber sb, sb2;
    ros::Publisher pb;
    double jLx,jLy;
    double jRx,jRy;
    WM_node * bgNode;
    
    int jAl, old_jAl;
    
    int jB2, old_jB2;
    
    bool command_sent;
};

#endif // JOYSTREAM_H
