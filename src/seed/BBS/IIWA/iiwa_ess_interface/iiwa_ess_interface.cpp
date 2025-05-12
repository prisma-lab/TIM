#include "iiwa_ess_interface.h"


ServerSocket::ServerSocket(){
    sd = -1;
    port = -1;
}


int ServerSocket::write(std::string str){

    const char *msg = &str[0];
    int n = -1;
    
    //std::cout<<"Sending: " << msg <<std::endl;
    //std::cout<<"    len: " << strlen(msg) <<std::endl;

    n = sendto(sd, (const char *)msg, strlen(msg),  
        MSG_CONFIRM, (const struct sockaddr *) &cliaddr, 
            cliaddr_len);   //(socklen_t*) &len); //why not?

    //std::cout<<"done" <<std::endl;
    return n;
}

std::string ServerSocket::read(int buff_size){

    //std::cout<<"Receiving" <<std::endl;

    char buffer[buff_size]; //buffer max size

    int n = recvfrom(sd, (char *)buffer, buff_size,  
                MSG_WAITALL, ( struct sockaddr *) &cliaddr, 
                (socklen_t*) &cliaddr_len); 
    buffer[n] = '\0';

    std::string str = buffer;

    //std::cout<<"done: "<<str <<std::endl;

    return str;
}

bool ServerSocket::connect(int in_port){

    port = in_port;

    // Creating socket file descriptor
    sd = socket(AF_INET, SOCK_DGRAM, 0);
    if ( sd < 0 ) { 
        std::cout<<"socket on port "<<port<<": creation failed"<<std::endl; 
        return false; 
    }

    int reuse = 1;
    if (setsockopt(sd, SOL_SOCKET, SO_REUSEADDR, (const char*)&reuse, sizeof(reuse)) < 0){
        std::cout<<"socket on port "<<port<<": SO_REUSEADDR failed"<<std::endl;
    }
      
    memset(&servaddr, 0, sizeof(servaddr)); 
    memset(&cliaddr, 0, sizeof(cliaddr)); 
      
    // Filling server information 
    servaddr.sin_family    = AF_INET; // IPv4 
    servaddr.sin_addr.s_addr = INADDR_ANY; 
    servaddr.sin_port = htons(port); 
      
    // Bind the socket with the server address 
    if ( bind(sd, (const struct sockaddr *)&servaddr, sizeof(servaddr)) < 0 ) 
    { 
        std::cout<<"socket on port "<<port<<": bind failed"<<std::endl; 
        return false;
    }

    cliaddr_len = sizeof(cliaddr);



    std::cout<<"socket on port "<<port<<": open"<<std::endl; 

    return true;
}


bool ServerSocket::disconnect(){

    std::cout<<"socket on port "<<port<<": closed"<<std::endl; 

    return close(sd);
}





IIWA_ess_interface::IIWA_ess_interface(){
    running = true;
    connected = false;

    //enable ctrl+c handler
    //sigIntHandler.sa_handler = &(IIWA_ess_interface::stop_handler);
    //sigemptyset(&sigIntHandler.sa_mask);
    //sigIntHandler.sa_flags = 0;
    //sigaction(SIGINT, &sigIntHandler, NULL);
/*
    out_jp = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}; //joint pos
    out_jt = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}; //joint tor
    out_js = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}; //joint stiffness
    out_jd = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}; //joint dumping
    out_cp = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};      //cart pos
    out_cs = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};      //cart stiffness
    out_cd = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};      //cart dumping
*/
    in_jp = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}; //joint pos
    in_jt = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}; //joint tor
    in_js = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}; //joint stiffness
    in_jd = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}; //joint dumping
    in_cp = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};      //cart pos
    //in_ct = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};      //cart tor
    in_cw = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};      //cart wrench
    in_cs = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};      //cart stiffness
    in_cd = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};      //cart dumping

}



bool IIWA_ess_interface::open_connection(int comm_port, int stat_port){

    if(connected){
        //std::cout<<ansi::yellow<<"WARNING: already connected to the robot"<<ansi::end<<std::endl;
        std::cout<<"WARNING: already connected to the robot"<<std::endl;
        return true;
    }

    if( !socket_comm.connect(comm_port) ){
        std::cout<<"ERROR: unable to open command socket"<<std::endl;
        return false;
    }

    if( !socket_stat.connect(stat_port) ){
        std::cout<<"ERROR: unable to open state socket"<<std::endl;
        return false;
    }

    //t = std::thread(&IIWA_ess_interface::th_communication, this );
    t_comm = std::thread(&IIWA_ess_interface::th_command, this);
    t_stat = std::thread(&IIWA_ess_interface::th_state, this);


    //wait connection
    //lock();
    while(!connected){
        //unlock();
        std::cout<<"Waiting for IIWA to connect..."<<std::endl;
        sleep(1);
        //lock();
    }
    //unlock();

    std::cout<<"connected"<<std::endl;

    return true;
}


bool IIWA_ess_interface::close_connection(){

    std::cout<<"closing connnection"<<std::endl;

    lock();
    if(!connected){
        unlock();
        //std::cout<<ansi::yellow<<"WARNING: connection already over"<<ansi::end<<std::endl;
        std::cout<<"WARNING: connection already over"<<std::endl;
        return true;
    }

    socket_comm.write("close");
    socket_stat.write("close");

    std::cout<<"close msgs sent"<<std::endl;

    socket_comm.disconnect();
    socket_stat.disconnect();

    running = false; //bool no need lock!
    unlock();

    if(t_comm.joinable())
        t_comm.join();

    if(t_stat.joinable())
        t_stat.join();

    std::cout<<"joined communication threads"<<std::endl;

    return true;
}

void IIWA_ess_interface::th_command(){
    std::cout<<"IIWA command thread started"<<std::endl;

    std::vector< std::vector<double> > to_send;
    std::string switch_control_command = "";
    bool do_nothing = false;

    std::string res_ack;

/*
    std::string ack_state = iiwa_receive(); //get first state as ack

    lock();
    update_state( unpack_string(ack_state) );

    //set initial position as default commands (this should be done when control switch!)
    out_jp = in_jp;
    out_cp = in_cp;

    connected = true;
    unlock();

    std::cout<<"ack state received"<<std::endl;
*/

    while( isRunning() ){

        switch_control_command = "";
        to_send.clear();

        if(!do_nothing){
            res_ack = socket_comm.read();
            //std::cout<<" received: "<<res_ack<<std::endl;
        }

        do_nothing = false;


        lock();
        if(iiwa_control_mode != control_mode){
            switch(control_mode){
                case(IIWA_control_mode::JOINT_POSITION):
                switch_control_command = "JPC";
                break;
                case(IIWA_control_mode::JOINT_IMPEDANCE):
                switch_control_command = "JIC";
                //send pos, stiff and dump also
                to_send.push_back( out_jp );
                to_send.push_back( out_js );
                to_send.push_back( out_jd );
                out_jp.clear();
                out_js.clear();
                out_jd.clear();
                break;
                case(IIWA_control_mode::CARTESIAN_POSITION):
                switch_control_command = "CPC";
                break;
                case(IIWA_control_mode::CARTESIAN_IMPEDANCE):
                switch_control_command = "CIC";
                to_send.push_back( out_cp );
                to_send.push_back( out_cs );
                to_send.push_back( out_cd );
                out_cp.clear();
                out_cs.clear();
                out_cd.clear();
                break;
            }
        }
        else {
            switch(control_mode){
                case(IIWA_control_mode::JOINT_POSITION):
                to_send.push_back( out_jp );
                out_jp.clear();
                break;
                case(IIWA_control_mode::JOINT_IMPEDANCE):
                to_send.push_back( out_jp );
                to_send.push_back( out_js );
                to_send.push_back( out_jd );
                out_jp.clear();
                out_js.clear();
                out_jd.clear();   
                break;
                case(IIWA_control_mode::CARTESIAN_POSITION):
                to_send.push_back( out_cp );
                out_cp.clear();
                break;
                case(IIWA_control_mode::CARTESIAN_IMPEDANCE):
                to_send.push_back( out_cp );
                to_send.push_back( out_cs );
                to_send.push_back( out_cd );
                out_cp.clear();
                out_cs.clear();
                out_cd.clear();
                break;
                default:
                do_nothing = true;
                break;
            }
        }
        unlock();

        //send "switch" command to change control mode
        if(switch_control_command!=""){

            std::cout<<"switching control mode"<<std::endl;

            //res_ack = socket_comm.read();
            std::cout<<" writing on socket "<<socket_comm.sd<<std::endl;

            //iiwa_send(switch_control_command);
            int n = socket_comm.write(switch_control_command + " " + pack_string(to_send) );
            std::cout<<"   "<<switch_control_command << " " << pack_string(to_send)<<std::endl;
            std::cout<<n<<" byte"<<std::endl;

            //res_ack = socket_comm.read();

            if(res_ack=="err"){
                //std::cout<<ansi::red<<"ERROR: IIWA failed to switch control to "<<switch_control_command<<ansi::end<<std::endl;
                std::cout<<"ERROR: IIWA failed to switch control to "<<switch_control_command<<std::endl;
                break;
            }
            else {
                //std::cout<<ansi::red<<"new control mode "<<switch_control_command<<" set"<<ansi::end<<std::endl;
                std::cout<<"new control mode "<<switch_control_command<<" set"<<std::endl;
            }

            lock();
            iiwa_control_mode = control_mode;
            unlock();

            //std::cout<<ansi::red<<"new control mode start"<<ansi::end<<std::endl;
            std::cout<<"new control mode start"<<std::endl;
        }
        //nothing to send, wait for user command
        else if(do_nothing){
            //std::cout<<"doing nothing..."<<std::endl;
            sleep(0.01);
            continue;
        }
        //send "go" command to move the robot
        else {

            //std::cout<<"sending: "<<"go "<< pack_string(to_send)<<std::endl;

            socket_comm.write( "go " + pack_string(to_send) ); //send command

            //std::string ack_state = socket_comm.read(); //get ack
        }
    }

    socket_comm.write("close");

    std::cout<<"IIWA command thread exited"<<std::endl;
}

void IIWA_ess_interface::th_state(){
    std::cout<<"IIWA state thread started"<<std::endl;

    std::string state = socket_stat.read(); //get first state as ack
    //socket_stat.write("full"); //get the full state on start

    std::cout<<"FULL STATE RECEIVED: "<<std::endl;
    std::cout<<"  "<<state<<std::endl;

    lock();
    update_state( unpack_string(state) );

    //set initial position as default commands (this should be done when control switch!)
    out_jp = in_jp;
    out_cp = in_cp;

    connected = true;
    unlock();

    std::string request = "";

    while( isRunning() ){

        //CONDWAIT for requests
        lock();
        while(!have_request){
            unlock();
            usleep(100);

            if(!isRunning())
                break;

            lock();
        }

        //find the requested info

        if( in_jp.empty() ) request = "jp";
        else if( in_jt.empty() ) request = "jt";
        else if( in_cp.empty() ) request = "cp";
        else if( in_cw.empty() ) request = "cw";
        else request = "full";

        unlock();

        if(!isRunning())
            break;

        //std::cout<<ansi::cyan<<"new request: "<<request<<ansi::end<<std::endl;

        socket_stat.write(request);

        state = socket_stat.read();
        //socket_stat.write("ack");
        //std::cout<<ansi::cyan<<"req-"<<request<<" ["<<state<<"]"<<ansi::end<<std::endl;

        lock();

        if( in_jp.empty() ) in_jp = unpack_string(state);
        else if( in_jt.empty() ) in_jt = unpack_string(state);
        else if( in_cp.empty() ) in_cp = unpack_string(state);
        else if( in_cw.empty() ) in_cw = unpack_string(state);
        else update_state( unpack_string(state) );

        have_request = false; //done

        //std::cout<<ansi::cyan<<"request completed"<<ansi::end<<std::endl;
        
        unlock();
    }

    //state = socket_stat.read();
    socket_stat.write("close");

    std::cout<<"IIWA state thread exited"<<std::endl;
}


std::string IIWA_ess_interface::pack_string(std::vector< std::vector<double> > in_vec){
    std::stringstream ss("");
    //ss<<"[ ";
    for(auto i=0; i<in_vec.size(); i++){
        if(!in_vec[i].empty())
            for(auto j=0; j<in_vec[i].size(); j++)
                ss<<in_vec[i][j]<<" ";
        else
            ss<<"null "; 
            
    }
        
    //ss<<"]";
    return ss.str();
};

std::vector<double> IIWA_ess_interface::unpack_string(std::string in_str){
    std::vector<double> out_vec;
    std::istringstream iss(in_str);
    std::string s;
    while ( getline( iss, s, ' ' ) ) {
        //out_vec.push_back( ston(s) );
        //std::cout<<"stod: "<<s<<std::endl;
        double d; // = std::stod(s);
        std::stringstream ss(s); //NEVER USE AGAIN STOD
        ss>>d;
        //std::cout<<"stod: "<<d<<std::endl;
        out_vec.push_back( d );
    }

    return out_vec;
};

//STATE IS NO MORE UPDATED IN A SINGLE READING, this works only the first time!
void IIWA_ess_interface::update_state(std::vector<double> state_vec){
    //from IIWA: jp jt cp cw
    //           #7 #7 #6 #6
    std::copy( state_vec.begin(),    state_vec.begin()+7,  in_jp.begin() );
    std::copy( state_vec.begin()+7,  state_vec.begin()+14, in_jt.begin() );
    std::copy( state_vec.begin()+14, state_vec.begin()+20, in_cp.begin() );
    std::copy( state_vec.begin()+20, state_vec.begin()+26, in_cw.begin() );

    //plot result
    //std::cout<<std::endl;
    //std::cout<<ansi::cyan<<"jp:"<<plot(in_jp)<<ansi::end<<std::endl;
    //std::cout<<ansi::cyan<<"jt:"<<plot(in_jt)<<ansi::end<<std::endl;
    //std::cout<<ansi::cyan<<"cp:"<<plot(in_cp)<<ansi::end<<std::endl;
};




//private methods





/*
// ------------------------------- OLD -------------------------------


#define MAXLINE 1024 

bool running = true;
int sockfd; //socket
  
// Driver code 
int main() { 
    
    char buffer[MAXLINE]; 
    const char *msg; //char msg[MAXLINE];// = "go"; 
    struct sockaddr_in servaddr, cliaddr;

    //enable ctrl+c handler
    struct sigaction sigIntHandler;

    sigIntHandler.sa_handler = stop_handler;
    sigemptyset(&sigIntHandler.sa_mask);
    sigIntHandler.sa_flags = 0;

    sigaction(SIGINT, &sigIntHandler, NULL);

      
    // Creating socket file descriptor 
    if ( (sockfd = socket(AF_INET, SOCK_DGRAM, 0)) < 0 ) { 
        std::cout<<"socket creation failed"<<std::endl; 
        exit(EXIT_FAILURE); 
    } 
      
    memset(&servaddr, 0, sizeof(servaddr)); 
    memset(&cliaddr, 0, sizeof(cliaddr)); 
      
    // Filling server information 
    servaddr.sin_family    = AF_INET; // IPv4 
    servaddr.sin_addr.s_addr = INADDR_ANY; 
    servaddr.sin_port = htons(PORT); 
      
    // Bind the socket with the server address 
    if ( bind(sockfd, (const struct sockaddr *)&servaddr,  
            sizeof(servaddr)) < 0 ) 
    { 
        std::cout<<"bind failed"<<std::endl; 
        exit(EXIT_FAILURE); 
    } 
      
    int len, n; 
  
    len = sizeof(cliaddr);  //len is value/resuslt 

    seed::time::Clock clk, overall_clk, comm_clk;

    clk.tic();
    overall_clk.tic();

    double j_target_dis = 0.25; //rad
    double j_target_time = 5.0; //rad
    bool returning = true;

    double j_pos = 0; //rad
    double elapsed_time;
    double receive_elapsed_time;
    double exec_time = 0;

    std::string s_msg;

    do {
  
    comm_clk.tic();
    //max is 78
    n = recvfrom(sockfd, (char *)buffer, 78,  
                MSG_WAITALL, ( struct sockaddr *) &cliaddr, 
                (socklen_t*) &len); 
    buffer[n] = '\0'; 
    receive_elapsed_time = comm_clk.toc();
    //std::cout<<"Client: " << buffer <<std::endl; 
    //std::cout<<"received in "<<comm_clk.toc()<<std::endl;



    if(exec_time > j_target_time){
        exec_time = 0;

        returning = !returning;

        j_pos = 0;
    }

    if(!returning)
        j_pos = j_target_dis * (exec_time/j_target_time);
    else
        j_pos = j_target_dis * ( 1.0 - (exec_time/j_target_time) );

    j_pos = seed::roundec(j_pos,4);

    std::stringstream ss;

    if(j_pos > 0)
        ss<<"+"<<j_pos<<" +"<<j_pos<<" +"<<j_pos<<" +"<<j_pos<<" +"<<j_pos<<" +"<<j_pos<<" +"<<j_pos;
    else
        ss<<j_pos<<" "<<j_pos<<" "<<j_pos<<" "<<j_pos<<" "<<j_pos<<" "<<j_pos<<" "<<j_pos;

    //snprintf(msg, MAXLINE-1, "%lf", j2_pos);
    //std::cout<<"Sending: " << ss.str() <<std::endl; 

    std::string str = ss.str();
    msg = &str[0];
    
    //std::cout<<"Sending: " << msg <<std::endl;
    //std::cout<<"    len: " << strlen(msg) <<std::endl;

    sendto(sockfd, (const char *)msg, strlen(msg),  
        MSG_CONFIRM, (const struct sockaddr *) &cliaddr, 
            len);

    elapsed_time = clk.toc();
    exec_time += elapsed_time;

    if(elapsed_time > 0.01){
        std::cout<<"WARNING Client-elapsed: " << elapsed_time << "at exec time: " << overall_clk.toc() <<std::endl;
        std::cout<<"        receiv-elapsed: " << receive_elapsed_time << std::endl;
        //std::cout<<"    len: " << strlen(msg) <<std::endl;
        std::cout<<"    rec: " << n <<std::endl;
    }

    clk.tic();
     

    if(overall_clk.toc() > 20){
        std::cout<<"timed out" <<std::endl;
        sleep(5);
        running = false;
    }

    } while(running);

    char *stp_msg = "stop";

    sendto(sockfd, (const char *)stp_msg, strlen(stp_msg),  
        MSG_CONFIRM, (const struct sockaddr *) &cliaddr, 
            len); 


    std::cout<<"Server exited."<<std::endl;

    close(sockfd);
      
    return 0; 
} 
*/
