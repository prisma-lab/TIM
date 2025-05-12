/* 
 * File:   test.h
 * Author: hargalaten
 *
 * Created on 17 giugno 2014, 14.28
 */

//#include "../seed_header.h"
#ifndef TEST_H
#define TEST_H

#include "seed.h"

//#include "openprs/opaque-pub.h"
//#include "openprs/mp-pub.h"
//#include "sys/times.h"
//#include "sys/vtimes.h"
//#include "sys/time.h"
//#include <csignal>
//#include <syscall.h>


/*
//  DEFAULT TEST BEHAVIOR, USE THIS AS A TEMPLATE TO WRITE YOUR OWN TEST BEHAVIOR

class TestBehavior : public Behavior{
public:
    TestBehavior(std::string instance){
        setName(instance2vector(instance)[0]);
        setInstance(instance);
        setRtm(QUIESCENCE);
    }
    bool perceptualSchema(){
        return true;
    }
    void motorSchema(){
        
        pthread_mutex_lock(&memMutex);
        this->setRtm(1);
        pthread_mutex_unlock(&memMutex);
    }
    void start() {
    }
    void exit(){
    }
protected:
};
*/


class TestBehavior : public Behavior{
public:
    TestBehavior(std::string instance){
        setInstance(instance);
        setRtm(QUIESCENCE);
    }
    std::string getName(){
        return "test";
    }
    bool perceptualSchema(){
        return true;
    }
    void motorSchema(){
        current_rtm -= 0.05;

        if(current_rtm<0.1)
            current_rtm = 1;

        std::cout<<"test-elapsed: "<<clk.toc()<<std::endl;
        clk.tic();

        pthread_mutex_lock(&memMutex);
        this->setRtm(current_rtm);
        pthread_mutex_unlock(&memMutex);
    }
    void start() {
        clk.tic();
        current_rtm = 1;
    }
    void exit(){
    }
protected:
    seed::time::Clock clk;
    double current_rtm;
};



/*
//test NeuralNetwork
//  neuralNetwork node is a node that allow to learn part of the task

#include "../utils/machine_learning/seed_nn.h"
//rand
//#include "../utils/seed_geometry.h"

//disable eigen multithread
//#define EIGEN_DONT_PARALLELIZE

#define FIND_BEST_NN 1

class TestBehavior : public Behavior{
public:
    TestBehavior(std::string instance){
        setName(instance2vector(instance)[0]);
        setInstance(instance);
        setRtm(QUIESCENCE);
        
        //path_train = SEED_HOME_PATH + "/datasets/nn_test";
        //path_test = SEED_HOME_PATH + "/datasets/nn_test";
        
        path_train = SEED_HOME_PATH + "/datasets/nn_icosaf_train";
        path_test = SEED_HOME_PATH + "/datasets/nn_icosaf_test";
        
        //ml::genData(SEED_HOME_PATH + "/datasets/nn_test",1000); //generates -in and -out files
        
        best_accuracy = 1000;
        best_topology = {5,7,4};
        
        //use old best:
        best_net = ml::NeuralNetwork( SEED_HOME_PATH + "/datasets/best_network.nn" );
        best_accuracy = 0.353125;
        best_topology = best_net.topology;
        
        
    }
    bool perceptualSchema(){
        
        seed::time::Clock clk;
        
        //testRandomNetwork("NN2");
        
        //testRandomNetwork("NN3");
        
        pthread_mutex_lock(&memMutex);
        
        pthread_mutex_unlock(&memMutex);
        
        return true;
    }
    void motorSchema(){
        
        std::vector<ml::RowVector*> in_dat, out_dat; 
        
        uint in_layer,out_layer;
        
#if FIND_BEST_NN
        
        in_layer = ml::ReadCSV(path_train + "-in", in_dat); 
        out_layer = ml::ReadCSV(path_train + "-out", out_dat);
        
        //create net
        ml::NeuralNetwork n = randomNetwork(in_layer,out_layer);
        
//        if(rnd.real(0,1) > 0.3){
//            curr_topology = randomTopology(in_layer,out_layer);
//        }
//        else
//            curr_topology = best_topology;
        
        //std::vector<uint> rt = {5,7,6,4};
        //curr_topology = {5,7,4};
        
        //ml::NeuralNetwork n( curr_topology );
#else
        //old network
        ml::NeuralNetwork n( SEED_HOME_PATH + "/datasets/best_network.nn" );
#endif
        
        // --
        
#if FIND_BEST_NN
        //shuffle dataset
        shuffle(in_dat, out_dat);
        
        //train net
        n.train(in_dat, out_dat);
        
        ml::DeleteCSV(in_dat); 
        ml::DeleteCSV(out_dat);
#endif
        
        ml::ReadCSV(path_test + "-in", in_dat); 
        ml::ReadCSV(path_test + "-out", out_dat);
        
#if !FIND_BEST_NN
        n.set_input_limits(in_dat);
#endif
        
        n.test(in_dat, out_dat);
        //std::cout<<"    " << net_name << " AVG test: "<< n.avgLastErrors(-1)<<std::endl<<std::endl;
        
        ml::DeleteCSV(in_dat); 
        ml::DeleteCSV(out_dat);
        
        double accuracy = n.avgLastErrors(-1);
        
        std::cout<<"    "<< " AVG test: "<< accuracy <<std::endl;
        
        pthread_mutex_lock(&memMutex);
        
        if(accuracy < 0.04){
            remove(WM->getNodesByInstance(this->getInstance())[0]);
            
            n.save(SEED_HOME_PATH + "/datasets/best_network.nn");
            std::cout<<ansi::green<< " BEST Network SAVED"<<ansi::end<<std::endl;
        }
        else if(accuracy < best_accuracy){
            best_accuracy = accuracy;
            best_topology = n.topology;
            best_activations = n.activation_type;
            best_net = n;
            
            n.save(SEED_HOME_PATH + "/datasets/best_network.nn");
            std::cout<<ansi::green<< " NEW BEST Network SAVED!"<<ansi::end<<std::endl;
        }

        pthread_mutex_unlock(&memMutex);
        
        std::cout<<ansi::cyan<< " CURR test: "<< accuracy <<std::endl;
        plotNetParams(n.topology,n.activation_type);
        std::cout<<"  rate: "<<n.learningRate<<std::endl;
        std::cout<<ansi::end<<std::endl;
        
        std::cout<<ansi::green<< " BEST test: "<< best_accuracy <<std::endl;
        plotNetParams(best_topology,best_activations);
        std::cout<<"  rate: "<<best_net.learningRate<<std::endl;
        std::cout<<ansi::end<<std::endl<<std::endl;
        
        //sleep(1);
    }
    void start() {
    }
    void exit(){
        
    }
    
    double testRandomNetwork(std::string net_name){
        std::vector<ml::RowVector*> in_dat, out_dat; 
        
        uint in_layer,out_layer;
        
        in_layer = ml::ReadCSV(path_train + "-in", in_dat); 
        out_layer = ml::ReadCSV(path_train + "-out", out_dat);
        
        //create net
        ml::NeuralNetwork n = randomNetwork(in_layer,out_layer);
        //std::vector<uint> rt = {5,7,6,4};
        
        //ml::NeuralNetwork n( curr_topology );
        // --
        
        //shuffle dataset
        shuffle(in_dat, out_dat);
        
        //train net
        n.train(in_dat, out_dat);
        
        ml::DeleteCSV(in_dat); 
        ml::DeleteCSV(out_dat);
        
        ml::ReadCSV(path_test + "-in", in_dat); 
        ml::ReadCSV(path_test + "-out", out_dat);
        
        n.test(in_dat, out_dat);
        //std::cout<<"    " << net_name << " AVG test: "<< n.avgLastErrors(-1)<<std::endl<<std::endl;
        
        ml::DeleteCSV(in_dat); 
        ml::DeleteCSV(out_dat);
        
        return n.avgLastErrors(-1);
    }
    
    void plotNetParams(std::vector<uint> topo, std::vector<std::string> act){
        std::cout<<"    topology: [ ";
        for(auto i=0; i<topo.size(); i++){
            std::cout<<topo[i]<<" ";
        }
        std::cout<<" ]"<<std::endl;
        
        std::cout<<"    activations: [ ";
        for(auto i=0; i<act.size(); i++){
            std::cout<<act[i]<<" ";
        }
        std::cout<<" ]"<<std::endl;
    }
    
    ml::NeuralNetwork randomNetwork(int in_layers, int out_layer){
        
        //params
        int min_hidden_layers = 3;
        int max_hidden_layers = 10;
        
        int min_nodes_ratio = 1;
        int max_nodes_ratio = 7;
        
        
        //generate random number of layers
        int h_layers = rnd.integer(min_hidden_layers,max_hidden_layers);
        int layer_nodes;
        
        //create the topology
        std::vector<uint> topo;
        
        std::vector<std::string> act;
        
        topo.push_back(in_layers);
        act.push_back("tanh"); //not used
        
        for(auto i=0; i<h_layers; i++){
            //generate random number of nodes for that layer
            layer_nodes = rnd.integer( in_layers * min_nodes_ratio, in_layers * max_nodes_ratio);
            
            topo.push_back( layer_nodes );
            
            double coin = rnd.real(0,1);
            
            if( coin > 0.6 )
                act.push_back("tanh");
            else if( coin > 0.4 )
                //act.push_back("leakyReLU");
                act.push_back("ReLU");
            else if( coin > 0.2 )
                //act.push_back("not_leakyReLU");
                act.push_back("not_ReLU");
            else
                act.push_back("gaussian");
            
//            act.push_back("tanh");
            
        }
        act.push_back("tanh"); //not used
        topo.push_back(out_layer);
        
        std::cout<<"NN, created random network:"<<std::endl;
        plotNetParams(topo, act);
        
        return ml::NeuralNetwork( topo, act, 5.0/std::pow(10,rnd.integer(2,5)) );
    }
    
    void shuffle(std::vector<ml::RowVector*> &input_data, std::vector<ml::RowVector*> &output_data) {

        int k = 3;
        
        int id1,id2;

        ml::RowVector* in_app;
        ml::RowVector* out_app;
        
        for (uint i = 0; i < input_data.size()*k; i++) {

            id1=rnd.integer(0,input_data.size()-1);
            id2=rnd.integer(0,input_data.size()-1);
            
//            std::cout<<i<<" on "<<input_data.size()*k<<std::endl;
//            std::cout<<"exchange "<<id1<<" with "<<id2<<std::endl;
            
            //exchange the 2 elements
            in_app = input_data[id1];
            out_app = output_data[id1];
            
            input_data[id1] = input_data[id2];
            output_data[id1] = output_data[id2];
            
            input_data[id2] = in_app;
            output_data[id2] = out_app;
        }
        
        std::cout<<"NN, dataset shuffled"<<std::endl;
    }
    
protected:
    std::string path_train;
    std::string path_test;
    
    Random rnd;
    
    std::vector<uint> curr_topology;
    std::vector<uint> best_topology;
    std::vector<std::string> best_activations;
    ml::NeuralNetwork best_net;
    double best_accuracy;
};
*/

/*
class TestBehavior : public Behavior{
public:
    TestBehavior(std::string instance){
        setName(instance2vector(instance)[0]);
        setInstance(instance);
        setRtm(QUIESCENCE);
    }
    bool perceptualSchema(){
        
        return true;
    }
    void motorSchema(){
        
        pthread_mutex_lock(&memMutex);
        this->setRtm(1);
        
        pthread_mutex_unlock(&memMutex);
    }
    void start() {
        // Data for visual representation
    int width = 512, height = 512;
    cv::Mat image = cv::Mat::zeros(height, width, CV_8UC3);
    int dn=6;
    // Set up training data
    float labels[6] = {-1.0, 1.0, 1.0, -1.0, 1.0, 1.0};
    cv::Mat labelsMat(dn, 1, CV_32FC1, labels);

    float trainingData[6][2] = { {300, 100}, {200, 100}, {100, 100}, {50, 100}, {150, 100}, {125, 100} };
    cv::Mat trainingDataMat(dn, 2, CV_32FC1, trainingData);

    // Set up SVM's parameters
    cv::SVMParams params;
    params.svm_type    = cv::SVM::C_SVC;
    params.kernel_type = cv::SVM::RBF;
    params.term_crit   = cv::TermCriteria(CV_TERMCRIT_ITER, 100, 1e-6);
    params.gamma       = 2;
    
    // Train the SVM
    cv::SVM SVM;
    SVM.train(trainingDataMat, labelsMat, cv::Mat(), cv::Mat(), params);

    cv::Vec3b green(0,255,0), blue (255,0,0);
    // Show the decision regions given by the SVM
    for (int i = 0; i < image.rows; ++i)
        for (int j = 0; j < image.cols; ++j)
        {
            cv::Mat sampleMat = (cv::Mat_<float>(1,2) << j,i);
            float response = SVM.predict(sampleMat);

            if (response == 1)
                image.at<cv::Vec3b>(i,j)  = green;
            else if (response == -1)
                 image.at<cv::Vec3b>(i,j)  = blue;
        }

    // Show the training data
    int thickness = -1;
    int lineType = 8;
    cv::Scalar pColor;
    for(int k=0;k<dn;k++){
        if(labels[k]==1.0)
            pColor=cv::Scalar(  0,   0,   0);
        else
            pColor=cv::Scalar(255, 255, 255);
        
        cv::circle( image, cv::Point(trainingData[k][0],  trainingData[k][1]), 5, pColor, thickness, lineType);
    }
//    cv::circle( image, cv::Point(501,  10), 5, cv::Scalar(  0,   0,   0), thickness, lineType);
//    cv::circle( image, cv::Point(255,  10), 5, cv::Scalar(255, 255, 255), thickness, lineType);
//    cv::circle( image, cv::Point(501, 255), 5, cv::Scalar(255, 255, 255), thickness, lineType);
//    cv::circle( image, cv::Point( 10, 501), 5, cv::Scalar(255, 255, 255), thickness, lineType);

    // Show support vectors
    thickness = 2;
    lineType  = 8;
    int c     = SVM.get_support_vector_count();

    for (int i = 0; i < c; ++i)
    {
        const float* v = SVM.get_support_vector(i);
        cv::circle( image,  cv::Point( (int) v[0], (int) v[1]),   6,  cv::Scalar(128, 128, 128), thickness, lineType);
    }

    cv::imwrite("result.png", image);        // save the image

    cv::imshow("SVM Simple Example", image); // show it to the user
    cv::waitKey(0);

    }
    void exit(){
    }
protected:
};
*/

/*
class CpuMonitorBehavior : public Behavior{
public:
    CpuMonitorBehavior(std::string instance){
        setName(instance2vector(instance)[0]);
        setInstance(instance);
        setRtm(QUIESCENCE);
        
        FILE* file;
        struct tms timeSample;
        char line[128];
        
        path="/home/hargalaten/Scrivania/seed_cpu/usage.txt";
        
        lastCPU = times(&timeSample);
        lastSysCPU = timeSample.tms_stime;
        lastUserCPU = timeSample.tms_utime;
    

        file = fopen("/proc/cpuinfo", "r");
        numProcessors = 0;
        while(fgets(line, 128, file) != NULL){
            if (strncmp(line, "processor", 9) == 0) numProcessors++;
        }
        fclose(file);
        
        oldTime=0;
        
        
    }
    bool perceptualSchema(){
        
        newTime=time(NULL);
        
        if(newTime>oldTime)
            oldTime=newTime;
        else
            return false;
        
        return true;
    }
    void motorSchema(){
        
        pthread_mutex_lock(&memMutex);
        this->setRtm(0.01);
        pthread_mutex_unlock(&memMutex);
        
        struct tms timeSample;
        double percent;
        

        now = times(&timeSample);
        if (now <= lastCPU || timeSample.tms_stime < lastSysCPU ||
            timeSample.tms_utime < lastUserCPU){
            //Overflow detection. Just skip this value.
            percent = -1.0;
        }
        else{
            percent = (timeSample.tms_stime - lastSysCPU) +
                (timeSample.tms_utime - lastUserCPU);
            percent /= (now - lastCPU);
            percent /= numProcessors;
            percent *= 100;
        }
        lastCPU = now;
        lastSysCPU = timeSample.tms_stime;
        lastUserCPU = timeSample.tms_utime;
        
        //std::cout<<"SYSTEM: ("<<newTime<<") CPU usage "<<percent<<"%\n";
        outFile.open(path.c_str(),std::ios::app);
        outFile<<newTime<<" "<<percent<<"\n";
        outFile.close();
        
    }
    void start() {

    }
    void exit(){
    }
protected:
    clock_t lastCPU, lastSysCPU, lastUserCPU;
    clock_t now;
    int numProcessors;
    time_t oldTime,newTime;
    std::ofstream outFile;
    std::string path;
};
*/



/* TESTING DI APPRENDIMENTO E PARAMATER SETTING
 *
#define DEFAULTSTEP 2 //frazione su cui basare il passo casuale (step=X/DEFAULTSTEP)

class TestBehavior : public Behavior{
public:
    TestBehavior(std::string instance){
        setName(instance2vector(instance)[0]);
        setInstance(instance);
        setRtm(QUIESCENCE);
        proximity=0.5;
        warning=0.01;
        k_weber=1;
        setLearning(2,&proximity,&warning);
        //setLearning(1,&k_weber);
        initNN();
        learned=false;
    }
    bool perceptualSchema(){
        
        //TEST: learning-avoidance
        double* s=new double[8];
        double appSin,appCos;
        int disabled;
        std::stringstream ss;
        totalMin=9999,frontalMin=9999;
        escapeSin=0,escapeCos=0;
        bool rel=false;
        
        pthread_mutex_lock(&memMutex);
        
	for(int i=0;i<8;i++){
            ss<<"sonar"<<i;
            s[i]=wmv_get<double>(ss.str());
            
            if(s[i]<totalMin)
                totalMin=s[i];
            
            if(i>=2 && i<=5 && s[i]<frontalMin)
                frontalMin=s[i];
            appSin=wmv_get<double>(ss.str()+".sin");
            appCos=wmv_get<double>(ss.str()+".cos");
            escapeSin+=appSin*s[i];
            escapeCos+=appCos*s[i];
            
            ss.str("");
        }
        
        //releaser interno
        disabled=wmv_get<double>("avoid.disabled");
        
        if(disabled==0 && totalMin<proximity)  
            rel=true;
        
        double fail=wmv_get<double>("engine.collide");
        
        pthread_mutex_unlock(&memMutex);
        
        if(fail==1 && !learned){
            std::cout<<"collide!!\n";
            //learnHS();
            learn();
            //learnHSNN();
            learned=true;
            //std::cout<<"newState: "<<proximity<<" "<<warning<<"\n";
            std::cout<<"newState: "<<k_weber<<"\n";
        }
        else if(fail==0 && learned)
            learned=false;
        
        performance=totalMin;
        
        
        return rel;
        
//        std::cout<<"newstep: ";
//        for(int i=0;i<toLearn.size();i++)
//            std::cout<<*toLearn[i]<<" ";
//        std::cout<<"\n";
    }
    void motorSchema(){
        
        double escapeTs=0,escapeFs=0;
        
        //performance++;
        runs++;
        
        escapeTs=rtod(atan2(-escapeSin,escapeCos))+fRand(-5,5);
        //std::cout<<"ts: "<<escapeTs<<"\n";
        //escapeFs=frontalMin-(1-warning); //inverse
        escapeFs=frontalMin-warning;
        
        if(escapeTs<-20) escapeTs=-20;
        else if(escapeTs>20) escapeTs=20;
        //else if(escapeTs>0 && escapeTs<5) escapeTs=5;
        //else if(escapeTs<0 && escapeTs>-5) escapeTs=-5;
        
        if(escapeFs>0.4) escapeFs=0.4;
        else if(escapeFs<-0.4) escapeFs=-0.4;
        
//        std::cout<<"fs: "<<escapeFs/2<<" ts: "<<escapeTs<<" totmin: "<<totalMin<<"\n";
        
        pthread_mutex_lock(&memMutex);
        
        updateRtm(totalMin,proximity,warning);
        //updateRtm(totalMin,1-proximity,1-warning); //inverse
        //updateRtm2(totalMin);
        
        wmv_compete("engineStream","engine.fs",escapeFs/2);
        wmv_compete("engineStream","engine.ts",escapeTs);

        
        pthread_mutex_unlock(&memMutex);
    }
    void start(){
        pthread_mutex_lock(&memMutex);
        wmv_set<double>("avoid.disabled",0);
        pthread_mutex_unlock(&memMutex);
    }
    void exit(){
    }
    void updateRtm2(double newStimulus){
        
        double newPeriod=0.01;
        double newRelevance;
        
        if(oldStimulus!=0) {
//            if( (oldFeature - newFeature) >= 0 )
//                newDelta = ( (oldFeature - newFeature) / oldFeature ) / oldRtm;
//            else
//                newDelta = oldRtm / ( (oldFeature - newFeature) / oldFeature );
            
            newRelevance = k_weber * ( (oldStimulus - newStimulus) / oldStimulus);
            
            newPeriod = oldPeriod - newRelevance;

            if (newPeriod < 0.01)
                newPeriod=0.01;
            else if (newPeriod > 1)
                newPeriod=1;
        }
        
        setRtm(newPeriod);
        
//        std::cout<<oldPeriod<<" >> "<<oldStimulus<<" -> "<<newStimulus<<" >> "<<this->getRtm()<<"\n";
        
        oldStimulus=newStimulus;
        oldPeriod=this->getRtm();
    }
    
    void learn() {
        std::vector<double> support;
        for(int i=0;i<toLearn.size();i++){
            support.push_back(*toLearn[i]);
        }
        randomWalk.push_back(support);
        bool discard = false;
        double distance;
        double sum;
        double count=0;
        std::vector<double> newstep;

        do {
            std::srand(std::time(NULL)+count);
            count++;
            std::cout<<"roll...\n";
            //nuovo random step
            newstep.clear();
            discard=false;
            for (int i = 0; i < support.size(); i++) {
                //ogni componente viene presa tra [0,2r] dalla componente del punto
                newstep.push_back(support[i] + (fRand(-1, 1)*((support[i]*2) / DEFAULTSTEP)));
                std::cout<<newstep[i]<<"\n";
            }
            
            //controlla la consistenza del nuovo punto
            for (int i = 0; i < randomWalk.size(); i++) {
                distance = 0;
                sum = 0;
                for (int j = 0; j < randomWalk[i].size(); j++) {
                    distance += (randomWalk[i][j] - newstep[j])*(randomWalk[i][j] - newstep[j]);
                    sum += randomWalk[i][j];
                    //std::cout<<"d: "<<distance<<"\n";
                }
                //se la distanza del newstep è inferiore alla media/DEFAULTSTEP 
                if (std::sqrt(distance) <= sum / (randomWalk[i].size() * DEFAULTSTEP))
                    //allora il punto si considera troppo vicino, quindi viene scartato
                    discard = true;
                
                //std::cout<<"dist: "<<std::sqrt(distance)<<" on "<<sum / (randomWalk[i].size() * DEFAULTSTEP)<<"\n";
            }
            //l'algoritmo viene iterato ficnhé non si trova un nuovo punto consistente
        } while(discard);
        
        //aggiorna lo stato
        for(int i=0;i<toLearn.size();i++){
            *toLearn[i]=newstep[i];
        }
    }
    void learnHSNN() {
        
        std::vector<double> support;
        for(int i=0;i<toLearn.size();i++){
            support.push_back(*toLearn[i]);
        }
        support.push_back(runs);
        trainingSet.push_back(support);
        
        runs=0;
        
        double maxRun=-1;
        
        for(int i=0;i<trainingSet.size(); i++){
            if(trainingSet[i][toLearn.size()]>maxRun)
               maxRun=trainingSet[i][toLearn.size()]; 
        }
        
        cv::Mat trainingData(trainingSet.size(),toLearn.size(),CV_32F);
        cv::Mat trainingClasses(trainingSet.size(),1,CV_32F);
        
        for(int r=0; r<trainingSet.size(); r++){
            for(int c=0; c<toLearn.size(); c++){
                trainingData.at<float>(r,c)=trainingSet[r][c];
            }
            trainingClasses.at<float>(r,0)=1-(trainingSet[r][toLearn.size()]/maxRun);
        }
        
        
        
        // train
        ann.train(trainingData, trainingClasses, cv::Mat(), cv::Mat(), params);
        
        
        cv::Mat trainImage((int) (origin[0] * 1000),(int) (origin[1] * 10000), CV_8UC3, cv::Scalar(255, 255, 255));
        cv::circle(trainImage, cv::Point((int) (origin[1] * 5000),(int) (origin[0] * 500)), 2, cv::Scalar(0, 0, 255),-1,8,0);
        
//        cv::imshow("randomPoints",trainImage);
//        cv::waitKey(5);
        
        cv::Mat response(1, 1, CV_32FC1);
        std::vector< std::vector<double> > randomSteps, harmonyMemory;
        double hms=10;
        double hmcr=0.9;
        double par=0.3;
        
        //aggoirna la harmonyMemory
        if (harmonyMemory.size() == hms) {
            int worstStep;
            double worstPerformance = -1;
            for (int i = 0; i < harmonyMemory.size(); i++) {
                if (worstPerformance == -1 ||
                        harmonyMemory[i][toLearn.size()] < worstPerformance) {
                    worstPerformance = harmonyMemory[i][toLearn.size()];
                    worstStep = i;
                }
            }
            
            std::cout<<"worst: ";
            for (int i = 0; i < harmonyMemory[worstStep].size(); i++) {
                std::cout<<harmonyMemory[worstStep][i]<<" ";
                harmonyMemory[worstStep][i] = support[i];
            }
            std::cout<<"\n";
        }
        else
            harmonyMemory.push_back(support);
        
        std::cout<<"HM-updated\n";
        
        for(int t=0;t<100;t++) {
            cv::Mat randomData(1,origin.size(),CV_32F);
            randomSteps.push_back(std::vector<double>());
            for (int i = 0; i < origin.size(); i++) {
                
                //scegli se prendere il valore dalla HarmonyMemory
                if(fRand(0.0,1.0)>hmcr){
                    //ogni componente viene presa tra [0,2x] dalla componente del punto
                    randomSteps[t].push_back(fGaussian(origin[i],origin[i]/2));
                    
                }
                else {
                    //notare che nel caso la matrice sia composta da un solo elemento
                    //questo caso è uguale al precedente.
                    timeval time;
                    gettimeofday(&time,NULL);
                    std::srand(((int) time.tv_sec) + t);
                    double step=std::rand() % harmonyMemory.size();
                    
                    
                    if(fRand(0.0,1.0)>par){ 
                        //non aggiustare il valore con probabilità 1-par
                        randomSteps[t].push_back(harmonyMemory[step][i]);
                        
                    }
                    else{
                        //aggiusta il valore con probabilità par
                        randomSteps[t].push_back(fGaussian(harmonyMemory[step][i],harmonyMemory[step][i]/2));
                        
                    }
                }
                
                
                
                //randomSteps[t].push_back(origin[i] + (fRand(-1.0, 1.0)*(origin[i])));
                //randomSteps[t].push_back(fGaussian(origin[i],origin[i]/2));
                //randomSteps[t].push_back(fRand(0.0, origin[i]));
                randomData.at<float>(0, i) = randomSteps[t][i];
            }
            
            ann.predict(randomData,response);
            randomSteps[t].push_back(response.at<float>(0,0));
            
            if((int) (randomSteps[t][1] * 5000)<=(int) (origin[1] * 10000) &&
                    (int) (randomSteps[t][0] * 500)<=(int) (origin[0] * 1000)){
                cv::circle(trainImage, cv::Point((int) (randomSteps[t][1] * 5000),(int) (randomSteps[t][0] * 500)), 1, cv::Scalar(255, 0, 0),1,8,0);
                std::stringstream ss;
                ss<<randomSteps[t][2];
                cv::putText(trainImage,ss.str(),cv::Point((int) (randomSteps[t][1] * 5000),(int) (randomSteps[t][0] * 500)), cv::FONT_HERSHEY_SIMPLEX,0.4,cv::Scalar(255, 0, 50),1,8,false);
            }
            
            
            
        } 
        cv::imshow("randomPoints",trainImage);
        cv::waitKey(5);
        
        double minVal=100;
        int minElem;
        
        for(int t=0;t<100;t++) {
            //std::cout<<randomSteps[t][toLearn.size()]<<"\n";
            if(randomSteps[t][toLearn.size()]<minVal){
                minVal=randomSteps[t][toLearn.size()];
                minElem=t;
            }
        }
        
        for(int i=0;i<toLearn.size();i++){
            *toLearn[i]=randomSteps[minElem][i];
        }
        std::cout<<"bestResponse: "<<randomSteps[minElem][toLearn.size()]<<"\n";
    }
    void learnNN() {
        
        std::vector<double> support;
        for(int i=0;i<toLearn.size();i++){
            support.push_back(*toLearn[i]);
        }
        support.push_back(runs);
        trainingSet.push_back(support);
        
        runs=0;
        
        double maxRun=-1;
        
        for(int i=0;i<trainingSet.size(); i++){
            if(trainingSet[i][toLearn.size()]>maxRun)
               maxRun=trainingSet[i][toLearn.size()]; 
        }
        
        cv::Mat trainingData(trainingSet.size(),toLearn.size(),CV_32F);
        cv::Mat trainingClasses(trainingSet.size(),1,CV_32F);
        
        for(int r=0; r<trainingSet.size(); r++){
            for(int c=0; c<toLearn.size(); c++){
                trainingData.at<float>(r,c)=trainingSet[r][c];
            }
            trainingClasses.at<float>(r,0)=1-(trainingSet[r][toLearn.size()]/maxRun);
        }
        
        
        
        // train
        ann.train(trainingData, trainingClasses, cv::Mat(), cv::Mat(), params);
        
        
//        cv::Mat trainImage((int) (origin[0] * 1000),(int) (origin[1] * 10000), CV_8UC3, cv::Scalar(255, 255, 255));
//        cv::circle(trainImage, cv::Point((int) (origin[1] * 5000),(int) (origin[0] * 500)), 2, cv::Scalar(0, 0, 255),-1,8,0);
        
//        cv::imshow("randomPoints",trainImage);
//        cv::waitKey(5);
        
        cv::Mat response(1, 1, CV_32FC1);
        std::vector< std::vector<double> > randomSteps;
        
        for(int t=0;t<100;t++) {
            cv::Mat randomData(1,origin.size(),CV_32F);
            randomSteps.push_back(std::vector<double>());
            for (int i = 0; i < origin.size(); i++) {
                //randomSteps[t].push_back(origin[i] + (fRand(-1.0, 1.0)*(origin[i])));
                randomSteps[t].push_back(fGaussian(origin[i],origin[i]/2));
                //randomSteps[t].push_back(fRand(0.0, origin[i]));
                randomData.at<float>(0, i) = randomSteps[t][i];
            }
            
            ann.predict(randomData,response);
            randomSteps[t].push_back(response.at<float>(0,0));
            
//            if((int) (randomSteps[t][1] * 5000)<=(int) (origin[1] * 10000) &&
//                    (int) (randomSteps[t][0] * 500)<=(int) (origin[0] * 1000)){
//                cv::circle(trainImage, cv::Point((int) (randomSteps[t][1] * 5000),(int) (randomSteps[t][0] * 500)), 1, cv::Scalar(255, 0, 0),1,8,0);
//                std::stringstream ss;
//                ss<<randomSteps[t][2];
//                cv::putText(trainImage,ss.str(),cv::Point((int) (randomSteps[t][1] * 5000),(int) (randomSteps[t][0] * 500)), cv::FONT_HERSHEY_SIMPLEX,0.4,cv::Scalar(255, 0, 50),1,8,false);
//            }
            
            
            
        } 
//        cv::imshow("randomPoints",trainImage);
//        cv::waitKey(5);
        
        double minVal=100;
        int minElem;
        
        for(int t=0;t<100;t++) {
            //std::cout<<randomSteps[t][toLearn.size()]<<"\n";
            if(randomSteps[t][toLearn.size()]<minVal){
                minVal=randomSteps[t][toLearn.size()];
                minElem=t;
            }
        }
        
        for(int i=0;i<toLearn.size();i++){
            *toLearn[i]=randomSteps[minElem][i];
            std::cout<<*toLearn[i]<<"\n";
        }
        std::cout<<"bestResponse: "<<randomSteps[minElem][toLearn.size()]<<"\n";
    }
     
    void initNN(){
        
        cv::Mat layers(4,1,CV_32S);
        layers.at<int>(0,0) = toLearn.size();//input layer
        layers.at<int>(1,0) = toLearn.size()*10;//hidden layer
        layers.at<int>(2,0) = toLearn.size()*15;//hidden layer
        layers.at<int>(3,0) = 1;//output layer
        
        runs=0;
        
        for(int i=0;i<toLearn.size();i++){
            origin.push_back(*toLearn[i]);
        }
        
        //ANN criteria for termination
        CvTermCriteria criter;
        criter.max_iter = 100;
        criter.epsilon = 0.00001f;
        criter.type = CV_TERMCRIT_ITER | CV_TERMCRIT_EPS;

        //ANN parameters
        params.train_method = CvANN_MLP_TrainParams::BACKPROP;
        params.bp_dw_scale = 0.05f;
        params.bp_moment_scale = 0.05f;
        params.term_crit = criter; //termination criteria

        ann.create(layers);
    }
    void learnHS() {
        //la matrice randomWalk diventa harmonyMemory
        double hms=3;
        double hmcr=0.9;
        double par=0.3;
        int randomStep;
        int max_iteration=hms*toLearn.size(); //numero di iterazioni
        
        std::vector<double> support;
        for(int i=0;i<toLearn.size();i++){
            support.push_back(*toLearn[i]);
        }
        support.push_back(performance);
        
        //aggoirna la harmonyMemory
        if (randomWalk.size() == hms) {
            int worstStep;
            double worstPerformance = -1;
            for (int i = 0; i < randomWalk.size(); i++) {
                if (worstPerformance == -1 ||
                        randomWalk[i][toLearn.size()] < worstPerformance) {
                    worstPerformance = randomWalk[i][toLearn.size()];
                    worstStep = i;
                }
            }
            
            std::cout<<"worst: ";
            for (int i = 0; i < randomWalk[worstStep].size(); i++) {
                std::cout<<randomWalk[worstStep][i]<<" ";
                randomWalk[worstStep][i] = support[i];
            }
            std::cout<<"\n";
        }
        else
            randomWalk.push_back(support);
        
        count=0;
        
        std::vector<double> newstep;

        //do {
            max_iteration--;
            std::cout<<"learn...\n";
            //nuovo random step
            newstep.clear();
            for (int i = 0; i < toLearn.size(); i++) {
                //scegli se prendere il valore dalla HarmonyMemory
                if(fRand(0.0,1.0)>hmcr){
                    //ogni componente viene presa tra [0,2x] dalla componente del punto
                    newstep.push_back(support[i]+(support[i]*fRand(-1, 1)));
                    std::cout<<"newstep: ";
                }
                else {
                    //notare che nel caso la matrice sia composta da un solo elemento
                    //questo caso è uguale al precedente.
                    randomStep=std::rand() % randomWalk.size();
                    
                    std::cout<<"oldstep, ";
                    
                    if(fRand(0.0,1.0)>par){ 
                        //non aggiustare il valore con probabilità 1-par
                        newstep.push_back(randomWalk[randomStep][i]);
                        std::cout<<"no-adjust: ";
                    }
                    else{
                        //aggiusta il valore con probabilità par
                        newstep.push_back(randomWalk[randomStep][i]+
                            fRand(-1, 1)*(randomWalk[randomStep][i] / DEFAULTSTEP) );
                        std::cout<<"adjust: ";
                    }
                }
                
                std::cout<<newstep[i]<<"\n";
            }
            
        //} while(max_iteration);
        
        //aggiorna lo stato
        for(int i=0;i<toLearn.size();i++){
            *toLearn[i]=newstep[i];
        }
    }
    double fRand(double fMin, double fMax) {
        count++;
//        std::srand(std::time(NULL)+count);        
//        double f = (double) std::rand() / RAND_MAX;
//        return fMin + f * (fMax - fMin);
        timeval time;
        gettimeofday(&time,NULL);
        boost::mt19937 seed(((int) time.tv_sec) + count);
        boost::uniform_real<> dist(fMin,fMax);
        boost::variate_generator<boost::mt19937&, boost::uniform_real<> > random(seed,dist);
        return random();
    }
    double fGaussian(double mean, double var) {
        count++;
        timeval time;
        gettimeofday(&time,NULL);
        boost::mt19937 seed(((int) time.tv_sec) + count);
        boost::normal_distribution<> dist(0,1);
        boost::variate_generator<boost::mt19937&, boost::normal_distribution<> > random(seed,dist);
        return (random()*var)+mean;
    }
    void setLearning(int argc, ...){
        va_list argv;
        va_start(argv,argc);
        for(int i=0;i<argc;i++){
            toLearn.push_back( va_arg(argv,double *) );
        }
        va_end(argv);
        
    }
protected:
    std::vector<double *> toLearn;
    std::vector< std::vector<double> > randomWalk;
    
    //learnNN
    std::vector< std::vector<double> > trainingSet;
    std::vector<double> origin;
    CvANN_MLP ann;
    CvANN_MLP_TrainParams params;
    double runs;
    
    //updateRtm
    double oldStimulus;
    double oldPeriod;
    double k_weber;
    
    bool learned;
    double escapeSin;
    double escapeCos;
    double totalMin;
    double frontalMin;
    double proximity,warning;
    double count;
    double performance;
};
*/

#endif	/* TEST_H */

