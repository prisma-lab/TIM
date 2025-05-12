#ifndef STD_BehaviorS_H
#define STD_BehaviorS_H

#include "seed.h"

using namespace seed;

/*
*   This header collects standard behaviors of SEED allowing 
*   basic functionalities.
*
*   Added in SEED 6.0 (ROS2 version)
*/


/*
*
* This behavior loads a schema to a specific node of the WM.
*   NOTE: if no node is specified, the alive node is selected.
*
*       remember(remNode,parentNode)
*           remNode: instance of the schema to be loaded
*           parentNode (optional): instance of the parent node
*/
class RememberBehavior : public Behavior{
public:
    RememberBehavior(std::string instance){
        setInstance(instance);
        setRtm(QUIESCENCE);

        std::vector<std::string> iv = instance2vector(this->getInstance());

        instance_to_remember = iv[1];

        if(iv.size()>2)
            parent_of_instance = iv[2];
        else
            parent_of_instance = "alive"; // add the node to alive by default

    }
    std::string getName(){
        return "remember";
    }
    bool perceptualSchema(){
        return true;
    }
    void motorSchema(){
        
        pthread_mutex_lock(&memMutex);
        //se la WM è stata deallocata esci
        if(WM==NULL){
            pthread_mutex_unlock(&memMutex);
            return;
        }
        //se l'instanza da ricordare non è presente nella WM
        std::vector<WM_node *> r_nodes = WM->getNodesByInstance(instance_to_remember);
        std::vector<WM_node *> p_nodes = WM->getNodesByInstance(parent_of_instance);
        if(r_nodes.size() == 0 && p_nodes.size() != 0)
            //aggiungi il nuovo nodo alla WM
            p_nodes[0]->addSon(instance_to_remember);
        //rimuovi il mio nodo dalla WM
        //remove((WM->getNodesByInstance(this->getInstance()))[0]);
        wmv_set<bool>(parent_of_instance + ".remember." + instance_to_remember, true);
        pthread_mutex_unlock(&memMutex);
    }
    void start(){};
    void exit(){
        // clear WMVs
        pthread_mutex_lock(&memMutex);
        wmv_set<bool>(parent_of_instance + ".remember." + instance_to_remember, false);
        pthread_mutex_unlock(&memMutex);

    };
private:
    std::string instance_to_remember;
    std::string parent_of_instance;
};


/*
*
* This behavior removes a schema from the WM.
*   NOTE: if the alive node is forgotten, the SEED's execution ends.
*
*       forget(forNode)
*           forNode: instance of the node to be removed
*/
class ForgetBehavior : public Behavior{
public:
    ForgetBehavior(std::string instance){
        setInstance(instance);
        setRtm(QUIESCENCE);
    }
    std::string getName(){
        return "forget";
    }
    bool perceptualSchema(){
        return true;
    }
    void motorSchema(){
        std::vector <WM_node *> remNode;
        pthread_mutex_lock(&memMutex);
        
        if(WM==NULL){
            pthread_mutex_unlock(&memMutex);
            return;
        }
        
        remNode=WM->getNodesByInstance(instance2vector(this->getInstance())[1]);
        std::cout<<"removing: "<<instance2vector(this->getInstance())[1]<<"\n";
        for(size_t i=0; i<remNode.size(); i++)
        {
            remove(remNode[i]);
            if(remNode[i]==WM){
                WM=NULL;
                pthread_mutex_unlock(&memMutex);
                return;
            }
        }

        //check if I have not removed myself!
        if(WM->getNodesByInstance(this->getInstance()).size()>0)
            remove(WM->getNodesByInstance(this->getInstance())[0]);
        
        pthread_mutex_unlock(&memMutex);
    }
    void exit(){};
    void start(){};
};


/*
*
* This behavior plots a list of all nodes currently loaded in WM.
*
*       listing
*           no parameters are requested
*/
class ListingBehavior : public Behavior{
public:
    ListingBehavior(std::string instance){
        setInstance(instance);
        setRtm(QUIESCENCE);
    }
    std::string getName(){
        return "listing";
    }
    bool perceptualSchema(){
        return true;
    }
    void motorSchema(){
        pthread_mutex_lock(&memMutex);
        if(dead()) return;
        remove((WM->getNodesByInstance(this->getInstance()))[0]);
        printWM(WM);
        pthread_mutex_unlock(&memMutex);
    }
    void run(){
        
    }
    void start(){}
    void exit(){}
};


/*
*
* This behavior allows interaction with stdin (command line).
*   All schemata provided in stdin are loaded into the WM under the
*   requestStream node.
*   NOTE: this node is usually loaded on start, along with alive.
*
*       inputStream
*           no parameters are requested
*/
class InputStreamBehavior : public Behavior {
public:   
    InputStreamBehavior(std::string instance){
        setInstance(instance);
        setRtm(QUIESCENCE);
        reqStream="requestStream";

    }
    std::string getName(){
        return "inputStream";
    }
    bool perceptualSchema(){
        //std::cin>>input;

        std::getline(std::cin,input);
        importance=0;
        while(input[input.size()-1]=='+'){
            input.erase(input.size()-1);
            importance++;
        }

        return true;
    }
    void motorSchema(){
        std::vector<WM_node *> reqs;
        
        //std::cout<< "SYSTEM: " << input << "\n";
        
        pthread_mutex_lock(&memMutex);
        if(dead())
            return;

        reqs=WM->getNodesByInstance( reqStream );
        if(reqs.size()>0){
            //WM_node *son;
            //son=reqs[0]->addSon(input);
            reqs[0]->addSon(input);
            //son->ltMagnitude+=importance;
        }
        else
            std::cout<<"SYSTEM: no requestStream!\n";
        
        pthread_mutex_unlock(&memMutex);
        
    }
    void exit(){
        //std::cout<<getName()<<" EXITED\n";
    }
    void start(){
        //std::cout<<"SYSTEM: "<<getName()<<" on!\n";
    }

    void replaceAll(std::string& str, const std::string& from, const std::string& to) {
        if(from.empty())
            return;
        size_t start_pos = 0;
        while((start_pos = str.find(from, start_pos)) != std::string::npos) {
            str.replace(start_pos, from.length(), to);
            start_pos += to.length(); // In case 'to' contains 'from', like replacing 'x' with 'yx'
        }
    }
protected:
    std::string input;
    int importance;
    std::string reqStream;

    std::string goal;
    bool new_goal;
};

#endif	/* STD_BehaviorS_H */