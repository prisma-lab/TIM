#ifndef LTM_BehaviorS_H
#define LTM_BehaviorS_H

#include "seed.h"

using namespace seed;

/*
*   This header collects behaviors for LTM management
*
*   Added in SEED 6.0 (ROS2 version)
*/

/*
*   posts a query to the LTM:
*       ltm(query_string) 
*           query_string: the string containing the query 
*                        
*                    eg. "ltm(schema(alive,X))" X will be
*                        unified with the list of sub-schemata
*                        of alive
*        
*       the result of the query is plotted for now
*
*       NOTE: this works with SWIpl, TOBE tested for Eclipseclp
*/
class LTMBehavior : public Behavior{
    public:
    LTMBehavior(std::string instance){
        setInstance(instance);
        setRtm(QUIESCENCE);

        ltm_query = instance2vector(instance)[1];
    }
    std::string getName(){
        return "ltm";
    }
    bool perceptualSchema(){
        return true;
    }
    void motorSchema(){
        pthread_mutex_lock(&memMutex);
        
        std::cout<<"LTM-QUERY: "<<std::endl;
        std::cout<<ltm_query<<std::endl;
        
        std::string ltm_ans = LTM->query(ltm_query);
        
        std::cout<<"LTM-ANS: "<<std::endl;
        std::cout<<ltm_ans<<std::endl;
        
        remove(WM->getNodesByInstance(this->getInstance())[0]);

        pthread_mutex_unlock(&memMutex);
    }
    void exit(){
    }
    void start(){
    }
    
    std::string ltm_query;
};

#endif	/* LTM_BehaviorS_H */