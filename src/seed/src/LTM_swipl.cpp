/*
 * File:   main.cpp
 * Author: hargalaten
 *
 * Created on 4 dicembre 2012, 13.59
 */

#include "LTM_swipl.h"



namespace seed {
    
LongTermMemory_swipl::LongTermMemory_swipl(int &argc, char **argv){

    //int ac = argc + 1;
    //char *av[] = {argv[0], (char*)"--no-signals"};
    //swi_engine = new PlEngine(ac,av);
    
    (void)argc; //suppress unused parameter warning
    swi_engine = new PlEngine(argv[0]);

}

bool LongTermMemory_swipl::close(){
    
    delete swi_engine;
    
    std::cout << "SWIPL: LTM closed"<<std::endl;
    
    return true;
}
    
bool LongTermMemory_swipl::loadLTM(std::string path){
    
    ltm_path = path;

    PL_thread_attach_engine(NULL);

    PlTermv args(1);
    args[0] = ltm_path.c_str();

    try{
        PlQuery pq("consult", args);

        if (pq.next_solution()) {
            std::cout << "SWIPL: LTM loaded"<<std::endl;
            PL_thread_destroy_engine();
            return true;
        } else {
            std::cout << "SWIPL: LTM load fail"<<std::endl;
            PL_thread_destroy_engine();
            return false;
        }
    }
    catch(const PlException& e){
        PL_thread_destroy_engine();
        return false;
    }
}

bool LongTermMemory_swipl::loadNodeSemantics(WM_node *new_node){
    
    bool have_semantics = false;
    
    PL_thread_attach_engine(NULL);
    
    std::string semantic_from_prolog, goal_from_prolog, regulations_from_prolog;
    
    //adjust the string and replace the \ operator with a space (replaced 01/12/2020 in SEED 4.0)
    //const char *cstr = new_node->instance.c_str();
    std::string new_inst = new_node->instance.c_str();
    replaceAll(new_inst," "," \\ ");
    const char *cstr = new_inst.c_str();
   
    try {
        
        //PlTermv inputs(2);
        //PlTermv inputs(3);
        PlTermv inputs(4);
        inputs[0] = PlCompound(cstr);

        //std::cout<<"QUERY: "<<cstr<<std::endl;
        
        PlQuery pq("schema", inputs);

        if( pq.next_solution() ) {
            //std::cout << (char*) inputs[1] << std::endl;
            
            semantic_from_prolog = (char*) inputs[1];
            
            //adjust the string and replace the \ operator with a space (added 01/12/2020 in SEED 4.0)
            replaceAll(semantic_from_prolog,"(is)","is");
            replaceAll(semantic_from_prolog," -","-");
            replaceAll(semantic_from_prolog,"\\"," ");
            
            //check if the semantic is a sequence
            if(semantic_from_prolog[0] == '$'){
                std::cout<<new_node->instance<<" is SEQUENTIAL!"<<std::endl;
                semantic_from_prolog.erase(semantic_from_prolog.begin());
                
                new_node->is_sequential = true;
            }
            
            replace_point(semantic_from_prolog);
            
            loadSemantics(semantic_from_prolog, new_node);
            
            //new GOAL procedure (added 02/12/2020 in SEED 4.0)
            goal_from_prolog = (char*) inputs[2];
            //std::cout<<"GOAL: "<<goal_from_prolog<<std::endl;
                
            //adjust the string and replace the \ operator with a space (added 01/12/2020 in SEED 4.0)
            replaceAll(goal_from_prolog,"(is)","is");
            replaceAll(goal_from_prolog," -","-");
            replaceAll(goal_from_prolog,"\\"," ");

            replace_point(goal_from_prolog);

            std::vector<std::string> goal_vec = read_list(goal_from_prolog);

            //if the list is not empty, the node is teleological
            new_node->teleological = ( goal_vec.size()>0 );
            
            for(size_t i=0; i<goal_vec.size(); i++ ){
                new_node->goal.push_back(goal_vec[i]);
            }
            
            //new REGULATIONS procedure (added 08/04/2022 in SEED 5.0)
            regulations_from_prolog = (char*) inputs[3];
            //std::cout<<"REGS: "<<regulations_from_prolog<<std::endl;

            //adjust the string and replace the \ operator with a space (added 01/12/2020 in SEED 4.0)
            replaceAll(regulations_from_prolog,"(is)","is");
            replaceAll(regulations_from_prolog," -","-");
            replaceAll(regulations_from_prolog,"\\"," ");

            replace_point(regulations_from_prolog);

            std::vector<std::string> reg_vec = read_list(regulations_from_prolog);

            for(size_t i=0; i<reg_vec.size(); i++ ){
                //std::cout<<"\t "<<reg_vec[i]<<std::endl;
                new_node->contribution[reg_vec[i]] = new double(0.0); //add 0 as default value
            }


            have_semantics = true;
            
        }
        else
            std::cout << "SWIPL: no semantics for "<<new_node->instance<<std::endl;
        
    } catch (PlException& e) {
        std::cout << "SWIPL: Query error for "<<new_node->instance<<std::endl;
        std::cout << (char*) e << std::endl;
    }
    
    PL_thread_destroy_engine();
    
    return have_semantics;
}



std::vector<std::string> LongTermMemory_swipl::read_list(std::string list){
    std::vector<std::string> lv;
    
    //std::cout<<"\t\t pre: "<<list<<std::endl;
    
    std::replace( list.begin(), list.begin()+1, '[', '(' );
    std::replace( list.begin()+list.size()-1, list.end(), ']', ')' );
    
    //std::cout<<"\t\t pos: "<<list<<std::endl;
    
    lv = instance2vector(list);
    lv.erase(lv.begin());
    
    return lv;
}

/**
 *
 * @param FromEclipse
 * @param expNode
 * carica la definizione semantica restituita da ECLIPSE (FromEclipse)
 * istanziando i figli del nodo da espandere (expNode)
 */
void LongTermMemory_swipl::loadSemantics(std::string semantic_from_prolog ,WM_node* expNode){
    
    WM_node *newSon;
    
    std::vector<std::string> sv = read_list(semantic_from_prolog);

    
    for(size_t i=0; i<sv.size(); i++){
        
        std::string sem_elem = sv[i];
        
        std::vector<std::string> ev = read_list(sem_elem);
        
        newSon=expNode->addSon( ev[0] );

        //set LTM contribution
        //float number = 0.0f;
        //std::istringstream istr(ev[1]);
        //istr.imbue(std::locale::classic());
        //istr >> number;
        double number = ston(ev[1]);

        double *ltm_contribution = new double(number); //new double(std::stod(ev[1]) - 1);
        //std::cout<<"\t contr: "<<ev[1]<<", "<<*ltm_contribution<<std::endl;
        if(*ltm_contribution > 0)
            newSon->contribution["LTM"] = ltm_contribution;
        
        if( expNode->is_sequential ){
            newSon->in_sequence = true;
            if( i==0 ){
                newSon->sequentialReleaser = true;
                std::cout<<"\t"<<newSon->instance<<" starts the SEQUENCE"<<std::endl;
            }
        }
        
        newSon->background = expNode->background;
        
        std::vector<std::string> rv = read_list(ev[2]);
        
        for(size_t j=0; j<rv.size(); j++){
            
            std::string rel_elem = rv[j];
            
            rel_elem.erase(std::remove(rel_elem.begin(), rel_elem.end(), '"'), rel_elem.end());
            
            newSon->releaser.push_back( rel_elem );
            //std::cout<<"\t "<<rel_elem<<std::endl;
        }
        
    }
        
}

PlTerm LongTermMemory_swipl::string2term(std::string str, std::unordered_map<std::string, PlTerm> &swi_vars){
    std::vector<std::string> strv = instance2vector(str);
    
//    if(strv.size()<=0)
//        return PlCompound(); //not allowed!
    
    //std::cout<<"inside string2compound"<<std::endl;
    
    std::string fun_name = strv[0];
    
    PlTermv fun_args( strv.size()-1 );
    
    try {
        
        //if it is an atom
        if(strv.size()<=1){
            
            if(fun_name.at(0) != '_' && !isupper(fun_name.at(0)) ){
                //std::cout<<"\t term: "<<fun_name<<std::endl;
                return PlTerm(fun_name.c_str());
            }
            else if(fun_name.at(0) != '_'){
                
                auto it = swi_vars.find(fun_name);
                
                if (it != swi_vars.end()){
                    //std::cout<<"\t old var: "<<fun_name<<std::endl;
                    return swi_vars[fun_name];
                }
                else {
                    //std::cout<<"\t new var: "<<fun_name<<std::endl;
                    swi_vars[fun_name] = PlCompound(fun_name.c_str());
                    return swi_vars[fun_name];
                }   
            }
        }
        
        //std::cout<<"\t compound: "<<str<<std::endl;
        
        for(size_t i=1; i<strv.size(); i++){
            fun_args[i-1] = string2term(strv[i],swi_vars);
        }
        
    } catch (PlException& e) {
        std::cout << "SWIPL: Query error!"<<std::endl;
        std::cout << (char*) e << std::endl;
    }
    
    return PlCompound(fun_name.c_str(),fun_args);
    
}


/**
 * post a generic goal to prolog and get the answer as a string 
 * containing a list of functors.
 * 
 * if no answer available, the returned string is empty
 * 
 */
std::string LongTermMemory_swipl::query(std::string request){
    
    std::unordered_map<std::string, PlTerm> swi_vars;// = new std::unordered_map<std::string, term_t>();
    
    std::string from_prolog = "";
    
    std::vector<std::string> rv = instance2vector(request);
    
    std::stringstream ss;
    
    if( rv.size()<=1 )
        return "";
    
    PL_thread_attach_engine(NULL);
    
    try {
        
        PlTermv inputs( rv.size()-1 );
        for(size_t i=1; i<rv.size(); i++){
            //if(rv[i].at(0) != '_' && !isupper(rv[i].at(0)) )
                //inputs[i-1] = PlCompound(rv[i].c_str());
            inputs[i-1] = string2term(rv[i],swi_vars);
        }
        
        //send the request
        PlQuery pq(rv[0].c_str(), inputs);

        if( pq.next_solution() ) {
        
            ss<<rv[0]<<"(";
            
            bool not_first = false;
            
            for(size_t i=0; i<rv.size()-1; i++){
                from_prolog = (char*) inputs[i];
                replace_point(from_prolog);
                
                if(not_first)
                    ss<<",";
                else
                    not_first = true;
                
                ss<<from_prolog;
            }
            ss<<")";
            
        }
        
    } catch (PlException& e) {
        std::cout << "SWIPL: Query error!"<<std::endl;
        std::cout << (char*) e << std::endl;
    }
    
    //free SWI
    PL_thread_destroy_engine();

    //return from_prolog;
    
    return ss.str();
}



} //seed namespace

