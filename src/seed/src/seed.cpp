/*
 * File:   main.cpp
 * Author: hargalaten
 *
 * Created on 4 dicembre 2012, 13.59
 */

//#include <qt5/QtWidgets/qapplication.h>

#include "seed.h"

//#include "LTM_eclipseclp.h" //eclipseclp PROLOG interface
#include "LTM_swipl.h" //swi PROLOG interface

//using namespace seed;

namespace seed {
    
WM_node *WM = new WM_node("alive",NULL);

WM_varmap WMV;

LongTermMemory *LTM;

BehaviorBasedSystem *BBS = new BehaviorBasedSystem();

pthread_mutex_t memMutex = PTHREAD_MUTEX_INITIALIZER;

std::string SYS_HOME_PATH;
std::string SEED_HOME_PATH;

std::string SEED_NAME;

time::Clock seed_global_clk;

rclcpp::Node::SharedPtr nh;

std::string winner("none");

int seed_wm_id = 0;

std::vector<pthread_t> seed_thread_list;

void faultHandler(int sig){
    void *trace[16];
    size_t size;
    char **messages = (char **)NULL;

    // get void*'s for all entries on the stack
    size = backtrace(trace, 16);

    // print out all the frames to stderr
    if(sig==11)
        fprintf(stderr, "SEED-ERROR (Segmentation Fault) backtrace:\n");
    else
        fprintf(stderr, "SEED-ERROR (signal %d) backtrace:\n", sig);

    //standard backtrace (no error line)
    //backtrace_symbols_fd(trace, size, STDERR_FILENO);

    //backtrace
    messages = backtrace_symbols(trace, size);
    for (size_t i = 1; i < size; ++i) {
        printf("[bt] #%d %s\n", (int) i, messages[i]);
        char syscom[256];
        //sprintf(syscom, "addr2line %p -e /home/phoenix/catkin_ws/devel/lib/seed_learn/seed_learn", trace[i]); //last parameter is the name of this app
        std::string address2line("addr2line %p -e " + SEED_HOME_PATH + "/../../install/seed/lib/seed/seed");
        //std::string address2line("addr2line %p -e " + SEED_HOME_PATH + "/../../build/seed/seed");
        sprintf(syscom, address2line.c_str(), trace[i]);
        system(syscom);
    }

    exit(1);
}

//convert radiant to degree
double rtod(double r){
    return r * 180.0/ M_PI;
}

//convert degree to radiant
double dtor(double r){
    return r * M_PI / 180.0;
}

////convert string to double
//double string_to_double(std::string string_to_convert, char separator){
//    std::stringstream ss(string_to_convert);
//    char c;
//    double new_num, d = 10.0;
//    int integer_part = 0;
//    double decimal_part = 0.0;
//    bool separator_found = false;
//    
//    ss>>c;
//    while(!ss.eof()){
//        
//        if(!separator_found && c == separator){
//            separator_found = true;
//            ss>>c;
//            continue;
//        }
//        
//        new_num = c - '0';
//        
//        if(!separator_found){
//            //integer part
//            integer_part = new_num + ( integer_part * 10 );
//        }
//        else{
//            decimal_part = decimal_part + ( new_num / d );
//            d = d * 10.0;
//        }
//        
//        ss>>c;
//    }
//    
//    return ( (double)integer_part + decimal_part );
//}


void printWM (WM_node *node){
    for(size_t i=0;i<node->son.size();i++)
    {
        std::cout<<node->name<<"->"
                <<(node->son[i])->instance<<", "
                <<(node->son[i])->rtm<<", "
                <<WM->getInstanceEmphasis(node->son[i]->instance);
        //copia esatta della funzione
        size_t j = 0;
        bool var;
        char c;
        bool isTrue = true;
        std::string app;
        //mentre il vettore non è finito ed il releaser è vero
        while (j < node->son[i]->releaser.size() ){ //&& isTrue) {
            isTrue = node->son[i]->eval(node->son[i]->releaser[j]);
            /*
            var = WMV.get<bool>(node->son[i]->releaser[j]);
            //se l'iesimo elemento non è negato
            if (node->son[i]->releaser[j][0] != '-' &&
                    //ed e falso nella memoria allora il releaser è falso
                    !var) isTrue = false;
                //altrimenti se è negato
            else if (node->son[i]->releaser[j][0] == '-') {
                //apri uno stream
                std::stringstream ss(node->son[i]->releaser[j]);
                //scarta il not (ie. il simbolo -)
                ss >> c>>app;
                var = WMV.get<bool>(app);
                //se nella memoria è vero allora il releaser è falso
                if (var) isTrue = false;
            }
            */

            if (isTrue)
                std::cout << "\033[0;32m "<<node->son[i]->releaser[j]<<" \033[0m";
            else
                std::cout << "\033[0;31m ~"<<node->son[i]->releaser[j]<<" \033[0m";

            j++;
        }

        if(node->son[i]->goalStatus())
            std::cout<<" accomplished\n";
        else
            std::cout<<"\n";
        printWM(node->son[i]);
    }
}




/**
 *
 * @param schemaInstance
 * @return vettore di stringhe rappresentanti i parametri dello schema
 * in versione prolog-like:
 * EG.
 *      vec[0]="nome schema"
 *      vec[1]="primo parametro"
 *      vec[2]="secondo parametro"
 *      etc.
 *
 * eventuali parametri che siano essi stessi funtori vengono
 * restituiti ugualmente come elemento del vettore
 * EG.
 *      vec[i]="fun1(fun2(x,y),z)"
 *
 * NOTE: This version also consider the [ ] as a list!
 */
std::vector<std::string> instance2vector(std::string schemaInstance){
    bool isAtom=true, isString=false;
    char c;
    std::string app;
    std::vector<std::string> result;
    std::stringstream ss(schemaInstance);
    int count=0;
    ss >> std::noskipws;
    //leggi il primo carattere della stringa
    ss>>c;
    //mentre non sei a fine stringa
    while(!ss.eof())
    {
        //se il carattere è un doppio apice e non sono in una stringa
        if(c=='"' && !isString){
            //allora sono in una stringa
            isString=true;
            //aggiungo l'apice
            app=app+c;
        }
        //se il carattere è un doppio apice e sono in una stringa
        else if(c=='"' && isString){
            //la stringa è finita
            isString=false;
            //aggiungo l'apice
            app=app+c;
            //aggiungila come elemento del funtore
            //result.push_back(app);
        }
        //mentre sono in una stringa
        else if(isString){
            //aggiungi il carattere senza controllarlo
            app=app+c;
        }
        //se sono un atomo ed il carattere letto è una parentesi aperta
        else if(c=='(' && isAtom){
            //non sono più un atomo
            isAtom=false;
            //inserisco il nome come primo elemento del vettore
            result.push_back(app);
            //pulisco la stringa d'appoggio
            app="";
            //salto la parentesi
//            ss>>c;
        }
        else if(c=='(' || c=='['){
            count++;
            app=app+c;
        }
        else if( ( c==')' || c==']' ) && count!=0){
            count--;
            app=app+c;
        }
        //se il carattere letto non è una virgola
        else if(c!=',' || count!=0)
            //aggiungilo alla stringa d'appoggio
            app=app+c;
        //altrimenti (ie. il carattere è una virgola)
        else {
            //inserisci la stringa d'appoggio nel vettore risultato
            result.push_back(app);
            //pulisci la stringa d'appoggio
            app="";
            //ho saltato la virgola
        }
        //leggi il successivo carattere
        ss>>c;
    }
    //se lo schema non ha parametri aggiungi il solo nome (vec[0])
    if(isAtom) {
        //check the \ character and split by it (added 01/12/2020 in seed 4.0)
        if( app.find('\\') != std::string::npos ){
            std::stringstream ss2(app);
            std::string substr;
            //std::cout<<"INSTANCE TO VECTOR: "<<schemaInstance<<std::endl;
            while(std::getline(ss2, substr, '\\')){
                result.push_back(substr);
                //std::cout<<"split: "<<substr<<std::endl;
            }
        }
        else
            result.push_back(app);
    }
    //altrimenti aggiungi l'ultima stringa rimuovendo l'ultima parentesi
    else{
        app.erase(app.size()-1);
        result.push_back(app);
    }
    //ritorna il vettore calcolato
    return result;
}





/**
 *
 * @param node
 * rimuove il sottoalbero radicato in node dalla Working Memory
 * se node è nullo la funzione non ha effetto
 */
void remove(WM_node *node){
    size_t i = 0;
    //esci se il nodo passato è null
    if(node==NULL) return;

    //per ogni figlio
    //for(i=0;i<node->son.size();i++){
    while(node->son.size() != 0){
        //deallocalo
        remove(node->son[i]);
    }
    //se ho un padre, cancella la mia reference da esso
    if(node->father!=NULL){

        for(i=0;i<node->father->son.size();i++)
            if(node->father->son[i]==node){
                node->father->son.erase(node->father->son.begin()+i);
            }
        
        if(node->expanded)
            node->saveWeights();
        
        //dealloca me stesso
        //std::cout<<node->instance<<" removed!\n";
        delete node;
    }
    //se non ho un padre... quindi sono la radice (alive)
    else{
        node->instance="zombie";
    }

}

/**
 *
 * @param vec
 *  insertion sort di std::vec<WM_node *>,
 *      l'ordine è decrescente (3 2 1 0)
 * @return sorted vec
 */
std::vector<WM_node *> sortNodes(std::vector<WM_node *> vec){

    std::vector<WM_node *> sorted;

    //controlla che l'elemento non sia già presente
    for(size_t i=0; i<vec.size(); i++){
        bool present=false;
        for(size_t j=0; j<sorted.size(); j++)
            if(vec[i]->instance == sorted[j]->instance)
                present=true;

        if(!present)
            sorted.push_back(vec[i]);
    }


    for (size_t i = 0; i < sorted.size(); i++) {
        WM_node *swap=sorted[i];
    	double magValue = WM->getInstanceEmphasis(sorted[i]->instance);
        //double rtmValue = sorted[i]->rtm;
        int pos = i;
        while (pos > 0 &&
                magValue > WM->getInstanceEmphasis(sorted[pos - 1]->instance) ) {
                //( (  rtmValue!=0 && rtmValue < sorted[pos - 1]->rtm ) || (
                //rtmValue == sorted[pos - 1]->rtm &&
                //    magValue > WM->getInstanceEmphasis(sorted[pos - 1]->instance ) ) ) ) {
            sorted[pos] = sorted[pos - 1];
            pos = pos - 1;
        }

        sorted[pos] = swap;
    }
    return sorted;
}

/**
 * controllo della presenza di alive in WM
 * @return true se alive è stato rimosso, false altrimenti
 *
 * la funzione rilascia il semaforo memMutex, va chiamata come segue:
 *
 *      if(dead()) return;
 *
 * in un qualsiasi ambito protetto.
 */
bool dead(){
    if(WM==NULL){
        std::cout<<"DEAD! unlock\n";
        //*(int*)0 = 0;
        pthread_mutex_unlock(&memMutex);
        return true;
    }
    return false;
}

void *execution(void *arg){
    //recast del parametro passato al thread
    Behavior *behavior=(Behavior*)arg;

    //inizializzazione del behavior
    behavior->start();
    //mentre il behavior è presente nella WM
    while(behavior->perceptWM())
    {
        //se il suo releaser è attivo
        if (behavior->perceptualSchema() && behavior->getReleaser() && !behavior->isFrozen()) {

            pthread_mutex_lock(&memMutex);
            behavior->setNodeInternalReleaser(true);
            pthread_mutex_unlock(&memMutex);

            //avvia il comportamento
            behavior->motorSchema();

            //se non è stato fatto l'update manuale del ritmo
            if (!behavior->isUpdated())
                //settalo al valore di defaut (1/magnitude)
                behavior->updateRtm();
        }
        //se il releaser non è attivo setta il ritmo a quiescenza
        else {
            pthread_mutex_lock(&memMutex);
            //metti il ritmo a quiescenza, ie. perdi tutte le competizioni
            behavior->setRtm(QUIESCENCE);
            behavior->setNodeInternalReleaser(false);
            pthread_mutex_unlock(&memMutex);
        }
        
        // REMOVED SINCE SEED 7.1
        //attendi in base al ritmo
        //unsigned int usec=(int) (behavior->getRtm()*MSECOND);
        //unsigned int usec=(int) (0.01*MSECOND);

        unsigned int usec=(int) ( ( 1 / behavior->getRate() ) * MSECOND );

        //durante la sleep si rilascia la memoria di lavoro
        usleep(usec);
    }
    //finalizzazione del behavior
    behavior->exit();
    
    //NOTE: this generates a warning, a virtual desctructor should be defined
    delete behavior;
    
    return NULL;
}

//spinning thread
void *ros2_th(void *arg){
    // Run the executor.
    (void) arg; //suppress unused parameter warning
    rclcpp::spin(nh);
    std::cout<<"ROS2 spinner closed"<<std::endl;

    return NULL;
}

} //seed namespace

/*
 *
 */
int main(int argc, char** argv) {

    //std::cout<<"SEED ARGS: ";
    //for (int i = 0; i < argc; ++i)
    //    std::cout << argv[i] << "|";
    //std::cout<<std::endl;

    seed::SEED_NAME = argc>1 ? "seed_" + std::string(argv[1]) : "seed";
    std::cout<<"SEED, starting node "<<seed::SEED_NAME<<std::endl;

    rclcpp::init(argc,argv);
    seed::nh = rclcpp::Node::make_shared("seed");

    //seed::SEED_HOME_PATH = ament_index_cpp::get_package_share_directory("seed"); // this returns: ".../ros2_ws/install/seed/share/seed"
    seed::SEED_HOME_PATH = ament_index_cpp::get_package_share_directory("seed") + "/../../../../src/seed";
    std::cout << seed::SEED_HOME_PATH << std::endl;

    seed::SYS_HOME_PATH = getenv("HOME");
    std::cout << seed::SYS_HOME_PATH << std::endl;

    //CLEAR LOG COMPETITION
    std::ofstream f;
    f.open(seed::SEED_HOME_PATH + "/log/" + seed::SEED_NAME + "_log.txt",std::ios::trunc);
    f <<"Start log\n";
    f.close();

    seed::seed_global_clk.tic();
    
    //seed::LTM = new seed::LongTermMemory_eclipseclp(argc,argv);
    seed::LTM = new seed::LongTermMemory_swipl(argc,argv);
    
    if( !seed::LTM->loadLTM(seed::SEED_HOME_PATH + "/LTM/" + seed::SEED_NAME + "_LTM.prolog") ){
        std::cout<<"SEED-ERROR: unable to find a LTM for the name "<<seed::SEED_NAME<<std::endl;
        return 1;
    }

    signal(SIGSEGV,seed::faultHandler);
    
    pthread_mutex_lock(&seed::memMutex);
    seed::WMV.set<bool>("seed", "TRUE",true);
    seed::WMV.set<bool>("seed", "FALSE",false);
    
    seed::WMV.set<bool>("seed", "joyStream",false); //this disables learning until joy is on
    pthread_mutex_unlock(&seed::memMutex);
    seed::AliveBehavior *Alive=new seed::AliveBehavior("alive");
    //initialize memoryStream
    pthread_t thread_alive;
    //start the alive thread
    pthread_create(&thread_alive, NULL,seed::execution, (void *)Alive);

    //start spinning thread
    pthread_t thread_ros2;
    pthread_detach(pthread_create(&thread_ros2, NULL, seed::ros2_th, NULL));

    //join the alive thread
    pthread_join(thread_alive, NULL);
    
    //join all threads
    for(size_t i=0; i<seed::seed_thread_list.size(); i++)
        pthread_join(seed::seed_thread_list[i], NULL);

    bool ros_exit = rclcpp::shutdown();
    std::cout<<"SYSTEM: ROS2 exited, condition: "<<ros_exit<<std::endl;
    
    seed::LTM->close();
    //NOTE: here a virtual destroyer should be defined
    delete seed::LTM;
    
    std::cout<<"SYSTEM: shutdown complete, bye"<<std::endl;

    return 0;
}
