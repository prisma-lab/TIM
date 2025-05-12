/*
 * File:   seed_header.h
 * Author: hargalaten
 *
 * Created on 12 dicembre 2012, 15.45
 */

#include "seed.h"

namespace seed{


// ********************* WM_node functions ********************* //


WM_node::WM_node(std::string newInstance, WM_node* instanceFather) {
    name = instance2vector(newInstance)[0];
    instance = newInstance;
    father = instanceFather;
    abstract = true;
    expanded = false;
    goalCount = 0;
    teleological = false;
    amplified = false;
    freezed = false;
    internalReleaser = true;
    background = false;
    
    is_sequential = false;
    in_sequence = false;
    sequentialReleaser = false;
}

void WM_node::loadWeights(){
    //std::ifstream file("/home/phoenix/catkin_ws/src/seed_learn/src/behavior/learn/learning/weights");
    //std::string file_path(SEED_HOME_PATH + "/src/behavior/learn/learning/weights");
    std::string file_path(SEED_HOME_PATH + "/learning/weights");
    std::ifstream file(file_path.c_str());
    std::string line;
    bool found = false;
    
    if(!file.is_open()){
        std::cout<<ansi::yellow<<"SYSTEM: cant load weights of"<< this->instance <<" from file "<<file_path<<ansi::end<<std::endl;
    }

    std::unordered_map<std::string, double*> weights = WMV.get< std::unordered_map<std::string, double*> >(this->name + ".weights");

    if (weights.size() == 0) {
        while (std::getline(file, line) && !found) {
            std::vector<std::string> linevec = instance2vector(line);
            if (linevec[1] == this->name) {
                //std::cout<<ansi::cyan<<"SYSTEM: load weights for "<<this->instance<< ": "<<line<<ansi::end<<std::endl;
                for (size_t i = 2; i < linevec.size(); i++) {
                    //weights[instance2vector(linevec[i])[1]] = new double( string_to_double(instance2vector(linevec[i])[2]) );
                    weights[instance2vector(linevec[i])[1]] = new double( ston(instance2vector(linevec[i])[2]) );
                    //std::cout<<ansi::cyan<<"\t"<<instance2vector(linevec[i])[1]<< ": "<<string_to_double(instance2vector(linevec[i])[2])<<" ("<< instance2vector(linevec[i])[2] <<")"<<ansi::end<<std::endl;
                }
                found = true;
            }
        }
        WMV.set< std::unordered_map<std::string, double*> >(this->instance, this->name + ".weights", weights);
    }
}

void WM_node::saveWeights(){
    //std::string infile_path(SEED_HOME_PATH + "/src/behavior/learn/learning/weights");
    //std::string outfile_path(SEED_HOME_PATH + "/src/behavior/learn/learning/_weights");
    std::string infile_path(SEED_HOME_PATH + "/learning/weights");
    std::string outfile_path(SEED_HOME_PATH + "/learning/_weights");
    std::ifstream infile;
    std::ofstream outfile;
    std::string line;
    int l=0;


    std::unordered_map<std::string, double*> weights = WMV.get< std::unordered_map<std::string, double*> >(this->name + ".weights");
//        std::cout<<"saveing "<<this->instance<<"\n";

    if (weights.size() != 0) {

        infile.open(infile_path.c_str());
        outfile.open(outfile_path.c_str());
        
        if(!infile.is_open()){
            std::cout<<ansi::yellow<<"SYSTEM: cant save weights of "<< this->instance <<" from file "<<infile_path<<ansi::end<<std::endl;
        }
        if(!outfile.is_open()){
            std::cout<<ansi::yellow<<"SYSTEM: cant save weights of "<< this->instance <<" on file "<<outfile_path<<ansi::end<<std::endl;
        }
        
        //std::cout<<ansi::cyan<<"SEED, save weights for "<<this->instance<<ansi::end<<std::endl;

        bool found = false;
        //per ogni riga del file dei pesi
        while (std::getline(infile,line)) {
            std::vector<std::string> linevec = instance2vector(line);
//                std::cout<<"line: "<<line<<"\n";
//                std::cout<<"linevec1: "<<linevec[1]<<"\n";

            if(l!=0)
                outfile<<"\n";

            //se la riga è quella dei miei pesi
            if (linevec[1] == this->name) {

                //aggiungo la nuova riga di pesi
                outfile << "w(" << this->name;
                for (std::unordered_map<std::string, double *>::iterator it = weights.begin(); it != weights.end(); ++it) {
                    outfile << ",(" << it->first << "," << *(it->second) << ")";
                    //std::cout<<ansi::cyan<<"\t"<<it->first << ": " << *(it->second)<<", found"<<ansi::end<<std::endl;
                }
                outfile << ")";
                //ho trovato la mia riga
                found = true;
            }                    //altrimenti
            else
                //lascia la vecchia riga
                outfile << line;

            l++;
        }
        //aggiungila come ultima riga se non la ho trovata in precedenza
        if (!found) {

            if(l!=0)
                outfile<<"\n";

            outfile << "w(" << this->name;
            for (std::unordered_map<std::string, double *>::iterator it = weights.begin(); it != weights.end(); ++it) {
                outfile << ",(" << it->first << "," << *(it->second) << ")";
                //std::cout<<ansi::cyan<<"\t"<<it->first << ": " << *(it->second)<<", NOT found"<<ansi::end<<std::endl;
            }
            outfile << ")";
        }
        outfile.close();
        infile.close();

        std::remove(infile_path.c_str());
        std::rename(outfile_path.c_str(), infile_path.c_str());

    }
}

/** funzione che trasforma un sottoalbero in una lista di nodi
 *
 * @return la lista di tutti i nodi nel sottoalbero radicato
 */
void WM_node::setBackground(bool value) {

    if(this->background != value) {
        this->background = value;
        for (size_t i = 0; i<this->son.size(); i++) {
            this->son[i]->setBackground(value);
        }
    }
}

/** funzione che trasforma un sottoalbero in una lista di nodi
 *
 * @return la lista di tutti i nodi nel sottoalbero radicato
 */
std::vector<WM_node *> WM_node::tree2list() {
    std::vector<WM_node *> nodeList;
    nodeList.push_back(this);
    for (size_t i = 0; i<this->son.size(); i++) {
        std::vector<WM_node *> subList = this->son[i]->tree2list();
        nodeList.insert(nodeList.end(), subList.begin(), subList.end());
    }

    return nodeList;
}
/** funzione che trasforma un sottoalbero in una lista di nodi
 *
 * @return la lista di tutti i nodi concreti nel sottoalbero radicato
 */
std::vector<WM_node *> WM_node::tree2list_concrete() {
    std::vector<WM_node *> nodeList;

    if (!this->expanded ) //|| !this->releaserStatus() || this->goalStatus())
        return nodeList;

    //std::cout<<"check: "<<this->abstract<<", "<<this->instance<<"\n";
    if( !this->abstract && this->instance != "alive"){
        //std::cout<<"    concrete: "<<this->abstract<<", "<<this->instance<<"\n";
        nodeList.push_back(this);
    }
    for (size_t i = 0; i<this->son.size(); i++) {
        std::vector<WM_node *> subList = this->son[i]->tree2list_concrete();
        nodeList.insert(nodeList.end(), subList.begin(), subList.end());
    }

    return nodeList;
}

/** funzione che trasforma un sottoalbero in una lista di nodi ATTIVI
 *      (ie. rilasciati e non completati)
 *
 * @return la lista di tutti i nodi ATTIVI nel sottoalbero radicato
 *
 *  NOTA: se la radice si trova in un sottoalbero NON attivo, la funzione
 *      restituisce ugualmente la lista dei sottonodi ATTIVI a partire
 *      dalla attuale radice. Per evitare questo si consiglia di utilizzare
 *      isBranchReleased() sulla radice prima di chiamare questa funzione.
 */
std::vector<WM_node *> WM_node::activetree2list() {
    std::vector<WM_node *> nodeList;
    if (this->releaserStatus() && !this->goalStatus())
        nodeList.push_back(this);
    for (size_t i = 0; i<this->son.size(); i++) {
        std::vector<WM_node *> subList = this->son[i]->tree2list();
        nodeList.insert(nodeList.end(), subList.begin(), subList.end());
    }

    return nodeList;
}

/** funzione di computo dell'enfasi per l'istanza
 *
 * @param instanceToFind l'istanza del behavior di cui calcolare la
 *  magnitudine
 * @return l'enfasi totale di tutti i behavior attivi (rilasciati)
 *  aventi come istanza "instanceToFind"
 */
double WM_node::getInstanceEmphasis(std::string instanceToFind) {
    double totalEmphasis = 0.0;
    //se sono rilasciato allora ( NB il controllo sul goal è stato aggiunto dopo)
    if (this->releaserStatus() && !this->goalStatus()) {
        //se io sono il nodo cercato
        if (this->instance == instanceToFind)
            //salva la mia magnitudine
            totalEmphasis += this->emphasis();

        //totalMagnitude+=instance2vector(this->instance)[2];

        //controlla se tra i miei figli vi sono istanze
        for (size_t i = 0; i < son.size(); i++)
            //somma la magnitudine delle istanze nel sottoalbero
            totalEmphasis += son[i]->getInstanceEmphasis(instanceToFind);
    }
    //ritorna il totale
    return totalEmphasis;
}
//funzione che calcola l'enfasi del singolo nodo (somma pesata di tutti i contributi)
double WM_node::emphasis(bool check) {
    double finalEmph = 0.0;

    if(check && !this->isBranchReleased())
        return 0.0;

    std::unordered_map<std::string, double*> weights = WMV.get< std::unordered_map<std::string, double*> >(this->name + ".weights");
    //std::cout<<this->instance<<" got weights"<<std::endl;
    //per ogni contributo dato al nodo
    for (std::unordered_map<std::string, double *>::iterator it = this->contribution.begin(); it != this->contribution.end(); ++it) {

        //controlla se l'elemento esista (ad es. è stato cercato nella hashmap) ma il contributo no
        if(it->second == NULL)
            continue;

        //prendi il peso associato allo schema che contribuisce
        double *w = weights[instance2vector(it->first)[0]];
        //se il peso non esiste (ie. è la prima volta che questo schema contribuisce)
        if(w == NULL){
            //aggiungilo con un valore di default
            w = new double;
            *w = SEED_DEFAULT_WEIGHT;
            weights[instance2vector(it->first)[0]] = w;
        }

        //aggiungi il contributo pesato
        finalEmph += (*w) * (*(it->second));
    }
    //AGGIUNTA HIERARCHICAL LEARNING
    //se mio padre è astratto allora devo considerare ricorsivamente il suo contributo
    if(this->instance != "alive" && this->father->abstract){
        //prendi il peso associato allo schema padre
        double *w = weights[this->father->name];
        //se il peso non esiste (ie. è la prima volta che il padre contribuisce)
        if(w == NULL){
            //aggiungilo con un valore di default
            w = new double;
            *w = SEED_DEFAULT_WEIGHT;
            weights[this->father->name] = w;
        }

        //aggiungi il contributo pesato
        finalEmph += (*w) * this->father->emphasis(false);
    }
    //FINE AGGIUNTA

//    //plot
//    std::cout<<ansi::cyan<<std::endl;
//    for(auto it = weights.begin(); it != weights.end(); ++it) {
//        std::cout <<"\t"<< it->first << " = "<< *(it->second) << '\n'; 
//    }
//    std::cout<<ansi::end<<std::endl<<std::endl;

    WMV.set< std::unordered_map<std::string, double*> >(this->instance, this->name + ".weights", weights);
    return finalEmph;
}

//ADDED on SEED 8.0
/** function evaluating if an atomic formula is satisfied (for releaser and goal checking).
    *
    * @param formula the formula to be evaluated 
    *
    * @return TRUE if the formula is satisfied
    */
/*
// DOUBLE VERSION!
bool WM_node::eval(std::string formula){
    double var;
    char neg;
    bool satisfied = true;
    std::string app;

    var = WMV.get<double>(formula);
    //if formula is not a negation but it is false in WM
    if (formula[0] != '-' && var == 0)
        //then the formula is not satisfied
        satisfied = false;
    //otherwise, if formula is a negation
    else if (formula[0] == '-') {
        //discard the negation symbol (-) from it
        std::stringstream ss(formula);
        ss >> neg>>app;
        var = WMV.get<double>(app);
        //if it is true in the WM
        if (var == 1)
            //then the formula is not satisfied
            satisfied = false;
    }
    return satisfied;
}
*/
#if SEED_USE_EVAL_SIMPLE
#pragma message("SEED: SIMPLE eval function enabled")
// BOOL VERSION 
bool WM_node::eval(std::string formula){
    bool var;
    char neg;
    bool satisfied = true;
    std::string app;

    var = WMV.get<bool>(formula);
    //if formula is not a negation but it is false in WM
    if (formula[0] != '-' && !var)
        //then the formula is not satisfied
        satisfied = false;
    //otherwise, if formula is a negation
    else if (formula[0] == '-') {
        //discard the negation symbol (-) from it
        std::stringstream ss(formula);
        ss >> neg>>app;
        var = WMV.get<bool>(app);
        //if it is true in the WM
        if (var)
            //then the formula is not satisfied
            satisfied = false;
    }
    return satisfied;
}
#else
#pragma message("SEED: EXTENDED eval function enabled")
// BOOL and EXTENDED VERSION 
//  NOTE: with respect to the BOOL-simple this function allows usage of
//      mathematical relations (==,>=,<=,<,>) between variables and constants,
//      on the other hand, it seems 5-10 times slower! 
bool WM_node::eval(std::string formula){
    bool negation = false;
    char n_sym;
    bool satisfied = true;
    std::string clear_formula, l_term, r_term;
    double l_val, r_val;
    std::stringstream ss;
    size_t pos = 0;

    //if the first char of the formula is a not (-)
    if(formula[0] == '-'){
        //the formula is a negation
        negation = true;
        //remove the negation symbol from the formula
        ss.str(formula);
        ss >> n_sym >> clear_formula;
    }
    //oth.
    else{
        //the formula is not changed
        clear_formula = formula;
    }

    //check for possible relations
    //  NOTE: only simple numerical relations are considered!
    if( (pos = clear_formula.find("==")) != std::string::npos ){
        //split the formula into the 2 terms
        l_term = clear_formula.substr(0,pos);
        r_term = clear_formula.substr(pos+2,clear_formula.length());

        //if( isdigit(l_term[0]) )
        //    l_val = ston(l_term);
        //else
        //    l_val = WMV.get<double>(l_term);
        //
        //if( isdigit(r_term[0]) )
        //    r_val = ston(r_term);
        //else
        //    r_val = WMV.get<double>(r_term);

        //get the left value (either it is constant or variable)
        l_val = ( (isdigit(l_term[0]) || l_term[0] == '-') ? ston(l_term) : WMV.get<double>(l_term) );
        //get the right value (either it is constant or variable)
        r_val = ( (isdigit(r_term[0]) || r_term[0] == '-') ? ston(r_term) : WMV.get<double>(r_term) );
        //return the result of their relation (XOR with negation var)
        satisfied = ( negation != (l_val == r_val) );

    }
    else if( (pos = clear_formula.find(">=")) != std::string::npos ){
        //split the formula into the 2 terms
        l_term = clear_formula.substr(0,pos);
        r_term = clear_formula.substr(pos+2,clear_formula.length());
        //std::cout<<"is a <= between: ["<<l_term<<"] and ["<<r_term<<"]"<<std::endl;
        //get the left value (either it is constant or variable)
        l_val = ( (isdigit(l_term[0]) || l_term[0] == '-') ? ston(l_term) : WMV.get<double>(l_term) );
        //get the right value (either it is constant or variable)
        r_val = ( (isdigit(r_term[0]) || r_term[0] == '-') ? ston(r_term) : WMV.get<double>(r_term) );
        //return the result of their relation (XOR with negation var)
        satisfied = ( negation != (l_val >= r_val) );
        //std::cout<<"values are: ["<<l_val<<"] and ["<<r_val<<"], result is: "<<satisfied<<std::endl;
    }
    else if( (pos = clear_formula.find("<=")) != std::string::npos ){
        //split the formula into the 2 terms
        l_term = clear_formula.substr(0,pos);
        r_term = clear_formula.substr(pos+2,clear_formula.length());

        //get the left value (either it is constant or variable)
        l_val = ( (isdigit(l_term[0]) || l_term[0] == '-') ? ston(l_term) : WMV.get<double>(l_term) );
        //get the right value (either it is constant or variable)
        r_val = ( (isdigit(r_term[0]) || r_term[0] == '-') ? ston(r_term) : WMV.get<double>(r_term) );
        //return the result of their relation (XOR with negation var)
        satisfied = ( negation != (l_val <= r_val) );
    }
    else if( (pos = clear_formula.find(">")) != std::string::npos ){
        //split the formula into the 2 terms
        l_term = clear_formula.substr(0,pos);
        r_term = clear_formula.substr(pos+1,clear_formula.length());

        //get the left value (either it is constant or variable)
        l_val = ( (isdigit(l_term[0]) || l_term[0] == '-') ? ston(l_term) : WMV.get<double>(l_term) );
        //get the right value (either it is constant or variable)
        r_val = ( (isdigit(r_term[0]) || r_term[0] == '-') ? ston(r_term) : WMV.get<double>(r_term) );
        //return the result of their relation (XOR with negation var)
        satisfied = ( negation != (l_val > r_val) );
    }
    else if( (pos = clear_formula.find("<")) != std::string::npos ){
        //split the formula into the 2 terms
        l_term = clear_formula.substr(0,pos);
        r_term = clear_formula.substr(pos+1,clear_formula.length());

        //get the left value (either it is constant or variable)
        l_val = ( (isdigit(l_term[0]) || l_term[0] == '-') ? ston(l_term) : WMV.get<double>(l_term) );
        //get the right value (either it is constant or variable)
        r_val = ( (isdigit(r_term[0]) || r_term[0] == '-') ? ston(r_term) : WMV.get<double>(r_term) );
        //return the result of their relation (XOR with negation var)
        satisfied = ( negation != (l_val < r_val) );
    }
    else{
        //formula is not a relation, return the result (XOR with negation var)
        satisfied = ( negation != WMV.get<bool>(clear_formula) );
    }

    return satisfied;
}
#endif

//MODIFIED on SEED 8.0 to use "eval" function
/** funzione di controllo del goal
 *
 * @return TRUE se il goal del nodo è vero
 */
bool WM_node::goalStatus() {
    int i = 0;
    bool isTrue = true;
    //std::cout<<this->instance<<" GOAL:"<<std::endl;
    //if the node is theleological (has no goal), the goal is always false
    //  (the node is never accomplished)
    if (!this->teleological) return false;
    //mentre il vettore non è finito ed il goal è vero
    //while the goal is true
    while ( (size_t)i < goal.size() && isTrue ) {
        //check if the element of the goal is true
        isTrue = eval(goal[i]);
        i++;
        //std::cout<<": "<<isTrue<<std::endl;
    }
    //std::cout<<"\t\t RESULT: "<<isTrue<<std::endl;
    //return the status of the goal
    return isTrue;
}

//MODIFIED on SEED 8.0 to use "eval" function
/** funzione di controllo del releaser per il singolo nodo
 *
 * @return TRUE se il releaser del nodo è vero
 */
bool WM_node::releaserStatus() {
    int i = 0;
    bool isTrue = true;
    
    //sequence checking (added 5/6/2020 in seed 3.0)
    //if the node is inside a sequence bot it is not his turn
    if(in_sequence && !sequentialReleaser)
        //the node is not released
        return false;
    
    //while the releaser is true
    while ( (size_t)i < releaser.size() && isTrue ) {
        //check the element of the releaser
        isTrue = eval(releaser[i]);
        i++;
    }
    //return the status of the releaser
    return isTrue;
}

/** funzione per il controllo dei releaser sul ramo
 *
 * @return TRUE se il ramo del nodo ha releaser veri
 *
 * si combina con releaserStatus() e goalStatus() per calcolare il valore dei
 * releaser e dei goal per ogni nodo del ramo di cui quello attuale è
 * foglia
 */
bool WM_node::isBranchReleased() {
    //se il mio releaser è vero ed ho un padre ritorna il suo releaser
    if (this->releaserStatus() && !this->goalStatus() && father != NULL) return father->isBranchReleased();
        //altrimenti se sono rilasciato ma non ho un padre ritorna TRUE
    else if (this->releaserStatus() && !this->goalStatus() && father == NULL) return true;
    //altrimenti non sono rilasciato
    return false;
}

/**
 * Funzione di controllo del releaser su tutta la memoria
 *
 * @param toFind istanza da cercare
 * @return true se esiste almeno una istanza con releaser vero,
 *          false altrimenti
 */
bool WM_node::isReleased(std::string toFind) {
    //se io sono il nodo cercato ritorno me stesso
    if (this->instance == toFind && this->isBranchReleased()) return true;
        //altrimenti chiedi ai figli il nodo da cercare
    else for (size_t i = 0; i < son.size(); i++)
            if (son[i]->isReleased(toFind))
                return true;
    //altrimenti il nodo non c'è
    return false;
}


/**
 * Set contribution for a specific node
 * 
 * NOTE: this contribution will be implicitly inherited by the hierarchy
 *
 */
void WM_node::setContribution(std::string contribution_name, double contribution_value) {
    if(this->contribution[contribution_name] == NULL)
        this->contribution[contribution_name] = new double(contribution_value);
    else
        *(this->contribution[contribution_name]) = contribution_value;
}

/**
 * Get contribution for a specific node
 * 
 * NOTE: if no contribution is available, the function returns NULL
 *
 */
double * WM_node::getContribution(std::string contribution_name) {
    return this->contribution[contribution_name];
}

//REMOVED on SEED 7.2
// /**
//  * aggiorna uno specifico contributo per tutto il sottoalbero
//  *  NOTE: il contributo deve essere presente nella lista dei contributi dei nodi
//  *
//  */
// void WM_node::updateContribution(std::string contribution_name, double contribution_value) {
//     if(this->contribution[contribution_name] != NULL){
//         *(this->contribution[contribution_name]) = contribution_value;

//         //std::cout<< "update "<< contribution_name << " -> " << this->instance << " by " << contribution_value << "(" << *(this->contribution[contribution_name]) << std::endl;;
//     }

//     for (size_t i = 0; i < (this->son).size(); i++)
//         (this->son[i])->updateContribution(contribution_name, contribution_value);
// }

// /**
//  * amplifica il sottoalbero, maggiorando l'importanza dei nodi con la
//  * propria magnitudine
//  * 
//  * NOTE (SEED 7.2): this looks very wrong, this is a cascade effect where sub nodes get twice the emphasis 
//  *      ...it should be checked
//  */
// void WM_node::amplify(std::string contributingSchema, double factor) {
//     if(this->contribution[contributingSchema] == NULL)
//         this->contribution[contributingSchema] = new double(factor);
//     else
//         *(this->contribution[contributingSchema]) += factor;

//     std::cout<< "amplify "<< contributingSchema << " -> " << this->instance << " by " << factor << "(" << *(this->contribution[contributingSchema]) << ")\n";

//     for (size_t i = 0; i < (this->son).size(); i++)
//         (this->son[i])->amplify(contributingSchema, factor);
// }

// /**
//  * amplifica il sottoalbero, maggiorando l'importanza dei nodi con la
//  * propria magnitudine
//  *
//  */
// void WM_node::amplify_hierarchical(std::string schemaInstance, double factor) {
//     if(this->contribution[schemaInstance] == NULL)
//         this->contribution[schemaInstance] = new double(factor);
//     else
//         *(this->contribution[schemaInstance]) += factor;

//     std::cout<< "amplify "<< schemaInstance << " -> " << this->instance << " by " << factor << "(" << *(this->contribution[schemaInstance]) << ")\n";

//     //for (int i = 0; i < (this->son).size(); i++)
//     //    (this->son[i])->amplify_hierarchical( this->instance, 0 );
// }

/**
 * cerca ed amplifica tutti i nodi che hanno appena raggiunto il goal
 */
void WM_node::amplifyNodes() {

    //controlla i figli
    for (size_t i = 0; i < son.size(); i++) {
        son[i]->amplifyNodes();
    }

    //se il nodo è teleologico
    if (this->teleological) {
        //se il goal è vero ma il nodo non è stato amolificato
        if (this->goalStatus() && !this->amplified) {
            std::cout<< "\033[0;33m "<<this->instance<<" success! \033[0m \n";
            //se è la prima volta che raggiunge il goal
            if (this->goalCount == 0) {
                //consideralo amplificato
                this->amplified = true;
                this->goalCount++;
                //amplifica il sottoalbero in esso radicato
                //(this->father)->amplify("teleology", TELEOLOGY_EMP);
                //(this->father)->amplify_hierarchical("teleology", TELEOLOGY_EMP);
                (this->father)->setContribution("teleology", TELEOLOGY_EMP);
            }//altrimenti (ie. è stato amplificato altre volte)
            else {
                //consideralo amplificato
                this->amplified = true;
                this->goalCount++;
            }
        }//altrimenti se il goal è falso ma il nodo è amplificato
        else if (!this->goalStatus() && this->amplified){
            //segnalalo come non amplificato
            this->amplified = false;

            //(this->father)->amplify("teleology", -TELEOLOGY_EMP);
        }
    }

}

/** function that searches and updates all sequences of the sub-tree, 
 *  if the current executing node of the sequence is accomplished, the
 *  next node is enabled.
 * 
 *      added 5/6/2020 in seed 3.0
 * 
 */
void WM_node::updateSequentialNodes(){
    
    //NOTE: the sequences are checked even if the releasers are false, this
    //      means that disabled sequences are still monitored by the system!
    
    //if I'm a sequential node
    if(is_sequential){
        //update the sequence of sons
        for (size_t i = 0; i < son.size()-1; i++) {
            //if son is the currently executing node, and it is accomplished
            if(son[i]->in_sequence && son[i]->sequentialReleaser && son[i]->goalStatus()){
                //this node is disabled (it will be never enabled again)
                son[i]->sequentialReleaser = false;
                //the next son is the currently executing node
                son[i+1]->sequentialReleaser = true;
                
                std::cout<<"SEQ "<<this->instance<<" updated: "<<son[i]->instance<<" -> "<<son[i+1]->instance<<std::endl;
                break;
            }
        }
    }
    //update the rest of the tree
    for (size_t i = 0; i < son.size(); i++) {
        son[i]->updateSequentialNodes();
    } 
}

/**
 * Scorre l'intera WM e controlla lo stato del fading.
 * Ogni nodo il cui fading è nullo viene rimosso dalla WM
 */
void WM_node::forgetNodes() {
    if (this->fading <= 0) {
        //remove(this);
    } else {
        for (size_t i = 0; i < son.size(); i++) {
            son[i]->forgetNodes();
        }
    }
}
//ritorna il primo nodo espandibile del sottoalbero

WM_node* WM_node::getExpandableNode() {
    int i = 0;
    //se il mio releaser è falso torna NULL
    WM_node *expNode = NULL;
    //bool rel = this->releaserStatus();

    //e non sono espanso torna me stesso
    /*espandimi anche se non sono vero... ma non espandere i miei figli*/
    if (!expanded /*&& rel*/) expNode = this;
        //altrimenti se sono stato espanso controlla i figli
    else if (expanded /*&& rel*/)
        while ( (size_t)i < son.size() && (expNode = son[i]->getExpandableNode()) == NULL) i++;

    return expNode;
}
//allocazione ed aggiunta di un nuovo figlio

WM_node* WM_node::addSon(std::string sonInstance) {
    WM_node *newSon = new WM_node(sonInstance, this);
    son.push_back(newSon);
    return newSon;
}
//cerca e restituisci un nodo qualsiasi, che istanzia "name"

std::vector<WM_node *> WM_node::getNodesByName(std::string name) {
    std::vector< WM_node*> result, sonResult;
    //se io sono il nodo cercato ritorno me stesso
    if (this->name == name) result.push_back(this);
    //altrimenti chiedi ai figli il nodo da cercare
    for (size_t i = 0; i < son.size(); i++) {
        sonResult = son[i]->getNodesByName(name);
        if (sonResult.size() != 0)
            for (size_t j = 0; j < sonResult.size(); j++)
                result.push_back(sonResult[j]);
    }
    //altrimenti il nodo non c'è
    return result;
}

/** cerca e restituisce tutti i nodi identificati dall'istanza
 *
 * @param name istanza da cercare nella memoria
 * @return lista di tutti i nodi aventi istanza "name"
 */
std::vector< WM_node*> WM_node::getNodesByInstance(std::string name) {
    std::vector< WM_node*> result, sonResult;
    //se io sono il nodo cercato ritorno me stesso
    if (this->instance == name) result.push_back(this); //SEGFAULT o shutdown (basic_string)
    //altrimenti chiedi ai figli il nodo da cercare
    for (size_t i = 0; i < son.size(); i++) {
        sonResult = son[i]->getNodesByInstance(name);
        if (sonResult.size() != 0)
            for (size_t j = 0; j < sonResult.size(); j++)
                result.push_back(sonResult[j]);
    }
    //altrimenti il nodo non c'è
    return result;
}

/** seeks and returns the specific node given its ID
 *
 * @param id_to_find: number that identify the node in the WM
 * @return the node associated to the id
 */
WM_node* WM_node::getNodeById(int id_to_find){
    WM_node* result;
    //if I am the node to find
    if (this->id == id_to_find)
        return this;
    //oth. ask the node to my sons
    for (size_t i = 0; i < son.size(); i++) {
        result = son[i]->getNodeById(id_to_find);
    }
    //oth. the node not exists
    return result;
}

/** seeks and returns the specific node following the instances in "path"
 *
 * @param path: vector where the first element is the target node and the
 *              last element is a son of the current node
 * @return the node whose instance is the first element of path
 */
WM_node* WM_node::getNodeByPath(std::vector<std::string> path) {
    size_t j=0;
        
    std::string current = path[path.size()-1];
    path.pop_back();
    
    while(j<this->son.size() && this->son[j]->instance != current)
        j++;
        
        
    if(j == this->son.size()){
        std::cout<<"SEED-WARNING: wrong path, node "<<current<<" is not a son of "<<this->instance<<std::endl;
        return NULL;
    }
    
    if( path.size() == 0 )
        return this->son[j];
    else
        return this->son[j]->getNodeByPath(path);
}

/** returns the path to the current node
 *
 * @return the vector containing all instances on the branch, where the
 *         first element is the current node and the last is a son of 
 *         "alive"
 * 
 *         Note: the function returns NULL if the path is wrong
 */
std::vector<std::string> WM_node::getPath() {
    std::vector<std::string> result, fathResult;
    
    if(this->instance == "alive")
        return result;
    
    result.push_back(this->instance);
    
    fathResult = this->father->getPath();
    
    result.insert(result.end(), fathResult.begin(), fathResult.end());
    
    return result;
}



/** funzione di controllo della presenza di un behavior (concreto) sveglio
 *
 * @param name istanza da ricercare
 * @return TRUE se esiste un behavior "name" già istanziato (sveglio)
 */
bool WM_node::isAwake(std::string name) {
    //se io sono il nodo cercato e sono sveglio ritorna true
    if (this->instance == name && this->expanded && !this->abstract) return true;
    //controlla se tra i miei figli vi sono istanze sveglie
    for (size_t i = 0; i < son.size(); i++)
        if (son[i]->isAwake(name)) return true;
    //altrimenti torna false
    return false;
}

//NO MORE USED after seed_learn inclusion
void WM_node::updateMagnitude() {
    WM_node *f = this->father;
    
    if (f != NULL) {
        //this->magnitude = ((f->magnitude)*(this->ltMagnitude));
        //double ltmContribution = this->ltm_emphasis;
        this->contribution["LTM"] = new double(this->ltm_emphasis);
    }
    for (size_t j = 0; j<this->son.size(); j++)
        this->son[j]->updateMagnitude();
}

void WM_node::tic() {
    this->fading = this->emphasis();
}

void WM_node::ticBranch() {
    this->tic();
    if (this->father != NULL)
        this->father->ticBranch();
}

/**
 * controlla se il sottoalbero ha almeno un nodo che esegue
 * delle operazioni
 *
 * @return true se esiste nel sottoalbero almeno un nodo
 *  concreto con releaser vero
 */
bool WM_node::isWorking() {

    //se ho raggiunto il goal
    if (this->goalStatus()) {
        //allora non sto lavorando
        return false;
    }//se il mio releaser è falso e sono stato espanso
    else if (!this->releaserStatus()) {
        //allora non sto lavorando
        return false;
    }//se non sono stato ancora espanso
    else if (!this->expanded) {
        //            std::cout<<this->instance<<"->isWorking: !expanded\n";
        //allora sto lavorando
        return true;
    }//altrimenti, se sono astratto (ed ho releaser vero)
    else if (this->abstract) {
        //controllo se tra i miei figli c'è un nodo concreto rilasciato
        for (size_t i = 0; i < this->son.size(); i++) {
            //se trovo un nodo concreto rilasciato
            if (this->son[i]->isWorking()) {
                //                    std::cout<<this->instance<<"->isWorking: sonWork\n";
                //allora sto lavorando
                return true;
            }
        }
        //se non trovo nodi concreti rilasciati allora non sto lavorando
        return false;
    }//se non sono in nessuno dei casi precedenti
    else {
        //            std::cout<<this->instance<<"->isWorking: concrete&released\n";
        //allora sono concreto ed attivo, quindi sto lavorando
        return true;
    }
}

} //seed namespace
