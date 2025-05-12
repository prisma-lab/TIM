/*
 * File:   seed_header.h
 * Author: hargalaten
 *
 * Created on 12 dicembre 2012, 15.45
 */

#include "seed.h"

namespace seed{

Behavior::Behavior() {
    oldPeriod = 0;
    oldStimulus = 0;

    rate = SEED_DEFAULT_RATE;
}


//update del ritmo (inteso come periodo di attesa)
//  NOTA: qui il rtm viene aggiornato considerato sia la distanza dall'affordance che l'enfasi del nodo!!
//  il rtm che risulta è sempre nell'intervallo [0.01, 1]
void Behavior::updateRtm(double affordance, double max, double min) {
    //retta passante per (min,0.1) e (max,1)
    double m = 9 / ((max - min)*10);
    double q = -((9 * min) / (10 * (max - min))) + 0.1;

    double result;

    if (affordance > min)
        if (((m * affordance) + q) > 1)
            result = SEED_DEFAULT_RTM;
        else
            result = (m * affordance) + q;
    else
        result = (m * min) + q;

//    //BOTTOM-UP FUORI DAL LEARNING
//    if(emp == 0)
//        emp = result;
//    else
//        emp = result/emp;

    //BOTTOM-UP NEL LEARNING
    setSelfContribution(1-result);

    //double emp = WM->getNodesByInstance(this->getInstance())[0]->emphasis();

    double emp = WM->getInstanceEmphasis(this->getInstance());

    if(emp <= 0)
        emp = 1;
    else
        emp = 1/emp;

    if( emp > 1 )
        setRtm(1);
    else if( emp < 0.01 )
        setRtm(0.01);
    else
        setRtm(emp);

    //comunica alla WM lo stato del ritmo (per eventuali stampe)
    std::vector<WM_node*> app = WM->getNodesByInstance(getInstance());
    for (size_t i = 0; i < app.size(); i++)
        app[i]->rtm = rtm;

    this->updated = true;
}

//update del ritmo (inteso come periodo di attesa) - EX sedDefaultRtm
//  NOTA: qui il rtm viene aggiornato considerato la sola enfasi del nodo!!
//  il rtm che risulta NON è compreso in nessun itnervallo
void Behavior::updateRtm() {

    double newRtm = 1;

    pthread_mutex_lock(&memMutex);

    if(dead()){
        std::cout<<this->getInstance()<<"\n";
        return;
    }

    std::vector<WM_node*> app = WM->getNodesByInstance(getInstance());

    updated = true;

    //double emph = app[0]->emphasis();
    double emph = WM->getInstanceEmphasis(this->getInstance());

    if (emph != 0)
        newRtm = 1 / emph;


    int trun = (int) (newRtm * PRECISION);
    rtm = ((double) trun) / PRECISION;

    //comunica alla WM lo stato del ritmo (per eventuali stampe)
    for (size_t i = 0; i < app.size(); i++)
        app[i]->rtm = rtm;


    pthread_mutex_unlock(&memMutex);
}


/*
//funzione di aggiornamento del periodo con legge di weber e soglia
// il max diventa la soglia oltre il quale lo stimolo viene percepito
//ATTENZIONE: qui il ritmo non viene integrato con la magnitudine
void Behavior::updateRtm_weber2(double newStimulus, double maxStimulus, double minStimulus) {
    //retta passante per (min,0.1) e (max,1)
    //double newPeriod=0.01;
    double newPeriod = 1;
    double newRelevance;

    //k neutra per prova
    k_weber = 1;

    if (oldStimulus != 0 && newStimulus < maxStimulus) {

        if (newStimulus != 0 && newStimulus <= oldStimulus) //&& newStimulus > minStimulus)
            newRelevance = k_weber * ((oldStimulus - newStimulus) / newStimulus);
        else if (newStimulus > oldStimulus)
            newRelevance = -k_weber * ((newStimulus - oldStimulus) / oldStimulus);
        else
            newRelevance = oldPeriod;

        newPeriod = oldPeriod - newRelevance;

        if (newPeriod < 0.01)
            newPeriod = 0.01;
        else if (newPeriod > 1)
            newPeriod = 1;
    }

    setRtm(newPeriod);

    //        std::cout<<oldPeriod<<" >> "<<oldStimulus<<" -> "<<newStimulus<<" >> "<<this->getRtm()<<"\n";

    oldStimulus = newStimulus;
    oldPeriod = this->getRtm();
}
//funzione di aggiornamento della frequenza con legge di Weber
//  gli ultimi 2 parametri sono fittizi servono solo per evitare di modificare le chiamate a funzione
//ATTENZIONE: qui il ritmo non viene integrato con la magnitudine

void Behavior::updateRtm_weber(double newStimulus, double fake1, double fake2) {

    //double newPeriod=0.01;
    double newPeriod = 1;
    double newRelevance;

    //k neutra per prova
    k_weber = 1;

    if (oldStimulus != 0) {
        //            if( (oldFeature - newFeature) >= 0 )
        //                newDelta = ( (oldFeature - newFeature) / oldFeature ) / oldRtm;
        //            else
        //                newDelta = oldRtm / ( (oldFeature - newFeature) / oldFeature );

        //newRelevance = k_weber * ( (oldStimulus - newStimulus) / oldStimulus);

        newRelevance = k_weber * ((oldStimulus - newStimulus) / newStimulus);

        newPeriod = oldPeriod - newRelevance;

        if (newPeriod < 0.01)
            newPeriod = 0.01;
        else if (newPeriod > 1)
            newPeriod = 1;
    }

    setRtm(newPeriod);

    //        std::cout<<oldPeriod<<" >> "<<oldStimulus<<" -> "<<newStimulus<<" >> "<<this->getRtm()<<"\n";

    oldStimulus = newStimulus;
    oldPeriod = this->getRtm();
}
*/

void Behavior::setSelfContribution(double c){
    //std::string self_cont_name = this->getName();
    std::string self_cont_name = "self";
    std::vector<WM_node *> nodes = WM->getNodesByInstance(this->getInstance());
    for (size_t i = 0; i < nodes.size(); i++) {
        if(nodes[i]->contribution[self_cont_name] == NULL)
            nodes[i]->contribution[self_cont_name] = new double(c);
        else
            *(nodes[i]->contribution[self_cont_name]) = c;
    }
}

void Behavior::setContribution(std::string source, std::string target, double c){
    std::vector<WM_node *> nodes = WM->getNodesByInstance(target);
    c=c/nodes.size(); //quando si contribuisce ad una generica istanza il contributo va diviso
    for (size_t i = 0; i < nodes.size(); i++) {
        if(nodes[i]->contribution[source] == NULL)
            nodes[i]->contribution[source] = new double(c);
        else
            *(nodes[i]->contribution[source]) = c;
    }
}

void Behavior::delContribution(std::string source, std::string target){
    std::vector<WM_node *> nodes = WM->getNodesByInstance(target);
    for (size_t i = 0; i < nodes.size(); i++) {
        if(nodes[i]->contribution[source] != NULL){
            delete(nodes[i]->contribution[source]);
            nodes[i]->contribution[source] = NULL;
        }
    }
}

//ritorna il nuovo contributo calcolato
double Behavior::addContribution(std::string source, std::string target, double c){
    double newContribute = c;
    std::vector<WM_node *> nodes = WM->getNodesByInstance(target);
    for (size_t i = 0; i < nodes.size(); i++) {
        if(nodes[i]->contribution[source] == NULL)
            nodes[i]->contribution[source] = new double(c);
        else {
            *(nodes[i]->contribution[source]) += c;
            newContribute = *(nodes[i]->contribution[source]);
        }
    }
    return newContribute;
}

double Behavior::getRtm() {
    return rtm;
}

//setta un qualsiasi rtm
//  NOTA: questo rtm non dipende da emfasi ne da affordance, va direttamente a modificare la frequenza del behavior
//        il rtm settato NON è vincolato in nessun intervallo
void Behavior::setRtm(double newRtm) {

    updated = true;

    int trun = (int) (newRtm * PRECISION);
    rtm = ((double) trun) / PRECISION;

    //comunica alla WM lo stato del ritmo (per eventuali stampe)
    if (WM != NULL) {
        std::vector<WM_node*> app = WM->getNodesByInstance(getInstance());
        for (size_t i = 0; i < app.size(); i++)
            app[i]->rtm = rtm;
    }

    //se il ritmo richiesto è quello di quiescenza (2 secs) non applicarlo
    //     quando si è in quiescenza non aspettare 2 secondi!!
    if (rtm == QUIESCENCE)
        rtm = SEED_DEFAULT_RTM;
}

//DEPRECATED: now name must be set into the static behavior_name attribute!
//setta il nome del behavior
//void Behavior::setName(std::string newName) {
//    name = newName;
//}

//DEPRECATED: now there is the static behavior_name attribute implemented into the sub-classes!
//restituisce il nome del behavior
//std::string Behavior::getName() {
//    return name;
//}
//setta l'istanza del behavior (nome+parametri)

void Behavior::setInstance(std::string newInstance) {
    instance = newInstance;
    args = instance2vector(newInstance);
}
//restituisce l'istanza del behavior, compreso di parametri

std::string Behavior::getInstance() {
    return instance;
}

void Behavior::setReleaser(bool r) {
    releaser = r;
}

bool Behavior::getReleaser() {
    return releaser;
}

double Behavior::getStimulus() {
    return oldStimulus;
}

double Behavior::setStimulus(double s) {
    oldStimulus = s;
    return oldStimulus;
}

bool Behavior::isUpdated() {
    return updated;
}

bool Behavior::isBackground() {
    return background;
}

pthread_t Behavior::getTid() {
    return tid;
}

void Behavior::setTid(pthread_t t) {
    tid = t;
}

/**
 *
 * @param instances vettore delle istanze da amplificare
 *
 */
void Behavior::amplifyAllInstances(std::string source, std::vector<WM_node *> instances) {
    //per ogni istanza
    for (size_t i = 0; i < instances.size(); i++)
        //se l'istanza è rilasciata
        if (instances[i]->isBranchReleased()) {
            //incrementane il contatore dei goal raggiunti
            instances[i]->goalCount++;
            //radice del sottoalbero da amplificare
            //(instances[i]->father)->amplify(source, 1);
            (instances[i]->father)->setContribution(source, 1);
        }
}



// check if this behavior has been frozen
//  TODO this mechanism must be revised!
bool Behavior::isFrozen() {
    bool result = false;
    pthread_mutex_lock(&memMutex);
    if (dead())
        return false;
    if (WM->isReleased("freeze(" + this->getInstance() + ")"))
        result = true;
        //questo ELSE è stato aggiunto per bloccare i sottoschemi di PLAN
    else {
        size_t i = 0;
        //finche ci sono schemi e sono freezzati
        while ( i < WM->getNodesByInstance(this->getInstance()).size() && WM->getNodesByInstance(this->getInstance())[i]->freezed)
            //incrementa
            i++;
        //se ho controllato tutti i figli (ie. erano tutti freezzati)
        if (i == WM->getNodesByInstance(this->getInstance()).size())
            //allora sono freezzato
            result = true;
    }
    pthread_mutex_unlock(&memMutex);
    return result;
}

/** funzione di settaggio del releaser interno nei nodi della WM
 *
 * @param ir: releaser interno del processo, viene dato in output
 *      dallo schema percettivo.
 *
 * NB. prima di eseguire questa funzione va lockato il semaforo della
 *      WM.
 */
void Behavior::setNodeInternalReleaser(bool ir) {
    for (size_t i = 0; i < myNodeList.size(); i++) {
        myNodeList[i]->internalReleaser = ir;
    }
}

/** funzione per la scrittura del file di debug (PROJ/debug/instance)
 *
 * @param s: stringa da scrivere nel file di debug
 */
void Behavior::debug_write(std::string filename ,std::string s) {
    std::ofstream file;
    std::string path = SEED_HOME_PATH + "/src/debug/" + filename;
    file.open(path.c_str(),std::ios::app);
    file<<s;
    file.close();
}

/** funzione di controllo del behavior in WM
 *
 * @return TRUE se il behavior è presente in WM
 *
 * inoltre, setta il "instance" e "rtm" sulla base
 * del nodo eventualmente trovato
 */
bool Behavior::perceptWM() {
    std::vector<WM_node*> me;
    bool result = false;

    pthread_mutex_lock(&memMutex);

    if (WM == NULL) {
        pthread_mutex_unlock(&memMutex);
        return false;
    }
    //controlla se sono in memoria
    me = WM->getNodesByInstance(getInstance());
    myNodeList = me;

    //se sono in memoria
    if (me.size() != 0) {
        //ritorna true
        result = true;

        //se il releaser è disattivo
        if (!WM->isReleased(getInstance()) ||
                //oppure sono teleologico ed ho raggiunto il goal
                (me[0]->teleological && me[0]->goalStatus())) {
            //inibisci lo schema motorio
            setRtm(QUIESCENCE);
            setReleaser(false);
        }//altrimenti sono rilasciato
        else {
            setReleaser(true);
        }

    }
    size_t i=0;
    while(i<me.size() && me[i]->background)
        i++;

    if(i == me.size())
        this->background = true;
    else
        this->background = false;

    //inizializza a false, verrà messa a true se qualcuno chiama updateRtm
    this->updated = false;

    pthread_mutex_unlock(&memMutex);
    //altrimenti NON sono in memoria e ritorna false
    return result;
}

} //namespace seed
