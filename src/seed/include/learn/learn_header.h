/*
 * File:   learn_header.h
 * Author: hargalaten
 *
 * Created on 17 marzo 2016, 14.58
 */

#ifndef LEARN_HEADER_H
#define	LEARN_HEADER_H

//#include "../../seed_header.h"
#include "../../seed.h"

#define LEARN_COUT 0

#ifdef	__cplusplus
extern "C" {
#endif

#define MAX_WEIGHT_VALUE 10.0

namespace seed{

/**
 *
 * @param x variabile
 * @param c massimo bonus [0,1]
 * @param l valore massimo della curva (1 default)
 * @param k steepness, ie. quanto è ripida (-10 default)
 * @param x0 valore medio della sigmoide (0.5 default)
 *
 * @return bonus/malus da dare al peso [c,c-1]
 *
 * Funzione logistica (sigmoidea) che restituisce il bonus da dare al peso.
 */
double learning_logistic( double x, double c, double l=1, double k=-10, double x0=0.5){
    return ( l / ( 1 + std::exp( -k * ( x - x0 ) ) ) ) + ( c - 1 );
}

double learning_step( double x, double c){
    if(x<=c)
        return 1;
    else
        return -1;
}

/*
 * Funzione ricorsiva per l'aggiornamento dei pesi (bottom-up).
 *
 */
void update_weights(std::string effectedInstance, double deErr, std::string left_spacing="\t"){
    double etha=0.01; //etha=0.1;
    //prendi tutte le istanze del nodo che ha partecipato all'ipotesi
    std::vector<WM_node *> effectedNodes = WM->getNodesByInstance(effectedInstance);

    if (effectedNodes.size() != 0) {
        //prendi la lsita dei pesi associati alla istanza
        std::unordered_map<std::string, double*> weights = WMV.get< std::unordered_map<std::string, double*> >(effectedNodes[0]->name + ".weights");

        double contSum = 0;
        //calcola la somma dei contributi
        for (std::unordered_map<std::string, double *>::iterator it = effectedNodes[0]->contribution.begin(); it != effectedNodes[0]->contribution.end(); ++it) {
            contSum += *(it->second);
        }

        //per ogni coppia peso-contributo
        for (std::unordered_map<std::string, double *>::iterator it = effectedNodes[0]->contribution.begin(); it != effectedNodes[0]->contribution.end(); ++it) {
            std::string sourceName = instance2vector(it->first)[0];
            //calcola la percentuale di contributo di questa istanza
            double pCont = *(it->second); // / contSum;

            double old_w = -1;
            double new_w = -1;

            //applica il learning factor a tutte le istanze del nodo
            double *w = weights[sourceName];
            if (w != NULL) {
                old_w = *w;
                *w = (*w) + etha * deErr * (*(it->second)); // / contSum;

                //saturo i pesi per evitare che vadano a +/-inf.. non dovrebbero saturare mai.
                if(*w>=MAX_WEIGHT_VALUE)
                    *w = MAX_WEIGHT_VALUE;
                else if (*w<=-MAX_WEIGHT_VALUE)
                    *w = -MAX_WEIGHT_VALUE;

                new_w = *w;
            }
//            else
//                std::cout<<"ERROR: "<<effectedInstance<<" has no W for "<<sourceName<<"\n";

#if LEARN_COUT
            std::cout << left_spacing <<deErr<<" "<<pCont <<"\n";

            if (old_w - new_w < -0.001)
                std::cout << left_spacing << "\033[0;32m " << it->first << ": " << old_w << "->" << new_w << " \033[0m \n";
            else if (old_w - new_w > 0.001)
                std::cout << left_spacing << "\033[0;31m " << it->first << ": " << old_w << "->" << new_w << " \033[0m \n";
            else
                std::cout << left_spacing << it->first << ": " << old_w << "->" << new_w << " \n";
#endif
        }

        WMV.set< std::unordered_map<std::string, double*> >("seed_learn", effectedNodes[0]->name + ".weights", weights);

        for(int i=0; i<effectedNodes.size(); i++){
            //chiamata ricorsiva per il contributo della gerarchia
            if( effectedNodes[i]->father->abstract && effectedNodes[i]->father->isBranchReleased()){

                //aggiorna il peso di mio padre
                std::string sourceName = effectedNodes[i]->father->name;
                //calcola il contributo del padre
                double fCont = effectedNodes[i]->father->emphasis(false); //WM->getInstanceEmphasis(effectedNodes[i]->father->instance);

                double old_w = -1;
                double new_w = -1;

                //applica il learning factor a tutte le istanze del nodo
                double *w = weights[sourceName];
                if (w != NULL) {
                    old_w = *w;
                    *w = (*w) + etha * deErr * fCont;

                    if(*w>=MAX_WEIGHT_VALUE)
                        *w = MAX_WEIGHT_VALUE;
                    else if (*w<=-MAX_WEIGHT_VALUE)
                        *w = -MAX_WEIGHT_VALUE;

                    new_w = *w;
                }
//                else
//                    std::cout<<"ERROR: "<<effectedInstance<<" has no W for "<<sourceName<<"\n";

#if LEARN_COUT
                if (old_w - new_w < -0.001)
                    std::cout << left_spacing << "\033[0;32m " << effectedNodes[i]->father->instance << ": " << old_w << "->" << new_w << " \033[0m \n";
                else if (old_w - new_w > 0.001)
                    std::cout << left_spacing << "\033[0;31m " << effectedNodes[i]->father->instance << ": " << old_w << "->" << new_w << " \033[0m \n";
                else
                    std::cout << left_spacing << effectedNodes[i]->father->instance << ": " << old_w << "->" << new_w << " \n";
#endif

                //std::cout<< left_spacing << ">go up<\n";
                update_weights(effectedNodes[0]->father->instance, old_w*deErr, left_spacing + "\t");
            }
            //std::cout<< left_spacing << effectedNodes[i]->father->instance <<" updated!\n";
        }

    }
}

void learn(std::string var){
    //discount factor
    //double etha = 0.1;
    double learning_th=0.1;
    
    std::cout<<ansi::cyan<<"learning for variable "<<var<<" started!"<<ansi::end<<std::endl;

    std::string var_star(var);
    var = "#" + var;

    //valore ipotizzato dal sistema
    
    //std::vector<double> valList = wmv_get< std::vector<double> >(var + ".values");
    //std::vector<contention_couple> contendersList = wmv_get< std::vector<contention_couple> >(var + ".contention");
    std::vector<WM_value> contenders = WMV.candidates(var);

    std::ofstream file;
    std::string path = SEED_HOME_PATH + "/src/behavior/learn/learning/" + var_star;
    file.open(path.c_str(),std::ios::app);

    double err_out = std::abs(WMV.get<double>(var_star) - WMV.get<double>(var));

    //segna l'errore solo se c'è stata una fase di learning, altrimenti metti errore nullo!
    if(contenders.size()!=0)
        file<<err_out<<"\n";
    else
        file<<0<<"\n";

    file.close();

    //valore ottimo
    double valueStar = WMV.get<double>(var_star);
    double valRange = WMV.get<double>(var_star + ".range");

    if(valRange == 0){
        std::cout<<"WARNING: variable "<<var_star<<" has no range! learning aborted\n";
        return;
    }

    double err_out_norm = err_out/valRange;

#if LEARN_COUT
    if(valList.size()!=0)
        std::cout<<"learn("<<var<<"): v*: "<< valueStar << " v: " << wmv_get<double>(var) <<" -> "<< err_out <<"\n";
#endif

    //per ogni nodo che ha partecipato alla creazione dell'ipotesi ( valList.size() == contendersList.size() )
    for(int i = 0; i<contenders.size(); i++){

#if LEARN_COUT
        std::cout<<"cont: "<<contendersList[i].instance<<" ("<< valList[i] <<"|"<<contendersList[i].emp<<")\n";
#endif
        //percentuale d'errore
        double err_node = std::abs(valueStar - *( (double *) contenders[i].value ) ) / valRange;
        ///aggiorna tutti i pesi

        double sigma_err_node=0;

        sigma_err_node = learning_logistic(err_node,0.3);
        //sigma_err_node = learning_step(err_node,0.3); //questa è proprio SBAGLIATA, non ha alcun senso matematico

        //stampa
        if (sigma_err_node > 0)
            std::cout << "\033[0;32m " << contenders[i].instance << ": " << sigma_err_node << " " << err_node << " (" << var << ") \033[0m \n";
        else if (sigma_err_node < 0)
            std::cout << "\033[0;31m " << contenders[i].instance << ": " << sigma_err_node << " " << err_node << " (" << var << ") \033[0m \n";
        else
            std::cout << contenders[i].instance << ": " << sigma_err_node << " " << err_node << " (" << var << ") \n";

#if LEARN_COUT
        std::cout<<"sigma(err_node): "<<sigma_err_node<<"\n";
#endif

        update_weights(contenders[i].instance, err_out_norm * sigma_err_node);
    }

    if (contenders.size() != 0) {
        std::cout << "\n";
        WMV.clear< double >(var);
    }

}


} //seed namespace


#ifdef	__cplusplus
}
#endif

#endif	/* LEARN_HEADER_H */

