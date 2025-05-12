/*
 * File:   seed_header.h
 * Author: hargalaten
 *
 * Created on 12 dicembre 2012, 15.45
 */

#ifndef LTM_SWIPL_H
#define	LTM_SWIPL_H

#include "seed.h"

#include <SWI-cpp.h>
#include <SWI-Prolog.h>

//#include "eclipseclass.h"

namespace seed {

//LTM class
class LongTermMemory_swipl : public LongTermMemory {
public:
    
    LongTermMemory_swipl(int &, char **);
    
    virtual bool loadLTM(std::string);
    
    virtual bool loadNodeSemantics(WM_node *);
    
    virtual std::string query(std::string);
    
    virtual bool close();
    
    //WM_node *saveNode(WM_node *);
    
private:
    
    std::vector<std::string> read_list(std::string);

    void loadSemantics(std::string, WM_node*);
    
    inline void replace_point(std::string &from_prolog, const std::string s = "*.*", const std::string t = "."){
        //std::replace( from_prolog.begin(), from_prolog.end(), '@', '.');
        std::string::size_type n = 0;
        while ( ( n = from_prolog.find( s, n ) ) != std::string::npos )
        {
            from_prolog.replace( n, s.size(), t );
            n += t.size();
        }
    }
    
    PlTerm string2term(std::string str, std::unordered_map<std::string, PlTerm> &swi_vars);
    
    std::string ltm_path;
    
    PlEngine *swi_engine;
    
};

} //namespace seed


#endif	/* LTM_SWIPL_H */

