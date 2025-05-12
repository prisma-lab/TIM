/*
 * File:   seed_header.h
 * Author: hargalaten
 *
 * Created on 12 dicembre 2012, 15.45
 */

#ifndef BBS_H
#define BBS_H

#include "seed.h"

#include <functional>

namespace seed {

//LTM class
class BehaviorBasedSystem {
public:
    
    //BehaviorBasedSystem(); //use default constructor
    
    inline void add(std::string name, std::function<Behavior*(std::string)> creation_function){
        repository[name] = creation_function;
    }
    
    inline Behavior * load(std::string name, std::string instance){
        return repository[name](instance);
    }
    
private:

    std::unordered_map<std::string, std::function<Behavior*(std::string)> > repository;
    
};

} //namespace seed


#endif	/* BBS_H */

