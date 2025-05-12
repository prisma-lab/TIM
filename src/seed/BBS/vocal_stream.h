#ifndef VOCAL_STREAM_H
#define VOCAL_STERAM_H

#include "seed_header.h"


//#include "openprs/opaque-pub.h"
//#include "openprs/mp-pub.h"
#include "sys/times.h"
#include "sys/vtimes.h"
#include "sys/time.h"
#include <csignal>
#include <syscall.h>


//test espeak -> sudo apt-get install espeak-data libespeak-dev espeak-ng
//#include <espeak-ng/speak_lib.h>
#include <espeak/speak_lib.h>

int espeak_SynthCallback(short *wav, int numsamples, espeak_EVENT *events) {
    (void) wav;
    (void) numsamples;
    (void) events;
    return 0;
}

class VocalStreamBehavior : public Behavior {
public:

    VocalStreamBehavior(std::string instance) {
        setInstance(instance);
        setRtm(QUIESCENCE);
        
        epseak_initialized = wmv_get<bool>("espeak_initialized");

    }
    std::string getName(){
        return "vocalStream";
    }
    bool perceptualSchema() {

        return true;
    }

    void motorSchema() {
        
        pthread_mutex_lock(&memMutex);

        //remove(WM->getNodesByInstance(this->getInstance()));
        setRtm(0.001);
        
        //to_say = wmv_get<std::string>("say");
        to_say = wmv_solve<std::string>("say");
        
        if(to_say == "" || to_say == "\"\""){
            //nothing to say
            pthread_mutex_unlock(&memMutex);
            return;
        }
        
        //set immediately the "done" variable to speedup the speech
        std::string done_str = "say(" + to_say + ").done";
        wmv_set<bool>(done_str,true);

        pthread_mutex_unlock(&memMutex);
        
        //the competition is solved quite fast, but the following call is very slow...
        //  this speech is effective by using say(X) with X quite big sentence
        if (espeak_Synth(to_say.c_str(), strlen(to_say.c_str()), 0, POS_CHARACTER, 0, espeakCHARS_AUTO, NULL, NULL) != EE_OK) {
            std::cout<<ansi::red<< this->getInstance() <<": error on synth creation"<<ansi::end<<std::endl;
        }
        
//        done_str = "say(" + to_say + ").done";
//        
//        pthread_mutex_lock(&memMutex);
//        
//        wmv_set<double>(done_str,1);
//        //wmv_set<std::string>("say","");
//        //wmv_set<std::string>("say.winner",""); //reset competition
//        
//        pthread_mutex_unlock(&memMutex);
        
        //std::cout << this->getInstance() << " " << done_str<< std::endl;
        
    }

    void start() {

        if(epseak_initialized)
            return;

        //must be called before any other functions
        //espeak initialize
        if (espeak_Initialize(AUDIO_OUTPUT_SYNCH_PLAYBACK, 0, NULL, espeakINITIALIZE_PHONEME_EVENTS) < 0) {
            std::cout<<ansi::red<<"vocalStream: could not initialize espeak"<<ansi::end<<std::endl;
            return;
        }
        
        espeak_SetSynthCallback(espeak_SynthCallback);

        espeak_SetVoiceByName("mb-us1"); //female
        //espeak_SetVoiceByName("mb-us2"); //male
        
        //set speech velocity
        espeak_SetParameter(espeakRATE,110,0); //120 is good
        
        char textBuff[255]={0};
	strcpy(textBuff, "hello");
        
        if (espeak_Synth(textBuff, strlen(textBuff), 0, POS_CHARACTER, 0, espeakCHARS_AUTO, NULL, NULL) != EE_OK) {
            std::cout<<ansi::red<< this->getInstance() <<": error on synth creation"<<ansi::end<<std::endl;
        }
        
        textBuff[255]={0};
	strcpy(textBuff, "world.");
        
        if (espeak_Synth(textBuff, strlen(textBuff), 0, POS_CHARACTER, 0, espeakCHARS_AUTO, NULL, NULL) != EE_OK) {
            std::cout<<ansi::red<< this->getInstance() <<": error on synth creation"<<ansi::end<<std::endl;
        }
        
        pthread_mutex_lock(&memMutex);
        
        wmv_set<bool>("espeak_initialized",true);
        
        pthread_mutex_unlock(&memMutex);
        
        
        std::cout<<ansi::cyan<<"vocalStream: espeak initialized"<<ansi::end<<std::endl;

    }

    void exit() {
    }

protected:
    int samplerate; // determined by espeak, will be in Hertz (Hz)
    const int buflength = 200; // passed to espeak, in milliseconds (ms)

    std::vector<short> sounddata;
    
    std::string to_say;
    bool epseak_initialized;
    
    //seed::time::Clock clk;
};





class SayBehavior : public Behavior {
public:

    SayBehavior(std::string instance) {
        setInstance(instance);
        setRtm(QUIESCENCE);
        
        to_say = instance2vector(instance)[1];
        
        epseak_initialized = wmv_get<bool>("espeak_initialized");

    }
    std::string getName(){
        return "say";
    }
    bool perceptualSchema() {

        pthread_mutex_lock(&memMutex);

        pthread_mutex_unlock(&memMutex);

        return true;
    }

    void motorSchema() {

        //espeak_ERROR speakErr;
        
        
        pthread_mutex_lock(&memMutex);

        //remove(WM->getNodesByInstance(this->getInstance()));
        //setRtm(0.1);
        
        winning = wmv_compete<std::string>("vocalStream", "say", to_say);

        pthread_mutex_unlock(&memMutex);
        
//        if(winning == 0){
//            std::cout << this->getInstance() << ": silent" << std::endl;
//            return;
//        }
//        
////        if( espeak_IsPlaying() == 1 ){
////            std::cout << "\t stop speaking!" << std::endl;
////            espeak_Cancel();
////        }
//        
////        if( espeak_Cancel() != EE_OK )
////            std::cout << this->getInstance() << ": unable to stop last speak" << std::endl;
//        
//        std::cout << this->getInstance() << ": SPEAK" << std::endl;
//        
//        
//        //std::cout << this->getInstance() << ": speak" << std::endl;
//
//        if (espeak_Synth(to_say.c_str(), strlen(to_say.c_str()), 0, POS_CHARACTER, 0, espeakCHARS_AUTO, NULL, NULL) != EE_OK) {
//            std::cout<<ansi::red<< this->getInstance() <<": error on synth creation"<<ansi::end<<std::endl;
//        }
//
//        std::cout << this->getInstance() << ": done" << std::endl;
        
    }

    void start() {
        
        if(epseak_initialized)
            return;

        //must be called before any other functions
        //espeak initialize
        if (espeak_Initialize(AUDIO_OUTPUT_SYNCH_PLAYBACK, 0, NULL, espeakINITIALIZE_PHONEME_EVENTS) < 0) {
            std::cout<<ansi::red<<"SAY: could not initialize espeak"<<ansi::end<<std::endl;
            return;
        }
        
        espeak_SetSynthCallback(espeak_SynthCallback);

        espeak_SetVoiceByName("mb-us1");
        
        espeak_SetParameter(espeakRATE,130,0);
        
        
        pthread_mutex_lock(&memMutex);
        
        wmv_set<bool>("espeak_initialized",true);
        
        pthread_mutex_unlock(&memMutex);
        
        
        std::cout<<ansi::cyan<<"SAY: espeak initialized"<<ansi::end<<std::endl;
        

    }

    void exit() {
        pthread_mutex_lock(&memMutex);
        
        //self reset "done variable"
        std::string done_str = this->getInstance() + ".done";
        if(wmv_get<bool>(done_str))
            wmv_set<bool>(done_str,false);
        
        pthread_mutex_unlock(&memMutex);
    }

protected:
    int samplerate; // determined by espeak, will be in Hertz (Hz)
    const int buflength = 200; // passed to espeak, in milliseconds (ms)

    std::vector<short> sounddata;
    
    std::string to_say;
    bool epseak_initialized;
    bool winning;
};


#endif
