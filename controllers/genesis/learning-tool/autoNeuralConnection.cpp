#include "autoNeuralConnection.h"
/***************
 * Author; Tao Sun
 * Date: 2019
 * This class is a basic component of the adaptive nerual communication.
 * It's function is to estimate the interlimb movement phase relationship,
 * and estimate the phase stability, it is responsible for trrigger the ACI,
 * thereby realizing the neural communications among decoupeld CPGs.
 * author email: suntao.n@gmail.com
 *
 * *********/

AutoNeuralConnection::AutoNeuralConnection(int leg_num){
    mpd = new MotionPhaseDiffV3(leg_num); // motion phase difference
    mps = new MotionPhaseStability(mpd->getOutput());// motion phase stability
    anctrigger = new ANCtrigger(); // trigger for activating adaptive neural control input
}
AutoNeuralConnection::~AutoNeuralConnection(){
    delete mpd;
    delete mps;
    delete anctrigger;
}
void AutoNeuralConnection::step(){
    mpd->step();
    mps->setInput(mpd->getOutput());
    mps->step();
    anctrigger->setInput(mps->getOutput());
    anctrigger->step();
}
void AutoNeuralConnection::setInput(unsigned int index, pcpgsig sig){
    mpd->setInput(index,sig);
}

void AutoNeuralConnection::setInput(unsigned int index, SO2CPGOuts sig){
    mpd->setInput(index,sig);
}
Matrix AutoNeuralConnection::getPhaseDiff(){
    return mpd->getOutput();
}
float AutoNeuralConnection::getPhaseStability(){
    return mps->getOutput();
}
bool AutoNeuralConnection::getANCtrigger(){
    return anctrigger->getOutput();
}
