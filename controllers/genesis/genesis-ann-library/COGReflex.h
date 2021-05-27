/*
 * COGReflex.h
 *
 *  Created on: 2020-02-01
 *      Author: suntao
 */

#ifndef COGREFLEX_H_
#define COGREFLEX_H_

#include <utils/ann-framework/ann.h>
#include <utils/ann-library/vrn.h>
#include <stdio.h>
#include <assert.h>
#include <iostream>
#include "neuralpreprocessing.h"
#include "learning-tool/lowPassfilter.h"
#include "learning-tool/movingAverage.h"
#include "learning-tool/dilearn.h"

//using namespace shark;
class NP;
class lowPass_filter;
using namespace std;
namespace stcontroller {

    //-------------------COG distribution class------------------//
    class COGDistribution{
        public:
            COGDistribution(bool leftRight=false);// defualt to calculate the hind and front side
            ~COGDistribution();
            void setInput(vector<float> input);
            void step();
            float getOutput();
        private:
            lowPass_filter *NP_oneSideLegsGRF;
            lowPass_filter *NP_anotherSideLegsGRF;
            MovingAverage *NP_movingAverage;
            lowPass_filter *filterCOG;
            float output;
            float one_side_leg_grf;
            float another_side_leg_grf;
            bool leftRight;

    };
    //
    //------------------COG Reflex class ----------------------------//
    class COGReflex: public ANN{
        public:
            COGReflex(unsigned int leg);
            virtual ~COGReflex();
            void setInput(vector<float> grf);
            void step();
            float getOutput(unsigned int index);
            void setSetpoint(float setpoint);
        private:

            unsigned int ID;//identity
            COGDistribution * cog_FH;// two paris(front-hind, and left-right)
            COGDistribution * cog_RL;
            ADILearn* dil_FH;//three joints
            DILearn* dil_RL;
            float output1,output2,output3;
            float setpoint;
    };

} /* namespace stcontroller */

#endif /* COGREFLEX_H_ */
