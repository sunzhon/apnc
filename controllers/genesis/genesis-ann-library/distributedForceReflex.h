/*
 * distributedForceReflex.h
 *
 *  Created on: 2020-05-15
 *      Author: suntao
 */

#ifndef DISTRIBUTEDFORCEREFLEX_H_
#define DISTRIBUTEDFORCEREFLEX_H_

#include <utils/ann-framework/ann.h>
#include <utils/ann-library/vrn.h>
#include <stdio.h>
#include <assert.h>
#include <iostream>
#include "esnForwardmodel.h"
#include "learning-tool/lowPassfilter.h"
#include "learning-tool/dilearn.h"
#include "COGReflex.h"


class ESNForwardmodel;
class lowPass_filter;
class DILearn;
using namespace std;
namespace stcontroller {
    //------------------Optimizer for calculate the control action -----------//
    class Optimizer{
        public:
            Optimizer();
            ~Optimizer();
            void setInput(float actual_feedback, float predictive_feedback=0.0);
            void step();
            float getOutput();
        private:
            DILearn * dil;
            float flag;
            float bias;// as a set-point
            float output;
            float actual_feedback,predictive_feedback;
            //error
            float error,error_old,error_delta;
    };

    class ReflexInputLayer: public ANN{
        public:
            ReflexInputLayer(){
                setDefaultTransferFunction(identityFunction());
                setNeuronNumber(2);
                // This indicates the desired GRFs distribution (\gamma)
                b(0, -1.1);
                b(1, -1.1);

                w(1,0,-1.0);
            }

    };


    class ReflexHiddenLayer: public ANN{
        public:
            ReflexHiddenLayer(){
                setDefaultTransferFunction(identityFunction());        
                setNeuronNumber(1);
                w(0,0,1.0);
            }

    };

    class ReflexOutputLayer: public ANN{
        public:
            ReflexOutputLayer(){
                setDefaultTransferFunction(identityFunction());
                setNeuronNumber(2);
            }

    };


    //-------------------Distribution force feedback-based reflex class------------------//
    class DistributedForceReflex : public ANN{
        public:
            DistributedForceReflex(unsigned int ID);
            ~DistributedForceReflex();
            void setInput(std::vector<float> grf);
            void step();
            float getOutput();
        private:
            //feedback GRFs distribution
            unsigned int ID;
            COGDistribution *cog_FH;
            float feedback;
            float control_1,control_2,control_1_old,control_2_old;//for hip2 and knee joint of reflex outputs

            //optimization
            Optimizer *optimizer;
            float optimize_variable;

            //esn model
            ESNForwardmodel *esn;
            std::vector<float> esn_targets, esn_inputs;
            float  esn_output;           

            //reflex neural network
            ReflexInputLayer* input_layer;
            ReflexHiddenLayer* hidden_layer;
            ReflexOutputLayer* output_layer;
            float w_error,w_derivation_error;

    };

} /* namespace stcontroller */

#endif /* DISTRIBUTEDFORCEREFLEX_H_ */
