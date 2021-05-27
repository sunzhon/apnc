#include "distributedForceReflex.h"

using namespace stcontroller;
namespace stcontroller{

    DistributedForceReflex::DistributedForceReflex(unsigned int ID){
        this->ID=ID;
        esn = new ESNForwardmodel(/*unsigned int ID*/ ID, /*string path=*/ "/home/suntao/workspace/experiment_data/ESN_parameters/", /*unsigned int  n_input*/ 2, /*unsigned int n_output*/ 1, /*unsigned int n_hidden*/ 60, /*bool feedback*/ false, /*bool feeding_input*/ false, /*leak rate*/ 0.5);

        esn->setParameters(/*transfer_func_out=linear*/ 0, /*transfer_func_hidden=tanh*/2, /*standard esn learning, 1 =RL, 2=RLS or */ 2, /*input connect to all hidden neurons*/ 0, /* input range [-1.0,1.0]*/ 2.0, /*learn_mode=RLS*/ 1, /*load_weight not load learned weight*/ false, /*noise_range*/ 0.001, /* RCneuronNoise, constant fixed bias*/ false);

        esn_inputs.resize(2); //hip2 offset and knee offset
        esn_targets.resize(1); // Distributed Force
        esn_output=0.0;// distributed Force

        // reflex neural network
        input_layer= new ReflexInputLayer();
        hidden_layer = new ReflexHiddenLayer();
        output_layer = new ReflexOutputLayer();
        // neural interace to output side
        setNeuronNumber(2);
        ANN::setDefaultTransferFunction(identityFunction());
        
        // plasible weights
        w_error = 0.003;//0.003;//0.0021;//0.006
        w_derivation_error = 0.0032;//0.0032;//0.0022;//0.006

        w(hidden_layer->getNeuron(0),input_layer->getNeuron(0),w_error);
        w(hidden_layer->getNeuron(0),input_layer->getNeuron(1),w_derivation_error);

        w(output_layer->getNeuron(0), hidden_layer->getNeuron(0), 2.0); // hip2 0.72*2
        w(output_layer->getNeuron(1), hidden_layer->getNeuron(0), 1.0); // knee 0.44*2


        // feedback compuate COG gamma
        cog_FH = new COGDistribution();//beweent front hind legs

        // optimer outputs manipulation variable
        optimizer= new Optimizer();
        feedback=0.0;
        optimize_variable=0.0;
        control_1=0.0;
        control_2=0.0;
        control_1_old=0.0;
        control_2_old=0.0;
    }

    DistributedForceReflex::~DistributedForceReflex(){
        delete esn;
        delete cog_FH;
        delete optimizer;

        // delete neural network of reflex computation
        delete input_layer;
        delete hidden_layer;
        delete output_layer;
    }

    void DistributedForceReflex::step(){
        //1) calculate cog
        cog_FH->step();
        feedback=cog_FH->getOutput();

        //2) esn as forward model
        if(feedback>0.1){// when robot was put on ground (Distribution Force), run esn
            if(false){//close esn model
                //esn learning step
                esn_inputs.at(0)=control_1_old;
                esn_inputs.at(1)=control_2_old;
                esn_targets.at(0)=feedback;
                esn->setDataSet(esn_inputs,esn_targets);
                esn->stepLearning();
                //esn predicting step
                esn_inputs.at(0)=control_1;
                esn_inputs.at(1)=control_2;
                esn->setDataSet(esn_inputs,esn_targets);
                esn->stepPredict();
                esn_output=esn->getOutput(0);
            }
            //3) obtain control varibale as inputs of esn
            //if(esn->getLearned()){// whether finish initial learning process, not the all learning process
            optimizer->setInput(feedback);
            optimizer->step();
            optimize_variable=optimizer->getOutput();
            w(hidden_layer->getNeuron(0),input_layer->getNeuron(0),optimize_variable*w_error);
            w(hidden_layer->getNeuron(0),input_layer->getNeuron(1),optimize_variable*w_derivation_error);
            //}

            //4) get Outputs of the reflex, here should be the control variable
            input_layer->setInput(0,feedback+0.1);
            input_layer->setInput(1,feedback+0.1);
            input_layer->step();

            hidden_layer->step();
            output_layer->step();

            control_1_old = control_1;
            control_2_old = control_2;
            control_1 = output_layer->getOutput(0);
            control_2 = output_layer->getOutput(1);
            //neural interface to outside
            ANN::setOutput(0,control_1);
            ANN::setOutput(1,control_2);
        }

       //if(ID==0)
            //printf("actual out: %0.2f, prediction out: %0.2f, prediction error: %0.2f\n", feedback, esn_inputs.at(0), abs(feedback-esn_output));
        printf("cog: %0.3f, reflex:%0.3f, error:%0.3f\n", feedback, control_1, feedback-1.0);

    }

    void DistributedForceReflex::setInput(std::vector<float> grf){
        assert(grf.size()==4); //the number of legs equal 4
        cog_FH->setInput(grf);
    }


    /************************Optimization****************************************/
    Optimizer::Optimizer(){
        bias = 1.0; //it represents the set-point 
        output =0.0;
        actual_feedback = 0.0;
        predictive_feedback = 0.0;
        flag=0.0;

        //error
        error=0.0;
        error_old=0.0;
        error_delta=0.0;
        // dual integral learn 
        dil= new DILearn();
        dil->setParameters(0.35, 2.5, 0.01, 0.85, 0.6, 0.001); //
        dil->setLimit(3.0,-3.0);
    }

    Optimizer::~Optimizer(){
        delete dil;

    }

    void Optimizer::setInput(float actual_feedback, float predictive_feedback){
        this->actual_feedback = actual_feedback;
        this->predictive_feedback = predictive_feedback;
    }

    void Optimizer::step(){
        error = actual_feedback - bias;
        error_delta = abs(error) - abs(error_old);

        dil->setInput(0.0/*expected*/,error_delta/*actual*/);
        dil->step();
        output = (dil->getOutput() + 1.0);// incrase the weight if has error
        //if(abs(error)<0.06)
        //    output=0.0;
        //printf("error_delta: %0.4f, output:%0.4f\n", error_delta, output);
        error_old = error;
    }

    float Optimizer::getOutput(){
        // return control actiion as as control variable
        return output;
    }


} /* namespace stcontroller */
