/*
 * COGDistribution.cpp
 *  Created on: 2020-02-01
 *      Author: suntao
 */

#include "COGReflex.h"
using namespace stcontroller;
namespace stcontroller {

    //----------------COGDistribution class -------------------------------//
    COGDistribution::COGDistribution(bool leftRight){
        /**
         * @descrption: Calculate the forces (GRFs) distribution between front and hind legs or left and right legs
         * @param: defualt is to calculate front and hind sides
         * There are four objects created. 
         */
        this->leftRight=leftRight;

        NP_oneSideLegsGRF=new lowPass_filter(0.3);
        NP_anotherSideLegsGRF=new lowPass_filter(0.3);
        NP_movingAverage=new MovingAverage(1,60);//60
        filterCOG=new lowPass_filter(0.3);
    }

    COGDistribution::~COGDistribution(){
        delete NP_oneSideLegsGRF;
        delete NP_anotherSideLegsGRF;
        delete NP_movingAverage;
        delete filterCOG;
    }

    float COGDistribution::getOutput(){
        return output;
    }

    void COGDistribution::setInput(vector<float> input){
        /**
         * @param: input, GRF (ground reaction forces) of all legs
         * @description: depeingding on leftRight ( a bool parameter) to set which sides are used to compare, default is front and hind sides
         * @return: void.
         */ 
        assert(input.size()==4);// four legs, please set it as the leg numbers
        if(leftRight){// between left and right side
            one_side_leg_grf=input.at(0)+input.at(1);   //right side
            another_side_leg_grf=input.at(2)+input.at(3);   //left side
        }else{
            one_side_leg_grf=input.at(0)+input.at(2);   //front side
            another_side_leg_grf=input.at(1)+input.at(3);   //right side
        }
    }

    void COGDistribution::step(){
        float temp=0.0;
        if((one_side_leg_grf>0.1)&&(another_side_leg_grf>0.1)){//this means the feet should be in stance phase
            temp = NP_oneSideLegsGRF->update(one_side_leg_grf)/NP_anotherSideLegsGRF->update(another_side_leg_grf);
        }
        //moving average filer and a low pass filter
        output=filterCOG->update(NP_movingAverage->update(temp));
    }


    //-----------------COGReflex class -----------------------//
    COGReflex::COGReflex(unsigned int leg){
        ID =leg;
        cog_FH = new COGDistribution(false);//beweent front hind legs
        cog_RL = new COGDistribution(true);//between right left legs
        dil_RL = new DILearn();// for hip 1 joit
        dil_FH = new ADILearn();//for hip 2 and knee joint
        dil_FH->setLimit(0.70,-1.5);
        dil_FH->setLimit(1.0,-1.0);
        //--Good parameters
        dil_RL->setParameters(0.65, 0.056, 0.000625, 0.85, 0.01875, 0.0000625); // flat and up 10, 20 deg
        //dil_FH->setParameters(0.65, 0.056, 0.000625, 0.95, 0.01875, 0.0000625); // flat and up 10, 20 deg
        dil_FH->setParameters(0.5, 0.045, 0.0009, 0.99, 0.0001, 0.0001); // ADIL for all slopes

        setNeuronNumber(3);
        setTransferFunction(0,identityFunction());//hip1
        setTransferFunction(1,identityFunction());//hip2
        setTransferFunction(2,identityFunction());//knee
        output1=0.0;// hip 1
        output2=0.0;// hip 2
        output3=0.0;//knee
        setpoint=1.0;//1.44;// on flat florr
    }

    COGReflex::~COGReflex(){
        delete cog_FH;
        delete cog_RL;
        delete dil_FH;
        delete dil_RL;
    }

    void COGReflex::setInput(vector<float> grf) {
        assert(grf.size()==4); //the number of legs equal 4
        cog_FH->setInput(grf);
        cog_RL->setInput(grf);
    }

    void COGReflex::step(){
        cog_FH->step();
        cog_RL->step();

        if(cog_RL->getOutput() > 0.1){
            dil_RL->setInput(0.91, cog_RL->getOutput());// ideal situation is 1.0, consider the diff of the geometry and physical partial between two sides;
            dil_RL->step();
        }
        /*   front hind leg offset adjustment     */
        if(cog_FH->getOutput() > 0.1){// when the robot was put on the ground
            dil_FH->setInput(setpoint,cog_FH->getOutput());// 1.6 is the standard value of the COG, the value was got by experiments
            dil_FH->step();
        }
        


        output1=dil_RL->getOutput();
        output2=0.72*dil_FH->getOutput();
        output3=0.44*dil_FH->getOutput();//0.44

        // good desired values
        //setpoint=1.44+0.35*output1+0.20*output2; //from -35 ~ 30
        setpoint=1.0;//1.01;//1.035+0.35*output2+0.20*output3; //good for0.0 (setpoint=1.035) 10 (setpoint=1.24), 20 (setpoint=1.04), 30 (setpoint=1.04) and 35 up (setpoint=0.97)

        ANN::setOutput(0,output1);
        ANN::setOutput(1,output2);
        ANN::setOutput(2,output3);

        //@print 
        if(ID==0){
           printf("cog_FH: %0.2f\t setpoint: %0.2f \t dil_FH:%0.3f\n", cog_FH->getOutput(), setpoint, dil_FH->getOutput());
        }
    }

    float COGReflex::getOutput(unsigned int index){
        assert(index<=2);
        if(index==1)
            return output1;
        else
            return output2;
    }

    void COGReflex::setSetpoint(float setpoint){
        this->setpoint=setpoint+1.0;
    }

} /* namespace stcontroller */
