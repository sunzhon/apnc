# <center> APNC</center>


## Introduction

This project provides a novel adaptive neural control based on the adaptive physical and neural communications (APNC) of decoupled CPGs.  The adaptive physical and neural communications (APC and ANC) build an phase synchronization channel among the decoupled CPGs through body-environment interaction dynamics and neural couplings. respectively. With the communications, the decoupled CPGs can synchronize their phases and thereby resulting in the robot legs controlled by the CPGs showing adaptively coordinated movement.  Specifically, the APC coordinate the decoupled CPGs using the sensory feedback, i.e., local ground reaction force (GRF) feedback. Sensory feedback gain of the APC play a critical role in the performance of the phase coordination, such as , phase convergence time, phase relationship, and so on. In this work, the sensory feedback gain is online modulated by a dual rate learning rate (DL) according to the match between the expected and actual GRFs. The ANC realizes the coordination through neural connections between the CPGs. The connections of the ANC rely on phase shifts of the CPG, while the phase shifts is generated by the APC. Thus, the ANC has a capability to adaptively coordinate the phase.

 

The adaptive neural control based on the APC and ANC is assessed via a quadruped robot (called Lilibot). The detailed of the robot can be seen at https://gitlab.com/neutron-nuaa/lilibot. Here, the control and robot are organized as two ROS nodes, which communicate through two ROS topics  (i.e., sensory topic and motor topic). 



<img src="/home/suntao/workspace/stbot/apnc/manual/figures/apnc.png" alt="Figure" style="zoom:50%;" />

## Framework

The project is organized by three sub-folders including **controllers**, **projects**, and **vrep-simulation**. In addition, the manual material is stored at **manual**.

- **controllers** consists of the code of the control methods, including synapticPlasticityCpg.cpp and synapticPlasticityCpg.h. The APC and ANC are also programed in these two files. 
- **project** contains the main.cpp of the project, and manage the software.
- **vrep-simulation** stores the simulation model which is based on VREP.  It has two quadruped robots: Lilibot



## The CPG-based control with PM and PR code is in:

- controllers/genesis/genesis-ann-library/synapticPlasticityCpg.cpp
- controllers/genesis/genesis-ann-lrbrary/synapticPlasticityCpg.h

## The implementation of the project
### Install necessary software on Ubuntu 18.04 or later version.
- The v4_1_0 or the latest version of the CoppeliaSim is necessary to run the simulation. The CoppeliaSim provides an platform to execute the simulated Lilibot. The CoppeliaSim can be download in here https://www.coppeliarobotics.com/.
- The vortex physical engine is necessary to execute dynamical computation of the simulation. The software and its activation can be seen in this link: https://www.cm-labs.com/vortex-studio/software/vortex-studio-academic-access/

### Steps to run the simulation
- open a terminal to run command: roscore
- start the CoppeliaSim and open the simulated Lilibot model at vrep_simulation/lilibot/lilibot-V5-P4.ttt of this project. 
- Click the run button at the toolbox of the CoppeliaSim.
- After click the button, the simulation is running. 


## Reference

Sun, T., Xiong, X., Dai, Z., Owaki, D., & Manoonpong, P. (2021). Robust and reusable self-organized locomotion under the adaptive physical and neural communications, IEEE Transactions on Cybernetics (under review).

If you have any questions/doubts  about how to implement this project on your computer, you are welcomed to raise issues and email to me. My email address is suntao.hn@gmail.com