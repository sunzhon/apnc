--[[
Callback function for reciving motor positions
from ROS controller node
--]]
-----------------------**********************--------------------------
function buttonClick_OpenMotor()
	sim.addStatusbarMessage('You clicked the button of OpenMotor')
	buttonClickState_OpenMotor = true
end
function buttonClick_CPGSType()
    sim.addStatusbarMessage('You clicked the button of CPGSType')
    buttonClickState_CPGSType = true
if(buttonClickState_CPGSType==true)
    then
if(controlParametersLeft[4]==0)
    then
    controlParametersLeft[4]=1
elseif(controlParametersLeft[4]==1)
    then
    controlParametersLeft[4]=2
elseif(controlParametersLeft[4]==2)
    then
    controlParametersLeft[4]=3
elseif(controlParametersLeft[4]==3)
    then
    controlParametersLeft[4]=4
elseif(controlParametersLeft[4]==4)
    then
    controlParametersLeft[4]=5
elseif(controlParametersLeft[4]==5)
    then
    controlParametersLeft[4]=6 --- set CPGSType equal to reflex
elseif(controlParametersLeft[4]==6)
    then 
    controlParametersLeft[4]=7
elseif(controlParametersLeft[4]==7) 
    then
    controlParametersLeft[4]=0
    end
    end
end
--[[
Function for handling slider changes on the QT UI
--]]
function sliderChange(ui,id,newVal)
    sliderChangeState=true;
	if id < 100
	 then	
        newVal = newVal/200
        simUI.setLabelText(ui,1000+id-10,'Value: '..newVal)
        controlParametersRight[id-9] = newVal
	else
        newVal = newVal/200
        simUI.setLabelText(ui,10000+id-100,'Value: '..newVal)
        controlParametersLeft[id-99] = newVal
	end
end


function setMotorPositions()
	data={}
	if (buttonClickState_OpenMotor == true)
	 then
   		for i = 1, #joint_array do
 	 		data[i] = CPGData[i] + reflexData[i]
		end

        for i = 1, #joint_array do
			simSetJointTargetPosition(joint_array[i], data[i])	
        end
	end
	
end


function setCPGMotorData(msg)
	CPGData=msg.data
end

function setReflexMotorData(msg)
	reflexData=msg.data
end

function graph_cb(msg)
    data = msg.data
    simSetGraphUserData(graphHandleRF,"CPGN0",data[1])
    simSetGraphUserData(graphHandleRF,"CPGN1",data[2])
    simSetGraphUserData(graphHandleRF,"PCPGN0",data[3])
    simSetGraphUserData(graphHandleRF,"PCPGN1",data[4])
    simSetGraphUserData(graphHandleRF,"PSN10",data[5])
    simSetGraphUserData(graphHandleRF,"PSN11",data[6])
    simSetGraphUserData(graphHandleRF,"VRNHip",data[7])
    simSetGraphUserData(graphHandleRF,"VRNKnee",data[8])
	simSetGraphUserData(graphHandleRF,"ReflexOut0",data[9])
    simSetGraphUserData(graphHandleRF,"ReflexOut1",data[10])
    simSetGraphUserData(graphHandleRF,"ReflexOut2",data[11])
    simSetGraphUserData(graphHandleRF,"GRF",data[12])

    simSetGraphUserData(graphHandleRH,"CPGN0",data[13])
    simSetGraphUserData(graphHandleRH,"CPGN1",data[14])
    simSetGraphUserData(graphHandleRH,"PCPGN0",data[15])
    simSetGraphUserData(graphHandleRH,"PCPGN1",data[16])
    simSetGraphUserData(graphHandleRH,"PSN10",data[17])
    simSetGraphUserData(graphHandleRH,"PSN11",data[17])
    simSetGraphUserData(graphHandleRH,"VRNHip",data[19])
    simSetGraphUserData(graphHandleRH,"VRNKnee",data[20])
	simSetGraphUserData(graphHandleRH,"ReflexOut0",data[21])
    simSetGraphUserData(graphHandleRH,"ReflexOut1",data[22])
    simSetGraphUserData(graphHandleRH,"ReflexOut2",data[23])
    simSetGraphUserData(graphHandleRH,"GRF",data[24])

    simSetGraphUserData(graphHandleLF,"CPGN0",data[25])
    simSetGraphUserData(graphHandleLF,"CPGN1",data[26])
    simSetGraphUserData(graphHandleLF,"PCPGN0",data[27])
    simSetGraphUserData(graphHandleLF,"PCPGN1",data[28])
    simSetGraphUserData(graphHandleLF,"PSN10",data[29])
    simSetGraphUserData(graphHandleLF,"PSN11",data[30])
    simSetGraphUserData(graphHandleLF,"VRNHip",data[31])
    simSetGraphUserData(graphHandleLF,"VRNKnee",data[32])
	simSetGraphUserData(graphHandleLF,"ReflexOut0",data[33])
    simSetGraphUserData(graphHandleLF,"ReflexOut1",data[34])
    simSetGraphUserData(graphHandleLF,"ReflexOut2",data[35])
    simSetGraphUserData(graphHandleLF,"GRF",data[36])

    simSetGraphUserData(graphHandleLH,"CPGN0",data[37])
    simSetGraphUserData(graphHandleLH,"CPGN1",data[38])
    simSetGraphUserData(graphHandleLH,"PCPGN0",data[39])
    simSetGraphUserData(graphHandleLH,"PCPGN1",data[40])
    simSetGraphUserData(graphHandleLH,"PSN10",data[41])
    simSetGraphUserData(graphHandleLH,"PSN11",data[42])
    simSetGraphUserData(graphHandleLH,"VRNHip",data[43])
    simSetGraphUserData(graphHandleLH,"VRNKnee",data[44])
	simSetGraphUserData(graphHandleLH,"ReflexOut0",data[45])
    simSetGraphUserData(graphHandleLH,"ReflexOut1",data[46])
    simSetGraphUserData(graphHandleLH,"ReflexOut2",data[47])
    simSetGraphUserData(graphHandleLH,"GRF",data[48])

end

--[[
Callback function for displaying data on a graph
resived from ROS controller node
--]]

function initParametersSlide()
 
	 -- User Interface setup
    xml = [[ <ui closeable="false" resizable="true" style="plastique" title="Control Right Legs ">
    <label text="Psn:" wordwrap="true" />
    <label text="Value: 0" id="1000" wordwrap="true" />
    <hslider id="10" tick-position="both-sides" tick-interval="1" minimum="-200" maximum="200" onchange="sliderChange" style="plastique" />

    <label text="VRNHip" wordwrap="true" />
    <label text="Value: 0" id="1001" wordwrap="true" />
    <hslider id="11" tick-position="both-sides" tick-interval="1" minimum="-200" maximum="200" onchange="sliderChange" style="plastique" />

    <label text="VRNKnee" wordwrap="true" />
    <label text="Value: 0" id="1002" wordwrap="true" />
    <hslider id="12" tick-position="both-sides" tick-interval="1" minimum="-200" maximum="200" onchange="sliderChange" style="plastique" />

    <label text="MNBias1" wordwrap="true" />
    <label text="Value: 0" id="1003" wordwrap="true" />
    <hslider id="13" tick-position="both-sides" tick-interval="1" minimum="-200" maximum="200" onchange="sliderChange" style="plastique" />
    
    <label text="MNBias2" wordwrap="true" />
    <label text="Value: 0" id="1004" wordwrap="true" />
    <hslider id="14" tick-position="both-sides" tick-interval="1" minimum="-200" maximum="200" onchange="sliderChange" style="plastique" />
    
    <label text="MNBias3" wordwrap="true" />
    <label text="Value: 0" id="1005" wordwrap="true" />
    <hslider id="15" tick-position="both-sides" tick-interval="1" minimum="-200" maximum="200" onchange="sliderChange" style="plastique" />
   
    <label text="CPGMi" wordwrap="true" />
    <label text="Value: 0" id="1006" wordwrap="true" />
    <hslider id="16" tick-position="both-sides" tick-interval="1" minimum="-200" maximum="200" onchange="sliderChange" style="plastique" />
  
    <label text="PCPGBeta" wordwrap="true" />
    <label text="Value: 0" id="1007" wordwrap="true" />
    <hslider id="17" tick-position="both-sides" tick-interval="1" minimum="-200" maximum="200" onchange="sliderChange" style="plastique" />
   
    <button text='Drive the Motor' on-click = "buttonClick_OpenMotor" />
	</ui> ]]
	buttonClickState_OpenMotor =false
    -- Create User Interface
    uiRight=simUI.create(xml)
	controlParametersRight={0.0,0.045,0.035, 0.0,-0.1,0.0, 0.02,0.8}--Psn,VrnKnee,VrnHip,bias1,bias2,bias3,NeuroNetworkMi,PNeuroNetworkbeta


    xml = [[ <ui closeable="false" resizable="true" style="plastique" title="Control Left Legs">
    <label text="Psn:" wordwrap="true" />
    <label text="Value: 0" id="10000" wordwrap="true" />
    <hslider id="100" tick-position="both-sides" tick-interval="1" minimum="-200" maximum="200" onchange="sliderChange" style="plastique" />

    <label text="VRNHip" wordwrap="true" />
    <label text="Value: 0" id="10001" wordwrap="true" />
    <hslider id="101" tick-position="both-sides" tick-interval="1" minimum="-200" maximum="200" onchange="sliderChange" style="plastique" />

    <label text="VRNKnee" wordwrap="true" />
    <label text="Value: 0" id="10002" wordwrap="true" />
    <hslider id="102" tick-position="both-sides" tick-interval="1" minimum="-200" maximum="200" onchange="sliderChange" style="plastique" />

    <button text='CPGSType' on-click = "buttonClick_CPGSType" />
	</ui> ]]
	buttonClickState_CPGSType =false
    sliderChangeState=false
    -- Create User Interface
    uiLeft=simUI.create(xml)
	controlParametersLeft={0.0, 0.045,0.035,5.0}--Psn,VrnKnee,VrnHip,CPGsType

	controlParameters={0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0}
    -- Initialize sliders and testparameters (these are shared with the ROS node)
    for i=1,#controlParametersRight do
	simExtCustomUI_setSliderValue(uiRight,9+i,controlParametersRight[i]*200)
    sliderChange(uiRight,9+i,controlParametersRight[i]*200)
    end
    for i=1,#controlParametersLeft-1 do
	simExtCustomUI_setSliderValue(uiLeft,99+i,controlParametersLeft[i]*200)
    sliderChange(uiLeft,99+i,controlParametersLeft[i]*200)
    end
	-- Set position of the User Interface
    x, y=simExtCustomUI_getPosition(uiRight)
    simExtCustomUI_setPosition(uiRight, x+800, y-300, true)
    x, y=simExtCustomUI_getPosition(uiLeft)
    simExtCustomUI_setPosition(uiLeft, x+600, y-300, true)

end

--[[
Initialization: Called once at the start of a simulation
--]]
function sysCall_init()
    -- init slide for adjust paramterts
    initParametersSlide() 
    -- Create all handles
    lilibotHandle=sim.getObjectAssociatedWithScript(sim.handle_self)
	
	FR_joint_1=sim.getObjectHandle("RF_joint1")
    FR_joint_2=sim.getObjectHandle("RF_joint2")
    FR_joint_3=sim.getObjectHandle("RF_joint3")

    RR_joint_1=sim.getObjectHandle("RH_joint1")
    RR_joint_2=sim.getObjectHandle("RH_joint2")
    RR_joint_3=sim.getObjectHandle("RH_joint3")
    
    FL_joint_1=sim.getObjectHandle("LF_joint1")
    FL_joint_2=sim.getObjectHandle("LF_joint2")
    FL_joint_3=sim.getObjectHandle("LF_joint3")

    RL_joint_1=sim.getObjectHandle("LH_joint1")
    RL_joint_2=sim.getObjectHandle("LH_joint2")
    RL_joint_3=sim.getObjectHandle("LH_joint3")
    
    joint_array={FR_joint_1, FR_joint_2, FR_joint_3, RR_joint_1, RR_joint_2, RR_joint_3, FL_joint_1, FL_joint_2, FL_joint_3, RL_joint_1, RL_joint_2, RL_joint_3}
	FR_foot_sensor=sim.getObjectHandle("RF_foot_sensor")
	FL_foot_sensor=sim.getObjectHandle("LF_foot_sensor")
	RR_foot_sensor=sim.getObjectHandle("RH_foot_sensor")
	RL_foot_sensor=sim.getObjectHandle("LH_foot_sensor")
	foot_sensor_array = { FR_foot_sensor, RR_foot_sensor, FL_foot_sensor,RL_foot_sensor }
	body = sim.getObjectHandle("body")
	for i = 1, #joint_array do
		print(sim.getObjectName(joint_array[i]))
	end
	
	for i = 1, #foot_sensor_array do
		print(sim.getObjectName(foot_sensor_array[i]))
	end

    graphHandleRF=sim.getObjectHandle("InspectableRF")
    graphHandleRH=sim.getObjectHandle("InspectableRH")
    graphHandleLF=sim.getObjectHandle("InspectableLF")
    graphHandleLH=sim.getObjectHandle("InspectableLH")
	-- Check if the required ROS plugin is loaded
    moduleName=0
    moduleVersion=0
    index=0
    pluginNotFound=true
    while moduleName do
        moduleName,moduleVersion=sim.getModuleName(index)
        if (moduleName=='RosInterface') then
            pluginNotFound=false
        end
        index=index+1
    end
    if (pluginNotFound) then
        sim.displayDialog('Error','The RosInterface was not found.',sim.dlgstyle_ok,false,nil,{0.8,0,0,0,0,0},{0.5,0,0,1,1,1})
    end

    -- If found then start the subscribers and publishers
    if (not pluginNotFound) then
        local motorName='MotorPositions'
        local simulationTimeName='simTime'
        local terminateControllerName='terminateController'
        local startSimulationName='startSimulation'
        local pauseSimulationName='pauseSimulation'
        local stopSimulationName='stopSimulation'
        local enableSyncModeName='enableSyncMode'
        local triggerNextStepName='pauseSimulation'
        local simulationStepDoneName='simulationStepDone'
        local simulationStateName='simulationState'
        local neuroNetworkOutputName='NeuroNetworkOutput' 
        local sensorValueName='sensorValues'
		local reflexMotorName= 'reflexMotors'
		local rosRateParameter ='50'
		local legNumber = '4'
		local motorNumber = '12'
		local sensorNumber ='23'

        -- Create the subscribers
        MotorSub=simROS.subscribe('/'..motorName,'std_msgs/Float32MultiArray','setCPGMotorData')
        NeuroNetworkOutputSub=simROS.subscribe('/'..neuroNetworkOutputName,'std_msgs/Float32MultiArray', 'graph_cb')

        -- Create the publishers
        terminateControllerPub=simROS.advertise('/'..terminateControllerName,'std_msgs/Bool')
        sensorValuePub=simROS.advertise('/'..sensorValueName,'std_msgs/Float32MultiArray')
        simulationTimePub=simROS.advertise('/'..simulationTimeName,'rosgraph_msgs/Clock')

        -- Start the client application (c++ node) for rythmic movement
        result=sim.launchExecutable(simGetStringParameter(sim_stringparam_scene_path) .. '/../../../../projects/stbot/genesis/catkin_ws/devel/lib/stbot/stbot_node', motorName.." "..simulationTimeName.." "..terminateControllerName.." "..startSimulationName.." "..pauseSimulationName.." "..stopSimulationName.." "..enableSyncModeName.." "..triggerNextStepName.." "..simulationStepDoneName.." "..simulationStateName.." "..neuroNetworkOutputName.." "..sensorValueName.." "..reflexMotorName.." "..rosRateParameter.." "..legNumber.." "..motorNumber.." "..sensorNumber,0)
        if (result==false) then
            sim.displayDialog('Error','External CPG ROS-Node not found',sim.dlgstyle_ok,false,nil,{0.8,0,0,0,0,0},{0.5,0,0,1,1,1})
        end
        -- Create the subscribers
        MotorSub=simROS.subscribe('/'..reflexMotorName,'std_msgs/Float32MultiArray','setReflexMotorData')
	-- Start the client application (python node)	for rapid reflex
        result=sim.launchExecutable(simGetStringParameter(sim_stringparam_scene_path) .. '/../../../../projects/stbot/genesis/catkin_ws/src/stbot/scripts/stbot_reflex2_node.py', motorName.." "..simulationTimeName.." "..terminateControllerName.." "..startSimulationName.." "..pauseSimulationName.." "..stopSimulationName.." "..enableSyncModeName.." "..triggerNextStepName.." "..simulationStepDoneName.." "..simulationStateName.." "..neuroNetworkOutputName.." "..sensorValueName.." "..reflexMotorName.." "..rosRateParameter.." "..legNumber.." "..motorNumber.." "..sensorNumber,0)
        if (result==false) then
            sim.displayDialog('Error','External reflex ROS-Node not found',sim.dlgstyle_ok,false,nil,{0.8,0,0,0,0,0},{0.5,0,0,1,1,1})
        end
--]]
    end


   	 -- Publish Initial Velocity and Position
	vel , pos = {}, {}
	sensor_array = {}
	for i = 1, #joint_array do
	pos[i] = simGetJointPosition(joint_array[i])
	sensor_array[i] = pos[i]
	end
    simROS.publish(sensorValuePub,{data=sensor_array})
    simROS.publish(simulationTimePub,{clock=simGetSimulationTime()})

	--Subscriber initial  data
	CPGData={}
    reflexData={}
	
	for i = 1, #joint_array do
	CPGData[i] = 0.0
	reflexData[i] = 0.0
	end

end


--[[
Actuation: This part will be executed in each simulation step
--]]
function sysCall_actuation()

    -- Publish Clock for Sync.

	setMotorPositions()
	
end

--[[
Sensing: This part will be executed in each simulation step
--]]
function sysCall_sensing()

	-- Publish sensor values----------------	

    -- Joint Velocity and Position, attitude angles of body ,and Foot contact force
	for i = 1, #joint_array do
	pos[i] = simGetJointPosition(joint_array[i])
	sensor_array[i] = pos[i]
	end	
	
    for i = 1, #foot_sensor_array do
    reuslt,force,torque = sim.readForceSensor(foot_sensor_array[i])
    sensor_array[i + #joint_array] = force[3]/10.0 --+ ((force[3]/10.0 > 0.1) and 0.5*math.random() or 0.0) --let the value into [0,1] --force[3]/10.0 
    end
    												-- please don't change this value,

    eularAngles=sim.getObjectOrientation(body,-1)
    for i = 1, #eularAngles do
	sensor_array[i + #joint_array + #foot_sensor_array]= eularAngles[i]
	end

	for i = 1, #foot_sensor_array do
	reuslt,force,torque = sim.readForceSensor(foot_sensor_array[i])
	sensor_array[i + #joint_array + #foot_sensor_array + #eularAngles] = force[1]/10.0	--let the value into [0,1]
	end												-- please don't change this value,
    simROS.publish(sensorValuePub,{data=sensor_array})

	-- publish clock
    simROS.publish(simulationTimePub,{clock=simGetSimulationTime()})

    -- set the parameters use parameters service
	for i=1,#controlParametersRight do
	controlParameters[i]=controlParametersRight[i]
	end
	for i=1,#controlParametersLeft do
	controlParameters[#controlParametersRight+i]=controlParametersLeft[i]
	end

    if(buttonClickState_CPGSType or sliderChangeState)
        then
        simROS.setParamDouble("PSN_right",controlParameters[1]);
        simROS.setParamDouble("VRN_hip_right",controlParameters[2]);
        simROS.setParamDouble("VRN_knee_right",controlParameters[3]);

        simROS.setParamDouble("MNB1",controlParameters[4]);
        simROS.setParamDouble("MNB2",controlParameters[5]);
        simROS.setParamDouble("MNB3",controlParameters[6]);

        simROS.setParamDouble("CPGMi",controlParameters[7]);
        simROS.setParamDouble("PCPGBeta",controlParameters[8]);

        simROS.setParamDouble("PSN_left",controlParameters[9]);
        simROS.setParamDouble("VRN_hip_left",controlParameters[10]);
        simROS.setParamDouble("VRN_knee_left",controlParameters[11]);
        simROS.setParamDouble("CPGSType",controlParameters[12]);
        print(controlParameters[12])
        buttonClickState_CPGSType=false
        sliderChangeState=false
    end
end

--[[
Sensing: This part will be executed one time just before a simulation ends
--]]
function sysCall_cleanup()
    
    -- Wait for the signal to arive at the nodes
    waitTimer=0
    while( waitTimer < 50000 ) do
        waitTimer = waitTimer+1
    end


    -- Clode ROS related stuff
    if not pluginNotFound then
        -- Send termination signal to external ROS nodes
        simROS.publish(terminateControllerPub,{data=true})

        -- Terminate subscriber
        simROS.shutdownSubscriber(NeuroNetworkOutputSub)
        simROS.shutdownSubscriber(MotorSub)
		-- terminate publisher
        simROS.shutdownPublisher(simulationTimePub)
        simROS.shutdownPublisher(sensorValuePub)
        simROS.shutdownPublisher(terminateControllerPub)
    end
	-- close serial port -----
--	sim.serialClose(SerialPort)
end

