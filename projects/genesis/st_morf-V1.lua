--[[
Callback function for reciving motor positions
from ROS controller node
--]]
-----------------------**********************--------------------------
function buttonClick()
	sim.addStatusbarMessage('You clicked the button')
	buttonClickState = true
end

function setMotorPositions()
	data={}
	if (buttonClickState == true)
	 then
        for i = 1 , #joint_array do
            data[i] = CPGData[i]
        end
        for i = 1, 16, 3 do
            data[i] = data[i+1]
            data[i+1] = data[i+2]
            data[i+2] = data[i+2]*0.6 -0.80
        end
        data[10] = -data[10]-- left motor direction is inverse
        data[13] = -data[13]
        data[16] = -data[16]
   		for i = 1 , #joint_array do
			simSetJointTargetPosition(joint_array[i], data[i])	
		end
	end
	
end


function setCPGMotorData(msg)
	CPGData=msg.data
end


function graph_cb(msg)
    data = msg.data
    simSetGraphUserData(graphHandleNeuroNetworkCPG,"CPGN0",data[1])
    simSetGraphUserData(graphHandleNeuroNetworkCPG,"CPGN1",data[2])
    simSetGraphUserData(graphHandleNeuroNetworkCPG,"PCPGN0",data[3])
    simSetGraphUserData(graphHandleNeuroNetworkCPG,"PCPGN1",data[4])
    simSetGraphUserData(graphHandleNeuroNetworkCPG,"PSN10",data[5])
    simSetGraphUserData(graphHandleNeuroNetworkCPG,"PSN11",data[6])
    simSetGraphUserData(graphHandleNeuroNetworkCPG,"VRNHip",data[7])
    simSetGraphUserData(graphHandleNeuroNetworkCPG,"VRNKnee",data[8])
	simSetGraphUserData(graphHandleNeuroNetworkCPG,"ReflexOut0",data[9])
    simSetGraphUserData(graphHandleNeuroNetworkCPG,"ReflexOut1",data[10])
    simSetGraphUserData(graphHandleNeuroNetworkCPG,"ReflexOut2",data[11])
    simSetGraphUserData(graphHandleNeuroNetworkCPG,"GRF",data[12])
end


--[[
Callback function for displaying data on a graph
resived from ROS controller node
--]]

--[[
Function for handling slider changes on the QT UI
--]]
function sliderChange(ui,id,newVal)
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
      <button text='Drive the Motor' on-click = "buttonClick" />
	</ui> ]]
	buttonClickState =false
    -- Create User Interface
    uiRight=simUI.create(xml)
	controlParametersRight={0.0,0.07,0.06, 0.0,0.3,-0.4, 0.02,0.0}--Psn,VrnKnee,VrnHip,bias1,bias2,bias3,NeuroNetworkMi,PNeuroNetworkbeta


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
	</ui> ]]
    -- Create User Interface
    uiLeft=simUI.create(xml)
	controlParametersLeft={0.0,0.07,0.06}--Psn,VrnKnee,VrnHip,

	controlParameters={0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0}
    -- Initialize sliders and testparameters (these are shared with the ROS node)
    for i=1,#controlParametersRight do
	simExtCustomUI_setSliderValue(uiRight,9+i,controlParametersRight[i]*200)
    sliderChange(uiRight,9+i,controlParametersRight[i]*200)
    end
    for i=1,#controlParametersLeft do
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
	
    TC_motor0=sim.getObjectHandle("TC0")    -- Handle of the TC motor
    TC_motor1=sim.getObjectHandle("TC1")    -- Handle of the TC motor
    TC_motor2=sim.getObjectHandle("TC2")    -- Handle of the TC motor
    TC_motor3=sim.getObjectHandle("TC3")    -- Handle of the TC motor
    TC_motor4=sim.getObjectHandle("TC4")    -- Handle of the TC motor
    TC_motor5=sim.getObjectHandle("TC5")    -- Handle of the TC motor

    CF_motor0=sim.getObjectHandle("CF0")    -- Handle of the CF motor
    CF_motor1=sim.getObjectHandle("CF1")    -- Handle of the CF motor
    CF_motor2=sim.getObjectHandle("CF2")    -- Handle of the CF motor
    CF_motor3=sim.getObjectHandle("CF3")    -- Handle of the CF motor
    CF_motor4=sim.getObjectHandle("CF4")    -- Handle of the CF motor
    CF_motor5=sim.getObjectHandle("CF5")    -- Handle of the CF motor

    FT_motor0=sim.getObjectHandle("FT0")    -- Handle of the FT motor
    FT_motor1=sim.getObjectHandle("FT1")    -- Handle of the FT motor
    FT_motor2=sim.getObjectHandle("FT2")    -- Handle of the FT motor
    FT_motor3=sim.getObjectHandle("FT3")    -- Handle of the FT motor
    FT_motor4=sim.getObjectHandle("FT4")    -- Handle of the FT motor
    FT_motor5=sim.getObjectHandle("FT5")    -- Handle of the FT motor

    joint_array={TC_motor3, CF_motor3, FT_motor3, TC_motor4, CF_motor4, FT_motor4, TC_motor5, CF_motor5, FT_motor5, TC_motor0, CF_motor0, FT_motor0, TC_motor1, CF_motor1, FT_motor1, TC_motor2, CF_motor2, FT_motor2}
	RF_foot_sensor=sim.getObjectHandle("3D_force3")
	RM_foot_sensor=sim.getObjectHandle("3D_force4")
	RH_foot_sensor=sim.getObjectHandle("3D_force5")
	LF_foot_sensor=sim.getObjectHandle("3D_force0")
	LM_foot_sensor=sim.getObjectHandle("3D_force1")
	LH_foot_sensor=sim.getObjectHandle("3D_force2")
	foot_sensor_array = {RF_foot_sensor, RM_foot_sensor, RH_foot_sensor,LF_foot_sensor, LM_foot_sensor, LH_foot_sensor }
	body = sim.getObjectHandle("morf_vis")
	for i = 1, #joint_array do
		print(sim.getObjectName(joint_array[i]))
	end
	
	for i = 1, #foot_sensor_array do
		print(sim.getObjectName(foot_sensor_array[i]))
	end

    graphHandleNeuroNetworkCPG=sim.getObjectHandle("NeuroNetworkGraphCPG")
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
		local rosRateParameter ='100'
		local legNumber = '6'
		local motorNumber = '18'
		local sensorNumber ='33'

        -- Create the subscribers
        MotorSub=simROS.subscribe('/'..motorName,'std_msgs/Float32MultiArray','setCPGMotorData')
        NeuroNetworkOutputSub=simROS.subscribe('/'..neuroNetworkOutputName,'std_msgs/Float32MultiArray', 'graph_cb')

        -- Create the publishers
        terminateControllerPub=simROS.advertise('/'..terminateControllerName,'std_msgs/Bool')
        sensorValuePub=simROS.advertise('/'..sensorValueName,'std_msgs/Float32MultiArray')
        simulationTimePub=simROS.advertise('/'..simulationTimeName,'rosgraph_msgs/Clock')

        -- Start the client application (c++ node)
        result=sim.launchExecutable(simGetStringParameter(sim_stringparam_scene_path) .. '/../../../../projects/stbot/genesis/catkin_ws/devel/lib/stbot/stbot_node', motorName.." "..simulationTimeName.." "..terminateControllerName.." "..startSimulationName.." "..pauseSimulationName.." "..stopSimulationName.." "..enableSyncModeName.." "..triggerNextStepName.." "..simulationStepDoneName.." "..simulationStateName.." "..neuroNetworkOutputName.." "..sensorValueName.." "..reflexMotorName.." "..rosRateParameter.." "..legNumber.." "..motorNumber.." "..sensorNumber,0)
        if (result==false) then
            sim.displayDialog('Error','External CPG ROS-Node not found',sim.dlgstyle_ok,false,nil,{0.8,0,0,0,0,0},{0.5,0,0,1,1,1})
        end

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
	
	for i = 1, #joint_array do
	CPGData[i] = 0.0
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
	sensor_array[i + #joint_array] = force[3]/20.0	--let the value into [0,1]
	end												-- please don't change this value,

	eularAngles=sim.getObjectOrientation(body,-1)
	for i = 1, #eularAngles do
	sensor_array[i + #joint_array + #foot_sensor_array]= eularAngles[i]
	end

	for i = 1, #foot_sensor_array do
	reuslt,force,torque = sim.readForceSensor(foot_sensor_array[i])
	sensor_array[i + #joint_array + #foot_sensor_array + #eularAngles] = force[1]/300.0	--let the value into [0,1]
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

	simROS.setParamDouble("PSN_right",controlParameters[1]);
	simROS.setParamDouble("VRN_knee_right",controlParameters[2]);
	simROS.setParamDouble("VRN_hip_right",controlParameters[3]);

	simROS.setParamDouble("MNB1",controlParameters[4]);
	simROS.setParamDouble("MNB2",controlParameters[5]);
	simROS.setParamDouble("MNB3",controlParameters[6]);

	simROS.setParamDouble("CPGMi",controlParameters[7]);
	simROS.setParamDouble("PCPGBeta",controlParameters[8]);

	simROS.setParamDouble("PSN_left",controlParameters[9]);
	simROS.setParamDouble("VRN_knee_left",controlParameters[10]);
	simROS.setParamDouble("VRN_hip_left",controlParameters[11]);
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

