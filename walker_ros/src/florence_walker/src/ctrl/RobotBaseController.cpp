#include "RobotBaseController.hpp"

using namespace std;
using namespace hardware_interface;

RobotBaseController::RobotBaseController(){
	ROS_INFO("Controller has been loaded!");
}

RobotBaseController::~RobotBaseController(){
	ROS_INFO("Controller will be unloaded!");
}

/* 
* function is declared virtual in controller_interface/controller_base and is called by controller manager
* ->get required interfaces and feed them to the init function
*/
bool RobotBaseController::initRequest(RobotHW*        robot_hw,
									ros::NodeHandle&  root_nh,
									ros::NodeHandle&  controller_nh,
									ClaimedResources& claimed_resources)
{
	// initialise the visualisation node (controller is a standalone plugin, so direct initialisation of member classes is not possible)
	this->fvn.init(controller_nh,&(this->mtx));

	// Check if construction finished cleanly
	if (state_ != CONSTRUCTED)
	{
		ROS_ERROR("Cannot initialize this controller because it failed to be constructed");
		return false;
	}

	// Get a pointer to the joint state measurement interface
	JointStateInterface* jnt_state_interface = robot_hw->get<JointStateInterface>();
	if (!jnt_state_interface)
	{
		ROS_ERROR("This controller requires a hardware interface of type '%s'."
		          " Make sure this is registered in the RobotHW"
		          " class.",
		          internal::demangledTypeName<JointStateInterface>().c_str());
		return false;
	}

	// Get a pointer to the load cell sensor interface
	LoadCellArrayInterface* fsen_interface = robot_hw->get<LoadCellArrayInterface>();

	if (!fsen_interface)
	{
		ROS_ERROR("This controller requires a hardware interface of type '%s'."
	          " Make sure this is registered in the RobotHW"
	          " class.",
	          internal::demangledTypeName<LoadCellArrayInterface>().c_str());
		return false;
	}

	// Get a pointer to the IMU sensor interface
	ImuSensorInterface* imu_sen_interface = robot_hw->get<ImuSensorInterface>();
	if (!imu_sen_interface)
	{
		ROS_ERROR("This controller requires a hardware interface of type '%s'."
	          " Make sure this is registered in the RobotHW"
	          " class.",
	          internal::demangledTypeName<ImuSensorInterface>().c_str());
		return false;
	}

	// Get a pointer to the joint position control interface
	PositionJointInterface* pos_interface = robot_hw->get<PositionJointInterface>();
	if (!pos_interface)
	{
		ROS_ERROR("This controller requires a hardware interface of type '%s'."
		          " Make sure this is registered in the RobotHW"
		          " class.",
		          internal::demangledTypeName<PositionJointInterface>().c_str());
		return false;
	}

	// Get a pointer to the joint velocity control interface
	VelocityJointInterface* vel_interface = robot_hw->get<VelocityJointInterface>();
	if (!vel_interface)
	{
		ROS_ERROR("This controller requires a hardware interface of type '%s'."
		          " Make sure this is registered in the RobotHW"
		          " class.",
		          internal::demangledTypeName<VelocityJointInterface>().c_str());
		return false;
	}

	// Get a pointer to the joint effort control interface
	EffortJointInterface* eff_interface = robot_hw->get<EffortJointInterface>();
	if (!eff_interface)
	{
		ROS_ERROR("This controller requires a hardware interface of type '%s'."
		          " Make sure this is registered in the RobotHW"
		          " class.",
		          internal::demangledTypeName<EffortJointInterface>().c_str());
		return false;
	}

	// Return which resources are claimed by this controller
	pos_interface->clearClaims();
	vel_interface->clearClaims();
	eff_interface->clearClaims();

	if (!init(jnt_state_interface,
        	fsen_interface,
	        imu_sen_interface,
	        pos_interface,
	        vel_interface,
	        eff_interface,
	        root_nh,
	        controller_nh))
	{
		ROS_ERROR("Failed to initialize the controller! Problem with resources!");
		return false;
	}

	claimed_resources.push_back(
	    InterfaceResources(internal::demangledTypeName<PositionJointInterface>(),
	                       pos_interface->getClaims()));
	claimed_resources.push_back(
	    InterfaceResources(internal::demangledTypeName<VelocityJointInterface>(),
	                       vel_interface->getClaims()));
	claimed_resources.push_back(
	    InterfaceResources(internal::demangledTypeName<EffortJointInterface>(),
	                       eff_interface->getClaims()));

	pos_interface->clearClaims();
	vel_interface->clearClaims();
	eff_interface->clearClaims();

	// resize vectors
	this->q_m.resize(joint_names.size());
	this->qp_m.resize(joint_names.size());
	this->qeff_m.resize(joint_names.size());
	this->q_c.resize(joint_names.size());
	this->qp_c.resize(joint_names.size());
	this->qeff_c.resize(joint_names.size());
	this->accel.resize(2);
	this->gyro.resize(2);
	this->fsen.resize(2);
	
	// success
	ROS_INFO_STREAM("Function initRequest has been executed");
	state_ = INITIALIZED;
	return true;
}

// get handles for all provided interfarces
bool RobotBaseController::init(JointStateInterface*  jnt_state_interface,
				     	  	LoadCellArrayInterface* fsen_interface,
				          	ImuSensorInterface*     imu_sen_interface,
				          	PositionJointInterface* pos_interface,
				          	VelocityJointInterface* vel_interface,
				          	EffortJointInterface*   eff_interface,
				          	ros::NodeHandle&        root_nh,
							ros::NodeHandle&        controller_nh)
{
	if (!initMotorInputs(jnt_state_interface, root_nh)	||
		!initForceSensors(fsen_interface, root_nh)		||
		!initImuSensors(imu_sen_interface, root_nh)		|| 
		!initMotorOutputs(pos_interface, vel_interface, eff_interface, root_nh))
	{
		ROS_ERROR_STREAM("Failed to initialize controller '"
						<< internal::demangledTypeName(*this) << "'");
		return false;
	}
	return true;
}

// get handles for the read only joint state interface (joint measurements)
bool RobotBaseController::initMotorInputs(JointStateInterface* jnt_state_interface, ros::NodeHandle& nh)
{
	if(!nh.getParam("/hw/joint/names",this->joint_names)){
		ROS_ERROR("Joint names not found.");
		return false;
	}

	// Retrieve joint state handles from the hardware interface RobotHW
	for(string jnt_state_element : this->joint_names){
		try
		{
			this->hiJointStates.push_back(jnt_state_interface->getHandle(jnt_state_element));
			q_m.push_back(0.0);
			qp_m.push_back(0.0);
			qeff_m.push_back(0.0);
			ROS_DEBUG_STREAM("Found joint state '" << jnt_state_element << "' in '" << internal::demangledTypeName(*jnt_state_interface) << "'");
		}
		catch (...)
		{
			ROS_ERROR("ERROR WITH MOTOR INPUTS");
			ROS_ERROR_STREAM("Could not find joint state '" << jnt_state_element << "' in '" << internal::demangledTypeName(*jnt_state_interface) << "'");
			return false;
		}
	}
	return true;
}

// get handles for the force sensor interface
bool RobotBaseController::initForceSensors(LoadCellArrayInterface* fsen_interface, ros::NodeHandle& nh)
{
	if(!nh.getParam("/hw/fsen/names",this->fsen_names)){
		ROS_ERROR("Force sensor names not found.");
		return false;
	}

	// there are two force cell arrays
	for(string fs_element : this->fsen_names){
		try
		{
			this->hiFtSensors.push_back(fsen_interface->getHandle(fs_element));
			ROS_DEBUG_STREAM("Found load cell array sensor '" << fs_element << "' in '" << internal::demangledTypeName(*fsen_interface) << "'");
		}
		catch (...)
		{
			ROS_ERROR("ERROR WITH FORCE SENSORS");
			ROS_ERROR_STREAM("Could not load cell array sensor '" << fs_element << "' in '" << internal::demangledTypeName(*fsen_interface) << "'");
			return false;
		}
	}
	return true;
}

// get handles for the IMU sensor interface
bool RobotBaseController::initImuSensors(ImuSensorInterface* imu_sen_interface, ros::NodeHandle& nh)
{
	// Obtain IMU handle names
	if(!this->joint_names.size()){
		ROS_ERROR("Joint names were not populated yet. IMU names can not be resolved");
		return false;
	}else{
		if(!nh.getParam("/hw/imu/used",this->used_imu)){
			ROS_ERROR("The indices of imu's that are used were not found.");
			return false;
		}
	}

	// only two IMUs are used
	this->imu_names.resize(2);
	//left side
	this->imu_names[0]=this->joint_names[this->used_imu[0]];
	this->imu_names[0].append("IMU"); 
	//right side	
	this->imu_names[1]=this->joint_names[this->used_imu[1]];
	this->imu_names[1].append("IMU"); 

	// Retrieve IMU handles from the hardware interface RobotHW
	for(string imu_element : this->imu_names){
		try
		{
			this->hiImus.push_back(imu_sen_interface->getHandle(imu_element));
			ROS_DEBUG_STREAM("Found IMU sensor '" << imu_element << "' in '" << internal::demangledTypeName(*imu_sen_interface) << "'");
		}
		catch (...)
		{
			ROS_ERROR("ERROR WITH IMU SENSORS");
			ROS_ERROR_STREAM("Could not find IMU sensor '" << imu_element << "' in '" << internal::demangledTypeName(*imu_sen_interface) << "'");
			return false;
		}
	}
	return true;
}

// get handles for all 3 joint output interfaces (joint commands)
bool RobotBaseController::initMotorOutputs(PositionJointInterface* pos_interface,
				          				VelocityJointInterface* vel_interface,
				          				EffortJointInterface*   eff_interface,
				          				ros::NodeHandle& nh)
{
	if(!this->joint_names.size()){
		ROS_ERROR("Joint names were not populated yet. Control handles can not be resolved.");
		return false;
	}

	for(string jnt_element : this->joint_names){
		try
		{
			this->hiJointPos.push_back(pos_interface->getHandle(jnt_element));
			ROS_DEBUG_STREAM("Found joint position handle for '" << jnt_element << "' in '" << internal::demangledTypeName(*pos_interface) << "'");
		}
		catch (...)
		{
			ROS_ERROR("ERROR WITH MOTOR OUTPUTS (POSITIONS)");
			ROS_ERROR_STREAM("Could not find joint position handle for '" << jnt_element << "' in '" << internal::demangledTypeName(*pos_interface) << "'");
			return false;
		}

		try
		{
			this->hiJointVel.push_back(vel_interface->getHandle(jnt_element));
			ROS_DEBUG_STREAM("Found joint velocity handle for '" << jnt_element << "' in '" << internal::demangledTypeName(*vel_interface) << "'");
		}
		catch (...)
		{
			ROS_ERROR("ERROR WITH MOTOR OUTPUTS (VELOCITIES)");
			ROS_ERROR_STREAM("Could not find joint velocity handle for '" << jnt_element << "' in '" << internal::demangledTypeName(*vel_interface) << "'");
			return false;
		}

		try
		{
			this->hiJointEff.push_back(eff_interface->getHandle(jnt_element));
			ROS_DEBUG_STREAM("Found joint effort handle for '" << jnt_element << "' in '" << internal::demangledTypeName(*eff_interface) << "'");
		}
		catch (...)
		{
			ROS_ERROR("ERROR WITH MOTOR OUTPUTS (EFFORTS)");
			ROS_ERROR_STREAM("Could not find joint effort handle for '" << jnt_element << "' in '" << internal::demangledTypeName(*eff_interface) << "'");
			return false;
		}
	}
	return true;
}

// controller is first loaded, then started (this function is called when the controller is starting)
void RobotBaseController::starting(const ros::Time& time)
{
	ROS_INFO_STREAM("Starting the RobotBaseController: " << time.toSec());
	// start visualising (activate the VisNode member class)
	this->fvn.startPubThread();	
	// start the additional routines from the class extending this one  (extending class overrides it)
	this->startController();	
	// Make sure that controllers don't send steps to the robot at the beginning (use a flag)
	this->readMeasurements();
	this->initCmds();
}

// called at the time when ControllerManager.update is called (check florence_controller.cpp)
void RobotBaseController::update(const ros::Time& time, const ros::Duration& period)
{  
	// get measurements from the shared resources provided by the hardware interface
	this->readMeasurements();
	// extending class overrides this function
	this->algorithmStep(period);
	// write commands back to the shared resources provided by the hardware interface
	this->sendCommands();
}

// controller is first stopped, then unloaded (this function is called when the controller is stopping)
void RobotBaseController::stopping(const ros::Time& time)
{
	ROS_INFO_STREAM("stopping the RobotBaseController: " << time.toSec());
	// stop visualising
	this->fvn.stopPubThread();
	// stop the additional routines from the class extending this one (extending class overrides it)
	this->stopController();
}

// make sure that robot does not receive false commands due to improper initialization
void RobotBaseController::initCmds()
{
	// copy the measured position into the command at the beginning (afterwards the inheriting class has to make sure that valid values are sent)
	//ROS_INFO("Robot base controller: Init. the cmds");	
	std::vector<double> q_m = this->getMeasPos();
	for(int i=0; i<q_m.size(); i++){
		this->setCmdPos(i, this->hiJointPos.at(i).getCommand());
	}
}

// get commands from their respective handles (shared resources)
void RobotBaseController::readMeasurements()
{
	// motor joint measurements
	for(int i=0; i<this->hiJointStates.size(); i++)
	{
		this->q_m.at(i) = this->hiJointStates.at(i).getPosition();
		this->qp_m.at(i) = this->hiJointStates.at(i).getVelocity();
		this->qeff_m.at(i) = this->hiJointStates.at(i).getEffort();
	}	

	// IMU measurements
	const double * accel; //constant value, but not a constant pointer
	const double * gyro; //constant value, but not a constant pointer
	for(int i=0; i<this->hiImus.size(); i++)
	{
		accel = this->hiImus[i].getLinearAcceleration();
		gyro = this->hiImus[i].getAngularVelocity();
		for(int j=0; j<3; j++){
			this->accel.at(i)[j] = accel[j];
			this->gyro.at(i)[j] = gyro[j];
		}
	}

	// Force sensor measurments
	const double * fsen; //constant value, but not a constant pointer
	for(int i=0; i<this->hiFtSensors.size(); i++)
	{
		fsen = this->hiFtSensors[i].getForceMeas();
		for(int j=0; j<4; j++){
			this->fsen.at(i)[j] = fsen[j];
		}
	}
}

// write commands into their respective handles (shared resources)
void RobotBaseController::sendCommands()
{
	for(int i=0; i<this->hiJointPos.size(); i++)
	{
		this->hiJointPos.at(i).setCommand(q_c.at(i));
		this->hiJointVel.at(i).setCommand(qp_c.at(i));
		this->hiJointEff.at(i).setCommand(qeff_c.at(i));
	}
}

// copy values for visualization
void RobotBaseController::visualise(std::vector<double> q_m,
					std::vector<double> qp_m, 
					std::vector<double> qeff_m, 
					std::vector<double> q_c,
					std::vector<double> qp_c,
					std::vector<double> qeff_c,
					std::vector<Vector4d> fs,
					Targets meas,
					Targets cmd,
					uint8_t rstatus)
{
	// copy states to the VisNode and let it publish them on a separate thread
	this->fvn.setStates(q_m,qp_m,qeff_m,q_c,qp_c,qeff_c,fs,meas,cmd,rstatus);
}