#include "RobotInterface.hpp"

RobotInterface::RobotInterface(ros::NodeHandle &nh)
  :lookup(), mot_group_fbk(NMOT), fs_group_fbk(NFS), mot_group_cmd(NMOT), last_time(ros::Time())
{ 
	// store the reference of the node handle
	this->nh = nh;
	// read required parameters from the parameter cloud
	if(!this->readParameters()){
		ROS_INFO("Parameters successfully read from the param-cloud.");
		// compute mapping matrices (inputs (16dof) -> states (12dof) -> outputs (16dof))
		this->computeMapMatrices();

		// list all the available modules
		this->HEBI_lookupEntries();
		// connect to modules
		this->HEBI_init();
		// read the initial angle values and initialise commands - do that before registering handles to avoid violation
		this->read();
		for(int i=0; i<NDOF; i++){
			this->cmd.pos[i] = this->meas.pos[i];
		}
		// link hardware resources to the class owned registers (interface setup)
		this->init();
	}
}	

bool RobotInterface::readParameters(){
	bool err = false;

	if(!this->nh.getParam("/ctrl/full_controller/publish_rate",this->Fctrl)){
		ROS_ERROR("Control frequency not found.");
		err = true;
	}

	if(!this->nh.getParam("/hw/fam_name",this->family_name)){
		ROS_ERROR("Family name was not found.");
		err = true;
	}
	
	if(!this->nh.getParam("/hw/joint/names",this->joint_names)){
		ROS_ERROR("Joint names were not found.");
		err = true;
	}else{
		//construct imu names (one IMU corresponds to 1 motor)
		for(std::string jname: this->joint_names){
			this->imu_names.push_back(jname.append("IMU"));
		}
	}

	std::vector<double> joffs_deg;
	if(!this->nh.getParam("/hw/joint/offs",joffs_deg)){
		ROS_ERROR("The indices of imu's that are to be used were not found.");
		err = true;
	}else{
		this->joint_offs.resize(joffs_deg.size());
		for(int i=0; i<joffs_deg.size(); i++){
			this->joint_offs[i] = joffs_deg[i]*M_PI/180.0;
		}
	}

	if(!this->nh.getParam("/hw/imu/used",this->used_imu)){
		ROS_ERROR("The indices of imu's that are to be used were not found.");
		err = true;
	}

	if(!this->nh.getParam("/hw/motor/names",this->motor_names)){
		ROS_ERROR("Motor names were not found.");
		err = true;
	}

	if(!this->nh.getParam("/hw/motor/map", this->motor_map)){
		ROS_ERROR("Motor map was not found.");
		err = true;
	}

	if(!this->nh.getParam("/hw/motor/coef", this->motor_coef)){
		ROS_ERROR("Motor coefficients were not found.");
		err = true;
	}

	if(!this->nh.getParam("/hw/fsen/names",this->fs_names)){
		ROS_ERROR("Force sensor names were not found.");
		err = true;
	}

	if(!this->nh.getParam("/hw/fsen/pins",this->fs_ind)){
		ROS_ERROR("Used load cell analog pins not found.");
		err = true;
	}

	std::vector<double> load_cell_offs;
	if(!this->nh.getParam("/hw/fsen/offs",load_cell_offs)){
		ROS_ERROR("Load cell offsets not found.");
		err = true;
	}else{
		int isize = load_cell_offs.size()/2;
		for(int i=0; i<isize;i++){
			this->fs_left.measOffs[i] = load_cell_offs[i];
			this->fs_right.measOffs[i] = load_cell_offs[isize+i];
		}
	}

	std::vector<double> joint_offs_comp;
	if(!this->nh.getParam("/hw/joint/comp",joint_offs_comp)){
		ROS_ERROR("Load cell offsets not found.");
		err = true;
	}else{
		for(int i=0; i<joint_offs_comp.size();i++){
			this->joint_comp[i] = joint_offs_comp[i];
		}
	}

	if(!this->nh.getParam("/hw/motor/gains/Kp",this->motor_gains_Kp)){
		ROS_ERROR("Proportional controller gains not found.");
		err = true;
	}else{
		this->motor_gains_Kp.insert(this->motor_gains_Kp.end(),this->motor_gains_Kp.begin(),this->motor_gains_Kp.end());
	}

	if(!this->nh.getParam("/hw/motor/gains/Kd",motor_gains_Kd)){
		ROS_ERROR("Differential controller gains not found.");
		err = true;
	}else{
		this->motor_gains_Kd.insert(this->motor_gains_Kd.end(),this->motor_gains_Kd.begin(),this->motor_gains_Kd.end());
	}

	if(!this->nh.getParam("/hw/motor/gains/Ki",motor_gains_Ki)){
		ROS_ERROR("Integral controller gains not found.");
		err = true;
	}else{
		this->motor_gains_Ki.insert(this->motor_gains_Ki.end(),this->motor_gains_Ki.begin(),this->motor_gains_Ki.end());
	}


	return err;
}

ros::Time RobotInterface::get_time(){
	return ros::Time::now();
}

ros::Duration RobotInterface::get_period(){
	return this->get_time() - this->last_time;
}

// create handles, register them to their respective interface and then register all interfaces
void RobotInterface::init(){
	// connect and register the joint state interface (MEASUREMENTS)
	int joint_size = this->joint_names.size();
	if (joint_size==NDOF){
		for(int i=0; i<joint_size; i++){
			// reading states
			hardware_interface::JointStateHandle state_handle(this->joint_names[i], &(this->meas.pos[i]), &(this->meas.vel[i]), &(this->meas.eff[i]));
			this->jnt_state_interface.registerHandle(state_handle);
		}
		registerInterface(&(this->jnt_state_interface));

		// create imu handles - IMUs are a part of a motor (hence detection of motors is a criterion here as well)
		hardware_interface::ImuSensorHandle limu_handle(
			this->imu_names[this->used_imu[0]],
			this->motor_names[this->used_imu[0]],
			(this->lbase.ori),
			(this->lbase.ori_cov),
			(this->lbase.gyro),
			(this->lbase.gyro_cov),
			(this->lbase.acc),
			(this->lbase.acc_cov)
		);

		hardware_interface::ImuSensorHandle rimu_handle(
			this->imu_names[this->used_imu[1]],
			this->motor_names[this->used_imu[1]],
			(this->rbase.ori),
			(this->rbase.ori_cov),
			(this->rbase.gyro),
			(this->rbase.gyro_cov),
			(this->rbase.acc),
			(this->rbase.acc_cov)
		);

		// register IMU handles
		this->imu_sen_interface.registerHandle(limu_handle);
		this->imu_sen_interface.registerHandle(rimu_handle);
		// register IMU interface
		registerInterface(&(this->imu_sen_interface));
	}

	//connect and register force sensor interface (MEASUREMENTS)
	int fs_size = this->fs_group->size();
	if (fs_size == NFS){
		// create force sensor handles
		hardware_interface::LoadCellArrayHandle lforce_handle(this->fs_names[0],std::string(""),this->fs_left.measForce);
		hardware_interface::LoadCellArrayHandle rforce_handle(this->fs_names[1],std::string(""),this->fs_right.measForce);
		// register force sensor handles
		this->fsen_interface.registerHandle(lforce_handle);
		this->fsen_interface.registerHandle(rforce_handle);
		// register force sensor interface
		registerInterface(&(this->fsen_interface));
	}


	
	// connect and register the joint position, velocity and effort interfaces (CONTROLS)
	if (joint_size==NDOF){
		for(int i=0; i<joint_size; i++){
			hardware_interface::JointStateHandle state_handle = this->jnt_state_interface.getHandle(this->joint_names[i]);
			// control of actuators -position
			hardware_interface::JointHandle pos_ctrl_handle(state_handle,&(this->cmd.pos[i]));
			this->jnt_pos_interface.registerHandle(pos_ctrl_handle);
			// control of actuators -velocity
			hardware_interface::JointHandle vel_ctrl_handle(state_handle,&(this->cmd.vel[i]));
			this->jnt_vel_interface.registerHandle(vel_ctrl_handle);
			// control of actuators -effort
			hardware_interface::JointHandle eff_ctrl_handle(state_handle,&(this->cmd.eff[i]));
			this->jnt_eff_interface.registerHandle(eff_ctrl_handle);				
		}
		registerInterface(&(this->jnt_pos_interface));
		registerInterface(&(this->jnt_vel_interface));
		registerInterface(&(this->jnt_eff_interface));
	}
}

/*
* This following function had to be overriden. Explaination:
* Controller computes 3 values for each joint (pos, vel, eff)which get aggregated inside of the
* actual HW module. In ROS-control framework all the 3 values are associated to the same joint, which
* causes a reource management conflict. This function namely does not check actual resources 
* (pointers to registers), but rather only resource names, which are the same for all 3 HW control 
* interfaces (reference to the same joint). The framework makes it difficult to compare pointers as control
* handles are more difficult to access than state handles (read only). Therefore, for now, this function 
* always returns false. 
*/
bool RobotInterface::checkForConflict(const std::list<hardware_interface::ControllerInfo> &info) const{
    // original fcn: https://github.com/ros-controls/ros_control/blob/noetic-devel/hardware_interface/include/hardware_interface/robot_hw.h
    return false;
}

// read values from hardware and write them into registers
void RobotInterface::read(){

	this->HEBI_motFb();
	this->HEBI_fsFb();
	this->HEBI_motFbMap();
	this->HEBI_mapIMU();
}

// write commands to hardware
void RobotInterface::write(ros::Duration elapsed_time){
	//write joint angles/velocities/efforts/gains
	this->HEBI_motCmdMap();

	this->HEBI_sendCmd();

	this->last_time = this->get_time();
}

void RobotInterface::HEBI_lookupEntries(){
	// Wait 2 seconds for the module list to populate, and then print out its contents
	std::this_thread::sleep_for(std::chrono::milliseconds(2000));

	// Entries of the lookup
	std::cout << "Get all the entries: " << std::endl;
	auto entry_list = this->lookup.getEntryList();
	for (auto entry : *entry_list)
		std::cout
			<< "Name: " << entry.name_ << std::endl
			<< "Family: " << entry.family_ << std::endl << std::endl;
}

// initialise HEBI variables
void RobotInterface::HEBI_init(){
	// get the group of motors by specifying the family name and module names
	this->mot_group = this->lookup.getGroupFromNames({this->family_name},this->motor_names);
	if (this->mot_group!=NULL){
		ROS_INFO("All motors were successfully detected.");
		auto mot_gsize = this->mot_group->size();
		std::cout<< mot_gsize << std::endl;
		for(int i=0; i<mot_gsize; i++){
			mdList.push_back(MotorData(this->family_name,this->motor_names[i]));
		}	
	}else{
		ROS_ERROR("Not all motors were detected!");
	}

	// get the group of force sensors by specifying the family name and module names
	this->fs_group = this->lookup.getGroupFromNames({this->family_name},this->fs_names);
	if (this->fs_group!=NULL){
		ROS_INFO("All force sensors were successfully detected.");
	}else{
		ROS_ERROR("Not all force sensors were detected!");
	}

	// setup feedback
	this->HEBI_setupFb();
	// setup commands
	this->HEBI_setupCmd();
}

// get motor measurements
void RobotInterface::HEBI_motFb(){
	if (this->mot_group->getNextFeedback(this->mot_group_fbk,1)){
		auto gyro = this->mot_group_fbk.getGyro();
		auto acc = this->mot_group_fbk.getAccelerometer();
		auto pos = this->mot_group_fbk.getPosition();
		auto vel = this->mot_group_fbk.getVelocity();
		auto eff = this->mot_group_fbk.getEffort();

		for (int i=0; i<this->mdList.size(); i++){
			this->mdList[i].updateMeasurements(pos[i], vel[i], eff[i], Eigen::Vector3d(acc(i,0),acc(i,1),acc(i,2)), Eigen::Vector3d(gyro(i,0),gyro(i,1),gyro(i,2)));
		}
	}else{
		ROS_WARN("No motor feedback received!");
	}
}

// get force measurements
void RobotInterface::HEBI_fsFb(){

	if (this->fs_group->getNextFeedback(this->fs_group_fbk,2)){
		//left leg:
		auto& analogLeft = this->fs_group_fbk[0].io();
		//right leg:
		auto& analogRight = this->fs_group_fbk[1].io();
		int ind = 0;
		for(int i=0; i<this->fs_ind.size(); i++){
			ind = this->fs_ind[i]+1;
			// values for the left sensor
			if(analogLeft.a().hasFloat(ind)) {
	      		this->fs_left.measVoltage[i] = (double) analogLeft.a().getFloat(ind);
		    } else {
	      		this->fs_left.measVoltage[i] = (double) analogLeft.a().getInt(ind);
		    }
		    // compute forces for the left sensor
		    this->fs_left.measForce[i] = (this->fs_left.measVoltage[i] - this->fs_left.measOffs[i]) * VOLT2NEWT;

		    // values for the right sensor
		    if (analogRight.a().hasFloat(ind)) {
	      		this->fs_right.measVoltage[i] = (double) analogRight.a().getFloat(ind);
		    } else {
	      		this->fs_right.measVoltage[i] = (double) analogRight.a().getInt(ind);
		    }
		    // compute forces for the right sensor
			this->fs_right.measForce[i] = (this->fs_right.measVoltage[i] - this->fs_right.measOffs[i]) * VOLT2NEWT;
		}
	}else{
		ROS_WARN("No force sensor feedback received!");
	}
}

// setup feedback collection
void RobotInterface::HEBI_setupFb(){
	if(!this->mot_group->setFeedbackFrequencyHz(this->Fctrl)){
		ROS_WARN("Feedback frequency for the motor group is too high!");
	}else{
		ROS_INFO("Feedback frequency for the motor group was set to %fHz.",this->Fctrl);
	}

	if(!this->fs_group->setFeedbackFrequencyHz(this->Fctrl)){
		ROS_WARN("Feedback frequency for the force sensor group is too high!");
	}else{
		ROS_INFO("Feedback frequency for the force sensor group was set to %fHz.",this->Fctrl);
	}
}

// TODO - tune the gains
void RobotInterface::HEBI_setupCmd(){
	int gsize = this->mot_group->size();
	hebi::GroupCommand init_cmd(gsize);
	// set Mstops for all the motors
	for(int i=0; i<gsize; i++){
		init_cmd[i].settings().actuator().mstopStrategy().set(hebi::Command::MstopStrategy::MotorOff);
		init_cmd[i].settings().actuator().controlStrategy().set(hebi::Command::ControlStrategy::Strategy4);
		// TODO set low level gains
		init_cmd[i].settings().actuator().positionGains().kP().set(this->motor_gains_Kp.at(i));
		init_cmd[i].settings().actuator().positionGains().kI().set(this->motor_gains_Ki.at(i));
 		init_cmd[i].settings().actuator().velocityGains().kP().set(this->motor_gains_Kd.at(i));
 		//init_cmd[i].settings().actuator().effortGains().kP().set(0.25);
	}

	if (this->mot_group->sendCommandWithAcknowledgement(init_cmd, 500))
	{
		ROS_INFO("Motors have been set up correctly.");
	}
	else
	{
		ROS_WARN("Did not receive acknowledgement! Motors not set up");
	}
}

// send the command to modules
void RobotInterface::HEBI_sendCmd(){
	ioVector cmd_p, cmd_v, cmd_e;

	for (int i = 0; i < NMOT; i++){
		cmd_p[i] = this->mdList[i].getPositionCmd();
		cmd_v[i] = this->mdList[i].getVelocityCmd();
		cmd_e[i] = this->mdList[i].getEffortCmd();
	}
	
	mot_group_cmd.setPosition(cmd_p);
	mot_group_cmd.setVelocity(cmd_v);
	mot_group_cmd.setEffort(cmd_e);

	if(RI_DEBUG){
		std::cout << "CMD position: " << cmd_p.transpose() << std::endl;
	}

	// TODO send commands
	if(OUT_ACTIVE)
	{
		if(!this->mot_group->sendCommand(mot_group_cmd)){
			ROS_WARN("Motor has not received the command!");
		}
	}
}

// matrices mapping the measurement space to the state space
void RobotInterface::computeMapMatrices(){
	this->io2state = io2stateMat::Constant(0);
	this->state2io = state2ioMat::Constant(0);
	auto io2stateCopy = this->io2state;
	auto io2stateCopyTq = this->io2state;
	auto io2stateCopyRev = this->io2state;
	for(int i = 0; i < this->motor_map.size(); i++){
		int j = motor_map[i];
		// for input decide whether the measurements should be averaged
		io2stateCopyTq(j,i) = motor_coef[i];
		io2stateCopy(j,i) = motor_coef[i] > 0 ? 1 : 0;
		io2stateCopyRev(j,i) = motor_coef[i] > 0 ? 1 : -1;
	}

	if(USE_ALL_MEAS){
		this->io2state = io2stateCopyTq;
	}else{
		this->io2state = io2stateCopy;
	}
	// for output always use all motors
	this->state2io = io2stateCopyRev.transpose();
	this->state2io_tq = io2stateCopyTq.transpose();

	if(RI_DEBUG){
		std::cout << "Here is the chosen matrix which maps inputs to states\n" << this->io2state << std::endl;
		std::cout << "Here is the original matrix which maps states to inputs\n" << io2stateCopy << std::endl;
		std::cout << "Here is the matrix which maps states to inputs\n" << this->state2io << std::endl;
	}
}

// transform the IMU measurements to the base coordinate frame and store them in the registers accessible to the ControllerManager
void RobotInterface::HEBI_mapIMU(){

	// transform all measurements to the equal frame
	Eigen::Vector3d accL = this->mdList[this->used_imu[0]].getAcceleration();
	Eigen::Vector3d accR = this->mdList[this->used_imu[1]].getAcceleration();
	Eigen::Vector3d gyroL = this->mdList[this->used_imu[0]].getGyro();
	Eigen::Vector3d gyroR = this->mdList[this->used_imu[1]].getGyro();
 	// store them into registered variables
	for(int i=0; i<3; i++){
		this->lbase.acc[i] = accL[i];
		this->rbase.acc[i] = accR[i];
		this->lbase.gyro[i] = gyroL[i];
		this->rbase.gyro[i] = gyroR[i];
	}
}

// mapping from 16 motors to 12DOF (inputs to states)
void RobotInterface::HEBI_motFbMap(){
	stateVector out_p, out_v, out_e;
	ioVector inp_p, inp_v, inp_e;

	// take stored measurements from the motor class
	for (int i = 0; i < this->mdList.size(); i++){
		inp_p[i] = this->mdList[i].getPosition();
		inp_v[i] = this->mdList[i].getVelocity();
		inp_e[i] = this->mdList[i].getEffort();
	}

	// remap motor measurements to states
	out_p = this->HEBI_modelOffsets(this->io2state * inp_p);
	out_v = this->io2state * inp_v;
	out_e = this->io2state * inp_e;

	// store states into shared registers
	for (int i = 0; i < NDOF; i++){
		this->meas.pos[i] = out_p[i];
		this->meas.vel[i] = out_v[i];
		this->meas.eff[i] = out_e[i];
	}

	if(RI_DEBUG){
		//std::cout << "MEAS position: " << inp_p.transpose() << std::endl;
		//std::cout << "MEAS velocity: " << inp_v.transpose() << std::endl;
		//std::cout << "MEAS effort: "   << inp_e.transpose() << std::endl;
	}
}

// mapping from 12DOF to 16 motors (states to inputs)
void RobotInterface::HEBI_motCmdMap(){
	stateVector inp_p, inp_v, inp_e;
	ioVector cmd_p, cmd_v, cmd_e;

	// take computed commands
	for (int i = 0; i < NDOF; i++){
		inp_p[i] = this->cmd.pos[i];
		inp_v[i] = this->cmd.vel[i];
		inp_e[i] = this->cmd.eff[i];
	}

	// remap states back to motor commands
	cmd_p = this->HEBI_driftCompensation(this->state2io * this->HEBI_originalOffsets(inp_p));
	cmd_v = this->state2io * inp_v;
	cmd_e = this->state2io * inp_e;

	// store data back to the motor data structures
	for (int i = 0; i < NMOT; i++){
		this->mdList[i].updateCommands(cmd_p[i], cmd_v[i], cmd_e[i]);
	}

	if(RI_DEBUG){
		//std::cout << "CMD position: " << cmd_p.transpose() << std::endl;
		//std::cout << "CMD velocity: " << cmd_v.transpose() << std::endl;
		//std::cout << "CMD effort: "   << cmd_e.transpose() << std::endl;
	}
}

// add offset to the measured joint values (transform them to the robot model joint values)
stateVector RobotInterface::HEBI_modelOffsets(stateVector sv){
	return sv + this->joint_offs;
}

stateVector RobotInterface::HEBI_originalOffsets(stateVector sv){
	return sv - this->joint_offs;
}

ioVector RobotInterface::HEBI_driftCompensation(ioVector sv){
	return sv + this->joint_comp;
}

