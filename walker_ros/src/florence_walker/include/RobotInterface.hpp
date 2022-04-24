// standard libraries
#include <iostream>
#include <string>
#include <cmath>
#include <thread>
#include <list>
#include <map>
#include <typeinfo>

// ROS standard libraries
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/WrenchStamped.h>

// ROS control
#include <hardware_interface/imu_sensor_interface.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>
#include <hardware_interface/internal/demangle_symbol.h>
#include <hardware_interface/internal/interface_manager.h>
#include <hardware_interface/hardware_interface.h>
#include <hardware_interface/controller_info.h>
#include "load_cell_array.h" //custom handle and interface library

// Data structure for measurements
#include "MotorData.hpp"

// HEBI libraries
#include "lookup.hpp"
#include "group_feedback.hpp"
#include "group_command.hpp"
#include "group_info.hpp"
#include "gains.hpp"
//#include "util/plot_functions.h"

//custom libraries
#include "definitions.hpp"

class RobotInterface : public hardware_interface::RobotHW
{
	public:
	// constructor
	RobotInterface(ros::NodeHandle & nh);
	// ROS control
	void read();
	void write(ros::Duration elapsed_time);
	bool checkForConflict(const std::list<hardware_interface::ControllerInfo> &info) const override;
	// timing
	ros::Time get_time();
	ros::Duration get_period();

	private:

	////////////////////////////////////////////////////////////////////////////////////////////////
	// VARIABLES:
	////////////////////////////////////////////////////////////////////////////////////////////////

	// General ROS
	ros::NodeHandle nh;
	ros::Time last_time;

	// Parameter holders
	std::string family_name;
	std::vector<std::string> joint_names;
	std::vector<std::string> imu_names;
	std::vector<int> used_imu;
	std::vector<std::string> motor_names;
	std::vector<int> motor_map;
	std::vector<double> motor_coef;
	std::vector<std::string> fs_names;
	std::vector<int> fs_ind;

	std::vector<double> motor_gains_Kp;
	std::vector<double> motor_gains_Ki;
	std::vector<double> motor_gains_Kd;

	// Control loop frequency
	double Fctrl;

	// HEBI variables
	hebi::Lookup lookup; 						// used to access general information about motors. Can return groups of modules.
	std::shared_ptr<hebi::Group> mot_group; 	// used to send and receive data to grouped motor modules.
	hebi::GroupFeedback mot_group_fbk; 			// stores the received data from motor modules.
	std::shared_ptr<hebi::Group> fs_group; 		// used to send and receive data to grouped motor force sensor modules. 	
	hebi::GroupFeedback fs_group_fbk; 			// stores the received data from force sensor modules.

	hebi::GroupCommand mot_group_cmd;
	
	// Force sensor data struct
	struct fs_data
	{
		double measVoltage[4] = {0};
		double measOffs[4] = {0.5};
		double measForce[4] = {0};
	} fs_left, fs_right;

	// Motor measurement storage
	std::vector<MotorData> mdList;

	// Mapping matrices
	io2stateMat io2state;
	state2ioMat state2io_tq;
	state2ioMat state2io;
	stateVector joint_offs;
	ioVector joint_comp;

	// Interface registers
	struct joint_data
	{
		double pos[NDOF] = {0};
		double vel[NDOF] = {0};
		double eff[NDOF] = {0};
	} meas, cmd;

	struct imu_data{
		double ori[4] = {0}; //estimated world orientation of the base frame
		double ori_cov[9] = {0};
		double gyro[3] = {0};
		double gyro_cov[9] = {0}; 
		double acc[3] = {0};
		double acc_cov[9] = {0}; 
		Eigen::Matrix<double,3,3> Rb_imu; //orientation in the base frame
	} lbase, rbase;

	
	// ROS control
	hardware_interface::JointStateInterface jnt_state_interface; 	// this registers the measurement interface
	hardware_interface::ImuSensorInterface imu_sen_interface;
	hardware_interface::LoadCellArrayInterface fsen_interface;
	hardware_interface::PositionJointInterface jnt_pos_interface; 	// this registers the position control interface
	hardware_interface::VelocityJointInterface jnt_vel_interface; 	// this registers the velocity control interface
	hardware_interface::EffortJointInterface jnt_eff_interface; 	// this registers the effort control interface

	////////////////////////////////////////////////////////////////////////////////////////////////
	// FUNCTIONS:
	//////////////////////////////////////////////////////////////////////////////////////////////// 

	// Calibration
	void readFsOffset();

	// ROS standard
	bool readParameters();

	// ROS control
	void init();
	
	// HEBI
	void HEBI_init();									// get access to HEBI modules
	void HEBI_lookupEntries();							// print all the available entries
	void HEBI_setupFb();								// setup the feedback class so measurements can be read
	void HEBI_motFb();									// get the feedback data for motors
	void HEBI_fsFb();									// get the feedback data for load cells
	void HEBI_motFbMap();								// map motor values to control model values (16DOF to 12DOF)
	void HEBI_mapIMU();									// map measured imu values to their handles
	void HEBI_setupCmd();								// setup the command class so that commands can be sent
	void HEBI_motCmd();									// send motor commands to modules
	void HEBI_motCmdMap();								// map control model commands to motor values (distribute torques when there are dual configurations available, take care of correct sign of the data, add required offsets)
	void HEBI_sendCmd();								// send command from handles to actual motors
	stateVector HEBI_modelOffsets(stateVector sv);		// add offsets to values (difference between model and motor zero points)
	stateVector HEBI_originalOffsets(stateVector sv);	// 
	ioVector HEBI_driftCompensation(ioVector sv);		// compensate the drift between motors in dual configuration

	// Mappings
	void computeMapMatrices();							// mapping matrices used in map functions

};