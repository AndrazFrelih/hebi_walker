// Standard libraries
#include <iostream>

// ROS standard libraries
#include <ros/ros.h>
#include <pluginlib/class_list_macros.h> // needed for exporting the class at the end

// ROS control
#include <controller_interface/controller_base.h> // needed for exporting the class at the end


// Include all the control related classes here
#include "FloatingBaseEst.hpp" //has to be included before the other custom classes (because of some conflict between Eigen and RBDL)
#include "PatternGenerator.hpp"
#include "TrajectoryGenerator.hpp"
#include "Balancer.hpp"
#include "Targets.hpp"
#include "FullBodyIKine.hpp"

// Base class
#include "RobotBaseController.hpp"


#include "definitions.hpp"

#define FFC_DEBUG false


class FlorenceFullController : public RobotBaseController
{
private:
	ros::NodeHandle nh;
	bool err;
	std::vector<double> zeros;
	ros::Time start_time;
	bool debug;

	// initialization
	bool isInit;

	// testing purposes
	std::vector<Vector4d> fake_fsen;

	// control algorithm modules
	FloatingBaseEst FBE;
	PatternGenerator PG;
	TrajectoryGenerator TG;
	Balancer BL;
	FullBodyIKine IK;

	// current state of modules
	Targets meas, cmd;
	phase_data pd;

	// initialize the LIP model and other controller modules
	void initModules(const std::vector<double> &q_m, const std::vector<double> &q_c);


public:
	// constructor and destructor
	FlorenceFullController();
	~FlorenceFullController();

	// every controller needs the algorithmStep. It is defined as virtual and called by the class RobotBaseController in each controller update phase
	void algorithmStep(const ros::Duration& period) override;

	// these two functions are defined as virtual functions in the base class and get called when the controller is starting / stopping respectively. If anything is needed at that time, declare the body of these functions
	void startController() override;
	void stopController() override;

};

// This declares the class as a plugin library, the namespace and the class name must match the above code.
// The last argument of the function must remain as it is because it defines it as a plugin for the controller manager.
PLUGINLIB_EXPORT_CLASS(FlorenceFullController, controller_interface::ControllerBase)