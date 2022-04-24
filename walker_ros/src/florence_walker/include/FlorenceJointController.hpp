// Standard libraries
#include <iostream>

// ROS standard libraries
#include <ros/ros.h>
#include <pluginlib/class_list_macros.h> // needed for exporting the class at the end

// ROS control
#include <controller_interface/controller_base.h> // needed for exporting the class at the end

// Include all the control related classes here
#include "RobotBaseController.hpp"

// Other custom libraries
#include "definitions.hpp"



class FlorenceJointController : public RobotBaseController
{
private:
	// read parameters
	ros::NodeHandle nh;
	// finite state machine parameters
	ros::Time refTime;
	double initWait;
	double transitionTime;

	// initial measurements
	int dim;
	std::vector<double> q_ini;
	std::vector<double> q_fin;
	std::vector<double> zeros;

	// trajectory storage
	std::vector<double> q_c_traj;
	std::vector<double> qp_c_traj;
	std::vector<polyCoefVec> poly_coef;

	//fsm enum
	enum state {START, CAPT, TRAJ, END};
	state fsm_state;

	// Finite state machine of this class
	ctrl_states fsm();

	// Simple trajectory generation functions
	void evalTraj(double time);
	// Helper for evalTraj (compute positions)
	double evalTrajPos(double time, polyCoefVec poly);
	// Helper for evalTraj (compute velocities)
	double evalTrajVel(double time, polyCoefVec poly);
	// Compute the parameters for the trajectory (polynomial coefficients)
	void compTrajParams();
	// 
	polyCoefVec genPolyCoef(double Tfin, double xini, double xfin);

public:
	// constructor and destructor
	FlorenceJointController();
	~FlorenceJointController(){};

	// every controller needs the algorithmStep. It is defined as virtual and called by the class RobotBaseController in each controller update phase
	void algorithmStep(const ros::Duration& period) override;

	// these two functions are defined as virtual functions in the base class and get called when the controller is starting / stopping respectively. If anything is needed at that time, declare the body of these functions
	void startController() override;
	void stopController() override;
};

// This declares the class as a plugin library, the namespace and the class name must match the above code.
// The last argument of the function must remain as it is because it defines it as a plugin for the controller manager.
PLUGINLIB_EXPORT_CLASS(FlorenceJointController, controller_interface::ControllerBase)