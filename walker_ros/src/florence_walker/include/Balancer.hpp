// Standard libraries
#include <iostream>
#include <string>
#include <cmath>

// custom libraries
#include "Targets.hpp"
#include "definitions.hpp"

// ROS library
#include <ros/ros.h>

/*
* implements the balancing strategy from the following paper:
* A Benchmarking of DCM Based Architectures for Position and Velocity Controlled Walking of Humanoid Robots
*/

class Balancer{
private:
	// node handle so parameters can be read
	ros::NodeHandle nh;
	// signalizez and error of the node
	bool err;

	// parameters
	double zdes;													// desired height of the COM 
	double omega;													// the constant of the LIPM - omega := sqrt(zdes/gravity)

	// integrator variables
	Eigen::Vector3d dcm_int;										// integrator of the error of divergent component of motion - dcm := com - comp/omega

	// gains
	bool activate_int;												// should the integrator be active?
	Eigen::Matrix3d Kdcm_p;											// proportional gains for the dcm control
	Eigen::Matrix3d Kdcm_i;											// integral gains for the dcm control
	Eigen::Matrix3d Kleak;											// leakage parameter for the integrator (so that there is no windup)
	Eigen::Matrix3d Kzmp;											// zmp gains for the second control stage (com - control)
	Eigen::Matrix3d Kcom;											// com gains for the second control stage (com - control)

	// output
	Eigen::Vector3d comp_r;											// reference com velocity


public:
	// initialize the class
	void init();
	// reset the integrator
	void reset();
	
	/* 
	* compute the reference com velocity:
	* _m denotes a measurement
	* _d denotes a desired value
	*/
	void update(Eigen::Vector3d com_m,
				Eigen::Vector3d zmp_m,
				Eigen::Vector3d dcm_m,
				Eigen::Vector3d com_d,
				Eigen::Vector3d comp_d,
				Eigen::Vector3d dcm_d,
				Eigen::Vector3d dcmp_d,
				ros::Duration dt);
	void update(const Targets & meas, const Targets & cmd, ros::Duration dt);

	// constructor & destructor
	Balancer(ros::NodeHandle & nh, bool debug);
	~Balancer(){};

	// getters
	bool getError(){return this->err;}
	Eigen::Vector3d getTargetComVelo(){return this->comp_r;}
};