// standard libraries
#include <iostream>
#include <cmath>
#include <deque>

// ROS libraries
#include <ros/ros.h>

// Control-toolbox library 
#include <ct/core/core.h> 
#include <ct/optcon/optcon.h> 

// Plotting
#include "matplotlibcpp.h"
namespace plt = matplotlibcpp;

// Custom libraries
#include "definitions.hpp"
#include "Targets.hpp"


class TrajectoryGenerator
{
private:
	// ros
	ros::NodeHandle nh;

	// settings
	bool err;
	bool debug;
	bool isInitialized;

	// parameters
	int Npreview;	// number of preview steps
	double zdes;	// desired com height
	double omega;	// lipm time constant
	double Tini;	// initial time needed for the data to pass from input to output
	double fs;		// update frequency (needed for simulation)
	double Ts;		// update period
	double mass;	// system mass
	double Fg;		// system gravitational force

	// double ended queue for the data
	pattern_gen_out def;
	std::deque<pattern_gen_out> container;

	// original discrete system (cart table model)
	Eigen::Matrix<double,3,3> Asys;
	Eigen::Matrix<double,3,1> Bsys;
	Eigen::Matrix<double,1,3> Csys;

	// feedback matrix matrix
	Eigen::Matrix<double,1,NPREV+4> K;
	// parts of the feedback matrix
	double Kint;
	Eigen::Matrix<double,1,3> Kcom;
	Eigen::Matrix<double,1,NPREV> Kref;

	// integrator
	Eigen::Vector3d com_int;

	// internal LIPM model
	Eigen::Vector3d com_x, com_x_next;
	Eigen::Vector3d com_y, com_y_next;
	double zmp_x, zmp_y;

	// grouped LIPM vectors
	Eigen::Vector3d com;
	Eigen::Vector3d comp;
	Eigen::Vector3d compp;
	Eigen::Vector3d dcm;
	Eigen::Vector3d dcmp;
	Eigen::Vector3d zmp;

	// output
	Targets out;

	// read parameters from rosparam cloud
	void readParameters();
	// compute the feedback matrix using LQR method
	void setupLIPM2Dsystem();
	void computeFeedbackMatrix();
	void plotKrefCoefficients();
	// resize deque to the desired size and set initial values
	void initDeque(pattern_gen_out def);
	// set the mass of the system
	void setMass(double mass);
	// reset states of the internal simulation
	void resetStates();
	// reset deque container (flush all the values and reinitialize it)
	void resetDeque();
	
	void updateDeque(pattern_gen_out update);
	Eigen::Vector3d computeCtrlSignal();

	// model update (technically cart table model is used - but LIP is equivalent to it)
	void stepLIPM2D(Eigen::Vector3d ctrlSignal);
	void propagateModel(double u, Eigen::Vector3d & vold, Eigen::Vector3d & vnew, double & out);
	void updateTargets();

public:
	TrajectoryGenerator(ros::NodeHandle &nh, bool debug);
	~TrajectoryGenerator(){}
	// init has to be executed before the first call to init
	void init(pattern_gen_out start, double mass);
	void step(pattern_gen_out update);
	void reset();

	// getters
	Targets getTargets(){return this->out;}
	Eigen::Vector3d getPGzmp(){return this->container.front().zmp;}
	phase_data getPhaseData(){return this->container.front().pd;}
	
};