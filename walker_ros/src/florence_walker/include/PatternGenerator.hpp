// Standard libraries
#define __STDCPP_WANT_MATH_SPEC_FUNCS__ 1 // for the beta function which is needed for bezier curves (has to be defined before any standard library is included)
#include <string>
#include <iostream>
#include <cmath>

// ROS libraries
#include <ros/ros.h>

// Custom libraries
#include "definitions.hpp"

class PatternGenerator
{
private:
	// ros
	ros::NodeHandle nh;									// copy of parent's node handle

	// settings
	bool debug;											// should debugging mode be activated?
	bool err;											// was there an error in the algorithm?

	int mode;											// mode: 0 - balance, 1 - weight shift, 2 - stepping in place, 3 - stepping forward
	int dir;											// dir: 0 - start with left foot first, 1 - right foot first
	double Tss, Tds;									// duration of respective support phases
	double Tini, Tfin;									// duration of initial and final phase

	double Lx, Ly, Lz;
	double proc_y;


	// finite state machine settings
	typedef enum {IDLE, INI, DS_INI, DS_L, DS_R, SS_L, SS_R, DS_FIN, FIN, ERR} pg_state;
	bool startTriggered;								// should the module start the generation
	pg_state fsm_stage;									// stage of the finite state machine
	bool fsm_start_stage;								// executes the initialization upon entering a stage
	int Nsteps;											// number of steps that have to be performed
	int istep;											// current number of executed steps
	bool isStartingStep, isEndingStep;					// if it is the starting or ending step, special behaviour is triggered
	ros::Time timeref;									// the time reference required for the algorithm
	bool hasEnded; 										// has the algorithm finished execution?

	// outputs
	Eigen::Vector3d zmp;								// momeantary zmp position
	phase_data pd;										// momeantary phase data (ds/ss and weight shift)
	Eigen::Vector3d pL,  pR;							// momeantary foot positions
	Eigen::Vector3d pLp, pRp;							// momeantary foot velocities
	Eigen::Vector3d pLpp, pRpp;							// momeantary foot accelerations
	pattern_gen_out pg_out;								// struct that stores all the data above			

	// pattern & trajectory generation variables
	Eigen::Vector3d zmp_start, zmp_end;					// zmp reference positions (beginning and end of each stage)
	Eigen::Vector3d pL_start, pL_end;
	std::vector<Eigen::Vector3d> ptsL;					// reference points for the bezier curve (left foot)
	Eigen::Vector3d pR_start, pR_end;
	std::vector<Eigen::Vector3d> ptsR;					// reference points for the bezier curve (right foot)
	double wshiftL_start, wshiftL_end;					// weight shift trajectory (left)
	double wshiftR_start, wshiftR_end;					// weight shift trajectory (right)

	// here also fb orientation could be added
	// TODO if the robot is able to walk


	// reset the internal variables
	void reset();

	// read parameters from the ROS param cloud
	void readParameters();

	// finite state machine 
	void fsmStep(ros::Time time);
	// FSM stage functions
	void fsm_idle(ros::Time time);						// initial state before the class is initialized
	void fsm_ini(ros::Time time, double Tphase);		// initialization of the class has been carried out
	void fsm_dsIni(ros::Time time, double Tphase);		// first double support phase is shorter (1/2 Tds) as the distance to travel is shorter
	void fsm_dsL(ros::Time time, double Tphase);		// double support which brings the COM from right to left
	void fsm_dsR(ros::Time time, double Tphase);		// double support which brings the COM from left to right
	void fsm_dsFin(ros::Time time, double Tphase);		// final double support phase is also shorter (1/2 Tds)
	void fsm_ssL(ros::Time time, double Tphase);		// single support phase when weight is on the left side
	void fsm_ssR(ros::Time time, double Tphase);		// single support phase when weight is on the right side
	void fsm_fin(ros::Time time, double Tphase);		// final state where the output is held constant
	void fsm_err(ros::Time time);						// error state

	double normaliseTime(double dT, double Tp);			// for bezier curves time needs to be between 0 and 1
	double getTimeDiff(ros::Time time);					// time does not start at zero, so offset has to be removed

	// set intervals where zmp, lleg EE and rleg EE positions will be interpolated over
	void setZmpInterval(const Eigen::Vector3d & start, const Eigen::Vector3d & end, double proc);	
	void setLLegInterval(const Eigen::Vector3d & start, const Eigen::Vector3d & end, double wstart, double wend);
	void setRLegInterval(const Eigen::Vector3d & start, const Eigen::Vector3d & end, double wstart, double wend);

	// for avoiding jumps in velocity and acceleration signals that are generated
	void setLLegIntermPts(bool isSwing);
	void setRLegIntermPts(bool isSwing);
	void swingLegIntermPts(std::vector<Eigen::Vector3d> & vect, const Eigen::Vector3d & start, const Eigen::Vector3d & end);
	void stanceLegIntermPts(std::vector<Eigen::Vector3d> & vect, const Eigen::Vector3d & start, const Eigen::Vector3d & end);

	// return the new position of the swing foot after the step (depends on step settings)
	Eigen::Vector3d getNextStepPos(const Eigen::Vector3d & pos_now);

	// splining for foot trajectories
	double binomCoeff(int n, int k);
	double bezierCoeff(int n, int k, double time_norm);
	Eigen::Vector3d bezierCurve(const std::vector<Eigen::Vector3d> & pts, double time_norm);				// bezier curve for position
	Eigen::Vector3d bezierFirstDerivative(const std::vector<Eigen::Vector3d> & pts, double time_norm);		// bezier curve for velocity
	Eigen::Vector3d bezierSecondDerivative(const std::vector<Eigen::Vector3d> & pts, double time_norm);		// bezier curve for acceleration
	
	// linear interpolation for zmp and weight shifting (template for different input types)
	template<typename Derived>
	Derived linInterp(const Derived & start, const Derived & end, double time_norm);

	// generate patterns and foot trajectories
	void generatePatterns(int phase, double time_norm);
	// update the output struct
	void updateOutputStruct();

public:
	// constructors
	PatternGenerator(ros::NodeHandle & nh, bool debug);
	~PatternGenerator(){};

	// interface
	void init(Eigen::Vector3d pLm, Eigen::Vector3d pRm, int Nsteps);
	void step(ros::Time time);
	void stop();

	// getters
	bool algorithmHasEnded(){return this->hasEnded;}
	Eigen::Vector3d getZmp(){return this->zmp;}
	Eigen::Vector3d getLLegPos(){return this->pL;}
	Eigen::Vector3d getRLegPos(){return this->pR;}
	Eigen::Vector3d getLLegVel(){return this->pLp;}
	Eigen::Vector3d getRLegVel(){return this->pRp;}
	Eigen::Vector3d getLLegAcc(){return this->pLpp;}
	Eigen::Vector3d getRLegAcc(){return this->pRpp;}
	phase_data getPhaseData(){return this->pd;}
	pattern_gen_out getOutput(){return this->pg_out;}


	
};