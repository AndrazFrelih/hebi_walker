// Standard libraries
#include <iostream>
#include <string>
#include <cmath>

// ROS library
#include <ros/ros.h>

// qpOASES library
#include <qpOASES.hpp>

// custom libraries
#include "definitions.hpp"
#include "Targets.hpp"

class FullBodyIKine{
private:
	// General application settings
	bool err;										// was there any error encountered in the execution
	bool debug;										// is debugging activated
	bool actInt;									// should integrators be active (com, position EE left and EE right)
	
	// Flag variables
	bool useFb;										// is feedback used in optimization (when controlling the actual robot, it is always on)
	bool initClass;									// is the class initialized

	// Solver settings
	bool initSolver;								// has solver already been called
	bool solveDual;									// should the dual solution be stored (not needed)
	qpOASES::SQProblem qp;							// sequential quadratic problem solver
	qpOASES::SQProblem * qp_solver;					// pointer to the sequential quadratic problem solver
	qpOASES::Options qp_options;					// options for the solver
	qpOASES::real_t xOptSol[NVRBL];					// primal solution storage array
	qpOASES::real_t yOptSol[NVRBL+NCSTR];			// dual solution storage array

	// full body ikine - relevant solutions for control
	std::vector<double> pos_ini, ori_ini;			// initial position and orientation
	Eigen::Matrix<double,NVRBL,1> X_ini;			// initial value for the integral of the solution (integral of velocity is position)
	Eigen::Matrix<double,NVRBL,1> X_sol;			// integral of the solution (integral of velocity is position)
	std::vector<double> Q_sol;						// only joint angle part of the solution
	std::vector<double> Qp_sol;						// only joint velocity part of the solution
	Eigen::Matrix<double,NVRBL,1> Xp_sol;			// direct solution (copied from xOptSol)

	// ROS variables
	ros::NodeHandle nh;

	// Internal variable to store simulation states and targets
	Eigen::Matrix<double,NVRBL+NCSTR,1> Yp_sol;
	Targets measTarg, cmdTarg;

	// Integrators of errors
	Eigen::Vector3d com_int;
	Eigen::Vector3d lleg_int;
	Eigen::Vector3d rleg_int;

	// Parameters & gains
	double wshift_offs;								// in case offset of the weight shifting is required
	Eigen::Matrix<double, NDOF, NDOF> Kq;
	Eigen::Matrix3d Kwfb, Kcom_p, Kcom_i;
	Eigen::Matrix3d Kvleg_p, Kvleg_i, Kwleg;
	Eigen::Matrix3d KvlegL_p, KvlegL_i, KwlegL;
	Eigen::Matrix3d KvlegR_p, KvlegR_i, KwlegR;
	std::vector<double> lims;						// actuator velocity limits (infeasible problems can be traced to too tight velocity limits - increase them in that case and then later saturate them after solution is there)
	Eigen::Matrix<double, NVRBL, 1> limsvec;		// lims has to be copied into an eigen vector

	// Foot damping control (look at Stephen Caron article about the LIPM based staircase climbing with HRP4 for the definition of the task - he uses a more advanced approach)
	bool actFdc;									// should foot damping control be activated?
	bool zmpSetsInit;								// are the foot zones already set up
	Eigen::Matrix3d Kfdc;							// feeback matrix for the foot damping control
	bool fdcL_active, fdcR_active;					// controllers are only active when local zmp reaches the edge of the foot (saturation based foot damping control)
	std::vector<double> sets;						// there are 3 sets (zones). If zmp gets to the edge and reaches the outermost zone, damping control brings it back to the zone one. (hysteresis based control)
	Eigen::Vector3d zmp_Z1, zmp_Z2, zmp_Z3;			// definition of zones

	// Foot pressure control
	double Kdfz, Tvdc;								// gains for foot pressure control (does not work well in practice because of inaccurate force sensors and was hence turned off)
	double mass, Fg;								// mass of the robot and gravitatonal force


	// QP-problem matrices in array form (qpOASES)
	qpOASES::real_t H[NVRBL * NVRBL];				// Hessean matrix
	qpOASES::real_t g[NVRBL];						// Gradient vector

	qpOASES::real_t A[NCSTR * NVRBL];				// Equality/Inequality constraints (for equality set UB = LB)
	qpOASES::real_t lbA[NCSTR];						// Lower constraint bounds
	qpOASES::real_t ubA[NCSTR];						// Upper constraint bounds
	
	qpOASES::real_t lb[NVRBL];						// Lower variable bounds
	qpOASES::real_t ub[NVRBL];						// Upper variable bounds

	// QP-problem matrices (Eigen)
	Eigen::Matrix<double, NVRBL, NVRBL> H_eig;		// Hessean matrix
	Eigen::Matrix<double, NVRBL, 1> g_eig;			// Gradient vector

	Eigen::Matrix<double, NCSTR, NVRBL> A_eig;		// Equality/Inequality constraints (for equality set UB = LB)
	Eigen::Matrix<double, NCSTR, 1> lbA_eig;		// Lower constraint bounds
	Eigen::Matrix<double, NCSTR, 1> ubA_eig;		// Upper constraint bounds

	Eigen::Matrix<double, NVRBL, 1> lb_eig;			// Lower variable bounds
	Eigen::Matrix<double, NVRBL, 1> ub_eig;			// Upper variable bounds


	// read params from ros
	void readParameters();
	// rewrite gains to the feedback matrix
	template<typename Derived>
	void createFbMatrix(Eigen::MatrixBase<Derived> & mat, std::vector<double> param);
	// use Eigen to compute QP matrices
	void setupQPmatrices(const Eigen::Matrix<double,NDOF,1> & qold,
						const Targets & meas,
						const Targets & cmd,
						const std::vector<WorldJacobian> & jlist,
						phase_data pd,
						ros::Duration dt);
	// depending on the phase of walking, gains can be adapted (using the weight shift parameter)
	void adaptGains(phase_data pd, double FL_m, double FR_m);
	// generate a skew matrix from another matrix
	Eigen::Matrix3d getSkewMat(Eigen::Matrix3d mat);
	// get a vector from the skew symmetric matrix
	Eigen::Vector3d skew2vec(Eigen::Matrix3d mat);
	// foot damping control - control law implementation
	void footDampingCtrl(const Eigen::Vector3d & zmploc_m, Eigen::Vector3d & wfdc, bool & ctrl_active);
	Eigen::Vector3d saturateZmp(const Eigen::Vector3d & zmploc_m);
	bool isStrictlySmaller(const Eigen::Vector3d & v1, const Eigen::Vector3d & v2);
	// map Eigen matrices to qpOases arrays
	void updateQParrays();
	// template so it can be used for different types of eigen elements (static/dynamic matrices, vectors)
	template<typename Derived>
	void copyMatrixToArray(const Eigen::MatrixBase<Derived> & mat, qpOASES::real_t * arr);
	// check if any of the states is at the border
	bool checkLimits();
	// reset the 3 feedback integrators
	void resetIntegrators();
	// restart the solver (execute init again)
	void resetSolver();
	// fill the Q_sol and Qp_sol vectors with new solutions
	void setCmdJoints();
	// solve the qp problem
	bool solveQPproblem();
	// template so it can be used for different types of eigen elements (static/dynamic matrices, vectors)
	template<typename Derived>
	void copyArrayToVector(const qpOASES::real_t * arr, Eigen::MatrixBase<Derived> & vec);
	

	// integrator for the velocity solution
	void integrate(ros::Duration dt);


public:
	// constructors
	FullBodyIKine(ros::NodeHandle & nh, bool debug);
	~FullBodyIKine();

	// interface
	bool update(const Targets & meas,
				const Targets & cmd,
				const std::vector<WorldJacobian> & jlist,
				phase_data pd,
				ros::Duration dt);
	// setters
	void useFeedback(bool fb);
	void setFootDims(Eigen::Vector3d footDims);
	void setMass(double mass);
	// initialization of the class
	void init(const std::vector<double> & Q_ini, Eigen::Vector3d footDims, double mass, bool useFb);
	// reset the class
	void reset();

	// getters
	Eigen::Matrix<double,NVRBL,1> getFullPosSol(){return this->X_sol;};
	Eigen::Matrix<double,NVRBL,1> getFullVelSol(){return this->Xp_sol;};
	Eigen::Matrix<double,NDOF_FB,1> getFbPosSol(){return this->X_sol.block<NDOF_FB,1>(0,0);};
	Eigen::Matrix<double,NDOF_FB,1> getFbVelSol(){return this->Xp_sol.block<NDOF_FB,1>(0,0);};
	std::vector<double> getJointPosSol(){return this->Q_sol;};
	std::vector<double> getJointVelSol(){return this->Qp_sol;};
};