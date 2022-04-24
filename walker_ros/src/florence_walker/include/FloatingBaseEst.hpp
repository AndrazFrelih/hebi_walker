// Standard libraries
#include <iostream>
#include <string>
#include <cmath>

// ROS library
#include <ros/ros.h>
#include <ros/package.h> //locate the package containing the robot urdf file

// RBDL libraries
#include <rbdl/rbdl.h> //core rbdl with kinematic algorithms
#include <rbdl/rbdl_utils.h> //utilities like com computation
#ifndef RBDL_BUILD_ADDON_URDFREADER
#error "Error: RBDL addon URDFReader not enabled."
#endif
#include <urdfreader.h> //reader module of robot urdf files

// custom libraries
#include "definitions.hpp"
#include "magdwick.hpp"
#include "Targets.hpp"

namespace RBD = RigidBodyDynamics;
namespace RBDM = RBD::Math;

/*
* Class responsible for calculation of:
* - forward kinematics
* - computation of zmp (local foot zmp and global zmp in wcf)
* - estimation of the floating base orientation and positon in the world coordinate frame
* - computation of other LIPM quantities (com, comp, dcm, dcmp)
* - forward differential kinematics (constraint and cost matrices for IKine)
*/	

class FloatingBaseEst{
private:
	// settings
	bool err;
	bool debug;

	// copy of the parent node handle
	ros::NodeHandle nh;
	
	// parameters
	std::vector<std::string> lfs_fnames;					// left leg frame names
	std::vector<std::string> rfs_fnames;					// right leg frame names			
	double alphaTra_drift, alphaOri_drift;					// drift compensation factor
	double alpha_cp;										// velocity filter time constant
	double alpha_cpp;										// acceleration filter time constant
	std::string lee_name;									// left endeffector name
	std::string ree_name;									// right endeffector name
	std::string eulconv;									// used euler convention
	double omega;											// the eigenvalue of LIPM		
	double zdes;											// desired com height
	std::vector<double> pzmp_c;								// probabilistic model coefficients for weight calculation
	std::vector<double> pcont_c;							// probabilistic model coefficients for weight calculation
	double Wimu;											// IMU weight for sensor fusion	
	std::vector<std::string> IMUnames; 						// names of the IMU urdf frames
	double totalMass;										// full robot mass

	// RBDL variables (since most of computations are done in RBDL convention and casting is required if variables were in Eigen convention, Eigen is taken as a default)
	RBD::Model * model;										// RBDL robot model
	RBDM::VectorNd Q;										// stored joint angle measurements
	RBDM::VectorNd Qp;										// stored joint velocity measurements
	RBDM::VectorNd Qeff;									// stored joint efforts measurements
	RBDM::Vector3d aL_b, aR_b;								// stored IMU accelerations	
	RBDM::Vector3d wL_b, wR_b;								// stored IMU rot. velocities		

	// Transformations
	RBDM::SpatialTransform Hw_b;							// base to world transformation
	RBDM::Vector3d X;										// base to world translation
	RBDM::Vector3d Eul;										// base to world orientation in euler angles
	std::vector<RBDM::SpatialTransform> Hb_i;				// leg to base transformations [ H(i) is the output of q(i) and not the input of q(i+1) (ie. proximal convention) ]	
		
	RBDM::SpatialTransform Hb_Lee;							// left leg to base transformation
	RBDM::SpatialTransform Hw_Lee;							// left leg to world transformation
	RBDM::SpatialTransform Hb_Ree;							// right leg to base transformation
	RBDM::SpatialTransform Hw_Ree;							// right leg to world transformation
	RBDM::SpatialTransform identity;						// identity transformation - to simplify the computation


	// FB-ori&pos estimation
	Eigen::Quaterniond quatL_old, quatR_old;				// Quaternion computation

	// COM jacobians
	RBDM::MatrixNd Jb_com_i; 								// temporary variable
	RBDM::MatrixNd Jb_eeL, Jb_eeR, Jb_com, Jb_b;			// jacobians in the base frame
	RBDM::MatrixNd Jw_eeL_g, Jw_eeR_g, Jw_com_g, Jw_b_g;	// jacobians in the world frame (generalised coordinates)
	RBDM::MatrixNd Jw_eeL, Jw_eeR, Jw_com, Jw_b;			// jacobians in the world frame (parametrised coordinates)
	RBDM::MatrixNd EulTfMatrix;								// velocity space transformation matrix (generalised to parametrized coordinates)
	std::vector<WorldJacobian> cstr_jlist;


	// fixed values (initialized at the beginning)
	RBDM::SpatialTransform HLeeFix;							// transformations from left EE to qL6
	RBDM::SpatialTransform HReeFix;							// transformations from right EE to qR6
	std::vector<RBDM::SpatialTransform> HLfootFix;			// transformations from left load cell frames to qL6
	std::vector<RBDM::Vector3d> teeL_fpts;					// positions of load cells in left EE frame
	std::vector<RBDM::SpatialTransform> HRfootFix;			// transformations from right load cell frames to qR6
	std::vector<RBDM::Vector3d> teeR_fpts;					// positions of load cells in right EE frame
	RBDM::SpatialTransform Hb_Limu;							// transformation between the left imu frame and base
	RBDM::SpatialTransform Hb_Rimu;							// transformation between the right imu frame and base	



	// relevant variables (balancer)
	RBDM::Vector3d zmp;										// world frame zero moment point
	RBDM::Vector3d zmplocL, zmplocR;						// local foot frame zero moment point
	double Ftot;											// total contact force
	double FLtot, FRtot;									// single leg contact force
	RBDM::Vector3d dcm;										// dcm computed from com and comp
	RBDM::Vector3d dcmp;									// dcmp computed from com and zmp

	bool initComCmp;										// flag that signals that the internal LIP model should be initialized
	RBDM::Vector3d com_b, com_b_old;						// base frame com position
	RBDM::Vector3d comp_b, comp_b_old;						// base frame com velocity
	RBDM::Vector3d compp_b, compp_b_old;					// base frame com acceleration
	RBDM::Vector3d com, comp, compp; 						// world frame

	// output of the class
	Targets measTarg;										// targets for the IKine


	// read rosparams
	void readParameters();

	// save fixed transforms as internal variables
	void storeFixedTransforms();

	// set integrators to zeto
	void resetIntegrators();

	// place the world coordinate frame between the two feet in the orientation of the floating base
	void setWcf(const std::vector<double> &q);

	// update the kinematic model of the robot (call RBDL fcns)
	void updateJntMeasurements(const std::vector<double> &q);
	void updateJntMeasurements(const std::vector<double> &q, 
								const std::vector<double> &qp, 
								const std::vector<double> &qeff);

	// forward kinematics
	void cmpFKine();
	
	// local foot frame zmp computation
	void cmpZmpLocal(RBDM::Vector3d & zmpLoc,
		 			double & Ftot, 
					const Vector4d & fsen_i, 
					const std::vector<RBDM::Vector3d> & pts);

	// global zmp computation
	void cmpZmpGlobal();

	/*
	* estimation of rotation and position of the floating base:
	* turn the measured vectors to the base coordinate frame:
	* ai_b - imu acceleration measured in i-IMU frame rotated to the base frame
	* wi_b - imu rot. velocity measured in i-IMU frame rotated to the base frame	
	*/
	void estFloatingBase(const std::vector<Eigen::Vector3d> &accel, 
						const std::vector<Eigen::Vector3d> &gyro,
						phase_data pd, 
						ros::Duration dt);

	// evaluate the probabilistic model for the contact forces
	double contactProbability(double Fleg_tot);

	// evaluate the probabilistic model for the local zmp position
	double zmpProbability(const RBDM::Vector3d & zmpLoc);

	// initialize the matrix which maps generalized velocities to parametrized velocities (including certain euler angle parametrization)
	void initEulTfMatrix(const RBDM::Vector3d & eul, std::string conv);

	// adapt the previously mentioned matrix (has to be adapted in each round)
	void adaptEulTfMatrix(RBDM::Vector3d eul, std::string conv);

	/* 
	* interpolates between two transformations. Also has a possibility to specify different convergence rates for orientation and position
	* alpha=0 returns the start transform, alpha=1 returns the end transform
	*/
	RBDM::SpatialTransform interpTransf(const RBDM::SpatialTransform & start,
										 const RBDM::SpatialTransform & end, 
										 double alphaTra, double alphaOri);
	
	// LIPM-dynamics functions
	void cmpLIPMdyn(ros::Duration dt);
	// compute com positon, velocity and acceleration
	void cmpCom(ros::Duration dt);
	// compute dcm and its rate of change
	void cmpDcm();

	// dynamic properties of the simplified LIPM model
	double getBodyMass(int bodyId);
	RBDM::Vector3d getBodyComPos(int bodyId);
	RBDM::SpatialTransform getFixedBodyTransform(std::string name);

	// forward differential kinematics
	void cmpFDkine();
	// functions for FDkine
	void resetJacobians();
	void resetTmpJacobian();
	void initJacobians();
	// rotation matrix to euler angles conversion
	RBDM::Vector3d rotm2eul(RBDM::Matrix3d rotm, std::string conv);
	// euler angles to rotation matrix conversion
	RBDM::Matrix3d eul2rotm(RBDM::Vector3d eul, std::string conv);
	// get the transformation matrix between generalized rotational velocities and parametrized euler velocities: w = B(e) * ep
	RBDM::Matrix3d rotVeloTf(RBDM::Vector3d eul, std::string conv);
	// get the 6x6 jacobian mapping from a point tbi on the floating base to the world frame
	RBDM::MatrixNd getBaseJacobian(const RBDM::Matrix3d & Rwb, 
									const RBDM::Vector3d & tbi);
	// update the world jacobian with a new floating base orientation estimation Rwb, new leg jacobian jacB and new endeffector positon tbi
	void updateWorldJacobian(RBDM::MatrixNd & jacW,
							 const RBDM::MatrixNd & jacB, 
							 const RBDM::Matrix3d & Rwb, 
							 const RBDM::Vector3d & tbi);
	// transform a vector to a skew symmetric matrix (equivalent to the cross product operation)
	RBDM::Matrix3d getSkewSymMat(const RBDM::Vector3d & tbi);

	// helper functions - multTfs is probably not needed
	void multTfs(const RBDM::SpatialTransform & Hl, 
				const RBDM::SpatialTransform & Hr, 
				RBDM::SpatialTransform & Hout);
	// copy internal variables to the Targets class (output of the module)
	void setTargets();
	// initialize the quaternion (w=1,x=0,y=0,z=0)
	void initQuaternion(Eigen::Quaterniond & quat);

	// print internal values
	void printInternalStates();

public:
	// constr. and destructor
	FloatingBaseEst(ros::NodeHandle & nh, bool debug);
	~FloatingBaseEst();

	// initialize the class (feed the initial angles)
	void init(const std::vector<double> &q);
	// reset the already initialized class
	void reset(const std::vector<double> &q);
	// execute the step of the algorithm
	void update(const std::vector<double> & q,
				const std::vector<double> & qp, 
				const std::vector<double> & qeff, 
				const std::vector<Vector4d> & fsen, 
				const std::vector<Eigen::Vector3d> & accel,
				const std::vector<Eigen::Vector3d> & gyro,
				phase_data pd,
				ros::Duration dt);

	// getters:
	// estimation results
	Targets getTargets(){return this->measTarg;}
	std::vector<WorldJacobian> & getConstraints();
	WorldJacobian getComJacobianW(){return (WorldJacobian)this->Jw_com;}
	WorldJacobian getBaseJacobianW(){return (WorldJacobian)this->Jw_b;}
	WorldJacobian getLefLegJacobianW(){return (WorldJacobian)this->Jw_eeL;}
	WorldJacobian getRigLegJacobianW(){return (WorldJacobian)this->Jw_eeR;}
	Eigen::Vector3d getZmp(){return (Eigen::Vector3d)this->zmp;}
	Eigen::Vector3d getCom(){return (Eigen::Vector3d)this->com;}
	Eigen::Vector3d getComp(){return (Eigen::Vector3d)this->comp;}
	Eigen::Vector3d getDcm(){return (Eigen::Vector3d)this->dcm;}
	Eigen::Vector3d getDcmp(){return (Eigen::Vector3d)this->dcmp;}
	Eigen::Vector3d getFPosL(){return (Eigen::Vector3d)this->Hw_Lee.r;}
	Eigen::Vector3d getFPosR(){return (Eigen::Vector3d)this->Hw_Ree.r;}
	// other constant variable that are needed by other modules
	double getLefLegForce(){return FLtot;}
	double getRigLegForce(){return FRtot;}
	Eigen::Vector3d getFootDims();
	double getModelMass();
};
