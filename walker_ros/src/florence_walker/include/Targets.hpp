#ifndef _TARGETS
#define _TARGETS

#include <iostream>
#include "definitions.hpp"

using namespace std;

// class for storing variables which are passed between modules
class Targets
{
public:
	Eigen::Vector3d zmp;					// zmp position in wcf
	Eigen::Vector3d com;					// com position in wcf
	Eigen::Vector3d comp;					// com velocity in wcf
	Eigen::Vector3d dcm;					// divergent component of motion in wcf
	Eigen::Vector3d dcmp;					// rate of change of the divergent component of motion in wcf
	Eigen::Vector3d pL, pR;					// end effector postion in wcf
	Eigen::Vector3d pLp, pRp;				// end effector translational velocity in wcf
	Eigen::Matrix3d RL, RR;					// rotation of the end effectors relative to the wcf
	Eigen::Vector3d Xfb;					// position of the base wrt wcf
	Eigen::Vector3d EulZYX;					// orientation of the base wrt wcf expressed in euler angle zyx convention (watch out order of angles in the vector is [roll_x,pitch_y,yaw_z] but convention is still zyx)
	Eigen::Matrix3d Rfb;					// orientation of the base wrt wcf
	Eigen::Matrix<double, NDOF, 1> Q;		// joint angles
	double FL, FR;							// left and right leg force
	Eigen::Vector3d zmplocL, zmplocR;		// local foot zmp value (in their respective foot frames)
	Targets(){
		zmp.setZero();
		com.setZero();
		comp.setZero();
		dcm.setZero();
		dcmp.setZero();
		pL.setZero();
		pR.setZero();
		pLp.setZero();
		pRp.setZero();
		RL.setZero();
		RR.setZero();
		Xfb.setZero();
		EulZYX.setZero();
		Rfb = Eigen::Matrix3d::Identity();
		Q.setZero();
		FL = 0;
		FR = 0;
		zmplocL.setZero();
		zmplocR.setZero();
	}
	~Targets(){}

	friend ostream & operator << (ostream & os, const Targets & targ){
		os << "------------------------------------------------" << endl;
		os << "Instance of targets containst following values: " << endl;
		os << "COM (world frame): [" << targ.com.transpose() << "]" << endl;
		os << "ZMP (world frame): [" << targ.zmp.transpose() << "]" << endl;
		os << "COMP (world frame): [" << targ.comp.transpose() << "]" << endl;
		os << "ZMP left (local foot frame): [" << targ.zmplocL.transpose() << "]" << endl;
		os << "ZMP right (local foot frame): [" << targ.zmplocR.transpose() << "]" << endl;
		os << "Left foot position is: [" << targ.pL.transpose() << "]" << endl;
		os << "Right foot position is: [" << targ.pR.transpose() << "]" << endl;
		os << "------------------------------------------------" << endl << endl;
	}
	
};

#endif