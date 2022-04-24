#include "Balancer.hpp"

using namespace std;
using namespace ros;
using namespace Eigen;

// read parameters from ros param cloud and initialize internal settings
Balancer::Balancer(NodeHandle & nh, bool debug)
: nh(nh)
{
	// set error if there is a problem
	this->err = false;

	// read all the required parameters from the param cloud
	if(this->err |= !this->nh.getParam("/tgen/zdes", this->zdes))
	{
		ROS_ERROR("BAL: Desired COM height parameter not found.");
	}else{
		this->omega = sqrt(GRAV/this->zdes);
	}

	vector<double> Kdcm_p_vec;
	if(this->err |= !this->nh.getParam("/bal/gains/Kdcm/prop",Kdcm_p_vec))
	{
		ROS_ERROR("BAL: DCM proportional gains not found.");
	}else{
		this->Kdcm_p = Matrix3d::Zero();
		this->Kdcm_p(0,0) = Kdcm_p_vec.at(0);
		this->Kdcm_p(1,1) = Kdcm_p_vec.at(1);
	}

	vector<double> Kdcm_i_vec;
	if(this->err |= !this->nh.getParam("/bal/gains/Kdcm/int",Kdcm_i_vec))
	{
		ROS_ERROR("BAL: DCM integral gains not found.");
	}else{
		this->Kdcm_i = Matrix3d::Zero();
		this->Kdcm_i(0,0) = Kdcm_i_vec.at(0);
		this->Kdcm_i(1,1) = Kdcm_i_vec.at(1);
	}

	vector<double> Kleak_vec;
	if(this->err |= !this->nh.getParam("/bal/gains/Kleak",Kleak_vec))
	{
		ROS_ERROR("BAL: Leakage factor  not found.");
	}else{
		this->Kleak = Matrix3d::Zero();
		this->Kleak(0,0) = Kleak_vec.at(0);
		this->Kleak(1,1) = Kleak_vec.at(0);
	}

	vector<double> Kzmp_vec;
	if(this->err |= !this->nh.getParam("/bal/gains/Kzmp",Kzmp_vec))
	{
		ROS_ERROR("BAL: ZMP gains not found.");
	}else{
		this->Kzmp = Matrix3d::Zero();
		this->Kzmp(0,0) = Kzmp_vec.at(0) * this->omega;
		this->Kzmp(1,1) = Kzmp_vec.at(1) * this->omega;
	}

	vector<double> Kcom_vec;
	if(this->err |= !this->nh.getParam("/bal/gains/Kcom",Kcom_vec))
	{
		ROS_ERROR("BAL: COM gains not found.");
	}else{
		this->Kcom = Matrix3d::Zero();
		this->Kcom(0,0) = Kcom_vec.at(0) * this->omega;
		this->Kcom(1,1) = Kcom_vec.at(1) * this->omega;
	}

	int act_int;
	if(this->err |= !this->nh.getParam("/bal/act_int", act_int))
	{
		ROS_ERROR("BAL: Integrator flag not found.");
	}else{
		this->activate_int = act_int!=0?true:false;
	}
}

// stabilizing control law for the robot
void Balancer::update(const Targets & meas, const Targets & cmd, Duration dt){
	this->update(meas.com, meas.zmp, meas.dcm, cmd.com, cmd.comp, cmd.dcm, cmd.dcmp, dt);
}

// stabilizing control law for the robot
void Balancer::update(Vector3d com_m,
					Vector3d zmp_m,
					Vector3d dcm_m,
					Vector3d com_d,
					Vector3d comp_d,
					Vector3d dcm_d,
					Vector3d dcmp_d,
					Duration dt)
{
	// control step sampling time
	double Ts = dt.toSec();

	// dcm error
	Vector3d dcm_err = dcm_d - dcm_m;
	// inclusion of leakage
	Vector3d derr = dcm_err - this->Kleak * dcm_int;
	derr *= this->activate_int?	1:0; 
	// integrate the error (with inclusion of leakage)
	dcm_int += Ts * derr;


	// compute the zmp reference
	Vector3d zmp_r = dcm_d - dcmp_d / this->omega + Kdcm_p * dcm_err + Kdcm_i * dcm_int;

	// velocity target of the COM
	this->comp_r = comp_d - Kzmp * (zmp_r - zmp_m) + Kcom * (com_d - com_m);
}

// reset the integrator and the com reference velocity (comp_r)
void Balancer::reset()
{
	comp_r.setZero();
	dcm_int.setZero();
}

// call the reset & maybe do some other stuff (in the future)
void Balancer::init(){
	this->reset();
}