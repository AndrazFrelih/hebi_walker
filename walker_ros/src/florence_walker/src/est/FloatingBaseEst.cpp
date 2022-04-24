#include "FloatingBaseEst.hpp"

using namespace std;
using namespace ros;
using namespace RigidBodyDynamics;
//using namespace RigidBodyDynamics::Math;  ---- potential collisions with Eigen

FloatingBaseEst::FloatingBaseEst(NodeHandle & nh, bool debug)
:nh(nh), debug(debug)
{

	ROS_INFO("FBEST: Starting the FloatingBaseEst node.");
	// load URDF
	rbdl_check_api_version(RBDL_API_VERSION);
	string path = ros::package::getPath("florence_description") + "/urdf/florenceRBDL.urdf";
	model = new Model();
	if (this->err |= !Addons::URDFReadFromFile(path.c_str(), model, false)) {
		ROS_ERROR("FBEST: Error loading the URDF file!");
	}

	if(this->debug){
		cout << "Degree of freedom overview:" << endl;
		cout << Utils::GetModelDOFOverview(*model);
		cout << "Model Hierarchy:" << endl;
		cout << Utils::GetModelHierarchy(*model);
	}

	this->readParameters();
	
	// initialize the rest of variables
	// resize Q, Qp and Qeff
	this->Q    = Math::VectorNd::Constant(NDOF,1, 0);
	this->Qp   = Math::VectorNd::Constant(NDOF,1, 0);
	this->Qeff = Math::VectorNd::Constant(NDOF,1, 0);

	// resize the transformations
	this->Hb_i.resize(NDOF);
	this->cstr_jlist.resize(4);

	// initialize quaternions
	this->initQuaternion(this->quatL_old);
	this->initQuaternion(this->quatR_old);

	// identity transformation (eye(4))
	this->identity.E = Math::Matrix3d::Identity();
	this->identity.r = Math::Vector3d::Zero();
	// execute functions that depend on the success of previous steps
	if(!this->err){
		// store fixed transformations
		this->storeFixedTransforms();
		// get the model mass
		this->totalMass = this->getModelMass();
		// initialize jacobians
		this->initJacobians();
		// matrix
		this->initEulTfMatrix(Math::Vector3d::Zero(), this->eulconv);
	}else{
		ROS_ERROR("Cannot finish initializing the class due to missing parameters.");
	}
}

FloatingBaseEst::~FloatingBaseEst(){}

// DONE read parameters from the cloud
void FloatingBaseEst::readParameters()
{
	this->err = false;
	// read all the required parameters from the param cloud
	if(this->err |= !this->nh.getParam("/est/fsen/fnames/left",this->lfs_fnames)){
		ROS_ERROR("FBEST: Left load cell frames not found.");
		return;
	}

	if(this->err |= !this->nh.getParam("/est/fsen/fnames/right",this->rfs_fnames)){
		ROS_ERROR("FBEST: Right load cell frames not found.");
		return;
	}

	if(this->err |= !this->nh.getParam("/est/endeffector/names/left",this->lee_name)){
		ROS_ERROR("FBEST: Left EE name not found.");
		return;
	}

	if(this->err |= !this->nh.getParam("/est/endeffector/names/right",this->ree_name)){
		ROS_ERROR("FBEST: Right EE name not found.");
		return;
	}

	if(this->err |= !this->nh.getParam("/est/eulconv", this->eulconv)){
		ROS_ERROR("FBEST: Eul. conv. param. not found.");
		return;
	}

	if(this->err |= !this->nh.getParam("/est/com/alpha_cp", this->alpha_cp)){
		ROS_ERROR("FBEST: Com velocity filter contant not found.");
		return;
	}

	if(this->err |= !this->nh.getParam("/est/com/alpha_cpp", this->alpha_cpp)){
		ROS_ERROR("FBEST: Com acceleration filter contant not found.");
		return;
	}

	if(this->err |= !this->nh.getParam("/tgen/zdes", this->zdes)){
		ROS_ERROR("FBEST: Desired height param. not found.");
		return;
	}else{
		this->omega = sqrt(GRAV/this->zdes);
	}
	
	if(this->err |= !this->nh.getParam("/est/prob/coef/zmp", this->pzmp_c)){
		ROS_ERROR("FBEST: ZMP probability coefficients not found.");
		return;
	}	

	if(this->err |= !this->nh.getParam("/est/prob/coef/cont", this->pcont_c)){
		ROS_ERROR("FBEST: Contact probability coefficients not found.");
		return;
	}

	if(this->err |= !this->nh.getParam("/est/sfus/Wimu", this->Wimu)){
		ROS_ERROR("FBEST: Sensor fusion IMU weight not found.");
		return;
	}	

	if(this->err |= !this->nh.getParam("/est/sfus/imu/names", this->IMUnames)){
		ROS_ERROR("FBEST: IMU names not found.");
		return;
	}


	if(this->err |= !this->nh.getParam("/est/sfus/alpha_tra", this->alphaTra_drift)){
		ROS_ERROR("FBEST: Drift factor not found.");
		return;
	}

	if(this->err |= !this->nh.getParam("/est/sfus/alpha_ori", this->alphaOri_drift)){
		ROS_ERROR("FBEST: Drift factor not found.");
		return;
	}	
}

// DONE
void FloatingBaseEst::init(const vector<double> &q)
{
	this->setWcf(q);
	this->resetIntegrators();
}

// DONE
void FloatingBaseEst::reset(const vector<double> &q)
{
	this->init(q);
	this->resetJacobians();
}

// DONE set all the values that are accumulating over time to zero (velocities and accelerations)
void FloatingBaseEst::resetIntegrators()
{
	this->initComCmp = false;
	this->comp.setZero();
	this->compp.setZero();
	this->comp_b.setZero();
	this->compp_b.setZero();
	this->comp_b_old.setZero();
	this->compp_b_old.setZero();
	this->cmpDcm();
}

//DONE
void FloatingBaseEst::setWcf(const vector<double> &q)
{
	// first angle measurements have to be available
	this->updateJntMeasurements(q);

	// get all the required body transformations
	this->cmpFKine();

	/* 
	* Initialize the believed location of feet in the world frame (fix the world frame):
	* The definition is done over the base kinematics, so both transformations are dependent 
	* (not arbitrary) - wcf is placed between the feet. Yaw angle should be the same as the torso yaw
	* angle. Since feet are not necessarly placed straight with regard to torso when it comes to yaw,
	* their respective rotations need to be computed
	*/
	double x, xL, xR;
	double y, yL, yR;
	// store feet positions in the base frame
	xL = this->Hb_Lee.r(0);
	xR = this->Hb_Ree.r(0);
	yL = this->Hb_Lee.r(1);
	yR = this->Hb_Ree.r(1);
	// compute the position in the middle of both feet
	x = (xL + xR)/2;
	y = (yL + yR)/2;

	// compute the feet positions in the frame between the feet
	Math::Vector3d tw_eeL(-x+xL, -y+yL, 0);
	Math::Vector3d tw_eeR(-x+xR, -y+yR, 0);

	// only yaw interests us (we take zyx because we want to see only how the foot needs to be rotated to allign with the torso)
	Math::Vector3d eulL = rotm2eul(this->Hb_Lee.E, "xyz");
	double yawL = eulL(2);

	Math::Vector3d eulR = rotm2eul(this->Hb_Ree.E, "xyz");
	double yawR = eulR(2);
	
	// define the wcf relative to the left leg
	Eigen::Matrix3d mL = Eigen::AngleAxisd(yawL, Eigen::Vector3d::UnitZ()).toRotationMatrix();
	this->Hw_Lee.E = (Math::Matrix3d) mL;
	this->Hw_Lee.r = tw_eeL;


	// define the wcf relative to the right leg
	Eigen::Matrix3d mR = Eigen::AngleAxisd(yawR, Eigen::Vector3d::UnitZ()).toRotationMatrix();
	this->Hw_Ree.E = (Math::Matrix3d)mR;
	this->Hw_Ree.r = tw_eeR;

	if(this->debug){
		cout<<"Initial euler angles of the left foot: "<<eulL.transpose()<<endl;
		cout<<"Left foot rotation matrix: "<<endl<<this->Hb_Lee.E<<endl;
		cout<<"Initial euler angles of the right foot: "<<eulR.transpose()<<endl;
		cout<<"Right foot rotation matrix: "<<endl<<this->Hb_Ree.E<<endl;
		cout<<"HwLee: "<<endl<<"RwLee:"<<endl<<this->Hw_Lee.E <<endl<<"twLee:"<<this->Hw_Lee.r<<endl;
		cout<<"HwRee: "<<endl<<"RwRee:"<<endl<<this->Hw_Ree.E <<endl<<"twRee:"<<this->Hw_Ree.r<<endl;
	}
}

//DONE
void FloatingBaseEst::storeFixedTransforms()
{
	// end effector fixed transforms
	this->HLeeFix = this->getFixedBodyTransform(this->lee_name);
	this->HReeFix = this->getFixedBodyTransform(this->ree_name);

	// load cell fixed transforms
	this->HLfootFix.resize(this->lfs_fnames.size()); // in qL6 output frame
	this->teeL_fpts.resize(this->lfs_fnames.size()); // in EE_L frame

	this->HRfootFix.resize(this->rfs_fnames.size()); // in qR6 output frame
	this->teeR_fpts.resize(this->rfs_fnames.size()); // in EE_R frame
	
	vector<FixedBody> bodies = model->mFixedBodies;
	Math::SpatialTransform HL,HR, HLeeFixInv, HReeFixInv, HLtest;
	HLeeFixInv = this->HLeeFix.inverse();
	HReeFixInv = this->HReeFix.inverse();
	for(int i=0; i<NLC; i++){
		this->HLfootFix.at(i) = this->getFixedBodyTransform(this->lfs_fnames.at(i));
		this->HRfootFix.at(i) = this->getFixedBodyTransform(this->rfs_fnames.at(i));
		// store locations of sensors in the left end effector foot frame
		this->multTfs(HLeeFixInv, this->HLfootFix.at(i), HL);
		this->teeL_fpts.at(i) = HL.r;
		// store locations of sensors in the right end effector foot frame
		this->multTfs(HReeFixInv, this->HRfootFix.at(i), HR);
		this->teeR_fpts.at(i) = HR.r;
	}

	// store IMU transforms (only the two IMUs fixed to the base of the robot are interfaced)
	this->Hb_Limu = this->getFixedBodyTransform(this->IMUnames.at(0));
	this->Hb_Rimu = this->getFixedBodyTransform(this->IMUnames.at(1));
}

// DONE
void FloatingBaseEst::update(const vector<double> & q,
							const vector<double> & qp, 
							const vector<double> & qeff, 
							const vector<Vector4d> & fsen, 
							const vector<Eigen::Vector3d> & accel,
							const vector<Eigen::Vector3d> & gyro,
							phase_data pd,
							Duration dt)
{

	// update the internal state of the robot
	this->updateJntMeasurements(q, qp, qeff);

	// get all the required transformations
	this->cmpFKine();

	// compute local feet zmps 
	this->cmpZmpLocal(this->zmplocL, this->FLtot, fsen.at(0), this->teeL_fpts);
	this->cmpZmpLocal(this->zmplocR, this->FRtot, fsen.at(1), this->teeR_fpts);
	this->Ftot = this->FLtot + this->FRtot;

	// estimate the position & orientation of the robot in the wcf
	this->estFloatingBase(accel, gyro, pd, dt);

	// compute the global zmp
	this->cmpZmpGlobal();

	// compute com related states: position C = [x,y,z] and velocity Cp = [xp,yp,zp]
	this->cmpLIPMdyn(dt);

	// compute required jacobians
	this->cmpFDkine();

	// update measured targets
	this->setTargets();

	if(this->debug){
		// if debug is true, this class is definitely not real time anymore
		this->printInternalStates();
	}
}

// DONE - only update 
void FloatingBaseEst::updateJntMeasurements(const vector<double> &q)
{
	std::vector<double> zeros(q.size(),0);
	this->updateJntMeasurements(q, zeros, zeros);
}

// DONE
void FloatingBaseEst::updateJntMeasurements(const vector<double> &q, const vector<double> &qp, const vector<double> &qeff)
{
	for(int i = 0; i < NDOF; i++){
		this->Q(i)    = q.at(i);
		this->Qp(i)   = qp.at(i);
		this->Qeff(i) = qeff.at(i);
	}
	// update the RBDL kinematics (model contains updated values after this step)
	bool updateVelo = false;
	if(updateVelo){
		// IF VELOCITIES ARE INCLUDED, THE ALGORITHM GETS WAY TO SLOW (EXECUTION INCREASES BY 2-4ms)
		UpdateKinematicsCustom(*(this->model),&(this->Q),&(this->Qp),NULL);
	}else{
		UpdateKinematicsCustom(*(this->model),&(this->Q),NULL,NULL);
	}
}

// DONE
void FloatingBaseEst::cmpFKine()
{
	// compute all joint to base transforms (proximal convention)
	// start from one, because index 0 is the base transformation
	for(int i = 1; i < NDOF + 1; i++){
		this->Hb_i.at(i-1).E = CalcBodyWorldOrientation(*(this->model), this->Q, i, false).transpose();
		this->Hb_i.at(i-1).r = CalcBodyToBaseCoordinates(*(this->model), this->Q, i, Math::Vector3d::Zero(3,1), false);
	}

	// end effector transformations (update Hb_Lee & Hb_Ree)
	this->multTfs(this->Hb_i.at(NDOF/2-1), this->HLeeFix, this->Hb_Lee);
	this->multTfs(this->Hb_i.at( NDOF -1), this->HReeFix, this->Hb_Ree);
}

// DONE - passing by reference so the value can be changed
void FloatingBaseEst::cmpZmpLocal(Math::Vector3d & zmpLoc, double & Ftot, const Vector4d & fsen_i, const vector<Math::Vector3d> & pts)
{
	double px, py;
	//simplest way of computing the zmp
	Ftot = max(0.0, fsen_i.sum());
	zmpLoc.setZero();
	if(Ftot > 0){
		for(int i=0; i<fsen_i.size(); i++){
			px = pts.at(i)[0];
			py = pts.at(i)[1];
			// zmp x
			zmpLoc(0) += px * fsen_i(i);
			// zmp y
			zmpLoc(1) += py * fsen_i(i);
		}
		zmpLoc(0) = zmpLoc(0) / Ftot;
		zmpLoc(1) = zmpLoc(1) / Ftot;
	}
}

// DONE
void FloatingBaseEst::estFloatingBase(const vector<Eigen::Vector3d> &accel, const vector<Eigen::Vector3d> &gyro, phase_data pd, Duration dt)
{
	this->aL_b = this->Hb_Limu.E * accel.at(0);
	this->aR_b = this->Hb_Rimu.E * accel.at(1);
	this->wL_b = this->Hb_Limu.E * gyro.at(0);
	this->wR_b = this->Hb_Rimu.E * gyro.at(1);

	Eigen::Quaterniond quatIMU_L, quatIMU_R, quatIMU;
	filterUpdate(this->aL_b, this->wL_b, quatIMU_L, this->quatL_old, dt.toSec());
	filterUpdate(this->aR_b, this->wR_b, quatIMU_R, this->quatR_old, dt.toSec());

	// store the old value of quaternion for the next iteration
	this->quatL_old = quatIMU_L;
	this->quatR_old = quatIMU_R;


	// merge quaternions
	quatIMU.w() = (quatIMU_L.w() + quatIMU_R.w()) / 2;
	quatIMU.x() = (quatIMU_L.x() + quatIMU_R.x()) / 2;
	quatIMU.y() = (quatIMU_L.y() + quatIMU_R.y()) / 2;
	quatIMU.z() = (quatIMU_L.z() + quatIMU_R.z()) / 2;
	quatIMU.normalize();

	// estimation of the rotation matrix from IMU measurement
	Math::Matrix3d R_IMU = (Math::Matrix3d) quatIMU.toRotationMatrix();

	// kinematic estimates for the rotation matrix
	Math::SpatialTransform Hw_b_estL, Hw_b_estR;
	this->multTfs(this->Hw_Lee,this->Hb_Lee.inverse(), Hw_b_estL);
	this->multTfs(this->Hw_Ree,this->Hb_Ree.inverse(),Hw_b_estR);

	// compute weights that determine how the measurements will get fused
	double WL_zmp, WR_zmp, WL_cont, WR_cont;
	double WL, WR, eps;
	WL_zmp = zmpProbability(this->zmplocL);
	WR_zmp = zmpProbability(this->zmplocR);
	WL_cont = contactProbability(this->FLtot);
	WR_cont = contactProbability(this->FRtot);
	WL = WL_zmp * WL_cont;
	WR = WR_zmp * WR_cont;
	eps = 1e-6;
	double W_norm1 = 1/(WR+WL+eps);
	double W_norm2 = 1/(WR+WL+this->Wimu+eps);
	double Wimu_n = this->Wimu*W_norm2;
	double WL_n = WL*W_norm1;
	double WR_n = WR*W_norm1;
	double Wkine = WR/(WL+WR+eps);
	double Wfuse = (WL+WR)/(WL+WR+this->Wimu+eps);

	// remove the yaw from the IMU measurement (if imu is used, this should be tested - rotm2eul is not always returning consistent results)
	Eigen::Vector3d eulIMU = rotm2eul(R_IMU, this->eulconv);
	eulIMU(2) = 0;
	R_IMU = (Math::Matrix3d) eul2rotm(eulIMU, this->eulconv);

	// average the orientations based on the computed weights
	Eigen::Quaterniond quatLleg(Hw_b_estL.E), quatRleg(Hw_b_estR.E);
	Eigen::Quaterniond quatKine = quatLleg.slerp(Wkine, quatRleg);
	Eigen::Quaterniond quatFuse = quatIMU.slerp(Wfuse, quatKine);

	// store the rotation matrix	
	this->Hw_b.E = quatFuse.toRotationMatrix();
	cout<<"this->Hw_b.E"<<endl<<this->Hw_b.E<<endl;
 	this->Eul = (Math::Vector3d)rotm2eul(this->Hw_b.E,this->eulconv);

	// estimate the position of the floating base wrt WF
	this->X = (WL * (this->Hw_Lee.r - this->Hw_b.E * this->Hb_Lee.r) + WR * (this->Hw_Ree.r - this->Hw_b.E * this->Hb_Ree.r)) * W_norm1;
	// store the translation vector
	this->Hw_b.r = this->X;

	//compute the drift of transformations
    Math::SpatialTransform HLtmp, HL_drift; 
    this->multTfs(this->Hw_Lee.inverse(), this->Hw_b, HLtmp);
    this->multTfs(HLtmp, this->Hb_Lee, HL_drift);
    Math::SpatialTransform HRtmp, HR_drift;
    this->multTfs(this->Hw_Ree.inverse(), this->Hw_b, HRtmp);
    this->multTfs(HRtmp, this->Hb_Ree, HR_drift);

    //compensation factor alpha_drift (between 0 [no convg] and 1 [full comp]) - has to be tuned
    // it is correct that left and right side are inverted, as stance foot will not drift, however it determines how much the swing foot will
    double alphaL_tra = this->alphaTra_drift * WR;
    double alphaR_tra = this->alphaTra_drift * WL;
    double alphaL_ori = this->alphaOri_drift * WR;
    double alphaR_ori = this->alphaOri_drift * WL;
    Math::SpatialTransform HL_corr = this->interpTransf(this->identity, HL_drift, alphaL_tra, alphaL_ori);
    Math::SpatialTransform HR_corr = this->interpTransf(this->identity, HR_drift, alphaR_tra, alphaR_ori);

    Math::SpatialTransform HL_Leetmp, HL_Reetmp; 
    // apply the correction mitigating the drift of feet
    this->multTfs(this->Hw_Lee, HL_corr, HL_Leetmp);
    this->multTfs(this->Hw_Ree, HR_corr, HL_Reetmp);
    this->Hw_Lee = HL_Leetmp;
    this->Hw_Ree = HL_Reetmp;
}

// DONE
Math::SpatialTransform FloatingBaseEst::interpTransf(const Math::SpatialTransform & src, const Math::SpatialTransform & snk, double alphaTra, double alphaOri)
{
	// make sure that convergence factors are bounded
	alphaTra = min(max(alphaTra,0.0),1.0);
	alphaOri = min(max(alphaOri,0.0),1.0);

	// interpolate between rotation matrices (transform them to quaternions)
	Eigen::Quaterniond q_src((Eigen::Matrix3d)src.E);
	Eigen::Quaterniond q_snk((Eigen::Matrix3d)snk.E);
	Eigen::Quaterniond q_res = q_src.slerp(alphaOri, q_snk); // this is not correct (1-alpha should be alpha!!)
	// interpolate between translation vectors
	Math::Vector3d v_res = (1-alphaTra) * src.r + alphaTra * snk.r;
	// construct the transformation
	Math::SpatialTransform Hres;
	Hres.E = q_res.toRotationMatrix();
	Hres.r = v_res;
	// return the result
	return Hres;
}

// DONE
double FloatingBaseEst::contactProbability(double Fleg_tot)
{
	// the probability is modelled by a polynomial p(x) = k0 + k1*x + k2*x² + k3*x³ + k4*x⁴;
	double prob = 0;
	double mult = 1;
	for(int i=0; i<this->pcont_c.size(); i++){
		prob += mult * pcont_c.at(i);
		mult *= Fleg_tot;  	
	}
	// saturate the probability (since polynomial does not extrapolate well, probability has to be brought back to an interval between 0 & 1)
	if (Fleg_tot>this->totalMass*GRAV){
		return 1.0f;
	}else if(Fleg_tot<0){
		return 0.0f;
	}else{
		return min(max(0.0,prob*3),1.0);
	}
}

// DONE - zmp should be inside the foot by design
double FloatingBaseEst::zmpProbability(const Math::Vector3d & zmpLoc)
{
	// probability is modelled with the following function p(x,y) = k0 + k1 * x² + k2 * y²;
	double prob = this->pzmp_c.at(0) + this->pzmp_c.at(1) * pow(zmpLoc(0),2) + this->pzmp_c.at(2) * pow(zmpLoc(1),2);
	// since polynomials do not extrapolate, we need to keep the output bounded by saturating it
	return min(max(0.0,prob*3),1.0);
}

//DONE
void FloatingBaseEst::cmpZmpGlobal()
{
	Math::Vector3d zmpbaseL, zmpbaseR, zmpgloL, zmpgloR;
	Math::Vector3d zmpgloL2, zmpgloR2;
	
	// first way of computing it
	zmpgloL = this->Hw_Lee.E * this->zmplocL + this->Hw_Lee.r;
	zmpgloR = this->Hw_Ree.E * this->zmplocR + this->Hw_Ree.r;

	// second way of computing it (if zmp was needed in base frame as well)
	zmpbaseL = this->Hb_Lee.E * this->zmplocL + this->Hb_Lee.r;
	zmpbaseR = this->Hb_Ree.E * this->zmplocR + this->Hb_Ree.r;
	zmpgloL2 = this->Hw_b.E * zmpbaseL + this->Hw_b.r; 
	zmpgloR2 = this->Hw_b.E * zmpbaseR + this->Hw_b.r;

	// join both zmps
	this->zmp = (this->FLtot * zmpgloL + this->FRtot * zmpgloR) / this->Ftot;
}

//TODO attachment jaccobian
void FloatingBaseEst::cmpFDkine()
{
	// jacobian matrices have to be set to zero. Nonzero elements do not get updated
	this->resetJacobians();

	// get end-effector jaccobians
	CalcPointJacobian6D(*(this->model), this->Q, NDOF/2, Math::Vector3d::Zero(), this->Jb_eeL, false);
	CalcPointJacobian6D(*(this->model), this->Q,   NDOF, Math::Vector3d::Zero(), this->Jb_eeR, false);

	// get body-com jacobians and average them over their respective masses to get the full jacobian
	for(int i = 0; i < NDOF + 1; i++){
		this->resetTmpJacobian(); // all inputs have to be set to 0 
		CalcPointJacobian6D(*(this->model), this->Q, i, this->getBodyComPos(i), this->Jb_com_i, false);
		this->Jb_com += this->getBodyMass(i) * this->Jb_com_i;
	}
	this->Jb_com = this->Jb_com / this->totalMass;

	// TODO compute the attachment jacobian (for the compensation of the gas-spring)



	// build World jacobians
	updateWorldJacobian(this->Jw_com_g, this->Jb_com, this->Hw_b.E, this->com_b);
	updateWorldJacobian(this->Jw_eeL_g, this->Jb_eeL, this->Hw_b.E, this->Hb_Lee.r);
	updateWorldJacobian(this->Jw_eeR_g, this->Jb_eeR, this->Hw_b.E, this->Hb_Ree.r);
	updateWorldJacobian(this->Jw_b_g,   this->Jb_b,   this->Hw_b.E, Math::Vector3d::Zero());

	// perform the general-angles 2 euler-angles mapping
	this->adaptEulTfMatrix(this->Eul, this->eulconv);
	this->Jw_com   = this->Jw_com_g * this->EulTfMatrix;
	this->Jw_eeL = this->Jw_eeL_g * this->EulTfMatrix;
	this->Jw_eeR = this->Jw_eeR_g * this->EulTfMatrix;
	this->Jw_b   = this->Jw_b_g   * this->EulTfMatrix;

}


//DONE
void FloatingBaseEst::cmpLIPMdyn(Duration dt){
	this->cmpCom(dt);
	this->cmpDcm();
}

//DONE
void FloatingBaseEst::cmpCom(Duration dt)
{
	double mass;
	Math::Vector3d com;
	Utils::CalcCenterOfMass(*model, this->Q, this->Qp, NULL, mass, com, NULL, NULL, NULL, NULL, false);

	double massBase = this->getBodyMass(0);
	Math::Vector3d comBase = this->getBodyComPos(0);
	
	double mass_compensated = (massBase + mass);
	this->com_b = (comBase*massBase + com*mass) / mass_compensated;
	if(this->initComCmp){
		this->com_b_old = this->com_b;
		this->initComCmp = true;
	}

	// velocity and acceleration
	// update new values (work with local values so that FB-estimation noise & bias do not get included here)
	this->comp_b = this->alpha_cp * (this->com_b - this->com_b_old) / dt.toSec() + (1-alpha_cp) * this->comp_b_old;
	this->compp_b = this->alpha_cpp * (this->comp_b - this->comp_b_old) / dt.toSec() + (1-alpha_cpp) * this->compp_b_old;
	
	// overwrite old values
	this->com_b_old = this->com_b;
	this->comp_b_old = this->comp_b;
	this->compp_b_old = this->compp_b;

	// floating base position and orientation should be ready by here
	this->com = this->Hw_b.E * this->com_b + this->Hw_b.r - Math::Vector3d(0.02,0,0);
	this->comp = this->Hw_b.E * this->comp_b;
	this->compp = this->Hw_b.E * this->compp_b;
}

//DONE
void FloatingBaseEst::cmpDcm()
{
	this->dcm = this->com + this->comp / this->omega;
	this->dcmp = this->omega * (this->dcm - this->zmp);
}


// DONE - 
void FloatingBaseEst::multTfs(const Math::SpatialTransform & Hl, const Math::SpatialTransform & Hr, Math::SpatialTransform & Hout)
{
	Hout.E = Hl.E * Hr.E;
	Hout.r = Hl.E * Hr.r + Hl.r;
}

//DONE - helper function: get the mass of a certain body (fixed link is at index 0, left leg links are from 1...NDOF/2, right leg links are from NDOF/2+1...NDOF)
double FloatingBaseEst::getBodyMass(int bodyId)
{
	return this->model->mBodies.at(bodyId).mMass;
}

//DONE - helper function: get full robot mass
double FloatingBaseEst::getModelMass()
{
	double mtot = 0;
	for(int i = 0; i < NDOF + 1; i++){
		mtot += this->getBodyMass(i);
	}
	return mtot;
}

//DONE - helper function: get the com position of a certain body in the link frame (fixed link is at index 0, left leg links are from 1...NDOF/2, right leg links are from NDOF/2+1...NDOF)
Math::Vector3d FloatingBaseEst::getBodyComPos(int bodyId)
{
	return this->model->mBodies.at(bodyId).mCenterOfMass;
}

//DONE - helper function:
Math::SpatialTransform FloatingBaseEst::getFixedBodyTransform(string name)
{
	return this->model->mFixedBodies.at(this->model->GetBodyId(name.c_str()) - this->model->fixed_body_discriminator).mParentTransform;
}

//DONE - helper function: jacobians need to be set to zero before fed to the RBDL get6Djacobian fcn
void FloatingBaseEst::resetJacobians(){
	this->resetTmpJacobian();
	this->Jb_eeL = Math::MatrixNd::Zero(NDOF_FB,NDOF);
	this->Jb_eeR = Math::MatrixNd::Zero(NDOF_FB,NDOF);
	this->Jb_com = Math::MatrixNd::Zero(NDOF_FB,NDOF);
	this->Jb_b = Math::MatrixNd::Zero(NDOF_FB,NDOF);
}

//DONE - helper function: has to be reset more regularly (hence own fcn)
void FloatingBaseEst::resetTmpJacobian(){
	this->Jb_com_i = Math::MatrixNd::Zero(NDOF_FB,NDOF);
}

void FloatingBaseEst::initJacobians()
{
	// jacobians that need regulary resets
	this->resetJacobians();
	// jacobians that only need to be initialized
	this->Jw_com_g = Math::MatrixNd::Zero(NDOF_FB, NDOF_FB+NDOF);
	this->Jw_eeL_g = Math::MatrixNd::Zero(NDOF_FB, NDOF_FB+NDOF);
	this->Jw_eeR_g = Math::MatrixNd::Zero(NDOF_FB, NDOF_FB+NDOF);
	this->Jw_b_g   = Math::MatrixNd::Zero(NDOF_FB, NDOF_FB+NDOF); 
	this->Jw_com   = Math::MatrixNd::Zero(NDOF_FB, NDOF_FB+NDOF);
	this->Jw_eeL = Math::MatrixNd::Zero(NDOF_FB, NDOF_FB+NDOF); 
	this->Jw_eeR = Math::MatrixNd::Zero(NDOF_FB, NDOF_FB+NDOF);
	this->Jw_b   = Math::MatrixNd::Zero(NDOF_FB, NDOF_FB+NDOF);
}

//DONE compute euler angles from the given rotation matrix rotm. Currently only xyz and zyx conventions are available
Math::Vector3d FloatingBaseEst::rotm2eul(Math::Matrix3d rotm, string conv)
{	
	if(conv=="xyz"){
		//xyz convention
		return rotm.eulerAngles(0, 1, 2);
	}else{
		//zyx convention
		Math::Vector3d ea;
		ea = rotm.eulerAngles(2, 1, 0);
		// reorder so that the angles are still in x,y,z order 
		return Math::Vector3d(ea(2),ea(1),ea(0));
	}
}

Math::Matrix3d FloatingBaseEst::eul2rotm(Math::Vector3d eul, string conv)
{
	Eigen::Matrix3d mat;
	if(conv == "xyz"){
		// xyz convention
		mat = (Eigen::AngleAxisd(eul[0], Eigen::Vector3d::UnitX())
			 	*Eigen::AngleAxisd(eul[1], Eigen::Vector3d::UnitY())
			 	*Eigen::AngleAxisd(eul[2], Eigen::Vector3d::UnitZ())); 
	}else{
		// zyx convention
		mat = (Eigen::AngleAxisd(eul[2], Eigen::Vector3d::UnitZ())
				*Eigen::AngleAxisd(eul[1], Eigen::Vector3d::UnitY())
				*Eigen::AngleAxisd(eul[0], Eigen::Vector3d::UnitX()));
	}	
	return (Math::Matrix3d)	mat;
}

//DONE transformation for the world jacobian - essentially transforms general velocities (wx,wy,wz) to parametrized euler velocities (epx, epy, epz)
void FloatingBaseEst::initEulTfMatrix(const Math::Vector3d & eul, string conv)
{
	// euler angles have to be in x,y,z order (and either constructed using xyz or zyx convention)
	this->EulTfMatrix = Math::MatrixNd::Zero(NDOF_FB+NDOF,NDOF_FB+NDOF);
	this->EulTfMatrix.block<NDOF_FB/2,NDOF_FB/2>(0,0) = Math::Matrix3d::Identity();
	this->adaptEulTfMatrix(eul, conv);
	this->EulTfMatrix.block<NDOF,NDOF>(NDOF_FB,NDOF_FB) = (Math::MatrixNd)Eigen::Matrix<double,NDOF,NDOF>::Identity();
}

//DONE transformation needs to be adapted depending on the value of euler angles (only a submatrix)
void FloatingBaseEst::adaptEulTfMatrix(Math::Vector3d eul, string conv)
{
	// euler angles have to be in x,y,z order (and either constructed using xyz or zyx convention)
	this->EulTfMatrix.block<NDOF_FB/2,NDOF_FB/2>(NDOF_FB/2,NDOF_FB/2) = this->rotVeloTf(eul, conv);
}

//DONE construct the required submatrix for EulTfMatrix
Math::Matrix3d FloatingBaseEst::rotVeloTf(Math::Vector3d eul, string conv){
	Math::Matrix3d tf;
	if(conv=="xyz"){
		//xyz convention
		tf(0,0) = 1;
		tf(0,1) = 0;
		tf(0,2) = +sin(eul(1)); 
		tf(1,0) = 0;
		tf(1,1) = +cos(eul(0));
		tf(1,2) = -cos(eul(1))*sin(eul(0));
		tf(2,0) = 0;
		tf(2,1) = +sin(eul(0));
		tf(2,2) = +cos(eul(1))*cos(eul(0));
	}else{
		//zyx convention
		tf(0,0) = +cos(eul(1))*cos(eul(2));
		tf(0,1) = -sin(eul(2));
		tf(0,2) = 0;
		tf(1,0) = +cos(eul(1))*sin(eul(2));
		tf(1,1) = +cos(eul(2));
		tf(1,2) = 0;
		tf(2,0) = -sin(eul(1));
		tf(2,1) = 0;
		tf(2,2) = 1;
	}
	return tf;
}

//DONE part of the world jacobian (arguments are: Rwb - tf from base to world; tbi - the reference point on the robot expressed in the base frame)
Math::MatrixNd FloatingBaseEst::getBaseJacobian(const Math::Matrix3d & Rwb, const Math::Vector3d & tbi){
	Math::MatrixNd Jac  = Math::MatrixNd::Zero(NDOF_FB,NDOF_FB);
	Jac.block<3,3>(0,0) = Math::Matrix3d::Identity();
	Jac.block<3,3>(0,3) = - Rwb * getSkewSymMat(tbi);
	Jac.block<3,3>(3,3) =   Rwb;
	return Jac;
}

//DONE jacobians need to be updated every iteration
void FloatingBaseEst::updateWorldJacobian(Math::MatrixNd & jacW, const Math::MatrixNd & jacB, const Math::Matrix3d & Rwb, const Math::Vector3d & tbi)
{
	jacW.block<NDOF_FB,NDOF_FB>(0,0) = getBaseJacobian(Rwb, tbi);
	// swap rotational and translational velocities
	jacW.block<NDOF_FB/2,NDOF>(0,NDOF_FB) = Rwb * jacB.block<NDOF_FB/2,NDOF>(NDOF_FB/2,0);
	jacW.block<NDOF_FB/2,NDOF>(NDOF_FB/2,NDOF_FB) = Rwb * jacB.block<NDOF_FB/2,NDOF>(0,0);
}

//DONE - helper function: returns a skew-sym matrix of the vector (cross product matrix)
Math::Matrix3d FloatingBaseEst::getSkewSymMat(const Math::Vector3d & tbi){
	Math::Matrix3d skewMat = Math::Matrix3d::Zero(); 
	skewMat <<       0, -tbi(2),  tbi(1),
    			tbi(2),       0, -tbi(0),
    		   -tbi(1),  tbi(0),       0;
   	return skewMat;
}


vector<WorldJacobian> & FloatingBaseEst::getConstraints(){
	this->cstr_jlist.at(0) = this->getComJacobianW();
	this->cstr_jlist.at(1) = this->getBaseJacobianW();
	this->cstr_jlist.at(2) = this->getLefLegJacobianW();
	this->cstr_jlist.at(3) = this->getRigLegJacobianW();
	return this->cstr_jlist;
}

void FloatingBaseEst::setTargets(){
	// com data
	this->measTarg.com = this->com;
	this->measTarg.comp = this->comp;
	this->measTarg.zmp = this->zmp;
	this->measTarg.dcm = this->dcm;
	this->measTarg.dcmp = this->dcmp;

	// leg position
	this->measTarg.pL = (Eigen::Vector3d)this->Hw_Lee.r;
	this->measTarg.pR = (Eigen::Vector3d)this->Hw_Ree.r;

	// leg orientation - set the yaw of the feet to zero (leave the degree of freedom open)
	Eigen::Vector3d eulL = this->rotm2eul(this->Hw_Lee.E,this->eulconv);
	Eigen::Vector3d eulR = this->rotm2eul(this->Hw_Ree.E,this->eulconv);

	eulL(2) = 0;
	eulR(2) = 0;
	this->measTarg.RL = this->eul2rotm(eulL,this->eulconv);
	this->measTarg.RR = this->eul2rotm(eulL,this->eulconv);

	// store the floating base orientation
	this->measTarg.Rfb = (Eigen::Matrix3d)this->Hw_b.E;
	this->measTarg.Xfb = (Eigen::Vector3d)this->X;
	this->measTarg.EulZYX = this->Eul;

	// angle measurements
	this->measTarg.Q = this->Q;

	// contact data
	this->measTarg.FL = this->FLtot;
	this->measTarg.FR = this->FRtot;
	this->measTarg.zmplocL = this->zmplocL;
	this->measTarg.zmplocR = this->zmplocR;
}

Eigen::Vector3d FloatingBaseEst::getFootDims(){
	Eigen::Vector3d v;
	v(0) = abs(this->HLfootFix.at(0).r(0))*2;
	v(1) = abs(this->HLfootFix.at(0).r(1))*2;
	v(2) = 0;
}

void FloatingBaseEst::initQuaternion(Eigen::Quaterniond & quat)
{
	quat.w() = 1.0;
	quat.x() = 0.0;
	quat.y() = 0.0;
	quat.z() = 0.0;
}

void FloatingBaseEst::printInternalStates(){
	// add quantities that should be printed in the debug mode:

	cout<<"Jacobian Jw_com:"<<endl<<this->Jw_com<<endl<<endl;
	cout<<"Jacobian Jw_eeL"<<endl<<this->Jw_eeL<<endl<<endl;
	cout<<"Jacobian Jw_eeR"<<endl<<this->Jw_eeR<<endl<<endl;
	cout<<"Jacobian Jw_b"<<endl<<this->Jw_b<<endl<<endl;
}