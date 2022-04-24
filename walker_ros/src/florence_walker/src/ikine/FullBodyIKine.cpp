#include "FullBodyIKine.hpp"

using namespace ros;
using namespace std;
using namespace Eigen;
//using namespace qpOASES; - conflicts with Eigen (both use Matrix class)


FullBodyIKine::FullBodyIKine(NodeHandle & nh, bool debug)
: nh(nh), debug(debug), qp(NVRBL, NCSTR)
{
	// initialize the solver first time
	this->useFb = true; //default
	this->zmpSetsInit = false;
	this->solveDual = false; 
	this->initClass = false;
	this->initSolver  = false;
	// set to zero (just in case)
	this->X_ini.setZero();
	this->Q_sol.resize(NDOF,0);
	this->Qp_sol.resize(NDOF,0);
	this->X_sol.setZero();
	this->Xp_sol.setZero();
	this->Yp_sol.setZero();
	// force difference control is only activated if zmp is close to the border
	this->fdcL_active = false;
	this->fdcR_active = false;
	// set integrators to zero
	this->resetIntegrators();
	// read parameters from the ros param cloud
	this->readParameters();

	// setup the QP solver
	this->qp_solver = &qp;
	this->qp_options.printLevel = qpOASES::PrintLevel::PL_NONE;
	this->qp_solver->setOptions(this->qp_options);
}

FullBodyIKine::~FullBodyIKine(){}

// DONE
void FullBodyIKine::readParameters()
{
	

	this->err = false;
	int par;
	vector<double> params;
	// read all the required parameters from the param cloud
	if(this->err |= !this->nh.getParam("/ikine/wshift_offs", this->wshift_offs))
	{
		ROS_ERROR("IKINE: Weight shift offset parameter not found.");
		return;
	}

	if(this->err |= !this->nh.getParam("/ikine/act_int", par))
	{
		ROS_ERROR("IKINE: Activate integrators parameter not found.");
		return;
	}else{
		this->actInt = par!=0?true:false;
	}

	if(this->err |= !this->nh.getParam("/ikine/fdc/act", par))
	{
		ROS_ERROR("IKINE: .");
		return;
	}else{
		this->actFdc = par!=0?true:false;
	}

	if(this->err |= !this->nh.getParam("/ikine/fdc/sets", this->sets))
	{
		ROS_ERROR("IKINE: .");
		return;
	}

	params.clear();
	if(this->err |= !this->nh.getParam("/ikine/gains/Kq", params))
	{
		ROS_ERROR("IKINE: .");
		return;
	}else{
		// extend the vector as params are given for one leg only
		params.insert(params.end(), params.begin(), params.end());
		this->createFbMatrix(this->Kq, params);
	}

	params.clear();
	if(this->err |= !this->nh.getParam("/ikine/gains/Kwfb", params))
	{
		ROS_ERROR("IKINE: .");
		return;
	}else{
		this->createFbMatrix(this->Kwfb, params);
		cout<<"Kwfb"<<endl<<this->Kwfb<<endl;
	}

	params.clear();
	if(this->err |= !this->nh.getParam("/ikine/gains/Kvleg/prop", params))
	{
		ROS_ERROR("IKINE: .");
		return;
	}else{
		this->createFbMatrix(this->Kvleg_p, params);
		this->createFbMatrix(this->KvlegL_p, params);
		this->createFbMatrix(this->KvlegR_p, params);
		cout<<"Kvleg_p"<<endl<<this->Kvleg_p<<endl;
	}

	params.clear();
	if(this->err |= !this->nh.getParam("/ikine/gains/Kvleg/int", params))
	{
		ROS_ERROR("IKINE: .");
		return;
	}else{
		this->createFbMatrix(this->Kvleg_i, params);
		this->createFbMatrix(this->KvlegL_i, params);
		this->createFbMatrix(this->KvlegR_i, params);
		cout<<"Kvleg_i"<<endl<<this->Kvleg_i<<endl;
	}

	params.clear();
	if(this->err |= !this->nh.getParam("/ikine/gains/Kwleg", params))
	{
		ROS_ERROR("IKINE: .");
		return;
	}else{
		this->createFbMatrix(this->Kwleg, params);
		this->createFbMatrix(this->KwlegL, params);
		this->createFbMatrix(this->KwlegR, params);
		cout<<"Kvleg_p"<<endl<<this->Kwleg<<endl;
	}

	if(this->err |= !this->nh.getParam("/ikine/gains/foot_diff/Kdfz", this->Kdfz))
	{
		ROS_ERROR("IKINE: .");
		return;
	}

	if(this->err |= !this->nh.getParam("/ikine/gains/foot_diff/Tvdc", this->Tvdc))
	{
		ROS_ERROR("IKINE: .");
		return;
	}

	params.clear();
	if(this->err |= !this->nh.getParam("/ikine/gains/Kcom/prop", params))
	{
		ROS_ERROR("IKINE: .");
		return;
	}else{
		this->createFbMatrix(this->Kcom_p, params);
	}

	params.clear();
	if(this->err |= !this->nh.getParam("/ikine/gains/Kcom/int", params))
	{
		ROS_ERROR("IKINE: .");
		return;
	}else{
		this->createFbMatrix(this->Kcom_i, params);
	}

	params.clear();
	if(this->err |= !this->nh.getParam("/ikine/gains/Kfdc", params))
	{
		ROS_ERROR("IKINE: .");
		return;
	}else{
		this->createFbMatrix(this->Kfdc, params);
	}

	params.clear();
	if(this->err |= !this->nh.getParam("/ikine/lims", params))
	{
		ROS_ERROR("IKINE: .");
		return;
	}else{
		double lim = 1e6; //just some big value for limits of the freely floating base
		this->lims.resize(6,lim);
		params.insert(params.end(), params.begin(), params.end());
		this->lims.insert(this->lims.end(), params.begin(), params.end());
		for(int i=0; i < this->lims.size(); i++){
			this->limsvec(i) = this->lims.at(i);
		}
	}

	// set the initial values for the floating base (position and orientation)
	if(this->err |= !this->nh.getParam("/ikine/init/pos", this->pos_ini))
	{
		ROS_ERROR("IKINE: .");
		return;
	}else{
		this->X_ini(0) = this->pos_ini.at(0);
		this->X_ini(1) = this->pos_ini.at(1);
		this->X_ini(2) = this->pos_ini.at(2);
	}

	if(this->err |= !this->nh.getParam("/ikine/init/ori", this->ori_ini))
	{
		ROS_ERROR("IKINE: .");
		return;
	}else{
		this->X_ini(3) = this->ori_ini.at(0);
		this->X_ini(4) = this->ori_ini.at(1);
		this->X_ini(5) = this->ori_ini.at(2);
	}
}

void FullBodyIKine::setCmdJoints()
{
	for(int i=0; i < NDOF; i++){
		this->Q_sol.at(i) = this->X_sol(i+NDOF_FB);
		this->Qp_sol.at(i) = this->Xp_sol(i+NDOF_FB);
	}
}

// DONE
void FullBodyIKine::reset()
{
	if(this->initClass){
		this->resetIntegrators();
		this->resetSolver();
	}else{
		ROS_WARN("Cannot reset the solver before it has been initialized!");
	}
}

// DONE
void FullBodyIKine::resetIntegrators()
{
	this->com_int.setZero();
	this->lleg_int.setZero();
	this->rleg_int.setZero();
}

// DONE
void FullBodyIKine::resetSolver()
{
	// reset velocities to zero
	this->Xp_sol.setZero();
	this->Qp_sol.resize(NDOF,0);
	// set positions to initial values (robot has to be brought to home position in that case)
	this->X_sol = this->X_ini;
	this->setCmdJoints();
	// reset the solver (solver )
	if(!this->qp_solver->reset()){
		ROS_ERROR("Could not reset the qpOASES structure.");
	}
}

// DONE
template<typename Derived>
void FullBodyIKine::createFbMatrix(MatrixBase<Derived> & mat, vector<double> param)
{
	mat.setZero();
	for(int i=0; i<param.size(); i++){
		mat(i,i) = param.at(i);
	}
}



// DONE - NOT USED YET
bool FullBodyIKine::checkLimits()
{
	for(int i=0; i<this->lims.size(); i++){
		if(abs(this->Xp_sol(i))>=this->lims.at(i)){
			return false;
		}
	}
	return true;
}

// DONE
void FullBodyIKine::updateQParrays()
{
	// copy all the eigen matrices to the variable types used by qpOASES' solvers
	this->copyMatrixToArray(this->H_eig, this->H);
	this->copyMatrixToArray(this->g_eig, this->g);
	this->copyMatrixToArray(this->A_eig, this->A);
	this->copyMatrixToArray(this->lbA_eig, this->lbA);
	this->copyMatrixToArray(this->ubA_eig, this->ubA);
	this->copyMatrixToArray(this->lb_eig, this->lb);
	this->copyMatrixToArray(this->ub_eig, this->ub);
}

// DONE
template<typename Derived>
void FullBodyIKine::copyMatrixToArray(const MatrixBase<Derived> & mat, qpOASES::real_t * arr)
{
	int n = 0;
	for(int i = 0; i < mat.rows(); i++){
		for(int j = 0; j < mat.cols(); j++){
			arr[n] = mat(i,j);
			n++;
		}
	}
}

// TODO
void FullBodyIKine::setupQPmatrices(const Matrix<double,NDOF,1> & qold, 
									const Targets & meas, 
									const Targets & cmd, 
									const vector<WorldJacobian> & jlist, 
									phase_data pd, 
									Duration dt)
{
	// 
	double Ts = dt.toSec();

	// phase data and weight shifting
	int phase = pd.phase;
	double wShL = pd.wshiftL, wShR = pd.wshiftL;
	double wShLada, wShRada, wSh;
	wShLada = wShL + this->wshift_offs;
	wShRada = wShR + this->wshift_offs;
	wSh = wShLada + wShRada;
	wShLada /= wSh;
	wShRada /= wSh;
	bool swingL = (phase==1) && (wShLada < wShRada);
	bool swingR = (phase==1) && (wShLada > wShRada);

	// depending on phase, weight shift and measured contact data adapt gains
	this->adaptGains(pd,meas.FL,meas.FR);

	// should feedback be included
	double fb_fact = this->useFb?1.0:0.0;
	// (1)-desired joint angle postion
    auto Qp_d = -this->Kq * (meas.Q - qold) * fb_fact;

    // (2)-orientation of the floating base
    auto wfb_d = -this->Kwfb * this->skew2vec(this->getSkewMat(meas.Rfb * cmd.Rfb.transpose())) * fb_fact;
    // (3)-position and orientation of the foot
    VectorXd vL_d(NDOF_FB);
	VectorXd vR_d(NDOF_FB);
    // ----integrators
    auto errL = meas.pL - cmd.pL;
    auto errR = meas.pR - cmd.pR;   
    this->lleg_int += Ts * errL * (this->actInt?1:0) ;
    this->rleg_int += Ts * errR * (this->actInt?1:0);
    // ----position feedback
    auto vL_fb = (-this->KvlegL_p * errL - this->KvlegL_i * this->lleg_int) * fb_fact;
    auto vR_fb = (-this->KvlegR_p * errR - this->KvlegR_i * this->rleg_int) * fb_fact;

    // ----orientation feedback
    auto wL_fb = (-this->KwlegL * this->skew2vec(this->getSkewMat(meas.RL * cmd.RL.transpose()))) * fb_fact;
    auto wR_fb = (-this->KwlegR * this->skew2vec(this->getSkewMat(meas.RR * cmd.RR.transpose()))) * fb_fact;
    // ----add velocities together to form a single target
	vL_d.segment<NDOF_FB/2>(0) = vL_fb + cmd.pLp;
	vR_d.segment<NDOF_FB/2>(0) = vR_fb + cmd.pRp;
	vL_d.segment<NDOF_FB/2>(NDOF_FB/2) = wL_fb;
	vR_d.segment<NDOF_FB/2>(NDOF_FB/2) = wR_fb;

	// ----foot difference control
	double v_dfz = this->Kdfz * ((cmd.FL - cmd.FR) - (meas.FL - meas.FR)) * fb_fact;
	//double v_dfz = this->Kdfz * ((cmd.FL - cmd.FR) - (meas.FL - meas.FR));
	double v_vdc = 1/this->Tvdc * ((cmd.pL(2) + cmd.pR(2)) - (meas.pL(2) + meas.pR(2))) * fb_fact; //this one is probably not necessary as it is enforced by --position feedback
	double vL_fd = -0.5 * v_dfz + 0.5 * v_vdc;
	double vR_fd = +0.5 * v_dfz + 0.5 * v_vdc;
	if(phase==0 && 0){
		// only use in double support phase
		vL_d(2) += vL_fd;
		vR_d(2) += vR_fd;
	}

	// ----foot damping (at saturation) => has to be activated and initialized
	if(this->actFdc && this->zmpSetsInit){
		Vector3d wfdcL;
		if(!swingL){
			this->footDampingCtrl(meas.zmplocL, wfdcL, this->fdcL_active);
			if(this->fdcL_active){
				// saturate left foot ZMP
				vL_d.segment(NDOF_FB/2,NDOF_FB-1) = wfdcL;
				//vL_d.segment(NDOF_FB/2,NDOF_FB-1) += wfdcL;
			}
		}
		Vector3d wfdcR;
		if(!swingR){
			this->footDampingCtrl(meas.zmplocR, wfdcR, this->fdcR_active);
			if(this->fdcL_active && !swingR){
				// saturate right foot ZMP
				vR_d.segment(NDOF_FB/2,NDOF_FB-1) = wfdcR;	
				//vR_d.segment(NDOF_FB/2,NDOF_FB-1) += wfdcR;	
			}
		}
	}

	// (4)-COM control
	// ----integrators
	auto err_com = meas.com - cmd.com;
    this->com_int =+ Ts * err_com * (this->actInt?1:0);
    auto vcom_d = cmd.comp  + (- this->Kcom_p * err_com - this->Kcom_i * com_int) * fb_fact;

    // get jacobians
    auto Jw_com = jlist.at(0);
    auto Jw_b = jlist.at(1);
    auto Jw_Lee = jlist.at(2);
    auto Jw_Ree = jlist.at(3);

    /*cout<<"this->Jw_com"<<endl<<Jw_com<<endl;
	cout<<"this->Jw_eeL"<<endl<<Jw_Lee<<endl;
	cout<<"this->Jw_eeR"<<endl<<Jw_Ree<<endl;
	cout<<"this->Jw_b"<<endl<<Jw_b<<endl;
	cout<<"Jw_b.block<NDOF_FB/2,NVRBL>(NDOF_FB/2,0)"<<endl<<Jw_b.block<NDOF_FB/2,NVRBL>(NDOF_FB/2,0)<<endl;*/
    // build matrices for qp
    // ----compute hessean matrix and gradient vector of the cost function
    auto R1x = Jw_b.block<NDOF_FB/2,NVRBL>(NDOF_FB/2,0);
    //cout<<"R1x"<<endl<<R1x<<endl;
    auto W1x = Matrix3d::Identity();
    auto s1x = wfb_d;
    auto P1x = R1x.transpose() * W1x * R1x;
    //cout<<"P1x"<<endl<<P1x<<endl;
    auto q1x = R1x.transpose() * W1x * s1x;

    Matrix<double,NDOF,NVRBL> R2x = Matrix<double,NDOF,NVRBL>::Zero(); //auto can not be used here as the static assertion fails
    R2x.block<NDOF,NDOF>(0,NDOF_FB) = Matrix<double,NDOF,NDOF>::Identity(); 
    auto W2x = Matrix<double,NDOF,NDOF>::Identity();
    auto s2x = Qp_d;
    auto P2x = R2x.transpose() * W2x * R2x;
    auto q2x = R2x.transpose() * W2x * s2x;

    this->H_eig = P1x + P2x;
    this->g_eig = q1x + q2x;

    // ----constraints
    auto A1eq = Jw_com.block<NDOF_FB/2, NVRBL>(0,0);
    auto b1eq = vcom_d;

    auto A2eq = Jw_Lee;
    auto b2eq = vL_d;

    auto A3eq = Jw_Ree;
    auto b3eq = vR_d;

    int ind = 0;
    this->A_eig.block<NDOF_FB/2,NVRBL>(ind,0) = A1eq;
    this->lbA_eig.block<NDOF_FB/2,1>(ind,0) = b1eq;
    ind += NDOF_FB/2;
    this->A_eig.block<NDOF_FB,NVRBL>(ind,0) = A2eq;
    this->lbA_eig.block<NDOF_FB,1>(ind,0) = b2eq;
    ind += NDOF_FB;
    this->A_eig.block<NDOF_FB,NVRBL>(ind,0) = A3eq;
    this->lbA_eig.block<NDOF_FB,1>(ind,0) = b3eq;
    this->ubA_eig = this->lbA_eig;

    /*cout<<"Hessean matrix:"<<endl<<this->H_eig<<endl<<endl;
    cout<<"Gradient vector:"<<endl<<this->g_eig.transpose()<<endl<<endl;
    cout<<"Constr matrix:"<<endl<<this->A_eig<<endl<<endl;
    cout<<"Eq. cstr. vector (LB):"<<endl<<this->lbA_eig.transpose()<<endl<<endl;
    cout<<"Eq. cstr. vector (UB):"<<endl<<this->ubA_eig.transpose()<<endl<<endl;*/

    // ----variable limits (no need of doing it every iteration) could be moved to the constructor
    this->lb_eig = -this->limsvec;
	this->ub_eig = this->limsvec;
}

void FullBodyIKine::adaptGains(phase_data pd, double FL_m, double FR_m){
	if(pd.phase == 0)
	{
		this->KvlegL_p = this->Kvleg_p;
		this->KvlegL_i = this->Kvleg_i;
		this->KwlegL = this->Kwleg;

		this->KvlegR_p = this->Kvleg_p;
		this->KvlegR_i = this->Kvleg_i;
		this->KwlegR = this->Kwleg;
	}
	else
	{
		if(pd.wshiftL > pd.wshiftR)
		{
			// left leg is fixed
			this->KvlegL_p = this->Kvleg_p;
			this->KvlegL_i = this->Kvleg_i;
			this->KwlegL = this->Kwleg;
			// right leg is swinging - DO STH
			
		}
		else
		{
			// right leg is fixed
			this->KvlegR_p = this->Kvleg_p;
			this->KvlegR_i = this->Kvleg_i;
			this->KwlegR = this->Kwleg;
			// left leg is swinging - DO STH
			
		}
	}
}

void FullBodyIKine::footDampingCtrl(const Vector3d & zmploc_m, Vector3d & wfdc, bool & ctrl_active)
{
	auto zmploc_d = this->saturateZmp(zmploc_m);
	auto zmp_abs = zmploc_d.cwiseAbs();

	bool e1 = this->isStrictlySmaller(zmp_abs, this->zmp_Z1);
	bool e2 = !e1 && this->isStrictlySmaller(zmp_abs, this->zmp_Z2);
	bool e3 = !e1 && !e2 && this->isStrictlySmaller(zmp_abs, this->zmp_Z3);
	if(!e1 && !e2 && !e3){
		ROS_ERROR("IKINE: ERROR: ZMP not inside of the foot!!");
	}
	wfdc = this->Kfdc * (zmploc_d - zmploc_m);

	if(e3){
		// zmp is in the outermost zone
		ctrl_active = true;
	}

	if(!ctrl_active || e1){
		// controller is not active or the zmp is in the safe zone
		ctrl_active = false;
	}
}

Vector3d FullBodyIKine::saturateZmp(const Vector3d & zmploc_m)
{
	Vector3d sat_sign = this->zmp_Z1;
	Vector3d zmp_abs = zmploc_m.cwiseAbs();
	Vector3d zmp_sat = zmploc_m;
	for(int i=0;i<zmploc_m.rows(); i++){
		// this->zmp_Z1 is always positive - sign has to be added if zmp is negative
		if(zmploc_m(i)<0){
			sat_sign(i) = -sat_sign(i);
		}
		// if zmp is out of Z1 area, saturate it (bring it back to Z1 area)
		if(zmp_abs(i)>this->zmp_Z1(i)){
			zmp_sat(i) = sat_sign(i);
		}
	}
	return zmp_sat;
}

bool FullBodyIKine::isStrictlySmaller(const Vector3d & v1, const Vector3d & v2)
{
	for(int i=0; i<2; i++){
		if(!(v1(i)<v2(i))){
			return false;
		}
	}
	return true;
}

Matrix3d FullBodyIKine::getSkewMat(Matrix3d mat){
	return (mat - mat.transpose())/2;
}

Vector3d FullBodyIKine::skew2vec(Matrix3d mat){
	Vector3d v;
	v << -mat(1,2), +mat(0,2), -mat(0,1);
	return v;
}

// DONE
bool FullBodyIKine::solveQPproblem()
{
	int nWSR = 200;
	int result;
	// instead of QProblem use the SQProblem. It takes new H and A matrices every round (QProblem just at initialization stage)
	if(!this->initSolver){
		result = this->qp_solver->init( this->H, this->g, this->A, this->lb, this->ub, this->lbA, this->ubA, nWSR);
	}else{
		result = this->qp_solver->hotstart( this->H, this->g, this->A, this->lb, this->ub, this->lbA, this->ubA, nWSR);
	}
	
	if(result == qpOASES::SUCCESSFUL_RETURN){
		// get the primal problem solution and copy it to the Eigen vector
		this->qp_solver->getPrimalSolution(this->xOptSol);
		this->copyArrayToVector(this->xOptSol, this->Xp_sol);
		// get the dual problem solution (not really necessary)
		if(this->solveDual){
			this->qp_solver->getDualSolution(this->yOptSol);
			this->copyArrayToVector(this->yOptSol, this->Yp_sol);
		}
		return true;	
	}
	else
	{
		ROS_ERROR("Error solving the QP problem: %d\n", result);
		return false;
	}	
}

// DONE
template<typename Derived>
void FullBodyIKine::copyArrayToVector(const qpOASES::real_t * arr, MatrixBase<Derived> & vec)
{
	for(int j = 0; j < NVRBL; j++){
		vec(j) = arr[j];
	}
}

// DONE
bool FullBodyIKine::update( const Targets & meas, 
							const Targets & cmd, 
							const vector<WorldJacobian> & jlist, 
							phase_data pd, 
							Duration dt)
{
	if(this->initClass){
		this->setupQPmatrices(this->X_sol.block<NDOF,1>(NDOF_FB,0), meas, cmd, jlist, pd, dt);
		this->updateQParrays();
		if(this->solveQPproblem()){
			// integrate the solution of the optimization
			this->integrate(dt);
			// copy joint values into vectors that are used int the parent class
			this->setCmdJoints();
			return true;
		}else{
			ROS_ERROR("Error encountered while solving the problem!");
			return false;
		}
	}else{
		ROS_ERROR("The solver has not yet been initialized (call init first)!");
		return false;
	}

}

// DONE
void FullBodyIKine::useFeedback(bool fb)
{

	this->useFb = fb;
}

// DONE
void FullBodyIKine::setFootDims(Vector3d footDims)
{
	// required for foot damping control
	Vector3d footCorns = footDims/2;
	this->zmp_Z1 = this->sets.at(0)*footCorns;
	this->zmp_Z2 = this->sets.at(1)*footCorns;
	this->zmp_Z3 = this->sets.at(2)*footCorns;
	this->zmpSetsInit = true;
}

void FullBodyIKine::setMass(double mass){
	this->mass = mass;
	this->Fg = mass*GRAV;
}

// Q_ini is necessary so that the switching between controllers is smooth
void FullBodyIKine::init(const vector<double> & Q_ini, Vector3d footDims, double mass, bool useFb){
	for(int i=0; i<NDOF; i++){
		this->X_ini(NDOF_FB+i) = Q_ini.at(i);
	}
	this->X_sol = this->X_ini;
	this->useFeedback(useFb);
	this->setFootDims(footDims);
	this->setMass(mass);
	this->initClass = true;
}

void FullBodyIKine::integrate(Duration dt){
	this->X_sol = this->X_sol + dt.toSec() * this->Xp_sol;
}