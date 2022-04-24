#include "PatternGenerator.hpp"

using namespace std;
using namespace ros;
using namespace Eigen;

PatternGenerator::PatternGenerator(ros::NodeHandle & nh, bool debug)
: nh(nh),debug(debug)
{
	this->fsm_stage = pg_state::IDLE;
	this->readParameters();
	this->Ly = 0.15;
	this->reset();
}

void PatternGenerator::readParameters(){
	this->err = false;
	// read all the required parameters from the param cloud
	if(this->err |= !this->nh.getParam("/pgen/config/mode",this->mode)){
		ROS_ERROR("PGEN: .");
		return;
	}

	if(this->err |= !this->nh.getParam("/pgen/config/dir",this->dir)){
		ROS_ERROR("PGEN.");
		return;
	}

	if(this->err |= !this->nh.getParam("/pgen/times/Tfin",this->Tfin)){
		ROS_ERROR("PGEN: .");
		return;
	}

	if(this->err |= !this->nh.getParam("/pgen/times/Tss",this->Tss)){
		ROS_ERROR("PGEN: .");
		return;
	}

	if(this->err |= !this->nh.getParam("/pgen/times/Tds", this->Tds)){
		ROS_ERROR("PGEN: .");
		return;
	}

	if(this->err |= !this->nh.getParam("/pgen/times/Tini", this->Tini)){
		ROS_ERROR("PGEN: .");
		return;
	}

	if(this->err |= !this->nh.getParam("/pgen/step/def/x", this->Lx)){
		ROS_ERROR("PGEN: .");
		return;
	}

	if(this->err |= !this->nh.getParam("/pgen/step/def/z", this->Lz)){
		ROS_ERROR("PGEN: .");
		return;
	}

	if(this->err |= !this->nh.getParam("/pgen/step/perc/py", this->proc_y)){
		ROS_ERROR("PGEN: .");
		return;
	}
}

void PatternGenerator::init(Eigen::Vector3d pLm, Eigen::Vector3d pRm, int Nsteps){
	this->Ly = (abs<double>(pLm(1)) + abs<double>(pRm(1))) / 2;
	this->startTriggered = true;
	this->reset();
	this->Nsteps = Nsteps;
	this->updateOutputStruct();
}


// TODO 
void PatternGenerator::step(Time time){
	this->fsmStep(time);
	// TODO - create a pgen output class
	this->updateOutputStruct();
}

void PatternGenerator::stop(){
	this->startTriggered = false;
}

void PatternGenerator::reset()
{
	// algorithm has been reset (so it has not yet ended)
	this->hasEnded = false;
	// the stage is the starting one
	this->fsm_start_stage = true;
	// zmp
	this->zmp.setZero();
	// positions
	this->pL.setZero();
	this->pL(1) = +this->Ly;
	this->pR.setZero();
	this->pR(1) = -this->Ly;
	// velocities
	this->pLp.setZero();
	this->pRp.setZero();
	// accelerations
	this->pLpp.setZero();
	this->pRpp.setZero();

	// reset number of steps
	this->istep = 0;
	this->isStartingStep = true;

	// initialise the phase data
	this->pd.phase = 0;
	this->pd.wshiftL = 0.5;
	this->pd.wshiftR = 0.5;

	// initialise the output structure
	this->updateOutputStruct();
}

// FINITE STATE MACHINE FUNCTIONS
void PatternGenerator::fsmStep(Time time){

	if(!this->startTriggered){
		this->fsm_stage = IDLE;
	}

	switch(this->fsm_stage){
		case IDLE:
			this->fsm_idle(time);
			break;
		case INI:
			this->fsm_ini(time, this->Tini);
			break;
		case DS_INI:
			this->fsm_dsIni(time, this->Tds/2);
			break;
		case DS_L:
			this->fsm_dsL(time, this->Tds);
			break;
		case DS_R:
			this->fsm_dsR(time, this->Tds);
			break;
		case DS_FIN:
			this->fsm_dsFin(time, this->Tds/2);
			break;
		case SS_L:
			this->fsm_ssL(time, this->Tss);
			break;
		case SS_R:
			this->fsm_ssR(time, this->Tss);
			break;
		case ERR:
			this->fsm_err(time);
			break;
		case FIN: default:
			this->fsm_fin(time, this->Tfin);
			break;
	}
}

void PatternGenerator::fsm_idle(Time time){
	if(this->fsm_start_stage){
		this->fsm_start_stage = false;
		this->timeref = time;
		if(debug){
			ROS_INFO("PG: IDLE");
		}
	}

	if(this->startTriggered){
		this->fsm_start_stage = true;
		this->fsm_stage = INI;
	}
}

// DONE
// is currently not much different than fsm_idle (but offers time for the balancer to get the robot to the desired configuration)
void PatternGenerator::fsm_ini(Time time, double Tphase){
	if(this->fsm_start_stage){
		this->fsm_start_stage = false;
		this->timeref = time;
		if(debug){
			ROS_INFO("PG: INI");
		}
	}

	// wait - this is when the ??
	if(this->getTimeDiff(time) >= Tphase){
		this->fsm_start_stage = true;
		if(this->mode!=0){
			// move to the double support phase
			this->fsm_stage = DS_INI;
			ROS_INFO("PG: STARTING WALKING.");
		}else{
			// balancer only mode. Stay in INI phase
			this->fsm_stage = INI;
		}
		
	}
}

// DONE
void PatternGenerator::fsm_dsIni(Time time, double Tphase){
	if(this->dir==0){
		// left step first
		this->fsm_dsL(time, Tphase);
	}else{
		// right step first
		this->fsm_dsR(time, Tphase);
	}
}

void PatternGenerator::fsm_dsL(Time time, double Tphase){
	if(this->fsm_start_stage){
		this->fsm_start_stage = false;
		this->timeref = time;

		if(debug){
			ROS_INFO("PG: DS left");
		}

		// setup starting and ending points
		this->setZmpInterval(this->zmp, this->pL, this->proc_y);
		this->setLLegInterval(this->pL, this->pL, this->pd.wshiftL, 1.0);
		this->setLLegIntermPts(false);
		this->setRLegInterval(this->pR, this->pR, this->pd.wshiftR, 0.0);
		this->setRLegIntermPts(false);
	}

	// INTERPOLATION BETWEEN START AND END
	this->generatePatterns(0, this->normaliseTime(this->getTimeDiff(time), Tphase));

	// TIME CRITERION FOR TRANSITION
	if(this->getTimeDiff(time) >= Tphase){
		this->fsm_start_stage = true;
		this->fsm_stage = SS_L;
	}
}

void PatternGenerator::fsm_dsR(Time time, double Tphase)
{
	if(this->fsm_start_stage){
		this->fsm_start_stage = false;
		this->timeref = time;

		if(debug){
			ROS_INFO("PG: DS right");
		}

		// setup starting and ending points
		this->setZmpInterval(this->zmp, this->pR, this->proc_y);
		this->setLLegInterval(this->pL, this->pL, this->pd.wshiftL, 0.0);
		this->setLLegIntermPts(false);
		this->setRLegInterval(this->pR, this->pR, this->pd.wshiftR, 1.0);
		this->setRLegIntermPts(false);
	}

	// INTERPOLATION BETWEEN START AND END
	this->generatePatterns(0, this->normaliseTime(this->getTimeDiff(time), Tphase));

	// TIME CRITERION FOR TRANSITION
	if(this->getTimeDiff(time) >= Tphase){
		this->fsm_start_stage = true;
		this->fsm_stage = SS_R;
	}
}

void PatternGenerator::fsm_dsFin(Time time, double Tphase)
{
	if(this->fsm_start_stage){
		this->fsm_start_stage = false;
		this->timeref = time;

		if(debug){
			ROS_INFO("PG: DS fin");
		}


		Vector3d v = this->zmp;
		v(1) = 0;
		// setup starting and ending points
		this->setZmpInterval(this->zmp, v, 1);
		this->setLLegInterval(this->pL, this->pL, this->pd.wshiftL, 0.5);
		this->setLLegIntermPts(false);
		this->setRLegInterval(this->pR, this->pR, this->pd.wshiftR, 0.5);
		this->setRLegIntermPts(false);
	}

	// INTERPOLATION BETWEEN START AND END
	this->generatePatterns(0, this->normaliseTime(this->getTimeDiff(time), Tphase));

	// TIME CRITERION FOR TRANSITION
	if(this->getTimeDiff(time) >= Tphase){
		this->fsm_start_stage = true;
		this->fsm_stage = FIN;
	}
}

void PatternGenerator::fsm_ssL(Time time, double Tphase)
{
	if(this->fsm_start_stage){
		this->fsm_start_stage = false;
		this->timeref = time;

		// count the next step
		this->istep++;
		if(this->istep == this->Nsteps){
			this->isEndingStep = true;
		}

		if(debug){
			ROS_INFO("PG: SS left. This is %d-th step", this->istep);
		}

		// setup starting and ending points
		this->setZmpInterval(this->zmp, this->zmp, 1);
		// left is support leg
		this->setLLegInterval(this->pL, this->pL, this->pd.wshiftL, this->pd.wshiftL);
		this->setLLegIntermPts(false);
		
		
		// right is swinging leg
		Vector3d pNew = this->getNextStepPos(this->pR);
		this->setRLegInterval(this->pR, pNew, this->pd.wshiftR, this->pd.wshiftR);
		this->setRLegIntermPts(true);
	}

	// INTERPOLATION BETWEEN START AND END
	if(mode == 1){
		// do not lift the foot
		this->generatePatterns(0, this->normaliseTime(this->getTimeDiff(time), Tphase));
	}else{
		// lift the foot
		this->generatePatterns(1, this->normaliseTime(this->getTimeDiff(time), Tphase));
	}

	// TIME CRITERION FOR TRANSITION
	if(this->getTimeDiff(time) >= Tphase){
		this->fsm_start_stage = true;
		if(this->istep < this->Nsteps){
			this->fsm_stage = DS_R; // double support that brings the robot back to the right side
		}else{
			this->fsm_stage = DS_FIN; // double support that brings the robot to the middle
		}
	}
}

void PatternGenerator::fsm_ssR(Time time, double Tphase)
{
	if(this->fsm_start_stage){
		this->fsm_start_stage = false;
		this->timeref = time;


		// count the next step
		this->istep++;
		if(this->istep == this->Nsteps){
			this->isEndingStep = true;
		}

		if(debug){
			ROS_INFO("PG: SS right. This is %d-th step", this->istep);
		}

		// setup starting and ending points
		this->setZmpInterval(this->zmp, this->zmp, 1);
		// right is support leg
		this->setRLegInterval(this->pR, this->pR, this->pd.wshiftR, this->pd.wshiftR);
		this->setRLegIntermPts(false);

		// left is swinging leg
		Vector3d pNew = this->getNextStepPos(this->pL);
		this->setLLegInterval(this->pL, pNew, this->pd.wshiftL, this->pd.wshiftL);
		this->setLLegIntermPts(true);
		
	}

	// INTERPOLATION BETWEEN START AND END
	if(mode == 1){
		// do not lift the foot
		this->generatePatterns(0, this->normaliseTime(this->getTimeDiff(time), Tphase));
	}else{
		// lift the foot
		this->generatePatterns(1, this->normaliseTime(this->getTimeDiff(time), Tphase));
	}

	// TIME CRITERION FOR TRANSITION
	if(this->getTimeDiff(time) >= Tphase){
		this->fsm_start_stage = true;
		if(this->istep < this->Nsteps){
			this->fsm_stage = DS_L; // double support that brings the robot back to the left side
		}else{
			this->fsm_stage = DS_FIN; // double support that brings the robot to the middle
		}
	}
}

void PatternGenerator::fsm_fin(Time time, double Tphase)
{
	if(this->fsm_start_stage){
		this->fsm_start_stage = false;
		this->timeref = time;
		
		if(debug){
			ROS_INFO("PG: FIN");
		}
	}

	if(this->getTimeDiff(time) >= Tphase){
		if(!this->hasEnded){
			ROS_INFO("PG has reached the final state!");
		}
		this->hasEnded = true;
	}
}


void PatternGenerator::fsm_err(Time time)
{
	if(this->fsm_start_stage){
		this->fsm_start_stage = false;
		this->timeref = time;
		
		if(debug){
			ROS_INFO("PG: ERROR");
		}
	}
}

// HELPER ROUTINES
double PatternGenerator::normaliseTime(double dT, double Tphase){
	return dT/Tphase;
}

double PatternGenerator::getTimeDiff(Time time)
{
	return time.toSec() - this->timeref.toSec();
}

void PatternGenerator::setZmpInterval(const Vector3d & start, const Vector3d & end, double proc)
{
	this->zmp_start = start;
	this->zmp_end = end;
	this->zmp_end(1) *= proc; // zmp_z does not necessairly go to the middle of the foot
}

void PatternGenerator::setLLegInterval(const Vector3d & start, const Vector3d & end, double wstart, double wend)
{
	this->pL_start = start;
	this->pL_end = end;
	this->wshiftL_start = wstart;
	this->wshiftL_end = wend;
}

void PatternGenerator::setRLegInterval(const Vector3d & start, const Vector3d & end, double wstart, double wend)
{
	this->pR_start = start;
	this->pR_end = end;
	this->wshiftR_start = wstart;
	this->wshiftR_end = wend;
}

void PatternGenerator::setLLegIntermPts(bool isSwing){
	if(isSwing){
		this->swingLegIntermPts(this->ptsL,this->pL_start,this->pL_end);
	}else{
		this->stanceLegIntermPts(this->ptsL,this->pL_start,this->pL_end);
	}
}

void PatternGenerator::setRLegIntermPts(bool isSwing){
	this->ptsR.clear();
	if(isSwing){
		this->swingLegIntermPts(this->ptsR,this->pR_start,this->pR_end);
	}else{
		this->stanceLegIntermPts(this->ptsR,this->pR_start,this->pR_end);
	}
}

void PatternGenerator::swingLegIntermPts(vector<Vector3d> & vect, const Vector3d & start, const Vector3d & end)
{
	Vector3d pt_s, pt_si, pt_ei, pt_e;
	pt_s = start;
	pt_e = end;
	pt_si = pt_s;
	pt_ei = pt_e;
	switch(this->mode){
		case 1: default:
			break;
		case 2: case 3:
			pt_si(2) = this->Lz;
			pt_ei(2) = this->Lz;
			break;
	}
	vect.clear();
	vect.push_back(pt_s);
	vect.push_back(pt_s);
	vect.push_back(pt_s);
	vect.push_back(pt_si);
	vect.push_back(pt_ei);
	vect.push_back(pt_e);
	vect.push_back(pt_e);
	vect.push_back(pt_e);
}

void PatternGenerator::stanceLegIntermPts(vector<Vector3d> & vect, const Vector3d & start, const Vector3d & end)
{
	Vector3d pt_s, pt_e;
	pt_s = start;
	pt_e = end;
	vect.clear();
	vect.push_back(pt_s);
	vect.push_back(pt_s);
	vect.push_back(pt_e);
	vect.push_back(pt_e);
}

Vector3d PatternGenerator::getNextStepPos(const Vector3d & pos_now){
	Vector3d pNew = pos_now;
	if(this->mode==3){
		if(this->isStartingStep || this->isEndingStep){
			pNew(0) += Lx/2;
		}else{
			pNew(0) += Lx;
		}

		if(this->isStartingStep){
			this->isStartingStep = false;
		}else{
			this->isEndingStep = false;
		}
	}
	return pNew;
}


// generate patterns and trajectories
void PatternGenerator::generatePatterns(int phase, double time_norm)
{
	this->zmp = this->linInterp<Vector3d>(this->zmp_start, this->zmp_end, time_norm);
	this->pd.phase = phase;
	this->pd.wshiftL = this->linInterp<double>(this->wshiftL_start, this->wshiftL_end, time_norm);
	this->pd.wshiftR = this->linInterp<double>(this->wshiftR_start, this->wshiftR_end, time_norm);
	this->pL = this->bezierCurve(this->ptsL, time_norm);
	this->pR = this->bezierCurve(this->ptsR, time_norm);
	this->pLp = this->bezierFirstDerivative(this->ptsL, time_norm);
	this->pRp = this->bezierFirstDerivative(this->ptsR, time_norm);
	this->pLpp = this->bezierSecondDerivative(this->ptsL, time_norm);
	this->pRpp = this->bezierSecondDerivative(this->ptsR, time_norm);
}

// binomial coefficients
double PatternGenerator::binomCoeff(int n, int k){
	return 1.0/((n+1) * std::beta(n-k+1,k+1));
}

// time_norm has to be between 0 and 1

// bezier coefficients for splining
double PatternGenerator::bezierCoeff(int n, int k, double time_norm){
	return this->binomCoeff(n, k)*pow(time_norm, k)*pow(1-time_norm, n-k);
}

// computation of foot positions
Vector3d PatternGenerator::bezierCurve(const vector<Vector3d> & pts, double time_norm){
	Vector3d res = Vector3d::Zero();
	int n = pts.size();
	for(int k = 0; k < n; k++){
		res += this->bezierCoeff(n,k,time_norm) * pts.at(k);
	}
	res += this->bezierCoeff(n,n,time_norm) * pts.at(n-1); //last point is taken 2 times
	return res;
}

// computation of foot velocities
Vector3d PatternGenerator::bezierFirstDerivative(const vector<Vector3d> & pts, double time_norm){
	Vector3d res = Vector3d::Zero();
	int n = pts.size()-1;
	for(int k = 0; k < n; k++){
		res += this->bezierCoeff(n,k,time_norm) * (pts.at(k+1)-pts.at(k));
	}
	res += this->bezierCoeff(n,n,time_norm) * (pts.at(n)-pts.at(n-1)); //last difference is taken 2 times
	return res;
}

// computation of foot accelerations
Vector3d PatternGenerator::bezierSecondDerivative(const vector<Vector3d> & pts, double time_norm){
	Vector3d res = Vector3d::Zero();
	int n = pts.size()-2;
	for(int k = 0; k < n; k++){
		res += this->bezierCoeff(n,k,time_norm) * (pts.at(k+2) - 2*pts.at(k+1) + pts.at(k));
	}
	res += this->bezierCoeff(n,n,time_norm) * (pts.at(n) - 2*pts.at(n-1) + pts.at(n-2)); //last difference is taken 2 times
	return res;
}

// computation of zmp positions and weight shifting coefficients
template<typename Derived>
Derived PatternGenerator::linInterp(const Derived & start, const Derived & end, double time_norm)
{
	return start * (1-time_norm) + end * time_norm;
}

void PatternGenerator::updateOutputStruct(){
	this->pg_out.zmp = this->zmp;
	this->pg_out.pL = this->pL;
	this->pg_out.pR = this->pR;
	this->pg_out.pLp = this->pLp;
	this->pg_out.pRp = this->pRp;
	this->pd = this->pd;
}

