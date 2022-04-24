#include "FlorenceJointController.hpp"

FlorenceJointController::FlorenceJointController()
{
	//ROS_INFO("Starting the joint controller.\nReading parameters from the cloud.");
	if(!this->nh.getParam("/hw/joint/qhome", this->q_fin)){
		ROS_ERROR("Desired joint values (defining home position) not found!");
	}

	if(!this->nh.getParam("/ctrl/joint_controller/times/Tini", this->initWait)){
		ROS_ERROR("Desired joint values (defining home position) not found!");
	}

	if(!this->nh.getParam("/ctrl/joint_controller/times/Ttra", this->transitionTime)){
		ROS_ERROR("Desired joint values (defining home position) not found!");
	}

	this->dim = this->q_fin.size();
	this->q_ini.resize(this->dim,0);
	this->zeros.resize(this->dim,0);

	this->q_c_traj.resize(this->dim,0);
	this->qp_c_traj.resize(this->dim,0);
	this->poly_coef.resize(this->dim);
}

void FlorenceJointController::startController()
{
	// entry state and reference time for the fsm
	this->fsm_state = state::START;
	this->refTime = ros::Time::now();
}

void FlorenceJointController::stopController()
{

}

void FlorenceJointController::algorithmStep(const ros::Duration& period)
{
	double time_diff = ros::Time::now().toSec()-this->refTime.toSec();

	// get required measurements
	std::vector<double> q_m = this->getMeasPos();
	std::vector<Vector4d> fsen = this->getMeasForces();
	std::vector<double> q_c = this->getCmdPos();
	std::vector<Eigen::Matrix<double,3,1>> accel = this->getMeasAccel();

	// enum describing the state of the robot
	ctrl_states cst;

	switch(this->fsm_state){
		// the controller is waiting for a certain time at the beginning
		case state::START:
		{
			cst = ctrl_states::IDLE;
			if (time_diff >= this->initWait){
				this->fsm_state = state::CAPT;
			}
			break;
		}

		// the current joint value is captured and trajectory parameters are computed
		case state::CAPT:
		{
			cst = ctrl_states::RUNNING;
			this->q_ini = q_c;
			this->compTrajParams();
			this->fsm_state = state::TRAJ;
			this->refTime = ros::Time::now();
			break;
		}

		// this phase brings the robot from the initial state to the home position
		case state::TRAJ:
		{
			cst = ctrl_states::RUNNING;
			if (time_diff <= this->transitionTime){
				// evaluate the trajectory generator function
				this->evalTraj(time_diff);
				// write commands into the registers
				for(int i=0; i<q_m.size(); i++){
					this->setCmdPos(i, this->q_c_traj.at(i));
					this->setCmdVel(i, this->qp_c_traj.at(i));
				}
			}else{
				this->fsm_state = state::END;
			}
			q_c = this->q_c_traj;

			break;
		}

		// robot stays in the home position and awaits until the manager switches to the next controller
		case state::END: default:
		{
			// terminal state
			cst = ctrl_states::STOPPED;
			q_c = this->q_c_traj;
		}
	}

	Targets meas, cmd;
	// copy the states to the publisher class
	std::mutex *mtx = this->getMtxPtr();
	if(mtx->try_lock()){
		this->visualise(q_m,
				this->zeros,
				this->zeros,
				q_c,
				this->zeros,
				this->zeros,
    			fsen,
    			meas,
				cmd,
    			(uint8_t)cst);
		mtx->unlock();
	}
	
}

void FlorenceJointController::evalTraj(double time)
{
	for(int i=0; i<this->dim; i++){
		this->q_c_traj[i] = this->evalTrajPos(time, this->poly_coef[i]);
		this->qp_c_traj[i] = this->evalTrajVel(time, this->poly_coef[i]);
	}
}

double FlorenceJointController::evalTrajPos(double time, polyCoefVec poly)
{
	double qval = 0;
	double mult = 1;
	for(int i=0; i<poly.rows(); i++){

		qval += mult * poly(i);
		mult = mult*time;
	}
	return qval;
}

double FlorenceJointController::evalTrajVel(double time, polyCoefVec poly)
{
	double qval = 0;
	double mult = 1;
	for(int i=1; i<poly.rows(); i++){
		qval += mult * i * poly(i);
		mult = mult*time;
	}
	return qval;
}

void FlorenceJointController::compTrajParams()
{
	for(int i=0; i<this->dim; i++){
		this->poly_coef[i] = this->genPolyCoef(this->transitionTime, this->q_ini[i], this->q_fin[i]);
	}
}

polyCoefVec FlorenceJointController::genPolyCoef(double Tfin, double xini, double xfin)
{
	Eigen::Matrix<double,6,1> out;
	out(0) = xini;
	out(1) = 0;
	out(2) = 0;
	out(3) = -(2*(5*xini - 5*xfin))/pow(Tfin,3);
	out(4) = (15*xini - 15*xfin)/pow(Tfin,4);
	out(5) = -(3*(2*xini - 2*xfin))/pow(Tfin,5);
	return out;
}