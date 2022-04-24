#include "FlorenceFullController.hpp"

using namespace std;

FlorenceFullController::FlorenceFullController()
: err(false), isInit(false), debug(FFC_DEBUG), FBE(this->nh, FFC_DEBUG), BL(this->nh, FFC_DEBUG), PG(this->nh, FFC_DEBUG), TG(this->nh, FFC_DEBUG), IK(this->nh, FFC_DEBUG)
{
	ROS_INFO("All LIPM classes in FlorenceFullController have been initialized.");
	this->zeros.resize(NDOF,0);
}

FlorenceFullController::~FlorenceFullController(){

}

void FlorenceFullController::initModules(const vector<double> &q_m, const vector<double> &q_c)
{
	// after this the order of initialization is important (FBE->PG->TG)
	double mass;
	this->FBE.init(q_c); //initialize with ctrl values so that the wcf is placed correctly
	mass = this->FBE.getModelMass();
	this->PG.init(this->FBE.getFPosL(), this->FBE.getFPosR(), 15);
	this->TG.init(this->PG.getOutput(), mass);
	this->BL.init();
	this->pd = TG.getPhaseData();
	if(OUT_ACTIVE){
		this->IK.init(q_c, this->FBE.getFootDims(), mass, true);
	}else{
		this->IK.init(q_c, this->FBE.getFootDims(), mass, false);
	}
	

	// testing purposes
	this->fake_fsen.resize(2, Vector4d::Constant(mass*GRAV/8));
}

void FlorenceFullController::algorithmStep(const ros::Duration& period){
	// measurement containers
	if(!this->err){
		std::vector<double> q_m, qp_m, qeff_m, q_c, qp_c, qeff_c;
		std::vector<Eigen::Matrix<double,4,1>> fsen;
		std::vector<Eigen::Matrix<double,3,1>> accel, gyro;

		// read measurements from registers
		q_m = this->getMeasPos();
		qp_m = this->getMeasVel();
		qeff_m = this->getMeasEff();
		fsen =this->getMeasForces();
		accel = this->getMeasAccel();
		gyro = this->getMeasGyro();

		// initialize commands (so that there are no problems if an optimization error occurs and IKine output is invalid)
		q_c = this->getCmdPos();
		qp_c = this->zeros;
		qeff_c = this->zeros;

		// initialize modules in the first step of the algorithm
		if(!this->isInit){
			// call the init function to set up the LIP model
			this->initModules(q_m, q_c);
			this->isInit = true;
			
		}else{
			// run the control algorithm :
			// do the PG and TG stuff (update cmd)
			this->PG.step(ros::Time::now());
			// compute a step of the tg-algorithm
			this->TG.step(this->PG.getOutput());
			this->pd = this->TG.getPhaseData();
			this->cmd = this->TG.getTargets();
			
			// state estimation
			if(!OUT_ACTIVE){
				q_m = this->getCmdPos();
				qp_m = this->getCmdVel();
				qeff_m = this->zeros;
				fsen = this->fake_fsen;
			}
			// run a step of FB estimation algorithm
			this->FBE.update(q_m, qp_m, qeff_m, fsen, accel, gyro, this->pd, period);
			// store targets	
			this->meas = this->FBE.getTargets();
			
			// balancer
			this->BL.update(this->meas, this->cmd, period);
			if(OUT_ACTIVE){
				this->cmd.comp = BL.getTargetComVelo();
			}

			// print out targets in debug mode
			if(this->debug){
				cout<<this->cmd<<endl;
				cout<<this->meas<<endl;
			}

			// execute optimization
			this->err |= !this->IK.update(this->meas, this->cmd, this->FBE.getConstraints(), this->pd, period);
			if(!this->err){
				// determine outputs
				q_c = this->IK.getJointPosSol();
				qp_c = this->IK.getJointVelSol();
				// TODO: module that compensates internal springs
				qeff_c = this->zeros;
			}else{
				cout << "There was an error with IKine module!" <<endl;
				//retry the execution (some problems are infeasible in one step and can be solved in the next one)
				this->err = false; 
			}
		}

		// this is required for the visualisation node
		auto Xfb_full = this->IK.getFbPosSol();
		this->cmd.Xfb = Xfb_full.block<NDOF_FB/2,1>(0,0);
		Eigen::EulerAnglesZYXd eul(Xfb_full(5),Xfb_full(4),Xfb_full(3));
		this->cmd.Rfb = eul.toRotationMatrix();

		// write commands into the registers
		for(int i=0; i<q_m.size(); i++){
			this->setCmdPos(i, q_c.at(i));
			this->setCmdVel(i, qp_c.at(i));
			this->setCmdEff(i, qeff_c.at(i));
		}

		// copy the states to the publisher class, make sure there are no run conditions by locking the memory before accesing it
		std::mutex *mtx = this->getMtxPtr();
		if(mtx->try_lock()){
			this->visualise(q_m,
							qp_m,
							qeff_m,
							q_c,
							qp_c,
							qeff_c,
			    			fsen,
							this->meas,
							this->cmd,
			    			ctrl_states::RUNNING);
			mtx->unlock();
		}
	}
}


void FlorenceFullController::startController(){
	this->start_time = ros::Time::now();
}

void FlorenceFullController::stopController(){
	//this->start_time = ros::Time::now();
}