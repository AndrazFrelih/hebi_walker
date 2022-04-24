#include "TrajectoryGenerator.hpp"

using namespace ros;
using namespace std;
using namespace Eigen;

#define USE_PRECOMPUTED_KMAT 1

// TODO
TrajectoryGenerator::TrajectoryGenerator(NodeHandle & nh, bool debug)
:nh(nh), debug(debug)
{
	// set to uninitialized
	this->isInitialized = false;
	// first read parameters from rosparam cloud
	this->readParameters();
	if(!this->err){
		this->setupLIPM2Dsystem();
		// use parameters and predefined macros to compute the feedback matrix
		if(!USE_PRECOMPUTED_KMAT){
			this->computeFeedbackMatrix();
		}
		this->plotKrefCoefficients();
	}
}

// DONE
void TrajectoryGenerator::readParameters()
{
	this->err = false;
	if(this->err |= !this->nh.getParam("/ctrl/full_controller/publish_rate", this->fs)){
		ROS_ERROR("TGEN: Publish rate parameter was not found.");
	}else{
		this->Ts = 1/this->fs;
	}

	this->Npreview = NPREV;

	if(this->err |= !this->nh.getParam("/tgen/zdes", this->zdes)){
		ROS_ERROR("TGEN: The desired COM height parameter not found.");
	}else{
		this->omega = sqrt(GRAV/this->zdes);
	}

	if(USE_PRECOMPUTED_KMAT){
		if(this->err |= !this->nh.getParam("/tgen/lqr/Kint", this->Kint)){
			ROS_ERROR("TGEN: Publish rate parameter was not found.");
		}

		vector<double> Kcom_vct;
		if(this->err |= !this->nh.getParam("/tgen/lqr/Kcom", Kcom_vct)){
			ROS_ERROR("TGEN: Publish rate parameter was not found.");
		}else{
			for(int i = 0; i < Kcom_vct.size(); i++){
				this->Kcom(i) = Kcom_vct.at(i);
			}
		}

		vector<double> Kref_vct;
		if(this->err |= !this->nh.getParam("/tgen/lqr/Kref", Kref_vct)){
			ROS_ERROR("TGEN: Publish rate parameter was not found.");
		}else{
			for(int i = 0; i < Kref_vct.size(); i++){
				this->Kref(i) = Kref_vct.at(i);
			}
		}
	}
}

void TrajectoryGenerator::setupLIPM2Dsystem(){
    // system matirces
    double dt = this->Ts;
    this->Asys <<   1.0, dt, pow(dt,2)/2,
    				0.0, 1.0, dt,
    				0.0, 0.0, 1.0;
    this->Bsys <<   pow(dt,3)/6,
    				pow(dt,2)/2,
    				dt;
    this->Csys <<   1.0, 0.0, -(this->zdes / GRAV);

    if(this->debug){
    	cout<<"Asys: "<<endl<<this->Asys<<endl;
    	cout<<"Bsys: "<<endl<<this->Bsys<<endl;
    	cout<<"Csys: "<<endl<<this->Csys<<endl;
    }
}


// DONE - is too slow and causes segfault for Npreview > 100
void TrajectoryGenerator::computeFeedbackMatrix()
{
	const size_t int_dim = 1;
	const size_t com_model_dim = 3;
	const size_t ref_dim = NPREV;
	const size_t state_dim = com_model_dim + int_dim + ref_dim;
	const size_t control_dim = 1;

    // matrices of references
    Matrix<double,Eigen::Dynamic,Eigen::Dynamic> Aref;
    Aref = Matrix<double,ref_dim,ref_dim>::Zero();
    Aref.block<ref_dim-1,ref_dim-1>(0,1) = Matrix<double,ref_dim-1,ref_dim-1>::Identity();

    Matrix<double,ref_dim,1> Bref;
    Bref.setZero();
    Bref(ref_dim-1) = 1.0;
    // augmented matrices
    // state transition
    Matrix<double,state_dim,state_dim> Amat;
	Amat = Matrix<double,state_dim,state_dim>::Zero();
	Amat(0,0) = 1.0;
	Amat.block<1,com_model_dim>(0, int_dim) = this->Csys * this->Asys;
	Amat(0,int_dim + com_model_dim) = -1;
	Amat.block<com_model_dim,com_model_dim>(int_dim,int_dim) = this->Asys;
	Amat.block<ref_dim,ref_dim>(int_dim + com_model_dim, int_dim +com_model_dim) = Aref;
	// system input matrix
    Matrix<double,state_dim,control_dim> Bmat;
    Bmat = Matrix<double,state_dim,control_dim>::Zero();
    Bmat(0) = this->Csys.dot(this->Bsys);
    Bmat.block<com_model_dim,control_dim>(int_dim,0) = this->Bsys;

    // state weights
    Matrix<double,state_dim,state_dim> Qmat;
    Qmat = Matrix<double,state_dim,state_dim>::Zero();
    Qmat(0,0) = 1.0;
    
    // input weights
    Eigen::Matrix<double,control_dim,control_dim> Rmat;
    Rmat(0) = 1e-6;
   
    // LQR gain
    ct::core::FeedbackMatrix<state_dim, control_dim> Kmat;
    // ricatti equation solver
    ct::optcon::LQR<state_dim, control_dim> lqrSolver;
 
    // last argument has to be true. Otherwise the method returns a runtime error (LAPACK is not found on runtime...)
    lqrSolver.compute(Qmat, Rmat, Amat, Bmat, Kmat, true, true);
    // cast the matrix to the eigen one
    this->K = (Eigen::MatrixXd)Kmat;

    // compute parts of matrices
    this->Kint = this->K(0);
    this->Kcom = this->K.segment<com_model_dim>(int_dim);
    this->Kref = this->K.segment<ref_dim>(int_dim + com_model_dim);

}

void TrajectoryGenerator::plotKrefCoefficients()
{
	if(this->debug){
    	ROS_INFO("LQR feedback matrix computed!");
    	vector<double> ilist;
    	vector<double> Kref_coeff;
    	for(int i = 0; i < this->Npreview; i++){
    		ilist.push_back((double)i);
    		Kref_coeff.push_back(this->Kref(i));
    	}
    	plt::figure_size(1200,720);
    	plt::named_plot("Kref(i)",ilist,Kref_coeff);
    	plt::title("Preview feedback reference coefficients:");
		plt::legend();
		plt::show();
    }
}

// DONE
void TrajectoryGenerator::init(pattern_gen_out start, double mass)
{
	ROS_INFO("TGEN: Initializing the trajectory generator node");
	this->setMass(mass);	
	this->def = start;
	this->initDeque(start);
	this->isInitialized = true;
	this->resetStates();
}

// DONE
void TrajectoryGenerator::initDeque(pattern_gen_out def)
{
	// delete all the previous elements
	this->container.clear();
	// assign the initial element N-times
	this->container.assign(this->Npreview, def);
}

// DONE
void TrajectoryGenerator::reset()
{
	ROS_INFO("TGEN: Resetting the class to the values provided when calling init() fcn.");
	this->resetDeque();
	this->resetStates();
}

// DONE
void TrajectoryGenerator::resetDeque()
{
	this->initDeque(this->def);
}

// DONE 
void TrajectoryGenerator::resetStates()
{
	this->com_x << 0,0,0;
	this->com_x_next = this->com_x;
	this->com_y << 0,0,0;
	this->com_y_next = this->com_y;
	this->zmp_x = 0;
	this->zmp_y = 0;
	this->stepLIPM2D(Vector3d::Zero());
	this->updateTargets();
}


// DONE
void TrajectoryGenerator::step(pattern_gen_out update)
{
	if(this->isInitialized){
		this->updateDeque(update);
		this->stepLIPM2D(this->computeCtrlSignal());
		this->updateTargets();
	}else{
		ROS_ERROR("TGEN: Not initialized yet. Cannot execute the step.");
	}	
}

// DONE
void TrajectoryGenerator::updateDeque(pattern_gen_out update)
{
	this->container.pop_front();
	this->container.push_back(update);
}

// DONE
Vector3d TrajectoryGenerator::computeCtrlSignal()
{
	Vector3d u;
	u.setZero();
	// subrtract the reference from the simulated value (since it comes from a discrete time model, there is no need for multiplication with dt)
	com_int += (this->zmp - this->container.front().zmp);

	// first part of the control signal (integrator of error between first zmp reference and zmp value from simulation)
	u += this->Kint/1 * com_int;
	// second part of the control signal (zmp references)
	for(int i=0; i<NPREV; i++){
		u += this->Kref(i)/1 * this->container.at(i).zmp;
	}
	// third part of the control signal (com feedback from the simulation)
	u(0) += this->Kcom.dot(this->com_x_next);
	u(1) += this->Kcom.dot(this->com_y_next);
	return -u;
}	

// DONE 
void TrajectoryGenerator::stepLIPM2D(Vector3d ctrlSignal)
{
	double ux = ctrlSignal(0);
	double uy = ctrlSignal(1);
	this->propagateModel(ux, this->com_x, this->com_x_next, this->zmp_x);
	this->propagateModel(uy, this->com_y, this->com_y_next, this->zmp_y);

	this->zmp << this->zmp_x, this->zmp_y, 0;

	this->com << this->com_x(0), this->com_y(0), this->zdes;
	this->comp << this->com_x(1), this->com_y(1), 0;
	this->compp << this->com_x(2), this->com_y(2), 0;

	this->dcm = this->com - this->comp / this->omega;
	this->dcmp = this->omega * (this->dcm - this->zmp);
}

// DONE 
void TrajectoryGenerator::propagateModel(double u, Vector3d & vold, Vector3d & vnew, double & out)
{
	Vector3d vtmp = vnew;
	vnew = vold;
	vold = this->Bsys * u + this->Asys * vtmp;
	out = this->Csys.dot(vnew);
}

// DONE 
void TrajectoryGenerator::updateTargets()
{
	// com targets
	this->out.com = this->com;
	this->out.comp = this->comp;
	this->out.dcm = this->dcm;
	this->out.dcmp = this->dcmp;
	this->out.zmp = this->zmp;

	// feet targets
	auto currData = this->container.front();
	this->out.pL = currData.pL;
	this->out.pR = currData.pR;
	this->out.pLp = currData.pLp;
	this->out.pRp = currData.pRp;
	this->out.FL = currData.pd.wshiftL * Fg;
	this->out.FR = currData.pd.wshiftR * Fg;

	// orientations
	this->out.RL = Matrix3d::Identity();
	this->out.RR = Matrix3d::Identity();
	this->out.Rfb = Matrix3d::Identity();
}

// DONE 
void TrajectoryGenerator::setMass(double mass)
{
	this->mass = mass;
	this->Fg = GRAV * mass;
}