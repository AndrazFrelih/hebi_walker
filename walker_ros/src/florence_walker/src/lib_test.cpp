// Basic libraries
#define __STDCPP_WANT_MATH_SPEC_FUNCS__ 1 // for beta function
#include <chrono>
#include <iostream>
#include <thread>
#include <string>
#include <cmath>

// ROS libraries
#include <ros/ros.h>
#include <ros/package.h>

// HEBI libraries
#include"lookup.hpp"

// RBDL libraries
#include <rbdl/rbdl.h>
#include <rbdl/rbdl_utils.h>
#ifndef RBDL_BUILD_ADDON_URDFREADER
#error "Error: RBDL addon URDFReader not enabled."
#endif
#include <urdfreader.h>

// Control-toolbox library (does not have to be dynamically linked in the CMake file - this header-only library is installed to the folder with global includes)
#include <ct/core/core.h> 
#include <ct/optcon/optcon.h> 

// qpOASES library
#include <qpOASES.hpp>

//used namespaces
using namespace std;
using namespace hebi;
using namespace RigidBodyDynamics;
using namespace RigidBodyDynamics::Math;
USING_NAMESPACE_QPOASES

void ETHcontrolToolboxTest(void);
void qpoasesTest(void);
Math::Matrix3d removeZeros(Math::Matrix3d Ri);

double binom(int n, int k) { 
	return 1/((n+1)*std::beta(n-k+1,k+1)); 
}

int main(int argc, char** argv)
{
	ros::init(argc, argv,"lib_test");
	ros::NodeHandle nh_;
	ros::Rate loop_rate(0.2);
	string path = ros::package::getPath("florence_description") + "/urdf/florenceRBDL.urdf";
	cout << path << endl;

	// HEBI test
	// Create a Lookup Object
	Lookup lookup;
	// Wait 2 seconds for the module list to populate, and then print out its contents
	this_thread::sleep_for(chrono::milliseconds(2000));
	cout << endl;
	auto entry_list = lookup.getEntryList();
	for (auto entry : *entry_list)
	cout
		<< "Name: " << entry.name_ << endl
		<< "Family: " << entry.family_ << endl << endl;

	// RBDL test
	rbdl_check_api_version(RBDL_API_VERSION);
	Model* model = new Model();
	//string filename = "florenceRBDL.urdf";
	if (!Addons::URDFReadFromFile(path.c_str(), model, false)) {
		cerr << "Error loading model " << argv[1] << endl;
		abort();
	}
	cout << "Degree of freedom overview:" << endl;
	cout << Utils::GetModelDOFOverview(*model);
	cout << "Model Hierarchy:" << endl;
	cout << Utils::GetModelHierarchy(*model);


	// test both math and eigen
	Math::VectorNd Q = Math::VectorNd::Constant(12,1,1);
	Q(11) = 0;
	Math::VectorNd const * ptrQ = &Q;

	//Q(3) = M_PI/2;
	//Q(4) = M_PI/3;
	//Q(5) = M_PI/4;
	cout << ptrQ->transpose() << endl;

	Eigen::MatrixXd Qp = Eigen::MatrixXd::Constant(12,1,0);
	Eigen::MatrixXd const * ptrQp = &Qp;


	// the cast has to be made in order to get the right value
	UpdateKinematicsCustom(*model,&Q,(Math::VectorNd const *)ptrQp,NULL);
	Math::Vector3d ti;
	Math::Matrix3d Ri;

	for(int i=0; i<model->dof_count+1; i++){
		Ri = CalcBodyWorldOrientation(*model,Q,i,false);
		ti = CalcBodyToBaseCoordinates (*model,Q, i, Math::Vector3d::Zero(3,1), false);
		Ri = removeZeros(Ri);
		cout << "\nBody " << model->GetBodyName(i) << endl;
		cout << "Rotmat of link " << i << " is: " << endl;
		cout << Ri.transpose() << endl;
		cout << "Transl of link " << i << " is: " << endl;
		cout << ti.transpose() << endl;
	}
	cout << endl;
	cout << "Id of Link1L is: " << (unsigned int) model->GetBodyId("Link1L") << endl;
	cout << "Id of EE_R is: " << ((unsigned int) model->GetBodyId("EE_R")) - model->fixed_body_discriminator << endl;	//this does not work
	cout << "If model->GetBodyName(model->GetBodyId('EE_R')) is executed, we get: " << model->GetBodyName(model->GetBodyId("EE_R")) << endl;

	vector<FixedBody> bodies = model->mFixedBodies;
	cout << "\nNow let's try to iterate through all fixed body indices: "<<endl;
	for (int i=0; i<bodies.size(); i++){
		unsigned int ind = i+model->fixed_body_discriminator;
		cout << "Index ["<<i<<"] corresponds to the ["<<ind<<"], which is: "<<model->GetBodyName(ind)<<endl;
	}

	vector<vector<unsigned int>> mu = model->mu;
	for(int j=0; j<mu.size(); j++){
		cout << "Body " << j << ":"<<endl;
		for(int k=0; k<mu.at(j).size(); k++){
			cout << mu.at(j).at(k) << ",";
		}
		cout << endl;
	}
	cout << endl;

	

	
	
	cout<<"Number of fixed bodies is: " << bodies.size() << endl;
	for(FixedBody fb : bodies){
		cout << "Rmat with parent ID " << fb.mMovableParent << ": \n" << removeZeros(fb.mParentTransform.E) << endl;
		cout << "tvec with parent ID " << fb.mMovableParent << ": \n" << fb.mParentTransform.r.transpose() << endl;
		cout << endl;
	}

	double mass;
	Math::Vector3d com;
	Q = Eigen::MatrixXd::Constant(12,1,1);
	Utils::CalcCenterOfMass(*model, Q, (Math::VectorNd) Qp, NULL, mass, com, NULL, NULL, NULL, NULL, true);
	cout << "Calculated COM for given angles: " << com.transpose() << "with the mass of" << mass << endl;
	cout << "The com of the main body is not considered here (as the floating base is fixed in the consideration [library problems]), so it has to be added manually"<<endl;

	Math::Vector3d comBase = model->mBodies.at(0).mCenterOfMass;
	double massBase = model->mBodies.at(0).mMass;

	double mass_compensated = (massBase + mass);
	Math::Vector3d com_compensated = (comBase*massBase + com*mass) / mass_compensated;
	cout << "Calculated full COM for given angles: " << com_compensated.transpose() << "with the mass of" << mass_compensated << endl;


	// JACOBIANS
	int desBodyId;
	Math::MatrixNd Jac;

	desBodyId = 6;
	Jac = Math::MatrixNd::Zero(6,12);
	CalcPointJacobian6D(*model, Q, desBodyId, model->mBodies.at(desBodyId).mCenterOfMass, Jac, false);
	cout << "Computed left ee-COM jacobian: " << endl;
	cout << Jac << endl << endl; 

	desBodyId = 12;
	Jac = Math::MatrixNd::Zero(6,12);
	CalcPointJacobian6D(*model, Q, desBodyId, model->mBodies.at(desBodyId).mCenterOfMass, Jac, false);
	cout << "Computed right ee-COM jacobian: " << endl;
	cout << Jac << endl << endl; 
	
	// Eigen test:
	Eigen::Matrix3f m;
	m = Eigen::AngleAxisf(M_PI/3, Eigen::Vector3f::UnitX())
	  * Eigen::AngleAxisf(M_PI/4, Eigen::Vector3f::UnitY())
	  * Eigen::AngleAxisf(M_PI/6, Eigen::Vector3f::UnitZ());


	cout << "original rotation:" << endl;
	cout << m << endl << endl;

	Eigen::Vector3f ea = m.eulerAngles(0, 1, 2); 
	Eigen::Vector3f ea2 = m.eulerAngles(2, 1, 0); 
	cout << "to Euler angles: (xyz-convetnion: first is roll_x, second is pitch_y, third is yaw_z)" << endl;
	cout << ea << endl << endl;
	cout << "to Euler angles: (zyx-convetnion: first is yaw_z, second is pitch_y, third is roll_x)" << endl;
	cout << ea2 << endl << endl;

	Eigen::Matrix3f n, o;
	n = Eigen::AngleAxisf(ea[0], Eigen::Vector3f::UnitX())
	  * Eigen::AngleAxisf(ea[1], Eigen::Vector3f::UnitY())
	  * Eigen::AngleAxisf(ea[2], Eigen::Vector3f::UnitZ()); 

	cout << "recalc original rotation (using xyz convention):" << endl;
	cout << n << endl;

  	o = Eigen::AngleAxisf(ea2[2], Eigen::Vector3f::UnitZ())
	  * Eigen::AngleAxisf(ea2[1], Eigen::Vector3f::UnitY())
	  * Eigen::AngleAxisf(ea2[0], Eigen::Vector3f::UnitX()); 

	cout << "recalc original rotation (using zyx convention):" << endl;
	cout << n << endl;


	// concept test
	// Eigen test:
	Eigen::Matrix3f Hb_eeL;
	cout<<"\n\npi/6 = "<<M_PI/6<<endl<<endl;
	m = Eigen::AngleAxisf(M_PI/6, Eigen::Vector3f::UnitZ())
	  * Eigen::AngleAxisf(M_PI/4, Eigen::Vector3f::UnitY())
	  * Eigen::AngleAxisf(M_PI, Eigen::Vector3f::UnitX());

	Eigen::Vector3f e_xyz = m.eulerAngles(0, 1, 2); 
	Eigen::Vector3f e_zyx = m.eulerAngles(2, 1, 0); 
	cout << "to Euler angles: (xyz-convetnion: first is roll_x, second is pitch_y, third is yaw_z)" << endl;
	cout << e_xyz << endl << endl;
	cout << "to Euler angles: (zyx-convetnion: first is yaw_z, second is pitch_y, third is roll_x)" << endl;
	cout << e_zyx << endl << endl;


	std::cout << "Pascal's triangle:\n";
    for(int n = 1; n < 10; ++n) {
        std::cout << std::string(20-n*2, ' ');
        for(int k = 1; k < n; ++k)
            std::cout << std::setw(3) << binom(n,k) << ' ';
        std::cout << '\n';
    }


    Eigen::Quaterniond qu1(Matrix3d::Identity());
    Eigen::Quaterniond qu2(0.9659,0,0,0.2588);
    Eigen::Quaterniond qu3, qu4;

    double alphaQ = 0.1;
    qu3 = qu1.slerp(alphaQ,qu2);
    qu4 = qu1.slerp(1-alphaQ,qu2);

    cout << "Quat 1 = ["<<qu1.w()<<","<<qu1.x()<<","<<qu1.y()<<","<<qu1.z()<<"]"<<endl;
    cout << "Quat 2 = ["<<qu2.w()<<","<<qu2.x()<<","<<qu2.y()<<","<<qu2.z()<<"]"<<endl;
    cout << "Quat 3 = ["<<qu3.w()<<","<<qu3.x()<<","<<qu3.y()<<","<<qu3.z()<<"] (interp between qu1 and qu2 with alpha = 0.1)"<<endl;
    cout << "Quat 4 = ["<<qu4.w()<<","<<qu4.x()<<","<<qu4.y()<<","<<qu4.z()<<"] (interp between qu1 and qu2 with alpha = 0.9)"<<endl;

    // Control toolbox test
	//cout << "Printing out a macro from the control toolbox library: " << ct::core::SecondOrderSystem::STATE_DIM << endl;
	//ETHcontrolToolboxTest();


	// qpOASES test
	qpoasesTest();

	cout<<"MAX MATRIX SIZE IN EIGEN IS : "<< EIGEN_STACK_ALLOCATION_LIMIT<<"(zero means that there is no limit)"<<endl;


	// ROS test
	int i = 0;

	while (ros::ok()){
		ros::spinOnce();
        loop_rate.sleep();
        cout << i << endl;
        i++;
	}
    return 0;    
} 


void qpoasesTest(void){
	// Setup data of first QP.
	real_t H[2*2] = { 1.0, 0.0, 0.0, 0.5 };
	real_t A[1*2] = { 1.0, 1.0 };
	real_t g[2] = { 1.5, 1.0 };
	real_t lb[2] = { 0.5, -2.0 };
	real_t ub[2] = { 5.0, 2.0 };
	real_t lbA[1] = { -1.0 };
	real_t ubA[1] = { 2.0 };

	// Setup data of second QP. 
	real_t g_new[2] = { 1.0, 1.5 };
	real_t lb_new[2] = { 0.0, -1.0 };
	real_t ub_new[2] = { 5.0, -0.5 };
	real_t lbA_new[1] = { -2.0 };
	real_t ubA_new[1] = { 1.0 };

	ROS_INFO("Setting up the qpProblem.");
	// Setting up QProblem object. 
	QProblem example( 2,1 );

	ROS_INFO("Initializing solver options.");
	Options options;
	example.setOptions( options );

	// Solve first QP. 
	int_t nWSR = 10;
	example.init( H,g,A,lb,ub,lbA,ubA, nWSR );

	// Get and print solution of first QP. 
	real_t xOpt[2];
	real_t yOpt[2+1];
	example.getPrimalSolution( xOpt );
	example.getDualSolution( yOpt );
	printf( "\nxOpt = [ %e, %e ];  yOpt = [ %e, %e, %e ];  objVal = %e\n\n", 
			xOpt[0],xOpt[1],yOpt[0],yOpt[1],yOpt[2],example.getObjVal() );
	
	// Solve second QP. 
	nWSR = 10;
	example.hotstart( g_new,lb_new,ub_new,lbA_new,ubA_new, nWSR );

	// Get and print solution of second QP. 
	example.getPrimalSolution( xOpt );
	example.getDualSolution( yOpt );
	printf( "\nxOpt = [ %e, %e ];  yOpt = [ %e, %e, %e ];  objVal = %e\n\n", 
			xOpt[0],xOpt[1],yOpt[0],yOpt[1],yOpt[2],example.getObjVal() );

	example.printOptions();
}

Math::Matrix3d removeZeros(Math::Matrix3d Ri){
	for(int j=0; j<3; j++)
		for(int k=0; k<3; k++)
			Ri(j,k) = abs(Ri(j,k))>1e-3?Ri(j,k):0;

	return Ri;
}

void ETHcontrolToolboxTest(void){
    const size_t state_dim = 2;
	const size_t control_dim = 1;
    Eigen::Matrix<double,state_dim,state_dim> Amat;
    Amat << 0,1,0,0;
    Eigen::Matrix<double,state_dim,control_dim> Bmat;
    Bmat << 0,1;
    Eigen::Matrix<double,state_dim,state_dim> Qmat;
    Qmat << 1,0,0,1;
    Eigen::MatrixXd Rmat(1,1);
    //Eigen::Matrix<double,control_dim,control_dim> Rmat;
    Rmat << 1;
    //Eigen::Matrix<double,state_dim,control_dim> Kmat;
    ct::core::FeedbackMatrix<state_dim, control_dim> Kmat;
    ct::optcon::LQR<state_dim, control_dim> lqrSolver;
    // last argument has to be true. Otherwise the method returns a runtime error (LAPACK is not found on runtime...)
    std::cout << "\n\nComputing an LQR controller: "<<endl;
    lqrSolver.compute(Qmat, Rmat, Amat, Bmat, Kmat,true,true);
    std::cout << "LQR gain matrix:" << std::endl << Kmat << std::endl;

    Eigen::MatrixXd Kmat_cast = (Eigen::MatrixXd)Kmat;
    std::cout << "Recast gain matrix:" << std::endl << Kmat_cast << std::endl;
}