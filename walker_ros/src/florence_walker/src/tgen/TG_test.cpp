#include <iostream>
#include <string>

#include <ros/ros.h>


#include "PatternGenerator.hpp"
#include "TrajectoryGenerator.hpp"
#include "definitions.hpp"
#include "Targets.hpp"

#include "matplotlibcpp.h"
namespace plt = matplotlibcpp;


using namespace std;
using namespace ros;
using namespace Eigen;

int main(int argc, char** argv)
{
	// initialize the node
	init(argc, argv,"TG_test");
	NodeHandle nh;
	ROS_INFO("STARTED THE TG TEST");

	// control loop rate
	double fs;
	nh.getParam("/ctrl/full_controller/publish_rate",fs);
	Rate loop_rate(fs);

	// create a PG instance
	PatternGenerator PG(nh, true);
	// set number of steps
	int NumberOfSteps = 3;
	// dummy initial feet positions
	Vector3d pLm, pRm;
	pLm.setZero();
	pLm(1) = 0.15;
	pRm.setZero();
	pRm(1) = -0.15;
	// initialize the PG with leg positions from FB-estimator
	PG.init(pLm, pRm, NumberOfSteps);
	// create a TG instance
	TrajectoryGenerator TG(nh, true);
	// dummy system mass
	double sysMass = 31;
	// initialize the TG
	auto pgini = PG.getOutput();
	TG.init(pgini, sysMass);

	// time control
	Time ros_t0,ros_t, ros_tend;
	double ti;
	vector<double> t;
	vector<double> talg;
	// logging
	vector<double> com_y_vect;
	vector<double> zmp_y_vect;
	vector<double> zmpin_y_vect;

	Targets out;

	// time monitoring
	auto start = std::chrono::high_resolution_clock::now();
	auto end = start;
	auto dTorg = end - start;
	long dT = 0;

	bool init_end = true;

	// start the loop
	ros_t0 = Time::now();
	while(ok()){
		// sort time out
		ros_t = Time::now();
		ti = ros_t.toSec() - ros_t0.toSec();
		t.push_back(ti);

		/////////////////////////////////////////////////////////////////////////
		start = std::chrono::high_resolution_clock::now();

		// compute a step of the pg-algorithm
		PG.step(ros_t);
		// compute a step of the tg-algorithm
		TG.step(PG.getOutput());

		end = std::chrono::high_resolution_clock::now();
    	dTorg = end - start;
    	dT = std::chrono::duration_cast<std::chrono::nanoseconds>(dTorg).count();
    	talg.push_back(((double)dT)/1e9);
    	/////////////////////////////////////////////////////////////////////////

		auto tgout = TG.getTargets();
		double com_y = tgout.com(1);
		double zmp_y = tgout.zmp(1);
		com_y_vect.push_back(com_y);
		zmp_y_vect.push_back(zmp_y);
		auto v = TG.getPGzmp();
		zmpin_y_vect.push_back(v(1));

		// check if the algorithm has already generated the entire pattern
		if(PG.algorithmHasEnded()){
			if(init_end){
				init_end = false;
				ros_tend = ros::Time::now();
				cout<<"PG has ended, but we have to wait for data to get through TG."<<endl;
			}

			double tcrit = ros_t.toSec() - ros_tend.toSec();

			if(tcrit > 5){
				cout<<"Ending time: "<<ros_t.toSec()-ros_t0.toSec()<<"s"<<endl;
				break;
			}
		}
		// sleep until the next iteration
		loop_rate.sleep();
		spinOnce();
	}
	int fig = 2;
	plt::figure_size(1200,720);
	if(fig == 1){
		plt::named_plot("talg(t)",t,talg);
		plt::title("Algorithm execution time (PG+TG).");
	}else{
		plt::named_plot("zmpin.y(t)",t,zmpin_y_vect);
		plt::named_plot("zmp.y(t)",t,zmp_y_vect);
		plt::named_plot("com.y(t)",t,com_y_vect);
		plt::title("Y-direction.");
	}
	plt::legend();
	plt::show();

    return 0;    
} 