#include <iostream>
#include <string>

#include <ros/ros.h>

#include "PatternGenerator.hpp"

#include "definitions.hpp"

#include "matplotlibcpp.h"
namespace plt = matplotlibcpp;



using namespace std;
using namespace ros;
using namespace Eigen;

int main(int argc, char** argv)
{
	// initialize the node
	init(argc, argv,"PG_test");
	NodeHandle nh;

	// control loop rate
	double fs;
	nh.getParam("/ctrl/full_controller/publish_rate",fs);
	Rate loop_rate(fs);

	// dummy initial feet positions
	Vector3d pLm, pRm;
	pLm.setZero();
	pLm(1) = 0.15;
	pRm.setZero();
	pRm(1) = -0.15;

	// create a pg instance
	PatternGenerator pg(nh, true);
	// set number of steps
	int NumberOfSteps = 3;
	// initialize it with leg positions from FB-estimator
	pg.init(pLm, pRm, NumberOfSteps);

	// data to be monitored
	Vector3d zmp, pL, pLp, pLpp;
	vector<double> zmp_y;
	vector<double> pL_z, pLp_z, pLpp_z;
	phase_data pd;
	vector<double> wShiftL;

	// time control
	Time rost0,rost;
	double ti;
	vector<double> t;

	// start the loop
	rost0 = Time::now();
	while(ok()){
		// sort time out
		rost = Time::now();
		ti = rost.toSec() - rost0.toSec();
		t.push_back(ti);
		// compute the step of the pg-algorithm
		pg.step(rost);

		// get return data from pg 
		zmp = pg.getZmp();
		pL = pg.getLLegPos();
		pLp = pg.getLLegVel();
		pLpp = pg.getLLegAcc();
		pd = pg.getPhaseData();

		// store data for plotting
		zmp_y.push_back(zmp(1));
		pL_z.push_back(pL(2));
		pLp_z.push_back(pLp(2));
		pLpp_z.push_back(pLpp(2));
		wShiftL.push_back(pd.wshiftL);

		// check if the algorithm has already generated the entire pattern
		if(pg.algorithmHasEnded()){
			break;
		}
		// sleep until the next iteration
		loop_rate.sleep();
		spinOnce();
	}


	// plotting patterns
	int plotNr =1;
	plt::figure_size(1200,720);
	switch(plotNr){
		case 1:
			plt::named_plot("zmp.y(t)",t,zmp_y);
			plt::title("Generated ZMP-Y.");
			break;
		case 2:
			plt::named_plot("pL.z(t)",t,pL_z);
			plt::named_plot("pLp.z(t)",t,pLp_z);
			//plt::named_plot("pLpp.z(t)",t,pLpp_z); - has to be corrected
			plt::title("Generated left foot trajectories.");
			break;
		case 3: default:
			plt::named_plot("wShiftL(t)",t,wShiftL);
			plt::title("Generated weight shifting coefficient.");
			break;
	}
	plt::legend();
	plt::show();

    return 0;    
} 