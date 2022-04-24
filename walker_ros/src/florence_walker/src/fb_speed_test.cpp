// Basic libraries
#include <chrono>
#include <iostream>
#include <thread>
#include <string>
#include <cmath>

// ROS libraries
#include <ros/ros.h>

// HEBI libraries
#include"lookup.hpp"
#include "group_feedback.hpp"
#include "group_command.hpp"
#include "group_info.hpp"
#include "gains.hpp"

// RBDL libraries
#include <rbdl/rbdl.h>
#include <rbdl/rbdl_utils.h>
#ifndef RBDL_BUILD_ADDON_URDFREADER
#error "Error: RBDL addon URDFReader not enabled."
#endif
#include <urdfreader.h>

// Control-toolbox library (does not have to be dynamically linked in the CMake file - this header-only library is installed to the folder with global includes)
#include <ct/optcon/optcon.h> 

// qpOASES library
#include <qpOASES.hpp>

//used namespaces
using namespace std;
using namespace hebi;


int main(int argc, char** argv)
{
	ros::init(argc, argv,"fb_speed_test");
	ros::NodeHandle nh_;
	ros::Rate loop_rate(100);

	// HEBI test
	// Create a Lookup Object
	Lookup lookup;
	vector<string> motor_names;
	if(!nh_.getParam("/hw/motor/names",motor_names)){
		ROS_ERROR("Joint names were not found. Make sure that parameters were correctly loaded to the parameter cloud!");
		return -1;
	}else{
		for (int i=0;i<motor_names.size();i++)
    		std::cout << motor_names[i] << ' ';
	}
	auto group = lookup.getGroupFromNames({"Florence"},motor_names);
	//auto group = lookup.getGroupFromFamily("Florence");
	group->setFeedbackFrequencyHz(1000);
	GroupFeedback group_fbk(group->size());

	auto start = std::chrono::high_resolution_clock::now();
	auto end = start;
	auto dTorg = end - start;
	long dT = 0;

	long avgExec = 0;
	long wcExec = 0;

	int cnt = 0;
	while (ros::ok()){
		ros::spinOnce();
        loop_rate.sleep();
        start = std::chrono::high_resolution_clock::now();
        if (group -> getNextFeedback(group_fbk)){
        	end = std::chrono::high_resolution_clock::now();
        	dTorg = end - start;
        	dT = std::chrono::duration_cast<std::chrono::nanoseconds>(dTorg).count();
        	wcExec = wcExec>dT?wcExec:dT;
        	avgExec = (long)((double)avgExec + (double)1.0/(cnt+1.0)*(dT-avgExec));
        	cout << "The reading of feedback has taken: " << dT << "ns to complete. AVG exec: " << avgExec << "ns, WC exec: " << wcExec << "ns." << endl;
        	cnt++;
        	auto pos = group_fbk.getPosition();
        	cout << "Read position values:" << pos.transpose() << endl;
        }
	}
    return 0;    
} 
