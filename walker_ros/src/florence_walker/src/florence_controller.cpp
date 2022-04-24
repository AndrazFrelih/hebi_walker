// Basic libraries
#include <iostream> // standard library
#include <chrono> 	// high resolution clock
#include <string>
#include <thread> 	// multi threaded programming
#include <memory> 	// shared pointer

// ROS libraries
#include <ros/ros.h>

// ROS control libraries
#include <controller_manager/controller_manager.h>
//#include <controller_manager_msgs/SwitchController.h>

// user defined libs
#ifndef ROB_INT 
#define ROB_INT
#include "RobotInterface.hpp"
#endif

using namespace std;

int main(int argc, char** argv)
{
	ros::init(argc, argv,"florence_ctrl");
	ros::NodeHandle nh;
	ros::NodeHandle ctrl_nh("~");

	// create an instance of the registered hardware interface
	RobotInterface rob(nh);
	// create an instance of the controller manager
	controller_manager::ControllerManager cm(&rob, nh);

	// control loop rate (read it from the cloud)
	double fs;
	nh.getParam("/ctrl/full_controller/publish_rate",fs);
	ros::Rate loop_rate(fs);
	// asynchronous (non blocking) multi threaded spinning
	ros::AsyncSpinner spinner(10); // 10 threads is a little bit overexaggerated
	spinner.start();

	// time monitoring
	auto start = std::chrono::high_resolution_clock::now();
	auto end = start;
	auto dTorg = end - start;
	long dT = 0;

	vector<double> Texec;

	// soft-RT loop (should be at least - hopefully)
	while (ros::ok()){
		start = std::chrono::high_resolution_clock::now();
		
		//////////////////////////////////////////////////////////////
		// read data from the robot
		rob.read();
		// controller update
		cm.update(rob.get_time(), rob.get_period());
		// robot write
		rob.write(rob.get_period());
		// sleep until the next call
		//////////////////////////////////////////////////////////////
		
		loop_rate.sleep();
		end = std::chrono::high_resolution_clock::now();
    	dTorg = end - start;
    	dT = std::chrono::duration_cast<std::chrono::nanoseconds>(dTorg).count();
    	
    	//execution times in ms
    	Texec.push_back(((double)dT)/1e6);
	}

    return 0;    
} 

