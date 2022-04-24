
#include <chrono>
#include <iostream>
#include <thread>
#include <cmath>

#include "lookup.hpp"
#include "group_feedback.hpp"
#include "group_command.hpp"
#include "group_info.hpp"
#include "gains.hpp"
#include "util/plot_functions.h"

#include <Eigen/Core>

#define DEBUG 0


namespace plt = matplotlibcpp;

using namespace hebi;

// needed helper functions
void changeFeedbackFreq(std::shared_ptr< Group > group, double freq);
void trajectoryTimestep(Eigen::Matrix<double, 2, 1> *qi, Eigen::Matrix<double, 2, 1> *qpi, double time, double Tfin);
void generateTrajectory(std::vector<double> tvec, std::vector<Eigen::Matrix<double, 2, 1>> qvec, std::vector<Eigen::Matrix<double, 2, 1>> qpvec, double Tfin, double freq);

int main() {
	// This object is used to get the reference to all the actuators
	Lookup lookup;

	// Wait 2 seconds for the module list to populate, and then print out its contents
	std::this_thread::sleep_for(std::chrono::milliseconds(2000));

	// Entries of the lookup
	std::cout << "Get all the entries: " << std::endl;
	auto entry_list = lookup.getEntryList();
	for (auto entry : *entry_list)
		std::cout
			<< "Name: " << entry.name_ << std::endl
			<< "Family: " << entry.family_ << std::endl << std::endl;

	// Groups of actuators
	std::string family_name("Test_HEBI");
	std::cout << "Get all the desired group formed from family: " << family_name << std::endl;
	auto group = lookup.getGroupFromFamily(family_name);
	int num_modules = group->size();
	std::cout << "Size of the group is: " << num_modules << std::endl;	

	// Create a setting_cmd object - slower update
	GroupCommand setting_cmd(num_modules);
	// Set the control strategy and gains for all the modules
	Command::ControlStrategy new_strategy = Command::ControlStrategy::Strategy4;
	for (int i = 0; i < num_modules; i++){
 		setting_cmd[i].settings().actuator().controlStrategy().set(new_strategy);
 		setting_cmd[i].settings().actuator().positionGains().kP().set(10);
 		setting_cmd[i].settings().actuator().velocityGains().kP().set(0.05);
 		setting_cmd[i].settings().actuator().effortGains().kP().set(0.25);
	}
	// Send the setting_cmd with acknowledgement
	long timeout_ms = 10;
	if (group -> sendCommandWithAcknowledgement(setting_cmd, timeout_ms)){
		std::cout << "Got acknowledgement. Control strategy successfully changed." << std::endl;
	}else{
		std::cout << "Did not receive acknowledgement!" << std::endl;
	}

	// Create a control command object - faster update
	GroupCommand control_cmd(num_modules);

	// Feedback frequency - first lower freq. as we are going to be reading and displaying some data
	double Tfin = 8;
	double T_m = 0;
	double fc = 50;
	double dTc = 1/fc;
	long Tc_us = (long)(dTc * 1e6);
	double fm = fc;
	double dTm = 1/fm;
	changeFeedbackFreq(group, fm);
	GroupFeedback group_fbk(num_modules);
	

	// Trajectory
	std::vector<double> tvec;
	std::vector<Eigen::Matrix<double, 2, 1>> qvec_c;
	std::vector<Eigen::Matrix<double, 2, 1>> qpvec_c;
	generateTrajectory(tvec, qvec_c, qpvec_c, Tfin, fc);

	// Measurements
	double Tstp = 0, Tstp_old = 0;
	std::vector<Eigen::Matrix<double, 2, 1>> qvec_m;
	std::vector<Eigen::Matrix<double, 2, 1>> qpvec_m;
	std::vector<double> gyroz2_m;
	std::vector<Eigen::Matrix<double, 2, 1>> accelxy2_m;

	// Time management
	auto start = std::chrono::high_resolution_clock::now();
	auto begin = start;
	auto end = start;
	auto dTorg = end - begin;
	long dT = 0;

	// Placeholder
	Eigen::Matrix<double, 2, 1> tmp;

	Eigen::Matrix<double, 2, 1> qi;
	Eigen::Matrix<double, 2, 1> qpi;

	while(T_m<=Tfin){
		// Feedback
		if (group -> getNextFeedback(group_fbk))
	    {
			// Obtain feedback from the groupFeedback object
			auto gyro = group_fbk.getGyro();
			auto accel = group_fbk.getAccelerometer();
			auto pos = group_fbk.getPosition();
			auto vel = group_fbk.getVelocity();
			gyroz2_m.push_back(gyro(0,2));
			tmp << accel(0,1),accel(0,2);
			accelxy2_m.push_back(tmp);
			qvec_m.push_back(pos);
			qpvec_m.push_back(vel);

			Tstp = group_fbk.getTime();
			if(T_m==0){
				Tstp_old = Tstp;
			}

			if(DEBUG){
				std::cout << "TIME: " << T_m << std::endl;
				std::cout << "TIMESTAMP: " << Tstp - Tstp_old<< std::endl;
				std::cout << "POS:\n " << pos << std::endl;
				std::cout << "VEL:\n " << vel << std::endl;
				std::cout << "GYRO:\n " << gyro << std::endl;
				std::cout << "ACCEL:\n " << accel << std::endl;
			}

			T_m += dTm;
			Tstp_old = Tstp;
	    }

		// Control
	    end = std::chrono::high_resolution_clock::now();
	    dTorg = end - begin;
	    dT = std::chrono::duration_cast<std::chrono::microseconds>(dTorg).count();
	    // Execute it if the debugging mode is deactivated. Otherwise printing may meddle with controls
	    
	    if(dT >= Tc_us){
    		dTorg = end - start;
    		dT = std::chrono::duration_cast<std::chrono::microseconds>(dTorg).count();
    		double Tfull = ((double)dT)/1e6;
	    	trajectoryTimestep(&qi, &qpi, Tfull, Tfin);
	    	control_cmd.setPosition(qi);
    		control_cmd.setVelocity(qpi);
	    	if(!DEBUG){
	    		group->sendCommand(control_cmd); 
	    	}else{
	    		std::cout << "Elapsed time (ctrl): " << Tfull << std::endl;
	    		std::cout << "Qctrl " << qi << std::endl;
	    	}


	    	begin = std::chrono::high_resolution_clock::now();
	    }

	}



}

void changeFeedbackFreq(std::shared_ptr< Group > group, double freq){
	if(!group->setFeedbackFrequencyHz(freq)){
		std::cout << "Feedback frequency out of bounds" << std::endl;
	}else{
		std::cout 
			<< "Feedback frequency set. New frequency is: " 
			<< group->getFeedbackFrequencyHz() 
			<< "Hz"
			<< std::endl;
	}
}


void generateTrajectory(std::vector<double> tvec, std::vector<Eigen::Matrix<double, 2, 1>> qvec, std::vector<Eigen::Matrix<double, 2, 1>> qpvec, double Tfin, double freq){
	double time = 0;
	double dt = 1.0f/freq;
	Eigen::Matrix<double, 2, 1> qi;
	Eigen::Matrix<double, 2, 1> qpi;
	for(int i = 0; i < Tfin*freq; i++){
		tvec.push_back(time);
		trajectoryTimestep(&qi, &qpi, time, Tfin);
		qvec.push_back(qi);
		qpvec.push_back(qpi);
	}
}

void trajectoryTimestep(Eigen::Matrix<double, 2, 1> *qi, Eigen::Matrix<double, 2, 1> *qpi, double time, double Tfin){
	double q1a = 30.0/180.0*M_PI;
	double q2a = 45.0/180.0*M_PI;
	double w1 = 2*M_PI/(Tfin/2.0);
	double w2 = 2*M_PI/(Tfin/4.0);
	(*qi)[0] = q1a*sin(w1*time);
	(*qi)[1] = q2a*sin(w2*time);
	(*qpi)[0] = -w1*q1a*cos(w1*time);
	(*qpi)[1] = -w2*q2a*cos(w2*time);
}


void pyplot(){
	// Now we plot the collected feedback
	/*plt::clf();
	plt::ylim(-3.14, 3.14);
	plt::xticks(x_ticks, x_labels);
	plt::title("IO Board Gyro Feedback");
	plt::xlabel("Axis");
	plt::ylabel("Angular Velocity (rad/s)");
	plt::bar(gyro_data);
	plt::pause(0.001);*/
}
