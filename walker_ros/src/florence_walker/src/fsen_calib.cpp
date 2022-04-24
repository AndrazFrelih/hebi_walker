// standard libraries
#include <iostream>
#include <string>
#include <cmath>
#include <thread>
#include <fstream>
#include <regex>

// eigen libraries
#include <Eigen/Core>

// HEBI libraries
#include "lookup.hpp"
#include "group_feedback.hpp"
#include "group_command.hpp"
#include "group_info.hpp"
#include "gains.hpp"

// ROS libraries
#include <ros/ros.h>

#define NMEAS 10

using namespace std;
using namespace Eigen;
using namespace hebi;

int main(int argc, char** argv)
{
	ros::init(argc, argv,"force_sensor_calibration");
	ros::NodeHandle nh;
	double fs = 10;
	ros::Rate loop_rate(fs);

	typedef Matrix<double,4,1> Vector4d;
	vector<Vector4d> fmeasL;
	vector<Vector4d> fmeasR;

	string family_name;
	vector<string> fs_names;
	vector<int> fs_ind;

	cout << "\n-------------------------------\nCallibrating load cells:\n-------------------------------\n " << endl;

	if(!nh.getParam("/hw/fam_name",family_name)){
		ROS_ERROR("Family name was not found. Make sure that parameters were correctly loaded to the parameter cloud!");
	}

	if(!nh.getParam("/hw/fsen/names",fs_names)){
		ROS_ERROR("Force sensor names were not found. Make sure that parameters were correctly loaded to the parameter cloud!");
	}

	if(!nh.getParam("/hw/fsen/pins",fs_ind)){
		ROS_ERROR("The variable storing used analog pins not found. Make sure that parameters were correctly loaded to the parameter cloud!");
	}


	Lookup lookup;
	std::shared_ptr<hebi::Group> fs_group = lookup.getGroupFromNames({family_name},fs_names);
	hebi::GroupFeedback fs_group_fbk(fs_group->size());
	fs_group->setFeedbackFrequencyHz(fs);

	double cnt = NMEAS;
	while(cnt--){
		loop_rate.sleep();

		if (fs_group->getNextFeedback(fs_group_fbk,500)){
			//left leg:
			auto& analogLeft = fs_group_fbk[0].io();
			//right leg:
			auto& analogRight = fs_group_fbk[1].io();

			int ind = 0;
			Vector4d measL;
			Vector4d measR;

			for(int i=0; i<fs_ind.size(); i++){
				ind = fs_ind[i]+1;
				// values for the left sensor
				if(analogLeft.a().hasFloat(ind)) {
		      		measL[i] = (double) analogLeft.a().getFloat(ind);
			    } else {
		      		measL[i] = (double) analogLeft.a().getInt(ind);
			    }
			    

			    // values for the right sensor
			    if (analogRight.a().hasFloat(ind)) {
		      		measR[i] = (double) analogRight.a().getFloat(ind);
			    } else {
		      		measR[i] = (double) analogRight.a().getInt(ind);
			    }
			}
			fmeasL.push_back(measL);
		    //cout<<"Left measurement is: "<<measL.transpose()<<endl;
		    fmeasR.push_back(measR);
		    //cout<<"Right measurement is: "<<measR.transpose()<<endl;

		}else{
			ROS_ERROR("No force sensor feedback received!");
		}
		
	}

	Vector4d sumL = Vector4d::Constant(0);
	Vector4d sumR = Vector4d::Constant(0);
	for(int i=0; i<NMEAS; i++){
		sumL += fmeasL[i];
		sumR += fmeasR[i];
	}

	sumL = sumL/(double)fmeasL.size();
	sumR = sumR/(double)fmeasR.size();

	cout << "Left offsets are: "  << sumL.transpose() << endl;
	cout << "Right offsets are: " << sumR.transpose() << endl;

	vector<double> list_offs;
	int isize = 8;
	list_offs.resize(isize);
	for(int i=0; i<isize/2; i++){
		list_offs[i] = sumL[i];
		list_offs[isize/2+i] = sumR[i];
	}

	
	cout<<"Stored measured values"<<endl;
	string rospackage = string(argv[0]);
	string pack_path = rospackage.substr(0,rospackage.find_last_of("\\/"));
	pack_path = regex_replace(pack_path,regex("devel/lib"),"src");
	string path = pack_path.append("/launch/config/yaml/fs_offs.yaml");
	cout<< path << endl;

	ofstream myfile(path);
	if (myfile.is_open())
	{
		myfile << "hw:\n";
		myfile << "  fsen:\n";
		myfile << "    offs: [";
		for(int i=0; i<isize; i++){
			if(i<isize-1){
				myfile << list_offs[i] << ", ";
			}else{
				myfile << list_offs[i] << "]\n";
			}
		}
		myfile.close();
	}
	else{
		cout << "Unable to open file";	
	}

	cout << "\n-------------------------------\nCallibration finished!\n-------------------------------\n " << endl;

    return 0;    
} 
