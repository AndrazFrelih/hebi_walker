// standard c++ libraries
#include <chrono>
#include <iostream>
#include <thread>
#include <cmath>
#include <fstream>
#include <regex>

// HEBI libraries
#include "lookup.hpp"
#include "group_feedback.hpp"

// Eigen libraries
#include <Eigen/Core>

// ROS libraries
#include <ros/ros.h>

using namespace ros;
using namespace std;
using namespace hebi;

int main(int argc, char** argv)
{
	init(argc, argv,"joint_offs_comp");
	NodeHandle nh;
	string family_name;
	vector<int> motor_coupl_ind1;
	vector<int> motor_coupl_ind2;
	vector<double> motor_coef;
	vector<string> motor_names;

	bool readSuccess = true;
	readSuccess &= nh.getParam("/hw/fam_name", family_name);
	readSuccess &= nh.getParam("/hw/motor/names", motor_names);
	readSuccess &= nh.getParam("/hw/motor/coupl/ind1", motor_coupl_ind1);
	readSuccess &= nh.getParam("/hw/motor/coupl/ind2", motor_coupl_ind2);
	readSuccess &= nh.getParam("/hw/motor/coef", motor_coef);

	if(!readSuccess){
		ROS_WARN("Some parametes do not exist!");
		return -1;
	}

	int isize = motor_names.size();

	Lookup lookup;
	auto group = lookup.getGroupFromNames({family_name},motor_names);
	group->setFeedbackFrequencyHz(100);
	GroupFeedback group_fbk(isize);

	if(group->getNextFeedback(group_fbk))
	{
		cout << "\n-------------------------------\nCallibrating double joint offsets:\n-------------------------------\n " << endl;
		auto pos = group_fbk.getPosition();
	
		vector<double> offsets(isize,0.0);

		int firstInd, secondInd, index;
		double firstVal, secondVal, offset;

		for(int i=0; i<motor_coupl_ind1.size(); i++)
		{
			firstInd = motor_coupl_ind1.at(i);
			secondInd = motor_coupl_ind2.at(i);
			firstVal = pos[firstInd];
			secondVal = pos[secondInd];
			if(motor_coef.at(firstInd)>motor_coef.at(secondInd)){
				index = secondInd;
			}else{
				index = firstInd;
			}
			// one is always positive the other negative
			offset = firstVal + secondVal;
			// store the offset
			offsets.at(index) = abs(offset*1e4)<0?0:round(offset*1e4)*1e-4;
			cout << "Correction for the motor at the index [" << index << "] is: " << offsets.at(index) << endl;
		}

		cout<<"Stored measured values at:"<<endl;
		string rospackage = string(argv[0]);
		string pack_path = rospackage.substr(0,rospackage.find_last_of("\\/"));
		pack_path = regex_replace(pack_path,regex("devel/lib"),"src");
		string path = pack_path.append("/launch/config/yaml/joint_offs.yaml");
		cout<< path << endl;

		ofstream myfile(path);
		if (myfile.is_open())
		{
			myfile << "hw:\n";
			myfile << "  joint:\n";
			myfile << "    comp: [";
			for(int i=0; i<isize; i++){
				if(i<isize-1){
					myfile << offsets[i] << ", ";
				}else{
					myfile << offsets[i] << "]\n";
				}
			}
			myfile.close();
		}
		else{
			cout << "Unable to open file";
			return -1;	
		}

		cout << "\n-------------------------------\nCallibration finished:\n-------------------------------\n " << endl;

		return 0;   
	}else{
		ROS_WARN("Could not access the family of modules!");
		return -1;
	}
} 