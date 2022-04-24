#include <iostream>
#include <string>
#include "definitions.hpp"

// information about the HEBI module (other info like gains & so on could potentially be added) - only a data storage class
class MotorData{
	private:
		std::string family;			// family of the motor (Florence in our case)
		std::string name; 			// name of the module (for example leftAnkle1, leftAnkle2, ...)
		// measurements
		double pos;	
		double vel;
		double eff;
		double temp;
		double acc[3];
		double gyro[3];
		// commands
		double pos_cmd;
		double vel_cmd;
		double eff_cmd;


	public:
		//constructor
		MotorData();
		MotorData(std::string family, std::string name);

		// setters
		void setFamily(std::string family);
		void setName(std::string name);
		void updateMeasurements(double pos, double vel, double eff, Eigen::Vector3d acc, Eigen::Vector3d gyro);
		void updateTemperature(double temp);
		void updateCommands(double pos, double vel, double eff);

		// getters
		std::string getFamily();
		std::string getName();
		double getPosition();
		double getVelocity();
		double getEffort();
		Eigen::Vector3d getAcceleration();
		Eigen::Vector3d getGyro();
		double getTemperature();
		double getPositionCmd();
		double getVelocityCmd();
		double getEffortCmd();
};