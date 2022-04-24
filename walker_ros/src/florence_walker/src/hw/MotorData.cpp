#include "MotorData.hpp"

MotorData::MotorData(std::string family, std::string name){
	this->setFamily(family);
	this->setName(name);
	Eigen::Vector3d zeros = Eigen::Vector3d::Constant(0);
	this->updateMeasurements(0.0, 0.0, 0.0, zeros, zeros);
	this->updateCommands(0.0, 0.0, 0.0);
}

MotorData::MotorData() : MotorData("", "") {}


// setters
void MotorData::setFamily(std::string family){
	this->family = family;
}

void MotorData::setName(std::string name){
	this->name = name;
}

void MotorData::updateMeasurements(double pos, double vel, double eff, Eigen::Vector3d acc, Eigen::Vector3d gyro){
	this->pos = pos;
	this->vel = vel;
	this->eff = eff;
	this->acc[0] = acc[0];
	this->acc[1] = acc[1];
	this->acc[2] = acc[2];
	this->gyro[0] = gyro[0];
	this->gyro[1] = gyro[1];
	this->gyro[2] = gyro[2];
}

void MotorData::updateTemperature(double temp){
	this->temp = temp;	
}

void MotorData::updateCommands(double pos, double vel, double eff){
	this->pos_cmd = pos;
	this->vel_cmd = vel;
	this->eff_cmd = eff;
}

// getters
std::string MotorData::getFamily(){
	return this->family;
}

std::string MotorData::getName(){
	return this->name;
}

double MotorData::getPosition(){
	return this->pos;
}

double MotorData::getVelocity(){
	return this->vel;
}

double MotorData::getEffort(){
	return this->eff;
}

double MotorData::getPositionCmd(){
	return this->pos_cmd;
}

double MotorData::getVelocityCmd(){
	return this->vel_cmd;
}

double MotorData::getEffortCmd(){
	return this->eff_cmd;
}

Eigen::Vector3d MotorData::getAcceleration(){
	Eigen::Vector3d acc; 
	acc << this->acc[0], this->acc[1], this->acc[2];
	return acc;
}

Eigen::Vector3d MotorData::getGyro(){
	Eigen::Vector3d gyro; 
	gyro << this->gyro[0], this->gyro[1], this->gyro[2];
	return gyro;
}

double MotorData::getTemperature(){
	return this->temp;
}