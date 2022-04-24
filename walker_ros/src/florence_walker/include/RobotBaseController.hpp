// Standard libraries
#include <iostream>
#include <string>
#include <cmath>
#include <thread>
#include <mutex>

// ROS standard libraries
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/WrenchStamped.h>

// ROS control
#include <controller_interface/controller_base.h>
#include <hardware_interface/internal/demangle_symbol.h>
#include <hardware_interface/imu_sensor_interface.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>
#include "load_cell_array.h" //custom handle and interface library

// Custom libraries
#include "definitions.hpp"

#include "FlorenceVisNode.hpp"
#include "Targets.hpp"

// Plotting
//#include "util/plot_functions.h"

class RobotBaseController : public controller_interface::ControllerBase
{
private:

	// Hardware interfaces
    std::vector<hardware_interface::JointStateHandle>    hiJointStates;
    std::vector<hardware_interface::ImuSensorHandle>     hiImus;
    std::vector<hardware_interface::LoadCellArrayHandle> hiFtSensors;
    std::vector<hardware_interface::JointHandle>         hiJointPos;
    std::vector<hardware_interface::JointHandle>         hiJointVel;
    std::vector<hardware_interface::JointHandle>         hiJointEff;

    // Sensor names
    std::vector<std::string> joint_names;   //actuators have the same name
    std::vector<std::string> fsen_names;
    std::vector<std::string> imu_names;
    std::vector<int> used_imu;

    // Measured values
    std::vector<double> q_m;
    std::vector<double> qp_m;
    std::vector<double> qeff_m;
    std::vector<Eigen::Vector3d> accel;
    std::vector<Eigen::Vector3d> gyro;
    std::vector<Vector4d> fsen;

    // Commanded values
    std::vector<double> q_c;
    std::vector<double> qp_c;
    std::vector<double> qeff_c;

    // Visualisation
    std::mutex mtx;
    FlorenceVisNode fvn;

    // Initialization functions
    bool init(hardware_interface::JointStateInterface*  jnt_state_interface,
               	hardware_interface::LoadCellArrayInterface* fsen_interface,
				hardware_interface::ImuSensorInterface*     imu_sen_interface,
				hardware_interface::PositionJointInterface* pos_interface,
				hardware_interface::VelocityJointInterface* vel_interface,
				hardware_interface::EffortJointInterface*   eff_interface,
				ros::NodeHandle&        nh,
				ros::NodeHandle&        controller_nh);

    // Interface initialization functions
    void initAlgorithm(ros::NodeHandle& root_nh, ros::NodeHandle& controller_nh);

    bool initMotorInputs(hardware_interface::JointStateInterface* jnt_state_interface,
                        ros::NodeHandle&      nh);

    bool initMotorOutputs(hardware_interface::PositionJointInterface* pos_interface,
                        hardware_interface::VelocityJointInterface*   vel_interface,
                        hardware_interface::EffortJointInterface*     eff_interface,
                        ros::NodeHandle&          nh);

    bool initForceSensors(hardware_interface::LoadCellArrayInterface* fsen_interface,
                          ros::NodeHandle&            nh);

    bool initImuSensors(hardware_interface::ImuSensorInterface* imu_sen_interface,
                          ros::NodeHandle&    nh);

    // Get measurements from input handles
    void readMeasurements();
    // Init commands so that robot does not perform ant undesired jerky movements
    void initCmds();
    // Compute a step of the algorithm - Has to be overwritten by the extending class
    virtual void algorithmStep(const ros::Duration& period) {};
    // Write commands back to their respective handless
    void sendCommands();

public:
    // functions required by ros_control controller_interface::ControllerBase 
    bool initRequest(hardware_interface::RobotHW* robot_hw,
                   ros::NodeHandle&             root_nh,
                   ros::NodeHandle&             controller_nh,
                   ClaimedResources&            claimed_resources);

    void starting(const ros::Time& time);
    void update(const ros::Time& time, const ros::Duration& period);
    void stopping(const ros::Time& time);
    std::string getHardwareInterfaceType() const; //we do not override this

    // convenience fcns - overwrite in the extending class
    virtual void startController(){};
    virtual void stopController(){};

    // constr. & destr.
    RobotBaseController();
    ~RobotBaseController();

    // getters (to make the controller more readable)
    std::vector<double> getMeasPos(){return this->q_m;}
    std::vector<double> getMeasVel(){return this->qp_m;}
    std::vector<double> getMeasEff(){return this->qeff_m;}
    std::vector<double> getCmdPos(){return this->q_c;}
    std::vector<double> getCmdVel(){return this->qp_c;}
    std::vector<double> getCmdEff(){return this->qeff_m;}
    std::vector<Vector4d> getMeasForces(){return this->fsen;}
    std::vector<Eigen::Vector3d> getMeasAccel(){return this->accel;}
    std::vector<Eigen::Vector3d> getMeasGyro(){return this->gyro;}
    void setCmdPos(int i, double val){this->q_c.at(i) = val;}
    void setCmdVel(int i, double val){this->qp_c.at(i) = val;}
    void setCmdEff(int i, double val){this->qeff_c.at(i) = val;}

    // share the mutex pointer
    std::mutex * getMtxPtr(){return &(this->mtx);}

    // interface to the visualisation node
    void visualise(std::vector<double> q_m,
					std::vector<double> qp_m, 
					std::vector<double> qeff_m, 
					std::vector<double> q_c,
					std::vector<double> qp_c,
					std::vector<double> qeff_c,
					std::vector<Vector4d> fs,
                    Targets meas,
                    Targets cmd,
                    uint8_t rstatus);
};