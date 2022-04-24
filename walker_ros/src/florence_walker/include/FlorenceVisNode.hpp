// Standard libraries
#include <iostream>
#include <mutex>
#include <string>
#include <thread>

// ROS standard libraries
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/WrenchStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <std_msgs/UInt8.h>

// TF library
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>

// custom libraries
#include "definitions.hpp"
#include "Targets.hpp"

class FlorenceVisNode{
private:
	// Node handle
	ros::NodeHandle main_nh; 													//controller handle
	ros::NodeHandle nh; 														//publishing handle
	int pub_rate;	 															//frequency of publishing
	int dim;

	// Thread programming
	std::mutex *mtx;															// pointer to the mutex passed by the parent class
	std::thread thd;															// thread on which the algorithm is running
	std::atomic<bool> stopExec;													// bool which is used to stop the thread from the main process

	// Visualisation
	std::vector<std::string> joint_names;										// names of all the joints (so that rviz can maps values correctly)
	int vis_counter;															// additional value for frequency decimation (deprecated - should be removed)
	int vis_lim;																// limit to which the counter counts
	// joint position measurement
	ros::Publisher jmeas_msg_pub;
	sensor_msgs::JointState jmeas_msg;
	// joint position command
	ros::Publisher jcmd_msg_pub;
	sensor_msgs::JointState jcmd_msg;
	// base to world transformation
	tf2_ros::TransformBroadcaster broadcaster;

	// endeffector names
	std::string eeL_name, eeR_name;												// names of endeffector coordinate frames	

	// LIPM visualisation
	std::string orig, orig_cmd, orig_meas;										// names of world frames (rviz origin, measurement robot model origin, control robot model origin)
	std::vector<std::string> cmd_lipm_list, meas_lipm_list, full_list;			// lists of all LIPM frames
	std::vector<ros::Publisher> lipm_msg_pub;
	geometry_msgs::PointStamped lipm_msg;										// LIPM variables should have a point appended to them so visualisation is clearer. Has not been implemented yet

	// robot state
	ros::Publisher rstatus_msg_pub;
	std_msgs::UInt8 rstatus_msg;												// status of the controller which is advertized over the VisNode. If there is an error, the NON-RT node can stop and unload the controller (send signals to Controller manager to do that)

	// wrenches for force measurement
	std::vector<std::string> lfs_fnames;
	std::vector<geometry_msgs::WrenchStamped> lfs_wrench_msgs;
	std::vector<ros::Publisher> lfs_wrench_pubs;
	std::vector<std::string> rfs_fnames;
	std::vector<geometry_msgs::WrenchStamped> rfs_wrench_msgs;
	std::vector<ros::Publisher> rfs_wrench_pubs;

	struct monitor{
		std::vector<double> q_m;												// measured joint angles
		std::vector<double> qp_m;												// measured joint velocities
		std::vector<double> qeff_m;												// measured joint efforts
		std::vector<double> q_c;												// commanded joint angles
		std::vector<double> qp_c;												// commanded joint velocities
		std::vector<double> qeff_c;												// commanded joint efforts
	    std::vector<Vector4d> fs;												// measured load cell force values
	    Targets meas;															// targets from the FBest module
	    Targets cmd;															// targets from the TG module
	    uint8_t rstatus;														// status of the controller (is defined in definitions, but enum is converted to uint for sending, so that message generation was not necessary)
	} logger_data;

	// callback function that is executed on the other thread
	void pubCallback();
	// call to all the publishers
	void pub();
	// helper function for pub()
	void transformPublisher(Eigen::Vector3d X, Eigen::Matrix3d Rotm, std::string child, std::string parent);
	// initialize the logger_data structure (so that robot is in a certain poisition)
	void initLogger();
	// initialize LIPM publishers
	void initLipmPubs();
	
	
public:
	// constructor and destructor
	FlorenceVisNode();
	~FlorenceVisNode();

	// initialize internal variables
	void init(ros::NodeHandle& main_nh, std::mutex * mtx);

	// function that is called by the controller to publish states
	void setStates( std::vector<double> q_m,
					std::vector<double> qp_m, 
					std::vector<double> qeff_m, 
					std::vector<double> q_c,
					std::vector<double> qp_c,
					std::vector<double> qeff_c,
					std::vector<Vector4d> fs,
					Targets meas,
					Targets cmd,
					uint8_t rstatus);

	// the RobotBaseController calls these two services when controllers are started and stopped
	void startPubThread();
	void stopPubThread();

};
