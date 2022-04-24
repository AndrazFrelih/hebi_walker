#include "FlorenceVisNode.hpp"

using namespace std;

FlorenceVisNode::FlorenceVisNode(){
	this->stopExec = false;
}

FlorenceVisNode::~FlorenceVisNode(){}

// DONE
void FlorenceVisNode::startPubThread()
{
	// wait for the thread to close
	ROS_INFO("Starting the publishing thread");
	// create a separate thread that publishes states
	this->stopExec = false;
	// start the publishing thread
	this->thd = std::thread(&FlorenceVisNode::pubCallback,this);
}

// DONE
void FlorenceVisNode::stopPubThread()
{
	// wait for the thread to close
	ROS_INFO("Stopping the publishing thread");
	// signal the thread to stop the execution
	this->stopExec = true;
	// wait for the thread to finish
	this->thd.join();
}

// initialise visualisation node (parent passes the mutex, so checking can be done on the parent side)
void FlorenceVisNode::init(ros::NodeHandle& main_nh, std::mutex * mtx)
{
	this->mtx = mtx;
	// copy the main node handle
	this->main_nh = main_nh;

	// read required parameters from the cloud
	if(!this->main_nh.getParam("/hw/joint/names",this->joint_names)){
		ROS_ERROR("VIS NODE: Joint names were not found.");
	}else{
		this->dim = this->joint_names.size();
	}
	if(!this->main_nh.getParam("/est/fsen/fnames/left",this->lfs_fnames)){
		ROS_ERROR("VIS NODE: Left load cell frames not found.");
	}
	if(!this->main_nh.getParam("/est/fsen/fnames/right",this->rfs_fnames)){
		ROS_ERROR("VIS NODE: Right load cell frames not found.");
	}
	int fs;
	if(!this->main_nh.getParam("/ctrl/full_controller/publish_rate",fs)){
		ROS_ERROR("VIS NODE: Controller rate not found.");
	}
	// publish with at least 1 Hz
	this->pub_rate = fs/10?fs/10:1;


	if(!this->main_nh.getParam("/est/endeffector/names/left",this->eeL_name)){
		ROS_ERROR("VIS NODE: Left end effector name not found.");
	}

	if(!this->main_nh.getParam("/est/endeffector/names/right",this->eeR_name)){
		ROS_ERROR("VIS NODE: Right end effector name not found.");
	}

	//
	this->initLogger();

	// joint states
	this->vis_counter = 0;
	this->vis_lim = 1;
	this->jmeas_msg_pub = this->nh.advertise<sensor_msgs::JointState>("/meas/joint_states", 1);
	this->jmeas_msg.name = this->joint_names;
	this->jmeas_msg.position.resize(dim);

	this->jcmd_msg_pub = this->nh.advertise<sensor_msgs::JointState>("/cmd/joint_states", 1);
	this->jcmd_msg.name = this->joint_names;
	this->jcmd_msg.position.resize(dim);

	this->rstatus_msg_pub = this->nh.advertise<std_msgs::UInt8>("/vis_node/ctrl_status", 1);

	// force sensors
	for(int i=0; i<lfs_fnames.size();i++){
		geometry_msgs::WrenchStamped fs_wrench_msg;
		fs_wrench_msg.header.frame_id = "meas/"+this->lfs_fnames[i];
		this->lfs_wrench_msgs.push_back(fs_wrench_msg);
		this->lfs_wrench_pubs.push_back(this->nh.advertise<geometry_msgs::WrenchStamped>("/load_cell_L"+std::to_string(i+1), 1));
	}
	for(int i=0; i<rfs_fnames.size();i++){
		geometry_msgs::WrenchStamped fs_wrench_msg;
		fs_wrench_msg.header.frame_id = "meas/"+this->rfs_fnames[i];
		this->rfs_wrench_msgs.push_back(fs_wrench_msg);
		this->rfs_wrench_pubs.push_back(this->nh.advertise<geometry_msgs::WrenchStamped>("/load_cell_R"+std::to_string(i+1), 1));
	}	

	// LIPM visualisation
	this->initLipmPubs();
}

// DONE
void FlorenceVisNode::initLogger()
{
	// initialise placeholders for publishers
	this->logger_data.q_m.resize(this->dim);
	this->logger_data.qp_m.resize(this->dim);
	this->logger_data.qeff_m.resize(this->dim);
	this->logger_data.q_c.resize(this->dim);
	this->logger_data.qp_c.resize(this->dim);
	this->logger_data.qeff_c.resize(this->dim);
	this->logger_data.fs.resize(2);
	this->logger_data.rstatus = (uint8_t)ctrl_states::IDLE;
	this->logger_data.meas.Rfb = Eigen::Matrix3d::Identity();
	this->logger_data.cmd.Rfb = Eigen::Matrix3d::Identity();
}

// DONE
void FlorenceVisNode::initLipmPubs()
{
	string meas = "meas/";
	string cmd = "cmd/";
	this->orig = "World";
	this->orig_meas = meas+this->orig;
	this->orig_cmd = cmd+this->orig;
	this->meas_lipm_list.push_back(meas+"ZMP");
	this->meas_lipm_list.push_back(meas+"COM");
	this->meas_lipm_list.push_back(meas+"ZMPlocL");
	this->meas_lipm_list.push_back(meas+"ZMPlocR");
	this->cmd_lipm_list.push_back(cmd+"ZMP");
	this->cmd_lipm_list.push_back(cmd+"COM");

	this->full_list = this->meas_lipm_list;
	this->full_list.insert(this->full_list.end(),this->cmd_lipm_list.begin(), this->cmd_lipm_list.end());

	for(int i=0; i < this->full_list.size(); i++){
		this->lipm_msg_pub.push_back(this->nh.advertise<geometry_msgs::PointStamped>(this->full_list.at(i)+"_topic", 1));
	}

	this->lipm_msg.point.x = 0;
	this->lipm_msg.point.y = 0;
	this->lipm_msg.point.z = 0;
}


// publish joints/forces/com/zmp/dcm
void FlorenceVisNode::pub()
{
	this->vis_counter++;
	if(this->vis_counter >= this->vis_lim){
		// publish joint angles
		for(int i=0; i < this->joint_names.size(); i++){
			this->jmeas_msg.position[i] = this->logger_data.q_m[i];
			this->jcmd_msg.position[i] = this->logger_data.q_c[i];
		}
		// measurement model
		this->jmeas_msg.header.stamp = ros::Time::now();
		this->jmeas_msg_pub.publish(this->jmeas_msg);
		
		// control model
		this->jcmd_msg.header.stamp = ros::Time::now();
		this->jcmd_msg_pub.publish(this->jcmd_msg);		

		// publish force measurents
		// TODO get W-EE tf and rotate all the force vectors (get from RBDL and not tf tree - is faster)
		for (int i = 0; i < NLC; i++)
		{
			this->lfs_wrench_msgs[i].wrench.force.z = this->logger_data.fs.at(0)[i];
			this->lfs_wrench_pubs[i].publish(this->lfs_wrench_msgs[i]);
			this->rfs_wrench_msgs[i].wrench.force.z = this->logger_data.fs.at(1)[i];
			this->rfs_wrench_pubs[i].publish(this->rfs_wrench_msgs[i]);
		}

		// publish robot state
		this->rstatus_msg.data = this->logger_data.rstatus;
		this->rstatus_msg_pub.publish(this->rstatus_msg);
		
		this->transformPublisher(this->logger_data.meas.Xfb, this->logger_data.meas.Rfb, this->orig, this->orig_meas);
		this->transformPublisher(this->logger_data.meas.zmp, Eigen::Matrix3d::Identity(), this->orig, this->meas_lipm_list.at(0));
		this->transformPublisher(this->logger_data.meas.com, Eigen::Matrix3d::Identity(), this->orig, this->meas_lipm_list.at(1));
		this->transformPublisher(this->logger_data.meas.zmplocL, Eigen::Matrix3d::Identity(), "meas/"+this->eeL_name, this->meas_lipm_list.at(2));
		this->transformPublisher(this->logger_data.meas.zmplocR, Eigen::Matrix3d::Identity(), "meas/"+this->eeR_name, this->meas_lipm_list.at(3));
		this->transformPublisher(this->logger_data.meas.Xfb, this->logger_data.meas.Rfb, this->orig, this->orig_cmd);
		this->transformPublisher(this->logger_data.cmd.zmp, Eigen::Matrix3d::Zero(), this->orig, this->cmd_lipm_list.at(0));
		this->transformPublisher(this->logger_data.cmd.com, Eigen::Matrix3d::Zero(), this->orig, this->cmd_lipm_list.at(1));

		
		for(int i = 0; i < this->lipm_msg_pub.size(); i++){
			this->lipm_msg.header.stamp = ros::Time::now();
			this->lipm_msg.header.frame_id = this->full_list.at(i);
			this->lipm_msg_pub.at(i).publish(this->lipm_msg);
		}

		// reset the counter
		this->vis_counter = 0;
	}
}

// run publishers on another thread
void FlorenceVisNode::pubCallback()
{
	ros::Rate loop_rate(this->pub_rate);
	while(ros::ok() && !this->stopExec){
		// lock the thread so that there are no write violations
		this->mtx->lock();
		// publish stuff
		this->pub();
		// unlock the thread
		this->mtx->unlock();
		// sleep for some time
		loop_rate.sleep();
	}	
}

// copy the states from the controller to the visualisation node (known execution time)
void FlorenceVisNode::setStates(std::vector<double> q_m,
								std::vector<double> qp_m, 
								std::vector<double> qeff_m, 
								std::vector<double> q_c,
								std::vector<double> qp_c,
								std::vector<double> qeff_c,
				    			std::vector<Eigen::Matrix<double,4,1>> fs,
				    			Targets meas,
								Targets cmd,
				    			uint8_t rstatus)
{
	// copy everything
	this->logger_data.q_m = q_m;
	this->logger_data.qp_m = qp_m;
	this->logger_data.qeff_m = qeff_m;
	this->logger_data.q_c = q_c;
	this->logger_data.qp_c = qp_c;
	this->logger_data.qeff_c = qeff_c;
	this->logger_data.fs = fs;
	this->logger_data.meas = meas;
	this->logger_data.cmd = cmd;
	this->logger_data.rstatus = rstatus;
}

// DONE
void FlorenceVisNode::transformPublisher(Eigen::Vector3d X, Eigen::Matrix3d Rotm, string parent, string child)
{
	Eigen::Quaternion<double> quat(Rotm);
	quat.normalize();
	geometry_msgs::TransformStamped tfStamped;

	tfStamped.header.stamp = ros::Time::now();
	tfStamped.header.frame_id = parent;
	tfStamped.child_frame_id = child;
	tfStamped.transform.translation.x = X(0);
	tfStamped.transform.translation.y = X(1);
	tfStamped.transform.translation.z = X(2);
	tfStamped.transform.rotation.x = quat.x();
	tfStamped.transform.rotation.y = quat.y();
	tfStamped.transform.rotation.z = quat.z();
	tfStamped.transform.rotation.w = quat.w();
	this->broadcaster.sendTransform(tfStamped);
}