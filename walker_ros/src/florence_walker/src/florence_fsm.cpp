// Standard libraries
#include <iostream> // standard library
#include <chrono> 	// high resolution clock
#include <string>
#include <mutex>
#include <thread>

// ROS libraries
#include <ros/ros.h>
#include <std_msgs/UInt8.h>

// ROS control libraries
#include <controller_manager_msgs/LoadController.h>
#include <controller_manager_msgs/UnloadController.h>
#include <controller_manager_msgs/SwitchController.h>
#include <controller_manager_msgs/ListControllers.h>

#include "definitions.hpp"

#define ENTER_KEY 0x0A

using namespace ros;
using namespace std;

void read_input();
void ctrl_callback(const std_msgs::UInt8::ConstPtr & msg);
bool load_controllers(NodeHandle nh, const vector<string> & list);
bool unload_controllers(NodeHandle nh, const vector<string> & list);
void prepare_switch_lists(vector<string> & start_list, vector<string> & stop_list, string next_ctrl);
bool call_switch_srv(NodeHandle nh, const vector<string> &start_list, const vector<string> &stop_list);
bool switch_controllers(NodeHandle nh, vector<string> &start_list, vector<string> &stop_list, string next_ctrl);

enum stage {INIT, J_CTRL, CSH_CTRL, F_CTRL, END};

// global variables
ctrl_states status;
bool inp_was_read;
mutex mtx;
char fsm_ctrl;

int main(int argc, char** argv)
{
	// spawn the node
	ros::init(argc, argv,"florence_fsm");
	ros::NodeHandle nh;

	// each phase has an initialisation step - this flag is responsible for one time execution of the initialisation
	bool phase_ini = false;
	// some phases also have a end stage, where they wait for the transition
	bool phase_fin = false;

	// entry point for the fsm 
	stage ctrl_stage = INIT;

	// initialise globals
	fsm_ctrl = 0;
	inp_was_read = false;
	status = ctrl_states::IDLE;

	// currently active controller
	string active_ctrl = "";

	//controller names
	string jnt_ctrl = "/ctrl/joint_controller";
	string cshft_ctrl = "/ctrl/com_shift_controller";
	string full_ctrl = "/ctrl/full_controller";
	
	vector<string> ctrl_to_load;
	ctrl_to_load.push_back(jnt_ctrl);
	ctrl_to_load.push_back(full_ctrl);

	// controllers to be fed to the switching service
	vector<string> ctrl_to_start;
	vector<string> ctrl_to_stop;

	// list controllers service
	ros::ServiceClient ctrl_list_cli = nh.serviceClient<controller_manager_msgs::ListControllers>("/controller_manager/list_controller");
	controller_manager_msgs::ListControllers ctrl_list_srv;

	// check the state of the controller
	ros::Subscriber ctrl_status_sub = nh.subscribe("/vis_node/ctrl_status",1,ctrl_callback);


	// load all controllers
	if(!load_controllers(nh, ctrl_to_load)){
		ROS_ERROR("LOADING: Florence finite state machine has failed!");
		return -1;
	}else{
		ROS_INFO("Successfully loaded all the requested controllers!");
	}
	

	// non-RT loop
	bool init_dowhile = false;
	ros::Rate loop_rate(10);
	
	thread thd;
	do{
		// try to get the input (non-blocking)
		if(!init_dowhile){
			init_dowhile = true;
			thd = thread(&read_input);
		}else{
			if(inp_was_read){
				inp_was_read = false;
				// joint the thread with the parent thread
				thd.join();
				if(fsm_ctrl!='q'){
					// restart the thread if there was no quit signal
					thd = thread(&read_input);
				}else{
					// quit the loop
					break;
				}
			}
		}

		// check if the controller has encountered any problems
		if(status==ctrl_states::ERROR){
			ROS_ERROR("Controller has encountered an error and is not working properly. Exiting the FSM.");
			break; // break out of the do-while loop
		}
		// simple fsm implementation
		switch(ctrl_stage)
		{
			case INIT:
			{
				if(!phase_ini)
				{
					ROS_INFO("Init stage: Press ENTER to bring the robot to the home position. Press q to exit!");
					phase_ini = true;
				}
				
				if(fsm_ctrl == ENTER_KEY){
					phase_ini = false;
					phase_fin = false;
					ctrl_stage = J_CTRL;
					fsm_ctrl = 0;
					ctrl_status_sub.shutdown();
				}
				break;
			}

			case J_CTRL:
			{
				if(!phase_ini)
				{
					// switch controllers
					active_ctrl = jnt_ctrl;
					switch_controllers(nh, ctrl_to_start, ctrl_to_stop, active_ctrl);
					status=ctrl_states::IDLE;
					ctrl_status_sub = nh.subscribe("/vis_node/ctrl_status",1,ctrl_callback);
					ROS_INFO("Joint controller stage: Press q to exit!");
					phase_ini = true;
				}

				// joint controller has finished
				if(status==ctrl_states::STOPPED){
				//if(1){
					if(!phase_fin){
						ROS_INFO("Robot in home position. ");
						ROS_INFO("Lower the robot to the ground and press ENTER to adapt robot's COM height. Press q to exit!");
						phase_fin = true;
					}

					if(fsm_ctrl == ENTER_KEY){
						phase_ini = false;
						phase_fin = false;
						ctrl_stage = CSH_CTRL;
						fsm_ctrl = 0;
						ctrl_status_sub.shutdown();
					}
				}
				break;
			}

			case CSH_CTRL:
			{
				// not yet implemented
				/*if(!phase_ini)
				{
					// switch controllers
					active_ctrl = cshft_ctrl;
					switch_controllers(nh, ctrl_to_start, ctrl_to_stop, active_ctrl);
					status=ctrl_states::IDLE;
					ctrl_status_sub = nh.subscribe("/vis_node/ctrl_status",1,ctrl_callback);
					ROS_INFO("COM-height-shifting stage: Press q to exit!");
					phase_ini = true;
				}

				// COM-shifting-controller has finished
				if(status==ctrl_states::STOPPED){
					if(!phase_fin){
						ROS_INFO("Robot at a correct height.");
						ROS_INFO("Press ENTER start the 'locomotion'. Press q to exit!");
						phase_fin = true;
					}

					if(fsm_ctrl == ENTER_KEY){
						phase_ini = false;
						phase_fin = false;
						ctrl_stage = F_CTRL;
						fsm_ctrl = 0;
						ctrl_status_sub.shutdown();
					}
				}
				break;
				*/
				ctrl_stage = F_CTRL;
			}

			case F_CTRL:
			{
				if(!phase_ini)
				{
					// switch controllers
					active_ctrl = full_ctrl;
					switch_controllers(nh, ctrl_to_start, ctrl_to_stop, active_ctrl);
					status=ctrl_states::IDLE;
					ctrl_status_sub = nh.subscribe("/vis_node/ctrl_status",1,ctrl_callback);
					ROS_INFO("Locomotion stage: Press q to exit!");
					phase_ini = true;
				}

				// End of the locomotion stage
				if(status==ctrl_states::STOPPED){
					phase_ini = false;
					phase_fin = false;
					ctrl_stage = END;
					ctrl_status_sub.shutdown();
				}

				break;
			}

			case END: default:
			{
				if(!phase_fin){
					ROS_INFO("Terminal state.");
					ROS_INFO("Press q to exit!");
					phase_fin = true;
				}
				break;
			}
		} 

		// sleep until the next call
		loop_rate.sleep();
		// allow communication with other nodes (state update)
		ros::spinOnce();
	}while(ros::ok());

	// stop all controllers
	active_ctrl = "";
	if(!switch_controllers(nh, ctrl_to_start, ctrl_to_stop, active_ctrl)){
		ROS_ERROR("STOPPING: Florence finite state machine has failed!");
		return -1;
	}else{
		ROS_INFO("All controllers have been stopped!");
	}
	

	// unload 
	if(!unload_controllers(nh, ctrl_to_load)){
		ROS_ERROR("UNLOADING: Florence finite state machine has failed!");
		return -1;
	}


    return 0;    
} 


void read_input(){
	char tmp = getchar();
	mtx.lock();
	fsm_ctrl = tmp;
	inp_was_read = true;
	mtx.unlock();
}

void ctrl_callback(const std_msgs::UInt8::ConstPtr & msg)
{
	// use of a global variable
	status = (ctrl_states)msg->data; 
}

bool load_controllers(NodeHandle nh, const vector<string> & list)
{
	// load controllers service
	ros::ServiceClient cli = nh.serviceClient<controller_manager_msgs::LoadController>("/controller_manager/load_controller");
	controller_manager_msgs::LoadController srv;

	for(int i=0; i < list.size(); i++){
		string cname;
		if(!nh.getParam(list.at(i)+"/type",cname)){
			ROS_WARN("Controller was not found in the rosparam cloud.");
			return false;
		}else{
			srv.request.name = list.at(i);
			if(cli.call(srv)){
				if(srv.response.ok){
					ROS_INFO("Controller %s successfully loaded.", cname.c_str());
				}
				else{
					ROS_INFO("Controller %s was not loaded.", cname.c_str());
					return false;
				}
			}else{
				ROS_INFO("Failed to contact the controller manager!");
				return false;
			}
		}
	}
	return true;
}

bool unload_controllers(NodeHandle nh, const vector<string> & list)
{
	// unload controllers service
	ros::ServiceClient cli = nh.serviceClient<controller_manager_msgs::UnloadController>("/controller_manager/unload_controller");
	controller_manager_msgs::UnloadController srv;

	for(int i=0; i < list.size(); i++){
		string cname;
		if(!nh.getParam(list.at(i)+"/type",cname)){
			ROS_WARN("Controller was not found in the rosparam cloud.");
			return false;
		}else{
			srv.request.name = list.at(i);
			if(cli.call(srv)){
				if(srv.response.ok){
					ROS_INFO("Controller %s successfully unloaded.", cname.c_str());
				}
				else{
					ROS_INFO("Controller %s was not unloaded.", cname.c_str());
					return false;
				}
			}else{
				ROS_INFO("Failed to contact the controller manager!");
				return false;
			}
		}
	}
	return true;
}

void prepare_switch_lists(vector<string> & start_list, vector<string> & stop_list, string next_ctrl){
	stop_list = start_list;
	start_list.clear();
	if(next_ctrl!="")
		start_list.push_back(next_ctrl);
}

bool call_switch_srv(NodeHandle nh, const vector<string> & start_list, const vector<string> & stop_list)
{
	// switch controllers service
	ros::ServiceClient cli = nh.serviceClient<controller_manager_msgs::SwitchController>("/controller_manager/switch_controller");
	controller_manager_msgs::SwitchController srv;
	srv.request.strictness = controller_manager_msgs::SwitchController::Request::STRICT;
	srv.request.start_asap = true;
	srv.request.start_controllers = start_list;
	srv.request.stop_controllers = stop_list;

	if(cli.call(srv)){
		if(!srv.response.ok){
			ROS_INFO("Switching has failed");
			return false;
		}
	}else{
		ROS_INFO("Failed to contact the controller manager!");
		return false;
	}
	return true;
}

bool switch_controllers(NodeHandle nh, vector<string> & start_list, vector<string> & stop_list, string next_ctrl)
{
	// prepare the lists of controllers
	prepare_switch_lists(start_list, stop_list, next_ctrl);
	// switch between two controller lists
	return call_switch_srv(nh, start_list, stop_list);
}