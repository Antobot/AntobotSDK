/*
# Copyright (c) 2023, ANTOBOT LTD.
# All rights reserved.

# # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # 

Description: 	The primary purpose of this code is to serve as a communication channel between the ROS components of
		the Xavier NX and the the Aurix. This channel goes in 2 directions. It reads information from antobot_robot/cmd_vel 
		and other relevant ROS topics, converts it to the format defined in AntoBridge, and sends the
		data to the AntoBridge ROS node via publishers. Likewise, it reads data from AntoBridge using subscribers, and
		passes this data along to the robot to calculate wheel odometry and other relevant information.

Contacts: 	daniel.freer@antobot.ai

# # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # #
*/

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Bool.h"
#include <sstream>
#include <numeric>
#include "stdlib.h"

#include <antobot_base/antobot_control.h>
#include <joint_limits_interface/joint_limits_interface.h>
#include <joint_limits_interface/joint_limits.h>
#include <joint_limits_interface/joint_limits_urdf.h>
#include <joint_limits_interface/joint_limits_rosparam.h>

#include <std_msgs/Float32.h>
#include <std_msgs/UInt8.h>
#include <antobot_msgs/UInt8_Array.h>
#include <antobot_msgs/Float32_Array.h>

using namespace hardware_interface;
using joint_limits_interface::JointLimits;
using joint_limits_interface::SoftJointLimits;
using joint_limits_interface::PositionJointSoftLimitsHandle;
using joint_limits_interface::PositionJointSoftLimitsInterface;
using joint_limits_interface::VelocityJointSoftLimitsHandle;
using joint_limits_interface::VelocityJointSoftLimitsInterface;


namespace antobot_hardware_interface
{
	
	antobotHardwareInterface::antobotHardwareInterface(ros::NodeHandle& nh) \
		: nh_(nh)
	{
		/* Constructor function - Initialises robot definition and serial port to Aurix, defines the ROS loop
		and callback function, sends calibration command to stepper motors, turns on lights */
		// // Inputs: ROS node handle
		
		// Initialise robot definition
		init();
		init_publishers();

		controller_manager_.reset(new controller_manager::ControllerManager(this, nh_));

		// Defines a ROS loop which calls the update() function at a rate of 25Hz
		nh_.param("/antobot/hardware_interface/loop_hz", loop_hz_, 25.0);
		ROS_DEBUG_STREAM_NAMED("constructor","Using loop frequency of " << loop_hz_ << " hz");
		ros::Duration update_freq = ros::Duration(1.0/loop_hz_);
		non_realtime_loop_ = nh_.createTimer(update_freq, &antobotHardwareInterface::update, this);

		ROS_INFO_NAMED("hardware_interface", "Loaded robot controller.");

		wheel_vel_windows = {{0}, {0}, {0}, {0}};
		wheel_vels_filt.data = {0, 0, 0, 0};

	}

	antobotHardwareInterface::~antobotHardwareInterface()
	{
		
	}

	void antobotHardwareInterface::init()
	{
		/* Robot initialisation - defines joint names, modes, and related variables as read from the yaml file defined in the launchfile. 
		Defines joint state handles, including joint position handles and joint velocity handles, sets limits for the controller,
		and registers all information to the ROS hardware interface. */
		
		// Get joint names
		nh_.getParam("/antobot_robot/hardware_interface/joints", joint_names_);
		if (joint_names_.size() == 0)
		{
		  ROS_FATAL_STREAM_NAMED("init","No joints found on parameter server for controller. Did you load the proper yaml file?");
		}
		num_joints_ = joint_names_.size();


		nh_.getParam("/antobot_robot/hardware_interface/joints_modes", joint_modes_);
		if (joint_modes_.size() == 0)
		{
		  ROS_FATAL_STREAM_NAMED("init","No joint modes found");
		}

		// Resize vectors
		joint_position_.resize(num_joints_);
		joint_velocity_.resize(num_joints_);
		joint_effort_.resize(num_joints_);
		joint_commands_.resize(num_joints_);

		// Initialize controller
		for (int i = 0; i < num_joints_; ++i)
		{
			//antobot_driver::Joint joint = antobot1.getJoint(joint_names_[i]);

		  	ROS_DEBUG_STREAM_NAMED("constructor","Loading joint name: " << joint_names_[i]);

		  	// Create joint state interface
			JointStateHandle jointStateHandle(joint_names_[i], &joint_position_[i], &joint_velocity_[i], &joint_effort_[i]);
		  	joint_state_interface_.registerHandle(jointStateHandle);

			if (joint_modes_[i] == 0) {
			  	create_position_joint(i, jointStateHandle);
			}
			else {
				create_velocity_joint(i, jointStateHandle);
			}
		}

		init_joint_variables();
	}

	void antobotHardwareInterface::create_position_joint(int i, JointStateHandle jointStateHandle)
	{
		/* Creates and registers a single position-based control joint */
		
		// Create position joint interface
		JointHandle jointPositionHandle(jointStateHandle, &joint_commands_[i]);
		JointLimits limits;
	 	SoftJointLimits softLimits;

		softLimits.k_position = HUGE_VAL;
		softLimits.k_velocity = HUGE_VAL;
		softLimits.max_position = HUGE_VAL;
		softLimits.min_position = -HUGE_VAL;

		if (getJointLimits(joint_names_[i], nh_, limits) == false) {
			ROS_ERROR_STREAM("Cannot set joint limits for " << joint_names_[i]);
		} else {
			// Adds joint limits to the position joint soft limits interface
			PositionJointSoftLimitsHandle jointLimitsHandle(jointPositionHandle, limits, softLimits);
			positionJointSoftLimitsInterface.registerHandle(jointLimitsHandle);
		}
		// Registers joint position handle to the position joint interface
		position_joint_interface_.registerHandle(jointPositionHandle);
	}

	void antobotHardwareInterface::create_velocity_joint(int i, JointStateHandle jointStateHandle)
	{
		/* Creates and registers a single velocity-based control joint */
		
		// Create velocity joint interface
		JointHandle jointVelocityHandle(jointStateHandle, &joint_commands_[i]);
		JointLimits limits;
	 	SoftJointLimits softLimits;

		softLimits.k_position = HUGE_VAL;
		softLimits.k_velocity = HUGE_VAL;
		softLimits.max_position = HUGE_VAL;
		softLimits.min_position = -HUGE_VAL;

		if (getJointLimits(joint_names_[i], nh_, limits) == false) {
			ROS_ERROR_STREAM("Cannot set joint limits for " << joint_names_[i]);
		} else {
			VelocityJointSoftLimitsHandle jointLimitsHandle(jointVelocityHandle, limits, softLimits);
			velocityJointSoftLimitsInterface.registerHandle(jointLimitsHandle);
		}
		// Registers joint velocity handle to the velocity joint interface
		velocity_joint_interface_.registerHandle(jointVelocityHandle);
	}

	void antobotHardwareInterface::init_joint_variables()
	{
		/* Registers the interfaces for each type of robot joint, and initialises variables to zero */
		
		// Registers the interface for each robot joint to the ROS hardware_interface system
		registerInterface(&joint_state_interface_);
		registerInterface(&position_joint_interface_);
		registerInterface(&velocity_joint_interface_);

		registerInterface(&positionJointSoftLimitsInterface);
		registerInterface(&velocityJointSoftLimitsInterface);

		joint_position_[0] = 0;
		joint_position_[1] = 0;
		joint_position_[2] = 0;
		joint_position_[3] = 0;

		joint_velocity_[0] = 0;
		joint_velocity_[1] = 0;
		joint_velocity_[2] = 0;
		joint_velocity_[3] = 0;

		joint_effort_[0] = 0;
		joint_effort_[1] = 0;
		joint_effort_[2] = 0;
		joint_effort_[3] = 0;

		wheel_vels[0] = 0;
		wheel_vels[1] = 0;
		wheel_vels[2] = 0;
		wheel_vels[3] = 0;

		steer_pos[0] = 0;
		steer_pos[1] = 0;
		steer_pos[2] = 0;
		steer_pos[3] = 0;
	}

	void antobotHardwareInterface::init_publishers()
	{
		/* Initialises ROS publishers which send commands to anto_bridge */
		
		wheel_vel_cmd_pub = nh_.advertise<antobot_msgs::Float32_Array>("/antobridge/wheel_vel_cmd", 1);
		steer_pos_cmd_pub = nh_.advertise<antobot_msgs::Float32_Array>("/antobridge/steer_pos_cmd", 1);
		wheel_vel_filt_pub = nh_.advertise<antobot_msgs::Float32_Array>("/am/control/wheel_vel_filt", 1);
	}

	void antobotHardwareInterface::update(const ros::TimerEvent& e)
	{
		/* The callback function for the ROS timer defined in the constructor. This will occur at a rate of 25Hz.
		The main purpose of this function is to call the push_motor_info() and write() functions, which read and write from
		the AntBridge interface. */
		
		_logInfo = "\n";
		_logInfo += "Joint Position Command:\n";
		for (int i = 0; i < num_joints_; i++)
		{
			std::ostringstream jointCommandStr;
			jointCommandStr << joint_commands_[i];
			_logInfo += "  " + joint_names_[i] + ": " + jointCommandStr.str() + "\n";
		}

		elapsed_time_ = ros::Duration(e.current_real - e.last_real);

		push_motor_info();
		controller_manager_->update(ros::Time::now(), elapsed_time_);
		write(elapsed_time_);

	}

	void antobotHardwareInterface::push_motor_info()
	{
		/* Pushes motor information from AntoBridge to the ros controller via the ROS-defined hardware interface
		*/

		if (num_joints_ < 5)	// Included to allow for more complex robots later
		{

			joint_velocity_[0] = wheel_vels[0];
			joint_velocity_[1] = wheel_vels[1];
			joint_velocity_[2] = wheel_vels[2];
			joint_velocity_[3] = wheel_vels[3];

			double dt;
			dt = elapsed_time_.toSec();

			joint_position_[0] += wheel_vels[0] * dt;
			joint_position_[1] += wheel_vels[1] * dt;
			joint_position_[2] += wheel_vels[2] * dt;
			joint_position_[3] += wheel_vels[3] * dt;

		}	
		
	}

	void antobotHardwareInterface::write(ros::Duration elapsed_time)
	{
		/* Writes robot commands from antobot_control to AntoBridge via ROS publishers */

		std::vector<float> motor_commands;
		antobot_msgs::Float32_Array wheel_vels_cmd;

		// If the commands are only for the 4 wheel motor commands
		if (num_joints_ < 5)
		{
			for (int i = 0; i < 4; i++)
			{
				double command = joint_commands_[i];
				motor_commands.push_back((float)command);
			}
		}
		else		// If steering of each wheel is also considered
		{
			antobot_msgs::Float32_Array steer_pos_cmd;
			std::vector<float> steer_commands;
			for (int i = 0; i < num_joints_; i++)
			{
				if (i < 5)
				{
					double sp_cmd = joint_commands_[i];
					steer_commands.push_back((float)joint_commands_[i]);
				}
				else
				{
					double cmd = joint_commands_[i];
					motor_commands.push_back((float)cmd);
				}
			}
			wheel_vel_cmd_pub.publish(steer_pos_cmd);
		}
		
		wheel_vels_cmd.data = motor_commands;
		wheel_vel_cmd_pub.publish(wheel_vels_cmd);
	}

	antobot_msgs::Float32_Array antobotHardwareInterface::filterWheelVels(float wheel_vel_ar[4])
	{
		/*  Gets filtered ultrasonic sensor data for each individual sensor, creates the structure
			for the data to be sent, and returns this to the main USS callback function. */
		//  Inputs: wheel_vel_ar <float[4]> - the most recent USS data pulled in for each of the 8 sensors
		//  Returns: uss_dist_filt_all <antobot_msgs::UInt16_Array> - the filtered data to publish

		antobot_msgs::Float32_Array wheel_vel_filt_all;
		float wheel_vel_filt_i;

		for (int i=0; i<4; i++)
		{
			wheel_vel_filt_i = wheelVelFilt_i(wheel_vel_ar[i], i);
			wheel_vel_filt_all.data.push_back(wheel_vel_filt_i);
		}

		wheel_vel_filt_pub.publish(wheel_vel_filt_all);

		return wheel_vel_filt_all; 
	}

	float antobotHardwareInterface::wheelVelFilt_i(float wheel_vel_i, int i)
	{
		/*  Calculates filtered data of a single wheel by first formatting data to fit into a predetermined window size,
			then carrying out the filter (currently a simple minimum magnitude) */
		//  Inputs: wheel_vel_i <uint16_t> - the newest data received for a particular sensor
		//          i <int> - the element of the corresponding USS sensor

		wheel_vel_windows[i].push_back(wheel_vel_i);

		if (wheel_vel_windows[i].size() > antobotHardwareInterface::wVel_win_size)
		{
			wheel_vel_windows[i] = antobotHardwareInterface::pop_front(wheel_vel_windows[i]);
		}

		std::vector<float> vec = wheel_vel_windows[i];

		float mean_wheel_vel_ii = std::accumulate(vec.begin(), vec.end(), 0.0) / vec.size();
		std::vector<float> vec_abs = vec;
		for (int i = 0; i<vec.size(); i++)
			vec_abs[i] = abs(vec[i]);

		float wheel_vel_filt_ii = *min_element(vec_abs.begin(), vec_abs.end());
		if (mean_wheel_vel_ii < 0)
			wheel_vel_filt_ii *= -1;

		return wheel_vel_filt_ii;
	}
	
	std::vector<float> antobotHardwareInterface::pop_front(std::vector<float> vec)
	{
		/*  Simple function to pop off the first element of a vector. */
		//  Input: vec <std::vector<float>> - a generic float vector of length > 1
		//  Returns: vec <std::vector<float>> - a generic float vector of length > 0
		
		assert(!vec.empty());
		vec.erase(vec.begin());

		return vec;
	}

	void antobotHardwareInterface::wheel_vel_Callback(const antobot_msgs::Float32_Array::ConstPtr& msg)
	{
		/* Callback function for wheel velocities. Passes the information along to class variables */

		
		wheel_vels[0] = msg->data[0];
		wheel_vels[1] = msg->data[1];
		wheel_vels[2] = msg->data[2];
		wheel_vels[3] = msg->data[3];

		if (false)
		{
			wheel_vels_filt = filterWheelVels(wheel_vels);
			wheel_vels[0] = wheel_vels_filt.data[0];
			wheel_vels[1] = wheel_vels_filt.data[1];
			wheel_vels[2] = wheel_vels_filt.data[2];
			wheel_vels[3] = wheel_vels_filt.data[3];
		}
			
	}
	
	void antobotHardwareInterface::steer_pos_Callback(const antobot_msgs::Float32_Array::ConstPtr& msg)
	{
		/* Callback function for wheel velocities. Only applies to more complicated robot designs with steerable wheels
		Passes the information along to class variables. */
		
		//ROS_INFO("Steer pos callback entered!");
		steer_pos[0] = msg->data[0];
		steer_pos[1] = msg->data[1];
		steer_pos[2] = msg->data[2];
		steer_pos[3] = msg->data[3];
	}

}

