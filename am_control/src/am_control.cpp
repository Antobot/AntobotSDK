/*
# Copyright (c) 2019, ANTOBOT LTD.
# All rights reserved.

# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
# A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
# OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
# SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
# LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
# DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
# THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
# (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

# # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # 

Description: 	The primary purpose of this code is to serve as a communication channel between the ROS components of
		the Xavier NX and the the Aurix. This channel goes in 2 directions. It reads information from am_robot/cmd_vel 
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

#include <am_control/am_control.h>
#include <joint_limits_interface/joint_limits_interface.h>
#include <joint_limits_interface/joint_limits.h>
#include <joint_limits_interface/joint_limits_urdf.h>
#include <joint_limits_interface/joint_limits_rosparam.h>

#include <serial/serial.h>
#include <std_msgs/Float32.h>
#include <std_msgs/UInt8.h>
//#include <am_bridge/UInt8_Array.h>
//#include <am_bridge/Float32_Array.h>
#include <anto_bridge_msgs/UInt8_Array.h>
#include <anto_bridge_msgs/Float32_Array.h>

using namespace hardware_interface;
using joint_limits_interface::JointLimits;
using joint_limits_interface::SoftJointLimits;
using joint_limits_interface::PositionJointSoftLimitsHandle;
using joint_limits_interface::PositionJointSoftLimitsInterface;
using joint_limits_interface::VelocityJointSoftLimitsHandle;
using joint_limits_interface::VelocityJointSoftLimitsInterface;

//using namespace boost::assign;


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

			//antobot1.setJoint(joint);

		  	// Create joint state interface
			JointStateHandle jointStateHandle(joint_names_[i], &joint_position_[i], &joint_velocity_[i], &joint_effort_[i]);
		  	joint_state_interface_.registerHandle(jointStateHandle);

			if (joint_modes_[i] == 0) {
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
			else {
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
		}

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
		wheel_vel_cmd_pub = nh_.advertise<anto_bridge_msgs::Float32_Array>("/antobridge/wheel_vel_cmd", 1);
		steer_pos_cmd_pub = nh_.advertise<anto_bridge_msgs::Float32_Array>("/antobridge/steer_pos_cmd", 1);
	}

	void antobotHardwareInterface::update(const ros::TimerEvent& e)
	{
		/* The callback function for the ROS timer defined in the constructor. This will occur at a rate of 25Hz.
		The main purpose of this function is to call the read() and write() functions, which read and write from
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

		//ROS_INFO_STREAM(elapsed_time_);

		//read();
		push_motor_info();
		controller_manager_->update(ros::Time::now(), elapsed_time_);
		write(elapsed_time_);

		//ROS_INFO_STREAM(_logInfo); 
	}

	void antobotHardwareInterface::push_motor_info()
	{
		/* Pushes motor information from AntoBridge to the ros controller via the ROS-defined hardware interface
		*/
		// // Inputs:

		//ROS_INFO("Pushing motor info to ros controller!");

		if (num_joints_ < 5)
		{
			//ROS_INFO("Less than 5 joints!");

			joint_velocity_[0] = wheel_vels[0];
			joint_velocity_[1] = wheel_vels[1];
			joint_velocity_[2] = wheel_vels[2];
			joint_velocity_[3] = wheel_vels[3];

			double dt;
			//dt = 1.0/loop_hz_;
			dt = elapsed_time_.toSec();

			joint_position_[0] += wheel_vels[0] * dt;
			joint_position_[1] += wheel_vels[1] * dt;
			joint_position_[2] += wheel_vels[2] * dt;
			joint_position_[3] += wheel_vels[3] * dt;

			//ROS_INFO_STREAM(joint_position_[0]);
		}
		
		// TODO: Should also incorporate ATWAS controller design (num_joints_ > 5)		
		
	}



	uint8_t antobotHardwareInterface::GetByteFromBools(const bool eightBools[8])
	{
		/* Converts an array of 8 booleans into a byte of 8 binary values */
		// // Input: const bool eightBools[8] - array of 8 booleans
		// // Returns: uint8_t ret - byte of 8 binary values
		
		uint8_t ret = 0;
		for (int i=0; i<8; i++) if (eightBools[i]) ret |= (1<<i);
		return ret;
	}

	void antobotHardwareInterface::DecodeByteIntoEightBools(uint8_t theByte, bool eightBools[8])
	{
		/* Converts a byte of 8 binary values into an array of 8 booleans */
		for (int i=0; i<8; i++) eightBools[i] = ((theByte & (1<<i)) != 0);
	}

	void antobotHardwareInterface::write(ros::Duration elapsed_time)
	{
		/* Writes robot commands from am_control to AntoBridge via ROS publishers */

		std::vector<float> motor_commands;
		anto_bridge_msgs::Float32_Array wheel_vels_cmd;

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
			anto_bridge_msgs::Float32_Array steer_pos_cmd;
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
	
	void antobotHardwareInterface::wheel_vel_Callback(const anto_bridge_msgs::Float32_Array::ConstPtr& msg)
	{
		//ROS_INFO("Wheel vel callback entered!");
		wheel_vels[0] = msg->data[0];
		wheel_vels[1] = msg->data[1];
		wheel_vels[2] = msg->data[2];
		wheel_vels[3] = msg->data[3];
	}
	
	void antobotHardwareInterface::steer_pos_Callback(const anto_bridge_msgs::Float32_Array::ConstPtr& msg)
	{
		//ROS_INFO("Steer pos callback entered!");
		steer_pos[0] = msg->data[0];
		steer_pos[1] = msg->data[1];
		steer_pos[2] = msg->data[2];
		steer_pos[3] = msg->data[3];
	}

}

