/*
# Copyright (c) 2023, ANTOBOT LTD.
# All rights reserved.

# # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # 

Description: 	The primary purpose of antobot_base is to serve as a communication channel between the ROS components of
		the Xavier NX and the the Aurix. This channel goes in 2 directions. It reads information from antobot_robot/cmd_vel 
		and other relevant ROS topics, converts it to the format defined in AntoBridge, and sends the
		data to the AntoBridge ROS node via publishers. Likewise, it reads data from AntoBridge using subscribers, and
		passes this data along to the robot to calculate wheel odometry and other relevant information.

		This is the header file. See antobot_base.cpp for source code and more information.

Contacts: 	daniel.freer@antobot.ai

# # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # #
*/


#ifndef ROS_CONTROL__ANTOBOT_HARDWARE_INTERFACE_H
#define ROS_CONTROL__ANTOBOT_HARDWARE_INTERFACE_H

#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/robot_hw.h>
#include <joint_limits_interface/joint_limits_interface.h>
#include <joint_limits_interface/joint_limits.h>
#include <joint_limits_interface/joint_limits_urdf.h>
#include <joint_limits_interface/joint_limits_rosparam.h>
#include <controller_manager/controller_manager.h>
#include <boost/scoped_ptr.hpp>
#include <ros/ros.h>

#include <antobot_base/antobot_hardware.h>

#include <std_msgs/Float32.h>
#include <std_msgs/Int16.h>
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

	class antobotHardwareInterface: public antobot_hardware_interface::antobotHardware
	{
		public:

			// Initialisation functions
			antobotHardwareInterface(ros::NodeHandle& nh);
			~antobotHardwareInterface();
			void init();
			void create_position_joint(int i, JointStateHandle jointStateHandle);
			void create_velocity_joint(int i, JointStateHandle jointStateHandle);
			void init_joint_variables();
			void init_publishers();

			// Update functions
			void update(const ros::TimerEvent& e);
			void push_motor_info();
			void write(ros::Duration elapsed_time);

			// Wheel filter functions
			antobot_msgs::Float32_Array filterWheelVels(float wheel_vel_ar[4]);
			float wheelVelFilt_i(float wheel_vel_i, int i);
			float get_wVel_vec_min(std::vector<float> vec);
			std::vector<float> pop_front(std::vector<float> vec);

			// Callback functions (ROS topics)
			void tw_Callback(const std_msgs::Float32::ConstPtr& msg);
			void wheel_vel_Callback(const antobot_msgs::Float32_Array::ConstPtr& msg);
			void steer_pos_Callback(const antobot_msgs::Float32_Array::ConstPtr& msg);
			

		protected:
			// ROS node definitions
			ros::NodeHandle nh_;
			ros::Timer non_realtime_loop_;
			ros::Duration control_period_;
			ros::Duration elapsed_time_;
			ros::Time calib_time_;

			// Publishers
			ros::Publisher wheel_vel_cmd_pub;
			ros::Publisher steer_pos_cmd_pub;
			ros::Publisher wheel_vel_filt_pub;

			// Joint interfaces
			PositionJointInterface positionJointInterface;
			PositionJointSoftLimitsInterface positionJointSoftLimitsInterface;
			VelocityJointInterface velocityJointInterface;
			VelocityJointSoftLimitsInterface velocityJointSoftLimitsInterface;

			// Variables needed for the controller manager
			double loop_hz_;
			boost::shared_ptr<controller_manager::ControllerManager> controller_manager_;
			double p_error_, v_error_, e_error_;
			std::string _logInfo;
			
			// Variables for dealing with wheel joints and their data
			float wheel_vels[4];
			float steer_pos[4];
			int wVel_win_size = 15;
			std::vector<std::vector<float>> wheel_vel_windows;
			antobot_msgs::Float32_Array wheel_vels_filt;

	};

}

#endif
