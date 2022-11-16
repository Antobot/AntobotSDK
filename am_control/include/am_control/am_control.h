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

//#include <am_driver/robot.h>
//#include <am_driver/joint.h>
//#include <antobot_hardware_interface/antobot_hardware.h>
#include <am_control/am_hardware.h>
//#include <am_bridge/am_bridge.h>

#include <serial/serial.h>

#include <std_msgs/Float32.h>
#include <std_msgs/Int16.h>
//#include <am_bridge/Float32_Array.h>
#include <anto_bridge_msgs/Float32_Array.h>

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
			union cmd
			{
				float cmd_float[9];
				uint8_t cmd_uint[36];
			};

			antobotHardwareInterface(ros::NodeHandle& nh);
			~antobotHardwareInterface();
			void init();
			void init_publishers();
			void update(const ros::TimerEvent& e);
			void read();
			int check_sum(uint8_t bufferin[]);
			void push_to_ros(uint8_t bufferin[]);
			void push_motor_info();
			
			uint8_t GetByteFromBools(const bool eightBools[]);
			void DecodeByteIntoEightBools(uint8_t theByte, bool eightBools[]);
			void write(ros::Duration elapsed_time);
			void motor_cmd(cmd *cmd_m);
			void tw_Callback(const std_msgs::Float32::ConstPtr& msg);
			void wheel_vel_Callback(const anto_bridge_msgs::Float32_Array::ConstPtr& msg);
			void steer_pos_Callback(const anto_bridge_msgs::Float32_Array::ConstPtr& msg);
			

		protected:
			//antobot_driver::Robot antobot1;
			ros::NodeHandle nh_;
			ros::Timer non_realtime_loop_;
			ros::Duration control_period_;
			ros::Duration elapsed_time_;
			ros::Time calib_time_;
			ros::Publisher wheel_vel_cmd_pub;
			ros::Publisher steer_pos_cmd_pub;

			PositionJointInterface positionJointInterface;
			PositionJointSoftLimitsInterface positionJointSoftLimitsInterface;
			VelocityJointInterface velocityJointInterface;
			VelocityJointSoftLimitsInterface velocityJointSoftLimitsInterface;

			double loop_hz_;
			boost::shared_ptr<controller_manager::ControllerManager> controller_manager_;
			double p_error_, v_error_, e_error_;
			std::string _logInfo;
			
			float wheel_vels[4];
			float steer_pos[4];

	};

}

#endif
