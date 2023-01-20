/*# Copyright (c) 2022, ANTOBOT LTD.
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

# # # Code Description:     Sets the parameters for operation of the small robot.

# Contact: daniel.freer@antobot.ai

# # # #  # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # #*/

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

#include <antobot_base/ant_hardware.h>

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

	class antobotControl: public antobot_hardware_interface::antobotHardware
	{
		public:
			union cmd
			{
				float cmd_float[9];
				uint8_t cmd_uint[36];
			};

			antobotControl(ros::NodeHandle& nh);
			~antobotControl();
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
			void wheel_vel_Callback(const antobot_msgs::Float32_Array::ConstPtr& msg);
			void steer_pos_Callback(const antobot_msgs::Float32_Array::ConstPtr& msg);
			

		protected:
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
