/*
# Copyright (c) 2022, ANTOBOT LTD.
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

Description:  This code defines the ROS node that is used with the antobot hardware interface, including subscribers for
              calibration and track width. It also links the node to the hardware interface via the included header files.

Contacts: 	daniel.freer@antobot.ai

# # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # #

*/

#include <antobot_base/antobot_control.h>
#include "ros/ros.h"
#include <std_msgs/Int16.h>
#include <std_msgs/Int8.h>

int main(int argc, char** argv)
{
  // Initialises the ROS node and gets the node handle
  ros::init(argc, argv, "antobot_control");
  ros::NodeHandle nh;

  // Defines an antobot_hardware_interface class object using the defined ROS node
  antobot_hardware_interface::antobotHardwareInterface antobot1(nh);

  // Defines subscribers for calibration and track width adjustment, and links them to the specific class instance of antobot_hardware_interface
  // ros::Subscriber sub_calib = nh.subscribe("calibration", 10, &antobot_hardware_interface::antobotHardwareInterface::calibration_Callback, &antobot1);
  // ros::Subscriber sub_tw = nh.subscribe("cmd_TW", 10, &antobot_hardware_interface::antobotHardwareInterface::tw_Callback, &antobot1);
  ros::Subscriber sub_vWheel = nh.subscribe("/antobridge/wheel_vel", 10, &antobot_hardware_interface::antobotHardwareInterface::wheel_vel_Callback, &antobot1);
  
  // NOTE: We run the ROS loop in a separate thread as external calls such
  // as service callbacks to load controllers can block the (main) control loop
  ros::MultiThreadedSpinner spinner(2);
  spinner.spin();

  return 0;
}
