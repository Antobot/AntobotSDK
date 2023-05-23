/*
# Copyright (c) 2023, ANTOBOT LTD.
# All rights reserved.

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

  // Creates a subscriber for wheel velocity feedback from anto_bridge
  ros::Subscriber sub_vWheel = nh.subscribe("/antobridge/wheel_vel", 10, &antobot_hardware_interface::antobotHardwareInterface::wheel_vel_Callback, &antobot1);

  // NOTE: We run the ROS loop in a separate thread as external calls such
  // as service callbacks to load controllers can block the (main) control loop
  ros::MultiThreadedSpinner spinner(2);
  spinner.spin();

  return 0;
}
