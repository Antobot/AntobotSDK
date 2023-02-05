/*
# Copyright (c) 2023, ANTOBOT LTD.
# All rights reserved.

# # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # 

Description: launches antobot_heading_node and runs the initial calibration.

Contacts: 	soyoung.kim@antobot.ai

# # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # #
*/
#include <antobot_heading/antobot_heading.h>

bool exit_loop = false;


int main(int argc, char** argv){
  ros::init(argc, argv, "antobot_heading_node", ros::init_options::NoSigintHandler);
  ros::NodeHandle nh;

  antobot_heading::AmHeading auto_calibration_node(nh);
  auto_calibration_node.InitialCalibration();

  ros::spin();

  return(0);
}