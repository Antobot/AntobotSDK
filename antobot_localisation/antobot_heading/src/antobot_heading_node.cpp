/*
# Copyright (c) 2023, ANTOBOT LTD.
# All rights reserved.

# # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # 
# Description: Launches antobot_heading_node and runs the initial calibration.
# Contacts: soyoung.kim@antobot.ai
# # # #  # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # #
*/

#include <antobot_heading/antobot_heading.h>

int main(int argc, char** argv){
  ros::init(argc, argv, "antobot_heading_node", ros::init_options::NoSigintHandler);
  ros::NodeHandle nh;

  antobot_heading::amHeading auto_calibration_node(nh);
  auto_calibration_node.initialCalibration();

  ros::spin();

  return(0);
}