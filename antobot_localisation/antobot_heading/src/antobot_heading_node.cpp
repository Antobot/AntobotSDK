#include <antobot_heading/antobot_heading.h>

int main(int argc, char** argv){
  ros::init(argc, argv, "antobot_heading_node");
  ros::NodeHandle nh;

  antobot_heading::AmHeading auto_calibration_node(nh);
  auto_calibration_node.InitialCalibration();

  ros::spin();
  return(0);
}