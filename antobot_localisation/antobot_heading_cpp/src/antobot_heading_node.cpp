#include <antobot_heading_cpp/antobot_heading.h>

int main(int argc, char** argv){
  ros::init(argc, argv, "am_heading_node");
  ros::NodeHandle nh;

  am_heading::AmHeading auto_calibration_node(nh);
  auto_calibration_node.InitialCalibration();

  ros::spin();
  return(0);
}