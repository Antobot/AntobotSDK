# AntobotSDK
The Software Development Kit for Antobot's Mobile Robot Platform.

It includes all software needed to run the robot, and to connect to any sensors that will be sold along with the robot. This includes:
- IMU sensor driver software (IMU built into uRCU)
  - Original repository for the IMU chip is here: https://github.com/dheera/ros-imu-bno055
- GPS driver software (chip built into uRCU)
- Robot description and robot control scripts to control the robot using a ROS Twist command
