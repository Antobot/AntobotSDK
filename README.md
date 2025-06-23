# AntobotSDK
The Software Development Kit for Antobot's Mobile Robot Platform.

It includes all software needed to run the robot, and to connect to any sensors that will be sold along with the robot. This includes:
- Robot bringup scripts to easily launch the most important functions of the robot (antobot_bringup)
- Robot description package for the different versions of the ant platform, including different sensor configurations (antobot_descriptions)
- Simulation of the robot's operation in a virtual environment in Windsor (antobot_gazebo)
- Example scripts for using the Human-Machine Interface (HMI) on the robot to display important information and enable users to provide simple inputs to the system
- The remaining scripts are in submodules:
  - AntobotDevices: handles all devices compatible with the Ant Platform, including GPS, IMU, LiDAR, and cameras. See the submodule for more information.
  - AntobotPlatform: the core software for the Ant Platform, including scripts specific to the robot, as well as for managing the Universal Robot Control Unit (uRCU). See the submodule for more information.

## Dependencies (depending on package purchased):
- antobridge: Antobot proprietary software for which source code is not provided for security. This piece communicates between the on-board Jetson and other components of the uRCU.
- ROS Noetic (Desktop Full)
- ROS Navigation (ros-noetic-navigation)
- geonav_transform: Please clone/download the scripts from [here](https://github.com/bsb808/geonav_transform) and place them into your catkin_ws/src
- IMU sensor driver software (IMU built into uRCU): https://github.com/dheera/ros-imu-bno055
- ZED Software
  - ZED SDK: https://www.stereolabs.com/developers/release/
  - ZED ROS Wrapper: https://github.com/stereolabs/zed-ros-wrapper
    - Enables use of ZED camera within ROS
- [Jetson Stats (jtop)](https://pypi.org/project/jetson-stats/)
- [pyserial](https://pyserial.readthedocs.io/en/latest/pyserial.html)
  
### Recommended:
- pointcloud_to_laserscan: https://github.com/ros-perception/pointcloud_to_laserscan
  - improves efficiency so ZED camera point cloud can be used for costmap directly
