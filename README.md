# AntobotSDK
The Software Development Kit for Antobot's Mobile Robot Platform.

It includes all software needed to run the robot, and to connect to any sensors that will be sold along with the robot. This includes:
- Robot bringup, description and control scripts to control the robot using a ROS Twist command
- Robot localisation scripts, including GPS driver software, EKF, and GPS-based heading calibration software
- Basic safety software which is able to stop the robot if the ultrasonic sensors
- Management software for cameras and video recording, data supervision, and joystick-based robot shutdown
- Software to teleoperate the robot using the keyboard or a joystick
- Simulation of the robot's operation in a virtual environment (Windsor)

## Dependencies:
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
