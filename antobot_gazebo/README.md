# antobot_gazebo

This package is for simulated testing using Gazebo.

To use the simulation requires:
  - Robot description files for each robot, which should be located in the antobot_description package 
  - **World** files detailing suitable testing scenarios, which are located in this package
  - Additional large model files (e.g. terrain/obstacles) which are usually installed into a hidden gazebo folder, located at "~.gazebo".

Launch a simulation of the robot at windsor:
**roslaunch antobot_gazebo windsor_ant_v3.launch **

At present, each simulation has a seperate launch and settings files. This is intended to make launching each simulation easy (with no command line inputs) and eliminate the risk of any changes to one simulation adversely affecting another.
