# antobot_safety

This package contains a variety of code related to the safe operation of our robot. 

This includes basic safety functions for navigation (e.g. stopping if an object is detected to be too close by the ultrasonic sensors), blinking
the lights when appropriate to let users know that a safety function has been triggered, and playing noises to make users aware of the robot's presence. It also includes a multiplexor based on code from https://github.com/yujinrobot/yujin_ocs/tree/devel/yocs_cmd_vel_mux. Their original license has been included in this package.

The safety software is launched directly from am_bringup, as well as in simulation setting launchfiles.
