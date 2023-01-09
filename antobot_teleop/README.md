# am_teleop

Space for all code to teleoperate the robot. Teleoperation can be keyboard teleop, 'Antobot' app teleop or joystick teleop.

At the moment, teleoperation is launched along with the rest of the robotic components on bringup. However, if you wish to launch the teleop (with an option to choose the teleop), run the following command:
**rosrun am_tel am_teleop.py**

The robot can be teleoperated using either the keyboard, joystick or app.

Keyboard control (needs to be updated):
* Press the '0' key to enter keyboard teleoperation mode.
* Press the 'a' key to start saving waypoints.
* Press the 's' key to stop saving waypoints. If 'a' is pressed again after this, it will continue saving waypoints to the same file.
* Press the 'd' key to stop saving waypoints (if they are currently being saved) and set the most recently recorded waypoints as the waypoints to be followed next when waypoint following mode is entered.
* Press the '3' key to enter autonomous waypoint following mode. If new waypoints haven't been saved prior to this, it will follow the most recently saved waypoints.

Joystick control:
* Clone the [joystick_drivers](http://wiki.ros.org/joystick_drivers) ROS package into /antobot_teleop/am_tel_joy
* Configure your joystick as appropriate, using the topic "/joy"
* Joystick controls image (to be updated and added):

App control:
* Press 'Live view' in Antobot app to enter app teleoperation mode (the switch must be on manual in the top-right corner).
* Press the switch in the top-right corner in the live view to automatic to enter waypoint following mode.
