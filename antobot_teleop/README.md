# antobot_teleop

Space for all code to teleoperate the robot. Teleoperation can be keyboard teleop or joystick teleop, and the main code simply switches between these two methods.

At the moment, teleoperation is launched along with the rest of the robotic components on bringup. However, if you wish to launch the teleop (with an option to choose the teleop), run the following command:
**rosrun antobot_tel antobot_teleop.py**

