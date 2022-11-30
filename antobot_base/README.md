# am_control

Space for all code on the Xavier NX related to controlling the physical movement of the mobile robot base. 

This includes the hardware interface with Aurix.

To launch only the controllers of big robot, launch the following:
**roslaunch am_control ATWAS_controller.launch**

To launch only the controllers of small robot, launch the following:
**roslaunch am_control CP1_controller.launch**

**Updates on Realease V1.1 Beta**

/antobot_robot/antobot_velocity_controller/cmd_vel is now replaced by **/am_robot/cmd_vel**

/antobot_robot/antobot_velocity_controller/odom is now replaced by **/am_robot/odom**
