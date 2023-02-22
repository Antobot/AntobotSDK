# antobot_base

Space for all code on the Xavier NX related to controlling the physical movement of the mobile robot base. 

This includes the connection between full robot commands, and what they mean for specific wheels and joints. It does not include
anto_bridge (the communication link between Xavier NX and Aurix), which is included in a separate repository. However, the launchfiles
in this repository will launch anto_bridge for simplicity.

To launch only the controllers of the ant, launch the following:
**roslaunch antobot_base ant_controller.launch**
