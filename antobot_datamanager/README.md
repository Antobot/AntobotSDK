# antobot_datamanager

This package is used to monitor and manage data from the ant platform.

This includes recording video data from ZED cameras, managing which cameras are on or off, supervising a wide range of data related to the health and safety of the robot for operational purposes, and monitoring the joystick for a soft shutdown service.


# Operating Commands
## Recording Video
<pre>
rosservice call /anto_manager/camera [camera_num] [command]

camera_num: 3 - left, 4 - right, 5 - both
command: 0 - close cameras, 2 - open cameras, 3 - start video recording, 4 - stop video recording
</pre>

For example, if you want to open the left camera, the command should be: 
<pre>
rosservice call /anto_manager/camera 3 2
</pre>
