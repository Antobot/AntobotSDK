# antobot_datamanager

This package is used to monitor and manage data from the ant platform.

This includes recording video data from ZED cameras, managing which cameras are on or off, supervising a wide range of data related to the health and safety of the robot for operational purposes, and monitoring the joystick for a soft shutdown service.


# Operating Commands
## Recording Video
<pre>
rosservice call /antobot_datamanager/camera [camera_num] [command]

camera_num: 1 - front, 2 - back, 3 - left, 4 - right, 5 - both left and right
command: 0 - close cameras, 1 - start ROS wrapper, 2 - open cameras, 3 - start video recording, 4 - stop video recording
</pre>

For example, if you want to open the left camera, the command should be: 
<pre>
rosservice call /antobot_datamanager/camera 3 2
</pre>

Please note that the IP address for each specific camera should also be configured in config/cameras.yaml to ensure the correct camera is opened every time.

## Anto_Supervisor

anto_supervisor.py publishes a wealth of information over ROS to give you an easy understanding of how the system is operating. This includes: 
- /as/cpu_temp: CPU temperature <float32>
- /as/cpu_load: CPU load (percentage) <uint8>
- /as/storage: Remaining storage space (gigabytes) <float64>
- /as/soc:  State of charge (percentage remaining) <uint8>
- /as/uBat: Battery level (volts)   <float32>
- /imu/slope_direction: Tells the direction of the slope    <uint8>
- /imu/roll_direction: Tells the direction of robot roll    <uint8>


