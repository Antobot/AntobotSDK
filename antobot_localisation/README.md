### am_localisation
As the name would indicate, this folder is in charge of the localisation and mapping of the robotic system.

This includes sensor fusion (using EKF or other means), Hector SLAM or gmapping, and any other similar functions.

In the future, this may also include a management script: sensor_manager, which will ensure good operation of all sensors being used as input, and will call for intervention to improve the sensor operation if necessary.

The following are the launch commands from am_localisation:
**roslaunch am_ekf ekf.launch**

to launch gmapping:
**roslaunch am_map gmapping.launch**
