
antobot_heading_cpp
====================

Includes antobot_heading node that takes in imu raw data and gps data and outputs corrected imu data (initial calibration). This corrected imu data is then used in EKF nodes. After the first calibration, it will run auto calibration logic every 30 seconds. 

###### launch files:
 * auto_calibration_cpp.launch : launches antobot_heading node written in C++. C++ version has significantly lower CPU load.
 * auto_calibration_python.launch : launches antobot_heading node written in pytho. 

###### How to:
 * launch heading_launcher.py. This node launches auto_calibration_cpp.launch and waits for the inital calibration to be finished. After the initial calibration, it launches EKF node.
 '''
 <node if="true" pkg="antobot_heading_cpp" type="heading_launcher.py" name="calibration_launcher" output="screen" />
 '''

###### Parameters:
With the current setting, robot will only calibrate when it moved at least 1m at linear velocity larger than 0.01 m/s and angular velocity smaller than 0.1 rad/s. It will check if the EKF odometry orientation and GPS odometry orientation has bigger than 3 degrees difference and if it does, it will run the auto calibration (calculate the new corrected imu).

For C++ version, in src/antobot_heading.cpp change below parameters
'''
calib_distance = 1.0; // calibration distance 
angular_zero_tol = 0.1; // Angular velocity (rad/sec)
lin_tol = 0.01; // linear velocity x (m/s)
calib_deg = 3; // deg
'''

For python version, in scripts/heading_auto_calib.py change below parameters 
'''
self.calib_distance = 1.0 # calibration distance 
self.angular_zero_tol = 0.1 # Angular velocity (rad/sec)
self.lin_tol = 0.01 # linear velocity x (m/s)
self.calib_deg = 3
'''



