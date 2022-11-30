# am_heading

The code in this package is used to determine the heading of the robot. There are currently two methods for this:
1. IMU with GPS correction - **heading_imu_calib**
2. Dual GPS with no IMU - **heading_dual_gps**

**heading_launcher** is used to select the method and launch the appropriate code for the method selected. Currently, only method 1 is used, so that method is launched automatically without input by the user.

**heading_sim.py** is deprecated
