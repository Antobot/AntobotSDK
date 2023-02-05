#!/usr/bin/env python3
# Copyright (c) 2023, ANTOBOT LTD.
# All rights reserved.

# # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # 
#
# Description: 	The primary purpose of this code is to launch calibration node (C++ version or Python version) and to launch
#               ekf nodes when the initial calibration is done.
#
# Contacts:     soyoung.kim@antobot.ai
#
# # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # #
import sys, signal
import rospy
from std_msgs.msg import UInt8
import roslaunch
import rospkg


heading_launch = None
ekf_launch = None
#exit_node = False
class RoslaunchWrapperObject(roslaunch.parent.ROSLaunchParent):

    def start(self):
        super(RoslaunchWrapperObject, self).start()
        print(self.server)

    def stop(self):
        print("Stopping...")
        print(self.server)
        server = self.server.server if self.server is not None and self.server.server is not None else None
        super(RoslaunchWrapperObject, self).shutdown()
        if server:
           server.shutdown()


def signal_handler(signal, frame):
    print("\nantobot_heading_launcher killed")
    if (heading_launch is not None):
        heading_launch.stop()
    if (ekf_launch is not None):
        ekf_launch.stop()

signal.signal(signal.SIGINT, signal_handler)

calib_flag = False

def calib_finished_callback(data):
    global calib_flag
    if data.data == 1:
        calib_flag = True



# main 
if __name__ == '__main__':
    try:
        # init node
        rospy.init_node('calibration_launcher', anonymous=True) 
      
        rospack = rospkg.RosPack()

        ekf_package_path = rospack.get_path('antobot_ekf') + "/launch/ekf.launch"

        # Auto calibration python version uses 10% of CPU, cpp version uses 0.4%
        # Uncomment the version you want to use
        #auto_cali_path = rospack.get_path('antobot_heading') + "/launch/auto_calibration_python.launch"
        auto_cali_path = rospack.get_path('antobot_heading') + "/launch/auto_calibration_cpp.launch"

        rospy.loginfo('launching %s',auto_cali_path)
        
        uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
        
        # To check if the initial calibration is finished
        rospy.Subscriber("/imu_calibration_status", UInt8, calib_finished_callback)

        # loop rate
        rate = rospy.Rate(1)  # 1hz
        
        rospy.loginfo('New heading launcher started')

        launched = False

        # main loop
        while not rospy.is_shutdown():
            if (not launched):

                # launch imu+gps calibration node
                cli_args = [auto_cali_path,'']
                roslaunch_args = cli_args[1:]
                roslaunch_file = [(roslaunch.rlutil.resolve_launch_arguments(cli_args)[0], roslaunch_args)]

                #parent = roslaunch.parent.ROSLaunchParent(uuid, roslaunch_file)
                heading_launch = RoslaunchWrapperObject(run_id = uuid, roslaunch_files = roslaunch_file)
                heading_launch.start()
                rospy.loginfo('[New] single gps yaw node started')
                
                
                while (not calib_flag and not rospy.is_shutdown()):
                    rospy.loginfo('[New] waiting for calibration to finish ... new')
                    rospy.sleep(1.0)
                    
                
                if (calib_flag):
                    # launch ekf
                    cli_args = [ekf_package_path,'yaw_offset:=0']
                    roslaunch_args = cli_args[1:]
                    roslaunch_file = [(roslaunch.rlutil.resolve_launch_arguments(cli_args)[0], roslaunch_args)]

                    #parent = roslaunch.parent.ROSLaunchParent(uuid, roslaunch_file)
                    ekf_launch = RoslaunchWrapperObject(run_id = uuid, roslaunch_files = roslaunch_file)
                    ekf_launch.start()
                    rospy.loginfo('[New] robot_localization started')
                    launched = True

            rate.sleep()

    except rospy.ROSInterruptException:
        pass

