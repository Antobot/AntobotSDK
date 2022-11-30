#!/usr/bin/env python3
# Copyright (c) 2019, ANTOBOT LTD.
# All rights reserved.

# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
# A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
# OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
# SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
# LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
# DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
# THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
# (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

# # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # 

#Description: 	The primary purpose of this code is to launch different imu-gps configurations.
#             	  
#Contacts:     mert.turanli@antobot.co.uk

# # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # #


import rospy
import time
import math
import sys

from sensor_msgs.msg import NavSatFix

from sensor_msgs.msg import Imu
from std_msgs.msg import Float32, UInt8
from tf.transformations import *
from std_srvs.srv import Empty, EmptyResponse
import sys, select, termios, tty
import roslaunch
import rospkg

sys.path.append('/home/aswathi/catkin_ws/src/AntoMove/am_teleop/am_tel_key/src')
#sys.path.append('/root/catkin_ws/src/AntoMove/am_teleop/keyboard_teleop/src')
#sys.path.append('/home/soyoung/catkin_ws/src/AntoMove/am_teleop/am_tel_key/src')
from geonav_transform import geonav_conversions as gc

settings = []

global calib_flag
calib_flag = False

global key
key = 0

# from keyboard_teleop.py
def get_key():
        global settings
        # # # Gets the key which was input via the keyboard and returns it as a character
        # Returns: key - For example: 'c', 'k', or '\x03' (ctrl+c)

        tty.setraw(sys.stdin.fileno())
        rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
        if rlist:
            key = sys.stdin.read(1)
        else:
            key = ''

        settings = termios.tcgetattr(sys.stdin)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        return key

def calib_finished_callback(data):
    global calib_flag
    if data.data == 1:
        calib_flag = True

# Callback function for the calibration mode
def mode_callback(data):
    global key
    
    key = chr(data.data)
    
    print("mode is ", key)

# main 
if __name__ == '__main__':
    # global calib_flag

    try:
        # init node
        rospy.init_node('rtk_yaw_launcher', anonymous=True)
                
      
        rospack = rospkg.RosPack()
        ekf_package_path = rospack.get_path('am_ekf') + "/launch/ekf.launch"
        
        dual_gps_package_path = rospack.get_path('am_heading') + "/launch/heading_dual_gps.launch"
        single_gps_package_path = rospack.get_path('am_heading') + "/launch/heading_imu_calib.launch" # "/launch/heading_single_gps.launch"
        imu_package_path = rospack.get_path('am_heading') + "/launch/heading_imu.launch"
        
        uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
        
        rospy.Subscriber("/imu_calibration_status", UInt8, calib_finished_callback)

        # loop rate
        rate = rospy.Rate(10)  # 10hz
        
        rospy.loginfo('rtk yaw launcher started')
        
        #rospy.loginfo("Select a configuration to launch, q to quit")
        #rospy.loginfo("1) IMU Yaw + RTK GPS")
        #rospy.loginfo("2) Dual GPS RTK Yaw + RTK GPS")
        #rospy.loginfo("3) Single GPS RTK Yaw + RTK GPS")
        
        key = '3'
        
        mode = rospy.Subscriber("/rtk_yaw_mode", UInt8, mode_callback)
        

        # main loop
        while not rospy.is_shutdown():
            
            # key = input()
            
            if (key == '1'):   # option 1: only imu
                
                # launch ekf
                cli_args = [ekf_package_path,'']
                roslaunch_args = cli_args[1:]
                roslaunch_file = [(roslaunch.rlutil.resolve_launch_arguments(cli_args)[0], roslaunch_args)]

                parent = roslaunch.parent.ROSLaunchParent(uuid, roslaunch_file)

                parent.start()
                
                rospy.loginfo('robot_localization started')
                
                # launch imu node
                cli_args = [imu_package_path,'']
                roslaunch_args = cli_args[1:]
                roslaunch_file = [(roslaunch.rlutil.resolve_launch_arguments(cli_args)[0], roslaunch_args)]

                parent = roslaunch.parent.ROSLaunchParent(uuid, roslaunch_file)

                parent.start()
                
                rospy.loginfo('imu node started')
                key = None

            elif (key == '2'):   # option 2: dual gps
                
                # launch ekf
                cli_args = [ekf_package_path,'yaw_offset:=0']
                roslaunch_args = cli_args[1:]
                roslaunch_file = [(roslaunch.rlutil.resolve_launch_arguments(cli_args)[0], roslaunch_args)]

                parent = roslaunch.parent.ROSLaunchParent(uuid, roslaunch_file)

                parent.start()
                
                rospy.loginfo('robot_localization started')
                
                # launch dual gps node
                cli_args = [dual_gps_package_path,'']
                roslaunch_args = cli_args[1:]
                roslaunch_file = [(roslaunch.rlutil.resolve_launch_arguments(cli_args)[0], roslaunch_args)]

                parent = roslaunch.parent.ROSLaunchParent(uuid, roslaunch_file)

                parent.start()
                
                rospy.loginfo('dual gps yaw node started')
                
                key = None

            elif (key == '3'):   # option 3: imu with gps correction
                
                
                
                # launch imu+gps calibration node
                cli_args = [single_gps_package_path,'']
                roslaunch_args = cli_args[1:]
                roslaunch_file = [(roslaunch.rlutil.resolve_launch_arguments(cli_args)[0], roslaunch_args)]

                parent = roslaunch.parent.ROSLaunchParent(uuid, roslaunch_file)

                parent.start()
                
                rospy.loginfo('single gps yaw node started')
                
                while (calib_flag == False):
                    rospy.loginfo('waiting for calibration to finish ...')
                    rospy.sleep(1.0)
                    
                
                if (calib_flag == True):
                    # launch ekf
                    cli_args = [ekf_package_path,'yaw_offset:=0']
                    roslaunch_args = cli_args[1:]
                    roslaunch_file = [(roslaunch.rlutil.resolve_launch_arguments(cli_args)[0], roslaunch_args)]

                    parent = roslaunch.parent.ROSLaunchParent(uuid, roslaunch_file)

                    parent.start()
                
                    rospy.loginfo('robot_localization started')
                
                key = None
                
            #if calib_flag:
                #break

                # break
            elif (key == 'q'):
                break
            elif (key == '\x03'):
                break
                 
            rate.sleep()
            # break

    except rospy.ROSInterruptException:
        pass

