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
#               Rewritten
#             	  
#Contacts:     soyoung.kim@antobot.ai

# # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # #
import sys, signal
import rospy
from std_msgs.msg import UInt8
import roslaunch
import rospkg


def signal_handler(signal, frame):
    print("\nheading_launcher killed")
    sys.exit(0)

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
        rospy.init_node('rtk_yaw_launcher', anonymous=True) 
      
        rospack = rospkg.RosPack()

        ekf_package_path = rospack.get_path('am_ekf') + "/launch/ekf.launch"
        #auto_cali_path = rospack.get_path('am_heading') + "/launch/heading_auto_calib.launch"
        auto_cali_path = rospack.get_path('am_heading_cpp') + "/launch/auto_launch.launch"
        
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

                parent = roslaunch.parent.ROSLaunchParent(uuid, roslaunch_file)
                parent.start()
                rospy.loginfo('[New] single gps yaw node started')
                
                
                while (not calib_flag):
                    rospy.loginfo('[New] waiting for calibration to finish ... new')
                    rospy.sleep(1.0)
                    
                
                if (calib_flag):
                    # launch ekf
                    cli_args = [ekf_package_path,'yaw_offset:=0']
                    roslaunch_args = cli_args[1:]
                    roslaunch_file = [(roslaunch.rlutil.resolve_launch_arguments(cli_args)[0], roslaunch_args)]

                    parent = roslaunch.parent.ROSLaunchParent(uuid, roslaunch_file)
                    parent.start()
                    rospy.loginfo('[New] robot_localization started')
                    launched = True

            rate.sleep()

    except rospy.ROSInterruptException:
        pass

