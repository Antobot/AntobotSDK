#!/usr/bin/env python3
# Copyright (c) 2023, ANTOBOT LTD.
# All rights reserved.

# # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # 
# Description: 	The primary purpose of this code is to launch calibration node (C++ version or Python version) and to launch
#               ekf nodes when the initial calibration is done.
# Contacts:     soyoung.kim@antobot.ai
# # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # #

import subprocess
import rospy
from std_srvs.srv import Trigger, TriggerResponse


def launchEkf(request):
    """
    Launch roslaunch file using subprocess - roslaunch package doesn't work outside of the main thread
    ref: https://answers.ros.org/question/42849/how-to-launch-a-launch-file-from-python-code/
    """    
    package = 'antobot_ekf'
    node_name = 'ekf.launch'
    command = "roslaunch {0} {1}".format(package, node_name)
    p = subprocess.Popen(command, shell=True)

    return TriggerResponse(success=True)



if __name__ == '__main__':

    # init node
    rospy.init_node('antobot_heading_ekf_launcher', anonymous=True) 
    rospy.loginfo('antobot_heading_ekf_launcher started')

    ser_amheading = rospy.Service('launch_ekf', Trigger, launchEkf)

    rospy.spin()



