#! /usr/bin/env python3

# Copyright (c) 2022, ANTOBOT LTD.
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

# # # Code Description:     Impersonates a ZED camera for camera manager

# Contacts: william.eaton@antobot.ai
#
# # # #  # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # #

import sys
import rospy

import roslaunch # Using this until we develop our own camera manager solution

from std_msgs.msg import Bool, Int16
from sensor_msgs.msg import Temperature
from anto_msgs.srv import camManager, camManagerResponse
from AntoVision.msg import cam_info
from sensor_msgs.msg import LaserScan
from diagnostic_msgs.msg import DiagnosticStatus



######################################################################################################
## Main
######################################################################################################

def main(args):
    
    
    rospy.init_node('fakeZed', anonymous=False)

    camera_name = rospy.get_param("/fake_camera_name")

    # Create a publisher
    diag_pub = rospy.Publisher("/"+camera_name+"_node/diagnostics", DiagnosticStatus, queue_size=1)

    rate = rospy.Rate(10) # 10hz

    # Due to rospy only allowing nodes to be called from within the main thread, we need to move them into here
    while not rospy.is_shutdown():

        emptyData=DiagnosticStatus()
        diag_pub.publish(emptyData)
        rate.sleep()

if __name__ == '__main__':
    main(sys.argv)

