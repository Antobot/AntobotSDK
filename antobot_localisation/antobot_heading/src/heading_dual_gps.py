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

#Description: 	The primary purpose of this code is to calculate the orientation quaternion from dual GPS configuration. 
#             	This script subscribes to the back and front RTK gps topics and publishes Imu messages over 
#		ant_rtk_imu/data topic.
#             	  
#Contacts:     mert.turanli@antobot.co.uk

# # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # #


import rospy
import time
import math
import sys
from sensor_msgs.msg import NavSatFix
from sensor_msgs.msg import Imu
from std_msgs.msg import Float32
from tf.transformations import *

sys.path.append('/home/aswathi/catkin_ws/src/AntoMove/am_teleop/keyboard_teleop/src')
#sys.path.append('/root/catkin_ws/src/AntoMove/am_teleop/keyboard_teleop/src')
from geonav_transform import geonav_conversions as gc

# gps covariance threshold for determining the quality of gps signal
gps_covariance_threshold = 0.07 * 0.07

# global variables
utm_front_x = 0
utm_front_y = 0
utm_front_zone = 0
utm_back_x = 0
utm_back_y = 0
utm_back_zone = 0

gps_front_received = 0
gps_back_received = 0

position_covariance_front = []
position_covariance_back = []

# imu
qImu = []
imuFrame = ''
imuReceived = 0


# callback routines
def gpsFrontCallback(msg):
    global utm_front_x, utm_front_y, utm_front_zone
    global gps_front_received
    global position_covariance_front
    # UTMNorthing, UTMEasting, UTMZone
    utm_front_y, utm_front_x, utm_front_zone = gc.LLtoUTM(msg.latitude, msg.longitude)
    position_covariance_front = msg.position_covariance
    
    gps_front_received = 1
    
def gpsBackCallback(msg):
    global utm_back_x, utm_back_y, utm_back_zone
    global gps_back_received
    global position_covariance_back
    utm_back_y, utm_back_x, utm_back_zone = gc.LLtoUTM(msg.latitude, msg.longitude)
    position_covariance_back = msg.position_covariance

    gps_back_received = 1    

def imuCallback(data):
    global qImu
    global imuFrame
    global imuReceived
    
    qImu = data.orientation
    imuFrame = data.header.frame_id
    
    imuReceived = 1

# main 
if __name__ == '__main__':

    try:
        # init node
        rospy.init_node('rtk_yaw', anonymous=True)
                
        # get parameters from rosparam
        if (rospy.has_param('~front_gps_topic')):
            frontTopic = rospy.get_param('~front_gps_topic')
        else:
            frontTopic = '/am_gps_urcu'
        
        if (rospy.has_param('~back_gps_topic')):
            backTopic = rospy.get_param('~back_gps_topic')
        else:
            backTopic = '/am_gps_ext/fix'   
            
        if (rospy.has_param('~imu_topic')):
            imuTopic = rospy.get_param('~imu_topic')
        else:
            imuTopic = 'imu/data'   
        
        
        # print the parameters
        print("front_gps_topic = ", frontTopic)
        print("back_gps_topic = ", backTopic)
        print("imu_topic = ", imuTopic)
        
        
        # subscribe to front and back gps topics
        subGps_front = rospy.Subscriber(frontTopic, NavSatFix, gpsFrontCallback)
        subGps_back = rospy.Subscriber(backTopic, NavSatFix, gpsBackCallback)
        
        # subscribe to imu topic
        subImu = rospy.Subscriber(imuTopic, Imu, imuCallback)
        
        # publish from rtk imu topic
        pubImu = rospy.Publisher('/ant_rtk_imu/data', Imu, queue_size=10)
        pubImuFloat = rospy.Publisher('/ant_rtk_imu/data_float', Float32, queue_size=10)
        
        # loop rate
        rate = rospy.Rate(10)  # 10hz
        
        rospy.loginfo('publishing from topic /ant_rtk_imu/data')
        
        ###
        # initialize variables for averaging
        count = 0
        
        dxSum = 0
        dySum = 0

        # main loop
        while not rospy.is_shutdown():
            
            # if the gps messages are received
            if (gps_front_received == 1 and gps_back_received == 1):
                # if the UTM zones are correct
                if (utm_front_zone == utm_back_zone):
                    # if the covariances are below threshold
                    if (position_covariance_back[0] < gps_covariance_threshold and position_covariance_back[4] < gps_covariance_threshold and position_covariance_front[0] < gps_covariance_threshold and position_covariance_front[4] < gps_covariance_threshold): 
                
                        # calculate gps yaw  
                        dy = utm_front_y - utm_back_y
                        dx = utm_front_x - utm_back_x
                    
                        #print("utm_front_y = ", utm_front_y)
                        #print("utm_back_y = ", utm_back_y)
                        #print("utm_front_x = ", utm_front_x)
                        #print("utm_back_x = ", utm_back_x)
                    
                        # print out the distance between two receivers
                        print("d = ", math.sqrt(dx*dx+dy*dy))
                    
                        # calculate sums for average
                        dxSum = dxSum + dx
                        dySum = dySum + dy
                    
                        count = count + 1
                    
                        # 
                        if (count == 1):
                        
                            dx = dxSum / count
                            dy = dySum / count
                                           
                              
                            ###
                            #print("dx = ", dx)
                            #print("dy = ", dy)

                
                            yaw = math.atan2(dy, dx)
                            q = quaternion_from_euler(yaw, 0, 0, 'rzyx')
                
                            # TODO: switch to imu/odometry if signal is poor
                
                
                            # create imu message
                            imuMsg = Imu()
                            imuMsg.header.stamp = rospy.Time.now()
                            imuMsg.header.frame_id = "gps_frame"
                            imuMsg.orientation.x = q[0]
                            imuMsg.orientation.y = q[1]
                            imuMsg.orientation.z = q[2]
                            imuMsg.orientation.w = q[3] 
                 
         	             # publish over ant_rtk_yaw topic
                            pubImu.publish(imuMsg)
                    
                    
                            pubImuFloat.publish(yaw)
                
                            rospy.loginfo('yaw = %f', yaw)
                        
                            ###
                        
                            dxSum = 0
                            dySum = 0
                            count = 0
                    else:
                        # if gps signal quality is low publish from imu topic
                        rospy.loginfo('gps position quality is low, publishing from %s', imuTopic)  
                        
                        if (imuReceived == 1):
                            # create imu message
                            imuMsg = Imu()
                            imuMsg.header.stamp = rospy.Time.now()
                            imuMsg.header.frame_id = imuFrame
                            imuMsg.orientation.x = qImu.x
                            imuMsg.orientation.y = qImu.y
                            imuMsg.orientation.z = qImu.z
                            imuMsg.orientation.w = qImu.w 
                 
         	             # publish over ant_rtk_yaw topic
                            pubImu.publish(imuMsg)
                            
                            imuReceived = 0
                        else:
                            rospy.loginfo('no messages from imu topic')
                    
                        
                        
                else:
                    rospy.loginfo('utm zone different')
                    
                    
                gps_front_received = 0
                gps_back_received = 0
                
            #else:
            #    rospy.loginfo('no message received')
                
            rate.sleep()

    except rospy.ROSInterruptException:
        pass
        
