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

#Description: 	The primary purpose of this code is to calculate yaw offset from the robot motion with single GPS. 
#             	This script subscribes to the IMU and RTK GPS topics and publishes Imu messages over 
#		imu/data_corrected topic after calibration is successful.
#             	  
#Contacts:     mert.turanli@antobot.co.uk
#              dylan.smith@antobot.ai

# # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # #

import rospy
import time
import math
import sys

from sensor_msgs.msg import NavSatFix, Imu, Range, LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32, UInt8

#importing custom messages
from am_msgs.msg import navigatorWGSpath as navGPSpath

# from tf.transformations import *
import tf
from geonav_transform import geonav_conversions as gc


## configuration parameters
calib_distance = 2.0# calibration distance for gps and obstacle detection (meters)
calib_distance_tol = 1.0 # calibration distance tolerance for gps and and obstacle detection (meters)
turning_velocity = 1.0 # turning velocity (calib.) (rad/s)
linear_velocity = 0.25 # linear velocity (calib.) (m/s)
robot_radius = 0.5 # robot radius (meters)
laser_angle_tol = 0.4 # laser scan angular tolerance for obstacle detection (rads)
odom_zero_tol = 0.1 # was 0.2 odometry tolerance for checking zero angular velocity (rad/sec)
lin_tol = 0.09 # TODO replace value with varaible name in the script
time_odom_offset = 5
time_calib_offset = 12

# global variables
gps_start = []
gps_end = []
gps_received = 0

utm_y = 0
utm_x = 0
utm_zone = 0

gps_state = 0
gps_frame = ''

# imu
q_imu = []
imu_frame = ''
imu_received = 0
imu_ang_vel_z = 0

imu_offset = 0

# odometry
odometry_v = 0
odometry_w = 0
odom_heading = 0
odometry_received = False
odom_v =0

# uss
uss_range = 0
uss_received = 0

# laser
laser_distance = 0
laser_received = 0
laser_orientation = 0

###
# callback functions

def gpsCallback(msg):
    global utm_x, utm_y, utm_zone
    global gps_received
    global gps_frame

    utm_y, utm_x, utm_zone = gc.LLtoUTM(msg.latitude, msg.longitude)   # convert from gps to utm coordinates
    gps_frame = msg.header.frame_id
        
    gps_received = 1   

def imuCallback(data):
    global q_imu
    global imu_frame
    global imu_received
    global imu_ang_vel_z
    
    # store imu data
    q_imu = data.orientation
    imu_frame = data.header.frame_id
    imu_ang_vel_z = data.angular_velocity.z
    
    imu_received = 1

def odometryCallback(data):
    global odometry_received, odometry_v, odometry_w , odom_heading, q_odom
    
    # store odometry data
    odometry_v = data.twist.twist.linear.x
    odometry_w = data.twist.twist.angular.z
    odom_heading = data.pose.pose.orientation.z
    q_odom = data.pose.pose.orientation

    odometry_received = True
    
def odomCallback(data):
    global odom_v
       
    # store odometry data
    odom_v = data.twist.twist.linear.x
    
    
def ussCallback(data):
    global uss_range, uss_received

    # store uss data
    if (data.range > data.min_range and data.range < data.max_range):
        uss_range = data.range
    else:
        uss_range = -1

    uss_received = 1

def navpath_Callback(data):
    
    #global nav_yaw
    target_method = data.target_method[0]
    
    #print("Target method is:",target_method)
    if target_method == -1:
        utm_start_y, utm_start_x, utm_zone_start = gc.LLtoUTM(data.lat[1], data.lon[1])
        utm_end_y, utm_end_x, utm_zone_end = gc.LLtoUTM(data.lat[2], data.lon[2])
        utm_dy = utm_end_y - utm_start_y
        utm_dx = utm_end_x - utm_start_x

        nav_yaw = math.atan2(utm_dy, utm_dx)
        #print("Nav yaw: ",nav_yaw)
    pass

def laserCallback(data):
    global laser_distance, laser_received
    global laser_angle_tol
    global laser_orientation
    
    # find the minimum distance to obstacle by using laser range measurements
    i = 0
    laser_distance = 0
    angle = data.angle_min  # start with minimum angle
    first = True
    
    # increment angle with angle_increment until angle_max
    while (angle <= data.angle_max):  
    
        angleNew = (angle + laser_orientation) % (2.0 * math.pi)
        if (angleNew < -math.pi):
            angleNew = angleNew + 2.0*math.pi
        elif (angleNew > math.pi):
            angleNew = angleNew - 2.0*math.pi
      
        if (angleNew >= -laser_angle_tol and angleNew <= laser_angle_tol): 
            # check minimum distance among the distance measurements
            if (first == True):
                laser_distance = data.ranges[i]
                
                first = False
            else:
                laser_distance = min(laser_distance, data.ranges[i])  # take minimum distance
        
        angle = angle + data.angle_increment
        i = i + 1
    
    
    
    laser_received = 1

def rtkstatusCallback(data):
    global rtk_status
    rtk_status = data.data
    #print("RTK status",rtk_status)

    
def checkDistance():
    global laser_distance, calib_distance, calib_distance_tol
    global robot_radius
    
    # check the minimum laser distance if it is above threshold (which means the front of the robot is clear from obstacles)
    #print("laser_distance = ", laser_distance)
    
    if (math.isinf(laser_distance) or laser_distance > calib_distance + robot_radius + calib_distance_tol):
        return True
    else:
        return False
        


if __name__ == '__main__':

    try:
        # init node
        rospy.init_node('am_heading_imu_calib', anonymous=True)
        
        # get parameters from rosparam
        # gps topic
        if (rospy.has_param('~gps_topic')):
            gpsTopic = rospy.get_param('~gps_topic')
        else:
            gpsTopic = 'am_gps_urcu'
        
        # imu topic
        if (rospy.has_param('~imu_topic')):
            imuTopic = rospy.get_param('~imu_topic')
        else:
            imuTopic = 'imu/data'   
        
        # wheel odometry topic
        if (rospy.has_param('~odometry_topic')):
            odometryTopic = rospy.get_param('~odometry_topic')
            
        else:
            odometryTopic = 'antobridge/wheel_vel' 
                
           
        # uss topic 
        if (rospy.has_param('~uss_topic')):
            ussTopic = rospy.get_param('~uss_topic')
        else:
            ussTopic = 'antobridge/uss_dist'   
            
        # laser scan topic
        if (rospy.has_param('~laser_topic')):
            laserTopic = rospy.get_param('~laser_topic')
        else:
            laserTopic = 'scan'   

        navGPS_topic = '/am/nav/WGS_path'
        rtk_statusTopic = '/am_gps_urcu_status'
            
        # obtain robot radius from move base if it is available
        if (rospy.has_param('/move_base/local_costmap/robot_radius')):
            robot_radius = rospy.get_param('/move_base/local_costmap/robot_radius')
            
            
        # obtain laser frame
        
        listener = tf.TransformListener()
        try:
                listener.waitForTransform("base_link", "laser", rospy.Time(), rospy.Duration(30.0))
        except (tf.Exception) as e:
                print("waitForTransform exception: ", e)
                
        try:
                (trans,rot) = listener.lookupTransform('base_link', 'laser', rospy.Time(0))
                print("trans:", trans)
                print("rot:", rot)
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
                print("Transform exception: ", e)

        R = tf.transformations.quaternion_matrix(rot)
        euler = tf.transformations.euler_from_matrix(R, 'rzyx')
        # we assume that the orientation of the laser is only about z axis
        laser_orientation = euler[0]
        rospy.loginfo("laser orientation (z) = %f", laser_orientation)
        
        
        # subscribe to gps topic
        subGps = rospy.Subscriber(gpsTopic, NavSatFix, gpsCallback)
        
        # subscribe to imu topic
        subImu = rospy.Subscriber(imuTopic, Imu, imuCallback)
        
        # subscribe to odometry topic
        subOdometry = rospy.Subscriber(odometryTopic, Odometry, odometryCallback)
        
        # subscribe to uss range topic
        subUSS = rospy.Subscriber(ussTopic, Range, ussCallback)
        
        # subscribe to laser topic
        #subLaser = rospy.Subscriber(laserTopic, LaserScan, laserCallback)

        # subscribe to nav WGS_path topic
        #subnavPath = rospy.Subscriber(navGPS_topic, navGPSpath, navpath_Callback )

        #subscribe to gps_status
        subRtkStatus = rospy.Subscriber(rtk_statusTopic, UInt8, rtkstatusCallback)

        #subscribe to cmd_vel
        subodom_v = rospy.Subscriber('/am_robot/odom',Odometry, odomCallback)
        
        
        # publish from imu topic
        pubImu = rospy.Publisher('/imu/data_corrected', Imu, queue_size=10)
        pubImuFloat = rospy.Publisher('/imu/data_corrected_float', Float32, queue_size=10)
        pubImuOffset = rospy.Publisher('/imu/data_offset', Float32, queue_size=10)
        #pubslopeDirection = rospy.Publisher("/imu/slope_direction", UInt8, queue_size=1) #topic to publish the slope direction
        pubreCalib = rospy.Publisher("/recalib_status",UInt8,queue_size=1)


        # publish calibration flag
        pubCalib = rospy.Publisher('/imu_calibration_status', UInt8, queue_size=1) # 0: calibration not required, 1: initial calibration, 2:auto calibration
        
        # publish velocity commands
        #pubCmdVel = rospy.Publisher('/antobot_robot/antobot_velocity_controller/cmd_vel', Twist, queue_size=10)
        pubCmdVel = rospy.Publisher('/am_robot/cmd_vel', Twist, queue_size=10)
    
        # publish gps location of map origin
        pubGps = rospy.Publisher('/gps_map_origin', NavSatFix, queue_size=1)   
    
        # loop rate
        rate = rospy.Rate(10)  # 10hz
        
        # main state for startup calibration
        state = 0
        
        # previous calibration time (last timestamp at which the calibration is performed)
        time_1_calib = rospy.get_time()
        # previous odometry time for checking if the robot is driving straight
        time_1_odom = rospy.get_time()
        
        driving_straight = False
        straight_calibration_state = 0
        odometry_received = True    #only for simulation
        direction = 0 #default value
        prev_direction = -2 # -1: backward, 1: forward, 0: default value of direction.
    
        while not rospy.is_shutdown():
        
            if (state == 0):
                # if the topics are received proceed to next state
                if (gps_received == 1 and imu_received == 1 and laser_received == 1): # and uss_received == 1
                    state = 1
                    print("Auto calib in state 1")
                # manually drive forward if no lidar data is received
                elif (gps_received == 1 and imu_received == 1 and laser_received == 0):
                #if (gps_received == 1 and imu_received == 1 and laser_received == 0):
                    state = 2
                    print("Manual calibration")
                #else:
                    # wait for topics
                    #rospy.loginfo("waiting for topics to come up ...")
        
            elif (state == 1):
                # ready to drive
                
                # check distance for obstacles
                if (checkDistance() == True): # and (uss_range != -1 and uss_range > calib_distance + robot_radius + calib_distance_tol)
                    # if there are no obstacles, stop and switch to the next state
                
                    state = 2
                    gps_state = 0
                    
                    
                    rospy.loginfo("no obstacles in front of the robot, starting calibration (distance = %f)  ...", laser_distance)
                    
                    # stop
                    twist = Twist()
                    twist.linear.x = 0
                    twist.linear.y = 0
                    twist.linear.z = 0
                    twist.angular.x = 0
                    twist.angular.y = 0
                    twist.angular.z = 0
                    
                    pubCmdVel.publish(twist)
                else:
                    # if there are obstacles in front of the robot, start turning to find an obstacle free region
                    rospy.loginfo("obstacles in front of the robot, stopping ...")
                    
                    # turn
                    twist = Twist()
                    twist.linear.x = 0
                    twist.linear.y = 0
                    twist.linear.z = 0
                    twist.angular.x = 0
                    twist.angular.y = 0
                    twist.angular.z = turning_velocity
                    #twist.angular.z = 0
                    
                    pubCmdVel.publish(twist)
                    
            elif (state == 2):
                # calibration state
                # the robot will drive 2 meters forward
                
                
                if (gps_state == 0):
                    # save first calibration point
                    rospy.loginfo('calibration started')
            
                    gps_start = [utm_x, utm_y, utm_zone]
            
                    gps_state = 1
                elif (gps_state == 1):
                    # check calibration distance if satisfied
                    gps_end = [utm_x, utm_y, utm_zone]
        
                    dx = gps_end[0] - gps_start[0]
                    dy = gps_end[1] - gps_start[1]
                    d = math.sqrt(dx*dx + dy*dy)   # distance between start and end points of calibration
        
                    if (d >= calib_distance):
                        # successful calibration
                        rospy.loginfo('calibration distance satisfied')
                        
                        gps_state = 2
           
                elif (gps_state == 2):
                    # calculate yaw offset by using first and last gps measurements
                    if odom_v >lin_tol: # was 0.01
                        dy = gps_end[1] - gps_start[1]
                        #print("gps_end, dy", gps_end[1])
                        dx = gps_end[0] - gps_start[0]
                    else:
                        dy = gps_start[1] - gps_end[1] 
                        #print("gps_end, dy", gps_end[1])
                        dx = gps_start[0] - gps_end[0] 
                 
            
                    gps_yaw = math.atan2(dy, dx)   # calculate yaw from gps start and end points
            
                    imu_angles = tf.transformations.euler_from_quaternion([q_imu.x, q_imu.y, q_imu.z, q_imu.w], 'rzyx')   # convert imu orientation from quaternion to euler
                
                    # imu_yaw + imu_offset = gps_yaw
                    imu_offset = gps_yaw - imu_angles[0]   # difference between orientations from imu and gps
            
                    rospy.loginfo('calibration successful (offset = %f degs)', imu_offset / 3.1415 * 180.0)
                    pubCalib.publish(1)

                    # save last calibration time
                    time_1_calib = rospy.get_time()
                    
                    # convert end UTM to GPS coordinates
                    origin_lat, origin_long = gc.UTMtoLL(gps_end[1], gps_end[0], gps_end[2])
                
                    # change state
                    state = 3
                
                # if the gps calibration is started, send the linear_velocity to the robot in order to move forward 
                if (gps_state < 2 and checkDistance() == True):
                
                    if (laser_received == 1):
                                        
                        # start 
                        twist = Twist()
                        twist.linear.x = linear_velocity
                        twist.linear.y = 0
                        twist.linear.z = 0
                        twist.angular.x = 0
                        twist.angular.y = 0
                        twist.angular.z = 0
                    
                        pubCmdVel.publish(twist)
                        
                    else:
                        rospy.loginfo('Manually drive robot forward')
                    
          
            elif (state == 3):
                # calculate imu heading and correct by using the yaw_offset
                # publish the imu message over the configured imu topic
                # publish gps location of the map origin (end point of calibration)
                
                angles = tf.transformations.euler_from_quaternion([q_imu.x, q_imu.y, q_imu.z, q_imu.w], 'rzyx')   # convert imu orientation from quaternion to euler
                
                q = tf.transformations.quaternion_from_euler(angles[0] + imu_offset, angles[1], angles[2], 'rzyx')   # add imu offset and convert back to quaternion

                # create imu message
                imuMsg = Imu()
                imuMsg.header.stamp = rospy.Time.now()
                imuMsg.header.frame_id = imu_frame
                imuMsg.orientation.x = q[0]
                imuMsg.orientation.y = q[1]
                imuMsg.orientation.z = q[2]
                imuMsg.orientation.w = q[3] 
                
                imuMsg.angular_velocity.z = imu_ang_vel_z  # for EKF inupt

                #print("pitch",angles[1])
                
                """
                #ros topic to publish the slope direction
                if angles[1] > 1 : #offset of 1 degree
                    pubslopeDirection.publish(1)
                elif angles[1] < -1:
                    pubslopeDirection.publish(2)
                else:
                    pubslopeDirection.publish(0)
                """
                # publish over imu_corrected topic
                pubImu.publish(imuMsg)

                yaw = angles[0] + imu_offset   # corrected yaw
                pubImuFloat.publish(yaw)

                pubImuOffset.publish(imu_offset)
                
                # create GPS message
                gpsMsg = NavSatFix()
                gpsMsg.header.stamp = rospy.Time.now()
                gpsMsg.header.frame_id = gps_frame
                gpsMsg.latitude = origin_lat
                gpsMsg.longitude = origin_long
                
                # publish gps map origin
                pubGps.publish(gpsMsg)
                
                # publish imu calibration flag
                #pubCalib.publish(1)

                #################################
                # auto-calibration routine
                
                # check if the robot is driving straight
                
                #print("odometry_ received", odometry_received)
                #if (odometry_received == True and rtk_status == 4): # only in real time
                if (abs(odometry_v)> lin_tol and rtk_status == 4 and odometry_received == True):
                #if (odometry_received == True ): # only for simulation
                    #print("odom_w, odom_v",odometry_w, odometry_v)
                    #print("driving_str",driving_straight)
                    if (driving_straight == False and abs(odometry_w) <= odom_zero_tol):
                        #print("driving straight,",driving_straight)       
                        #print("odometry_w",abs(odometry_w))
                        #print("odometry_v",odometry_v)
                        if odometry_v > lin_tol: #was -0.01
                            direction = 1

                        #elif odometry_v < 0:
                        #elif odometry_v < -0.01: #was
                        else:
                            direction = -1

                        prev_direction = direction
                        #print("prev_direction",prev_direction)
                        #print("odom_v",odometry_v)
                        
                        
                        # if the robot is moving straight set the driving_straight flag and store last odometry timestamp
                        time_1_odom = rospy.get_time()
                        driving_straight = True
                        gps_set = 1
                        #print("HERE")
                        #rospy.loginfo('robot is driving straight')
                    elif (driving_straight == True and abs(odometry_w) > odom_zero_tol):
                        # if the angular velocity tolerance is not satisfied reset the driving_straight flag and cancel the auto-calibration
                        driving_straight = False
                        time_1_odom = rospy.get_time()
                        #time_1_calib = rospy.get_time()
                        #gps_set = 0 #TO TEST
                    
                        if (straight_calibration_state == 1):
                            # cancel calibration if started
                            straight_calibration_state = 0
                            #print("Auto calibration stopped due to turning of the robot")
                            rospy.loginfo('auto-calibration canceled (odometry_w = %f)', odometry_w)
        
                    # straight calibration states

                    if (straight_calibration_state == 0 and abs(odometry_w) <= odom_zero_tol): #was 0.009
                        # calibration start state
                        #print("Straight calibration state")
                        if abs(odometry_v) >lin_tol :
                            #print("time_1_calib",rospy.get_time() - time_1_calib)
                            if (rospy.get_time() - time_1_calib >= time_calib_offset): # 120 in real time unit: sec
                                if gps_set == 1:
                                    gps_start = [utm_x, utm_y, utm_zone]
                                    #gps_end = [utm_x, utm_y, utm_zone]
                                    print("gps_start 1 set",odometry_w)
                                    time_1_odom = rospy.get_time()
                                    rospy.loginfo('auto-calibration started 1')
                                    pubreCalib.publish(1)
                                    gps_set = 0
                                #print("HERE")
                                if odometry_v > lin_tol: #was -0.01
                                    direction = 1
                                    if prev_direction != direction:
                                        time_1_odom = rospy.get_time()
                                        prev_direction = direction
                                        gps_start = [utm_x, utm_y, utm_zone]
                                        print("gps_start 2 set", odometry_w)
                                        rospy.loginfo('auto-calibration started 2')
                                        pubreCalib.publish(1)

                                #elif odometry_v < 0:
                                else:
                                    direction = -1
                                    if prev_direction != direction:
                                        time_1_odom = rospy.get_time()
                                        prev_direction = direction
                                        gps_start = [utm_x, utm_y, utm_zone]
                                        print("gps_start 3 set",odometry_w)
                                        rospy.loginfo('auto-calibration started 3')
                                        pubreCalib.publish(1)

                                
                                
                                #print("Driving state and odom_w", driving_straight,odometry_w )
                                #print("Time for odom_straight", rospy.get_time() - time_1_odom)
                                if (driving_straight == True and rospy.get_time() - time_1_odom >= time_odom_offset): #and checkDistance() == True
                                    # if driving straight for 5 seconds and there are no obstacles in front of the robot
                                    print("About to calibrate")
                                    #TODO CHECK IF THE GPS OFFSET IS >10
                                    # check if driving forwards or backwards
                                    if odometry_v > lin_tol: #was -0.01
                                        direction = 1
                                    #elif odometry_v < 0:
                                    else:
                                        direction = -1
                        
                                    if prev_direction == direction :
                                        
                                        # store the gps measurement
                                        # gps_end = [utm_x, utm_y, utm_zone]
                                        #print ("can get gps_end")
                                        gps_end = [utm_x, utm_y, utm_zone]
                                        time_1_odom = rospy.get_time()
                                        # switch to next state
                                        #pubCalib.publish(2) # publishes 2 if it is auto calibrating
                                        straight_calibration_state = 1
                                    else:
                                        print("Auto calib cancelled")
                                        straight_calibration_state = 0
                                        time_1_odom = rospy.get_time()
                                        pubreCalib.publish(0)
                        
                    elif(straight_calibration_state == 0 and abs(odometry_w) > odom_zero_tol):
                            time_1_odom = rospy.get_time()
                            driving_straight == False
                            #gps_set = 0 #TO TEST
                            #print("Robot rotating")
                            
                                
                    elif (straight_calibration_state == 1 ):
                
                        
                                # check calibration distance for gps by using the initial measurement
                                #gps_end = [utm_x, utm_y, utm_zone]

                                #dx = gps_end[0] - gps_start[0]
                                #dy = gps_end[1] - gps_start[1]       
                                #d = math.sqrt(dx*dx + dy*dy)   # distance between start and end points of calibration
                
                                #pubCalib.publish(2)
                                #if (d >= calib_distance):
                                    # success if the calib_distance is satisfied
                                #    rospy.loginfo('auto-calibration distance satisfied')
                                print("Inside the calibration")
                                    # calculate yaw offset
                                if direction == 1:
                                    dy = gps_end[1] - gps_start[1]
                                    dx = gps_end[0] - gps_start[0]
                                elif direction == -1:
                                    dy = gps_start[1] - gps_end[1]
                                    dx = gps_start[0] - gps_end[0]
                                    
                                    
                                gps_yaw = math.atan2(dy, dx)   # calculate yaw from gps start and end points 

                                odom_angles = tf.transformations.euler_from_quaternion([q_odom.x, q_odom.y, q_odom.z, q_odom.w], 'rzyx') #
                                print("odom angles", odom_angles)
                                

                                imu_angles = tf.transformations.euler_from_quaternion([q_imu.x, q_imu.y, q_imu.z, q_imu.w], 'rzyx')   # convert imu orientation from quaternion to euler 
                                print("Imu angles",imu_angles)
                                print("The offset degree:", (gps_yaw - imu_angles[0])*180.0/3.14)
                                print("The offset rad:", (gps_yaw - imu_angles[0]))
                                # imu_yaw + imu_offset = gps_yaw
                                if abs(gps_yaw - odom_angles[0]) > (5*3.14)/180 and abs(gps_yaw - odom_angles[0]) < (360*3.14)/180:   #greater than 5 degree and less than 360 degree
                                    
                                    imu_offset = gps_yaw - imu_angles[0]   # difference between orientations from imu and gps
                                    
                                    rospy.loginfo('auto-calibration successful (offset = %f degs)', imu_offset / 3.1415 * 180.0)
                                    pubCalib.publish(2)
                                    pubreCalib.publish(2)
                                elif abs(gps_yaw - odom_angles[0]) > (360*3.14)/180:
                                    
                                    imu_offset = 6.28319 - (gps_yaw - imu_angles[0])
                                    pubCalib.publish(2)
                                    pubreCalib.publish(2)

                                else:
                                    rospy.loginfo('auto-calibration not required (offset = %f degs)', (gps_yaw - odom_angles[0]) / 3.1415 * 180.0)
                                    pubCalib.publish(0)
                                    pubreCalib.publish(0)

                                
                                # save calibration time to check if 5 minutes has passed
                                time_1_calib = rospy.get_time()
                                time_1_odom = rospy.get_time()
                            
                                # reset straight calibration state to perform the auto-calibration again
                                straight_calibration_state = 0

                        
                #else:
                    #rospy.loginfo("waiting for odometry") 
                else:
                    time_1_odom = rospy.get_time() 
                    driving_straight == False 
                    #gps_set = 0    #TO TEST
               
                # end of auto-calibration routine
                #################################
        
            rate.sleep()
    
    except rospy.ROSInterruptException:
        pass
        
