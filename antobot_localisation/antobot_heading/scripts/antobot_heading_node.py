#!/usr/bin/env python3
# Copyright (c) 2023, ANTOBOT LTD.
# All rights reserved.

# # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # 
# Description: 	The primary purpose of this code is to calculate yaw offset from the robot motion with single GPS. 
#             	This script subscribes to the IMU and GPS topics and publishes Imu messages over 
#		        imu/data_corrected topic.     
#               This node performs the same functions as the antobot_heading.cpp but is written in Python.   
# Subscribes to: GPS topic (/antobot_gps_urcu)
#                imu topic (/imu/data)
#                EKF odometry topic (/odometry/filtered)
#                wheel odometry topic (/antobot_robot/odom)
# Publishes : Calibrated imu topic (/imu/data_corrected) - which is then used in EKF
# Contacts:     soyoung.kim@antobot.ai
# # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # #

import rospy
import time
import math
import sys, signal
import tf
from geonav_transform import geonav_conversions as gc
from sensor_msgs.msg import NavSatFix, Imu
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32, UInt8
from std_srvs.srv import *

def signal_handler(signal, frame):
    print("\nantobot_heading_node_python killed")
    sys.exit(0)

signal.signal(signal.SIGINT, signal_handler)

class autoCalibration:

    def __init__(self):
        # Description: Initialises autoCalibration class

        ## configuration parameters 
        self.calib_distance = 1.0 # calibration distance 
        self.angular_zero_tol = 0.1 # Angular velocity (rad/sec)
        self.lin_tol = 0.01 # linear velocity x (m/s)
        self.calib_deg = 3

        # GPS
        self.gps_start = []
        self.gps_end = []
        self.gps_received = False
        self.utm_y = 0
        self.utm_x = 0
        self.utm_zone = 0
        self.rtk_status = None

        # imu
        self.q_imu = []
        self.imu_frame = 'imu'
        self.imu_received = False
        self.imu_ang_vel_z = 0
        self.imu_offset = 0

        # odometry
        self.q_odom = []
        self.odometry_received = False 

        # wheel odometry
        self.wheel_odom_v = 0
        
        # Other parameters
        self.direction = None # True: robot moving forward (linear.x velocity > 0), False: moving backward
        self.gps_yaw = None

        # Read parameters to set the topic names
        self.gps_topic = rospy.get_param('~gps_topic', 'antobot_gps_urcu')
        self.imu_topic = rospy.get_param('~imu_topic','imu/data')
        self.odometry_topic = rospy.get_param('~odometry_topic', 'odometry/filtered')
        self.wheelodometry_topic = rospy.get_param('~wheel_odometry_topic', 'antobot_robot/odom')
        self.sim = rospy.get_param("/simulation",False) 

        self.rtk_target_status = 3 # rtk status is 3 in 3D fixed mode
        if self.sim:
            self.rtk_target_status = 0 # in simulation, gps status is always 0

        # Subscribers
        self.sub_gps = rospy.Subscriber(self.gps_topic, NavSatFix, self.gpsCallback)
        self.sub_imu = rospy.Subscriber(self.imu_topic, Imu, self.imuCallback)
        self.sub_odometry = rospy.Subscriber(self.odometry_topic, Odometry, self.odometryCallback)
        self.sub_wheel_odom = rospy.Subscriber(self.wheelodometry_topic,Odometry, self.wheelOdomCallback)
        
        # Publishers
        self.pub_imu = rospy.Publisher('/imu/data_corrected', Imu, queue_size=10)
        self.pub_imu_offset = rospy.Publisher('/imu/data_offset', Float32, queue_size=10)

        # ServiceClient
        rospy.wait_for_service('launch_ekf')
        self.launch_ekf_service = rospy.ServiceProxy('launch_ekf', Trigger)
        print('service found')


        # imu_calibration_status topic is used in antobot_heading_launcher (starts EKF node after the initial calibration)
        # -1: inital value, 1: initial calibration finished, 2: auto calibration running (every 30 seconds)
        self.imu_calibration_status = -1
        self.pub_calib = rospy.Publisher('/imu_calibration_status', UInt8, queue_size=10)

        # calculate and see if auto calibraion is needed
        self.timer = rospy.Timer(rospy.Duration(30), self.autoCalibrate) 

###################################################################################################       

### Callback functions 

    def gpsCallback(self,msg):
        # Description: GPS callback function that gets utm coordinates(from lat, long values) and rts status

        # UTMNorthing, UTMEasting, UTMZone
        self.utm_y, self.utm_x, self.utm_zone = gc.LLtoUTM(msg.latitude, msg.longitude)  
        self.rtk_status = msg.status.status
        if (not self.gps_received):
            self.gps_received = True

    def imuCallback(self,data):  
        # Description: IMU callback function (IMU sensor raw data)

        self.q_imu = data.orientation
        self.imu_ang_vel_z = data.angular_velocity.z # value used for checking angular vel
        if (not self.imu_received):
            self.imu_received = True

    def wheelOdomCallback(self,data):
        # Description: Wheel odometry callback function that gets linear x velocity 

        self.wheel_odom_v = data.twist.twist.linear.x # value used for checking linear vel

    def odometryCallback(self,data):
        # Description: EKF odometry topic callback function.

        self.q_odom = data.pose.pose.orientation # value to compare with gps yaw degree
        if (not self.odometry_received):
            self.odometry_received = True

###################################################################################################  

### Other functions

    def checkInputs(self):
        # Description: Check if the imu, gps data are recieved and check if the RTK status is fixed mode value

        while ((not self.imu_received) or (not self.gps_received)):
            print('Waiting for initial imu and gps data new')
            rospy.sleep(0.1)

        print('IMU and gps value received, check for RTK status')

        while (self.rtk_status != self.rtk_target_status):
            print('Waiting for RTK status to become ',self.rtk_target_status)
            rospy.sleep(0.1)

        print('RTK status is {}- start calibration'.format(self.rtk_target_status))


    def initialCalibration(self):
        # Description: Function that is called once in the main function. When all the required topics are recieved (self.checkInputs)
        # this function performs the initial calibration. In the while loop, the starting gps postion is saved and when the 
        # Calibration conditions are met, the imu_offset is calculated. 

        # Check if all input toics are being published 
        self.checkInputs() 

        rospy.loginfo('calibration started')
        state = 1
        while (True):
            if state == 1:
                gps_saved = self.saveStartGPS()
                if gps_saved: # if the start gps position is saved
                    state = 2
            elif state == 2:
                result= self.checkCondition() 
                state = result
                if (state == 3):
                    break
            rospy.sleep(0.1) # othewise the ros shutdown deosn't work (from heading launcher)
      
        print('gps_yaw {}'.format(self.gps_yaw))
        # convert imu orientation from quaternion to euler
        imu_angles = tf.transformations.euler_from_quaternion([self.q_imu.x, self.q_imu.y, self.q_imu.z, self.q_imu.w], 'rzyx')  
        print('imu angle : {} {} {}'.format(imu_angles[0],imu_angles[1],imu_angles[2])) 
        # Formula used: imu_yaw + imu_offset = gps_yaw
        self.imu_offset = self.calculateDifference(self.gps_yaw,imu_angles[0])

        rospy.loginfo('calibration successful (offset = %f degs)', self.imu_offset / 3.1415 * 180.0)
        self.imu_calibration_status = 1

        # Let heading launcher know the inital calibration is finished 
        self.pub_calib.publish(self.imu_calibration_status)
        print('Initial calibration finished - now launch ekf nodes - python')


        
        rospy.wait_for_service('launch_ekf')
        srv = TriggerRequest()
        response = self.launch_ekf_service(srv)
       




    def saveStartGPS(self):
        # Description: Save gps point as the starting point if the robot is moving. Set direction depending on the linear x velocity.
        # Robot wheel odometry's linear x should be larger than self.lin_tol and the imu angular z velocity should be smaller than self.angular_zero_tol.
        # self.direction is set to True when the robot is moving forward.
        # Returns: True if the start GPS point is saved, False if the robot velcoity didn't meet the conditons and failed to save the start GPS point.

        vel = self.wheel_odom_v
        if (abs(self.imu_ang_vel_z) < self.angular_zero_tol and (abs(vel) > self.lin_tol)): 
            self.gps_start = [self.utm_x, self.utm_y, self.utm_zone] # Save start gps position
            if vel > 0:
                self.direction = True # forward
            else:
                self.direction = False # backward
            return True
        else:
            return False

    def checkCondition(self):
        # Description: Check several conditions in order to start calibration and returns the state that will be used in the while loop that called this function.
        # Condition: 1. imu angular z velocity should be smaller than angular_zero_tol.
        #            2. Robot's direction (forward/backward) should be consistent and hasn't change since the saving of the staring GPS point.
        #            3. Robot has moved more than calib_distance (calculated based on gps lat/long data)
        # Returns: state that will be used in the while loop that called this function
        #          state 1: Reset and save another GPS start point
        #          state 2: Distance moved is not enough, continue to call checkCondition
        #          state 3: Distance moved is larger than calib_distance, now exit the while loop and use the self.gps_yaw in the next step of the calibration.

        # Reset if angular vel is large
        if (abs(self.imu_ang_vel_z) > self.angular_zero_tol):
            print('Reset due to angular velocity')
            return 1

        # Reset if direction is changed
        if self.wheel_odom_v > self.lin_tol:
            curret_dir = True # forward
        elif (abs(self.wheel_odom_v) <= self.lin_tol):
            curret_dir = self.direction # it is stopped - hasn't changed the direction
        else:
            curret_dir = False # backward
        
        if (curret_dir != self.direction):
            print('Reset due to change in direction')
            return 1

        # Check the distance
        self.gps_end = [self.utm_x, self.utm_y, self.utm_zone]

        if self.direction: # forward
            dy = self.gps_end[1] - self.gps_start[1]
            dx = self.gps_end[0] - self.gps_start[0]
        else: # backward
            dy = self.gps_start[1] - self.gps_end[1] 
            dx = self.gps_start[0] - self.gps_end[0] 

        d = math.sqrt(dx*dx + dy*dy)   # distance between start and end points of calibration
        self.gps_yaw = math.atan2(dy, dx)   # calculate yaw from gps start and end points

        if (d >= self.calib_distance):
            rospy.loginfo('calibration distance satisfied')
            return 3
        else:
            return 2 # not satisfied
        
    def calculateDifference(self, input_a, input_b):
        # Description: Compare two values in [-pi,pi] and return signed value in the same range
        # Input: Two radian values in [-pi,pi]
        # Returns: Difference between two input values in [-pi,pi] 

        diff =  input_a - input_b
        if diff > math.pi:
            diff -= 2*math.pi
        elif diff < -math.pi:
            diff += 2*math.pi
        return diff


    def autoCalibrate(self,event):
        # Description: Function that is called every 30 seconds to check if the calibration is needed. 
        # In the while loop, several conditions are checked and if theses conditions are met within 20 seconds, the new imu_offset is calculated (calibration)
        
        self.started_time = rospy.get_time()
        if (self.imu_calibration_status == 1 and self.odometry_received):
            print('auto calibration called but skip one time') # Since the initial calibration was done recently
            self.imu_calibration_status = 2
        elif(self.imu_calibration_status == 2):
            print('auto calibration checking started')
            state = 0 
            while(True):
                if state == 0: # Check rtk status and ekf odometry
                    if (self.rtk_status == self.rtk_target_status and self.odometry_received):
                        state = 1
                elif state == 1: # Save start gps position
                    gps_saved = self.saveStartGPS()
                    if gps_saved: # If the first gps position is saved
                        state = 2
                elif state == 2: # Check if the auto calibration conditions are met
                    result= self.checkCondition() 
                    state = result
                elif state == 3: # Conditions met, start the calibration
                    # gps angle calculated 
                    print("gps angles", self.gps_yaw)

                    # yaw angle from raw imu
                    # 'rzyx' is axes - returns yaw deg as the first value 
                    imu_angles = tf.transformations.euler_from_quaternion([self.q_imu.x, self.q_imu.y, self.q_imu.z, self.q_imu.w], 'rzyx')  
                    print("Imu angles", imu_angles[0])

                    # yaw angle from /odometry/filtered
                    odom_angles = tf.transformations.euler_from_quaternion([self.q_odom.x, self.q_odom.y, self.q_odom.z, self.q_odom.w], 'rzyx') 
                    print("odom angles", odom_angles[0])

                    # Formula used: imu_yaw + imu_offset = gps_yaw
                    diff_rad = self.calculateDifference(self.gps_yaw,odom_angles[0])
                    diff_deg = abs(diff_rad*180.0/math.pi)
                    print('angle diff = ',diff_deg)

                    if diff_deg > self.calib_deg:
                        self.imu_offset = self.calculateDifference(self.gps_yaw,imu_angles[0])  # difference between orientations from imu and gps
                        rospy.loginfo('auto-calibration successful (imu offset = %f degs)', self.imu_offset / 3.1415 * 180.0)
                    else:
                        rospy.loginfo('auto-calibration not required {} deg'.format(diff_deg))
                    break

                duration = (rospy.get_time() - self.started_time)
                if duration > 20.0:
                    print('auto calibration conditions not met')
                    break
                rospy.sleep(0.1)
        else:
            print('auto calibration called but ignored - Do initial calibration first!')

    def publishNewIMU(self):
        # Description: Publish calibrated imu data (/imu/dadta_coreccted) and a topic for debugging purpose (/imu/data_offset)
        # Input: imu raw data and calculated imu_offset. imu_offset is added to the yaw value of the imu data. 

        # convert imu orientation from quaternion to euler
        angles = tf.transformations.euler_from_quaternion([self.q_imu.x, self.q_imu.y, self.q_imu.z, self.q_imu.w], 'rzyx')
        # add imu offset to the yaw value  and then convert it  back to quaternion      
        q = tf.transformations.quaternion_from_euler(angles[0] + self.imu_offset, angles[1], angles[2], 'rzyx')   

        # create imu message
        imuMsg = Imu()
        imuMsg.header.stamp = rospy.Time.now()
        imuMsg.header.frame_id = self.imu_frame # imu
        imuMsg.orientation.x = q[0]
        imuMsg.orientation.y = q[1]
        imuMsg.orientation.z = q[2]
        imuMsg.orientation.w = q[3] 
        imuMsg.angular_velocity.z = self.imu_ang_vel_z  # for EKF inupt

        # publish /imu/data_corrected
        self.pub_imu.publish(imuMsg)

        # Publsih /imu/data_offset
        self.pub_imu_offset.publish(self.imu_offset) 
               
        
###################################################################################################       

### Main loop

if __name__ == '__main__':
    # init node
    rosnode = rospy.init_node('antobot_heading_node_py', anonymous=True)
    
    # Define an autoCalibration object and run the initial calibration
    autoCalib = autoCalibration()
    autoCalib.initialCalibration()

    # loop rate
    rate = rospy.Rate(10)  # 10hz for imu publishing
    
    while not rospy.is_shutdown():
        if (autoCalib.imu_calibration_status > 0): # after initial calibration
            autoCalib.publishNewIMU()
        rate.sleep()
