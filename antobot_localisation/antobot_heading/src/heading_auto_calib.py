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
#		        imu/data_corrected topic after calibration is successful.
#               Rewritten version
#             	  
#Contacts:     soyoung.kim@antobot.ai

# # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # #

import rospy
import time
import math
import sys, signal
from geonav_transform import geonav_conversions as gc

import tf
from sensor_msgs.msg import NavSatFix, Imu
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32, UInt8

def signal_handler(signal, frame):
    print("\nHeading_auto_calib killed")
    sys.exit(0)

signal.signal(signal.SIGINT, signal_handler)

class AutoCalibration:

    def __init__(self):

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
        self.direction = None
        self.gps_yaw = None

        # Read parameters to set the topic names
        self.gpsTopic = rospy.get_param('~gps_topic', 'am_gps_urcu')
        self.imuTopic = rospy.get_param('~imu_topic','imu/data')
        self.odometryTopic = rospy.get_param('~odometry_topic', 'odometry/filtered')
        self.wheelodometryTopic = rospy.get_param('~wheel_odometry_topic', 'am_robot/odom')

        self.new = rospy.get_param('~new', False)
        self.sim = rospy.get_param("/simulation",False) 

        if self.new:
            print('use new am_gps_urcu')
            self.rtk_target_status = 2 # with the new code, status 2 is 3D fixed mode
            if self.sim:
                self.rtk_target_status = 0 # in simulation, gps status is always 0
        else:
            print('use old am_gps_urcu')
            self.rtk_statusTopic = '/am_gps_urcu_status'
            self.subRtkStatus = rospy.Subscriber(self.rtk_statusTopic, UInt8, self.rtkstatusCallback)
            self.rtk_target_status = 4 

        # Subscribers
        self.subGps = rospy.Subscriber(self.gpsTopic, NavSatFix, self.gpsCallback)
        self.subImu = rospy.Subscriber(self.imuTopic, Imu, self.imuCallback)
        self.subOdometry = rospy.Subscriber(self.odometryTopic, Odometry, self.odometryCallback)
        self.subwheelOdom = rospy.Subscriber(self.wheelodometryTopic,Odometry, self.WheelodomCallback)
        
        
        # Publishers
        # TODO: fill imu data properly
        self.pubImu = rospy.Publisher('/imu/data_corrected', Imu, queue_size=10)
        self.pubImuOffset = rospy.Publisher('/imu/data_offset', Float32, queue_size=10)

        # publish calibration status - launches EKF when initial calibraiton is done (data == 1)
        # -1: inital value, 1: initial calibration finished, 2: auto calibration running
        self.imu_calibration_status = -1
        self.pubCalib = rospy.Publisher('/imu_calibration_status', UInt8, queue_size=10)

        # calculate and see if auto calibraion is needed?  
        self.timer = rospy.Timer(rospy.Duration(30), self.AutoCalibrate) # findMidLine is the main function that runs corridor following

###################################################################################################       

### Callback functions 

    def gpsCallback(self,msg):
        # UTMNorthing, UTMEasting, UTMZone
        self.utm_y, self.utm_x, self.utm_zone = gc.LLtoUTM(msg.latitude, msg.longitude)   # convert from gps to utm coordinates  
        if self.new: # use new gps code
            self.rtk_status = msg.status.status
        if (not self.gps_received):
            self.gps_received = True

    def imuCallback(self,data):   
        # store imu data
        self.q_imu = data.orientation
        self.imu_ang_vel_z = data.angular_velocity.z # value used for checking angular vel
        
        if (not self.imu_received):
            self.imu_received = True

    def WheelodomCallback(self,data):
        self.wheel_odom_v = data.twist.twist.linear.x # value used for checking linear vel

    def odometryCallback(self,data):
        # EKF odometry orientation is the same as calibrated imu's orientation
        self.q_odom = data.pose.pose.orientation # value to compare with gps yaw degree
        
        if (not self.odometry_received):
            self.odometry_received = True

    def rtkstatusCallback(self,data):
        self.rtk_status = data.data

###################################################################################################       

    def InitialCalibration(self):

        while ((not self.imu_received) or (not self.gps_received)):
            print('Waiting for initial imu and gps data new')
            rospy.sleep(0.1)

        print('IMU and gps value received, check for RTK status')

        while (self.rtk_status != self.rtk_target_status):
            print('Waiting for RTK status to become ',self.rtk_target_status)
            rospy.sleep(0.1)

        print('RTK status is {}- start calibration'.format(self.rtk_target_status))
      
        # save first calibration point
        rospy.loginfo('calibration started')

        state = 1
        while (True):
            if state == 1:
                # moving forward/backward, no angular : set start gps point
                gps_saved = self.SaveStartGPS()
                if gps_saved: # when the first gps position is saved
                    state = 2
            elif state == 2:
                result= self.CheckCondition() # returns the status
                state = result
            elif state == 3: # use self.gps_yaw
                print()
                break
            rospy.sleep(0.1) # othewise the ros shutdown deosn't work (from heading launcher)
    
      
        print('gps_yaw {}'.format(self.gps_yaw)) # wrt map x frame
        imu_angles = tf.transformations.euler_from_quaternion([self.q_imu.x, self.q_imu.y, self.q_imu.z, self.q_imu.w], 'rzyx')   # convert imu orientation from quaternion to euler
        # imu_angles[0] is angle wrt to map x frame (not sure why it's not roll, pitch,yaw)
        print('imu angle : {} {} {}'.format(imu_angles[0],imu_angles[1],imu_angles[2])) 
        # imu_yaw + imu_offset = gps_yaw
        self.imu_offset = self.gps_yaw - imu_angles[0]   # difference between orientations from imu and gps

        rospy.loginfo('calibration successful (offset = %f degs)', self.imu_offset / 3.1415 * 180.0)
        self.imu_calibration_status = 1

        # Let heading launcher know the inital calibration is finished 
        self.pubCalib.publish(self.imu_calibration_status)
        print('Initial calibration finished')

    def SaveStartGPS(self):
        vel = self.wheel_odom_v # to use consistent value
        if (abs(self.imu_ang_vel_z) < self.angular_zero_tol and (abs(vel) > self.lin_tol)): 
            self.gps_start = [self.utm_x, self.utm_y, self.utm_zone] # Save start gps position
            if vel > 0:
                self.direction = True # forward
            else:
                self.direction = False # backward
            return True
        else:
            return False


    def CheckCondition(self): # return the status
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

    def AutoCalibrate(self,event):
        self.started_time = rospy.get_time()
        if (self.imu_calibration_status == 1 and self.odometry_received):
            print('auto calibration called but skip one time')
            self.imu_calibration_status = 2
        elif(self.imu_calibration_status == 2): # depending on the timer setting, run this periodically 
            print('auto calibration checking started')
            state = 0

            while(True):
                if state == 0:
                    if (self.rtk_status == self.rtk_target_status and self.odometry_received):
                        state = 1
                elif state == 1:
                    # moving forward/backward, no angular : set start gps point
                    gps_saved = self.SaveStartGPS()
                    if gps_saved: # when the first gps position is saved
                        state = 2
                elif state == 2:
                    result= self.CheckCondition() # returns the status
                    state = result
                elif state == 3:
                    # Calibrate 
                    # gps angle calculated 
                    print("gps angles", self.gps_yaw)

                    # yaw angle from raw imu
                    # 'rzyx' is axes - returns yaw deg as the first value 
                    imu_angles = tf.transformations.euler_from_quaternion([self.q_imu.x, self.q_imu.y, self.q_imu.z, self.q_imu.w], 'rzyx')  
                    print("Imu angles", imu_angles[0])

                    # yaw angle from /odometry/filtered
                    odom_angles = tf.transformations.euler_from_quaternion([self.q_odom.x, self.q_odom.y, self.q_odom.z, self.q_odom.w], 'rzyx') 
                    print("odom angles", odom_angles[0])

                    # imu_yaw + imu_offset = gps_yaw
                    # Compare two angles in [-pi,pi] and returns signed value in radian
                    diff = self.gps_yaw - odom_angles[0]
                    if diff > math.pi:
                        diff -= 2*math.pi
                    elif diff < -math.pi:
                        diff += 2*math.pi
                    print('diff = ', diff)
                    diff_deg = abs(diff*180.0/math.pi)
                    print('angle diff = ',diff_deg)

                    if diff_deg > self.calib_deg:
                        self.imu_offset = self.gps_yaw - imu_angles[0]   # difference between orientations from imu and gps
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

###################################################################################################   

    def PublishNewIMU(self):
        # convert imu orientation from quaternion to euler
        angles = tf.transformations.euler_from_quaternion([self.q_imu.x, self.q_imu.y, self.q_imu.z, self.q_imu.w], 'rzyx')
        # add imu offset and convert back to quaternion      
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
        self.pubImu.publish(imuMsg)

        # Publsih /imu/data_offset
        self.pubImuOffset.publish(self.imu_offset) 
               
        
###################################################################################################       

### Main loop

if __name__ == '__main__':
    
    # init node
    rosnode = rospy.init_node('am_heading_imu_calib', anonymous=True)
    
    autoCalib = AutoCalibration()
    autoCalib.InitialCalibration() # First do initial calibration

    # loop rate
    rate = rospy.Rate(10)  # 10hz for imu publishing
    
    while not rospy.is_shutdown():
        if (autoCalib.imu_calibration_status > 0): # after initial calibration
            autoCalib.PublishNewIMU()
        rate.sleep()
