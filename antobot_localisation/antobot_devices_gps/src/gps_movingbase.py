#!/usr/bin/env python3
# Copyright (c) 2023, ANTOBOT LTD.
# All rights reserved

# # # Code Description: This code is used to connect to ublox Things stream server to subscribe the GNSS augmentation data and implement PPP IP mode in ublox f9p chip. 
# This code should only be used in the Ublox chip : "ZED-F9P-04B-01" (>uRCUv1.2c) with firmware upgraded to HPG1.32.
# The chip was connected to the Xavier via SPI. To run the script at 8Hz, the following modifications should be made in the f9p chip. (run gps_config_movingbase.py first)
 #   CFG-RATE-MEAS = 125 sec
 #   VALSET: CFG-MSGOUT-NMEA-ID_GSV_UART1 = 0
 #   VALSET: CFG-MSGOUT-NMEA-ID_GSA_UART1 = 0
 #   VALSET: CFG-MSGOUT-NMEA-ID_GLL_UART1 = 0

#This script reports the following on GPS frequency status:
# GPS Frequency status : Critical ; when the GPS frequency <2Hz
# GPS Frequency status : Warning ; when the 2>= GPS frequency <6Hz 
# GPS Frequency status : Good ; when the GPS frequency >= 6Hz

#This script reports the following on Heading status:
# Heading status : Invalid ; Heading status = 0
# Heading status : Valid ; Heading status = 1

#This script reports the following on Heading frequency status:
# Heading Frequency status : Critical ; when the Heading frequency <2Hz
# Heading Frequency status : Warning ; when the 2>= Heading frequency <6Hz 
# Heading Frequency status : Good ; when the Heading frequency >= 6Hz

# Contact: Aswathi Muralidharan
# email:  aswathi.muralidharan@antobot.ai
########################################################################################################################################################

import threading
import asyncio
import time
import math

import rospy,rostopic
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import TwistWithCovarianceStamped
from std_msgs.msg import UInt8
from geometry_msgs.msg import Vector3
from std_msgs.msg import Float64
import importlib
#import Jetson.GPIO as GPIO

from antobot_devices_gps.movingbase import MovingBase

class MovingBase_Ros:
    def __init__(self, base_port_uart, rover_port, base_port_spi, mode=1):

        self.node_type = "movingbase"
        self.base_port_uart = base_port_uart
        self.rover_port = rover_port
        self.base_port_spi = base_port_spi
        self.mode = mode
        
        self.device_connect = False
        self.publish_first = True
        self.freq_low = 2
        self.freq_high = 4

        self.time_buf_len = 10

        self.pub_heading_urcu = rospy.Publisher('/antobot_gps/heading/urcu', Float64, queue_size=10)
        self.pub_heading_robot = rospy.Publisher('/antobot_gps/heading/robot', Float64, queue_size=10)
        self.pub_relposned = rospy.Publisher('/antobot_gps/relposned', Vector3, queue_size=10)
        self.heading_hz = 0
        self.heading_freq_status = None
        self.heading_time_buf = []
        self.heading_status = False
        self.heading_status_str = None

        # Only considering px and py (two antennas should be placed at the same height)
        urcu_px = rospy.get_param("/gps/urcu/px",0.1)
        urcu_py = rospy.get_param("/gps/urcu/py",0.6)

        rover_px = rospy.get_param("/gps/movingbase/px",0.1)
        rover_py = rospy.get_param("/gps/movingbase/py",-0.6)

        self.robot_angle_correction = self.calculate_enu_angle(urcu_px, urcu_py, rover_px, rover_py)

    def ros_pub_relposned(self, frame):

        if frame:
            self.heading_time_buf.append(time.time())
            heading = frame.relPosHeading/100000
            self.heading_status = frame.relPosHeadingValid

            if self.heading_status:
                self.pub_heading_urcu.publish(heading) # True North heading - keep it for debugging

                # Convert the value to ENU (East-North-Up)
                enu_heading = (450.0 - heading) % 360.0

                # Calculate and publish the robot heading
                msgs = Float64()
                msgs.data = ((enu_heading) - self.robot_angle_correction)%360.0 # Normalise the result to be within 0 to 360
                self.pub_heading_robot.publish(msgs)

                # Publish the relative position in north, east, down directions
                relposned = Vector3()
                relposned.x = frame.relPosN
                relposned.y = frame.relPosE
                relposned.z = frame.relPosD
                self.pub_relposned(relposned)

                
        elif len(self.heading_time_buf) > 0:
            self.heading_time_buf.pop(0)
        elif len(self.heading_time_buf) == 0:
            self.heading_status = False

        if self.heading_status is False and self.heading_status_str != "Invalid":
            rospy.logerr("SN4020: Heading Status: Invalid")
            self.heading_status_str  = "Invalid"
        elif self.heading_status and self.heading_status_str != "Valid":
            rospy.loginfo("SN4020: Heading Status: Valid")
            self.heading_status_str = "Valid"

        if len(self.heading_time_buf) > self.time_buf_len:
            self.heading_time_buf.pop(0)

        if len(self.heading_time_buf)>2:
            self.heading_hz = (len(self.heading_time_buf)-1) / (time.time() - self.heading_time_buf[0])  
        else:
            self.heading_hz = 0

        if self.heading_hz < self.freq_low and self.heading_freq_status != "Critical":
            rospy.logerr(f"SN4022: Heading Frequency status: Critical (<{self.freq_low} hz)")
            self.heading_freq_status = "Critical"
        elif self.heading_hz >= self.freq_low and self.heading_hz < self.freq_high and self.heading_freq_status != "Warning":
            rospy.logwarn(f"SN4022: Heading Frequency status: Warning (<{self.freq_high} hz)")
            self.heading_freq_status = "Warning"
        elif self.heading_hz >= self.freq_high and self.heading_freq_status != "Good":
            rospy.loginfo(f"SN4022: Heading Frequency status: Good (>{self.freq_high} hz)")
            self.heading_freq_status = "Good"
    
    def calculate_enu_angle(self, xA, yA, xB, yB):
        # Compute the differences in the ENU (East-North) coordinate system
        dx = xB - xA  # East-West difference (X axis)
        dy = yB - yA  # North-South difference (Y axis)
        
        # Calculate the angle using atan2, relative to the East (positive X-axis)
        angle_radians = math.atan2(dy, dx)
        
        # Convert the angle to degrees
        angle_degrees = math.degrees(angle_radians)
        
        # Normalize the angle to be within 0 to 360 degrees
        if angle_degrees < 0:
            angle_degrees += 360
        
        return angle_degrees
    
    
    async def create_MovingBase(self):
        while not self.device_connect:
            try:
                MB = await MovingBase.create(self.base_port_uart, self.rover_port, self.base_port_spi, self.mode)
                self.device_connect = True
                return MB
            except Exception as e:
                if self.publish_first:
                    rospy.logerr('MovingBase initialization failed: %s:', str(e))
                    self.publish_first = False
                time.sleep(1)
        
    async def main(self):
        MB = await self.create_MovingBase()
        while True:
            try:
                headFrame = await MB.get_RELPOSNEDframe()
                self.ros_pub_relposned(headFrame)

            except:
                rospy.logerr(f"MovingBase: Close the MovingBase node")
                break

if __name__ == '__main__':
    
    rospy.init_node("nRTK", disable_signals=True)

    base_port_uart = rospy.get_param("/gps/urcu/device_port","/dev/ttyTHS0")
    rover_port = rospy.get_param("/gps/ublox_rover/device_port","/dev/AntoF9P")
    base_port_spi = None

    movebase = MovingBase_Ros(base_port_uart, rover_port, base_port_spi, mode=1)
    loop = asyncio.get_event_loop()
    try:
        loop.run_until_complete(movebase.main())
    except Exception as e:
        loop.close()
