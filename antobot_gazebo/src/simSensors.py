#!/usr/bin/env python3

# Copyright (c) 2023, ANTOBOT LTD.
# All rights reserved.

# # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # 
# # # Code Description:     Converts simulated data for GPS and ultrasonic sensors into the same format as actual robot
# Contact: william.eaton@antobot.ai
# # # #  # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # #

import sys
import rospy
from sensor_msgs.msg import Range
from std_msgs.msg import UInt8
from gazebo_msgs.msg import ModelStates

from antobot_platform_msgs.msg import UInt16_Array


class gpsStatus:
	"""Simulates GPS status data"""

	def __init__(self):

		self.gpsState = UInt8(4) # Very simple, just assumes good RTK

		self.gpsStatus_pub = rospy.Publisher('antobot_gps_status', UInt8, queue_size=10)

		rospy.Timer(rospy.Duration(0.1), self.publishGPSState)


	def publishGPSState(self,event):
		
		self.gpsStatus_pub.publish(self.gpsState)


class ultrasonics:
	"""Stores and combines Ultrasonic data"""
	
	def __init__(self):
		
		self.ranges = UInt16_Array()

		self.range1=0
		self.range2=0
		self.range3=0
		self.range4=0
		self.range5=0
		self.range6=0
		self.range7=0
		self.range8=0


		self.range1_sub = rospy.Subscriber('/ultrasonic1', Range , self.range1_callback)   
		self.range2_sub = rospy.Subscriber('/ultrasonic2', Range , self.range2_callback)   
		self.range3_sub = rospy.Subscriber('/ultrasonic3', Range , self.range3_callback)   
		self.range4_sub = rospy.Subscriber('/ultrasonic4', Range , self.range4_callback)   
		self.range5_sub = rospy.Subscriber('/ultrasonic5', Range , self.range5_callback)   
		self.range6_sub = rospy.Subscriber('/ultrasonic6', Range , self.range6_callback)   
		self.range7_sub = rospy.Subscriber('/ultrasonic7', Range , self.range7_callback)   
		self.range8_sub = rospy.Subscriber('/ultrasonic8', Range , self.range8_callback)   

		# Create publishers
		self.ultrasonic_pub = rospy.Publisher('/antobridge/uss_dist',UInt16_Array, queue_size=5)

		# Create timer
		rospy.Timer(rospy.Duration(0.04), self.publishRanges)

	def publishRanges(self,event):
		self.ranges.data=[self.range1,self.range2,self.range3,self.range4,self.range5,self.range6,self.range7,self.range8]
		self.ultrasonic_pub.publish(self.ranges)

	def range1_callback(self,data):
		self.range1=round(data.range*100)

	def range2_callback(self,data):
		self.range2=round(data.range*100)

	def range3_callback(self,data):
		self.range3=round(data.range*100)

	def range4_callback(self,data):
		self.range4=round(data.range*100)

	def range5_callback(self,data):
		self.range5=round(data.range*100)

	def range6_callback(self,data):
		self.range6=round(data.range*100)

	def range7_callback(self,data):
		self.range7=round(data.range*100)

	def range8_callback(self,data):
		self.range8=round(data.range*100)



def main(args):


	rosnode=rospy.init_node('simSensors', anonymous=False)
	rospy.wait_for_message('/gazebo/model_states',ModelStates)

	gps=gpsStatus()
	range=ultrasonics()

	rospy.spin()


if __name__ == '__main__':
    main(sys.argv)
