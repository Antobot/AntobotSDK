#!/usr/bin/env python3

# Copyright (c) 2023, ANTOBOT LTD.
# All rights reserved.

# # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # 

#Description:   This is a python script to 
#                 
#Contact:     zhuang.zhou@antobot.ai

# # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # 

#ROS related
import rospy
from std_msgs.msg import String, Bool,UInt8,Float64,Float32,Int8
from sensor_msgs.msg import NavSatFix, Imu
from nav_msgs.msg import Odometry
from antobot_msgs.msg import UInt8_Array,Float32_Array,UInt16_Array ##check syntax
from antobot_msgs.srv import softShutdown

#3rd party
import socket
from jtop import jtop
import geonav_transform.geonav_conversions as gc
import tf

#python
import time
import os
import shutil
import sys
import math
from shutil import copyfile


#global parameter
emergency_alarm_switch = True #true as turn on, false as turn off, apply in main 
disk="./"
min_gb=2
min_percent=10



class Monitor():
    """
    Attributes:
        As_fBat: the robot battery level measured by uRCU and pass through antobridge
        As_As_sBatlvl: the battery level of robot based on SoC, above 80% is high, between 80% and 55% is medium, between 55% and 30% is low, below 30% is alert
        As_fPwr: robot power consumption measured by PDU, passed through antobridge
        As_uCPUtemp: CPU tempreture measured by s-tui
        As_uUtil: CPU utilization measured by s-tui
        As_uRobTemp: robot tempreture measured by sensor? maybe two
        As_b4G: 4G connectivity status, true means connected and false means disconnected
        As_bWifi: wifi connectivity status, true means connected and false means disconnected
        As_bEthernet: ethernet connectivity status,  true means connected and false means disconnected
        As_bRTK:RTK status (bolean or string?)
        As_bGNSS: GNSS status(bolean or string?)
        As_bStorage: storage status of the disk, true is still have space and false means full
        As_uAlarm: 1 means robot stuck in the mud or somewhere uneven, 2 means force stop triggered and need human help, 3 means need push, 0 means no alarm
    """
    def __init__(self):
        """Inits the class Minitor with all its attributes""" 
        self.As_uBat = 55
        self.As_uSoC = 100
        self.As_sBatlvl = None
        self.As_fPwr = 0
        self.As_uCPUtemp = 0
        self.pub_cpu_load = 0
        self.As_uUtil= 0
        self.As_uRobTemp = 0
        self.As_b4G = False
        self.As_Wifi = False
        self.As_bEthernet = False
        self.As_bNetwork = False
        self.As_bRTK = False
        self.As_bGNSS = False
        self.As_bStorage = False
        self.As_uAlarm = 0
        self.As_bStorage = False
        self.voltage_pre = 55
        self.voltage_cnt = 0
        self.soft_shutdown_req = False
        self.slope_offset = 15*3.14/180 # imu_slope offset

        # self.pub_GPS_status = False

        """mileage"""
        self.uMode = 0 #robot operation mode
        self.vel = 0
        self.angular_vel = 0
        self.driving_state = 0
        self.timestamp = 0 # Used to time how long robot drives straight
        self.lon = None
        self.lat = None
        self.b_robot_movement = False # default robot is not move
        self.u_robot_movement_cnt = 0

        """ alarm data initialization"""
        self.As_wheel_cmd = None
        self.As_wheel_vel = None
        self.Am_force_stop = 0
        self.Ab_force_stop = False
        self.As_uss_data = None
        self.As_b_vel_cmd = False    # default no moving command is being sent
        self.anto_bridge_function = False
        self.alarm_3_cnt = 0
        
        #Xavier monitor
        self.jetson = jtop()
        self.jetson.start()


        self.pub_GPS_status = rospy.Publisher("/as/GPS_status",Bool,queue_size = 1)

        # ROS subscriber
        self.sub_As_uBat = rospy.Subscriber("/antobridge/Ab_uBat",UInt8_Array,self.battery_callback)             #pass battery voltage to anto_supervisor, it's an array data type
        self.sub_GPS_data = rospy.Subscriber("/antobot_gps",NavSatFix,self.GPS_callback)                         #pass GPS data to anto_supervisor, it's a data type of NavSatFix, include data.latitude & data.longitude
        self.sub_wheel_vel_cmd = rospy.Subscriber("/antobridge/wheel_vel",Float32_Array,self.wheel_vel_callback) #pass wheel_vel speed to anto_supervisor, it's a float array data type 
        self.sub_wheel_cmd = rospy.Subscriber("/antobridge/wheel_vel_cmd",Float32_Array,self.wheel_cmd_callback) #pass teleop wheel_vel_cmd to anto_supervisor, it's a float array data type
        self.sub_force_stop = rospy.Subscriber("/antobridge/force_stop",Bool,self.ab_force_stop_callback)           #pass force stop signal to anto_supervisor, it's a boolean type
        self.sub_am_force_stop = rospy.Subscriber("/antobot_safety/force_stop_type",Int8,self.am_force_stop_callback)
        self.sub_uss_dist = rospy.Subscriber("/antobridge/uss_dist",UInt16_Array,self.uss_dist_callback)         #pass uss_dist data to anto_supervisor, it's a UInt16 array type
        self.sub_mode = rospy.Subscriber('/switch_mode', UInt8, self.mode_callback)                                #mode 0:keyboard, 1: app, 2: joystick 3:autonomous 4:go home
        self.sub_soft_shutdown_button = rospy.Subscriber('/antobridge/soft_shutdown_button',Bool,self.soft_shutdown_callback)
        self.sub_slope_dir = rospy.Subscriber('/imu/data_corrected', Imu, self.slope_dir)
        self.imu_calib_status = rospy.Subscriber('/imu_calibration_status',UInt8, self.imu_calib)
        self.recalib_status = rospy.Subscriber('/recalib_status',UInt8,self.imu_recalib)
        
        # ROS Publisher
        self.pubslopeDirection = rospy.Publisher("/imu/slope_direction", UInt8, queue_size=1) #topic to publish the slope direction
        self.pub_roll_Direction = rospy.Publisher("/imu/roll_direction", UInt8, queue_size=1)
        self.pub_cpu_temp = rospy.Publisher("/as/cpu_temp",Float32,queue_size=1)#topic to publish the cpu tempreture
        self.pub_cpu_load = rospy.Publisher("/as/cpu_load",UInt8,queue_size=1)#topic to publish the cpu tempreture
        self.pub_storage = rospy.Publisher("/as/storage",Float64, queue_size = 1)
        self.pub_soc = rospy.Publisher("/as/soc",UInt8,queue_size = 1)#percentage soc
        self.pub_As_uBat = rospy.Publisher("/as/As_uBat",Float32,queue_size = 1)
        self.pub_batlvl = rospy.Publisher("/as/batlvl",String,queue_size = 1)
        self.pub_network = rospy.Publisher("/as/network",Bool,queue_size = 1)
        self.pub_As_uAlarm = rospy.Publisher("/as/alarm",UInt8,queue_size =1)
        self.pub_soft_shutdown_req = rospy.Publisher("/antobridge/soft_shutdown_req", Bool,queue_size = 1)

        # ROS Service
        self.soft_shutdown_client = rospy.ServiceProxy('soft_shutdown_req',softShutdown)
        
    
    def imu_calib(self,data):
        if data == 0:
            print("Heading is correct")
        elif data == 1:
            print("Initial calibration")
        elif data == 2:
            print("Heading is off")


    def imu_recalib(self,data):
        if data == 0:
            print("recalibration cancelled either due to rotation or no heading offset")
        elif data == 1:
            print("in recalibration mode")
        elif data ==2:
            print("Recalibration occured")



    def xavier_monitor(self):
        #access to Jtop and read the xavier info
        jtopstats = self.jetson.stats      
        self.As_cputemp = jtopstats["Temp CPU"]
        CPU1 = jtopstats["CPU1"]
        CPU2 = jtopstats["CPU2"]
        CPU3 = jtopstats["CPU3"]
        CPU4 = jtopstats["CPU4"]
        CPU5 = jtopstats["CPU5"]
        CPU6 = jtopstats["CPU6"]
        self.As_cpuload = (CPU1+CPU2+CPU3+CPU4+CPU5+CPU6)//6
        self.pub_cpu_temp.publish(self.As_cputemp)
        self.pub_cpu_load.publish(self.As_cpuload)



    def network_status(self): #works
       #Returns False if it fails to resolve Google's URL, True otherwise#
        try:
            socket.gethostbyname("www.google.co.uk")
            self.As_bNetwork = True
            self.pub_network.publish(self.As_bNetwork)
        except:
            self.As_bNetwork = False
            self.pub_network.publish(self.As_bNetwork)


    def storage_management(self): #works
            #Returns True if there isn't enough disk space, False otherwise.#
        
        du = shutil.disk_usage(disk)
        self.As_bStorage = False
        # Calculate the percentage of free space
        percent_free = 100 * du.free / du.total
        # Calculate how many free gigabytes
        gigabytes_free = du.free / (2**30)  
        #print("gigabytes_free:",gigabytes_free)
        self.pub_storage.publish(gigabytes_free)

        if gigabytes_free < min_gb or percent_free < min_percent: #min_percent=5%
            self.As_bStorage = True
        return self.As_bStorage


    def emergency_alarm(self): #GNSS data needed
        #emergency alarm to report the abnormal status of rebot 
        self.As_uAlarm = 0
        if self.Am_force_stop > 0 or self.Ab_force_stop == True:
            self.As_uAlarm = 2 #force stop triggered
        if self.anto_bridge_function == False and self.alarm_3_cnt == 0 : #anto_bridge not work (either node die or spi communication fail)
            self.As_uAlarm = 1
        else:
            self.alarm_3_cnt = 0
        if self.As_b_vel_cmd == True: #command is send
            if abs(self.As_wheel_vel[0]) < abs(self.As_wheel_cmd[0]) * 0.5 or abs(self.As_wheel_vel[1]) < abs(self.As_wheel_cmd[1]) * 0.5 or abs(self.As_wheel_vel[2]) < abs(self.As_wheel_cmd[2])*0.5 or abs(self.As_wheel_vel[3]) < abs(self.As_wheel_cmd[3])*0.5:
                self.As_stuck_cnt = self.As_stuck_cnt + 1
            else:
                self.As_stuck_cnt = 0
            if self.As_stuck_cnt > 4:
                self.As_uAlarm = 3
            if self.As_wheel_cmd[0]*self.As_wheel_cmd[2] < 0: #spot turn command
                if self.turn == False:
                    self.As_uAlarm = 3 #immediate report
        else:
            self.u_robot_movement_cnt = 0
        self.anto_bridge_function = False #set this flag to false before next antobridge topic callback
        self.pub_As_uAlarm.publish(self.As_uAlarm)
    
    def print_monitor(self):
        #sys.stdout.write("self.As_uSoC:",self.As_uSoC)
        print("self.As_bNetwork",self.As_bNetwork)
        print("self.As_uBat:",self.As_uBat)
        print("self.As_uSoC:",self.As_uSoC)
        print("self.As_sBatlvl:" ,self.As_sBatlvl)
        print("self.As_bStorage:" , self.As_bStorage)
        print("self.As_bGNSS:",self.As_bGNSS)
        print("cpu temp:",self.As_cputemp)


    # Callback function for robot operation mode
    # mode 0:keyboard, 1: app, 2: joystick 3:autonomous 4:go home
    def mode_callback(self, mode):
        self.uMode = mode.data
        return self.uMode


    def battery_callback(self, Ab_uBat): #works
            # # # The callback function for battery data (/antobridge/Ab_uBat), get the voltage,filter it and pass it to battery_indication function to get the percentage voltage and level
        voltage_decimal = float(Ab_uBat.data[1])
        voltage = float(Ab_uBat.data[0])+ voltage_decimal/100
        if voltage == self.voltage_pre:
            self.voltage_cnt = self.voltage_cnt+1
        else:
            self.voltage_cnt = 0
        self.voltage_pre = voltage
        #print(voltage)
        if self.voltage_cnt > 5:
            if voltage <self.As_uBat:
                self.As_uBat = voltage #low pass

        
    def soft_shutdown_callback(self,soft_shutdown):  #soft shutdown button on joystick pressed
        if soft_shutdown.data == True:
            self.soft_shutdown_req = True


    def soft_shutdown_process(self): #send req to antobridge, call the power off service
        if self.soft_shutdown_req == True:
            self.pub_soft_shutdown_req.publish(self.soft_shutdown_req)
            soft_shutdown_reponse = self.soft_shutdown_client(1)



    def battery_lvl(self):#works
        counter = 100
        #print("voltage in battery_lvl", voltage)
        #The look up SOC table
        #print("self.As_uBat:",self.As_uBat)
        while counter >0:
            #print(counter)
            if self.As_uBat >= 39+counter*0.156 : #0.156=0.012(increasment)*13(series)
                self.As_uSoC = counter
                break
            counter = counter - 1
        #The level convertion 
        if self.As_uSoC >= 80:
            self.As_sBatlvl = "high"
        elif self.As_uSoC >= 55:
            self.As_sBatlvl = "medium"
        elif self.As_uSoC >= 30:
            self.As_sBatlvl = "low"
        else:
            self.As_sBatlvl = "alert"
        self.pub_soc.publish(self.As_uSoC)
        self.pub_As_uBat.publish(self.As_uBat)
        self.pub_batlvl.publish(self.As_sBatlvl)


    def GPS_callback(self,gps_msg):
        #gps callback function, if gps status is 3 then it's in fix mode
        self.GPS_freq_cnt = self.GPS_freq_cnt + 1
        if  gps_msg.status.status == 3:
            self.As_bGNSS = True
            self.As_lat = gps_msg.latitude
            self.As_lon = gps_msg.longitude
            #print("GPS True")
        else:
            #print("GPS False")
            self.As_bGNSS = False
        self.pub_GPS_status.publish(self.As_bGNSS)



    def wheel_vel_callback(self,data):
        #print("sending wheel velocity data from callback")
        self.anto_bridge_function = True
        self.alarm_3_cnt = self.alarm_3_cnt +1
        self.As_wheel_vel = data.data
        return

    def wheel_cmd_callback(self,data):
        #print("sending teleop speed command data from callback")
        self.wheel_cmd = data.data
        if self.wheel_cmd[0] !=0 or self.wheel_cmd[1] !=0 or self.wheel_cmd[2] !=0 or self.wheel_cmd[3] !=0: #command any of the wheel move
            self.b_vel_cmd = True  #sent wheel moving cmd 
        else:
            self.b_vel_cmd = False 
        return

    def am_force_stop_callback(self,data):
        self.Am_force_stop = data.data #True as triggered, false as not triggered
        return

    def ab_force_stop_callback(self,data):
        #print("sending force stop signal from callback")
        self.Ab_force_stop = data.data #True as triggered, false as not triggered
        return

    def uss_dist_callback(self,data):
        #print("sending force stop signal from callback")
        self.uss_data = data 
        return

    def slope_dir(self,data):
        angles = tf.transformations.euler_from_quaternion([data.orientation.x,data.orientation.y,data.orientation.z,data.orientation.w],'rzyx') #data.y, data.z, data.w], 'rzyx')   # convert imu orientation from quaternion to euler
        if angles[1] < -self.slope_offset : #offset of 1 degree
            self.pubslopeDirection.publish(1) #slope is upwards
        elif angles[1] > self.slope_offset:
            self.pubslopeDirection.publish(2) #slope downwards
        else:
            self.pubslopeDirection.publish(0) #level ground

        if angles[2] < -self.roll_offset : #offset of 1 degree  //angle 0 =roll,10 degree
            self.pub_roll_Direction.publish(1) #roll left
        elif angles[2] > self.roll_offset:
            self.pub_roll_Direction.publish(2) #roll right
        else:
            self.pub_roll_Direction.publish(0) #robot balance

        if angles[0] != self.yaw_past : #offset of 1 degree 
            self.turn = True #doing turning
        else:
            self.turn = False#not turning
        self.yaw_past = angles[0]



def main():
    #print("Here")
    rospy.init_node ('anto_supervisor')   
    rate = rospy.Rate(1)
    MonitorNode = Monitor()
    try:
        while not rospy.is_shutdown():
            MonitorNode.storage_management()
            MonitorNode.network_status()
            MonitorNode.battery_lvl()
            MonitorNode.emergency_alarm()
            MonitorNode.xavier_monitor()
            MonitorNode.soft_shutdown_process()
            MonitorNode.print_monitor()
            rate.sleep()
    except:
        print("Exception occured!!")
    finally:
        MonitorNode.jetson.close

if __name__ == '__main__':
    main()

