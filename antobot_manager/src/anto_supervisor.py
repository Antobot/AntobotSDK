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

#Description:   This is a python script to 
#                 
#Contact:     zhuang.zhou@antobot.ai

# # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # 

#ROS related
import rospy
from std_msgs.msg import String, Bool,UInt8,Float64,Float32
from sensor_msgs.msg import NavSatFix, Imu
from nav_msgs.msg import Odometry
from anto_bridge_msgs.msg import UInt8_Array,Float32_Array,UInt16_Array ##check syntax
from anto_msgs.srv import softshutdown
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
        self.path = '/mileage' # path where mileage tracker file is saved
        self.filename = 'mileage.txt'
        self.filenameTmp = 'mileageTmp.txt'
        self.vel = 0
        self.angular_vel = 0
        self.driving_state = 0
        self.timestamp = 0 # Used to time how long robot drives straight
        self.lon = None
        self.lat = None
        self.utm = None
        self.previous_utm = None

         # Create directory for file if it doesn't exist
        if not os.path.exists(self.path):
            os.makedirs(self.path)
        
        # If file exists, read mileages from file
          # If not, set mileages as 0
        if os.path.exists(os.path.join(self.path, self.filename)):
            with open(os.path.join(self.path, self.filename)) as f:
                lines = f.readlines()
                self.mileage_temp = float(lines[1]) #mileage from last job
                self.mileage_total_temp = float(lines[4])
                self.mileage_total = float(lines[7])
                if self.mileage_total_temp > self.mileage_total: #the running of the antosupervisor must be earlier then antocartogrpher,so this number will be from last job
                    self.mileage_total = self.mileage_total_temp
        else:
            self.mileage_temp = 0
            self.mileage_total = 0
            self.mileage_total_temp = 0

        self.mileage_past = 0
        self.b_robot_movement = False # default robot is not move
        self.u_robot_movement_cnt = 0

        """ alarm data initialization"""
        self.previous_lati = None
        self.previous_longti = None
        self.wheel_fdbk = None
        self.wheel_cmd = None
        self.force_stop = None
        self.uss_data = None
        self.b_vel_cmd = False    # default no moving command is being sent

        self.anto_bridge_function = False
        
        "Xavier monitor"
        self.jetson = jtop()
        self.jetson.start()



        # ROS subscriber
        self.sub_As_uBat = rospy.Subscriber("/antobridge/Ab_uBat",UInt8_Array,self.battery_callback)             #pass battery voltage to anto_supervisor, it's an array data type
        self.sub_GPS_data = rospy.Subscriber("/am_gps_urcu",NavSatFix,self.GPS_callback)                         #pass GPS data to anto_supervisor, it's a data type of NavSatFix, include data.latitude & data.longitude
        self.sub_wheel_vel_cmd = rospy.Subscriber("/antobridge/wheel_vel",Float32_Array,self.wheel_vel_callback) #pass wheel_vel speed to anto_supervisor, it's a float array data type 
        self.sub_wheel_cmd = rospy.Subscriber("/antobridge/wheel_vel_cmd",Float32_Array,self.wheel_cmd_callback) #pass teleop wheel_vel_cmd to anto_supervisor, it's a float array data type
        self.sub_force_stop = rospy.Subscriber("/antobridge/force_stop",Bool,self.force_stop_callback)           #pass force stop signal to anto_supervisor, it's a boolean type
        self.sub_uss_dist = rospy.Subscriber("/antobridge/uss_dist",UInt16_Array,self.uss_dist_callback)         #pass uss_dist data to anto_supervisor, it's a UInt16 array type
        #self.sub_odom = rospy.Subscriber('/odometry/filtered', Odometry, self.odom_callback)                     #we are not using this for mileage tracker
        self.sub_mode = rospy.Subscriber('/switch_mode', UInt8, self.mode_callback)                                #mode 0:keyboard, 1: app, 2: joystick 3:autonomous 4:go home
        #self.sub_gps_status = rospy.Subscriber('am_gps_urcu_status',UInt8,self.GPS_status_callback)
        # self.sub_mileage_tracker = rospy.Subscriber('/am_nav/distanceTravelled',Float32,self.mileage_callback)
        self.sub_soft_shutdown_button = rospy.Subscriber('/antobridge/soft_shutdown_button',Bool,self.soft_shutdown_callback)
        self.sub_slope_dir = rospy.Subscriber('/imu/data', Imu, self.slope_dir)
        self.imu_calib_status = rospy.Subscriber('/imu_calibration_status',UInt8, self.imu_calib)
        self.recalib_status = rospy.Subscriber('/recalib_status',UInt8,self.imu_recalib)
        
        # ROS Publisher
        self.pubslopeDirection = rospy.Publisher("/imu/slope_direction", UInt8, queue_size=1) #topic to publish the slope direction
        self.pub_cpu_temp = rospy.Publisher("/as/cpu_temp",Float32,queue_size=1)#topic to publish the cpu tempreture
        self.pub_cpu_load = rospy.Publisher("/as/cpu_load",UInt8,queue_size=1)#topic to publish the cpu tempreture
        self.pub_storage = rospy.Publisher("/as/storage",Float64, queue_size = 1)
        #self.pub_auto_mileage = rospy.Publisher("/as/auto_mileage",Float32,queue_size = 1) #data type tbc
        self.pub_total_mileage = rospy.Publisher("/as/total_mileage",Float32,queue_size = 1) #data type tbc
        self.pub_soc = rospy.Publisher("/as/soc",UInt8,queue_size = 1)#percentage soc
        self.pub_As_uBat = rospy.Publisher("/as/As_uBat",Float32,queue_size = 1)
        self.pub_batlvl = rospy.Publisher("/as/batlvl",String,queue_size = 1)
        self.pub_network = rospy.Publisher("/as/network",Bool,queue_size = 1)
        self.pub_GPS_status = rospy.Publisher("/as/GPS_status",Bool,queue_size = 1)
        self.pub_As_uAlarm = rospy.Publisher("/as/alarm",UInt8,queue_size =1)
        self.pub_soft_shutdown_req = rospy.Publisher("/antobridge/soft_shutdown_req", Bool,queue_size = 1)

        # ROS Service
        self.soft_shutdown_client = rospy.ServiceProxy('soft_shutdown_req',softshutdown)
        
    
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

             


    """
    def power_consumption_robot():
        ##if Aurix can send power consumption of robot, pass it through antobridge, here can subscribe and broadcast it
    """
 
    def xavier_monitor(self):
        # with jtop() as jetson:
        #     while jetson.ok():
        #         jtopstats= jetson.stats
        #         #print(type(jtopstats))
        #         self.As_cputemp = jtopstats["Temp CPU"]
        #jetson = jtop()
        #jetson.start()
        jtopstats = self.jetson.stats      
        #jetson.close
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

    """ Nice to have
    def robot_temperature():

    """

    def network_status(self): #works
       #Returns False if it fails to resolve Google's URL, True otherwise#
        try:
            socket.gethostbyname("www.google.co.uk")
            self.As_bNetwork = True
            self.pub_network.publish(self.As_bNetwork)
            #self.pub_network.publish(True)
            #return True
        except:
            self.As_bNetwork = False
            self.pub_network.publish(self.As_bNetwork)
            #self.pub_network.publish(False)
            #return False

    """
    def GPS_status_callback(self,data):

        if data == 4: #maybe syntax error?
            self.As_bGNSS = True #is in fix mode
        else:
            self.As_bGNSS = False
        self.pub_GPS_status.publish(self.As_bGNSS)


        #return self.As_bGNSS
    """

   
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
        #receive the all /antobridge/wheel_vel from antobridge = wheel_fdbk
        #receive the all /antobridge/wheel_vel_cmd from antobridge = wheel_cmd
        good_flg = 0
        #GPS_data.latitude = 0
        #GPS_data.longtitude = 0
        # difference_lati = abs(self.lat - self.previous_lati)
        # differenece_longti = abs(self.lon - self.previous_longti)
        # previous_lati = self.lat
        # previous_longti = self.lon
        # lati_threshold = 0 #TBC
        # longti_threshold = 0 #TBC, when stand still, check what's the lati and longti change range
        #how to deal with delay under this algorithm
        self.As_uAlarm = 0

        if self.force_stop == True:
            self.As_uAlarm = 2

        if self.anto_bridge_function == False: #anto_bridge not work (either node die or spi communication fail)
            self.As_uAlarm = 3
        elif self.b_vel_cmd == True:
            if self.u_robot_movement_cnt > 3:
                self.As_uAlarm = 1 # alarm 1, robot stuck some where


        self.anto_bridge_function = False #set this flag to false before next antobridge topic callback


        # #this for loop maybe has some issue
        # for i in self.wheel_cmd:
        #     if i > 0.5: #what's the minimum speed cmd?
        #         for j in self.wheel_fdbk:
        #             if j >0.5 : #robot wheel is moving
        #                 good_flg = good_flg+1
        #                 if self.As_bRTK and self.As_bGNSS == True:
        #                     if difference_lati < lati_threshold and differenece_longti < longti_threshold: #The robot not move
        #                         self.As_uAlarm = 1 # alarm 1, robot stuck some where
        #             else: #cmd has sent but wheel not move
        #                 if self.force_stop == True:
        #                     if self.uss_data[1] < 25 and self.uss_data[5] < 25: #25 is a distance threshold that indicating obstacle are blocking both front and back of the robot 
        #                         self.As_uAlarm = 2
        #                 else: #force stop not triggered
        #                     self.As_uAlarm = 3 #mainly harware issue, wheel/motor stop working
        #self.As_uAlarm = 0
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

        """
        sys.stdout.write("self.As_sBatlvl:" + self.As_sBatlvl + "\n")
        sys.stdout.write("self.As_fPwr:" + self.As_fPwr + "\n")
        sys.stdout.write("self.As_uCPUtemp:" + self.As_uCPUtemp + "\n")
        sys.stdout.write("self.As_uUtil:" + self.As_uUtil + "\n")
        sys.stdout.write("self.As_b4G:" + self.As_b4G + "\n")
        sys.stdout.write("self.As_Wifi:" + self.As_Wifi + "\n")
        sys.stdout.write("self.bEthernet:" + self.bEthernet + "\n")
        sys.stdout.write("self.As_bRTK:" + self.As_bRTK + "\n")
        sys.stdout.write("self.As_bGNSS:" + self.As_bGNSS + "\n")
        sys.stdout.write("self.As_bStorage:" + self.As_bStorage + "\n")
        sys.stdout.write("self.As_uAlarm:" + self.As_uAlarm + "\n")
        sys.stdout.write("self.As_bStorage:" + self.As_bStorage + "\n")
        """

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

        #print(self.As_sBatlvl)
        #return self.As_sBatlvl

    def GPS_callback(self,gps_msg):
        #print("sending GPS data from callback")
        #GPS_data = data
        #record mileage using GPS
        self.lat = gps_msg.latitude
        self.lon = gps_msg.longitude
        #print("GPS topic received")
        utmx, utmy, utmzone = gc.LLtoUTM(self.lat, self.lon) #convert to UTM coordinates
        self.utm = [utmx, utmy, utmzone]
        if gps_msg.status == 2: #2 = fix mode
            self.As_bGNSS = True
        else:                   #0 = neither fixed or float, 1 =  float mode
            self.As_bGNSS = False 
        
        """
        if self.previous_utm is not None:
            if self.utm[2] == self.previous_utm[2]: # if both points are in the same utm zone
                dist = math.sqrt((self.utm[0] - self.previous_utm[0])**2 + (self.utm[1] - self.previous_utm[1])**2)
                self.total_dist += dist
                if self.uMode >= 3: # autonomous mode
                    self.auto_dist += dist
        self.previous_utm = self.utm
        """
    def mileage_callback(self,mileage):
        self.mileage_temp = mileage.data
        if self.mileage_temp == self.mileage_past:
            self.b_robot_movement = False
            self.u_robot_movement_cnt = self.u_robot_movement_cnt+1
        else:
            self.b_robot_movement = True
            self.u_robot_movement_cnt = 0
        self.mileage_past = self.mileage_temp

        if self.mileage_temp ==0:
            self.mileage_total = self.mileage_total_temp
        self.mileage_total_temp = self.mileage_temp + self.mileage_total
        self.pub_total_mileage.publish(self.mileage_total_temp)



    def writeToFile(self):
        lines = ['mileage for current job(m):\n', str(self.mileage_temp), '\n\n', 'Total autonomous mileage include current job(m):\n', str(self.mileage_total_temp), '\n\n', 'Total autonomous mileage history(m):\n', str(self.mileage_total)]
        with open(os.path.join(self.path, self.filenameTmp), 'w') as f: #if it not exist, need to create first
            f.writelines(lines) # write mileages to temporary file
        copyfile(os.path.join(self.path, self.filenameTmp), os.path.join(self.path, self.filename)) # copy mileages from temporary file to permanent file
        os.remove(os.path.join(self.path, self.filenameTmp)) # delete temporary file
        #self.pub_auto_mileage.publish(self.auto_dist)
        
        

    def wheel_vel_callback(self,data):
        #print("sending wheel velocity data from callback")
        self.anto_bridge_function = True
        #print("wheel_fdbk:",self.wheel_fdbk[1])
        return

    def wheel_cmd_callback(self,data):
        #print("sending teleop speed command data from callback")
        self.wheel_cmd = data.data
        if self.wheel_cmd[0] !=0 or self.wheel_cmd[1] !=0 or self.wheel_cmd[2] !=0 or self.wheel_cmd[3] !=0: #command any  of the wheel move
            self.b_vel_cmd = True  #sent wheel moving cmd 
        else:
            self.b_vel_cmd = False 
        print("wheel_vel_cmd sent:",self.b_vel_cmd)
        return

    def force_stop_callback(self,data):
        #print("sending force stop signal from callback")
        self.force_stop = data.data #True as triggered, false as not triggered
        return

    def uss_dist_callback(self,data):
        #print("sending force stop signal from callback")
        self.uss_data = data 
        return

    def slope_dir(self,data):
        angles = tf.transformations.euler_from_quaternion([data.orientation.x,data.orientation.y,data.orientation.z,data.orientation.w],'rzyx') #data.y, data.z, data.w], 'rzyx')   # convert imu orientation from quaternion to euler
        #print("pitch",angles[1]*180/3.14)
        #print("yaw",angles[0])
        #print("rosll",angles[2])
        if angles[1] < -self.slope_offset : #offset of 1 degree
            self.pubslopeDirection.publish(1) #slope is upwards
        elif angles[1] > self.slope_offset:
            self.pubslopeDirection.publish(2) #slope downwards
        else:
            self.pubslopeDirection.publish(0) #level ground

    
"""
if __name__ == '__main__':
    rospy.init_node ('Monitor_node')
    MonitorNode = Monitor()
    rate = rospy.Rate(1)

    while not rospy.is_shutdown():
        
        
        #MonitorNode.battery_callback()
        MonitorNode.storage_management()
        MonitorNode.network_status()
        MonitorNode.battery_lvl()

        #MonitorNode.distbyGPS()
        MonitorNode.writeToFile()

        MonitorNode.xavier_monitor()

        MonitorNode.print_monitor()
        MonitorNode.writeToFile()

        rate.sleep()
"""

def main():
    #print("Here")
    rospy.init_node ('anto_supervisor')
    
    rate = rospy.Rate(1)
    MonitorNode = Monitor()
    #MonitorNode.print_monitor()
    #MonitorNode.writeToFile()
    #MonitorNode = Monitor()
    try:
        while not rospy.is_shutdown():
             #MonitorNode.battery_callback()
            MonitorNode.storage_management()
            MonitorNode.network_status()
            MonitorNode.battery_lvl()

            #MonitorNode.distbyGPS()
            MonitorNode.writeToFile()
            MonitorNode.emergency_alarm()
            MonitorNode.xavier_monitor()
            MonitorNode.soft_shutdown_process()
            MonitorNode.print_monitor()
            #MonitorNode.writeToFile()
            rate.sleep()
    except:
        print("Exception occured!!")
    finally:
        MonitorNode.jetson.close

if __name__ == '__main__':
    main()

