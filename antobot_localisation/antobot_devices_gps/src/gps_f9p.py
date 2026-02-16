#!/usr/bin/env python3
# Copyright (c) 2023, ANTOBOT L
# All rights reserved

# # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # 

# # # Code Description:     The purpose of this code is to process the GPS data received via SPI from the Ublox F9P chip
# # #                       and publish a GPS message as a rostopic using this data.

#This script reports the following on GPS status:
# GPS status : Critical ; GPS status = 0
# GPS status : Warning ;GPS status = 1 - Float
# GPS status : Good ; GPS status = 3 - Fix

# Contact: Daniel Freer 
# email: daniel.freer@antobot.ai

# # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # #

import rospy
import rospkg
import spidev
import sys
import time
from datetime import datetime
import pynmea2
import yaml
import serial

from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import TwistWithCovarianceStamped
from std_msgs.msg import UInt8, Float32, String
#from antobot_devices_msgs.msg import gpsQual
from antobot_devices_gps.ublox_gps import UbloxGps

class F9P_GPS:


    def __init__(self, dev_type, serial_port=None, method="stream", pub_name="antobot_gps", pub_name_qual="antobot_gps/quality"):

        # # # GPS class initialisation
        #     Inputs: dev_type - the device type of the F9P chip. 
        #           "urcu" - if using the F9P inside of the URCU
        #           "usb" - if using an external F9P conncected via USB

        self.node_type = "gps_f9p"

        self.gpsfix = NavSatFix()
        self.gpsfix.header.frame_id = 'gps_frame'  # FRAME_ID
        self.message = "GGA" #or"GNS"
        self.dev_type = dev_type
        self.method = method
        self.poll_buff = 1
        self.poll_buff_pre =1
        if self.dev_type == "urcu":
            self.port = spidev.SpiDev()
        elif self.dev_type == "usb":
            if serial_port == None:
                self.baud = 460800 # 38400?? 460800?Need to resolve baudrate difference with baudrate_rtk below
                self.port = serial.Serial("/dev/ttyUSB0", self.baud,timeout=2)
            else:
                self.port = serial_port
        self.gps_dev = UbloxGps(self.port)
        self.geo = None
        self.fix_status = 0                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                              
        self.gps_status = "Critical"
        self.gps_freq_status = "Critical"
        self.gps_time_buf = []
        self.hAcc = 500
        self.h_acc_thresh = 0.1  # 
       
        current_time = rospy.Time.now()
        self.gps_time_i=0.1
        self.gpsfix.header.stamp = current_time
        self.gps_timestamp = current_time
        self.gps_time_offset=2
        # Initial parameters for quality
        self.geo_sep = 0
        self.cogt = 0
        self.sogk = 0
        self.gps_hz = 0

        self.gps_pub = rospy.Publisher(pub_name, NavSatFix, queue_size=10)
        #self.gps_qual_pub = rospy.Publisher(pub_name_qual, gpsQual, queue_size=10)
        self.gga_msg_pub=rospy.Publisher("/antobot_gps/gga", String, queue_size=10)

        return


    def uart2_config(self,baud):
        #set the baud rate of uart2 to appropriate value (38400?)
        self.gps_dev.ubx_set_val(0x40530001,baud)
        #set the uart2 enable true
        self.gps_dev.ubx_set_val(0x10530005,0x01) #cfg-uart2-enable

        
    def get_gps(self,event=None):
        # Get the data from the F9P
        if self.method == "poll":
            self.geo = self.gps_dev.geo_coords() #poll method
            self.hAcc=self.geo.hAcc
            if  self.geo.lat is not None and self.geo.lat != 0:
                self.create_gps_msg_poll()
                self.get_gps_freq()
                if self.hAcc < 500:
                    self.gps_pub.publish(self.gpsfix)
                    
        if self.method == "stream":
            if self.dev_type =="urcu":
                streamed_data = self.gps_dev.stream_nmea(self.poll_buff) #.decode('utf-8') #stream method
            if self.dev_type == "usb":
                streamed_data = self.gps_dev.stream_nmea(self.poll_buff) #.decode('utf-8')  self.poll_buff
            self.get_gps_quality(streamed_data)


            #print(streamed_data)


            # Check the new data is viable and update message
            if self.correct_gps_format(streamed_data):                
                self.create_gps_msg()
                self.get_gps_freq()

                #self.create_quality_msg()   
                if self.hAcc < 5000:
                    self.gps_pub.publish(self.gpsfix)

    

    def correct_gps_format(self, streamed_data):
        # Function to check whether the streamed data matches the desired 
        if self.message == "GGA":
            if isinstance(streamed_data,str) and streamed_data.startswith("$GNGGA"):
                self.geo = pynmea2.parse(streamed_data)
                return True
        if self.message == "GNS":
            if isinstance(streamed_data,str) and streamed_data.startswith("$GNGNS"):
                self.geo = pynmea2.parse(streamed_data)
                self.fix_status = 4
                return True

        return False

    def get_fix_status(self):
        # print(self.geo.gps_qual)
        if self.geo.gps_qual == 4: 
            if self.gps_status != 'Good':
                rospy.loginfo("SN4010: GPS Fix Status: Fixed Mode")
                self.gps_status = 'Good'
                self.fix_status = 3
        elif self.geo.gps_qual == 2 or self.geo.gps_qual == 5:
            if self.hAcc < self.h_acc_thresh:
                self.fix_status = 3
                if self.gps_status != 'Good':
                    rospy.loginfo("SN4010: GPS Fix Status: Fixed Mode")
                    self.gps_status = 'Good'
            else:   
                self.fix_status = 1
                if self.gps_status != 'Warning':
                    rospy.logwarn("SN4010: GPS Fix Status: Float Mode")
                    self.gps_status = 'Warning'
        else:
            self.fix_status = 0 #no fix
            if self.gps_status != 'Critical':
                rospy.logerr("SN4010: GPS Fix Status: Critical")
                self.gps_status = 'Critical'
        
        return self.fix_status

    def get_fix_status_poll(self):
        h_acc = 75

        if self.geo.flags.carrSoln == 2:  #fix mode =2 ; float mode = 1
            self.fix_status = self.geo.fixType #3: 3Dfix, 2:2Dfix

            if self.fix_status == 3 and self.gps_status != 'Good':
                rospy.loginfo("SN4010: GPS Fix Status: Fixed Mode")
                self.gps_status = 'Good'

        elif self.geo.flags.carrSoln == 1: #float conditions
            #PPP-IP can show float even if the horizontal accuracy is good, so adding another loop to check the fix mode
            if self.hAcc < self.h_acc_thresh:
                self.fix_status = 3
                if self.gps_status != 'Good':
                    rospy.loginfo("SN4010: GPS Fix Status: Fixed Mode")
                    self.gps_status = 'Good'
            elif self.geo.hAcc > h_acc :
                self.fix_status = 1
                if self.gps_status != 'Warning':
                    rospy.logwarn("SN4010: GPS Fix Status: Float Mode")
                    self.gps_status = 'Warning'

        else:
            self.fix_status = 0 #no fix
            if self.gps_status != 'Critical':
                rospy.logerr("SN4010: GPS Fix Status: Critical")
                self.gps_status = 'Critical'

        return self.fix_status

    def create_gps_msg(self):

        self.gpsfix.position_covariance_type = NavSatFix.COVARIANCE_TYPE_APPROXIMATED

        self.gpsfix.altitude = 0
        self.gpsfix.latitude = 0
        self.gpsfix.longitude = 0
        if self.message == "GGA":
             if  self.geo.latitude is not None and self.geo.latitude != 0:
                self.gpsfix.latitude = self.geo.latitude
                self.gpsfix.longitude = self.geo.longitude

                self.gpsfix.altitude = self.geo.altitude
        
        # Get GPS fix status
        self.gpsfix.status.status = self.get_fix_status()

        # Assumptions made on covariance
        self.gpsfix.position_covariance[0] = (self.hAcc)**2 
        self.gpsfix.position_covariance[4] = (self.hAcc)**2 
        self.gpsfix.position_covariance[8] = (4*self.hAcc)**2 

        # Set the time of the GPS message
        self.set_gps_msg_time()
        
        return

    def create_gps_msg_poll(self):

        self.gpsfix.position_covariance_type = NavSatFix.COVARIANCE_TYPE_APPROXIMATED

        self.gpsfix.altitude = 0

        self.gpsfix.latitude = self.geo.lat
        self.gpsfix.longitude = self.geo.lon
        self.gpsfix.altitude = self.geo.height
        
        # Get GPS fix status
        self.gpsfix.status.status = self.get_fix_status_poll()

        # Assumptions made on covariance  ###hAcc unit might be different, tbd
        self.gpsfix.position_covariance[0] = (self.hAcc*0.001)**2 
        self.gpsfix.position_covariance[4] = (self.hAcc*0.001)**2 
        self.gpsfix.position_covariance[8] = (4*self.hAcc*0.001)**2 

        # Set the time of the GPS message
        self.set_gps_msg_time()

        return

    def set_gps_msg_time(self):

        # Getting time
        current_time = rospy.Time.now()
        dt0 = self.get_gps_timestamp_utc()
        #print("current time (ROS): {}".format(current_time.to_sec()))
        #print("datetime timestamp: {}".format(dt0.timestamp()))
        if (dt0!=None):
            self.gps_time_i=(dt0.timestamp()-self.gpsfix.header.stamp.to_sec())
            self.gps_time_offset = current_time.to_sec() - dt0.timestamp()      # Calculating offset between current time and GPS timestamp
            
            if dt0.timestamp() > current_time.to_sec():
                rospy.logerr("SN4013: GPS time is {}s ahead of the system time".format(self.gps_time_offset))
            # # Assigning timestamp part of NavSatFix message
            self.gps_timestamp = rospy.Time.from_sec(dt0.timestamp())
            self.gpsfix.header.stamp = self.gps_timestamp            # Assigning time received from F9P
        # self.gpsfix.header.stamp = current_time.to_sec()       # Assigning current time (ROS) - DEPRECATED
        else:
            self.gps_time_offset == 99
        if self.gps_time_offset > 0.5 and self.gps_time_offset != 99:
            rospy.logerr("SN4013: GPS time offset is high: {}s".format(self.gps_time_offset))
            self.poll_buff =(self.gps_time_offset//0.125)*3
            if self.poll_buff_pre !=1 and  self.poll_buff_pre!= 24 and self.poll_buff_pre!= 3:
                self.poll_buff = 3            
        elif self.gps_time_offset !=99:
            self.poll_buff = 1
        else:
            self.poll_buff = 24
        self.poll_buff_pre=self.poll_buff
        
        #print("pulled sentence:",self.poll_buff)            
        
    def get_gps_timestamp_utc(self):

        today_date = datetime.today()
        year=today_date.year
        month=today_date.month
        day=today_date.day
        try:
            hour_i = self.geo.timestamp.hour
            minute_i = self.geo.timestamp.minute
            second_i = self.geo.timestamp.second
            mic_sec_i = self.geo.timestamp.microsecond
            dt0 = datetime(year, month, day, hour=hour_i, minute=minute_i, second=second_i, microsecond=mic_sec_i)
            return dt0 
        except:
            print("GPS timestamp invalid")
        

        

    def get_gps_freq(self):
        # # # Gets the frequency of the published GPS message and sends a message if there has been a significant change

        # Create a buffer to find the average frequency
        time_buf_len = 10
        self.gps_time_buf.append(self.gps_time_i)
        if len(self.gps_time_buf) > time_buf_len:
            self.gps_time_buf.pop(0)

        # Inverted average time to calculate hertz
        gps_hz = len(self.gps_time_buf) / sum(self.gps_time_buf)
        self.gps_hz = gps_hz

        #rospy.loginfo(f'GPS Frequency: {self.gps_hz} Hz')
        if gps_hz < 2 and self.gps_freq_status != "Critical":
            rospy.logerr("SN4012: GPS Frequency status: Critical (<2 hz)")
            self.gps_freq_status = "Critical"
        elif gps_hz >=2 and gps_hz < 6 and self.gps_freq_status != "Warning":
            rospy.logwarn("SN4012: GPS Frequency status: Warning (<6 hz)")
            self.gps_freq_status = "Warning"
        elif gps_hz >= 6 and self.gps_freq_status != "Good":
            rospy.loginfo("SN4012: GPS Frequency status: Good (>6 hz)")
            self.gps_freq_status = "Good" 

    def get_gps_quality(self, streamed_data):

        if isinstance(streamed_data,str):
            if streamed_data.startswith("$GNGST"):
                gst_parse = pynmea2.parse(streamed_data)

                try:
                    self.hAcc=((gst_parse.std_dev_latitude)**2+(gst_parse.std_dev_longitude)**2)**0.5
                except:
                    print("hAcc is invalid")

                #print(self.hAcc)
            if streamed_data.startswith("$GNGGA"):
                if self.gps_time_offset < 0.5:
                    self.gga_msg_pub.publish(streamed_data)
                gga_parse = pynmea2.parse(streamed_data)
                try:
                    self.gga_gps_qual = int(gga_parse.gps_qual)
                    self.num_sats = int(gga_parse.num_sats)         # Number of satellites
                except:
                    print("GPS_quality value invalid")
                
                try:
                    self.hor_dil = float(gga_parse.horizontal_dil)  # Horizontal dilution of precision (HDOP)
                except:
                    print("hor_dil value invalid") 
                try:
                    self.geo_sep = float(gga_parse.geo_sep)         # Geoid separation
                except:
                    print("Geoid separation value invalid")
            if streamed_data.startswith("$GNGNS"):
                gns_parse = pynmea2.parse(streamed_data)
                # self.pos_mode = int(gns_parse.mode_indicator)

                try:
                    self.num_sats = int(gns_parse.num_sats)             # Number of satellites
                    self.hor_dil = float(gns_parse.hdop)                # Horizontal dilution of precision (HDOP)
                    self.geo_sep = float(gns_parse.geo_sep)         # Geoid separation
                except:
                    print("GNS information invalid")
            if streamed_data.startswith("$GNGSA"):      # Full satellite information
                gsa_parse = pynmea2.parse(streamed_data)
                # Add parser here (?)
            if streamed_data.startswith("$GNVTG"):      # Velocity
                vtg_parse = pynmea2.parse(streamed_data)
                try:
                    self.cogt = float(vtg_parse.true_track)                  # Course over ground (true)
                    # self.cogm = vtg_parse.mag_track                 # Course over ground (magnetic)
                    # self.sogn = vtg_parse.spd_over_grnd_kts         # Speed over ground (knots)
                    self.sogk = float(vtg_parse.spd_over_grnd_kmph)          # Speed over ground (km/h)
                except TypeError:
                    pass
                    #print("VTG information invalid")
                # TODO: Calculate ENU velocity
            

        return

    def create_quality_msg(self):
        gpsQualMsg = gpsQual()
        gpsQualMsg.stamp = self.gps_timestamp
        gpsQualMsg.t_offset = self.gps_time_offset
        gpsQualMsg.hAcc = self.hAcc
        gpsQualMsg.gpsQualVal = self.gga_gps_qual
        gpsQualMsg.numSats = self.num_sats
        gpsQualMsg.horDil = self.hor_dil
        gpsQualMsg.geoSep = self.geo_sep
        # gpsQualMsg.satInfo = ???
        gpsQualMsg.vCOG = self.cogt
        gpsQualMsg.vSOG = self.sogk
        gpsQualMsg.frequency = self.gps_hz
        self.gps_qual_pub.publish(gpsQualMsg)


def main(args):
    mqtt_publish = False

    # init node
    rospy.init_node('rtk', anonymous=True)
    
    gps_f9p = F9P_GPS("urcu")

    baudrate_rtk = 460800 #38400            # Need to resolve baudrate
    gps_f9p.uart2_config(baudrate_rtk)

    mode = 2 # 1: RTK base station; 2: PPP-IP; 3: LBand
    

    if gps_f9p.method == "poll":
        gpsRate=8  # 8hz
    if gps_f9p.method == "stream":
        gpsRate=50  # 50hz

    rospy.Timer(rospy.Duration(1/gpsRate), gps_f9p.get_gps)  # Runs periodically without blocking
    rospy.spin() 




if __name__ == '__main__':   
    main(sys.argv)

