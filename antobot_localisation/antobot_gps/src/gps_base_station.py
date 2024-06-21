#!/usr/bin/env python3
# Copyright (c) 2023, ANTOBOT LTD.
# All rights reserved

# # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # 

# # # Code Description:     The purpose of this code is to process the GPS data received via SPI from the Ublox F9P chip
# # #                       and publish a GPS message as a rostopic using this data.
# # #                       This script is used when base station is used to get the 433MHz correction messages

#This script reports the following on GPS status:
# GPS status : Critical ; GPS status = 0
# GPS status : Warning ;GPS status = 1 - Float
# GPS status : Good ; GPS status = 3 - Fix

# Contact: Aswathi Muralidharan 
# email: aswathi.muralidharan@antobot.ai

# # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # #
import rospy
import rospkg
import spidev
import sys
import time
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import TwistWithCovarianceStamped
from std_msgs.msg import UInt8, Float32
from antobot_gps.ublox_gps import UbloxGps

class F9P_GPS:

    def __init__(self):
        #GPS class initialisation 
        #self.baud = 460800# 38400
        self.port = spidev.SpiDev()
        self.gps_spi = UbloxGps(self.port)

        self.fix_status = 0
        self.gps_status = "Critical"
        self.gps_time_buf = []


    def uart2_config(self,baud):
        #set the baud rate of uart2 to 38400
        self.gps_spi.ubx_set_val(0x40530001,baud)
        #set the uart2 enable true
        self.gps_spi.ubx_set_val(0x10530005,0x01) #cfg-uart2-enable

        
    def get_gps(self):
        #function to parse and publish the UBX parser GPS coordinates
        # mode = 1: RTK base station, 2: PPP-IP, 3: LBand
                
        # Get the data from the F9P
        self.geo = self.gps_spi.geo_coords()

    def get_fix_status(self):
        h_acc = 75

        if self.geo.flags.carrSoln == 2:  #fix mode =2 ; float mode = 1
            self.fix_status = self.geo.fixType #3: 3Dfix, 2:2Dfix

            if self.fix_status == 3 and self.gps_status != 'Good':
                rospy.loginfo("SN4010: GPS Fix Status: Fixed Mode")
                self.gps_status = 'Good'

        elif self.geo.flags.carrSoln == 1: #float conditions
            #PPP-IP can show float even if the horizontal accuracy is good, so adding another loop to check the fix mode
            if self.geo.hAcc < h_acc:
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

## Example function

def main(args):
    gps_freq_status = "Critical"

    try:
        # init node
        rospy.init_node('rtk', anonymous=True)
        rate = rospy.Rate(8)  # 8hz
        gps_f9p = F9P_GPS()

        baudrate_rtk = 38400
        gps_f9p.uart2_config(baudrate_rtk)

        gps_pub = rospy.Publisher('antobot_gps', NavSatFix, queue_size=10)
        gpsfix = NavSatFix()
        gpsfix.header.stamp = rospy.Time.now()
        gpsfix.header.frame_id = 'gps_frame'  # FRAME_ID
        gpsfix.position_covariance_type = NavSatFix.COVARIANCE_TYPE_APPROXIMATED
        #last_print = time.monotonic()

        mode = 1 # 1: RTK base station; 2: PPP-IP; 3: LBand
        while not rospy.is_shutdown():
            gps_f9p.get_gps()

            # Check the new data is viable and update message
            if gps_f9p.geo.lat is not None and gps_f9p.geo.lat != 0:                 
                gpsfix.latitude = gps_f9p.geo.lat 
                gpsfix.longitude = gps_f9p.geo.lon 
                gpsfix.altitude = gps_f9p.geo.height
                
                # Get GPS fix status
                gpsfix.status.status = gps_f9p.get_fix_status()

                # Assumptions made on covariance
                gpsfix.position_covariance[0] = (gps_f9p.geo.hAcc*0.001)**2 
                gpsfix.position_covariance[4] = (gps_f9p.geo.hAcc*0.001)**2 
                gpsfix.position_covariance[8] = (4*gps_f9p.geo.hAcc*0.001)**2 

                # Update the navsatfix messsage
                current_time = rospy.Time.now()
                gps_time_i=(current_time.to_sec()-gpsfix.header.stamp.to_sec())
                gpsfix.header.stamp = current_time

                # Create a buffer to find the average frequency
                time_buf_len = 10
                gps_f9p.gps_time_buf.append(gps_time_i)
                if len(gps_f9p.gps_time_buf) > time_buf_len:
                    gps_f9p.gps_time_buf.pop(0)

                # Inverted average time to calculate hertz
                gps_hz = len(gps_f9p.gps_time_buf) / sum(gps_f9p.gps_time_buf)        

                if gps_f9p.geo.hAcc < 500:
                    gps_pub.publish(gpsfix)

                #rospy.loginfo(f'GPS Frequency: {self.gps_hz} Hz')
                if gps_hz < 2 and gps_freq_status != "Critical":
                    rospy.logerr("SN4012: GPS Frequency status: Critical (<2 hz)")
                    gps_freq_status = "Critical"
                elif gps_hz >=2 and gps_hz < 6 and gps_freq_status != "Warning":
                    rospy.logwarn("SN4012: GPS Frequency status: Warning (<6 hz)")
                    gps_freq_status = "Warning"
                elif gps_hz >= 6 and gps_freq_status != "Good":
                    rospy.loginfo("SN4012: GPS Frequency status: Good (>6 hz)")
                    gps_freq_status = "Good"

            rate.sleep()
    except rospy.ROSInterruptException:
        pass


if __name__ == '__main__':   
    main(sys.argv)

