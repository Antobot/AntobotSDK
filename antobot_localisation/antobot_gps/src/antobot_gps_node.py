#!/usr/bin/env python3
# Copyright (c) 2023, ANTOBOT LTD.
# All rights reserved

# # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # 

# # # Code Description:     The purpose of this code is to process the GPS data received via SPI from the Ublox F9P chip
# # #                       and publish a GPS message as a rostopic using this data.

# Contact: Aswathi Muralidharan 
# email: aswathi.muralidharan@antobot.ai

# # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # #
import rospy
import rospkg
import spidev
import time
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import TwistWithCovarianceStamped
from std_msgs.msg import UInt8, Float32
from antobot_gps.ublox_gps import UbloxGps

if __name__ == '__main__':
    try:
        # init node
        rospy.init_node('rtk', anonymous=True)
        gps_pub = rospy.Publisher('antobot_gps', NavSatFix, queue_size=10)
        rate = rospy.Rate(10)  # 10hz
        baud = 38400
        port = spidev.SpiDev()
        gps = UbloxGps(port)
        #set the baud rate of uart2 to 38400
        baud_uart2 = gps.ubx_set_val(0x40530001,baud)
        #set the uart2 enable true
        set_uart2 = gps.ubx_set_val(0x10530005,0x01) #cfg-uart2-enable
        gpsfix = NavSatFix()
        gpsfix.header.stamp = rospy.Time.now()
        gpsfix.header.frame_id = 'gps_frame'  # FRAME_ID
        gpsfix.position_covariance_type = NavSatFix.COVARIANCE_TYPE_APPROXIMATED
        #last_print = time.monotonic()
        while not rospy.is_shutdown():
            try:                
                geo = gps.geo_coords() #to get the fix type
                # Publish position update
                if geo.lat is not None and geo.lat != 0:                 
                    gpsfix.latitude = geo.lat 
                    gpsfix.longitude = geo.lon 
                    gpsfix.altitude = geo.height
                    print("gps carrsoln",geo.flags.carrSoln)
                    if geo.flags.carrSoln == 2:  #fix mode =2 ; float mode = 1
                        gpsfix.status.status = geo.fixType
                    #print("carrSoln: ",geo.flags[4])
                    # Assumptions made on covariance
                    gpsfix.position_covariance[0] = (geo.hAcc*0.001)**2 
                    gpsfix.position_covariance[4] = (geo.hAcc*0.001)**2 
                    gpsfix.position_covariance[8] = (4*geo.hAcc*0.001)**2 
                    gps_pub.publish(gpsfix)
            except (ValueError, IOError) as err:
                print(err)
            rate.sleep()
    except rospy.ROSInterruptException:
        pass

