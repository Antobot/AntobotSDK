#!/usr/bin/env python3
# Copyright (c) 2021, ANTOBOT LTD.
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

# # # Code Description:     The purpose of this code is to process the GPS data received via SPI from the Ublox F9P chip
# # #                       and publish a GPS message as a rostopic using this data.

# Contact: mert.turanli@antobot.co.uk

# # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # #



import rospy
import rospkg
import spidev
import time
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import TwistWithCovarianceStamped
from std_msgs.msg import UInt8
from am_gps_urcu.ublox_gps import UbloxGps

if __name__ == '__main__':
    try:
        # init node
        rospy.init_node('rtk', anonymous=True)

        gps_pub = rospy.Publisher('am_gps_urcu', NavSatFix, queue_size=10)
        #gps_vel_pub = rospy.Publisher('am_gps_urcu_vel', TwistWithCovarianceStamped, queue_size=10)
        #gps_status_pub = rospy.Publisher('am_gps_urcu_status', UInt8, queue_size=10)
        rate = rospy.Rate(10)  # 10hz
        baud = 38400

        port = spidev.SpiDev()
        gps = UbloxGps(port)
        
        #set the baud rate of uart2 to 38400
        baud_uart2 = gps.ubx_set_val(0x40530001,baud)
        #print("baud_uart2",baud_uart2)

        #set the uart2 enable true
        set_uart2 = gps.ubx_set_val(0x10530005,0x01) #cfg-uart2-enable
        #print("set_uart2",set_uart2)
        

        gpsfix = NavSatFix()
        gpsfix.header.stamp = rospy.Time.now()
        gpsfix.header.frame_id = 'gps_frame'  # FRAME_ID
        gpsfix.position_covariance_type = NavSatFix.COVARIANCE_TYPE_APPROXIMATED

        #last_print = time.monotonic()

        while not rospy.is_shutdown():
 
            try:


                #https://cdn.sparkfun.com/assets/learn_tutorials/1/1/7/2/ZED-F9R_Interfacedescription__UBX-19056845_.pdf

                
                #geo=gps.hp_geo_coords() # High precision
                geo = gps.geo_coords() #to get the fix type


                # Publish position update
                #if geo.latHp is not None and geo.latHp != 0:
                if geo.lat is not None and geo.lat != 0:


                    
                    
                    gpsfix.latitude = geo.lat #+ geo.latHp
                    gpsfix.longitude = geo.lon #+ geo.lonHp
                    gpsfix.altitude = geo.height #+ geo.heightHp


                    #gpsfix.status.status = geo.flags[4] #fix mode =2 ; float mode = 1


                    gpsfix.status.status = geo.fixType
                    print("carrSoln: ",geo.fixType)
                    # Assumptions made on covariance
                    gpsfix.position_covariance[0] = (geo.hAcc*0.1)**2 #std_dev error position estimate = 0.1
                    gpsfix.position_covariance[4] = (geo.hAcc*0.1)**2 #std_dev error position estimate = 0.1
                    gpsfix.position_covariance[8] = (4*geo.hAcc*0.1)**2 #std_dev error position estimate = 0.1
                    gps_pub.publish(gpsfix)
                    #gpsfix.status = geo.fixType
                    #gps_status_pub.publish(geo.fixType)

            except (ValueError, IOError) as err:
                print(err)

            rate.sleep()

 

    except rospy.ROSInterruptException:
        pass

