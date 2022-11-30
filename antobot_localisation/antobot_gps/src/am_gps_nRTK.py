#!/usr/bin/env python3
"""
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

# # # Code Description:     This code is used to connect to ublox MQTT server to subscribe the GNSS augmentation data and implement nRTK mode in ublox f9p chip. Currently this code was tested in the Ublox chip : "ZED-F9P-01B-01" (GNSS Base 3) with firmware upgraded to HPG1.32.
#  The chip was connected to the system via USB. To run the script at 8Hz, the following modifications are made in the f9p chip.
                                    CFG-RATE-MEAS = 125 sec
                                    VALSET: CFG-MSGOUT-NMEA-ID_GSV_UART1 = 0
                                    VALSET: CFG-MSGOUT-NMEA-ID_GSA_UART1 = 0
                                    VALSET: CFG-MSGOUT-NMEA-ID_GLL_UART1 = 0

# Contact: aswathi.muralidharan@antobot.ai

# # # #  # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # 
"""

import rospy
import ssl
#import struct
import time
import sys

import paho.mqtt.client as mqtt
import serial
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import TwistWithCovarianceStamped
from std_msgs.msg import UInt8


class nRTK:
    
    def __init__(self):
        
        print("Check point 1")

        self.echo = True
        self.latitude = 0.0
        self.longitude = 0.0
        self.fix_quality = 0
        self.satellites = 0
        self.horizontal_dilution = 0

        self.altitude_m = 0
        self.height_geoid = 0


        self.client_id = '58bab427-cbcd-4954-ab76-b49d68d8893d' #from the pointperfect platform
        self.server = 'pp.services.u-blox.com'
        self.connect = False
        
        self.gnss = serial.Serial(port='/dev/ttyUSB0', baudrate=460800, timeout=0.1) #baudrate=38400

        # Topic names and QoS
        self.mqtt_topics = [(f"/pp/ip/uk", 0), ("/pp/ubx/mga", 0), ("/pp/ubx/0236/ip", 0)]
        
        #print("The mqtt topics",self.mqtt_topics)

        self.userdata = { 'gnss': self.gnss, 'topics': self.mqtt_topics }
        #print("User data",userdata)
        self.client = mqtt.Client(client_id=self.client_id, userdata=self.userdata)
        self.client.on_connect = self.on_connect
        self.client.on_message = self.on_message

        #self.connect_broker()

        #self.client.connect(self.server, port=8883)

        self.client.tls_set(certfile=f'device-{self.client_id}-pp-cert.crt', keyfile=f'device-{self.client_id}-pp-key.pem')
        
        self.connect_broker()

    def connect_broker(self):
        try:
            self.client.connect(self.server, port=8883)
            print("1. broker connected") 
        except:
            print("Trying to connect ...")
        time.sleep(2)
        
    # The callback for when the client receives a CONNACK response from the server.
    def on_connect(self,client, userdata, flags, rc):
        print("Check point 2")
        if rc == 0:
            print("Connected to broker; subscribing\n")
            self.client.subscribe(userdata['topics'])
            self.connect = True
        else:
            print("Connection failed!",rc)


    # The callback for when a PUBLISH message is received from the server.
    def on_message(self,client,userdata, msg):
        print("Checkpoint 3",self.connect)
        print('Received', msg.topic, len(msg.payload) )
        
        # send payload to the GNSS receiver over serial
        self.userdata['gnss'].write(msg.payload)

    def parse_degrees(self,nmea_data):
        # Parse a NMEA lat/long data pair 'dddmm.mmmm' into a pure degrees value.
        # Where ddd is the degrees, mm.mmmm is the minutes.
        if nmea_data is None or len(nmea_data) < 3:
            return None
        raw = float(nmea_data)
        deg = raw // 100
        minutes = raw % 100
        return deg + minutes / 60
        
        
    def parse_int(self,nmea_data):
        if nmea_data is None or nmea_data == "":
            return None
        return int(nmea_data)


    def parse_float(self,nmea_data):
        if nmea_data is None or nmea_data == "":
            return None
        return float(nmea_data)
    
    def gngga_parse(self,line):
        data = line.split(",")
        
        if data is None or len(data) != 15 or (data[0] == ""):
            return  # Unexpected number of params.
            
        self.latitude = self.parse_degrees(data[2])
        if self.latitude is not None and data[3] is not None and data[3].lower() == "s":
            self.latitude *= -1.0
        self.longitude = self.parse_degrees(data[4])
        if (self.longitude is not None and data[5] is not None and data[5].lower() == "w"):
            self.longitude *= -1.0
            
        self.fix_quality = self.parse_int(data[6])
        self.satellites = self.parse_int(data[7])
        self.horizontal_dilution = self.parse_float(data[8])

        self.altitude_m = self.parse_float(data[9])
        self.height_geoid = self.parse_float(data[11])

        self.publisher()
        #am_gps_urcu_status = rospy.Publisher ('am_gps_urcu_status',UInt8, queue_size=10)
        #return 0
        #am_gps_urcu = rospy.Publisher('am_gps_urcu', NavSatFix, queue_size=10)

    def publisher(self):
        
        #print("gps_nrtk_latitude",self.latitude)
        #gps_nrtk_pub = rospy.Publisher('gps_nrtk_pub', NavSatFix, queue_size=10)
        am_gps_urcu = rospy.Publisher('am_gps_urcu', NavSatFix, queue_size=10)
        am_gps_urcu_status = rospy.Publisher('am_gps_urcu_status',UInt8, queue_size=10)
        """
        if self.latitude is not None and self.latitude != 0:
            print("Latitude: {0:.6f} degrees".format(self.latitude))
        if self.longitude is not None and self.longitude != 0:
            print("Longitude: {0:.6f} degrees".format(self.longitude))
        if self.fix_quality is not None:
            print("Fix quality: {}".format(self.fix_quality))

        if self.satellites is not None and self.satellites != 0:
            print("# satellites: {}".format(self.satellites))
        if self.altitude_m is not None and self.altitude_m != 0:
            print("Altitude: {} meters".format(self.altitude_m))

        if self.horizontal_dilution is not None:
            print("Horizontal dilution: {}".format(self.horizontal_dilution))
        if self.height_geoid is not None:
            print("Height geo ID: {} meters".format(self.height_geoid))
        # Publish update
        print("Latitude",self.latitude)
        """
        if self.latitude is not None and self.latitude != 0:
            gpsfix = NavSatFix()
            gpsfix.header.stamp = rospy.Time.now()
            gpsfix.header.frame_id = 'gps_frame'  # FRAME_ID
            gpsfix.latitude = self.latitude
            gpsfix.longitude = self.longitude
            gpsfix.altitude = self.altitude_m
            gpsfix.position_covariance_type = NavSatFix.COVARIANCE_TYPE_APPROXIMATED
            gpsfix.position_covariance[0] = (self.horizontal_dilution*0.1)**2 #std_dev error position estimate = 0.1
            gpsfix.position_covariance[4] = (self.horizontal_dilution*0.1)**2 #std_dev error position estimate = 0.1
            gpsfix.position_covariance[8] = (4*self.horizontal_dilution*0.1)**2 #std_dev error position estimate = 0.1
            #gps_nrtk_pub.publish(gpsfix)
            am_gps_urcu.publish(gpsfix)
            am_gps_urcu_status.publish(self.fix_quality)
            #print("Published")
        return 0
        
    
        
       
    def parse_data(self):
        #reading gnss lines from serial port
        if(self.connect):
           for line in self.gnss.readlines():
                 print("Readline in gnss readlines",line)
                 if line.startswith(b'$GNGGA'):
                     #record.add(Record.Kind.GGA, line)
                     print("GNGGA message received")
                     gngga_data = line.decode().strip()
                     print(gngga_data)
                           
                     #print("Latitude is:",self.latitude)
                     #if self.latitude is not None:
                     #    k = self.publisher(gngga_data)
                     self.gngga_parse(gngga_data)
                     
                            
            #return 0
        #gps_nrtk.client.loop_start()
     


def main(args):
    rospy.init_node ('nRTK')
    rospy.Rate(8)
    nRTK_node = nRTK()
    while not rospy.is_shutdown():
        nRTK_node.client.loop_start()
        nRTK_node.parse_data()

if __name__ == '__main__':
    main(sys.argv)
