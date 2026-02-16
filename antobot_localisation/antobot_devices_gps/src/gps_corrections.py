#!/usr/bin/env python3

# Copyright (c) 2023, ANTOBOT LTD.
# All rights reserved

# # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # 

# # # Code Description:     This code receives GPS correction messages from a variety of sources (usually MQTT) and writes 
# # #                       the corrections to the F9P chip.

# Contact: Daniel Freer 
# email: daniel.freer@antobot.ai

# # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # #


import os
import time
import yaml
import paho.mqtt.client as mqtt
import rospy, rospkg
import importlib
import serial
import socket
import base64
from std_msgs.msg import  String
from antobot_devices_msgs.msg import gpsQual
import threading


class gpsCorrections():
    def __init__(self):
        # # # Initialisation of GPS corrections class
        #     Inputs: dev_type - the type of device that this script is running on
        #           "urcu" - the URCU; Jetson-based packages should be used
        #           "rasPi" - antoScout or another rasPi-based system; rasPi-compatible packages should be used


        self.sock = None
        self.corr_type = "ntrip"
        self.gga_interval=10
        self.latest_gga = None
        self.sub_gga = rospy.Subscriber("/antobot_gps/gga",String,self.gga_callback)
        self.time_offset = rospy.Subscriber("/antobot_gps/quality",gpsQual, self.time_offset_callback)
        self.count = 0
        self.running = True
        self.sent_time = rospy.Time.now()
        # Reading configuration file
        rospack = rospkg.RosPack()
        packagePath=rospack.get_path('antobot_description')
        path = packagePath + "/config/platform_config.yaml"
        GPIO = importlib.import_module("Jetson.GPIO") 
        #dev_port = "/dev/ttyTHS0"
        baud = 460800
        self.serial_port = serial.Serial(port=dev_port, baudrate=baud)  #38400



            
        packagePath=rospack.get_path('antobot_devices_gps')
        print("packagePath: {}".format(packagePath))
        yaml_file_path = packagePath + "/config/corrections_config.yaml"
        with open(yaml_file_path, 'r') as file:
            config = yaml.safe_load(file)
            self.ntrip_server = config['ntrip']['address']
            self.ntrip_port = config['ntrip']['port']
            self.ntrip_username = config['ntrip']['username']
            self.ntrip_password = config['ntrip']['password']
            self.ntrip_mountpoint = config['ntrip']['mountpoint']


        # If the uRCU is being used, the appropriate GPIO pin must be set to "high" to enable corrections from Xavier 
        #if "urcu" in dev_type:
            # Set the GPIO pin of the URCU high
        self.gpio01 = 29
        self.GPIO = GPIO
        self.GPIO.setmode(GPIO.BOARD)
        self.GPIO.setup(self.gpio01, GPIO.OUT)
        #self.GPIO.output(self.gpio01, GPIO.HIGH) # if need 20240424


###added for check

 # Setting up the MQTT Client
        if self.corr_type != "ntrip":
            if self.corr_type == "ppp":
                self.client = mqtt.Client(client_id=self.ppp_client_id, userdata=self.userdata)
                self.certfile=os.path.join(packagePath,"config/")+f'device-{self.ppp_client_id}-pp-cert.crt'
                self.keyfile=os.path.join(packagePath,"config/")+f'device-{self.ppp_client_id}-pp-key.pem'
                self.client.tls_set(certfile=self.certfile,keyfile=self.keyfile)
            elif self.corr_type == "ant_mqtt":
                self.client = mqtt.Client(self.ant_client_id)
                self.client.username_pw_set(mqtt_username, mqtt_password)
            self.client.on_connect = self.on_connect
            self.client.on_message = self.on_message

            self.connect_broker()

            self.last_receive_time = time.time()
            self.timer = rospy.Timer(rospy.Duration(30), self.check_RTCM_timeout) 
            

    # Attempts to connect to the broker
    def connect_broker(self):
        try:
            if self.corr_type == "ppp":
                self.client.connect(self.ppp_server, port=8883)
            elif self.corr_type == "ant_mqtt":
                self.client.connect(self.ant_broker, self.ant_mqtt_port, self.mqtt_keepalive)
            
            # rospy.loginfo("SN4500: Connected to MQTT broker successfully.")
        except Exception as e:
            rospy.logerr("SN4500: Connected to MQTT broker failed. ({e})")

        time.sleep(2)
        
    # The callback for when the client receives a CONNECT response from the server.
    def on_connect(self,client, userdata, flags, rc):
        
        if rc == 0:
            self.connect = True
            if self.corr_type == "ppp":
                self.client.subscribe(userdata['topics'])
            elif self.corr_type == "ant_mqtt":
                self.client.subscribe(self.ant_mqtt_topic_sub)
            rospy.loginfo("SN4500: Connected to broker successfully")
        else: 
            rospy.logerr("SN4500: Connected to broker failed. (rc = {rc})")

    # The callback for when a PUBLISH message is received from the server.
    def on_message(self,client,userdata, msg):
        # write the corrections via UART2
        try:
            if self.corr_type == "ppp":
                data = userdata['gnss'].write(msg.payload)
            elif self.corr_type == "ant_mqtt":
                data = self.serial_port.write(msg.payload)
            self.last_receive_time = time.time()
        except Exception as e:
            rospy.logerr(f"SN4500: Write the corrections failed. ({e})")
            dev_port = "/dev/ttyTHS0"
            baud = 460800
            self.serial_port = serial.Serial(port=dev_port, baudrate=baud)  #38400


    def check_RTCM_timeout(self, event):
        if time.time() - self.last_receive_time > 30: # 30s
            rospy.logerr("SN4500: The RTCM message update timeout ({} > 30s).".format(time.time() - self.last_receive_time))
            #self.last_receive_time = time.time()
        #print(time.time() - self.last_receive_time) # test
###end

    def make_ntrip_request(self):
        auth = base64.b64encode(f"{self.ntrip_username}:{self.ntrip_password}".encode()).decode()
        request = (
            f"GET /{self.ntrip_mountpoint} HTTP/1.1\r\n"
            f"Host: {self.ntrip_server}:{self.ntrip_port}\r\n"
            "User-Agent: NTRIP u-blox client\r\n"
            f"Authorization: Basic {auth}\r\n"
            "Connection: close\r\n"
            "\r\n"
        )
        return request.encode()

    def connect_ntrip(self):
        try:
            self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.sock.connect((self.ntrip_server, self.ntrip_port))
            self.sock.send(self.make_ntrip_request())
            print("[INFO] Connected to NTRIP server") 
        except:
            print("[ERROR] can't connect to NTRIP server")
    def stream_corrections(self,event=None):
        try:
            print("[INFO] Streaming corrections...")
            data = self.sock.recv(1024)
            if not data:
                print("[WARN] NTRIP server closed connection")
                self.connect_ntrip()
            #print(data)
            self.serial_port.write(data)
        except:
            print("[ERROR] NTRIP server break, reconnecting")
            self.connect_ntrip()
    def gga_callback(self,data):
        self.latest_gga=data.data
        return

    def time_offset_callback(self,data):
        self.time_offset = data.t_offset
        return

    def send_gga(self,event=None):
        
        print("in send gga")
        """ Periodically send latest GGA to NTRIP server """ 
        if self.latest_gga and self.time_offset<0.5:
            try:
                self.sock.send(self.latest_gga.encode())
                print(f"[INFO] Sent GGA: {self.latest_gga.strip()}")
                self.sent_time = rospy.Time.now()
            except Exception as e:
                print(f"[WARN] Failed to send GGA: {e}")
                #self.running=False
                self.connect_ntrip()
        if (rospy.Time.now()-self.sent_time).to_sec() > 35:
            self.connect_ntrip()


    def close(self):
        self.running=False
        if self.serial_port:
            self.serial_port.close()
        if self.sock:
            self.sock.close()
        print("[INFO] Closed connections")


def main():
    rospy.init_node ('gps_correction')
    gps_corr = gpsCorrections()
    rate=rospy.Rate(0.1)
    try:
        if gps_corr.corr_type == "ntrip":
            gps_corr.connect_ntrip()
            rospy.Timer(rospy.Duration(5), gps_corr.send_gga)
            rospy.Timer(rospy.Duration(0.1), gps_corr.stream_corrections) 
            rospy.spin() 
        if gps_corr.corr_type == "ppp" or "ant_mqtt":
            gps_corr.client.loop_start()
            rate = rospy.Rate(1) # 1hz
            while not rospy.is_shutdown():
                #print("gpsCorrections is running") # for test
                rate.sleep()
            gps_corr.client.loop_stop()

    except KeyboardInterrupt:
        print("[INFO] Interrupted by user")
    except Exception as e:
        print(f"[ERROR] {e}")
    finally:
        if gps_corr.corr_type == "ntrip":
            gps_corr.close()
            pass

if __name__ == '__main__':
    main()
    
