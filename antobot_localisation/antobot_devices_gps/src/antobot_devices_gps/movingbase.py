#!/usr/bin/env python3
# Copyright (c) 2023, ANTOBOT LTD.
# All rights reserved

# # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # 

# # # Code Description: The purpose of this code is to use two GPS antennas to obtain accurate positioning and heading 
# information. The code retrieves correction messages from a base station and sends them to the u-blox F9P chip inside 
# the uRCU. It then receives correction messages from the u-blox F9P chip inside the uRCU and forwards them to another 
# u-blox F9P chip located externally.
# This script is intended for use with a dual GPS antenna setup, where a base station provides correction messages via MQTT.
    
#This script reports the following on GPS status:
# GPS status : Critical ; GPS status = 0
# GPS status : Warning ; GPS status = 1 - Float
# GPS status : Good ; GPS status = 3 - Fix

# Contact: Huaide Wang
# email: huaide.wang@antobot.ai

# # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # #
import asyncio
import serial
import serial_asyncio
import struct
from dataclasses import dataclass

import threading
import time

import yaml
import os

from paho.mqtt import client as mqtt_client
#from pyrtcm import RTCMReader

import logging

import spidev
 
from antobot_devices_gps.ublox_gps import UbloxGps


# LOG
logger = logging.getLogger('movingbase')
logger.setLevel(level=logging.INFO)

formatter = logging.Formatter('%(asctime)s - [line:%(lineno)d] - %(levelname)s: %(message)s')

"""
file_handler = logging.FileHandler('movingbase_test.log')
file_handler.setLevel(level=logging.INFO)
file_handler.setFormatter(formatter)
"""
stream_handler = logging.StreamHandler()
stream_handler.setLevel(logging.INFO)
stream_handler.setFormatter(formatter)


#logger.addHandler(file_handler)
logger.addHandler(stream_handler)


class TimeoutException(Exception):
    pass
    
        
class MovingBase:

    # read rtcm message from base and write it to rover
    class RTCMFramer(asyncio.Protocol):

        START_BYTE = 0xd3

        def __init__(self, movebase, port):
            super().__init__()

            self.movebase = movebase
            self.port = port

            self._in_frame = False
            self.transport = None
            self._frame = bytearray()
            self._frame_size = 0
            self._frame_count = 0

        def connection_made(self, transport):
            self.transport = transport

            # if want to read from UART, must write first.
            self.transport.write(b'0')

        def data_received(self, data):
            self.transport_rover.write(data)
            logger.debug(f"RTCMFramer received data: {data}")
            
        def connection_lost(self, exc):
            logger.error('Moving Base: Connection lost, attempting to reconnect...')
            asyncio.get_event_loop().create_task(self.reconnect())
            #asyncio.get_event_loop().stop()

        async def reconnect(self):
            await asyncio.sleep(1)
            try:
                self.movebase._transport_base, self.movebase._protocol_base = await serial_asyncio.create_serial_connection(
                    asyncio.get_running_loop(),
                    lambda: MovingBase.RTCMFramer(self.movebase, self.port),
                    self.port,
                    baudrate=460800
                    )
                
                
                if self.movebase._protocol_base:
                    self.movebase._protocol_base.transport_rover = self.movebase._transport_rover
                    
                    self.movebase.MyMQTT.set_transport(self.movebase._transport_base)
                    logger.info(f'Moving Base: Reconnection succeeded')
                
            except Exception as e:
                logger.error(f'Moving Base: Reconnection failed: {e}')
                asyncio.get_event_loop().create_task(self.reconnect())

    # read RELPOSNED message from rover and parse it to get heading
    class RELPOSNEDFramer(asyncio.Protocol):

        SYNC1_BYTE = 0xb5
        SYNC2_BYTE = 0x62

        def __init__(self, movebase, port):
            super().__init__()

            self.movebase = movebase
            self.port = port

            self._in_frame = False
            self.transport = None
            self._frame = bytearray()
            self.frames = asyncio.Queue()
            self._frame_size = 0
            self._frame_count = 0

        def connection_made(self, transport):
            self.transport = transport

        def data_received(self, data):
            for b in data:
                if self._in_frame:
                    self._frame.append(b)

                    if len(self._frame) == 2 and self._frame[1] != MovingBase.RELPOSNEDFramer.SYNC2_BYTE:
                        self._in_frame = False

                    if len(self._frame) == 6:
                        self._frame_size = int.from_bytes(self._frame[4:6], byteorder='little', signed=False) 

                    elif self._frame_count > self._frame_size: #checksum=3
                        self._in_frame = False
                        try:
                            relposnedFrame = self.parse_NAV_RELPOSNED(self._frame)
                            logger.debug(f"RELPOSNEDFramer: {relposnedFrame}")
                            asyncio.run_coroutine_threadsafe(self.frames.put(relposnedFrame), asyncio.get_running_loop())
                        except:
                            logger.warning("RELPOSNEDFramer: Can not get Full RELPOSNEDFrame!")

                    elif len(self._frame) > 6:
                        self._frame_count += 1

                else:
                    if b == MovingBase.RELPOSNEDFramer.SYNC1_BYTE:
                        self._in_frame = True
                        self._frame.clear()
                        self._frame.append(b)
                        self._frame_size = 0
                        self._frame_count = 0
 
        def connection_lost(self, exc):
            logger.error('Moving Rover: Connection lost, attempting to reconnect...')
            asyncio.get_event_loop().create_task(self.reconnect())
            #asyncio.get_event_loop().stop()

        async def reconnect(self):
            await asyncio.sleep(1)
            try:
                self.movebase._transport_rover, self.movebase._protocol_rover = await serial_asyncio.create_serial_connection(
                    asyncio.get_running_loop(),
                    lambda: MovingBase.RELPOSNEDFramer(self.movebase, self.port),
                    self.port,
                    baudrate=460800
                    )
                    
                if self.movebase._protocol_base:
                    self.movebase._protocol_base.transport_rover = self.movebase._transport_rover
                    logger.info(f'Moving Rover: Reconnection succeeded')
                
            except Exception as e:
                logger.error(f'Moving Rover: Reconnection failed: {e}')
                asyncio.get_event_loop().create_task(self.reconnect())

        def parse_NAV_RELPOSNED(self, relposnedMessage):
            #print("ubx_message:", ubx_message)
            #version = struct.unpack('B', ubx_message[6:7])[0]
            #print(f"version: {version}")

            iTOW = struct.unpack('<i', relposnedMessage[10:14])[0]
            #print(f"iTOW: {iTOW/1000}s")

            # North, East, and Down components of the 
            relPosN = struct.unpack('<i', relposnedMessage[14:18])[0]
            relPosE = struct.unpack('<i', relposnedMessage[18:22])[0]
            relPosD = struct.unpack('<i', relposnedMessage[22:26])[0]

            relPosLength = struct.unpack('<i', relposnedMessage[26:30])[0]   # cm
            relPosHeading = struct.unpack('<i', relposnedMessage[30:34])[0]  # deg, 1e-5
            accLength = struct.unpack('<I', relposnedMessage[54:58])[0]      # mm, 0.1
            accHeading = struct.unpack('<I', relposnedMessage[58:62])[0]     # deg, 1e-5
            
            # prase flags
            flags = struct.unpack('<I', relposnedMessage[66:70])[0]          # flag
            flag_gnssFixOK = bool(flags & 0b00000001)      
            flag_diffSoln = bool(flags & 0b00000010)       
            flag_relPosValid = bool(flags & 0b00000100)    
            flag_carrSoln = (flags >> 3) & 0b00000011        
            flag_isMoving = bool(flags & 0b00100000)      
            relPosHeadingValid = bool(flags & 0b100000000)
            relPosNormalized = bool(flags & 0b1000000000) 

            relposnedFrame = MovingBase.RELPOSNEDFrame(iTOW, relPosN, relPosE, relPosD, relPosLength, relPosHeading, accLength, accHeading, 
                                        flag_gnssFixOK, flag_diffSoln, flag_relPosValid, flag_carrSoln, 
                                        flag_isMoving, relPosHeadingValid, relPosNormalized)

            return relposnedFrame

    # mqtt class: receive the rtcm message from fixed base and write it to base
    class MyMqtt:
        def __init__(self, transport, mode):
            
            self.transport = transport
            self.mode = mode

            if self.mode == 1:
                logger.info(f'nRTK Mode: base station')     
                parent_directory = os.path.dirname(os.path.abspath(__file__))
                yaml_file_path = os.path.join(parent_directory, "../../config/mqtt_config.yaml")
               
                with open(yaml_file_path, 'r') as file:
                    config = yaml.safe_load(file)
                    self.client_id = 'Anto_MQTT_F9P_Sub_' + config['device_ID']
                    self.topics_sub = 'Anto_MQTT_F9P_' + config['base_ID']
                    self.broker = config['mqtt_Broker']
                    self.port = config['mqtt_Port']
                    self.keepalive = config['mqtt_keepalive']
                    self.mqtt_username = config['mqtt_UserName']
                    self.mqtt_password = config['mqtt_PassWord']

                self.client = mqtt_client.Client(self.client_id)
                #self.client = mqtt_client.Client(mqtt_client.CallbackAPIVersion.VERSION1, self.client_id) # for paho-mqtt 2.0

                self.client.username_pw_set(self.mqtt_username, self.mqtt_password)

            elif self.mode == 2:     
                logger.info(f'nRTK Mode: ppp')
                parent_directory = os.path.dirname(os.path.abspath(__file__))
                yaml_file_path = os.path.join(parent_directory, "../../config/ppp_config.yaml")
  
                with open(yaml_file_path, 'r') as file:
                        config = yaml.safe_load(file)
                        client_id = config['device_ID']

                self.client_id = client_id 
                self.broker = 'pp.services.u-blox.com'
                self.port=8883
                self.topics_sub = [(f"/pp/ip/eu", 0), ("/pp/ubx/mga", 0), ("/pp/ubx/0236/ip", 0)]
                self.client = mqtt_client.Client(client_id=self.client_id)
                self.client.tls_set(certfile=os.path.join(parent_directory,"../../config/")+f'device-{self.client_id}-pp-cert.crt',keyfile=os.path.join(parent_directory,"../../config/")+f'device-{self.client_id}-pp-key.pem')
                            
            self.client.on_connect = self.on_connect
            self.client.on_message = self.on_message

            
            self.connect_broker()

        def connect_broker(self):
            try:
                if self.mode == 1:
                    self.client.connect(self.broker, self.port, self.keepalive)
                elif self.mode == 2:
                    self.client.connect(self.broker, self.port)

            except Exception as e:
                logger.error(f"MQTT: Connection failed: {e}")

            self.client.loop_start() 

        def disconnect_broker(self):
            self.client.loop_stop()
            self.client.disconnect()

        def on_connect(self, client, userdata, flags, rc):
            if rc == 0:
                logger.info(f"MQTT: Connected to MQTT Broker successfully!")
                self.client.subscribe(self.topics_sub)
            else:
                logger.error("MQTT: Failed to connect, return code {0}".format(rc))

        def publish_message(self, msg):
            result = self.client.publish(topic=self.topic, payload=msg, qos=0, retain=True)

            if result[0] == 0:
                return True
            else:
                logger.error("MQTT: Failed to send message {0} to topic {1}".format(msg, self.topic))
                return False
        
        def on_message(self, client, userdata, msg):
            self.transport.write(msg.payload)

        def set_transport(self, transport_):
            self.transport = transport_

    @dataclass
    class RELPOSNEDFrame:
        iTOW: int                 # GPS time of week of the navigation epoch. (ms)
        relPosN: int              # North component of relative position vector
        relPosE: int              # East component of relative position vector
        relPosD: int              # Down component of relative position vector
        relPosLength: int         # Length of the relative position vector  (cm)
        relPosHeading: int        # Heading of the relative position vector (1e-5 deg)
        accLength: int            # Accuracy of length of the relative position vector (0.1 mm)
        accHeading: int           # Accuracy of heading of the relative position vector (1e-5 deg)
        flag_gnssFixOK: bool      # A valid fix (i.e within DOP & accuracy masks)
        flag_diffSoln: bool       # 1 if differential corrections were applied
        flag_relPosValid: bool    # 1 if relative position components and accuracies are valid and, in moving base mode only, if baseline is valid
        flag_carrSoln: int        # 0 = no carrier phase range solution; 1 = carrier phase range solution with floating; 2 = carrier phase range solution with fixed ambiguities
        flag_isMoving: bool       # 1 if the receiver is operating in moving base mode
        relPosHeadingValid: bool  # 1 if relPosHeading is valid
        relPosNormalized: bool    # 1 if the components of the relative position vector (including the high-precision parts) are normalized
        
    @dataclass
    class PVTFrame:
        iTOW: int                 # GPS time of week of the navigation epoch. (ms)
        fixType: int              # 0 = no fix; 1 = dead reckoning only; 2 = 2D-fix; 3 = 3D-fix; 4 = GNSS + dead reckoning combined; 5 = time only fix 
        flag_carrSoln: int        # 0 = no carrier phase range solution; 1 = carrier phase range solution with floating ambiguities; 2 = carrier phase range solution with fixed ambiguities
        lon: float                # Longitude (1e-7 deg)
        lat: float                # Latitude (1e-7 deg)
        height: float             # Height above ellipsoid (mm)
        hAcc: int                 # Horizontal accuracy estimat(mm)
        status_fix: int           # fix type: 0, 1, 3
        
    def __init__(self):
        self._transport_base = None
        self._protocol_base = None

        self._transport_rover = None
        self._protocol_rover = None

        self.MyMQTT = None
        self.F9P_Base = None
        
        self.time_limit = 0.25
        
    @staticmethod
    async def create(port1='port_movingbase', port2='port_rover', port3='port_base', mode=1):
        movebase = MovingBase()
        movebase._transport_base, movebase._protocol_base = await serial_asyncio.create_serial_connection(
            asyncio.get_running_loop(),
            lambda: MovingBase.RTCMFramer(movebase, port1),
            port1,
            baudrate=460800
            )
        
        movebase._transport_rover, movebase._protocol_rover = await serial_asyncio.create_serial_connection(
            asyncio.get_running_loop(),
            lambda: MovingBase.RELPOSNEDFramer(movebase, port2),
            port2,
            baudrate=460800
            )
        
        if movebase._protocol_base:
            movebase._protocol_base.transport_rover = movebase._transport_rover
        
        if movebase._transport_base:
            movebase.MyMQTT = MovingBase.MyMqtt(movebase._transport_base, mode)
        else:
            logger.error(f"MyMqtt: No movebase transport_base")

        
        movebase.F9P_Base  = MovingBase.F9P_GPS(port3)

        return movebase
    
    async def get_RELPOSNEDframe(self):

        # self._protocol_rover is an asyncio serial connection (defined in line 363)
        # its frames are 
        try:
            result = await asyncio.wait_for(self._protocol_rover.frames.get(), timeout=self.time_limit)
            return result
        except Exception as e:
            #logger.error(f"Timeout: get_RELPOSNEDframe ({e})")
            return None
    
    def pub_PVT_Heading(self, data):
        self.MyMQTT.publish_message(data)
    
    # for test
    def close(self):
        if self._transport_base.is_closing():
           logger.error(f"transport_base is closing")
        else:
           self._transport_base.close()
           logger.error(f"close transport_base") 

        if self._transport_rover.is_closing():
           logger.error(f"transport_rover is closing")
        else: 
           self._transport_rover.close() 
           logger.error(f"close transport_rover")
   
async def main():

    logger.info(f"===============================================")

    base_port_uart2 = "/dev/ttyTHS0"
    rover_port2 = "/dev/AntoF9P"
    base_port_spi = None
    
    movebase = await MovingBase.create(base_port_uart2, rover_port2, base_port_spi)

    #time_start = time.time()
    
    publish_mqtt = True
    while True:
        
        time1 = time.time()
        headFrame = await movebase.get_RELPOSNEDframe()
        time2 = time.time()
        
        if publish_mqtt:
            
            if headFrame:
                movebase.pub_PVT_Heading(f'headFrame: {headFrame}')        

if __name__ == '__main__':
    loop = asyncio.get_event_loop()
    loop.run_until_complete(main())
    loop.run_forever()
    loop.close()
