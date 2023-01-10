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

#
# Dual GPS Yaw Calculation Simulation Node
# (for Gazebo simulation)
#

import rospy
import spidev
import time
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import TwistWithCovarianceStamped
from std_msgs.msg import UInt8

from ubxtranslator import core

# Internal helper parsing functions.
# These handle input that might be none or null and return none instead of
# throwing errors.
def _parse_degrees(nmea_data):
    # Parse a NMEA lat/long data pair 'dddmm.mmmm' into a pure degrees value.
    # Where ddd is the degrees, mm.mmmm is the minutes.
    if nmea_data is None or len(nmea_data) < 3:
        return None
    raw = float(nmea_data)
    deg = raw // 100
    minutes = raw % 100
    return deg + minutes / 60


def _parse_int(nmea_data):
    if nmea_data is None or nmea_data == "":
        return None
    return int(nmea_data)


def _parse_float(nmea_data):
    if nmea_data is None or nmea_data == "":
        return None
    return float(nmea_data)


def _parse_str(nmea_data):
    if nmea_data is None or nmea_data == "":
        return None
    return str(nmea_data)

    
class GPS:
    """GPS parsing module.	Can parse simple NMEA data sentences from SPI GPS
    modules to read latitude, longitude, and more."""

    def __init__(self, spiport):
        self.spiport = spiport
        # Initialize null starting values for GPS attributes.
        self.datatype = None
        self.timestamp_utc = None
        self.latitude = 0
        self.longitude = 0
        self.fix_quality = None
        self.fix_quality_3d = None
        self.satellites = 0
        self.satellites_prev = None
        self.horizontal_dilution = None
        self.altitude_m = 0
        self.height_geoid = None
        self.speed_knots = None
        self.track_angle_deg = None
        self.sats = None
        self.isactivedata = None
        self.true_track = None
        self.mag_track = None
        self.sat_prns = None
        self.sel_mode = None
        self.pdop = None
        self.hdop = None
        self.vdop = None
        self.total_mess_num = None
        self.mess_num = None
        self._raw_sentence = None
        self.debug = None
        self.enu_velocity = None
        self.enu_covariances = None


    def readSentence(self, spiport):
        # s = ""
        buf = ""
        isstart = False
        while True:
            # print('test:------------')
            # print(type(spiport.readbytes(1)[0]))
            # print(type(spiport.readbytes(1)))
            c = chr(spiport.readbytes(1)[0])
            if isstart is False:
                if c == '$':
                    isstart = True
                    buf += c
            else:
                if c != '\n':
                    buf += c
                else:
                    buf += c
                    isstart = False
                    break
        return buf
        
    def readSentenceBuffer(self, buffer):
        
        buf = ""
        bufArray = []
        
        isstart = False
        
        i = 0
        nBufArray = 0

        while i < len(buffer):
            
            # print('test:------------')
            # print(type(spiport.readbytes(1)[0]))
            # print(type(spiport.readbytes(1)))
            
            c = chr(buffer[i])
            if isstart is False:
                if c == '$':
                    isstart = True
                    buf += c
            else:
                if c != '\n':
                    buf += c
                else:
                    buf += c
                    
                    bufArray.append(buf)
                    nBufArray = nBufArray + 1
                    
                    buf = ""
                    isstart = False
            
            i = i + 1
        
        return bufArray

    def send_command(self, command, add_checksum=True):
        """Send a command string to the GPS.  If add_checksum is True (the
        default) a NMEA checksum will automatically be computed and added.
        Note you should NOT add the leading $ and trailing * to the command
        as they will automatically be added!"""
        self.write(b"$")
        self.write(command)
        if add_checksum:
            checksum = 0
            for char in command:
                checksum ^= char
            self.write(b"*")
            self.write(bytes("{:02x}".format(checksum).upper(), "ascii"))
        self.write(b"\r\n")

    def write(self, bytestr):
        """Write a bytestring data to the GPS directly, without parsing or
        checksums"""
        return self.spiport.writebytes(bytestr)
        
    def prepare_nmea(self, command):
        """Prepares NMEA protocol sentence"""
        # calculate checksum
        checksum = 0
        for i in range(0, len(command)):
            checksum ^= ord(command[i])
        
        checksum_str = hex(checksum)
        # prepare nmea packet
        nmea_str = command + "*" + checksum_str[2:].upper()
        nmea_str = "$" + nmea_str + "\r\n"
        nmea_str = bytes(nmea_str, 'ascii')

        return nmea_str
        
    def prepare_ubx(self, class_id, msg_id):
        """Prepares UBX protocol sentence"""
        
        packet = bytearray(8)
        # prepare packet
        packet[0] = 0xb5
        packet[1] = 0x62
        packet[2] = class_id
        packet[3] = msg_id
        packet[4] = 0x00
        packet[5] = 0x00
        
        # calculate ubx checksum
        chk_a = 0
        chk_b = 0
        for i in range(2, 6):
            chk_a = chk_a + packet[i]
            chk_b = chk_b + chk_a
        
        
        packet[6] = chk_a & 0xff
        packet[7] = chk_b & 0xff
        
        return packet

    def prepare_ubx_uart_enable(self):
        """Prepares UBX protocol sentence"""
        
        # CFG-UART2-ENABLED
        # 0x10530005
        
        packet = bytearray(17)
        
        # prepare packet
        packet[0] = 0xb5
        packet[1] = 0x62
        packet[2] = 0x06
        packet[3] = 0x8a
        
        packet[4] = 0x09 # length 0
        packet[5] = 0x00 # length 1
        
        packet[6] = 0x00 # version
        packet[7] = 0x05 # layers
        
        packet[8] = 0x00 # reserved
        packet[9] = 0x00 # reserved
        
        # CFG-UART2-ENABLED
        packet[10] = 0x05
        packet[11] = 0x00
        packet[12] = 0x53
        packet[13] = 0x10
        
        # CFG-UART2-ENABLED is 1
        packet[14] = 0x01
        
        # calculate ubx checksum
        chk_a = 0
        chk_b = 0
        for i in range(2, 15):
            chk_a = chk_a + packet[i]
            chk_b = chk_b + chk_a
        
        
        packet[15] = chk_a & 0xff
        packet[16] = chk_b & 0xff
        
        return packet        
                        
    def prepare_ubx_uart_set_baudrate(self, baud):
        """Prepares UBX UART2 baud rate protocol sentence"""
        
        # CFG-UART2-BAUDRATE 0x40530001 U4 - - The baud rate that should be configured on the UART2
        # CFG-UART2-ENABLED
        # B5 62 06 8A 0C 00 00 05 00 00 01 00 53 40 00 08 07 00 44 B5
        
        packet = bytearray(20)
        # prepare packet
        packet[0] = 0xb5
        packet[1] = 0x62
        packet[2] = 0x06
        packet[3] = 0x8a
        
        packet[4] = 0x0c # length 0
        packet[5] = 0x00 # length 1
        
        packet[6] = 0x00 # version
        packet[7] = 0x05 # layers
        
        packet[8] = 0x00 # reserved
        packet[9] = 0x00 # reserved
        
        # CFG-UART2-BAUDRATE
        packet[10] = 0x01
        packet[11] = 0x00
        packet[12] = 0x53
        packet[13] = 0x40
        
        packet[14] = baud & 0xff
        packet[15] = (baud >> 8) & 0xff
        packet[16] = (baud >> 16) & 0xff
        packet[17] = (baud >> 24) & 0xff
        
        # calculate ubx checksum
        chk_a = 0
        chk_b = 0
        for i in range(2, 18):
            chk_a = chk_a + packet[i]
            chk_b = chk_b + chk_a
        
        
        packet[18] = chk_a & 0xff
        packet[19] = chk_b & 0xff
        
        return packet
        
    def cfg_rate_meas(self):
        #print("In config_rate")
        #CFG-RATE-MEAS ; VALUE = 0x7d = 125ms = 8Hz; c8 = 200ms = 5hz; 64 = 100ms = 10hz; 33= 50ms
        
        packet = bytearray(18)
        # prepare packet
        packet[0] = 0xb5
        
        packet[1] = 0x62
        packet[2] = 0x06
        packet[3] = 0x8a
        
        packet[4] = 0x0a # length 0
        packet[5] = 0x00 # length 1
        
        packet[6] = 0x00 # version
        packet[7] = 0x05 # layers
        
        packet[8] = 0x00 # reserved
        packet[9] = 0x00 # reserved
        
      
        packet[10] = 0x01
        packet[11] = 0x00
        
        packet[12] = 0x21
        
        packet[13] = 0x30
        
        packet[14] = 0x7d # value
        
        packet[15] = 0x00
        
        chk_a = 0
        chk_b = 0
        for i in range(2, 16):
            chk_a = chk_a + packet[i]
            chk_b = chk_b + chk_a
        
        
        packet[16] = chk_a & 0xff
        packet[17] = chk_b & 0xff
        
        #print("Packet in config_rate ",packet)
        return packet
        
    def cfg_rate_timeref(self):
        #print("In config_rate")
        #CFG-RATE-TIMEREF ; VALUE: 0: utc
        
        packet = bytearray(18)
        # prepare packet
        packet[0] = 0xb5
        
        packet[1] = 0x62
        packet[2] = 0x06
        packet[3] = 0x8a
        
        packet[4] = 0x0a # length 0
        packet[5] = 0x00 # length 1
        
        packet[6] = 0x00 # version
        packet[7] = 0x05 # layers
        
        packet[8] = 0x00 # reserved
        packet[9] = 0x00 # reserved
        
      
        packet[10] = 0x03
        packet[11] = 0x00
        
        packet[12] = 0x21
        
        packet[13] = 0x20
        
        packet[14] = 0x00 # value
        
        packet[15] = 0x00
        
        chk_a = 0
        chk_b = 0
        for i in range(2, 16):
            chk_a = chk_a + packet[i]
            chk_b = chk_b + chk_a
        
        
        packet[16] = chk_a & 0xff
        packet[17] = chk_b & 0xff
        
        #print("Packet in config_rate ",packet)
        return packet
        
    def cfg_rate_nav(self):
        #print("In config_rate_nav")
        #CFG-RATE-NAV ; VALUE = 0x01
        
        packet = bytearray(18)
        # prepare packet
        packet[0] = 0xb5
        
        packet[1] = 0x62
        packet[2] = 0x06
        packet[3] = 0x8a
        
        packet[4] = 0x0a # length 0
        packet[5] = 0x00 # length 1
        
        packet[6] = 0x00 # version
        packet[7] = 0x05 # layers
        
        packet[8] = 0x00 # reserved
        packet[9] = 0x00 # reserved
        
      
        packet[10] = 0x02
        packet[11] = 0x00
        
        packet[12] = 0x21
        
        packet[13] = 0x30
        
        packet[14] = 0x01 #value
        
        packet[15] = 0x00
        
        # calculate ubx checksum
        chk_a = 0
        chk_b = 0
        for i in range(2, 16):
            chk_a = chk_a + packet[i]
            chk_b = chk_b + chk_a
        
        
        packet[16] = chk_a & 0xff
        packet[17] = chk_b & 0xff
        
        #print("Packet in config_rate_nav ",packet)
        return packet
        
    def convert_to_decimal(self, integer):
        """Converts the integer variable to decimal,
        handles the sign bit (negative numbers)""" 
        if (integer & 0x80000000): # if sign bit is one
            # number is negative
            sign = -1
            # take complement of the number
            complement = 0
            for i in range(0, 32):
                bit = (integer >> i) & 1
                if (bit == 0):
                    complement |= (1 << i)
            # add one
            decimal = complement + 1
        else:
            # number is positive
            sign = 1
            decimal = integer
    
        # return number with its sign
        decimal = float(decimal) * sign
     
        return decimal
   
    def receive_ubx_bytes_from_spi(self):
        # allocate receive buffer
        buffer_size = 1024 #2048  4096 * 2 #4096

        received_bytes = bytearray(buffer_size)
        
        start = False  # flag showing that synchronization character is found
        i = 0 # iteration
        j = 0 #
        
        while (j < 2048):  # 8960 max. number of iterations 40960

            byte = self.spiport.readbytes(1)[0] # read one byte from spi
            if (byte == 0xb5): # if synchronization character is found
                start = True

            if (start == True): # if the packet start is found
                received_bytes[i] = byte # put byte to the buffer
                i = i + 1

            if (start == True and i == buffer_size):
                break # if buffer is full, exit loop

            j = j + 1
        # number of bytes read
        
        return received_bytes

    
    def check_ubx_uart(self, received_bytes):
        read_bytes = len(received_bytes)
    
        header = False    # packet header flag
        
        if (read_bytes > 0):   
            i = 1
            while (i < read_bytes): # read until no bytes left
                
                if (header == False and received_bytes[i - 1] == 0xb5 and received_bytes[i] == 0x62):
                    # check if packet header matches
                    # print("[ubx] found header")
                    #print("[ubx]  ", received_bytes[i - 1])
                    #print("[ubx]  ", received_bytes[i])
                    
                    header = True
                    
                    packet_start = i - 1
                    
                    i = i + 1
                elif (header == True and received_bytes[i - 1] == 0x05 and (received_bytes[i] == 0x01 or received_bytes[i] == 0x00)):
                    # check if class id and msg. id match
                    #print("[ubx] found clsid and msgid")        
                    #print("[ubx]  clsid = ", received_bytes[i - 1])
                    #print("[ubx]  msgid = ", received_bytes[i])
                    
                    if (received_bytes[i] == 0x01):
                        #print("[ubx] UBX-ACK-ACK")
                        pass
                    elif (received_bytes[i] == 0x00):
                        #print("[ubx] UBX-ACK-NACK")
                        pass
                        
                    #if (i + 2 <= read_bytes):
                    #
                    #    size = received_bytes[i + 1] | (received_bytes[i + 2] >> 8)
                    # 
                    #    bytes_buffer = received_bytes[i - 1:i - 1 + size] 
                    # 
                    #    print("[ubx]  bytes = ", bytes_buffer)
                    
                    i = i + 1
                else:
                    i = i + 1
                    


        
    def config_uart2(self, baud):
        
        #print("configuring the navigation rate:")
        nav_rate = self.cfg_rate_nav()
        self.spiport.writebytes(nav_rate)
        received_bytes = self.receive_ubx_bytes_from_spi()
        self.check_ubx_uart(received_bytes)
        
        
        #print("configuring the rate measure:")
        meas_rate = self.cfg_rate_meas()
        self.spiport.writebytes(meas_rate)
        received_bytes = self.receive_ubx_bytes_from_spi()
        self.check_ubx_uart(received_bytes)
        
        """
        time_ref = self.cfg_rate_timeref()
        self.spiport.writebytes(time_ref)
        received_bytes = self.receive_ubx_bytes_from_spi()
        self.check_ubx_uart(received_bytes)
        """
        
        # config uart2 baudrate
        ubx_str = self.prepare_ubx_uart_set_baudrate(baud)
        self.spiport.writebytes(ubx_str)
        #print("uart2 baudrate configuration sent") 
    
        received_bytes = self.receive_ubx_bytes_from_spi()
        #print("Received bytes from set_baudrate",ubx_str)
        self.check_ubx_uart(received_bytes) 
        
        # enable uart2
        ubx_str = self.prepare_ubx_uart_enable()
        
        self.spiport.writebytes(ubx_str)
        #print("uart2 enable configuration sent")
        
        received_bytes = self.receive_ubx_bytes_from_spi()
        #print("Received bytes enable uart2: ",received_bytes)
        self.check_ubx_uart(received_bytes)
        
        

    def update(self, read_data = 1):
        """Check for updated data from the GPS module and process it
        accordingly. Returns True if new data was processed, and False if
        nothing new was received."""
        
        ###        
        # send ubx command for getting gps ned velocity
        ubx_str = self.prepare_ubx(0x01, 0x14) # class and msg ids for ned velocity 12; 14 for HPPOSLLH

        # send and receive over spi
        self.spiport.writebytes(ubx_str)
        #print("UBX string receievd for ned vel",ubx_str)
        
        
        # send a nmea sentence for polling
        nmea_str = self.prepare_nmea("GNGNQ,GGA")
        #print("NMEA string",nmea_str)
        self.spiport.writebytes(nmea_str)       
        
        # nmea_str = self.prepare_nmea("GNGNQ,RMC")
        # self.spiport.writebytes(nmea_str)
        
        # allocate receive buffer

        buffer_size = 1024 #4096  # was 2048, 4096* 2

        received_bytes = bytearray(buffer_size)
        
        start = False  # flag showing that synchronization character is found
        i = 0 # iteration
        j = 0 #

        while (j < 2048): #8960): #1025 was 40960  # max. number of iterations

            byte = self.spiport.readbytes(1)[0] # read one byte from spi
            #print("BYTES: ",byte)
            #byte = byte[0]
            
            if (byte == 0xb5): # if synchronization character is found
                start = True
                #print("Start: ",start)

            if (start == True): # if the packet start is found
                received_bytes[i] = byte # put byte to the buffer
                i = i + 1
                #print("value of i",i)

            if (start == True and i == buffer_size):
                break # if buffer is full, exit loop

            j = j + 1
        
        # number of bytes read
        read_bytes = len(received_bytes)
        #print("[spi] read_bytes = ", read_bytes)
        #print(received_bytes)
        #
        # handles the ubx protocol and extracts ned velocity packet    
        # if there is bytes available on buffer    
        """
        if (read_bytes > 0):
            header = False    # packet header flag
            velNED = False    # velocity flag
            length = 0        # packet length
            packet_start = 0 # variable keeping start byte index of the packet 
            
            i = 1
            while (i < read_bytes): # read until no bytes left
                
                if (header == False and received_bytes[i - 1] == 0xb5 and received_bytes[i] == 0x62):
                    # check if packet header matches
                    # print("[ubx] found header")
                    #print("[ubx]  ", received_bytes[i - 1])
                    #print("[ubx]  ", received_bytes[i])
                    
                    header = True
                    
                    packet_start = i - 1
                    
                    i = i + 1
                elif (header == True and received_bytes[i - 1] == 0x01 and received_bytes[i] == 0x12):
                    # check if class id and msg. id match
                    #print("[ubx] found clsid and msgid")        
                    #print("[ubx]  clsid = ", received_bytes[i - 1])
                    #print("[ubx]  msgid = ", received_bytes[i])
                     
                    velNED = True
                     
                    i = i + 1
                elif (header == True and velNED == True and length == 0):
                    # extract packet length
                    length = (received_bytes[i]) | (received_bytes[i + 1] << 8)
                            
                    #print("[ubx] length = ", length)
                            
                    i = i + 2
                            
                elif (header == True and velNED == True and length > 0):
                    # if everything is correct and packet length is greater than 0, check checksum
                    
                    if (packet_start + 6 + length + 1 >= len(received_bytes) or packet_start + 6 + length >= len(received_bytes)):
                        break
                    # get packet checksum from received bytes
                    chk_packet_a = received_bytes[packet_start + 6 + length]
                    chk_packet_b = received_bytes[packet_start + 6 + length + 1]
                    
                    
                    # calculate checksum from received packet bytes by using the checksum algorithm shown in datasheet
                    chk_a = 0
                    chk_b = 0
                    for j in range(2, 6 + length):
                        chk_a = chk_a + received_bytes[packet_start + j]
                        chk_b = chk_b + chk_a
                                
                    chk_a &= 0xff
                    chk_b &= 0xff
                                
                    # print(chk_a, " == ", chk_packet_a) 
                    # print(chk_b, " == ", chk_packet_b)
                    
                    # if checksum matches, extract velocity and covariance information
                    if (chk_a == chk_packet_a and chk_b == chk_packet_b):
                                
                        # process packet
                        iTOW = (received_bytes[i]) | (received_bytes[i + 1] << 8) | (received_bytes[i + 2] << 16) | (received_bytes[i + 3] << 24)
                        iTOW /= 1000.0
                                
                        vel_n = (received_bytes[i + 4]) | (received_bytes[i + 5] << 8) | (received_bytes[i + 6] << 16) | (received_bytes[i + 7] << 24)
                        vel_e = (received_bytes[i + 8]) | (received_bytes[i + 9] << 8) | (received_bytes[i + 10] << 16) | (received_bytes[i + 11] << 24)
                        vel_d = (received_bytes[i + 12]) | (received_bytes[i + 13] << 8) | (received_bytes[i + 14] << 16) | (received_bytes[i + 15] << 24)
                        
                        vel_n = self.convert_to_decimal(vel_n)
                        vel_e = self.convert_to_decimal(vel_e)
                        vel_d = self.convert_to_decimal(vel_d)
                        
                        # convert ned velocity to enu velocity
                        vel_n /= 100.0
                        vel_e /= 100.0
                        vel_u = vel_d / -100.0
                                
                        # extract std. dev. of ned velocity
                        std_vel = (received_bytes[i + 28]) | (received_bytes[i + 29] << 8) | (received_bytes[i + 30] << 16) | (received_bytes[i + 31] << 24)
                        std_vel /= 100.0
                                
                        #print("[ubx] iTOW (s) = ", iTOW)
                        
                        #print("[ubx] enu vel (m/s) = ", vel_e, vel_n, vel_u)
                        #print("[ubx] cov vel (m^2/s^2) = ", std_vel * std_vel)
                        
                        # assign velocities and covariances to class variables
                        self.enu_velocity = [vel_e, vel_n, vel_u]
                        self.enu_covariances = [0] * 9
                        self.enu_covariances[0] = std_vel * std_vel
                        self.enu_covariances[4] = std_vel * std_vel
                        self.enu_covariances[8] = std_vel * std_vel
                        
                        break
                    else:
                        # checksum mismatch
                        # exit loop
                        #print("[ubx] checksum mismatch")
                                
                        break
                            
                else:
                    # otherwise progress by increasing i by 1
                    i = i + 1
        """
        # for nmea protocol
        # Grab a sentence and check its data type to call the appropriate
        # parsing function.
        if (read_bytes > 0):
            try:
                # read sentences to the variable from spi buffer (into array)
                sentences = self.readSentenceBuffer(received_bytes)
                print("First",sentences)
            except UnicodeError:
                return None

            if sentences is not None:
                
                nSentences = len(sentences) # sentence count in the array
                
                # iterate over sentences
                for i in range(0, nSentences):
                   sentence = sentences[i]
                   try:
                       # print out and parse sentences
                       #print("Sec", sentence)
                       self.datatype = sentence.split(',')[0]
                       #print(type(self.datatype)) #MYEDIT
                       # self.datatype[2:3] = '*'
                       #print(self.datatype)
                       #print("check point 2")
                       if self.datatype == "$GPGGA" or self.datatype == "$GNGGA":
                           self.parse_gpgga(sentence)
                           #print("check point 1")
                       # elif self.datatype == "$GPRMC" or self.datatype == "$GNRMC":
                       # self.parse_gprmc(sentence)
                   except UnicodeError:
                       continue 
        
        #print("exit read")

        return True
    def parse_gpgga(self, sentence):
        data = sentence.split(",")
        #print("check point 3")
        #print(data)
        if data is None or len(data) != 15 or (data[0] == ""):
            return  # Unexpected number of params.
        # Parse fix time.
        #print(data[1])
        try: # Attempting to prevent single time errors
            time_utc = int(_parse_float(data[1]))
        except:
            time_utc = None

        if time_utc is not None:
            hours = time_utc // 10000 + 8
            mins = (time_utc // 100) % 100
            secs = time_utc % 100
            # Set or update time to a friendly python time struct.
            if self.timestamp_utc is not None:
                self.timestamp_utc = time.struct_time((
                    self.timestamp_utc.tm_year,
                    self.timestamp_utc.tm_mon,
                    self.timestamp_utc.tm_mday,
                    hours,
                    mins,
                    secs,
                    0,
                    0,
                    -1,
                ))
            else:
                self.timestamp_utc = time.struct_time(
                    (0, 0, 0, hours, mins, secs, 0, 0, -1))
        # Parse latitude and longitude.
        self.latitude = _parse_degrees(data[2])
        if self.latitude is not None and data[3] is not None and data[3].lower() == "s":
            self.latitude *= -1.0
        self.longitude = _parse_degrees(data[4])
        if (self.longitude is not None and data[5] is not None and data[5].lower() == "w"):
            self.longitude *= -1.0
        # Parse out fix quality and other simple numeric values.
        # print('check',data[6],'int',int(data[6]))
        self.fix_quality = _parse_int(data[6])
        self.satellites = _parse_int(data[7])
        self.horizontal_dilution = _parse_float(data[8])

        self.altitude_m = _parse_float(data[9])
        self.height_geoid = _parse_float(data[11])

        return 0


    def parse_gprmc(self, sentence):
        # Parse the arguments (everything after data type) for NMEA GPRMC
        # minimum location fix sentence.
        data = sentence.split(",")
        if data is None or len(data) < 11 or data[0] is None or (data[0]== ""):
            return  # Unexpected number of params.
        # Parse fix time.
        time_utc = int(_parse_float(data[1]))
        if time_utc is not None:
            hours = time_utc // 10000 + 8
            mins = (time_utc // 100) % 100
            secs = time_utc % 100
            # Set or update time to a friendly python time struct.
            if self.timestamp_utc is not None:
                self.timestamp_utc = time.struct_time((
                    self.timestamp_utc.tm_year,
                    self.timestamp_utc.tm_mon,
                    self.timestamp_utc.tm_mday,
                    hours,
                    mins,
                    secs,
                    0,
                    0,
                    -1,
                ))
            else:
                self.timestamp_utc = time.struct_time(
                    (0, 0, 0, hours, mins, secs, 0, 0, -1))
        # Parse status (active/fixed or void).
        status = data[2]
        self.fix_quality = 0
        if status is not None and status.lower() == "a":
            self.fix_quality = 1
        # Parse latitude and longitude.
        self.latitude = _parse_degrees(data[3])
        if self.latitude is not None and data[4] is not None and data[4].lower(
        ) == "s":
            self.latitude *= -1.0
        self.longitude = _parse_degrees(data[5])
        if (self.longitude is not None and data[6] is not None
                and data[6].lower() == "w"):
            self.longitude *= -1.0
        # Parse out speed and other simple numeric values.
        self.speed_knots = _parse_float(data[7])
        self.track_angle_deg = _parse_float(data[8])
        # Parse date.
        if data[8] is not None and len(data[9]) == 6:
            day = int(data[9][0:2])
            month = int(data[9][2:4])
            year = 2000 + int(
                data[9][4:6])  # Y2k bug, 2 digit year assumption.
            # This is a problem with the NMEA
            # spec and not this code.
            if self.timestamp_utc is not None:
                # Replace the timestamp with an updated one.
                # (struct_time is immutable and can't be changed in place)
                self.time.struct_time((
                    year,
                    month,
                    day,
                    self.timestamp_utc.tm_hour,
                    self.timestamp_utc.tm_min,
                    self.timestamp_utc.tm_sec,
                    0,
                    0,
                    -1,
                ))
            else:
                # Time hasn't been set so create it.
                self.timestamp_utc = time.struct_time(
                    (year, month, day, 0, 0, 0, 0, 0, -1))
                    

if __name__ == '__main__':
    try:
        # init node
        rospy.init_node('rtk', anonymous=True)
        gps_pub = rospy.Publisher('am_gps_urcu', NavSatFix, queue_size=10)

        gps_status_pub = rospy.Publisher('am_gps_urcu_status', UInt8, queue_size=10)
        rate = rospy.Rate(10)  # 10hz
        
        # start to run
        spiport = spidev.SpiDev()
        spiport.open(2, 0)  # (2,0) for spi1, (0,0) for spi0

        spiport.max_speed_hz = 7800000 #12500000 #7800000 #1000000....

        #i = 0

        gps = GPS(spiport)
        
        gps.config_uart2(38400) #38400
        
        #ret_bytes = gps.config_nav()
        while not rospy.is_shutdown():
            gps.update()
            if gps.latitude is not None and gps.latitude != 0:
                gpsfix = NavSatFix()
                gpsfix.header.stamp = rospy.Time.now()
                gpsfix.header.frame_id = 'gps_frame'  # FRAME_ID
                gpsfix.latitude = gps.latitude
                gpsfix.longitude = gps.longitude
                gpsfix.altitude = gps.altitude_m
                gpsfix.position_covariance_type = NavSatFix.COVARIANCE_TYPE_APPROXIMATED
                gpsfix.position_covariance[0] = (gps.horizontal_dilution*0.1)**2 #std_dev error position estimate = 0.1
                gpsfix.position_covariance[4] = (gps.horizontal_dilution*0.1)**2 #std_dev error position estimate = 0.1
                gpsfix.position_covariance[8] = (4*gps.horizontal_dilution*0.1)**2 #std_dev error position estimate = 0.1
                gps_pub.publish(gpsfix)

            """
            if gps.enu_velocity is not None and gps.enu_covariances is not None:
                gpsfixvel = TwistWithCovarianceStamped()
                gpsfixvel.header.stamp = rospy.Time.now()
                gpsfixvel.header.frame_id = 'gps_frame'  # FRAME_ID
               
                gpsfixvel.twist.twist.linear.x = gps.enu_velocity[0]
                gpsfixvel.twist.twist.linear.y = gps.enu_velocity[1]
                gpsfixvel.twist.twist.linear.z = gps.enu_velocity[2]
               
                gpsfixvel.twist.covariance[0] = gps.enu_covariances[0]
                gpsfixvel.twist.covariance[4] = gps.enu_covariances[4]
                gpsfixvel.twist.covariance[8] = gps.enu_covariances[8]
               
                gps_vel_pub.publish(gpsfixvel)
            """
            gps_status_pub.publish(gps.fix_quality)

            rate.sleep()

    except rospy.ROSInterruptException:
        pass

 
