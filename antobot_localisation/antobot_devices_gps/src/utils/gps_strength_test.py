#!/usr/bin/env python3
# Copyright (c) 2023, ANTOBOT LTD.
# All rights reserved

# # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # 

# # # Code Description:     Run this code to check if the GPS module is getting enough satellite information.
##                          Also it can check the strength of GPS signal, uses the message type UBX-NAV-SAT
##                          Number of satellites for a constellation of 5 is 5*4; each constellation needs atleast 4 satellite to be in fixed mode

# Contact: Daniel Freer 
# email: daniel.freer@antobot.ai

# # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # #

import spidev
import time
import sys

from antobot_devices_gps.ublox_gps import UbloxGps

from gps_f9p import F9P_GPS

class gps_strength():
    def __init__(self):
        self.gps = F9P_GPS("urcu")
        #baud = 38400
        #port = spidev.SpiDev()
        #gps = UbloxGps(port)
        #self.gps_info = gps.gps_spi.satellites()
        self.cno_threshold = 38 #signal strength threshold
        #print("gps info",gps_info)

    def main(self,args):
        
        try:
            while True:
                num_satellites = 0
                gps_info = self.gps.gps_spi.satellites()
                #print("GPS_info",gps_info)
                num_sat = gps_info.numSvs
                #print("Number of satellites available: ",num_sat)
                for i in range(num_sat):
                    #cno value should preferably be >38dBz to get good satellite signal
                    #If cno is <38 means there are RFI or EMI
                    print ("strength of GNSS signal in dBz: ",gps_info.RB[i].cno)
                    if (gps_info.RB[i].cno >= gps_str.cno_threshold and gps_info.RB[i].flags.svUsed==1): 
                            #print("Satellite used",gps_info.RB[i].flags.svUsed,type(gps_info.RB[i].flags.svUsed)) 
                            #if (gps_info.RB[i].flags.svUsed==1):
                            num_satellites +=1
                            print("Checking if RTCM correction message is used",gps_info.RB[i].flags.rtcmCorrUsed)
                            #print ("strength of GNSS signal in dBz: ",gps_info.RB[i].cno)
                    #if 
                print("Number of usable satellites: ",num_satellites)
                #num_satellites = 0 #check if necessary
        except:
            pass

if __name__ == '__main__':
            
    gps_str = gps_strength()
    gps_str.main(sys.argv)
    
