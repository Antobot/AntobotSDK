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

#Description:   This is a python script to update status of robot to HMI and receive request from HMI.
#Interface:     requires permission of /dev/ttyACM0 to run the code
#Inputs:        
#Contact:     zhuang.zhou@antobot.ai

# # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # 

import sys
import serial
import rospy
import time
import serial.tools.list_ports as ports
from std_msgs.msg import Bool, UInt8,String,Float32

class HMIBridge():
    def __init__(self):
        #Inits the class Minitor with all its variables
        #feedback
        self.cs_status = False
        self.X2A_Header = (56,89)
        self.X2A_bPower= False
        self.X2A_uBat = 100 #SoC

        #request
        self.A2X_bPower = 0
 
        #state_machine
        self.CaliState = 0
        self.current_state = 0
        self.job_state = 0 #none 0 for autonomous driving
        self.robot_state_fix = 0

        #serial communication init
        self.AdPort = serial.Serial('/dev/ttyACM0')#/dev/ttyACM0
        self.AdPort.baudrate = 57600
        self.AdPort.write_timeout = 0
        self.AdPort.read_timeout = 0

        #ROS subscriber 

        self.sub_battery_soc = rospy.Subscriber("/as/soc",UInt8,self.battery_soc_callback)
        self.sub_soft_shutdown_button = rospy.Subscriber('/antobridge/soft_shutdown_button',Bool,self.soft_shutdown_callback)
        #ROS Publisher
        self.pub_soft_shutdown_button = rospy.Publisher("/antobridge/soft_shutdown_button", Bool,queue_size = 1)

        #ROS Service
        #receive status of calibration 
        #ROS Client
        #client for request pause and resume

        # Use a client to update calibration when the button is pressed 




    def A2X_read(self): #read request
        Ad_dataHead =b'3859'
        start_time = time.time()
        #print("beforeread!")
        A2X_data = self.AdPort.readline()
        #print("Arduino data read", A2X_data)
        if A2X_data[0:4] == Ad_dataHead:# and len(A2X_data)==12):
            #print("Entering the OCR reception mode:")
            A2X_data = A2X_data.decode('utf-8')
            if int('0x'+A2X_data[0:2],16)== 56:
                if (int('0x'+A2X_data[2:4],16)) == 89:
                    #print("Header pass!!!")
                    #splitting the payload from the data received
                    self.A2X_bPower = int('0x'+A2X_data[4],16)
                    self.A2X_cs = int('0x'+A2X_data[5:7],16)
                    #print("self.A2X_cs",self.A2X_cs)
                    #calling function to check the checksum
                    check_cs = self.A2X_checkCs(A2X_data)
                    #print("Returned received data:",check_cs[0])
                    cs_received = check_cs[1]
                    #print("cs_received[-2:]:",cs_received[-2:])
                    if cs_received[-2:] == '00': #checksum passed
                        self.cs_status = True
                        self.data_decoded = check_cs[0]
                        #print("Data decoded after checksum", self.data_decoded)
                    else:
                        self.cs_status = False
                        print("checksum failed") #set to default request value
                        self.data_decoded =[0,0,0,0]
                        self.A2X_bPower = 0 

        else:
            self.cs_status = False
            self.data_decoded =[0,0,0,0]

        #print("Data decoded return",self.data_decoded)              
        return self.data_decoded

    def A2X_checkCs(self,data):
        """function to check if checksum is satisfied or not
        Args:
        data: data received from Arduino
        Variables:
        received_data: array to split and store the data from Arduino
        cs: varibale to store the checksum
        Return:
        array with received data and the calculated checksum
        """
        received_data = []
        received_data.append(int('0x'+data[0:2],16))
        received_data.append(int('0x'+data[2:4],16))
        received_data.append(self.A2X_bPower)
        received_data.append(self.A2X_cs)
        #print("Received data:",received_data)
        cs = self.checksum(received_data)
        #print("Check checksum:",cs)
        return [received_data,cs]


    def X2A_write(self): #send feedback, write
        data = []
        data_hex = []
        for i in self.X2A_Header:
            data.append(i)

        data.append(self.X2A_bPower)



        for i in data:
            data_hex.append(hex(i)[2:])#.zfill(2))

        data_hex = ''.join(data_hex)
        X2A_uBat_str =str(hex(self.X2A_uBat)[2:]).zfill(2)
        X2A_uv_soc_str = str(hex(self.uv_uSoC)[2:]).zfill(2)
        data.append(self.X2A_uBat)
        cs = self.checksum(data)
        data_hex=data_hex+X2A_uBat_str +'\n'
        #print("data write:",data_hex)
        no_bWritten = self.AdPort.write(data_hex.encode('utf-8'))



    def checksum(self,data):
        """ function to find the checksum of the data
        checksum is the sum of all the int in data
        Args: 
            input data
        Ret: 
    `       Hex value of the checksum"""
        total =0
        for i in data:
            total +=i
        d = 256 - total % 256 #taking sum and converting to int
        e = hex(d)[2:] #the hex number part
        return e 

    
    def state_machine(self):
        #logic of 
        if  self.cs_status == True:
            if self.A2X_bPower == 1:
                self.X2A_bPower = 1
                self.pub_soft_shutdown_button.publish(1)
            


    def battery_soc_callback(self,data):
        self.X2A_uBat = data.data
        return

    def soft_shutdown_callback(self,soft_shutdown):  
        # # # Soft shutdown button on joystick pressed
        if soft_shutdown.data == True:
            self.X2A_bPower = 1


def main():
    rospy.init_node ('anto_HMIbridge') 
    rate = rospy.Rate(50)
    HMIBridgeNode = HMIBridge()
    try:
        while not rospy.is_shutdown():
            #print("main loop")
            HMIBridgeNode.A2X_read()
            #print("after read")
            HMIBridgeNode.state_machine()
            #print("before write") 
            HMIBridgeNode.X2A_write()
            rate.sleep()
    except:
        print("Exception occured!!")

if __name__ == '__main__':
    main()
