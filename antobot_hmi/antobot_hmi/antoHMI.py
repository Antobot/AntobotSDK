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
import rclpy
from rclpy.node import Node
import time
import threading
import serial.tools.list_ports as ports
from std_msgs.msg import Bool, UInt8,String,Float32, Int32
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import Twist


class HMIBridge(Node):
    def __init__(self):
        #Inits the class Minitor with all its variables
        super().__init__('anto_HMI')
        #pages
        self.startup=1
        self.inoperation=2
        self.poweroff1 =3 #searching for GPS signal
        self.poweroff2=4
        self.current_page = 2
        #variables
        self.A2X_bUp = 1
        self.A2X_bMiddle = 1
        self.A2X_bBottom = 1
        self.A2X_bUp_pre = 1
        self.A2X_bMiddle_pre = 1
        self.A2X_bBottom_pre = 1   
        self.button_sum =0  #int     
        
        
        
        self.cs_status = False
        self.A2X_header1 = 56
        self.A2X_header2 = 89
        self.A2X_bPoweroff=False
        self.X2A_bPower = 0
        self.X2A_Header=[56,89] #int
        self.X2A_uBat = 100 #SoC
        self.X2A_uPage = 0 #int
        #self.X2A_uSoc = 0 #int
        self.X2A_LinearA = 0 #int
        self.X2A_LinearB = 0 #int
        self.X2A_uGPS = 0 #int
        self.A2X_uPage = 0
        self.A2X_cs = 0
        self.data_decoded = []
        self.cs_status = False
        self.A2X_uSoc = 0
        self.A2X2_bUp = 1
        self.A2X2_bMiddle = 1
        self.A2X2_bBottom = 1
        self.A2X2_bUp_pre = 1
        self.A2X2_bMiddle_pre = 1
        self.A2X2_bBottom_pre = 1 
        
        self.A2X2_uSoc = 0
        self.A2X2_uPage = 0
        self.A2X2_cs = 0
        self.A2X2_cs_status = False
        self.A2X2_data_decoded = []
        
        #serial communication init
        self._primary_port_path = '/dev/anto_hmi_1'
        self._secondary_port_path = '/dev/anto_hmi_2'
        self._enable_secondary_hmi = True
        self.AdPort = self._open_serial(self._primary_port_path, "primary")
        self.AdPort2 = None
        if self._enable_secondary_hmi:
          self.AdPort2 = self._open_serial(self._secondary_port_path, "secondary")
        self._serial_retry_period = 2.0
        self._serial_retry_timer = self.create_timer(
            self._serial_retry_period,
            self._ensure_serial_ports,
        )
        
        #ROS subscriber 

        self.sub_GPS = self.create_subscription(NavSatFix, "/antobot_gps", self.gps_callback, 10)
        self.sub_cmd_vel = self.create_subscription(Twist,"/antobot/robot/cmd_vel",self.vel_callback,10)
        self.sub_soft_shutdown_button = self.create_subscription(Bool, '/antobridge/soft_shutdown_button', self.soft_shutdown_callback, 10)
        
        #ROS Publisher
        self.pub_soc = self.create_publisher(UInt8, '/HMI/soc', 1)  #pub soc from bmi
        self.pub_soft_shutdown_button = self.create_publisher(Bool, "/antobridge/soft_shutdown_button", 1)
        
        
    def _readline_if_available(self, port, timeout: float = 0.0):
        """Return one line from *port* only if bytes are available."""
        if port is None:
            return b''

        try:
            waiting = port.in_waiting
        except (serial.SerialException, OSError) as exc:
            self.get_logger().warning(f"Serial in_waiting failed: {exc}")
            return b''

        if timeout > 0.0 and waiting == 0:
            deadline = time.monotonic() + timeout
            while port.in_waiting == 0 and time.monotonic() < deadline:
                time.sleep(0.001)

        if port.in_waiting == 0:
            return b''

        try:
            return port.readline()
        except serial.SerialException as exc:
            self.get_logger().warning(f"Serial read failed: {exc}")
            return b''    


    def A2X_read(self): #read request
        Ad_dataHead =b'3859'
        
        #while self.AdPort.in_waiting:
        #A2X_data = self.AdPort.read(self.AdPort.in_waiting + 17)
        if self.AdPort is not None:
            try:
                if self.AdPort.in_waiting > 0:
                    while self.AdPort.in_waiting > 0:
                        A2X_data = self.AdPort.readline()
        #A2X_data = self._readline_if_available(self.AdPort, timeout=0.005)
                        if A2X_data:
                            print("port 1 data read:", A2X_data)
                            if A2X_data[0:4] == Ad_dataHead: # and len(A2X_data)==12):
                                #print("Entering the OCR reception mode:")
                                A2X_data = A2X_data.decode('utf-8')
                                if int('0x'+A2X_data[0:2],16)== 56:
                                    if (int('0x'+A2X_data[2:4],16)) == 89:

                                        #print("Header pass!!!")

                                        #splitting the payload from the data received
                                        self.A2X_bUp = int('0x'+A2X_data[4],16)
                                        self.A2X_bMiddle = int('0x'+A2X_data[5],16)
                                        self.A2X_bBottom = int('0x'+A2X_data[6],16)
                                        self.A2X_uSoc= int('0x'+A2X_data[7:9],16)
                                        self.A2X_uPage = int('0x'+A2X_data[9:11],16)   
                                        self.A2X_cs = int('0x'+A2X_data[11:13],16)
                                        #print("self.A2X_cs",self.A2X_cs)
                                        #calling function to check the checksum
                                        check_cs = self.A2X_checkCs(A2X_data)
                                        #print("Returned received data:",check_cs[0])
                                        cs_received = check_cs[1]
                                        #print("cs_received[-2:]:",cs_received[-2:])

                                        if cs_received[-2:] == 'aa': #checksum passed
                                            self.cs_status = True
                                            self.data_decoded = check_cs[0]
                                            #print("Data decoded after checksum", self.data_decoded)

                                            
                                            #break
                                        else:
                                            self.cs_status = False
                                            print("checksum failed") #set to default request value
                                            self.data_decoded =[0,0,0,0,0,0,0,0]
                                            self.A2X_bUp = 0
                                            self.A2X_bMiddle = 0
                                            self.A2X_bBottom = 0
                                            self.A2X_uSoc= 0
                                            self.A2X_uPage = 0
                                            #break

                            else:
                                self.cs_status = False
                                self.data_decoded =[0,0,0,0,0,0,0,0]

                                #break
                        else:
                            self.cs_status = False
            except (serial.SerialException, OSError) as exc:
                self.get_logger().warning(f"Primary HMI serial error: {exc}")
                self._mark_port_unavailable('primary', self.AdPort)
            except Exception as e:
                self.get_logger().error(f"Serial error: {e}")

        if self.AdPort2:
            self._read_secondary_port(Ad_dataHead)

        
        soc_msg = UInt8()
        soc_msg.data = min([x for x in (int(self.A2X_uSoc) ,int(self.A2X2_uSoc)) if x!=0], default=0)
        self.pub_soc.publish(soc_msg)
        return self.data_decoded

    def _read_secondary_port(self, Ad_dataHead):
        """Read button and SoC data from the optional /dev/ttyACM1 port."""
        #A2X_data = self._readline_if_available(self.AdPort2, timeout=0.005)
        if self.AdPort2 is not None:
            try:
                if self.AdPort2.in_waiting <= 0:
                    return
                while self.AdPort2.in_waiting > 0:
                    A2X_data = self.AdPort2.readline()
                    try:
                        A2X_data = A2X_data.decode('utf-8')
                    except UnicodeDecodeError:
                        self.get_logger().warning("Failed to decode secondary HMI payload")
                        return

                    if int('0x'+A2X_data[0:2],16)!= 56:
                        return
                    if int('0x'+A2X_data[2:4],16)!= 89:
                        return
                    #print("port 2 data read:", A2X_data)
                    self.A2X2_bUp = int('0x'+A2X_data[4],16)
                    self.A2X2_bMiddle = int('0x'+A2X_data[5],16)
                    self.A2X2_bBottom = int('0x'+A2X_data[6],16)
                    self.A2X2_uSoc= int('0x'+A2X_data[7:9],16)
                    self.A2X2_uPage = int('0x'+A2X_data[9:11],16)
                    self.A2X2_cs = int('0x'+A2X_data[11:13],16)

                    check_cs = self.A2X_secondary_checkCs(
                        A2X_data,
                        self.A2X2_bUp,
                        self.A2X2_bMiddle,
                        self.A2X2_bBottom,
                        self.A2X2_uSoc,
                        self.A2X2_uPage,
                        self.A2X2_cs
                    )
                    cs_received = check_cs[1]
                    if cs_received[-2:] == 'aa':
                        self.A2X2_cs_status = True
                        self.A2X2_data_decoded = check_cs[0]
                        #print("Data decoded after checksum 2", self.A2X2_data_decoded)
                    else:
                        self.A2X2_cs_status = False
                        self.A2X2_data_decoded =[0,0,0,0,0,0,0,0]
                        #self.A2X2_bUp = 0
                        #self.A2X2_bMiddle = 0
                        #self.A2X2_bBottom = 0
                        #self.A2X2_uSoc= 0
                        #self.A2X2_uPage = 0
            except (serial.SerialException, OSError) as exc:
                self.get_logger().warning(f"Secondary HMI serial error: {exc}")
                self._mark_port_unavailable('secondary', self.AdPort2)
            except Exception as e:
                self.get_logger().error(f"Serial error: {e}")



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
        received_data.append(self.A2X_bUp)
        received_data.append(self.A2X_bMiddle)
        received_data.append(self.A2X_bBottom)
        received_data.append(self.A2X_uSoc)
        received_data.append(self.A2X_uPage)
        received_data.append(self.A2X_cs)
        #print("Received data:",received_data)
        cs = self.checksum(data)
        #print("Check checksum:",cs)
        return [received_data,cs]

    def A2X_secondary_checkCs(self,data,bUp,bMiddle,bBottom,uSoc,uPage,uCs):
        received_data = []
        received_data.append(int('0x'+data[0:2],16))
        received_data.append(int('0x'+data[2:4],16))
        received_data.append(bUp)
        received_data.append(bMiddle)
        received_data.append(bBottom)
        received_data.append(uSoc)
        received_data.append(uPage)
        received_data.append(uCs)
        cs = self.checksum(data)
        return [received_data,cs]

    def _write_serial_async(self, payload_bytes):
        """Write payload to every available serial port in parallel threads."""
        threads = []
        for label, port in (('primary', self.AdPort), ('secondary', self.AdPort2)):
            if port is None:
                continue
            thread = threading.Thread(
                target=self._write_single_port,
                args=(port, label, payload_bytes),
                daemon=True,
            )
            threads.append(thread)
            thread.start()

        for thread in threads:
            thread.join(timeout=0.05)

    def _write_single_port(self, port, label, payload_bytes):
        try:
            port.write(payload_bytes)
        except (serial.SerialException, OSError) as exc:
            self.get_logger().warning(f"{label.capitalize()} HMI serial write failed: {exc}")
            self._mark_port_unavailable(label, port)

    def _open_serial(self, port_path: str, label: str):
        try:
            port = serial.Serial(
                port_path,
                baudrate=115200,
                write_timeout=1,
                timeout=0.01,
            )
        except (serial.SerialException, OSError) as exc:
            self.get_logger().warning(
                f"{label.capitalize()} HMI serial port '{port_path}' not available: {exc}"
            )
            return None
        self.get_logger().info(f"Connected to {label} HMI serial port: '{port_path}'")
        return port

    def _ensure_serial_ports(self):
        if self.AdPort is None:
            self.AdPort = self._open_serial(self._primary_port_path, "primary")
        if self._enable_secondary_hmi and self.AdPort2 is None:
            self.AdPort2 = self._open_serial(self._secondary_port_path, "secondary")

    def _mark_port_unavailable(self, label: str, port):
        if port is None:
            return
        try:
            port.close()
        except Exception:
            pass
        if label == 'primary' and self.AdPort is port:
            self.AdPort = None
        elif label == 'secondary' and self.AdPort2 is port:
            self.AdPort2 = None


    def X2A_write(self): #send feedback, write
        
        payload_parts = [
              f'{self.X2A_Header[0]:02x}',
              f'{self.X2A_Header[1]:02x}',
              f'{self.X2A_uPage:02x}',
              f'{self.X2A_LinearA:02x}',
              f'{self.X2A_LinearB:02x}',
              f'{self.X2A_uGPS & 0xF:x}',
          ]
        payload_prefix = ''.join(payload_parts)
        checksum_hex = self._x2a_ascii_lrc(payload_prefix)
        payload_hex = payload_prefix + checksum_hex + '\n'
        print("data write:",payload_hex)
        payload_bytes = payload_hex.encode('utf-8')
        self._write_serial_async(payload_bytes)
        


    def checksum(self, data: str) -> str:
        """ASCII-LRC: sum(payload ASCII) & 0xFF plus CS must equal 0xAA."""
        payload = data.strip()
        if len(payload) < 3:
            return 'ff'

        body = payload[:-2]
        checksum_digits = payload[-2:]
        try:
            checksum_value = int(checksum_digits, 16)
        except ValueError:
            self.get_logger().warning(
                f"Invalid checksum digits '{checksum_digits}' in HMI payload {payload!r}"
            )
            return 'ff'

        ascii_total = sum(ord(char) for char in body)
        combined = ((ascii_total & 0xFF) + checksum_value) & 0xFF
        return f'{combined:02x}' 

    def _x2a_ascii_lrc(self, payload_prefix: str, seed: int = 0xAA) -> str:
        """Checksum used by x2a_proto: sum(ASCII payload) + cs == 0xAA."""
        ascii_bytes = payload_prefix.encode('ascii', errors='strict')
        total = sum(ascii_bytes)
        checksum_value = (seed - (total & 0xFF)) & 0xFF
        return f'{checksum_value:02x}'

    
    def state_machine(self):
        #logic of state transit
        if  self.cs_status == True:
            if self.button_sum > 0:   #has button press
                match self.current_page:
                    case self.inoperation:
                        if self.button_sum ==100:
                            #print("go to shutdown")
                            self.current_page = self.poweroff1
                    
                    case self.poweroff1:
                        if self.button_sum ==100:   #yes
                            self.pub_soft_shutdown_button.publish(1)
                            self.current_page = self.poweroff2
                        elif self.button_sum==1:
                            self.current_page = self.inoperation
            else:
                if self.X2A_bPower == 1:
                    self.current_page=self.poweroff2
                
        return


    def battery_soc_callback(self,data):
        self.X2A_uBat = data.data
        return

    def soft_shutdown_callback(self,soft_shutdown):  
        # # # Soft shutdown button on joystick pressed
        if soft_shutdown.data == True:
            self.X2A_bPower = 1
        return
        
    def ButtonPress(self): 
        #function to check which button(es) is being pressed
        
        self.button_sum = 0
        if (self.A2X_bUp == 0 and self.A2X_bUp_pre == 1):
            self.button_sum = self.button_sum + 100
        if (self.A2X_bMiddle == 0 and self.A2X_bMiddle_pre == 1) :       
            self.button_sum = self.button_sum + 10;       
        if (self.A2X_bBottom == 0 and self.A2X_bBottom_pre == 1) :
            self.button_sum = self.button_sum + 1
        if (self.A2X2_bUp == 0 and self.A2X2_bUp_pre == 1):
            self.button_sum = self.button_sum + 100
        if (self.A2X2_bMiddle == 0 and self.A2X2_bMiddle_pre == 1) :       
            self.button_sum = self.button_sum + 10;       
        if (self.A2X2_bBottom == 0 and self.A2X2_bBottom_pre == 1) :
            self.button_sum = self.button_sum + 1
        
        self.A2X_bUp_pre = self.A2X_bUp
        self.A2X_bMiddle_pre = self.A2X_bMiddle
        self.A2X_bBottom_pre = self.A2X_bBottom
        self.A2X2_bUp_pre = self.A2X2_bUp
        self.A2X2_bMiddle_pre = self.A2X2_bMiddle
        self.A2X2_bBottom_pre = self.A2X2_bBottom
        # print("button_sum = ",self.button_sum)
        # print("button Up = ",self.A2X_bUp and self.A2X2_bUp)
        # print("button Middle = ",self.A2X_bMiddle and self.A2X2_bMiddle)
        # print("buttonBottom = ",self.A2X_bBottom and self.A2X2_bBottom)


        #delay(10);
        




    def gps_callback(self,data):
        if data.status.status==1:
          self.X2A_uGPS = 2
        elif data.status.status==3:
          self.X2A_uGPS = 1
        else:
          self.X2A_uGPS = 0
       

    def vel_callback(self,data):
      # Split linear.x into integer and two-digit decimal parts and encode sign in LinearB MSB
      try:
        val = float(data.linear.x)
      except Exception:
        val = 0.0
      is_negative = val < 0.0
      abs_val = -val if is_negative else val
      if abs_val > 255.99:
        abs_val = 255.99
      integer_part = int(abs_val)
      decimal_part = int((abs_val - integer_part) * 100)
      if decimal_part > 99:
        decimal_part = 99
      # Encode sign in MSB of LinearB: 1 = negative, 0 = positive
      linear_b = decimal_part | 0x80 if is_negative else decimal_part
      self.X2A_LinearA = integer_part
      self.X2A_LinearB = linear_b
      #decode: is_negative = (linear_b & 0x80) != 0
      #        decimal = linear_b & 0x7F  # 0..99
      #        value = float(linear_a) + decimal / 100.0
      #        return -value if is_negative else value

    def loop(self,event=None):

        #print("main loop")        
        self.ButtonPress()
        self.state_machine()
        #print("before write") 
        self.X2A_write()
        
def main():
    rclpy.init() 
    HMIBridgeNode = HMIBridge()
    
    try:
        HMIBridgeNode.create_timer(0.05, HMIBridgeNode.A2X_read)
        HMIBridgeNode.create_timer(0.1, HMIBridgeNode.loop)  # Runs periodically without blocking
        rclpy.spin(HMIBridgeNode) 

    except Exception as e:
        print("Exception occured!!",e)
    finally:
        HMIBridgeNode.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

