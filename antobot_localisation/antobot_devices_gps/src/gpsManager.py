#!/usr/bin/env python3

# Copyright (c) 2024, ANTOBOT LTD.
# All rights reserved.

# # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # #

# # # Code Description:     gpsManager is intended to launch and manage the appropriate GPS scripts and data.

# Contacts: daniel.freer@antobot.ai

# # # #  # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # #

import yaml
import socket
import serial
import asyncio

import rospy

import roslaunch
import rospkg

from antobot_urcu.launchManager import Node, RoslaunchWrapperObject
# from antobot_manager_msgs.srv import netMonitorLaunch, netMonitorLaunchResponse

from gps_f9p import F9P_GPS
from gps_movingbase import MovingBase_Ros
from gps_corrections import gpsCorrections

from utils.f9p_config import configure_f9p

import importlib
#import Jetson.GPIO as GPIO


class gpsManager():
    def __init__(self):

        self.launch_nodes = False
        self.launch_corrections = "ntrip"
        self.use_class = True

        self.gps_nodes = []
        self.dual_gps = 'false'
        self.movingbase = False
        self.urcu_gps_node = None

        # Create serial interface placeholder
        self.f9p_usb_port = None
        self.f9p_urcu_serial_port = None
        self.baud = 460800
        self.method = "stream"  #"stream" or "poll"


        # Read gps config
        gps_data = self.read_gps_config()

        # Initialise ROS Launcher
        self._launch = roslaunch.scriptapi.ROSLaunch()
        self._launch.start()

        # Launch GPS nodes
        for k, v in gps_data.items():
            #if k == "urcu" or "movingbase":
                #GPIO = importlib.import_module("Jetson.GPIO")
            if self.launch_nodes:
                # Launch the appropriate GPS script
                exec_name, node_name = self.get_node_name(k, v)
                self.createLauncher(exec_name, node_name)
                print("launching executable {}".format(exec_name))
            elif self.use_class:
                gps_cls_tmp = self.get_gps_class(k, v)
                self.gps_nodes.append(gps_cls_tmp)

        # Launch corrections node - should it be launched directly from SW manager? will be removed after test
        """
        if self.launch_corrections == "mqtt":
            self.node_c = Node(name_arg="gpsCorrections", package_arg="antobot_devices_gps", executable_arg="gps_corrections.py", err_code_id="SW234", node_type="sensor", name_space="/", input_args=[])
            self.node_c.define_node()
            self.node_c.launch(self._launch)
        elif self.launch_corrections == "ntrip":
            self.node_c = Node(name_arg="gpsCorrections", package_arg="antobot_devices_gps", executable_arg="gps_corrections_new.py", err_code_id="SW234",node_type="sensor", name_space="/", input_args= [] )
            self.node_c.define_node()
            self.node_c.launch(self._launch)
        """
        return

    def read_gps_config(self):
        
        gps_data = None
        device_type = None

        rospack = rospkg.RosPack()
        packagePath=rospack.get_path('antobot_description')
        path = packagePath + "/config/platform_config.yaml"

        with open(path, 'r') as yamlfile:
            data = yaml.safe_load(yamlfile)
            gps_data = data['gps']

        return gps_data

    def get_node_name(self, k, v):

        node_name = "gps_" + k

        if k == "urcu":
            exec_name = "gps_f9p.py"
        if k == "movingbase":
            exec_name = "gps_movingbase.py"
        if k == "f9p_usb" or k == "f9p_usb2":
            exec_name = "gps_f9p.py"

        return exec_name, node_name
    
    def createLauncher(self, exec_name, node_name):

        # Generate a unique id for the node
        uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
        roslaunch.configure_logging(uuid)

        cli_args = ['antobot_devices_gps', 'gps_config.launch', 'exec_name:='+exec_name, 'node_name:='+node_name, 'dual_gps:='+self.dual_gps]

        roslaunch_file = roslaunch.rlutil.resolve_launch_arguments(cli_args)[0]
        print(roslaunch_file)
        roslaunch_args = cli_args[2:]

        launch_files = [(roslaunch_file, roslaunch_args)]

        launcher = RoslaunchWrapperObject(run_id = uuid, roslaunch_files = launch_files)

        # start the launch file
        launcher.start_node_name(node_name)

        return launcher
    
    def get_gps_class(self, k, v):

        baud=460800 #460800

        if k == "urcu":

            # Define serial port for the F9P inside of the uRCU (for movingbase or otherwise)
            if self.f9p_urcu_serial_port == None:
                self.f9p_urcu_serial_port = serial.Serial(v['device_port'], baud)

            # Define the class object
            gps_cls = F9P_GPS("urcu",method=self.method)
            self.urcu_gps_node = gps_cls

        if k == "movingbase":
            # Define serial port for the movingbase F9P (connected via USB)

            if self.f9p_usb_port == None:
                self.f9p_usb_port = serial.Serial(v['device_port'], baud)

            # Define the class object
            gps_cls = MovingBase_Ros(self.f9p_urcu_serial_port, self.f9p_usb_port, None)
        if k == "f9p_usb" or k == "f9p_usb2":
            if self.f9p_usb_port == None:
                self.f9p_usb_port = serial.Serial(v['device_port'], 38400)
            gps_cls = F9P_GPS("usb", serial_port=self.f9p_usb_port, method=self.method,pub_name="antobot_" + k)

        return gps_cls
    
    def check_gps(self,event=None):
        # Checks whether there have been any changes to the robot's network

        if not self.launch_nodes:
            for gps_node in self.gps_nodes:
                if gps_node.node_type == "gps_f9p":
                    self.check_gps_node(gps_node)

        return
    




    async def check_gps_async(self):

        MB = await self.create_MovingBase()
        while not rospy.is_shutdown(): # this isn't rate limited, so shouldn't have wasted time
            try:

                self.check_gps_node(self.urcu_gps_node)

                headFrame = await MB.get_RELPOSNEDframe()
                self.pub_head(headFrame)

            except:
                rospy.logerr(f"MovingBase: Close the MovingBase node")
                break
        return
    
    def check_gps_node(self, gps_node):
        # If no longer running, re-launch

        gps_node.get_gps()


        # Publish any changes to fixed status, frequency, etc.

        return
    

def main():
    rospy.init_node ('gpsManager')

    # configure_f9p()

    gpsMgr = gpsManager()

    # If movingbase is being used with dual-GPS
    if gpsMgr.movingbase:
        loop = asyncio.get_event_loop()
        try:
            loop.run_until_complete(gpsMgr.check_gps_async)
        except Exception as e:     
            GPIO = importlib.import_module("Jetson.GPIO")
            GPIO.cleanup()
            loop.close()
    else:       # Most other situations
        rospy.Timer(rospy.Duration(0.02), gpsMgr.check_gps)  # Runs periodically without blocking
        rospy.spin() 
        

if __name__ == '__main__':
    main()
