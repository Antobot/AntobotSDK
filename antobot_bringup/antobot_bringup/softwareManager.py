#!/usr/bin/env python3

# Copyright (c) 2024, ANTOBOT LTD.
# All rights reserved.

# # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # #

# # # Code Description:     softwareManager is intended to monitor how the various software components are running and 
#                           intervene if necessary

# Contacts: daniel.freer@antobot.ai

# # # #  # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # #

import os
import yaml
import subprocess

import rclpy
from rclpy.node import Node

from std_srvs.srv import Trigger
from ament_index_python.packages import get_package_share_directory

class softwareManager(Node):
    def __init__(self):
        super().__init__("softwareManager")
        self.srvLaunchEkf = self.create_service(Trigger, "launch_ekf", self.launchEkf)
    
    def launchEkf(self, request, response):
        """
        Launch roslaunch file using subprocess
        """  

        packagePath = get_package_share_directory('antobot_description')
        device_config_path = packagePath + "/config/platform_config.yaml"
        robot_hardware = False
        with open(device_config_path, 'r') as yamlfile:
            data = yaml.safe_load(yamlfile)

            robot_hardware = data['robot_hardware']

        package = 'antobot_ekf'
        node_name = 'ekf.launch.py'

        if robot_hardware:
            command = "ros2 launch {0} {1}".format(package, node_name)
        else:
            command = "ros2 launch {0} {1} use_sim_time:=true".format(package, node_name)

        print(command)
        p = subprocess.Popen(command, shell=True)
        response.success = True
        return response


def main():
    rclpy.init() 
    swMgr = softwareManager()
    rclpy.spin(swMgr) 


if __name__ == '__main__':
    main()