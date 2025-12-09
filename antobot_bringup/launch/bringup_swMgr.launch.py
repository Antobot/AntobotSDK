# Copyright (c) 2023, ANTOBOT LTD.
# All rights reserved.

# # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # 

# # # Code Description:     Launches the necessary files for operation and autonomous navigation of the v2 robot when using
# 							softwareManager to launch the remaining scripts.

# Contact: daniel.freer@antobot.ai

# # # #  # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # #

import os

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

import antobot_manager_software.nodeInfo as nodeInfo

class bringupSWMgr():

    def __init__(self):

        self.nodeDict = nodeInfo.nodeDict
        self.groupDict = nodeInfo.groupDict
        self.launchDict = nodeInfo.launchDict

        self.generate_launch_description()
        return

    def include_launch_files(self):
        for launchName in self.launchDict:
            launchObj = self.launchDict[launchName]
            self.ld.add_action(launchObj.include_launch())

    
    def generate_launch_description(self):

        node_list = []
        self.ld = LaunchDescription()

        self.include_launch_files()

        # Launch all nodes at the beginning
        for nodeName,node in self.nodeDict.items():
            print(nodeName)
            node_i = node.define_node()
            self.ld.add_action(node_i)

        return self.ld





def generate_launch_description():
    bringup_sw_mgr = bringupSWMgr()
    return bringup_sw_mgr.ld


    