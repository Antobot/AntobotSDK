#!/usr/bin/env python3

# Copyright (c) 2024, ANTOBOT LTD.
# All rights reserved.

# # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # #

# # # Code Description:     Reads in a configuration from antobot_bringup/config/software_config.yaml, then returns
#                           the appropriate scripts to launch and their information to softwareManager.

# Contacts: daniel.freer@antobot.ai

# # # #  # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # #

import yaml
from pathlib import Path
import rospkg
from antobot_urcu.launchManager import AntobotSWNode, Launchfile
# from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource, AnyLaunchDescriptionSource
from launch_ros.actions import Node
import os


rospack = rospkg.RosPack()

# # # Configuration folders and files
parent_folder = get_package_share_directory('antobot_bringup')

software_config_path = str(parent_folder) + "/config/software_config.yaml"


# Initial values
robot_hardware = False
urcu_hardware = False
sensor_hardware = False
lidar_hardware = False
location= "uk"
dual_gps = False

navigation_parameter = None

# For Indoor demo
indoor_demo = False


with open(software_config_path, 'r') as yamlfile:
    data = yaml.safe_load(yamlfile)

    device_type = data['device_type']
    master_device = data['master_device']

    jobSoftware = data['jobSoftware']
    
    anto_supervisor = data['anto_supervisor']
    opcua_launch = data['opcua_launch']
    webui_launch = data['webui_launch']
    hmi_hardware = data['hmi_hardware']

    auto_launch = data['auto_launch']

if device_type == "robot":
    print("Loading robot parameters!")
    
    packagePath = get_package_share_directory('antobot_description')

    device_config_path = packagePath + "/config/platform_config.yaml"
    with open(device_config_path, 'r') as yamlfile:
        data = yaml.safe_load(yamlfile)

        # Parameter file locations
        robot_hardware = data['robot_hardware']
        urcu_hardware = robot_hardware
        sensor_hardware = robot_hardware
        lidar_hardware = True
        robot_platform = data['robot_platform']
        robot_version = data['robot_version']
        robot_role = data['robot_role']
        prefix_isolated_cpu = []
        aRCU_cfg = data.get('aRCU', {})
        

        # dual gps
        if 'f9p_2' in data['gps']:
            dual_gps = True
            gps_x_1 = data['gps']['urcu']['px']
            gps_y_1 = data['gps']['urcu']['py']
            gps_z_1 = data['gps']['urcu']['pz']

            gps_x_2 = data['gps']['f9p_2']['px']
            gps_y_2 = data['gps']['f9p_2']['py']
            gps_z_2 = data['gps']['f9p_2']['pz']

            dx = gps_x_1 - gps_x_2
            dy = gps_y_1 - gps_y_2
            dz = gps_z_1 - gps_z_2

            import math
            antenna_baseline = math.sqrt(dx**2 + dy**2 + dz**2)
            
            port_movingrover = data['gps']['f9p_2']['device_port']


        if aRCU_cfg:
            aRCU_ssh_list = [
                aRCU_cfg.get('user', ''),
                aRCU_cfg.get('host', ''),
                aRCU_cfg.get('ws', '')
            ]
        else:
            aRCU_ssh_list = []
        
        # prefix for isolated_cpu
        if data.get('isolated_cpu'):
            prefix_isolated_cpu = ['taskset -c 5']

    # navigation parameters
    navigation_parameter = packagePath + "/config/navigation_parameter.yaml"

    # control parameters
    control_parameter = packagePath + "/config/control_config.yaml"

elif device_type == "tower":
    print("Loading sensor tower parameters!")

    packagePath = get_package_share_directory('antobot_description')

    device_config_path = packagePath + "/config/platform_config.yaml"
    with open(device_config_path, 'r') as yamlfile:
        data = yaml.safe_load(yamlfile)

        urcu_hardware = data['urcu_hardware']     
        sensor_hardware = data['sensor_hardware']
        lidar_hardware = data['sensor_hardware']
        aRCU_cfg = data.get('aRCU', {})

        if aRCU_cfg:
            aRCU_ssh_list = [
                aRCU_cfg.get('user', ''),
                aRCU_cfg.get('host', ''),
                aRCU_cfg.get('ws', '')
            ]
        else:
            aRCU_ssh_list = []

elif device_type == "scout":
    print("Loading scout parameters!")

    packagePath = get_package_share_directory('antobot_description')

    device_config_path = packagePath + "/config/platform_config.yaml"
    with open(device_config_path, 'r') as yamlfile:
        data = yaml.safe_load(yamlfile)

        urcu_hardware = data['urcu_hardware']     
        sensor_hardware = data['sensor_hardware']
        aRCU_cfg = data.get('aRCU', {})

        if aRCU_cfg:
            aRCU_ssh_list = [
                aRCU_cfg.get('user', ''),
                aRCU_cfg.get('host', ''),
                aRCU_cfg.get('ws', '')
            ]
        else:
            aRCU_ssh_list = []


# Initialisation of node lists
nodeDict=dict()
groupDict=dict() # Nodes can be grouped into function groups to allow them to be addressed using a single service call
launchDict=dict()


if auto_launch:
    nodeDict['softwareManager'] = AntobotSWNode("swMgr_node", "antobot_bringup", "softwareManager", "SW000", "/", [], "system")
    if not robot_hardware: # gazebo topic remap node - needed for simulation EKF
        nodeDict['remapSimNode']=AntobotSWNode("remap_gazebo_wheel_odom", 'antobot_ekf', 'remap_gazebo_wheel_odom', "SW100", "/", [], "system")

    if device_type == "robot" and robot_hardware:
        nodeDict['controlNode'] = AntobotSWNode("ant_control_node", "antobot_control", "ant_control_node", "SW100", "/", [], "system", param_files=control_parameter, param_dict = {'use_sim_time': not robot_hardware}, prefix=prefix_isolated_cpu)
 
    if urcu_hardware:
        nodeDict['urcuMonitor']=AntobotSWNode("urcuMonitor","antobot_urcu","urcuMonitor","SW212","/",[],"system")
        nodeDict['shutdownSrv']=AntobotSWNode("shutdownSrv","antobot_urcu","softshutdown","SW214","/",[],"system")
 
    if lidar_hardware:
        nodeDict['lidarManager']=AntobotSWNode("lidarManager","antobot_devices_lidar","lidar_manager.py","SW232","/",[],"sensor", ssh=aRCU_ssh_list)

    # Adding costmap launch script - need to update lidarManager (in ROS1, costmap was launched inside the lidarManager)
    launchDict['costmapNode']=Launchfile("costmapNode", 'antobot_nav2_costmap', 'costmap_launch.py', ssh=aRCU_ssh_list) 

    if sensor_hardware:
        launchDict['imuManager']=Launchfile("imuManager", 'antobot_devices_imu', 'imu.launch.py') 
        launchDict['gpsManager']=Launchfile("gpsManager","antobot_devices_gps","gps_f9p.launch.py")
        
        if dual_gps:
            nodeDict['gpsMovingbase']=AntobotSWNode("gpsMovingbase", 'antobot_devices_gps', 'gps_movingbase', "","sensor",[],"auto", param_dict = {'use_sim_time': not robot_hardware, 'port_movingrover': port_movingrover, 'antenna_baseline': antenna_baseline})
        else:
            nodeDict['gpsCorrections']=AntobotSWNode("gpsCorrections", 'antobot_devices_gps', 'gps_corrections', "","sensor",[],"auto")

    #if device_type == "robot" or device_type == "tower":  ##comment out for sensor test Aug15
    nodeDict['amHeading']=AntobotSWNode("amHeading","antobot_heading","heading_node","SW106","/",[],"sensor", param_dict = {'use_sim_time': not robot_hardware})   ##comment out for sensor test Aug15

    if device_type == "robot":
        launchDict['rosbridge_server']=Launchfile("rosbridgeServer","rosbridge_server","rosbridge_websocket_launch.xml")

        if robot_hardware: # Software which relies on having robot hardware
            launchDict['statePublisherLaunch']=Launchfile("statePublisherLaunch", 'antobot_description', 'ant_v4_state_publisher.launch.py') # TODO: use robot_version (currently only ant v4)
            if robot_platform == "ant":
                nodeDict['antoBridge']=AntobotSWNode("antoBridge","anto_bridge","anto_bridge_node","SW000","/",[],"system", prefix=prefix_isolated_cpu)
        else:
            launchDict['simLaunch']=Launchfile("simLaunch", 'antobot_sim_bringup', 'antobot_sim.launch.py')
 
        # Robot Control
        nodeDict['amSafety']=AntobotSWNode("antSafety","antobot_safety","safety_node","SW101","/",[],"system")
        launchDict['cmdVelMux']=Launchfile("cmdVelMux", 'cmd_vel_mux', 'cmd_vel_mux-launch.py')
        launchDict['sdkTeleop']=Launchfile("sdkTeleop", 'antobot_teleop', 'teleop_launch.py')
