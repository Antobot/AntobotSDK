import os
import xacro

import yaml

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node


########################################################################

def generate_launch_description():
    
    # Configure ROS nodes for launch
    ld = LaunchDescription()

    pkg_antobot_description = get_package_share_directory('antobot_description')
    platform_config_path = str(pkg_antobot_description) + "/config/platform_config.yaml"

    with open(platform_config_path, 'r') as yamlfile:
        data = yaml.safe_load(yamlfile)

        robot_platform = data['robot_platform']
        if robot_platform == "ant":
            model_xacro = 'ant_v4.urdf.xacro'
        elif robot_platform == "allWheel":
            model_xacro = 'allWheel.urdf.xacro'

    # Locate your Xacro file
    xacro_file = os.path.join(pkg_antobot_description,'urdf', model_xacro)

    # Process Xacro to URDF
    doc = xacro.process_file(xacro_file)
    urdf_content = doc.toxml()

    # Takes the description and joint angles as inputs and publishes the 3D poses of the robot links
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='both',
        parameters=[
            {'use_sim_time': False},
            {'robot_description': urdf_content},
        ]
    )

    ld.add_action(robot_state_publisher)





    return ld