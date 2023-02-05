#!/usr/bin/env python3

# # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # 

# # # Functional Description:     Creates a master program to allow for switching between our main control methods:
# # #                       1) Keyboard control (inherited from keyboard_teleop.Keyboard_Teleop) and waypoint saving (inherited from save_waypoints.waypoint_saver);
# # #                       2) Joystick control (receives messages from antobot_tel_joy)

# # # Interface:
# # # Inputs: Robot operation mode [Int8 msg] - received from /switch_mode topic from antobot_teleop/antobot_tel_joy/bin/antobot_tel_joy.py and AntoCom/ac_mqtt/src/mqtt_interface_node.py
# # #                                         - 0: keyboard teleoperation, 1: app teleoperation, 2: joystick teleoperation, 3: follow waypoints (autonomous), 4: go home (autonomous)
# # #         Velocity commands from joystick [Twist msg] - received from /joy/cmd_vel topic from antobot_teleop/antobot_tel_joy/bin/antobot_tel_joy.py
# # #         Waypoint filename [String msg] - received from /wp_fname topic from antobot_teleop/keyboard_teleop/src/save_waypoints.py
# # # Outputs: Robot operation mode [Int8 msg] - sent over /switch_mode topic to antobot_navigation/src/waypoint.py
# # #                                          - 0: keyboard teleoperation, 1: app teleoperation, 2: joystick teleoperation, 3: follow waypoints (autonomous), 4: go home (autonomous)
# # #          Robot velocity commands [Twist msg] - sent over /am_robot/cmd_vel topic 
# # #                                              - final velocity commands sent to am_bridge
# # #          Waypoint recording status [Int8 msg] - sent over /wp_status topic to antobot_teleop/keyboard_teleop/src/save_waypoints.py
# # #                                               - 0: start recording, 1: stop recording, 2: change waypoint file to be followed to the most recently saved

# Owner: daniel.freer@antobot.ai

# # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # 

import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry, Path
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import UInt8, String
import sys, os

from pathlib import Path

# Including path for keyboard teleoperation
tel_key_path = Path(__file__).resolve().parent.parent.parent / 'antobot_tel_key' / 'src'
sys.path.append(str(tel_key_path))

from antobot_keyboard_teleop import Keyboard_Teleop
import roslaunch
import rospkg


class MasterTeleop:
    def __init__(self):
        # # # Initialisation of MasterTeleop class

        # Sets default operation mode to 0 (keyboard teleop)
        self.mode = None
        self.previous_mode = None

        # Publisher for physical robot movement
        self.rctrl_pub = rospy.Publisher('/teleop/cmd_vel', Twist, queue_size=5)
         
        # Publisher to switch operation mode
        self.mode_pub = rospy.Publisher("/switch_mode", UInt8, queue_size=5, latch=True)
        
        # Publisher to start and stop saving waypoints
        self.wp_status_pub = rospy.Publisher('/wp_status', UInt8, queue_size=1)

        # Define subscribers to mode switch, mqtt (app) velocity, joystick velocity, imu calibration and waypoint filename topics
        self.mode_sub = rospy.Subscriber("/switch_mode", UInt8, self.mode_callback)
        self.joy_cmd_sub = rospy.Subscriber("/joy/cmd_vel", Twist, self.joy_cmd_callback)
        self.fname_sub = rospy.Subscriber('/wp_fname', String, self.fname_callback)

        # Defining a timer for the update loop
        self.rctrl_loop_hz = 30.0
        rospy.Timer(rospy.Duration(1 / self.rctrl_loop_hz), self.update)

        self.mqtt_twist = None
        self.uuid = None
        self.joy_twist = None
        
        # Initialises different class instances which may be used in the code (but may not be)
        self.kt = Keyboard_Teleop()
        
        # Selects most recently saved waypoint file
        self.path = "/tmp/waypoint_files/"
        if os.path.exists(self.path):
            files = os.listdir(self.path) # list files in waypoint files directory
            files.sort() # sort into alphabetical order so most recent is last in list
            self.fname = files[-1] # select last in list
    
    def mode_callback(self, mode_in):
        # # # Callback function for the "/switch_mode" topic (Int8). The mode received by this function will define
        # # # which teleoperation state the robot is in.
        # Inputs: mode_in [Int8.msg] - 0: keyboard teleoperation; 1: app control; 2: joystick control; 3: autonomous navigation
            
        self.mode = mode_in.data
        if self.mode == 0:
            self.kt.print_msg()
        if self.mode == 2:
            print("Robot being operated by joystick")
            
    def fname_callback(self, fname):
        # # # Callback function which will change the waypoint file to be followed to the one received by the subscriber
        self.fname = fname.data

    def joy_cmd_callback(self, data):
        # # # Callback function for the "/joy/cmd_vel" ROS topic. Passes the information along to self.joy_twist,
        # # # which, if the robot is in joystick teleoperation mode, will be sent to am_bridge.
        # Input: data [Twist.msg] - ROS message for movement (linear velocity in x direction, angular velocity in z) 
        self.joy_twist = data

    def update(self, timer):
        # # # Update function - called iteratively at a rate defined by self.rctrl_loop_hz. The main purpose is to check
        # # # for keyboard or app input, which could change the mode or control the robot
        # Inputs: timer [ROS Timer] - unused, but required for callback function definition

        key = self.kt.getKey()
        if key == '0':
            if self.mode != 0:
                self.mode_pub.publish(0)
        if key == '3':
            if self.mode != 3:
                self.mode_pub.publish(3)
        if key == '\x03':
            os.system("rosnode kill master_teleop")

        if self.mode == 0:          # Keyboard Teleop mode
            twist_msg = self.kt.get_twist(key)
            self.rctrl_pub.publish(twist_msg)
        elif self.mode == 2:	 # Joystick Teleoperation 
            if self.joy_twist is not None:
                twist_msg = self.joy_twist
                self.rctrl_pub.publish(twist_msg)
        
# Begins main function
if __name__ == '__main__':
    rospy.init_node("master_teleop")
    mt = MasterTeleop()
    rospy.spin()
