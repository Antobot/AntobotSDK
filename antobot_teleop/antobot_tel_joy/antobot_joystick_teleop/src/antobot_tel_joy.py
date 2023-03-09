#!/usr/bin/env python3

# Copyright (c) 2023, ANTOBOT LTD.
# All rights reserved.

# # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # 

# # # Functional Description:     Teleoperation node with joystick
# # #                       The node handles the Joy messages coming from the joy ros driver

# # # Interface: 
# # # Inputs: Joystick buttons [Joy message] - received from /joy topic from /antobot_teleop/antobot_tel_joy/joystick_drivers/joy/src/joy_node.cpp
# # #                                        - Contains one vector for axes, one for buttons (each element is a specific button or axis)
# # #                                        - Buttons: each element is 1 if button is pressed and 0 if not (digital)
# # #                                        - Axes: each element is a value between -1 and 1 (analogue)
# # #         Robot operation mode [Int8 message] - received from /switch_mode ROS topic
# # #                                             - 0: keyboard teleoperation, 2: joystick teleoperation, 3: autonomous mode
# # # Outputs: Joystick velocity commands [Twist message] - sent over /joy/cmd_vel
# # #                                                     - Contains individual components of linear and angular velocity commands to be given to the robot
# # #          Robot operation mode [Int8 message] - sent over /switch_mode ROS topic
# # #                                              - 0: keyboard teleoperation, 2: joystick teleoperation, 3: autonomous mode
# # #          Row following initialisation     - sent over /row_following ROS topic, starts/stops forward or backward row-following
# # #          Force Stop Release             - sent over antobridge/force_stop_release ROS topic
# # #                                         - if the robot has been forced to stop by Aurix or antobot_safety (USS sensor detection, pressed bumper switch),
# # #                                           this will release the force stop and allow for robot movement/teleoperation
# # #          Soft Shutdown                  - sent over /antobridge/soft_shutdown_button
# # #                                         - tells the robot to shut down in a safe way (handled by anto_supervisor)

# Contact: daniel.freer@antobot.ai

# # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # 

import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry, Path
from sensor_msgs.msg import NavSatFix
from sensor_msgs.msg import Joy
from std_msgs.msg import Int8, UInt8, Empty, Bool
import actionlib
import time
from actionlib_msgs.msg import *
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

class JoystickTeleop:
    def __init__(self):
        # # # Initialisation of JoystickTeleop class
        
        self.mode = None  # initialise robot operation mode, modes used are 0: joystick teleop; 1: forward row-following; 2: backward row-following;
        ###
        self.debug = 0  # print out debug info if self.debug = 1


        self.cmd_vel_pub = rospy.Publisher('/joy/cmd_vel', Twist, queue_size=5)  # publisher for cmd_vel topic
        
        self.rctrl_loop_hz = 10.0  # timer frequency for update function
        rospy.Timer(rospy.Duration(1 / self.rctrl_loop_hz), self.update)  # timer for sending cmd_vel messages
        
        self.joy_sub = rospy.Subscriber("/joy", Joy, self.joy_callback)  # subscriber for Joy message
        
        self.mode_sub = rospy.Subscriber("/switch_mode", UInt8, self.mode_callback) # subscriber for robot operation mode
        
        self.switch_mode_pub = rospy.Publisher('/switch_mode', UInt8, queue_size=5, latch=True)  # publisher for switching operation modes
        
        self.row_follow_pub = rospy.Publisher('/row_following', Empty, queue_size=1) # publisher to start and stop row following
        
        self.force_stop_release_pub = rospy.Publisher('/antobridge/force_stop_release', Bool, queue_size=1)  # publisher to release the force stop on Aurix
        
        self.soft_shutdown_pub = rospy.Publisher('/antobridge/soft_shutdown_button',Bool,queue_size=1) # publisher to send the soft shutdown signal to antobridge
        ###
        ## joystick axis and button configuration
        # axis and button values are their position within the vectors in /joy message (e.g. 'A' button is 1st element in buttons array, 'B' button is 2nd element)
        
        # axis numbers
        self.axis_analog_left = 1  # left joystick up/down axis number (linear velocity)
        self.axis_analog_right = 3  # right joystick left/right axis number (angular velocity)
        
        self.axis_dpad_back_forward = 7  # dpad up/down axis number
        self.axis_dpad_left_right = 6  # dpad left/right axis number
        
        self.axis_lt = 2 # left trigger axis number
        self.axis_rt = 5 # right trigger axis number (modifier)
        
        # button numbers
        self.button_a = 0
        self.button_b = 1
        self.button_x = 2  # brake
        self.button_y = 3
        self.button_lb = 4  # init manual control with rb
        self.button_rb = 5  # init manual control with lb
        self.button_back = 6 # back button
        self.button_start = 7 # start button
        self.button_logitech = 8 # logitech button
        self.button_ls = 9  # left joystick button
        self.button_rs = 10  # right joystick button
        
        # button description strings
        self.button_names = [""] * 12
        self.button_names[self.button_a] = "Button A"
        self.button_names[self.button_b] = "Button B"
        self.button_names[self.button_x] = "Button X"
        self.button_names[self.button_y] = "Button Y"
        self.button_names[self.button_lb] = "Button LB"
        self.button_names[self.button_rb] = "Button RB"
        self.button_names[self.button_back] = "Button BACK"
        self.button_names[self.button_start] = "Button START"
        self.button_names[self.button_logitech] = "Button LOGITECH"        
        self.button_names[self.button_ls] = "Button LS"
        self.button_names[self.button_rs] = "Button RS"
        
        # initialise axes and buttons arrays
        self.axes = [0] * 8   # 8 elements in axis array
        self.buttons = [0] * 11   # 11 elements in buttons array
        self.buttons_previous = self.buttons  # previous buttons pressed
        
        rospy.loginfo("Joystick Teleop Started")
        
        # initialise linear and angular velocities
        self.v = 0
        self.w = 0
        
    def mode_callback(self, mode):
        # # # Callback function for the robot operation mode
        # Inputs: mode [Int8 message] - Used for communicating robot operation mode. 0 - keyboard teleop; 3 - autonomous mode
            
        # # # If the robot operation mode is changed from a different script, the self.mode paramter will be set to None, 
        # # # so that the previous mode can be entered again from this script
        
        # keyboard teleoperation  
        if mode.data == 0:   
            self.mode = None
            
        # autonomous mode
        if mode.data == 3:
            self.mode = None
        
    def joy_callback(self, msg):
        # # # Callback function for the joystick
        # Inputs: msg [Joy message] - Used for reading the Joystick axes and buttons
        
        if (self.debug == 1):
            rospy.loginfo("joy")
            rospy.loginfo(" axis linear: %s", msg.axes[self.axis_analog_left])
            rospy.loginfo(" axis angular: %s", msg.axes[self.axis_analog_right])
        
            rospy.loginfo(" axis dpad back/forward: %s", msg.axes[self.axis_dpad_back_forward])
            rospy.loginfo(" axis dpad left/right: %s", msg.axes[self.axis_dpad_left_right])
            
            rospy.loginfo(" LT axis: %s", msg.axes[self.axis_lt])
            rospy.loginfo(" RT axis: %s", msg.axes[self.axis_rt])
        
            # check the button statuses
            for i in range(0, 12):
                if (self.buttons[i] == 1 and self.buttons_previous[i] == 0):
                    rospy.loginfo(" %s pressed", self.button_names[i])
        
        # copy the buttons and axes vars to class variables
        self.buttons = msg.buttons
        self.axes = msg.axes
        
        # reset user input for manager client
        userInput = None
        
        ###
        # main state machine for reading the joystick buttons
        self.force_stop_release = False
        self.soft_shutdown = False
        
        if (self.buttons[self.button_lb] == 1 and self.buttons[self.button_rb] == 1):
            rospy.loginfo("Manual mode")
            self.switch_mode_pub.publish(2)  # sent to master_teleop to send velocity commands from joystick
            self.force_stop_release = True
            self.mode = 0
            
        elif (self.axes[self.axis_rt] == 0 or self.axes[self.axis_rt] == 1):
            
            if (self.buttons[self.button_a] == 1 and self.buttons_previous[self.button_a] == 0):
                userInput = 1 # resume job
            
            elif (self.buttons[self.button_y] == 1 and self.buttons_previous[self.button_y] == 0):
                userInput = 2 # pause job   
            
            elif (self.buttons[self.button_b] == 1 and self.buttons_previous[self.button_b] == 0):
                userInput = 3 # abort job
            
        elif (self.axes[self.axis_rt] == -1):
            # if rt is pressed
            if(self.axes[self.axis_dpad_back_forward] == 1):  # dpad forward
                rospy.loginfo("Drive forward autonomously through rows")
                self.mode = 1
            
                # publish the mode
                self.switch_mode_pub.publish(10)
            elif (self.mode != 2 and self.axes[self.axis_dpad_back_forward] == -1):  # dpad backward
                rospy.loginfo("Drive backward autonomously through rows")
                self.mode = 2
                
                # publish the mode
                self.switch_mode_pub.publish(11)

        if (self.buttons[self.button_back] == 1 and self.buttons[self.button_start] == 1):
            rospy.loginfo("Xavier will be powered off now")
            self.soft_shutdown = True
          
        self.force_stop_release_pub.publish(self.force_stop_release)
        self.soft_shutdown_pub.publish(self.soft_shutdown)
        
        ##
        # if in manual mode check the brake button and get the analog readings
        if (self.mode == 0):
            # if X button is pressed (brake)
            if (self.buttons[self.button_x] == 1 and self.buttons_previous[self.button_x] == 0):
                # if X is pressed, stop
                self.v = 0
                self.w = 0
                
                # prepare and send twist message
                cmd_vel_msg = Twist()
                cmd_vel_msg.linear.x = self.v
                cmd_vel_msg.linear.y = 0
                cmd_vel_msg.linear.z = 0
                cmd_vel_msg.angular.x = 0
                cmd_vel_msg.angular.y = 0
                cmd_vel_msg.angular.z = self.w
            
                self.cmd_vel_pub.publish(cmd_vel_msg)
                
                rospy.loginfo("Brake")
            else:                
                self.v = self.axes[self.axis_analog_left] * 2.0
                self.w = self.axes[self.axis_analog_right] * 1.0
        
        self.buttons_previous = msg.buttons
        
    def update(self, timer):
        # # # Update function - called iteratively at a rate defined by self.rctrl_loop_hz. The main purpose is to check
        # # # for keyboard or app input, which could change the mode or control the robot
        # Inputs: timer [ROS Timer] - unused, but required for callback function definition

        # if in manual mode, send the linear and angular velocities
        # prepare and send twist message to /joy/cmd_vel topic
        if self.mode == 0:
            cmd_vel_msg = Twist()
            cmd_vel_msg.linear.x = self.v
            cmd_vel_msg.linear.y = 0
            cmd_vel_msg.linear.z = 0
            cmd_vel_msg.angular.x = 0
            cmd_vel_msg.angular.y = 0
            cmd_vel_msg.angular.z = self.w
            
            self.cmd_vel_pub.publish(cmd_vel_msg)

# Begins main function
if __name__ == '__main__':
    rospy.init_node("antobot_tel_joy")
    jt = JoystickTeleop()
    
    rospy.spin()
