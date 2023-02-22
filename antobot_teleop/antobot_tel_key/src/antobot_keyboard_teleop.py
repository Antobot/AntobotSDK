#!/usr/bin/env python3
# -*- coding: utf-8 -*-

# Copyright (c) 2023, ANTOBOT LTD.
# All rights reserved.

# # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # 

# # # Code Description:     Allows for keyboard teleoperation of the robot, using the right side of the keyboard for control commands: [[u, i, o],
# # #                       and the left side to increase/decrease speed: [q/z] (both), [w/x] (linear), e/c (angular)                   [j, k, l],
# # #                       If main command, sends Twist commands to ROS publisher. Called by antobot_teleop.py                              [m, ,, .]]

# Contact:      daniel.freer@antobot.ai

# # # #  # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # 

import rospy
from geometry_msgs.msg import Twist
import sys, select, termios, tty

# Defines move bindings (keys associated with robot control)
moveBindings = {
        'i':(1,0),
        'o':(1,-1),
        'j':(0,1),
        'l':(0,-1),
        'u':(1,1),
        ',':(-1,0),
        '.':(-1,1),
        'm':(-1,-1),
           }

# Defines speed bindings (keys associated with increasing or decreasing max linear/angular velocity)
speedBindings={
        'q':(1.1,1.1),
        'z':(.9,.9),
        'w':(1.1,1),
        'x':(.9,1),
        'e':(1,1.1),
        'c':(1,.9),
          }


class Keyboard_Teleop:

    def __init__(self):
        # # # Initialisation function for Keyboard_Teleop class
        
        # Sets keyboard input settings
        self.settings = termios.tcgetattr(sys.stdin)
        
        # Defines initial values for class variables
        self.x = 0
        self.th = 0
        self.status = 0
        self.count= 0
        self.acc = 0.1
        self.target_speed = 0
        self.target_turn = 0
        self.control_speed = 0
        self.control_turn = 0

        # Defines initial values for linear and angular velocity
        self.speed = 0.7
        self.turn = 1


    def print_msg(self):
        # # # Prints the message explaining the robot controls to terminal
        
        self.msg = """
        Control mrobot!
        ---------------------------
        Moving around:
           u    i    o
           j    k    l
           m    ,    .

        q/z : increase/decrease max speeds by 10%
        w/x : increase/decrease only linear speed by 10%
        e/c : increase/decrease only angular speed by 10%
        space key, k : force stop
        anything else : stop smoothly

        CTRL-C to quit
        """

        print(self.msg)


    def getKey(self):
        # # # Gets the key which was input via the keyboard and returns it as a character
        # Returns: key - For example: 'c', 'k', or '\x03' (ctrl+c)

        tty.setraw(sys.stdin.fileno())
        rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
        if rlist:
            key = sys.stdin.read(1)
        else:
            key = ''

        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        return key


    def vels(self, speed, turn):
        # # # Returns a printout of the linear and angular velocities as a string
        # Returns: vels
        return "currently:\tspeed %s\tturn %s " % (speed,turn)


    def get_twist(self, key):
        # # # Gets the correct twist message for robot control from an input key and class variables
        # Inputs: key (char) - the key which was pressed by the user
        
        # Motion control direction key (1: positive direction, -1 negative direction)
        if key in moveBindings.keys():
            self.x = moveBindings[key][0]
            self.th = moveBindings[key][1]
            self.count = 0

        # Speed modifier key
        elif key in speedBindings.keys():
            self.speed = self.speed * speedBindings[key][0]  # Linear speed increased by 10%
            self.turn = self.turn * speedBindings[key][1]  # Angular velocity increased by 10%
            self.count = 0

            rospy.loginfo(self.vels(self.speed, self.turn))     # prints current speeds if they are changed
            if (self.status == 14):
                rospy.loginfo(self.msg)             # Re-prints teloperation instructions if 15 speed modifications have occurred
            self.status = (self.status + 1) % 15

        # Stop button
        elif key == ' ' or key == 'k':
            self.x = 0
            self.th = 0
            self.control_speed = 0
            self.control_turn = 0
        else:
            self.count = self.count + 1
            if self.count > 4:
                self.x = 0
                self.th = 0
            if (key == '\x03'):         # ctrl+c
                return None

        # Target speed = speed value * direction value
        self.target_speed = self.speed * self.x
        self.target_turn = self.turn * self.th

        # Speed limit to prevent the speed from increasing or decreasing too fast
        if self.target_speed > self.control_speed:
            self.control_speed = min(self.target_speed, self.control_speed + 0.02)
        elif self.target_speed < self.control_speed:
            self.control_speed = max(self.target_speed, self.control_speed - 0.02)
        else:
            self.control_speed = self.target_speed

        if self.target_turn > self.control_turn:
            self.control_turn = min(self.target_turn, self.control_turn + 0.1)
        elif self.target_turn < self.control_turn:
            self.control_turn = max(self.target_turn, self.control_turn - 0.1)
        else:
            self.control_turn = self.target_turn

        # Create and publish twist message

        twist = Twist()
        twist.linear.x = self.control_speed
        twist.linear.y = 0
        twist.linear.z = 0
        twist.angular.x = 0
        twist.angular.y = 0
        twist.angular.z = self.control_turn

        return twist

