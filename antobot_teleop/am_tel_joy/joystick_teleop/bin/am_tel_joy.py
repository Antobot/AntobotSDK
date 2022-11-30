#!/usr/bin/env python3

# Copyright (c) 2021, ANTOBOT LTD.
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

# # # Functional Description:     Teleoperation node with joystick
# # #                       The node handles the Joy messages coming from the joy ros driver

# # # Interface: 
# # # Inputs: Joystick buttons [Joy message] - received from /joy topic from /am_teleop/joystick_drivers/joy/src/joy_node.cpp
# # #                                        - Contains one vector for axes, one for buttons (each element is a specific button or axis)
# # #                                        - Buttons: each element is 1 if button is pressed and 0 if not (digital)
# # #                                        - Axes: each element is a value between -1 and 1 (analogue)
# # #         Robot operation mode [Int8 message] - received from /switch_mode
# # #                                             - 0: keyboard teleoperation, 1: app teleoperation, 2: joystick teleoperation, 3: follow waypoints (autonomous), 4: go home (autonomous)
# # # Outputs: Joystick velocity commands [Twist message] - sent over /joy/cmd_vel
# # #                                                     - Contains individual components of linear and angular velocity commands to be given to the robot
# # #          Robot operation mode [Int8 message] - sent over /switch_mode
# # #                                              - 0: keyboard teleoperation, 1: app teleoperation, 2: joystick teleoperation, 3: follow waypoints (autonomous), 4: go home (autonomous)
# # #          Waypoint recording status [Int8 message] - sent over /wp_status
# # #                                                   - 0: start recording, 1: stop recording, 2: change waypoint file to be followed to the most recently saved

# Created by: mert.turanli@antobot.co.uk
# Modified by: dylan.smith@antobot.ai
# Reviewed by: howard.wu@antobot.ai

# # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # 

import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry, Path
from sensor_msgs.msg import NavSatFix
from sensor_msgs.msg import Joy
from std_msgs.msg import Int8, UInt8, Empty, Bool
import actionlib
#from evdev import ecodes, InputDevice, ff, list_devices
import time
from actionlib_msgs.msg import *
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from anto_manager.managerUserInputClient import directUserInputClient

class JoystickTeleop:
    def __init__(self):
        # # # Initialisation of JoystickTeleop class
        
        # gets the home position from rosparam server
        # robot will navigate autonomously to this position when given the command
        if (rospy.has_param('~home_position')):
            self.home_position = rospy.get_param('~home_position')
        else:
            self.home_position = [0, 0, 0]
            
            
        rospy.loginfo("Home position = %s", self.home_position)
        
        self.mode = None  # initialise robot operation mode, modes used are 0-4
        ###
        self.debug = 0  # print out debug info if self.debug = 1


        self.cmd_vel_pub = rospy.Publisher('/joy/cmd_vel', Twist, queue_size=5)  # publisher for cmd_vel topic
        
        self.rctrl_loop_hz = 10.0  # timer frequency for update function
        rospy.Timer(rospy.Duration(1 / self.rctrl_loop_hz), self.update)  # timer for sending cmd_vel messages
        
        self.joy_sub = rospy.Subscriber("/joy", Joy, self.joy_callback)  # subscriber for Joy message
        
        self.mode_sub = rospy.Subscriber("/switch_mode", UInt8, self.mode_callback) # subscriber for robot operation mode
        
        self.switch_mode_pub = rospy.Publisher('/switch_mode', UInt8, queue_size=5, latch=True)  # publisher for switching operation modes
        
        self.wp_status_pub = rospy.Publisher('/wp_status', UInt8, queue_size=5)  # publisher for waypoint status
        
        self.row_follow_pub = rospy.Publisher('/row_following', Empty, queue_size=1) # publisher to start and stop row following
        
        self.force_stop_release_pub = rospy.Publisher('/antobridge/force_stop_release', Bool, queue_size=1)  # publisher to release the force stop on Aurix
        
        ###
        ## joystick axis and button configuration
        # axis and button values are their position within the vectors in /joy message (e.g. 'A' button is 1st element in buttons array, 'B' button is 2nd element)
        
        # axis numbers
        self.axis_analog_left = 1  # left joystick up/down axis number (linear velocity)
        self.axis_analog_right = 3  # right joystick left/right axis number (angular velocity)
        
        self.axis_dpad_back_forward = 7  # dpad up/down axis number
        self.axis_dpad_left_right = 6  # dpad left/right axis number
        
        self.axis_lt = 2 # left trigger axis number (go home)
        self.axis_rt = 5 # right trigger axis number (modifier)
        
        # button numbers
        self.button_a = 0  # start saving waypoints
        self.button_b = 1  # stop saving waypoints
        self.button_x = 2  # brake
        self.button_y = 3  # store favourite waypoints
        self.button_lb = 4  # init manual control with rb
        self.button_rb = 5  # init manual control with lb
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
        
        # initialise axes and buttons arrays
        self.axes = [0] * 8   # 8 elements in axis array
        self.buttons = [0] * 11   # 11 elements in buttons array
        self.buttons_previous = self.buttons  # previous buttons pressed
        
        rospy.loginfo("Joystick Teleop Started")
        
        # initialise linear and angular velocities
        self.v = 0
        self.w = 0
        
        # initialise waypoint recording status
        self.wp_status = None
        
        # initialise manager client
        self.userClient = directUserInputClient(userInput=0, sourceID='Joystick')

        # initialise the move base client
        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self.client.wait_for_server() # wait for move base to come up
        
        ##
    def cancel_nav_goal(self):
        # # # Function to cancel active naviagtion goals
        self.client.cancel_all_goals()   # Cancel any active navigation goals
        while not self.client.get_state() == GoalStatus.PREEMPTED:   # wait for goal to be cancelled
            print("Waiting for goal to be cancelled")
        print("Goal cancelled")
        
    def mode_callback(self, mode):
        # # # Callback function for the robot operation mode
        # Inputs: mode [Int8 message] - Used for communicating robot operation mode
        if (mode.data == 0 or mode.data == 1 or mode.data == 2) and self.client.get_state() == GoalStatus.ACTIVE:
            self.cancel_nav_goal()
            
        # # # If the robot operation mode is changed from a different script, the self.mode paramter will be set to None, 
        # # # so that the previous mode can be entered again from this script
        
        # keyboard or app teleoperation  
        if mode.data == 0 or mode.data == 1:   
            self.mode = None
            
        # if waypoint following mode not entered from this script
        if mode.data == 3 and self.mode != 4:
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
                userInput = 3 # arbort job
            
        elif (self.axes[self.axis_rt] == -1):
            # if rt is pressed
            if(self.axes[self.axis_dpad_back_forward] == 1):  # dpad forward
                rospy.loginfo("Drive forward autonomously through rows")
                self.mode = 1
            
                # publish the mode
                self.switch_mode_pub.publish(10)
                #empty_msg = Empty()
                #self.row_follow_pub.publish(empty_msg)
            elif (self.mode != 2 and self.axes[self.axis_dpad_back_forward] == -1):  # dpad backward
                rospy.loginfo("Drive backward autonomously through rows")
                self.mode = 2
                
                # publish the mode
                self.switch_mode_pub.publish(11)
            elif (self.mode != 3 and self.axes[self.axis_dpad_left_right] == -1):  # dpad right
                rospy.loginfo("Follow me")
                self.mode = 3
                
                # publish the mode
                #self.switch_mode_pub.publish(3)    
            elif (self.mode != 4 and self.axes[self.axis_dpad_left_right] == 1):  # dpad left
                rospy.loginfo("Follow the most recently stored set of waypoints")
                self.mode = 4
                
                # publish the mode
                self.switch_mode_pub.publish(3)
            elif (self.mode != 5 and self.axes[self.axis_lt] == -1):
                rospy.loginfo("Go home")
                self.mode = 5
                
                # publish the mode
                self.switch_mode_pub.publish(4)

                # send the goal (home) position
                # home position is configured through the rosparam server
                # sends the goal to the action server
                goal = MoveBaseGoal()
                goal.target_pose.header.frame_id = 'map'
                goal.target_pose.header.stamp = rospy.get_rostime()
                goal.target_pose.pose.position.x = self.home_position[0]
                goal.target_pose.pose.position.y = self.home_position[1]
                goal.target_pose.pose.position.z = self.home_position[2]
                goal.target_pose.pose.orientation.w = 1.0
                self.client.send_goal(goal) 
            
            # Waypoint saving whilst in manual mode    
            elif (self.mode == 0 and self.buttons[self.button_a] == 1 and self.buttons_previous[self.button_a] == 0 and self.wp_status != 0):
                # A pressed while in manual mode (start recording waypoints)
                rospy.loginfo("Start recording waypoints")
                self.wp_status = 0
                self.wp_status_pub.publish(self.wp_status)
            elif (self.mode == 0 and self.buttons[self.button_b] == 1 and self.buttons_previous[self.button_b] == 0 and self.wp_status == 0):
                # B pressed while in manual mode (stop recording waypoints)
                rospy.loginfo("Stop recording waypoints")
                self.wp_status = 1
                self.wp_status_pub.publish(self.wp_status)
            elif (self.mode == 0 and self.buttons[self.button_y] == 1 and self.buttons_previous[self.button_y] == 0): # and (self.wp_status == 0 or self.wp_status == 1)):
                # Y pressed while in manual mode (stores the waypoints)
                rospy.loginfo("Store waypoints")
                self.wp_status = 2
                self.wp_status_pub.publish(self.wp_status)
                
        self.force_stop_release_pub.publish(self.force_stop_release)
        
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
                # if brake is not pressed
                # read analog values and scale the velocities
                self.v = self.axes[self.axis_analog_left] * 2.0
                self.w = self.axes[self.axis_analog_right] * 1.0
          
        # rumble once if left stick pressed  
        if (self.buttons[self.button_ls] == 1 and self.buttons_previous[self.button_ls] == 0):
            self.rumble(repeat_count=1)
            
        # rumble twice if right stick pressed
        if (self.buttons[self.button_rs] == 1 and self.buttons_previous[self.button_rs] == 0):
            self.rumble(repeat_count=2)
        
        self.buttons_previous = msg.buttons
        
        # Send request to manager server if input is made by user
        if userInput is not None:
            if self.userClient.checkForService():
                self.userClient.userInput = userInput
                managerResponse = self.userClient.sendDirectUserInput()
                print("User input was " + str(userInput))
                print("Server response was " + str(managerResponse.responseBool))
                print(managerResponse.responseString)
                
                # rumble once if user input is accepted, rumble twice if not accepted
                #if managerResponse.responseBool:
                #    self.switch_mode_pub.publish(6)
                #    self.rumble(repeat_count=1)
                #else:
                #    self.rumble(repeat_count=2)
                    
            else:
                print("Unable to make request")
                print("ROS service " + self.userClient.serviceName + " is not available")
        
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
            
    def rumble(self, repeat_count):
        
        # Find first EV_FF capable event device (that we have permissions to use).
        for name in list_devices():
            dev = InputDevice(name)
            if ecodes.EV_FF in dev.capabilities():
                print(name)
                break
        
        rumble = ff.Rumble(strong_magnitude=0xffff, weak_magnitude=0xffff)
        effect_type = ff.EffectType(ff_rumble_effect=rumble)
        duration_ms = 200
        
        effect = ff.Effect(
            ecodes.FF_RUMBLE, -1, 0,
            ff.Trigger(0, 0),
            ff.Replay(duration_ms, duration_ms),
            effect_type
        )

        effect_id = dev.upload_effect(effect)
        dev.write(ecodes.EV_FF, effect_id, repeat_count)
        time.sleep(1)
        dev.erase_effect(effect_id)

# Begins main function
if __name__ == '__main__':
    rospy.init_node("am_tel_joy")
    jt = JoystickTeleop()
    
    rospy.spin()
