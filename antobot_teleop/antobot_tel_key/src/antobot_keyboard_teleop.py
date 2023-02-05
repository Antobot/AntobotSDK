#!/usr/bin/env python3
# -*- coding: utf-8 -*-


# # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # 

# # # Code Description:     Allows for keyboard teleoperation of the robot, using the right side of the keyboard for control commands: [[u, i, o],
# # #                       and the left side to increase/decrease speed: [q/z] (both), [w/x] (linear), e/c (angular)                   [j, k, l],
# # #                       If main command, sends Twist commands to ROS publisher. Called by master_teleop.py                          [m, ,, .]]

# Contact:      daniel.freer@antobot.ai
#               hao.an@antobot.ai

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

# Defines initial linear and angular velocities
speed = .7
turn = 1

# Defines initial values for global variables
x = 0
th = 0
status = 0
count = 0
acc = 0.1
target_speed = 0
target_turn = 0
control_speed = 0
control_turn = 0


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
        
        # 运动控制方向键（1：正方向，-1负方向） Motion control direction key (1: positive direction, -1 negative direction)
        if key in moveBindings.keys():
            self.x = moveBindings[key][0]
            self.th = moveBindings[key][1]
            self.count = 0

        # 速度修改键 Speed modifier key
        elif key in speedBindings.keys():
            self.speed = self.speed * speedBindings[key][0]  # 线速度增加0.1倍 Linear speed increased by 0.1 times
            self.turn = self.turn * speedBindings[key][1]  # 角速度增加0.1倍 Angular velocity increased by 0.1 times
            self.count = 0

            print(self.vels(self.speed, self.turn))
            if (self.status == 14):
                print(self.msg)
            status = (self.status + 1) % 15

        # 停止键 Stop button
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

        # 目标速度=速度值*方向值 Target speed = speed value * direction value
        self.target_speed = self.speed * self.x
        self.target_turn = self.turn * self.th

        # 速度限位，防止速度增减过快 Speed limit to prevent the speed from increasing or decreasing too fast
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

        # 创建并发布twist消息 Create and publish twist message

        twist = Twist()
        twist.linear.x = self.control_speed
        twist.linear.y = 0
        twist.linear.z = 0
        twist.angular.x = 0
        twist.angular.y = 0
        twist.angular.z = self.control_turn

        return twist


if __name__=="__main__":
    settings = termios.tcgetattr(sys.stdin)

    # Initialises ROS node and publisher
    rospy.init_node('mrobot_teleop')
    #pub = rospy.Publisher('antobot_robot/antobot_velocity_controller/cmd_vel', Twist, queue_size=5)
    pub = rospy.Publisher('am_robot/cmd_vel', Twist, queue_size=5)

    # Defines Keyboard_Teleop object
    kt = Keyboard_Teleop()
    try:
        # Prints initial message and initial velocity values
        kt.print_msg()
        print(kt.vels(speed,turn))
        
        # Starts while loop to continually check for keyboard input and send messages
        while(1):
            key = kt.getKey()
            twist = kt.get_twist(key)
            if twist == None:
                break
            pub.publish(twist)

    # except:
    #     print(e)

    # Sends final "stop" command when the program ends
    finally:
        twist = Twist()
        twist.linear.x = 0; twist.linear.y = 0; twist.linear.z = 0
        twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = 0
        pub.publish(twist)

    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
