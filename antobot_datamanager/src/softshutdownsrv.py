#!/usr/bin/env python3

# Copyright (c) 2023, ANTOBOT LTD.
# All rights reserved.

# # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # 

#Description:   This is a python script to kill the nodes and shutdown host pc using ssh
#Inputs:      soft shutdown request [softshutdown] - soft shutdown client request send from Anto_supervisor      
#Contact:     zhuang.zhou@antobot.ai

# # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # 
from antobot_msgs.srv import softShutdown, softShutdownResponse
import os
import rospy
import time
 
def softShutdownprocess(req):
    return_msg = softShutdownResponse()
    time.sleep(4)   # leave time to send /antobridge/soft_shutdown_req topic to antobridge  
    
    os.system("rosnode kill /anto_bridge")
    return_msg.responseBool = True
    return_msg.responseString = "Success!"
    
    time.sleep(2)
    os.system("shutdown -h now")  # shutdown computer now
    return return_msg

def soft_shutdown_server():
    #receiving req from antosupervisor 
    rospy.init_node('soft_shutdown_server')
    s = rospy.Service('soft_shutdown_req', softShutdown,softShutdownprocess)
    rospy.spin()

        
if __name__ == '__main__': 
    soft_shutdown_server()
    
