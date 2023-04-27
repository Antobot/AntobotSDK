#!/usr/bin/env python3

# Copyright (c) 2019, ANTOBOT LTD.
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

#Description:   This is a python script to kill the nodes and shutdown host pc using ssh
#Interface:
#Inputs:      soft shutdown request [softshutdown] - soft shutdown client request send from Anto_supervisor      
#Contact:     zhuang.zhou@antobot.ai

# # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # 
from antobot_msgs.srv import softShutdown, softShutdownResponse
import os
import rospy
import time


 
def softShutdownprocess(req):
    return_msg = softShutdownResponse()
    print("soft shutdown service callback entered!")
    print("Sending softshutdown request to Aurix")
    time.sleep(4) #leave time to send /antobridge/soft_shutdown_req topic to antobridge, repeat 100 times    
    print("killing nodes now")
    os.system("rosnode kill /anto_bridge")
    return_msg.responseBool = True
    return_msg.responseString = "Success!"
    
    print("will shutdown in 2 sec ")
    time.sleep(2)
    os.system("shutdown -h now") #shutdown computer now
    return return_msg

def soft_shutdown_server():
    #receiving req from antosupervisor 
    rospy.init_node('soft_shutdown_server')
    s = rospy.Service('soft_shutdown_req', softShutdown,softShutdownprocess)
    rospy.spin()

        
if __name__ == '__main__': 
    soft_shutdown_server()
    
