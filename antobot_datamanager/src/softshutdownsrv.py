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
import paramiko #the 3rd party tool to ssh
from anto_msgs.srv import softshutdown, softshutdownResponse
import os
import rospy
import time

#host info to access from inside docker, can be removed if no docker in robot platform
host = 'antobot-desktop'
user = 'antobot'
password = 'Antobot2021-'
 
def ssh_exec_command(command): 
    #ssh the host, send command passed from softshutdownprocess function
    try: 
        ssh_client = paramiko.SSHClient() 
        ssh_client.set_missing_host_key_policy(paramiko.AutoAddPolicy()) 
        ssh_client.connect(host,22,user,password)       
        print("command: " + command) #command passed from softshutdownprocess function
        std_in, std_out, std_err = ssh_client.exec_command(command, get_pty=True) 
        std_in.write(password + '\n')
        for line in std_out: 
            print(line.strip("\n"))
        for line in std_err: 
            print(line.strip("\n"))         
        ssh_client.close() 
    except Exception as e: 
        print("error: " + str(e)) 

def softshutdownprocess(req):
    return_msg = softshutdownResponse()
    print("soft shutdown service callback entered!")
    print("Sending softshutdown request to Aurix")
    #ssh_exec_command("sudo shutdown +30") #shut down in 30 sec sudo shutdown -h now -h -t 30
    time.sleep(4) #leave time to send /antobridge/soft_shutdown_req topic to antobridge, repeat 100 times
     
    print("killing nodes now")
    ##if want to kill all the nodes, it will take 15 sec
    # nodes = os.popen("rosnode list").readlines()
    # for i in range(len(nodes)):
    #     nodes[i] = nodes[i].replace("\n","")
    # for node in nodes:
    #     if node != "/anto_bridge" and node != "/anto_supervisor" and node !="/soft_shutdown_server": #tbc if need this 
    #         os.system("rosnode kill "+ node)
    #os.system(" rosnode kill /anto_supervisor")
    os.system("rosnode kill /anto_bridge")
    return_msg.responseBool = True
    return_msg.responseString = "Success!"
    
    print("will shutdown in 2 sec ")
    time.sleep(2)
    os.system("sudo shutdown -h now") #shutdown computer now
    return return_msg

def soft_shutdown_server():
    #receiving req from antosupervisor 
    rospy.init_node('soft_shutdown_server')
    s = rospy.Service('soft_shutdown_req', softshutdown,softshutdownprocess)
    rospy.spin()

        
if __name__ == '__main__': 
    soft_shutdown_server()
    
