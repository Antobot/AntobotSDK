# -*- coding:utf-8 -*- 
import paramiko 
from anto_msgs.srv import softShutdown, softShutdownResponse
import os

# We definitely cannot put our passwords on the SDK!
# However, I suppose if we are installing the system, we will have to tell them their hostname, username, and password.
host = 'antobot-desktop'
user = 'antobot'
password = 'Antobot2021-'
 
def ssh_exec_command(command): 
    try: 
        ssh_client = paramiko.SSHClient() 
        ssh_client.set_missing_host_key_policy(paramiko.AutoAddPolicy()) 
        ssh_client.connect(host,22,user,password) 
        
        print("command: " + command)
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
    return_msg = softShutdownResponse()
    print("soft shutdown service callback entered!")

    #main function: send topic to antobridge, repeat 60 times, kill all the other nodes, shutdown 

    ssh_exec_command("sudo shutdown -h -t 30") #shut down in 30 sec sudo shutdown -h now
     
    print("computer will be poweroff in 30 seconds")
    nodes = os.popen("rosnode list").readlines()
    for i in range(len(nodes)):
        nodes[i] = nodes[i].replace("\n","")

    for node in nodes:
        if node != anto_bridge or anto_supervisor: #tbc if need this 
            os.system("rosnode kill "+ node)
   
    return_msg.responseBool = True
    return_msg.responseString = "Success!"

    print("shutdown command sent and nodes killed")

    return return_msg

def soft_shutdown_server():
    rospy.init_node('soft_shutdown_server')
    s = rospy.Service('soft_shutdown_req', softShutdown,softshutdownprocess)
    
    rospy.spin()

        
if __name__ == '__main__': 
    soft_shutdown_server()
    
