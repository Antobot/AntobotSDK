#!/usr/bin/env python3
"""Example script to send higher level nodes progress updates"""

import rospy
import rosservice
from anto_msgs.srv import camManager, camManagerRequest,camManagerResponse

class camManagerClient():
    """A class that handles a client to provide updates to higher level nodes"""
    
    def __init__(self,camera_num=1,command=0,serviceName = '/antobot_manager/camera'):    
        
        self.serviceName = serviceName
        
        self.camManagerClient = rospy.ServiceProxy(self.serviceName, camManager)
        self.camera_num=camera_num # Data that will be sent. 0 is default, doesnt do anything, just used to test.
        self.command=command


    def checkForService(self):
        service_list = rosservice.get_service_list()
        if self.serviceName in service_list:
            return True
        else:
            return False

    def sendCameraCommand(self):
        
        # In ROS it's common to wait for a service. However, this blocks execution and is not always useful. Use checkForService method instead.
        #rospy.wait_for_service('localUserInput')    
        #camCommand = camManagerRequest
        #camCommand.camera_num=self.camera_num
        #camCommand.command=self.command

        try:
            response = self.camManagerClient(self.camera_num,self.command)
            return response

        except rospy.ServiceException as e:
            print("Service call failed: %s"%e)


if __name__ == "__main__":
    
    # Create the class to handle client-side interaction
    camClient = camManagerClient(camera_num=1,command=0) 

    # Check that the service is availble before trying to send requests
    serviceState=camClient.checkForService()

    if serviceState: # If the service is available

       camManagerResponse = camClient.sendCameraCommand()

    else:
        print('Unable to make request')
        print('ROS service ' + camClient.serviceName + ' is not available')
