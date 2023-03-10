#! /usr/bin/env python3

# Copyright (c) 2023, ANTOBOT LTD.
# All rights reserved.

# # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # #

# # # Code Description:     This code manages the different cameras on Antobot's robots. It receives as requests to
# # #                       open/close and/or use cameras, and then starts the appropriate scripts based on the request,
# # #                       or passes along a request to another script, when appropriate.
# # #                       It returns whether the request was successful or not.

# Contact:     jinhuan.liu@antobot.ai

# # # #  # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # #

import sys
import yaml
import rospy
from pathlib import Path

import roslaunch # Using this until we develop our own camera manager solution

from sensor_msgs.msg import Temperature, CameraInfo
from antobot_msgs.srv import camManager, camManagerResponse, antoRec
from sensor_msgs.msg import LaserScan
#from diagnostic_msgs.msg import DiagnosticStatus

class RoslaunchWrapperObject(roslaunch.parent.ROSLaunchParent):

    def start(self):
        super(RoslaunchWrapperObject, self).start()
        print(self.server)

    def stop(self):
        print("Stopping...")
        print(self.server)
        server = self.server.server if self.server is not None and self.server.server is not None else None
        super(RoslaunchWrapperObject, self).shutdown()
        if server:
           server.shutdown()

class camera:
    '''Stores camera specific information'''
    def __init__(self,name='front',serialNumber=28853283,timeOut=20):


        self.active=False # Camera will always start inactive
        self.lastTime=rospy.get_rostime() # Get the current time
        
        self.serialNumber=serialNumber
        self.name=name
        self.nodeName='zed2_' + self.name + '_node'

        self.timeOut=timeOut # Ammount of time to allow before camera is assumed to be unresponsive and the node killed

        self.mainThreadCommand=-1 # Don't do anythin

        # Subscribe to the camera info topic  
        self.camera_info_sub = rospy.Subscriber((self.nodeName + '/rgb/camera_info'), CameraInfo, self.camera_info_callback)

        # Create a timer object in start, that checks the camera topic occassionally. 
        # If it fails to get a repsonse, run shutdown
        self.activeCheckTimer = rospy.Timer(rospy.Duration(0.5), self.checkElapsedTime) # real robot could perform upto 2.5Hz


    def createLauncher(self):
        '''Starts a camera by calling the zed_wrapper launch file'''

        # Generate a unique id for the node
        uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
        roslaunch.configure_logging(uuid)

        cli_args = ['zed_wrapper', 'zed_no_tf.launch', 'node_name:=zed2_'+self.name+'_node', 'camera_name:=zed2_'+self.name, 'serial_number:='+str(self.serialNumber)]

        roslaunch_file = roslaunch.rlutil.resolve_launch_arguments(cli_args)[0]
        roslaunch_args = cli_args[2:]

        launch_files = [(roslaunch_file, roslaunch_args)]
        #launch_files = [roslaunch_file]
        #self.launch = roslaunch.parent.ROSLaunchParent(uuid, launch_files)
        self.launch = RoslaunchWrapperObject(run_id = uuid, roslaunch_files = launch_files)
        ##http://wiki.ros.org/roslaunch/API%20Usage


    def start(self):
        if self.active==False:
            self.active=True
            self.launch.start()

    def shutdown(self):
        if self.active==True:
            self.active=False
            self.launch.stop()
        

    def camera_info_callback(self,data):
        '''Function that subscribes to the camera info data'''
        self.lastTime=rospy.get_rostime() # Get the current time


    def checkElapsedTime(self,data):
        if self.active:
            elapsedTime = rospy.get_rostime().secs - self.lastTime.secs
            print(elapsedTime)
            if elapsedTime>self.timeOut:
                self.active=False
                self.launch.shutdown()


def get_cam_sn():
    # function to read the camera serial numbers from yaml file

    parent_folder = Path(__file__).resolve().parent.parent
    path = str(parent_folder) + "/config/cameras.yaml"

    with open(path, 'r') as file:
        data = yaml.safe_load(file)
        front_cam_sn = data['front_cam_sn']
        back_cam_sn = data['back_cam_sn']

    return front_cam_sn, back_cam_sn


class AvCamMgr:
    def __init__(self):
        front_cam_sn, back_cam_sn = get_cam_sn()

        # Create objects for each camera
        self.cameras=dict()
        self.cameras[1]=camera(name='front',serialNumber=front_cam_sn,timeOut=20)
        self.cameras[2]=camera(name='back',serialNumber=back_cam_sn,timeOut=20)

        ##############################################################################
        ## Run Camera manager    
        ##############################################################################

        print('Camera Manager: ZED2 camera commands.')

        # Create a service to allow other nodes to start/stop cameras
        self.srvCamMgr = rospy.Service("/antobot_datamanager/camera", camManager, self._serviceCallbackCamMgr)

        self.antoRecClientLeft = rospy.ServiceProxy('anto_rec_left', antoRec)
        self.antoRecClientRight = rospy.ServiceProxy('anto_rec_right', antoRec)

        # Some state variables
        self.cam_state = 0
        self.ros_cams = []
        self.rec_cams = []
        self.rtr_num = 0


    ##############################################################################
    ## RosService callback - This is the main method of interaction   
    ##############################################################################

    def _serviceCallbackCamMgr(self, request):
       
        ## ROS service input:
        #int8 camera_num		# 1 - front, 2 - back, 3 - left, 4 - right, 5 - left and right
        #int8 command			# 0 - stop cameras
                                # 1 - Start front or back cameras using ROS launch for antomove
                                # 2 - Pass the command along to antoVision to open either left or right cameras
        
        ## ROS Service response:
        #int8 responseCode		# 1 - success, 0 - failure
        #string responseString	# Additional info

        # Create the return message
        return_msg = camManagerResponse()

        if request.command == 2:  # Recording video using antoVision

            # The logic here can only handle 1 camera at once
            # Presumably antoVision handles multi camera functionality

            if request.camera_num == 3 or request.camera_num == 5:  # Turn left camera on for recording video
                try:
                    open_response = self.antoRecClientLeft(0)

                    if open_response.responseCode:
                        rec_response = self.antoRecClientLeft(2)
                        return_msg.responseCode = rec_response.responseCode
                        return_msg.responseString = rec_response.responseString
                    else:
                        return_msg.responseCode = open_response.responseCode
                        return_msg.responseString = open_response.responseString
                except rospy.ServiceException as e:
                    print("Service call failed: %s" % e)
            if request.camera_num == 4 or request.camera_num == 5:  # Turn right camera on for recording video
                try:
                    open_response = self.antoRecClientRight(0)

                    if open_response.responseCode:
                        rec_response = self.antoRecClientRight(2)
                        return_msg.responseCode &= rec_response.responseCode
                        return_msg.responseString += rec_response.responseString
                    else:
                        return_msg.responseCode &= open_response.responseCode
                        return_msg.responseString += open_response.responseString

                except rospy.ServiceException as e:
                    print("Service call failed: %s" % e)


        elif request.command == 1:  # Roslaunch
            print("Roslaunch requested")
            self.cameras[request.camera_num].createLauncher()
            #self.cameras[request.camera_num].start()
            self.cameras[request.camera_num].mainThreadCommand=1 # Start the camera node

            self.cameras[request.camera_num].lastTime=rospy.get_rostime() # Get the current time


            self.ros_cams.append(request.camera_num)
            return_msg.responseCode = 1
            return_msg.responseString = "Camera " + str(request.camera_num) + " launched"

        elif request.command == 0:  # Closes all cameras
            # stop recording and close cameras
            self.stop_rec_and_close()

            self.cameras[1].shutdown()
            self.cameras[2].shutdown()

            self.ros_cams = []
            self.rec_cams = []
            return_msg.responseCode = 1
            return_msg.responseString = "Turning off all cameras"

        return return_msg

    ##############################################################################
    ## antoVision specific functions below
    ##############################################################################

    def stop_rec_and_close(self):
        return_msg = camManagerResponse()

        try:
            stop_rec_response = self.antoRecClientLeft(3)
            if stop_rec_response.responseCode:
                close_response = self.antoRecClientLeft(1)
                return_msg.responseCode = close_response.responseCode
                return_msg.responseString = close_response.responseString
            else:
                return_msg.responseCode = stop_rec_response.responseCode
                return_msg.responseString = stop_rec_response.responseString
        except rospy.ServiceException as e:
            print("Service call failed: %s" % e)

        try:
            stop_rec_response = self.antoRecClientRight(3)
            if stop_rec_response.responseCode:
                close_response = self.antoRecClientRight(1)
                return_msg.responseCode &= close_response.responseCode
                return_msg.responseString += close_response.responseString
            else:
                return_msg.responseCode &= stop_rec_response.responseCode
                return_msg.responseString += stop_rec_response.responseString
        except rospy.ServiceException as e:
            print("Service call failed: %s" % e)

        return return_msg
   

######################################################################################################
## Main
######################################################################################################

def main(args):
    
    rospy.init_node('antobot_cam_mgr', anonymous=False)
    cameraManager = AvCamMgr()

    rate = rospy.Rate(10) # 10hz

    # Due to rospy only allowing nodes to be called from within the main thread, we need to move them into here
    while not rospy.is_shutdown():

        for cameraIdx in cameraManager.cameras:

            if cameraManager.cameras[cameraIdx].mainThreadCommand==1:
                cameraManager.cameras[cameraIdx].start()
                cameraManager.cameras[cameraIdx].mainThreadCommand=-1 # Reset the command
                print('Camera Launched')
    
        rate.sleep()

if __name__ == '__main__':
    main(sys.argv)

