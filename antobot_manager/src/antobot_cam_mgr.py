#! /usr/bin/env python3

# Copyright (c) 2022, ANTOBOT LTD.
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

# # # Code Description:     This code manages the different cameras on Antobot's robots. It receives as requests to
# # #                       open/close and/or use cameras, and then starts the appropriate scripts based on the request,
# # #                       or passes along a request to another script, when appropriate.
# # #                       It returns whether the request was successful or not.

# Contacts: daniel.freer@antobot.ai
#           william.eaton@antobot.ai
#           meiru.zhang@antobot.ai
#           jinhuan.liu@antobot.ai

# # # #  # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # #

import sys
import yaml
import rospy
from pathlib import Path

import roslaunch # Using this until we develop our own camera manager solution

from sensor_msgs.msg import Temperature, CameraInfo
from anto_msgs.srv import camManager, camManagerResponse
from AntoVision.srv import antoRec
from sensor_msgs.msg import LaserScan
#from diagnostic_msgs.msg import DiagnosticStatus


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

        ##############################################################################
        ## Check for the simulation parameter which should be set by antobot_gazebo     
        ##############################################################################
           
        try:
            self.sim = rospy.get_param("/simulation")
        except:
            self.sim=False # If the simulation parameter has not been assigned, assume not a simulation

        if self.sim:
            self.nodeName='mobile_base/zed2_' + self.name
            # Subscribe to the camera info topic  
            self.camera_info_sub = rospy.Subscriber((self.nodeName + '/left/camera_info'), CameraInfo, self.camera_info_callback)
        else:
            # Subscribe to the camera info topic  
            self.camera_info_sub = rospy.Subscriber((self.nodeName + '/rgb/camera_info'), CameraInfo, self.camera_info_callback)




        
        # Create a publisher for laserscan data, to force empty the costmap - 
        self.laserPub_back = rospy.Publisher(('/camera/scan/'+ self.name), LaserScan, queue_size=5)
        
        # Movebase has a service for this already, but it might not work TODO - Check if this is cleaner
        # rosservice call /move_base/clear_costmaps "{}" 

        # Create a timer object in start, that checks the camera topic occassionally. 
        # If it fails to get a repsonse, run shutdown
        self.activeCheckTimer = rospy.Timer(rospy.Duration(0.5), self.checkElapsedTime) # real robot could perform upto 2.5Hz


    def createLauncher(self):
        '''Starts a camera by calling the zed_wrapper launch file'''

        # Generate a unique id for the node
        uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
        roslaunch.configure_logging(uuid)

        #cli_args = ['am_cartograph', 'antoCartographer.launch']
        cli_args = ['zed_wrapper', 'zed_no_tf.launch', 'node_name:=zed2_'+self.name+'_node', 'camera_name:=zed2_'+self.name, 'serial_number:='+str(self.serialNumber)]

        roslaunch_file = roslaunch.rlutil.resolve_launch_arguments(cli_args)[0]
        roslaunch_args = cli_args[2:]

        launch_files = [(roslaunch_file, roslaunch_args)]
        #launch_files = [roslaunch_file]
        self.launch = roslaunch.parent.ROSLaunchParent(uuid, launch_files)
        ##http://wiki.ros.org/roslaunch/API%20Usage


    def createFakeLauncher(self):
        '''Starts a pointcloud to laserscan camera to test camera manager in simulation'''

        # Generate a unique id for the node
        uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
        roslaunch.configure_logging(uuid)

        #cli_args = ['am_cartograph', 'antoCartographer.launch']
        cli_args = ['antobot_navigation', 'pointcloud_to_laser_sim.launch', 'camera_to_activate:='+self.name]

        roslaunch_file = roslaunch.rlutil.resolve_launch_arguments(cli_args)[0]
        roslaunch_args = cli_args[2:]

        launch_files = [(roslaunch_file, roslaunch_args)]
        #launch_files = [roslaunch_file]
        self.launch = roslaunch.parent.ROSLaunchParent(uuid, launch_files)
        ##http://wiki.ros.org/roslaunch/API%20Usage


    def start(self):
        if self.active==False:
            self.active=True
            self.launch.start()

    def shutdown(self):
        if self.active==True:
            self.active=False
            self.launch.shutdown()
            self.clear_costmap()
        

    def camera_info_callback(self,data):
        '''Function that subscribes to the camera info data'''
        self.lastTime=rospy.get_rostime() # Get the current time


    # 
    def clear_costmap(self):
        laserdata = LaserScan()
        laserdata.angle_min = -3.14
        laserdata.angle_max = 3.14
        laserdata.angle_increment= 0.00870
        laserdata.scan_time=0.333 
        laserdata.ranges = []
        laserdata.intensities = [0]
        laserdata.header.stamp = rospy.get_rostime()
        laserdata.header.frame_id = "zed2_" + self.name + "_camera_center"
        self.laserPub_back.publish(laserdata)


    def checkElapsedTime(self,data):
        if self.active:
            elapsedTime = rospy.get_rostime().secs - self.lastTime.secs
            print(elapsedTime)
            if elapsedTime>self.timeOut:
                self.active=False
                self.launch.shutdown()


def get_cam_sn():
    # function to read the camera serial numbers from yaml file

    parent_folder = Path(__file__).resolve().parent
    path = str(parent_folder) + "/cameras.yaml"

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
        ## Check for the simulation parameter which should be set by antobot_gazebo     
        ##############################################################################
           
        try:
            self.sim = rospy.get_param("/simulation")

        except:
            self.sim=False # If the simulation parameter has not been assigned, assume not a simulation


        ##############################################################################
        ## Run either real or simulated manager    
        ##############################################################################

        if self.sim:
            print('Camera Manager: This is a simulation - using fake camera calls.')

            # Create a service to allow other nodes to start/stop cameras
            self.srvCamMgr = rospy.Service("/antobot_manager/camera", camManager, self._serviceCallbackCamMgrFake)

            # Some state variables
            self.cam_state = 0
            self.ros_cams = []
            self.rec_cams = []
            self.rtr_num = 0
        else:
            
            print('Camera Manager: This is not a simulation - using real ZED2 camera commands.')

            # Create a service to allow other nodes to start/stop cameras
            self.srvCamMgr = rospy.Service("/antobot_manager/camera", camManager, self._serviceCallbackCamMgr)

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
    ## RosService callback Fake - This is for simulation only
    ##############################################################################

    def _serviceCallbackCamMgrFake(self, request):
       
        ## ROS service input:
        #int8 camera_num		# 1 - front, 2 - back, 3 - left, 4 - right
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

            # Create a custom message for antoVision
            cam_info_msg = cam_info()
            cam_info_msg.open_flag = True  # send the flag to open cameras
            cam_info_msg.cam_sn.append(self.cameras[request.camera_num].serialNumber)

            if request.camera_num == 3:  # Turn left camera on for recording video
                cam_info_msg.cam_id.append(0)  # left
            elif request.camera_num == 4:  # Turn right camera on for recording video
                cam_info_msg.cam_id.append(1)  # right
            
            self.open_cam_pub.publish(cam_info_msg)
        

            self.rec_cams.append(request.camera_num)
            return_msg.responseCode = 1
            return_msg.responseString = "Recording initialised for camera " + str(request.camera_num)

        elif request.command == 1:  # Roslaunch
            print("Roslaunch requested")
            self.cameras[request.camera_num].createFakeLauncher()
            #self.cameras[request.camera_num].start()
            self.cameras[request.camera_num].mainThreadCommand=1 # Start the camera node

            self.cameras[request.camera_num].lastTime=rospy.get_rostime() # Get the current time


            self.ros_cams.append(request.camera_num)
            return_msg.responseCode = 1
            return_msg.responseString = "Camera " + str(request.camera_num) + " launched"

        elif request.command == 0:  # Closes all cameras
            
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




    def check_time_f(self):
        # # # Checks the time since ROS messages have been received from the front ZED camera. If it is greater than 0.5s,
        # # # the camera is considered to be inactive.

        time_inactive = perf_counter() - self.zed_f_active_time
        if time_inactive > 0.5:
            self.zed_f_active = False

    def check_time_b(self):
        # # # Checks the time since ROS messages have been received from the back ZED camera. If it is greater than 0.5s,
        # # # the camera is considered to be inactive.

        time_inactive = perf_counter() - self.zed_b_active_time
        if time_inactive > 0.5:
            self.zed_b_active = False

    def zed_front_callback(self, data):
        # # # A callback function for a (random) ZED topic from the front camera.
        # # # This is only used to determine whether the ZED camera is still active.

        self.zed_f_active_time = perf_counter()
        self.zed_f_active = True
        return

    def zed_back_callback(self, data):
        # # # A callback function for a (random) ZED topic from the back camera.
        # # # This is only used to determine whether the ZED camera is still active.

        self.zed_b_active_time = perf_counter()
        self.zed_b_active = True
        return


    

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

