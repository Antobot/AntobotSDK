#! /usr/bin/env python3
# Copyright (c) 2023, ANTOBOT LTD.
# All rights reserved.

# # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # #

# # # Code Description:     This code contains the main program for svo file and corresponding json file recording.
# # # Interfaces:           _serviceCallbackAntoRec - receive request from antobot_datamanager.antobot_cam_mgr and send response.

# Contacts: Author/Owner:  jinhuan.liu@antobot.ai


# # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # #
import os
import cv2
import yaml
import json
import threading
import numpy as np
import pyzed.sl as sl
from datetime import datetime
from pathlib import Path
from signal import signal, SIGINT

import rospy
import tf2_ros
from sensor_msgs.msg import NavSatFix
from antobot_msgs.srv import antoRec, antoRecResponse


def param_init():
    """
    Function to set parameters of the cameras.

    Returns:
        init_params (sl.InitParameters): parameters and runtime settings

    """
    init_params = sl.InitParameters()
    init_params.camera_fps = 60  # set frames per second
    init_params.camera_resolution = sl.RESOLUTION.HD720
    init_params.depth_mode = sl.DEPTH_MODE.NONE
    init_params.coordinate_units = sl.UNIT.CENTIMETER  # Use CENTIMETER units

    return init_params


def get_name():
    """
    This function generates the camera name and output file name based on the zed camera name and current timestamp

    Returns:
        cam_name (str): camera name
        output_basename (str): generated base name of the recording svo and json file

    """
    # Get the absolute path of output file
    parent_folder = Path(__file__).resolve().parent
    path = str(parent_folder) + "/RecordingConfig.yaml"
    print(path)

    with open(path, 'r') as file:
        data = yaml.safe_load(file)
        save_path = data['save_path']
        cam_name = data['zed_name']
        srv_name = data['service_name']

    if not os.path.exists(save_path):
        os.mkdir(save_path)

    current_timestamp = datetime.now().strftime("%d_%m_%Y_%H_%M_%S")
    output_basename = f"{save_path}/{cam_name}_{current_timestamp}"  # Sets save path for the camera
    return srv_name, cam_name, output_basename


class AntoRec:
    def __init__(self, argv=None):
        """
        Initialises a new AntoRec class object.

        Args:
            argv (sys.argv): system arguments
            
        """
        self.cam = sl.Camera()
        self.init_params = param_init()
        self.runtime = sl.RuntimeParameters()
        self.runtime.sensing_mode = sl.SENSING_MODE.STANDARD
        srv_name, self.cam_name, self.output_basename = get_name()

        self.display_image = False

        self.use_gps = False
        self.listener = None
        self.tfBuffer = None

        rospy.init_node(self.cam_name, anonymous=False)
        self.srvAntoRec = rospy.Service(srv_name, antoRec, self._serviceCallbackAntoRec)
        self.json_dict = self.init_transforms()
        self.thread = threading.Thread(target=self.grab_run)
        self.stop_signal = False

        signal(SIGINT, self.signal_handler)  # Allow interrupt from keyboard (CTRL + C).

        rospy.spin()

    def init_transforms(self):
        """
        Initialises a dictionary to save map origin and map to zed transforms

        Returns:
            json_dict (dict): initialised dictionary including map origin

        """
        self.tfBuffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tfBuffer)  # to receive the transforms
        # Initialise a dictionary to convert ros msg to json file
        if self.use_gps:
            gps_data = rospy.wait_for_message("/gps_map_origin", NavSatFix)
            json_dict = dict(origin=dict(longitude=gps_data.longitude, latitude=gps_data.latitude), camera_pose=[])
        else:
            json_dict = dict(origin=dict(longitude=0, latitude=0), camera_pose=[])

        return json_dict

    def grab_run(self):
        """
        Function to retrieve transforms while recording and update camera exposure settings automatically.

        """
        last_time = self.cam.get_timestamp(sl.TIME_REFERENCE.CURRENT).get_seconds()
        while not self.stop_signal:
            if self.cam.grab(self.runtime) == sl.ERROR_CODE.SUCCESS:
                if self.display_image:
                    self.show_image()

                # get current frame's map to camera transform
                self.retrieve_transform()

                # perform auto exposure every 5 seconds
                current_time = self.cam.get_timestamp(sl.TIME_REFERENCE.CURRENT).get_seconds()
                if current_time - last_time > 5:
                    left_img = sl.Mat()
                    self.cam.retrieve_image(left_img, sl.VIEW.LEFT)
                    self.set_cam_roi(left_img)
                    last_time = self.cam.get_timestamp(sl.TIME_REFERENCE.CURRENT).get_seconds()

    def _serviceCallbackAntoRec(self, request):
        """
        Callback function to handle the service request from cam manager.

        Args:
            request (antoRecRequest): command (int8)   0 - open cam, 1 - close cam,
                                                       2 - start recording, 3 - stop recording
        Returns:
            return_msg (antoRecResponse): responseCode (bool)
                                          responseString (string)
                                          
        """
        return_msg = antoRecResponse()

        if request.command == 0:      # open cam
            status = self.open_camera()

            if status == sl.ERROR_CODE.SUCCESS:
                return_msg.responseCode = True
                return_msg.responseString = f"{self.cam_name} is open."
            else:
                return_msg.responseCode = False
                return_msg.responseString = f"Cannot open {self.cam_name}, {repr(status)}."

        elif request.command == 1:    # close cam
            self.close_camera()

            return_msg.responseCode = True
            return_msg.responseString = f"{self.cam_name} has been closed."

        elif request.command == 2:    # start recording
            status = self.start_recording()

            if status == sl.ERROR_CODE.SUCCESS:
                return_msg.responseCode = True
                return_msg.responseString = f"{self.cam_name} starts recording, use Ctrl-C or send command to stop."
            else:
                return_msg.responseCode = False
                return_msg.responseString = f"{self.cam_name} recording failed, {repr(status)}."

        elif request.command == 3:    # stop recording
            self.stop_recording()

            return_msg.responseCode = True
            return_msg.responseString = f"{self.cam_name} stopped recording."

        print(return_msg.responseString)
        return return_msg

    def open_camera(self):
        """
        This function opens the requested camera and display current view if asked
         
        Returns:
            status (sl.ERROR_CODE): If SUCCESS is returned, the camera is open. Every other code indicates an error.

        """

        status = self.cam.open(self.init_params)
        if status != sl.ERROR_CODE.SUCCESS:
            print(repr(status))
            return status

        return status

    def close_camera(self):
        """
        This function close the requested camera. If recording hasn't been stopped, it will stop it firstly.

        """
        cv2.destroyAllWindows()
        if self.cam.get_recording_status().is_recording:  # if recording hasn't been stopped, stop it first
            self.stop_recording()
        self.cam.close()

    def start_recording(self):
        """
        Function to enable record the footage from the zed camera.
        
        Returns:
            err (sl.ERROR_CODE): If SUCCESS is returned, recording is started. Every other code indicates an error.

        """

        _, _, self.output_basename = get_name()  # generate recording name based on current time stamp
        recording_param = sl.RecordingParameters(f"{self.output_basename}.svo", sl.SVO_COMPRESSION_MODE.H265)
        err = self.cam.enable_recording(recording_param)
        if err != sl.ERROR_CODE.SUCCESS:
            print(repr(err))
            return err
        self.stop_signal = False
        self.thread = threading.Thread(target=self.grab_run)
        self.thread.start()

        return err

    def stop_recording(self):
        """
        Function to stop recording and save camera position to json file.

        """
        if self.cam.get_recording_status().is_recording:  # if recording hasn't been stopped, stop it first
            self.cam.disable_recording()
            self.dict2json()
            self.stop_signal = True
            self.thread.join()


    def signal_handler(self, signal_received, frame):
        """
        Function to handle interrupt from keyboard (CTRL + C). Stop recording and close camera.

        Args:
            signal_received (): the signal number
            frame ():  the current stack frame (None or a frame object)

        """
        self.close_camera()
        exit(0)


    def retrieve_transform(self):
        """
        Retrieve map to camera transforms of current frame and save it to a dictionary.

        """
        try:
            if self.use_gps:
                if self.cam_name == "ZED_LEFT":
                    cam_name = "scouting_frame_left"
                elif self.cam_name == "ZED_RIGHT":
                    cam_name = "scouting_frame_right"

                tf_msg = self.tfBuffer.lookup_transform("map", cam_name, rospy.Time(), rospy.Duration(1.0))
                trans = tf_msg.transform.translation
                rot = tf_msg.transform.rotation

                trans = [trans.x, trans.y, trans.z]
                rot = [rot.x, rot.y, rot.z, rot.w]
            else:
                trans = [0, 0, 0]
                rot = [0, 0, 0, 0]

        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            raise

        cam_pose_dict = dict(pos=trans, ori=rot)

        self.json_dict["camera_pose"].append(cam_pose_dict)


    def dict2json(self):
        """
        Dump the camera pose to a json file.

        """
        filename = f"{self.output_basename}.json"

        try:
            with open(filename, "w") as f:
                json.dump(self.json_dict, f)  # writes the camera pose as a json file
        except rospy.ROSInterruptException:
            pass

    def show_image(self):
        """
        Display the left and right view images side by side.

        """
        img = sl.Mat()
        self.cam.retrieve_image(img, sl.VIEW.SIDE_BY_SIDE)

        img_np = img.get_data()
        cv2.imshow(self.cam_name, img_np)
        cv2.waitKey(10)

    def set_cam_roi(self, left_img_mat):
        """
        Function to find dark region of current image and send this ROI to camera exposure settings.

        Args:
            left_img_mat (pyzed.sl.mat): sl.Mat class to store the left image data.

        """
        left_img_array = left_img_mat.get_data()
        left_img_gray = cv2.cvtColor(left_img_array, cv2.COLOR_BGR2GRAY)  # grayscale image
        # Utilizes the Otsu's Binarization to find the dark region(not a perfect rectangle, with noise)
        _, binary_img = cv2.threshold(left_img_gray, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
        # Return the rows which the total number of black pixels is greater than half of the width of the image
        mask = np.count_nonzero(binary_img, axis=1) < left_img_array.shape[0] / 2
        dark_idx = np.nonzero(mask)
        # find min, max index of the dark region
        try:
            min_dark_idx = np.max(np.min(dark_idx) - 5, 0)
        except:
            min_dark_idx = 0
        try:
            max_dark_idx = np.max(np.max(dark_idx) + 5, left_img_array.shape[0] - 1)
        except:
            max_dark_idx = left_img_array.shape[0] - 1
        print(min_dark_idx, max_dark_idx)
        # generate region of interest(ROI)
        roi = sl.Rect(0, min_dark_idx, left_img_array.shape[1], max_dark_idx - min_dark_idx)
        # sent ROI to zed camera settings
        self.cam.set_camera_settings_roi(sl.VIDEO_SETTINGS.AEC_AGC_ROI, roi, sl.SIDE.BOTH)


if __name__ == "__main__":
    avRec = AntoRec()