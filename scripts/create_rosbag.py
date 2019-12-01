#!/usr/bin/env python3.6
# -*- coding: utf-8 -*-

'''
Author  : Heethesh Vhavle
Email   : heethesh@cmu.edu
Version : 1.0.0
Date    : Nov 30, 2019
'''

# Python 2/3 compatibility
from __future__ import print_function, absolute_import, division

import os

# External modules
import numpy as np

# Handle OpenCV import
import import_cv2

# ROS modules
import rospy
import rosbag
from sensor_msgs.msg import Image, CameraInfo

import utils
from argoverse.data_loading.argoverse_tracking_loader import ArgoverseTrackingLoader


class BagConverter:
    def __init__(self, cameras_list):
        self.dataset_path = '/mnt/data/Datasets/Argoverse/argoverse-tracking/sample'
        self.log_id = 'c6911883-1843-3727-8eaa-41dc8cda8993'

        self.argoverse_loader = ArgoverseTrackingLoader(self.dataset_path)
        self.argoverse_data = self.argoverse_loader.get(self.log_id)
        
        # List of cameras to publish
        self.cameras_list = cameras_list
        self.load_camera_info()

    def load_camera_info(self):
        self.camera_info_dict = {
            camera: self.argoverse_data.get_calibration(camera)
            for camera in self.argoverse_loader.CAMERA_LIST
            if camera in self.cameras_list
            }

        cc = self.camera_info_dict[self.cameras_list[0]].camera_config
        print(utils.make_camera_info_message(cc))
        # print(camera, self.argoverse_loader.CAMERA_LIST)
        # calib = self.argoverse_data.get_calibration(camera)
        # print(calib.__dict__)



if __name__ == '__main__':
    bag_converter = BagConverter(['stereo_front_left'])

