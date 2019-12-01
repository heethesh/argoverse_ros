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

# Handle OpenCV import
import import_cv2

# External modules
import numpy as np

# ROS modules
import rospy
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image, CameraInfo


def cv2_to_message(img, pub, coding='bgr8'):
    # Publish image using CV bridge
    if len(img.shape) == 2:
        img = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)
    try:
        pub.publish(CV_BRIDGE.cv2_to_imgmsg(img, coding))
    except CvBridgeError as e: 
        print(e)
        rospy.logerr(e)


def make_camera_info_message(camera_config):
    msg = CameraInfo()
    msg.height = camera_config.img_height
    msg.width = camera_config.img_width
    msg.distortion_model = 'plumb_bob'
    dist = camera_config.distortion_coeffs  # K1, K2, K3
    msg.D = np.array([dist[0], dist[1], 0.0, 0.0, dist[2]])  # K1, K2, T1, T2, K3
    msg.K = camera_config.intrinsic[:3, :3].flatten()
    msg.R = np.eye(3).flatten()
    # TODO: Update TX, TY for stereo images
    P = np.zeros((3, 4))
    msg.P = camera_config.intrinsic.flatten()
    return msg
