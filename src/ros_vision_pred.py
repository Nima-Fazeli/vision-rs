#!/usr/bin/env python

import sys, time
import os
import argparse
import time

sys.path.insert(0, '/home/robot2/anaconda3/envs/python-3.5/lib/python3.5/site-packages')

from Jenga4D.predict_4dpos import Jenga4Dpos

sys.path.insert(0, os.path.join(os.path.dirname(__file__), 'Jenga4D'))

# numpy
import numpy as np
from scipy.ndimage import filters

# OpenCV
import cv2

# Ros libraries
import roslib
import rospy

# Ros Messages
from sensor_msgs.msg import CompressedImage
import std_srvs


class Vision:
    def __init__(self):
        '''Initializes and cleanup ros node'''
        print('Initialized vision class ... \n')
        rospy.init_node('vision_pd', anonymous=True)
    
    def getImage(self):
        # get message
        camera_message = '/camera/color/image_raw/compressed'
        while True:
            try:
                ros_data = rospy.wait_for_message(camera_message, CompressedImage)
                print('message received')
                break
            except:
                print('waiting for message')
        
        #### direct conversion ####
        np_arr = np.fromstring(ros_data.data, np.uint8)
        image_np = cv2.imdecode(np_arr, cv2.IMREAD_COLOR) # OpenCV >= 3.0:
                
        #~ debug
        #~ cv2.imshow('image', image_np)
        #~ cv2.waitKey(0)
        #~ cv2.destroyAllWindows()
        
        return image_np
