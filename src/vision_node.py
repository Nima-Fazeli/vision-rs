#!/usr/bin/env python
"""
For Launch:
# <node name="vision_node" pkg="vision-rs" type="vision_node.py" />

catkin package:
catkin_create_pkg vision-rs std_msgs std_srvs rospy roscpp

then run catkin_make
"""

import sys, time, os

sys.path.insert(0, '/home/robot2/anaconda3/envs/python-3.5/lib/python3.5/site-packages')

from Jenga4D.predict_4dpos import Jenga4Dpos

sys.path.insert(0, os.path.join(os.path.dirname(__file__), 'Jenga4D'))

# numpy
import numpy as np
from scipy.ndimage import filters

# OpenCV
import cv2

# Ros libraries
import roslib, rospy

# Ros Messages
from sensor_msgs.msg import CompressedImage
import std_srvs.srv


class Vision:
    def __init__(self):
        # Initialize the ros node
        rospy.init_node("vision_node_server")
        
        # Initialize the vision class Jenga4D
        predictor = Jenga4Dpos()
        
        # vision service
        self.visSrv = rospy.Service('vision', std_srvs.srv.Empty, self.handle_vision_service)
    
    def get_image(self):
        # get image from camera and convert to cv format
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
    
    def handle_vision_service(self, args):
        # service handle for the vision
        
        cv_image = self.get_image()
        
        # pass cv_image to Jenga4D Predictor
        
        # write the output to a file
        return []


if __name__ == "__main__":
    try:
        v = Vision()
        rospy.loginfo("[VISION] - Vision service running ...")
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("[VISION] - Vision service stopped.")
        pass
