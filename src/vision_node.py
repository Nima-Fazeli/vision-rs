#!/usr/bin/env python
"""
For Launch:
# <node name="vision_node" pkg="vision_rs" type="vision_node.py" />

catkin package:
catkin_create_pkg vision_rs std_msgs std_srvs rospy roscpp

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
from geometry_msgs.msg import TransformStamped, Pose
from vision_rs.msg import BlocksPose
from vision_rs.srv import BlockPoseService






class Vision:
    def __init__(self):
        # Initialize the ros node
        rospy.init_node("vision_node_server")
        
        # Initialize the vision class Jenga4D
        self.predictor = Jenga4Dpos()

        # vision service
        self.visSrv = rospy.Service('vision_rs/blocks_poses', BlockPoseService, self.handle_vision_service)



    def pack_blocks(self, blocks_list):
        # Initialize
        bp = BlocksPose()
        bp.header.stamp = rospy.Time.now()
        bp.header.frame_id = '/jenga_tower'
        poses = []
        for block in blocks_list:
            # TODO: fill the data with the one form blocks_list
            # Pack the values
            pose = Pose()
            pose.position.x = block['x']
            pose.position.y = block['y']
            pose.position.z = block['z']
            pose.orientation.x = block['qx']
            pose.orientation.y = block['qy']
            pose.orientation.z = block['qz']
            pose.orientation.w = block['qw']
            poses.append(pose)

        bp.blocks = poses
        return bp
    
    def get_image(self):
        # get image from camera and convert to cv format
        camera_message = '/camera/color/image_raw/compressed'
        while True:
            try:
                ros_data = rospy.wait_for_message(camera_message, CompressedImage)
                print('message received')
                rospy.loginfo('message received')
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
        
        #~ pose = self.predictor.predict_4dpos(image_np)
        #~ print(pose)
        return image_np
    
    def handle_vision_service(self, args):
        # service handle for the vision
        blocks_pose_list = []
        cv_image = self.get_image()
        
        # pass cv_image to Jenga4D Predictor
        # TODO : Uncommment
        # blocks_pose_list = self.predictor.predict_4dpos(cv_image)
        # DEBUGG:
        blocks_pose_list = []
        blocks_pose_list.append({'x': 0.0, 'y': 0.0, 'z':0.0, 'qw': 1.0, 'qx': 0.0, 'qy': 0.0, 'qz': 0.0})
        blocks_pose_list.append({'x': 0.0, 'y': 0.0, 'z':0.0143*2, 'qw': 1.0, 'qx': 0.0, 'qy': 0.0, 'qz': 0.0})
        #~ blocks_pose_list.append({'x': 0.026, 'y': 0.0, 'z': 0.0143*5, 'qw': 0.707, 'qx':0.0, 'qy': 0.0, 'qz': 0.707})
        blocks_pose_list.append({'x': -0.026, 'y': 0.0, 'z': 0.0143*5, 'qw': 0.707, 'qx':0.0, 'qy': 0.0, 'qz': 0.707})
        blocks_pose_list.append({'x': 0.026, 'y': 0.0, 'z': 0.0143*7, 'qw': 0.707, 'qx':0.0, 'qy': 0.0, 'qz': 0.707})

        # write the output to a file

        # Pack the values observed into a BlocksPose msg
        bp = self.pack_blocks(blocks_pose_list)

        # Return the service result
        return bp


if __name__ == "__main__":
    try:
        v = Vision()
        rospy.loginfo("[VISION] - Vision service running ...")
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("[VISION] - Vision service stopped.")
        pass
