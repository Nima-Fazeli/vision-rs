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


root =  os.path.dirname(os.path.abspath(__file__)).split('/mo_jenga')[0]



class Vision:
    def __init__(self):
        # Initialize the ros node
        rospy.init_node("vision_node_server")
        
        # Initialize the vision class Jenga4D
        self.predictor = Jenga4Dpos()

        # vision service
        self.visSrv = rospy.Service('vision_rs/blocks_poses', BlockPoseService, self.handle_vision_service)
        
        # Memory for storing images
        self.filecode = -1  # Last filecode recieved
        self.image_count = 0    



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
    
    def get_image(self, color=True):
        # get image from camera and convert to cv format
        if color:
            camera_message = '/camera/color/image_raw/compressed'
        else:
            camera_message = '/camera/aligned_depth_to_color/image_raw/compressed'#'/camera/depth/image_rect_raw/compressed'
        while True:
            try:
                ros_data = rospy.wait_for_message(camera_message, CompressedImage)
                print('message received')
                rospy.loginfo('message received')
                break
            except:
                print('waiting for message')
        
        #### direct conversion ####
        if color:
            np_arr = np.fromstring(ros_data.data, np.uint8)
            print(np_arr.shape)
            image_np = cv2.imdecode(np_arr, cv2.IMREAD_COLOR) # OpenCV >= 3.0:
        
        else:
            encoding = ros_data.format
            print('encoding: %s' % encoding)
            
            np_arr = np.fromstring(ros_data.data, np.uint8)
            max_d = np.max(np_arr)
            print(max_d)
            np_arr_norm = np_arr/float(max_d)*255
            print(np_arr.shape)
            image_np = cv2.imdecode(np_arr_norm.astype(int), cv2.IMREAD_COLOR)
        #~ debug
        #~ cv2.imshow('image', image_np)
        #~ cv2.waitKey(0)
        #~ cv2.destroyAllWindows()
        
        #~ pose = self.predictor.predict_4dpos(image_np)
        #~ print(pose)
        return image_np
        
    def save_image(self,img, depth=False):
        print('Saving image')
        cd = {True:'depth',False:'color'}
        c = cd[depth]
        print('ok')
        filename = '%s_fc_%d_%d.png'%(c, self.filecode, self.image_count)
        print('ok')
        path = os.path.join(root,'mo_jenga', 'data', 'imgs', c, 'filecode_%d'%self.filecode)
        print('path creating')
        print(path)
        if not os.path.exists(path):
            print ('Creating directory: %s'%path)
            os.makedirs(path)
        print('path created')
        file_path = os.path.join(path, filename)
        rospy.loginfo('ready to save')        
        cv2.imwrite(file_path, img)
            
        
    
    def handle_vision_service(self, args):
        # service handle for the vision
        
        # work with filecode
        filecode = args.filecode
        layer = args.layer
        row = args.row

        rospy.loginfo('Filecode received: %d'%filecode)
        rospy.loginfo('Previous filecode: %d'%self.filecode)
        if filecode != self.filecode:
            self.filecode = filecode
            self.image_count = 0
        else:
            self.image_count += 1
        
        
        blocks_pose_list = []
        cv_image = self.get_image()
        print('we have image')
        
        # pass cv_image to Jenga4D Predictor
        
        # Save the image to process later
        self.save_image(cv_image) # Color
        self.save_image(self.get_image(color=False), depth=True)
        
        # TODO : Uncommment
        # blocks_pose_list = self.predictor.predict_4dpos(cv_image)
        # DEBUGG:
        blocks_pose_list = []
        blocks_pose_list.append({'x': 0.0, 'y': 0.0, 'z':0.0, 'qw': 1.0, 'qx': 0.0, 'qy': 0.0, 'qz': 0.0})
        blocks_pose_list.append({'x': 0.0, 'y': 0.0, 'z':0.0143*2, 'qw': 1.0, 'qx': 0.0, 'qy': 0.0, 'qz': 0.0})
        blocks_pose_list.append({'x': 0.026, 'y': 0.0, 'z': 0.0143*1, 'qw': 0.707, 'qx':0.0, 'qy': 0.0, 'qz': 0.707})
        blocks_pose_list.append({'x': 0.0, 'y': 0.0, 'z': 0.0143*1, 'qw': 0.707, 'qx':0.0, 'qy': 0.0, 'qz': 0.707})
        blocks_pose_list.append({'x': -0.026, 'y': 0.0, 'z': 0.0143*1, 'qw': 0.707, 'qx':0.0, 'qy': 0.0, 'qz': 0.707})
        blocks_pose_list.append({'x': 0.026, 'y': 0.0, 'z': 0.0143*3, 'qw': 0.707, 'qx':0.0, 'qy': 0.0, 'qz': 0.707})
        blocks_pose_list.append({'x': 0.0, 'y': 0.0, 'z': 0.0143*3, 'qw': 0.707, 'qx':0.0, 'qy': 0.0, 'qz': 0.707})
        blocks_pose_list.append({'x': -0.026, 'y': 0.0, 'z': 0.0143*3, 'qw': 0.707, 'qx':0.0, 'qy': 0.0, 'qz': 0.707})
        blocks_pose_list.append({'x': 0.026, 'y': 0.0, 'z': 0.0143*5, 'qw': 0.707, 'qx':0.0, 'qy': 0.0, 'qz': 0.707})
        blocks_pose_list.append({'x': 0.0, 'y': 0.0, 'z': 0.0143*5, 'qw': 0.707, 'qx':0.0, 'qy': 0.0, 'qz': 0.707})
        blocks_pose_list.append({'x': -0.026, 'y': 0.0, 'z': 0.0143*5, 'qw': 0.707, 'qx':0.0, 'qy': 0.0, 'qz': 0.707})
        blocks_pose_list.append({'x': 0.026, 'y': 0.0, 'z': 0.0143*7, 'qw': 0.707, 'qx':0.0, 'qy': 0.0, 'qz': 0.707})
        blocks_pose_list.append({'x': 0.0, 'y': 0.0, 'z': 0.0143*7, 'qw': 0.707, 'qx':0.0, 'qy': 0.0, 'qz': 0.707})
        blocks_pose_list.append({'x': -0.026, 'y': 0.0, 'z': 0.0143*7, 'qw': 0.707, 'qx':0.0, 'qy': 0.0, 'qz': 0.707})
        blocks_pose_list.append({'x': 0.026, 'y': 0.0, 'z': 0.0143*9, 'qw': 0.707, 'qx':0.0, 'qy': 0.0, 'qz': 0.707})
        blocks_pose_list.append({'x': 0.0, 'y': 0.0, 'z': 0.0143*9, 'qw': 0.707, 'qx':0.0, 'qy': 0.0, 'qz': 0.707})
        blocks_pose_list.append({'x': -0.026, 'y': 0.0, 'z': 0.0143*9, 'qw': 0.707, 'qx':0.0, 'qy': 0.0, 'qz': 0.707})
        blocks_pose_list.append({'x': 0.026, 'y': 0.0, 'z': 0.0143*11, 'qw': 0.707, 'qx':0.0, 'qy': 0.0, 'qz': 0.707})
        blocks_pose_list.append({'x': 0.0, 'y': 0.0, 'z': 0.0143*11, 'qw': 0.707, 'qx':0.0, 'qy': 0.0, 'qz': 0.707})
        blocks_pose_list.append({'x': -0.026, 'y': 0.0, 'z': 0.0143*11, 'qw': 0.707, 'qx':0.0, 'qy': 0.0, 'qz': 0.707})
        
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
