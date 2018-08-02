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
import tf

# Ros Messages
from sensor_msgs.msg import CompressedImage
import std_srvs.srv
from std_msgs.msg import Header
from geometry_msgs.msg import TransformStamped, Pose
from vision_rs.msg import BlocksPose
from vision_rs.srv import BlocksPoseService
from mo_jenga_planning.msg import Block, BlockBelief, Layer, Tower





class Vision:
    def __init__(self):
        # Initialize the ros node
        rospy.init_node("vision_node_server")
        
        # Initialize the vision class Jenga4D
        self.predictor = Jenga4Dpos()

        # vision service
        self.visSrv = rospy.Service('vision', BlocksPoseService, self.handle_vision_service)

        self.tower_state_world = np.empty((17, 3))  # Contains where the block should be
        self.tower_state_tower = np.empty((17, 3))  # Contains where the block should be

        # Subscriber for updating the tower state
        self.tower_subscriber = rospy.Subscriber('tower_state', Tower, self.updateTowerState)

        self.transformer = tf.Transformer(True)
        self.tf_broadcaster = tf.TransformBroadcaster()

    def pack_blocks(self, blocks_list):
        # Initialize
        bp = BlocksPose()
        bp.header = Header()
        bp.header.stamp = rospy.Time.now()
        bp.header.frame_id = '/jenga_tower'
        blocks = []
        for block in blocks_list:
            # TODO: fill the data with the one form blocks_list
            bb = {'x':None, 'y':None, 'z':None, 'qw':None, 'qx':None, 'qy':None, 'qz':None, 'exists':True}
            layer, row = self.get_the_closest_block(bb)
            # Broadcast the block
            self.broadcast_block(layer,row,bb)

            # Pack the values
            block_msg = Block()
            block_msg.header.stamp = rospy.Time.now()
            block_msg.header.frame_id = '/jenga_tower'
            block_msg.layer = layer
            block_msg.row = row
            block_msg.exists = bb['exisits']
            block_msg.pose.position.x = bb['x']
            block_msg.pose.position.y = bb['y']
            block_msg.pose.position.z = bb['z']
            block_msg.pose.orientation.x = bb['qx']
            block_msg.pose.orientation.y = bb['qy']
            block_msg.pose.orientation.z = bb['qz']
            block_msg.pose.orientation.w = bb['qw']
            
        bp.blocks = blocks

        return bp

    def updateTowerState(self,tower_msg):
        """
        Update the current tower state (the positions of where the blocks should be)
        Unpack the values form the tower_msg to the self.tower_state
        :param tower_msg:
        :return:
        """
        num_layers = tower_msg.num_layers

        updated_tower_world = np.full((num_layers,3),{'x':None, 'y':None, 'z':None, 'qw':None, 'qx':None, 'qy':None, 'qz':None, 'exists':False})
        updated_tower_tower = np.full((num_layers,3),{'x':None, 'y':None, 'z':None, 'qw':None, 'qx':None, 'qy':None, 'qz':None, 'exists':False})

        for i_layer in range(num_layers):
            layer = tower_msg.layers[i_layer]
            for i_row in range(3):
                # Update the world coordinates
                block = layer.layer_blocks[i_row]
                updated_tower_world[i_layer,i_row]['x'] = block.pose.position.x
                updated_tower_world[i_layer,i_row]['y'] = block.pose.position.y
                updated_tower_world[i_layer,i_row]['z'] = block.pose.position.z
                updated_tower_world[i_layer,i_row]['qw'] = block.pose.orientation.w
                updated_tower_world[i_layer,i_row]['qx'] = block.pose.orientation.x
                updated_tower_world[i_layer,i_row]['qy'] = block.pose.orientation.y
                updated_tower_world[i_layer,i_row]['qz'] = block.pose.orientation.z
                updated_tower_world[i_layer,i_row]['exists'] = block.exists

                # Get the coordinates in /jenga_tf reference
                m = TransformStamped()
                m.header.frame_id = '/world'
                m.child_frame_id = '/block_%d_%d'%(i_layer+1,i_row+1)
                m.transform.translation.x = block.pose.position.x
                m.transform.translation.y = block.pose.position.y
                m.transform.translation.z = block.pose.position.z
                m.transform.rotation.x = block.pose.orientation.x
                m.transform.rotation.y = block.pose.orientation.y
                m.transform.rotation.z = block.pose.orientation.z
                m.transform.rotation.w = block.pose.orientation.w
                # Set the transform
                self.transformer.setTransform()
                print(self.transformer.getFrameStrings())
                pos, rot = self.transformer.lookupTransform('/jenga_tf','/block_%d_%d'%(i_layer+1,i_row+1),rospy.Time(0))
                # Update the results
                updated_tower_tower[i_layer, i_row]['x'] = pos.x
                updated_tower_tower[i_layer, i_row]['y'] = pos.y
                updated_tower_tower[i_layer, i_row]['z'] = pos.z
                updated_tower_tower[i_layer, i_row]['qw'] = rot.w
                updated_tower_tower[i_layer, i_row]['qx'] = rot.x
                updated_tower_tower[i_layer, i_row]['qy'] = rot.y
                updated_tower_tower[i_layer, i_row]['qz'] = rot.z
                updated_tower_tower[i_layer, i_row]['exists'] = block.exists


        self.tower_state_world = updated_tower_world
        self.tower_state_tower = updated_tower_tower

    def get_the_closest_block(self, block):
        """
        Return the layer and row of the closes block in the tower_state
        :param block: <dict> containing the cart and orientation
        :return: <(int,int)> layer and row of the block
        """

        block_ori = np.array([block['qx'], block['qy'], block['qz'], block['qw']])[:,None]

        # Priority list:
        # Layer & Orientation
        num_layers = self.tower_state_tower.shape[0]
        layers_z = np.empty(num_layers)
        layers_ori = np.empty((num_layers,4))
        for i_layer in range(num_layers):
            layers_z[i_layer] = self.tower_state_tower[i_layer, 0]['z']
            layers_ori[i_layer,:] = np.array([self.tower_state_tower[i_layer, 0]['qx'], self.tower_state_tower[i_layer, 0]['qy'], self.tower_state_tower[i_layer, 0]['qz'], self.tower_state_tower[i_layer, 0]['qw']])
        # Compute the distance
        layers_diff = np.abs(layers_z-block['z'])
        layers_ori_diff = np.sum(np.abs(layers_ori-np.repeat(block_ori.T, 4, axis=0)),axis=1)
        # Compute the score to minimize
        layers_score = layers_diff/max(layers_diff) + layers_ori_diff/max(layers_ori_diff)
        result_layer = np.argmin(layers_score) + 1

        # row position given that
        # TODO: Check if that's correct
        is_y_oriented = True
        if layers_ori[result_layer-1,0] == 0 and layers_ori[result_layer-1,1] == 0 and layers_ori[result_layer-1,2] != 0 and layers_ori[result_layer-1,3] !=0:
            is_y_oriented = False

        horizontal_vals = []
        for i_row in range(3):
            tower_block = self.tower_state_tower[result_layer - 1, i_row]
            # TODO: Check if that's correct
            if is_y_oriented:
                key = 'x'
            else:
                key = 'y'
            if tower_block['exists']:
                horizontal_vals.append(tower_block[key])
            else:
                horizontal_vals.append(10000.0)
            result_row = np.argmin(np.abs(np.array(horizontal_vals) - block[key]))

        return (result_layer, result_row)
    
    
    
    def broadcast_block(self, layer, row, block_info):
        self.tf_broadcaster([block_info['x'],block_info['y'],block_info['z']], [block_info['qx'],block_info['qy'],block_info['qz'],block_info['qw']], rospy.Time.now(), 'block_%d_%d'%(layer, row), 'jenga_tf')
    
    
    
    

    
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
        
        #~ pose = self.predictor.predict_4dpos(image_np)
        #~ print(pose)
        return image_np
    
    def handle_vision_service(self, args):
        # service handle for the vision

        blocks_pose_list = []
        cv_image = self.get_image()
        
        # pass cv_image to Jenga4D Predictor
        blocks_pose_list = self.predictor.predict_4dpos(cv_image)

        # write the output to a file

        # Pack the values observed into a BlocksPose msg
        bp = self.pack_blocks(blocks_pose_list)

        # Return the service result
        return BlocksPoseServiceResponse(bp)


if __name__ == "__main__":
    try:
        v = Vision()
        rospy.loginfo("[VISION] - Vision service running ...")
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("[VISION] - Vision service stopped.")
        pass
