#!/usr/bin/env python

import rospy
import tf
from apriltags2_ros.msg import AprilTagDetectionArray
# Ros Messages
from sensor_msgs.msg import CompressedImage, Image
import std_srvs.srv
from geometry_msgs.msg import TransformStamped, Pose, PoseStamped
from vision_rs.msg import BlocksPose
from vision_rs.srv import BlockPoseService


class VisionApril:
    def __init__(self):
        self.rospy.init_node('april_vision')
        # self.listener = tf.TransformListener()
        self.listener = tf.TransformerROS()
        self.visSrv = rospy.Service('vision_rs/blocks_poses', BlockPoseService, self.handle_vision_service)

    ####################################################
    # Service handle
    ####################################################
    def handle_vision_service(self, args):
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

        tag_id = self.get_tag_id(layer, row)

        apriltag_detections = rospy.wait_for_message('/tag_detections', AprilTagDetectionArray)
        tag_pose = [0]*7
        for det in apriltag_detections.detections:
            if det.id == tag_id:
                tag_pose[0]= det[det.id].pose.pose.pose.position.x
                tag_pose[1]= det[det.id].pose.pose.pose.position.y
                tag_pose[2]= det[det.id].pose.pose.pose.position.z
                tag_pose[3]= det[det.id].pose.pose.pose.orientation.x
                tag_pose[4]= det[det.id].pose.pose.pose.orientation.y
                tag_pose[5]= det[det.id].pose.pose.pose.orientation.z
                tag_pose[6]= det[det.id].pose.pose.pose.orientation.w

        block_pose = self.get_block_pose(tag_pose, tag_id)

        blocks_pose_list.append({'x': block_pose.pose.position.x,
                                 'y': block_pose.pose.position.y,
                                 'z': block_pose.pose.position.z,
                                 'qw': block_pose.pose.orientation.w,
                                 'qx': block_pose.pose.orientation.x,
                                 'qy': block_pose.pose.orientation.y,
                                 'qz': block_pose.pose.orientation.z})

        # write the output to a file
        str_test =  "[VISION] -  ... {}".format(blocks_pose_list[0]['z'])
        rospy.loginfo(str_test)
        os.system('echo {} > test.txt'.format(str_test))

        # Pack the values observed into a BlocksPose msg
        bp = self.pack_blocks(blocks_pose_list)
        return bp

    def get_block_pose(self, tag_pose, tag_id):
        # construct the pose
        p = PoseStamped()
        p.header.frame_id = 'tag_id'
        p.header.stamp = rospy.Time.now()
        p.pose.position.x = tag_pose[0]
        p.pose.position.y = tag_pose[1]
        p.pose.position.z = tag_pose[2]
        p.pose.orientation.x = tag_pose[3]
        p.pose.orientation.y = tag_pose[4]
        p.pose.orientation.z = tag_pose[5]
        p.pose.orientation.w = tag_pose[6]

        bp = listener.transformPose('jenga_tf', p)
        return bp

    def get_tag_id(self, layer, row):
        tag_id = (layer/2-1)*3 + row-1
        return tag_id

    ####################################################
    # Packing
    ####################################################
    def pack_blocks(self, blocks_list):
        # Initialize
        bp = BlocksPose()
        bp.header.stamp = rospy.Time.now()
        bp.header.frame_id = '/jenga_tower'
        poses = []
        for i, block in enumerate(blocks_list):
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

if __name__ == "__main__":
    try:
        v = VisionApril()
        rospy.loginfo("[VISION] - Vision service running ...")
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("[VISION] - Vision service stopped.")
        pass
