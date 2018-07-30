#!/usr/bin/env python

# numpy
import numpy as np

# OpenCV
import cv2

# Ros libraries
import roslib
import rospy
import tf

# Ros Messages
import sensor_msgs.msg
from geometry_msgs.msg import PointStamped
from cv_bridge import CvBridge, CvBridgeError

from image_geometry import PinholeCameraModel

class DrawFrame:
    def __init__(self):
        rospy.init_node('draw_frames', anonymous=True)
        self.listener = tf.TransformListener()
        camera_str = '/camera/color/image_raw'
        self.bridge = CvBridge()
        self.im_sub = rospy.Subscriber(camera_str, sensor_msgs.msg.Image, self.callback)
        
        cam_info_str = "/camera/color/camera_info"
        cam_info_msg = rospy.wait_for_message(cam_info_str, sensor_msgs.msg.CameraInfo)
        
        self.ph = PinholeCameraModel()
        self.ph.fromCameraInfo(cam_info_msg)
        
    def callback(self, ros_data):
        try:
            cv_im = self.bridge.imgmsg_to_cv2(ros_data, "bgr8")
        except CvBridgeError as e:
            print e
        
        (rows, cols, channels) = cv_im.shape
        if cols > 60 and rows > 60:
            cv2.circle(cv_im, (50, 50), 10, 255)
        
        # get ros time
        now = rospy.get_rostime()
        # wait for 1/30 seconds
        rospy.sleep(1./30.)
        # get tf transform between desired frames
        (trans,rot) = self.listener.lookupTransform('/camera_link','/jenga_tf', now)
        # use project3dToPixel for the origin, get it in camera (u,v)
        ps = PointStamped()
        ps.point.x = trans[0]
        ps.point.y = trans[1]
        ps.point.z = trans[2]
        ps.header = ros_data.header
        ps.header.stamp = now
        point = self.listener.transformPoint("/camera_link", ps)
        print(trans)
        print point
        uv = self.ph.project3dToPixel((point.point.x, point.point.y, point.point.z))
        print uv
        # draw a circle at (u,v)
        cv2.circle(cv_im, (int(uv[0]),int(uv[1])), 10, 255)
        
        cv2.imshow('image', cv_im)
        cv2.waitKey(3)
        
    
if __name__ == "__main__":
    try:
        v = DrawFrame()
        rospy.loginfo("[VISION] - Draw service running ...")
        rospy.spin()
    except rospy.ROSInterruptException:
        cv2.destroyAllWindows()
        rospy.loginfo("[VISION] - Draw service stopped.")
        pass
