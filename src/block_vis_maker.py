#!/usr/bin/env python

import rospy
from interactive_markers.interactive_marker_server import *
from visualization_msgs.msg import *
import tf

class DrawMarker:
    def __init__(self):
        rospy.init_node('marker_draw', anonymous=True)
        self.br = tf.TransformBroadcaster()
        
        self.server = InteractiveMarkerServer("jenga_block")
        
        self.int_marker = self.make_marker()
        self.box = self.make_block()
        self.box_control = self.make_control()
        
        self.box_control.markers.append( self.box )
        self.int_marker.controls.append( self.box_control )
        
        self.server.insert(self.int_marker, self.processFeedback)
        self.server.applyChanges()
        
    def make_marker(self):
        int_marker = InteractiveMarker()
        int_marker.header.frame_id = "camera_link"
        int_marker.name = "jenga"
        int_marker.description = "Simple jenga block"
        int_marker.pose.position.x = 0.3
        int_marker.pose.position.y = 0.0
        int_marker.pose.position.z = 0.0
        int_marker.pose.orientation.x = 5
        
        return int_marker
    
    def make_block(self):
        box_marker = Marker()
        box_marker.type = Marker.CUBE
        box_marker.scale.x = 75./1000.
        box_marker.scale.y = 25./1000.
        box_marker.scale.z = 14./1000.
        box_marker.color.r = 0.9
        box_marker.color.g = 0.3
        box_marker.color.b = 0.2
        box_marker.color.a = 1.0
        
        return box_marker
    
    def make_control(self):
        box_control = InteractiveMarkerControl()
        box_control.orientation_mode = InteractiveMarkerControl.FIXED
        box_control.always_visible = True
        
        return box_control

        
        
    def processFeedback(self):
        print "hi"
    
    
if __name__ == "__main__":
    try:
        v = DrawMarker()
        rospy.loginfo("[VISION] - Drawing markers...")
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("[VISION] - Drawing markers stopped.")
        pass
