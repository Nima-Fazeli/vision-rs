#!/usr/bin/env python

#~ Last Calibratio
#~ 0.643959164619 0.053028345108 -0.105566866696 0.0493049994111 -0.0910816714168 -0.315657109022 0.943204343319

from interactive_markers.interactive_marker_server import *
from visualization_msgs.msg import *

def processFeedback(feedback):
    p = feedback.pose.position
    print feedback.marker_name + " is now at " + str(p.x) + ", " + str(p.y) + ", " + str(p.z)


import rospy

from interactive_markers.interactive_marker_server import *
from visualization_msgs.msg import *

def framePublisher( msg ):
    p = msg.pose.position
    q = msg.pose.orientation
    s = 'Pose" ' + str(p.x) + ' '+ str(p.y) + ' '+ str(p.z) + ' '
    s = s + str(q.x) + ' ' + str(q.y) + ' ' + str(q.z) + ' ' + str(q.w)
    print(s)

if __name__=="__main__":
    rospy.init_node("calibration_marker")
    
    # create an interactive marker server on the topic namespace simple_marker
    server = InteractiveMarkerServer("simple_marker")
    
    # create an interactive marker for our server
    int_marker = InteractiveMarker()
    int_marker.header.frame_id = "camera_link"
    int_marker.name = "my_marker"
    int_marker.description = "Simple 1-DOF Control"

    # create a grey box marker
    box_marker = Marker()
    box_marker.type = Marker.CUBE
    box_marker.scale.x = 75./1000.
    box_marker.scale.y = 25./1000.
    box_marker.scale.z = 14./1000.
    box_marker.color.r = 0.0
    box_marker.color.g = 0.5
    box_marker.color.b = 0.5
    box_marker.color.a = 1.0

    # create a non-interactive control which contains the box
    box_control = InteractiveMarkerControl()
    box_control.always_visible = True
    box_control.markers.append( box_marker )

    # add the control to the interactive marker
    int_marker.controls.append( box_control )
    
    # create a control which will move the box
    control = InteractiveMarkerControl()
    control.name = "move_x"
    control.orientation.w = 1
    control.orientation.x = 1
    control.orientation.y = 0
    control.orientation.z = 0
    control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
    int_marker.controls.append(control);
    
    control = InteractiveMarkerControl()
    control.name = "rotate_x"
    control.orientation.w = 1
    control.orientation.x = 1
    control.orientation.y = 0
    control.orientation.z = 0
    control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
    int_marker.controls.append(control);
    
    control = InteractiveMarkerControl()
    control.name = "move_y"
    control.orientation.w = 1
    control.orientation.x = 0
    control.orientation.y = 1
    control.orientation.z = 0
    control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
    int_marker.controls.append(control);
    
    control = InteractiveMarkerControl()
    control.name = "rotate_y"
    control.orientation.w = 1
    control.orientation.x = 0
    control.orientation.y = 1
    control.orientation.z = 0
    control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
    int_marker.controls.append(control);
    
    control = InteractiveMarkerControl()
    control.name = "move_z"
    control.orientation.w = 1
    control.orientation.x = 0
    control.orientation.y = 0
    control.orientation.z = 1
    control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
    int_marker.controls.append(control);
    
    control = InteractiveMarkerControl()
    control.name = "rotate_z"
    control.orientation.w = 1
    control.orientation.x = 0
    control.orientation.y = 0
    control.orientation.z = 1
    control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
    int_marker.controls.append(control);
    
    # add the interactive marker to our collection &
    # tell the server to call processFeedback() when feedback arrives for it
    server.insert(int_marker, framePublisher)

    # 'commit' changes and send to all clients
    server.applyChanges()

    rospy.spin()
