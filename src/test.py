#!/usr/bin/env python

import rospy
from vision_rs.srv import BlockPoseService

proxy_srv = rospy.ServiceProxy('vision_rs/blocks_poses', BlockPoseService)

resp = proxy_srv(0)

print(resp)
