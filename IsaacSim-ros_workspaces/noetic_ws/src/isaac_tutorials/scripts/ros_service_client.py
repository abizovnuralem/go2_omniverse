#!/usr/bin/env python

# Copyright (c) 2021-2022, NVIDIA CORPORATION.  All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto.  Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.


import rospy
import numpy as np
from isaac_ros_messages.srv import IsaacPose
from isaac_ros_messages.srv import IsaacPoseRequest
from geometry_msgs.msg import Pose


def teleport_client(msg):
    rospy.wait_for_service("teleport")
    try:
        teleport = rospy.ServiceProxy("teleport", IsaacPose)
        teleport(msg)
        return
    except rospy.ServiceException as e:
        print("Service call failed: %s" % e)


# compose teleport messages
cube_pose = Pose()
cube_pose.position.x = np.random.uniform(-2, 2)
cube_pose.position.y = 0
cube_pose.position.z = 0
cube_pose.orientation.w = 1
cube_pose.orientation.x = 0
cube_pose.orientation.y = 0
cube_pose.orientation.z = 0

cone_pose = Pose()
cone_pose.position.x = 0
cone_pose.position.y = np.random.uniform(-2, 2)
cone_pose.position.z = 0
cone_pose.orientation.w = 1
cone_pose.orientation.x = 0
cone_pose.orientation.y = 0
cone_pose.orientation.z = 0

teleport_msg = IsaacPoseRequest()
teleport_msg.names = ["/World/Cube", "/World/Cone"]
teleport_msg.poses = [cube_pose, cone_pose]

teleport_client(teleport_msg)
