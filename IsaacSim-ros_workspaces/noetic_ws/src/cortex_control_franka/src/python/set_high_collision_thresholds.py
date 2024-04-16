#!/usr/bin/env python

# Copyright (c) 2021-2022, NVIDIA CORPORATION.  All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto.  Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.
# Simple action client interface to the gripper action server.

import rospy
from franka_control.srv import SetJointImpedance
from franka_control.srv import SetJointImpedanceRequest
from franka_control.srv import SetForceTorqueCollisionBehavior
from franka_control.srv import SetForceTorqueCollisionBehaviorRequest

rospy.init_node("set_control_parameters")
force_torque_srv = "/franka_control/set_force_torque_collision_behavior"

lower_torque_thresholds_nominal = [1000.0, 1000.0, 1000.0, 1000.0, 1000.0, 1000.0, 1000.0]
upper_torque_thresholds_nominal = [1000.0, 1000.0, 1000.0, 1000.0, 1000.0, 1000.0, 1000.0]
lower_force_thresholds_nominal = [1000.0, 1000.0, 1000.0, 1000.0, 1000.0, 1000.0]
upper_force_thresholds_nominal = [1000.0, 1000.0, 1000.0, 1000.0, 1000.0, 1000.0]

ft_req = SetForceTorqueCollisionBehaviorRequest()

ft_req.lower_torque_thresholds_nominal = lower_torque_thresholds_nominal
ft_req.upper_torque_thresholds_nominal = upper_torque_thresholds_nominal
ft_req.lower_force_thresholds_nominal = lower_force_thresholds_nominal
ft_req.upper_force_thresholds_nominal = upper_force_thresholds_nominal

print(ft_req)

rospy.loginfo("Waiting for services...")
rospy.wait_for_service(force_torque_srv)
rospy.loginfo("Services ready.")

ft_srv = rospy.ServiceProxy(force_torque_srv, SetForceTorqueCollisionBehavior)

resp = ft_srv(ft_req)
failed = False
if not resp.success:
    rospy.logerr("Could not set force torque collision behavior!")
    failed = True
else:
    rospy.loginfo("Set force torque collision behavior!")

if failed:
    raise RuntimeError("Failed to set control parameters")
