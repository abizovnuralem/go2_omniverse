#!/usr/bin/env python

# Copyright (c) 2021-2022, NVIDIA CORPORATION.  All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto.  Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.


import rospy
from sensor_msgs.msg import JointState

joints_dict = {}


def joint_states_callback(message):

    joint_commands = JointState()

    joint_commands.header = message.header

    for i, name in enumerate(message.name):

        # Storing arm joint names and positions
        joints_dict[name] = message.position[i]

        if name == "panda_finger_joint1":

            # Adding additional panda_finger_joint2 state info (extra joint used in isaac sim)
            # panda_finger_joint2 mirrors panda_finger_joint1
            joints_dict["panda_finger_joint2"] = message.position[i]

    joint_commands.name = joints_dict.keys()
    joint_commands.position = joints_dict.values()

    # Publishing combined message containing all arm and finger joints
    pub.publish(joint_commands)

    return


if __name__ == "__main__":
    rospy.init_node("panda_combined_joints_publisher")
    pub = rospy.Publisher("/joint_command", JointState, queue_size=1)
    rospy.Subscriber("/joint_command_desired", JointState, joint_states_callback, queue_size=1)
    rospy.spin()
