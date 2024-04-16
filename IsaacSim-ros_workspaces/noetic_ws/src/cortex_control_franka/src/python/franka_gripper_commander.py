# Copyright (c) 2019-2022, NVIDIA CORPORATION.  All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto.  Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.
# Simple action client interface to the gripper action server.

from __future__ import absolute_import, division, print_function, unicode_literals

from franka_gripper.msg import GraspAction, GraspGoal, GraspEpsilon, MoveAction, MoveGoal

import numpy as np
import rospy
import actionlib

import argparse

# A gripper opening width of 0.8 appears full open, but Franka claims it will cause issues.
# The nominal maximum opening width is 0.7.  Here we compromise between the two.
open_pos = 0.75


class FrankaGripperCommander(object):
    def __init__(self, verbose=False):
        self.verbose = verbose
        self.grasp_client = actionlib.SimpleActionClient("/franka_gripper/grasp", GraspAction)
        self.move_client = actionlib.SimpleActionClient("/franka_gripper/move", MoveAction)
        if self.verbose:
            print("Waiting for grasp client...")
        self.grasp_client.wait_for_server()
        if self.verbose:
            print("Waiting for move client...")
        self.move_client.wait_for_server()

    def close(self, width=0.0, speed=0.03, force=40.0, grasp_eps=(0.2, 0.2), wait=True):
        grasp_goal = GraspGoal()
        grasp_goal.width = width
        grasp_goal.speed = speed
        grasp_goal.force = force
        grasp_goal.epsilon = GraspEpsilon(inner=grasp_eps[0], outer=grasp_eps[1])

        self.grasp_client.send_goal(grasp_goal)

        if wait:
            self.grasp_client.wait_for_result()
        if self.verbose:
            print("result:", self.grasp_client.get_result())

    def move(self, width, speed=0.03, wait=True):
        move_goal = MoveGoal()
        move_goal.width = width
        move_goal.speed = speed
        print("sending goal")
        self.move_client.send_goal(move_goal)
        if wait:
            print("waiting for finish")
            self.move_client.wait_for_result()
        if self.verbose:
            print("result:", self.move_client.get_result())
        print("move complete")

    def open(self, speed=0.03, wait=True):
        self.move(open_pos, speed=speed, wait=wait)


if __name__ == "__main__":

    def Grasp(args):
        print("Grasping...")
        client = actionlib.SimpleActionClient("/franka_gripper/grasp", GraspAction)

        # Waits until the action server has started up and started
        # listening for goals.
        client.wait_for_server()

        # Creates a goal to send to the action server.
        grasp_goal = GraspGoal()
        grasp_goal.width = args.grasp_width
        grasp_goal.speed = args.speed
        grasp_goal.force = args.force
        grasp_goal.epsilon = GraspEpsilon(inner=args.eps_inner, outer=args.eps_outer)

        # Sends the goal to the action server.
        print(">>>>", grasp_goal)
        client.send_goal(grasp_goal)

        # Waits for the server to finish performing the action.
        client.wait_for_result()

        # Prints out the result of executing the action
        print("result:", client.get_result())

    def Move(args):
        print("Moving...")
        client = actionlib.SimpleActionClient("/franka_gripper/move", MoveAction)

        # Waits until the action server has started up and started
        # listening for goals.
        client.wait_for_server()

        # Creates a goal to send to the action server.
        move_goal = GraspGoal()
        move_goal.width = args.width
        move_goal.speed = args.speed

        # Sends the goal to the action server.
        client.send_goal(move_goal)

        # Waits for the server to finish performing the action.
        client.wait_for_result()

        # Prints out the result of executing the action
        print("result:", client.get_result())

    def FrankaGripperCommanderTest(args):
        print("Creating gripper commander...")
        gripper_commander = FrankaGripperCommander()

        print("Closing...")
        gripper_commander.close()

        print("Opening to all the way...")
        gripper_commander.move(0.08)

        print("Opening to .2...")
        gripper_commander.move(0.02)

        print("Opening to .5...")
        gripper_commander.move(0.05)

        print("Closing...")
        gripper_commander.close()

        print("Opening to all the way...")
        gripper_commander.move(0.08)

    def RobustnessTest(args):
        commander = FrankaGripperCommander()
        mode = "open"
        while not rospy.is_shutdown():
            if mode == "open":
                commander.open(speed=0.2, wait=False)
                print("opening...")
                mode = "close"
            elif mode == "close":
                commander.close(speed=0.2, wait=False)
                print("closing...")
                mode = "open"
            else:
                raise RuntimeError("Invalid mode:", mode)

            wait_time = abs(np.random.normal(loc=0.5, scale=0.75))
            print("  wait:", wait_time)
            rospy.sleep(wait_time)

    parser = argparse.ArgumentParser("gripper_test")
    parser.add_argument(
        "--mode", type=str, required=True, help="Which mode: close, move, gripper_commander_test, robustness_test."
    )
    parser.add_argument(
        "--width",
        type=float,
        default=None,
        help="How wide in meters. Note that the gripper can open to about .8m wide.",
    )
    parser.add_argument("--speed", type=float, default=0.03, help="How fast to go in meter per second.")
    parser.add_argument("--force", type=float, default=0.03, help="How strongly to grip.")
    parser.add_argument(
        "--grasp_width",
        type=float,
        default=0.0,
        help="Width of the grasp. Defaults to closing all the way. "
        "In conjunction with the default error (set wide) the default "
        "behavior is to just close until it feels something.",
    )
    parser.add_argument(
        "--eps_inner", type=float, default=0.2, help="Inner epsilon threshold. Defaults to enabling any error."
    )
    parser.add_argument(
        "--eps_outer", type=float, default=0.2, help="Outer epsilon threshold. Defaults to enabling any error."
    )

    args = parser.parse_args()
    rospy.init_node("gripper_test")

    if args.mode == "move":
        Move(args)
    elif args.mode == "close":
        Grasp(args)
    elif args.mode == "gripper_commander_test":
        FrankaGripperCommanderTest(args)
    elif args.mode == "robustness_test":
        RobustnessTest(args)
    else:
        print("ERROR -- unrecognized mode:", args.mode)
