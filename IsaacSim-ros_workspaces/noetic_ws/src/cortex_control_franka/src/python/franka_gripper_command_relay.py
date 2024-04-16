#!/usr/bin/python

# Copyright (c) 2019-2022, NVIDIA CORPORATION.  All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto.  Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.
# Simple action client interface to the gripper action server.

from __future__ import print_function

import argparse
import json
import threading

import rospy
from sensor_msgs.msg import JointState
from std_msgs.msg import String

from franka_gripper_commander import FrankaGripperCommander


pinch_width = 0.0265
speed = 0.2


class SimGripperCommander(object):
    def __init__(self):
        pass

    def move(self, width, speed, wait=True):
        print("[move] width: %.4f, speed %.2f" % (width, speed))

    def close(self, width=0.0, speed=0.03, force=40.0, grasp_eps=(0.2, 0.2), wait=True):
        print("[close] width: %.4f, speed: %.2f, force: %.2f" % (width, speed, force))


class FrankaGripperCommandRelay(object):
    def __init__(self, is_sim=False):
        print("Setting up gripper commander")
        self.is_sim = is_sim
        if self.is_sim:
            print("<is sim>")
            self.gripper_commander = SimGripperCommander()
        else:
            print("<is real>")
            self.gripper_commander = FrankaGripperCommander(verbose=True)

        self.start_time = rospy.Time.now()
        self.last_tick_time = self.start_time
        self.seconds_between_tick_prints = 0.1

        self.command_queue = []
        self.command_queue_lock = threading.Lock()

        print("Starting subscriber...")
        self.command_sub = rospy.Subscriber("/cortex/gripper/command", String, self.command_callback)
        print("<ready and listening>")

    def command_callback(self, msg):
        try:
            command = json.loads(msg.data)
            try:
                self.command_queue_lock.acquire()
                self.command_queue.append(command)
            finally:
                self.command_queue_lock.release()
        except ValueError as ve:
            print("Jsg parse error -- could not parse command:\n", msg.data)
        except Exception as e:
            print("Exception in processing command:", e)
            print("message data:\n", msg.data)

    def process_latest_commands(self):
        now = rospy.Time.now()
        if (now - self.last_tick_time).to_sec() >= self.seconds_between_tick_prints:
            self.last_tick_time = now

        try:
            self.command_queue_lock.acquire()
            command_queue = self.command_queue
            self.command_queue = []
        finally:
            self.command_queue_lock.release()

        for command in command_queue:
            self.process_latest_command(command)

    def process_latest_command(self, cmd):
        try:
            print("\nprocessing command:", cmd["command"])
            if cmd["command"] == "move_to":
                print("moving to:", cmd["width"])
                self.gripper_commander.move(cmd["width"], speed=speed, wait=True)
            elif cmd["command"] == "close_to_grasp":
                print("closing to grasp")
                self.gripper_commander.close(speed=speed)
            else:
                print("WARNING -- unrecognized gripper command:", cmd["command"])

        except Exception as e:
            print("ERROR processing command:\n", cmd)
            print("exception:", e)

    def run(self):
        rate = rospy.Rate(60.0)
        while not rospy.is_shutdown():
            self.process_latest_commands()
            rate.sleep()


if __name__ == "__main__":
    node_name = "franka_gripper_commander_relay"
    rospy.init_node(node_name)

    parser = argparse.ArgumentParser(node_name)
    parser.add_argument("--is_sim", action="store_true", help="Set to start in simulated env.")
    parser.add_argument("--open", action="store_true", help="Open the gripper then exit.")
    parser.add_argument("--close", action="store_true", help="Close the gripper then exit.")
    parser.add_argument("--close_pinch", action="store_true", help="Close the gripper then exit.")

    args = parser.parse_args()

    if args.open:
        gripper_commander = FrankaGripperCommander(verbose=True)
        gripper_commander.open(speed=speed)
    elif args.close:
        gripper_commander = FrankaGripperCommander(verbose=True)
        gripper_commander.close(speed=speed)
    elif args.close_pinch:
        gripper_commander = FrankaGripperCommander(verbose=True)
        gripper_commander.move(pinch_width, speed=speed, wait=True)
    else:
        listener = FrankaGripperCommandRelay(args.is_sim)
        listener.run()
