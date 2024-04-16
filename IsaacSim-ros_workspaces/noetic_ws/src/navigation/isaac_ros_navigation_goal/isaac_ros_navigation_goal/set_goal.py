#!/usr/bin/env python

from __future__ import absolute_import
import rospy
import actionlib
import sys
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from obstacle_map import GridMap
from goal_generators import RandomGoalGenerator, GoalReader
from geometry_msgs.msg import PoseWithCovarianceStamped


class SetNavigationGoal:
    def __init__(self):
        self.__goal_generator = self.__create_goal_generator()
        action_server_name = rospy.get_param("action_server_name", "move_base")
        self._action_client = actionlib.SimpleActionClient(action_server_name, MoveBaseAction)
        self.MAX_ITERATION_COUNT = rospy.get_param("iteration_count", 1)
        assert self.MAX_ITERATION_COUNT > 0
        self.curr_iteration_count = 1
        self.__initial_goal_publisher = rospy.Publisher("initialpose", PoseWithCovarianceStamped, queue_size=1)
        self.__initial_pose = rospy.get_param("initial_pose", None)
        self.__is_initial_pose_sent = True if self.__initial_pose is None else False

    def __send_initial_pose(self):
        """
        Publishes the initial pose.
        This function is only called once that too before sending any goal pose
        to the mission server.
        """
        goal = PoseWithCovarianceStamped()
        goal.header.frame_id = rospy.get_param("frame_id", "map")
        goal.header.stamp = rospy.get_rostime()
        goal.pose.pose.position.x = self.__initial_pose[0]
        goal.pose.pose.position.y = self.__initial_pose[1]
        goal.pose.pose.position.z = self.__initial_pose[2]
        goal.pose.pose.orientation.x = self.__initial_pose[3]
        goal.pose.pose.orientation.y = self.__initial_pose[4]
        goal.pose.pose.orientation.z = self.__initial_pose[5]
        goal.pose.pose.orientation.w = self.__initial_pose[6]
        rospy.sleep(1)
        self.__initial_goal_publisher.publish(goal)

    def send_goal(self):
        """
        Sends the goal to the action server.
        """
        if not self.__is_initial_pose_sent:
            rospy.loginfo("Sending initial pose")
            self.__send_initial_pose()
            self.__is_initial_pose_sent = True

            # Assumption is that initial pose is set after publishing first time in this duration.
            # Can be changed to more sophisticated way. e.g. /particlecloud topic has no msg until
            # the initial pose is set.
            rospy.sleep(10)
            rospy.loginfo("Sending first goal")

        self._action_client.wait_for_server()
        goal_msg = self.__get_goal()

        if goal_msg is None:
            rospy.signal_shutdown("Goal message not generated.")
            sys.exit(1)

        self._action_client.send_goal(goal_msg, feedback_cb=self.__goal_response_callback)

    def __goal_response_callback(self, feedback):
        """
        Callback function to check the response(goal accpted/rejected) from the server.\n
        If the Goal is rejected it stops the execution for now.(We can change to resample the pose if rejected.)
        """

        if self.verify_goal_state():
            rospy.loginfo("Waiting to reach goal")
            wait = self._action_client.wait_for_result()

            if self.verify_goal_state():
                self.__get_result_callback(True)
           
    def verify_goal_state(self):
        print("Action Client State:", self._action_client.get_state(), self._action_client.get_goal_status_text())
        if self._action_client.get_state() not in [0, 1, 3]:
            rospy.signal_shutdown("Goal Rejected :(")
            return False

        return True

    def __get_goal(self):
        goal_msg = MoveBaseGoal()
        goal_msg.target_pose.header.frame_id = rospy.get_param("frame_id", "map")
        goal_msg.target_pose.header.stamp = rospy.get_rostime()
        pose = self.__goal_generator.generate_goal()

        # couldn't sample a pose which is not close to obstacles. Rare but might happen in dense maps.
        if pose is None:
            rospy.logerr("Could not generate next goal. Returning. Possible reasons for this error could be:")
            rospy.logerr(
                "1. If you are using GoalReader then please make sure iteration count <= no of goals avaiable in file."
            )
            rospy.logerr(
                "2. If RandomGoalGenerator is being used then it was not able to sample a pose which is given distance away from the obstacles."
            )
            return

        rospy.loginfo("Generated goal pose: {0}".format(pose))
        goal_msg.target_pose.pose.position.x = pose[0]
        goal_msg.target_pose.pose.position.y = pose[1]
        goal_msg.target_pose.pose.orientation.x = pose[2]
        goal_msg.target_pose.pose.orientation.y = pose[3]
        goal_msg.target_pose.pose.orientation.z = pose[4]
        goal_msg.target_pose.pose.orientation.w = pose[5]
        return goal_msg

    def __get_result_callback(self, wait):

        if wait and self.curr_iteration_count < self.MAX_ITERATION_COUNT:
            self.curr_iteration_count += 1
            self.send_goal()
        else:
            rospy.signal_shutdown("Iteration done or Goal not reached.")

    # in this callback func we can compare/compute/log something while the robot is on its way to goal.
    def __feedback_callback(self, feedback_msg):
        pass

    def __create_goal_generator(self):
        goal_generator_type = rospy.get_param("goal_generator_type", "RandomGoalGenerator")
        goal_generator = None
        if goal_generator_type == "RandomGoalGenerator":
            if rospy.get_param("map_yaml_path", None) is None:
                rospy.loginfo("Yaml file path is not given. Returning..")
                sys.exit(1)

            yaml_file_path = rospy.get_param("map_yaml_path", None)
            grid_map = GridMap(yaml_file_path)
            obstacle_search_distance_in_meters = rospy.get_param("obstacle_search_distance_in_meters", 0.2)
            assert obstacle_search_distance_in_meters > 0
            goal_generator = RandomGoalGenerator(grid_map, obstacle_search_distance_in_meters)

        elif goal_generator_type == "GoalReader":
            if rospy.get_param("goal_text_file_path", None) is None:
                rospy.loginfo("Goal text file path is not given. Returning..")
                sys.exit(1)

            file_path = rospy.get_param("goal_text_file_path", None)
            goal_generator = GoalReader(file_path)
        else:
            rospy.loginfo("Invalid goal generator specified. Returning...")
            sys.exit(1)
        return goal_generator


def main():
    rospy.init_node("set_goal_py")
    set_goal = SetNavigationGoal()
    result = set_goal.send_goal()
    rospy.spin()


if __name__ == "__main__":
    main()
