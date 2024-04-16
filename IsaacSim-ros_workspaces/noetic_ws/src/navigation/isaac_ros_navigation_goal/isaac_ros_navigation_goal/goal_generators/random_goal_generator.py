from __future__ import absolute_import
import numpy as np
from .goal_generator import GoalGenerator


class RandomGoalGenerator(GoalGenerator):
    """
    Random goal generator.

    parameters
    ----------
    grid_map: GridMap Object
    distance: distance in meters to check vicinity for obstacles.
    """

    def __init__(self, grid_map, distance):
        self.__grid_map = grid_map
        self.__distance = distance

    def generate_goal(self, max_num_of_trials=1000):
        """
        Generate the goal.

        Parameters
        ----------
        max_num_of_trials: maximum number of pose generations when generated pose keep is not a valid pose.

        Returns
        -------
        [List][Pose]: Pose in format [pose.x,pose.y,orientaion.x,orientaion.y,orientaion.z,orientaion.w]
        """
        range_ = self.__grid_map.get_range()
        trial_count = 0
        while trial_count < max_num_of_trials:
            x = np.random.uniform(range_[0][0], range_[0][1])
            y = np.random.uniform(range_[1][0], range_[1][1])
            orient_x = 0  # not needed because robot is in x,y plane
            orient_y = 0  # not needed because robot is in x,y plane
            orient_z = np.random.uniform(0, 1)
            orient_w = np.random.uniform(0, 1)
            if self.__grid_map.is_valid_pose([x, y], self.__distance):
                goal = [x, y, orient_x, orient_y, orient_z, orient_w]
                return goal
            trial_count += 1
