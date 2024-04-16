from abc import ABC, abstractmethod


class GoalGenerator(ABC):
    """
    Parent class for the Goal generators
    """

    def __init__(self):
        pass

    @abstractmethod
    def generate_goal(self, max_num_of_trials=2000):
        """
        Generate the goal.

        Parameters
        ----------
        max_num_of_trials: maximum number of pose generations when generated pose keep is not a valid pose.

        Returns
        -------
        [List][Pose]: Pose in format [pose.x,pose.y,orientaion.x,orientaion.y,orientaion.z,orientaion.w]
        """
        pass
