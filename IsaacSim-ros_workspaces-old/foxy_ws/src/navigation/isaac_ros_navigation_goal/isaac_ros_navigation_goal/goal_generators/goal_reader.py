from .goal_generator import GoalGenerator


class GoalReader(GoalGenerator):
    def __init__(self, file_path):
        self.__file_path = file_path
        self.__generator = self.__get_goal()

    def generate_goal(self, max_num_of_trials=1000):
        try:
            return next(self.__generator)
        except StopIteration:
            return

    def __get_goal(self):
        for row in open(self.__file_path, "r"):
            yield list(map(float, row.strip().split(" ")))
