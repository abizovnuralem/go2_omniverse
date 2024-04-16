from __future__ import absolute_import
import numpy as np
import yaml
import os
import math
from PIL import Image


class GridMap:
    def __init__(self, yaml_file_path):
        self.__get_meta_from_yaml(yaml_file_path)
        self.__get_raw_map()
        self.__add_max_range_to_meta()

    def __get_meta_from_yaml(self, yaml_file_path):
        """
        Reads map meta from the yaml file.
        
        Parameters
        ----------
        yaml_file_path: path of the yaml file.
        """
        with open(yaml_file_path, "r") as f:
            file_content = f.read()
        self.__map_meta = yaml.safe_load(file_content)
        self.__map_meta["image"] = os.path.join(os.path.dirname(yaml_file_path), self.__map_meta["image"])

    def __get_raw_map(self):
        """
        Reads the map image and generates the grid map.\n
        Grid map is a 2D boolean matrix where True=>occupied space & False=>Free space.
        """
        img = Image.open(self.__map_meta.get("image"))
        img = np.array(img)

        # Anything greater than free_thresh is considered as occupied
        if self.__map_meta["negate"]:
            res = np.where((img / 255)[:, :, 0] > self.__map_meta["free_thresh"])
        else:
            res = np.where(((255 - img) / 255)[:, :, 0] > self.__map_meta["free_thresh"])

        self.__grid_map = np.zeros(shape=(img.shape[:2]), dtype=bool)

        for i in range(res[0].shape[0]):
            self.__grid_map[res[0][i], res[1][i]] = 1

    def __add_max_range_to_meta(self):
        """
        Calculates and adds the max value of pose in x & y direction to the meta.
        """
        max_x = self.__grid_map.shape[1] * self.__map_meta["resolution"] + self.__map_meta["origin"][0]
        max_y = self.__grid_map.shape[0] * self.__map_meta["resolution"] + self.__map_meta["origin"][1]
        self.__map_meta["max_x"] = round(max_x, 2)
        self.__map_meta["max_y"] = round(max_y, 2)

    def __pad_obstacles(self, distance):
        pass

    def get_range(self):
        """
        Returns the bounds of pose values in x & y direction.\n
        
        Returns
        -------
        [List]:\n
        Where   list[0][0]: min value in x direction
                list[0][1]: max value in x direction
                list[1][0]: min value in y direction
                list[1][1]: max value in y direction
        """
        return [
            [self.__map_meta["origin"][0], self.__map_meta["max_x"]],
            [self.__map_meta["origin"][1], self.__map_meta["max_y"]],
        ]

    def __transform_to_image_coordinates(self, point):
        """
        Transforms a pose in meters to image pixel coordinates.

        Parameters
        ----------
        Point: A point as list. where list[0]=>pose.x and list[1]=pose.y

        Returns
        -------
        [Tuple]: tuple[0]=>pixel value in x direction. i.e column index.
                tuple[1]=> pixel vlaue in y direction. i.e row index.
        """
        p_x, p_y = point
        i_x = math.floor((p_x - self.__map_meta["origin"][0]) / self.__map_meta["resolution"])
        i_y = math.floor((p_y - self.__map_meta["origin"][1]) / self.__map_meta["resolution"])
        # because origin in yaml is at bottom left of image
        i_y = self.__grid_map.shape[0] - i_y
        return i_x, i_y

    def __transform_distance_to_pixels(self, distance):
        """
        Converts the distance in meters to number of pixels based on the resolution.

        Parameters
        ----------
        distance: value in meters

        Returns
        -------
        [Integer]: number of pixel which represent the same distance.
        """
        return math.ceil(distance / self.__map_meta["resolution"])

    def __is_obstacle_in_distance(self, img_point, distance):
        """
        Checks if any obstacle is in vicinity of the given image point.

        Parameters
        ----------
        img_point: pixel values of the point
        distance: distnace in pixels in which there shouldn't be any obstacle.

        Returns
        -------
        [Bool]: True if any obstacle found else False.
        """
        # need to make sure that patch xmin & ymin are >=0,
        # because of python's negative indexing capability
        row_start_idx = 0 if img_point[1] - distance < 0 else img_point[1] - distance
        col_start_idx = 0 if img_point[0] - distance < 0 else img_point[0] - distance

        # image point acts as the center of the square, where each side of square is of size
        # 2xdistance. using int() because in python2.x math.floor() returns float.
        patch = self.__grid_map[
            int(row_start_idx) : int(img_point[1] + distance), int(col_start_idx) : int(img_point[0] + distance)
        ]
        obstacles = np.where(patch == True)
        return len(obstacles[0]) > 0

    def is_valid_pose(self, point, distance=0.2):
        """
        Checks if a given pose is "distance" away from a obstacle.

        Parameters
        ----------
        point: pose in 2D space. where point[0]=pose.x and point[1]=pose.y
        distance: distance in meters.

        Returns
        -------
        [Bool]: True if pose is valid else False
        """
        assert len(point) == 2
        img_point = self.__transform_to_image_coordinates(point)
        img_pixel_distance = self.__transform_distance_to_pixels(distance)
        return not self.__is_obstacle_in_distance(img_point, img_pixel_distance)
