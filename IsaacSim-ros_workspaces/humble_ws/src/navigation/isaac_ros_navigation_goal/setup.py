from setuptools import setup
from glob import glob
import os

package_name = "isaac_ros_navigation_goal"

setup(
    name=package_name,
    version="0.0.1",
    packages=[package_name, package_name + "/goal_generators"],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
        ("share/" + package_name + "/assets", glob("assets/*")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="isaac sim",
    maintainer_email="isaac-sim@todo.todo",
    description="Package to set goals for navigation stack.",
    license="NVIDIA Isaac ROS Software License",
    tests_require=["pytest"],
    entry_points={"console_scripts": ["SetNavigationGoal = isaac_ros_navigation_goal.set_goal:main"]},
)
