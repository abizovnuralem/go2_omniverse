# Change directory to IsaacSim-ros_workspaces
source /opt/ros/${ROS_DISTRO}/setup.bash
cd IsaacSim-ros_workspaces/${ROS_DISTRO}_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build
source install/setup.bash
cd ../..
cd go2_omniverse_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build
source install/setup.bash
cd ..

eval "$(conda shell.bash hook)"
conda activate orbit
export LD_PRELOAD=/usr/lib/x86_64-linux-gnu/libstdc++.so.6

# Run the Python script
python main.py