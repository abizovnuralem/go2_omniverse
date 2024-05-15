# Change directory to IsaacSim-ros_workspaces
cd IsaacSim-ros_workspaces

# Source the necessary ROS setup scripts
source build_ws/${ROS_DISTRO}/${ROS_DISTRO}_ws/install/setup.bash
source build_ws/${ROS_DISTRO}/isaac_sim_ros_ws/install/local_setup.bash

# Changing directory back to the original location
cd ..

eval "$(conda shell.bash hook)"
conda activate orbit

# Run the Python script
python main.py