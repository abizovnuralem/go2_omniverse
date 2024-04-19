# Change directory to IsaacSim-ros_workspaces
cd IsaacSim-ros_workspaces

# Execute the build_foxy.sh script
./build_foxy.sh
if [ $? -ne 0 ]; then
    echo "Building foxy failed, exiting"
    exit 1
fi

# Source the necessary ROS setup scripts
source build_ws/foxy/foxy_ws/install/setup.bash
source build_ws/foxy/isaac_sim_ros_ws/install/local_setup.bash

# Changing directory back to the original location
cd ..

eval "$(conda shell.bash hook)"
conda activate orbit

# Run the Python script
python main.py