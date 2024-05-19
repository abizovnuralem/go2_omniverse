# Welcome to the Unitree Go2 Omniverse Project!

I am thrilled to announce that the Unitree Go2 robot has now been integrated with the Nvidia Isaac Sim (Orbit), marking a major step forward in robotics research and development. The combination of these two cutting-edge technologies opens up a world of possibilities for creating and testing algorithms in a variety of simulated environments.

Get ready to take your research to the next level with this powerful new resource at your fingertips!


Real time Go2 Balancing:

<p align="center">
<img width="1280" height="600" src="https://github.com/abizovnuralem/go2_omniverse/assets/33475993/60c2233a-7586-49b6-a134-a7bddc4dd9ae" alt='Go2'>
</p>


Go2 Ros2 Camera stream:

<p align="center">
<img width="1200" height="440" src="https://github.com/abizovnuralem/go2_omniverse/assets/33475993/c740147b-ce00-4d7c-94de-0140be135e3e" alt='Go2'>
</p>


URDF real-time joints sync:

<p align="center">
<img width="1200" height="440" src="https://github.com/abizovnuralem/go2_omniverse/assets/33475993/a8060b6e-e9b7-4d30-89f2-8a50b7510a2b" alt='Go2'>
</p>

Foot force data stream:

<p align="center">
<img width="1200" height="440" src="https://github.com/abizovnuralem/go2_omniverse/assets/33475993/95a34b03-471e-496a-88cc-38e7c4e1906d" alt='Go2'>
</p>


Real-time RTX lidar stream:

<p align="center">
<img width="1200" height="440" src="https://github.com/abizovnuralem/go2_omniverse/assets/33475993/3f078bf2-e4b6-45ca-8807-36537a4125b5" alt='Go2'>
</p>


Custom envs (Office):

<p align="center">
<img width="1200" height="440" src="https://github.com/abizovnuralem/go2_omniverse/assets/33475993/e2e9bdd0-1f40-41a8-86bc-c1097ab3fd7b" alt='Go2'>
</p>


Custom envs (Warehouse):

<p align="center">
<img width="1200" height="440" src="https://github.com/abizovnuralem/go2_omniverse/assets/33475993/5db6f331-60be-40bd-9b4b-ead44064ee44" alt='Go2'>
</p>


## Project RoadMap:
1. PPO balancing algorithm :white_check_mark: 
2. Keyboard real time control :white_check_mark: 
3. Camera stream to ROS2 :white_check_mark: 
4. RTX Lidar stream to ROS2 :white_check_mark:
5. IMU data stream to ROS2 :white_check_mark: 
6. URDF real-time joints sync :white_check_mark:
7. Foot force data stream :white_check_mark:
8. Real-time control from ROS2
9. Nav2 with Slam_toolbox
10. Bunch of RL-envs for custom dog training :white_check_mark:

## Your feedback and support mean the world to us. 

If you're as enthusiastic about this project as we are, please consider giving it a :star: star on our GitHub repository. 

Your encouragement fuels our passion and helps us develop our RoadMap further. We welcome any help or suggestions you can offer!

Together, let's push the boundaries of what's possible with the Unitree Go2 and ROS2!


## System requirements
You need to install:
1. Ubuntu 22.04
2. Nvidia Isaac Sim 2023.1.1
3. Nvidia Orbit 0.3.0

Full instruction:
```
https://isaac-orbit.github.io/orbit/source/setup/installation.html
```

Some suggestions from me:
1. You need to check nvidia-smi, it should work, before install Isaac Sim
2. You need to install Miniconda and execute: conda config --set auto_activate_base false
3. Install Omniverse launcher and then install Isaac Sim.
4. Create conda env then activate it, also execute ./orbit.sh --install and ./orbit.sh --extra and ./orbit.sh --extra rsl_rl

Also, you need to install ROS2 on your system and configure it:

```
https://docs.omniverse.nvidia.com/isaacsim/latest/installation/install_ros.html#isaac-sim-app-install-ros
```

## Setup the Unitree L1 Lidar:

First, you need to copy files from Isaac Sim folder to your local Isaac Sim installation in order to use Unitree L1 lidar inside Orbit.

```
1. You need to replace original file that located in ~/.local/share/ov/pkg/isaac_sim-2023.1.1/exts/omni.isaac.sensor/config/extrensiom.toml 
with Isaac_sim/extension.toml
2. You need to add Unitree folder that is inside Isaac_sim folder to  ~/.local/share/ov/pkg/isaac_sim-2023.1.1/exts/omni.isaac.sensor/data/lidar_configs/
```

## Usage
The current project was tested on Ubuntu 20.04 (foxy ros2 bridge), IsaacSim 2023.1.1 with Orbit 0.2.0 and Nvidia Driver Version: 535.171.04 with CUDA Version: 12.2.
Go inside the repo folder, first, you need to build ros2 env for your Nvidia orbit:


```
1. go inside IsaacSim-ros_workspaces folder
2. execute ./build_foxy.sh or ./build_humble.sh (not tested)
3. source build_ws/foxy/foxy_ws/install/setup.bash or humble (not tested)
4. source build_ws/foxy/isaac_sim_ros_ws/install/local_setup.bash or humble (not tested))
5. go back to the root folder
6. conda activate orbit
7. python main.py
```

You can control the dog using "WASD" keyboard commands

## ROS2 SDK

You can use https://github.com/abizovnuralem/go2_ros2_sdk or https://github.com/unitreerobotics/unitree_ros2 as a basement for your ROS2 setup.


## Select custom env

To use predifined custom envs, you need to download files from https://drive.google.com/drive/folders/1vVGuO1KIX1K6mD6mBHDZGm9nk2vaRyj3?usp=sharing and place them to /envs folder.
Then you can execute it via python main.py --custom_env=office or python main.py --custom_env=warehouse commands

## Easy start

For you convenience I wrote two launche files: run_orbit.sh and build_and_run_orbit.sh

## Development

To contribute or modify the project, refer to these resources for implementing additional features or improving the existing codebase. PRs are welcome!

## Thanks
Special thanks to Leul Tesfaye for his expertise in Orbit lidars

## License

This project is licensed under the BSD 2-clause License - see the [LICENSE](https://github.com/abizovnuralem/go2_omniverse/blob/master/LICENSE) file for details.
