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


VR support:

<p align="center">
<img width="1200" height="440" src="https://github.com/abizovnuralem/go2_omniverse/assets/33475993/d5b82fac-d945-4462-8b3d-4026456847f4" alt='Go2'>
</p>


## Project RoadMap:
1. PPO balancing algorithm :white_check_mark: 
2. Keyboard real time control :white_check_mark: 
3. Camera stream to ROS2 :white_check_mark: 
4. RTX Lidar stream to ROS2 :white_check_mark:
5. IMU data stream to ROS2 :white_check_mark: 
6. URDF real-time joints sync :white_check_mark:
7. Foot force data stream :white_check_mark:
8. Real-time control from ROS2 :white_check_mark:
9. Nav2 with Slam_toolbox :white_check_mark:
10. Bunch of RL-envs for custom dog training :white_check_mark:
11. Custom numbers of robots

## Your feedback and support mean the world to us. 

If you're as enthusiastic about this project as we are, please consider giving it a :star: star on our GitHub repository. 

Your encouragement fuels our passion and helps us develop our RoadMap further. We welcome any help or suggestions you can offer!

Together, let's push the boundaries of what's possible with the Unitree Go2 and ROS2!


## System requirements
You need to install:
1. Ubuntu 22.04
2. Nvidia Isaac Sim 2023.1.1
3. Nvidia Orbit 0.3.0
4. Ros2 Humble


Full instruction:
```
https://isaac-orbit.github.io/orbit/source/setup/installation.html
```

Some suggestions from me:
1. You need to check nvidia-smi, it should work, before installing Isaac Sim
2. You need to install Miniconda and execute: conda config --set auto_activate_base false
3. Install Omniverse launcher and then install Isaac Sim.
4. Create conda env then activate it, also execute ./orbit.sh --install and ./orbit.sh --extra and ./orbit.sh --extra rsl_rl

Also, you need to install ROS2 on your system and configure it:

```
https://docs.omniverse.nvidia.com/isaacsim/latest/installation/install_ros.html#isaac-sim-app-install-ros
```

## Downloading the code

To start with the local development environment, clone this repo:

```
git clone https://github.com/abizovnuralem/go2_omniverse/ --recurse-submodules -j8 --depth=1

```

## Setup the Unitree L1 Lidar:

First, you need to copy files from Isaac Sim folder to your local Isaac Sim installation in order to use Unitree L1 lidar inside Orbit.

```
1. You need to replace original file that located in ~/.local/share/ov/pkg/isaac_sim-2023.1.1/exts/omni.isaac.sensor/config/extrensiom.toml 
with Isaac_sim/extension.toml
2. You need to add Unitree folder that is inside Isaac_sim folder to  ~/.local/share/ov/pkg/isaac_sim-2023.1.1/exts/omni.isaac.sensor/data/lidar_configs/
3. You need to go inside Orbit project folder and then mkdir -p /source/exts/omni.isaac.sensor/data/lidar_configs/Unitree/
4. Put Unitree_L1.json inside orbit/source/exts/omni.isaac.sensor/data/lidar_configs/Unitree/
```

## Usage
The current project was tested on Ubuntu 22.04, IsaacSim 2023.1.1 with Orbit 0.3.0 and Nvidia Driver Version: 545.
To start the project, execute:

```
./run_sim.sh
```

You can control the dog using "WASD" keyboard commands

## ROS2 SDK

You can use https://github.com/abizovnuralem/go2_ros2_sdk or https://github.com/unitreerobotics/unitree_ros2 as a basement for your ROS2 setup.


## Select custom env

To use predifined custom envs, you need to download files from https://drive.google.com/drive/folders/1vVGuO1KIX1K6mD6mBHDZGm9nk2vaRyj3?usp=sharing and place them to /envs folder.
Then you can execute it via python main.py --custom_env=office or python main.py --custom_env=warehouse commands (The whole cmd you can read from run_sim script). If you are doing it first time, it will take 2-3 minutes to configure the env. Please, wait.


## Development

To contribute or modify the project, refer to these resources for implementing additional features or improving the existing codebase. PRs are welcome!

## VR support

To enable VR support on linux will take some time, but it works!
I have tested it on:
1. Ubuntu 22.04
2. Nvidia drivers are 545.29.06 
3. SteamVR 2.4.4 (IMPORTANT! It should be 2.4.4) and you need to go to Compatibility tab (Inside Steam app) and "Force the use of a specific Steam Play compatibility tool" and switch to "Steam-Play-None", additional info you can find in ALVR github issues tab.
4. ALVR streamer 20.8.1 + Oculus Quest 2 (client ALVR you can install via SideQuest app) (How to install it: https://github.com/alvr-org/ALVR)
5. Execute IsaacSim, Go to Window -> Extensions, find STEAMVR INPUT/OUTPUT then enable it and enable AutoLoad. Reopen IsaacSim. Use OpenXR mode.
6. Enjoy Omniverse in VR mode!

## Thanks
Special thanks to Leul Tesfaye for his expertise in Orbit lidars and Tamas @tfoldi for his contribution to this project.

## VR support

To enable VR support on linux will take some time, but it works!
I have tested it on:
1. Ubuntu 22.04
2. Nvidia drivers are 545.29.06 
3. SteamVR 2.4.4 (IMPORTANT! It should be 2.4.4) and you need to go to Compatibility tab (Inside Steam app) and "Force the use of a specific Steam Play compatibility tool" and switch to "Steam-Play-None", additional info you can find in ALVR github issues tab.
4. ALVR streamer 20.8.1 + Oculus Quest 2 (client ALVR you can install via SideQuest app) (How to install it: https://github.com/alvr-org/ALVR)
5. Execute IsaacSim, Go to Window -> Extensions, find STEAMVR INPUT/OUTPUT then enable it and enable AutoLoad. Reopen IsaacSim. Use OpenXR mode.
6. Enjoy Omniverse in VR mode!

## License

This project is licensed under the BSD 2-clause License - see the [LICENSE](https://github.com/abizovnuralem/go2_omniverse/blob/master/LICENSE) file for details.
