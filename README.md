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


## Project RoadMap:
1. PPO balancing algorithm :white_check_mark: 
2. Keyboard real time control :white_check_mark: 
3. Camera stream to ROS2 :white_check_mark: 
4. Lidar stream to ROS2
5. IMU data stream to ROS2
6. URDF real-time joints sync

## Your feedback and support mean the world to us. 

If you're as enthusiastic about this project as we are, please consider giving it a :star: star on our GitHub repository. 

Your encouragement fuels our passion and helps us develop our RoadMap further. We welcome any help or suggestions you can offer!

Together, let's push the boundaries of what's possible with the Unitree Go2 and ROS2!


## System requirements
You need to install Ubuntu 20.04 with Nvidia Isaac Sim and Nvidia Orbit.
The full instruction:
```
https://isaac-orbit.github.io/orbit/source/setup/installation.html
```

Also, you need to install ROS2 on your system and configure it:

```
https://docs.omniverse.nvidia.com/isaacsim/latest/installation/install_ros.html#isaac-sim-app-install-ros
```

## Usage

Go inside the repo folder, then

```
conda activate orbit
python main.py
```

## Development

To contribute or modify the project, refer to these resources for implementing additional features or improving the existing codebase. PRs are welcome!

## License

This project is licensed under the BSD 2-clause License - see the [LICENSE](https://github.com/abizovnuralem/go2_omniverse/blob/master/LICENSE) file for details.
