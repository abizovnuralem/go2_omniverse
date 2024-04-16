ARG BASE_IMAGE=ubuntu:20.04
FROM ${BASE_IMAGE}

ENV ROS_DISTRO=humble
ENV ROS_ROOT=humble_ws
ENV ROS_PYTHON_VERSION=3

ENV DEBIAN_FRONTEND=noninteractive

WORKDIR /workspace

RUN apt-get update && \
    apt-get install -y --no-install-recommends \
        git \
		cmake \
		build-essential \
		curl \
		wget \
		gnupg2 \
		lsb-release


# Upgrade installed packages
RUN apt update && apt upgrade -y && apt clean

# Install Python3.10
RUN apt update && \
    apt install --no-install-recommends -y build-essential software-properties-common && \
    add-apt-repository -y ppa:deadsnakes/ppa && \
    apt install --no-install-recommends -y python3.10 python3.10-dev python3.10-distutils

# Setting up locale stuff
RUN apt update && apt install locales

RUN locale-gen en_US en_US.UTF-8 && \
    update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8 && \
    export LANG=en_US.UTF-8

# Set default Python3 to Python3.10
RUN update-alternatives --install /usr/bin/python3 python3 /usr/bin/python3.8 1
RUN update-alternatives --install /usr/bin/python3 python3 /usr/bin/python3.10 2

# Pip install stuff
RUN curl -s https://bootstrap.pypa.io/get-pip.py -o get-pip.py && \
    python3.10 get-pip.py --force-reinstall && \
    rm get-pip.py

RUN wget --no-check-certificate https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc && apt-key add ros.asc
RUN sh -c 'echo "deb [arch=$(dpkg --print-architecture)] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" > /etc/apt/sources.list.d/ros2-latest.list'

RUN apt update && apt install -y \
  python3-pip \
  python3-pytest-cov \
  python3-rosinstall-generator \
  ros-dev-tools \
  libbullet-dev \
  libasio-dev \
  libtinyxml2-dev \
  libcunit1-dev \
  libacl1-dev 

RUN python3 -m pip install -U \
  argcomplete \
  flake8-blind-except \
  flake8-builtins \
  flake8-class-newline \
  flake8-comprehensions \
  flake8-deprecated \
  flake8-docstrings \
  flake8-import-order \
  flake8-quotes \
  pytest-repeat \
  pytest-rerunfailures \
  pytest \
  lark

RUN python3.10 -m pip uninstall numpy -y
RUN python3.10 -m pip install --upgrade pip
RUN python3.10 -m pip install numpy

RUN pip3 install setuptools==58.2.

RUN mkdir -p ${ROS_ROOT}/src && \
    cd ${ROS_ROOT} && \
    rosinstall_generator --deps --rosdistro ${ROS_DISTRO} rosidl_runtime_c rcutils rcl rmw tf2_msgs geometry_msgs nav_msgs std_msgs  rosgraph_msgs sensor_msgs vision_msgs rclpy ros2topic ros2pkg ros2doctor ros2run ros2node ros_environment > ros2.${ROS_DISTRO}.${ROS_PKG}.rosinstall && \
    cat ros2.${ROS_DISTRO}.${ROS_PKG}.rosinstall && \
    vcs import src < ros2.${ROS_DISTRO}.${ROS_PKG}.rosinstall


RUN rosdep init && rosdep update

# Build ROS workspace set python3 flag for some reason still default remains python3.8 when colcon building 
RUN cd ${ROS_ROOT} && colcon build --cmake-args ' -DPython3_EXECUTABLE=/usr/bin/python3.10' --merge-install

# Need these to maintain compatibility on non 20.04 systems
RUN cp /usr/lib/x86_64-linux-gnu/libtinyxml2.so* /workspace/humble_ws/install/lib/
RUN cp /usr/lib/x86_64-linux-gnu/libssl.so* /workspace/humble_ws/install/lib/
RUN cp /usr/lib/x86_64-linux-gnu/libcrypto.so* /workspace/humble_ws/install/lib/

# Next, build the additional workspace 
RUN cd ${WORKDIR} && mkdir build_ws
COPY humble_ws build_ws

RUN /bin/bash -c "source ${ROS_ROOT}/install/setup.sh && cd build_ws && colcon build --cmake-args ' -DPython3_EXECUTABLE=/usr/bin/python3.10' --packages-select custom_message --merge-install"