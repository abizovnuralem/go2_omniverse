#!/bin/bash

set -e
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"

docker build . --network=host -f ubuntu_20_humble_minimal.dockerfile -t isaac_sim_ros:ubuntu_20_humble

rm -rf build_ws/humble
mkdir -p build_ws/humble

pushd build_ws/humble

docker cp $(docker create --rm isaac_sim_ros:ubuntu_20_humble):/workspace/humble_ws humble_ws

docker cp $(docker create --rm isaac_sim_ros:ubuntu_20_humble):/workspace/build_ws isaac_sim_ros_ws

popd