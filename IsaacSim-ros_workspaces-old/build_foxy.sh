#!/bin/bash

set -e
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
# MY_DIR="$(realpath -s "$SCRIPT_DIR"/build_ws/foxy)"

docker build . --network=host -f ubuntu_20_foxy_minimal.dockerfile -t isaac_sim_ros:ubuntu_20_foxy

rm -rf build_ws/foxy
mkdir -p build_ws/foxy

pushd build_ws/foxy

docker cp $(docker create --rm isaac_sim_ros:ubuntu_20_foxy):/workspace/foxy_ws foxy_ws

docker cp $(docker create --rm isaac_sim_ros:ubuntu_20_foxy):/workspace/build_ws isaac_sim_ros_ws


popd