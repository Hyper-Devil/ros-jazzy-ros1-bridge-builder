#!/usr/bin/env bash
set -eo pipefail
repo="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
bash "$repo/runtime/prepare_bridge_source.sh"
jobs="${BRIDGE_JOBS:-4}"
export MAKEFLAGS="-j$jobs" CMAKE_BUILD_PARALLEL_LEVEL="$jobs"
build="$repo/.runtime"
mkdir -p "$build"
(
  unset ROS_DISTRO
  source /opt/ros/noetic/setup.bash
  colcon --log-base "$build/messages-ros1/log" build \
    --base-paths "$repo/custom_msgs/custom_msgs_ros1_ws/src" \
    --build-base "$build/messages-ros1/build" --install-base "$build/messages-ros1/install" \
    --parallel-workers 2 --cmake-args -DCMAKE_BUILD_TYPE=Release
)
(
  unset ROS_DISTRO
  source /opt/ros/jazzy/setup.bash
  colcon --log-base "$build/messages-ros2/log" build \
    --base-paths "$repo/custom_msgs/custom_msgs_ros2_ws/src" \
    --build-base "$build/messages-ros2/build" --install-base "$build/messages-ros2/install" \
    --parallel-workers 2 --cmake-args -DCMAKE_BUILD_TYPE=Release
)
(
  source /opt/ros/noetic/setup.bash
  source /opt/ros/jazzy/setup.bash
  source "$build/messages-ros1/install/local_setup.bash"
  source "$build/messages-ros2/install/local_setup.bash"
  ROS_DISTRO=humble colcon --log-base "$build/bridge/log" build \
    --base-paths "$repo/ros1_bridge" --build-base "$build/bridge/build" \
    --install-base "$build/bridge/install" --parallel-workers 1 \
    --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF
)
