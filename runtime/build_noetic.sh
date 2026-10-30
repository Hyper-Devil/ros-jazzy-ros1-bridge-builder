#!/usr/bin/env bash
# Build canonical host sources; symlinks are workspace membership, not copies.
set -eo pipefail
repo="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
root="${PROJECTS_ROOT:-$(dirname "$repo")}"
workspace="$repo/.runtime/noetic"
source "$repo/runtime/env.sh" noetic gpu
mkdir -p "$workspace/src"
ln -sfn "$root/stepp_adapter/third_party/FAST_LIO_SAM/FAST_LIO_SAM" "$workspace/src/fast_lio_sam"
ln -sfn "$root/emcp/src/elevation_mapping_cupy/elevation_map_msgs" "$workspace/src/elevation_map_msgs"
ln -sfn "$root/emcp/src/elevation_mapping_cupy/elevation_mapping_cupy" "$workspace/src/elevation_mapping_cupy"
ln -sfn "$root/uam2te/scripts/ros_workspace/src/ros_interface" "$workspace/src/uam2te_msgs"
ln -sfn "$root/traversability_dataset_builder" "$workspace/src/traversability_dataset_builder"
ln -sfn "$root/calibrator" "$workspace/src/calibrator"
# Livox messages are a compile-time dependency even when playing PointCloud2.
ln -sfn "$root/rellis_builder_runs/20261027/ws/src/livox_ros_driver2" "$workspace/src/livox_ros_driver2"
catkin config --workspace "$workspace" --extend /opt/ros/noetic --no-install \
  --cmake-args -DCMAKE_BUILD_TYPE=Release \
  -DPYTHON_EXECUTABLE=/usr/bin/python3
catkin build --workspace "$workspace" livox_ros_driver2 elevation_map_msgs -j4 -p2 --no-status
catkin build --workspace "$workspace" fast_lio_sam uam2te_msgs traversability_dataset_builder calibrator \
  --no-deps -j4 -p2 --no-status
# Only EMCP's embedded interpreter uses 3.10; ROS cv_bridge Python nodes keep
# Noetic's system 3.8 ABI. Do not change the workspace-wide Python selection.
catkin build --workspace "$workspace" elevation_mapping_cupy --no-deps -j4 -p1 --no-status \
  --cmake-args -DPYTHON_EXECUTABLE=/opt/emcupy-py310/bin/python \
  -DPYTHON_INCLUDE_DIR=/opt/emcupy-py310/include/python3.10 \
  -DPYTHON_LIBRARY=/opt/emcupy-py310/lib/libpython3.10.so
