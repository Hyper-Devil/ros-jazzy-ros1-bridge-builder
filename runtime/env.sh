#!/usr/bin/env bash
# source runtime/env.sh noetic|jazzy|bridge [gpu]
runtime_repo="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
runtime_root="${PROJECTS_ROOT:-$(dirname "$runtime_repo")}"
runtime_build="$runtime_repo/.runtime"
case "${1:-}" in
  noetic)
    source /opt/ros/noetic/setup.bash
    export ROS_PACKAGE_PATH="$runtime_root/uam2te/scripts/ros_workspace/src:${ROS_PACKAGE_PATH:-}"
    # Source files come from the host, while builds remain environment-specific.
    if [[ -f "$runtime_build/noetic/devel/setup.bash" ]]; then
      source "$runtime_build/noetic/devel/setup.bash"
    fi
    export ROS_PACKAGE_PATH="$runtime_repo/runtime:${ROS_PACKAGE_PATH:-}"
    export PYTHONPATH="$runtime_root/emcp/src/elevation_mapping_cupy/elevation_mapping_cupy/script:${PYTHONPATH:-}"
    if [[ "${2:-}" == gpu ]]; then
      export PATH="/opt/emcupy-py310/bin:$PATH"
      export LD_LIBRARY_PATH="/opt/emcupy-py310/lib:/opt/emcupy-py310/targets/x86_64-linux/lib:${LD_LIBRARY_PATH:-}"
      export CUDA_PATH=/opt/emcupy-py310
    fi
    ;;
  jazzy)
    source /opt/ros/jazzy/setup.bash
    [[ ! -f "$runtime_build/messages-ros2/install/local_setup.bash" ]] || source "$runtime_build/messages-ros2/install/local_setup.bash"
    if [[ "${2:-}" == gpu ]]; then export PATH="/opt/gpu/bin:/usr/local/cuda/bin:$PATH"; fi
    ;;
  bridge)
    unset ROS_DISTRO
    source /opt/ros/noetic/setup.bash
    source /opt/ros/jazzy/setup.bash
    source "$runtime_build/messages-ros1/install/local_setup.bash"
    source "$runtime_build/messages-ros2/install/local_setup.bash"
    source "$runtime_build/bridge/install/local_setup.bash"
    ;;
  *) echo 'source env.sh noetic|jazzy|bridge [gpu]' >&2; return 2 ;;
esac
export ROS_MASTER_URI="${ROS_MASTER_URI:-http://127.0.0.1:11671}"
export ROS_IP=127.0.0.1 ROS_HOSTNAME=localhost
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-71}"
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export FASTDDS_DEFAULT_PROFILES_FILE="${FASTDDS_DEFAULT_PROFILES_FILE:-$runtime_repo/runtime/fastdds.xml}"
