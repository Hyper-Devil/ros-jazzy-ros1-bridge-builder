#!/usr/bin/env bash
set -eo pipefail
repo="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
source "$repo/runtime/env.sh" bridge
rosparam load "${BRIDGE_TOPICS:-$repo/runtime/topics.yaml}"
exec ros2 run ros1_bridge parameter_bridge
