#!/usr/bin/env bash
set -euo pipefail
repo="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
mode="${1:?Usage: enter.sh noetic|jazzy|bridge}"
case "$mode" in
  noetic) container=ros1-noetic; gpu=gpu ;;
  jazzy) container=ros2-jazzy; gpu=gpu ;;
  bridge) container=ros1-ros2-bridge; gpu= ;;
  *) exit 2 ;;
esac
exec docker exec -it -w "$(dirname "$repo")" "$container" bash -lc \
  "source '$repo/runtime/env.sh' '$mode' '$gpu'; exec bash --noprofile --norc -i"
