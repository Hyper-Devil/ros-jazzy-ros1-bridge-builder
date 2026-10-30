#!/usr/bin/env bash
# Pin the bridge source used by the upstream builder without overwriting edits.
set -euo pipefail
repo="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
revision=8f1b7e903e1fb4462218db659bb4981228c12206
if [[ ! -e "$repo/ros1_bridge" ]]; then
  git clone --branch action_bridge_humble https://github.com/smith-doug/ros1_bridge.git "$repo/ros1_bridge"
  git -C "$repo/ros1_bridge" checkout --detach "$revision"
fi
if [[ "$(git -C "$repo/ros1_bridge" rev-parse HEAD)" != "$revision" ]]; then
  echo "ros1_bridge differs from tested revision $revision; inspect it before building." >&2
  exit 1
fi
