#!/usr/bin/env bash
# The host model venv stays authoritative, including its interpreter mount.
set -eo pipefail
repo="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
source "$repo/runtime/env.sh" noetic gpu
cd "${PROJECTS_ROOT:-$(dirname "$repo")}/uam2te"
exec .venv/bin/python scripts/serve_navigation_inference.py "$@"
