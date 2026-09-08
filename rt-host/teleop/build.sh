#!/bin/bash
# teleop/build.sh - build every teleop binary inside the PART A image.
# Thin wrapper: all compiler flags live in docker/build-inside.sh (runs as the calling user).
#   FRANKA_RUN=/path/to/franka-run ./build.sh   # override the wrapper location if needed
set -euo pipefail
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT="$(cd "$HERE/.." && pwd)"
FRANKA_RUN="${FRANKA_RUN:-$ROOT/bin/franka-run}"
if [ ! -x "$FRANKA_RUN" ]; then
  echo "build.sh: franka-run not found at $FRANKA_RUN (set FRANKA_RUN=...)" >&2
  exit 1
fi
exec env FRANKA_RUN_AS_USER=1 "$FRANKA_RUN" /franka/docker/build-inside.sh "$@"
