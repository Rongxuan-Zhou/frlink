#!/bin/bash
# build-image.sh — build the franka-rt image and record what was built.
set -euo pipefail
cd "$(dirname "$0")"
IMAGE="${FRANKA_IMAGE:-franka-rt:0.17.0-jazzy}"
docker build --pull -t "$IMAGE" -f Dockerfile .
{
  echo "image=$IMAGE built=$(date -Is) host=$(hostname)"
  echo "base_digest=$(docker image inspect --format '{{index .RepoDigests 0}}' ubuntu:24.04)"
  echo "image_id=$(docker image inspect --format '{{.Id}}' "$IMAGE")"
  docker run --rm "$IMAGE" cat /opt/franka-rt/IMAGE_MANIFEST.txt
} | tee last-build.txt
