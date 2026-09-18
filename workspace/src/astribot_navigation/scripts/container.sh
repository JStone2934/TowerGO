#!/bin/bash
set -euo pipefail
REPO=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)
WS=$(cd "$REPO/../.." && pwd)
IMAGE=towergo-offline:humble
case "${1:-help}" in
  build)
    python3 "$REPO/scripts/fetch_vendor.py"
    sudo docker build --build-arg "BASE_IMAGE=${BASE_IMAGE:-ros:humble-ros-base}" -f "$REPO/docker/Dockerfile" -t "$IMAGE" "$REPO"
    mkdir -p "$WS/data/reports"
    sudo docker image inspect "$IMAGE" > "$WS/data/reports/image-inspect.json"
    ;;
  run)
    shift
    mkdir -p "$WS/data/reports" "$WS/data/maps" "$WS/data/bags"
    IMAGE_ID=$(sudo docker image inspect --format "{{.Id}}" "$IMAGE")
    sudo docker run --rm --network none --ipc private --cap-drop ALL \
      --security-opt no-new-privileges --user "$(id -u):$(id -g)" \
      -e "TOWERGO_IMAGE_ID=$IMAGE_ID" -e HOME=/tmp -e ROS_DOMAIN_ID=125 -e ROS_LOCALHOST_ONLY=1 \
      -v "$WS/data:/data" "$IMAGE" "$@"
    ;;
  *) echo 'Usage: container.sh build | run <command...>'; exit 2;;
esac
