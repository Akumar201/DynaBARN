#!/usr/bin/env bash
# Run on a machine with Docker (not the cluster). Builds the ready-to-run DynaBARN II image, with
# the workspace and the training worlds inside, and saves it as one file to copy to the cluster.
#
# Usage: build_image.sh [output.tar]       (default ./dynabarn2_image.tar, about 6 GB)
# Then on the cluster:  apptainer build dynabarn2.sif dynabarn2_image.tar
#   (the plain .tar path works with recent Apptainer; older versions need docker-archive://<path>)
set -eu
OUT=$(realpath -m "${1:-dynabarn2_image.tar}")
ROS2_PKG=$(cd "$(dirname "$0")/../../../../../.." && pwd)   # dyna_barn/ros2_pkg
docker image inspect ros2-jazzy-gz:latest > /dev/null 2>&1 || \
  docker compose -f "$ROS2_PKG/docker_env/docker-compose.yml" build
# The training worlds are not in git; regenerate them (deterministic, about 11 minutes) if missing.
PKG=$ROS2_PKG/ws/src/dynabarn2
[ -f "$PKG/worlds/dynabarn2_train/manifest.json" ] || docker run --rm --user "$(id -u):$(id -g)" \
  -v "$PKG:/pkg" -w /pkg ros2-jazzy-gz:latest python3 -m dynabarn2_worlds.generate_dynabarn2 \
  --save_dir worlds/dynabarn2_train --seed 2030 --per_cell 20
docker build -f "$ROS2_PKG/docker_env/Dockerfile.release" -t dynabarn2:latest "$ROS2_PKG"
docker save dynabarn2:latest -o "$OUT"
echo "Saved $OUT ($(du -h "$OUT" | cut -f1)). Copy it to the cluster, then: apptainer build dynabarn2.sif $(basename "$OUT")"
