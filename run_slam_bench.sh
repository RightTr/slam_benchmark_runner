#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_ROOT="$(cd -- "$SCRIPT_DIR/../.." && pwd)"
ROS_SETUP="${ROS_SETUP:-/opt/ros/humble/setup.bash}"
OVERLAY_SETUP="$WORKSPACE_ROOT/install/setup.bash"

DATASET_DIR="${SLAM_BENCH_DATASET_DIR:-$SCRIPT_DIR/data/encrypted}"
KEY_FILE="${SLAM_BENCH_KEY_FILE:-$SCRIPT_DIR/key/dataset_aes256.key}"
OUTPUT_DIR="${SLAM_BENCH_OUTPUT_DIR:-$SCRIPT_DIR/slam_bench_runs}"
TEACHER_PUBLIC_KEY="${SLAM_BENCH_TEACHER_PUBLIC_KEY:-$SCRIPT_DIR/key/teacher_public.key}"

if [[ ! -f "$ROS_SETUP" ]]; then
  echo "ROS 2 setup not found: $ROS_SETUP" >&2
  exit 2
fi
if [[ ! -f "$OVERLAY_SETUP" ]]; then
  echo "Workspace overlay not found: $OVERLAY_SETUP" >&2
  echo "Build from $WORKSPACE_ROOT first: colcon build --base-paths src --packages-up-to slam_bench_runner" >&2
  exit 2
fi
if [[ ! -f "$DATASET_DIR/sequence_01.bench" ]]; then
  echo "Encrypted dataset not found under: $DATASET_DIR" >&2
  exit 2
fi
if [[ ! -f "$KEY_FILE" ]]; then
  echo "Dataset key not found: $KEY_FILE" >&2
  exit 2
fi
if [[ ! -f "$TEACHER_PUBLIC_KEY" ]]; then
  echo "Teacher public key not found: $TEACHER_PUBLIC_KEY" >&2
  exit 2
fi

# ROS 2 setup hooks reference optional variables that are unset in a clean shell.
set +u
source "$ROS_SETUP"
source "$OVERLAY_SETUP"
set -u
mkdir -p "$OUTPUT_DIR"
cd "$WORKSPACE_ROOT"

ROS_ARGS=(
  -p "dataset_dir:=$DATASET_DIR"
  -p "dataset_key_file:=$KEY_FILE"
  -p "output_dir:=$OUTPUT_DIR"
)
ROS_ARGS+=( -p "teacher_public_key:=$TEACHER_PUBLIC_KEY" )

exec ros2 run slam_bench_runner slam_bench --ros-args "${ROS_ARGS[@]}" "$@"
