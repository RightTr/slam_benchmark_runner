#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_ROOT="$(cd -- "$SCRIPT_DIR/../.." && pwd)"
ROS_SETUP="${ROS_SETUP:-/opt/ros/humble/setup.bash}"
OVERLAY_SETUP="$WORKSPACE_ROOT/install/setup.bash"

DATASET_CONFIG="$SCRIPT_DIR/slam_bench_runner/config/datasets.json"
DATASET_DIR="${SLAM_BENCH_DATASET_DIR:-}"
if [[ -z "$DATASET_DIR" ]]; then
  DATASET_DIR="$(python3 -c 'import json,pathlib,sys; print(json.loads(pathlib.Path(sys.argv[1]).read_text())["dataset_dir"])' "$DATASET_CONFIG")"
fi
mapfile -t SEQUENCE_PATHS < <(python3 - "$DATASET_CONFIG" "$DATASET_DIR" <<'PY'
import json, pathlib, sys
config = json.loads(pathlib.Path(sys.argv[1]).read_text())
root = pathlib.Path(sys.argv[2])
if [item["id"] for item in config["sequences"]] != list(range(1, 7)):
    raise SystemExit("student dataset config must list sequence ids 1 through 6 in order")
for key in ("archive", "calibration"):
    for item in config["sequences"]:
        path = pathlib.Path(item[key])
        print(path if path.is_absolute() else root / path)
PY
)
if [[ ${#SEQUENCE_PATHS[@]} -ne 12 ]]; then
  echo "Dataset config must provide six archive and six calibration paths: $DATASET_CONFIG" >&2
  exit 2
fi
ARCHIVE_PATHS=("${SEQUENCE_PATHS[@]:0:6}")
CALIBRATION_PATHS=("${SEQUENCE_PATHS[@]:6:6}")
OUTPUT_DIR="${SLAM_BENCH_OUTPUT_DIR:-$SCRIPT_DIR/slam_bench_runs}"
TEACHER_PUBLIC_KEY="${SLAM_BENCH_TEACHER_PUBLIC_KEY:-$SCRIPT_DIR/key/teacher_public.key}"
VOCABULARY="$WORKSPACE_ROOT/src/slam_benchmark_runner/ORB_SLAM3/Vocabulary/ORBvoc.txt"

if [[ ! -f "$ROS_SETUP" ]]; then
  echo "ROS 2 setup not found: $ROS_SETUP" >&2
  exit 2
fi
if [[ ! -f "$OVERLAY_SETUP" ]]; then
  echo "Workspace overlay not found: $OVERLAY_SETUP" >&2
  echo "Build from $WORKSPACE_ROOT first: colcon build --base-paths src --packages-up-to slam_bench_runner" >&2
  exit 2
fi
for dataset_file in "${ARCHIVE_PATHS[@]}" "${CALIBRATION_PATHS[@]}"; do
  if [[ ! -f "$dataset_file" ]]; then
    echo "Configured dataset file not found: $dataset_file" >&2
    exit 2
  fi
done
if [[ ! -f "$TEACHER_PUBLIC_KEY" ]]; then
  echo "Teacher public key not found: $TEACHER_PUBLIC_KEY" >&2
  exit 2
fi
if [[ ! -f "$VOCABULARY" ]]; then
  echo "ORB-SLAM3 vocabulary not found: $VOCABULARY" >&2
  exit 2
fi

# ROS 2 setup hooks reference optional variables that are unset in a clean shell.
set +u
source "$ROS_SETUP"
source "$OVERLAY_SETUP"
set -u
mkdir -p "$OUTPUT_DIR"
cd "$WORKSPACE_ROOT"

ORB_LOG="$OUTPUT_DIR/orbslam3_launch.log"
setsid ros2 launch orb_slam3 rgbd.launch.py \
  vocabulary:="src/slam_benchmark_runner/ORB_SLAM3/Vocabulary/ORBvoc.txt" \
  >"$ORB_LOG" 2>&1 &
ORB_LAUNCH_PID=$!
cleanup_orbslam3() {
  local process_group="-$ORB_LAUNCH_PID"
  if ! kill -0 -- "$process_group" 2>/dev/null; then
    wait "$ORB_LAUNCH_PID" 2>/dev/null || true
    return
  fi

  # Let ROS 2 tear down the node and Pangolin viewer cleanly first.
  kill -INT -- "$process_group" 2>/dev/null || true
  for _ in {1..30}; do
    kill -0 -- "$process_group" 2>/dev/null || break
    sleep 0.1
  done

  # Do not leave a viewer or launch child behind if graceful shutdown stalls.
  if kill -0 -- "$process_group" 2>/dev/null; then
    kill -TERM -- "$process_group" 2>/dev/null || true
    for _ in {1..10}; do
      kill -0 -- "$process_group" 2>/dev/null || break
      sleep 0.1
    done
  fi
  if kill -0 -- "$process_group" 2>/dev/null; then
    kill -KILL -- "$process_group" 2>/dev/null || true
  fi
  wait "$ORB_LAUNCH_PID" 2>/dev/null || true
}
trap cleanup_orbslam3 EXIT
sleep 2
if ! kill -0 "$ORB_LAUNCH_PID" 2>/dev/null; then
  echo "ORB-SLAM3 RGB-D launch failed. Log: $ORB_LOG" >&2
  tail -n 40 "$ORB_LOG" >&2 || true
  exit 2
fi
echo "ORB-SLAM3 RGB-D started (Pangolin). Log: $ORB_LOG"

ROS_ARGS=(
  -p "dataset_dir:=$DATASET_DIR"
  -p "output_dir:=$OUTPUT_DIR"
  -p "sequence_archive_paths:=$(python3 -c 'import json,sys; print(json.dumps(sys.argv[1:]))' "${ARCHIVE_PATHS[@]}")"
  -p "sequence_calibration_paths:=$(python3 -c 'import json,sys; print(json.dumps(sys.argv[1:]))' "${CALIBRATION_PATHS[@]}")"
)
ROS_ARGS+=( -p "teacher_public_key:=$TEACHER_PUBLIC_KEY" )

set +e
ros2 run slam_bench_runner slam_bench --ros-args "${ROS_ARGS[@]}" "$@"
RUNNER_STATUS=$?
set -e
exit "$RUNNER_STATUS"
