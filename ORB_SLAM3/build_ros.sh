#!/usr/bin/env bash
set -euo pipefail

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
core_root_input="${ORB_SLAM3_ROOT:-$repo_root}"
core_root="$(cd "$core_root_input" && pwd)"
workspace_root="$(cd "$repo_root/../../.." && pwd)"
ros_distro="${ROS_DISTRO:-humble}"
setup_file="/opt/ros/${ros_distro}/setup.bash"

if [[ ! -f "$setup_file" ]]; then
  echo "ROS 2 setup not found: $setup_file" >&2
  exit 1
fi
vendored_pangolin="$repo_root/Thirdparty/Pangolin"
pangolin_args=()
if [[ -f "$vendored_pangolin/CMakeLists.txt" ]]; then
  pangolin_build="$workspace_root/build/pangolin_vendor"
  pangolin_prefix="$workspace_root/build/pangolin_install"
  cmake -S "$vendored_pangolin" -B "$pangolin_build" \
    -DCMAKE_BUILD_TYPE=Release \
    -DCMAKE_INSTALL_PREFIX="$pangolin_prefix" \
    -DCMAKE_INSTALL_LIBDIR=lib \
    -DBUILD_PANGOLIN_PYTHON=OFF \
    -DBUILD_TOOLS=OFF \
    -DBUILD_EXAMPLES=OFF \
    -DBUILD_TESTS=OFF
  cmake --build "$pangolin_build" --parallel "${BUILD_JOBS:-4}"
  cmake --install "$pangolin_build"
  export CMAKE_PREFIX_PATH="$pangolin_prefix${CMAKE_PREFIX_PATH:+:$CMAKE_PREFIX_PATH}"
  export LD_LIBRARY_PATH="$pangolin_prefix/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
  pangolin_args=("-DPangolin_DIR=$pangolin_prefix/lib/cmake/Pangolin")
fi
core_build="$workspace_root/build/orbslam3_core"
mkdir -p "$core_build"
cmake -S "$core_root" -B "$core_build" \
  -DCMAKE_BUILD_TYPE=Release \
  "${pangolin_args[@]}" \
  -Dg2o_LIBRARY_OUTPUT_DIRECTORY="$core_build/Thirdparty/g2o/lib"
cmake --build "$core_build" --parallel "${BUILD_JOBS:-4}"
set +u
source "$setup_file"
set -u
cd "$workspace_root"
colcon build \
  --base-paths "$repo_root/../orb_slam3" "$repo_root/../slam_bench_msgs" \
  --packages-up-to orb_slam3 \
  --cmake-args -DORB_SLAM3_ROOT="$core_root" -DORB_SLAM3_BUILD_ROOT="$core_build" \
  "${pangolin_args[@]}"
