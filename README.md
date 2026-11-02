# SLAM-Bench Runner

**English** | [简体中文](README.zh-CN.md)

SLAM-Bench Runner is an offline RGB-D benchmark for the **Robot Vision and Navigation** course assignment. Run your ORB-SLAM3 ROS 2 node locally, play encrypted sequences through the console, and inspect trajectory accuracy. You can change the SLAM algorithm as long as its benchmark ROS 2 interface remains compatible.

## What is included

- An ORB-SLAM3 RGB-D ROS 2 wrapper and benchmark control messages.
- A console runner and a supplied, compiled benchmark core for encrypted datasets.
- **Test:** run one selected sequence at 1×, 2×, 2.5×, or 3× speed and view metrics and 2D/3D trajectory plots.
- **Evaluation:** run all six sequences and produce an encrypted submission BIN after successful completion.

## Environment and setup

The following steps target **Ubuntu 22.04 with ROS 2 Humble**. Place this repository at `<workspace>/src/slam_benchmark_runner`; the launch script derives the workspace root from that layout. Install [ROS 2 Humble](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html) first, including the ROS 2 development tools and `colcon`.

Install the native build dependencies used by this repository's ORB-SLAM3 CMake files:

```bash
sudo apt update
sudo apt install build-essential cmake git python3-colcon-common-extensions \
  libopencv-dev libeigen3-dev libboost-serialization-dev libssl-dev \
  libsodium23 libzstd1 libgl1-mesa-dev libegl1-mesa-dev \
  libepoxy-dev libglew-dev libx11-dev pkg-config
```

ORB-SLAM3 also needs **Pangolin** for its viewer. This repository includes Pangolin v0.9.6 under `ORB_SLAM3/Thirdparty/Pangolin`; `build_ros.sh` builds it into the workspace's `build/pangolin_install` directory automatically. No separate Pangolin clone or system-wide installation is needed. Its optional Python and vcpkg submodules are not used by this C++ benchmark; see the [vendored source note](ORB_SLAM3/Thirdparty/Pangolin/VENDORED_FROM.md) and [Pangolin's documentation](https://github.com/stevenlovegrove/Pangolin/blob/master/README.md).

Pangolin opens a graphical viewer, so run the benchmark in a desktop session with a working OpenGL display. This repository's ORB-SLAM3 build requires a C++17 compiler, OpenCV 4.4 or newer, Eigen3, Boost Serialization, OpenSSL, and Pangolin. DBoW2 and g2o are included under `ORB_SLAM3/Thirdparty`. See the [upstream ORB-SLAM3 documentation](ORB_SLAM3/README.md) for background; use this repository's `build_ros.sh` for its ROS 2 wrapper.

The student distribution includes `slam_bench_runner/lib/libslam_bench_core.so` and `libslam_bench_cli.so`. The public runner source links against these prebuilt libraries; students do not need the instructor-only core source or dataset decryption key. Keep the repository at the expected workspace path and use a compatible Ubuntu 22.04 system and CPU architecture for these binaries.

Build the native ORB-SLAM3 library and its ROS 2 wrapper from this repository root:

```bash
cd ORB_SLAM3
./build_ros.sh
cd ..
```

`build_ros.sh` builds the bundled Pangolin, the ORB-SLAM3 core and its included DBoW2/g2o libraries, then builds the `orb_slam3` wrapper and `slam_bench_msgs` with `colcon`. The launcher expects `ORB_SLAM3/Vocabulary/ORBvoc.txt`. If that file is missing, extract it from the included archive before launching:

```bash
tar -xf ORB_SLAM3/Vocabulary/ORBvoc.txt.tar.gz -C ORB_SLAM3/Vocabulary
```

Build the public benchmark runner against the supplied libraries. Run these commands from the repository root after `build_ros.sh` has finished:

```bash
cd ../..
source /opt/ros/humble/setup.bash
source install/setup.bash
colcon build --base-paths src/slam_benchmark_runner/slam_bench_runner \
  --packages-select slam_bench_runner
source install/setup.bash
cd src/slam_benchmark_runner
```

This installs the runner executable and both `.so` files in the workspace overlay. The launcher checks for `<workspace>/install/setup.bash`. If a library is missing, the runner build stops with a path-specific error; obtain the complete student distribution before continuing.

## Configure the dataset

Edit `slam_bench_runner/config/datasets.json`: set `dataset_dir` to your local dataset directory and check the archive and calibration paths for sequence IDs 1–6. Relative paths are resolved under `dataset_dir`. Alternatively, set `SLAM_BENCH_DATASET_DIR` to override only the directory at launch time.

All six archive and calibration paths must exist before the launcher starts, including when you intend to run just one test sequence. A complete six-sequence package is required for Evaluation.

## Run

From the repository root:

```bash
./run_slam_bench.sh
```

The script starts the ORB-SLAM3 RGB-D node and Pangolin viewer, then opens the benchmark console. Enter `test` to select a sequence and speed, or `eval` to enter your **13-digit student ID** and run all six sequences. During a run, use `status`, `pause`, `resume`, or `cancel`; enter `help` to see all commands.

The SLAM node receives `/benchmark/rgb`, `/benchmark/depth`, `/benchmark/camera_info`, and `/benchmark/control`. It publishes `/benchmark/progress`, `/benchmark/control_ack`, and the final camera path on `/benchmark/trajectory_final`. Image and trajectory timestamps must match exactly. You may modify ORB-SLAM3's Tracking, Local Mapping, Loop Closing, and Bundle Adjustment code; keep `ORB_SLAM3/Examples_ROS` and the benchmark message protocol compatible.

## Results and submission

Results are written under `slam_bench_runs/test/` or `slam_bench_runs/eval/`. Each session includes `metrics.txt`, `trajectory_2d.png`, and `trajectory_3d.png`. Metrics include ATE RMSE, translational RTE RMSE, rotational RPE RMSE, and tracking ratio. The ORB-SLAM3 launch log is `slam_bench_runs/orbslam3_launch.log`.

After all six Evaluation sessions complete, submit the generated `submission_<student_id>.bin` from that evaluation run's directory. The launcher uses the supplied `key/teacher_public.key` to encrypt the BIN; no separate dataset decryption key file is needed at runtime.
