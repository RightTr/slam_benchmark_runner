# 🧭 SLAM-Bench Runner

SLAM-Bench Runner is the offline RGB-D benchmark for the **Robot Vision and Navigation** course assignment. Run your own ORB-SLAM3 ROS 2 node on your computer, then use the benchmark console to play a sequence and view trajectory accuracy. You may change the SLAM algorithm; keep the ROS 2 interface compatible.

## 📦 What is included

- 🖥️ A console runner and compiled benchmark core library.
- 🔒 Encrypted RGB-D sequences and ROS 2 control messages.
- 🧪 Test mode: choose a sequence and playback speed, then view ATE, RTE, and 2D/3D trajectory plots.
- 📊 Evaluation mode: run all six sequences and create one encrypted submission BIN, in addition to the metrics and plots.

## ▶️ Run

### Build ORB-SLAM3

The ROS 2 RGB-D wrapper is included in `ORB_SLAM3/Examples_ROS`. It reuses the `slam_bench_msgs` package in this repository; there is one shared definition of the benchmark control messages.

Build the native ORB-SLAM3 library and ROS 2 wrapper from the workspace:

```bash
cd ORB_SLAM3
./build_ros.sh
```

### Start a run

1. Start the student ORB-SLAM3 node in one terminal:

   ```bash
   cd /home/right/slambr_ws
   source /opt/ros/humble/setup.bash
   source /home/right/slambr_ws/install/setup.bash
   ros2 launch orb_slam3 rgbd.launch.py \
     vocabulary:=src/slam_benchmark_runner/ORB_SLAM3/Vocabulary/ORBvoc.txt
   ```

   The wrapper subscribes to `/benchmark/rgb`, `/benchmark/depth`, and `/benchmark/camera_info`; it handles `/benchmark/control` and publishes the final camera trajectory on `/benchmark/trajectory_final`.

2. Start the benchmark in another terminal:

   ```bash
   ./run_slam_bench.sh
   ```

3. Choose `test` or `eval` in the console. Test lets you pick a sequence and speed; Evaluation asks for your 13-digit student ID. Use `status`, `pause`, `resume`, or `cancel` during a run.

The SLAM node sends its final camera trajectory on `/benchmark/trajectory_final` and acknowledges control commands on `/benchmark/control_ack`. Image and trajectory timestamps must match exactly. The full six-sequence dataset package is required for Evaluation.

Results are saved under `slam_bench_runs/test/` or `slam_bench_runs/eval/`. Submit the `submission_<student_id>.bin` from a completed Evaluation run. The runner does not require a separate dataset decryption key file; it does require the supplied teacher **public** key for BIN encryption.
