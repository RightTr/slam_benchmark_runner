# 🧭 SLAM-Bench Runner

SLAM-Bench is the offline RGB-D benchmark for the **Robot Vision and Navigation** course assignment. Run your own ORB-SLAM3 ROS 2 node on your computer, then use the benchmark console to play a sequence and view trajectory accuracy. You may change the SLAM algorithm; keep the ROS 2 interface compatible.

## 📦 What is included

- 🖥️ A console runner and compiled benchmark core library.
- 🔒 Encrypted RGB-D sequences and ROS 2 control messages.
- 🧪 Test mode: choose a sequence and playback speed, then view ATE, RTE, and 2D/3D trajectory plots.
- 📊 Evaluation mode: run all six sequences and create one encrypted submission BIN, in addition to the metrics and plots.

## ▶️ Run

1. Start your ORB-SLAM3 ROS 2 RGB-D node in one terminal. It must subscribe to `/benchmark/rgb`, `/benchmark/depth`, and `/benchmark/camera_info`, and respond to `/benchmark/control`.
2. Start the benchmark in another terminal:

   ```bash
   ./run_slam_bench.sh
   ```

3. Choose `test` or `eval` in the console. Test lets you pick a sequence and speed; Evaluation asks for your 13-digit student ID. Use `status`, `pause`, `resume`, or `cancel` during a run.

The SLAM node sends its final camera trajectory on `/benchmark/trajectory_final` and acknowledges control commands on `/benchmark/control_ack`. Image and trajectory timestamps must match exactly. The full six-sequence dataset package is required for Evaluation.

Results are saved under `slam_bench_runs/test/` or `slam_bench_runs/eval/`. Submit the `submission_<student_id>.bin` from a completed Evaluation run. The runner does not require a separate dataset decryption key file; it does require the supplied teacher **public** key for BIN encryption.
