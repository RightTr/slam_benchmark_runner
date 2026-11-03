# SLAM-Bench Runner

[English](README.md) | **简体中文**

SLAM-Bench Runner 是**机器人视觉与导航**课程作业使用的离线 RGB-D 评测工具。在本机运行 ORB-SLAM3 ROS 2 节点，通过控制台播放加密数据序列，并查看轨迹精度。可以修改 SLAM 算法，但需保持评测所用的 ROS 2 接口兼容。

## 项目内容

- ORB-SLAM3 RGB-D ROS 2 封装，以及评测控制消息。
- 控制台运行程序和提供的预编译评测核心，用于读取加密数据集。
- **测试模式（Test）：**选择一条序列，以 1×、2×、2.5× 或 3× 速度播放，查看指标和二维、三维轨迹图。
- **评估模式（Evaluation）：**运行全部六条序列；全部成功完成后生成加密的提交 BIN 文件。

## 环境与构建

以下步骤以 **Ubuntu 22.04 + ROS 2 Humble** 为例。将本仓库放在 `<工作区>/src/slam_benchmark_runner`；启动脚本依靠这个目录结构定位工作区。先按 [ROS 2 Humble 安装文档](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html)安装 ROS 2，包括开发工具和 `colcon`。

安装本项目 ORB-SLAM3 CMake 文件所需的原生编译依赖：

```bash
sudo apt update
sudo apt install build-essential cmake git python3-colcon-common-extensions \
  libopencv-dev libeigen3-dev libboost-serialization-dev libssl-dev \
  libsodium23 libzstd1 libgl1-mesa-dev libegl1-mesa-dev \
  libepoxy-dev libglew-dev libx11-dev pkg-config
```

ORB-SLAM3 的查看器还需要 **Pangolin**。本仓库已将 Pangolin v0.9.6 放在 `ORB_SLAM3/Thirdparty/Pangolin`；`build_ros.sh` 会自动编译到工作区的 `build/pangolin_install`，无需另行克隆或安装到系统目录。这个 C++ 评测程序不使用 Pangolin 可选的 Python 和 vcpkg 子模块；参见[源码说明](ORB_SLAM3/Thirdparty/Pangolin/VENDORED_FROM.md)和 [Pangolin 项目文档](https://github.com/stevenlovegrove/Pangolin/blob/master/README.md)。

Pangolin 会打开图形查看器，因此运行评测需要可用的桌面与 OpenGL 显示环境。本仓库的 ORB-SLAM3 编译需要支持 C++17 的编译器、OpenCV 4.4 或更新版本、Eigen3、Boost Serialization、OpenSSL 和 Pangolin。DBoW2 与 g2o 已放在 `ORB_SLAM3/Thirdparty`。上游背景参见 [ORB-SLAM3 文档](ORB_SLAM3/README.md)；编译本仓库的 ROS 2 封装请使用这里的 `build_ros.sh`。

学生发放包包含 `slam_bench_runner/lib/libslam_bench_core.so` 和 `libslam_bench_cli.so`。公开的评测入口源码会链接这两份预编译库；学生无需教师专用的核心源码或数据集解密密钥。仓库须放在约定的工作区位置，并使用与这些二进制文件兼容的 Ubuntu 22.04 系统和 CPU 架构。

在本仓库根目录编译 ORB-SLAM3 原生库和 ROS 2 封装：

```bash
cd ORB_SLAM3
./build_ros.sh
cd ..
```

`build_ros.sh` 会先编译附带的 Pangolin、ORB-SLAM3 核心及其 DBoW2/g2o 库，再通过 `colcon` 编译 `orb_slam3` 封装和 `slam_bench_msgs`。启动脚本需要 `ORB_SLAM3/Vocabulary/ORBvoc.txt`。若该文件不存在，请在启动前从附带的压缩包解压：

```bash
tar -xf ORB_SLAM3/Vocabulary/ORBvoc.txt.tar.gz -C ORB_SLAM3/Vocabulary
```

完成 `build_ros.sh` 后，在本仓库根目录构建工作区。工作区会识别 ROS 2 封装、评测消息和公开评测入口三个包：

```bash
cd ../..
source /opt/ros/humble/setup.bash
source install/setup.bash
colcon build
source install/setup.bash
cd src/slam_benchmark_runner
```

构建完成后，评测器可执行文件和两份 `.so` 都会安装到工作区 overlay。启动脚本会检查 `<工作区>/install/setup.bash`。若缺少库文件，评测器构建会指出缺失路径；请先获取完整的学生发放包。

## 配置数据集

编辑 `slam_bench_runner/config/datasets.json`：将 `dataset_dir` 改为本机数据集目录，并检查编号 1–6 的数据包与标定文件路径。相对路径以 `dataset_dir` 为基准。也可以在启动时设置 `SLAM_BENCH_DATASET_DIR`，仅覆盖数据集目录。

即使只测试一条序列，启动脚本也会先检查六条序列的数据包和标定文件是否都存在。评估模式必须使用完整的六序列数据集。

## 运行

在本仓库根目录执行：

```bash
./run_slam_bench.sh
```

脚本会启动 ORB-SLAM3 RGB-D 节点与 Pangolin 查看器，再打开评测控制台。输入 `test` 选择测试序列和速度；输入 `eval`，填写 **13 位学号**并运行全部六条序列。运行时可输入 `status`、`pause`、`resume` 或 `cancel`；输入 `help` 可查看全部命令。

SLAM 节点接收 `/benchmark/rgb`、`/benchmark/depth`、`/benchmark/camera_info` 和 `/benchmark/control`，发布 `/benchmark/progress`、`/benchmark/control_ack`，并通过 `/benchmark/trajectory_final` 发布最终相机轨迹。图像和轨迹的时间戳必须严格一致。可以修改 ORB-SLAM3 的 Tracking、Local Mapping、Loop Closing 和 Bundle Adjustment 算法代码；请保持 `ORB_SLAM3/Examples_ROS` 和评测消息协议兼容。

## 结果与提交

结果保存在 `slam_bench_runs/test/` 或 `slam_bench_runs/eval/` 下。每次运行包含 `metrics.txt`、`trajectory_2d.png` 和 `trajectory_3d.png`。指标包括 ATE 均方根误差、平移 RTE 均方根误差、旋转 RPE 均方根误差及跟踪率。每次 ORB-SLAM3 启动日志单独保存在 `slam_bench_runs/orbslam3_launch_<时间>_<进程号>.log`，启动时也会打印具体路径。

评估模式的六次运行全部完成后，提交该次评估目录下生成的 `submission_<student_id>.bin`。启动脚本使用随项目提供的 `key/teacher_public.key` 加密 BIN；运行时无需单独提供数据集解密密钥文件。
