#pragma once

#include <cstdint>
#include <memory>
#include <string>
#include <vector>
#include <opencv2/core.hpp>

#if defined(_WIN32)
# if defined(SLAM_BENCH_CORE_BUILD)
#  define SLAM_BENCH_API __declspec(dllexport)
# else
#  define SLAM_BENCH_API __declspec(dllimport)
# endif
#else
# define SLAM_BENCH_API __attribute__((visibility("default")))
#endif

namespace slam_bench {
enum class Mode { Test, Evaluation };
struct Score { double ate_rmse{}, rpe_trans{}, rpe_rot{}, tracking_ratio{}; };
struct ImageFrame { uint64_t frame_id{}; int64_t stamp_ns{}, relative_ns{}; cv::Mat rgb, depth; };
struct PoseData { double px{},py{},pz{},qx{},qy{},qz{},qw{1.0}; bool valid{}; };
struct FinalPose { int64_t stamp_ns{}; PoseData pose; };
struct Timing { int64_t rgb_begin{}, rgb_end{}, depth_begin{}, depth_end{}; };
enum class CompletionState { WaitingForImages, WaitingForFinalTrajectory, Complete, Incomplete, Cancelled };
struct SessionProgress {
  CompletionState state{CompletionState::WaitingForImages};
  uint64_t expected_frames{}, published_frames{}, received_frames{}, missing_frames{};
  int64_t last_image_stamp_ns{};
  bool final_trajectory_received{};
};

class SLAM_BENCH_API BenchmarkCore {
 public:
  static std::string generateRunUuid();
  static int64_t generateTimestampBaseNs();
  BenchmarkCore();
  ~BenchmarkCore();
  BenchmarkCore(BenchmarkCore&&) noexcept;
  BenchmarkCore& operator=(BenchmarkCore&&) noexcept;
  BenchmarkCore(const BenchmarkCore&) = delete;
  BenchmarkCore& operator=(const BenchmarkCore&) = delete;
  void initialize(Mode mode, const std::string& dataset_dir);
  void selectSequence(int sequence_id);
  std::vector<int> sequenceOrder() const;
  void beginSession(const std::string& run_uuid, const std::string& session_id);
  bool nextFrame(ImageFrame& frame);
  void registerFrame(uint64_t frame_id);
  void recordRgbPublishBegin(uint64_t frame_id, int64_t time_ns);
  void recordRgbPublishEnd(uint64_t frame_id, int64_t time_ns);
  void recordDepthPublishBegin(uint64_t frame_id, int64_t time_ns);
  void recordDepthPublishEnd(uint64_t frame_id, int64_t time_ns);
  void recordImagePublish(uint64_t frame_id, const Timing& timing);
  void recordFinalTrajectory(const std::vector<FinalPose>& trajectory);
  void recordKeyframeTrajectory(const std::vector<FinalPose>& trajectory);
  void markAllImagesPublished(int64_t final_depth_end_ns);
  SessionProgress completionStatus(int64_t now_steady_ns) const;
  void finishSession(int64_t now_steady_ns);
  void cancelSession();
  void setFinalTrajectoryTimeoutMs(int64_t timeout_ms);
  void setTeacherPublicKey(const std::string& path);
  Score getScore() const;
  Score getKeyframeScore() const;
  void savePlot(const std::string& path) const;
  void saveTrajectory3D(const std::string& path) const;
  void saveEvaluationBin(const std::string& path, const std::string& student_id) const;
 private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};
}
