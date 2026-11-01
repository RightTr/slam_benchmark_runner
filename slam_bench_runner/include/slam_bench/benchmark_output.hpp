#pragma once
#include "slam_bench/benchmark_core.hpp"
namespace slam_bench {
class SLAM_BENCH_API BenchmarkOutput {
 public:
  explicit BenchmarkOutput(BenchmarkCore& core);
  void registerFrame(uint64_t frame_id);
  void recordRgbPublishBegin(uint64_t frame_id,int64_t time_ns);
  void recordRgbPublishEnd(uint64_t frame_id,int64_t time_ns);
  void recordDepthPublishBegin(uint64_t frame_id,int64_t time_ns);
  void recordDepthPublishEnd(uint64_t frame_id,int64_t time_ns);
  void recordFinalTrajectory(const std::vector<FinalPose>& trajectory);
  void markAllImagesPublished(int64_t final_depth_end_ns);
  SessionProgress completionStatus(int64_t now_steady_ns) const;
  void finishSession(int64_t now_steady_ns);
  void cancelSession();
  Score evaluate() const;
  void saveTrajectory2D(const std::string& path) const;
  void saveTrajectory3D(const std::string& path) const;
 private:
  BenchmarkCore& core_;
};
}
