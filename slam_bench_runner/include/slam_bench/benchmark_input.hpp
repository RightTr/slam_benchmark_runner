#pragma once
#include "slam_bench/benchmark_core.hpp"
namespace slam_bench {
class SLAM_BENCH_API BenchmarkInput {
 public:
  explicit BenchmarkInput(BenchmarkCore& core);
  bool nextFrame(ImageFrame& frame);
 private:
  BenchmarkCore& core_;
};
}
