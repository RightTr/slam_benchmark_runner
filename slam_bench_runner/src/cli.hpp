#pragma once

#include <functional>
#include <cstdint>
#include <string>

namespace slam_bench::cli {

std::string paint(const std::string& text, const char* ansi_code);
void printMetrics(double ate_rmse, double rte_rmse, double rotation_rpe, double tracking_ratio);
struct StatusView {
  bool test_mode{}, connected{}, running{}, progress_known{};
  int sequence{};
  double speed{1.0};
  std::string phase;
  std::uint64_t published{}, expected{}, received{};
};
void printStatus(const StatusView& status);

struct Actions {
  std::function<void()> test;
  std::function<void(const std::string&)> evaluation;
  std::function<void(int)> select;
  std::function<void(double)> set_speed;
  std::function<void()> play;
  std::function<void()> pause;
  std::function<void()> resume;
  std::function<void()> cancel;
  std::function<void()> status;
  std::function<bool()> is_running;
  std::function<std::string()> take_error;
};

class Console {
 public:
  explicit Console(Actions actions);
  int run();

 private:
  Actions actions_;
};

}  // namespace slam_bench::cli
