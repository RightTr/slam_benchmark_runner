#pragma once

#include <functional>
#include <string>

namespace slam_bench::cli {

std::string paint(const std::string& text, const char* ansi_code);

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
