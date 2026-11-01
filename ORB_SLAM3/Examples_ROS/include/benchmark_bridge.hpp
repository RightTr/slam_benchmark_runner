#pragma once

#include <functional>
#include <condition_variable>
#include <chrono>
#include <cstdint>
#include <map>
#include <mutex>
#include <string>
#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/path.hpp>
#include <slam_bench_msgs/msg/benchmark_control.hpp>
#include <slam_bench_msgs/msg/benchmark_control_ack.hpp>
#include <slam_bench_msgs/msg/benchmark_progress.hpp>

class BenchmarkBridge {
public:
    using Control = slam_bench_msgs::msg::BenchmarkControl;
    BenchmarkBridge(const rclcpp::Node::SharedPtr& node,
                    std::function<void()> reset,
                    std::function<void()> cancel,
                    std::function<void(Control)> end);
    bool accepting() const;
    void registerInputStamp(const builtin_interfaces::msg::Time& stamp);
    void publishProgress(uint64_t processed_frames);
    bool waitForInputStamp(int64_t stamp_ns, std::chrono::milliseconds timeout);
    void finishEnd(const Control& request, const std::string& trajectory_file);
    void rejectEnd(const Control& request, const std::string& detail);
private:
    void ack(const Control& request, bool accepted, const std::string& detail);
    rclcpp::Node::SharedPtr node_;
    std::function<void()> reset_;
    std::function<void()> cancel_;
    std::function<void(Control)> end_;
    rclcpp::Publisher<slam_bench_msgs::msg::BenchmarkControlAck>::SharedPtr ack_pub_;
    rclcpp::Publisher<slam_bench_msgs::msg::BenchmarkProgress>::SharedPtr progress_pub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr trajectory_pub_;
    rclcpp::Subscription<Control>::SharedPtr control_sub_;
    mutable std::mutex mutex_;
    std::condition_variable stamp_cv_;
    std::map<int64_t, builtin_interfaces::msg::Time> stamps_;
    bool accepting_{false};
    std::string run_, session_;
    uint64_t reset_command_id_{0};
    bool has_reset_{false};
    bool ending_{false};
    uint64_t end_command_id_{0};
    bool has_completed_end_{false};
    uint64_t completed_end_command_id_{0};
    int64_t expected_final_stamp_ns_{0};
    bool final_input_seen_{false};
    bool cancelled_{false};
};
