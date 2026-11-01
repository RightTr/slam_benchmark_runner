#include "benchmark_bridge.hpp"
#include "benchmark_tum_reader.hpp"
#include <stdexcept>

BenchmarkBridge::BenchmarkBridge(const rclcpp::Node::SharedPtr& node,
    std::function<void()> reset, std::function<void()> cancel,
    std::function<void(Control)> end)
    : node_(node), reset_(std::move(reset)), cancel_(std::move(cancel)), end_(std::move(end))
{
    ack_pub_ = node_->create_publisher<slam_bench_msgs::msg::BenchmarkControlAck>(
        "/benchmark/control_ack", 10);
    progress_pub_ = node_->create_publisher<slam_bench_msgs::msg::BenchmarkProgress>(
        "/benchmark/progress", rclcpp::QoS(10).reliable());
    trajectory_pub_ = node_->create_publisher<nav_msgs::msg::Path>(
        "/benchmark/trajectory_final", 1);
    control_sub_ = node_->create_subscription<Control>("/benchmark/control", 10,
        [this](Control::ConstSharedPtr m) {
            try {
                if (m->command == Control::READY) { ack(*m, true, "ready"); return; }
                if (m->command == Control::CANCEL) {
                    bool session_matches = false;
                    {
                        std::lock_guard<std::mutex> lock(mutex_);
                        session_matches = run_ == m->run_uuid && session_ == m->session_id;
                        if (session_matches) {
                            accepting_ = false;
                            ending_ = false;
                            has_reset_ = false;
                            stamps_.clear();
                            expected_final_stamp_ns_ = 0;
                            final_input_seen_ = false;
                            cancelled_ = true;
                        }
                    }
                    if (!session_matches) { ack(*m, false, "session mismatch"); return; }
                    stamp_cv_.notify_all();
                    cancel_();
                    ack(*m, true, "queued frames discarded; ORB-SLAM3 kept running");
                } else if (m->command == Control::RESET) {
                    RCLCPP_INFO(node_->get_logger(),
                        "RESET received (command_id=%lu, session=%s)",
                        static_cast<unsigned long>(m->command_id), m->session_id.c_str());
                    { std::lock_guard<std::mutex> lock(mutex_);
                      if (has_reset_ && reset_command_id_==m->command_id &&
                          run_==m->run_uuid && session_==m->session_id) {
                          ack(*m,true,"reset complete"); return;
                      }
                      accepting_ = false;
                      ending_ = false;
                      stamps_.clear();
                      run_ = m->run_uuid; session_ = m->session_id;
                      has_reset_ = false; has_completed_end_ = false;
                      expected_final_stamp_ns_ = 0; final_input_seen_ = false; cancelled_ = false; }
                    RCLCPP_INFO(node_->get_logger(), "RESET: initializing ORB-SLAM3 System");
                    reset_();
                    RCLCPP_INFO(node_->get_logger(), "RESET: ORB-SLAM3 System initialized; sending ACK");
                    { std::lock_guard<std::mutex> lock(mutex_);
                      run_ = m->run_uuid; session_ = m->session_id;
                      reset_command_id_=m->command_id;has_reset_=true;
                      has_completed_end_=false; ending_=false;
                      expected_final_stamp_ns_=0; final_input_seen_=false; cancelled_=false; }
                    ack(*m, true, "reset complete");
                } else if (m->command == Control::BEGIN) {
                    { std::lock_guard<std::mutex> lock(mutex_);
                      if (run_ != m->run_uuid || session_ != m->session_id)
                          throw std::runtime_error("session mismatch");
                      if (accepting_) { ack(*m,true,"begin already active"); return; }
                      if (!has_reset_) throw std::runtime_error("RESET required before BEGIN");
                      accepting_ = true; }
                    ack(*m, true, "begin");
                } else if (m->command == Control::END) {
                    { std::lock_guard<std::mutex> lock(mutex_);
                      if (run_ != m->run_uuid || session_ != m->session_id)
                          throw std::runtime_error("session mismatch");
                      if (has_completed_end_ && completed_end_command_id_==m->command_id) {
                          ack(*m,true,"end already complete"); return;
                      }
                      if (ending_) {
                          if (end_command_id_ == m->command_id) return;
                          throw std::runtime_error("END already in progress");
                      }
                      if (!accepting_) throw std::runtime_error("BEGIN required before END");
                      if (m->final_image_stamp_ns<=0) throw std::runtime_error("END must include final image stamp");
                      accepting_ = false; ending_=true; end_command_id_=m->command_id;
                      expected_final_stamp_ns_=m->final_image_stamp_ns;
                      final_input_seen_=stamps_.find(expected_final_stamp_ns_)!=stamps_.end(); }
                    end_(*m);
                }
            } catch (const std::exception& e) {
                RCLCPP_ERROR(node_->get_logger(), "Control command %u failed: %s",
                    static_cast<unsigned>(m->command), e.what());
                if (m->command==Control::END) { std::lock_guard<std::mutex> lock(mutex_); ending_=false; }
                ack(*m, false, e.what());
            }
        });
}

bool BenchmarkBridge::accepting() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return accepting_ || (ending_ && !final_input_seen_);
}
void BenchmarkBridge::registerInputStamp(const builtin_interfaces::msg::Time& stamp) {
    const int64_t ns=int64_t(stamp.sec)*1000000000LL+stamp.nanosec;
    std::lock_guard<std::mutex> lock(mutex_);
    if (accepting_ || ending_) {
        stamps_.emplace(ns,stamp);
        if (ending_ && ns==expected_final_stamp_ns_) final_input_seen_=true;
        stamp_cv_.notify_all();
    }
}
void BenchmarkBridge::publishProgress(uint64_t processed_frames) {
    slam_bench_msgs::msg::BenchmarkProgress progress;
    {
        std::lock_guard<std::mutex> lock(mutex_);
        progress.run_uuid = run_;
        progress.session_id = session_;
    }
    if (progress.run_uuid.empty() || progress.session_id.empty()) return;
    progress.processed_frames = processed_frames;
    progress_pub_->publish(progress);
}
bool BenchmarkBridge::waitForInputStamp(int64_t stamp_ns,std::chrono::milliseconds timeout) {
    std::unique_lock<std::mutex> lock(mutex_);
    if (stamp_ns!=expected_final_stamp_ns_) return false;
    stamp_cv_.wait_for(lock,timeout,[this]{return final_input_seen_ || cancelled_;});
    return final_input_seen_ && !cancelled_;
}
void BenchmarkBridge::ack(const Control& m, bool accepted, const std::string& detail) {
    slam_bench_msgs::msg::BenchmarkControlAck a;
    a.command=m.command; a.command_id=m.command_id;
    a.run_uuid=m.run_uuid; a.session_id=m.session_id;
    a.accepted=accepted; a.detail=detail; ack_pub_->publish(a);
}
void BenchmarkBridge::finishEnd(const Control& m, const std::string& trajectory_file) {
    try {
        std::map<int64_t,builtin_interfaces::msg::Time> stamps;
        { std::lock_guard<std::mutex> lock(mutex_); stamps=stamps_; }
        auto trajectory=benchmark_tum_reader::read(trajectory_file,"map",stamps,false);
        trajectory_pub_->publish(trajectory);
        ack(m,true,"loop-closed camera trajectory published");
        std::lock_guard<std::mutex> lock(mutex_); ending_=false;
        has_completed_end_=true; completed_end_command_id_=m.command_id;
    } catch (const std::exception& e) {
        { std::lock_guard<std::mutex> lock(mutex_); ending_=false; }
        ack(m,false,e.what());
    }
}
void BenchmarkBridge::rejectEnd(const Control& m,const std::string& detail) {
    { std::lock_guard<std::mutex> lock(mutex_); ending_=false; }
    ack(m,false,detail);
}
