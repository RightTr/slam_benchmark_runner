#include "slam_bench/benchmark_core.hpp"
#include "slam_bench/benchmark_input.hpp"
#include "slam_bench/benchmark_output.hpp"
#include "cli.hpp"
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <nav_msgs/msg/path.hpp>
#include <slam_bench_msgs/msg/benchmark_control.hpp>
#include <slam_bench_msgs/msg/benchmark_control_ack.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/core.hpp>
#include <algorithm>
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdlib>
#include <filesystem>
#include <iomanip>
#include <iostream>
#include <mutex>
#include <sstream>
#include <thread>

using namespace std::chrono_literals;
namespace fs = std::filesystem;
static int64_t steadyNs() {
  return std::chrono::duration_cast<std::chrono::nanoseconds>(
    std::chrono::steady_clock::now().time_since_epoch()).count();
}

class Runner final : public rclcpp::Node {
  enum class Phase { Idle, Preparing, Playing, Paused, WaitingFinalTrajectory, Cancelling, Completed, Incomplete, Cancelled, Error };
  static const char* phaseName(Phase p) {
    switch(p) {
      case Phase::Idle:return "IDLE"; case Phase::Preparing:return "PREPARING";
      case Phase::Playing:return "PLAYING"; case Phase::Paused:return "PAUSED";
      case Phase::WaitingFinalTrajectory:return "WAIT_FINAL_TRAJECTORY";
      case Phase::Cancelling:return "CANCELLING";
      case Phase::Completed:return "COMPLETED"; case Phase::Incomplete:return "INCOMPLETE";
      case Phase::Cancelled:return "CANCELLED"; case Phase::Error:return "ERROR";
    }
    return "UNKNOWN";
  }
 public:
  Runner() : Node("slam_bench_runner") {
    dataset_dir_ = declare_parameter<std::string>("dataset_dir", "");
    dataset_key_file_ = declare_parameter<std::string>("dataset_key_file", "./key/dataset_aes256.key");
    output_dir_ = declare_parameter<std::string>("output_dir", "slam_bench_runs");
    // The release bundle supplies this public key; students only enter their ID.
    teacher_public_key_ = declare_parameter<std::string>("teacher_public_key", "./key/teacher_public.key");
    camera_info_file_ = declare_parameter<std::string>("camera_info_file", "");
    camera_info_topic_ = declare_parameter<std::string>("camera_info_topic", "/benchmark/camera_info");
    publish_camera_info_ = declare_parameter<bool>("publish_camera_info", true);
    rate_scale_ = declare_parameter<double>("rate_scale", 1.0);
    final_trajectory_timeout_ms_ = declare_parameter<int>("final_trajectory_timeout_ms", 60000);
    ack_timeout_ms_ = declare_parameter<int>("ack_timeout_ms", 5000);
    reset_timeout_ms_ = declare_parameter<int>("reset_timeout_ms", 60000);
    if (dataset_dir_.empty() || rate_scale_ <= 0 || final_trajectory_timeout_ms_ <= 0 ||
        ack_timeout_ms_ <= 0 || reset_timeout_ms_ <= 0)
      throw std::runtime_error("set dataset_dir and valid timing parameters");
    fs::create_directories(output_dir_);
    run_uuid_ = slam_bench::BenchmarkCore::generateRunUuid();
    core_.initialize(slam_bench::Mode::Test, dataset_dir_, dataset_key_file_);
    core_.setTeacherPublicKey(teacher_public_key_);
    core_.setFinalTrajectoryTimeoutMs(final_trajectory_timeout_ms_);
    auto image_qos = rclcpp::QoS(rclcpp::KeepLast(5)).reliable();
    rgb_pub_ = create_publisher<sensor_msgs::msg::Image>("/benchmark/rgb", image_qos);
    depth_pub_ = create_publisher<sensor_msgs::msg::Image>("/benchmark/depth", image_qos);
    if (publish_camera_info_)
      camera_info_pub_ = create_publisher<sensor_msgs::msg::CameraInfo>(
        camera_info_topic_, rclcpp::QoS(1).reliable().transient_local());
    control_pub_ = create_publisher<slam_bench_msgs::msg::BenchmarkControl>("/benchmark/control",10);
    ack_sub_ = create_subscription<slam_bench_msgs::msg::BenchmarkControlAck>(
      "/benchmark/control_ack", 10,
      [this](slam_bench_msgs::msg::BenchmarkControlAck::ConstSharedPtr msg) { onAck(*msg); });
    trajectory_sub_ = create_subscription<nav_msgs::msg::Path>("/benchmark/trajectory_final", 1,
      [this](nav_msgs::msg::Path::ConstSharedPtr msg) { onFinalPath(*msg); });
  }
  ~Runner() override { stop(); wait(); }
  void wait() { if (worker_.joinable()) worker_.join(); }
  bool isRunning() const { std::lock_guard<std::mutex> lock(mu_); return running_; }
  std::string takeRunError() {
    std::lock_guard<std::mutex> lock(mu_);
    std::string error=std::move(pending_error_);
    pending_error_.clear();
    return error;
  }
  void setMode(slam_bench::Mode mode, const std::string& student_id = {}) {
    bool join_worker=false;
    bool cancelled_for_mode_switch=false;
    {
      std::lock_guard<std::mutex> lock(mu_);
      if (running_) {
        if (stop_.load()) {
          join_worker=true;
        } else if (phase_==Phase::Preparing || (phase_==Phase::Paused && !playback_started_)) {
          stop_=true;
          phase_=Phase::Cancelling;
          join_worker=true;
          cancelled_for_mode_switch=true;
        } else {
          throw std::runtime_error("session is playing; cancel it before changing mode");
        }
      }
    }
    if (cancelled_for_mode_switch) cv_.notify_all();
    if (join_worker && worker_.joinable()) {
      std::cout << (cancelled_for_mode_switch
        ? "  Stopping session preparation before changing mode...\n"
        : "  Finishing cancellation before changing mode...\n");
      worker_.join();
    }
    std::lock_guard<std::mutex> lock(mu_);
    if (running_) throw std::runtime_error("previous session is still stopping");
    if (mode==slam_bench::Mode::Evaluation) {
      const bool valid_student_id=student_id.size()==13 &&
        std::all_of(student_id.begin(),student_id.end(),[](char c){return c>='0' && c<='9';});
      if (!valid_student_id) throw std::runtime_error("student ID must contain exactly 13 digits");
      student_id_=student_id;
    }
    mode_ = mode; core_.initialize(mode_,dataset_dir_,dataset_key_file_); phase_=Phase::Idle;
    core_.setTeacherPublicKey(teacher_public_key_);
  }
  void select(int id) {
    std::lock_guard<std::mutex> lock(mu_);
    if (running_ || mode_ != slam_bench::Mode::Test) throw std::runtime_error("selection requires idle Test mode");
    core_.selectSequence(id); selected_=id;
  }
  void play() {
    if (mode_==slam_bench::Mode::Evaluation) {
      const bool valid_student_id=student_id_.size()==13 &&
        std::all_of(student_id_.begin(),student_id_.end(),[](char c){return c>='0' && c<='9';});
      if (!valid_student_id)
        throw std::runtime_error("Evaluation requires student_id to be exactly 13 digits");
    }
    {
      std::lock_guard<std::mutex> lock(mu_);
      if (running_) throw std::runtime_error("already running");
      run_uuid_=slam_bench::BenchmarkCore::generateRunUuid();
      running_=true; stop_=false; paused_=false; playback_started_=false; paused_total_ns_=0;
      pending_error_.clear(); phase_=Phase::Preparing;
    }
    if (worker_.joinable()) worker_.join();
    worker_=std::thread([this] { run(); });
  }
  void pause() {
    std::lock_guard<std::mutex> lock(mu_);
    if ((phase_!=Phase::Preparing && phase_!=Phase::Playing) || paused_)
      throw std::runtime_error("pause requires a preparing or playing session");
    paused_=true; pause_started_ns_=steadyNs(); phase_=Phase::Paused; cv_.notify_all();
  }
  void resume() {
    std::lock_guard<std::mutex> lock(mu_);
    if (phase_!=Phase::Paused || !paused_) throw std::runtime_error("resume requires paused playback");
    if (playback_started_) paused_total_ns_+=steadyNs()-pause_started_ns_;
    paused_=false; phase_=playback_started_?Phase::Playing:Phase::Preparing; cv_.notify_all();
  }
  void cancel() {
    {
      std::lock_guard<std::mutex> lock(mu_);
      if (!running_) throw std::runtime_error("no active run to cancel");
      if (stop_) return;
      stop_=true;
      phase_=Phase::Cancelling;
    }
    cv_.notify_all();
  }
  void stop() {
    if (isRunning()) cancel();
  }
  void status() const {
    std::lock_guard<std::mutex> lock(mu_);
    slam_bench::SessionProgress progress{};
    // beginSession currently decrypts/decompresses the archive while holding
    // the Core lock. Keep the interactive status command responsive during it.
    const bool has_progress=playback_started_;
    if (has_progress) progress=output_.completionStatus(steadyNs());
    const auto rgb_count=rgb_pub_->get_subscription_count();
    const auto depth_count=depth_pub_->get_subscription_count();
    const bool connected=rgb_count>0 && depth_count>0;
    const uint64_t total=progress.expected_frames;
    const uint64_t sent=std::min(progress.published_frames,total);
    const int percent=total?static_cast<int>(sent*100/total):0;
    constexpr size_t bar_width=24;
    const size_t filled=total?static_cast<size_t>(percent)*bar_width/100:0;
    const std::string bar=has_progress
      ? "["+std::string(filled,'#')+std::string(bar_width-filled,'-')+"] "+std::to_string(percent)+"%"
      : (running_?"Preparing dataset...":"[------------------------] 0%");
    std::cout << '\n' << slam_bench::cli::paint("  ╭─ Session status ────────────────────────────────────╮", "38;5;45") << '\n';
    auto row=[&](const std::string& label,const std::string& value,const char* color=nullptr) {
      std::cout << "  │ " << std::left << std::setw(11) << label;
      if (color) std::cout << slam_bench::cli::paint(value,color); else std::cout << value;
      const size_t used=15+value.size();
      if (used<56) std::cout << std::string(56-used,' ');
      std::cout << "│\n";
    };
    row("Mode",mode_==slam_bench::Mode::Test?"Test":"Evaluation");
    row("Sequence",mode_==slam_bench::Mode::Test?std::to_string(selected_):"Randomized (six sessions)");
    row("Phase",running_&&stop_.load()?"CANCELLING":phaseName(phase_));
    row("SLAM link",connected?"● connected":"○ waiting",connected?"38;5;82":"38;5;214");
    row("Progress",bar,"38;5;45");
    row("Images",std::to_string(progress.published_frames)+" / "+std::to_string(total)+" published");
    row("Trajectory",std::to_string(progress.received_frames)+" poses received");
    std::cout << slam_bench::cli::paint("  ╰──────────────────────────────────────────────────────╯", "38;5;45") << "\n\n";
  }
 private:
  void send(uint8_t command, uint64_t id) {
    slam_bench_msgs::msg::BenchmarkControl m;
    m.command=command; m.command_id=id; m.run_uuid=run_uuid_; m.session_id=session_id_;
    m.evaluation_mode=(mode_==slam_bench::Mode::Evaluation);
    if (command==slam_bench_msgs::msg::BenchmarkControl::END)
      m.final_image_stamp_ns=expected_final_stamp_;
    control_pub_->publish(m);
  }
  bool awaitAck(uint8_t command, uint64_t id, int timeout_ms) {
    const auto deadline=std::chrono::steady_clock::now()+std::chrono::milliseconds(timeout_ms);
    std::unique_lock<std::mutex> lock(mu_);
    ack_command_=command; ack_id_=id; acked_=false; ack_accepted_=false; ack_detail_.clear();
    while (!stop_ && !acked_ && std::chrono::steady_clock::now()<deadline) {
      lock.unlock(); send(command,id); lock.lock();
      cv_.wait_for(lock,500ms,[this] { return stop_.load() || acked_; });
    }
    if (stop_) return false;
    if (!acked_) return false;
    if (!ack_accepted_)
      throw std::runtime_error("control command rejected: "+ack_detail_);
    return true;
  }
  bool awaitImageSubscribers() {
    const auto deadline=std::chrono::steady_clock::now()+std::chrono::milliseconds(ack_timeout_ms_);
    while (!stop_ && std::chrono::steady_clock::now()<deadline) {
      if (rgb_pub_->get_subscription_count()>0 && depth_pub_->get_subscription_count()>0) return true;
      std::unique_lock<std::mutex> lock(mu_);
      cv_.wait_for(lock,100ms,[this]{return stop_.load();});
    }
    return !stop_ && rgb_pub_->get_subscription_count()>0 && depth_pub_->get_subscription_count()>0;
  }
  void onAck(const slam_bench_msgs::msg::BenchmarkControlAck& m) {
    std::lock_guard<std::mutex> lock(mu_);
    if (m.run_uuid==run_uuid_ && m.session_id==session_id_ && m.command==ack_command_
        && m.command_id==ack_id_) {
      acked_=true; ack_accepted_=m.accepted; ack_detail_=m.detail; cv_.notify_all();
    }
  }
  void onFinalPath(const nav_msgs::msg::Path& m) {
    const int64_t stamp=int64_t(m.header.stamp.sec)*1000000000LL+m.header.stamp.nanosec;
    std::lock_guard<std::mutex> lock(mu_);
    if (stamp!=expected_final_stamp_ || m.poses.empty()) return;
    trajectory_path_=m; trajectory_received_=true;
    cv_.notify_all();
  }
  bool awaitFinal(uint64_t id) {
    std::unique_lock<std::mutex> lock(mu_);
    ack_command_=slam_bench_msgs::msg::BenchmarkControl::END;
    ack_id_=id;acked_=false;ack_accepted_=false;
    lock.unlock();send(slam_bench_msgs::msg::BenchmarkControl::END,id);lock.lock();
    const auto deadline=std::chrono::steady_clock::now()+std::chrono::milliseconds(final_trajectory_timeout_ms_);
    cv_.wait_until(lock,deadline,[this]{return stop_.load() || (acked_ && !ack_accepted_) ||
      (acked_ && trajectory_received_);});
    return !stop_ && acked_ && ack_accepted_ && trajectory_received_;
  }
  sensor_msgs::msg::CameraInfo loadCameraInfo(int sequence_id) const {
    const fs::path file=camera_info_file_.empty()
      ? fs::path(dataset_dir_)/("sequence_0"+std::to_string(sequence_id))/"calibration.yaml"
      : fs::path(camera_info_file_);
    cv::FileStorage settings(file.string(),cv::FileStorage::READ);
    if (!settings.isOpened()) throw std::runtime_error("cannot read camera calibration: "+file.string());
    sensor_msgs::msg::CameraInfo info;
    info.header.frame_id="camera";
    info.distortion_model="plumb_bob";
    int width=0,height=0;
    if (settings["image_width"].empty()) settings["Camera.width"]>>width;
    else settings["image_width"]>>width;
    if (settings["image_height"].empty()) settings["Camera.height"]>>height;
    else settings["image_height"]>>height;
    info.width=static_cast<uint32_t>(std::max(0,width));
    info.height=static_cast<uint32_t>(std::max(0,height));
    cv::Mat k;
    const cv::FileNode camera_matrix=settings["camera_matrix"];
    if (!camera_matrix.empty()) camera_matrix>>k;
    if (k.total()>=9) {
      k=k.reshape(1,1); for(int i=0;i<9;++i) info.k[i]=k.at<double>(0,i);
    } else {
      settings["Camera.fx"]>>info.k[0];settings["Camera.cx"]>>info.k[2];
      settings["Camera.fy"]>>info.k[4];settings["Camera.cy"]>>info.k[5];info.k[8]=1.0;
    }
    info.r[0]=info.r[4]=info.r[8]=1.0;
    info.p[0]=info.k[0];info.p[2]=info.k[2];info.p[5]=info.k[4];info.p[6]=info.k[5];info.p[10]=1.0;
    cv::Mat d;
    const cv::FileNode distortion=settings["distortion_coefficients"];
    if (!distortion.empty()) distortion>>d;
    if (d.empty()) {
      double values[5]{};
      const char* distortion_keys[]={"Camera.k1","Camera.k2","Camera.p1","Camera.p2","Camera.k3"};
      bool has_distortion=false;
      for (const char* key:distortion_keys) has_distortion=has_distortion||!settings[key].empty();
      settings["Camera.k1"]>>values[0];settings["Camera.k2"]>>values[1];
      settings["Camera.p1"]>>values[2];settings["Camera.p2"]>>values[3];settings["Camera.k3"]>>values[4];
      if (has_distortion) for(double v:values) info.d.push_back(v);
    } else { d=d.reshape(1,1);for(int i=0;i<static_cast<int>(d.total());++i) info.d.push_back(d.at<double>(0,i)); }
    if (!info.width||!info.height||info.k[0]<=0||info.k[4]<=0)
      throw std::runtime_error("calibration must define dimensions and positive focal lengths");
    return info;
  }
  static sensor_msgs::msg::Image imageMsg(const cv::Mat& input,int64_t stamp,bool rgb) {
    cv::Mat data;
    if (rgb) cv::cvtColor(input,data,cv::COLOR_BGR2RGB);
    else if (input.type()==CV_32FC1) data=input;
    else if (input.type()==CV_16UC1) input.convertTo(data,CV_32FC1,0.001);
    else throw std::runtime_error("depth must be 16UC1 or 32FC1");
    sensor_msgs::msg::Image msg;
    msg.header.stamp.sec=static_cast<int32_t>(stamp/1000000000LL);
    msg.header.stamp.nanosec=static_cast<uint32_t>(stamp%1000000000LL);
    msg.header.frame_id="camera";msg.height=data.rows;msg.width=data.cols;
    msg.encoding=rgb?"rgb8":"32FC1";msg.is_bigendian=false;
    msg.step=static_cast<uint32_t>(data.cols*data.elemSize());
    msg.data.assign(data.datastart,data.dataend);
    return msg;
  }
  void run() {
    try {
      auto order=core_.sequenceOrder();
      size_t completed_sessions=0;
      for (int seq: order) {
        if (stop_) {
          std::lock_guard<std::mutex> lock(mu_);
          phase_=Phase::Cancelled;
          break;
        }
        core_.selectSequence(seq);
        { std::lock_guard<std::mutex> lock(mu_); selected_=seq; session_id_=slam_bench::BenchmarkCore::generateRunUuid();
          if (completed_sessions>0) { paused_=false;playback_started_=false;paused_total_ns_=0; }
          phase_=paused_?Phase::Paused:Phase::Preparing; }
        core_.beginSession(run_uuid_,session_id_);
        if (camera_info_pub_) camera_info_pub_->publish(loadCameraInfo(seq));
        if (!awaitImageSubscribers()) {
          if (stop_) { output_.cancelSession(); std::lock_guard<std::mutex> lock(mu_); phase_=Phase::Cancelled; break; }
          throw std::runtime_error("timeout waiting for student subscriptions to /benchmark/rgb and /benchmark/depth; start the SLAM node first");
        }
        if (!awaitAck(slam_bench_msgs::msg::BenchmarkControl::READY,0,ack_timeout_ms_)) {
          if (stop_) { output_.cancelSession(); std::lock_guard<std::mutex> lock(mu_); phase_=Phase::Cancelled; break; }
          throw std::runtime_error("READY timeout");
        }
        if (!awaitAck(slam_bench_msgs::msg::BenchmarkControl::RESET,++command_id_,reset_timeout_ms_)) {
          if (stop_) { output_.cancelSession(); std::lock_guard<std::mutex> lock(mu_); phase_=Phase::Cancelled; break; }
          throw std::runtime_error("RESET ACK timeout after "+std::to_string(reset_timeout_ms_)+" ms");
        }
        if (!awaitAck(slam_bench_msgs::msg::BenchmarkControl::BEGIN,++command_id_,ack_timeout_ms_)) {
          if (stop_) { output_.cancelSession(); std::lock_guard<std::mutex> lock(mu_); phase_=Phase::Cancelled; break; }
          throw std::runtime_error("BEGIN timeout");
        }
        { std::lock_guard<std::mutex> lock(mu_); playback_started_=true;paused_total_ns_=0;
          if(paused_){pause_started_ns_=steadyNs();phase_=Phase::Paused;}else phase_=Phase::Playing; }
        auto start=std::chrono::steady_clock::now();
        slam_bench::ImageFrame f;
        int64_t final_depth_end_ns=0;
        while (!stop_ && input_.nextFrame(f)) {
          std::unique_lock<std::mutex> lock(mu_);
          while (!stop_) {
            if (paused_) {
              cv_.wait(lock,[this]{return stop_.load() || !paused_;});
              continue;
            }
            auto target=start+std::chrono::nanoseconds(
              static_cast<int64_t>(f.relative_ns/rate_scale_)+paused_total_ns_);
            if (std::chrono::steady_clock::now()>=target) break;
            cv_.wait_until(lock,target,[this]{return stop_.load() || paused_;});
          }
          if (stop_) break;
          lock.unlock();
          auto rgb=imageMsg(f.rgb,f.stamp_ns,true);
          auto depth=imageMsg(f.depth,f.stamp_ns,false);
          output_.registerFrame(f.frame_id);
          output_.recordRgbPublishBegin(f.frame_id,steadyNs()); rgb_pub_->publish(rgb);
          output_.recordRgbPublishEnd(f.frame_id,steadyNs());
          output_.recordDepthPublishBegin(f.frame_id,steadyNs()); depth_pub_->publish(depth);
          final_depth_end_ns=steadyNs(); output_.recordDepthPublishEnd(f.frame_id,final_depth_end_ns);
        }
        if (stop_) {
          output_.cancelSession();
          { std::lock_guard<std::mutex> lock(mu_); phase_=Phase::Cancelled; }
          break;
        }
        output_.markAllImagesPublished(final_depth_end_ns);
        const auto awaiting=output_.completionStatus(steadyNs());
        { std::lock_guard<std::mutex> lock(mu_);
          phase_=Phase::WaitingFinalTrajectory;
          expected_final_stamp_=awaiting.last_image_stamp_ns;
          trajectory_received_=false; }
        if (!awaitFinal(++command_id_))
          throw std::runtime_error("END export or final trajectories timeout");
        nav_msgs::msg::Path final_path;
        { std::lock_guard<std::mutex> lock(mu_); final_path=trajectory_path_; }
        std::vector<slam_bench::FinalPose> final_poses;
        for (const auto& item:final_path.poses) {
          slam_bench::PoseData p;
          p.px=item.pose.position.x;p.py=item.pose.position.y;p.pz=item.pose.position.z;
          p.qx=item.pose.orientation.x;p.qy=item.pose.orientation.y;
          p.qz=item.pose.orientation.z;p.qw=item.pose.orientation.w;p.valid=true;
          final_poses.push_back({int64_t(item.header.stamp.sec)*1000000000LL+item.header.stamp.nanosec,p});
        }
        output_.recordFinalTrajectory(final_poses);
        output_.finishSession(steadyNs());
        auto progress=output_.completionStatus(steadyNs());
        { std::lock_guard<std::mutex> lock(mu_);
          phase_=progress.state==slam_bench::CompletionState::Complete?Phase::Completed:Phase::Incomplete; }
        std::cout << '\n' << slam_bench::cli::paint("  ✓ Session finalized", "1;38;5;82")
                  << "  " << progress.received_frames << "/" << progress.expected_frames
                  << " poses · " << progress.missing_frames << " missing\n";
        if (!stop_) {
          try {
            auto score=output_.evaluate();
            auto metric = [](double value, int precision) {
              if (!std::isfinite(value)) return std::string("N/A");
              std::ostringstream out; out << std::fixed << std::setprecision(precision) << value;
              return out.str();
            };
            auto row = [&](const std::string& name, double value, int precision, const char* unit) {
              std::cout << "  │ " << std::left << std::setw(23) << name << "│ "
                        << std::right << std::setw(13) << metric(value,precision) << " │ "
                        << std::left << std::setw(6) << unit << " │\n";
            };
            const std::string table_top="  ╭────────────────────────┬───────────────┬────────╮";
            const std::string table_mid="  ├────────────────────────┼───────────────┼────────┤";
            const std::string table_bottom="  ╰────────────────────────┴───────────────┴────────╯";
            std::cout << "\n  Loop-closed camera trajectory results\n"
                      << slam_bench::cli::paint(table_top,"38;5;45") << '\n'
                      << "  │ " << std::left << std::setw(23) << "Metric" << "│ "
                      << std::right << std::setw(13) << "Value" << " │ "
                      << std::left << std::setw(6) << "Unit" << " │\n"
                      << slam_bench::cli::paint(table_mid,"38;5;45") << '\n';
            row("ATE RMSE",score.ate_rmse,8,"m");
            row("RTE RMSE",score.rpe_trans,8,"m");
            row("Rotation RPE",score.rpe_rot,6,"deg");
            row("Tracking",score.tracking_ratio*100.0,2,"%");
            std::cout << slam_bench::cli::paint(table_bottom,"38;5;45") << '\n';
            std::ostringstream sequence_label;
            sequence_label << "sequence_" << std::setw(2) << std::setfill('0') << seq;
            const std::string plot_prefix = mode_==slam_bench::Mode::Test
                ? sequence_label.str()+"_" : std::string{};
            const auto plot2d=fs::path(output_dir_)/(plot_prefix+session_id_+"_2d.png");
            const auto plot3d=fs::path(output_dir_)/(plot_prefix+session_id_+"_3d.png");
            output_.saveTrajectory2D(plot2d.string());
            output_.saveTrajectory3D(plot3d.string());
            std::cout << std::flush;
          } catch(const std::exception& e) { throw std::runtime_error(std::string("Scoring or plot failed: ")+e.what()); }
        }
        if (mode_==slam_bench::Mode::Test)
          std::cout << "  ✓ Sequence " << (seq<10?"0":"") << seq
                    << (progress.state==slam_bench::CompletionState::Complete
                        ? " completed · Returning to main menu" : " incomplete");
        else
          std::cout << "  ✓ Session"
                    << (progress.state==slam_bench::CompletionState::Complete?" completed":" incomplete");
        std::cout << '\n';
        if (progress.state!=slam_bench::CompletionState::Complete) break;
        ++completed_sessions;
      }
      if (mode_==slam_bench::Mode::Evaluation && completed_sessions==6 && !stop_) {
        const auto path=fs::path(output_dir_)/("submission_"+student_id_+".bin");
        core_.saveEvaluationBin(path.string(),student_id_);
        std::cout<<"Evaluation BIN: "<<path<<'\n';
      }
    } catch (const std::exception& e) {
      core_.cancelSession();
      {
        std::lock_guard<std::mutex> lock(mu_);
        phase_=stop_?Phase::Cancelled:Phase::Error;
        if (!stop_) pending_error_=e.what();
      }
    }
    if (stop_) {
      std::lock_guard<std::mutex> lock(mu_);
      phase_=Phase::Cancelled;
    }
    { std::lock_guard<std::mutex> lock(mu_); running_=false; }
  }
  std::string dataset_dir_,dataset_key_file_,output_dir_,student_id_,teacher_public_key_,camera_info_file_,camera_info_topic_,run_uuid_,session_id_;
  bool publish_camera_info_{true};
  slam_bench::BenchmarkCore core_; slam_bench::BenchmarkInput input_{core_};
  slam_bench::BenchmarkOutput output_{core_}; slam_bench::Mode mode_{slam_bench::Mode::Test};
  double rate_scale_{1};int final_trajectory_timeout_ms_{60000},ack_timeout_ms_{5000},reset_timeout_ms_{60000},selected_{1};
  uint64_t command_id_{0},ack_id_{0}; uint8_t ack_command_{0};bool acked_{false},ack_accepted_{false},running_{false};
  std::string ack_detail_;
  int64_t expected_final_stamp_{0}; bool trajectory_received_{false};
  nav_msgs::msg::Path trajectory_path_;
  Phase phase_{Phase::Idle}; bool paused_{false},playback_started_{false}; int64_t pause_started_ns_{0},paused_total_ns_{0};
  std::atomic<bool> stop_{false};mutable std::mutex mu_;std::condition_variable cv_;std::thread worker_;
  std::string pending_error_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr rgb_pub_,depth_pub_;
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_pub_;
  rclcpp::Publisher<slam_bench_msgs::msg::BenchmarkControl>::SharedPtr control_pub_;
  rclcpp::Subscription<slam_bench_msgs::msg::BenchmarkControlAck>::SharedPtr ack_sub_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr trajectory_sub_;
};
int main(int argc,char** argv) {
  rclcpp::init(argc,argv);std::shared_ptr<Runner> runner;
  try { runner=std::make_shared<Runner>(); }
  catch(const std::exception& e){std::cerr<<e.what()<<'\n';rclcpp::shutdown();return 2;}
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(runner);
  std::thread spin([&]{executor.spin();});
  slam_bench::cli::Actions actions;
  actions.test=[runner]{runner->setMode(slam_bench::Mode::Test);};
  actions.evaluation=[runner](const std::string& student_id){runner->setMode(slam_bench::Mode::Evaluation,student_id);};
  actions.select=[runner](int id){runner->select(id);};
  actions.play=[runner]{runner->play();};
  actions.pause=[runner]{runner->pause();};
  actions.resume=[runner]{runner->resume();};
  actions.cancel=[runner]{runner->cancel();};
  actions.status=[runner]{runner->status();};
  actions.is_running=[runner]{return runner->isRunning();};
  actions.take_error=[runner]{return runner->takeRunError();};
  slam_bench::cli::Console console(std::move(actions));
  const int result=console.run();
  runner->stop();runner->wait();
  executor.cancel();
  if(spin.joinable()) spin.join();
  executor.remove_node(runner);
  runner.reset();
  rclcpp::shutdown();
  return result;
}
