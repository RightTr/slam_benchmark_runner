#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/synchronizer.h>
/**
* This file is part of ORB-SLAM3
*
* Copyright (C) 2017-2021 Carlos Campos, Richard Elvira, Juan J. Gómez Rodríguez, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
* Copyright (C) 2014-2016 Raúl Mur-Artal, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
*
* ORB-SLAM3 is free software: you can redistribute it and/or modify it under the terms of the GNU General Public
* License as published by the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM3 is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even
* the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License along with ORB-SLAM3.
* If not, see <http://www.gnu.org/licenses/>.
*/


#include<iostream>
#include<fstream>

#include <cv_bridge/cv_bridge.h>
#include <Eigen/Geometry>
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <functional>
#include <string>
#include <chrono>
#include <condition_variable>

#include<opencv2/core/core.hpp>

#include <System.h>
#include"../include/sensor_sync.hpp"
#include"../include/benchmark_bridge.hpp"
#include<memory>
#include<thread>
#include<filesystem>
#include<regex>
#include<mutex>

using namespace std;

static cv::FileNode settingsNode(cv::FileStorage& settings, const std::string& name)
{
    cv::FileNode node = settings[name];
    if (!node.empty()) return node;
    const std::size_t separator = name.find('.');
    if (separator == std::string::npos) return node;
    const cv::FileNode section = settings[name.substr(0, separator)];
    if (section.empty()) return node;
    return section[name.substr(separator + 1)];
}

class ImageGrabber
{
public:
    ImageGrabber(ORB_SLAM3::System* pSLAM, const rclcpp::Node::SharedPtr& nh, const std::string& settings,
                 bool offline, int img_size, std::function<void(uint64_t)> progress):mpSLAM(pSLAM),
                 progress_(std::move(progress)),
                 sync_queue(new SensorSynchronizer<std::pair<sensor_msgs::msg::Image::ConstSharedPtr,sensor_msgs::msg::Image::ConstSharedPtr>>(offline,img_size))
    {
        rgbd_sync_tolerance = nh->get_parameter("rgbd_sync_tolerance").as_double();
        cv::FileStorage fs(settings, cv::FileStorage::READ);
        if (fs.isOpened()) {
            settingsNode(fs, "Camera.width") >> processing_width;
            settingsNode(fs, "Camera.height") >> processing_height;
            input_width=processing_width; input_height=processing_height;
            cv::FileNode new_width = settingsNode(fs, "Camera.newWidth");
            cv::FileNode new_height = settingsNode(fs, "Camera.newHeight");
            if (!new_width.empty()) new_width >> processing_width;
            if (!new_height.empty()) new_height >> processing_height;
        }
    }

    void GrabRGBD(const sensor_msgs::msg::Image::ConstSharedPtr& msgRGB,const sensor_msgs::msg::Image::ConstSharedPtr& msgD);
    void ProcessFrames();
    void TrackRGBDFrame(const sensor_msgs::msg::Image::ConstSharedPtr& msgRGB,const sensor_msgs::msg::Image::ConstSharedPtr& msgD);

    ORB_SLAM3::System* mpSLAM;
    std::function<void(uint64_t)> progress_;
    std::unique_ptr<SensorSynchronizer<std::pair<sensor_msgs::msg::Image::ConstSharedPtr,sensor_msgs::msg::Image::ConstSharedPtr>>> sync_queue;
    double rgbd_sync_tolerance{0.02};
    int processing_width{0}, processing_height{0};
    int input_width{0}, input_height{0};
};

// Owns the independent ORB-SLAM3 instance and the two worker lifetimes for a
// benchmark Session. The ROS callbacks only delegate RESET/END to this object.
class BenchmarkSession {
public:
    void reset(const rclcpp::Node::SharedPtr& node, const std::string& vocabulary,
               const std::string& settings, std::function<void(uint64_t)> progress) {
        joinFinalizer();
        stopActive();
        if (grabber_) grabber_->sync_queue->Stop();
        tracker_ = std::thread{};
        grabber_.reset();
        // Destroy the completed System and its viewer before creating the next
        // map; do not overlap Pangolin or backend state across Sessions.
        slam_.reset();
        finalized_=true;
        RCLCPP_INFO(rclcpp::get_logger("rgbd"),
                    "Previous ORB-SLAM3 System, viewer, and RGB-D queue cleared");
        slam_=std::make_unique<ORB_SLAM3::System>(vocabulary,settings,ORB_SLAM3::System::RGBD,
            node->get_parameter("use_pangolin").as_bool());
        grabber_=std::make_shared<ImageGrabber>(slam_.get(),node,settings,true,
            node->get_parameter("img_size").as_int(),std::move(progress));
        tracker_=std::thread([g=grabber_]{g->ProcessFrames();});
        finalized_=false;
    }
    void cancel() {
        if (grabber_ && !finalized_) {
            // Drop queued RGB-D frames immediately. The current TrackRGBD call
            // may finish, but no backlog will be drained before the next RESET.
            grabber_->sync_queue->Stop();
            if (!finisher_.joinable() && slam_) {
                auto* system = slam_.get();
                finisher_ = std::thread([this, system]() {
                    try {
                        if (tracker_.joinable()) tracker_.join();
                        if (!finalized_) {
                            system->Shutdown();
                            finalized_ = true;
                        }
                        grabber_.reset();
                        slam_.reset();
                        RCLCPP_INFO(rclcpp::get_logger("rgbd"),
                                    "Cancelled Session state released; wrapper is ready for RESET");
                    } catch (const std::exception& e) {
                        RCLCPP_ERROR(rclcpp::get_logger("rgbd"),
                                     "Cancelled Session cleanup failed: %s", e.what());
                    }
                });
            }
            RCLCPP_INFO(rclcpp::get_logger("rgbd"),
                        "Session cancelled; queued RGB-D frames discarded, backend cleanup running in background");
        }
    }
    std::shared_ptr<ImageGrabber> grabber() const { return grabber_; }
    void finish(BenchmarkBridge::Control command, const std::filesystem::path& output_dir,
                BenchmarkBridge* bridge) {
        if (!grabber_ || !slam_ || finalized_) throw std::runtime_error("no active SLAM Session");
        if (finisher_.joinable()) throw std::runtime_error("Session finalization already running");
        auto grabber=grabber_;
        auto* system=slam_.get();
        finisher_=std::thread([this,grabber,system,command,output_dir,bridge]() {
            try {
                if (!bridge->waitForInputStamp(command.final_image_stamp_ns,std::chrono::seconds(10)))
                    throw std::runtime_error("final RGB-D pair did not arrive before input drain timeout");
                grabber->sync_queue->CloseInput();
                if (tracker_.joinable()) tracker_.join();
                RCLCPP_INFO(rclcpp::get_logger("rgbd"),
                            "All input frames tracked; waiting for local mapping and loop closing to finish");
                std::mutex shutdown_mutex;
                std::condition_variable shutdown_cv;
                bool shutdown_complete=false;
                const auto shutdown_started=std::chrono::steady_clock::now();
                std::thread shutdown_status([&]() {
                    std::unique_lock<std::mutex> lock(shutdown_mutex);
                    while (!shutdown_cv.wait_for(lock,std::chrono::seconds(15),
                                                 [&]() { return shutdown_complete; })) {
                        const auto elapsed=std::chrono::duration_cast<std::chrono::seconds>(
                            std::chrono::steady_clock::now()-shutdown_started).count();
                        lock.unlock();
                        RCLCPP_INFO(rclcpp::get_logger("rgbd"),
                            "SLAM backend is still finalizing local mapping / loop closure / global BA (%ld s)",
                            static_cast<long>(elapsed));
                        lock.lock();
                    }
                });
                try {
                    system->Shutdown();
                } catch (...) {
                    { std::lock_guard<std::mutex> lock(shutdown_mutex); shutdown_complete=true; }
                    shutdown_cv.notify_all();
                    shutdown_status.join();
                    throw;
                }
                { std::lock_guard<std::mutex> lock(shutdown_mutex); shutdown_complete=true; }
                shutdown_cv.notify_all();
                shutdown_status.join();
                finalized_=true;
                RCLCPP_INFO(rclcpp::get_logger("rgbd"),
                            "SLAM backend stopped; exporting the final camera trajectory");
                const auto trajectory=(output_dir/(command.session_id+"_trajectory.txt")).string();
                system->SaveTrajectoryTUM(trajectory);
                bridge->finishEnd(command,trajectory);
                finalized_=true;
                grabber_.reset();
                slam_.reset();
            } catch(const std::exception& e) {
                try {
                    grabber->sync_queue->CloseInput();
                    if (tracker_.joinable()) tracker_.join();
                    if (!finalized_) { system->Shutdown(); finalized_=true; }
                } catch (...) {}
                bridge->rejectEnd(command,e.what());
                grabber_.reset();
                slam_.reset();
            }
        });
    }
    void shutdown() { joinFinalizer(); stopActive(); }
    ~BenchmarkSession() { shutdown(); }
private:
    void joinFinalizer() { if (finisher_.joinable()) finisher_.join(); }
    void stopActive() {
        if (!grabber_ || finalized_) return;
        grabber_->sync_queue->CloseInput();
        if (tracker_.joinable()) tracker_.join();
        if (slam_) slam_->Shutdown();
        finalized_=true;
    }
    std::unique_ptr<ORB_SLAM3::System> slam_;
    std::shared_ptr<ImageGrabber> grabber_;
    std::thread tracker_,finisher_;
    bool finalized_{true};
};

static int runBenchmark(const rclcpp::Node::SharedPtr& nh,
                        const std::string& vocabulary, const std::string& settings)
{
    namespace fs = std::filesystem;
    nh->declare_parameter<std::string>("benchmark_output_dir", "benchmark_trajectories");
    nh->declare_parameter<std::string>("camera_info_topic", "/benchmark/camera_info");
    const fs::path output_dir=nh->get_parameter("benchmark_output_dir").as_string();
    fs::create_directories(output_dir);
    std::mutex camera_info_mutex;
    sensor_msgs::msg::CameraInfo::ConstSharedPtr camera_info;
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_sub;
    camera_info_sub = nh->create_subscription<sensor_msgs::msg::CameraInfo>(
            nh->get_parameter("camera_info_topic").as_string(),
            rclcpp::QoS(1).reliable().transient_local(),
            [&](sensor_msgs::msg::CameraInfo::ConstSharedPtr msg) {
                if (msg->k[0] <= 0.0 || msg->k[4] <= 0.0 || msg->width == 0 || msg->height == 0) {
                    RCLCPP_WARN(nh->get_logger(), "Ignoring invalid CameraInfo calibration");
                    return;
                }
                std::lock_guard<std::mutex> lock(camera_info_mutex);
                camera_info = std::move(msg);
            });
    BenchmarkSession session;
    BenchmarkBridge* bridge_ptr=nullptr;
    auto reset=[&]() {
        RCLCPP_INFO(nh->get_logger(), "RESET: loading CameraInfo and preparing session settings");
        std::string session_settings=settings;
            sensor_msgs::msg::CameraInfo::ConstSharedPtr info;
            { std::lock_guard<std::mutex> lock(camera_info_mutex); info=camera_info; }
            if (!info) throw std::runtime_error("valid CameraInfo required on camera_info_topic before RESET");
            RCLCPP_INFO(nh->get_logger(), "RESET: CameraInfo received (%ux%u), writing session calibration",
                        info->width, info->height);
            std::ifstream source(settings);
            if (!source) throw std::runtime_error("cannot read ORB-SLAM3 settings: "+settings);
            std::string content((std::istreambuf_iterator<char>(source)), {});
            std::vector<std::pair<std::string,double>> values={
                {"Camera.fx",info->k[0]}, {"Camera.fy",info->k[4]},
                {"Camera.cx",info->k[2]}, {"Camera.cy",info->k[5]},
                {"Camera.width",static_cast<double>(info->width)},
                {"Camera.height",static_cast<double>(info->height)}};
            const char* distortion_keys[]={"Camera.k1","Camera.k2","Camera.p1","Camera.p2","Camera.k3"};
            for (size_t i=0;i<std::min<size_t>(5,info->d.size());++i)
                values.emplace_back(distortion_keys[i],info->d[i]);
            for (const auto& entry:values) {
                const bool integer_field=entry.first=="Camera.width"||entry.first=="Camera.height";
                const std::string value=integer_field
                    ? std::to_string(static_cast<uint32_t>(entry.second)) : std::to_string(entry.second);
                const std::regex key_line("^([ \\t]*"+entry.first+"[ \\t]*:[ \\t]*).*$");
                std::istringstream input(content);
                std::ostringstream updated;
                std::string line;
                bool found=false;
                while (std::getline(input,line)) {
                    std::smatch match;
                    if (std::regex_match(line,match,key_line)) { line=match[1].str()+value; found=true; }
                    updated<<line<<'\n';
                }
                if (!found) updated<<entry.first<<": "<<value<<'\n';
                content=updated.str();
            }
            static uint64_t settings_index=0;
            session_settings=(output_dir/("settings_session_"+std::to_string(++settings_index)+".yaml")).string();
            std::ofstream generated(session_settings,std::ios::trunc);
            generated<<content;
            generated.close();
            if (!generated) throw std::runtime_error("cannot write generated CameraInfo settings");
        RCLCPP_INFO(nh->get_logger(), "RESET: constructing ORB-SLAM3 System (vocabulary loading may take a while)");
        session.reset(nh,vocabulary,session_settings,[&](uint64_t processed) {
            if (bridge_ptr) bridge_ptr->publishProgress(processed);
        });
        RCLCPP_INFO(nh->get_logger(), "RESET: ORB-SLAM3 System is ready");
    };
    auto cancel=[&]() { session.cancel(); };
    auto end=[&](BenchmarkBridge::Control command) { session.finish(command,output_dir,bridge_ptr); };
    BenchmarkBridge bridge(nh,reset,cancel,end);
    bridge_ptr=&bridge;
    const std::string rgb_topic="/benchmark/rgb";
    const std::string depth_topic="/benchmark/depth";
    rmw_qos_profile_t benchmark_qos = rmw_qos_profile_default;
    benchmark_qos.history = RMW_QOS_POLICY_HISTORY_KEEP_LAST;
    benchmark_qos.depth = 10;
    benchmark_qos.reliability = RMW_QOS_POLICY_RELIABILITY_RELIABLE;
    benchmark_qos.durability = RMW_QOS_POLICY_DURABILITY_VOLATILE;
    message_filters::Subscriber<sensor_msgs::msg::Image> rgb_sub(nh.get(),rgb_topic,benchmark_qos);
    message_filters::Subscriber<sensor_msgs::msg::Image> depth_sub(nh.get(),depth_topic,benchmark_qos);
    using Policy=message_filters::sync_policies::ExactTime<sensor_msgs::msg::Image,sensor_msgs::msg::Image>;
    message_filters::Synchronizer<Policy> sync(Policy(10),rgb_sub,depth_sub);
    auto onImages=[&](sensor_msgs::msg::Image::ConstSharedPtr rgb,
                       sensor_msgs::msg::Image::ConstSharedPtr depth) {
        auto grabber=session.grabber();
        if (bridge.accepting() && grabber) {
            grabber->GrabRGBD(rgb,depth);
            bridge.registerInputStamp(rgb->header.stamp);
        }
    };
    sync.registerCallback(std::bind(onImages,std::placeholders::_1,std::placeholders::_2));
    rclcpp::spin(nh);
    session.shutdown();
    rclcpp::shutdown();
    return 0;
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto nh = std::make_shared<rclcpp::Node>("rgbd");
    nh->declare_parameter<std::string>("vocabulary", "");
    nh->declare_parameter<std::string>("settings", "");
    const std::string vocabulary = nh->get_parameter("vocabulary").as_string();
    const std::string settings = nh->get_parameter("settings").as_string();
    if (vocabulary.empty() || settings.empty()) {
        RCLCPP_ERROR(nh->get_logger(), "Both vocabulary and settings parameters are required");
        return 1;
    }
    nh->declare_parameter<bool>("use_pangolin", true);
    nh->declare_parameter<double>("rgbd_sync_tolerance", 0.02);
    nh->declare_parameter<int>("img_size", 3);
    return runBenchmark(nh,vocabulary,settings);
}

void ImageGrabber::GrabRGBD(const sensor_msgs::msg::Image::ConstSharedPtr& msgRGB,const sensor_msgs::msg::Image::ConstSharedPtr& msgD)
{
    const double trgb=rclcpp::Time(msgRGB->header.stamp).seconds(), td=rclcpp::Time(msgD->header.stamp).seconds();
    if(std::abs(trgb-td)>rgbd_sync_tolerance) return;
    sync_queue->PushFrame(trgb,{msgRGB,msgD});
}

void ImageGrabber::ProcessFrames()
{
    uint64_t processed_frames = 0;
    while(rclcpp::ok()) {
        std::pair<sensor_msgs::msg::Image::ConstSharedPtr,sensor_msgs::msg::Image::ConstSharedPtr> frame;
        double timestamp=-1.0;
        if(!sync_queue->WaitFrame(timestamp,frame)) return;
        try { TrackRGBDFrame(frame.first,frame.second); }
        catch (const std::exception& e) {
            RCLCPP_ERROR(rclcpp::get_logger("rgbd"), "Frame processing failed: %s",e.what());
        }
        if (progress_) progress_(++processed_frames);
    }
}

void ImageGrabber::TrackRGBDFrame(const sensor_msgs::msg::Image::ConstSharedPtr& msgRGB,const sensor_msgs::msg::Image::ConstSharedPtr& msgD)
{
        const auto cv_ptrRGB = cv_bridge::toCvShare(msgRGB);
        const auto cv_ptrD = cv_bridge::toCvShare(msgD);
        if (cv_ptrRGB->image.size() != cv_ptrD->image.size()) {
            RCLCPP_ERROR(rclcpp::get_logger("rgbd"), "RGB and depth dimensions differ; registered depth is required");
            return;
        }
        if ((input_width>0 && cv_ptrRGB->image.cols!=input_width) ||
            (input_height>0 && cv_ptrRGB->image.rows!=input_height)) {
            RCLCPP_ERROR(rclcpp::get_logger("rgbd"), "RGB-D resolution does not match CameraInfo/config calibration");
            return;
        }
        (void)mpSLAM->TrackRGBD(cv_ptrRGB->image,cv_ptrD->image,
            rclcpp::Time(cv_ptrRGB->header.stamp).seconds());
}
