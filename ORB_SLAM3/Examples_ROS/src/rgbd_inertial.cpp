#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/path.hpp>
#include <std_msgs/msg/u_int8.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
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
#include<algorithm>
#include<fstream>

#include <cv_bridge/cv_bridge.h>
#include <Eigen/Geometry>
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <string>
#include <thread>
#include <vector>

#include<opencv2/core/core.hpp>
#include <opencv2/imgproc.hpp>

#include <System.h>
#include <ImuTypes.h>
#include"../include/sensor_sync.hpp"
#include"../include/ros_vis.hpp"

using namespace std;

class ImageGrabber
{
public:
    ImageGrabber(ORB_SLAM3::System* pSLAM, const rclcpp::Node::SharedPtr& nh,
                 const std::string& settings):mpSLAM(pSLAM)
    {
        pose_pub = nh->create_publisher<geometry_msgs::msg::PoseStamped>("/orbslam3/pose", 10);
        path_pub = nh->create_publisher<nav_msgs::msg::Path>("/orbslam3/path", 10);
        state_pub = nh->create_publisher<std_msgs::msg::UInt8>("/orbslam3/tracking_state", 10);
        tf_broadcaster = std::make_shared<tf2_ros::TransformBroadcaster>(nh);
        publish_tf = nh->get_parameter("publish_tf").as_bool();
        map_frame = nh->get_parameter("map_frame").as_string();
        camera_frame = nh->get_parameter("camera_frame").as_string();
        imu_size = nh->get_parameter("imu_size").as_int();
        img_size = nh->get_parameter("img_size").as_int();
        rgbd_sync_tolerance = nh->get_parameter("rgbd_sync_tolerance").as_double();
        std::string queue_mode = nh->get_parameter("queue_mode").as_string();
        sync_queue.reset(new SensorSynchronizer<Pair>(queue_mode == "offline", img_size,
                                                      imu_size));
        cv::FileStorage fs(settings, cv::FileStorage::READ);
        if (fs.isOpened()) {
            fs["Camera.width"] >> processing_width;
            fs["Camera.height"] >> processing_height;
            if (!fs["Camera.newWidth"].empty()) fs["Camera.newWidth"] >> processing_width;
            if (!fs["Camera.newHeight"].empty()) fs["Camera.newHeight"] >> processing_height;
        }
        visualization.reset(new RosTrackingVisualization(mpSLAM, nh, map_frame,
                                                         processing_width, processing_height, 0.1));
    }

    void GrabRGBD(const sensor_msgs::msg::Image::ConstSharedPtr& msgRGB,const sensor_msgs::msg::Image::ConstSharedPtr& msgD);
    void GrabImu(const sensor_msgs::msg::Imu::ConstSharedPtr& msg);
    void ProcessFrames();
    void StopProcessing();
    bool TrackPair(const sensor_msgs::msg::Image::ConstSharedPtr& msgRGB,const sensor_msgs::msg::Image::ConstSharedPtr& msgD,
                   const std::vector<ORB_SLAM3::IMU::Point>& imu);

    ORB_SLAM3::System* mpSLAM;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub;
    rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr state_pub;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster;
    nav_msgs::msg::Path path;
    std::string map_frame, camera_frame;
    bool publish_tf;
    struct Pair { sensor_msgs::msg::Image::ConstSharedPtr rgb, depth; };
    std::unique_ptr<SensorSynchronizer<Pair>> sync_queue;
    double rgbd_sync_tolerance{0.02};
    int imu_size, img_size;
    int processing_width{0}, processing_height{0};
    std::unique_ptr<RosTrackingVisualization> visualization;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto nh = std::make_shared<rclcpp::Node>("rgbd_inertial");
    nh->declare_parameter<std::string>("vocabulary", "");
    nh->declare_parameter<std::string>("settings", "");
    const std::string vocabulary = nh->get_parameter("vocabulary").as_string();
    const std::string settings = nh->get_parameter("settings").as_string();
    if (vocabulary.empty() || settings.empty()) {
        RCLCPP_ERROR(nh->get_logger(), "Both vocabulary and settings parameters are required");
        return 1;
    }
    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    nh->declare_parameter<bool>("use_pangolin", true);
    nh->declare_parameter<bool>("publish_tf", false);
    nh->declare_parameter<std::string>("map_frame", "map");
    nh->declare_parameter<std::string>("camera_frame", "camera_link");
    nh->declare_parameter<int>("imu_size", 5000);
    nh->declare_parameter<int>("img_size", 30);
    nh->declare_parameter<double>("rgbd_sync_tolerance", 0.02);
    nh->declare_parameter<std::string>("queue_mode", "offline");
    nh->declare_parameter<std::string>("rgb_topic", "/camera/rgb/image_raw");
    nh->declare_parameter<std::string>("depth_topic", "/camera/depth_registered/image_raw");
    nh->declare_parameter<std::string>("imu_topic", "/imu");
    ORB_SLAM3::System SLAM(vocabulary,settings,ORB_SLAM3::System::IMU_RGBD,
                           nh->get_parameter("use_pangolin").as_bool());

    ImageGrabber igb(&SLAM, nh, settings);

    std::string rgb_topic, depth_topic, imu_topic;
    rgb_topic = nh->get_parameter("rgb_topic").as_string();
    depth_topic = nh->get_parameter("depth_topic").as_string();
    imu_topic = nh->get_parameter("imu_topic").as_string();
    message_filters::Subscriber<sensor_msgs::msg::Image> rgb_sub(nh.get(), rgb_topic, rmw_qos_profile_sensor_data);
    message_filters::Subscriber<sensor_msgs::msg::Image> depth_sub(nh.get(), depth_topic, rmw_qos_profile_sensor_data);
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::Image, sensor_msgs::msg::Image> sync_pol;
    message_filters::Synchronizer<sync_pol> sync(sync_pol(5), rgb_sub,depth_sub);
    sync.registerCallback(std::bind(&ImageGrabber::GrabRGBD, &igb, std::placeholders::_1, std::placeholders::_2));
    auto imu_sub = nh->create_subscription<sensor_msgs::msg::Imu>(imu_topic, 2000,
        [&igb](sensor_msgs::msg::Imu::ConstSharedPtr msg) { igb.GrabImu(msg); });
    std::thread tracking_thread(&ImageGrabber::ProcessFrames, &igb);

    rclcpp::spin(nh);

    igb.StopProcessing();
    tracking_thread.join();

    // Stop all threads
    SLAM.Shutdown();

    // Save camera trajectory
    SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");

    rclcpp::shutdown();

    return 0;
}

void ImageGrabber::GrabRGBD(const sensor_msgs::msg::Image::ConstSharedPtr& msgRGB,const sensor_msgs::msg::Image::ConstSharedPtr& msgD)
{
    {
        const double trgb = rclcpp::Time(msgRGB->header.stamp).seconds();
        const double tdepth = rclcpp::Time(msgD->header.stamp).seconds();
        if (std::abs(trgb - tdepth) > rgbd_sync_tolerance) {
            return;
        }
        sync_queue->PushFrame(trgb, Pair{msgRGB, msgD});
    }
}

void ImageGrabber::GrabImu(const sensor_msgs::msg::Imu::ConstSharedPtr& msg)
{
    const double t = rclcpp::Time(msg->header.stamp).seconds();
    sync_queue->PushImu(ORB_SLAM3::IMU::Point(
        cv::Point3f(msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z),
        cv::Point3f(msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z), t));
}

void ImageGrabber::StopProcessing()
{
    sync_queue->Stop();
}

void ImageGrabber::ProcessFrames()
{
    while (true) {
        double timestamp = -1.0;
        Pair frame;
        std::vector<ORB_SLAM3::IMU::Point> measurements;
        if (!sync_queue->WaitNext(timestamp, frame, measurements)) return;
        if (!TrackPair(frame.rgb, frame.depth, measurements)) {
            sync_queue->Stop();
            return;
        }
    }
}

bool ImageGrabber::TrackPair(const sensor_msgs::msg::Image::ConstSharedPtr& msgRGB,const sensor_msgs::msg::Image::ConstSharedPtr& msgD,
                             const std::vector<ORB_SLAM3::IMU::Point>& imu)
{
    // Copy the ros image message to cv::Mat.
    const cv_bridge::CvImageConstPtr cv_ptrRGB = cv_bridge::toCvShare(msgRGB);
    const cv_bridge::CvImageConstPtr cv_ptrD = cv_bridge::toCvShare(msgD);
    if (cv_ptrRGB->image.size() != cv_ptrD->image.size()) {
        RCLCPP_ERROR(rclcpp::get_logger("rgbd"), "RGB and depth dimensions differ; registered depth is required");
        return false;
    }
    const Sophus::SE3f Tcw = mpSLAM->TrackRGBD(cv_ptrRGB->image,cv_ptrD->image,rclcpp::Time(cv_ptrRGB->header.stamp).seconds(),imu);
    std_msgs::msg::UInt8 state;
    state.data = static_cast<uint8_t>(mpSLAM->GetTrackingState());
    state_pub->publish(state);
    const std::vector<cv::KeyPoint> keypoints = mpSLAM->GetTrackedKeyPointsUn();
    const std::vector<ORB_SLAM3::MapPoint*> map_points = mpSLAM->GetTrackedMapPoints();
    std::vector<uint8_t> associated(keypoints.size(), 0);
    for (size_t i = 0; i < std::min(keypoints.size(), map_points.size()); ++i)
        associated[i] = map_points[i] != nullptr;
    visualization->PublishTrackedImage(msgRGB, state.data, keypoints, associated);
    if (state.data != 2) return true;
    const Sophus::SE3f Twc = Tcw.inverse();
    const Eigen::Vector3f t = Twc.translation();
    const Eigen::Quaternionf q(Twc.rotationMatrix());
    geometry_msgs::msg::PoseStamped pose;
    pose.header = msgRGB->header;
    pose.header.frame_id = map_frame;
    pose.pose.position.x=t.x(); pose.pose.position.y=t.y(); pose.pose.position.z=t.z();
    pose.pose.orientation.x=q.x(); pose.pose.orientation.y=q.y(); pose.pose.orientation.z=q.z(); pose.pose.orientation.w=q.w();
    pose_pub->publish(pose);
    if (path.header.frame_id != map_frame) { path.poses.clear(); path.header.frame_id = map_frame; }
    path.header.stamp = pose.header.stamp;
    path.poses.push_back(pose);
    if (path.poses.size() > 100000) path.poses.erase(path.poses.begin(), path.poses.begin() + 1000);
    path_pub->publish(path);
    if (publish_tf) {
        geometry_msgs::msg::TransformStamped tf;
        tf.header = pose.header; tf.child_frame_id = camera_frame;
        tf.transform.translation.x=t.x(); tf.transform.translation.y=t.y(); tf.transform.translation.z=t.z();
        tf.transform.rotation = pose.pose.orientation;
        tf_broadcaster->sendTransform(tf);
    }
    visualization->PublishMapVisualization(msgRGB);
    return true;
}

