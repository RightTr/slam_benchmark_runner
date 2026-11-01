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
#include<chrono>
#include<vector>
#include<thread>

#include<cv_bridge/cv_bridge.h>

#include<opencv2/core/core.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>

#include <System.h>
#include <ImuTypes.h>
#include"../include/sensor_sync.hpp"
#include<memory>

using namespace std;
using StereoFrame = std::pair<sensor_msgs::msg::Image::ConstSharedPtr, sensor_msgs::msg::Image::ConstSharedPtr>;

class ImuGrabber
{
public:
    explicit ImuGrabber(SensorSynchronizer<std::pair<sensor_msgs::msg::Image::ConstSharedPtr, sensor_msgs::msg::Image::ConstSharedPtr>>* queue)
        : sync_queue(queue) {}
    void GrabImu(const sensor_msgs::msg::Imu::ConstSharedPtr &imu_msg);
    SensorSynchronizer<std::pair<sensor_msgs::msg::Image::ConstSharedPtr, sensor_msgs::msg::Image::ConstSharedPtr>>* sync_queue;
};

class ImageGrabber
{
public:
    ImageGrabber(ORB_SLAM3::System* pSLAM, ImuGrabber *pImuGb,
                 SensorSynchronizer<std::pair<sensor_msgs::msg::Image::ConstSharedPtr, sensor_msgs::msg::Image::ConstSharedPtr>>* queue,
                 const bool offline_mode, int frame_limit, const bool bRect, const bool bClahe,
                 double stereo_tolerance): mpSLAM(pSLAM), mpImuGb(pImuGb), sync_queue(queue),
                 stereo_sync(*queue, offline_mode, frame_limit, stereo_tolerance),
                 do_rectify(bRect), mbClahe(bClahe){}

    void GrabImageLeft(const sensor_msgs::msg::Image::ConstSharedPtr& msg);
    void GrabImageRight(const sensor_msgs::msg::Image::ConstSharedPtr& msg);
    cv::Mat GetImage(const sensor_msgs::msg::Image::ConstSharedPtr &img_msg);
    void SyncWithImu();

    ORB_SLAM3::System* mpSLAM;
    ImuGrabber *mpImuGb;
    SensorSynchronizer<std::pair<sensor_msgs::msg::Image::ConstSharedPtr, sensor_msgs::msg::Image::ConstSharedPtr>>* sync_queue;
    StereoFrameSynchronizer<sensor_msgs::msg::Image::ConstSharedPtr> stereo_sync;

    const bool do_rectify;
    cv::Mat M1l,M2l,M1r,M2r;

    const bool mbClahe;
    cv::Ptr<cv::CLAHE> mClahe = cv::createCLAHE(3.0, cv::Size(8, 8));
};



int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto n = std::make_shared<rclcpp::Node>("stereo_inertial");
  n->declare_parameter<bool>("do_rectify", false);
  n->declare_parameter<bool>("do_equalize", false);
  n->declare_parameter<bool>("use_pangolin", true);
  n->declare_parameter<int>("img_size", 3);
  n->declare_parameter<int>("imu_size", 5000);
  n->declare_parameter<double>("stereo_sync_tolerance", 0.002);
  n->declare_parameter<std::string>("queue_mode", "online");
  n->declare_parameter<std::string>("imu_topic", "/imu");
  n->declare_parameter<std::string>("left_topic", "/camera/left/image_raw");
  n->declare_parameter<std::string>("right_topic", "/camera/right/image_raw");
  const bool do_rectify = n->get_parameter("do_rectify").as_bool();
  const bool bEqual = n->get_parameter("do_equalize").as_bool();
  n->declare_parameter<std::string>("vocabulary", "");
  n->declare_parameter<std::string>("settings", "");
  const std::string vocabulary = n->get_parameter("vocabulary").as_string();
  const std::string settings = n->get_parameter("settings").as_string();
  if (vocabulary.empty() || settings.empty()) {
      RCLCPP_ERROR(n->get_logger(), "Both vocabulary and settings parameters are required");
      return 1;
  }
  // Create SLAM system. It initializes all system threads and gets ready to process frames.
  ORB_SLAM3::System SLAM(vocabulary,settings,ORB_SLAM3::System::IMU_STEREO,
                         n->get_parameter("use_pangolin").as_bool());

  int img_size = 3, imu_size = 5000;
  double stereo_sync_tolerance = 0.002;
  std::string queue_mode = "online";
  img_size = n->get_parameter("img_size").as_int();
  imu_size = n->get_parameter("imu_size").as_int();
  stereo_sync_tolerance = n->get_parameter("stereo_sync_tolerance").as_double();
  queue_mode = n->get_parameter("queue_mode").as_string();
  const bool offline = queue_mode == "offline";
  SensorSynchronizer<StereoFrame> sync_queue(offline, img_size, imu_size);
  ImuGrabber imugb(&sync_queue);
  ImageGrabber igb(&SLAM,&imugb,&sync_queue,offline,img_size,do_rectify,bEqual,stereo_sync_tolerance);

    if(igb.do_rectify)
    {
        // Load settings related to stereo calibration
        cv::FileStorage fsSettings(settings, cv::FileStorage::READ);
        if(!fsSettings.isOpened())
        {
            cerr << "ERROR: Wrong path to settings" << endl;
            return -1;
        }

        cv::Mat K_l, K_r, P_l, P_r, R_l, R_r, D_l, D_r;
        fsSettings["LEFT.K"] >> K_l;
        fsSettings["RIGHT.K"] >> K_r;

        fsSettings["LEFT.P"] >> P_l;
        fsSettings["RIGHT.P"] >> P_r;

        fsSettings["LEFT.R"] >> R_l;
        fsSettings["RIGHT.R"] >> R_r;

        fsSettings["LEFT.D"] >> D_l;
        fsSettings["RIGHT.D"] >> D_r;

        int rows_l = fsSettings["LEFT.height"];
        int cols_l = fsSettings["LEFT.width"];
        int rows_r = fsSettings["RIGHT.height"];
        int cols_r = fsSettings["RIGHT.width"];

        if(K_l.empty() || K_r.empty() || P_l.empty() || P_r.empty() || R_l.empty() || R_r.empty() || D_l.empty() || D_r.empty() ||
                rows_l==0 || rows_r==0 || cols_l==0 || cols_r==0)
        {
            cerr << "ERROR: Calibration parameters to rectify stereo are missing!" << endl;
            return -1;
        }

        cv::initUndistortRectifyMap(K_l,D_l,R_l,P_l.rowRange(0,3).colRange(0,3),cv::Size(cols_l,rows_l),CV_32F,igb.M1l,igb.M2l);
        cv::initUndistortRectifyMap(K_r,D_r,R_r,P_r.rowRange(0,3).colRange(0,3),cv::Size(cols_r,rows_r),CV_32F,igb.M1r,igb.M2r);
    }

  // Maximum delay, 5 seconds
  std::string imu_topic, left_topic, right_topic;
  imu_topic = n->get_parameter("imu_topic").as_string();
  left_topic = n->get_parameter("left_topic").as_string();
  right_topic = n->get_parameter("right_topic").as_string();
  auto sub_imu = n->create_subscription<sensor_msgs::msg::Imu>(imu_topic, 1000,
      [&imugb](sensor_msgs::msg::Imu::ConstSharedPtr msg) { imugb.GrabImu(msg); });
  auto sub_img_left = n->create_subscription<sensor_msgs::msg::Image>(left_topic, 100,
      [&igb](sensor_msgs::msg::Image::ConstSharedPtr msg) { igb.GrabImageLeft(msg); });
  auto sub_img_right = n->create_subscription<sensor_msgs::msg::Image>(right_topic, 100,
      [&igb](sensor_msgs::msg::Image::ConstSharedPtr msg) { igb.GrabImageRight(msg); });

  std::thread sync_thread(&ImageGrabber::SyncWithImu,&igb);

  rclcpp::spin(n);
  sync_queue.Stop();
  rclcpp::shutdown();
  sync_thread.join();
  SLAM.Shutdown();
  SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory_TUM_Format.txt");
  SLAM.SaveTrajectoryTUM("FrameTrajectory_TUM_Format.txt");
  SLAM.SaveTrajectoryKITTI("FrameTrajectory_KITTI_Format.txt");

  return 0;
}



void ImageGrabber::GrabImageLeft(const sensor_msgs::msg::Image::ConstSharedPtr &img_msg)
{
  if (!stereo_sync.PushLeft(rclcpp::Time(img_msg->header.stamp).seconds(), img_msg))
    RCLCPP_ERROR(rclcpp::get_logger("stereo_inertial"), "Left image queue overflow; wrapper paused, restart after fixing input rate");
}

void ImageGrabber::GrabImageRight(const sensor_msgs::msg::Image::ConstSharedPtr &img_msg)
{
  if (!stereo_sync.PushRight(rclcpp::Time(img_msg->header.stamp).seconds(), img_msg))
    RCLCPP_ERROR(rclcpp::get_logger("stereo_inertial"), "Right image queue overflow; wrapper paused, restart after fixing input rate");
}

cv::Mat ImageGrabber::GetImage(const sensor_msgs::msg::Image::ConstSharedPtr &img_msg)
{
  // Copy the ros image message to cv::Mat.
  const cv_bridge::CvImageConstPtr cv_ptr =
      cv_bridge::toCvShare(img_msg, sensor_msgs::image_encodings::MONO8);
  return cv_ptr->image.clone();
}

void ImageGrabber::SyncWithImu()
{
  while(rclcpp::ok())
  {
    StereoFrame frame;
    double timestamp = -1.0;
    vector<ORB_SLAM3::IMU::Point> measurements;
    if(!sync_queue->WaitNext(timestamp, frame, measurements)) return;

    cv::Mat imLeft = GetImage(frame.first);
    cv::Mat imRight = GetImage(frame.second);
    if(imLeft.empty() || imRight.empty()) {
      RCLCPP_ERROR(rclcpp::get_logger("stereo_inertial"), "Could not convert stereo images after dequeuing the sensor batch; stopping the worker");
      sync_queue->Stop();
      return;
    }
    if(mbClahe) { mClahe->apply(imLeft,imLeft); mClahe->apply(imRight,imRight); }
    if(do_rectify) {
      cv::Mat rectifiedLeft, rectifiedRight;
      cv::remap(imLeft,rectifiedLeft,M1l,M2l,cv::INTER_LINEAR);
      cv::remap(imRight,rectifiedRight,M1r,M2r,cv::INTER_LINEAR);
      imLeft = std::move(rectifiedLeft);
      imRight = std::move(rectifiedRight);
    }
    if(imLeft.size() != imRight.size()) {
      RCLCPP_ERROR(rclcpp::get_logger("stereo_inertial"), "Stereo image dimensions differ after dequeuing the sensor batch; stopping the worker");
      sync_queue->Stop();
      return;
    }
    mpSLAM->TrackStereo(imLeft,imRight,timestamp,measurements);
  }
}

void ImuGrabber::GrabImu(const sensor_msgs::msg::Imu::ConstSharedPtr &imu_msg)
{
  const double timestamp = rclcpp::Time(imu_msg->header.stamp).seconds();
  sync_queue->PushImu(ORB_SLAM3::IMU::Point(
      cv::Point3f(imu_msg->linear_acceleration.x, imu_msg->linear_acceleration.y, imu_msg->linear_acceleration.z),
      cv::Point3f(imu_msg->angular_velocity.x, imu_msg->angular_velocity.y, imu_msg->angular_velocity.z), timestamp));
}
