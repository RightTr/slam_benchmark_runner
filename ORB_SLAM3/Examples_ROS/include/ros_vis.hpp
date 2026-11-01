#ifndef ROS_VIS_HPP
#define ROS_VIS_HPP

#include <algorithm>
#include <chrono>
#include <sstream>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/features2d.hpp>
#include <opencv2/imgproc.hpp>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <System.h>

class RosTrackingVisualization
{
public:
    RosTrackingVisualization(ORB_SLAM3::System* slam, const rclcpp::Node::SharedPtr& node,
                             const std::string& map_frame, int width, int height,
                             double tracked_image_period = 0.0)
        : slam_(slam), map_frame_(map_frame), width_(width), height_(height),
          tracked_image_period_(tracked_image_period)
    {
        map_points_pub_ = node->create_publisher<sensor_msgs::msg::PointCloud2>("/orbslam3/map_points", 2);
        keyframes_pub_ = node->create_publisher<visualization_msgs::msg::MarkerArray>("/orbslam3/keyframes", 2);
        tracked_image_pub_ = node->create_publisher<sensor_msgs::msg::Image>("/orbslam3/tracked_image", 2);
    }

    void PublishTrackedImage(const sensor_msgs::msg::Image::ConstSharedPtr& image, uint8_t state,
                             const std::vector<cv::KeyPoint>& keypoints,
                             const std::vector<uint8_t>& associated, bool waiting_for_imu = false)
    {
        const auto now = std::chrono::steady_clock::now();
        if (tracked_image_period_ > 0.0 && last_tracked_image_.time_since_epoch().count() != 0 &&
            std::chrono::duration<double>(now - last_tracked_image_).count() < tracked_image_period_)
            return;
        last_tracked_image_ = now;

        const cv_bridge::CvImageConstPtr input = cv_bridge::toCvShare(image);
        cv::Mat output;
        if (width_ > 0 && height_ > 0 &&
            (input->image.cols != width_ || input->image.rows != height_))
            cv::resize(input->image, output, cv::Size(width_, height_));
        else
            output = input->image.clone();

        const size_t count = waiting_for_imu ? 0 : std::min(keypoints.size(), associated.size());
        const cv::Scalar tracked_color(40, 230, 40);
        const bool rgb = image->encoding == "rgb8" || image->encoding == "rgba8";
        const cv::Scalar unmatched_color = rgb ? cv::Scalar(255, 80, 30) : cv::Scalar(30, 80, 255);
        size_t associated_count = 0;
        for (size_t i = 0; i < count; ++i)
        {
            const bool has_association = associated[i] != 0;
            if (has_association) ++associated_count;
            cv::circle(output, cv::Point(cvRound(keypoints[i].pt.x), cvRound(keypoints[i].pt.y)),
                       3, has_association ? tracked_color : unmatched_color, 1, cv::LINE_AA);
        }

        std::ostringstream label;
        if (waiting_for_imu)
            label << "WAITING IMU";
        else
            label << (state == 2 ? "TRACKING" : state == 3 ? "LOST" : "INITIALIZING")
                  << " | ORB " << keypoints.size() << " | MAP " << associated_count;
        const cv::Scalar text_color = waiting_for_imu || state == 2 ? tracked_color : unmatched_color;
        cv::putText(output, label.str(), cv::Point(18, 32), cv::FONT_HERSHEY_SIMPLEX,
                    0.8, text_color, 2, cv::LINE_AA);
        cv_bridge::CvImage result(image->header, image->encoding, output);
        tracked_image_pub_->publish(*result.toImageMsg());
    }

    void PublishMapVisualization(const sensor_msgs::msg::Image::ConstSharedPtr& image)
    {
        const auto now = std::chrono::steady_clock::now();
        if (last_map_visualization_.time_since_epoch().count() != 0 &&
            std::chrono::duration<double>(now - last_map_visualization_).count() < 0.5)
            return;
        last_map_visualization_ = now;

        sensor_msgs::msg::PointCloud2 cloud;
        cloud.header = image->header;
        cloud.header.frame_id = map_frame_;
        const std::vector<Eigen::Vector3f> points = slam_->GetMapPointPositions();
        sensor_msgs::PointCloud2Modifier modifier(cloud);
        modifier.setPointCloud2FieldsByString(1, "xyz");
        modifier.resize(points.size());
        sensor_msgs::PointCloud2Iterator<float> x(cloud, "x"), y(cloud, "y"), z(cloud, "z");
        for (const Eigen::Vector3f& point : points)
        {
            *x = point.x(); *y = point.y(); *z = point.z();
            ++x; ++y; ++z;
        }
        map_points_pub_->publish(cloud);

        const auto keyframes = slam_->GetKeyFrameVisualizations();
        std::unordered_map<unsigned long, Eigen::Vector3f> centers;
        centers.reserve(keyframes.size());
        for (const auto& keyframe : keyframes)
            centers.emplace(keyframe.id, keyframe.camera_pose.translation());

        visualization_msgs::msg::MarkerArray markers;
        visualization_msgs::msg::Marker frusta, connections;
        frusta.header = connections.header = cloud.header;
        frusta.ns = "keyframe_frusta";
        frusta.id = 0;
        frusta.type = visualization_msgs::msg::Marker::LINE_LIST;
        frusta.action = visualization_msgs::msg::Marker::ADD;
        frusta.pose.orientation.w = 1.0;
        frusta.scale.x = 0.006;
        frusta.color.r = 0.1; frusta.color.g = 0.65; frusta.color.b = 1.0; frusta.color.a = 1.0;
        connections.ns = "keyframe_covisibility";
        connections.id = 1;
        connections.type = visualization_msgs::msg::Marker::LINE_LIST;
        connections.action = visualization_msgs::msg::Marker::ADD;
        connections.pose.orientation.w = 1.0;
        connections.scale.x = 0.003;
        connections.color.r = 1.0; connections.color.g = 0.65;
        connections.color.b = 0.1; connections.color.a = 0.8;

        const auto append_line = [](visualization_msgs::msg::Marker& marker,
                                    const Eigen::Vector3f& a, const Eigen::Vector3f& b)
        {
            geometry_msgs::msg::Point first, second;
            first.x = a.x(); first.y = a.y(); first.z = a.z();
            second.x = b.x(); second.y = b.y(); second.z = b.z();
            marker.points.push_back(first);
            marker.points.push_back(second);
        };
        for (const auto& keyframe : keyframes)
        {
            if (keyframe.fx <= 0.0f || keyframe.fy <= 0.0f) continue;
            const float depth = 0.18f;
            const Eigen::Vector3f origin = keyframe.camera_pose.translation();
            const Eigen::Vector3f corners[4] = {
                keyframe.camera_pose * Eigen::Vector3f((keyframe.min_x-keyframe.cx)*depth/keyframe.fx, (keyframe.min_y-keyframe.cy)*depth/keyframe.fy, depth),
                keyframe.camera_pose * Eigen::Vector3f((keyframe.max_x-keyframe.cx)*depth/keyframe.fx, (keyframe.min_y-keyframe.cy)*depth/keyframe.fy, depth),
                keyframe.camera_pose * Eigen::Vector3f((keyframe.max_x-keyframe.cx)*depth/keyframe.fx, (keyframe.max_y-keyframe.cy)*depth/keyframe.fy, depth),
                keyframe.camera_pose * Eigen::Vector3f((keyframe.min_x-keyframe.cx)*depth/keyframe.fx, (keyframe.max_y-keyframe.cy)*depth/keyframe.fy, depth)
            };
            for (int i = 0; i < 4; ++i)
            {
                append_line(frusta, origin, corners[i]);
                append_line(frusta, corners[i], corners[(i + 1) % 4]);
            }
            for (unsigned long id : keyframe.covisible_ids)
            {
                const auto connected = centers.find(id);
                if (keyframe.id < id && connected != centers.end())
                    append_line(connections, origin, connected->second);
            }
        }
        markers.markers.push_back(std::move(frusta));
        markers.markers.push_back(std::move(connections));
        keyframes_pub_->publish(markers);
    }

private:
    ORB_SLAM3::System* slam_;
    std::string map_frame_;
    int width_, height_;
    double tracked_image_period_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr map_points_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr keyframes_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr tracked_image_pub_;
    std::chrono::steady_clock::time_point last_tracked_image_{};
    std::chrono::steady_clock::time_point last_map_visualization_{};
};

#endif
