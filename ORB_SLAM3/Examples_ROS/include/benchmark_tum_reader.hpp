#pragma once

#include <Eigen/Geometry>
#include <builtin_interfaces/msg/time.hpp>
#include <nav_msgs/msg/path.hpp>

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <fstream>
#include <map>
#include <iomanip>
#include <sstream>
#include <stdexcept>
#include <string>

// ORB-SLAM3's TUM exporter writes timestamps to microsecond precision. Keep
// this conversion in one place and recover the original ROS stamp by lookup;
// never reconstruct a nanosecond stamp from the rounded text value.
namespace benchmark_tum_reader {
// ORB-SLAM3 passes rclcpp::Time::seconds() through a double and writes the
// value with fixed six-decimal precision. Match ROS 2's ns / 1e9 conversion
// exactly; multiplying by 1e-9 can round a boundary to the adjacent microsecond.
inline std::string tumTimestamp(int64_t ns) {
    std::ostringstream out;
    out << std::fixed << std::setprecision(6)
        << (static_cast<double>(ns) / 1e9);
    return out.str();
}

inline nav_msgs::msg::Path read(const std::string& file, const std::string& frame,
    const std::map<int64_t, builtin_interfaces::msg::Time>& exact_stamps,
    bool keyframes = false)
{
    std::ifstream input(file);
    if (!input) throw std::runtime_error("trajectory export missing: " + file);

    std::map<std::string, builtin_interfaces::msg::Time> by_tum_timestamp;
    for (const auto& [ns, stamp] : exact_stamps) {
        if (!by_tum_timestamp.emplace(tumTimestamp(ns), stamp).second)
            throw std::runtime_error("TUM timestamp collision at microsecond precision");
    }

    nav_msgs::msg::Path path;
    path.header.frame_id = frame;
    if (!exact_stamps.empty()) path.header.stamp = exact_stamps.rbegin()->second;
    std::string sec;
    double x, y, z, qx, qy, qz, qw;
    int64_t previous = -1;
    double previous_pose[7]{};
    while (input >> sec >> x >> y >> z >> qx >> qy >> qz >> qw) {
        const auto match = by_tum_timestamp.find(sec);
        if (match == by_tum_timestamp.end())
            throw std::runtime_error("TUM timestamp not in current session: " + sec);
        const auto ns = int64_t(match->second.sec) * 1000000000LL + match->second.nanosec;
        if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z) ||
            !std::isfinite(qx) || !std::isfinite(qy) || !std::isfinite(qz) || !std::isfinite(qw))
            throw std::runtime_error("nonfinite TUM pose");
        const double current_pose[7]{x, y, z, qx, qy, qz, qw};
        if (ns == previous) {
            bool identical = true;
            for (size_t i = 0; i < 7; ++i)
                identical = identical && current_pose[i] == previous_pose[i];
            if (identical) continue;  // ORB-SLAM3 repeats its cached pose for unset frames.
            throw std::runtime_error("duplicate TUM timestamp has different poses: " + sec);
        }
        if (ns < previous)
            throw std::runtime_error("TUM trajectory is out of timestamp order at " + sec);
        previous = ns;
        for (size_t i = 0; i < 7; ++i) previous_pose[i] = current_pose[i];
        Eigen::Quaterniond q(qw, qx, qy, qz);
        if (q.norm() < 1e-8) throw std::runtime_error("invalid TUM quaternion");
        q.normalize();
        geometry_msgs::msg::PoseStamped pose;
        pose.header = path.header;
        pose.header.stamp = match->second;
        pose.pose.position.x = x; pose.pose.position.y = y; pose.pose.position.z = z;
        pose.pose.orientation.x = q.x(); pose.pose.orientation.y = q.y();
        pose.pose.orientation.z = q.z(); pose.pose.orientation.w = q.w();
        path.poses.push_back(pose);
    }
    if (path.poses.empty()) throw std::runtime_error(keyframes ? "no keyframes" : "no final poses");
    if (keyframes) {
        const auto& first = path.poses.front().pose;
        Eigen::Quaterniond q0(first.orientation.w, first.orientation.x,
                              first.orientation.y, first.orientation.z);
        const Eigen::Vector3d t0(first.position.x, first.position.y, first.position.z);
        for (auto& pose : path.poses) {
            Eigen::Vector3d t(pose.pose.position.x, pose.pose.position.y, pose.pose.position.z);
            Eigen::Quaterniond q(pose.pose.orientation.w, pose.pose.orientation.x,
                                 pose.pose.orientation.y, pose.pose.orientation.z);
            t = q0.conjugate() * (t - t0); q = q0.conjugate() * q;
            pose.pose.position.x = t.x(); pose.pose.position.y = t.y(); pose.pose.position.z = t.z();
            pose.pose.orientation.x = q.x(); pose.pose.orientation.y = q.y();
            pose.pose.orientation.z = q.z(); pose.pose.orientation.w = q.w();
        }
    }
    return path;
}
} // namespace benchmark_tum_reader
