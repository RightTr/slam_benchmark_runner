#ifndef SENSOR_SYNC_HPP
#define SENSOR_SYNC_HPP

#include <algorithm>
#include <cmath>
#include <condition_variable>
#include <cstddef>
#include <deque>
#include <mutex>
#include <utility>
#include <vector>

#include "ImuTypes.h"

// ROS-independent, single-consumer queue used by the sensor wrappers.
template<class FramePayload>
class SensorSynchronizer
{
public:
    SensorSynchronizer(bool offline, int img_size, int imu_size)
        : offline_(offline), img_size_(static_cast<std::size_t>(std::max(1, img_size))),
          imu_size_(static_cast<std::size_t>(std::max(1, imu_size)))
    {}

    void PushFrame(double timestamp, FramePayload frame)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (!running_ || input_closed_) return;
        const double newest_frame = frames_.empty() ? last_frame_time_ : frames_.back().first;
        if (timestamp <= newest_frame) return;

        if (!offline_ && frames_.size() >= img_size_)
            frames_.pop_front();
        frames_.emplace_back(timestamp, std::move(frame));
        cv_.notify_one();
    }

    void PushImu(const ORB_SLAM3::IMU::Point& sample)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (!running_ || sample.t <= last_imu_time_) return;
        last_imu_time_ = sample.t;
        if (!offline_ && imus_.size() >= imu_size_)
            imus_.pop_front();
        imus_.push_back(sample);
        cv_.notify_one();
    }

    bool WaitNext(double& timestamp, FramePayload& frame,
                  std::vector<ORB_SLAM3::IMU::Point>& measurements)
    {
        std::unique_lock<std::mutex> lock(mutex_);
        while (running_)
        {
            cv_.wait(lock, [this]() {
                return !running_ || (!frames_.empty() && !imus_.empty() &&
                                     imus_.back().t > frames_.front().first);
            });
            if (!running_) return false;

            const double image_time = frames_.front().first;
            if (imus_.front().t >= image_time && last_frame_time_ < 0.0)
            {
                frames_.pop_front();
                continue;
            }

            const auto endpoint = std::upper_bound(
                imus_.begin(), imus_.end(), image_time,
                [](double time, const ORB_SLAM3::IMU::Point& sample) {
                    return time < sample.t;
                });
            const double endpoint_time = endpoint->t;

            timestamp = image_time;
            frame = std::move(frames_.front().second);
            frames_.pop_front();
            last_frame_time_ = image_time;

            measurements.clear();
            while (!imus_.empty() && imus_.front().t <= endpoint_time)
            {
                measurements.push_back(std::move(imus_.front()));
                imus_.pop_front();
            }
            return true;
        }
        return false;
    }

    bool WaitFrame(double& timestamp, FramePayload& frame)
    {
        std::unique_lock<std::mutex> lock(mutex_);
        cv_.wait(lock, [this]() { return !running_ || !frames_.empty() || input_closed_; });
        if (!running_ || frames_.empty()) return false;
        timestamp = frames_.front().first;
        frame = std::move(frames_.front().second);
        frames_.pop_front();
        last_frame_time_ = timestamp;
        cv_.notify_one();
        return true;
    }

    void Stop()
    {
        std::lock_guard<std::mutex> lock(mutex_);
        running_ = false;
        cv_.notify_all();
    }

    void CloseInput()
    {
        std::lock_guard<std::mutex> lock(mutex_);
        input_closed_ = true;
        cv_.notify_all();
    }

private:
    bool offline_;
    std::size_t img_size_, imu_size_;
    std::mutex mutex_;
    std::condition_variable cv_;
    std::deque<std::pair<double, FramePayload>> frames_;
    std::deque<ORB_SLAM3::IMU::Point> imus_;
    bool running_{true};
    bool input_closed_{false};
    double last_frame_time_{-1.0};
    double last_imu_time_{-1.0};
};

// Matches two timestamped image streams before handing frames to the queue.
template<class ImagePayload>
class StereoFrameSynchronizer
{
public:
    using Frame = std::pair<ImagePayload, ImagePayload>;

    StereoFrameSynchronizer(SensorSynchronizer<Frame>& frames, bool offline,
                            int img_size, double tolerance)
        : frames_(frames), offline_(offline),
          img_size_(static_cast<std::size_t>(std::max(1, img_size))),
          tolerance_(tolerance)
    {}

    bool PushLeft(double timestamp, ImagePayload image)
    {
        return Push(timestamp, std::move(image), true);
    }

    bool PushRight(double timestamp, ImagePayload image)
    {
        return Push(timestamp, std::move(image), false);
    }

private:
    using TimedImage = std::pair<double, ImagePayload>;

    bool Push(double timestamp, ImagePayload image, bool is_left)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        auto& own = is_left ? left_ : right_;
        own.emplace_back(timestamp, std::move(image));
        if (own.size() > img_size_)
        {
            if (offline_)
            {
                frames_.Stop();
                return false;
            }
            own.pop_front();
        }

        while (!left_.empty() && !right_.empty())
        {
            auto& left = left_.front();
            auto& right = right_.front();
            if (std::abs(left.first - right.first) > tolerance_)
            {
                (left.first < right.first ? left_ : right_).pop_front();
                continue;
            }
            frames_.PushFrame(left.first, Frame{std::move(left.second), std::move(right.second)});
            left_.pop_front();
            right_.pop_front();
        }
        return true;
    }

    SensorSynchronizer<Frame>& frames_;
    bool offline_;
    std::size_t img_size_;
    double tolerance_;
    std::mutex mutex_;
    std::deque<TimedImage> left_, right_;
};

#endif
