#ifndef SENSOR_SYNC_HPP
#define SENSOR_SYNC_HPP

#include <algorithm>
#include <cmath>
#include <condition_variable>
#include <cstddef>
#include <deque>
#include <mutex>
#include <utility>

// ROS-independent, single-consumer queue used by the sensor wrappers.
template<class FramePayload>
class SensorSynchronizer
{
public:
    SensorSynchronizer(bool offline, int img_size)
        : offline_(offline), img_size_(static_cast<std::size_t>(std::max(1, img_size)))
    {}

    void PushFrame(double timestamp, FramePayload frame)
    {
        std::unique_lock<std::mutex> lock(mutex_);
        if (!running_ || input_closed_) return;

        // Offline benchmark playback must preserve every frame without keeping
        // an unbounded backlog of full-resolution RGB-D messages in memory.
        // Apply backpressure when full; the tracking worker signals space after
        // it removes a frame from the queue.
        if (offline_)
            space_cv_.wait(lock, [this]() {
                return !running_ || input_closed_ || frames_.size() < img_size_;
            });
        if (!running_ || input_closed_) return;

        const double newest_frame = frames_.empty() ? last_frame_time_ : frames_.back().first;
        if (timestamp <= newest_frame) return;

        if (!offline_ && frames_.size() >= img_size_)
            frames_.pop_front();
        frames_.emplace_back(timestamp, std::move(frame));
        cv_.notify_one();
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
        space_cv_.notify_all();
        cv_.notify_one();
        return true;
    }

    void Stop()
    {
        std::lock_guard<std::mutex> lock(mutex_);
        running_ = false;
        cv_.notify_all();
        space_cv_.notify_all();
    }

    void CloseInput()
    {
        std::lock_guard<std::mutex> lock(mutex_);
        input_closed_ = true;
        cv_.notify_all();
        space_cv_.notify_all();
    }

private:
    bool offline_;
    std::size_t img_size_;
    std::mutex mutex_;
    std::condition_variable cv_;
    std::condition_variable space_cv_;
    std::deque<std::pair<double, FramePayload>> frames_;
    bool running_{true};
    bool input_closed_{false};
    double last_frame_time_{-1.0};
};

#endif
