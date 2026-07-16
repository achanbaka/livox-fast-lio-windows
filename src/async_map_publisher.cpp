#include "async_map_publisher.h"

#include <algorithm>
#include <chrono>
#include <exception>
#include <iostream>
#include <utility>

AsyncMapPublisher::AsyncMapPublisher(MapAccumulator& accumulator,
                                     PublishCallback callback,
                                     PublishCallback delta_callback,
                                     size_t max_pending_delta_points,
                                     size_t max_pending_frames)
    : accumulator_(accumulator),
      callback_(std::move(callback)),
      delta_callback_(std::move(delta_callback)),
      max_pending_delta_points_(max_pending_delta_points),
      max_pending_frames_(max_pending_frames)
{}

AsyncMapPublisher::~AsyncMapPublisher()
{
    stop();
}

void AsyncMapPublisher::start()
{
    std::lock_guard<std::mutex> lock(mutex_);
    if (running_) {
        return;
    }

    stopping_ = false;
    pending_ = false;
    pending_frames_.clear();
    pending_delta_.clear();
    busy_ = false;
    pending_request_sequence_ = 0;
    next_frame_sequence_ = 0;
    last_built_sequence_ = 0;
    stats_ = Stats{};
    running_ = true;
    worker_ = std::thread(&AsyncMapPublisher::run, this);
}

void AsyncMapPublisher::enqueueFrame(PointCloudXYZI frame, double timestamp)
{
    if (frame.empty()) {
        return;
    }

    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (!running_ || stopping_) {
            return;
        }

        ++stats_.frames_enqueued;
        const uint64_t sequence = ++next_frame_sequence_;
        if (max_pending_frames_ > 0 &&
            pending_frames_.size() >= max_pending_frames_)
        {
            pending_frames_.pop_front();
            ++stats_.frames_dropped;
        }
        pending_frames_.push_back(FrameTask{std::move(frame), timestamp, sequence});
        stats_.max_frame_queue_depth =
            std::max(stats_.max_frame_queue_depth, pending_frames_.size());
    }
    work_cv_.notify_one();
}

void AsyncMapPublisher::request(double timestamp)
{
    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (!running_ || stopping_) {
            return;
        }
        ++stats_.requested;
        if (pending_) {
            ++stats_.coalesced;
        }
        pending_timestamp_ = timestamp;
        pending_request_sequence_ = next_frame_sequence_;
        pending_ = true;
    }
    work_cv_.notify_one();
}

void AsyncMapPublisher::enqueueDelta(PointCloudXYZI delta, double timestamp)
{
    if (delta.empty()) {
        return;
    }

    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (!running_ || stopping_ || !delta_callback_) {
            return;
        }
        enqueueDeltaLocked(std::move(delta), timestamp);
    }
    work_cv_.notify_one();
}

void AsyncMapPublisher::enqueueDeltaLocked(PointCloudXYZI delta, double timestamp)
{
    ++stats_.delta_requested;
    if (!pending_delta_.empty()) {
        ++stats_.delta_coalesced;
    }

    bool needs_resync = false;
    if (max_pending_delta_points_ > 0 &&
        delta.size() > max_pending_delta_points_)
    {
        stats_.delta_dropped_points +=
            static_cast<uint64_t>(delta.size() - max_pending_delta_points_);
        delta.points.resize(max_pending_delta_points_);
        delta.width = static_cast<uint32_t>(delta.size());
        needs_resync = true;
    }

    if (max_pending_delta_points_ > 0 &&
        pending_delta_.size() + delta.size() > max_pending_delta_points_)
    {
        stats_.delta_dropped_points += static_cast<uint64_t>(pending_delta_.size());
        pending_delta_.clear();
        needs_resync = true;
    }

    if (pending_delta_.empty()) {
        pending_delta_ = std::move(delta);
    } else {
        pending_delta_ += delta;
    }
    pending_delta_.width = static_cast<uint32_t>(pending_delta_.size());
    pending_delta_.height = 1;
    pending_delta_.is_dense = true;
    pending_delta_timestamp_ = timestamp;

    if (needs_resync && callback_) {
        if (pending_) {
            ++stats_.coalesced;
        }
        pending_ = true;
        pending_timestamp_ = timestamp;
        pending_request_sequence_ = last_built_sequence_;
        ++stats_.requested;
        ++stats_.delta_resync_requests;
    }
}

void AsyncMapPublisher::flush()
{
    std::unique_lock<std::mutex> lock(mutex_);
    idle_cv_.wait(lock, [this] {
        return !pending_ && pending_frames_.empty() &&
               pending_delta_.empty() && !busy_;
    });
}

void AsyncMapPublisher::stop()
{
    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (!running_) {
            return;
        }
        stopping_ = true;
    }
    work_cv_.notify_one();
    if (worker_.joinable()) {
        worker_.join();
    }

    std::lock_guard<std::mutex> lock(mutex_);
    running_ = false;
    stopping_ = false;
    pending_ = false;
    pending_frames_.clear();
    pending_delta_.clear();
    busy_ = false;
    idle_cv_.notify_all();
}

bool AsyncMapPublisher::isRunning() const
{
    std::lock_guard<std::mutex> lock(mutex_);
    return running_ && !stopping_;
}

AsyncMapPublisher::Stats AsyncMapPublisher::stats() const
{
    std::lock_guard<std::mutex> lock(mutex_);
    Stats result = stats_;
    result.current_frame_queue_depth = pending_frames_.size();
    result.current_delta_points = pending_delta_.size();
    result.full_map_pending = pending_;
    result.busy = busy_;
    return result;
}

void AsyncMapPublisher::run()
{
    while (true) {
        enum class TaskType { Frame, FullMap, Delta };
        TaskType task_type = TaskType::FullMap;
        double timestamp = 0.0;
        FrameTask frame;
        PointCloudXYZI delta;
        {
            std::unique_lock<std::mutex> lock(mutex_);
            work_cv_.wait(lock, [this] {
                return stopping_ || pending_ || !pending_frames_.empty() ||
                       !pending_delta_.empty();
            });
            if (stopping_ && !pending_ && pending_frames_.empty() &&
                pending_delta_.empty())
            {
                break;
            }

            const bool full_map_ready =
                pending_ && last_built_sequence_ >= pending_request_sequence_;
            if (full_map_ready || (pending_ && pending_frames_.empty())) {
                task_type = TaskType::FullMap;
                timestamp = pending_timestamp_;
                pending_ = false;
            } else if (!pending_frames_.empty()) {
                task_type = TaskType::Frame;
                frame = std::move(pending_frames_.front());
                pending_frames_.pop_front();
            } else {
                task_type = TaskType::Delta;
                timestamp = pending_delta_timestamp_;
                delta = std::move(pending_delta_);
                pending_delta_.clear();
            }
            busy_ = true;
        }

        if (task_type == TaskType::Frame) {
            PointCloudXYZI inserted_points;
            const auto build_begin = std::chrono::steady_clock::now();
            accumulator_.addFrame(
                frame.cloud, delta_callback_ ? &inserted_points : nullptr);
            const auto build_end = std::chrono::steady_clock::now();

            {
                std::lock_guard<std::mutex> lock(mutex_);
                last_built_sequence_ =
                    std::max(last_built_sequence_, frame.sequence);
                ++stats_.frames_built;
                stats_.last_frame_point_count = frame.cloud.size();
                stats_.last_frame_build_ms =
                    std::chrono::duration<double, std::milli>(
                        build_end - build_begin).count();
                if (!inserted_points.empty() && delta_callback_) {
                    enqueueDeltaLocked(std::move(inserted_points), frame.timestamp);
                }
                busy_ = false;
            }
            work_cv_.notify_one();
            idle_cv_.notify_all();
            continue;
        }

        if (task_type == TaskType::Delta) {
            bool failed = false;
            const auto publish_begin = std::chrono::steady_clock::now();
            try {
                if (!delta.empty() && delta_callback_) {
                    delta_callback_(delta, timestamp);
                }
            } catch (const std::exception& error) {
                failed = true;
                std::cerr << "[AsyncMapPublisher] Delta publish failed: "
                          << error.what() << std::endl;
            } catch (...) {
                failed = true;
                std::cerr << "[AsyncMapPublisher] Delta publish failed with an unknown error."
                          << std::endl;
            }
            const auto publish_end = std::chrono::steady_clock::now();

            {
                std::lock_guard<std::mutex> lock(mutex_);
                ++stats_.delta_published;
                if (failed) {
                    ++stats_.failures;
                }
                stats_.last_delta_point_count = delta.size();
                stats_.last_delta_publish_ms =
                    std::chrono::duration<double, std::milli>(
                        publish_end - publish_begin).count();
                busy_ = false;
            }
            idle_cv_.notify_all();
            continue;
        }

        const auto snapshot_begin = std::chrono::steady_clock::now();
        PointCloudXYZI map = accumulator_.snapshot();
        const auto snapshot_end = std::chrono::steady_clock::now();
        bool failed = false;
        const auto publish_begin = snapshot_end;
        try {
            if (!map.empty() && callback_) {
                callback_(map, timestamp);
            }
        } catch (const std::exception& error) {
            failed = true;
            std::cerr << "[AsyncMapPublisher] Publish failed: " << error.what() << std::endl;
        } catch (...) {
            failed = true;
            std::cerr << "[AsyncMapPublisher] Publish failed with an unknown error." << std::endl;
        }
        const auto publish_end = std::chrono::steady_clock::now();

        {
            std::lock_guard<std::mutex> lock(mutex_);
            ++stats_.published;
            if (failed) {
                ++stats_.failures;
            }
            stats_.last_point_count = map.size();
            stats_.last_snapshot_ms =
                std::chrono::duration<double, std::milli>(snapshot_end - snapshot_begin).count();
            stats_.last_publish_ms =
                std::chrono::duration<double, std::milli>(publish_end - publish_begin).count();
            busy_ = false;
        }
        idle_cv_.notify_all();
    }

    std::lock_guard<std::mutex> lock(mutex_);
    busy_ = false;
    idle_cv_.notify_all();
}
