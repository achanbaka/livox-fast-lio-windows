#include "realtime_foxglove_worker.h"

#include <algorithm>
#include <exception>
#include <iostream>
#include <limits>
#include <type_traits>
#include <utility>

namespace {

constexpr size_t kControlTaskBytes = 64;
constexpr size_t kPoseTaskBytes = 192;
constexpr size_t kImuTaskBytes = 128;
constexpr size_t kPathTaskBytes = 64;

size_t cloudTaskBytes(const PointCloudXYZI& cloud)
{
    if (cloud.size() >
        (std::numeric_limits<size_t>::max() - sizeof(PointCloudXYZI)) /
            sizeof(PointType))
    {
        return std::numeric_limits<size_t>::max();
    }
    return sizeof(PointCloudXYZI) + cloud.size() * sizeof(PointType);
}

size_t saturatedAddSize(size_t left, size_t right) noexcept
{
    if (right > std::numeric_limits<size_t>::max() - left) {
        return std::numeric_limits<size_t>::max();
    }
    return left + right;
}

}  // namespace

RealtimeFoxgloveWorker::RealtimeFoxgloveWorker(Callbacks callbacks, Config config)
    : callbacks_(std::move(callbacks)), config_(config)
{
    // A zero path bound would turn the persistent visualization history into an
    // unbounded allocation.  Keep at least the newest pose.
    config_.max_path_points = std::max<size_t>(1, config_.max_path_points);
}

RealtimeFoxgloveWorker::~RealtimeFoxgloveWorker()
{
    stop(false);
}

void RealtimeFoxgloveWorker::start()
{
    std::lock_guard<std::mutex> lock(mutex_);
    if (running_) return;
    stopping_ = false;
    drain_ = true;
    busy_ = false;
    in_callback_ = false;
    worker_exited_ = false;
    clearPendingLocked();
    path_history_.clear();
    last_path_timestamp_ = 0.0;
    inflight_points_ = 0;
    inflight_bytes_ = 0;
    stats_ = Stats{};
    running_ = true;
    worker_ = std::thread(&RealtimeFoxgloveWorker::run, this);
}

template <typename T>
bool RealtimeFoxgloveWorker::admitLatestLocked(
    std::optional<T>& slot, T task, size_t points, size_t bytes,
    ChannelStats& channel)
{
    ++channel.attempted;
    if (!running_ || stopping_) {
        ++channel.dropped;
        return false;
    }

    refreshStatsLocked();
    size_t old_points = 0;
    size_t old_bytes = 0;
    if (slot) {
        if constexpr (std::is_same_v<T, CloudTask>) {
            old_points = slot->cloud.size();
            old_bytes = cloudTaskBytes(slot->cloud);
        } else if constexpr (std::is_same_v<T, PathTask>) {
            old_bytes = kPathTaskBytes;
        } else if constexpr (std::is_same_v<T, PoseTask>) {
            old_bytes = kPoseTaskBytes;
        } else if constexpr (std::is_same_v<T, ImuData>) {
            old_bytes = kImuTaskBytes;
        } else {
            old_bytes = kControlTaskBytes;
        }
    }

    const size_t projected_points = saturatedAddSize(
        saturatedAddSize(stats_.current_points - old_points,
                         inflight_points_),
        points);
    const size_t projected_bytes = saturatedAddSize(
        saturatedAddSize(stats_.current_bytes - old_bytes,
                         inflight_bytes_),
        bytes);
    if ((config_.max_pending_points > 0 &&
         projected_points > config_.max_pending_points) ||
        (config_.max_pending_bytes > 0 &&
         projected_bytes > config_.max_pending_bytes))
    {
        ++channel.dropped;
        return false;
    }

    if (slot) ++channel.overwritten;
    slot = std::move(task);
    ++channel.accepted;
    refreshStatsLocked();
    work_cv_.notify_one();
    return true;
}

bool RealtimeFoxgloveWorker::tryEnqueueImu(ImuData imu)
{
    std::lock_guard<std::mutex> lock(mutex_);
    if (!callbacks_.imu) {
        ++stats_.imu.attempted;
        ++stats_.imu.dropped;
        return false;
    }
    return admitLatestLocked(imu_, std::move(imu), 0, kImuTaskBytes,
                             stats_.imu);
}

bool RealtimeFoxgloveWorker::tryEnqueuePose(
    V3D position, Eigen::Quaterniond orientation, double timestamp)
{
    std::lock_guard<std::mutex> lock(mutex_);
    if (!callbacks_.pose) {
        ++stats_.pose.attempted;
        ++stats_.pose.dropped;
        return false;
    }
    return admitLatestLocked(
        pose_, PoseTask{std::move(position), std::move(orientation), timestamp},
        0, kPoseTaskBytes, stats_.pose);
}

bool RealtimeFoxgloveWorker::tryEnqueueCloud(
    PointCloudXYZI cloud, double timestamp)
{
    std::lock_guard<std::mutex> lock(mutex_);
    if (!callbacks_.cloud) {
        ++stats_.cloud.attempted;
        ++stats_.cloud.dropped;
        return false;
    }
    const size_t points = cloud.size();
    const size_t bytes = cloudTaskBytes(cloud);
    return admitLatestLocked(
        cloud_, CloudTask{std::move(cloud), timestamp}, points, bytes,
        stats_.cloud);
}

bool RealtimeFoxgloveWorker::tryEnqueuePathPoint(
    V3D position, double timestamp, bool request_publish)
{
    std::lock_guard<std::mutex> lock(mutex_);
    ++stats_.path_points_received;
    if (!callbacks_.path || !running_ || stopping_) {
        ++stats_.path_points_dropped;
        return false;
    }

    refreshStatsLocked();
    const size_t point_bytes = sizeof(V3D);
    const bool replace_oldest =
        pending_path_points_.size() >= config_.max_path_points;
    const size_t retained_points = stats_.current_points -
        (replace_oldest ? 1U : 0U);
    const size_t retained_bytes = stats_.current_bytes -
        (replace_oldest ? point_bytes : 0U);
    const size_t projected_points = saturatedAddSize(
        saturatedAddSize(retained_points, inflight_points_), 1U);
    const size_t projected_bytes = saturatedAddSize(
        saturatedAddSize(retained_bytes, inflight_bytes_), point_bytes);
    if ((config_.max_pending_points > 0 &&
         projected_points > config_.max_pending_points) ||
        (config_.max_pending_bytes > 0 &&
         projected_bytes > config_.max_pending_bytes))
    {
        ++stats_.path_points_dropped;
        return false;
    }
    if (replace_oldest) {
        pending_path_points_.pop_front();
        ++stats_.path_points_dropped;
    }
    pending_path_points_.push_back(std::move(position));
    last_path_timestamp_ = timestamp;
    refreshStatsLocked();
    if (!request_publish) return true;

    return admitLatestLocked(path_, PathTask{timestamp}, 0, kPathTaskBytes,
                             stats_.path);
}

bool RealtimeFoxgloveWorker::tryEnqueueTime(double timestamp)
{
    std::lock_guard<std::mutex> lock(mutex_);
    if (!callbacks_.time) {
        ++stats_.time.attempted;
        ++stats_.time.dropped;
        return false;
    }
    return admitLatestLocked(time_, timestamp, 0, kControlTaskBytes,
                             stats_.time);
}

bool RealtimeFoxgloveWorker::tryEnqueuePlayback(
    FoxglovePublisher::PlaybackState state)
{
    std::lock_guard<std::mutex> lock(mutex_);
    if (!callbacks_.playback) {
        ++stats_.playback.attempted;
        ++stats_.playback.dropped;
        return false;
    }
    return admitLatestLocked(playback_, std::move(state), 0,
                             kControlTaskBytes, stats_.playback);
}

bool RealtimeFoxgloveWorker::flushFor(std::chrono::milliseconds timeout)
{
    std::unique_lock<std::mutex> lock(mutex_);
    return idle_cv_.wait_for(lock, timeout, [this] {
        return !hasWorkLocked() && !busy_;
    });
}

bool RealtimeFoxgloveWorker::stopFor(
    std::chrono::milliseconds timeout, bool drain)
{
    // Signal cancellation first; only the owning thread performs the join.
    const auto deadline = std::chrono::steady_clock::now() + timeout;
    {
        std::unique_lock<std::mutex> lock(mutex_);
        if (!running_) return true;
        if (!stopping_) {
            stopping_ = true;
            drain_ = drain;
        } else if (!drain) {
            drain_ = false;
        }
        if (!drain_) {
            refreshStatsLocked();
            stats_.tasks_cancelled += stats_.current_tasks;
            clearPendingLocked();
        }
        work_cv_.notify_all();
        if (!exit_cv_.wait_until(lock, deadline,
                                 [this] { return worker_exited_; })) {
            ++stats_.stop_timeouts;
            return false;
        }
    }

    if (worker_.joinable()) worker_.join();
    std::lock_guard<std::mutex> lock(mutex_);
    running_ = false;
    stopping_ = false;
    busy_ = false;
    in_callback_ = false;
    inflight_points_ = 0;
    inflight_bytes_ = 0;
    refreshStatsLocked();
    idle_cv_.notify_all();
    return true;
}

void RealtimeFoxgloveWorker::stop(bool drain)
{
    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (!running_) return;
        if (!stopping_) {
            stopping_ = true;
            drain_ = drain;
        } else if (!drain) {
            drain_ = false;
        }
        if (!drain_) {
            refreshStatsLocked();
            stats_.tasks_cancelled += stats_.current_tasks;
            clearPendingLocked();
        }
    }
    work_cv_.notify_all();
    if (worker_.joinable()) worker_.join();
    std::lock_guard<std::mutex> lock(mutex_);
    running_ = false;
    stopping_ = false;
    busy_ = false;
    in_callback_ = false;
    inflight_points_ = 0;
    inflight_bytes_ = 0;
    refreshStatsLocked();
    idle_cv_.notify_all();
}

bool RealtimeFoxgloveWorker::isRunning() const
{
    std::lock_guard<std::mutex> lock(mutex_);
    return running_ && !stopping_;
}

RealtimeFoxgloveWorker::Stats RealtimeFoxgloveWorker::stats() const
{
    std::lock_guard<std::mutex> lock(mutex_);
    Stats result = stats_;
    result.busy = busy_;
    result.in_callback = in_callback_;
    result.stopping = stopping_;
    result.worker_exited = worker_exited_;
    return result;
}

bool RealtimeFoxgloveWorker::hasWorkLocked() const
{
    return playback_ || time_ || pose_ || imu_ || cloud_ || path_ ||
        (stopping_ && drain_ && !pending_path_points_.empty());
}

void RealtimeFoxgloveWorker::clearPendingLocked()
{
    playback_.reset();
    time_.reset();
    pose_.reset();
    imu_.reset();
    cloud_.reset();
    path_.reset();
    if (!pending_path_points_.empty()) {
        stats_.path_points_dropped += pending_path_points_.size();
        pending_path_points_.clear();
    }
    refreshStatsLocked();
}

void RealtimeFoxgloveWorker::run()
{
    while (true) {
        TaskKind kind = TaskKind::None;
        FoxglovePublisher::PlaybackState playback;
        double time = 0.0;
        PoseTask pose;
        ImuData imu;
        CloudTask cloud;
        PathTask path;
        std::deque<V3D> pending_path_points;
        size_t task_points = 0;
        size_t task_bytes = 0;

        {
            std::unique_lock<std::mutex> lock(mutex_);
            work_cv_.wait(lock, [this] { return stopping_ || hasWorkLocked(); });
            if (stopping_ && (!drain_ || !hasWorkLocked())) break;

            // Control/state always wins over visualization payloads.
            if (playback_) {
                kind = TaskKind::Playback;
                playback = std::move(*playback_);
                playback_.reset();
                task_bytes = kControlTaskBytes;
            } else if (time_) {
                kind = TaskKind::Time;
                time = *time_;
                time_.reset();
                task_bytes = kControlTaskBytes;
            } else if (pose_) {
                kind = TaskKind::Pose;
                pose = std::move(*pose_);
                pose_.reset();
                task_bytes = kPoseTaskBytes;
            } else if (imu_) {
                kind = TaskKind::Imu;
                imu = std::move(*imu_);
                imu_.reset();
                task_bytes = kImuTaskBytes;
            } else if (cloud_) {
                kind = TaskKind::Cloud;
                task_points = cloud_->cloud.size();
                task_bytes = cloudTaskBytes(cloud_->cloud);
                cloud = std::move(*cloud_);
                cloud_.reset();
            } else if (path_ ||
                       (stopping_ && drain_ && !pending_path_points_.empty())) {
                kind = TaskKind::Path;
                path.timestamp = path_ ? path_->timestamp : last_path_timestamp_;
                path_.reset();
                pending_path_points.swap(pending_path_points_);
                task_points = pending_path_points.size();
                task_bytes = kPathTaskBytes +
                    pending_path_points.size() * sizeof(V3D);
            }

            busy_ = true;
            in_callback_ = true;
            inflight_points_ = task_points;
            inflight_bytes_ = task_bytes;
            refreshStatsLocked();
        }

        bool failed = false;
        const auto begin = std::chrono::steady_clock::now();
        try {
            switch (kind) {
                case TaskKind::Playback:
                    callbacks_.playback(playback);
                    break;
                case TaskKind::Time:
                    callbacks_.time(time);
                    break;
                case TaskKind::Pose:
                    callbacks_.pose(pose.position, pose.orientation,
                                    pose.timestamp);
                    break;
                case TaskKind::Imu:
                    callbacks_.imu(imu);
                    break;
                case TaskKind::Cloud:
                    callbacks_.cloud(cloud.cloud, cloud.timestamp);
                    break;
                case TaskKind::Path: {
                    uint64_t trimmed = 0;
                    for (auto& point : pending_path_points) {
                        path_history_.push_back(std::move(point));
                    }
                    while (path_history_.size() > config_.max_path_points) {
                        path_history_.pop_front();
                        ++trimmed;
                    }
                    std::vector<V3D> snapshot(path_history_.begin(),
                                              path_history_.end());
                    callbacks_.path(snapshot, path.timestamp);
                    if (trimmed > 0) {
                        std::lock_guard<std::mutex> lock(mutex_);
                        stats_.path_points_trimmed += trimmed;
                    }
                    break;
                }
                case TaskKind::None:
                    break;
            }
        } catch (const std::exception& error) {
            failed = true;
            std::cerr << "[RealtimeFoxgloveWorker] output failed: "
                      << error.what() << std::endl;
        } catch (...) {
            failed = true;
            std::cerr << "[RealtimeFoxgloveWorker] output failed with unknown error"
                      << std::endl;
        }
        const double elapsed_ms = std::chrono::duration<double, std::milli>(
            std::chrono::steady_clock::now() - begin).count();

        {
            std::lock_guard<std::mutex> lock(mutex_);
            ChannelStats* channel = nullptr;
            switch (kind) {
                case TaskKind::Playback: channel = &stats_.playback; break;
                case TaskKind::Time: channel = &stats_.time; break;
                case TaskKind::Pose: channel = &stats_.pose; break;
                case TaskKind::Imu: channel = &stats_.imu; break;
                case TaskKind::Cloud: channel = &stats_.cloud; break;
                case TaskKind::Path: channel = &stats_.path; break;
                case TaskKind::None: break;
            }
            if (channel) {
                channel->last_publish_ms = elapsed_ms;
                if (failed) {
                    ++channel->failures;
                    ++stats_.failures;
                } else {
                    ++channel->published;
                }
            }
            busy_ = false;
            in_callback_ = false;
            inflight_points_ = 0;
            inflight_bytes_ = 0;
            stats_.path_history_points = path_history_.size();
            stats_.path_history_bytes =
                path_history_.size() * sizeof(V3D);
            refreshStatsLocked();
        }
        idle_cv_.notify_all();
    }

    {
        std::lock_guard<std::mutex> lock(mutex_);
        busy_ = false;
        in_callback_ = false;
        worker_exited_ = true;
    }
    idle_cv_.notify_all();
    exit_cv_.notify_all();
}

void RealtimeFoxgloveWorker::refreshStatsLocked()
{
    size_t tasks = 0;
    size_t points = 0;
    size_t bytes = 0;
    if (playback_) { ++tasks; bytes += kControlTaskBytes; }
    if (time_) { ++tasks; bytes += kControlTaskBytes; }
    if (pose_) { ++tasks; bytes += kPoseTaskBytes; }
    if (imu_) { ++tasks; bytes += kImuTaskBytes; }
    if (cloud_) {
        ++tasks;
        points += cloud_->cloud.size();
        bytes += cloudTaskBytes(cloud_->cloud);
    }
    if (path_) {
        ++tasks;
        bytes += kPathTaskBytes;
    }
    if (!pending_path_points_.empty()) {
        if (!path_) ++tasks;
        points += pending_path_points_.size();
        bytes += pending_path_points_.size() * sizeof(V3D);
    }
    stats_.current_tasks = tasks;
    stats_.current_points = points;
    stats_.current_bytes = bytes;
    stats_.max_tasks = std::max(
        stats_.max_tasks, saturatedAddSize(tasks, busy_ ? 1U : 0U));
    stats_.max_points = std::max(
        stats_.max_points, saturatedAddSize(points, inflight_points_));
    stats_.max_bytes = std::max(
        stats_.max_bytes, saturatedAddSize(bytes, inflight_bytes_));
    stats_.inflight_points = inflight_points_;
    stats_.inflight_bytes = inflight_bytes_;
    stats_.busy = busy_;
}
