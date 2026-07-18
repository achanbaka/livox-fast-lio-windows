#ifndef REALTIME_FOXGLOVE_WORKER_H
#define REALTIME_FOXGLOVE_WORKER_H

#include "foxglove_publisher.h"

#include <chrono>
#include <condition_variable>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <functional>
#include <mutex>
#include <optional>
#include <thread>
#include <vector>

// Bounded, latest-value realtime output queue.  This worker deliberately does
// not own FoxglovePublisher: its callbacks are invoked by one consumer thread,
// and the caller must stop the worker before stopping the publisher.
class RealtimeFoxgloveWorker
{
public:
    struct Callbacks {
        std::function<void(const ImuData&)> imu;
        std::function<void(const V3D&, const Eigen::Quaterniond&, double)> pose;
        std::function<void(const PointCloudXYZI&, double)> cloud;
        std::function<void(const std::vector<V3D>&, double)> path;
        std::function<void(double)> time;
        std::function<void(const FoxglovePublisher::PlaybackState&)> playback;
    };

    struct Config {
        size_t max_pending_points = 2000000;
        size_t max_pending_bytes = 128ULL * 1024ULL * 1024ULL;
        size_t max_path_points = 100000;
    };

    struct ChannelStats {
        uint64_t attempted = 0;
        uint64_t accepted = 0;
        uint64_t published = 0;
        uint64_t overwritten = 0;
        uint64_t dropped = 0;
        uint64_t failures = 0;
        double last_publish_ms = 0.0;
    };

    struct Stats {
        ChannelStats playback;
        ChannelStats time;
        ChannelStats pose;
        ChannelStats imu;
        ChannelStats cloud;
        ChannelStats path;
        uint64_t failures = 0;
        uint64_t path_points_received = 0;
        uint64_t path_points_dropped = 0;
        uint64_t path_points_trimmed = 0;
        uint64_t tasks_cancelled = 0;
        uint64_t stop_timeouts = 0;
        size_t current_tasks = 0;
        size_t current_points = 0;
        size_t current_bytes = 0;
        size_t max_tasks = 0;
        size_t max_points = 0;
        size_t max_bytes = 0;
        size_t inflight_points = 0;
        size_t inflight_bytes = 0;
        size_t path_history_points = 0;
        size_t path_history_bytes = 0;
        bool busy = false;
        bool in_callback = false;
        bool stopping = false;
        bool worker_exited = false;
    };

    explicit RealtimeFoxgloveWorker(Callbacks callbacks,
                                     Config config = Config{});
    ~RealtimeFoxgloveWorker();

    RealtimeFoxgloveWorker(const RealtimeFoxgloveWorker&) = delete;
    RealtimeFoxgloveWorker& operator=(const RealtimeFoxgloveWorker&) = delete;

    void start();
    bool tryEnqueueImu(ImuData imu);
    bool tryEnqueuePose(V3D position, Eigen::Quaterniond orientation,
                        double timestamp);
    bool tryEnqueueCloud(PointCloudXYZI cloud, double timestamp);
    bool tryEnqueuePathPoint(V3D position, double timestamp,
                             bool request_publish = true);
    bool tryEnqueueTime(double timestamp);
    bool tryEnqueuePlayback(FoxglovePublisher::PlaybackState state);
    bool flushFor(std::chrono::milliseconds timeout);
    // On timeout no join is attempted. Queued latest-value tasks are cancelled
    // when drain=false, while a callback already in progress remains
    // non-cancellable and requires the object to stay alive until retry.
    bool stopFor(std::chrono::milliseconds timeout, bool drain = true);
    void stop(bool drain = true);
    bool isRunning() const;
    Stats stats() const;

private:
    struct PoseTask {
        V3D position = V3D::Zero();
        Eigen::Quaterniond orientation = Eigen::Quaterniond::Identity();
        double timestamp = 0.0;
    };
    struct CloudTask {
        PointCloudXYZI cloud;
        double timestamp = 0.0;
    };
    struct PathTask {
        double timestamp = 0.0;
    };

    enum class TaskKind { None, Playback, Time, Pose, Imu, Cloud, Path };

    template <typename T>
    bool admitLatestLocked(std::optional<T>& slot, T task, size_t points,
                           size_t bytes, ChannelStats& channel);
    bool hasWorkLocked() const;
    void clearPendingLocked();
    void run();
    void refreshStatsLocked();

    Callbacks callbacks_;
    Config config_;
    mutable std::mutex mutex_;
    std::condition_variable work_cv_;
    std::condition_variable idle_cv_;
    std::condition_variable exit_cv_;
    std::thread worker_;
    bool running_ = false;
    bool stopping_ = false;
    bool drain_ = true;
    bool busy_ = false;
    bool in_callback_ = false;
    bool worker_exited_ = true;
    std::optional<FoxglovePublisher::PlaybackState> playback_;
    std::optional<double> time_;
    std::optional<PoseTask> pose_;
    std::optional<ImuData> imu_;
    std::optional<CloudTask> cloud_;
    std::optional<PathTask> path_;
    std::deque<V3D> pending_path_points_;
    std::deque<V3D> path_history_;
    double last_path_timestamp_ = 0.0;
    size_t inflight_points_ = 0;
    size_t inflight_bytes_ = 0;
    Stats stats_;
};

#endif
