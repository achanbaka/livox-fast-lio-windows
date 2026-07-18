#ifndef FOXGLOVE_OUTPUT_WORKER_H
#define FOXGLOVE_OUTPUT_WORKER_H

#include "bounded_timing_stats.h"
#include "tiled_map_store.h"

#include <chrono>
#include <condition_variable>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <mutex>
#include <thread>
#include <unordered_map>

class FoxgloveOutputWorker
{
public:
    using PublishCallback = std::function<void(const PointCloudXYZI&, double)>;
    using TilePublishCallback = std::function<void(const TileUpdate&, double)>;
    using TileFailureCallback = std::function<void(const TileUpdate&, double)>;

    struct Config {
        size_t max_pending_tiles = 256;
        size_t max_pending_points = 200000;
        size_t max_pending_bytes = 64ULL * 1024ULL * 1024ULL;
        double tile_publish_hz = 0.0;
        // A continuously dirty tiled map must not starve the compatible
        // PointCloud snapshot. When a full snapshot is pending, publish it
        // after at most this many Tile callbacks. Zero prioritizes the full
        // snapshot immediately.
        size_t max_tile_burst_before_full = 8;
    };

    struct Stats {
        uint64_t full_enqueued = 0;
        uint64_t full_published = 0;
        uint64_t full_overwritten = 0;
        uint64_t full_dropped = 0;
        uint64_t tiles_enqueued = 0;
        uint64_t tiles_published = 0;
        uint64_t tiles_merged = 0;
        uint64_t tiles_dropped = 0;
        uint64_t tiles_retried = 0;
        uint64_t full_cancelled = 0;
        uint64_t tiles_cancelled = 0;
        uint64_t stop_timeouts = 0;
        uint64_t failures = 0;
        size_t current_tasks = 0;
        size_t current_points = 0;
        size_t current_bytes = 0;
        size_t max_bytes = 0;
        size_t inflight_points = 0;
        size_t inflight_bytes = 0;
        double last_full_publish_ms = 0.0;
        double last_tile_publish_ms = 0.0;
        bool busy = false;
        bool in_callback = false;
        bool stopping = false;
        bool worker_exited = false;
    };

    struct TimingStats {
        TimingWindowSummary full_publish;
        TimingWindowSummary tile_publish;
    };

    FoxgloveOutputWorker(PublishCallback full_callback,
                         TilePublishCallback tile_callback,
                         Config config = Config{},
                         TileFailureCallback tile_failure_callback =
                             TileFailureCallback{});
    ~FoxgloveOutputWorker();

    FoxgloveOutputWorker(const FoxgloveOutputWorker&) = delete;
    FoxgloveOutputWorker& operator=(const FoxgloveOutputWorker&) = delete;

    void start();
    bool enqueueFull(PointCloudXYZI cloud, double timestamp);
    bool enqueueTiles(std::vector<TileUpdate> updates, double timestamp);
    bool flushFor(std::chrono::milliseconds timeout);
    // Requests shutdown and joins only after the worker has reported exit.
    // A false result leaves the object in the stopping state so the caller can
    // retry (usually with drain=false). An already-running publish callback is
    // synchronous and cannot be cancelled safely; the object must outlive it.
    bool stopFor(std::chrono::milliseconds timeout, bool drain = true);
    void stop(bool drain = true);
    bool isRunning() const;
    Stats stats() const;
    TimingStats timingStats() const;

private:
    struct FullTask {
        PointCloudXYZI cloud;
        double timestamp = 0.0;
    };
    struct TileTask {
        TileUpdate update;
        double timestamp = 0.0;
    };

    void run();
    void refreshQueueStatsLocked();

    PublishCallback full_callback_;
    TilePublishCallback tile_callback_;
    TileFailureCallback tile_failure_callback_;
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
    bool has_full_ = false;
    FullTask full_;
    std::unordered_map<TileKey, TileTask, TileKeyHash> tiles_;
    // One failed Tile remains charged as the in-flight reservation until it
    // succeeds, is superseded, or a non-draining stop cancels it. This fixed
    // slot also preserves evicted-tile tombstones that the map store can no
    // longer reconstruct during a full resync.
    bool has_retry_tile_ = false;
    TileTask retry_tile_;
    size_t inflight_points_ = 0;
    size_t inflight_bytes_ = 0;
    bool has_inflight_tile_ = false;
    TileKey inflight_tile_key_;
    uint64_t inflight_tile_version_ = 0;
    size_t tiles_since_full_ = 0;
    std::chrono::steady_clock::time_point last_tile_publish_{};
    Stats stats_;
    BoundedTimingWindow<> full_publish_timing_;
    BoundedTimingWindow<> tile_publish_timing_;
};

#endif
