#ifndef MAP_BUILD_WORKER_H
#define MAP_BUILD_WORKER_H

#include "foxglove_output_worker.h"
#include "tiled_map_store.h"

#include <chrono>
#include <condition_variable>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <list>
#include <mutex>
#include <thread>
#include <unordered_map>

class MapBuildWorker
{
public:
    using PublishCallback = FoxgloveOutputWorker::PublishCallback;
    using TilePublishCallback = FoxgloveOutputWorker::TilePublishCallback;

    struct Config {
        TiledMapStore::Config store;
        double full_voxel_leaf_m = 0.2;
        size_t input_queue_capacity = 64;
        size_t input_queue_max_points = 1000000;
        size_t input_queue_max_bytes = 128ULL * 1024ULL * 1024ULL;
        size_t max_tiles_per_update = 32;
        size_t max_points_per_update = 200000;
        size_t max_bytes_per_update = 32ULL * 1024ULL * 1024ULL;
        FoxgloveOutputWorker::Config output;
    };

    struct Stats {
        uint64_t frames_enqueued = 0;
        uint64_t frames_built = 0;
        uint64_t frames_dropped = 0;
        uint64_t input_incomplete = 0;
        uint64_t input_points_dropped = 0;
        uint64_t input_bytes_dropped = 0;
        uint64_t input_tile_changes_enqueued = 0;
        uint64_t input_tile_changes_merged = 0;
        uint64_t input_tile_changes_dropped = 0;
        uint64_t input_tile_tasks_evicted = 0;
        uint64_t input_resync_requested = 0;
        uint64_t full_requested = 0;
        uint64_t full_built = 0;
        uint64_t full_coalesced = 0;
        uint64_t tile_resync_requested = 0;
        uint64_t tile_resync_on_failure = 0;
        uint64_t tile_resync_completed = 0;
        uint64_t tile_batches = 0;
        uint64_t tile_updates = 0;
        uint64_t frames_cancelled = 0;
        uint64_t control_tasks_cancelled = 0;
        uint64_t stop_timeouts = 0;
        uint64_t failures = 0;
        size_t input_queue_tasks = 0;
        size_t input_queue_points = 0;
        size_t input_queue_bytes = 0;
        size_t max_input_queue_tasks = 0;
        size_t max_input_queue_points = 0;
        size_t max_input_queue_bytes = 0;
        size_t last_frame_points = 0;
        size_t last_full_points = 0;
        double last_build_ms = 0.0;
        double last_tile_extract_ms = 0.0;
        double last_full_snapshot_ms = 0.0;
        bool incomplete = false;
        bool busy = false;
        bool stopping = false;
        bool worker_exited = false;
        TiledMapStore::Stats store;
        FoxgloveOutputWorker::Stats output;
    };

    struct TimingStats {
        TimingWindowSummary map_build;
        TimingWindowSummary tile_extract;
        TimingWindowSummary full_snapshot;
        FoxgloveOutputWorker::TimingStats output;
    };

    MapBuildWorker(Config config,
                   PublishCallback full_callback,
                   TilePublishCallback tile_callback = TilePublishCallback{});
    ~MapBuildWorker();

    MapBuildWorker(const MapBuildWorker&) = delete;
    MapBuildWorker& operator=(const MapBuildWorker&) = delete;

    void start();
    bool enqueueFrame(PointCloudXYZI frame, double timestamp,
                      const PointType* current_position = nullptr);
    void request(double timestamp);
    void requestTileResync(double timestamp = 0.0);
    bool flushFor(std::chrono::milliseconds timeout);
    void flush();
    // Uses one deadline for both the map-builder and output workers. On
    // timeout, no unbounded join is performed. Retry with drain=false to
    // cancel queued work; an in-flight build or output callback is not safely
    // cancellable and the object must remain alive until a later successful
    // stopFor()/stop().
    bool stopFor(std::chrono::milliseconds timeout, bool drain = true);
    void stop(bool drain = true);
    bool isRunning() const;
    Stats stats() const;
    TimingStats timingStats() const;

private:
    struct TileContribution {
        PointCloudXYZI cloud;
        double timestamp = 0.0;
        uint64_t sequence = 0;
        PointType current_position;
        bool has_current_position = false;
    };

    struct PendingTileKey {
        TileKey tile;
        uint64_t epoch = 0;

        bool operator==(const PendingTileKey& other) const noexcept
        {
            return tile == other.tile && epoch == other.epoch;
        }
    };

    struct PendingTileKeyHash {
        size_t operator()(const PendingTileKey& key) const noexcept;
    };

    struct TileTask {
        PendingTileKey key;
        std::list<TileContribution> contributions;
        size_t point_count = 0;
    };

    struct FrameProgress {
        size_t pending_contributions = 0;
        bool sealed = false;
        bool any_built = false;
        bool incomplete = false;
        bool cancelled = false;
    };

    using TileTaskList = std::list<TileTask>;
    using TileTaskIndex = std::unordered_map<
        PendingTileKey, TileTaskList::iterator, PendingTileKeyHash>;

    void run();
    void refreshInputStatsLocked();
    bool emitDirtyTiles(double timestamp);
    size_t inputPointLimit() const noexcept;
    size_t estimateInputBytesLocked() const noexcept;
    size_t estimateTileTaskBytes(const TileTask& task) const noexcept;
    void advanceBuiltSequenceLocked();
    void completeContributionLocked(uint64_t sequence, bool built,
                                    bool cancelled = false);
    void recordInputLossLocked(uint64_t sequence, size_t points);
    size_t discardOldestPointsLocked(TileTaskList::iterator task,
                                     size_t requested);
    void evictTileTaskLocked(TileTaskList::iterator task);
    void eraseEmptyTileTaskLocked(TileTaskList::iterator task);
    void scheduleInputResyncLocked(double timestamp, uint64_t sequence);
    size_t cancelQueuedInputLocked();

    Config config_;
    TiledMapStore store_;
    bool tile_output_enabled_ = false;
    FoxgloveOutputWorker output_;
    mutable std::mutex mutex_;
    std::condition_variable work_cv_;
    std::condition_variable idle_cv_;
    std::condition_variable exit_cv_;
    std::thread worker_;
    bool running_ = false;
    bool stopping_ = false;
    bool drain_ = true;
    bool busy_ = false;
    bool worker_exited_ = true;
    bool full_pending_ = false;
    bool tile_resync_pending_ = false;
    bool tile_resync_in_progress_ = false;
    bool dirty_drain_pending_ = false;
    double full_timestamp_ = 0.0;
    double dirty_timestamp_ = 0.0;
    uint64_t full_sequence_ = 0;
    uint64_t tile_resync_sequence_ = 0;
    uint64_t next_sequence_ = 0;
    uint64_t built_sequence_ = 0;
    uint64_t input_epoch_ = 0;
    size_t pending_input_points_ = 0;
    size_t pending_input_bytes_ = 0;
    TileTaskList tile_tasks_;
    TileTaskIndex tile_task_index_;
    std::unordered_map<uint64_t, FrameProgress> frame_progress_;
    Stats stats_;
    BoundedTimingWindow<> map_build_timing_;
    BoundedTimingWindow<> tile_extract_timing_;
    BoundedTimingWindow<> full_snapshot_timing_;
};

#endif
