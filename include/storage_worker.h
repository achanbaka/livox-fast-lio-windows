#ifndef STORAGE_WORKER_H
#define STORAGE_WORKER_H

#include "bag_writer.h"
#include "bounded_timing_stats.h"
#include "types.h"

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <functional>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

class StorageWorker
{
public:
    enum class Mode { Realtime, Reliable };
    enum class PcdFormat { Binary, BinaryCompressed };

    struct Config {
        static constexpr size_t kDefaultPcdChunkPoints = 1000000;

        Mode mode = Mode::Realtime;
        size_t queue_max_bytes = 512ULL * 1024ULL * 1024ULL;
        double bag_path_publish_hz = 1.0;
        PcdFormat pcd_format = PcdFormat::BinaryCompressed;
        size_t pcd_chunk_points = kDefaultPcdChunkPoints;
        // Optional legacy-compatible frame-count chunk trigger. Zero disables
        // it; the point-count bound remains independently active.
        size_t pcd_chunk_frames = 0;
        std::string output_root = ".";
        std::string bag_path;
        bool enable_pcd = false;
        // Optional diagnostics/fault-injection hook. Production leaves this
        // empty; tests can throw for "write_message" or "close".
        BagWriter::IoHook bag_io_hook;
        // Optional diagnostics/fault-injection hook. Called immediately
        // before a PCD file write; throwing simulates an I/O failure without
        // requiring filesystem races in tests.
        std::function<void(const char* operation)> pcd_io_hook;
        // Optional startup diagnostics/fault-injection hook. Production
        // leaves this empty; tests may block at "before_open".
        std::function<void(const char* operation)> startup_io_hook;
    };

    struct Stats {
        uint64_t tasks_enqueued = 0;
        uint64_t tasks_written = 0;
        uint64_t tasks_dropped = 0;
        uint64_t tasks_overwritten = 0;
        uint64_t tasks_cancelled = 0;
        uint64_t stop_timeouts = 0;
        uint64_t startup_timeouts = 0;
        uint64_t failures = 0;
        uint64_t bag_failures = 0;
        uint64_t bag_tasks_cancelled = 0;
        uint64_t pcd_failures = 0;
        uint64_t pcd_tasks_dropped = 0;
        uint64_t pcd_points_dropped = 0;
        uint64_t pcd_bytes_dropped = 0;
        uint64_t pcd_tasks_cancelled = 0;
        uint64_t pcd_points_cancelled = 0;
        uint64_t pcd_bytes_cancelled = 0;
        uint64_t pcd_chunks_written = 0;
        size_t queue_tasks = 0;
        size_t queue_bytes = 0;
        size_t max_queue_bytes = 0;
        // The queue hard limit applies to queued + currently executing Task
        // payloads. PCD accumulation and the explicit chunk staging buffer
        // are separate bounded budgets reported below.
        size_t inflight_tasks = 0;
        size_t inflight_points = 0;
        size_t inflight_bytes = 0;
        size_t hard_usage_bytes = 0;
        size_t max_hard_usage_bytes = 0;
        size_t pcd_pending_points = 0;
        size_t pcd_pending_bytes = 0;
        size_t max_pcd_pending_bytes = 0;
        size_t pcd_pending_capacity_points = 0;
        size_t pcd_pending_tasks = 0;
        size_t pcd_scratch_bytes = 0;
        size_t max_pcd_scratch_bytes = 0;
        size_t pcd_queue_points = 0;
        size_t pcd_queue_bytes = 0;
        size_t pcd_queue_tasks = 0;
        size_t pcd_chunk_points_limit = 0;
        size_t pcd_chunk_frames_limit = 0;
        double last_write_ms = 0.0;
        double last_flush_ms = 0.0;
        bool failed = false;
        bool bag_disabled = false;
        bool pcd_disabled = false;
        bool busy = false;
        bool in_io = false;
        bool stopping = false;
        bool worker_exited = false;
    };

    struct TimingStats {
        TimingWindowSummary write;
        TimingWindowSummary bag_write;
        TimingWindowSummary pcd_write;
        TimingWindowSummary flush;
    };

    explicit StorageWorker(Config config);
    ~StorageWorker();

    StorageWorker(const StorageWorker&) = delete;
    StorageWorker& operator=(const StorageWorker&) = delete;

    // StorageWorker is intentionally one-shot. Any second start/startFor call
    // is rejected, including after a startup or worker failure.
    bool start();
    // Bounded startup. A timeout marks startup failed and requests worker
    // shutdown. Third-party synchronous open calls are not forcibly
    // interruptible; the object must remain alive until stopFor() succeeds.
    bool startFor(std::chrono::milliseconds timeout);
    bool enqueueOdometryTf(uint64_t time_ns, const V3D& position,
                           const Eigen::Quaterniond& orientation);
    bool enqueueCloud(uint64_t time_ns, PointCloudXYZI cloud,
                      std::string frame_id, std::string topic);
    bool enqueuePath(uint64_t time_ns, const std::vector<V3D>& path,
                     std::string frame_id);
    bool enqueuePcd(PointCloudXYZI cloud);
    bool flushFor(std::chrono::milliseconds timeout);
    // Cancels queued work when drain=false and joins only after the worker has
    // reported exit. Synchronous bag/PCD I/O already in progress cannot be
    // interrupted portably; false means the object must stay alive and the
    // caller should retry after the I/O returns.
    bool stopFor(std::chrono::milliseconds timeout, bool drain = true);
    void stop(bool drain = true);
    bool isRunning() const;
    bool isBagOpen() const { return bag_open_.load(); }
    bool isPcdEnabled() const { return pcd_enabled_.load(); }
    Stats stats() const;
    TimingStats timingStats() const;

    static Mode parseMode(const std::string& value);
    static PcdFormat parsePcdFormat(const std::string& value);

private:
    enum class TaskType { OdomTf, Cloud, Path, Pcd };
    struct Task {
        TaskType type = TaskType::Cloud;
        uint64_t time_ns = 0;
        V3D position = V3D::Zero();
        Eigen::Quaterniond orientation = Eigen::Quaterniond::Identity();
        PointCloudXYZI cloud;
        std::vector<V3D> path;
        std::string frame_id;
        std::string topic;
        size_t points = 0;
        size_t bytes = 0;
    };

    bool enqueue(Task task);
    static bool isBagTask(TaskType type);
    void fuseBagOutputAfterWriteFailure();
    void fusePcdOutputAfterWriteFailure();
    void run();
    uint64_t process(Task& task);
    void writePcdChunk(PointCloudXYZI& cloud, bool final_chunk);
    void commitPcdPoints(size_t point_count);
    void setPcdScratchBytes(size_t bytes);
    void releasePcdPendingStorage() noexcept;
    void compactPcdPendingStorage() noexcept;
    size_t pendingPcdPointCount() const;
    void writePendingPcdChunk(bool final_chunk);
    void cancelPendingPcdLocked();
    void cancelQueuedTasksLocked();
    void accountDroppedTaskLocked(const Task& task);
    void accountCancelledTaskLocked(const Task& task);
    void accountQueuedTaskAddedLocked(const Task& task);
    void accountQueuedTaskRemovedLocked(const Task& task);
    void setInflightTaskLocked(const Task& task);
    void clearInflightTaskLocked();
    void updatePcdStatsLocked();
    void refreshStatsLocked();
    static size_t saturatedAdd(size_t left, size_t right);
    static size_t saturatedMultiply(size_t left, size_t right);
    static size_t taskResidentBytes(const Task& task);

    Config config_;
    mutable std::mutex mutex_;
    std::condition_variable work_cv_;
    std::condition_variable ready_cv_;
    std::condition_variable idle_cv_;
    std::condition_variable exit_cv_;
    std::thread worker_;
    std::deque<Task> tasks_;
    bool running_ = false;
    bool stopping_ = false;
    bool drain_ = true;
    bool busy_ = false;
    bool in_io_ = false;
    bool worker_exited_ = true;
    bool startup_complete_ = false;
    bool startup_ok_ = true;
    bool start_attempted_ = false;
    bool startup_timed_out_ = false;
    bool reliable_admission_failed_ = false;
    bool fatal_sink_failure_ = false;
    bool flush_requested_ = false;
    // flushFor callers that overlap share one generation. The worker records
    // completion separately from success so an I/O exception cannot be
    // mistaken for a successfully drained storage barrier.
    uint64_t flush_requested_generation_ = 0;
    uint64_t flush_completed_generation_ = 0;
    uint64_t first_failed_flush_generation_ = 0;
    uint64_t last_path_time_ns_ = 0;
    size_t queued_bytes_ = 0;
    size_t inflight_tasks_ = 0;
    size_t inflight_points_ = 0;
    size_t inflight_bytes_ = 0;
    size_t pcd_scratch_bytes_ = 0;
    BagWriter bag_writer_;
    std::atomic<bool> bag_open_{false};
    std::atomic<bool> pcd_enabled_{false};
    PointCloudXYZI pcd_pending_;
    size_t pcd_pending_head_ = 0;
    std::deque<size_t> pcd_pending_task_points_;
    size_t queued_pcd_points_ = 0;
    size_t queued_pcd_bytes_ = 0;
    size_t queued_pcd_tasks_ = 0;
    uint64_t pcd_chunk_sequence_ = 0;
    uint64_t pcd_chunks_written_ = 0;
    Stats stats_;
    // Protected by mutex_. Each window has fixed capacity, so storage
    // observability remains bounded regardless of process uptime.
    BoundedTimingWindow<> write_timing_;
    BoundedTimingWindow<> bag_write_timing_;
    BoundedTimingWindow<> pcd_write_timing_;
    BoundedTimingWindow<> flush_timing_;
};

#endif
