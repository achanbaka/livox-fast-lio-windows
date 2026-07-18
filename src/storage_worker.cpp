#include "storage_worker.h"

#include "foxglove_publisher.h"
#include "ros_message.h"

#include <algorithm>
#include <filesystem>
#include <iostream>
#include <iterator>
#include <limits>
#include <pcl/io/pcd_io.h>
#include <sstream>
#include <stdexcept>
#include <utility>

StorageWorker::StorageWorker(Config config)
    : config_(std::move(config)), bag_writer_(config_.bag_io_hook)
{
    // Zero used to disable the split loop and let pcd_pending_ grow until
    // shutdown. Keep legacy configurations safe by restoring the documented,
    // finite default instead.
    if (config_.pcd_chunk_points == 0) {
        config_.pcd_chunk_points = Config::kDefaultPcdChunkPoints;
    }
}

StorageWorker::~StorageWorker()
{
    stop(false);
}

bool StorageWorker::start()
{
    return startFor(std::chrono::milliseconds::max());
}

bool StorageWorker::startFor(std::chrono::milliseconds timeout)
{
    std::unique_lock<std::mutex> lock(mutex_);
    if (start_attempted_) return false;
    start_attempted_ = true;
    const bool bounded = timeout != std::chrono::milliseconds::max();
    auto wait_for_startup = [this, bounded, timeout, &lock]() {
        if (!bounded) {
            ready_cv_.wait(lock, [this] { return startup_complete_; });
            return true;
        }
        return ready_cv_.wait_for(
            lock, timeout, [this] { return startup_complete_; });
    };
    stopping_ = false;
    drain_ = true;
    busy_ = false;
    in_io_ = false;
    worker_exited_ = false;
    startup_complete_ = false;
    startup_ok_ = true;
    startup_timed_out_ = false;
    reliable_admission_failed_ = false;
    fatal_sink_failure_ = false;
    flush_requested_ = false;
    flush_requested_generation_ = 0;
    flush_completed_generation_ = 0;
    first_failed_flush_generation_ = 0;
    std::deque<Task>().swap(tasks_);
    queued_bytes_ = 0;
    inflight_tasks_ = 0;
    inflight_points_ = 0;
    inflight_bytes_ = 0;
    pcd_scratch_bytes_ = 0;
    queued_pcd_points_ = 0;
    queued_pcd_bytes_ = 0;
    queued_pcd_tasks_ = 0;
    releasePcdPendingStorage();
    bag_open_ = false;
    pcd_enabled_ = false;
    pcd_chunks_written_ = 0;
    pcd_chunk_sequence_ = 0;
    last_path_time_ns_ = 0;
    stats_ = Stats{};
    write_timing_ = BoundedTimingWindow<>{};
    bag_write_timing_ = BoundedTimingWindow<>{};
    pcd_write_timing_ = BoundedTimingWindow<>{};
    flush_timing_ = BoundedTimingWindow<>{};
    stats_.pcd_chunk_points_limit = config_.pcd_chunk_points;
    stats_.pcd_chunk_frames_limit = config_.pcd_chunk_frames;
    running_ = true;
    try {
        worker_ = std::thread(&StorageWorker::run, this);
    } catch (...) {
        running_ = false;
        worker_exited_ = true;
        startup_complete_ = true;
        startup_ok_ = false;
        stats_.failed = true;
        ++stats_.failures;
        return false;
    }
    if (!wait_for_startup()) {
        startup_timed_out_ = true;
        startup_ok_ = false;
        stopping_ = true;
        drain_ = false;
        stats_.failed = true;
        ++stats_.failures;
        ++stats_.startup_timeouts;
        work_cv_.notify_all();
        return false;
    }
    return startup_ok_;
}

bool StorageWorker::enqueueOdometryTf(
    uint64_t time_ns, const V3D& position,
    const Eigen::Quaterniond& orientation)
{
    if (!bag_open_) return false;
    Task task;
    task.type = TaskType::OdomTf;
    task.time_ns = time_ns;
    task.position = position;
    task.orientation = orientation;
    task.bytes = taskResidentBytes(task);
    return enqueue(std::move(task));
}

bool StorageWorker::enqueueCloud(
    uint64_t time_ns, PointCloudXYZI cloud,
    std::string frame_id, std::string topic)
{
    if (!bag_open_ || cloud.empty()) return false;
    Task task;
    task.type = TaskType::Cloud;
    task.time_ns = time_ns;
    task.cloud = std::move(cloud);
    task.frame_id = std::move(frame_id);
    task.topic = std::move(topic);
    task.points = task.cloud.size();
    task.bytes = taskResidentBytes(task);
    return enqueue(std::move(task));
}

bool StorageWorker::enqueuePath(
    uint64_t time_ns, const std::vector<V3D>& path, std::string frame_id)
{
    if (!bag_open_ || path.empty() || config_.bag_path_publish_hz <= 0.0) return false;
    {
        std::lock_guard<std::mutex> lock(mutex_);
        const uint64_t interval = static_cast<uint64_t>(
            1e9 / config_.bag_path_publish_hz);
        if (last_path_time_ns_ != 0 && time_ns < last_path_time_ns_ + interval) {
            return false;
        }
        last_path_time_ns_ = time_ns;
    }
    Task task;
    task.type = TaskType::Path;
    task.time_ns = time_ns;
    task.path = path;
    task.frame_id = std::move(frame_id);
    task.points = task.path.size();
    task.bytes = taskResidentBytes(task);
    return enqueue(std::move(task));
}

bool StorageWorker::enqueuePcd(PointCloudXYZI cloud)
{
    if (!pcd_enabled_.load() || cloud.empty()) return false;
    Task task;
    task.type = TaskType::Pcd;
    task.cloud = std::move(cloud);
    task.points = task.cloud.size();
    task.bytes = taskResidentBytes(task);
    return enqueue(std::move(task));
}

bool StorageWorker::enqueue(Task task)
{
    std::lock_guard<std::mutex> lock(mutex_);
    if (!running_ || stopping_ || stats_.failed) return false;
    // Recheck under the queue mutex so a producer that observed bag_open_
    // immediately before a write failure cannot enqueue behind the fuse.
    if (isBagTask(task.type) && !bag_open_.load()) return false;
    if (task.type == TaskType::Pcd && !pcd_enabled_.load()) return false;
    auto fail_reliable_admission = [this]() {
        stats_.failed = true;
        reliable_admission_failed_ = true;
        ++stats_.failures;
        work_cv_.notify_all();
    };

    // A queued Path supersedes an older full-history Path message in realtime
    // mode. Erase the old task and append the replacement at the tail so its
    // newer timestamp cannot move ahead of intervening odometry/cloud tasks.
    if (config_.mode == Mode::Realtime && task.type == TaskType::Path) {
        const auto existing = std::find_if(
            tasks_.rbegin(), tasks_.rend(),
            [](const Task& queued) { return queued.type == TaskType::Path; });
        if (existing != tasks_.rend()) {
            const auto erase_position = std::prev(existing.base());
            queued_bytes_ -= erase_position->bytes;
            accountQueuedTaskRemovedLocked(*erase_position);
            tasks_.erase(erase_position);
            ++stats_.tasks_overwritten;
        }
    }
    refreshStatsLocked();
    auto projected_usage = [this, &task]() {
        return saturatedAdd(
            saturatedAdd(queued_bytes_, inflight_bytes_), task.bytes);
    };
    while (config_.queue_max_bytes > 0 && !tasks_.empty() &&
           projected_usage() > config_.queue_max_bytes)
    {
        if (config_.mode == Mode::Reliable) {
            accountDroppedTaskLocked(task);
            fail_reliable_admission();
            return false;
        }
        queued_bytes_ -= tasks_.front().bytes;
        accountQueuedTaskRemovedLocked(tasks_.front());
        accountDroppedTaskLocked(tasks_.front());
        tasks_.pop_front();
        refreshStatsLocked();
    }
    if (config_.queue_max_bytes > 0 &&
        projected_usage() > config_.queue_max_bytes)
    {
        accountDroppedTaskLocked(task);
        if (config_.mode == Mode::Reliable) fail_reliable_admission();
        return false;
    }
    tasks_.push_back(std::move(task));
    queued_bytes_ = saturatedAdd(queued_bytes_, tasks_.back().bytes);
    accountQueuedTaskAddedLocked(tasks_.back());
    ++stats_.tasks_enqueued;
    refreshStatsLocked();
    work_cv_.notify_one();
    return true;
}

bool StorageWorker::isBagTask(TaskType type)
{
    return type == TaskType::OdomTf || type == TaskType::Cloud ||
           type == TaskType::Path;
}

void StorageWorker::fuseBagOutputAfterWriteFailure()
{
    const bool was_open = bag_open_.exchange(false);
    {
        std::lock_guard<std::mutex> lock(mutex_);
        stats_.bag_disabled = true;
        ++stats_.bag_failures;
        for (auto it = tasks_.begin(); it != tasks_.end();) {
            if (!isBagTask(it->type)) {
                ++it;
                continue;
            }
            queued_bytes_ -= it->bytes;
            accountCancelledTaskLocked(*it);
            it = tasks_.erase(it);
        }
        if (config_.mode == Mode::Reliable && !pcd_enabled_.load()) {
            stats_.failed = true;
            fatal_sink_failure_ = true;
            work_cv_.notify_all();
        }
        refreshStatsLocked();
    }

    if (!was_open) return;
    try {
        bag_writer_.close();
    } catch (const std::exception& error) {
        std::cerr << "[StorageWorker] bag close after write failure failed: "
                  << error.what() << std::endl;
        std::lock_guard<std::mutex> lock(mutex_);
        ++stats_.failures;
        ++stats_.bag_failures;
    } catch (...) {
        std::cerr << "[StorageWorker] bag close after write failure failed with "
                     "unknown error" << std::endl;
        std::lock_guard<std::mutex> lock(mutex_);
        ++stats_.failures;
        ++stats_.bag_failures;
    }
}

void StorageWorker::fusePcdOutputAfterWriteFailure()
{
    pcd_enabled_.store(false);
    std::lock_guard<std::mutex> lock(mutex_);
    stats_.pcd_disabled = true;
    ++stats_.pcd_failures;

    if (config_.mode == Mode::Reliable) {
        // Preserve both the in-memory accumulator and the queued tasks.  The
        // caller can report exactly what remains unwritten, and the worker
        // exits without consuming unrelated storage work after a reliable
        // sink has failed.
        stats_.failed = true;
        refreshStatsLocked();
        return;
    }

    const uint64_t pending_points =
        static_cast<uint64_t>(pendingPcdPointCount());
    const uint64_t dropped_tasks =
        static_cast<uint64_t>(pcd_pending_task_points_.size());
    stats_.tasks_dropped += dropped_tasks;
    stats_.pcd_tasks_dropped += dropped_tasks;
    stats_.pcd_points_dropped += pending_points;
    stats_.pcd_bytes_dropped +=
        pending_points * static_cast<uint64_t>(sizeof(PointType));
    for (auto it = tasks_.begin(); it != tasks_.end();) {
        if (it->type != TaskType::Pcd) {
            ++it;
            continue;
        }
        queued_bytes_ -= it->bytes;
        accountQueuedTaskRemovedLocked(*it);
        accountCancelledTaskLocked(*it);
        it = tasks_.erase(it);
    }
    releasePcdPendingStorage();
    if (tasks_.empty()) std::deque<Task>().swap(tasks_);
    updatePcdStatsLocked();
    refreshStatsLocked();
}

bool StorageWorker::flushFor(std::chrono::milliseconds timeout)
{
    std::unique_lock<std::mutex> lock(mutex_);
    if (!running_ || stopping_ || worker_exited_ || stats_.failed) return false;
    if (!flush_requested_) {
        ++flush_requested_generation_;
        flush_requested_ = true;
        work_cv_.notify_one();
    }
    const uint64_t generation = flush_requested_generation_;
    const bool completed = idle_cv_.wait_for(
        lock, timeout, [this, generation] {
            return flush_completed_generation_ >= generation ||
                   worker_exited_ || stats_.failed;
        });
    if (!completed || flush_completed_generation_ < generation) return false;
    return first_failed_flush_generation_ == 0 ||
           generation < first_failed_flush_generation_;
}

bool StorageWorker::stopFor(
    std::chrono::milliseconds timeout, bool drain)
{
    const auto deadline = std::chrono::steady_clock::now() + timeout;
    {
        std::unique_lock<std::mutex> lock(mutex_);
        if (!running_) {
            if (!drain) {
                cancelQueuedTasksLocked();
                cancelPendingPcdLocked();
            }
            return true;
        }
        if (!stopping_) {
            stopping_ = true;
            drain_ = drain;
        } else if (!drain) {
            drain_ = false;
        }
        if (!drain_) {
            cancelQueuedTasksLocked();
            flush_requested_ = false;
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
    if (!drain_) {
        cancelQueuedTasksLocked();
        cancelPendingPcdLocked();
    }
    running_ = false;
    stopping_ = false;
    busy_ = false;
    in_io_ = false;
    refreshStatsLocked();
    idle_cv_.notify_all();
    return true;
}

void StorageWorker::stop(bool drain)
{
    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (!running_) {
            if (!drain) {
                cancelQueuedTasksLocked();
                cancelPendingPcdLocked();
            }
            return;
        }
        if (!stopping_) {
            stopping_ = true;
            drain_ = drain;
        } else if (!drain) {
            drain_ = false;
        }
        if (!drain_) {
            cancelQueuedTasksLocked();
            flush_requested_ = false;
        }
    }
    work_cv_.notify_all();
    if (worker_.joinable()) worker_.join();
    std::lock_guard<std::mutex> lock(mutex_);
    if (!drain_) {
        cancelQueuedTasksLocked();
        cancelPendingPcdLocked();
    }
    running_ = false;
    stopping_ = false;
    busy_ = false;
    in_io_ = false;
    refreshStatsLocked();
    idle_cv_.notify_all();
}

bool StorageWorker::isRunning() const
{
    std::lock_guard<std::mutex> lock(mutex_);
    return running_ && !stopping_ && !stats_.failed;
}

StorageWorker::Stats StorageWorker::stats() const
{
    std::lock_guard<std::mutex> lock(mutex_);
    Stats result = stats_;
    result.busy = busy_;
    result.in_io = in_io_;
    result.stopping = stopping_;
    result.worker_exited = worker_exited_;
    return result;
}

StorageWorker::TimingStats StorageWorker::timingStats() const
{
    std::lock_guard<std::mutex> lock(mutex_);
    return TimingStats{
        write_timing_.summary(),
        bag_write_timing_.summary(),
        pcd_write_timing_.summary(),
        flush_timing_.summary()};
}

void StorageWorker::run()
{
    const bool bag_requested = !config_.bag_path.empty();
    const bool pcd_requested = config_.enable_pcd;
    bool bag_available = false;
    bool pcd_available = false;
    bool startup_prerequisite_ok = true;

    try {
        if (config_.startup_io_hook) {
            config_.startup_io_hook("before_open");
        }
    } catch (const std::exception& error) {
        std::cerr << "[StorageWorker] startup hook failed: "
                  << error.what() << std::endl;
        startup_prerequisite_ok = false;
        std::lock_guard<std::mutex> lock(mutex_);
        ++stats_.failures;
    } catch (...) {
        std::cerr << "[StorageWorker] startup hook failed with unknown error"
                  << std::endl;
        startup_prerequisite_ok = false;
        std::lock_guard<std::mutex> lock(mutex_);
        ++stats_.failures;
    }

    if (bag_requested && startup_prerequisite_ok) {
        try {
            const bool opened = bag_writer_.open(config_.bag_path);
            if (opened) {
                bag_writer_.addConnection("/odometry", "geometry_msgs/PoseStamped",
                    ros_msg_defs::POSE_STAMPED_MD5, ros_msg_defs::POSE_STAMPED);
                bag_writer_.addConnection(kFastLioRegisteredCloudTopic,
                    "sensor_msgs/PointCloud2", ros_msg_defs::POINT_CLOUD2_MD5,
                    ros_msg_defs::POINT_CLOUD2);
                bag_writer_.addConnection(kFastLioMapTopic,
                    "sensor_msgs/PointCloud2", ros_msg_defs::POINT_CLOUD2_MD5,
                    ros_msg_defs::POINT_CLOUD2);
                bag_writer_.addConnection("/path", "geometry_msgs/PoseArray",
                    ros_msg_defs::POSE_ARRAY_MD5, ros_msg_defs::POSE_ARRAY);
                bag_writer_.addConnection("/tf", "tf2_msgs/TFMessage",
                    ros_msg_defs::TF_MESSAGE_MD5, ros_msg_defs::TF_MESSAGE);
                bag_open_ = true;
                bag_available = true;
            } else {
                std::lock_guard<std::mutex> lock(mutex_);
                ++stats_.failures;
                ++stats_.bag_failures;
                stats_.bag_disabled = true;
            }
        } catch (const std::exception& error) {
            std::cerr << "[StorageWorker] bag startup failed: "
                      << error.what() << std::endl;
            bag_open_ = false;
            std::lock_guard<std::mutex> lock(mutex_);
            ++stats_.failures;
            ++stats_.bag_failures;
            stats_.bag_disabled = true;
        } catch (...) {
            std::cerr << "[StorageWorker] bag startup failed with unknown error"
                      << std::endl;
            bag_open_ = false;
            std::lock_guard<std::mutex> lock(mutex_);
            ++stats_.failures;
            ++stats_.bag_failures;
            stats_.bag_disabled = true;
        }
    }

    if (pcd_requested && startup_prerequisite_ok) {
        try {
            std::filesystem::create_directories(
                std::filesystem::path(config_.output_root) / "PCD");
            pcd_enabled_ = true;
            pcd_available = true;
        } catch (const std::exception& error) {
            std::cerr << "[StorageWorker] PCD startup failed: "
                      << error.what() << std::endl;
            std::lock_guard<std::mutex> lock(mutex_);
            ++stats_.failures;
            ++stats_.pcd_failures;
            stats_.pcd_disabled = true;
        } catch (...) {
            std::cerr << "[StorageWorker] PCD startup failed with unknown error"
                      << std::endl;
            std::lock_guard<std::mutex> lock(mutex_);
            ++stats_.failures;
            ++stats_.pcd_failures;
            stats_.pcd_disabled = true;
        }
    }

    // A failed optional sink must not disable a healthy sibling sink.  Start
    // fails only when at least one sink was requested and none is usable.
    const bool any_requested = bag_requested || pcd_requested;
    const bool initialized = startup_prerequisite_ok &&
        (!any_requested || bag_available || pcd_available);
    bool abort_startup = false;
    {
        std::lock_guard<std::mutex> lock(mutex_);
        abort_startup = startup_timed_out_;
        startup_ok_ = initialized && !abort_startup;
        startup_complete_ = true;
        if (!startup_ok_) {
            stats_.failed = true;
        }
    }
    ready_cv_.notify_all();

    if (abort_startup) {
        pcd_enabled_ = false;
        if (bag_open_.exchange(false)) {
            try {
                bag_writer_.close();
            } catch (...) {
                std::lock_guard<std::mutex> lock(mutex_);
                ++stats_.failures;
                ++stats_.bag_failures;
                stats_.bag_disabled = true;
            }
        }
        releasePcdPendingStorage();
        {
            std::lock_guard<std::mutex> lock(mutex_);
            busy_ = false;
            in_io_ = false;
            worker_exited_ = true;
            updatePcdStatsLocked();
            refreshStatsLocked();
        }
        idle_cv_.notify_all();
        exit_cv_.notify_all();
        return;
    }

    bool finalize_pending = false;
    bool reliable_pcd_failure = false;
    while (true) {
        Task task;
        {
            std::unique_lock<std::mutex> lock(mutex_);
            work_cv_.wait(lock, [this] {
                return stopping_ || reliable_admission_failed_ ||
                       fatal_sink_failure_ ||
                       !tasks_.empty() || flush_requested_;
            });
            if (reliable_admission_failed_ || fatal_sink_failure_) {
                finalize_pending = false;
                busy_ = true;
                in_io_ = false;
                break;
            }
            if (stopping_ && (!drain_ || tasks_.empty())) {
                finalize_pending = drain_;
                busy_ = true;
                in_io_ = true;
                break;
            }
            if (tasks_.empty() && flush_requested_) {
                const uint64_t flush_generation =
                    flush_requested_generation_;
                busy_ = true;
                in_io_ = true;
                lock.unlock();
                const auto flush_begin = std::chrono::steady_clock::now();
                bool flush_failed = false;
                try {
                    if (pcd_enabled_.load() && pendingPcdPointCount() > 0) {
                        const size_t point_count = pendingPcdPointCount();
                        writePendingPcdChunk(true);
                        commitPcdPoints(point_count);
                    }
                } catch (const std::exception& error) {
                    flush_failed = true;
                    std::cerr << "[StorageWorker] flush failed: "
                              << error.what() << std::endl;
                    lock.lock();
                    ++stats_.failures;
                    lock.unlock();
                    fusePcdOutputAfterWriteFailure();
                    reliable_pcd_failure = config_.mode == Mode::Reliable;
                } catch (...) {
                    flush_failed = true;
                    std::cerr << "[StorageWorker] flush failed with unknown error"
                              << std::endl;
                    lock.lock();
                    ++stats_.failures;
                    lock.unlock();
                    fusePcdOutputAfterWriteFailure();
                    reliable_pcd_failure = config_.mode == Mode::Reliable;
                }
                lock.lock();
                flush_completed_generation_ = std::max(
                    flush_completed_generation_, flush_generation);
                if (flush_failed && first_failed_flush_generation_ == 0) {
                    first_failed_flush_generation_ = flush_generation;
                }
                flush_requested_ =
                    flush_requested_generation_ > flush_generation;
                busy_ = false;
                in_io_ = false;
                updatePcdStatsLocked();
                stats_.pcd_chunks_written = pcd_chunks_written_;
                stats_.last_flush_ms = std::chrono::duration<double, std::milli>(
                    std::chrono::steady_clock::now() - flush_begin).count();
                flush_timing_.record(stats_.last_flush_ms);
                lock.unlock();
                idle_cv_.notify_all();
                if (reliable_pcd_failure) {
                    finalize_pending = false;
                    break;
                }
                continue;
            }
            queued_bytes_ -= tasks_.front().bytes;
            accountQueuedTaskRemovedLocked(tasks_.front());
            setInflightTaskLocked(tasks_.front());
            task = std::move(tasks_.front());
            tasks_.pop_front();
            busy_ = true;
            in_io_ = true;
            refreshStatsLocked();
        }
        const auto begin = std::chrono::steady_clock::now();
        bool bag_write_failed = false;
        bool pcd_write_failed = false;
        try {
            const uint64_t committed_tasks = process(task);
            std::lock_guard<std::mutex> lock(mutex_);
            stats_.tasks_written += committed_tasks;
        } catch (const std::exception& error) {
            std::cerr << "[StorageWorker] write failed: " << error.what() << std::endl;
            std::lock_guard<std::mutex> lock(mutex_);
            ++stats_.failures;
            bag_write_failed = isBagTask(task.type);
            pcd_write_failed = task.type == TaskType::Pcd;
        } catch (...) {
            std::cerr << "[StorageWorker] write failed with unknown error" << std::endl;
            std::lock_guard<std::mutex> lock(mutex_);
            ++stats_.failures;
            bag_write_failed = isBagTask(task.type);
            pcd_write_failed = task.type == TaskType::Pcd;
        }
        {
            const double write_ms = std::chrono::duration<double, std::milli>(
                std::chrono::steady_clock::now() - begin).count();
            std::lock_guard<std::mutex> lock(mutex_);
            stats_.last_write_ms = write_ms;
            write_timing_.record(write_ms);
            if (isBagTask(task.type)) {
                bag_write_timing_.record(write_ms);
            } else {
                pcd_write_timing_.record(write_ms);
            }
        }
        if (bag_write_failed) {
            // The failed Bag task has already left the queue and was not
            // written; account it separately from queued Bag tasks cancelled
            // by the component fuse.
            {
                std::lock_guard<std::mutex> lock(mutex_);
                accountDroppedTaskLocked(task);
            }
            fuseBagOutputAfterWriteFailure();
        }
        if (pcd_write_failed) {
            // The current uncommitted PCD contribution is included in the
            // pending-task ledger consumed by the PCD fuse.
            fusePcdOutputAfterWriteFailure();
        }
        {
            std::lock_guard<std::mutex> lock(mutex_);
            busy_ = false;
            in_io_ = false;
            clearInflightTaskLocked();
            updatePcdStatsLocked();
            stats_.pcd_chunks_written = pcd_chunks_written_;
            refreshStatsLocked();
        }
        idle_cv_.notify_all();
        if (pcd_write_failed && config_.mode == Mode::Reliable) {
            reliable_pcd_failure = true;
            finalize_pending = false;
            break;
        }
    }

    const auto flush_begin = std::chrono::steady_clock::now();
    bool final_pcd_failed = false;
    if (finalize_pending && pcd_enabled_.load() &&
        pendingPcdPointCount() > 0) {
        try {
            const size_t point_count = pendingPcdPointCount();
            writePendingPcdChunk(true);
            commitPcdPoints(point_count);
        } catch (const std::exception& error) {
            std::cerr << "[StorageWorker] final PCD write failed: "
                      << error.what() << std::endl;
            final_pcd_failed = true;
        } catch (...) {
            std::cerr << "[StorageWorker] final PCD write failed with unknown error"
                      << std::endl;
            final_pcd_failed = true;
        }
    }
    if (final_pcd_failed) {
        {
            std::lock_guard<std::mutex> lock(mutex_);
            ++stats_.failures;
        }
        fusePcdOutputAfterWriteFailure();
        {
            std::lock_guard<std::mutex> lock(mutex_);
            stats_.failed = true;
        }
        reliable_pcd_failure = config_.mode == Mode::Reliable;
    }
    if (bag_open_.exchange(false)) {
        try {
            bag_writer_.close();
        } catch (const std::exception& error) {
            std::cerr << "[StorageWorker] bag close failed: "
                      << error.what() << std::endl;
            std::lock_guard<std::mutex> lock(mutex_);
            ++stats_.failures;
            ++stats_.bag_failures;
            stats_.bag_disabled = true;
        } catch (...) {
            std::cerr << "[StorageWorker] bag close failed with unknown error"
                      << std::endl;
            std::lock_guard<std::mutex> lock(mutex_);
            ++stats_.failures;
            ++stats_.bag_failures;
            stats_.bag_disabled = true;
        }
    }
    pcd_enabled_ = false;
    {
        std::lock_guard<std::mutex> lock(mutex_);
        const bool preserve_pending = reliable_pcd_failure ||
                                      reliable_admission_failed_;
        if (!finalize_pending && !preserve_pending) {
            cancelPendingPcdLocked();
        }
        busy_ = false;
        in_io_ = false;
        worker_exited_ = true;
        clearInflightTaskLocked();
        updatePcdStatsLocked();
        stats_.pcd_chunks_written = pcd_chunks_written_;
        stats_.last_flush_ms = std::chrono::duration<double, std::milli>(
            std::chrono::steady_clock::now() - flush_begin).count();
        flush_timing_.record(stats_.last_flush_ms);
    }
    idle_cv_.notify_all();
    exit_cv_.notify_all();
}

uint64_t StorageWorker::process(Task& task)
{
    switch (task.type) {
    case TaskType::OdomTf:
        if (bag_open_) {
            bag_writer_.writeOdometry(task.time_ns, task.position, task.orientation,
                                      kFastLioMapFrame);
            bag_writer_.writeTF(task.time_ns, task.position, task.orientation,
                                kFastLioMapFrame, kFastLioBodyFrame);
        }
        return 1;
    case TaskType::Cloud:
        if (bag_open_) {
            bag_writer_.writePointCloud(task.time_ns, task.cloud,
                                        task.frame_id, task.topic);
        }
        return 1;
    case TaskType::Path:
        if (bag_open_) bag_writer_.writePath(task.time_ns, task.path, task.frame_id);
        return 1;
    case TaskType::Pcd: {
        const size_t appended_points = task.cloud.size();
        compactPcdPendingStorage();
        if (pendingPcdPointCount() == 0) {
            pcd_pending_ = std::move(task.cloud);
            pcd_pending_head_ = 0;
        } else {
            pcd_pending_.points.reserve(
                saturatedAdd(pcd_pending_.size(), task.cloud.size()));
            pcd_pending_.points.insert(
                pcd_pending_.points.end(),
                std::make_move_iterator(task.cloud.points.begin()),
                std::make_move_iterator(task.cloud.points.end()));
        }
        pcd_pending_task_points_.push_back(appended_points);
        try {
            while (pendingPcdPointCount() >= config_.pcd_chunk_points)
            {
                // The logical head advances only after a successful write.
                // No prefix erase occurs per chunk, so large tasks are not
                // repeatedly shifted through memory.
                PointCloudXYZI chunk;
                const auto begin = pcd_pending_.points.begin() +
                    static_cast<std::ptrdiff_t>(pcd_pending_head_);
                chunk.points.assign(
                    begin, begin + static_cast<std::ptrdiff_t>(
                        config_.pcd_chunk_points));
                setPcdScratchBytes(saturatedMultiply(
                    chunk.points.capacity(), sizeof(PointType)));
                try {
                    writePcdChunk(chunk, false);
                } catch (...) {
                    setPcdScratchBytes(0);
                    throw;
                }
                setPcdScratchBytes(0);
                commitPcdPoints(config_.pcd_chunk_points);
            }
            if (config_.pcd_chunk_frames > 0 &&
                pendingPcdPointCount() > 0 &&
                pcd_pending_task_points_.size() >=
                    config_.pcd_chunk_frames) {
                const size_t point_count = pendingPcdPointCount();
                writePendingPcdChunk(false);
                commitPcdPoints(point_count);
            }
        } catch (...) {
            compactPcdPendingStorage();
            throw;
        }
        compactPcdPendingStorage();
        return 0;
    }
    }
    return 0;
}

void StorageWorker::writePcdChunk(
    PointCloudXYZI& cloud, bool final_chunk)
{
    if (cloud.empty()) return;
    cloud.width = static_cast<uint32_t>(cloud.size());
    cloud.height = 1;
    cloud.is_dense = true;
    std::ostringstream name;
    name << "scans_";
    if (final_chunk) name << "final_";
    name << pcd_chunk_sequence_ << ".pcd";
    const std::string path = (std::filesystem::path(config_.output_root) /
                              "PCD" / name.str()).string();
    if (config_.pcd_io_hook) config_.pcd_io_hook("write");
    int result = 0;
    if (config_.pcd_format == PcdFormat::BinaryCompressed) {
        result = pcl::io::savePCDFileBinaryCompressed(path, cloud);
    } else {
        result = pcl::io::savePCDFileBinary(path, cloud);
    }
    if (result != 0) throw std::runtime_error("failed to write PCD chunk: " + path);
    ++pcd_chunk_sequence_;
    ++pcd_chunks_written_;
}

void StorageWorker::commitPcdPoints(size_t point_count)
{
    if (point_count > pendingPcdPointCount()) {
        throw std::logic_error("PCD commit exceeds pending point count");
    }
    pcd_pending_head_ += point_count;

    size_t remaining = point_count;
    uint64_t committed_tasks = 0;
    while (remaining > 0 && !pcd_pending_task_points_.empty()) {
        size_t& contribution = pcd_pending_task_points_.front();
        if (remaining < contribution) {
            contribution -= remaining;
            remaining = 0;
        } else {
            remaining -= contribution;
            pcd_pending_task_points_.pop_front();
            ++committed_tasks;
        }
    }
    if (pendingPcdPointCount() == 0) releasePcdPendingStorage();
    if (committed_tasks > 0) {
        std::lock_guard<std::mutex> lock(mutex_);
        stats_.tasks_written += committed_tasks;
        updatePcdStatsLocked();
    }
}

void StorageWorker::setPcdScratchBytes(size_t bytes)
{
    std::lock_guard<std::mutex> lock(mutex_);
    pcd_scratch_bytes_ = bytes;
    stats_.pcd_scratch_bytes = bytes;
    stats_.max_pcd_scratch_bytes =
        std::max(stats_.max_pcd_scratch_bytes, bytes);
    updatePcdStatsLocked();
}

void StorageWorker::releasePcdPendingStorage() noexcept
{
    pcd_pending_.points.clear();
    try {
        decltype(pcd_pending_.points) released_points;
        pcd_pending_.points.swap(released_points);
    } catch (...) {
    }
    pcd_pending_.width = 0;
    pcd_pending_.height = 0;
    pcd_pending_.is_dense = true;
    pcd_pending_head_ = 0;
    pcd_pending_task_points_.clear();
    try {
        std::deque<size_t>().swap(pcd_pending_task_points_);
    } catch (...) {
    }
}

void StorageWorker::compactPcdPendingStorage() noexcept
{
    if (pcd_pending_head_ == 0) return;
    try {
        if (pendingPcdPointCount() == 0) {
            releasePcdPendingStorage();
            return;
        }
        PointCloudXYZI remainder;
        remainder.points.assign(
            pcd_pending_.points.begin() +
                static_cast<std::ptrdiff_t>(pcd_pending_head_),
            pcd_pending_.points.end());
        remainder.width = static_cast<uint32_t>(remainder.size());
        remainder.height = 1;
        remainder.is_dense = true;
        pcd_pending_.points.swap(remainder.points);
        pcd_pending_.width = static_cast<uint32_t>(pcd_pending_.size());
        pcd_pending_.height = 1;
        pcd_pending_.is_dense = true;
        pcd_pending_head_ = 0;
    } catch (...) {
        // The logical head remains authoritative. A later process/flush can
        // retry compaction without losing or duplicating persisted points.
    }
}

size_t StorageWorker::pendingPcdPointCount() const
{
    return pcd_pending_head_ <= pcd_pending_.size()
        ? pcd_pending_.size() - pcd_pending_head_ : 0;
}

void StorageWorker::writePendingPcdChunk(bool final_chunk)
{
    const size_t point_count = pendingPcdPointCount();
    if (point_count == 0) return;
    if (pcd_pending_head_ == 0) {
        writePcdChunk(pcd_pending_, final_chunk);
        return;
    }

    PointCloudXYZI chunk;
    chunk.points.assign(
        pcd_pending_.points.begin() +
            static_cast<std::ptrdiff_t>(pcd_pending_head_),
        pcd_pending_.points.end());
    setPcdScratchBytes(saturatedMultiply(
        chunk.points.capacity(), sizeof(PointType)));
    try {
        writePcdChunk(chunk, final_chunk);
    } catch (...) {
        setPcdScratchBytes(0);
        throw;
    }
    setPcdScratchBytes(0);
}

void StorageWorker::cancelPendingPcdLocked()
{
    const uint64_t tasks =
        static_cast<uint64_t>(pcd_pending_task_points_.size());
    const uint64_t points = static_cast<uint64_t>(pendingPcdPointCount());
    stats_.tasks_cancelled += tasks;
    stats_.pcd_tasks_cancelled += tasks;
    stats_.pcd_points_cancelled += points;
    stats_.pcd_bytes_cancelled +=
        points * static_cast<uint64_t>(sizeof(PointType));
    releasePcdPendingStorage();
    updatePcdStatsLocked();
}

void StorageWorker::cancelQueuedTasksLocked()
{
    for (const auto& task : tasks_) {
        accountQueuedTaskRemovedLocked(task);
        accountCancelledTaskLocked(task);
    }
    std::deque<Task>().swap(tasks_);
    queued_bytes_ = 0;
    refreshStatsLocked();
}

void StorageWorker::accountDroppedTaskLocked(const Task& task)
{
    ++stats_.tasks_dropped;
    if (task.type != TaskType::Pcd) return;
    ++stats_.pcd_tasks_dropped;
    stats_.pcd_points_dropped += static_cast<uint64_t>(task.points);
    stats_.pcd_bytes_dropped +=
        static_cast<uint64_t>(task.points) * sizeof(PointType);
}

void StorageWorker::accountCancelledTaskLocked(const Task& task)
{
    ++stats_.tasks_cancelled;
    if (isBagTask(task.type)) {
        ++stats_.bag_tasks_cancelled;
        return;
    }
    if (task.type != TaskType::Pcd) return;
    ++stats_.pcd_tasks_cancelled;
    stats_.pcd_points_cancelled += static_cast<uint64_t>(task.points);
    stats_.pcd_bytes_cancelled +=
        static_cast<uint64_t>(task.points) * sizeof(PointType);
}

void StorageWorker::setInflightTaskLocked(const Task& task)
{
    inflight_tasks_ = 1;
    inflight_points_ = task.points;
    inflight_bytes_ = task.bytes;
    refreshStatsLocked();
}

void StorageWorker::clearInflightTaskLocked()
{
    inflight_tasks_ = 0;
    inflight_points_ = 0;
    inflight_bytes_ = 0;
    refreshStatsLocked();
}

void StorageWorker::updatePcdStatsLocked()
{
    stats_.pcd_pending_points = pendingPcdPointCount();
    stats_.pcd_pending_capacity_points = pcd_pending_.points.capacity();
    stats_.pcd_pending_bytes = saturatedMultiply(
        pcd_pending_.points.capacity(), sizeof(PointType));
    stats_.max_pcd_pending_bytes = std::max(
        stats_.max_pcd_pending_bytes, stats_.pcd_pending_bytes);
    stats_.pcd_pending_tasks = pcd_pending_task_points_.size();
    stats_.pcd_scratch_bytes = pcd_scratch_bytes_;
}

void StorageWorker::refreshStatsLocked()
{
    stats_.queue_tasks = tasks_.size();
    stats_.queue_bytes = queued_bytes_;
    stats_.max_queue_bytes = std::max(stats_.max_queue_bytes, queued_bytes_);
    stats_.inflight_tasks = inflight_tasks_;
    stats_.inflight_points = inflight_points_;
    stats_.inflight_bytes = inflight_bytes_;
    stats_.hard_usage_bytes = saturatedAdd(queued_bytes_, inflight_bytes_);
    stats_.max_hard_usage_bytes = std::max(
        stats_.max_hard_usage_bytes, stats_.hard_usage_bytes);
    stats_.pcd_queue_points = queued_pcd_points_;
    stats_.pcd_queue_bytes = queued_pcd_bytes_;
    stats_.pcd_queue_tasks = queued_pcd_tasks_;
}

void StorageWorker::accountQueuedTaskAddedLocked(const Task& task)
{
    if (task.type != TaskType::Pcd) return;
    ++queued_pcd_tasks_;
    queued_pcd_points_ = saturatedAdd(queued_pcd_points_, task.points);
    queued_pcd_bytes_ = saturatedAdd(
        queued_pcd_bytes_, saturatedMultiply(
            task.cloud.points.capacity(), sizeof(PointType)));
}

void StorageWorker::accountQueuedTaskRemovedLocked(const Task& task)
{
    if (task.type != TaskType::Pcd) return;
    if (queued_pcd_tasks_ > 0) --queued_pcd_tasks_;
    queued_pcd_points_ -= std::min(queued_pcd_points_, task.points);
    const size_t bytes = saturatedMultiply(
        task.cloud.points.capacity(), sizeof(PointType));
    queued_pcd_bytes_ -= std::min(queued_pcd_bytes_, bytes);
}

size_t StorageWorker::saturatedAdd(size_t left, size_t right)
{
    const size_t maximum = std::numeric_limits<size_t>::max();
    return right > maximum - left ? maximum : left + right;
}

size_t StorageWorker::saturatedMultiply(size_t left, size_t right)
{
    if (left == 0 || right == 0) return 0;
    const size_t maximum = std::numeric_limits<size_t>::max();
    return left > maximum / right ? maximum : left * right;
}

size_t StorageWorker::taskResidentBytes(const Task& task)
{
    size_t bytes = sizeof(Task);
    bytes = saturatedAdd(bytes, saturatedMultiply(
        task.cloud.points.capacity(), sizeof(PointType)));
    bytes = saturatedAdd(bytes, saturatedMultiply(
        task.path.capacity(), sizeof(V3D)));
    bytes = saturatedAdd(bytes, task.frame_id.capacity());
    bytes = saturatedAdd(bytes, task.topic.capacity());
    return bytes;
}

StorageWorker::Mode StorageWorker::parseMode(const std::string& value)
{
    if (value == "reliable") return Mode::Reliable;
    if (value == "realtime") return Mode::Realtime;
    throw std::invalid_argument("unknown storage mode: " + value);
}

StorageWorker::PcdFormat StorageWorker::parsePcdFormat(const std::string& value)
{
    if (value == "binary") return PcdFormat::Binary;
    if (value == "binary_compressed") return PcdFormat::BinaryCompressed;
    throw std::invalid_argument("unknown PCD format: " + value);
}
