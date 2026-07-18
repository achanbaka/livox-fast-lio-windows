#include "map_build_worker.h"

#include <algorithm>
#include <cmath>
#include <exception>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <utility>

namespace {

size_t saturatedAddSize(size_t left, size_t right) noexcept
{
    return right > std::numeric_limits<size_t>::max() - left
        ? std::numeric_limits<size_t>::max()
        : left + right;
}

size_t saturatedMultiplySize(size_t left, size_t right) noexcept
{
    if (left != 0 && right > std::numeric_limits<size_t>::max() / left) {
        return std::numeric_limits<size_t>::max();
    }
    return left * right;
}

uint64_t saturatedAddU64(uint64_t left, size_t right) noexcept
{
    const uint64_t converted = right >
        static_cast<size_t>(std::numeric_limits<uint64_t>::max())
        ? std::numeric_limits<uint64_t>::max()
        : static_cast<uint64_t>(right);
    if (converted > std::numeric_limits<uint64_t>::max() - left) {
        return std::numeric_limits<uint64_t>::max();
    }
    return left + converted;
}

bool exceedsBound(size_t current, size_t incoming, size_t limit) noexcept
{
    return limit > 0 &&
        (incoming > limit || current > limit - incoming);
}

bool tryTileKeyForPoint(const PointType& point, double tile_size,
                        TileKey& key) noexcept
{
    const double x = std::floor(static_cast<double>(point.x) / tile_size);
    const double y = std::floor(static_cast<double>(point.y) / tile_size);
    const double z = std::floor(static_cast<double>(point.z) / tile_size);
    const double minimum =
        static_cast<double>(std::numeric_limits<int32_t>::min());
    const double maximum =
        static_cast<double>(std::numeric_limits<int32_t>::max());
    if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z) ||
        x < minimum || x > maximum || y < minimum || y > maximum ||
        z < minimum || z > maximum)
    {
        return false;
    }
    key = TileKey{static_cast<int32_t>(x), static_cast<int32_t>(y),
                  static_cast<int32_t>(z)};
    return true;
}

MapBuildWorker::Config normalizeMapBuildConfig(
    MapBuildWorker::Config config, bool tile_output_enabled)
{
    // A zero task limit used to mean an unbounded frame queue. The Tile input
    // path always keeps a hard task ceiling; retain the documented default
    // when legacy callers pass zero.
    if (config.input_queue_capacity == 0) {
        config.input_queue_capacity = 64;
    }
    if (!tile_output_enabled) {
        // Full-only mode has no per-Tile wire limit and must not silently
        // truncate a dense Tile merely because incremental defaults exist.
        config.store.max_voxels_per_tile = 0;
        return config;
    }

    auto intersectBound = [](size_t& batch_limit, size_t output_limit) {
        if (output_limit > 0 &&
            (batch_limit == 0 || batch_limit > output_limit)) {
            batch_limit = output_limit;
        }
    };
    // enqueueTiles is all-or-nothing. A batch larger than the empty output
    // queue can never be admitted and would otherwise be restored forever.
    intersectBound(config.max_tiles_per_update,
                   config.output.max_pending_tiles);
    intersectBound(config.max_points_per_update,
                   config.output.max_pending_points);
    intersectBound(config.max_bytes_per_update,
                   config.output.max_pending_bytes);

    size_t limit = std::numeric_limits<size_t>::max();
    bool bounded = false;
    if (config.max_points_per_update > 0) {
        limit = std::min(limit, config.max_points_per_update);
        bounded = true;
    }
    if (config.max_bytes_per_update > 0) {
        limit = std::min(
            limit,
            TileUpdate::maxPointCountForBytes(config.max_bytes_per_update));
        bounded = true;
    }
    if (config.output.max_pending_points > 0) {
        limit = std::min(limit, config.output.max_pending_points);
        bounded = true;
    }
    if (config.output.max_pending_bytes > 0) {
        limit = std::min(
            limit,
            TileUpdate::maxPointCountForBytes(
                config.output.max_pending_bytes));
        bounded = true;
    }
    if (bounded && limit == 0) {
        throw std::invalid_argument(
            "tile update byte limit cannot hold even one voxel");
    }
    if (bounded &&
        (config.store.max_voxels_per_tile == 0 ||
         config.store.max_voxels_per_tile > limit))
    {
        config.store.max_voxels_per_tile = limit;
    }
    return config;
}

}  // namespace

size_t MapBuildWorker::PendingTileKeyHash::operator()(
    const PendingTileKey& key) const noexcept
{
    size_t hash = TileKeyHash{}(key.tile);
    const size_t epoch = static_cast<size_t>(
        key.epoch ^ (key.epoch >> (sizeof(size_t) * 4)));
    hash ^= epoch + static_cast<size_t>(0x9e3779b9U) +
        (hash << 6) + (hash >> 2);
    return hash;
}

MapBuildWorker::MapBuildWorker(
    Config config, PublishCallback full_callback, TilePublishCallback tile_callback)
    : config_(normalizeMapBuildConfig(
          std::move(config), static_cast<bool>(tile_callback))),
      store_(config_.store),
      tile_output_enabled_(static_cast<bool>(tile_callback)),
      output_(
          std::move(full_callback), std::move(tile_callback), config_.output,
          [this](const TileUpdate&, double timestamp) {
              std::lock_guard<std::mutex> lock(mutex_);
              if (!running_ || stopping_) return;
              if (!tile_resync_pending_) {
                  tile_resync_sequence_ = next_sequence_;
                  tile_resync_pending_ = true;
              } else {
                  tile_resync_sequence_ = std::max(
                      tile_resync_sequence_, next_sequence_);
              }
              ++input_epoch_;
              dirty_timestamp_ = timestamp;
              ++stats_.tile_resync_requested;
              ++stats_.tile_resync_on_failure;
              work_cv_.notify_one();
          })
{}

MapBuildWorker::~MapBuildWorker()
{
    stop(false);
}

void MapBuildWorker::start()
{
    std::lock_guard<std::mutex> lock(mutex_);
    if (running_) return;
    tile_tasks_.clear();
    tile_task_index_.clear();
    frame_progress_.clear();
    pending_input_points_ = 0;
    pending_input_bytes_ = 0;
    store_.clear();
    stats_ = Stats{};
    map_build_timing_ = BoundedTimingWindow<>{};
    tile_extract_timing_ = BoundedTimingWindow<>{};
    full_snapshot_timing_ = BoundedTimingWindow<>{};
    stopping_ = false;
    drain_ = true;
    busy_ = false;
    worker_exited_ = false;
    full_pending_ = false;
    tile_resync_pending_ = false;
    tile_resync_in_progress_ = false;
    dirty_drain_pending_ = false;
    tile_resync_sequence_ = 0;
    next_sequence_ = 0;
    built_sequence_ = 0;
    input_epoch_ = 0;
    output_.start();
    running_ = true;
    try {
        worker_ = std::thread(&MapBuildWorker::run, this);
    } catch (...) {
        running_ = false;
        worker_exited_ = true;
        output_.stop(false);
        throw;
    }
}

bool MapBuildWorker::enqueueFrame(
    PointCloudXYZI frame, double timestamp, const PointType* current_position)
{
    if (frame.empty()) return true;
    const size_t source_point_count = frame.size();
    uint64_t admitted_sequence = 0;
    bool frame_counted = false;
    try {
    // Partition before taking the worker mutex. This keeps the queue mutation
    // short and, unlike the old whole-frame queue, bounds the staging set by
    // the same Tile-task and point limits used by the retained queue.
    TileTaskList prepared;
    TileTaskIndex prepared_index;
    const size_t point_limit = inputPointLimit();
    const size_t task_limit = config_.input_queue_capacity > 0
        ? config_.input_queue_capacity : 64;
    size_t prepared_points = 0;
    size_t prepared_dropped_points = 0;
    uint64_t prepared_changes_dropped = 0;

    auto preparedBytes = [&]() noexcept {
        size_t bytes = prepared_index.empty() ? 0 : saturatedMultiplySize(
            prepared_index.bucket_count(), 2 * sizeof(void*));
        for (const auto& task : prepared) {
            bytes = saturatedAddSize(
                bytes, sizeof(TileTask) + 2 * sizeof(void*));
            bytes = saturatedAddSize(
                bytes, sizeof(PendingTileKey) +
                    sizeof(TileTaskList::iterator) + 2 * sizeof(void*));
            for (const auto& contribution : task.contributions) {
                bytes = saturatedAddSize(
                    bytes, sizeof(TileContribution) + 2 * sizeof(void*));
                bytes = saturatedAddSize(
                    bytes, saturatedMultiplySize(
                        contribution.cloud.points.capacity(),
                        sizeof(PointType)));
            }
        }
        return bytes;
    };
    auto erasePrepared = [&](TileTaskList::iterator task) {
        prepared_dropped_points = saturatedAddSize(
            prepared_dropped_points, task->point_count);
        prepared_points -= std::min(prepared_points, task->point_count);
        prepared_changes_dropped = saturatedAddU64(
            prepared_changes_dropped, task->contributions.size());
        prepared_index.erase(task->key);
        prepared.erase(task);
    };

    for (const PointType& point : frame.points) {
        TileKey tile;
        if (!tryTileKeyForPoint(point, config_.store.tile_size_m, tile)) {
            prepared_dropped_points = saturatedAddSize(
                prepared_dropped_points, 1);
            continue;
        }

        const PendingTileKey prepared_key{tile, 0};
        auto found = prepared_index.find(prepared_key);
        if (found == prepared_index.end()) {
            while (prepared.size() >= task_limit && !prepared.empty()) {
                erasePrepared(prepared.begin());
            }
            TileTask task;
            task.key = prepared_key;
            task.contributions.emplace_back();
            prepared.push_back(std::move(task));
            auto inserted = std::prev(prepared.end());
            prepared_index.emplace(prepared_key, inserted);
            found = prepared_index.find(prepared_key);
        }
        auto task = found->second;
        PointCloudXYZI& cloud = task->contributions.front().cloud;

        // At point pressure, retain the newest observation for this Tile
        // instead of discarding the whole spatial region.
        if (point_limit != std::numeric_limits<size_t>::max() &&
            prepared_points >= point_limit)
        {
            if (!cloud.empty()) {
                cloud.points.erase(cloud.points.begin());
                --task->point_count;
                --prepared_points;
                prepared_dropped_points = saturatedAddSize(
                    prepared_dropped_points, 1);
            } else if (!prepared.empty()) {
                auto victim = prepared.begin();
                if (victim == task && std::next(victim) != prepared.end()) {
                    ++victim;
                }
                if (victim != task) erasePrepared(victim);
            }
        }
        if (point_limit == 0 ||
            (point_limit != std::numeric_limits<size_t>::max() &&
             prepared_points >= point_limit))
        {
            prepared_dropped_points = saturatedAddSize(
                prepared_dropped_points, 1);
            continue;
        }

        if (cloud.size() == cloud.points.capacity()) {
            const size_t current_capacity = cloud.points.capacity();
            size_t desired_capacity = current_capacity == 0
                ? 1 : saturatedAddSize(current_capacity,
                                       std::max<size_t>(1, current_capacity / 2));
            if (point_limit != std::numeric_limits<size_t>::max()) {
                desired_capacity = std::min(
                    desired_capacity,
                    saturatedAddSize(current_capacity,
                                     point_limit - prepared_points));
            }
            const size_t growth_bytes = saturatedMultiplySize(
                desired_capacity - current_capacity, sizeof(PointType));
            const size_t staged_bytes = preparedBytes();
            if (config_.input_queue_max_bytes > 0 &&
                exceedsBound(staged_bytes, growth_bytes,
                             config_.input_queue_max_bytes))
            {
                if (!cloud.empty()) {
                    // Capacity is already charged. Reuse it as a bounded
                    // latest-value ring rather than allocating beyond the
                    // byte ceiling.
                    cloud.points.erase(cloud.points.begin());
                    cloud.push_back(point);
                    prepared_dropped_points = saturatedAddSize(
                        prepared_dropped_points, 1);
                    continue;
                }
                prepared_dropped_points = saturatedAddSize(
                    prepared_dropped_points, 1);
                continue;
            }
            cloud.points.reserve(desired_capacity);
            if (config_.input_queue_max_bytes > 0 &&
                preparedBytes() > config_.input_queue_max_bytes)
            {
                erasePrepared(task);
                prepared_dropped_points = saturatedAddSize(
                    prepared_dropped_points, 1);
                continue;
            }
        }
        cloud.push_back(point);
        ++task->point_count;
        ++prepared_points;
        prepared.splice(prepared.end(), prepared, task);
    }

    std::lock_guard<std::mutex> lock(mutex_);
    if (!running_ || stopping_) return false;

    const uint64_t sequence = next_sequence_ + 1;
    auto progress_result = frame_progress_.emplace(sequence, FrameProgress{});
    next_sequence_ = sequence;
    admitted_sequence = sequence;
    frame_counted = true;
    ++stats_.frames_enqueued;
    FrameProgress& progress = progress_result.first->second;
    bool input_loss = prepared_dropped_points > 0;
    if (prepared_dropped_points > 0) {
        recordInputLossLocked(sequence, prepared_dropped_points);
        stats_.input_tile_changes_dropped = saturatedAddU64(
            stats_.input_tile_changes_dropped,
            static_cast<size_t>(prepared_changes_dropped));
    }

    for (auto prepared_task = prepared.begin();
         prepared_task != prepared.end(); ++prepared_task)
    {
        TileContribution contribution =
            std::move(prepared_task->contributions.front());
        contribution.timestamp = timestamp;
        contribution.sequence = sequence;
        if (current_position) {
            contribution.current_position = *current_position;
            contribution.has_current_position = true;
        }
        const size_t contribution_points = contribution.cloud.size();
        if (contribution_points == 0) continue;

        PendingTileKey key{prepared_task->key.tile, input_epoch_};
        auto target_found = tile_task_index_.find(key);
        const bool merge = target_found != tile_task_index_.end();
        while (!merge && config_.input_queue_capacity > 0 &&
               tile_tasks_.size() >= config_.input_queue_capacity &&
               !tile_tasks_.empty())
        {
            input_loss = true;
            evictTileTaskLocked(tile_tasks_.begin());
        }

        target_found = tile_task_index_.find(key);
        TileTaskList::iterator target;
        if (target_found == tile_task_index_.end()) {
            TileTask task;
            task.key = key;
            tile_tasks_.push_back(std::move(task));
            target = std::prev(tile_tasks_.end());
            try {
                tile_task_index_.emplace(key, target);
            } catch (...) {
                tile_tasks_.erase(target);
                throw;
            }
        } else {
            target = target_found->second;
        }

        const size_t hard_point_limit = inputPointLimit();
        while (hard_point_limit != std::numeric_limits<size_t>::max() &&
               exceedsBound(pending_input_points_, contribution_points,
                            hard_point_limit))
        {
            input_loss = true;
            if (!target->contributions.empty()) {
                discardOldestPointsLocked(
                    target, target->contributions.front().cloud.size());
            } else {
                auto victim = tile_tasks_.begin();
                if (victim == target && std::next(victim) != tile_tasks_.end()) {
                    ++victim;
                }
                if (victim == target) break;
                evictTileTaskLocked(victim);
            }
        }

        refreshInputStatsLocked();
        const size_t contribution_bytes = saturatedAddSize(
            sizeof(TileContribution) + 2 * sizeof(void*),
            saturatedMultiplySize(contribution.cloud.points.capacity(),
                                  sizeof(PointType)));
        // The empty target task/index node and frame-progress node are already
        // present in pending_input_bytes_ after refreshInputStatsLocked().
        const size_t new_task_bytes = 0;
        while (config_.input_queue_max_bytes > 0 &&
               exceedsBound(pending_input_bytes_,
                            saturatedAddSize(contribution_bytes,
                                             new_task_bytes),
                            config_.input_queue_max_bytes))
        {
            input_loss = true;
            if (!target->contributions.empty()) {
                discardOldestPointsLocked(
                    target, target->contributions.front().cloud.size());
            } else {
                auto victim = tile_tasks_.begin();
                if (victim == target && std::next(victim) != tile_tasks_.end()) {
                    ++victim;
                }
                if (victim == target) break;
                evictTileTaskLocked(victim);
            }
            refreshInputStatsLocked();
        }

        refreshInputStatsLocked();
        if ((hard_point_limit != std::numeric_limits<size_t>::max() &&
             exceedsBound(pending_input_points_, contribution_points,
                          hard_point_limit)) ||
            (config_.input_queue_max_bytes > 0 &&
             exceedsBound(pending_input_bytes_,
                          saturatedAddSize(contribution_bytes,
                                           new_task_bytes),
                          config_.input_queue_max_bytes)))
        {
            input_loss = true;
            recordInputLossLocked(sequence, contribution_points);
            ++stats_.input_tile_changes_dropped;
            eraseEmptyTileTaskLocked(target);
            continue;
        }

        target->contributions.push_back(std::move(contribution));
        target->point_count = saturatedAddSize(
            target->point_count, contribution_points);
        pending_input_points_ = saturatedAddSize(
            pending_input_points_, contribution_points);
        ++progress.pending_contributions;
        ++stats_.input_tile_changes_enqueued;
        if (merge) ++stats_.input_tile_changes_merged;
        tile_tasks_.splice(tile_tasks_.end(), tile_tasks_, target);
        refreshInputStatsLocked();
    }

    const bool retained = progress.pending_contributions > 0;
    progress.sealed = true;
    advanceBuiltSequenceLocked();
    if (input_loss) {
        scheduleInputResyncLocked(timestamp, sequence);
        ++input_epoch_;
    }
    refreshInputStatsLocked();
    work_cv_.notify_one();
    return retained;
    } catch (const std::exception& error) {
        std::cerr << "[MapBuildWorker] input admission failed: "
                  << error.what() << std::endl;
    } catch (...) {
        std::cerr << "[MapBuildWorker] input admission failed with unknown error"
                  << std::endl;
    }

    // Admission is an exception boundary for the SLAM thread. If any Tile
    // contributions were committed before an allocation failed, seal their
    // source frame so a full/resync sequence barrier can still advance.
    std::lock_guard<std::mutex> lock(mutex_);
    if (!running_ || stopping_) return false;
    ++stats_.failures;
    if (admitted_sequence != 0) {
        const auto progress = frame_progress_.find(admitted_sequence);
        if (progress != frame_progress_.end()) {
            recordInputLossLocked(admitted_sequence, 0);
            progress->second.sealed = true;
            advanceBuiltSequenceLocked();
            scheduleInputResyncLocked(timestamp, admitted_sequence);
            ++input_epoch_;
        }
    } else {
        if (!frame_counted) ++stats_.frames_enqueued;
        ++stats_.frames_dropped;
        ++stats_.input_incomplete;
        stats_.input_points_dropped = saturatedAddU64(
            stats_.input_points_dropped, source_point_count);
        stats_.input_bytes_dropped = saturatedAddU64(
            stats_.input_bytes_dropped,
            saturatedMultiplySize(source_point_count, sizeof(PointType)));
        stats_.incomplete = true;
        scheduleInputResyncLocked(timestamp, next_sequence_);
        ++input_epoch_;
    }
    refreshInputStatsLocked();
    work_cv_.notify_one();
    return false;
}

void MapBuildWorker::request(double timestamp)
{
    std::lock_guard<std::mutex> lock(mutex_);
    if (!running_ || stopping_) return;
    ++stats_.full_requested;
    if (full_pending_) ++stats_.full_coalesced;
    full_pending_ = true;
    full_timestamp_ = timestamp;
    full_sequence_ = next_sequence_;
    ++input_epoch_;
    work_cv_.notify_one();
}

void MapBuildWorker::requestTileResync(double timestamp)
{
    std::lock_guard<std::mutex> lock(mutex_);
    if (!running_ || stopping_) return;
    if (!tile_resync_pending_) {
        tile_resync_sequence_ = next_sequence_;
        tile_resync_pending_ = true;
    } else {
        tile_resync_sequence_ = std::max(
            tile_resync_sequence_, next_sequence_);
    }
    ++input_epoch_;
    dirty_timestamp_ = timestamp;
    ++stats_.tile_resync_requested;
    work_cv_.notify_one();
}

bool MapBuildWorker::flushFor(std::chrono::milliseconds timeout)
{
    const auto deadline = std::chrono::steady_clock::now() + timeout;
    while (true) {
        std::unique_lock<std::mutex> lock(mutex_);
        if (!idle_cv_.wait_until(lock, deadline, [this] {
                return tile_tasks_.empty() && !full_pending_ &&
                       !tile_resync_pending_ && !dirty_drain_pending_ && !busy_;
            }))
        {
            return false;
        }
        lock.unlock();

        const auto now = std::chrono::steady_clock::now();
        if (now >= deadline ||
            !output_.flushFor(
                std::chrono::duration_cast<std::chrono::milliseconds>(
                    deadline - now)))
        {
            return false;
        }

        // An output failure can request a Tile resync while the first phase is
        // waiting. Recheck the builder after output becomes idle and repeat
        // under the same deadline until both stages form a stable barrier.
        lock.lock();
        if (tile_tasks_.empty() && !full_pending_ && !tile_resync_pending_ &&
            !dirty_drain_pending_ && !busy_) {
            return true;
        }
    }
}

void MapBuildWorker::flush()
{
    // Compatibility helper for tests and pause handling. It is bounded so a
    // failed network consumer cannot hold the control thread forever.
    flushFor(std::chrono::seconds(10));
}

bool MapBuildWorker::stopFor(
    std::chrono::milliseconds timeout, bool drain)
{
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
            stats_.frames_cancelled += cancelQueuedInputLocked();
            stats_.control_tasks_cancelled +=
                (full_pending_ ? 1U : 0U) +
                (tile_resync_pending_ ? 1U : 0U) +
                (dirty_drain_pending_ ? 1U : 0U);
            full_pending_ = false;
            tile_resync_pending_ = false;
            dirty_drain_pending_ = false;
            refreshInputStatsLocked();
        }
        work_cv_.notify_all();
        if (!exit_cv_.wait_until(lock, deadline,
                                 [this] { return worker_exited_; })) {
            ++stats_.stop_timeouts;
            return false;
        }
    }

    if (worker_.joinable()) worker_.join();
    const auto now = std::chrono::steady_clock::now();
    const auto remaining = now < deadline
        ? std::chrono::duration_cast<std::chrono::milliseconds>(deadline - now)
        : std::chrono::milliseconds(0);
    if (!output_.stopFor(remaining, drain)) {
        std::lock_guard<std::mutex> lock(mutex_);
        ++stats_.stop_timeouts;
        return false;
    }

    std::lock_guard<std::mutex> lock(mutex_);
    running_ = false;
    stopping_ = false;
    busy_ = false;
    refreshInputStatsLocked();
    idle_cv_.notify_all();
    return true;
}

void MapBuildWorker::stop(bool drain)
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
            stats_.frames_cancelled += cancelQueuedInputLocked();
            stats_.control_tasks_cancelled +=
                (full_pending_ ? 1U : 0U) +
                (tile_resync_pending_ ? 1U : 0U) +
                (dirty_drain_pending_ ? 1U : 0U);
            full_pending_ = false;
            tile_resync_pending_ = false;
            dirty_drain_pending_ = false;
            refreshInputStatsLocked();
        }
    }
    work_cv_.notify_all();
    if (worker_.joinable()) worker_.join();
    output_.stop(drain);
    std::lock_guard<std::mutex> lock(mutex_);
    running_ = false;
    stopping_ = false;
    busy_ = false;
    refreshInputStatsLocked();
    idle_cv_.notify_all();
}

bool MapBuildWorker::isRunning() const
{
    std::lock_guard<std::mutex> lock(mutex_);
    return running_ && !stopping_;
}

MapBuildWorker::Stats MapBuildWorker::stats() const
{
    std::lock_guard<std::mutex> lock(mutex_);
    Stats result = stats_;
    result.input_queue_tasks = tile_tasks_.size();
    result.output = output_.stats();
    result.busy = busy_;
    result.stopping = stopping_;
    result.worker_exited = worker_exited_;
    return result;
}

MapBuildWorker::TimingStats MapBuildWorker::timingStats() const
{
    BoundedTimingWindow<> map_build;
    BoundedTimingWindow<> tile_extract;
    BoundedTimingWindow<> full_snapshot;
    {
        std::lock_guard<std::mutex> lock(mutex_);
        map_build = map_build_timing_;
        tile_extract = tile_extract_timing_;
        full_snapshot = full_snapshot_timing_;
    }
    TimingStats result;
    result.map_build = map_build.summary();
    result.tile_extract = tile_extract.summary();
    result.full_snapshot = full_snapshot.summary();
    result.output = output_.timingStats();
    return result;
}

void MapBuildWorker::run()
{
    while (true) {
        enum class Work { Tile, Full, TileResync, DirtyDrain };
        Work work = Work::Tile;
        TileTask tile_task;
        double timestamp = 0.0;
        {
            std::unique_lock<std::mutex> lock(mutex_);
            work_cv_.wait(lock, [this] {
                return stopping_ || !tile_tasks_.empty() || full_pending_ ||
                       tile_resync_pending_ || dirty_drain_pending_;
            });
            if (stopping_ && (!drain_ ||
                (tile_tasks_.empty() && !full_pending_ && !tile_resync_pending_ &&
                 !dirty_drain_pending_)))
            {
                break;
            }

            const bool full_ready = full_pending_ &&
                built_sequence_ >= full_sequence_;
            const bool tile_resync_ready = tile_resync_pending_ &&
                built_sequence_ >= tile_resync_sequence_;
            if (!tile_tasks_.empty() && !full_ready && !tile_resync_ready) {
                auto queued = tile_tasks_.begin();
                tile_task_index_.erase(queued->key);
                pending_input_points_ -= std::min(
                    pending_input_points_, queued->point_count);
                tile_task = std::move(*queued);
                tile_tasks_.erase(queued);
                work = Work::Tile;
            } else if (tile_resync_ready) {
                tile_resync_pending_ = false;
                tile_resync_sequence_ = 0;
                work = Work::TileResync;
                timestamp = dirty_timestamp_;
            } else if (full_ready) {
                timestamp = full_timestamp_;
                full_pending_ = false;
                work = Work::Full;
            } else if (dirty_drain_pending_) {
                dirty_drain_pending_ = false;
                work = Work::DirtyDrain;
                timestamp = dirty_timestamp_;
            } else {
                // A sequence can only be incomplete while a Tile task is
                // queued or in flight. Reaching this branch would indicate a
                // bookkeeping defect; fail the pending controls closed rather
                // than running a snapshot before its barrier.
                ++stats_.failures;
                full_pending_ = false;
                tile_resync_pending_ = false;
                continue;
            }
            busy_ = true;
            refreshInputStatsLocked();
        }

        if (work == Work::Tile) {
            double last_timestamp = 0.0;
            for (auto& contribution : tile_task.contributions) {
                const auto begin = std::chrono::steady_clock::now();
                bool built = false;
                try {
                    store_.addFrame(
                        contribution.cloud, contribution.timestamp,
                        contribution.has_current_position
                            ? &contribution.current_position : nullptr);
                    built = true;
                } catch (const std::exception& error) {
                    std::cerr << "[MapBuildWorker] Tile input failed: "
                              << error.what() << std::endl;
                } catch (...) {
                    std::cerr << "[MapBuildWorker] Tile input failed with unknown error"
                              << std::endl;
                }
                const double build_ms = std::chrono::duration<double, std::milli>(
                    std::chrono::steady_clock::now() - begin).count();
                last_timestamp = contribution.timestamp;
                const TiledMapStore::Stats contribution_store_stats =
                    store_.stats();
                {
                    std::lock_guard<std::mutex> lock(mutex_);
                    if (!built) {
                        ++stats_.failures;
                        recordInputLossLocked(contribution.sequence, 0);
                    }
                    completeContributionLocked(
                        contribution.sequence, built);
                    stats_.store = contribution_store_stats;
                    stats_.incomplete = stats_.incomplete ||
                        contribution_store_stats.incomplete;
                    stats_.last_frame_points = contribution.cloud.size();
                    stats_.last_build_ms = build_ms;
                    map_build_timing_.record(build_ms);
                }
            }
            bool dirty_remains = false;
            try {
                dirty_remains = emitDirtyTiles(last_timestamp);
            } catch (const std::exception& error) {
                std::cerr << "[MapBuildWorker] Tile output failed: "
                          << error.what() << std::endl;
                dirty_remains = tile_output_enabled_ &&
                    store_.stats().dirty_count > 0;
                std::lock_guard<std::mutex> lock(mutex_);
                ++stats_.failures;
            } catch (...) {
                std::cerr << "[MapBuildWorker] Tile output failed with unknown error"
                          << std::endl;
                dirty_remains = tile_output_enabled_ &&
                    store_.stats().dirty_count > 0;
                std::lock_guard<std::mutex> lock(mutex_);
                ++stats_.failures;
            }
            const TiledMapStore::Stats store_stats = store_.stats();
            std::lock_guard<std::mutex> lock(mutex_);
            dirty_drain_pending_ = dirty_remains;
            dirty_timestamp_ = last_timestamp;
            stats_.store = store_stats;
            stats_.incomplete = stats_.incomplete || store_stats.incomplete;
            if (tile_resync_in_progress_ && !dirty_remains) {
                tile_resync_in_progress_ = false;
                ++stats_.tile_resync_completed;
            }
        } else try {
            if (work == Work::TileResync) {
                tile_resync_in_progress_ = true;
                store_.markAllDirty();
                const bool dirty_remains = emitDirtyTiles(timestamp);
                std::lock_guard<std::mutex> lock(mutex_);
                dirty_drain_pending_ = dirty_remains;
                if (!dirty_remains) {
                    tile_resync_in_progress_ = false;
                    ++stats_.tile_resync_completed;
                }
            } else if (work == Work::DirtyDrain) {
                const bool dirty_remains = emitDirtyTiles(timestamp);
                if (dirty_remains) std::this_thread::sleep_for(std::chrono::milliseconds(1));
                std::lock_guard<std::mutex> lock(mutex_);
                dirty_drain_pending_ = dirty_remains;
                if (tile_resync_in_progress_ && !dirty_remains) {
                    tile_resync_in_progress_ = false;
                    ++stats_.tile_resync_completed;
                }
            } else {
                const auto begin = std::chrono::steady_clock::now();
                PointCloudXYZI snapshot = store_.snapshot(config_.full_voxel_leaf_m);
                const double snapshot_ms = std::chrono::duration<double, std::milli>(
                    std::chrono::steady_clock::now() - begin).count();
                const size_t point_count = snapshot.size();
                const bool accepted = output_.enqueueFull(std::move(snapshot), timestamp);
                std::lock_guard<std::mutex> lock(mutex_);
                ++stats_.full_built;
                if (!accepted) ++stats_.failures;
                stats_.last_full_points = point_count;
                stats_.last_full_snapshot_ms = snapshot_ms;
                full_snapshot_timing_.record(snapshot_ms);
            }
        } catch (const std::exception& error) {
            std::cerr << "[MapBuildWorker] task failed: " << error.what() << std::endl;
            const bool dirty_remains = tile_output_enabled_ &&
                store_.stats().dirty_count > 0;
            std::lock_guard<std::mutex> lock(mutex_);
            ++stats_.failures;
            if (dirty_remains && (!stopping_ || drain_)) {
                dirty_drain_pending_ = true;
                dirty_timestamp_ = timestamp;
            }
        } catch (...) {
            std::cerr << "[MapBuildWorker] task failed with unknown error" << std::endl;
            const bool dirty_remains = tile_output_enabled_ &&
                store_.stats().dirty_count > 0;
            std::lock_guard<std::mutex> lock(mutex_);
            ++stats_.failures;
            if (dirty_remains && (!stopping_ || drain_)) {
                dirty_drain_pending_ = true;
                dirty_timestamp_ = timestamp;
            }
        }

        {
            std::lock_guard<std::mutex> lock(mutex_);
            busy_ = false;
            stats_.store = store_.stats();
            stats_.incomplete = stats_.incomplete || stats_.store.incomplete;
        }
        work_cv_.notify_one();
        idle_cv_.notify_all();
    }
    {
        std::lock_guard<std::mutex> lock(mutex_);
        busy_ = false;
        worker_exited_ = true;
    }
    idle_cv_.notify_all();
    exit_cv_.notify_all();
}

bool MapBuildWorker::emitDirtyTiles(double timestamp)
{
    if (!tile_output_enabled_) {
        store_.discardDirtyTiles();
        return false;
    }
    const auto begin = std::chrono::steady_clock::now();
    std::vector<TileUpdate> updates = store_.consumeDirtyTiles(
        config_.max_tiles_per_update,
        config_.max_points_per_update,
        config_.max_bytes_per_update);
    const double extract_ms = std::chrono::duration<double, std::milli>(
        std::chrono::steady_clock::now() - begin).count();
    const size_t count = updates.size();
    if (!updates.empty()) {
        bool accepted = false;
        try {
            accepted = output_.enqueueTiles(updates, timestamp);
        } catch (...) {
            // enqueueTiles takes ownership of a copy and may allocate while
            // staging the atomic batch. Keep the store retryable if that
            // allocation fails or a future implementation throws.
            store_.restoreDirtyTiles(updates);
            throw;
        }
        if (accepted) {
            store_.confirmTilesHandedOff(updates);
        } else {
            store_.restoreDirtyTiles(updates);
        }
    }
    const bool dirty_remains = store_.stats().dirty_count > 0;
    std::lock_guard<std::mutex> lock(mutex_);
    if (count > 0) ++stats_.tile_batches;
    stats_.tile_updates += count;
    stats_.last_tile_extract_ms = extract_ms;
    tile_extract_timing_.record(extract_ms);
    return dirty_remains;
}

void MapBuildWorker::refreshInputStatsLocked()
{
    size_t points = 0;
    for (const auto& task : tile_tasks_) {
        points = saturatedAddSize(points, task.point_count);
    }
    pending_input_points_ = points;
    pending_input_bytes_ = estimateInputBytesLocked();
    stats_.input_queue_tasks = tile_tasks_.size();
    stats_.input_queue_points = pending_input_points_;
    stats_.input_queue_bytes = pending_input_bytes_;
    stats_.max_input_queue_tasks =
        std::max(stats_.max_input_queue_tasks, tile_tasks_.size());
    stats_.max_input_queue_points =
        std::max(stats_.max_input_queue_points, pending_input_points_);
    stats_.max_input_queue_bytes =
        std::max(stats_.max_input_queue_bytes, stats_.input_queue_bytes);
}

size_t MapBuildWorker::inputPointLimit() const noexcept
{
    size_t limit = std::numeric_limits<size_t>::max();
    if (config_.input_queue_max_points > 0) {
        limit = std::min(limit, config_.input_queue_max_points);
    }
    if (config_.input_queue_max_bytes > 0) {
        limit = std::min(
            limit, config_.input_queue_max_bytes / sizeof(PointType));
    }
    return limit;
}

size_t MapBuildWorker::estimateTileTaskBytes(const TileTask& task) const noexcept
{
    size_t bytes = sizeof(TileTask) + 2 * sizeof(void*);
    bytes = saturatedAddSize(
        bytes, sizeof(PendingTileKey) + sizeof(TileTaskList::iterator) +
            2 * sizeof(void*));
    for (const auto& contribution : task.contributions) {
        bytes = saturatedAddSize(
            bytes, sizeof(TileContribution) + 2 * sizeof(void*));
        bytes = saturatedAddSize(
            bytes, saturatedMultiplySize(
                contribution.cloud.points.capacity(), sizeof(PointType)));
    }
    return bytes;
}

size_t MapBuildWorker::estimateInputBytesLocked() const noexcept
{
    size_t bytes = tile_task_index_.empty() ? 0 : saturatedMultiplySize(
        tile_task_index_.bucket_count(), 2 * sizeof(void*));
    for (const auto& task : tile_tasks_) {
        bytes = saturatedAddSize(bytes, estimateTileTaskBytes(task));
    }
    if (!frame_progress_.empty()) {
        bytes = saturatedAddSize(
            bytes, saturatedMultiplySize(
                frame_progress_.bucket_count(), 2 * sizeof(void*)));
        bytes = saturatedAddSize(
            bytes, saturatedMultiplySize(
                frame_progress_.size(),
                sizeof(decltype(frame_progress_)::value_type) +
                    2 * sizeof(void*)));
    }
    return bytes;
}

void MapBuildWorker::advanceBuiltSequenceLocked()
{
    while (built_sequence_ < next_sequence_) {
        const uint64_t candidate = built_sequence_ + 1;
        const auto progress = frame_progress_.find(candidate);
        if (progress == frame_progress_.end() || !progress->second.sealed ||
            progress->second.pending_contributions != 0)
        {
            break;
        }
        if (progress->second.any_built) {
            ++stats_.frames_built;
        } else if (!progress->second.cancelled) {
            ++stats_.frames_dropped;
        }
        frame_progress_.erase(progress);
        built_sequence_ = candidate;
    }
}

void MapBuildWorker::completeContributionLocked(
    uint64_t sequence, bool built, bool cancelled)
{
    const auto progress = frame_progress_.find(sequence);
    if (progress == frame_progress_.end()) return;
    if (progress->second.pending_contributions > 0) {
        --progress->second.pending_contributions;
    }
    progress->second.any_built = progress->second.any_built || built;
    progress->second.cancelled = progress->second.cancelled || cancelled;
    advanceBuiltSequenceLocked();
}

void MapBuildWorker::recordInputLossLocked(uint64_t sequence, size_t points)
{
    const auto progress = frame_progress_.find(sequence);
    if (progress != frame_progress_.end() && !progress->second.incomplete) {
        progress->second.incomplete = true;
        ++stats_.input_incomplete;
    }
    stats_.input_points_dropped = saturatedAddU64(
        stats_.input_points_dropped, points);
    stats_.input_bytes_dropped = saturatedAddU64(
        stats_.input_bytes_dropped,
        saturatedMultiplySize(points, sizeof(PointType)));
    stats_.incomplete = true;
}

size_t MapBuildWorker::discardOldestPointsLocked(
    TileTaskList::iterator task, size_t requested)
{
    size_t discarded = 0;
    while (discarded < requested && !task->contributions.empty()) {
        auto contribution = task->contributions.begin();
        const size_t available = contribution->cloud.size();
        const size_t count = std::min(requested - discarded, available);
        if (count == available) {
            const uint64_t sequence = contribution->sequence;
            recordInputLossLocked(sequence, count);
            task->point_count -= std::min(task->point_count, count);
            pending_input_points_ -= std::min(pending_input_points_, count);
            task->contributions.erase(contribution);
            ++stats_.input_tile_changes_dropped;
            completeContributionLocked(sequence, false);
        } else {
            contribution->cloud.points.erase(
                contribution->cloud.points.begin(),
                contribution->cloud.points.begin() +
                    static_cast<std::ptrdiff_t>(count));
            contribution->cloud.width = static_cast<uint32_t>(
                contribution->cloud.size());
            contribution->cloud.height = 1;
            recordInputLossLocked(contribution->sequence, count);
            task->point_count -= std::min(task->point_count, count);
            pending_input_points_ -= std::min(pending_input_points_, count);
        }
        discarded = saturatedAddSize(discarded, count);
    }
    refreshInputStatsLocked();
    return discarded;
}

void MapBuildWorker::evictTileTaskLocked(TileTaskList::iterator task)
{
    if (task == tile_tasks_.end()) return;
    ++stats_.input_tile_tasks_evicted;
    while (!task->contributions.empty()) {
        discardOldestPointsLocked(
            task, task->contributions.front().cloud.size());
    }
    tile_task_index_.erase(task->key);
    tile_tasks_.erase(task);
    refreshInputStatsLocked();
}

void MapBuildWorker::eraseEmptyTileTaskLocked(TileTaskList::iterator task)
{
    if (task == tile_tasks_.end() || !task->contributions.empty()) return;
    tile_task_index_.erase(task->key);
    tile_tasks_.erase(task);
    refreshInputStatsLocked();
}

void MapBuildWorker::scheduleInputResyncLocked(
    double timestamp, uint64_t sequence)
{
    if (!tile_output_enabled_) return;
    tile_resync_pending_ = true;
    tile_resync_sequence_ = std::max(tile_resync_sequence_, sequence);
    dirty_timestamp_ = timestamp;
    ++stats_.input_resync_requested;
    ++stats_.tile_resync_requested;
}

size_t MapBuildWorker::cancelQueuedInputLocked()
{
    size_t cancelled_frames = 0;
    for (auto& progress : frame_progress_) {
        if (progress.second.pending_contributions > 0 &&
            !progress.second.cancelled)
        {
            progress.second.cancelled = true;
            ++cancelled_frames;
        }
    }
    for (const auto& task : tile_tasks_) {
        for (const auto& contribution : task.contributions) {
            completeContributionLocked(contribution.sequence, false, true);
        }
    }
    tile_tasks_.clear();
    tile_task_index_.clear();
    pending_input_points_ = 0;
    pending_input_bytes_ = 0;
    refreshInputStatsLocked();
    return cancelled_frames;
}
