#include "foxglove_output_worker.h"

#include <chrono>
#include <exception>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <utility>

namespace {

bool checkedAddSize(size_t left, size_t right, size_t& result) noexcept
{
    if (right > std::numeric_limits<size_t>::max() - left) return false;
    result = left + right;
    return true;
}

size_t saturatedAddSize(size_t left, size_t right) noexcept
{
    size_t result = 0;
    return checkedAddSize(left, right, result)
        ? result
        : std::numeric_limits<size_t>::max();
}

size_t saturatedMultiplySize(size_t left, size_t right) noexcept
{
    if (left != 0 && right > std::numeric_limits<size_t>::max() / left) {
        return std::numeric_limits<size_t>::max();
    }
    return left * right;
}

}  // namespace

FoxgloveOutputWorker::FoxgloveOutputWorker(
    PublishCallback full_callback, TilePublishCallback tile_callback, Config config,
    TileFailureCallback tile_failure_callback)
    : full_callback_(std::move(full_callback)),
      tile_callback_(std::move(tile_callback)),
      tile_failure_callback_(std::move(tile_failure_callback)),
      config_(config)
{}

FoxgloveOutputWorker::~FoxgloveOutputWorker()
{
    stop(false);
}

void FoxgloveOutputWorker::start()
{
    std::lock_guard<std::mutex> lock(mutex_);
    if (running_) return;
    stopping_ = false;
    drain_ = true;
    busy_ = false;
    in_callback_ = false;
    worker_exited_ = false;
    has_full_ = false;
    tiles_.clear();
    has_retry_tile_ = false;
    retry_tile_ = TileTask{};
    inflight_points_ = 0;
    inflight_bytes_ = 0;
    has_inflight_tile_ = false;
    inflight_tile_key_ = TileKey{};
    inflight_tile_version_ = 0;
    tiles_since_full_ = 0;
    last_tile_publish_ = std::chrono::steady_clock::time_point{};
    stats_ = Stats{};
    full_publish_timing_ = BoundedTimingWindow<>{};
    tile_publish_timing_ = BoundedTimingWindow<>{};
    running_ = true;
    try {
        worker_ = std::thread(&FoxgloveOutputWorker::run, this);
    } catch (...) {
        running_ = false;
        worker_exited_ = true;
        throw;
    }
}

bool FoxgloveOutputWorker::enqueueFull(PointCloudXYZI cloud, double timestamp)
{
    std::lock_guard<std::mutex> lock(mutex_);
    if (!running_ || stopping_ || !full_callback_) return false;
    ++stats_.full_enqueued;
    refreshQueueStatsLocked();
    const size_t old_points = has_full_ ? full_.cloud.size() : 0;
    const size_t old_bytes = saturatedMultiplySize(old_points, sizeof(PointType));
    const size_t new_points = cloud.size();
    const size_t new_bytes = saturatedMultiplySize(new_points, sizeof(PointType));
    size_t retained_points = stats_.current_points - old_points;
    size_t retained_bytes = stats_.current_bytes - old_bytes;
    if (!checkedAddSize(retained_points, inflight_points_, retained_points) ||
        !checkedAddSize(retained_bytes, inflight_bytes_, retained_bytes))
    {
        ++stats_.full_dropped;
        return false;
    }
    size_t projected_points = 0;
    size_t projected_bytes = 0;
    if (!checkedAddSize(retained_points, new_points, projected_points) ||
        !checkedAddSize(retained_bytes, new_bytes, projected_bytes) ||
        (config_.max_pending_points > 0 &&
         projected_points > config_.max_pending_points) ||
        (config_.max_pending_bytes > 0 &&
         projected_bytes > config_.max_pending_bytes))
    {
        ++stats_.full_dropped;
        return false;
    }
    if (has_full_) ++stats_.full_overwritten;
    full_ = FullTask{std::move(cloud), timestamp};
    has_full_ = true;
    refreshQueueStatsLocked();
    work_cv_.notify_one();
    return true;
}

bool FoxgloveOutputWorker::enqueueTiles(
    std::vector<TileUpdate> updates, double timestamp)
{
    if (!tile_callback_ || updates.empty()) return false;
    std::lock_guard<std::mutex> lock(mutex_);
    if (!running_ || stopping_) return false;

    struct ProspectiveTile {
        TileUpdate* replacement = nullptr;
        uint64_t version = 0;
        bool has_version = false;
        bool queued = false;
        bool inflight_only = false;
    };
    std::unordered_map<TileKey, ProspectiveTile, TileKeyHash> prospective;
    prospective.reserve(updates.size());
    for (auto& update : updates) {
        auto result = prospective.try_emplace(update.key);
        ProspectiveTile& tile = result.first->second;
        if (result.second) {
            const auto existing = tiles_.find(update.key);
            tile.queued = existing != tiles_.end();
            if (tile.queued) {
                tile.version = existing->second.update.version;
                tile.has_version = true;
            } else if (has_inflight_tile_ &&
                       inflight_tile_key_ == update.key) {
                tile.version = inflight_tile_version_;
                tile.has_version = true;
                tile.inflight_only = true;
            }
        }
        if (!tile.has_version) {
            tile.replacement = &update;
            tile.version = update.version;
            tile.has_version = true;
        } else if (update.version > tile.version ||
                   (update.resync && tile.inflight_only &&
                    update.version == tile.version)) {
            tile.replacement = &update;
            tile.version = update.version;
        }
    }

    refreshQueueStatsLocked();
    size_t final_tiles = saturatedAddSize(
        tiles_.size(), has_inflight_tile_ ? 1U : 0U);
    size_t final_points = 0;
    size_t final_bytes = 0;
    if (!checkedAddSize(stats_.current_points, inflight_points_, final_points) ||
        !checkedAddSize(stats_.current_bytes, inflight_bytes_, final_bytes))
    {
        stats_.tiles_enqueued += updates.size();
        stats_.tiles_dropped += updates.size();
        return false;
    }
    for (const auto& item : prospective) {
        const ProspectiveTile& tile = item.second;
        if (!tile.replacement) continue;  // Every update was older than queued state.
        const size_t new_points = tile.replacement->points.size();
        const size_t new_bytes = tile.replacement->estimatedBytes();
        if (tile.queued) {
            const TileTask& old = tiles_.find(item.first)->second;
            final_points -= old.update.points.size();
            final_bytes -= old.update.estimatedBytes();
        } else {
            if (final_tiles == std::numeric_limits<size_t>::max()) {
                stats_.tiles_enqueued += updates.size();
                stats_.tiles_dropped += updates.size();
                return false;
            }
            ++final_tiles;
        }
        if (new_points > std::numeric_limits<size_t>::max() - final_points ||
            new_bytes > std::numeric_limits<size_t>::max() - final_bytes)
        {
            stats_.tiles_enqueued += updates.size();
            stats_.tiles_dropped += updates.size();
            return false;
        }
        final_points += new_points;
        final_bytes += new_bytes;
    }

    if ((config_.max_pending_tiles > 0 &&
         final_tiles > config_.max_pending_tiles) ||
        (config_.max_pending_points > 0 &&
         final_points > config_.max_pending_points) ||
        (config_.max_pending_bytes > 0 &&
         final_bytes > config_.max_pending_bytes))
    {
        stats_.tiles_enqueued += updates.size();
        stats_.tiles_dropped += updates.size();
        return false;
    }

    // Materialize every replacement in an isolated container first. A failed
    // allocation leaves the live queue untouched; node handles then make the
    // logical commit allocation-free after the destination bucket reserve.
    std::unordered_map<TileKey, TileTask, TileKeyHash> staged;
    staged.reserve(prospective.size());
    size_t new_queued_keys = 0;
    for (auto& item : prospective) {
        ProspectiveTile& tile = item.second;
        if (!tile.replacement) continue;
        if (!tile.queued) ++new_queued_keys;
        staged.emplace(item.first, TileTask{
            std::move(*tile.replacement), timestamp});
    }
    size_t required_queue_size = 0;
    if (!checkedAddSize(tiles_.size(), new_queued_keys,
                        required_queue_size)) {
        throw std::overflow_error("Foxglove Tile queue size overflow");
    }
    tiles_.reserve(required_queue_size);

    while (!staged.empty()) {
        auto staged_iterator = staged.begin();
        auto found = tiles_.find(staged_iterator->first);
        if (found != tiles_.end()) {
            found->second = std::move(staged_iterator->second);
            staged.erase(staged_iterator);
        } else {
            auto node = staged.extract(staged_iterator);
            tiles_.insert(std::move(node));
        }
    }
    stats_.tiles_enqueued += updates.size();
    stats_.tiles_merged += updates.size() - new_queued_keys;
    refreshQueueStatsLocked();
    work_cv_.notify_one();
    return true;
}

bool FoxgloveOutputWorker::flushFor(std::chrono::milliseconds timeout)
{
    std::unique_lock<std::mutex> lock(mutex_);
    return idle_cv_.wait_for(lock, timeout, [this] {
        return !has_full_ && tiles_.empty() && !has_retry_tile_ && !busy_;
    });
}

bool FoxgloveOutputWorker::stopFor(
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
            if (has_full_) ++stats_.full_cancelled;
            stats_.tiles_cancelled += tiles_.size();
            if (has_retry_tile_) ++stats_.tiles_cancelled;
            has_full_ = false;
            tiles_.clear();
            if (has_retry_tile_) {
                has_retry_tile_ = false;
                retry_tile_ = TileTask{};
                inflight_points_ = 0;
                inflight_bytes_ = 0;
                has_inflight_tile_ = false;
                inflight_tile_version_ = 0;
            }
            refreshQueueStatsLocked();
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
    refreshQueueStatsLocked();
    idle_cv_.notify_all();
    return true;
}

void FoxgloveOutputWorker::stop(bool drain)
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
            if (has_full_) ++stats_.full_cancelled;
            stats_.tiles_cancelled += tiles_.size();
            if (has_retry_tile_) ++stats_.tiles_cancelled;
            has_full_ = false;
            tiles_.clear();
            if (has_retry_tile_) {
                has_retry_tile_ = false;
                retry_tile_ = TileTask{};
                inflight_points_ = 0;
                inflight_bytes_ = 0;
                has_inflight_tile_ = false;
                inflight_tile_version_ = 0;
            }
            refreshQueueStatsLocked();
        }
    }
    work_cv_.notify_all();
    if (worker_.joinable()) worker_.join();
    std::lock_guard<std::mutex> lock(mutex_);
    running_ = false;
    stopping_ = false;
    busy_ = false;
    in_callback_ = false;
    refreshQueueStatsLocked();
    idle_cv_.notify_all();
}

bool FoxgloveOutputWorker::isRunning() const
{
    std::lock_guard<std::mutex> lock(mutex_);
    return running_ && !stopping_;
}

FoxgloveOutputWorker::Stats FoxgloveOutputWorker::stats() const
{
    std::lock_guard<std::mutex> lock(mutex_);
    Stats result = stats_;
    result.busy = busy_;
    result.in_callback = in_callback_;
    result.stopping = stopping_;
    result.worker_exited = worker_exited_;
    return result;
}

FoxgloveOutputWorker::TimingStats FoxgloveOutputWorker::timingStats() const
{
    BoundedTimingWindow<> full;
    BoundedTimingWindow<> tile;
    {
        std::lock_guard<std::mutex> lock(mutex_);
        full = full_publish_timing_;
        tile = tile_publish_timing_;
    }
    return TimingStats{full.summary(), tile.summary()};
}

void FoxgloveOutputWorker::run()
{
    while (true) {
        bool is_tile = false;
        bool is_retry = false;
        FullTask full;
        TileTask tile;
        {
            std::unique_lock<std::mutex> lock(mutex_);
            work_cv_.wait(lock, [this] {
                return stopping_ || has_retry_tile_ || has_full_ ||
                       !tiles_.empty();
            });
            if (stopping_ && (!drain_ ||
                (!has_retry_tile_ && !has_full_ && tiles_.empty()))) break;

            // Tile updates normally have priority, but a continuously dirty
            // map can keep this queue non-empty forever. Bound the Tile burst
            // so the compatible /map PointCloud snapshot cannot starve.
            const bool full_due = has_full_ &&
                tiles_since_full_ >= config_.max_tile_burst_before_full;
            if (full_due) {
                inflight_points_ = full_.cloud.size();
                inflight_bytes_ = saturatedMultiplySize(
                    full_.cloud.size(), sizeof(PointType));
                has_inflight_tile_ = false;
                full = std::move(full_);
                has_full_ = false;
                tiles_since_full_ = 0;
            } else if (has_retry_tile_) {
                tile = std::move(retry_tile_);
                retry_tile_ = TileTask{};
                has_retry_tile_ = false;
                is_tile = true;
                is_retry = true;
                if (tiles_since_full_ < config_.max_tile_burst_before_full) {
                    ++tiles_since_full_;
                }
                // The failed task remained charged to the in-flight ledger.
                inflight_points_ = tile.update.points.size();
                inflight_bytes_ = tile.update.estimatedBytes();
                has_inflight_tile_ = true;
                inflight_tile_key_ = tile.update.key;
                inflight_tile_version_ = tile.update.version;
            } else if (!tiles_.empty()) {
                auto it = tiles_.begin();
                inflight_points_ = it->second.update.points.size();
                inflight_bytes_ = it->second.update.estimatedBytes();
                has_inflight_tile_ = true;
                inflight_tile_key_ = it->second.update.key;
                inflight_tile_version_ = it->second.update.version;
                tile = std::move(it->second);
                tiles_.erase(it);
                is_tile = true;
                if (tiles_since_full_ < config_.max_tile_burst_before_full) {
                    ++tiles_since_full_;
                }
            } else if (has_full_) {
                inflight_points_ = full_.cloud.size();
                inflight_bytes_ = saturatedMultiplySize(
                    full_.cloud.size(), sizeof(PointType));
                has_inflight_tile_ = false;
                full = std::move(full_);
                has_full_ = false;
                tiles_since_full_ = 0;
            }
            busy_ = true;
            refreshQueueStatsLocked();
        }

        bool failed = false;
        bool cancelled = false;
        if (is_tile && (config_.tile_publish_hz > 0.0 || is_retry) &&
            last_tile_publish_ != std::chrono::steady_clock::time_point{})
        {
            const auto interval = config_.tile_publish_hz > 0.0
                ? std::chrono::duration<double>(1.0 / config_.tile_publish_hz)
                : std::chrono::duration<double>(0.01);
            const auto publish_at = last_tile_publish_ +
                std::chrono::duration_cast<std::chrono::steady_clock::duration>(interval);
            std::unique_lock<std::mutex> lock(mutex_);
            work_cv_.wait_until(lock, publish_at, [this] {
                return stopping_ && !drain_;
            });
            cancelled = stopping_ && !drain_;
        }

        // Commit the task to the synchronous callback under the same lock used
        // by stopFor(). A non-draining stop can cancel it up to this point; once
        // in_callback_ is true the callback is intentionally treated as
        // non-cancellable.
        {
            std::lock_guard<std::mutex> lock(mutex_);
            if (stopping_ && !drain_) cancelled = true;
            if (!cancelled) in_callback_ = true;
        }
        const auto begin = std::chrono::steady_clock::now();
        if (!cancelled) {
            try {
                if (is_tile) {
                    if (tile_callback_) tile_callback_(tile.update, tile.timestamp);
                } else if (full_callback_) {
                    full_callback_(full.cloud, full.timestamp);
                }
            } catch (const std::exception& error) {
                failed = true;
                std::cerr << "[FoxgloveOutputWorker] output failed: "
                          << error.what() << std::endl;
            } catch (...) {
                failed = true;
                std::cerr << "[FoxgloveOutputWorker] output failed with unknown error"
                          << std::endl;
            }
        }
        if (failed && is_tile && tile_failure_callback_) {
            // The update has already left the bounded queue. Notify the map
            // owner so it can rebuild dirty state instead of silently losing
            // the failed tile version.
            try {
                tile_failure_callback_(tile.update, tile.timestamp);
            } catch (const std::exception& error) {
                std::cerr << "[FoxgloveOutputWorker] tile failure callback failed: "
                          << error.what() << std::endl;
            } catch (...) {
                std::cerr << "[FoxgloveOutputWorker] tile failure callback failed"
                          << std::endl;
            }
        }
        const double elapsed = std::chrono::duration<double, std::milli>(
            std::chrono::steady_clock::now() - begin).count();

        {
            std::lock_guard<std::mutex> lock(mutex_);
            in_callback_ = false;
            if (failed) ++stats_.failures;
            bool retained_retry = false;
            if (is_tile) {
                if (cancelled) {
                    ++stats_.tiles_cancelled;
                } else {
                    if (!failed) ++stats_.tiles_published;
                    stats_.last_tile_publish_ms = elapsed;
                    last_tile_publish_ = std::chrono::steady_clock::now();
                    if (failed && !stopping_) {
                        const auto successor = tiles_.find(tile.update.key);
                        const bool superseded = successor != tiles_.end() &&
                            successor->second.update.version >=
                                tile.update.version;
                        if (!superseded) {
                            retry_tile_ = std::move(tile);
                            has_retry_tile_ = true;
                            retained_retry = true;
                            ++stats_.tiles_retried;
                        }
                    }
                }
            } else {
                if (cancelled) {
                    ++stats_.full_cancelled;
                } else {
                    if (!failed) ++stats_.full_published;
                    stats_.last_full_publish_ms = elapsed;
                }
            }
            if (!cancelled) {
                if (is_tile) {
                    tile_publish_timing_.record(elapsed);
                } else {
                    full_publish_timing_.record(elapsed);
                }
            }
            busy_ = false;
            if (!retained_retry) {
                inflight_points_ = 0;
                inflight_bytes_ = 0;
                has_inflight_tile_ = false;
                inflight_tile_version_ = 0;
            }
            refreshQueueStatsLocked();
        }
        idle_cv_.notify_all();
    }
    {
        std::lock_guard<std::mutex> lock(mutex_);
        busy_ = false;
        in_callback_ = false;
        has_retry_tile_ = false;
        retry_tile_ = TileTask{};
        has_inflight_tile_ = false;
        inflight_tile_version_ = 0;
        worker_exited_ = true;
    }
    idle_cv_.notify_all();
    exit_cv_.notify_all();
}

void FoxgloveOutputWorker::refreshQueueStatsLocked()
{
    size_t points = has_full_ ? full_.cloud.size() : 0;
    size_t bytes = has_full_
        ? saturatedMultiplySize(full_.cloud.size(), sizeof(PointType))
        : 0;
    for (const auto& tile : tiles_) {
        points = saturatedAddSize(points, tile.second.update.points.size());
        bytes = saturatedAddSize(bytes, tile.second.update.estimatedBytes());
    }
    stats_.current_tasks = saturatedAddSize(
        tiles_.size(), has_full_ ? 1U : 0U);
    stats_.current_points = points;
    stats_.current_bytes = bytes;
    stats_.inflight_points = inflight_points_;
    stats_.inflight_bytes = inflight_bytes_;
    stats_.max_bytes = std::max(
        stats_.max_bytes, saturatedAddSize(bytes, inflight_bytes_));
}
