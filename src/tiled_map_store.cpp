#include "tiled_map_store.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <iostream>
#include <iterator>
#include <stdexcept>

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

void adjustEstimate(size_t& estimate, size_t before, size_t after) noexcept
{
    if (after >= before) {
        estimate = saturatedAddSize(estimate, after - before);
        return;
    }
    const size_t released = before - after;
    estimate = released > estimate ? 0 : estimate - released;
}

void subtractEstimate(size_t& estimate, size_t released) noexcept
{
    estimate = released > estimate ? 0 : estimate - released;
}

void hashMix(uint64_t& hash, uint64_t value)
{
    hash ^= value;
    hash *= 1099511628211ULL;
}

int64_t floorIndex(double coordinate, double size)
{
    const double value = std::floor(coordinate / size);
    if (!std::isfinite(value) ||
        value < static_cast<double>(std::numeric_limits<int32_t>::min()) ||
        value > static_cast<double>(std::numeric_limits<int32_t>::max()))
    {
        throw std::overflow_error("map coordinate exceeds integer index range");
    }
    return static_cast<int64_t>(value);
}

int32_t checkedInt32(int64_t value)
{
    if (value < std::numeric_limits<int32_t>::min() ||
        value > std::numeric_limits<int32_t>::max())
    {
        throw std::overflow_error("map coordinate exceeds tiled-map index range");
    }
    return static_cast<int32_t>(value);
}

uint64_t checkedNextVersion(uint64_t value)
{
    if (value == std::numeric_limits<uint64_t>::max()) {
        throw std::overflow_error("tiled-map version counter exhausted");
    }
    return value + 1;
}

}  // namespace

size_t TileKeyHash::operator()(const TileKey& key) const noexcept
{
    uint64_t hash = 1469598103934665603ULL;
    hashMix(hash, static_cast<uint64_t>(static_cast<int64_t>(key.x)));
    hashMix(hash, static_cast<uint64_t>(static_cast<int64_t>(key.y)));
    hashMix(hash, static_cast<uint64_t>(static_cast<int64_t>(key.z)));
    return static_cast<size_t>(hash);
}

size_t TileUpdate::estimatedBytes() const
{
    return estimatedBytesForPointCount(points.size());
}

size_t TileUpdate::estimatedBytesForPointCount(size_t point_count)
{
    // SceneEntity voxel primitives carry pose, size and color in addition to
    // the source point. Keep queue admission conservative with respect to the
    // serialized representation rather than counting only the PCL payload.
    const size_t payload_limit =
        std::numeric_limits<size_t>::max() - sizeof(TileUpdate);
    if (point_count > payload_limit / kEstimatedBytesPerPoint) {
        return std::numeric_limits<size_t>::max();
    }
    return sizeof(TileUpdate) + point_count * kEstimatedBytesPerPoint;
}

size_t TileUpdate::maxPointCountForBytes(size_t max_bytes)
{
    if (max_bytes == 0) return std::numeric_limits<size_t>::max();
    if (max_bytes <= sizeof(TileUpdate)) return 0;
    return (max_bytes - sizeof(TileUpdate)) / kEstimatedBytesPerPoint;
}

size_t TiledMapStore::VoxelKeyHash::operator()(const VoxelKey& key) const
{
    uint64_t hash = 1469598103934665603ULL;
    hashMix(hash, static_cast<uint64_t>(static_cast<int64_t>(key.x)));
    hashMix(hash, static_cast<uint64_t>(static_cast<int64_t>(key.y)));
    hashMix(hash, static_cast<uint64_t>(static_cast<int64_t>(key.z)));
    return static_cast<size_t>(hash);
}

TiledMapStore::TiledMapStore(Config config) : config_(std::move(config))
{
    if (!std::isfinite(config_.tile_size_m) ||
        !std::isfinite(config_.voxel_leaf_m) ||
        !(config_.tile_size_m > 0.0) || !(config_.voxel_leaf_m > 0.0)) {
        throw std::invalid_argument("tile and voxel sizes must be finite and positive");
    }
    if (config_.tile_size_m < config_.voxel_leaf_m) {
        throw std::invalid_argument("tile size must be at least one voxel");
    }
    const double voxels_per_tile =
        config_.tile_size_m / config_.voxel_leaf_m;
    if (!std::isfinite(voxels_per_tile) ||
        voxels_per_tile >
            static_cast<double>(std::numeric_limits<int32_t>::max()) + 1.0)
    {
        throw std::invalid_argument(
            "tile-to-voxel ratio exceeds tiled-map index range");
    }
    const double rounded_voxels_per_tile = std::round(voxels_per_tile);
    if (std::abs(voxels_per_tile - rounded_voxels_per_tile) >
        1e-9 * std::max(1.0, std::abs(voxels_per_tile)))
    {
        throw std::invalid_argument(
            "tile size must be an integer multiple of voxel size");
    }
    if (config_.max_pending_deletions == 0) {
        config_.max_pending_deletions = kDefaultMaxPendingDeletions;
    }
    // Reserve the bounded tombstone index up front. Restoring a rejected
    // deletion batch can then reinsert nodes without an unplanned bucket
    // rehash that would temporarily invalidate the map memory contract.
    if (config_.max_pending_deletions > 0) {
        pending_deletions_.reserve(config_.max_pending_deletions);
    }
    stats_.estimated_bytes = sizeof(*this);
    stats_.estimated_bytes = saturatedAddSize(
        stats_.estimated_bytes, estimateTileContainerBytes());
    stats_.estimated_bytes = saturatedAddSize(
        stats_.estimated_bytes, estimatePendingDeletionContainerBytes());
    if (config_.max_memory_bytes > 0 &&
        stats_.estimated_bytes > config_.max_memory_bytes)
    {
        throw std::invalid_argument(
            "tiled-map memory limit is smaller than its bounded metadata");
    }
}

TileKey TiledMapStore::tileKeyForPoint(const PointType& point) const
{
    return TileKey{
        checkedInt32(floorIndex(point.x, config_.tile_size_m)),
        checkedInt32(floorIndex(point.y, config_.tile_size_m)),
        checkedInt32(floorIndex(point.z, config_.tile_size_m)),
    };
}

TiledMapStore::VoxelKey TiledMapStore::voxelKeyForPoint(
    const PointType& point, const TileKey& tile) const
{
    const double origin_x = static_cast<double>(tile.x) * config_.tile_size_m;
    const double origin_y = static_cast<double>(tile.y) * config_.tile_size_m;
    const double origin_z = static_cast<double>(tile.z) * config_.tile_size_m;
    return VoxelKey{
        checkedInt32(floorIndex(static_cast<double>(point.x) - origin_x,
                                config_.voxel_leaf_m)),
        checkedInt32(floorIndex(static_cast<double>(point.y) - origin_y,
                                config_.voxel_leaf_m)),
        checkedInt32(floorIndex(static_cast<double>(point.z) - origin_z,
                                config_.voxel_leaf_m)),
    };
}

std::vector<TileKey> TiledMapStore::addFrame(
    const PointCloudXYZI& frame, double timestamp, const PointType* current_position)
{
    // Reserve before mutating the store. At most one key is recorded per
    // input point, so every later push is allocation-free and a failed
    // allocation cannot leave committed voxels invisible to the dirty list.
    std::vector<TileKey> changed;
    changed.reserve(frame.size());
    ++stats_.frames;
    stats_.points_observed += static_cast<uint64_t>(frame.size());
    auto finalize_changed = [&]() {
        std::sort(changed.begin(), changed.end(),
                  [](const TileKey& left, const TileKey& right) {
                      if (left.x != right.x) return left.x < right.x;
                      if (left.y != right.y) return left.y < right.y;
                      return left.z < right.z;
                  });
        changed.erase(std::unique(changed.begin(), changed.end()), changed.end());
        for (const TileKey& key : changed) {
            auto found = tiles_.find(key);
            if (found != tiles_.end()) {
                if (found->second.version == 0) {
                    if (recreated_version_floor_ == 0) {
                        found->second.version = 1;
                    } else {
                        recreated_version_floor_ =
                            checkedNextVersion(recreated_version_floor_);
                        found->second.version = recreated_version_floor_;
                    }
                } else {
                    found->second.version =
                        checkedNextVersion(found->second.version);
                }
                markDirty(key);
            }
        }
    };

    try {
        for (const auto& point : frame.points) {
            if (!std::isfinite(static_cast<double>(point.x)) ||
                !std::isfinite(static_cast<double>(point.y)) ||
                !std::isfinite(static_cast<double>(point.z)))
            {
                ++stats_.points_rejected_invalid;
                stats_.incomplete = true;
                continue;
            }

            const TileKey tile_key = tileKeyForPoint(point);
            const VoxelKey voxel_key = voxelKeyForPoint(point, tile_key);
            const auto existing_tile = tiles_.find(tile_key);
            const bool existing_voxel =
                existing_tile != tiles_.end() &&
                existing_tile->second.voxels.find(voxel_key) !=
                    existing_tile->second.voxels.end();

            // stop_accumulating is a growth barrier, not a freeze:
            // observations of existing voxels can still refresh the map.
            if (stats_.accumulation_stopped && !existing_voxel) {
                ++stats_.points_rejected_memory;
                stats_.incomplete = true;
                continue;
            }

            size_t bytes_before = existing_tile == tiles_.end()
                ? 0
                : estimateTileBytes(existing_tile->second);
            const size_t tile_container_before = estimateTileContainerBytes();
            const size_t tile_bucket_count_before = tiles_.bucket_count();
            TileMap::iterator tile_iterator;
            bool tile_inserted = false;
            try {
                auto tile_result = tiles_.try_emplace(tile_key);
                tile_iterator = tile_result.first;
                tile_inserted = tile_result.second;
            } catch (...) {
                adjustEstimate(stats_.estimated_bytes, tile_container_before,
                               estimateTileContainerBytes());
                throw;
            }
            adjustEstimate(stats_.estimated_bytes, tile_container_before,
                           estimateTileContainerBytes());
            Tile& tile = tile_iterator->second;
            if (tile_inserted) {
                // Account for the empty Tile immediately. A later voxel-node
                // allocation failure must never leave an uncounted Tile node.
                bytes_before = estimateTileBytes(tile);
                adjustEstimate(stats_.estimated_bytes, 0, bytes_before);
            }
            tile.last_access_sequence = ++access_sequence_;
            if (!existing_voxel && config_.max_voxels_per_tile > 0 &&
                tile.voxels.size() >= config_.max_voxels_per_tile)
            {
                ++stats_.points_rejected_tile_capacity;
                stats_.incomplete = true;
                continue;
            }

            using VoxelMap = decltype(tile.voxels);
            std::pair<VoxelMap::iterator, bool> voxel_result;
            try {
                voxel_result = tile.voxels.try_emplace(voxel_key);
            } catch (...) {
                const size_t bytes_after_failure = estimateTileBytes(tile);
                adjustEstimate(stats_.estimated_bytes, bytes_before,
                               bytes_after_failure);
                if (tile_inserted) {
                    const size_t container_before_erase =
                        estimateTileContainerBytes();
                    tiles_.erase(tile_iterator);
                    subtractEstimate(stats_.estimated_bytes, bytes_after_failure);
                    adjustEstimate(stats_.estimated_bytes, container_before_erase,
                                   estimateTileContainerBytes());
                    // try_emplace(tile_key) may have grown the outer bucket
                    // array before the inner voxel allocation failed. Restore
                    // that metadata as part of the same transaction whenever
                    // retaining it would violate the configured hard limit.
                    if (config_.max_memory_bytes > 0 &&
                        stats_.estimated_bytes > config_.max_memory_bytes &&
                        tiles_.bucket_count() != tile_bucket_count_before)
                    {
                        const size_t compact_before =
                            estimateTileContainerBytes();
                        TileMap compacted;
                        compacted.rehash(tile_bucket_count_before);
                        while (!tiles_.empty()) {
                            auto node = tiles_.extract(tiles_.begin());
                            compacted.insert(std::move(node));
                        }
                        tiles_.swap(compacted);
                        adjustEstimate(stats_.estimated_bytes, compact_before,
                                       estimateTileContainerBytes());
                    }
                }
                throw;
            }

            VoxelCell& cell = voxel_result.first->second;
            bool voxel_changed = false;
            if (voxel_result.second) {
                cell.representative = point;
                cell.sum_x = point.x;
                cell.sum_y = point.y;
                cell.sum_z = point.z;
                cell.sum_intensity = point.intensity;
                cell.observation_count = 1;
                ++stats_.voxels_inserted;
                ++stats_.voxel_count;
                voxel_changed = true;
            } else {
                ++cell.observation_count;
                cell.sum_x += point.x;
                cell.sum_y += point.y;
                cell.sum_z += point.z;
                cell.sum_intensity += point.intensity;
                if (config_.update_policy == VoxelUpdatePolicy::Latest) {
                    cell.representative = point;
                    voxel_changed = true;
                } else if (config_.update_policy == VoxelUpdatePolicy::Centroid) {
                    const double count =
                        static_cast<double>(cell.observation_count);
                    cell.representative.x =
                        static_cast<float>(cell.sum_x / count);
                    cell.representative.y =
                        static_cast<float>(cell.sum_y / count);
                    cell.representative.z =
                        static_cast<float>(cell.sum_z / count);
                    cell.representative.intensity =
                        static_cast<float>(cell.sum_intensity / count);
                    voxel_changed = true;
                }
                if (voxel_changed) ++stats_.voxels_updated;
            }

            if (voxel_changed) {
                tile.last_update_timestamp = timestamp;
                changed.push_back(tile_key);
            }

            if (voxel_result.second) {
                const size_t bytes_after = estimateTileBytes(tile);
                adjustEstimate(stats_.estimated_bytes, bytes_before, bytes_after);
                auto rollback_inserted_voxel = [&]() {
                    auto rollback_tile = tiles_.find(tile_key);
                    if (rollback_tile == tiles_.end() ||
                        rollback_tile->second.voxels.find(voxel_key) ==
                            rollback_tile->second.voxels.end()) {
                        return;
                    }
                    const size_t rollback_before =
                        estimateTileBytes(rollback_tile->second);
                    rollback_tile->second.voxels.erase(voxel_key);
                    --stats_.voxels_inserted;
                    --stats_.voxel_count;
                    if (rollback_tile->second.voxels.empty()) {
                        const size_t container_before =
                            estimateTileContainerBytes();
                        clearDirty(rollback_tile->first);
                        tiles_.erase(rollback_tile);
                        subtractEstimate(stats_.estimated_bytes,
                                         rollback_before);
                        adjustEstimate(stats_.estimated_bytes,
                                       container_before,
                                       estimateTileContainerBytes());
                        if (config_.max_memory_bytes > 0 &&
                            stats_.estimated_bytes > config_.max_memory_bytes)
                        {
                            const size_t compact_before =
                                estimateTileContainerBytes();
                            TileMap compacted;
                            compacted.rehash(tile_bucket_count_before);
                            while (!tiles_.empty()) {
                                auto node = tiles_.extract(tiles_.begin());
                                compacted.insert(std::move(node));
                            }
                            tiles_.swap(compacted);
                            adjustEstimate(stats_.estimated_bytes,
                                           compact_before,
                                           estimateTileContainerBytes());
                        }
                    } else {
                        try {
                            rollback_tile->second.voxels.rehash(0);
                        } catch (...) {
                            adjustEstimate(
                                stats_.estimated_bytes, rollback_before,
                                estimateTileBytes(rollback_tile->second));
                            throw;
                        }
                        adjustEstimate(
                            stats_.estimated_bytes, rollback_before,
                            estimateTileBytes(rollback_tile->second));
                    }
                    if (voxel_changed && !changed.empty()) changed.pop_back();
                    ++stats_.points_rejected_memory;
                    stats_.incomplete = true;
                };

                try {
                    enforceMemoryLimit(current_position);
                } catch (...) {
                    // A failed eviction/tombstone allocation must turn into a
                    // growth fuse, not leave later frames free to grow beyond
                    // the configured cap. Roll back this insertion first.
                    stats_.accumulation_stopped = true;
                    stats_.incomplete = true;
                    rollback_inserted_voxel();
                    throw;
                }

                // stop_accumulating is a hard boundary: roll back the voxel
                // that crossed it instead of returning above the hard cap.
                if (stats_.accumulation_stopped &&
                    config_.max_memory_bytes > 0 &&
                    stats_.estimated_bytes > config_.max_memory_bytes)
                {
                    rollback_inserted_voxel();
                }
            }
        }

    } catch (...) {
        // All key storage was reserved up front; this cannot allocate. Ensure
        // every earlier committed change remains publishable before exposing
        // the failure to the worker task boundary.
        finalize_changed();
        stats_.tile_count = tiles_.size();
        stats_.dirty_count = pendingDirtyCount();
        stats_.pending_deletion_count = pending_deletions_.size();
        throw;
    }

    finalize_changed();

    stats_.tile_count = tiles_.size();
    stats_.dirty_count = pendingDirtyCount();
    stats_.pending_deletion_count = pending_deletions_.size();
    return changed;
}

TileUpdate TiledMapStore::makeUpdate(const TileKey& key, const Tile& tile) const
{
    TileUpdate update;
    update.key = key;
    update.version = tile.version;
    update.points.reserve(tile.voxels.size());
    for (const auto& voxel : tile.voxels) {
        update.points.push_back(voxel.second.representative);
    }
    update.points.width = static_cast<uint32_t>(update.points.size());
    update.points.height = 1;
    update.points.is_dense = true;
    return update;
}

std::vector<TileUpdate> TiledMapStore::consumeDirtyTiles(
    size_t max_tiles, size_t max_points, size_t max_bytes)
{
    std::vector<TileUpdate> updates;
    std::vector<TileKey> consumed_deletions;
    std::vector<TileKey> consumed_dirty_tiles;
    size_t points = 0;
    size_t bytes = 0;
    auto fits = [&](size_t point_count, size_t estimated_bytes,
                    size_t& projected_points, size_t& projected_bytes) {
        if (max_tiles > 0 && updates.size() >= max_tiles) return false;
        if (!checkedAddSize(points, point_count, projected_points) ||
            !checkedAddSize(bytes, estimated_bytes, projected_bytes))
        {
            return false;
        }
        return (max_points == 0 || projected_points <= max_points) &&
               (max_bytes == 0 || projected_bytes <= max_bytes);
    };

    // Select and materialize the complete batch before mutating either dirty
    // container. If allocating a later TileUpdate throws, all earlier dirty
    // entries remain available for a subsequent retry.
    for (auto it = pending_deletions_.begin();
         it != pending_deletions_.end(); ++it) {
        if (max_tiles > 0 && updates.size() >= max_tiles) break;
        const TileUpdate& update = it->second;
        if (update.resync) continue;  // Awaiting handoff acknowledgement.
        size_t projected_points = 0;
        size_t projected_bytes = 0;
        if (!fits(update.points.size(), update.estimatedBytes(),
                  projected_points, projected_bytes)) continue;
        updates.push_back(update);
        consumed_deletions.push_back(it->first);
        points = projected_points;
        bytes = projected_bytes;
    }

    TileKey dirty_key = dirty_head_;
    bool has_dirty_key = has_dirty_head_;
    size_t dirty_visited = 0;
    while (has_dirty_key && dirty_visited < dirty_count_) {
        const auto tile = tiles_.find(dirty_key);
        if (tile == tiles_.end()) {
            throw std::logic_error("dirty tiled-map link references a missing tile");
        }
        const bool has_next = tile->second.has_dirty_next;
        const TileKey next_key = tile->second.dirty_next;
        ++dirty_visited;
        if (max_tiles > 0 && updates.size() >= max_tiles) {
            break;
        }
        // Check bounded admission from retained metadata before copying the
        // tile's full point payload. Once the batch has some work, stop at the
        // first item that does not fit so each pass is O(batch), not O(backlog).
        const size_t point_count = tile->second.voxels.size();
        const size_t estimated_bytes =
            TileUpdate::estimatedBytesForPointCount(point_count);
        size_t projected_points = 0;
        size_t projected_bytes = 0;
        if (!fits(point_count, estimated_bytes,
                  projected_points, projected_bytes)) {
            break;
        }
        TileUpdate update = makeUpdate(tile->first, tile->second);
        update.resync = tile->second.resync_dirty;
        updates.push_back(std::move(update));
        consumed_dirty_tiles.push_back(tile->first);
        points = projected_points;
        bytes = projected_bytes;
        dirty_key = next_key;
        has_dirty_key = has_next;
    }

    for (const TileKey& key : consumed_deletions) {
        auto deletion = pending_deletions_.find(key);
        if (deletion != pending_deletions_.end() &&
            !deletion->second.resync) {
            deletion->second.resync = true;
            ++inflight_deletion_count_;
        }
    }
    for (const TileKey& key : consumed_dirty_tiles) {
        auto tile = tiles_.find(key);
        if (tile != tiles_.end()) tile->second.resync_dirty = false;
        clearDirty(key);
    }
    const bool below_memory_limit = config_.max_memory_bytes == 0 ||
        stats_.estimated_bytes <= config_.max_memory_bytes;
    if (deletion_backpressure_ && below_memory_limit &&
        pending_deletions_.size() < config_.max_pending_deletions)
    {
        deletion_backpressure_ = false;
        if (config_.memory_policy == MapMemoryPolicy::Lru ||
            config_.memory_policy == MapMemoryPolicy::EvictFar) {
            stats_.accumulation_stopped = false;
        }
    }

    stats_.dirty_count = pendingDirtyCount();
    stats_.pending_deletion_count = pending_deletions_.size();
    return updates;
}

PointCloudXYZI TiledMapStore::snapshot(double voxel_leaf_m) const
{
    PointCloudXYZI cloud;
    cloud.reserve(stats_.voxel_count);
    const bool filter_snapshot = voxel_leaf_m > config_.voxel_leaf_m;
    std::unordered_set<VoxelKey, VoxelKeyHash> snapshot_voxels;
    if (filter_snapshot) snapshot_voxels.reserve(stats_.voxel_count);
    for (const auto& tile : tiles_) {
        for (const auto& voxel : tile.second.voxels) {
            const PointType& point = voxel.second.representative;
            if (filter_snapshot) {
                const VoxelKey key{
                    checkedInt32(floorIndex(point.x, voxel_leaf_m)),
                    checkedInt32(floorIndex(point.y, voxel_leaf_m)),
                    checkedInt32(floorIndex(point.z, voxel_leaf_m)),
                };
                if (!snapshot_voxels.insert(key).second) continue;
            }
            cloud.push_back(point);
        }
    }
    cloud.width = static_cast<uint32_t>(cloud.size());
    cloud.height = 1;
    cloud.is_dense = true;
    return cloud;
}

std::vector<TileUpdate> TiledMapStore::snapshotAllTiles() const
{
    std::vector<TileUpdate> result;
    result.reserve(tiles_.size());
    for (const auto& tile : tiles_) {
        result.push_back(makeUpdate(tile.first, tile.second));
    }
    return result;
}

void TiledMapStore::markAllDirty()
{
    // Reuse the intrusive dirty list. This is one O(N) pass per explicit
    // reconnect, followed by O(batch) drains with no generation rescans.
    for (auto& tile : tiles_) {
        markDirty(tile.first);
        tile.second.resync_dirty = true;
    }
    stats_.dirty_count = pendingDirtyCount();
}

void TiledMapStore::restoreDirtyTiles(const std::vector<TileUpdate>& updates)
{
    for (const auto& update : updates) {
        if (update.deleted) {
            auto pending = pending_deletions_.find(update.key);
            if (pending != pending_deletions_.end() &&
                pending->second.version == update.version &&
                pending->second.resync)
            {
                pending->second.resync = false;
                if (inflight_deletion_count_ > 0) {
                    --inflight_deletion_count_;
                }
                continue;
            }
            TileUpdate deletion;
            deletion.key = update.key;
            deletion.version = update.version;
            deletion.deleted = true;
            queueDeletion(std::move(deletion));
            continue;
        }
        const auto tile = tiles_.find(update.key);
        if (tile != tiles_.end() && tile->second.version >= update.version) {
            markDirty(update.key);
            if (update.resync) tile->second.resync_dirty = true;
        }
    }
    stats_.dirty_count = pendingDirtyCount();
    stats_.pending_deletion_count = pending_deletions_.size();
}

void TiledMapStore::confirmTilesHandedOff(
    const std::vector<TileUpdate>& updates)
{
    const size_t deletion_bytes_before =
        estimatePendingDeletionContainerBytes();
    for (const auto& update : updates) {
        if (update.deleted) {
            auto pending = pending_deletions_.find(update.key);
            if (pending != pending_deletions_.end() &&
                pending->second.version == update.version &&
                pending->second.resync)
            {
                pending_deletions_.erase(pending);
                if (inflight_deletion_count_ > 0) {
                    --inflight_deletion_count_;
                }
            }
            continue;
        }
        const auto tile = tiles_.find(update.key);
        if (tile != tiles_.end() && tile->second.version >= update.version) {
            tile->second.ever_handed_off = true;
        }
    }
    adjustEstimate(stats_.estimated_bytes, deletion_bytes_before,
                   estimatePendingDeletionContainerBytes());
    const bool below_memory_limit = config_.max_memory_bytes == 0 ||
        stats_.estimated_bytes <= config_.max_memory_bytes;
    if (deletion_backpressure_ && below_memory_limit &&
        pending_deletions_.size() < config_.max_pending_deletions)
    {
        deletion_backpressure_ = false;
        if (config_.memory_policy == MapMemoryPolicy::Lru ||
            config_.memory_policy == MapMemoryPolicy::EvictFar) {
            stats_.accumulation_stopped = false;
        }
    }
    stats_.dirty_count = pendingDirtyCount();
    stats_.pending_deletion_count = pending_deletions_.size();
}

void TiledMapStore::discardDirtyTiles()
{
    clearAllDirty();
    const size_t deletion_bytes_before =
        estimatePendingDeletionContainerBytes();
    pending_deletions_.clear();
    inflight_deletion_count_ = 0;
    const size_t deletion_bytes_after =
        estimatePendingDeletionContainerBytes();
    adjustEstimate(stats_.estimated_bytes, deletion_bytes_before,
                   deletion_bytes_after);
    stats_.dirty_count = 0;
    stats_.pending_deletion_count = 0;
    deletion_backpressure_ = false;
    if (config_.memory_policy == MapMemoryPolicy::Lru ||
        config_.memory_policy == MapMemoryPolicy::EvictFar)
    {
        stats_.accumulation_stopped = config_.max_memory_bytes > 0 &&
            stats_.estimated_bytes > config_.max_memory_bytes;
    }
}

void TiledMapStore::clear()
{
    clearAllDirty();
    TileMap empty_tiles;
    tiles_.swap(empty_tiles);
    pending_deletions_.clear();
    inflight_deletion_count_ = 0;
    stats_ = Stats{};
    stats_.estimated_bytes = sizeof(*this);
    stats_.estimated_bytes = saturatedAddSize(
        stats_.estimated_bytes, estimateTileContainerBytes());
    stats_.estimated_bytes = saturatedAddSize(
        stats_.estimated_bytes, estimatePendingDeletionContainerBytes());
    dirty_head_ = TileKey{};
    dirty_tail_ = TileKey{};
    dirty_count_ = 0;
    has_dirty_head_ = false;
    access_sequence_ = 0;
    recreated_version_floor_ = 0;
    deletion_backpressure_ = false;
}

size_t TiledMapStore::estimateTileBytes(const Tile& tile) const
{
    size_t bytes = sizeof(Tile);
    bytes = saturatedAddSize(bytes, saturatedMultiplySize(
        tile.voxels.bucket_count(), 2 * sizeof(void*)));
    bytes = saturatedAddSize(bytes, saturatedMultiplySize(
        tile.voxels.size(),
        sizeof(VoxelKey) + sizeof(VoxelCell) + 2 * sizeof(void*)));
    return bytes;
}

size_t TiledMapStore::estimateTileContainerBytes() const
{
    size_t bytes = saturatedMultiplySize(
        tiles_.bucket_count(), 2 * sizeof(void*));
    return saturatedAddSize(bytes, saturatedMultiplySize(
        tiles_.size(), sizeof(TileKey) + 2 * sizeof(void*)));
}

size_t TiledMapStore::estimatePendingDeletionBytes(
    const TileUpdate& deletion) const
{
    return saturatedAddSize(
        deletion.estimatedBytes(), sizeof(TileKey) + 2 * sizeof(void*));
}

size_t TiledMapStore::estimatePendingDeletionContainerBytes() const
{
    size_t bytes = saturatedMultiplySize(
        pending_deletions_.bucket_count(), 2 * sizeof(void*));
    for (const auto& deletion : pending_deletions_) {
        bytes = saturatedAddSize(
            bytes, estimatePendingDeletionBytes(deletion.second));
    }
    return bytes;
}

size_t TiledMapStore::pendingDirtyCount() const
{
    const size_t ready_deletions = inflight_deletion_count_ >=
        pending_deletions_.size()
        ? 0
        : pending_deletions_.size() - inflight_deletion_count_;
    return saturatedAddSize(dirty_count_, ready_deletions);
}

void TiledMapStore::markDirty(const TileKey& key)
{
    auto tile = tiles_.find(key);
    if (tile == tiles_.end() || tile->second.dirty) return;

    Tile& entry = tile->second;
    entry.dirty = true;
    entry.has_dirty_previous = has_dirty_head_;
    entry.has_dirty_next = false;
    if (has_dirty_head_) {
        entry.dirty_previous = dirty_tail_;
        auto previous = tiles_.find(dirty_tail_);
        if (previous == tiles_.end() || !previous->second.dirty) {
            throw std::logic_error("dirty tiled-map tail references a missing tile");
        }
        previous->second.dirty_next = key;
        previous->second.has_dirty_next = true;
    } else {
        dirty_head_ = key;
        has_dirty_head_ = true;
    }
    dirty_tail_ = key;
    ++dirty_count_;
}

void TiledMapStore::clearDirty(const TileKey& key)
{
    auto tile = tiles_.find(key);
    if (tile == tiles_.end() || !tile->second.dirty) return;

    const bool has_previous = tile->second.has_dirty_previous;
    const bool has_next = tile->second.has_dirty_next;
    const TileKey previous_key = tile->second.dirty_previous;
    const TileKey next_key = tile->second.dirty_next;

    if (has_previous) {
        auto previous = tiles_.find(previous_key);
        if (previous != tiles_.end()) {
            previous->second.has_dirty_next = has_next;
            if (has_next) previous->second.dirty_next = next_key;
        }
    } else if (has_next) {
        dirty_head_ = next_key;
    } else {
        has_dirty_head_ = false;
    }

    if (has_next) {
        auto next = tiles_.find(next_key);
        if (next != tiles_.end()) {
            next->second.has_dirty_previous = has_previous;
            if (has_previous) next->second.dirty_previous = previous_key;
        }
    } else if (has_previous) {
        dirty_tail_ = previous_key;
    } else {
        dirty_tail_ = TileKey{};
    }

    tile->second.dirty = false;
    tile->second.has_dirty_previous = false;
    tile->second.has_dirty_next = false;
    tile->second.dirty_previous = TileKey{};
    tile->second.dirty_next = TileKey{};
    if (dirty_count_ > 0) --dirty_count_;
}

void TiledMapStore::clearAllDirty() noexcept
{
    TileKey key = dirty_head_;
    bool has_key = has_dirty_head_;
    size_t visited = 0;
    while (has_key && visited < dirty_count_) {
        auto tile = tiles_.find(key);
        if (tile == tiles_.end()) break;
        const bool has_next = tile->second.has_dirty_next;
        const TileKey next_key = tile->second.dirty_next;
        tile->second.dirty = false;
        tile->second.resync_dirty = false;
        tile->second.has_dirty_previous = false;
        tile->second.has_dirty_next = false;
        tile->second.dirty_previous = TileKey{};
        tile->second.dirty_next = TileKey{};
        key = next_key;
        has_key = has_next;
        ++visited;
    }
    dirty_head_ = TileKey{};
    dirty_tail_ = TileKey{};
    dirty_count_ = 0;
    has_dirty_head_ = false;
}

bool TiledMapStore::queueDeletion(
    TileUpdate deletion, size_t anticipated_release)
{
    // Tombstones never carry voxel payload. Normalizing here keeps retries
    // fixed-size even if an external caller supplies a populated deletion.
    deletion.points = PointCloudXYZI{};
    deletion.deleted = true;
    deletion.resync = false;
    auto existing = pending_deletions_.find(deletion.key);
    if (existing != pending_deletions_.end()) {
        if (deletion.version > existing->second.version) {
            if (existing->second.resync && inflight_deletion_count_ > 0) {
                --inflight_deletion_count_;
            }
            const size_t bytes_before =
                estimatePendingDeletionBytes(existing->second);
            existing->second = std::move(deletion);
            adjustEstimate(stats_.estimated_bytes, bytes_before,
                           estimatePendingDeletionBytes(existing->second));
        }
        recreated_version_floor_ = std::max(
            recreated_version_floor_, existing->second.version);
        ++stats_.deletions_coalesced;
        return true;
    }
    if (pending_deletions_.size() >= config_.max_pending_deletions)
    {
        ++stats_.deletions_backpressured;
        deletion_backpressure_ = true;
        stats_.accumulation_stopped = true;
        stats_.incomplete = true;
        return false;
    }
    const size_t bucket_bytes_before =
        saturatedMultiplySize(
            pending_deletions_.bucket_count(), 2 * sizeof(void*));
    const size_t deletion_bytes = estimatePendingDeletionBytes(deletion);
    const size_t retained_after_release = anticipated_release >=
        stats_.estimated_bytes
        ? 0
        : stats_.estimated_bytes - anticipated_release;
    const size_t projected_bytes = saturatedAddSize(
        retained_after_release, deletion_bytes);
    if (config_.max_memory_bytes > 0 &&
        projected_bytes > config_.max_memory_bytes)
    {
        ++stats_.deletions_backpressured;
        deletion_backpressure_ = true;
        stats_.accumulation_stopped = true;
        stats_.incomplete = true;
        return false;
    }
    const uint64_t deletion_version = deletion.version;
    pending_deletions_.emplace(deletion.key, std::move(deletion));
    const size_t bucket_bytes_after =
        saturatedMultiplySize(
            pending_deletions_.bucket_count(), 2 * sizeof(void*));
    const size_t bucket_growth = bucket_bytes_after >= bucket_bytes_before
        ? bucket_bytes_after - bucket_bytes_before
        : 0;
    stats_.estimated_bytes = saturatedAddSize(
        stats_.estimated_bytes, saturatedAddSize(
            deletion_bytes, bucket_growth));
    recreated_version_floor_ = std::max(
        recreated_version_floor_, deletion_version);
    stats_.pending_deletion_count = pending_deletions_.size();
    return true;
}

void TiledMapStore::enforceMemoryLimit(const PointType* current_position)
{
    if (config_.max_memory_bytes == 0) return;
    while (stats_.estimated_bytes > config_.max_memory_bytes) {
        if (config_.memory_policy == MapMemoryPolicy::StopAccumulating ||
            config_.memory_policy == MapMemoryPolicy::SpillToDisk)
        {
            stats_.accumulation_stopped = true;
            break;
        }
        if (tiles_.empty()) break;
        if (!evictOne(current_position)) break;
    }

    if (stats_.estimated_bytes > config_.max_memory_bytes) {
        // Rolling back a new Tile removes its node, but an outer-map growth
        // rehash can retain excess buckets even while older Tiles remain.
        // Compact that metadata before declaring the hard limit violated.
        const size_t before = estimateTileContainerBytes();
        // Propagate allocation failure: returning success with retained bytes
        // above a configured hard limit would be a false contract. The map
        // worker's task boundary records the frame as incomplete and survives.
        tiles_.rehash(0);
        adjustEstimate(stats_.estimated_bytes, before,
                       estimateTileContainerBytes());
    }

    if (stats_.estimated_bytes > config_.max_memory_bytes) {
        stats_.accumulation_stopped = true;
        stats_.incomplete = true;
        if (config_.memory_policy == MapMemoryPolicy::Lru ||
            config_.memory_policy == MapMemoryPolicy::EvictFar)
        {
            if (!deletion_backpressure_) ++stats_.deletions_backpressured;
            deletion_backpressure_ = true;
        }
    } else if (deletion_backpressure_ &&
               pending_deletions_.size() < config_.max_pending_deletions)
    {
        deletion_backpressure_ = false;
        if (config_.memory_policy == MapMemoryPolicy::Lru ||
            config_.memory_policy == MapMemoryPolicy::EvictFar) {
            stats_.accumulation_stopped = false;
        }
    }
}

bool TiledMapStore::evictOne(const PointType* current_position)
{
    if (tiles_.empty()) return false;
    const bool deletion_slots_full =
        pending_deletions_.size() >= config_.max_pending_deletions;
    auto eligible = [&](const TileMap::const_iterator& tile) {
        return !tile->second.ever_handed_off || !deletion_slots_full ||
            pending_deletions_.find(tile->first) != pending_deletions_.end();
    };
    auto candidate = tiles_.end();
    if (config_.memory_policy == MapMemoryPolicy::EvictFar && current_position) {
        double best_distance = -1.0;
        for (auto it = tiles_.begin(); it != tiles_.end(); ++it) {
            if (!eligible(it)) continue;
            const double cx = (static_cast<double>(it->first.x) + 0.5) * config_.tile_size_m;
            const double cy = (static_cast<double>(it->first.y) + 0.5) * config_.tile_size_m;
            const double cz = (static_cast<double>(it->first.z) + 0.5) * config_.tile_size_m;
            const double dx = cx - current_position->x;
            const double dy = cy - current_position->y;
            const double dz = cz - current_position->z;
            const double distance = dx * dx + dy * dy + dz * dz;
            if (candidate == tiles_.end() || distance > best_distance ||
                (distance == best_distance &&
                 it->second.last_access_sequence < candidate->second.last_access_sequence))
            {
                candidate = it;
                best_distance = distance;
            }
        }
    } else {
        for (auto it = tiles_.begin(); it != tiles_.end(); ++it) {
            if (!eligible(it)) continue;
            if (candidate == tiles_.end() ||
                it->second.last_access_sequence < candidate->second.last_access_sequence) {
                candidate = it;
            }
        }
    }

    if (candidate == tiles_.end()) {
        ++stats_.deletions_backpressured;
        deletion_backpressure_ = true;
        stats_.accumulation_stopped = true;
        stats_.incomplete = true;
        return false;
    }

    // Only a Tile whose update was accepted by the downstream output queue is
    // client-visible. Dirty/versioned but never handed-off Tiles can be
    // discarded without consuming the bounded tombstone index.
    if (candidate->second.ever_handed_off) {
        TileUpdate deletion;
        deletion.key = candidate->first;
        deletion.version = std::max(
            checkedNextVersion(candidate->second.version),
            checkedNextVersion(recreated_version_floor_));
        deletion.deleted = true;
        const size_t anticipated_release = saturatedAddSize(
            estimateTileBytes(candidate->second),
            sizeof(TileKey) + 2 * sizeof(void*));
        const uint64_t deletion_version = deletion.version;
        if (!queueDeletion(std::move(deletion), anticipated_release)) {
            return false;
        }
        recreated_version_floor_ = std::max(
            recreated_version_floor_, deletion_version);
    }
    clearDirty(candidate->first);
    const size_t removed_bytes = estimateTileBytes(candidate->second);
    stats_.voxel_count -= candidate->second.voxels.size();
    ++stats_.tiles_evicted;
    stats_.incomplete = true;
    const size_t tile_container_before = estimateTileContainerBytes();
    tiles_.erase(candidate);
    subtractEstimate(stats_.estimated_bytes, removed_bytes);
    adjustEstimate(stats_.estimated_bytes, tile_container_before,
                   estimateTileContainerBytes());
    return true;
}

TiledMapStore::Stats TiledMapStore::stats() const
{
    Stats result = stats_;
    result.tile_count = tiles_.size();
    result.dirty_count = pendingDirtyCount();
    result.pending_deletion_count = pending_deletions_.size();
    return result;
}

VoxelUpdatePolicy TiledMapStore::parseUpdatePolicy(const std::string& value)
{
    if (value == "first") return VoxelUpdatePolicy::First;
    if (value == "latest") return VoxelUpdatePolicy::Latest;
    if (value == "centroid") return VoxelUpdatePolicy::Centroid;
    throw std::invalid_argument("unknown tiled-map voxel update policy: " + value);
}

MapMemoryPolicy TiledMapStore::parseMemoryPolicy(const std::string& value)
{
    if (value == "lru") return MapMemoryPolicy::Lru;
    if (value == "evict_far") return MapMemoryPolicy::EvictFar;
    if (value == "stop_accumulating") return MapMemoryPolicy::StopAccumulating;
    if (value == "spill_to_disk") {
        std::cerr << "[TiledMapStore] spill_to_disk is not available yet; "
                     "using stop_accumulating." << std::endl;
        return MapMemoryPolicy::StopAccumulating;
    }
    throw std::invalid_argument("unknown tiled-map memory policy: " + value);
}
