#ifndef TILED_MAP_STORE_H
#define TILED_MAP_STORE_H

#include "types.h"

#include <cstddef>
#include <cstdint>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

struct TileKey
{
    int32_t x = 0;
    int32_t y = 0;
    int32_t z = 0;

    bool operator==(const TileKey& other) const noexcept
    {
        return x == other.x && y == other.y && z == other.z;
    }

    bool operator!=(const TileKey& other) const noexcept { return !(*this == other); }
};

struct TileKeyHash
{
    size_t operator()(const TileKey& key) const noexcept;
};

enum class VoxelUpdatePolicy
{
    First,
    Latest,
    Centroid,
};

enum class MapMemoryPolicy
{
    StopAccumulating,
    Lru,
    EvictFar,
    SpillToDisk,
};

struct TileUpdate
{
    static constexpr size_t kEstimatedBytesPerPoint = 256;

    TileKey key;
    uint64_t version = 0;
    PointCloudXYZI points;
    bool deleted = false;
    // Internal delivery provenance. It is not serialized on the wire; it lets
    // a rejected bounded batch restore generation state without allocating a
    // per-tile dirty hash entry.
    bool resync = false;

    size_t estimatedBytes() const;
    static size_t estimatedBytesForPointCount(size_t point_count);
    static size_t maxPointCountForBytes(size_t max_bytes);
};

class TiledMapStore
{
public:
    static constexpr size_t kDefaultMaxPendingDeletions = 4096;

    struct Config {
        double tile_size_m = 20.0;
        double voxel_leaf_m = 0.2;
        VoxelUpdatePolicy update_policy = VoxelUpdatePolicy::Latest;
        size_t max_memory_bytes = 1024ULL * 1024ULL * 1024ULL;
        MapMemoryPolicy memory_policy = MapMemoryPolicy::StopAccumulating;
        // A non-zero value guarantees that one tile update can stay within
        // the downstream point/byte admission limits. Existing voxels remain
        // updateable after the tile reaches this capacity.
        size_t max_voxels_per_tile = 0;
        // Deletions are coalesced by TileKey. When this many distinct
        // deletions await delivery, eviction applies backpressure instead of
        // growing an unbounded tombstone queue. Zero is normalized to the
        // same bounded default rather than enabling an unbounded queue.
        size_t max_pending_deletions = kDefaultMaxPendingDeletions;
    };

    struct Stats {
        uint64_t frames = 0;
        uint64_t points_observed = 0;
        uint64_t voxels_inserted = 0;
        uint64_t voxels_updated = 0;
        uint64_t points_rejected_invalid = 0;
        uint64_t points_rejected_memory = 0;
        uint64_t points_rejected_tile_capacity = 0;
        uint64_t tiles_evicted = 0;
        uint64_t tiles_spilled = 0;
        uint64_t deletions_coalesced = 0;
        uint64_t deletions_backpressured = 0;
        size_t tile_count = 0;
        size_t dirty_count = 0;
        size_t voxel_count = 0;
        size_t pending_deletion_count = 0;
        size_t estimated_bytes = 0;
        bool accumulation_stopped = false;
        bool incomplete = false;
    };

    explicit TiledMapStore(Config config = Config{});

    std::vector<TileKey> addFrame(const PointCloudXYZI& frame,
                                  double timestamp,
                                  const PointType* current_position = nullptr);
    std::vector<TileUpdate> consumeDirtyTiles(size_t max_tiles,
                                               size_t max_points,
                                               size_t max_bytes);
    PointCloudXYZI snapshot(double voxel_leaf_m = 0.0) const;
    std::vector<TileUpdate> snapshotAllTiles() const;
    void markAllDirty();
    void restoreDirtyTiles(const std::vector<TileUpdate>& updates);
    void confirmTilesHandedOff(const std::vector<TileUpdate>& updates);
    void discardDirtyTiles();
    void clear();

    TileKey tileKeyForPoint(const PointType& point) const;
    Stats stats() const;
    const Config& config() const { return config_; }

    static VoxelUpdatePolicy parseUpdatePolicy(const std::string& value);
    static MapMemoryPolicy parseMemoryPolicy(const std::string& value);

private:
    struct VoxelKey {
        int32_t x = 0;
        int32_t y = 0;
        int32_t z = 0;

        bool operator==(const VoxelKey& other) const
        {
            return x == other.x && y == other.y && z == other.z;
        }
    };

    struct VoxelKeyHash {
        size_t operator()(const VoxelKey& key) const;
    };

    struct VoxelCell {
        PointType representative;
        double sum_x = 0.0;
        double sum_y = 0.0;
        double sum_z = 0.0;
        double sum_intensity = 0.0;
        uint32_t observation_count = 0;
    };

    struct Tile {
        std::unordered_map<VoxelKey, VoxelCell, VoxelKeyHash> voxels;
        uint64_t version = 0;
        double last_update_timestamp = 0.0;
        uint64_t last_access_sequence = 0;
        // Dirty membership is intrusive so marking an existing tile never
        // allocates a hash node (and therefore cannot exceed the map's hard
        // retained-memory limit after a voxel update has already committed).
        TileKey dirty_previous;
        TileKey dirty_next;
        bool dirty = false;
        bool resync_dirty = false;
        bool ever_handed_off = false;
        bool has_dirty_previous = false;
        bool has_dirty_next = false;
    };

    using TileMap = std::unordered_map<TileKey, Tile, TileKeyHash>;

    VoxelKey voxelKeyForPoint(const PointType& point, const TileKey& tile) const;
    TileUpdate makeUpdate(const TileKey& key, const Tile& tile) const;
    size_t estimateTileBytes(const Tile& tile) const;
    size_t estimateTileContainerBytes() const;
    size_t estimatePendingDeletionBytes(const TileUpdate& deletion) const;
    size_t estimatePendingDeletionContainerBytes() const;
    size_t pendingDirtyCount() const;
    void markDirty(const TileKey& key);
    void clearDirty(const TileKey& key);
    void clearAllDirty() noexcept;
    bool queueDeletion(TileUpdate deletion, size_t anticipated_release = 0);
    void enforceMemoryLimit(const PointType* current_position);
    bool evictOne(const PointType* current_position);

    Config config_;
    TileMap tiles_;
    std::unordered_map<TileKey, TileUpdate, TileKeyHash> pending_deletions_;
    size_t inflight_deletion_count_ = 0;
    Stats stats_;
    TileKey dirty_head_;
    TileKey dirty_tail_;
    size_t dirty_count_ = 0;
    bool has_dirty_head_ = false;
    uint64_t access_sequence_ = 0;
    uint64_t recreated_version_floor_ = 0;
    bool deletion_backpressure_ = false;
};

#endif
