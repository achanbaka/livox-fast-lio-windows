#ifndef MAP_ACCUMULATOR_H
#define MAP_ACCUMULATOR_H

#include "types.h"

#include <algorithm>
#include <array>
#include <atomic>
#include <cmath>
#include <cstdint>
#include <mutex>
#include <unordered_map>
#include <utility>
#include <vector>

class MapAccumulator
{
public:
    void setLeafSize(float leaf_size)
    {
        leaf_size_.store(std::max(leaf_size, 0.0f), std::memory_order_release);
    }

    void clear()
    {
        {
            std::lock_guard<std::mutex> lock(raw_mutex_);
            raw_cloud_.clear();
        }
        for (auto& shard : shards_) {
            std::lock_guard<std::mutex> lock(shard.mutex);
            shard.voxels.clear();
        }
        voxel_size_.store(0, std::memory_order_release);
    }

    void addFrame(const PointCloudXYZI& frame, PointCloudXYZI* inserted_points = nullptr)
    {
        if (inserted_points) {
            inserted_points->clear();
        }
        if (frame.empty()) {
            return;
        }

        const float leaf_size = leaf_size_.load(std::memory_order_acquire);
        if (leaf_size <= 0.0f) {
            std::lock_guard<std::mutex> lock(raw_mutex_);
            raw_cloud_ += frame;
            if (inserted_points) {
                *inserted_points = frame;
            }
            return;
        }

        using PendingPoint = std::pair<VoxelKey, const PointType*>;
        std::array<std::vector<PendingPoint>, kShardCount> pending;
        const double inv_leaf = 1.0 / static_cast<double>(leaf_size);
        for (const auto& point : frame.points) {
            const VoxelKey key{
                static_cast<int64_t>(std::floor(static_cast<double>(point.x) * inv_leaf)),
                static_cast<int64_t>(std::floor(static_cast<double>(point.y) * inv_leaf)),
                static_cast<int64_t>(std::floor(static_cast<double>(point.z) * inv_leaf)),
            };
            pending[shardIndex(key)].emplace_back(key, &point);
        }

        for (size_t i = 0; i < kShardCount; ++i) {
            if (pending[i].empty()) {
                continue;
            }
            auto& shard = shards_[i];
            std::lock_guard<std::mutex> lock(shard.mutex);
            for (const auto& entry : pending[i]) {
                if (shard.voxels.try_emplace(entry.first, *entry.second).second) {
                    voxel_size_.fetch_add(1, std::memory_order_relaxed);
                    if (inserted_points) {
                        inserted_points->push_back(*entry.second);
                    }
                }
            }
        }

        if (inserted_points) {
            inserted_points->width = static_cast<uint32_t>(inserted_points->size());
            inserted_points->height = 1;
            inserted_points->is_dense = true;
        }
    }

    PointCloudXYZI snapshot() const
    {
        if (leaf_size_.load(std::memory_order_acquire) <= 0.0f) {
            std::lock_guard<std::mutex> lock(raw_mutex_);
            return raw_cloud_;
        }

        PointCloudXYZI cloud;
        cloud.points.reserve(voxel_size_.load(std::memory_order_acquire));
        for (const auto& shard : shards_) {
            std::lock_guard<std::mutex> lock(shard.mutex);
            for (const auto& entry : shard.voxels) {
                cloud.points.push_back(entry.second);
            }
        }
        cloud.width = static_cast<uint32_t>(cloud.points.size());
        cloud.height = 1;
        cloud.is_dense = true;
        return cloud;
    }

    size_t size() const
    {
        if (leaf_size_.load(std::memory_order_acquire) <= 0.0f) {
            std::lock_guard<std::mutex> lock(raw_mutex_);
            return raw_cloud_.size();
        }
        return voxel_size_.load(std::memory_order_acquire);
    }

private:
    struct VoxelKey {
        int64_t x = 0;
        int64_t y = 0;
        int64_t z = 0;

        bool operator==(const VoxelKey& other) const
        {
            return x == other.x && y == other.y && z == other.z;
        }
    };

    struct VoxelKeyHash {
        size_t operator()(const VoxelKey& key) const
        {
            uint64_t h = 1469598103934665603ULL;
            mix(h, static_cast<uint64_t>(key.x));
            mix(h, static_cast<uint64_t>(key.y));
            mix(h, static_cast<uint64_t>(key.z));
            return static_cast<size_t>(h);
        }

        static void mix(uint64_t& hash, uint64_t value)
        {
            hash ^= value;
            hash *= 1099511628211ULL;
        }
    };

    static constexpr size_t kShardCount = 64;

    struct Shard {
        mutable std::mutex mutex;
        std::unordered_map<VoxelKey, PointType, VoxelKeyHash> voxels;
    };

    static size_t shardIndex(const VoxelKey& key)
    {
        return VoxelKeyHash{}(key) % kShardCount;
    }

    mutable std::mutex raw_mutex_;
    PointCloudXYZI raw_cloud_;
    mutable std::array<Shard, kShardCount> shards_;
    std::atomic<size_t> voxel_size_{0};
    std::atomic<float> leaf_size_{0.5f};
};

#endif
