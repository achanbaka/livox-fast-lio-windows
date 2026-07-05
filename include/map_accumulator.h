#ifndef MAP_ACCUMULATOR_H
#define MAP_ACCUMULATOR_H

#include "types.h"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <mutex>
#include <unordered_map>

class MapAccumulator
{
public:
    void setLeafSize(float leaf_size)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        leaf_size_ = std::max(leaf_size, 0.0f);
    }

    void clear()
    {
        std::lock_guard<std::mutex> lock(mutex_);
        raw_cloud_.clear();
        voxel_cloud_.clear();
    }

    void addFrame(const PointCloudXYZI& frame)
    {
        if (frame.empty()) {
            return;
        }

        std::lock_guard<std::mutex> lock(mutex_);
        if (leaf_size_ <= 0.0f) {
            raw_cloud_ += frame;
            return;
        }

        const double inv_leaf = 1.0 / static_cast<double>(leaf_size_);
        for (const auto& point : frame.points) {
            const VoxelKey key{
                static_cast<int64_t>(std::floor(static_cast<double>(point.x) * inv_leaf)),
                static_cast<int64_t>(std::floor(static_cast<double>(point.y) * inv_leaf)),
                static_cast<int64_t>(std::floor(static_cast<double>(point.z) * inv_leaf)),
            };
            voxel_cloud_.try_emplace(key, point);
        }
    }

    PointCloudXYZI snapshot()
    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (leaf_size_ <= 0.0f) {
            return raw_cloud_;
        }

        PointCloudXYZI cloud;
        cloud.points.reserve(voxel_cloud_.size());
        for (const auto& entry : voxel_cloud_) {
            cloud.points.push_back(entry.second);
        }
        cloud.width = static_cast<uint32_t>(cloud.points.size());
        cloud.height = 1;
        cloud.is_dense = true;
        return cloud;
    }

    size_t size() const
    {
        std::lock_guard<std::mutex> lock(mutex_);
        return leaf_size_ <= 0.0f ? raw_cloud_.size() : voxel_cloud_.size();
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

    mutable std::mutex mutex_;
    PointCloudXYZI raw_cloud_;
    std::unordered_map<VoxelKey, PointType, VoxelKeyHash> voxel_cloud_;
    float leaf_size_ = 0.5f;
};

#endif
