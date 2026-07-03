#ifndef MAP_ACCUMULATOR_H
#define MAP_ACCUMULATOR_H

#include "types.h"

#include <algorithm>
#include <mutex>

#include <pcl/filters/voxel_grid.h>

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
        cloud_.clear();
        frames_since_filter_ = 0;
    }

    void addFrame(const PointCloudXYZI& frame)
    {
        if (frame.empty()) {
            return;
        }

        std::lock_guard<std::mutex> lock(mutex_);
        cloud_ += frame;
        frames_since_filter_++;

        if (leaf_size_ > 0.0f && frames_since_filter_ >= filter_interval_) {
            downsampleLocked();
            frames_since_filter_ = 0;
        }
    }

    PointCloudXYZI snapshot()
    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (leaf_size_ > 0.0f && frames_since_filter_ > 0) {
            downsampleLocked();
            frames_since_filter_ = 0;
        }
        return cloud_;
    }

    size_t size() const
    {
        std::lock_guard<std::mutex> lock(mutex_);
        return cloud_.size();
    }

private:
    void downsampleLocked()
    {
        if (cloud_.empty()) {
            return;
        }

        PointCloudXYZI filtered;
        pcl::VoxelGrid<PointType> filter;
        filter.setLeafSize(leaf_size_, leaf_size_, leaf_size_);
        filter.setInputCloud(cloud_.makeShared());
        filter.filter(filtered);
        cloud_.swap(filtered);
    }

    mutable std::mutex mutex_;
    PointCloudXYZI cloud_;
    float leaf_size_ = 0.5f;
    int filter_interval_ = 10;
    int frames_since_filter_ = 0;
};

#endif
