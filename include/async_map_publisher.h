#ifndef ASYNC_MAP_PUBLISHER_H
#define ASYNC_MAP_PUBLISHER_H

#include "map_accumulator.h"

#include <condition_variable>
#include <cstdint>
#include <deque>
#include <functional>
#include <mutex>
#include <thread>

class AsyncMapPublisher
{
public:
    using PublishCallback = std::function<void(const PointCloudXYZI&, double)>;

    struct Stats {
        uint64_t requested = 0;
        uint64_t published = 0;
        uint64_t coalesced = 0;
        uint64_t failures = 0;
        size_t last_point_count = 0;
        double last_snapshot_ms = 0.0;
        double last_publish_ms = 0.0;
        uint64_t frames_enqueued = 0;
        uint64_t frames_built = 0;
        uint64_t frames_dropped = 0;
        size_t current_frame_queue_depth = 0;
        size_t max_frame_queue_depth = 0;
        size_t last_frame_point_count = 0;
        double last_frame_build_ms = 0.0;
        uint64_t delta_requested = 0;
        uint64_t delta_published = 0;
        uint64_t delta_coalesced = 0;
        uint64_t delta_dropped_points = 0;
        uint64_t delta_resync_requests = 0;
        size_t current_delta_points = 0;
        size_t last_delta_point_count = 0;
        double last_delta_publish_ms = 0.0;
        bool full_map_pending = false;
        bool busy = false;
    };

    AsyncMapPublisher(MapAccumulator& accumulator,
                      PublishCallback callback,
                      PublishCallback delta_callback = PublishCallback{},
                      size_t max_pending_delta_points = 200000,
                      size_t max_pending_frames = 64);
    ~AsyncMapPublisher();

    AsyncMapPublisher(const AsyncMapPublisher&) = delete;
    AsyncMapPublisher& operator=(const AsyncMapPublisher&) = delete;

    void start();
    void enqueueFrame(PointCloudXYZI frame, double timestamp);
    void request(double timestamp);
    void enqueueDelta(PointCloudXYZI delta, double timestamp);
    void flush();
    void stop();
    bool isRunning() const;
    Stats stats() const;

private:
    struct FrameTask {
        PointCloudXYZI cloud;
        double timestamp = 0.0;
        uint64_t sequence = 0;
    };

    void enqueueDeltaLocked(PointCloudXYZI delta, double timestamp);
    void run();

    MapAccumulator& accumulator_;
    PublishCallback callback_;
    PublishCallback delta_callback_;
    size_t max_pending_delta_points_ = 0;
    size_t max_pending_frames_ = 0;

    mutable std::mutex mutex_;
    std::condition_variable work_cv_;
    std::condition_variable idle_cv_;
    std::thread worker_;
    bool running_ = false;
    bool stopping_ = false;
    bool pending_ = false;
    bool busy_ = false;
    double pending_timestamp_ = 0.0;
    uint64_t pending_request_sequence_ = 0;
    uint64_t next_frame_sequence_ = 0;
    uint64_t last_built_sequence_ = 0;
    std::deque<FrameTask> pending_frames_;
    PointCloudXYZI pending_delta_;
    double pending_delta_timestamp_ = 0.0;
    Stats stats_;
};

#endif
