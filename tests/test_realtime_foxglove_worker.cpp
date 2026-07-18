#include "realtime_foxglove_worker.h"

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <iostream>
#include <mutex>
#include <stdexcept>
#include <thread>

namespace {

#define CHECK(condition)                                                        \
    do {                                                                        \
        if (!(condition)) {                                                     \
            std::cerr << "CHECK failed at line " << __LINE__ << ": "          \
                      << #condition << std::endl;                               \
            return 1;                                                           \
        }                                                                       \
    } while (false)

PointCloudXYZI makeCloud(size_t count, float marker)
{
    PointCloudXYZI cloud;
    cloud.resize(count);
    for (auto& point : cloud) point.x = marker;
    return cloud;
}

int testLatestWinsAndControlPriority()
{
    std::mutex gate_mutex;
    std::condition_variable gate_cv;
    bool cloud_entered = false;
    bool release_cloud = false;
    std::mutex result_mutex;
    std::vector<std::string> order;
    float last_cloud_marker = 0.0f;
    double last_pose_x = 0.0;
    uint64_t last_playback_time = 0;
    double last_path_x = 0.0;
    size_t last_path_size = 0;

    RealtimeFoxgloveWorker::Callbacks callbacks;
    callbacks.cloud = [&](const PointCloudXYZI& cloud, double) {
        {
            std::unique_lock<std::mutex> lock(gate_mutex);
            if (!cloud_entered) {
                cloud_entered = true;
                gate_cv.notify_all();
                gate_cv.wait(lock, [&] { return release_cloud; });
            }
        }
        std::lock_guard<std::mutex> lock(result_mutex);
        order.push_back("cloud");
        last_cloud_marker = cloud.front().x;
    };
    callbacks.pose = [&](const V3D& position, const Eigen::Quaterniond&, double) {
        std::lock_guard<std::mutex> lock(result_mutex);
        order.push_back("pose");
        last_pose_x = position.x();
    };
    callbacks.playback = [&](const FoxglovePublisher::PlaybackState& state) {
        std::lock_guard<std::mutex> lock(result_mutex);
        order.push_back("playback");
        last_playback_time = state.current_time_ns;
    };
    callbacks.path = [&](const std::vector<V3D>& path, double) {
        std::lock_guard<std::mutex> lock(result_mutex);
        order.push_back("path");
        last_path_x = path.back().x();
        last_path_size = path.size();
    };
    callbacks.time = [](double) {};

    RealtimeFoxgloveWorker::Config config;
    config.max_pending_bytes = 4ULL * 1024ULL * 1024ULL;
    config.max_pending_points = 10000;
    config.max_path_points = 2;
    RealtimeFoxgloveWorker worker(std::move(callbacks), config);
    worker.start();

    CHECK(worker.tryEnqueueCloud(makeCloud(10, 1.0f), 1.0));
    {
        std::unique_lock<std::mutex> lock(gate_mutex);
        CHECK(gate_cv.wait_for(lock, std::chrono::seconds(2),
                               [&] { return cloud_entered; }));
    }

    CHECK(worker.tryEnqueueCloud(makeCloud(10, 2.0f), 2.0));
    CHECK(worker.tryEnqueueCloud(makeCloud(10, 3.0f), 3.0));
    CHECK(worker.tryEnqueuePose(V3D(1.0, 0.0, 0.0),
                                Eigen::Quaterniond::Identity(), 1.0));
    CHECK(worker.tryEnqueuePose(V3D(2.0, 0.0, 0.0),
                                Eigen::Quaterniond::Identity(), 2.0));
    CHECK(worker.tryEnqueuePlayback({
        FoxglovePublisher::PlaybackStatus::Playing, 10, 1.0f, false}));
    CHECK(worker.tryEnqueuePlayback({
        FoxglovePublisher::PlaybackStatus::Paused, 20, 1.0f, false}));
    CHECK(worker.tryEnqueuePathPoint(V3D(1.0, 0.0, 0.0), 1.0));
    CHECK(worker.tryEnqueuePathPoint(V3D(2.0, 0.0, 0.0), 2.0));

    {
        std::lock_guard<std::mutex> lock(gate_mutex);
        release_cloud = true;
    }
    gate_cv.notify_all();
    CHECK(worker.flushFor(std::chrono::seconds(2)));

    const auto stats = worker.stats();
    CHECK(stats.cloud.overwritten == 1);
    CHECK(stats.pose.overwritten == 1);
    CHECK(stats.playback.overwritten == 1);
    CHECK(stats.path.overwritten == 1);
    CHECK(stats.cloud.published == 2);
    CHECK(stats.failures == 0);
    CHECK(last_cloud_marker == 3.0f);
    CHECK(last_pose_x == 2.0);
    CHECK(last_playback_time == 20);
    CHECK(last_path_x == 2.0);
    CHECK(last_path_size == 2);
    {
        std::lock_guard<std::mutex> lock(result_mutex);
        CHECK(order.size() >= 5);
        // After the already in-flight cloud, playback and pose must run before
        // the queued visualization payloads.
        CHECK(order[1] == "playback");
        CHECK(order[2] == "pose");
    }
    worker.stop();
    return 0;
}

int testBoundAndExceptionIsolation()
{
    std::atomic<int> time_calls{0};
    RealtimeFoxgloveWorker::Callbacks callbacks;
    callbacks.cloud = [](const PointCloudXYZI&, double) {
        throw std::runtime_error("expected test failure");
    };
    callbacks.time = [&](double) { ++time_calls; };

    RealtimeFoxgloveWorker::Config config;
    config.max_pending_bytes = 1024;
    config.max_pending_points = 4;
    RealtimeFoxgloveWorker worker(std::move(callbacks), config);
    worker.start();

    CHECK(!worker.tryEnqueueCloud(makeCloud(100, 1.0f), 1.0));
    CHECK(worker.tryEnqueueCloud(makeCloud(1, 2.0f), 2.0));
    CHECK(worker.tryEnqueueTime(2.0));
    CHECK(worker.flushFor(std::chrono::seconds(2)));
    const auto stats = worker.stats();
    CHECK(stats.cloud.dropped == 1);
    CHECK(stats.cloud.failures == 1);
    CHECK(stats.failures == 1);
    CHECK(time_calls == 1);
    CHECK(stats.current_tasks == 0);
    CHECK(stats.current_bytes == 0);
    worker.stop();
    return 0;
}

int testBoundedStopCancelsPendingButNotInflightCallback()
{
    std::mutex gate_mutex;
    std::condition_variable gate_cv;
    bool callback_entered = false;
    bool release_callback = false;
    std::atomic<int> cloud_calls{0};
    std::atomic<int> time_calls{0};

    RealtimeFoxgloveWorker::Callbacks callbacks;
    callbacks.cloud = [&](const PointCloudXYZI&, double) {
        ++cloud_calls;
        std::unique_lock<std::mutex> lock(gate_mutex);
        callback_entered = true;
        gate_cv.notify_all();
        gate_cv.wait(lock, [&] { return release_callback; });
    };
    callbacks.time = [&](double) { ++time_calls; };

    RealtimeFoxgloveWorker worker(std::move(callbacks));
    worker.start();
    CHECK(worker.tryEnqueueCloud(makeCloud(1, 1.0f), 1.0));
    {
        std::unique_lock<std::mutex> lock(gate_mutex);
        CHECK(gate_cv.wait_for(lock, std::chrono::seconds(2),
                               [&] { return callback_entered; }));
    }

    CHECK(worker.tryEnqueueCloud(makeCloud(1, 2.0f), 2.0));
    CHECK(worker.tryEnqueueTime(2.0));
    const auto stop_begin = std::chrono::steady_clock::now();
    CHECK(!worker.stopFor(std::chrono::milliseconds(50), false));
    const auto stop_elapsed = std::chrono::steady_clock::now() - stop_begin;
    CHECK(stop_elapsed < std::chrono::milliseconds(500));

    const auto timed_out_stats = worker.stats();
    CHECK(timed_out_stats.stopping);
    CHECK(timed_out_stats.in_callback);
    CHECK(!timed_out_stats.worker_exited);
    CHECK(timed_out_stats.stop_timeouts == 1);
    CHECK(timed_out_stats.tasks_cancelled == 2);
    CHECK(timed_out_stats.current_tasks == 0);

    {
        std::lock_guard<std::mutex> lock(gate_mutex);
        release_callback = true;
    }
    gate_cv.notify_all();
    CHECK(worker.stopFor(std::chrono::seconds(2), false));
    const auto final_stats = worker.stats();
    CHECK(final_stats.worker_exited);
    CHECK(!final_stats.stopping);
    CHECK(cloud_calls == 1);
    CHECK(time_calls == 0);
    return 0;
}

}  // namespace

int main()
{
    if (testLatestWinsAndControlPriority()) return 1;
    if (testBoundAndExceptionIsolation()) return 1;
    if (testBoundedStopCancelsPendingButNotInflightCallback()) return 1;
    std::cout << "RealtimeFoxgloveWorker tests passed" << std::endl;
    return 0;
}
