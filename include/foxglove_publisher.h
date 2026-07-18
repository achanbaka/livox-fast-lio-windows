#ifndef FOXGLOVE_PUBLISHER_H
#define FOXGLOVE_PUBLISHER_H

#include "bounded_timing_stats.h"

#include <string>
#include <atomic>
#include <functional>
#include <mutex>
#include <shared_mutex>
#include <optional>
#include <vector>
#include "types.h"
#include "tiled_map_store.h"

constexpr const char* kFastLioMapFrame = "map";
constexpr const char* kFastLioBodyFrame = "base_link";
constexpr const char* kFastLioRegisteredCloudTopic = "/cloud_registered";
constexpr const char* kFastLioMapTopic = "/map";
constexpr const char* kFastLioMapDeltaTopic = "/map_delta";
constexpr const char* kFastLioMapTilesTopic = "/map_tiles";
constexpr const char* kFastLioImuTopic = "/imu";
constexpr const char* kFastLioImuFrame = "livox_imu";

class FoxglovePublisher
{
public:
    enum class PlaybackCommand {
        Play,
        Pause,
    };

    enum class PlaybackStatus {
        Playing,
        Paused,
        Buffering,
        Ended,
    };

    struct PlaybackControlRequest {
        PlaybackCommand command = PlaybackCommand::Play;
        float speed = 1.0f;
        std::optional<uint64_t> seek_time_ns;
    };

    struct PlaybackState {
        PlaybackStatus status = PlaybackStatus::Paused;
        uint64_t current_time_ns = 0;
        float speed = 1.0f;
        bool did_seek = false;
    };

    struct MapTimingStats {
        TimingWindowSummary serialize;
        TimingWindowSummary send;
    };

    using PlaybackControlCallback =
        std::function<PlaybackState(const PlaybackControlRequest&)>;

    FoxglovePublisher();
    ~FoxglovePublisher() noexcept;

    bool start(const std::string& host = "127.0.0.1", uint16_t port = 8765,
               size_t message_backlog_size = 64);
    void setPlaybackControl(uint64_t start_time_ns, uint64_t end_time_ns,
                            PlaybackControlCallback callback);
    void stop();
    bool isRunning() const { return running_; }

    // Publish functions (thread-safe, can be called from any thread)
    void publishPointCloud(const PointCloudXYZI& cloud, double timestamp);
    void publishMap(const PointCloudXYZI& map, double timestamp);
    void publishMapDelta(const PointCloudXYZI& delta, double timestamp);
    void publishMapTile(const TileUpdate& tile, double voxel_size,
                        double timestamp);
    void publishImu(const ImuData& imu);
    void publishOdometry(const V3D& position, const Eigen::Quaterniond& orientation, double timestamp);
    void publishPath(const std::vector<V3D>& path);
    void publishPath(const std::vector<V3D>& path, double timestamp);
    void publishTransform(const V3D& translation, const Eigen::Quaterniond& rotation,
                          const std::string& parent_frame, const std::string& child_frame, double timestamp);
    void broadcastTime(double timestamp);
    void broadcastPlaybackState(const PlaybackState& state);
    void clearSession();

    uint16_t getPort() const { return port_; }
    size_t getClientCount() const;
    uint64_t getClientConnectionGeneration() const {
        return client_connection_generation_.load();
    }
    MapTimingStats mapTimingStats() const;

private:
    std::atomic<bool> running_{false};
    std::string host_;
    uint16_t port_;
    std::atomic<size_t> client_count_{0};
    std::atomic<uint64_t> client_connection_generation_{0};
    std::optional<std::pair<uint64_t, uint64_t>> playback_time_range_;
    PlaybackControlCallback playback_control_callback_;

    // Server pointer (opaque to avoid including Foxglove SDK headers here)
    void* server_impl_;
    mutable std::shared_mutex publish_mutex_;
    mutable std::mutex timing_mutex_;
    BoundedTimingWindow<> map_serialize_timing_;
    BoundedTimingWindow<> map_send_timing_;
};

#endif
