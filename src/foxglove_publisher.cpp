#include "foxglove_publisher.h"
#include "ros_message.h"

#include <foxglove/messages.hpp>
#include <foxglove/channel.hpp>
#include <foxglove/websocket.hpp>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <iostream>
#include <memory>
#include <optional>
#include <utility>

namespace fg = foxglove;
namespace fgm = foxglove::messages;

namespace {

struct FoxglovePublisherImpl {
    std::optional<fg::WebSocketServer> server;
    std::optional<fgm::PointCloudChannel> cloud;
    std::optional<fgm::PointCloudChannel> map;
    std::optional<fgm::PointCloudChannel> map_delta;
    std::optional<fg::RawChannel> imu;
    std::optional<fgm::OdometryChannel> odom;
    std::optional<fgm::PosesInFrameChannel> path;
    std::optional<fgm::FrameTransformsChannel> tf;
};

struct PackedPoint {
    float x;
    float y;
    float z;
    float intensity;
    uint8_t tag;
    uint8_t line;
    uint8_t pad0;
    uint8_t pad1;
};

static_assert(sizeof(PackedPoint) == 20, "Foxglove point stride must stay stable");

constexpr char kImuJsonSchema[] = R"json({
  "$schema": "https://json-schema.org/draft/2020-12/schema",
  "type": "object",
  "properties": {
    "timestamp": {
      "type": "object",
      "properties": {
        "sec": { "type": "integer" },
        "nsec": { "type": "integer" }
      },
      "required": ["sec", "nsec"]
    },
    "frame_id": { "type": "string" },
    "angular_velocity": { "$ref": "#/$defs/vector3" },
    "linear_acceleration": { "$ref": "#/$defs/vector3" }
  },
  "required": ["timestamp", "frame_id", "angular_velocity", "linear_acceleration"],
  "$defs": {
    "vector3": {
      "type": "object",
      "properties": {
        "x": { "type": "number" },
        "y": { "type": "number" },
        "z": { "type": "number" }
      },
      "required": ["x", "y", "z"]
    }
  }
})json";

fgm::Timestamp toFoxgloveTimestamp(double timestamp) {
    uint32_t sec = 0;
    uint32_t nsec = 0;
    doubleToRosTime(timestamp, sec, nsec);
    return fgm::Timestamp{sec, nsec};
}

fgm::Pose makePose(const V3D& position, const Eigen::Quaterniond& orientation) {
    fgm::Pose pose;
    pose.position = fgm::Vector3{position.x(), position.y(), position.z()};
    pose.orientation = fgm::Quaternion{
        orientation.x(), orientation.y(), orientation.z(), orientation.w()};
    return pose;
}

fg::PlaybackStatus toSdkPlaybackStatus(FoxglovePublisher::PlaybackStatus status) {
    switch (status) {
        case FoxglovePublisher::PlaybackStatus::Playing:
            return fg::PlaybackStatus::Playing;
        case FoxglovePublisher::PlaybackStatus::Paused:
            return fg::PlaybackStatus::Paused;
        case FoxglovePublisher::PlaybackStatus::Buffering:
            return fg::PlaybackStatus::Buffering;
        case FoxglovePublisher::PlaybackStatus::Ended:
            return fg::PlaybackStatus::Ended;
    }
    return fg::PlaybackStatus::Paused;
}

fg::PlaybackState toSdkPlaybackState(const FoxglovePublisher::PlaybackState& state) {
    fg::PlaybackState sdk_state;
    sdk_state.status = toSdkPlaybackStatus(state.status);
    sdk_state.current_time = state.current_time_ns;
    sdk_state.playback_speed = state.speed;
    sdk_state.did_seek = state.did_seek;
    return sdk_state;
}

uint8_t clampToUint8(float value) {
    if (!std::isfinite(value)) {
        return 0;
    }
    const auto rounded = static_cast<int>(std::lround(value));
    return static_cast<uint8_t>(std::clamp(rounded, 0, 255));
}

FoxglovePublisherImpl* getImpl(void* ptr) {
    return static_cast<FoxglovePublisherImpl*>(ptr);
}

const FoxglovePublisherImpl* getImpl(const void* ptr) {
    return static_cast<const FoxglovePublisherImpl*>(ptr);
}

template <typename T>
bool assignChannel(std::optional<T>& channel, fg::FoxgloveResult<T>& result,
                   const char* topic) {
    if (!result) {
        std::cerr << "[Foxglove] Failed to create channel " << topic
                  << ": " << static_cast<int>(result.error()) << std::endl;
        return false;
    }
    channel.emplace(std::move(result.value()));
    return true;
}

fgm::PointCloud makePointCloudMessage(const PointCloudXYZI& cloud, double timestamp) {
    fgm::PointCloud msg;
    msg.timestamp = toFoxgloveTimestamp(timestamp);
    msg.frame_id = kFastLioMapFrame;
    msg.point_stride = sizeof(PackedPoint);
    msg.fields = {
        {"x", 0, fgm::PackedElementField::NumericType::FLOAT32},
        {"y", 4, fgm::PackedElementField::NumericType::FLOAT32},
        {"z", 8, fgm::PackedElementField::NumericType::FLOAT32},
        {"intensity", 12, fgm::PackedElementField::NumericType::FLOAT32},
        {"tag", 16, fgm::PackedElementField::NumericType::UINT8},
        {"line", 17, fgm::PackedElementField::NumericType::UINT8},
    };

    msg.data.resize(cloud.points.size() * sizeof(PackedPoint));
    for (size_t i = 0; i < cloud.points.size(); ++i) {
        const auto& src = cloud.points[i];
        const PackedPoint packed{
            src.x,
            src.y,
            src.z,
            src.intensity,
            clampToUint8(src.normal_x),
            clampToUint8(src.normal_y),
            0,
            0,
        };
        std::memcpy(msg.data.data() + i * sizeof(PackedPoint), &packed, sizeof(PackedPoint));
    }

    return msg;
}

}  // namespace

FoxglovePublisher::FoxglovePublisher()
    : port_(8765), server_impl_(nullptr)
{}

FoxglovePublisher::~FoxglovePublisher() {
    stop();
}

bool FoxglovePublisher::start(const std::string& host, uint16_t port,
                              size_t message_backlog_size) {
    std::unique_lock<std::shared_mutex> lock(publish_mutex_);
    if (running_) {
        return true;
    }

    auto impl = std::make_unique<FoxglovePublisherImpl>();

    fg::WebSocketServerOptions options;
    options.name = "livox-fast-lio";
    options.host = host;
    options.port = port;
    options.capabilities = fg::WebSocketServerCapabilities::Time;
    // Data and control messages share the configured queue depth. A backlog of
    // eight is too small when a client is decoding a large point cloud: the
    // control queue can fill and the SDK then disconnects the slow client.
    options.message_backlog_size = std::max<size_t>(message_backlog_size, 8);
    client_count_ = 0;
    client_connection_generation_ = 0;
    options.callbacks.onClientConnect = [this]() {
        const size_t count = client_count_.fetch_add(1) + 1;
        client_connection_generation_.fetch_add(1);
        std::cout << "[Foxglove] Client connected (clients=" << count << ")"
                  << std::endl;
    };
    options.callbacks.onClientDisconnect = [this]() {
        size_t current = client_count_.load();
        while (current > 0 &&
               !client_count_.compare_exchange_weak(current, current - 1)) {}
        std::cout << "[Foxglove] Client disconnected (clients="
                  << client_count_.load() << ")" << std::endl;
    };
    if (playback_time_range_ && playback_control_callback_) {
        options.playback_time_range = playback_time_range_;
        options.capabilities =
            fg::WebSocketServerCapabilities::Time |
            fg::WebSocketServerCapabilities::PlaybackControl;
        options.callbacks.onPlaybackControlRequest =
            [this](const fg::PlaybackControlRequest& sdk_request)
            -> std::optional<fg::PlaybackState> {
                PlaybackControlRequest request;
                request.command =
                    sdk_request.playback_command == fg::PlaybackCommand::Pause
                        ? PlaybackCommand::Pause
                        : PlaybackCommand::Play;
                request.speed = sdk_request.playback_speed;
                request.seek_time_ns = sdk_request.seek_time;
                return toSdkPlaybackState(playback_control_callback_(request));
            };
    }
    options.supported_encodings = {"protobuf", "json"};

    auto server_result = fg::WebSocketServer::create(std::move(options));
    if (!server_result) {
        std::cerr << "[Foxglove] Failed to start WebSocket server: "
                  << static_cast<int>(server_result.error()) << std::endl;
        return false;
    }
    impl->server.emplace(std::move(server_result.value()));

    auto cloud_result = fgm::PointCloudChannel::create(kFastLioRegisteredCloudTopic);
    auto map_result = fgm::PointCloudChannel::create(kFastLioMapTopic);
    auto map_delta_result = fgm::PointCloudChannel::create(kFastLioMapDeltaTopic);
    const fg::Schema imu_schema{
        "livox_fast_lio/Imu",
        "jsonschema",
        reinterpret_cast<const std::byte*>(kImuJsonSchema),
        sizeof(kImuJsonSchema) - 1,
    };
    auto imu_result = fg::RawChannel::create(
        kFastLioImuTopic, "json", imu_schema);
    auto odom_result = fgm::OdometryChannel::create("/odometry");
    auto path_result = fgm::PosesInFrameChannel::create("/path");
    auto tf_result = fgm::FrameTransformsChannel::create("/tf");

    if (!assignChannel(impl->cloud, cloud_result, kFastLioRegisteredCloudTopic) ||
        !assignChannel(impl->map, map_result, kFastLioMapTopic) ||
        !assignChannel(impl->map_delta, map_delta_result, kFastLioMapDeltaTopic) ||
        !assignChannel(impl->imu, imu_result, kFastLioImuTopic) ||
        !assignChannel(impl->odom, odom_result, "/odometry") ||
        !assignChannel(impl->path, path_result, "/path") ||
        !assignChannel(impl->tf, tf_result, "/tf")) {
        if (impl->server) {
            impl->server->stop();
        }
        return false;
    }

    host_ = host;
    port_ = impl->server->port();
    server_impl_ = impl.release();
    running_ = true;

    std::cout << "[Foxglove] WebSocket server started on " << host_ << ":" << port_
              << std::endl;
    return true;
}

void FoxglovePublisher::setPlaybackControl(uint64_t start_time_ns,
                                           uint64_t end_time_ns,
                                           PlaybackControlCallback callback) {
    std::unique_lock<std::shared_mutex> lock(publish_mutex_);
    playback_time_range_ = std::make_pair(start_time_ns, end_time_ns);
    playback_control_callback_ = std::move(callback);
}

void FoxglovePublisher::stop() {
    std::unique_lock<std::shared_mutex> lock(publish_mutex_);
    running_ = false;

    auto* impl = getImpl(server_impl_);
    if (!impl) {
        return;
    }

    if (impl->cloud) impl->cloud->close();
    if (impl->map) impl->map->close();
    if (impl->map_delta) impl->map_delta->close();
    if (impl->imu) impl->imu->close();
    if (impl->odom) impl->odom->close();
    if (impl->path) impl->path->close();
    if (impl->tf) impl->tf->close();
    if (impl->server) impl->server->stop();

    delete impl;
    server_impl_ = nullptr;
    client_count_ = 0;
}

size_t FoxglovePublisher::getClientCount() const {
    std::shared_lock<std::shared_mutex> lock(publish_mutex_);
    auto* impl = getImpl(server_impl_);
    if (!impl || !impl->server || !running_) {
        return 0;
    }
    return impl->server->clientCount();
}

void FoxglovePublisher::publishPointCloud(const PointCloudXYZI& cloud, double timestamp) {
    std::shared_lock<std::shared_mutex> lock(publish_mutex_);
    auto* impl = getImpl(server_impl_);
    if (!impl || !impl->cloud || !running_) {
        return;
    }

    auto msg = makePointCloudMessage(cloud, timestamp);
    impl->cloud->log(msg, doubleToNs(timestamp));
}

void FoxglovePublisher::publishMap(const PointCloudXYZI& map, double timestamp) {
    if (!running_) {
        return;
    }

    // Packing grows linearly with the full map. Keep that work outside the
    // lifecycle mutex so odometry/TF publishing is not blocked by map building.
    auto msg = makePointCloudMessage(map, timestamp);

    std::shared_lock<std::shared_mutex> lock(publish_mutex_);
    auto* impl = getImpl(server_impl_);
    if (!impl || !impl->map || !running_) {
        return;
    }

    impl->map->log(msg, doubleToNs(timestamp));
}

void FoxglovePublisher::publishMapDelta(const PointCloudXYZI& delta, double timestamp) {
    if (!running_) {
        return;
    }

    auto msg = makePointCloudMessage(delta, timestamp);

    std::shared_lock<std::shared_mutex> lock(publish_mutex_);
    auto* impl = getImpl(server_impl_);
    if (!impl || !impl->map_delta || !running_) {
        return;
    }

    impl->map_delta->log(msg, doubleToNs(timestamp));
}

void FoxglovePublisher::publishImu(const ImuData& imu) {
    std::shared_lock<std::shared_mutex> lock(publish_mutex_);
    auto* impl = getImpl(server_impl_);
    if (!impl || !impl->imu || !running_) {
        return;
    }

    uint32_t sec = 0;
    uint32_t nsec = 0;
    doubleToRosTime(imu.timestamp, sec, nsec);

    // Fixed-size formatting avoids heap churn on the 200 Hz sensor callback.
    char json[512];
    const int length = std::snprintf(
        json, sizeof(json),
        "{\"timestamp\":{\"sec\":%u,\"nsec\":%u},"
        "\"frame_id\":\"%s\","
        "\"angular_velocity\":{\"x\":%.12g,\"y\":%.12g,\"z\":%.12g},"
        "\"linear_acceleration\":{\"x\":%.12g,\"y\":%.12g,\"z\":%.12g}}",
        sec, nsec, kFastLioImuFrame,
        imu.gyro.x(), imu.gyro.y(), imu.gyro.z(),
        imu.acc.x(), imu.acc.y(), imu.acc.z());
    if (length <= 0 || static_cast<size_t>(length) >= sizeof(json)) {
        return;
    }

    impl->imu->log(reinterpret_cast<const std::byte*>(json),
                   static_cast<size_t>(length), doubleToNs(imu.timestamp));
}

void FoxglovePublisher::publishOdometry(const V3D& position,
                                        const Eigen::Quaterniond& orientation,
                                        double timestamp) {
    std::shared_lock<std::shared_mutex> lock(publish_mutex_);
    auto* impl = getImpl(server_impl_);
    if (!impl || !impl->odom || !running_) {
        return;
    }

    fgm::Odometry msg;
    msg.timestamp = toFoxgloveTimestamp(timestamp);
    msg.frame_id = kFastLioMapFrame;
    msg.body_frame_id = kFastLioBodyFrame;
    msg.pose = makePose(position, orientation);

    impl->odom->log(msg, doubleToNs(timestamp));
}

void FoxglovePublisher::publishPath(const std::vector<V3D>& path) {
    publishPath(path, 0.0);
}

void FoxglovePublisher::publishPath(const std::vector<V3D>& path, double timestamp) {
    std::shared_lock<std::shared_mutex> lock(publish_mutex_);
    auto* impl = getImpl(server_impl_);
    if (!impl || !impl->path || !running_) {
        return;
    }

    fgm::PosesInFrame msg;
    msg.timestamp = toFoxgloveTimestamp(timestamp);
    msg.frame_id = kFastLioMapFrame;

    const size_t start = path.size() > 500 ? path.size() - 500 : 0;
    msg.poses.reserve(path.size() - start);
    const Eigen::Quaterniond identity = Eigen::Quaterniond::Identity();
    for (size_t i = start; i < path.size(); ++i) {
        msg.poses.push_back(makePose(path[i], identity));
    }

    impl->path->log(msg, doubleToNs(timestamp));
}

void FoxglovePublisher::publishTransform(const V3D& translation,
                                         const Eigen::Quaterniond& rotation,
                                         const std::string& parent_frame,
                                         const std::string& child_frame,
                                         double timestamp) {
    std::shared_lock<std::shared_mutex> lock(publish_mutex_);
    auto* impl = getImpl(server_impl_);
    if (!impl || !impl->tf || !running_) {
        return;
    }

    fgm::FrameTransform transform;
    transform.timestamp = toFoxgloveTimestamp(timestamp);
    transform.parent_frame_id = parent_frame;
    transform.child_frame_id = child_frame;
    transform.translation = fgm::Vector3{translation.x(), translation.y(), translation.z()};
    transform.rotation = fgm::Quaternion{rotation.x(), rotation.y(), rotation.z(), rotation.w()};

    fgm::FrameTransforms msg;
    msg.transforms.push_back(std::move(transform));

    impl->tf->log(msg, doubleToNs(timestamp));
}

void FoxglovePublisher::broadcastTime(double timestamp) {
    std::shared_lock<std::shared_mutex> lock(publish_mutex_);
    auto* impl = getImpl(server_impl_);
    if (!impl || !impl->server || !running_) {
        return;
    }
    impl->server->broadcastTime(doubleToNs(timestamp));
}

void FoxglovePublisher::broadcastPlaybackState(const PlaybackState& state) {
    std::shared_lock<std::shared_mutex> lock(publish_mutex_);
    auto* impl = getImpl(server_impl_);
    if (!impl || !impl->server || !running_) {
        return;
    }
    impl->server->broadcastPlaybackState(toSdkPlaybackState(state));
}

void FoxglovePublisher::clearSession() {
    std::shared_lock<std::shared_mutex> lock(publish_mutex_);
    auto* impl = getImpl(server_impl_);
    if (!impl || !impl->server || !running_) {
        return;
    }
    (void)impl->server->clearSession();
}
