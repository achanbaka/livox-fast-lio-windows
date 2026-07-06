#include "livox_adapter.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstring>
#include <iostream>

namespace {

constexpr double kGToMetersPerSecondSquared = 9.80665;
constexpr double kPi = 3.14159265358979323846;

uint64_t packetTimestampNs(const LivoxEthPacket* packet)
{
    if (!packet) return 0;
    uint64_t ts = 0;
    std::memcpy(&ts, packet->timestamp, sizeof(ts));
    return ts;
}

double packetTimestampSec(const LivoxEthPacket* packet)
{
    const uint64_t ts = packetTimestampNs(packet);
    if (ts > 0) {
        return static_cast<double>(ts) * 1e-9;
    }
    const auto now = std::chrono::steady_clock::now().time_since_epoch();
    return std::chrono::duration<double>(now).count();
}

void appendPoint(std::vector<LvxPoint>& out,
                 float x,
                 float y,
                 float z,
                 uint8_t reflectivity,
                 uint8_t tag,
                 uint8_t line)
{
    LvxPoint point;
    point.x = x;
    point.y = y;
    point.z = z;
    point.reflectivity = reflectivity;
    point.tag = tag;
    point.line = line;
    point.offset_time = 0;
    out.push_back(point);
}

void appendSphericalPoint(std::vector<LvxPoint>& out,
                          uint32_t depth,
                          uint16_t theta,
                          uint16_t phi,
                          uint8_t reflectivity,
                          uint8_t tag,
                          uint8_t line)
{
    const double r = static_cast<double>(depth) * 0.001;
    const double theta_rad = static_cast<double>(theta) * 0.01 * kPi / 180.0;
    const double phi_rad = static_cast<double>(phi) * 0.01 * kPi / 180.0;
    const float x = static_cast<float>(r * std::sin(theta_rad) * std::cos(phi_rad));
    const float y = static_cast<float>(r * std::sin(theta_rad) * std::sin(phi_rad));
    const float z = static_cast<float>(r * std::cos(theta_rad));
    appendPoint(out, x, y, z, reflectivity, tag, line);
}

std::vector<LvxPoint> decodeLivoxPoints(const LivoxEthPacket* packet, uint32_t data_num)
{
    std::vector<LvxPoint> points;
    if (!packet || data_num == 0) {
        return points;
    }

    const uint8_t line = packet->id;
    switch (packet->data_type) {
    case kCartesian: {
        const auto* raw = reinterpret_cast<const LivoxRawPoint*>(packet->data);
        points.reserve(data_num);
        for (uint32_t i = 0; i < data_num; ++i) {
            appendPoint(points,
                        raw[i].x * 0.001f,
                        raw[i].y * 0.001f,
                        raw[i].z * 0.001f,
                        raw[i].reflectivity,
                        0,
                        line);
        }
        break;
    }
    case kSpherical: {
        const auto* raw = reinterpret_cast<const LivoxSpherPoint*>(packet->data);
        points.reserve(data_num);
        for (uint32_t i = 0; i < data_num; ++i) {
            appendSphericalPoint(points, raw[i].depth, raw[i].theta, raw[i].phi,
                                 raw[i].reflectivity, 0, line);
        }
        break;
    }
    case kExtendCartesian: {
        const auto* raw = reinterpret_cast<const LivoxExtendRawPoint*>(packet->data);
        points.reserve(data_num);
        for (uint32_t i = 0; i < data_num; ++i) {
            appendPoint(points,
                        raw[i].x * 0.001f,
                        raw[i].y * 0.001f,
                        raw[i].z * 0.001f,
                        raw[i].reflectivity,
                        raw[i].tag,
                        line);
        }
        break;
    }
    case kExtendSpherical: {
        const auto* raw = reinterpret_cast<const LivoxExtendSpherPoint*>(packet->data);
        points.reserve(data_num);
        for (uint32_t i = 0; i < data_num; ++i) {
            appendSphericalPoint(points, raw[i].depth, raw[i].theta, raw[i].phi,
                                 raw[i].reflectivity, raw[i].tag, line);
        }
        break;
    }
    case kDualExtendCartesian: {
        const auto* raw = reinterpret_cast<const LivoxDualExtendRawPoint*>(packet->data);
        points.reserve(static_cast<size_t>(data_num) * 2);
        for (uint32_t i = 0; i < data_num; ++i) {
            appendPoint(points, raw[i].x1 * 0.001f, raw[i].y1 * 0.001f, raw[i].z1 * 0.001f,
                        raw[i].reflectivity1, raw[i].tag1, line);
            appendPoint(points, raw[i].x2 * 0.001f, raw[i].y2 * 0.001f, raw[i].z2 * 0.001f,
                        raw[i].reflectivity2, raw[i].tag2, line);
        }
        break;
    }
    case kDualExtendSpherical: {
        const auto* raw = reinterpret_cast<const LivoxDualExtendSpherPoint*>(packet->data);
        points.reserve(static_cast<size_t>(data_num) * 2);
        for (uint32_t i = 0; i < data_num; ++i) {
            appendSphericalPoint(points, raw[i].depth1, raw[i].theta, raw[i].phi,
                                 raw[i].reflectivity1, raw[i].tag1, line);
            appendSphericalPoint(points, raw[i].depth2, raw[i].theta, raw[i].phi,
                                 raw[i].reflectivity2, raw[i].tag2, line);
        }
        break;
    }
    case kTripleExtendCartesian: {
        const auto* raw = reinterpret_cast<const LivoxTripleExtendRawPoint*>(packet->data);
        points.reserve(static_cast<size_t>(data_num) * 3);
        for (uint32_t i = 0; i < data_num; ++i) {
            appendPoint(points, raw[i].x1 * 0.001f, raw[i].y1 * 0.001f, raw[i].z1 * 0.001f,
                        raw[i].reflectivity1, raw[i].tag1, line);
            appendPoint(points, raw[i].x2 * 0.001f, raw[i].y2 * 0.001f, raw[i].z2 * 0.001f,
                        raw[i].reflectivity2, raw[i].tag2, line);
            appendPoint(points, raw[i].x3 * 0.001f, raw[i].y3 * 0.001f, raw[i].z3 * 0.001f,
                        raw[i].reflectivity3, raw[i].tag3, line);
        }
        break;
    }
    case kTripleExtendSpherical: {
        const auto* raw = reinterpret_cast<const LivoxTripleExtendSpherPoint*>(packet->data);
        points.reserve(static_cast<size_t>(data_num) * 3);
        for (uint32_t i = 0; i < data_num; ++i) {
            appendSphericalPoint(points, raw[i].depth1, raw[i].theta, raw[i].phi,
                                 raw[i].reflectivity1, raw[i].tag1, line);
            appendSphericalPoint(points, raw[i].depth2, raw[i].theta, raw[i].phi,
                                 raw[i].reflectivity2, raw[i].tag2, line);
            appendSphericalPoint(points, raw[i].depth3, raw[i].theta, raw[i].phi,
                                 raw[i].reflectivity3, raw[i].tag3, line);
        }
        break;
    }
    default:
        break;
    }
    return points;
}

void logCommand(const char* name, livox_status status, uint8_t handle, uint8_t response)
{
    std::cout << "[LivoxAdapter] " << name
              << " status=" << status
              << " handle=" << static_cast<int>(handle)
              << " response=" << static_cast<int>(response) << std::endl;
}

void OnCartesianCallback(livox_status status, uint8_t handle, uint8_t response, void*)
{
    logCommand("SetCartesianCoordinate", status, handle, response);
}

void OnImuFreqCallback(livox_status status, uint8_t handle, uint8_t response, void*)
{
    logCommand("LidarSetImuPushFrequency", status, handle, response);
}

void OnReturnModeCallback(livox_status status, uint8_t handle, uint8_t response, void*)
{
    logCommand("LidarSetPointCloudReturnMode", status, handle, response);
}

} // namespace

LivoxAdapter* LivoxAdapter::instance_ = nullptr;

LivoxAdapter::LivoxAdapter() : device_handle_(0)
{
    instance_ = this;
}

LivoxAdapter::~LivoxAdapter()
{
    stop();
    Uninit();
    instance_ = nullptr;
}

bool LivoxAdapter::init()
{
    if (!Init()) {
        std::cerr << "[LivoxAdapter] SDK init failed." << std::endl;
        return false;
    }

    LivoxSdkVersion version{};
    GetLivoxSdkVersion(&version);
    std::cout << "[LivoxAdapter] SDK version "
              << version.major << '.'
              << version.minor << '.'
              << version.patch << std::endl;

    SetBroadcastCallback(OnDeviceBroadcast);
    SetDeviceStateUpdateCallback(OnDeviceChange);

    if (!Start()) {
        std::cerr << "[LivoxAdapter] SDK discovery start failed." << std::endl;
        Uninit();
        return false;
    }

    sdk_started_ = true;
    std::cout << "[LivoxAdapter] Discovering Livox devices..." << std::endl;
    return true;
}

bool LivoxAdapter::start(const std::string& broadcast_code)
{
    {
        std::lock_guard<std::mutex> lock(state_mutex_);
        target_broadcast_code_ = broadcast_code;
    }
    running_ = true;
    if (!broadcast_code.empty()) {
        std::cout << "[LivoxAdapter] Waiting for broadcast code: "
                  << broadcast_code << std::endl;
    }
    return sdk_started_;
}

void LivoxAdapter::stop()
{
    running_ = false;
    if (sampling_) {
        LidarStopSampling(device_handle_, OnStopSampleCallback, this);
        sampling_ = false;
    }
    connected_ = false;
}

void LivoxAdapter::OnDeviceBroadcast(const BroadcastDeviceInfo* info)
{
    if (!instance_ || !info || info->dev_type == kDeviceTypeHub) {
        return;
    }

    std::string target;
    {
        std::lock_guard<std::mutex> lock(instance_->state_mutex_);
        target = instance_->target_broadcast_code_;
    }

    std::cout << "[LivoxAdapter] Broadcast: code=" << info->broadcast_code
              << " type=" << static_cast<int>(info->dev_type) << std::endl;

    if (!target.empty() && target != info->broadcast_code) {
        return;
    }

    uint8_t handle = 0;
    const livox_status result = AddLidarToConnect(info->broadcast_code, &handle);
    if (result != kStatusSuccess) {
        std::cerr << "[LivoxAdapter] AddLidarToConnect failed, status="
                  << result << std::endl;
        return;
    }

    {
        std::lock_guard<std::mutex> lock(instance_->state_mutex_);
        instance_->device_handle_ = handle;
    }

    SetDataCallback(handle, OnDataCallback, instance_);
    SetErrorMessageCallback(handle, OnErrorStatus);
    std::cout << "[LivoxAdapter] Added LiDAR handle="
              << static_cast<int>(handle) << std::endl;
}

void LivoxAdapter::OnDeviceChange(const DeviceInfo* info, DeviceEvent type)
{
    if (!instance_ || !info) {
        return;
    }

    std::cout << "[LivoxAdapter] Device event=" << static_cast<int>(type)
              << " code=" << info->broadcast_code
              << " handle=" << static_cast<int>(info->handle)
              << " state=" << static_cast<int>(info->state) << std::endl;

    if (type == kEventDisconnect) {
        instance_->connected_ = false;
        instance_->sampling_ = false;
        return;
    }

    {
        std::lock_guard<std::mutex> lock(instance_->state_mutex_);
        instance_->device_handle_ = info->handle;
    }

    if (type == kEventConnect) {
        QueryDeviceInformation(info->handle, OnDeviceInformation, instance_);
    }

    if (info->state != kLidarStateNormal || instance_->sampling_) {
        return;
    }

    SetCartesianCoordinate(info->handle, OnCartesianCallback, nullptr);
    LidarSetImuPushFrequency(info->handle, kImuFreq200Hz, OnImuFreqCallback, nullptr);
    LidarSetPointCloudReturnMode(info->handle, kStrongestReturn, OnReturnModeCallback, nullptr);

    const livox_status result = LidarStartSampling(info->handle, OnSampleCallback, instance_);
    if (result != kStatusSuccess) {
        std::cerr << "[LivoxAdapter] LidarStartSampling failed, status="
                  << result << std::endl;
        return;
    }

    instance_->sampling_ = true;
    instance_->connected_ = true;
}

void LivoxAdapter::OnSampleCallback(livox_status status, uint8_t handle, uint8_t response, void* client_data)
{
    auto* self = static_cast<LivoxAdapter*>(client_data);
    std::cout << "[LivoxAdapter] Start sampling callback status=" << status
              << " handle=" << static_cast<int>(handle)
              << " response=" << static_cast<int>(response) << std::endl;
    if (!self) return;
    const bool ok = (status == kStatusSuccess && response == 0);
    self->connected_.store(ok);
    self->sampling_.store(ok);
}

void LivoxAdapter::OnStopSampleCallback(livox_status status, uint8_t handle, uint8_t response, void*)
{
    std::cout << "[LivoxAdapter] Stop sampling callback status=" << status
              << " handle=" << static_cast<int>(handle)
              << " response=" << static_cast<int>(response) << std::endl;
}

void LivoxAdapter::OnDeviceInformation(livox_status status,
                                       uint8_t handle,
                                       DeviceInformationResponse* response,
                                       void*)
{
    if (status != kStatusSuccess || !response) {
        std::cout << "[LivoxAdapter] Firmware query failed, status="
                  << status << " handle=" << static_cast<int>(handle) << std::endl;
        return;
    }

    std::cout << "[LivoxAdapter] Firmware "
              << static_cast<int>(response->firmware_version[0]) << '.'
              << static_cast<int>(response->firmware_version[1]) << '.'
              << static_cast<int>(response->firmware_version[2]) << '.'
              << static_cast<int>(response->firmware_version[3])
              << " handle=" << static_cast<int>(handle) << std::endl;
}

void LivoxAdapter::OnErrorStatus(livox_status status, uint8_t handle, ErrorMessage* message)
{
    static uint32_t error_count = 0;
    if (status != kStatusSuccess || !message) {
        return;
    }
    if (++error_count % 100 == 0) {
        std::cout << "[LivoxAdapter] Device status handle="
                  << static_cast<int>(handle)
                  << " error_code=0x" << std::hex
                  << message->error_code << std::dec << std::endl;
    }
}

void LivoxAdapter::OnDataCallback(uint8_t, LivoxEthPacket* data, uint32_t data_num, void* client_data)
{
    auto* self = static_cast<LivoxAdapter*>(client_data);
    if (!self || !data || data_num == 0) {
        return;
    }

    const double timestamp = packetTimestampSec(data);

    if (data->data_type == kImu) {
        const auto* imu_points = reinterpret_cast<const LivoxImuPoint*>(data->data);
        for (uint32_t i = 0; i < data_num; ++i) {
            ImuData imu;
            imu.timestamp = timestamp;
            imu.gyro = V3D(imu_points[i].gyro_x, imu_points[i].gyro_y, imu_points[i].gyro_z);
            imu.acc = V3D(imu_points[i].acc_x * kGToMetersPerSecondSquared,
                          imu_points[i].acc_y * kGToMetersPerSecondSquared,
                          imu_points[i].acc_z * kGToMetersPerSecondSquared);
            if (self->imu_cb_) {
                self->imu_cb_(imu);
            }
        }
        return;
    }

    std::vector<LvxPoint> points = decodeLivoxPoints(data, data_num);
    if (points.empty()) {
        static uint32_t unsupported_count = 0;
        if (++unsupported_count % 100 == 1) {
            std::cout << "[LivoxAdapter] Ignored packet data_type="
                      << static_cast<int>(data->data_type)
                      << " data_num=" << data_num << std::endl;
        }
        return;
    }

    if (self->lidar_cb_) {
        self->lidar_cb_(points, timestamp);
    }
}

bool LivoxAdapter::getLidarData(LivoxEthPacket*& data, uint32_t& num, double& timestamp, int timeout_ms)
{
    std::unique_lock<std::mutex> lock(lidar_mutex_);
    if (!lidar_cv_.wait_for(lock, std::chrono::milliseconds(timeout_ms),
        [this]{ return !lidar_buffer_.empty(); }))
        return false;

    auto& frame = lidar_buffer_.front();
    data = frame.packets.empty() ? nullptr : frame.packets.data();
    num = frame.num;
    timestamp = frame.timestamp;
    lidar_buffer_.pop_front();
    return data != nullptr;
}

bool LivoxAdapter::getImuData(ImuData& imu, int timeout_ms)
{
    std::unique_lock<std::mutex> lock(imu_mutex_);
    if (!imu_cv_.wait_for(lock, std::chrono::milliseconds(timeout_ms),
        [this]{ return !imu_buffer_.empty(); }))
        return false;

    imu = imu_buffer_.front();
    imu_buffer_.pop_front();
    return true;
}
