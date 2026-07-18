#include "livox_adapter.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstring>
#include <exception>
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

} // namespace

LivoxAdapter* LivoxAdapter::instance_ = nullptr;
std::mutex LivoxAdapter::instance_mutex_;
thread_local LivoxAdapter* LivoxAdapter::callback_thread_owner_ = nullptr;
thread_local size_t LivoxAdapter::callback_thread_depth_ = 0;

void LivoxAdapter::OnCartesianCallback(livox_status status, uint8_t handle,
                                       uint8_t response,
                                       void* client_data) noexcept
{
    auto* self = static_cast<LivoxAdapter*>(client_data);
    try {
        auto callback = self ? self->acquireCallback() : CallbackLease{};
        if (!callback) return;
        try {
            logCommand("SetCartesianCoordinate", status, handle, response);
        } catch (const std::exception&) {
            self->recordCallbackFailure();
        } catch (...) {
            self->recordCallbackFailure();
        }
    } catch (const std::exception&) {
        recordInstanceCallbackFailure();
    } catch (...) {
        recordInstanceCallbackFailure();
    }
}

void LivoxAdapter::OnImuFreqCallback(livox_status status, uint8_t handle,
                                     uint8_t response,
                                     void* client_data) noexcept
{
    auto* self = static_cast<LivoxAdapter*>(client_data);
    try {
        auto callback = self ? self->acquireCallback() : CallbackLease{};
        if (!callback) return;
        try {
            logCommand("LidarSetImuPushFrequency", status, handle, response);
        } catch (const std::exception&) {
            self->recordCallbackFailure();
        } catch (...) {
            self->recordCallbackFailure();
        }
    } catch (const std::exception&) {
        recordInstanceCallbackFailure();
    } catch (...) {
        recordInstanceCallbackFailure();
    }
}

void LivoxAdapter::OnReturnModeCallback(livox_status status, uint8_t handle,
                                        uint8_t response,
                                        void* client_data) noexcept
{
    auto* self = static_cast<LivoxAdapter*>(client_data);
    try {
        auto callback = self ? self->acquireCallback() : CallbackLease{};
        if (!callback) return;
        try {
            logCommand("LidarSetPointCloudReturnMode", status, handle, response);
        } catch (const std::exception&) {
            self->recordCallbackFailure();
        } catch (...) {
            self->recordCallbackFailure();
        }
    } catch (const std::exception&) {
        recordInstanceCallbackFailure();
    } catch (...) {
        recordInstanceCallbackFailure();
    }
}

LivoxAdapter::CallbackLease::~CallbackLease() noexcept
{
    if (owner_) owner_->releaseCallback();
}

LivoxAdapter::LivoxAdapter() : device_handle_(0)
{
    std::lock_guard<std::mutex> lock(instance_mutex_);
    instance_ = this;
}

LivoxAdapter::~LivoxAdapter()
{
    stop();
    std::lock_guard<std::mutex> lock(instance_mutex_);
    if (instance_ == this) instance_ = nullptr;
}

bool LivoxAdapter::init()
{
    {
        std::lock_guard<std::mutex> lock(shutdown_mutex_);
        if (shutdown_started_) return false;
    }
    if (!Init()) {
        std::cerr << "[LivoxAdapter] SDK init failed." << std::endl;
        return false;
    }
    sdk_started_ = true;
    callback_failed_ = false;
    {
        std::lock_guard<std::mutex> lock(callback_mutex_);
        accepting_callbacks_ = true;
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
        // Do not turn an initialization failure into an unbounded destructor
        // wait. The owner gets another bounded stop opportunity and must keep
        // the adapter alive (or terminate the process) if this times out.
        stopFor(std::chrono::seconds(5));
        return false;
    }

    std::cout << "[LivoxAdapter] Discovering Livox devices..." << std::endl;
    return true;
}

bool LivoxAdapter::start(const std::string& broadcast_code)
{
    {
        std::lock_guard<std::mutex> lock(shutdown_mutex_);
        if (!sdk_started_ || shutdown_started_) return false;
    }
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

void LivoxAdapter::setLidarCallback(LidarDataCallback cb)
{
    std::lock_guard<std::mutex> lock(callback_mutex_);
    lidar_cb_ = std::move(cb);
}

void LivoxAdapter::setImuCallback(ImuDataCallback cb)
{
    std::lock_guard<std::mutex> lock(callback_mutex_);
    imu_cb_ = std::move(cb);
}

LivoxAdapter::CallbackLease LivoxAdapter::acquireCallback()
{
    std::lock_guard<std::mutex> lock(callback_mutex_);
    if (!accepting_callbacks_ || callback_failed_.load()) {
        return CallbackLease{};
    }
    callbacks_in_flight_.fetch_add(1, std::memory_order_relaxed);
    callback_thread_owner_ = this;
    ++callback_thread_depth_;
    return CallbackLease(this);
}

LivoxAdapter::CallbackLease LivoxAdapter::acquireInstanceCallback()
{
    std::lock_guard<std::mutex> lock(instance_mutex_);
    return instance_ ? instance_->acquireCallback() : CallbackLease{};
}

void LivoxAdapter::releaseCallback() noexcept
{
    if (callback_thread_owner_ == this && callback_thread_depth_ > 0) {
        --callback_thread_depth_;
        if (callback_thread_depth_ == 0) callback_thread_owner_ = nullptr;
    }
    const size_t previous =
        callbacks_in_flight_.fetch_sub(1, std::memory_order_acq_rel);
    if (previous == 1) callback_cv_.notify_all();
}

void LivoxAdapter::recordCallbackFailure() noexcept
{
    callback_failures_.fetch_add(1);
    callback_failed_ = true;
    running_ = false;
    connected_ = false;
}

void LivoxAdapter::recordInstanceCallbackFailure() noexcept
{
    try {
        std::lock_guard<std::mutex> lock(instance_mutex_);
        if (instance_) instance_->recordCallbackFailure();
    } catch (...) {
        // This is already an exception boundary for a C callback. There is no
        // further safe recovery if even the lifetime mutex is unavailable.
    }
}

void LivoxAdapter::requestStop()
{
    running_ = false;
    connected_ = false;
    {
        std::lock_guard<std::mutex> lock(callback_mutex_);
        accepting_callbacks_ = false;
    }
}

void LivoxAdapter::shutdownSdk()
{
    // Let callbacks which crossed the admission gate finish all SDK calls
    // before tearing down SDK-owned state beneath them.
    {
        std::unique_lock<std::mutex> lock(callback_mutex_);
        while (callbacks_in_flight_.load(std::memory_order_acquire) != 0) {
            // The in-flight counter is released without taking this mutex so
            // exception unwinding cannot itself throw. A bounded periodic
            // wake also closes the notify-before-wait race.
            callback_cv_.wait_for(lock, std::chrono::milliseconds(10));
        }
    }
    if (sampling_.exchange(false)) {
        uint8_t handle = 0;
        {
            std::lock_guard<std::mutex> lock(state_mutex_);
            handle = device_handle_;
        }
        LidarStopSampling(handle, OnStopSampleCallback, this);
    }
    if (sdk_started_.exchange(false)) Uninit();

    // Uninit joins the Livox discovery, command, and per-device data threads.
    {
        std::lock_guard<std::mutex> lock(callback_mutex_);
        lidar_cb_ = LidarDataCallback{};
        imu_cb_ = ImuDataCallback{};
    }
    {
        std::lock_guard<std::mutex> lock(shutdown_mutex_);
        shutdown_complete_ = true;
    }
    shutdown_cv_.notify_all();
}

bool LivoxAdapter::stopFor(std::chrono::milliseconds timeout)
{
    requestStop();
    const bool called_from_callback =
        callback_thread_owner_ == this && callback_thread_depth_ > 0;
    {
        std::unique_lock<std::mutex> lock(shutdown_mutex_);
        if (!shutdown_started_) {
            shutdown_started_ = true;
            shutdown_complete_ = false;
            try {
                shutdown_thread_ = std::thread(&LivoxAdapter::shutdownSdk, this);
            } catch (...) {
                shutdown_started_ = false;
                return false;
            }
        }
        // The shutdown worker must wait for this lease. Waiting here would
        // create a callback -> stopFor -> shutdown -> callback self-cycle.
        if (called_from_callback) return false;
        if (!shutdown_cv_.wait_for(lock, timeout, [this] {
                return shutdown_complete_;
            })) {
            return false;
        }
    }
    if (shutdown_thread_.joinable()) shutdown_thread_.join();
    return true;
}

void LivoxAdapter::stop()
{
    requestStop();
    const bool called_from_callback =
        callback_thread_owner_ == this && callback_thread_depth_ > 0;
    bool run_synchronously = false;
    {
        std::lock_guard<std::mutex> lock(shutdown_mutex_);
        if (!shutdown_started_) {
            shutdown_started_ = true;
            shutdown_complete_ = false;
            try {
                shutdown_thread_ = std::thread(&LivoxAdapter::shutdownSdk, this);
            } catch (...) {
                if (called_from_callback) {
                    shutdown_started_ = false;
                } else {
                    run_synchronously = true;
                }
            }
        }
    }
    if (called_from_callback) return;
    if (run_synchronously) shutdownSdk();
    if (shutdown_thread_.joinable()) shutdown_thread_.join();
}

void LivoxAdapter::OnDeviceBroadcast(const BroadcastDeviceInfo* info) noexcept
{
    CallbackLease callback;
    try {
        callback = acquireInstanceCallback();
        LivoxAdapter* self = callback.owner();
        if (!self || !info || info->dev_type == kDeviceTypeHub) {
            return;
        }

        std::string target;
        {
            std::lock_guard<std::mutex> lock(self->state_mutex_);
            target = self->target_broadcast_code_;
        }

        std::cout << "[LivoxAdapter] Broadcast: code=" << info->broadcast_code
                  << " type=" << static_cast<int>(info->dev_type) << std::endl;

        if (!target.empty() && target != info->broadcast_code) {
            return;
        }

        uint8_t handle = 0;
        const livox_status result =
            AddLidarToConnect(info->broadcast_code, &handle);
        if (result != kStatusSuccess) {
            std::cerr << "[LivoxAdapter] AddLidarToConnect failed, status="
                      << result << std::endl;
            return;
        }

        {
            std::lock_guard<std::mutex> lock(self->state_mutex_);
            self->device_handle_ = handle;
        }

        SetDataCallback(handle, OnDataCallback, self);
        SetErrorMessageCallback(handle, OnErrorStatus);
        std::cout << "[LivoxAdapter] Added LiDAR handle="
                  << static_cast<int>(handle) << std::endl;
    } catch (const std::exception&) {
        if (callback) callback.owner()->recordCallbackFailure();
        else recordInstanceCallbackFailure();
    } catch (...) {
        if (callback) callback.owner()->recordCallbackFailure();
        else recordInstanceCallbackFailure();
    }
}

void LivoxAdapter::OnDeviceChange(const DeviceInfo* info,
                                  DeviceEvent type) noexcept
{
    CallbackLease callback;
    try {
        callback = acquireInstanceCallback();
        LivoxAdapter* self = callback.owner();
        if (!self || !info) {
            return;
        }

        std::cout << "[LivoxAdapter] Device event=" << static_cast<int>(type)
                  << " code=" << info->broadcast_code
                  << " handle=" << static_cast<int>(info->handle)
                  << " state=" << static_cast<int>(info->state) << std::endl;

        if (type == kEventDisconnect) {
            self->connected_ = false;
            self->sampling_ = false;
            return;
        }

        {
            std::lock_guard<std::mutex> lock(self->state_mutex_);
            self->device_handle_ = info->handle;
        }

        if (type == kEventConnect) {
            QueryDeviceInformation(info->handle, OnDeviceInformation, self);
        }

        if (info->state != kLidarStateNormal || self->sampling_) {
            return;
        }

        SetCartesianCoordinate(info->handle, OnCartesianCallback, self);
        LidarSetImuPushFrequency(
            info->handle, kImuFreq200Hz, OnImuFreqCallback, self);
        LidarSetPointCloudReturnMode(
            info->handle, kStrongestReturn, OnReturnModeCallback, self);
        if (self->callback_failed_.load()) return;

        const livox_status result =
            LidarStartSampling(info->handle, OnSampleCallback, self);
        if (result != kStatusSuccess) {
            std::cerr << "[LivoxAdapter] LidarStartSampling failed, status="
                      << result << std::endl;
            return;
        }

        self->sampling_ = true;
        self->connected_ = true;
    } catch (const std::exception&) {
        if (callback) callback.owner()->recordCallbackFailure();
        else recordInstanceCallbackFailure();
    } catch (...) {
        if (callback) callback.owner()->recordCallbackFailure();
        else recordInstanceCallbackFailure();
    }
}

void LivoxAdapter::OnSampleCallback(livox_status status, uint8_t handle,
                                    uint8_t response,
                                    void* client_data) noexcept
{
    auto* self = static_cast<LivoxAdapter*>(client_data);
    CallbackLease callback;
    try {
        callback = self ? self->acquireCallback() : CallbackLease{};
        if (!callback) return;
        std::cout << "[LivoxAdapter] Start sampling callback status=" << status
                  << " handle=" << static_cast<int>(handle)
                  << " response=" << static_cast<int>(response) << std::endl;
        const bool ok = (status == kStatusSuccess && response == 0);
        self->connected_.store(ok);
        self->sampling_.store(ok);
    } catch (const std::exception&) {
        if (callback) callback.owner()->recordCallbackFailure();
        else recordInstanceCallbackFailure();
    } catch (...) {
        if (callback) callback.owner()->recordCallbackFailure();
        else recordInstanceCallbackFailure();
    }
}

void LivoxAdapter::OnStopSampleCallback(livox_status status, uint8_t handle,
                                        uint8_t response,
                                        void*) noexcept
{
    try {
        std::cout << "[LivoxAdapter] Stop sampling callback status=" << status
                  << " handle=" << static_cast<int>(handle)
                  << " response=" << static_cast<int>(response) << std::endl;
    } catch (const std::exception&) {
        recordInstanceCallbackFailure();
    } catch (...) {
        recordInstanceCallbackFailure();
    }
}

void LivoxAdapter::OnDeviceInformation(livox_status status,
                                       uint8_t handle,
                                       DeviceInformationResponse* response,
                                       void* client_data) noexcept
{
    auto* self = static_cast<LivoxAdapter*>(client_data);
    CallbackLease callback;
    try {
        callback = self ? self->acquireCallback() : CallbackLease{};
        if (!callback) return;
        if (status != kStatusSuccess || !response) {
            std::cout << "[LivoxAdapter] Firmware query failed, status="
                      << status << " handle=" << static_cast<int>(handle)
                      << std::endl;
            return;
        }

        std::cout << "[LivoxAdapter] Firmware "
                  << static_cast<int>(response->firmware_version[0]) << '.'
                  << static_cast<int>(response->firmware_version[1]) << '.'
                  << static_cast<int>(response->firmware_version[2]) << '.'
                  << static_cast<int>(response->firmware_version[3])
                  << " handle=" << static_cast<int>(handle) << std::endl;
    } catch (const std::exception&) {
        if (callback) callback.owner()->recordCallbackFailure();
        else recordInstanceCallbackFailure();
    } catch (...) {
        if (callback) callback.owner()->recordCallbackFailure();
        else recordInstanceCallbackFailure();
    }
}

void LivoxAdapter::OnErrorStatus(livox_status status, uint8_t handle,
                                 ErrorMessage* message) noexcept
{
    CallbackLease callback;
    try {
        callback = acquireInstanceCallback();
        LivoxAdapter* self = callback.owner();
        if (!self || status != kStatusSuccess || !message) {
            return;
        }
        static std::atomic<uint32_t> error_count{0};
        const uint32_t count = error_count.fetch_add(1) + 1;
        if (count % 100 == 0) {
            std::cout << "[LivoxAdapter] Device status handle="
                      << static_cast<int>(handle)
                      << " error_code=0x" << std::hex
                      << message->error_code << std::dec << std::endl;
        }
    } catch (const std::exception&) {
        if (callback) callback.owner()->recordCallbackFailure();
        else recordInstanceCallbackFailure();
    } catch (...) {
        if (callback) callback.owner()->recordCallbackFailure();
        else recordInstanceCallbackFailure();
    }
}

void LivoxAdapter::OnDataCallback(uint8_t, LivoxEthPacket* data,
                                  uint32_t data_num,
                                  void* client_data) noexcept
{
    auto* self = static_cast<LivoxAdapter*>(client_data);
    CallbackLease callback;
    try {
        callback = self ? self->acquireCallback() : CallbackLease{};
        if (!callback || !self->running_ || !data || data_num == 0) {
            return;
        }

        const double timestamp = packetTimestampSec(data);

        if (data->data_type == kImu) {
            ImuDataCallback imu_callback;
            {
                std::lock_guard<std::mutex> lock(self->callback_mutex_);
                imu_callback = self->imu_cb_;
            }
            const auto* imu_points =
                reinterpret_cast<const LivoxImuPoint*>(data->data);
            for (uint32_t i = 0; i < data_num; ++i) {
                ImuData imu;
                imu.timestamp = timestamp;
                imu.gyro = V3D(
                    imu_points[i].gyro_x,
                    imu_points[i].gyro_y,
                    imu_points[i].gyro_z);
                imu.acc = V3D(
                    imu_points[i].acc_x * kGToMetersPerSecondSquared,
                    imu_points[i].acc_y * kGToMetersPerSecondSquared,
                    imu_points[i].acc_z * kGToMetersPerSecondSquared);
                if (imu_callback) {
                    imu_callback(imu);
                }
            }
            return;
        }

        std::vector<LvxPoint> points = decodeLivoxPoints(data, data_num);
        if (points.empty()) {
            static std::atomic<uint32_t> unsupported_count{0};
            const uint32_t count = unsupported_count.fetch_add(1) + 1;
            if (count % 100 == 1) {
                std::cout << "[LivoxAdapter] Ignored packet data_type="
                          << static_cast<int>(data->data_type)
                          << " data_num=" << data_num << std::endl;
            }
            return;
        }

        LidarDataCallback lidar_callback;
        {
            std::lock_guard<std::mutex> lock(self->callback_mutex_);
            lidar_callback = self->lidar_cb_;
        }
        if (lidar_callback) {
            lidar_callback(points, timestamp);
        }
    } catch (const std::exception&) {
        if (callback) callback.owner()->recordCallbackFailure();
        else recordInstanceCallbackFailure();
    } catch (...) {
        if (callback) callback.owner()->recordCallbackFailure();
        else recordInstanceCallbackFailure();
    }
}

bool LivoxAdapter::getLidarData(std::vector<LivoxEthPacket>& data,
                                uint32_t& num,
                                double& timestamp,
                                int timeout_ms)
{
    data.clear();
    num = 0;
    timestamp = 0.0;
    std::unique_lock<std::mutex> lock(lidar_mutex_);
    if (!lidar_cv_.wait_for(lock, std::chrono::milliseconds(timeout_ms),
        [this]{ return !lidar_buffer_.empty(); }))
        return false;

    auto& frame = lidar_buffer_.front();
    data = std::move(frame.packets);
    num = frame.num;
    timestamp = frame.timestamp;
    lidar_buffer_.pop_front();
    return !data.empty();
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
