#ifndef LIVOX_ADAPTER_H
#define LIVOX_ADAPTER_H

#include <string>
#include <chrono>
#include <cstdint>
#include <deque>
#include <mutex>
#include <condition_variable>
#include <thread>
#include <functional>
#include <atomic>
#include <vector>
#include "types.h"
#include "livox_sdk.h"
#include "lvx_reader.h"

// Callback types
using LidarDataCallback = std::function<void(const std::vector<LvxPoint>& points, double timestamp)>;
using ImuDataCallback = std::function<void(const ImuData& imu)>;

class LivoxAdapter
{
public:
    LivoxAdapter();
    ~LivoxAdapter();

    bool init();
    bool start(const std::string& broadcast_code = "");
    // Stops SDK discovery/data threads and waits for every callback that
    // acquired this adapter. On timeout the shutdown thread remains owned by
    // this object; callers must keep it alive or terminate the process.
    bool stopFor(std::chrono::milliseconds timeout);
    void stop();
    bool isConnected() const { return connected_; }
    bool hasCallbackFailure() const noexcept {
        return callback_failed_.load();
    }
    uint64_t callbackFailureCount() const noexcept {
        return callback_failures_.load();
    }

    void setLidarCallback(LidarDataCallback cb);
    void setImuCallback(ImuDataCallback cb);

    // Get buffered data (thread-safe)
    // Transfers ownership of the buffered packets to the caller. Returning a
    // pointer into the queue would become dangling as soon as the queue entry
    // is removed.
    bool getLidarData(std::vector<LivoxEthPacket>& data,
                      uint32_t& num,
                      double& timestamp,
                      int timeout_ms = 100);
    bool getImuData(ImuData& imu, int timeout_ms = 100);

private:
    friend struct LivoxAdapterTestAccess;

    class CallbackLease {
    public:
        CallbackLease() = default;
        explicit CallbackLease(LivoxAdapter* owner) noexcept : owner_(owner) {}
        CallbackLease(const CallbackLease&) = delete;
        CallbackLease& operator=(const CallbackLease&) = delete;
        CallbackLease(CallbackLease&& other) noexcept
            : owner_(other.owner_) { other.owner_ = nullptr; }
        CallbackLease& operator=(CallbackLease&& other) noexcept {
            if (this != &other) {
                if (owner_) owner_->releaseCallback();
                owner_ = other.owner_;
                other.owner_ = nullptr;
            }
            return *this;
        }
        ~CallbackLease() noexcept;
        explicit operator bool() const noexcept { return owner_ != nullptr; }
        LivoxAdapter* owner() const noexcept { return owner_; }

    private:
        LivoxAdapter* owner_ = nullptr;
    };

    static void OnDeviceBroadcast(const BroadcastDeviceInfo* info) noexcept;
    static void OnDeviceChange(const DeviceInfo* info, DeviceEvent type) noexcept;
    static void OnSampleCallback(livox_status status, uint8_t handle,
                                 uint8_t response, void* client_data) noexcept;
    static void OnStopSampleCallback(livox_status status, uint8_t handle,
                                     uint8_t response,
                                     void* client_data) noexcept;
    static void OnDeviceInformation(livox_status status, uint8_t handle,
                                    DeviceInformationResponse* response,
                                    void* client_data) noexcept;
    static void OnCartesianCallback(livox_status status, uint8_t handle,
                                    uint8_t response,
                                    void* client_data) noexcept;
    static void OnImuFreqCallback(livox_status status, uint8_t handle,
                                  uint8_t response,
                                  void* client_data) noexcept;
    static void OnReturnModeCallback(livox_status status, uint8_t handle,
                                     uint8_t response,
                                     void* client_data) noexcept;
    static void OnErrorStatus(livox_status status, uint8_t handle,
                              ErrorMessage* message) noexcept;
    static void OnDataCallback(uint8_t handle, LivoxEthPacket* data,
                               uint32_t data_num,
                               void* client_data) noexcept;
    CallbackLease acquireCallback();
    static CallbackLease acquireInstanceCallback();
    void releaseCallback() noexcept;
    void recordCallbackFailure() noexcept;
    static void recordInstanceCallbackFailure() noexcept;
    void requestStop();
    void shutdownSdk();

    std::atomic<bool> connected_{false};
    std::atomic<bool> running_{false};
    std::atomic<bool> sampling_{false};
    std::atomic<bool> sdk_started_{false};
    std::atomic<bool> callback_failed_{false};
    std::atomic<uint64_t> callback_failures_{0};
    uint8_t device_handle_;
    std::string target_broadcast_code_;
    std::mutex state_mutex_;

    LidarDataCallback lidar_cb_;
    ImuDataCallback imu_cb_;

    // Buffers
    struct LidarFrame {
        std::vector<LivoxEthPacket> packets;
        uint32_t num;
        double timestamp;
    };
    std::deque<LidarFrame> lidar_buffer_;
    std::deque<ImuData> imu_buffer_;
    std::mutex lidar_mutex_;
    std::mutex imu_mutex_;
    std::condition_variable lidar_cv_;
    std::condition_variable imu_cv_;

    static LivoxAdapter* instance_;
    static std::mutex instance_mutex_;
    static thread_local LivoxAdapter* callback_thread_owner_;
    static thread_local size_t callback_thread_depth_;
    std::mutex callback_mutex_;
    std::condition_variable callback_cv_;
    bool accepting_callbacks_ = false;
    std::atomic<size_t> callbacks_in_flight_{0};
    std::mutex shutdown_mutex_;
    std::condition_variable shutdown_cv_;
    std::thread shutdown_thread_;
    bool shutdown_started_ = false;
    bool shutdown_complete_ = false;
};

#endif
