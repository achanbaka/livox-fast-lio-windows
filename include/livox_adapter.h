#ifndef LIVOX_ADAPTER_H
#define LIVOX_ADAPTER_H

#include <string>
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
    void stop();
    bool isConnected() const { return connected_; }

    void setLidarCallback(LidarDataCallback cb) { lidar_cb_ = cb; }
    void setImuCallback(ImuDataCallback cb) { imu_cb_ = cb; }

    // Get buffered data (thread-safe)
    bool getLidarData(LivoxEthPacket*& data, uint32_t& num, double& timestamp, int timeout_ms = 100);
    bool getImuData(ImuData& imu, int timeout_ms = 100);

private:
    static void OnDeviceBroadcast(const BroadcastDeviceInfo* info);
    static void OnDeviceChange(const DeviceInfo* info, DeviceEvent type);
    static void OnSampleCallback(livox_status status, uint8_t handle, uint8_t response, void* client_data);
    static void OnStopSampleCallback(livox_status status, uint8_t handle, uint8_t response, void* client_data);
    static void OnDeviceInformation(livox_status status, uint8_t handle, DeviceInformationResponse* response, void* client_data);
    static void OnErrorStatus(livox_status status, uint8_t handle, ErrorMessage* message);
    static void OnDataCallback(uint8_t handle, LivoxEthPacket* data, uint32_t data_num, void* client_data);

    std::atomic<bool> connected_{false};
    std::atomic<bool> running_{false};
    std::atomic<bool> sampling_{false};
    std::atomic<bool> sdk_started_{false};
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
};

#endif
