#ifndef LIVOX_ADAPTER_H
#define LIVOX_ADAPTER_H

#include <string>
#include <deque>
#include <mutex>
#include <condition_variable>
#include <thread>
#include <functional>
#include <atomic>
#include "types.h"
#include "livox_sdk.h"

// Callback types
using LidarDataCallback = std::function<void(const LivoxEthPacket* data, uint32_t num, double timestamp)>;
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
    static void OnDeviceConnect(const uint8_t dev_type, const char* serial_num, const char* user_data);
    static void OnDeviceChange(const uint8_t dev_type, const char* serial_num, uint8_t state);
    static void OnDataCallback(uint8_t handle, LivoxEthPacket* data, uint32_t data_num, void* client_data);

    std::atomic<bool> connected_{false};
    std::atomic<bool> running_{false};
    uint8_t device_handle_;

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
