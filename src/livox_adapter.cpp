#include "livox_adapter.h"
#include "livox_sdk.h"
#include <iostream>
#include <cstring>

LivoxAdapter* LivoxAdapter::instance_ = nullptr;

LivoxAdapter::LivoxAdapter() : device_handle_(0)
{
    instance_ = this;
}

LivoxAdapter::~LivoxAdapter()
{
    stop();
    UninitLivoxSdk();
    instance_ = nullptr;
}

bool LivoxAdapter::init()
{
    if (!Init())
    {
        std::cerr << "[LivoxAdapter] SDK init failed!" << std::endl;
        return false;
    }
    SetBroadcastCallback(OnDeviceConnect);
    SetDeviceStateUpdateCallback(OnDeviceChange);
    StartSample();
    std::cout << "[LivoxAdapter] SDK initialized, waiting for device..." << std::endl;
    return true;
}

bool LivoxAdapter::start(const std::string& broadcast_code)
{
    running_ = true;
    return true;
}

void LivoxAdapter::stop()
{
    running_ = false;
    if (connected_)
    {
        StopSample();
        connected_ = false;
    }
}

void LivoxAdapter::OnDeviceConnect(const uint8_t dev_type, const char* serial_num, const char* user_data)
{
    if (!instance_) return;
    
    std::cout << "[LivoxAdapter] Device connected: type=" << (int)dev_type 
              << " serial=" << serial_num << std::endl;

    uint8_t handle = 0;
    AddDeviceToConnect(serial_num, &handle);
    instance_->device_handle_ = handle;
    
    // Set data callback
    SetDataCallback(handle, OnDataCallback, instance_);
    instance_->connected_ = true;
}

void LivoxAdapter::OnDeviceChange(const uint8_t dev_type, const char* serial_num, uint8_t state)
{
    if (!instance_) return;
    
    if (state == kConnectStateOff)
    {
        std::cout << "[LivoxAdapter] Device disconnected: " << serial_num << std::endl;
        instance_->connected_ = false;
    }
}

void LivoxAdapter::OnDataCallback(uint8_t handle, LivoxEthPacket* data, uint32_t data_num, void* client_data)
{
    LivoxAdapter* self = (LivoxAdapter*)client_data;
    if (!self || !data || data_num == 0) return;

    // Separate lidar and IMU data based on data_type
    for (uint32_t i = 0; i < data_num; i++)
    {
        const LivoxEthPacket& packet = data[i];
        
        if (packet.data_type == kImu)
        {
            // IMU packet
            LivoxImuPacket* imu_pkt = (LivoxImuPacket*)packet.data;
            ImuData imu;
            // Convert timestamp from ns to seconds
            imu.timestamp = packet.timestamp_point / 1e9;
            imu.gyro = V3D(imu_pkt->gyro[0], imu_pkt->gyro[1], imu_pkt->gyro[2]);
            imu.acc = V3D(imu_pkt->acc[0], imu_pkt->acc[1], imu_pkt->acc[2]);
            
            if (self->imu_cb_)
            {
                self->imu_cb_(imu);
            }
            else
            {
                std::lock_guard<std::mutex> lock(self->imu_mutex_);
                self->imu_buffer_.push_back(imu);
                if (self->imu_buffer_.size() > 10000)
                    self->imu_buffer_.pop_front();
                self->imu_cv_.notify_one();
            }
        }
        else if (packet.data_type == kExtendCartesian || packet.data_type == kCartesian)
        {
            // Lidar point data - batch and send as frame
            // For Livox Horizon, data comes in batches
            // Pass the whole batch to the callback
            double ts = packet.timestamp_point / 1e9; // ns → seconds
            
            if (self->lidar_cb_)
            {
                self->lidar_cb_(&packet, 1, ts);
            }
            else
            {
                std::lock_guard<std::mutex> lock(self->lidar_mutex_);
                LidarFrame frame;
                frame.packets.push_back(packet);
                frame.num = 1;
                frame.timestamp = ts;
                self->lidar_buffer_.push_back(std::move(frame));
                if (self->lidar_buffer_.size() > 100)
                    self->lidar_buffer_.pop_front();
                self->lidar_cv_.notify_one();
            }
        }
    }
}

bool LivoxAdapter::getLidarData(LivoxEthPacket*& data, uint32_t& num, double& timestamp, int timeout_ms)
{
    std::unique_lock<std::mutex> lock(lidar_mutex_);
    if (!lidar_cv_.wait_for(lock, std::chrono::milliseconds(timeout_ms),
        [this]{ return !lidar_buffer_.empty(); }))
        return false;

    auto& frame = lidar_buffer_.front();
    num = frame.num;
    timestamp = frame.timestamp;
    // Note: caller should copy data before next call
    // This is a simplified interface
    lock.unlock();
    lidar_mutex_.lock();
    if (!lidar_buffer_.empty()) {
        lidar_buffer_.pop_front();
    }
    lidar_mutex_.unlock();
    return true;
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
