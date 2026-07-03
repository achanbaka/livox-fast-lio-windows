#ifndef BAG_READER_H
#define BAG_READER_H

#include <string>
#include <functional>
#include <atomic>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <vector>
#include "types.h"
#include "ros_bag.h"
#include "ros_message.h"

// Reuse LvxPoint for compatibility with existing SLAM pipeline
#include "lvx_reader.h"

using BagFrameCallback = std::function<void(const std::vector<LvxPoint>&,
                                            const std::vector<ImuData>&,
                                            double)>;

/**
 * BagReader — reads a ROS1 bag file containing Livox CustomMsg + Imu data,
 * and provides the same callback interface as LvxReader for the SLAM pipeline.
 */
class BagReader {
public:
    BagReader();
    ~BagReader();

    bool open(const std::string& filepath);
    void close();

    // Start playback in a background thread
    // speed: 0.0 = as fast as possible, >0 = real-time multiplier
    void play(double speed = 0.0);
    void setSpeed(double speed);
    void pause();
    void resume();
    void stop();
    bool isEOF() const { return eof_; }
    bool isOpen() const { return bag_reader_.isOpen(); }

    void setFrameCallback(BagFrameCallback cb) { frame_cb_ = cb; }

    uint64_t getTotalMessages() const { return total_messages_; }
    uint64_t getCurrentMessage() const { return current_message_; }
    uint64_t getStartTimeNs() const { return start_time_ns_; }
    uint64_t getEndTimeNs() const { return end_time_ns_; }

private:
    void playbackThread();

    bag::BagFileReader bag_reader_;
    std::string filepath_;

    // Connection IDs for Livox topics
    int lidar_conn_id_ = -1;
    int imu_conn_id_ = -1;

    BagFrameCallback frame_cb_;

    std::thread playback_thread_;
    std::atomic<bool> stop_flag_{false};
    std::atomic<bool> eof_{false};
    std::atomic<uint64_t> total_messages_{0};
    std::atomic<uint64_t> current_message_{0};
    std::atomic<uint64_t> start_time_ns_{0};
    std::atomic<uint64_t> end_time_ns_{0};

    std::atomic<double> playback_speed_{0.0};
    std::atomic<bool> paused_{false};
    std::mutex pause_mutex_;
    std::condition_variable pause_cv_;
};

#endif // BAG_READER_H
