#ifndef BAG_READER_H
#define BAG_READER_H

#include <string>
#include <functional>
#include <atomic>
#include <array>
#include <chrono>
#include <cstddef>
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
    static constexpr std::size_t kMaxPlaybackErrorLength = 255;

    struct PlaybackStats {
        uint64_t failures = 0;
        bool failed = false;
        bool eof = false;
        bool thread_exited = true;
        // A snapshot of the most recent playback-thread exception, truncated
        // to kMaxPlaybackErrorLength bytes.
        std::string last_error;
    };

    BagReader();
    ~BagReader();

    bool open(const std::string& filepath) noexcept;
    void close();

    // Start playback in a background thread
    // speed: 0.0 = as fast as possible, >0 = real-time multiplier
    void play(double speed = 0.0) noexcept;
    void setSpeed(double speed);
    void pause();
    void resume();
    bool seekToTimeNs(uint64_t time_ns);
    // Bounded counterpart to stop(). A timed-out callback remains in flight;
    // the reader must outlive it until a later successful stop or process exit.
    bool stopFor(std::chrono::milliseconds timeout);
    void stop();
    bool isEOF() const { return eof_; }
    bool isOpen() const { return bag_reader_.isOpen(); }
    bool hasPlaybackFailure() const noexcept { return playback_failed_.load(); }
    PlaybackStats playbackStats() const;

    void setFrameCallback(BagFrameCallback cb) { frame_cb_ = cb; }

    uint64_t getTotalMessages() const { return total_messages_; }
    uint64_t getCurrentMessage() const { return current_message_; }
    uint64_t getStartTimeNs() const { return start_time_ns_; }
    uint64_t getEndTimeNs() const { return end_time_ns_; }

private:
    struct TimedMessage {
        uint32_t conn_id = 0;
        uint64_t bag_time_ns = 0;
        double sensor_time = 0.0;
        std::vector<uint8_t> data;
    };

    bool loadMessages() noexcept;
    void playbackThread() noexcept;
    void finishPlaybackThread(bool reached_eof) noexcept;
    void clearPlaybackFailure();
    void recordPlaybackFailure(const char* message) noexcept;
    bool waitWhilePaused();
    bool sleepInterruptible(std::chrono::milliseconds duration);

    bag::BagFileReader bag_reader_;
    std::string filepath_;

    // Connection IDs for Livox topics
    int lidar_conn_id_ = -1;
    int imu_conn_id_ = -1;

    BagFrameCallback frame_cb_;

    std::thread playback_thread_;
    std::atomic<bool> stop_flag_{false};
    std::atomic<bool> eof_{false};
    std::atomic<bool> playback_failed_{false};
    std::atomic<uint64_t> playback_failure_count_{0};
    std::atomic<uint64_t> total_messages_{0};
    std::atomic<uint64_t> current_message_{0};
    std::atomic<uint64_t> start_time_ns_{0};
    std::atomic<uint64_t> end_time_ns_{0};

    std::atomic<double> playback_speed_{0.0};
    std::atomic<bool> paused_{false};
    std::vector<TimedMessage> messages_;
    bool messages_loaded_ = false;
    size_t playback_index_ = 0;
    bool timing_reset_requested_ = true;
    std::mutex control_mutex_;
    std::mutex pause_mutex_;
    std::condition_variable pause_cv_;
    mutable std::mutex thread_state_mutex_;
    std::condition_variable thread_exit_cv_;
    std::atomic<bool> playback_thread_exited_{true};
    std::array<char, kMaxPlaybackErrorLength + 1> last_playback_error_{};
};

#endif // BAG_READER_H
