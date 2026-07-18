#include "bag_reader.h"
#include <algorithm>
#include <iostream>
#include <chrono>
#include <exception>
#include <limits>
#include <stdexcept>
#include <utility>

namespace {

template <typename Function>
class ScopeExit {
public:
    explicit ScopeExit(Function function) : function_(std::move(function)) {}
    ScopeExit(const ScopeExit&) = delete;
    ScopeExit& operator=(const ScopeExit&) = delete;
    ~ScopeExit() noexcept { function_(); }

private:
    Function function_;
};

template <typename Function>
ScopeExit<Function> makeScopeExit(Function function) {
    return ScopeExit<Function>(std::move(function));
}

} // namespace

BagReader::BagReader() {}

BagReader::PlaybackStats BagReader::playbackStats() const {
    PlaybackStats result;
    std::lock_guard<std::mutex> lock(thread_state_mutex_);
    result.failures = playback_failure_count_.load();
    result.failed = playback_failed_.load();
    result.eof = eof_.load();
    result.thread_exited = playback_thread_exited_.load();
    size_t length = 0;
    while (length < kMaxPlaybackErrorLength && last_playback_error_[length] != '\0') {
        ++length;
    }
    result.last_error.assign(last_playback_error_.data(), length);
    return result;
}

void BagReader::clearPlaybackFailure() {
    std::lock_guard<std::mutex> lock(thread_state_mutex_);
    last_playback_error_[0] = '\0';
    playback_failed_ = false;
}

void BagReader::recordPlaybackFailure(const char* message) noexcept {
    try {
        std::lock_guard<std::mutex> lock(thread_state_mutex_);
        const char* source = message ? message : "unknown playback exception";
        size_t length = 0;
        while (length < kMaxPlaybackErrorLength && source[length] != '\0') {
            last_playback_error_[length] = source[length];
            ++length;
        }
        last_playback_error_[length] = '\0';
        playback_failure_count_.fetch_add(1);
        playback_failed_ = true;
    } catch (...) {
        // Failure reporting must never escape the std::thread exception
        // boundary. The atomic failure fields remain available even if the
        // diagnostic mutex itself cannot be acquired.
        playback_failure_count_.fetch_add(1);
        playback_failed_ = true;
    }
}

void BagReader::finishPlaybackThread(bool reached_eof) noexcept {
    try {
        std::lock_guard<std::mutex> lock(thread_state_mutex_);
        eof_ = reached_eof && !playback_failed_.load();
        playback_thread_exited_ = true;
    } catch (...) {
        // The flag is atomic so stopFor() can still observe completion even
        // in the exceptional mutex-failure path.
        eof_ = reached_eof && !playback_failed_.load();
        playback_thread_exited_ = true;
    }
    thread_exit_cv_.notify_all();
}

BagReader::~BagReader() {
    stop();
    close();
}

bool BagReader::open(const std::string& filepath) noexcept
try {
    filepath_ = filepath;

    if (!bag_reader_.open(filepath)) {
        return false;
    }

    // Find Livox topics
    // Common topic names used by livox_ros_driver:
    //   /livox/lidar  or  /livox_lidar  → CustomMsg
    //   /livox/imu    or  /livox_imu    → sensor_msgs/Imu
    const char* lidar_topics[] = {
        "/livox/lidar", "/livox_lidar", "/livox/custom_lidar"
    };
    const char* imu_topics[] = {
        "/livox/imu", "/livox_imu"
    };

    for (const auto& topic : lidar_topics) {
        lidar_conn_id_ = bag_reader_.findConnectionByTopic(topic);
        if (lidar_conn_id_ >= 0) {
            std::cout << "[BagReader] Found LiDAR topic: " << topic << std::endl;
            break;
        }
    }

    for (const auto& topic : imu_topics) {
        imu_conn_id_ = bag_reader_.findConnectionByTopic(topic);
        if (imu_conn_id_ >= 0) {
            std::cout << "[BagReader] Found IMU topic: " << topic << std::endl;
            break;
        }
    }

    if (lidar_conn_id_ < 0) {
        std::cerr << "[BagReader] WARNING: No LiDAR topic found! Available topics:" << std::endl;
        for (const auto& pair : bag_reader_.connections()) {
            std::cerr << "  " << pair.second.topic << " (" << pair.second.type << ")" << std::endl;
        }
    }

    // Debug: show all available connections
    std::cout << "[BagReader] Available connections:" << std::endl;
    for (const auto& pair : bag_reader_.connections()) {
        std::cout << "  conn_id=" << pair.first << " topic=" << pair.second.topic
                  << " type=" << pair.second.type << std::endl;
    }
    std::cout << "[BagReader] LiDAR conn_id=" << lidar_conn_id_
              << " IMU conn_id=" << imu_conn_id_ << std::endl;

    if (!loadMessages()) {
        if (playback_failed_.load()) {
            bag_reader_.close();
            return false;
        }
        uint64_t start_ns = std::numeric_limits<uint64_t>::max();
        uint64_t end_ns = 0;
        for (const auto& chunk : bag_reader_.chunkInfos()) {
            if (chunk.start_time > 0) start_ns = std::min(start_ns, chunk.start_time);
            end_ns = std::max(end_ns, chunk.end_time);
        }
        if (start_ns != std::numeric_limits<uint64_t>::max() && end_ns >= start_ns) {
            start_time_ns_ = start_ns;
            end_time_ns_ = end_ns;
        }
    }

    return true;
} catch (const std::exception& error) {
    recordPlaybackFailure(error.what());
    try {
        bag_reader_.close();
        std::cerr << "[BagReader] Open failed: " << error.what() << std::endl;
    } catch (...) {
    }
    return false;
} catch (...) {
    recordPlaybackFailure("unknown exception while opening ROS bag");
    try {
        bag_reader_.close();
        std::cerr << "[BagReader] Open failed with an unknown exception"
                  << std::endl;
    } catch (...) {
    }
    return false;
}

void BagReader::close() {
    stop();
    bag_reader_.close();
}

void BagReader::play(double speed) noexcept {
    try {
    if (!bag_reader_.isOpen()) return;
    if (!messages_loaded_ && !loadMessages()) return;
    stop();
    stop_flag_ = false;
    paused_ = false;
    eof_ = false;
    clearPlaybackFailure();
    playback_speed_ = speed;
    timing_reset_requested_ = true;
    {
        std::lock_guard<std::mutex> lock(thread_state_mutex_);
        playback_thread_exited_ = false;
    }
    try {
        playback_thread_ = std::thread(&BagReader::playbackThread, this);
    } catch (const std::exception& error) {
        recordPlaybackFailure(error.what());
        finishPlaybackThread(false);
    } catch (...) {
        recordPlaybackFailure("unknown exception while starting playback thread");
        finishPlaybackThread(false);
    }
    } catch (const std::exception& error) {
        recordPlaybackFailure(error.what());
        finishPlaybackThread(false);
    } catch (...) {
        recordPlaybackFailure("unknown exception while preparing ROS bag playback");
        finishPlaybackThread(false);
    }
}

void BagReader::setSpeed(double speed) {
    playback_speed_ = speed;
    pause_cv_.notify_all();
}

void BagReader::pause() {
    paused_ = true;
    pause_cv_.notify_all();
}

void BagReader::resume() {
    paused_ = false;
    pause_cv_.notify_all();
}

bool BagReader::seekToTimeNs(uint64_t time_ns) {
    if (!messages_loaded_ && !loadMessages()) {
        return false;
    }
    if (messages_.empty()) {
        return false;
    }

    const double target_sec = static_cast<double>(time_ns) / 1e9;
    auto it = std::lower_bound(
        messages_.begin(), messages_.end(), target_sec,
        [](const TimedMessage& msg, double value) {
            return msg.sensor_time < value;
        });
    size_t index = static_cast<size_t>(std::distance(messages_.begin(), it));
    if (index >= messages_.size()) {
        index = messages_.size() - 1;
    }

    {
        std::lock_guard<std::mutex> lock(control_mutex_);
        playback_index_ = index;
        current_message_ = index;
        eof_ = false;
        timing_reset_requested_ = true;
    }
    pause_cv_.notify_all();
    return true;
}

bool BagReader::stopFor(std::chrono::milliseconds timeout) {
    stop_flag_ = true;
    paused_ = false;
    pause_cv_.notify_all();
    if (!playback_thread_.joinable()) return true;
    {
        std::unique_lock<std::mutex> lock(thread_state_mutex_);
        if (!thread_exit_cv_.wait_for(lock, timeout, [this] {
                return playback_thread_exited_.load();
            })) {
            return false;
        }
    }
    playback_thread_.join();
    return true;
}

void BagReader::stop() {
    stop_flag_ = true;
    paused_ = false;
    pause_cv_.notify_all();
    if (playback_thread_.joinable()) playback_thread_.join();
}

bool BagReader::loadMessages() noexcept {
    try {
        std::lock_guard<std::mutex> lock(control_mutex_);
        if (messages_loaded_) {
            return true;
        }

        std::cout << "[BagReader] Reading all messages from bag..." << std::endl;

        const bool read_ok = bag_reader_.readAllMessages(
            [&](uint32_t conn_id, uint64_t time_ns, const uint8_t* data, size_t len) {
                TimedMessage tm;
                tm.conn_id = conn_id;
                tm.bag_time_ns = time_ns;
                tm.data.assign(data, data + len);

                // Extract sensor timestamp from message. This is intentionally
                // inside the no-throw boundary: malformed point counts may
                // throw length_error/bad_alloc while resizing.
                RosDeserializer deserializer(tm.data);
                if (static_cast<int32_t>(conn_id) == lidar_conn_id_) {
                    LivoxCustomMsg msg = LivoxCustomMsg::deserialize(deserializer);
                    tm.sensor_time = msg.getTimestamp();
                } else if (static_cast<int32_t>(conn_id) == imu_conn_id_) {
                    RosImu msg = RosImu::deserialize(deserializer);
                    if (!deserializer.good()) {
                        throw std::runtime_error(
                            "truncated sensor_msgs/Imu message");
                    }
                    tm.sensor_time = msg.getTimestamp();
                } else {
                    tm.sensor_time = 0.0;
                }

                messages_.push_back(std::move(tm));
            });
        if (!read_ok) {
            throw std::runtime_error("failed to read ROS bag messages");
        }

        total_messages_ = messages_.size();
        std::cout << "[BagReader] Total messages: " << messages_.size() << std::endl;

        // Sort by SENSOR timestamp (not bag timestamp)
        std::sort(messages_.begin(), messages_.end(),
                  [](const TimedMessage& a, const TimedMessage& b) {
                      return a.sensor_time < b.sensor_time;
                  });

        if (!messages_.empty()) {
            start_time_ns_ = static_cast<uint64_t>(
                std::max(0.0, messages_.front().sensor_time) * 1e9);
            end_time_ns_ = static_cast<uint64_t>(
                std::max(0.0, messages_.back().sensor_time) * 1e9);
        }
        playback_index_ = 0;
        messages_loaded_ = true;
        return !messages_.empty();
    } catch (const std::exception& error) {
        messages_.clear();
        messages_loaded_ = false;
        total_messages_ = 0;
        playback_index_ = 0;
        recordPlaybackFailure(error.what());
        try {
            std::cerr << "[BagReader] Failed to load messages: "
                      << error.what() << std::endl;
        } catch (...) {
        }
        return false;
    } catch (...) {
        messages_.clear();
        messages_loaded_ = false;
        total_messages_ = 0;
        playback_index_ = 0;
        recordPlaybackFailure("unknown exception while loading ROS bag");
        try {
            std::cerr << "[BagReader] Failed to load messages with an unknown exception"
                      << std::endl;
        } catch (...) {
        }
        return false;
    }
}

bool BagReader::waitWhilePaused() {
    std::unique_lock<std::mutex> lock(pause_mutex_);
    pause_cv_.wait(lock, [this]{ return !paused_.load() || stop_flag_.load(); });
    return !stop_flag_.load();
}

bool BagReader::sleepInterruptible(std::chrono::milliseconds duration) {
    std::unique_lock<std::mutex> lock(pause_mutex_);
    pause_cv_.wait_for(lock, duration);
    return !stop_flag_.load();
}

void BagReader::playbackThread() noexcept {
    bool reached_eof = false;
    auto exit_guard = makeScopeExit([this, &reached_eof]() noexcept {
        finishPlaybackThread(reached_eof);
    });

    try {
        if (!messages_loaded_ && !loadMessages()) {
            reached_eof = true;
            return;
        }

        // Second pass: dispatch messages with optional timing
        auto wall_start = std::chrono::steady_clock::now();
        double bag_start_sec = messages_.empty() ? 0.0 : messages_.front().sensor_time;

        // Accumulate LiDAR points for current frame
        std::vector<LvxPoint> current_points;
        double current_frame_time = 0.0;
        bool has_frame = false;
        int lidar_msg_count = 0;
        int imu_msg_count = 0;

        size_t i = 0;
        while (!stop_flag_) {
            if (!waitWhilePaused()) break;

            {
                std::lock_guard<std::mutex> lock(control_mutex_);
                i = playback_index_;
                if (i >= messages_.size()) {
                    reached_eof = true;
                    break;
                }
                if (timing_reset_requested_) {
                    wall_start = std::chrono::steady_clock::now();
                    bag_start_sec = messages_[i].sensor_time;
                    current_points.clear();
                    current_frame_time = 0.0;
                    has_frame = false;
                    timing_reset_requested_ = false;
                }
                current_message_ = i + 1;
            }

            const auto& msg = messages_[i];

            if (static_cast<int32_t>(msg.conn_id) == lidar_conn_id_) {
                lidar_msg_count++;
                // Parse CustomMsg
                RosDeserializer deserializer(msg.data);
                LivoxCustomMsg custom_msg = LivoxCustomMsg::deserialize(deserializer);

                double msg_time = custom_msg.getTimestamp();

                // Convert CustomMsg points to LvxPoints
                // Check if this is a new frame (time gap > 1ms)
                if (has_frame && (msg_time - current_frame_time) > 0.001) {
                    // Dispatch accumulated frame
                    if (!current_points.empty() && frame_cb_) {
                        frame_cb_(current_points, {}, current_frame_time);
                    }
                    current_points.clear();
                }

                if (!has_frame) {
                    current_frame_time = msg_time;
                    has_frame = true;
                }

                for (const auto& pt : custom_msg.points) {
                    LvxPoint lp;
                    lp.x = pt.x;
                    lp.y = pt.y;
                    lp.z = pt.z;
                    lp.reflectivity = pt.reflectivity;
                    lp.tag = pt.tag;
                    lp.line = pt.line;
                    lp.offset_time = pt.offset_time;
                    current_points.push_back(lp);
                }

                current_frame_time = msg_time;
            }
            else if (static_cast<int32_t>(msg.conn_id) == imu_conn_id_) {
                imu_msg_count++;
                // Parse Imu
                RosDeserializer deserializer(msg.data);
                RosImu imu_msg = RosImu::deserialize(deserializer);
                ImuData imu = imu_msg.toImuData();

                // Debug: log first few IMU messages
                static int imu_log_count = 0;
                if (imu_log_count < 5) {
                    std::cout << "[BagReader] IMU msg #" << imu_log_count
                              << " t=" << imu.timestamp
                              << " gyro=(" << imu.gyro[0] << "," << imu.gyro[1] << "," << imu.gyro[2] << ")"
                              << " acc=(" << imu.acc[0] << "," << imu.acc[1] << "," << imu.acc[2] << ")"
                              << std::endl;
                    imu_log_count++;
                }

                // Dispatch IMU sample (empty LiDAR points)
                if (frame_cb_) {
                    std::vector<LvxPoint> empty_points;
                    std::vector<ImuData> imu_samples = {imu};
                    frame_cb_(empty_points, imu_samples, imu.timestamp);
                }
            }

            {
                std::lock_guard<std::mutex> lock(control_mutex_);
                if (playback_index_ == i) {
                    playback_index_ = i + 1;
                }
            }

            // Throttle for real-time playback
            const double playback_speed = playback_speed_.load();
            if (playback_speed > 0.0) {
                double elapsed_bag_sec = msg.sensor_time - bag_start_sec;
                auto wall_elapsed = std::chrono::steady_clock::now() - wall_start;
                double wall_elapsed_sec = std::chrono::duration<double>(wall_elapsed).count();
                double target_sec = elapsed_bag_sec / playback_speed;
                if (target_sec > wall_elapsed_sec) {
                    double sleep_ms = (target_sec - wall_elapsed_sec) * 1000.0;
                    if (sleep_ms > 0.1 && sleep_ms < 1000.0) {
                        if (!sleepInterruptible(std::chrono::milliseconds(static_cast<int>(sleep_ms)))) {
                            break;
                        }
                    }
                }
            }

            // Progress logging
            if (i > 0 && i % 5000 == 0) {
                std::cout << "[BagReader] Message " << i << "/" << messages_.size() << std::endl;
            }
        }

        // Dispatch last accumulated frame only for natural completion.
        if (reached_eof && !current_points.empty() && frame_cb_) {
            frame_cb_(current_points, {}, current_frame_time);
        }

        if (reached_eof) {
            std::cout << "[BagReader] Playback finished (" << messages_.size() << " messages, "
                      << lidar_msg_count << " LiDAR, " << imu_msg_count << " IMU)" << std::endl;
        }
    } catch (const std::exception& error) {
        reached_eof = false;
        recordPlaybackFailure(error.what());
        try {
            std::cerr << "[BagReader] Playback failed: " << error.what() << std::endl;
        } catch (...) {
        }
    } catch (...) {
        reached_eof = false;
        recordPlaybackFailure("unknown playback exception");
        try {
            std::cerr << "[BagReader] Playback failed with an unknown exception" << std::endl;
        } catch (...) {
        }
    }
}
