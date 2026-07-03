#include "bag_reader.h"
#include <algorithm>
#include <iostream>
#include <chrono>
#include <limits>

BagReader::BagReader() {}

BagReader::~BagReader() {
    stop();
    close();
}

bool BagReader::open(const std::string& filepath) {
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

    return true;
}

void BagReader::close() {
    stop();
    bag_reader_.close();
}

void BagReader::play(double speed) {
    if (!bag_reader_.isOpen()) return;
    stop_flag_ = false;
    paused_ = false;
    eof_ = false;
    playback_speed_ = speed;
    playback_thread_ = std::thread(&BagReader::playbackThread, this);
}

void BagReader::setSpeed(double speed) {
    playback_speed_ = speed;
}

void BagReader::pause() {
    paused_ = true;
}

void BagReader::resume() {
    paused_ = false;
    pause_cv_.notify_all();
}

void BagReader::stop() {
    stop_flag_ = true;
    paused_ = false;
    pause_cv_.notify_all();
    if (playback_thread_.joinable())
        playback_thread_.join();
}

void BagReader::playbackThread() {
    // First pass: collect all messages and extract sensor timestamps
    struct TimedMessage {
        uint32_t conn_id;
        uint64_t bag_time_ns;  // Bag recording timestamp
        double sensor_time;    // Sensor timestamp (seconds)
        std::vector<uint8_t> data;
    };

    std::vector<TimedMessage> messages;

    std::cout << "[BagReader] Reading all messages from bag..." << std::endl;

    bag_reader_.readAllMessages(
        [&](uint32_t conn_id, uint64_t time_ns, const uint8_t* data, size_t len) {
            if (stop_flag_) return;

            TimedMessage tm;
            tm.conn_id = conn_id;
            tm.bag_time_ns = time_ns;
            tm.data.assign(data, data + len);

            // Extract sensor timestamp from message
            RosDeserializer deserializer(tm.data);
            if (static_cast<int32_t>(conn_id) == lidar_conn_id_) {
                LivoxCustomMsg msg = LivoxCustomMsg::deserialize(deserializer);
                tm.sensor_time = msg.getTimestamp();
            } else if (static_cast<int32_t>(conn_id) == imu_conn_id_) {
                RosImu msg = RosImu::deserialize(deserializer);
                tm.sensor_time = msg.getTimestamp();
            } else {
                tm.sensor_time = 0.0;
            }

            messages.push_back(std::move(tm));
        });

    if (stop_flag_) return;

    total_messages_ = messages.size();
    std::cout << "[BagReader] Total messages: " << messages.size() << std::endl;

    // Sort by SENSOR timestamp (not bag timestamp)
    std::sort(messages.begin(), messages.end(),
              [](const TimedMessage& a, const TimedMessage& b) {
                  return a.sensor_time < b.sensor_time;
              });

    // Second pass: dispatch messages with optional timing
    auto wall_start = std::chrono::steady_clock::now();
    double bag_start_sec = messages.empty() ? 0.0 : messages.front().sensor_time;

    // Accumulate LiDAR points for current frame
    std::vector<LvxPoint> current_points;
    double current_frame_time = 0.0;
    bool has_frame = false;
    int lidar_msg_count = 0;
    int imu_msg_count = 0;

    for (size_t i = 0; i < messages.size() && !stop_flag_; i++) {
        {
            std::unique_lock<std::mutex> lock(pause_mutex_);
            pause_cv_.wait(lock, [this]{ return !paused_ || stop_flag_; });
        }
        if (stop_flag_) break;

        const auto& msg = messages[i];
        current_message_ = i + 1;

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
                    std::this_thread::sleep_for(std::chrono::milliseconds(static_cast<int>(sleep_ms)));
                }
            }
        }

        // Progress logging
        if (i > 0 && i % 5000 == 0) {
            std::cout << "[BagReader] Message " << i << "/" << messages.size() << std::endl;
        }
    }

    // Dispatch last accumulated frame
    if (!current_points.empty() && frame_cb_) {
        frame_cb_(current_points, {}, current_frame_time);
    }

    eof_ = true;
    std::cout << "[BagReader] Playback finished (" << messages.size() << " messages, "
              << lidar_msg_count << " LiDAR, " << imu_msg_count << " IMU)" << std::endl;
}
