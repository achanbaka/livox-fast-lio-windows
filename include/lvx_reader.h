#ifndef LVX_READER_H
#define LVX_READER_H

#include <string>
#include <fstream>
#include <thread>
#include <mutex>
#include <atomic>
#include <array>
#include <chrono>
#include <condition_variable>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <functional>
#include <vector>
#include "types.h"

// ═══ LVX v1.1 File Format Structures ═══
// Based on Livox LVX Specification v1.1
// All fields are little-endian.

#pragma pack(push, 1)

// Public Header Block (24 bytes)
struct LvxPublicHeader {
    uint8_t  signature[16];     // "livox_tech" + 6 null bytes
    uint8_t  version[4];        // [major, minor, patch, reserved]
    uint32_t magic_code;        // 0xAC0EA767
};

// Private Header Block (5 bytes)
struct LvxPrivateHeader {
    uint32_t frame_duration;    // milliseconds (always 50 for v1.1)
    uint8_t  device_count;
};

// Device Info Block (59 bytes per device)
struct LvxDeviceInfo {
    uint8_t  lidar_broadcast_code[16];
    uint8_t  hub_broadcast_code[16];
    uint8_t  device_index;
    uint8_t  device_type;
    uint8_t  extrinsic_enable;
    float    roll;
    float    pitch;
    float    yaw;
    float    x;
    float    y;
    float    z;
};

// Frame Header (24 bytes)
struct LvxFrameHeader {
    uint64_t current_offset;
    uint64_t next_offset;
    uint64_t frame_index;
};

#pragma pack(pop)

// ─── LVX File Packet Header (LvxBasePackDetail on-disk format) ───
// 19-byte header written by Livox SDK's SaveFrameToLvxFile.
// Note: this differs from the 18-byte LivoxEthPacket UDP header.
static const size_t kLivoxPacketHeaderSize = 19;

// ─── Packet data types (Section 3.3) ───
static const uint8_t kLvxDataTypeCartesian        = 0;   // 100 pts x 13B
static const uint8_t kLvxDataTypeSpherical         = 1;   // 100 pts x  9B
static const uint8_t kLvxDataTypeExtendCartesian   = 2;   //  96 pts x 14B
static const uint8_t kLvxDataTypeExtendSpherical   = 3;   //  96 pts x 10B
static const uint8_t kLvxDataTypeDualCartesian     = 4;   //  48 pts x 28B
static const uint8_t kLvxDataTypeDualSpherical     = 5;   //  48 pts x 16B
static const uint8_t kLvxDataTypeImu               = 6;   //   1  x 24B
static const uint8_t kLvxDataTypeTripleCartesian   = 7;   //  30 pts x 42B (Avia)
static const uint8_t kLvxDataTypeTripleSpherical   = 8;   //  30 pts (Avia)

// Parsed point from lvx
struct LvxPoint {
    float x, y, z;             // meters
    uint8_t reflectivity = 0;  // Livox reflectivity, maps to intensity
    uint8_t line = 0;
    uint8_t tag = 0;
    uint32_t offset_time = 0;  // ns from frame start
};

struct LvxFrameParseResult {
    std::vector<LvxPoint> points;
    std::vector<ImuData> imus;
    double frame_time = 0.0;
    bool has_points = false;
    bool has_imu = false;
    uint32_t unsupported_packet_count = 0;
    uint32_t truncated_packet_count = 0;
};

using LidarFrameCallback = std::function<void(const std::vector<LvxPoint>&, const std::vector<ImuData>&, double)>;

class LvxReader
{
public:
    static constexpr std::size_t kMaxPlaybackErrorLength = 255;

    struct PlaybackStats {
        uint64_t failures = 0;
        bool failed = false;
        bool eof = false;
        bool playing = false;
        bool thread_exited = true;
        // A snapshot of the most recent playback-thread exception, truncated
        // to kMaxPlaybackErrorLength bytes.
        std::string last_error;
    };

    LvxReader();
    ~LvxReader();

    bool open(const std::string& filepath) noexcept;
    void close();
    void play(double speed = 1.0) noexcept;
    void setSpeed(double speed);
    void pause();
    void resume();
    bool seekToTimeNs(uint64_t time_ns);
    // Requests playback cancellation and joins only after the playback thread
    // reports exit. A false result leaves the object alive/stopping so callers
    // can enforce a process-wide shutdown deadline without an unbounded join.
    bool stopFor(std::chrono::milliseconds timeout);
    void stop();
    bool isOpen() const { return file_.is_open(); }
    bool isPlaying() const { return playing_; }
    bool isEOF() const { return eof_; }
    bool hasPlaybackFailure() const noexcept { return playback_failed_.load(); }
    PlaybackStats playbackStats() const;

    void setFrameCallback(LidarFrameCallback cb) { frame_cb_ = cb; }

    uint64_t getTotalFrames() const { return total_frames_; }
    uint64_t getCurrentFrame() const { return current_frame_; }
    uint32_t getFrameDurationMs() const { return frame_duration_ms_; }
    uint64_t getDurationNs() const {
        return total_frames_ * static_cast<uint64_t>(frame_duration_ms_) * 1000000ULL;
    }
    LvxFrameParseResult parsePacketsForTest(const std::vector<uint8_t>& data);

private:
    friend struct LvxReaderTestAccess;

    void playbackThread() noexcept;
    void finishPlaybackThread(bool reached_eof) noexcept;
    void clearPlaybackFailure();
    void recordPlaybackFailure(const char* message) noexcept;
    bool readFrame();
    bool sleepInterruptible(std::chrono::milliseconds duration);
    LvxFrameParseResult parseFrameData(const std::vector<uint8_t>& data,
                                       bool reset_base_time);
    void parseFrameData(const std::vector<uint8_t>& data,
                        uint64_t& base_time_ns,
                        double& frame_time,
                        std::vector<LvxPoint>& points, std::vector<ImuData>& imus);

    std::ifstream file_;
    std::string filepath_;
    uint64_t total_frames_;
    uint64_t current_frame_;
    uint32_t frame_duration_ms_;
    uint64_t next_read_offset_;  // file offset for next readFrame call
    std::vector<uint64_t> frame_offsets_;
    double playback_time_offset_sec_{0.0};
    uint64_t base_time_ns_;      // first packet timestamp (ns) for time alignment
    bool     has_base_time_;
    bool     seen_points_;
    bool     seen_imu_;
    uint32_t unsupported_packet_count_;
    uint32_t truncated_packet_count_;

    LvxPrivateHeader private_header_;
    std::vector<LvxDeviceInfo> devices_;

    std::thread playback_thread_;
    std::atomic<bool> playing_{false};
    std::atomic<bool> paused_{false};
    std::atomic<bool> stop_flag_{false};
    std::atomic<bool> eof_{false};
    std::atomic<bool> playback_failed_{false};
    std::atomic<uint64_t> playback_failure_count_{0};
    std::atomic<double> playback_speed_{1.0};

    LidarFrameCallback frame_cb_;
    std::mutex control_mutex_;
    std::mutex pause_mutex_;
    std::condition_variable pause_cv_;
    mutable std::mutex thread_state_mutex_;
    std::condition_variable thread_exit_cv_;
    std::atomic<bool> playback_thread_exited_{true};
    std::array<char, kMaxPlaybackErrorLength + 1> last_playback_error_{};
};

#endif
