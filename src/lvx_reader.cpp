#include "lvx_reader.h"
#include <algorithm>
#include <cmath>
#include <iostream>
#include <chrono>
#include <cstring>

namespace {

constexpr double kPi = 3.14159265358979323846;
constexpr size_t kHeaderSize = kLivoxPacketHeaderSize;

template <typename T>
T readScalar(const uint8_t* data) {
    T value{};
    std::memcpy(&value, data, sizeof(T));
    return value;
}

struct PacketLayout {
    int raw_point_count = 0;
    int bytes_per_raw_point = 0;
    bool is_point = false;
    bool is_imu = false;
};

PacketLayout packetLayout(uint8_t data_type) {
    switch (data_type) {
        case kLvxDataTypeCartesian:        return {100, 13, true, false};
        case kLvxDataTypeSpherical:        return {100, 9,  true, false};
        case kLvxDataTypeExtendCartesian:  return {96,  14, true, false};
        case kLvxDataTypeExtendSpherical:  return {96,  10, true, false};
        case kLvxDataTypeDualCartesian:    return {48,  28, true, false};
        case kLvxDataTypeDualSpherical:    return {48,  16, true, false};
        case kLvxDataTypeImu:              return {1,   24, false, true};
        case kLvxDataTypeTripleCartesian:  return {30,  42, true, false};
        case kLvxDataTypeTripleSpherical:  return {30,  22, true, false};
        default:                           return {};
    }
}

uint64_t pointIntervalNs(uint8_t data_type) {
    switch (data_type) {
        case kLvxDataTypeCartesian:
        case kLvxDataTypeSpherical:
            return 1000000000ULL / 100000ULL;
        default:
            return 1000000000ULL / 240000ULL;
    }
}

uint32_t offsetFromFrameStart(uint64_t packet_time_ns, uint64_t frame_start_ns,
                              uint64_t point_index, uint64_t interval_ns) {
    const uint64_t point_time_ns = packet_time_ns + point_index * interval_ns;
    if (point_time_ns <= frame_start_ns) {
        return 0;
    }
    const uint64_t offset = point_time_ns - frame_start_ns;
    return static_cast<uint32_t>(std::min<uint64_t>(offset, UINT32_MAX));
}

LvxPoint makeCartesianPoint(int32_t x_mm, int32_t y_mm, int32_t z_mm,
                            uint8_t reflectivity, uint8_t tag,
                            uint32_t offset_time) {
    LvxPoint point;
    point.x = x_mm / 1000.0f;
    point.y = y_mm / 1000.0f;
    point.z = z_mm / 1000.0f;
    point.reflectivity = reflectivity;
    point.tag = tag;
    point.line = 0;
    point.offset_time = offset_time;
    return point;
}

LvxPoint makeSphericalPoint(uint32_t depth_mm, uint16_t theta_centideg,
                            uint16_t phi_centideg, uint8_t reflectivity,
                            uint8_t tag, uint32_t offset_time) {
    const double depth_m = static_cast<double>(depth_mm) / 1000.0;
    const double theta = static_cast<double>(theta_centideg) * 0.01 * kPi / 180.0;
    const double phi = static_cast<double>(phi_centideg) * 0.01 * kPi / 180.0;

    LvxPoint point;
    point.x = static_cast<float>(depth_m * std::sin(theta) * std::cos(phi));
    point.y = static_cast<float>(depth_m * std::sin(theta) * std::sin(phi));
    point.z = static_cast<float>(depth_m * std::cos(theta));
    point.reflectivity = reflectivity;
    point.tag = tag;
    point.line = 0;
    point.offset_time = offset_time;
    return point;
}

void appendCartesianReturns(uint8_t data_type, const uint8_t* raw,
                            uint64_t packet_time_ns, uint64_t frame_start_ns,
                            uint64_t& point_index, std::vector<LvxPoint>& points) {
    const uint64_t interval_ns = pointIntervalNs(data_type);
    if (data_type == kLvxDataTypeCartesian) {
        points.push_back(makeCartesianPoint(readScalar<int32_t>(raw),
                                            readScalar<int32_t>(raw + 4),
                                            readScalar<int32_t>(raw + 8),
                                            raw[12], 0,
                                            offsetFromFrameStart(packet_time_ns, frame_start_ns, point_index++, interval_ns)));
    } else if (data_type == kLvxDataTypeExtendCartesian) {
        points.push_back(makeCartesianPoint(readScalar<int32_t>(raw),
                                            readScalar<int32_t>(raw + 4),
                                            readScalar<int32_t>(raw + 8),
                                            raw[12], raw[13],
                                            offsetFromFrameStart(packet_time_ns, frame_start_ns, point_index++, interval_ns)));
    } else if (data_type == kLvxDataTypeDualCartesian) {
        points.push_back(makeCartesianPoint(readScalar<int32_t>(raw),
                                            readScalar<int32_t>(raw + 4),
                                            readScalar<int32_t>(raw + 8),
                                            raw[12], raw[13],
                                            offsetFromFrameStart(packet_time_ns, frame_start_ns, point_index++, interval_ns)));
        points.push_back(makeCartesianPoint(readScalar<int32_t>(raw + 14),
                                            readScalar<int32_t>(raw + 18),
                                            readScalar<int32_t>(raw + 22),
                                            raw[26], raw[27],
                                            offsetFromFrameStart(packet_time_ns, frame_start_ns, point_index++, interval_ns)));
    } else if (data_type == kLvxDataTypeTripleCartesian) {
        points.push_back(makeCartesianPoint(readScalar<int32_t>(raw),
                                            readScalar<int32_t>(raw + 4),
                                            readScalar<int32_t>(raw + 8),
                                            raw[12], raw[13],
                                            offsetFromFrameStart(packet_time_ns, frame_start_ns, point_index++, interval_ns)));
        points.push_back(makeCartesianPoint(readScalar<int32_t>(raw + 14),
                                            readScalar<int32_t>(raw + 18),
                                            readScalar<int32_t>(raw + 22),
                                            raw[26], raw[27],
                                            offsetFromFrameStart(packet_time_ns, frame_start_ns, point_index++, interval_ns)));
        points.push_back(makeCartesianPoint(readScalar<int32_t>(raw + 28),
                                            readScalar<int32_t>(raw + 32),
                                            readScalar<int32_t>(raw + 36),
                                            raw[40], raw[41],
                                            offsetFromFrameStart(packet_time_ns, frame_start_ns, point_index++, interval_ns)));
    }
}

void appendSphericalReturns(uint8_t data_type, const uint8_t* raw,
                            uint64_t packet_time_ns, uint64_t frame_start_ns,
                            uint64_t& point_index, std::vector<LvxPoint>& points) {
    const uint64_t interval_ns = pointIntervalNs(data_type);
    if (data_type == kLvxDataTypeSpherical) {
        points.push_back(makeSphericalPoint(readScalar<uint32_t>(raw),
                                            readScalar<uint16_t>(raw + 4),
                                            readScalar<uint16_t>(raw + 6),
                                            raw[8], 0,
                                            offsetFromFrameStart(packet_time_ns, frame_start_ns, point_index++, interval_ns)));
    } else if (data_type == kLvxDataTypeExtendSpherical) {
        points.push_back(makeSphericalPoint(readScalar<uint32_t>(raw),
                                            readScalar<uint16_t>(raw + 4),
                                            readScalar<uint16_t>(raw + 6),
                                            raw[8], raw[9],
                                            offsetFromFrameStart(packet_time_ns, frame_start_ns, point_index++, interval_ns)));
    } else if (data_type == kLvxDataTypeDualSpherical) {
        const uint16_t theta = readScalar<uint16_t>(raw);
        const uint16_t phi = readScalar<uint16_t>(raw + 2);
        points.push_back(makeSphericalPoint(readScalar<uint32_t>(raw + 4), theta, phi,
                                            raw[8], raw[9],
                                            offsetFromFrameStart(packet_time_ns, frame_start_ns, point_index++, interval_ns)));
        points.push_back(makeSphericalPoint(readScalar<uint32_t>(raw + 10), theta, phi,
                                            raw[14], raw[15],
                                            offsetFromFrameStart(packet_time_ns, frame_start_ns, point_index++, interval_ns)));
    } else if (data_type == kLvxDataTypeTripleSpherical) {
        const uint16_t theta = readScalar<uint16_t>(raw);
        const uint16_t phi = readScalar<uint16_t>(raw + 2);
        points.push_back(makeSphericalPoint(readScalar<uint32_t>(raw + 4), theta, phi,
                                            raw[8], raw[9],
                                            offsetFromFrameStart(packet_time_ns, frame_start_ns, point_index++, interval_ns)));
        points.push_back(makeSphericalPoint(readScalar<uint32_t>(raw + 10), theta, phi,
                                            raw[14], raw[15],
                                            offsetFromFrameStart(packet_time_ns, frame_start_ns, point_index++, interval_ns)));
        points.push_back(makeSphericalPoint(readScalar<uint32_t>(raw + 16), theta, phi,
                                            raw[20], raw[21],
                                            offsetFromFrameStart(packet_time_ns, frame_start_ns, point_index++, interval_ns)));
    }
}

bool isCartesianType(uint8_t data_type) {
    return data_type == kLvxDataTypeCartesian ||
           data_type == kLvxDataTypeExtendCartesian ||
           data_type == kLvxDataTypeDualCartesian ||
           data_type == kLvxDataTypeTripleCartesian;
}

bool isSphericalType(uint8_t data_type) {
    return data_type == kLvxDataTypeSpherical ||
           data_type == kLvxDataTypeExtendSpherical ||
           data_type == kLvxDataTypeDualSpherical ||
           data_type == kLvxDataTypeTripleSpherical;
}

} // namespace

LvxReader::LvxReader()
    : total_frames_(0), current_frame_(0), frame_duration_ms_(50), next_read_offset_(0),
      base_time_ns_(0), has_base_time_(false), seen_points_(false), seen_imu_(false),
      unsupported_packet_count_(0), truncated_packet_count_(0)
{
}

LvxReader::~LvxReader()
{
    close();
}

bool LvxReader::open(const std::string& filepath)
{
    filepath_ = filepath;
    file_.open(filepath, std::ios::binary);
    if (!file_.is_open())
    {
        std::cerr << "[LvxReader] Cannot open file: " << filepath << std::endl;
        return false;
    }

    // ── Read Public Header (24 bytes) ──
    LvxPublicHeader pub_header;
    file_.read(reinterpret_cast<char*>(&pub_header), sizeof(pub_header));
    if (!file_)
    {
        std::cerr << "[LvxReader] Failed to read public header" << std::endl;
        file_.close();
        return false;
    }

    // Validate signature
    if (memcmp(pub_header.signature, "livox_tech", 10) != 0)
    {
        std::cerr << "[LvxReader] Invalid signature" << std::endl;
        file_.close();
        return false;
    }

    // Validate magic code
    if (pub_header.magic_code != 0xAC0EA767)
    {
        std::cerr << "[LvxReader] Invalid magic code: 0x"
                  << std::hex << pub_header.magic_code << std::dec << std::endl;
        file_.close();
        return false;
    }

    std::cout << "[LvxReader] LVX version "
              << (int)pub_header.version[0] << "."
              << (int)pub_header.version[1] << "."
              << (int)pub_header.version[2] << std::endl;

    // ── Read Private Header (5 bytes) ──
    file_.read(reinterpret_cast<char*>(&private_header_), sizeof(private_header_));
    if (!file_)
    {
        std::cerr << "[LvxReader] Failed to read private header" << std::endl;
        file_.close();
        return false;
    }

    frame_duration_ms_ = private_header_.frame_duration;
    std::cout << "[LvxReader] Frame duration: " << frame_duration_ms_ << " ms"
              << ", Devices: " << (int)private_header_.device_count << std::endl;

    // ── Read Device Info (59 bytes each) ──
    devices_.resize(private_header_.device_count);
    for (uint8_t i = 0; i < private_header_.device_count; i++)
    {
        file_.read(reinterpret_cast<char*>(&devices_[i]), sizeof(LvxDeviceInfo));
        if (!file_)
        {
            std::cerr << "[LvxReader] Failed to read device info " << (int)i << std::endl;
            file_.close();
            return false;
        }
        std::cout << "[LvxReader] Device " << (int)devices_[i].device_index
                  << " type=" << (int)devices_[i].device_type
                  << " SN=" << (char*)devices_[i].lidar_broadcast_code
                  << " extrinsic=" << (int)devices_[i].extrinsic_enable
                  << " rpy=[" << devices_[i].roll << "," << devices_[i].pitch << "," << devices_[i].yaw << "]"
                  << " xyz=[" << devices_[i].x << "," << devices_[i].y << "," << devices_[i].z << "]"
                  << std::endl;
    }

    // ── First pass: count valid non-empty frames by following offsets ──
    total_frames_ = 0;
    const uint64_t first_frame_offset = static_cast<uint64_t>(file_.tellg());
    file_.seekg(0, std::ios::end);
    const uint64_t file_size = static_cast<uint64_t>(file_.tellg());

    uint64_t current_pos = first_frame_offset;
    while (current_pos != 0 && current_pos + sizeof(LvxFrameHeader) <= file_size)
    {
        LvxFrameHeader fh;
        file_.seekg(current_pos);
        file_.read(reinterpret_cast<char*>(&fh), sizeof(fh));
        if (!file_.good())
        {
            break;
        }

        if (fh.current_offset != current_pos ||
            fh.next_offset < fh.current_offset + sizeof(LvxFrameHeader) ||
            fh.next_offset > file_size)
        {
            break;
        }

        const uint64_t data_size = fh.next_offset - fh.current_offset - sizeof(LvxFrameHeader);
        if (data_size == 0)
        {
            break;
        }
        if (data_size > 10 * 1024 * 1024)
        {
            break;
        }

        total_frames_++;
        current_pos = fh.next_offset;
    }

    if (total_frames_ == 0)
    {
        std::cerr << "[LvxReader] No valid frame data found" << std::endl;
        file_.close();
        return false;
    }

    // Store the offset of the first frame for readFrame to seek to
    file_.clear();
    next_read_offset_ = first_frame_offset;

    current_frame_ = 0;
    eof_ = false;
    base_time_ns_ = 0;
    has_base_time_ = false;
    seen_points_ = false;
    seen_imu_ = false;
    unsupported_packet_count_ = 0;
    truncated_packet_count_ = 0;

    std::cout << "[LvxReader] Opened: " << filepath
              << " (" << total_frames_ << " frames, "
              << (int)private_header_.device_count << " devices)"
              << std::endl;
    return true;
}

void LvxReader::close()
{
    stop();
    if (file_.is_open())
        file_.close();
}

void LvxReader::play(double speed)
{
    if (!file_.is_open()) return;
    stop_flag_ = false;
    paused_ = false;
    playback_speed_ = speed;
    playing_ = true;

    playback_thread_ = std::thread(&LvxReader::playbackThread, this);
}

void LvxReader::setSpeed(double speed)
{
    playback_speed_ = speed;
}

void LvxReader::pause()
{
    paused_ = true;
}

void LvxReader::resume()
{
    paused_ = false;
    pause_cv_.notify_all();
}

void LvxReader::stop()
{
    stop_flag_ = true;
    paused_ = false;
    pause_cv_.notify_all();
    if (playback_thread_.joinable())
        playback_thread_.join();
    playing_ = false;
}

void LvxReader::playbackThread()
{
    auto last_frame_wall = std::chrono::steady_clock::now();

    while (!stop_flag_ && !eof_)
    {
        // Check pause
        {
            std::unique_lock<std::mutex> lock(pause_mutex_);
            pause_cv_.wait(lock, [this]{ return !paused_ || stop_flag_; });
        }
        if (stop_flag_) break;

        if (!readFrame())
        {
            eof_ = true;
            break;
        }
        current_frame_++;

        // Throttle: sleep for frame_duration minus time spent reading
        auto now = std::chrono::steady_clock::now();
        auto read_time = std::chrono::duration<double, std::milli>(now - last_frame_wall).count();
        const double speed = playback_speed_.load();
        const double target_frame_ms =
            speed > 0.0 ? static_cast<double>(frame_duration_ms_) / speed : 0.0;
        double sleep_ms = target_frame_ms - read_time;
        if (sleep_ms > 1.0 && sleep_ms < 500.0)
        {
            std::this_thread::sleep_for(std::chrono::milliseconds((int)sleep_ms));
        }
        last_frame_wall = std::chrono::steady_clock::now();
    }

    playing_ = false;
    if (eof_)
    {
        std::cout << "[LvxReader] Playback finished (EOF)" << std::endl;
        if (!seen_points_)
        {
            std::cerr << "[LvxReader] ERROR: no supported Livox SDK1 LVX v1.1 point packets were parsed."
                      << std::endl;
        }
        if (!seen_imu_)
        {
            std::cerr << "[LvxReader] ERROR: no IMU packets were parsed; FAST-LIO LiDAR/IMU mapping cannot run."
                      << std::endl;
        }
        if (unsupported_packet_count_ > 0 || truncated_packet_count_ > 0)
        {
            std::cerr << "[LvxReader] Packet diagnostics: unsupported="
                      << unsupported_packet_count_
                      << " truncated=" << truncated_packet_count_ << std::endl;
        }
    }
}

bool LvxReader::readFrame()
{
    // Seek to the expected frame position
    file_.clear();
    file_.seekg(next_read_offset_);
    if (!file_.good())
    {
        std::cerr << "[LvxReader] readFrame: seek failed to offset " << next_read_offset_ << std::endl;
        return false;
    }

    // Read frame header (24 bytes)
    LvxFrameHeader fh;
    file_.read(reinterpret_cast<char*>(&fh), sizeof(fh));
    if (!file_.good())
    {
        std::cerr << "[LvxReader] readFrame: failed to read frame header" << std::endl;
        return false;
    }

    // Seek to next frame for the next call
    next_read_offset_ = fh.next_offset;

    // Calculate frame data size
    uint64_t data_start = file_.tellg();
    uint64_t data_size = fh.next_offset - fh.current_offset - sizeof(LvxFrameHeader);

    if (fh.next_offset < fh.current_offset + sizeof(LvxFrameHeader) ||
        data_size > 10 * 1024 * 1024) // sanity check: max 10MB
    {
        std::cerr << "[LvxReader] Invalid frame data size: " << data_size
                  << " cur_off=" << fh.current_offset
                  << " next_off=" << fh.next_offset
                  << " frame_idx=" << fh.frame_index << std::endl;
        return false;
    }
    if (data_size == 0)
    {
        return false;
    }

    // Read entire frame data
    std::vector<uint8_t> frame_data(data_size);
    file_.read(reinterpret_cast<char*>(frame_data.data()), data_size);
    if (!file_.good())
    {
        if (file_.gcount() <= 0) return false;
        frame_data.resize(file_.gcount());
    }

    // Parse frame data into points and IMU samples
    std::vector<LvxPoint> points;
    std::vector<ImuData> imus;
    double frame_time = 0.0;
    parseFrameData(frame_data, base_time_ns_, frame_time, points, imus);

    // Log first few frames
    if (current_frame_ < 3)
    {
        std::cout << "[LvxReader] Frame " << current_frame_
                  << " data_size=" << data_size
                  << " pts=" << points.size()
                  << " imu=" << imus.size()
                  << " t=" << frame_time << "s";
        if (!points.empty())
            std::cout << " pt0=(" << points[0].x << "," << points[0].y << "," << points[0].z << ")";
        std::cout << std::endl;
    }
    else if (current_frame_ % 200 == 0)
    {
        std::cout << "[LvxReader] Frame " << current_frame_
                  << "/" << total_frames_
                  << " pts=" << points.size()
                  << " imu=" << imus.size() << std::endl;
    }

    if (!points.empty() || !imus.empty())
    {
        if (frame_cb_)
            frame_cb_(points, imus, frame_time);
    }

    return true;
}

void LvxReader::parseFrameData(const std::vector<uint8_t>& data,
                                uint64_t& base_time_ns,
                                double& frame_time,
                                std::vector<LvxPoint>& points, std::vector<ImuData>& imus)
{
    const LvxFrameParseResult parsed = parseFrameData(data, false);
    base_time_ns = base_time_ns_;
    frame_time = parsed.frame_time;
    points = parsed.points;
    imus = parsed.imus;
}

LvxFrameParseResult LvxReader::parsePacketsForTest(const std::vector<uint8_t>& data)
{
    return parseFrameData(data, true);
}

LvxFrameParseResult LvxReader::parseFrameData(const std::vector<uint8_t>& data,
                                              bool reset_base_time)
{
    if (reset_base_time)
    {
        base_time_ns_ = 0;
        has_base_time_ = false;
    }

    LvxFrameParseResult result;
    bool frame_time_set = false;
    uint64_t frame_start_ns = 0;
    size_t pos = 0;

    while (pos + kHeaderSize <= data.size())
    {
        const uint8_t data_type = data[pos + 10];
        const uint64_t packet_time_ns = readScalar<uint64_t>(&data[pos + 11]);
        const PacketLayout layout = packetLayout(data_type);

        if (layout.bytes_per_raw_point == 0)
        {
            result.unsupported_packet_count++;
            break;
        }

        const size_t payload_size = static_cast<size_t>(layout.raw_point_count) *
                                    static_cast<size_t>(layout.bytes_per_raw_point);
        const size_t total_packet_size = kHeaderSize + payload_size;

        if (pos + total_packet_size > data.size())
        {
            result.truncated_packet_count++;
            break;
        }

        if (!has_base_time_)
        {
            base_time_ns_ = packet_time_ns;
            has_base_time_ = true;
        }

        const double timestamp = static_cast<double>(
            static_cast<int64_t>(packet_time_ns - base_time_ns_)) / 1e9;

        const uint8_t* payload = &data[pos + kHeaderSize];
        if (layout.is_point)
        {
            if (!frame_time_set)
            {
                frame_start_ns = packet_time_ns;
                result.frame_time = timestamp;
                frame_time_set = true;
            }

            uint64_t point_index = result.points.empty() ? 0 : result.points.back().offset_time;
            point_index = 0;
            for (int i = 0; i < layout.raw_point_count; ++i)
            {
                const uint8_t* raw = payload + static_cast<size_t>(i) * layout.bytes_per_raw_point;
                if (isCartesianType(data_type))
                {
                    appendCartesianReturns(data_type, raw, packet_time_ns, frame_start_ns,
                                           point_index, result.points);
                }
                else if (isSphericalType(data_type))
                {
                    appendSphericalReturns(data_type, raw, packet_time_ns, frame_start_ns,
                                           point_index, result.points);
                }
            }
        }
        else if (layout.is_imu)
        {
            ImuData imu;
            imu.timestamp = timestamp;
            imu.gyro = V3D(readScalar<float>(payload),
                           readScalar<float>(payload + 4),
                           readScalar<float>(payload + 8));
            imu.acc = V3D(readScalar<float>(payload + 12),
                          readScalar<float>(payload + 16),
                          readScalar<float>(payload + 20));
            result.imus.push_back(imu);
        }

        pos += total_packet_size;
    }

    result.has_points = !result.points.empty();
    result.has_imu = !result.imus.empty();
    seen_points_ = seen_points_ || result.has_points;
    seen_imu_ = seen_imu_ || result.has_imu;
    unsupported_packet_count_ += result.unsupported_packet_count;
    truncated_packet_count_ += result.truncated_packet_count;

    return result;
}
