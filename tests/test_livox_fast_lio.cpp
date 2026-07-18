#include <cassert>
#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <limits>
#include <map>
#include <mutex>
#include <stdexcept>
#include <string>
#include <system_error>
#include <thread>
#include <vector>

#include <pcl/io/pcd_io.h>

#ifdef near
#undef near
#endif

#include "async_map_publisher.h"
#include "bag_reader.h"
#include "bag_writer.h"
#include "bounded_timing_stats.h"
#include "foxglove_output_worker.h"
#include "foxglove_publisher.h"
#include "fast_lio_observation.h"
#include "lidar_imu_sync.h"
#include "livox_adapter.h"
#include "livox_sdk.h"
#include "livox_point_utils.h"
#include "lvx_reader.h"
#include "map_accumulator.h"
#include "map_build_worker.h"
#include "playback_status.h"
#include "ros_bag.h"
#include "storage_worker.h"
#include "tiled_map_store.h"
#include "yaml_config.h"

struct LivoxAdapterTestAccess {
    static void armCallbacks(LivoxAdapter& adapter) {
        std::lock_guard<std::mutex> lock(adapter.callback_mutex_);
        adapter.callback_failed_ = false;
        adapter.accepting_callbacks_ = true;
        adapter.running_ = true;
    }

    static void dispatchData(LivoxAdapter& adapter,
                             LivoxEthPacket& packet,
                             uint32_t count = 1) noexcept {
        LivoxAdapter::OnDataCallback(0, &packet, count, &adapter);
    }

    static size_t callbacksInFlight(const LivoxAdapter& adapter) noexcept {
        return adapter.callbacks_in_flight_.load();
    }
};

struct LvxReaderTestAccess {
    static bool truncateBackingFileAfterOpen(LvxReader& reader,
                                             const char* path,
                                             uint64_t size) {
        reader.file_.close();
        std::error_code error;
        std::filesystem::resize_file(path, size, error);
        if (error) return false;
        reader.file_.clear();
        reader.file_.open(path, std::ios::binary);
        return reader.file_.is_open();
    }
};

static int expectAt(bool condition, const char* expression, int line) {
    if (!condition) {
        std::cerr << "Expectation failed at line " << line << ": " << expression << std::endl;
        return 1;
    }
    return 0;
}

#define expect(condition) expectAt((condition), #condition, __LINE__)

static int testBoundedTimingWindowKeepsLatestSamples() {
    BoundedTimingWindow<4> timing;
    timing.record(1.0);
    timing.record(2.0);
    timing.record(3.0);
    timing.record(4.0);
    timing.record(5.0);
    timing.record(6.0);
    timing.record(-1.0);
    timing.record(std::numeric_limits<double>::infinity());
    const auto summary = timing.summary();
    if (expect(BoundedTimingWindow<4>::capacity() == 4)) return 1;
    if (expect(summary.total_samples == 6)) return 1;
    if (expect(summary.window_samples == 4)) return 1;
    if (expect(std::abs(summary.average_ms - 4.5) < 1e-12)) return 1;
    if (expect(summary.p95_ms == 6.0)) return 1;
    if (expect(summary.p99_ms == 6.0)) return 1;
    return 0;
}

static int testPlaybackTerminalStatusAndLogSanitization() {
    if (expect(!isPlaybackTerminal(false, false))) return 1;
    if (expect(isPlaybackTerminal(true, false))) return 1;
    if (expect(isPlaybackTerminal(false, true))) return 1;
    if (expect(isPlaybackTerminal(true, true))) return 1;
    if (expect(sanitizePlaybackErrorForLog("bad\n error=value") ==
               "bad__error_value")) return 1;
    if (expect(sanitizePlaybackErrorForLog("") == "unknown")) return 1;
    if (expect(sanitizePlaybackErrorForLog(std::string(400, 'x')).size() ==
               255)) return 1;
    return 0;
}

template <typename T>
static void appendScalar(std::vector<uint8_t>& bytes, T value) {
    const uint8_t* raw = reinterpret_cast<const uint8_t*>(&value);
    bytes.insert(bytes.end(), raw, raw + sizeof(T));
}

static void appendPacketHeader(std::vector<uint8_t>& bytes, uint8_t data_type, uint64_t timestamp_ns) {
    bytes.push_back(0);  // device_index
    bytes.push_back(5);  // version
    bytes.push_back(0);  // port_id
    bytes.push_back(0);  // lidar_index
    bytes.push_back(0);  // rsvd
    appendScalar<uint32_t>(bytes, 0);
    bytes.push_back(0);  // timestamp_type
    bytes.push_back(data_type);
    appendScalar<uint64_t>(bytes, timestamp_ns);
}

static void appendCartesian(std::vector<uint8_t>& bytes, int32_t x, int32_t y, int32_t z,
                            uint8_t reflectivity) {
    appendScalar<int32_t>(bytes, x);
    appendScalar<int32_t>(bytes, y);
    appendScalar<int32_t>(bytes, z);
    bytes.push_back(reflectivity);
}

static void appendExtendedCartesian(std::vector<uint8_t>& bytes, int32_t x, int32_t y, int32_t z,
                                    uint8_t reflectivity, uint8_t tag) {
    appendCartesian(bytes, x, y, z, reflectivity);
    bytes.push_back(tag);
}

static void appendSpherical(std::vector<uint8_t>& bytes, uint32_t depth, uint16_t theta,
                            uint16_t phi, uint8_t reflectivity) {
    appendScalar<uint32_t>(bytes, depth);
    appendScalar<uint16_t>(bytes, theta);
    appendScalar<uint16_t>(bytes, phi);
    bytes.push_back(reflectivity);
}

static void appendExtendedSpherical(std::vector<uint8_t>& bytes, uint32_t depth, uint16_t theta,
                                    uint16_t phi, uint8_t reflectivity, uint8_t tag) {
    appendSpherical(bytes, depth, theta, phi, reflectivity);
    bytes.push_back(tag);
}

static void appendPacket(std::vector<uint8_t>& frame, uint8_t data_type, uint64_t timestamp_ns,
                         const std::vector<uint8_t>& payload) {
    appendPacketHeader(frame, data_type, timestamp_ns);
    frame.insert(frame.end(), payload.begin(), payload.end());
}

static bool near(float actual, float expected, float epsilon = 0.0001f) {
    return std::fabs(actual - expected) <= epsilon;
}

static int testLivoxSdkStubMatchesOfficialLayout() {
    if (expect(kCartesian == 0)) return 1;
    if (expect(kSpherical == 1)) return 1;
    if (expect(kExtendCartesian == 2)) return 1;
    if (expect(kExtendSpherical == 3)) return 1;
    if (expect(kDualExtendCartesian == 4)) return 1;
    if (expect(kDualExtendSpherical == 5)) return 1;
    if (expect(kImu == 6)) return 1;
    if (expect(kTripleExtendCartesian == 7)) return 1;
    if (expect(kTripleExtendSpherical == 8)) return 1;

    if (expect(sizeof(LivoxRawPoint) == 13)) return 1;
    if (expect(sizeof(LivoxSpherPoint) == 9)) return 1;
    if (expect(sizeof(LivoxExtendRawPoint) == 14)) return 1;
    if (expect(sizeof(LivoxExtendSpherPoint) == 10)) return 1;
    if (expect(sizeof(LivoxDualExtendRawPoint) == 28)) return 1;
    if (expect(sizeof(LivoxDualExtendSpherPoint) == 16)) return 1;
    if (expect(sizeof(LivoxTripleExtendRawPoint) == 42)) return 1;
    if (expect(sizeof(LivoxTripleExtendSpherPoint) == 22)) return 1;
    if (expect(sizeof(LivoxImuPoint) == 24)) return 1;
    return 0;
}

static int testFoxglovePublishesImuJson() {
    FoxglovePublisher publisher;
    if (expect(publisher.start("127.0.0.1", 0, 64))) return 1;
    if (expect(publisher.getPort() != 0)) return 1;

    ImuData imu;
    imu.timestamp = 123.005;
    imu.gyro = V3D(0.1, -0.2, 0.3);
    imu.acc = V3D(1.0, 2.0, 9.81);
    publisher.publishImu(imu);
    publisher.broadcastPlaybackState(FoxglovePublisher::PlaybackState{
        FoxglovePublisher::PlaybackStatus::Playing, 123005000000ULL, 1.0f, false});
    publisher.stop();
    return 0;
}

static int testLvxParsesCartesianPackets() {
    std::vector<uint8_t> frame;

    std::vector<uint8_t> type0;
    appendCartesian(type0, 1000, -2000, 3000, 42);
    for (int i = 1; i < 100; ++i) appendCartesian(type0, 0, 0, 0, 1);
    appendPacket(frame, kLvxDataTypeCartesian, 1000000000ULL, type0);

    std::vector<uint8_t> type2;
    appendExtendedCartesian(type2, -4000, 5000, 6000, 43, 7);
    for (int i = 1; i < 96; ++i) appendExtendedCartesian(type2, 0, 0, 0, 2, 0);
    appendPacket(frame, kLvxDataTypeExtendCartesian, 1001000000ULL, type2);

    std::vector<uint8_t> type4;
    appendExtendedCartesian(type4, 7000, 0, 0, 44, 8);
    appendExtendedCartesian(type4, 0, 8000, 0, 45, 9);
    for (int i = 1; i < 48; ++i) {
        appendExtendedCartesian(type4, 0, 0, 0, 3, 0);
        appendExtendedCartesian(type4, 0, 0, 0, 4, 0);
    }
    appendPacket(frame, kLvxDataTypeDualCartesian, 1002000000ULL, type4);

    std::vector<uint8_t> type7;
    appendExtendedCartesian(type7, 0, 0, 9000, 46, 10);
    appendExtendedCartesian(type7, 10000, 0, 0, 47, 11);
    appendExtendedCartesian(type7, 0, 11000, 0, 48, 12);
    for (int i = 1; i < 30; ++i) {
        appendExtendedCartesian(type7, 0, 0, 0, 5, 0);
        appendExtendedCartesian(type7, 0, 0, 0, 6, 0);
        appendExtendedCartesian(type7, 0, 0, 0, 7, 0);
    }
    appendPacket(frame, kLvxDataTypeTripleCartesian, 1003000000ULL, type7);

    LvxReader reader;
    LvxFrameParseResult parsed = reader.parsePacketsForTest(frame);
    if (expect(parsed.points.size() == 382)) return 1;
    if (expect(parsed.has_points)) return 1;
    if (expect(!parsed.has_imu)) return 1;
    if (expect(parsed.unsupported_packet_count == 0)) return 1;
    if (expect(parsed.truncated_packet_count == 0)) return 1;

    if (expect(near(parsed.points[0].x, 1.0f))) return 1;
    if (expect(near(parsed.points[0].y, -2.0f))) return 1;
    if (expect(near(parsed.points[0].z, 3.0f))) return 1;
    if (expect(parsed.points[0].reflectivity == 42)) return 1;
    if (expect(parsed.points[0].tag == 0)) return 1;

    if (expect(near(parsed.points[100].x, -4.0f))) return 1;
    if (expect(parsed.points[100].reflectivity == 43)) return 1;
    if (expect(parsed.points[100].tag == 7)) return 1;

    if (expect(near(parsed.points[196].x, 7.0f))) return 1;
    if (expect(near(parsed.points[197].y, 8.0f))) return 1;
    if (expect(parsed.points[196].tag == 8)) return 1;
    if (expect(parsed.points[197].tag == 9)) return 1;

    if (expect(near(parsed.points[292].z, 9.0f))) return 1;
    if (expect(near(parsed.points[293].x, 10.0f))) return 1;
    if (expect(near(parsed.points[294].y, 11.0f))) return 1;

    uint32_t previous_offset = 0;
    for (const LvxPoint& point : parsed.points) {
        if (expect(point.offset_time >= previous_offset)) return 1;
        previous_offset = point.offset_time;
        if (expect(point.line == 0)) return 1;
    }
    return 0;
}

static int testLvxParsesSphericalPackets() {
    std::vector<uint8_t> frame;

    std::vector<uint8_t> type1;
    appendSpherical(type1, 1000, 9000, 0, 51);
    for (int i = 1; i < 100; ++i) appendSpherical(type1, 0, 0, 0, 1);
    appendPacket(frame, kLvxDataTypeSpherical, 2000000000ULL, type1);

    std::vector<uint8_t> type3;
    appendExtendedSpherical(type3, 2000, 9000, 9000, 52, 13);
    for (int i = 1; i < 96; ++i) appendExtendedSpherical(type3, 0, 0, 0, 2, 0);
    appendPacket(frame, kLvxDataTypeExtendSpherical, 2001000000ULL, type3);

    std::vector<uint8_t> type5;
    appendScalar<uint16_t>(type5, 9000);
    appendScalar<uint16_t>(type5, 18000);
    appendScalar<uint32_t>(type5, 3000);
    type5.push_back(53);
    type5.push_back(14);
    appendScalar<uint32_t>(type5, 4000);
    type5.push_back(54);
    type5.push_back(15);
    for (int i = 1; i < 48; ++i) {
        appendScalar<uint16_t>(type5, 0);
        appendScalar<uint16_t>(type5, 0);
        appendScalar<uint32_t>(type5, 0);
        type5.push_back(3);
        type5.push_back(0);
        appendScalar<uint32_t>(type5, 0);
        type5.push_back(4);
        type5.push_back(0);
    }
    appendPacket(frame, kLvxDataTypeDualSpherical, 2002000000ULL, type5);

    std::vector<uint8_t> type8;
    appendScalar<uint16_t>(type8, 9000);
    appendScalar<uint16_t>(type8, 27000);
    appendScalar<uint32_t>(type8, 5000);
    type8.push_back(55);
    type8.push_back(16);
    appendScalar<uint32_t>(type8, 6000);
    type8.push_back(56);
    type8.push_back(17);
    appendScalar<uint32_t>(type8, 7000);
    type8.push_back(57);
    type8.push_back(18);
    for (int i = 1; i < 30; ++i) {
        appendScalar<uint16_t>(type8, 0);
        appendScalar<uint16_t>(type8, 0);
        appendScalar<uint32_t>(type8, 0);
        type8.push_back(5);
        type8.push_back(0);
        appendScalar<uint32_t>(type8, 0);
        type8.push_back(6);
        type8.push_back(0);
        appendScalar<uint32_t>(type8, 0);
        type8.push_back(7);
        type8.push_back(0);
    }
    appendPacket(frame, kLvxDataTypeTripleSpherical, 2003000000ULL, type8);

    LvxReader reader;
    LvxFrameParseResult parsed = reader.parsePacketsForTest(frame);
    if (expect(parsed.points.size() == 382)) return 1;
    if (expect(near(parsed.points[0].x, 1.0f))) return 1;
    if (expect(near(parsed.points[0].y, 0.0f))) return 1;
    if (expect(near(parsed.points[0].z, 0.0f))) return 1;
    if (expect(parsed.points[0].reflectivity == 51)) return 1;

    if (expect(near(parsed.points[100].x, 0.0f))) return 1;
    if (expect(near(parsed.points[100].y, 2.0f))) return 1;
    if (expect(parsed.points[100].tag == 13)) return 1;

    if (expect(near(parsed.points[196].x, -3.0f))) return 1;
    if (expect(near(parsed.points[197].x, -4.0f))) return 1;
    if (expect(parsed.points[196].tag == 14)) return 1;
    if (expect(parsed.points[197].tag == 15)) return 1;

    if (expect(near(parsed.points[292].y, -5.0f))) return 1;
    if (expect(near(parsed.points[293].y, -6.0f))) return 1;
    if (expect(near(parsed.points[294].y, -7.0f))) return 1;
    return 0;
}

static int testLvxParsesImuAndDiagnostics() {
    std::vector<uint8_t> frame;

    std::vector<uint8_t> type0;
    appendCartesian(type0, 1000, 0, 0, 1);
    for (int i = 1; i < 100; ++i) appendCartesian(type0, 0, 0, 0, 1);
    appendPacket(frame, kLvxDataTypeCartesian, 5000000000ULL, type0);

    std::vector<uint8_t> imu;
    appendScalar<float>(imu, 0.1f);
    appendScalar<float>(imu, 0.2f);
    appendScalar<float>(imu, 0.3f);
    appendScalar<float>(imu, 1.1f);
    appendScalar<float>(imu, 1.2f);
    appendScalar<float>(imu, 1.3f);
    appendPacket(frame, kLvxDataTypeImu, 5002000000ULL, imu);

    LvxReader reader;
    LvxFrameParseResult parsed = reader.parsePacketsForTest(frame);
    if (expect(parsed.has_points)) return 1;
    if (expect(parsed.has_imu)) return 1;
    if (expect(parsed.imus.size() == 1)) return 1;
    if (expect(std::fabs(parsed.frame_time - 0.0) < 0.000001)) return 1;
    if (expect(std::fabs(parsed.imus[0].timestamp - 0.002) < 0.000001)) return 1;
    if (expect(std::fabs(parsed.imus[0].gyro[0] - 0.1) < 0.000001)) return 1;
    if (expect(std::fabs(parsed.imus[0].gyro[2] - 0.3) < 0.000001)) return 1;
    if (expect(std::fabs(parsed.imus[0].acc[0] - 1.1) < 0.000001)) return 1;
    if (expect(std::fabs(parsed.imus[0].acc[2] - 1.3) < 0.000001)) return 1;

    std::vector<uint8_t> unknown;
    appendPacketHeader(unknown, 99, 1);
    LvxFrameParseResult unknown_parsed = reader.parsePacketsForTest(unknown);
    if (expect(unknown_parsed.unsupported_packet_count == 1)) return 1;
    if (expect(!unknown_parsed.has_points)) return 1;

    std::vector<uint8_t> truncated;
    appendPacketHeader(truncated, kLvxDataTypeCartesian, 1);
    truncated.push_back(0);
    LvxFrameParseResult truncated_parsed = reader.parsePacketsForTest(truncated);
    if (expect(truncated_parsed.truncated_packet_count == 1)) return 1;
    if (expect(!truncated_parsed.has_points)) return 1;
    return 0;
}

static int testLvxOpenIgnoresEmptyTailFrame() {
    const char* path = "lvx_empty_tail_test.lvx";
    std::remove(path);

    std::ofstream out(path, std::ios::binary);
    if (expect(out.is_open())) return 1;

    LvxPublicHeader public_header{};
    std::memcpy(public_header.signature, "livox_tech", 10);
    public_header.version[0] = 1;
    public_header.version[1] = 1;
    public_header.magic_code = 0xAC0EA767;
    out.write(reinterpret_cast<const char*>(&public_header), sizeof(public_header));

    LvxPrivateHeader private_header{};
    private_header.frame_duration = 50;
    private_header.device_count = 1;
    out.write(reinterpret_cast<const char*>(&private_header), sizeof(private_header));

    LvxDeviceInfo device{};
    device.device_type = 3;
    out.write(reinterpret_cast<const char*>(&device), sizeof(device));

    const uint64_t first_frame_offset = sizeof(LvxPublicHeader) + sizeof(LvxPrivateHeader) + sizeof(LvxDeviceInfo);
    std::vector<uint8_t> imu_payload;
    appendScalar<float>(imu_payload, 0.0f);
    appendScalar<float>(imu_payload, 0.0f);
    appendScalar<float>(imu_payload, 0.0f);
    appendScalar<float>(imu_payload, 0.0f);
    appendScalar<float>(imu_payload, 0.0f);
    appendScalar<float>(imu_payload, 1.0f);

    std::vector<uint8_t> packet;
    appendPacket(packet, kLvxDataTypeImu, 1000000000ULL, imu_payload);

    LvxFrameHeader frame{};
    frame.current_offset = first_frame_offset;
    frame.next_offset = first_frame_offset + sizeof(LvxFrameHeader) + packet.size();
    frame.frame_index = 0;
    out.write(reinterpret_cast<const char*>(&frame), sizeof(frame));
    out.write(reinterpret_cast<const char*>(packet.data()), static_cast<std::streamsize>(packet.size()));

    LvxFrameHeader empty_tail{};
    empty_tail.current_offset = frame.next_offset;
    empty_tail.next_offset = frame.next_offset + sizeof(LvxFrameHeader);
    empty_tail.frame_index = 1;
    out.write(reinterpret_cast<const char*>(&empty_tail), sizeof(empty_tail));
    out.close();

    LvxReader reader;
    if (expect(reader.open(path))) return 1;
    if (expect(reader.getTotalFrames() == 1)) return 1;

    std::mutex callback_mutex;
    std::condition_variable callback_cv;
    bool callback_entered = false;
    bool release_callback = false;
    reader.setFrameCallback(
        [&](const std::vector<LvxPoint>&, const std::vector<ImuData>&, double) {
            std::unique_lock<std::mutex> lock(callback_mutex);
            callback_entered = true;
            callback_cv.notify_all();
            callback_cv.wait(lock, [&] { return release_callback; });
        });
    reader.play(0.0);
    bool entered_in_time = false;
    {
        std::unique_lock<std::mutex> lock(callback_mutex);
        entered_in_time = callback_cv.wait_for(
            lock, std::chrono::seconds(2), [&] { return callback_entered; });
    }
    const auto stop_begin = std::chrono::steady_clock::now();
    const bool timed_stop = entered_in_time
        ? reader.stopFor(std::chrono::milliseconds(50))
        : true;
    const auto stop_elapsed = std::chrono::steady_clock::now() - stop_begin;
    {
        std::lock_guard<std::mutex> lock(callback_mutex);
        release_callback = true;
    }
    callback_cv.notify_all();
    const bool final_stop = reader.stopFor(std::chrono::seconds(2));
    reader.close();

    LvxReader throwing_reader;
    if (expect(throwing_reader.open(path))) return 1;
    std::mutex throwing_callback_mutex;
    std::condition_variable throwing_callback_cv;
    bool throwing_callback_entered = false;
    throwing_reader.setFrameCallback(
        [&](const std::vector<LvxPoint>&, const std::vector<ImuData>&, double) {
            {
                std::lock_guard<std::mutex> lock(throwing_callback_mutex);
                throwing_callback_entered = true;
            }
            throwing_callback_cv.notify_all();
            throw std::runtime_error("injected LVX callback failure");
        });
    throwing_reader.play(0.0);
    bool throwing_callback_entered_in_time = false;
    {
        std::unique_lock<std::mutex> lock(throwing_callback_mutex);
        throwing_callback_entered_in_time = throwing_callback_cv.wait_for(
            lock, std::chrono::seconds(2),
            [&] { return throwing_callback_entered; });
    }
    const bool throwing_reader_stopped =
        throwing_reader.stopFor(std::chrono::seconds(2));
    const LvxReader::PlaybackStats throwing_stats =
        throwing_reader.playbackStats();
    throwing_reader.close();

    LvxReader short_read_reader;
    if (expect(short_read_reader.open(path))) return 1;
    const bool truncated = LvxReaderTestAccess::truncateBackingFileAfterOpen(
        short_read_reader, path, frame.next_offset - 1);
    if (!truncated) {
        short_read_reader.close();
        std::remove(path);
        if (expect(truncated)) return 1;
    }
    short_read_reader.play(0.0);
    const auto short_read_deadline =
        std::chrono::steady_clock::now() + std::chrono::seconds(2);
    while (!short_read_reader.hasPlaybackFailure() &&
           std::chrono::steady_clock::now() < short_read_deadline) {
        std::this_thread::yield();
    }
    const bool short_read_failure_observed =
        short_read_reader.hasPlaybackFailure();
    const bool short_read_stopped =
        short_read_reader.stopFor(std::chrono::seconds(2));
    const LvxReader::PlaybackStats short_read_stats =
        short_read_reader.playbackStats();
    short_read_reader.close();

    std::remove(path);
    if (expect(entered_in_time)) return 1;
    if (expect(!timed_stop)) return 1;
    if (expect(stop_elapsed < std::chrono::milliseconds(500))) return 1;
    if (expect(final_stop)) return 1;
    if (expect(throwing_callback_entered_in_time)) return 1;
    if (expect(throwing_reader_stopped)) return 1;
    if (expect(throwing_stats.failures == 1)) return 1;
    if (expect(throwing_stats.failed)) return 1;
    if (expect(!throwing_stats.eof)) return 1;
    if (expect(!throwing_stats.playing)) return 1;
    if (expect(throwing_stats.thread_exited)) return 1;
    if (expect(throwing_stats.last_error ==
               "injected LVX callback failure")) return 1;
    if (expect(short_read_failure_observed)) return 1;
    if (expect(short_read_stopped)) return 1;
    if (expect(short_read_stats.failures == 1)) return 1;
    if (expect(short_read_stats.failed)) return 1;
    if (expect(!short_read_stats.eof)) return 1;
    if (expect(!short_read_stats.playing)) return 1;
    if (expect(short_read_stats.thread_exited)) return 1;
    if (expect(short_read_stats.last_error ==
               "failed to read a validated LVX frame")) return 1;
    return 0;
}

static int testLivoxAdapterStopForIsSafeWithoutInitializedSdk() {
    LivoxAdapter adapter;
    if (expect(!adapter.init())) return 1;
    std::vector<LivoxEthPacket> packets;
    uint32_t packet_count = 0;
    double timestamp = 0.0;
    if (expect(!adapter.getLidarData(
            packets, packet_count, timestamp, 0))) return 1;
    if (expect(adapter.stopFor(std::chrono::seconds(2)))) return 1;
    if (expect(adapter.stopFor(std::chrono::milliseconds(0)))) return 1;
    return 0;
}

static int testLivoxAdapterContainsThrowingSdkCallbacks() {
    LivoxAdapter adapter;
    LivoxEthPacket packet{};
    packet.data_type = kCartesian;
    auto* raw_point = reinterpret_cast<LivoxRawPoint*>(packet.data);
    raw_point->x = 1000;
    raw_point->y = 2000;
    raw_point->z = 3000;
    raw_point->reflectivity = 42;

    adapter.setLidarCallback(
        [](const std::vector<LvxPoint>&, double) {
            throw std::runtime_error("injected lidar callback failure");
        });
    LivoxAdapterTestAccess::armCallbacks(adapter);
    LivoxAdapterTestAccess::dispatchData(adapter, packet);
    if (expect(adapter.hasCallbackFailure())) return 1;
    if (expect(adapter.callbackFailureCount() == 1)) return 1;
    if (expect(LivoxAdapterTestAccess::callbacksInFlight(adapter) == 0)) return 1;

    // The failure latch prevents repeated SDK callbacks from invoking a
    // broken user callback again.
    LivoxAdapterTestAccess::dispatchData(adapter, packet);
    if (expect(adapter.callbackFailureCount() == 1)) return 1;

    packet = LivoxEthPacket{};
    packet.data_type = kImu;
    auto* imu_point = reinterpret_cast<LivoxImuPoint*>(packet.data);
    imu_point->gyro_x = 1.0f;
    imu_point->acc_z = 1.0f;
    adapter.setImuCallback(
        [](const ImuData&) {
            throw std::runtime_error("injected imu callback failure");
        });
    LivoxAdapterTestAccess::armCallbacks(adapter);
    LivoxAdapterTestAccess::dispatchData(adapter, packet);
    if (expect(adapter.hasCallbackFailure())) return 1;
    if (expect(adapter.callbackFailureCount() == 2)) return 1;
    if (expect(LivoxAdapterTestAccess::callbacksInFlight(adapter) == 0)) return 1;

    packet = LivoxEthPacket{};
    packet.data_type = kCartesian;
    raw_point = reinterpret_cast<LivoxRawPoint*>(packet.data);
    raw_point->x = 1000;
    adapter.setLidarCallback(
        [](const std::vector<LvxPoint>&, double) {
            throw 42;
        });
    LivoxAdapterTestAccess::armCallbacks(adapter);
    LivoxAdapterTestAccess::dispatchData(adapter, packet);
    if (expect(adapter.hasCallbackFailure())) return 1;
    if (expect(adapter.callbackFailureCount() == 3)) return 1;
    if (expect(LivoxAdapterTestAccess::callbacksInFlight(adapter) == 0)) return 1;
    if (expect(adapter.stopFor(std::chrono::seconds(2)))) return 1;
    return 0;
}

static int testLivoxAdapterStopForFromCallbackDoesNotSelfWait() {
    LivoxAdapter adapter;
    LivoxEthPacket packet{};
    packet.data_type = kCartesian;
    auto* raw_point = reinterpret_cast<LivoxRawPoint*>(packet.data);
    raw_point->x = 1000;

    bool callback_called = false;
    bool callback_stop_result = true;
    std::chrono::steady_clock::duration callback_stop_elapsed{};
    adapter.setLidarCallback(
        [&](const std::vector<LvxPoint>&, double) {
            callback_called = true;
            const auto begin = std::chrono::steady_clock::now();
            callback_stop_result =
                adapter.stopFor(std::chrono::seconds(5));
            callback_stop_elapsed = std::chrono::steady_clock::now() - begin;
        });
    LivoxAdapterTestAccess::armCallbacks(adapter);
    LivoxAdapterTestAccess::dispatchData(adapter, packet);

    if (expect(callback_called)) return 1;
    if (expect(!callback_stop_result)) return 1;
    if (expect(callback_stop_elapsed < std::chrono::milliseconds(500))) return 1;
    if (expect(LivoxAdapterTestAccess::callbacksInFlight(adapter) == 0)) return 1;
    if (expect(adapter.stopFor(std::chrono::seconds(2)))) return 1;
    return 0;
}

static int testBagReaderRejectsMalformedPointCountWithoutThrowing() {
    const char* path = "bag_reader_malformed_point_count_test.bag";
    std::remove(path);

    BagWriter writer;
    if (expect(writer.open(path))) return 1;
    const uint32_t connection = writer.addConnection(
        "/livox/lidar", "livox_ros_driver/CustomMsg",
        "e4d6829bdfe657cb6c21a746c86b21a6",
        "std_msgs/Header header\nuint64 timebase\nuint32 point_num\n");
    RosSerializer serializer;
    RosHeader header;
    header.stamp_sec = 1;
    serializeHeader(serializer, header);
    serializer.writeUint64(1000000000ULL);
    serializer.writeUint32(std::numeric_limits<uint32_t>::max());
    serializer.writeUint8(1);
    serializer.writeUint8(0);
    serializer.writeUint8(0);
    serializer.writeUint8(0);
    serializer.writeUint32(std::numeric_limits<uint32_t>::max());
    writer.writeMessage(connection, 1000000000ULL,
                        serializer.data(), serializer.size());
    writer.close();

    BagReader reader;
    bool open_threw = false;
    bool opened = false;
    try {
        opened = reader.open(path);
    } catch (...) {
        open_threw = true;
    }
    const BagReader::PlaybackStats stats = reader.playbackStats();
    reader.close();
    std::remove(path);

    if (expect(!open_threw)) return 1;
    if (expect(!opened)) return 1;
    if (expect(stats.failures == 1)) return 1;
    if (expect(stats.failed)) return 1;
    if (expect(!stats.eof)) return 1;
    if (expect(stats.thread_exited)) return 1;
    if (expect(stats.last_error == "invalid Livox CustomMsg point array")) return 1;
    return 0;
}

static int testBagReaderContainsThrowingFrameCallback() {
    const char* path = "bag_reader_callback_exception_test.bag";
    std::remove(path);

    BagWriter writer;
    if (expect(writer.open(path))) return 1;
    const uint32_t connection = writer.addConnection(
        "/livox/imu", "sensor_msgs/Imu",
        "6a62c6daae103f4ff57a132d6f95cec2",
        "std_msgs/Header header\ngeometry_msgs/Quaternion orientation\n");

    RosSerializer serializer;
    RosHeader header;
    header.stamp_sec = 1;
    header.stamp_nsec = 250000000;
    header.frame_id = "livox_imu";
    serializeHeader(serializer, header);
    for (int i = 0; i < 4; ++i) serializer.writeFloat64(i == 3 ? 1.0 : 0.0);
    for (int i = 0; i < 9; ++i) serializer.writeFloat64(0.0);
    for (int i = 0; i < 3; ++i) serializer.writeFloat64(0.0);
    for (int i = 0; i < 9; ++i) serializer.writeFloat64(0.0);
    for (int i = 0; i < 3; ++i) serializer.writeFloat64(i == 2 ? 9.81 : 0.0);
    for (int i = 0; i < 9; ++i) serializer.writeFloat64(0.0);
    writer.writeMessage(connection, 1250000000ULL,
                        serializer.data(), serializer.size());
    writer.close();

    BagReader reader;
    if (expect(reader.open(path))) return 1;
    std::mutex callback_mutex;
    std::condition_variable callback_cv;
    bool callback_entered = false;
    reader.setFrameCallback(
        [&](const std::vector<LvxPoint>&, const std::vector<ImuData>&, double) {
            {
                std::lock_guard<std::mutex> lock(callback_mutex);
                callback_entered = true;
            }
            callback_cv.notify_all();
            throw std::runtime_error("injected Bag callback failure");
        });
    reader.play(0.0);
    bool callback_entered_in_time = false;
    {
        std::unique_lock<std::mutex> lock(callback_mutex);
        callback_entered_in_time = callback_cv.wait_for(
            lock, std::chrono::seconds(2), [&] { return callback_entered; });
    }
    const bool stopped = reader.stopFor(std::chrono::seconds(2));
    const BagReader::PlaybackStats stats = reader.playbackStats();

    std::mutex unknown_callback_mutex;
    std::condition_variable unknown_callback_cv;
    bool unknown_callback_entered = false;
    reader.setFrameCallback(
        [&](const std::vector<LvxPoint>&, const std::vector<ImuData>&, double) {
            {
                std::lock_guard<std::mutex> lock(unknown_callback_mutex);
                unknown_callback_entered = true;
            }
            unknown_callback_cv.notify_all();
            throw 42;
        });
    reader.play(0.0);
    bool unknown_callback_entered_in_time = false;
    {
        std::unique_lock<std::mutex> lock(unknown_callback_mutex);
        unknown_callback_entered_in_time = unknown_callback_cv.wait_for(
            lock, std::chrono::seconds(2),
            [&] { return unknown_callback_entered; });
    }
    const bool unknown_stopped = reader.stopFor(std::chrono::seconds(2));
    const BagReader::PlaybackStats unknown_stats = reader.playbackStats();
    reader.close();
    std::remove(path);

    if (expect(callback_entered_in_time)) return 1;
    if (expect(stopped)) return 1;
    if (expect(stats.failures == 1)) return 1;
    if (expect(stats.failed)) return 1;
    if (expect(!stats.eof)) return 1;
    if (expect(stats.thread_exited)) return 1;
    if (expect(stats.last_error == "injected Bag callback failure")) return 1;
    if (expect(unknown_callback_entered_in_time)) return 1;
    if (expect(unknown_stopped)) return 1;
    if (expect(unknown_stats.failures == 2)) return 1;
    if (expect(unknown_stats.failed)) return 1;
    if (expect(!unknown_stats.eof)) return 1;
    if (expect(unknown_stats.thread_exited)) return 1;
    if (expect(unknown_stats.last_error == "unknown playback exception")) return 1;
    return 0;
}

static int testBagReaderStopForIsIdempotentBeforePlayback() {
    BagReader reader;
    if (expect(reader.stopFor(std::chrono::milliseconds(0)))) return 1;
    if (expect(reader.stopFor(std::chrono::milliseconds(0)))) return 1;
    return 0;
}

static PointType makeTestPoint(float x, float y, float z, float intensity = 1.0f) {
    PointType point;
    point.x = x;
    point.y = y;
    point.z = z;
    point.intensity = intensity;
    return point;
}

static TileUpdate makeTestTileUpdate(int32_t tile_x, uint64_t version,
                                     float point_x) {
    TileUpdate update;
    update.key = TileKey{tile_x, 0, 0};
    update.version = version;
    update.points.push_back(makeTestPoint(point_x, 0.0f, 0.0f));
    update.points.width = 1;
    update.points.height = 1;
    update.points.is_dense = true;
    return update;
}

template <typename Predicate>
static bool waitUntil(Predicate predicate, std::chrono::milliseconds timeout) {
    const auto deadline = std::chrono::steady_clock::now() + timeout;
    do {
        if (predicate()) return true;
        std::this_thread::yield();
    } while (std::chrono::steady_clock::now() < deadline);
    return predicate();
}

class ScopedStorageTestDirectory {
public:
    explicit ScopedStorageTestDirectory(const char* name) {
        const auto unique_id = std::chrono::steady_clock::now()
                                   .time_since_epoch().count();
        path_ = std::filesystem::current_path() /
                (std::string(name) + "_" + std::to_string(unique_id));
        std::error_code error;
        std::filesystem::remove_all(path_, error);
        std::filesystem::create_directories(path_);
    }

    ~ScopedStorageTestDirectory() {
        std::error_code error;
        std::filesystem::remove_all(path_, error);
    }

    const std::filesystem::path& path() const { return path_; }

private:
    std::filesystem::path path_;
};

static std::vector<std::filesystem::path> storagePcdFiles(
    const std::filesystem::path& output_root) {
    std::vector<std::filesystem::path> files;
    const auto pcd_directory = output_root / "PCD";
    if (!std::filesystem::exists(pcd_directory)) return files;
    for (const auto& entry : std::filesystem::directory_iterator(pcd_directory)) {
        if (entry.is_regular_file() && entry.path().extension() == ".pcd") {
            files.push_back(entry.path());
        }
    }
    std::sort(files.begin(), files.end());
    return files;
}

using RawBagFields = std::map<std::string, std::vector<uint8_t>>;

struct RawBagRecord {
    uint64_t offset = 0;
    RawBagFields fields;
    std::vector<uint8_t> data;
};

static bool parseRawBagFields(const uint8_t* bytes, size_t size,
                              RawBagFields& fields) {
    size_t offset = 0;
    while (offset < size) {
        if (size - offset < sizeof(uint32_t)) return false;
        uint32_t field_size = 0;
        std::memcpy(&field_size, bytes + offset, sizeof(field_size));
        offset += sizeof(field_size);
        if (field_size > size - offset) return false;
        const uint8_t* field = bytes + offset;
        const uint8_t* equals = static_cast<const uint8_t*>(
            std::memchr(field, '=', field_size));
        if (equals == nullptr) return false;
        const size_t key_size = static_cast<size_t>(equals - field);
        const std::string key(reinterpret_cast<const char*>(field), key_size);
        const uint8_t* value = equals + 1;
        const size_t value_size = field_size - key_size - 1;
        fields[key] = std::vector<uint8_t>(value, value + value_size);
        offset += field_size;
    }
    return true;
}

static bool readRawBagRecord(std::istream& stream, RawBagRecord& record) {
    const std::streampos position = stream.tellg();
    if (position == std::streampos(-1)) return false;
    record = RawBagRecord{};
    record.offset = static_cast<uint64_t>(position);
    uint32_t header_size = 0;
    stream.read(reinterpret_cast<char*>(&header_size), sizeof(header_size));
    if (!stream || header_size > 16 * 1024 * 1024) return false;
    std::vector<uint8_t> header(header_size);
    if (header_size > 0) {
        stream.read(reinterpret_cast<char*>(header.data()), header.size());
        if (!stream) return false;
    }
    if (!parseRawBagFields(header.data(), header.size(), record.fields)) return false;
    uint32_t data_size = 0;
    stream.read(reinterpret_cast<char*>(&data_size), sizeof(data_size));
    if (!stream) return false;
    record.data.resize(data_size);
    if (data_size > 0) {
        stream.read(reinterpret_cast<char*>(record.data.data()), record.data.size());
        if (!stream) return false;
    }
    return true;
}

static bool readRawBagRecord(const std::vector<uint8_t>& bytes, size_t& offset,
                             RawBagRecord& record) {
    const size_t record_offset = offset;
    if (offset > bytes.size()) return false;
    if (bytes.size() - offset < sizeof(uint32_t)) return false;
    uint32_t header_size = 0;
    std::memcpy(&header_size, bytes.data() + offset, sizeof(header_size));
    offset += sizeof(header_size);
    if (header_size > bytes.size() - offset) return false;
    record = RawBagRecord{};
    record.offset = record_offset;
    if (!parseRawBagFields(bytes.data() + offset, header_size, record.fields)) {
        return false;
    }
    offset += header_size;
    if (bytes.size() - offset < sizeof(uint32_t)) return false;
    uint32_t data_size = 0;
    std::memcpy(&data_size, bytes.data() + offset, sizeof(data_size));
    offset += sizeof(data_size);
    if (data_size > bytes.size() - offset) return false;
    record.data.assign(bytes.begin() + static_cast<std::ptrdiff_t>(offset),
                       bytes.begin() + static_cast<std::ptrdiff_t>(offset + data_size));
    offset += data_size;
    return true;
}

static const std::vector<uint8_t>* rawBagField(
    const RawBagRecord& record, const std::string& key) {
    const auto found = record.fields.find(key);
    return found == record.fields.end() ? nullptr : &found->second;
}

static bool decodeRawBagU32(const std::vector<uint8_t>* field, uint32_t& value) {
    if (field == nullptr || field->size() != sizeof(value)) return false;
    std::memcpy(&value, field->data(), sizeof(value));
    return true;
}

static bool decodeRawBagU64(const std::vector<uint8_t>* field, uint64_t& value) {
    if (field == nullptr || field->size() != sizeof(value)) return false;
    std::memcpy(&value, field->data(), sizeof(value));
    return true;
}

static bool decodeRawBagTime(const std::vector<uint8_t>* field,
                             uint32_t& sec, uint32_t& nsec) {
    if (field == nullptr || field->size() != 2 * sizeof(uint32_t)) return false;
    std::memcpy(&sec, field->data(), sizeof(sec));
    std::memcpy(&nsec, field->data() + sizeof(sec), sizeof(nsec));
    return nsec < 1000000000U;
}

static uint8_t rawBagOp(const RawBagRecord& record) {
    const auto* op = rawBagField(record, "op");
    return op != nullptr && op->size() == 1 ? (*op)[0] : 0;
}

static int testMapAccumulatorKeepsDistantFrames() {
    MapAccumulator accumulator;
    accumulator.setLeafSize(0.1f);

    PointCloudXYZI near_cloud;
    near_cloud.push_back(makeTestPoint(0.0f, 0.0f, 0.0f));
    near_cloud.push_back(makeTestPoint(1.0f, 0.0f, 0.0f));

    PointCloudXYZI far_cloud;
    far_cloud.push_back(makeTestPoint(2000.0f, 0.0f, 0.0f));
    far_cloud.push_back(makeTestPoint(2001.0f, 0.0f, 0.0f));

    accumulator.addFrame(near_cloud);
    accumulator.addFrame(far_cloud);

    PointCloudXYZI snapshot = accumulator.snapshot();
    if (expect(snapshot.size() == 4)) return 1;

    bool has_near = false;
    bool has_far = false;
    for (const auto& point : snapshot.points) {
        has_near = has_near || point.x < 10.0f;
        has_far = has_far || point.x > 1000.0f;
    }

    if (expect(has_near)) return 1;
    if (expect(has_far)) return 1;
    return 0;
}

static int testMapAccumulatorReturnsOnlyNewVoxels() {
    MapAccumulator accumulator;
    accumulator.setLeafSize(1.0f);

    PointCloudXYZI first;
    first.push_back(makeTestPoint(0.1f, 0.1f, 0.1f));
    first.push_back(makeTestPoint(0.2f, 0.2f, 0.2f));
    first.push_back(makeTestPoint(1.1f, 0.1f, 0.1f));
    PointCloudXYZI first_delta;
    accumulator.addFrame(first, &first_delta);
    if (expect(first_delta.size() == 2)) return 1;

    PointCloudXYZI second;
    second.push_back(makeTestPoint(0.3f, 0.3f, 0.3f));
    second.push_back(makeTestPoint(2.1f, 0.1f, 0.1f));
    PointCloudXYZI second_delta;
    accumulator.addFrame(second, &second_delta);
    if (expect(second_delta.size() == 1)) return 1;
    if (expect(near(second_delta.front().x, 2.1f))) return 1;
    if (expect(accumulator.size() == 3)) return 1;
    return 0;
}

static int testAsyncMapPublisherBuildsLatestSnapshot() {
    MapAccumulator accumulator;
    accumulator.setLeafSize(0.1f);

    PointCloudXYZI first_cloud;
    first_cloud.push_back(makeTestPoint(0.0f, 0.0f, 0.0f));

    std::mutex result_mutex;
    std::vector<size_t> published_sizes;
    std::vector<double> published_timestamps;
    std::vector<size_t> delta_sizes;
    AsyncMapPublisher publisher(
        accumulator,
        [&](const PointCloudXYZI& map, double timestamp) {
            std::lock_guard<std::mutex> lock(result_mutex);
            published_sizes.push_back(map.size());
            published_timestamps.push_back(timestamp);
        },
        [&](const PointCloudXYZI& delta, double) {
            std::lock_guard<std::mutex> lock(result_mutex);
            delta_sizes.push_back(delta.size());
        },
        100);

    publisher.start();
    publisher.enqueueFrame(std::move(first_cloud), 0.9);
    publisher.request(1.0);
    publisher.flush();

    PointCloudXYZI second_cloud;
    second_cloud.push_back(makeTestPoint(100.0f, 0.0f, 0.0f));
    publisher.enqueueFrame(std::move(second_cloud), 1.9);
    publisher.request(2.0);
    publisher.flush();

    PointCloudXYZI delta;
    delta.push_back(makeTestPoint(101.0f, 0.0f, 0.0f));
    delta.push_back(makeTestPoint(102.0f, 0.0f, 0.0f));
    publisher.enqueueDelta(std::move(delta), 2.1);
    publisher.flush();
    const AsyncMapPublisher::Stats stats = publisher.stats();
    publisher.stop();

    std::lock_guard<std::mutex> lock(result_mutex);
    if (expect(published_sizes.size() == 2)) return 1;
    if (expect(published_sizes[0] == 1)) return 1;
    if (expect(published_sizes[1] == 2)) return 1;
    if (expect(published_timestamps[1] == 2.0)) return 1;
    if (expect(stats.requested == 2)) return 1;
    if (expect(stats.published == 2)) return 1;
    if (expect(stats.frames_enqueued == 2)) return 1;
    if (expect(stats.frames_built == 2)) return 1;
    if (expect(stats.frames_dropped == 0)) return 1;
    if (expect(delta_sizes.size() == 3)) return 1;
    if (expect(delta_sizes[0] == 1)) return 1;
    if (expect(delta_sizes[1] == 1)) return 1;
    if (expect(delta_sizes[2] == 2)) return 1;
    if (expect(stats.delta_requested == 3)) return 1;
    if (expect(stats.delta_published == 3)) return 1;
    if (expect(stats.failures == 0)) return 1;
    return 0;
}

static int testAsyncMapPublisherResyncsAfterDeltaOverflow() {
    MapAccumulator accumulator;
    PointCloudXYZI map_cloud;
    map_cloud.push_back(makeTestPoint(0.0f, 0.0f, 0.0f));
    accumulator.addFrame(map_cloud);

    std::mutex gate_mutex;
    std::condition_variable gate_cv;
    bool first_full_started = false;
    bool release_first_full = false;
    size_t published_delta_size = 0;

    AsyncMapPublisher publisher(
        accumulator,
        [&](const PointCloudXYZI&, double) {
            std::unique_lock<std::mutex> lock(gate_mutex);
            if (!first_full_started) {
                first_full_started = true;
                gate_cv.notify_all();
                gate_cv.wait(lock, [&] { return release_first_full; });
            }
        },
        [&](const PointCloudXYZI& delta, double) {
            published_delta_size = delta.size();
        },
        2);

    publisher.start();
    publisher.request(1.0);
    {
        std::unique_lock<std::mutex> lock(gate_mutex);
        gate_cv.wait(lock, [&] { return first_full_started; });
    }

    PointCloudXYZI first_delta;
    first_delta.push_back(makeTestPoint(1.0f, 0.0f, 0.0f));
    first_delta.push_back(makeTestPoint(2.0f, 0.0f, 0.0f));
    publisher.enqueueDelta(std::move(first_delta), 1.1);

    PointCloudXYZI second_delta;
    second_delta.push_back(makeTestPoint(3.0f, 0.0f, 0.0f));
    second_delta.push_back(makeTestPoint(4.0f, 0.0f, 0.0f));
    publisher.enqueueDelta(std::move(second_delta), 1.2);

    {
        std::lock_guard<std::mutex> lock(gate_mutex);
        release_first_full = true;
    }
    gate_cv.notify_all();
    publisher.flush();
    const AsyncMapPublisher::Stats stats = publisher.stats();
    publisher.stop();

    if (expect(stats.delta_dropped_points == 2)) return 1;
    if (expect(stats.delta_resync_requests == 1)) return 1;
    if (expect(stats.published == 2)) return 1;
    if (expect(stats.delta_published == 1)) return 1;
    if (expect(published_delta_size == 2)) return 1;
    return 0;
}

static int testAsyncMapPublisherBoundsFrameQueue() {
    MapAccumulator accumulator;
    accumulator.setLeafSize(0.1f);
    PointCloudXYZI initial_map;
    initial_map.push_back(makeTestPoint(-10.0f, 0.0f, 0.0f));
    accumulator.addFrame(initial_map);

    std::mutex gate_mutex;
    std::condition_variable gate_cv;
    bool snapshot_started = false;
    bool release_snapshot = false;

    AsyncMapPublisher publisher(
        accumulator,
        [&](const PointCloudXYZI&, double) {
            std::unique_lock<std::mutex> lock(gate_mutex);
            snapshot_started = true;
            gate_cv.notify_all();
            gate_cv.wait(lock, [&] { return release_snapshot; });
        },
        AsyncMapPublisher::PublishCallback{},
        100,
        2);

    publisher.start();
    publisher.request(1.0);
    {
        std::unique_lock<std::mutex> lock(gate_mutex);
        gate_cv.wait(lock, [&] { return snapshot_started; });
    }

    for (int i = 0; i < 3; ++i) {
        PointCloudXYZI frame;
        frame.push_back(makeTestPoint(static_cast<float>(i), 0.0f, 0.0f));
        publisher.enqueueFrame(std::move(frame), 1.1 + i * 0.1);
    }

    {
        std::lock_guard<std::mutex> lock(gate_mutex);
        release_snapshot = true;
    }
    gate_cv.notify_all();
    publisher.flush();
    const AsyncMapPublisher::Stats stats = publisher.stats();
    publisher.stop();

    if (expect(stats.frames_enqueued == 3)) return 1;
    if (expect(stats.frames_built == 2)) return 1;
    if (expect(stats.frames_dropped == 1)) return 1;
    if (expect(stats.max_frame_queue_depth == 2)) return 1;
    if (expect(accumulator.size() == 3)) return 1;
    return 0;
}

static int testTiledMapStoreHandlesCoordinatesAndUpdatePolicies() {
    bool rejected_misaligned_grid = false;
    try {
        TiledMapStore::Config invalid_grid;
        invalid_grid.tile_size_m = 10.0;
        invalid_grid.voxel_leaf_m = 3.0;
        TiledMapStore invalid_store(invalid_grid);
    } catch (const std::invalid_argument&) {
        rejected_misaligned_grid = true;
    }
    if (expect(rejected_misaligned_grid)) return 1;

    TiledMapStore::Config latest_config;
    latest_config.tile_size_m = 10.0;
    latest_config.voxel_leaf_m = 1.0;
    latest_config.update_policy = VoxelUpdatePolicy::Latest;
    latest_config.max_memory_bytes = 0;
    TiledMapStore latest_store(latest_config);

    const TileKey mixed_key = latest_store.tileKeyForPoint(
        makeTestPoint(-0.001f, 10.0f, -10.001f));
    if (expect(mixed_key.x == -1)) return 1;
    if (expect(mixed_key.y == 1)) return 1;
    if (expect(mixed_key.z == -2)) return 1;

    const TileKey negative_boundary = latest_store.tileKeyForPoint(
        makeTestPoint(-10.0f, 0.0f, 9.999f));
    if (expect(negative_boundary.x == -1)) return 1;
    if (expect(negative_boundary.y == 0)) return 1;
    if (expect(negative_boundary.z == 0)) return 1;

    const TileKey positive_boundary = latest_store.tileKeyForPoint(
        makeTestPoint(10.0f, -10.001f, -0.001f));
    if (expect(positive_boundary.x == 1)) return 1;
    if (expect(positive_boundary.y == -2)) return 1;
    if (expect(positive_boundary.z == -1)) return 1;

    PointCloudXYZI first;
    first.push_back(makeTestPoint(0.1f, 0.1f, 0.1f, 2.0f));
    latest_store.addFrame(first, 1.0);
    PointCloudXYZI second;
    second.push_back(makeTestPoint(0.8f, 0.2f, 0.2f, 8.0f));
    latest_store.addFrame(second, 2.0);

    const std::vector<TileUpdate> latest_updates =
        latest_store.consumeDirtyTiles(0, 0, 0);
    if (expect(latest_updates.size() == 1)) return 1;
    if (expect(latest_updates[0].version == 2)) return 1;
    if (expect(latest_updates[0].points.size() == 1)) return 1;
    if (expect(near(latest_updates[0].points[0].x, 0.8f))) return 1;
    if (expect(near(latest_updates[0].points[0].intensity, 8.0f))) return 1;

    TiledMapStore::Config centroid_config = latest_config;
    centroid_config.update_policy = VoxelUpdatePolicy::Centroid;
    TiledMapStore centroid_store(centroid_config);
    PointCloudXYZI centroid_first;
    centroid_first.push_back(makeTestPoint(0.2f, 0.2f, 0.2f, 2.0f));
    centroid_store.addFrame(centroid_first, 1.0);
    PointCloudXYZI centroid_second;
    centroid_second.push_back(makeTestPoint(0.6f, 0.4f, 0.8f, 6.0f));
    centroid_store.addFrame(centroid_second, 2.0);

    const PointCloudXYZI centroid = centroid_store.snapshot();
    if (expect(centroid.size() == 1)) return 1;
    if (expect(near(centroid[0].x, 0.4f))) return 1;
    if (expect(near(centroid[0].y, 0.3f))) return 1;
    if (expect(near(centroid[0].z, 0.5f))) return 1;
    if (expect(near(centroid[0].intensity, 4.0f))) return 1;
    return 0;
}

static int testTiledMapStoreMatchesFirstVoxelAccumulatorSnapshot() {
    constexpr float leaf = 1.0f;
    MapAccumulator accumulator;
    accumulator.setLeafSize(leaf);

    TiledMapStore::Config config;
    config.tile_size_m = 10.0;
    config.voxel_leaf_m = leaf;
    config.update_policy = VoxelUpdatePolicy::First;
    config.max_memory_bytes = 0;
    TiledMapStore tiled(config);

    PointCloudXYZI first;
    first.push_back(makeTestPoint(0.1f, 0.1f, 0.1f, 1.0f));
    first.push_back(makeTestPoint(0.8f, 0.8f, 0.8f, 2.0f));
    first.push_back(makeTestPoint(10.0f, 0.0f, 0.0f, 3.0f));
    first.push_back(makeTestPoint(-0.1f, 0.0f, 0.0f, 4.0f));
    first.push_back(makeTestPoint(-10.0f, -0.1f, 1.1f, 5.0f));
    PointCloudXYZI second;
    second.push_back(makeTestPoint(1.2f, 0.2f, 0.2f, 6.0f));
    second.push_back(makeTestPoint(-0.8f, 0.8f, 0.8f, 7.0f));
    second.push_back(makeTestPoint(20.1f, -10.1f, 2.1f, 8.0f));

    accumulator.addFrame(first);
    accumulator.addFrame(second);
    tiled.addFrame(first, 1.0);
    tiled.addFrame(second, 2.0);

    auto voxel_map = [leaf](const PointCloudXYZI& cloud) {
        std::map<std::array<int64_t, 3>, PointType> result;
        for (const auto& point : cloud.points) {
            const std::array<int64_t, 3> key{
                static_cast<int64_t>(std::floor(point.x / leaf)),
                static_cast<int64_t>(std::floor(point.y / leaf)),
                static_cast<int64_t>(std::floor(point.z / leaf)),
            };
            result.emplace(key, point);
        }
        return result;
    };
    const auto reference = voxel_map(accumulator.snapshot());
    const auto rebuilt = voxel_map(tiled.snapshot());
    if (expect(reference.size() == rebuilt.size())) return 1;
    if (expect(reference.size() == accumulator.size())) return 1;
    for (const auto& entry : reference) {
        const auto found = rebuilt.find(entry.first);
        if (expect(found != rebuilt.end())) return 1;
        if (expect(near(found->second.x, entry.second.x) &&
                   near(found->second.y, entry.second.y) &&
                   near(found->second.z, entry.second.z) &&
                   near(found->second.intensity, entry.second.intensity))) return 1;
    }
    return 0;
}

static int testTiledMapStoreCoalescesDirtyVersionsAndHonorsLimits() {
    TiledMapStore::Config config;
    config.tile_size_m = 10.0;
    config.voxel_leaf_m = 1.0;
    config.update_policy = VoxelUpdatePolicy::Latest;
    config.max_memory_bytes = 0;
    TiledMapStore store(config);

    PointCloudXYZI initial;
    initial.push_back(makeTestPoint(0.1f, 0.0f, 0.0f));
    initial.push_back(makeTestPoint(10.1f, 0.0f, 0.0f));
    initial.push_back(makeTestPoint(20.1f, 0.0f, 0.0f));
    store.addFrame(initial, 1.0);
    if (expect(store.stats().dirty_count == 3)) return 1;
    if (expect(store.consumeDirtyTiles(0, 0, 0).size() == 3)) return 1;

    PointCloudXYZI tile_zero_update;
    tile_zero_update.push_back(makeTestPoint(0.2f, 0.0f, 0.0f));
    store.addFrame(tile_zero_update, 2.0);
    tile_zero_update[0].x = 0.3f;
    store.addFrame(tile_zero_update, 3.0);
    const std::vector<TileUpdate> coalesced = store.consumeDirtyTiles(0, 0, 0);
    const TileKey tile_zero_key{0, 0, 0};
    if (expect(coalesced.size() == 1)) return 1;
    if (expect(coalesced[0].key == tile_zero_key)) return 1;
    if (expect(coalesced[0].version == 3)) return 1;
    if (expect(coalesced[0].points.size() == 1)) return 1;
    if (expect(near(coalesced[0].points[0].x, 0.3f))) return 1;

    const size_t bytes_before_resync = store.stats().estimated_bytes;
    store.markAllDirty();
    if (expect(store.stats().estimated_bytes == bytes_before_resync)) return 1;
    const std::vector<TileUpdate> tile_limited =
        store.consumeDirtyTiles(1, 0, 0);
    if (expect(tile_limited.size() == 1)) return 1;
    store.restoreDirtyTiles(tile_limited);
    if (expect(store.stats().dirty_count == 3)) return 1;
    if (expect(store.stats().estimated_bytes == bytes_before_resync)) return 1;
    if (expect(store.consumeDirtyTiles(1, 0, 0).size() == 1)) return 1;
    if (expect(store.stats().dirty_count == 2)) return 1;
    if (expect(store.consumeDirtyTiles(0, 0, 0).size() == 2)) return 1;

    store.markAllDirty();
    const std::vector<TileUpdate> point_limited =
        store.consumeDirtyTiles(0, 1, 0);
    if (expect(point_limited.size() == 1)) return 1;
    if (expect(point_limited[0].points.size() == 1)) return 1;
    if (expect(store.stats().dirty_count == 2)) return 1;
    store.consumeDirtyTiles(0, 0, 0);

    const std::vector<TileUpdate> all_tiles = store.snapshotAllTiles();
    if (expect(all_tiles.size() == 3)) return 1;
    const size_t one_tile_bytes = all_tiles[0].estimatedBytes();
    if (expect(one_tile_bytes > 0)) return 1;
    store.markAllDirty();
    if (expect(store.consumeDirtyTiles(0, 0, one_tile_bytes - 1).empty())) return 1;
    if (expect(store.stats().dirty_count == 3)) return 1;
    const std::vector<TileUpdate> byte_limited =
        store.consumeDirtyTiles(0, 0, one_tile_bytes);
    if (expect(byte_limited.size() == 1)) return 1;
    if (expect(store.stats().dirty_count == 2)) return 1;

    // Dirty membership, explicit resync, and rejection restore are
    // allocation-free retained metadata operations, even when the voxel
    // store is exactly at its cap. Drain the resync one Tile at a time to
    // guard against accidentally rescanning/materializing the full backlog.
    TiledMapStore::Config bounded_config = config;
    bounded_config.max_memory_bytes = store.stats().estimated_bytes;
    TiledMapStore bounded_store(bounded_config);
    bounded_store.addFrame(initial, 1.0);
    if (expect(bounded_store.stats().estimated_bytes ==
               bounded_config.max_memory_bytes)) return 1;
    if (expect(bounded_store.consumeDirtyTiles(0, 0, 0).size() == 3)) return 1;
    bounded_store.markAllDirty();
    const std::vector<TileUpdate> rejected_batch =
        bounded_store.consumeDirtyTiles(1, 0, 0);
    if (expect(rejected_batch.size() == 1)) return 1;
    if (expect(rejected_batch[0].resync)) return 1;
    bounded_store.restoreDirtyTiles(rejected_batch);
    if (expect(bounded_store.stats().dirty_count == 3)) return 1;
    if (expect(bounded_store.stats().estimated_bytes ==
               bounded_config.max_memory_bytes)) return 1;

    size_t resync_tiles_drained = 0;
    while (bounded_store.stats().dirty_count > 0) {
        const std::vector<TileUpdate> batch =
            bounded_store.consumeDirtyTiles(1, 0, 0);
        if (expect(batch.size() == 1)) return 1;
        if (expect(batch[0].resync)) return 1;
        ++resync_tiles_drained;
    }
    if (expect(resync_tiles_drained == 3)) return 1;
    if (expect(bounded_store.stats().estimated_bytes ==
               bounded_config.max_memory_bytes)) return 1;
    return 0;
}

static int testTiledMapStoreBoundsSingleTileAndKeepsExistingVoxelsLive() {
    TiledMapStore::Config capacity_config;
    capacity_config.tile_size_m = 100.0;
    capacity_config.voxel_leaf_m = 1.0;
    capacity_config.update_policy = VoxelUpdatePolicy::Latest;
    capacity_config.max_memory_bytes = 0;
    capacity_config.max_voxels_per_tile = 2;
    TiledMapStore capacity_store(capacity_config);

    PointCloudXYZI initial;
    initial.push_back(makeTestPoint(0.1f, 0.0f, 0.0f, 1.0f));
    initial.push_back(makeTestPoint(1.1f, 0.0f, 0.0f, 2.0f));
    initial.push_back(makeTestPoint(2.1f, 0.0f, 0.0f, 3.0f));
    capacity_store.addFrame(initial, 1.0);
    auto capacity_stats = capacity_store.stats();
    if (expect(capacity_stats.voxel_count == 2)) return 1;
    if (expect(capacity_stats.points_rejected_tile_capacity == 1)) return 1;
    if (expect(capacity_stats.incomplete)) return 1;

    PointCloudXYZI refresh;
    refresh.push_back(makeTestPoint(0.8f, 0.0f, 0.0f, 9.0f));
    capacity_store.addFrame(refresh, 2.0);
    const PointCloudXYZI capacity_snapshot = capacity_store.snapshot();
    const auto refreshed = std::find_if(
        capacity_snapshot.begin(), capacity_snapshot.end(),
        [](const PointType& point) { return near(point.intensity, 9.0f); });
    if (expect(refreshed != capacity_snapshot.end())) return 1;
    if (expect(TileUpdate::maxPointCountForBytes(
                   TileUpdate::estimatedBytesForPointCount(3)) == 3)) return 1;

    TiledMapStore::Config memory_config;
    memory_config.tile_size_m = 100.0;
    memory_config.voxel_leaf_m = 1.0;
    memory_config.update_policy = VoxelUpdatePolicy::Latest;
    memory_config.max_memory_bytes = 0;
    memory_config.memory_policy = MapMemoryPolicy::StopAccumulating;
    TiledMapStore memory_probe(memory_config);
    memory_config.max_memory_bytes =
        memory_probe.stats().estimated_bytes + 4096;
    TiledMapStore memory_store(memory_config);
    PointCloudXYZI growth;
    for (int i = 0; i < 100; ++i) {
        growth.push_back(makeTestPoint(
            static_cast<float>(i) + 0.1f, 0.0f, 0.0f,
            static_cast<float>(i)));
    }
    memory_store.addFrame(growth, 1.0);
    const auto stopped_stats = memory_store.stats();
    if (expect(stopped_stats.accumulation_stopped)) return 1;
    if (expect(stopped_stats.voxel_count > 0)) return 1;
    if (expect(stopped_stats.estimated_bytes <=
               memory_config.max_memory_bytes)) return 1;
    const PointType retained = memory_store.snapshot().front();
    PointCloudXYZI retained_refresh;
    retained_refresh.push_back(makeTestPoint(
        retained.x, retained.y, retained.z, 123.0f));
    memory_store.addFrame(retained_refresh, 2.0);
    const PointCloudXYZI stopped_snapshot = memory_store.snapshot();
    const auto updated_after_stop = std::find_if(
        stopped_snapshot.begin(), stopped_snapshot.end(),
        [](const PointType& point) { return near(point.intensity, 123.0f); });
    if (expect(updated_after_stop != stopped_snapshot.end())) return 1;
    if (expect(memory_store.stats().estimated_bytes <=
               memory_config.max_memory_bytes)) return 1;

    MapBuildWorker::Config tile_worker_config;
    tile_worker_config.store.tile_size_m = 100.0;
    tile_worker_config.store.voxel_leaf_m = 1.0;
    tile_worker_config.store.max_memory_bytes = 0;
    tile_worker_config.max_points_per_update = 2;
    tile_worker_config.max_bytes_per_update =
        TileUpdate::estimatedBytesForPointCount(2);
    tile_worker_config.output.max_pending_points = 10;
    tile_worker_config.output.max_pending_bytes = 1024 * 1024;
    MapBuildWorker tile_worker(
        tile_worker_config, MapBuildWorker::PublishCallback{},
        [](const TileUpdate&, double) {});
    tile_worker.start();
    if (expect(tile_worker.enqueueFrame(PointCloudXYZI(initial), 1.0))) return 1;
    if (expect(tile_worker.flushFor(std::chrono::seconds(2)))) return 1;
    tile_worker.stop(true);
    if (expect(tile_worker.stats().store.voxel_count == 2)) return 1;
    if (expect(tile_worker.stats().store.points_rejected_tile_capacity == 1)) return 1;

    MapBuildWorker full_only_worker(
        tile_worker_config, MapBuildWorker::PublishCallback{},
        MapBuildWorker::TilePublishCallback{});
    full_only_worker.start();
    if (expect(full_only_worker.enqueueFrame(PointCloudXYZI(initial), 1.0))) return 1;
    if (expect(full_only_worker.flushFor(std::chrono::seconds(2)))) return 1;
    full_only_worker.stop(true);
    if (expect(full_only_worker.stats().store.voxel_count == 3)) return 1;
    return 0;
}

static int testTiledMapStoreBoundsAndCoalescesPendingDeletions() {
    TiledMapStore::Config normalized_config;
    normalized_config.max_pending_deletions = 0;
    TiledMapStore normalized_store(normalized_config);
    if (expect(normalized_store.config().max_pending_deletions ==
               TiledMapStore::kDefaultMaxPendingDeletions)) return 1;

    TiledMapStore::Config config;
    config.tile_size_m = 1.0;
    config.voxel_leaf_m = 0.1;
    config.max_memory_bytes = 0;
    config.memory_policy = MapMemoryPolicy::Lru;
    config.max_pending_deletions = 2;
    TiledMapStore store(config);

    TileUpdate first_deletion;
    first_deletion.key = TileKey{0, 0, 0};
    first_deletion.version = 4;
    first_deletion.deleted = true;
    TileUpdate second_deletion;
    second_deletion.key = TileKey{1, 0, 0};
    second_deletion.version = 8;
    second_deletion.deleted = true;
    TileUpdate overflow_deletion;
    overflow_deletion.key = TileKey{2, 0, 0};
    overflow_deletion.version = 12;
    overflow_deletion.deleted = true;

    store.restoreDirtyTiles({first_deletion, second_deletion});
    TileUpdate newer_deletion = first_deletion;
    newer_deletion.version += 7;
    store.restoreDirtyTiles({newer_deletion});
    store.restoreDirtyTiles({first_deletion});
    store.restoreDirtyTiles({overflow_deletion});
    auto stats = store.stats();
    if (expect(stats.pending_deletion_count == config.max_pending_deletions)) return 1;
    if (expect(stats.deletions_coalesced >= 2)) return 1;
    if (expect(stats.deletions_backpressured == 1)) return 1;
    if (expect(stats.accumulation_stopped)) return 1;
    if (expect(stats.incomplete)) return 1;

    const std::vector<TileUpdate> first = store.consumeDirtyTiles(
        1, 0, TileUpdate::estimatedBytesForPointCount(0));
    if (expect(first.size() == 1)) return 1;
    if (expect(first[0].deleted)) return 1;
    if (expect(store.stats().pending_deletion_count == 2)) return 1;
    if (expect(store.stats().dirty_count == 1)) return 1;

    // Rejection restores readiness without allocating a new tombstone node.
    store.restoreDirtyTiles(first);
    stats = store.stats();
    if (expect(stats.pending_deletion_count == 2)) return 1;
    if (expect(stats.dirty_count == 2)) return 1;
    const std::vector<TileUpdate> restored = store.consumeDirtyTiles(0, 0, 0);
    const auto restored_deletion = std::find_if(
        restored.begin(), restored.end(), [&](const TileUpdate& update) {
            return update.deleted && update.key == first_deletion.key;
        });
    if (expect(restored_deletion != restored.end())) return 1;
    if (expect(restored_deletion->version == newer_deletion.version)) return 1;
    store.confirmTilesHandedOff(restored);
    if (expect(store.stats().pending_deletion_count == 0)) return 1;
    if (expect(!store.stats().accumulation_stopped)) return 1;

    PointCloudXYZI recreated_frame;
    recreated_frame.push_back(makeTestPoint(0.1f, 0.0f, 0.0f));
    store.addFrame(recreated_frame, 20.0);
    const std::vector<TileUpdate> after_recreation =
        store.consumeDirtyTiles(0, 0, 0);
    const auto recreated = std::find_if(
        after_recreation.begin(), after_recreation.end(),
        [&](const TileUpdate& update) {
            return !update.deleted && update.key == first_deletion.key;
        });
    if (expect(recreated != after_recreation.end())) return 1;
    if (expect(recreated->version > newer_deletion.version)) return 1;

    TiledMapStore fresh(config);
    store.clear();
    if (expect(store.stats().pending_deletion_count == 0)) return 1;
    if (expect(store.stats().estimated_bytes ==
               fresh.stats().estimated_bytes)) return 1;
    return 0;
}

static int testTiledMapStoreOnlyDeletesTilesConfirmedHandedOff() {
    TiledMapStore::Config probe_config;
    probe_config.tile_size_m = 1.0;
    probe_config.voxel_leaf_m = 0.1;
    probe_config.max_memory_bytes = 0;
    probe_config.max_pending_deletions = 2;
    TiledMapStore probe(probe_config);
    PointCloudXYZI first_frame;
    first_frame.push_back(makeTestPoint(0.1f, 0.0f, 0.0f));
    probe.addFrame(first_frame, 1.0);

    TiledMapStore::Config config = probe_config;
    config.memory_policy = MapMemoryPolicy::Lru;
    const size_t deletion_bytes =
        TileUpdate::estimatedBytesForPointCount(0) +
        sizeof(TileKey) + 2 * sizeof(void*);
    config.max_memory_bytes = probe.stats().estimated_bytes + deletion_bytes;

    PointCloudXYZI second_frame;
    second_frame.push_back(makeTestPoint(1.1f, 0.0f, 0.0f));
    const TileKey first_key{0, 0, 0};
    const TileKey second_key{1, 0, 0};

    TiledMapStore unconfirmed(config);
    unconfirmed.addFrame(first_frame, 1.0);
    const std::vector<TileUpdate> unconfirmed_update =
        unconfirmed.consumeDirtyTiles(0, 0, 0);
    if (expect(unconfirmed_update.size() == 1)) return 1;
    if (expect(unconfirmed_update[0].key == first_key)) return 1;
    unconfirmed.addFrame(second_frame, 2.0);
    const auto unconfirmed_stats = unconfirmed.stats();
    if (expect(unconfirmed_stats.tiles_evicted == 1)) return 1;
    if (expect(unconfirmed_stats.pending_deletion_count == 0)) return 1;
    const std::vector<TileUpdate> after_unconfirmed_eviction =
        unconfirmed.consumeDirtyTiles(0, 0, 0);
    if (expect(after_unconfirmed_eviction.size() == 1)) return 1;
    if (expect(!after_unconfirmed_eviction[0].deleted)) return 1;
    if (expect(after_unconfirmed_eviction[0].key == second_key)) return 1;

    TiledMapStore confirmed(config);
    confirmed.addFrame(first_frame, 1.0);
    const std::vector<TileUpdate> confirmed_update =
        confirmed.consumeDirtyTiles(0, 0, 0);
    if (expect(confirmed_update.size() == 1)) return 1;
    confirmed.confirmTilesHandedOff(confirmed_update);
    confirmed.addFrame(second_frame, 2.0);
    const auto confirmed_stats = confirmed.stats();
    if (expect(confirmed_stats.tiles_evicted == 1)) return 1;
    if (expect(confirmed_stats.pending_deletion_count == 1)) return 1;
    const std::vector<TileUpdate> after_confirmed_eviction =
        confirmed.consumeDirtyTiles(0, 0, 0);
    const auto deletion = std::find_if(
        after_confirmed_eviction.begin(), after_confirmed_eviction.end(),
        [&](const TileUpdate& update) {
            return update.deleted && update.key == first_key;
        });
    const auto replacement = std::find_if(
        after_confirmed_eviction.begin(), after_confirmed_eviction.end(),
        [&](const TileUpdate& update) {
            return !update.deleted && update.key == second_key;
        });
    if (expect(deletion != after_confirmed_eviction.end())) return 1;
    if (expect(deletion->version > confirmed_update[0].version)) return 1;
    if (expect(replacement != after_confirmed_eviction.end())) return 1;
    return 0;
}

static int testTiledMapStoreRejectsNonFiniteCoordinates() {
    TiledMapStore::Config config;
    config.tile_size_m = 10.0;
    config.voxel_leaf_m = 1.0;
    config.max_memory_bytes = 0;
    TiledMapStore store(config);

    PointCloudXYZI frame;
    frame.push_back(makeTestPoint(
        std::numeric_limits<float>::quiet_NaN(), 0.0f, 0.0f));
    frame.push_back(makeTestPoint(
        0.0f, std::numeric_limits<float>::infinity(), 0.0f));
    frame.push_back(makeTestPoint(
        0.0f, 0.0f, -std::numeric_limits<float>::infinity()));
    frame.push_back(makeTestPoint(0.25f, 0.5f, 0.75f, 7.0f));

    const std::vector<TileKey> changed = store.addFrame(frame, 1.0);
    const auto stats = store.stats();
    if (expect(changed.size() == 1)) return 1;
    if (expect(stats.points_observed == 4)) return 1;
    if (expect(stats.points_rejected_invalid == 3)) return 1;
    if (expect(stats.voxel_count == 1)) return 1;
    if (expect(stats.tile_count == 1)) return 1;
    if (expect(stats.incomplete)) return 1;

    const PointCloudXYZI snapshot = store.snapshot();
    if (expect(snapshot.size() == 1)) return 1;
    if (expect(std::isfinite(snapshot[0].x) &&
               std::isfinite(snapshot[0].y) &&
               std::isfinite(snapshot[0].z))) return 1;
    if (expect(near(snapshot[0].intensity, 7.0f))) return 1;
    const std::vector<TileUpdate> updates =
        store.consumeDirtyTiles(0, 0, 0);
    if (expect(updates.size() == 1)) return 1;
    if (expect(updates[0].points.size() == 1)) return 1;
    return 0;
}

static int testTiledMapStoreNeverCreatesTombstoneAboveMemoryLimit() {
    TiledMapStore::Config probe_config;
    probe_config.tile_size_m = 1.0;
    probe_config.voxel_leaf_m = 0.1;
    probe_config.max_memory_bytes = 0;
    TiledMapStore probe(probe_config);
    const size_t empty_store_bytes = probe.stats().estimated_bytes;

    TiledMapStore::Config config = probe_config;
    config.memory_policy = MapMemoryPolicy::Lru;
    config.max_memory_bytes = empty_store_bytes;
    TiledMapStore store(config);
    PointCloudXYZI frame;
    frame.push_back(makeTestPoint(0.1f, 0.0f, 0.0f));
    store.addFrame(frame, 1.0);

    auto stats = store.stats();
    if (expect(stats.tile_count == 0)) return 1;
    if (expect(stats.pending_deletion_count == 0)) return 1;
    if (expect(stats.estimated_bytes <= config.max_memory_bytes)) return 1;
    // The just-created version-zero tile was never published, so rolling it
    // back needs no tombstone and cannot force metadata above the hard cap.
    if (expect(store.consumeDirtyTiles(0, 0, 0).empty())) return 1;
    return 0;
}

static int testTiledMapStoreCompactsOuterBucketsAfterGrowthRollback() {
    TiledMapStore::Config probe_config;
    probe_config.tile_size_m = 1.0;
    probe_config.voxel_leaf_m = 0.1;
    probe_config.max_memory_bytes = 0;
    probe_config.memory_policy = MapMemoryPolicy::StopAccumulating;
    TiledMapStore probe(probe_config);

    size_t minimum_normal_growth = std::numeric_limits<size_t>::max();
    size_t trigger_tile_count = 0;
    size_t bytes_before_trigger = 0;
    for (size_t i = 0; i < 256; ++i) {
        const size_t before = probe.stats().estimated_bytes;
        PointCloudXYZI frame;
        frame.push_back(makeTestPoint(
            static_cast<float>(i) + 0.1f, 0.0f, 0.0f));
        probe.addFrame(frame, static_cast<double>(i));
        probe.consumeDirtyTiles(0, 0, 0);
        const size_t after = probe.stats().estimated_bytes;
        const size_t growth = after - before;
        if (i >= 2 && minimum_normal_growth !=
                std::numeric_limits<size_t>::max() &&
            growth > minimum_normal_growth + 8 * sizeof(void*))
        {
            trigger_tile_count = i + 1;
            bytes_before_trigger = before;
            break;
        }
        minimum_normal_growth = std::min(minimum_normal_growth, growth);
    }
    if (expect(trigger_tile_count > 1)) return 1;

    TiledMapStore::Config bounded_config = probe_config;
    bounded_config.max_memory_bytes = bytes_before_trigger;
    TiledMapStore bounded(bounded_config);
    for (size_t i = 0; i + 1 < trigger_tile_count; ++i) {
        PointCloudXYZI frame;
        frame.push_back(makeTestPoint(
            static_cast<float>(i) + 0.1f, 0.0f, 0.0f));
        bounded.addFrame(frame, static_cast<double>(i));
        bounded.consumeDirtyTiles(0, 0, 0);
    }
    if (expect(bounded.stats().estimated_bytes <=
               bounded_config.max_memory_bytes)) return 1;

    PointCloudXYZI rejected;
    rejected.push_back(makeTestPoint(
        static_cast<float>(trigger_tile_count - 1) + 0.1f,
        0.0f, 0.0f));
    bounded.addFrame(rejected, static_cast<double>(trigger_tile_count));
    const auto stats = bounded.stats();
    if (expect(stats.tile_count == trigger_tile_count - 1)) return 1;
    if (expect(stats.estimated_bytes <= bounded_config.max_memory_bytes)) return 1;
    if (expect(stats.pending_deletion_count == 0)) return 1;
    return 0;
}

static int testMapBuildWorkerNormalizesTileCapacityToOutputLimits() {
    MapBuildWorker::Config config;
    config.store.tile_size_m = 100.0;
    config.store.voxel_leaf_m = 1.0;
    config.store.max_memory_bytes = 0;
    config.max_points_per_update = 100;
    config.max_bytes_per_update = TileUpdate::estimatedBytesForPointCount(100);
    config.output.max_pending_points = 2;
    config.output.max_pending_bytes = TileUpdate::estimatedBytesForPointCount(1);
    MapBuildWorker worker(
        config, MapBuildWorker::PublishCallback{},
        [](const TileUpdate&, double) {});
    worker.start();
    PointCloudXYZI frame;
    frame.push_back(makeTestPoint(0.1f, 0.0f, 0.0f));
    frame.push_back(makeTestPoint(1.1f, 0.0f, 0.0f));
    frame.push_back(makeTestPoint(2.1f, 0.0f, 0.0f));
    if (expect(worker.enqueueFrame(std::move(frame), 1.0))) return 1;
    if (expect(worker.flushFor(std::chrono::seconds(2)))) return 1;
    worker.stop(true);
    const auto stats = worker.stats();
    if (expect(stats.store.voxel_count == 1)) return 1;
    if (expect(stats.store.points_rejected_tile_capacity == 2)) return 1;

    MapBuildWorker::Config batch_config;
    batch_config.store.tile_size_m = 10.0;
    batch_config.store.voxel_leaf_m = 1.0;
    batch_config.store.max_memory_bytes = 0;
    batch_config.max_tiles_per_update = 0;
    batch_config.max_points_per_update = 100;
    batch_config.max_bytes_per_update =
        TileUpdate::estimatedBytesForPointCount(100);
    batch_config.output.max_pending_tiles = 1;
    batch_config.output.max_pending_points = 2;
    batch_config.output.max_pending_bytes =
        TileUpdate::estimatedBytesForPointCount(2);
    std::vector<TileUpdate> published_tiles;
    MapBuildWorker batch_worker(
        batch_config, MapBuildWorker::PublishCallback{},
        [&](const TileUpdate& update, double) {
            published_tiles.push_back(update);
        });
    PointCloudXYZI multiple_tiles;
    multiple_tiles.push_back(makeTestPoint(0.1f, 0.0f, 0.0f));
    multiple_tiles.push_back(makeTestPoint(1.1f, 0.0f, 0.0f));
    multiple_tiles.push_back(makeTestPoint(10.1f, 0.0f, 0.0f));
    multiple_tiles.push_back(makeTestPoint(11.1f, 0.0f, 0.0f));
    batch_worker.start();
    if (expect(batch_worker.enqueueFrame(std::move(multiple_tiles), 2.0))) return 1;
    if (expect(batch_worker.stopFor(std::chrono::seconds(3), true))) return 1;
    const auto batch_stats = batch_worker.stats();
    if (expect(published_tiles.size() == 2)) return 1;
    if (expect(std::all_of(
            published_tiles.begin(), published_tiles.end(),
            [](const TileUpdate& update) { return update.points.size() == 2; }))) return 1;
    if (expect(batch_stats.output.tiles_published == 2)) return 1;
    if (expect(batch_stats.output.current_tasks == 0)) return 1;
    if (expect(batch_stats.stop_timeouts == 0)) return 1;

    MapBuildWorker::Config invalid = config;
    invalid.output.max_pending_bytes =
        TileUpdate::estimatedBytesForPointCount(0);
    bool threw = false;
    try {
        MapBuildWorker invalid_worker(
            invalid, MapBuildWorker::PublishCallback{},
            [](const TileUpdate&, double) {});
    } catch (const std::invalid_argument&) {
        threw = true;
    }
    if (expect(threw)) return 1;
    return 0;
}

static int testFoxgloveOutputWorkerBoundsAndMergesTileQueue() {
    std::mutex gate_mutex;
    std::condition_variable gate_cv;
    bool first_callback_started = false;
    bool release_first_callback = false;
    std::vector<float> published_x;

    FoxgloveOutputWorker::Config config;
    config.max_pending_tiles = 3;
    // The hard memory admission includes the point currently owned by the
    // blocked callback, while current_points below reports queued ownership.
    config.max_pending_points = 3;
    config.max_pending_bytes = 0;
    FoxgloveOutputWorker worker(
        FoxgloveOutputWorker::PublishCallback{},
        [&](const TileUpdate& update, double) {
            const PointCloudXYZI& cloud = update.points;
            std::unique_lock<std::mutex> lock(gate_mutex);
            if (!cloud.empty()) published_x.push_back(cloud[0].x);
            if (!first_callback_started) {
                first_callback_started = true;
                gate_cv.notify_all();
                gate_cv.wait(lock, [&] { return release_first_callback; });
            }
        },
        config);
    worker.start();

    std::vector<TileUpdate> first_batch;
    first_batch.push_back(makeTestTileUpdate(0, 1, 0.0f));
    const bool first_accepted = worker.enqueueTiles(std::move(first_batch), 1.0);
    bool callback_started_in_time = false;
    {
        std::unique_lock<std::mutex> lock(gate_mutex);
        callback_started_in_time = gate_cv.wait_for(
            lock, std::chrono::seconds(2), [&] { return first_callback_started; });
    }
    if (!callback_started_in_time) {
        {
            std::lock_guard<std::mutex> lock(gate_mutex);
            release_first_callback = true;
        }
        gate_cv.notify_all();
        worker.stop(false);
        if (expect(callback_started_in_time)) return 1;
    }

    std::vector<TileUpdate> pending;
    pending.push_back(makeTestTileUpdate(1, 1, 1.0f));
    pending.push_back(makeTestTileUpdate(2, 1, 2.0f));
    const bool pending_accepted = worker.enqueueTiles(std::move(pending), 1.1);
    std::vector<TileUpdate> replacement;
    replacement.push_back(makeTestTileUpdate(1, 2, 10.0f));
    const bool replacement_accepted =
        worker.enqueueTiles(std::move(replacement), 1.2);
    std::vector<TileUpdate> overflow;
    overflow.push_back(makeTestTileUpdate(3, 1, 3.0f));
    const bool overflow_accepted = worker.enqueueTiles(std::move(overflow), 1.3);
    const FoxgloveOutputWorker::Stats queued_stats = worker.stats();

    {
        std::lock_guard<std::mutex> lock(gate_mutex);
        release_first_callback = true;
    }
    gate_cv.notify_all();
    const bool flushed = worker.flushFor(std::chrono::seconds(3));
    worker.stop(true);
    const FoxgloveOutputWorker::Stats final_stats = worker.stats();
    const auto timing_stats = worker.timingStats();

    if (expect(first_accepted)) return 1;
    if (expect(pending_accepted)) return 1;
    if (expect(replacement_accepted)) return 1;
    if (expect(!overflow_accepted)) return 1;
    if (expect(queued_stats.busy)) return 1;
    if (expect(queued_stats.current_tasks == 2)) return 1;
    if (expect(queued_stats.current_points == 2)) return 1;
    if (expect(queued_stats.tiles_merged == 1)) return 1;
    if (expect(queued_stats.tiles_dropped == 1)) return 1;
    if (expect(flushed)) return 1;
    if (expect(final_stats.tiles_published == 3)) return 1;
    if (expect(final_stats.failures == 0)) return 1;
    if (expect(timing_stats.tile_publish.total_samples == 3)) return 1;
    if (expect(timing_stats.tile_publish.window_samples == 3)) return 1;
    if (expect(timing_stats.tile_publish.p99_ms >=
               timing_stats.tile_publish.p95_ms)) return 1;
    if (expect(published_x.size() == 3)) return 1;
    if (expect(std::find(published_x.begin(), published_x.end(), 10.0f) !=
               published_x.end())) return 1;
    if (expect(std::find(published_x.begin(), published_x.end(), 1.0f) ==
               published_x.end())) return 1;
    return 0;
}

static int testFoxgloveOutputWorkerRejectsTileBatchesAtomically() {
    std::mutex gate_mutex;
    std::condition_variable gate_cv;
    bool first_callback_started = false;
    bool release_first_callback = false;
    std::vector<int32_t> published_keys;

    FoxgloveOutputWorker::Config config;
    config.max_pending_tiles = 2;
    config.max_pending_points = 2;
    config.max_pending_bytes = 0;
    FoxgloveOutputWorker worker(
        FoxgloveOutputWorker::PublishCallback{},
        [&](const TileUpdate& update, double) {
            std::unique_lock<std::mutex> lock(gate_mutex);
            published_keys.push_back(update.key.x);
            if (!first_callback_started) {
                first_callback_started = true;
                gate_cv.notify_all();
                gate_cv.wait(lock, [&] { return release_first_callback; });
            }
        },
        config);
    worker.start();

    if (expect(worker.enqueueTiles(
                   {makeTestTileUpdate(0, 1, 0.0f)}, 1.0))) return 1;
    {
        std::unique_lock<std::mutex> lock(gate_mutex);
        if (expect(gate_cv.wait_for(
                       lock, std::chrono::seconds(2),
                       [&] { return first_callback_started; }))) return 1;
    }
    if (expect(worker.enqueueTiles(
                   {makeTestTileUpdate(1, 1, 1.0f)}, 1.1))) return 1;

    std::vector<TileUpdate> retry_batch;
    retry_batch.push_back(makeTestTileUpdate(2, 1, 2.0f));
    retry_batch.push_back(makeTestTileUpdate(3, 1, 3.0f));
    if (expect(!worker.enqueueTiles(retry_batch, 1.2))) return 1;
    const auto rejected_stats = worker.stats();
    if (expect(rejected_stats.current_tasks == 1)) return 1;
    if (expect(rejected_stats.current_points == 1)) return 1;
    if (expect(rejected_stats.tiles_dropped == 2)) return 1;

    {
        std::lock_guard<std::mutex> lock(gate_mutex);
        release_first_callback = true;
    }
    gate_cv.notify_all();
    if (expect(worker.flushFor(std::chrono::seconds(2)))) return 1;

    if (expect(worker.enqueueTiles(std::move(retry_batch), 1.3))) return 1;
    if (expect(worker.flushFor(std::chrono::seconds(2)))) return 1;
    worker.stop(true);
    const auto final_stats = worker.stats();

    std::sort(published_keys.begin(), published_keys.end());
    if (expect(published_keys == std::vector<int32_t>({0, 1, 2, 3}))) return 1;
    if (expect(final_stats.tiles_published == 4)) return 1;
    if (expect(final_stats.tiles_enqueued == 6)) return 1;
    return 0;
}

static int testFoxgloveOutputWorkerDoesNotStarveFullSnapshot() {
    std::mutex gate_mutex;
    std::condition_variable gate_cv;
    bool first_tile_started = false;
    bool release_first_tile = false;
    std::vector<char> publish_order;

    FoxgloveOutputWorker::Config config;
    config.max_pending_tiles = 8;
    config.max_pending_points = 64;
    config.max_pending_bytes = 0;
    config.max_tile_burst_before_full = 2;
    FoxgloveOutputWorker worker(
        [&](const PointCloudXYZI&, double) {
            std::lock_guard<std::mutex> lock(gate_mutex);
            publish_order.push_back('F');
        },
        [&](const TileUpdate&, double) {
            std::unique_lock<std::mutex> lock(gate_mutex);
            publish_order.push_back('T');
            if (!first_tile_started) {
                first_tile_started = true;
                gate_cv.notify_all();
                gate_cv.wait(lock, [&] { return release_first_tile; });
            }
        },
        config);
    worker.start();
    if (expect(worker.enqueueTiles(
            {makeTestTileUpdate(0, 1, 0.0f)}, 1.0))) return 1;
    {
        std::unique_lock<std::mutex> lock(gate_mutex);
        if (expect(gate_cv.wait_for(
                lock, std::chrono::seconds(2),
                [&] { return first_tile_started; }))) return 1;
    }

    PointCloudXYZI snapshot;
    snapshot.push_back(makeTestPoint(100.0f, 0.0f, 0.0f));
    if (expect(worker.enqueueFull(std::move(snapshot), 1.1))) return 1;
    std::vector<TileUpdate> pending_tiles;
    for (int32_t key = 1; key <= 5; ++key) {
        pending_tiles.push_back(makeTestTileUpdate(
            key, 1, static_cast<float>(key)));
    }
    if (expect(worker.enqueueTiles(std::move(pending_tiles), 1.2))) return 1;

    {
        std::lock_guard<std::mutex> lock(gate_mutex);
        release_first_tile = true;
    }
    gate_cv.notify_all();
    if (expect(worker.flushFor(std::chrono::seconds(3)))) return 1;
    worker.stop(true);

    const auto full = std::find(
        publish_order.begin(), publish_order.end(), 'F');
    if (expect(full != publish_order.end())) return 1;
    if (expect(std::distance(publish_order.begin(), full) <= 2)) return 1;
    const auto stats = worker.stats();
    if (expect(stats.full_published == 1)) return 1;
    if (expect(stats.tiles_published == 6)) return 1;
    return 0;
}

static int testFoxgloveOutputWorkerRejectsStaleInflightTileVersion() {
    std::mutex gate_mutex;
    std::condition_variable gate_cv;
    bool first_callback_started = false;
    bool release_first_callback = false;
    std::vector<uint64_t> published_versions;

    FoxgloveOutputWorker worker(
        FoxgloveOutputWorker::PublishCallback{},
        [&](const TileUpdate& update, double) {
            std::unique_lock<std::mutex> lock(gate_mutex);
            published_versions.push_back(update.version);
            if (!first_callback_started) {
                first_callback_started = true;
                gate_cv.notify_all();
                gate_cv.wait(lock, [&] { return release_first_callback; });
            }
        });
    worker.start();
    if (expect(worker.enqueueTiles(
            {makeTestTileUpdate(7, 10, 10.0f)}, 1.0))) return 1;
    {
        std::unique_lock<std::mutex> lock(gate_mutex);
        if (expect(gate_cv.wait_for(
                lock, std::chrono::seconds(2),
                [&] { return first_callback_started; }))) return 1;
    }

    if (expect(worker.enqueueTiles(
            {makeTestTileUpdate(7, 9, 9.0f)}, 1.1))) return 1;
    auto stats = worker.stats();
    if (expect(stats.current_tasks == 0)) return 1;
    if (expect(stats.tiles_merged == 1)) return 1;
    if (expect(worker.enqueueTiles(
            {makeTestTileUpdate(7, 11, 11.0f)}, 1.2))) return 1;
    stats = worker.stats();
    if (expect(stats.current_tasks == 1)) return 1;

    {
        std::lock_guard<std::mutex> lock(gate_mutex);
        release_first_callback = true;
    }
    gate_cv.notify_all();
    if (expect(worker.flushFor(std::chrono::seconds(2)))) return 1;
    worker.stop(true);
    if (expect(published_versions == std::vector<uint64_t>({10, 11}))) return 1;
    return 0;
}

static int testFoxgloveOutputWorkerQueuesEqualInflightResyncVersion() {
    std::mutex gate_mutex;
    std::condition_variable gate_cv;
    bool first_callback_started = false;
    bool release_first_callback = false;
    std::vector<uint64_t> published_versions;

    FoxgloveOutputWorker::Config config;
    config.max_pending_tiles = 2;
    config.max_pending_points = 2;
    config.max_pending_bytes = 0;
    FoxgloveOutputWorker worker(
        FoxgloveOutputWorker::PublishCallback{},
        [&](const TileUpdate& update, double) {
            std::unique_lock<std::mutex> lock(gate_mutex);
            published_versions.push_back(update.version);
            if (!first_callback_started) {
                first_callback_started = true;
                gate_cv.notify_all();
                gate_cv.wait(lock, [&] { return release_first_callback; });
            }
        },
        config);
    worker.start();
    if (expect(worker.enqueueTiles(
            {makeTestTileUpdate(1, 10, 1.0f)}, 1.0))) return 1;
    {
        std::unique_lock<std::mutex> lock(gate_mutex);
        if (expect(gate_cv.wait_for(
                lock, std::chrono::seconds(2),
                [&] { return first_callback_started; }))) return 1;
    }

    TileUpdate resync = makeTestTileUpdate(1, 10, 1.0f);
    resync.resync = true;
    const bool resync_accepted = worker.enqueueTiles({resync}, 1.1);
    {
        std::lock_guard<std::mutex> lock(gate_mutex);
        release_first_callback = true;
    }
    gate_cv.notify_all();
    const bool flushed = worker.flushFor(std::chrono::seconds(3));
    worker.stop(true);

    if (expect(resync_accepted)) return 1;
    if (expect(flushed)) return 1;
    if (expect(published_versions == std::vector<uint64_t>({10, 10}))) return 1;
    return 0;
}

static int testFoxgloveOutputWorkerRetriesFailedDeletionInFixedSlot() {
    std::atomic<int> callback_attempts{0};
    std::atomic<int> failure_callbacks{0};
    std::vector<TileKey> attempted_keys;
    std::vector<uint64_t> attempted_versions;
    std::vector<bool> attempted_deletions;

    FoxgloveOutputWorker::Config config;
    config.max_pending_tiles = 1;
    config.max_pending_points = 1;
    config.max_pending_bytes = TileUpdate::estimatedBytesForPointCount(0);
    FoxgloveOutputWorker worker(
        FoxgloveOutputWorker::PublishCallback{},
        [&](const TileUpdate& update, double) {
            attempted_keys.push_back(update.key);
            attempted_versions.push_back(update.version);
            attempted_deletions.push_back(update.deleted);
            if (++callback_attempts == 1) {
                throw std::runtime_error("injected first deletion failure");
            }
        },
        config,
        [&](const TileUpdate& update, double) {
            if (update.deleted) ++failure_callbacks;
        });

    TileUpdate deletion;
    deletion.key = TileKey{42, -3, 7};
    deletion.version = 19;
    deletion.deleted = true;
    worker.start();
    if (expect(worker.enqueueTiles({deletion}, 4.2))) return 1;
    if (expect(worker.flushFor(std::chrono::seconds(2)))) return 1;
    worker.stop(true);

    const auto stats = worker.stats();
    if (expect(callback_attempts.load() == 2)) return 1;
    if (expect(failure_callbacks.load() == 1)) return 1;
    if (expect(attempted_keys ==
               std::vector<TileKey>({deletion.key, deletion.key}))) return 1;
    if (expect(attempted_versions ==
               std::vector<uint64_t>({deletion.version, deletion.version}))) return 1;
    if (expect(attempted_deletions == std::vector<bool>({true, true}))) return 1;
    if (expect(stats.tiles_enqueued == 1)) return 1;
    if (expect(stats.tiles_retried == 1)) return 1;
    if (expect(stats.tiles_published == 1)) return 1;
    if (expect(stats.tiles_dropped == 0)) return 1;
    if (expect(stats.tiles_cancelled == 0)) return 1;
    if (expect(stats.failures == 1)) return 1;
    if (expect(stats.current_tasks == 0)) return 1;
    if (expect(stats.current_points == 0)) return 1;
    if (expect(stats.current_bytes == 0)) return 1;
    return 0;
}

static int testFoxgloveOutputWorkerBoundedStopCancelsQueuedTiles() {
    std::mutex gate_mutex;
    std::condition_variable gate_cv;
    bool callback_entered = false;
    bool release_callback = false;
    std::atomic<int> callback_count{0};

    FoxgloveOutputWorker worker(
        FoxgloveOutputWorker::PublishCallback{},
        [&](const TileUpdate&, double) {
            ++callback_count;
            std::unique_lock<std::mutex> lock(gate_mutex);
            callback_entered = true;
            gate_cv.notify_all();
            gate_cv.wait(lock, [&] { return release_callback; });
        });
    worker.start();
    if (expect(worker.enqueueTiles({makeTestTileUpdate(0, 1, 0.0f)}, 1.0))) return 1;
    {
        std::unique_lock<std::mutex> lock(gate_mutex);
        if (expect(gate_cv.wait_for(lock, std::chrono::seconds(2),
                                    [&] { return callback_entered; }))) return 1;
    }
    if (expect(worker.enqueueTiles({makeTestTileUpdate(1, 1, 1.0f)}, 2.0))) return 1;

    const auto begin = std::chrono::steady_clock::now();
    if (expect(!worker.stopFor(std::chrono::milliseconds(50), false))) return 1;
    if (expect(std::chrono::steady_clock::now() - begin <
               std::chrono::milliseconds(500))) return 1;
    const auto timed_out_stats = worker.stats();
    if (expect(timed_out_stats.stopping)) return 1;
    if (expect(timed_out_stats.in_callback)) return 1;
    if (expect(!timed_out_stats.worker_exited)) return 1;
    if (expect(timed_out_stats.stop_timeouts == 1)) return 1;
    if (expect(timed_out_stats.tiles_cancelled == 1)) return 1;
    if (expect(timed_out_stats.current_tasks == 0)) return 1;

    {
        std::lock_guard<std::mutex> lock(gate_mutex);
        release_callback = true;
    }
    gate_cv.notify_all();
    if (expect(worker.stopFor(std::chrono::seconds(2), false))) return 1;
    const auto final_stats = worker.stats();
    if (expect(final_stats.worker_exited)) return 1;
    if (expect(!final_stats.stopping)) return 1;
    if (expect(callback_count == 1)) return 1;
    return 0;
}

static int testMapBuildWorkerContinuesWhileOutputCallbackIsBlocked() {
    std::mutex gate_mutex;
    std::condition_variable gate_cv;
    bool output_started = false;
    bool release_output = false;

    MapBuildWorker::Config config;
    config.store.tile_size_m = 10.0;
    config.store.voxel_leaf_m = 1.0;
    config.store.max_memory_bytes = 0;
    config.input_queue_capacity = 16;
    config.input_queue_max_points = 0;
    config.input_queue_max_bytes = 0;
    config.output.max_pending_tiles = 16;
    config.output.max_pending_points = 100;
    config.output.max_pending_bytes = 0;
    MapBuildWorker worker(
        config,
        MapBuildWorker::PublishCallback{},
        [&](const TileUpdate&, double) {
            std::unique_lock<std::mutex> lock(gate_mutex);
            if (!output_started) {
                output_started = true;
                gate_cv.notify_all();
                gate_cv.wait(lock, [&] { return release_output; });
            }
        });
    worker.start();

    PointCloudXYZI first;
    first.push_back(makeTestPoint(0.1f, 0.0f, 0.0f));
    bool all_enqueued = worker.enqueueFrame(std::move(first), 1.0);
    bool callback_started_in_time = false;
    {
        std::unique_lock<std::mutex> lock(gate_mutex);
        callback_started_in_time = gate_cv.wait_for(
            lock, std::chrono::seconds(2), [&] { return output_started; });
    }
    if (!callback_started_in_time) {
        {
            std::lock_guard<std::mutex> lock(gate_mutex);
            release_output = true;
        }
        gate_cv.notify_all();
        worker.stop(false);
        if (expect(callback_started_in_time)) return 1;
    }

    for (int i = 1; i <= 5; ++i) {
        PointCloudXYZI frame;
        frame.push_back(makeTestPoint(static_cast<float>(i * 10) + 0.1f,
                                      0.0f, 0.0f));
        all_enqueued = worker.enqueueFrame(std::move(frame), 1.0 + i) &&
                       all_enqueued;
    }
    const bool built_while_blocked = waitUntil(
        [&] { return worker.stats().frames_built == 6; },
        std::chrono::seconds(3));
    const MapBuildWorker::Stats blocked_stats = worker.stats();

    {
        std::lock_guard<std::mutex> lock(gate_mutex);
        release_output = true;
    }
    gate_cv.notify_all();
    const bool flushed = worker.flushFor(std::chrono::seconds(3));
    worker.stop(true);
    const MapBuildWorker::Stats final_stats = worker.stats();
    const auto timing_stats = worker.timingStats();

    if (expect(all_enqueued)) return 1;
    if (expect(built_while_blocked)) return 1;
    if (expect(blocked_stats.frames_built == 6)) return 1;
    if (expect(blocked_stats.store.voxel_count == 6)) return 1;
    if (expect(blocked_stats.output.busy)) return 1;
    if (expect(flushed)) return 1;
    if (expect(final_stats.frames_dropped == 0)) return 1;
    if (expect(final_stats.output.tiles_published == 6)) return 1;
    if (expect(timing_stats.map_build.total_samples == 6)) return 1;
    if (expect(timing_stats.tile_extract.total_samples >= 6)) return 1;
    return 0;
}

static int testMapBuildWorkerResyncsTileAfterPublishFailure() {
    std::atomic<int> publish_attempts{0};
    MapBuildWorker::Config config;
    config.store.tile_size_m = 10.0;
    config.store.voxel_leaf_m = 1.0;
    config.store.max_memory_bytes = 0;
    config.output.max_pending_tiles = 4;
    config.output.max_pending_points = 16;
    config.output.max_pending_bytes = 0;

    MapBuildWorker worker(
        config, MapBuildWorker::PublishCallback{},
        [&](const TileUpdate&, double) {
            if (++publish_attempts == 1) {
                throw std::runtime_error("injected tile publish failure");
            }
        });
    worker.start();
    PointCloudXYZI frame;
    frame.push_back(makeTestPoint(0.1f, 0.0f, 0.0f));
    const bool accepted = worker.enqueueFrame(std::move(frame), 1.0);
    const bool republished = waitUntil(
        [&] { return publish_attempts.load() >= 2; },
        std::chrono::seconds(3));
    const bool flushed = worker.flushFor(std::chrono::seconds(3));
    worker.stop(true);
    const MapBuildWorker::Stats stats = worker.stats();

    if (expect(accepted)) return 1;
    if (expect(republished)) return 1;
    if (expect(flushed)) return 1;
    // The fixed output retry slot and the map-level resync are intentionally
    // redundant. Depending on which worker wins the handoff race, the
    // successful retry may itself be followed by the equal-version resync.
    if (expect(publish_attempts.load() >= 2 &&
               publish_attempts.load() <= 3)) return 1;
    if (expect(stats.tile_resync_requested == 1)) return 1;
    if (expect(stats.tile_resync_on_failure == 1)) return 1;
    if (expect(stats.tile_resync_completed == 1)) return 1;
    if (expect(stats.output.failures == 1)) return 1;
    if (expect(stats.output.tiles_published ==
               static_cast<uint64_t>(publish_attempts.load() - 1))) return 1;
    if (expect(stats.output.tiles_retried <= 1)) return 1;
    return 0;
}

static int testMapBuildWorkerBoundedStopPropagatesOutputTimeout() {
    std::mutex gate_mutex;
    std::condition_variable gate_cv;
    bool callback_entered = false;
    bool release_callback = false;
    std::atomic<int> callback_count{0};

    MapBuildWorker::Config config;
    config.store.tile_size_m = 1.0;
    config.store.voxel_leaf_m = 0.1;
    config.store.max_memory_bytes = 0;
    config.output.max_pending_tiles = 8;
    config.output.max_pending_points = 100;
    config.output.max_pending_bytes = 0;
    MapBuildWorker worker(
        config,
        MapBuildWorker::PublishCallback{},
        [&](const TileUpdate&, double) {
            ++callback_count;
            std::unique_lock<std::mutex> lock(gate_mutex);
            callback_entered = true;
            gate_cv.notify_all();
            gate_cv.wait(lock, [&] { return release_callback; });
        });
    worker.start();
    PointCloudXYZI first;
    first.push_back(makeTestPoint(0.1f, 0.0f, 0.0f));
    const bool first_accepted = worker.enqueueFrame(std::move(first), 1.0);
    bool entered_in_time = false;
    {
        std::unique_lock<std::mutex> lock(gate_mutex);
        entered_in_time = gate_cv.wait_for(
            lock, std::chrono::seconds(2), [&] { return callback_entered; });
    }
    if (!entered_in_time) {
        {
            std::lock_guard<std::mutex> lock(gate_mutex);
            release_callback = true;
        }
        gate_cv.notify_all();
        worker.stop(false);
        if (expect(entered_in_time)) return 1;
    }

    PointCloudXYZI second;
    second.push_back(makeTestPoint(2.1f, 0.0f, 0.0f));
    const bool second_accepted = worker.enqueueFrame(std::move(second), 2.0);
    const bool second_built = waitUntil(
        [&] { return worker.stats().frames_built == 2; },
        std::chrono::seconds(2));
    const auto begin = std::chrono::steady_clock::now();
    const bool first_stop = worker.stopFor(std::chrono::milliseconds(50), false);
    const auto elapsed = std::chrono::steady_clock::now() - begin;
    const auto timed_out_stats = worker.stats();

    {
        std::lock_guard<std::mutex> lock(gate_mutex);
        release_callback = true;
    }
    gate_cv.notify_all();
    const bool final_stop = worker.stopFor(std::chrono::seconds(2), false);
    const auto final_stats = worker.stats();

    if (expect(first_accepted)) return 1;
    if (expect(second_accepted)) return 1;
    if (expect(second_built)) return 1;
    if (expect(!first_stop)) return 1;
    if (expect(elapsed < std::chrono::milliseconds(500))) return 1;
    if (expect(timed_out_stats.stopping)) return 1;
    if (expect(timed_out_stats.worker_exited)) return 1;
    if (expect(timed_out_stats.stop_timeouts >= 1)) return 1;
    if (expect(timed_out_stats.output.in_callback)) return 1;
    if (expect(timed_out_stats.output.tiles_cancelled >= 1)) return 1;
    if (expect(final_stop)) return 1;
    if (expect(final_stats.worker_exited)) return 1;
    if (expect(!final_stats.stopping)) return 1;
    if (expect(callback_count == 1)) return 1;
    return 0;
}

static int testMapBuildWorkerBoundsInputQueue() {
    MapBuildWorker::Config reject_config;
    reject_config.store.max_memory_bytes = 0;
    reject_config.input_queue_capacity = 2;
    reject_config.input_queue_max_points = 2;
    reject_config.input_queue_max_bytes = 0;
    MapBuildWorker reject_worker(
        reject_config,
        MapBuildWorker::PublishCallback{},
        MapBuildWorker::TilePublishCallback{});
    reject_worker.start();
    PointCloudXYZI oversized;
    oversized.push_back(makeTestPoint(0.0f, 0.0f, 0.0f));
    oversized.push_back(makeTestPoint(1.0f, 0.0f, 0.0f));
    oversized.push_back(makeTestPoint(2.0f, 0.0f, 0.0f));
    const bool oversized_accepted =
        reject_worker.enqueueFrame(std::move(oversized), 1.0);
    reject_worker.flushFor(std::chrono::seconds(1));
    reject_worker.stop(true);
    const MapBuildWorker::Stats rejected_stats = reject_worker.stats();
    if (expect(oversized_accepted)) return 1;
    if (expect(rejected_stats.frames_enqueued == 1)) return 1;
    if (expect(rejected_stats.frames_dropped == 0)) return 1;
    if (expect(rejected_stats.input_incomplete == 1)) return 1;
    if (expect(rejected_stats.input_points_dropped == 1)) return 1;
    if (expect(rejected_stats.input_bytes_dropped ==
               sizeof(PointType))) return 1;
    if (expect(rejected_stats.incomplete)) return 1;
    if (expect(rejected_stats.frames_built == 1)) return 1;
    if (expect(rejected_stats.store.voxel_count == 2)) return 1;
    if (expect(rejected_stats.max_input_queue_points <= 2)) return 1;

    MapBuildWorker::Config capacity_config;
    capacity_config.store.tile_size_m = 100.0;
    capacity_config.store.voxel_leaf_m = 1.0;
    capacity_config.store.max_memory_bytes = 0;
    capacity_config.input_queue_capacity = 2;
    capacity_config.input_queue_max_points = 0;
    capacity_config.input_queue_max_bytes = 0;
    MapBuildWorker capacity_worker(
        capacity_config,
        MapBuildWorker::PublishCallback{},
        MapBuildWorker::TilePublishCallback{});

    PointCloudXYZI blocking_frame;
    blocking_frame.resize(750000);
    for (auto& point : blocking_frame.points) {
        point = makeTestPoint(0.1f, 0.0f, 0.0f);
    }
    capacity_worker.start();
    const bool blocking_accepted =
        capacity_worker.enqueueFrame(std::move(blocking_frame), 2.0);
    const bool observed_busy = waitUntil(
        [&] { return capacity_worker.stats().busy; },
        std::chrono::seconds(2));

    bool queued_frames_accepted = true;
    for (int i = 1; i <= 3; ++i) {
        PointCloudXYZI frame;
        frame.push_back(makeTestPoint(static_cast<float>(i * 10), 0.0f, 0.0f));
        queued_frames_accepted =
            capacity_worker.enqueueFrame(std::move(frame), 2.0 + i) &&
            queued_frames_accepted;
    }
    const bool capacity_flushed =
        capacity_worker.flushFor(std::chrono::seconds(5));
    capacity_worker.stop(true);
    const MapBuildWorker::Stats capacity_stats = capacity_worker.stats();

    if (expect(blocking_accepted)) return 1;
    if (expect(observed_busy)) return 1;
    if (expect(queued_frames_accepted)) return 1;
    if (expect(capacity_flushed)) return 1;
    if (expect(capacity_stats.frames_enqueued == 4)) return 1;
    if (expect(capacity_stats.frames_dropped == 0)) return 1;
    if (expect(capacity_stats.frames_built == 4)) return 1;
    if (expect(capacity_stats.input_incomplete == 0)) return 1;
    if (expect(capacity_stats.input_points_dropped == 0)) return 1;
    if (expect(!capacity_stats.incomplete)) return 1;
    if (expect(capacity_stats.input_tile_changes_merged >= 2)) return 1;
    if (expect(capacity_stats.max_input_queue_tasks <= 2)) return 1;
    if (expect(capacity_stats.input_queue_tasks == 0)) return 1;

    MapBuildWorker::Config spatial_config;
    spatial_config.store.tile_size_m = 1.0;
    spatial_config.store.voxel_leaf_m = 0.1;
    spatial_config.store.max_memory_bytes = 0;
    spatial_config.input_queue_capacity = 2;
    spatial_config.input_queue_max_points = 10;
    spatial_config.input_queue_max_bytes = 0;
    MapBuildWorker spatial_worker(
        spatial_config, MapBuildWorker::PublishCallback{},
        MapBuildWorker::TilePublishCallback{});
    spatial_worker.start();
    PointCloudXYZI spatial_frame;
    spatial_frame.push_back(makeTestPoint(0.1f, 0.0f, 0.0f));
    spatial_frame.push_back(makeTestPoint(1.1f, 0.0f, 0.0f));
    spatial_frame.push_back(makeTestPoint(2.1f, 0.0f, 0.0f));
    const bool spatial_accepted =
        spatial_worker.enqueueFrame(std::move(spatial_frame), 10.0);
    const bool spatial_flushed =
        spatial_worker.flushFor(std::chrono::seconds(2));
    spatial_worker.stop(true);
    const auto spatial_stats = spatial_worker.stats();
    if (expect(spatial_accepted)) return 1;
    if (expect(spatial_flushed)) return 1;
    if (expect(spatial_stats.frames_built == 1)) return 1;
    if (expect(spatial_stats.frames_dropped == 0)) return 1;
    if (expect(spatial_stats.input_incomplete == 1)) return 1;
    if (expect(spatial_stats.input_points_dropped == 1)) return 1;
    if (expect(spatial_stats.input_tile_changes_dropped == 1)) return 1;
    if (expect(spatial_stats.store.voxel_count == 2)) return 1;
    if (expect(spatial_stats.max_input_queue_tasks <= 2)) return 1;

    MapBuildWorker::Config byte_config;
    byte_config.store.tile_size_m = 100.0;
    byte_config.store.voxel_leaf_m = 0.1;
    byte_config.store.max_memory_bytes = 0;
    byte_config.input_queue_capacity = 4;
    byte_config.input_queue_max_points = 0;
    byte_config.input_queue_max_bytes = 8192;
    MapBuildWorker byte_worker(
        byte_config, MapBuildWorker::PublishCallback{},
        MapBuildWorker::TilePublishCallback{});
    byte_worker.start();
    PointCloudXYZI byte_frame;
    for (int i = 0; i < 500; ++i) {
        byte_frame.push_back(makeTestPoint(
            0.01f * static_cast<float>(i), 0.0f, 0.0f));
    }
    const bool byte_accepted =
        byte_worker.enqueueFrame(std::move(byte_frame), 11.0);
    const bool byte_flushed = byte_worker.flushFor(std::chrono::seconds(2));
    byte_worker.stop(true);
    const auto byte_stats = byte_worker.stats();
    if (expect(byte_accepted)) return 1;
    if (expect(byte_flushed)) return 1;
    if (expect(byte_stats.max_input_queue_bytes <=
               byte_config.input_queue_max_bytes)) return 1;
    if (expect(byte_stats.input_points_dropped > 0)) return 1;
    if (expect(byte_stats.input_incomplete == 1)) return 1;
    return 0;
}

static int testMapBuildWorkerKeepsFullRequestBehindWholeSourceFrame() {
    std::mutex snapshot_mutex;
    std::vector<PointCloudXYZI> snapshots;
    MapBuildWorker::Config config;
    config.store.tile_size_m = 10.0;
    config.store.voxel_leaf_m = 1.0;
    config.store.max_memory_bytes = 0;
    config.input_queue_capacity = 8;
    config.input_queue_max_points = 100;
    config.input_queue_max_bytes = 0;
    MapBuildWorker worker(
        config,
        [&](const PointCloudXYZI& cloud, double) {
            std::lock_guard<std::mutex> lock(snapshot_mutex);
            snapshots.push_back(cloud);
        },
        MapBuildWorker::TilePublishCallback{});
    worker.start();

    PointCloudXYZI before;
    before.push_back(makeTestPoint(0.1f, 0.0f, 0.0f));
    before.push_back(makeTestPoint(10.1f, 0.0f, 0.0f));
    const bool before_accepted = worker.enqueueFrame(std::move(before), 1.0);
    worker.request(1.5);
    PointCloudXYZI after;
    after.push_back(makeTestPoint(1.1f, 0.0f, 0.0f));
    after.push_back(makeTestPoint(11.1f, 0.0f, 0.0f));
    const bool after_accepted = worker.enqueueFrame(std::move(after), 2.0);
    const bool flushed = worker.flushFor(std::chrono::seconds(3));
    worker.stop(true);
    const auto stats = worker.stats();

    size_t snapshot_points = 0;
    {
        std::lock_guard<std::mutex> lock(snapshot_mutex);
        if (expect(snapshots.size() == 1)) return 1;
        snapshot_points = snapshots.front().size();
    }
    if (expect(before_accepted)) return 1;
    if (expect(after_accepted)) return 1;
    if (expect(flushed)) return 1;
    if (expect(snapshot_points == 2)) return 1;
    if (expect(stats.frames_enqueued == 2)) return 1;
    if (expect(stats.frames_built == 2)) return 1;
    if (expect(stats.frames_dropped == 0)) return 1;
    if (expect(stats.full_built == 1)) return 1;
    return 0;
}

static int testMapBuildWorkerCancelsCoalescedTileInput() {
    MapBuildWorker::Config config;
    config.store.tile_size_m = 100.0;
    config.store.voxel_leaf_m = 1.0;
    config.store.max_memory_bytes = 0;
    config.input_queue_capacity = 4;
    config.input_queue_max_points = 1000000;
    config.input_queue_max_bytes = 0;
    MapBuildWorker worker(
        config, MapBuildWorker::PublishCallback{},
        MapBuildWorker::TilePublishCallback{});
    worker.start();
    PointCloudXYZI blocking;
    blocking.resize(750000);
    for (auto& point : blocking.points) {
        point = makeTestPoint(0.1f, 0.0f, 0.0f);
    }
    const bool blocking_accepted =
        worker.enqueueFrame(std::move(blocking), 1.0);
    const bool observed_busy = waitUntil(
        [&] { return worker.stats().busy; }, std::chrono::seconds(2));
    bool queued = true;
    for (int i = 0; i < 20; ++i) {
        PointCloudXYZI frame;
        frame.push_back(makeTestPoint(10.1f + 0.01f * i, 0.0f, 0.0f));
        queued = worker.enqueueFrame(std::move(frame), 2.0 + i) && queued;
    }
    const bool stopped = worker.stopFor(std::chrono::seconds(3), false);
    const auto stats = worker.stats();
    if (expect(blocking_accepted)) return 1;
    if (expect(observed_busy)) return 1;
    if (expect(queued)) return 1;
    if (expect(stopped)) return 1;
    if (expect(stats.worker_exited)) return 1;
    if (expect(stats.input_queue_tasks == 0)) return 1;
    if (expect(stats.frames_cancelled > 0)) return 1;
    if (expect(stats.frames_built + stats.frames_dropped +
               stats.frames_cancelled >= stats.frames_enqueued)) return 1;
    return 0;
}

static int testStorageWorkerRealtimeRejectsOversizedTask() {
    ScopedStorageTestDirectory output("storage_worker_queue_test");
    StorageWorker::Config config;
    config.mode = StorageWorker::Mode::Realtime;
    config.queue_max_bytes = 1;
    config.enable_pcd = true;
    config.output_root = output.path().string();

    StorageWorker worker(config);
    if (expect(worker.start())) return 1;

    PointCloudXYZI cloud;
    cloud.push_back(makeTestPoint(1.0f, 2.0f, 3.0f));
    const bool accepted = worker.enqueuePcd(std::move(cloud));
    const StorageWorker::Stats stats = worker.stats();
    worker.stop(false);

    if (expect(!accepted)) return 1;
    if (expect(stats.tasks_enqueued == 0)) return 1;
    if (expect(stats.tasks_dropped == 1)) return 1;
    if (expect(stats.queue_tasks == 0)) return 1;
    if (expect(stats.queue_bytes == 0)) return 1;
    if (expect(!stats.failed)) return 1;
    return 0;
}

static int testBagWriterRoundTripsHeaderAndMessageThroughReader() {
    ScopedStorageTestDirectory output("bag_writer_stream_test");
    const auto bag_path = output.path() / "checked.bag";
    BagWriter writer;
    if (expect(writer.open(bag_path.string()))) return 1;
    const uint32_t connection = writer.addConnection(
        "/raw", "std_msgs/UInt8", "7c8164229e7d2c17eb95e9231617fdee",
        "uint8 data\n");
    const uint8_t payload = 42;
    writer.writeMessage(connection, 123, &payload, sizeof(payload));
    writer.close();

    // Validate the ROS v2 layout: header bytes plus data padding occupy the
    // 4096-byte reservation, while the two length prefixes sit outside it.
    // The padding is record data, not bytes hidden by a fixed-offset jump.
    std::ifstream layout(bag_path, std::ios::binary);
    std::string version;
    std::getline(layout, version);
    uint32_t header_size = 0;
    uint32_t padding_size = 0;
    layout.read(reinterpret_cast<char*>(&header_size), sizeof(header_size));
    layout.seekg(header_size, std::ios::cur);
    layout.read(reinterpret_cast<char*>(&padding_size), sizeof(padding_size));
    if (expect(static_cast<bool>(layout))) return 1;
    if (expect(version == "#ROSBAG V2.0")) return 1;
    if (expect(header_size + padding_size == 4096)) return 1;

    bag::BagFileReader reader;
    if (expect(reader.open(bag_path.string()))) return 1;
    if (expect(reader.connCount() == 1)) return 1;
    if (expect(reader.chunkCount() == 1)) return 1;
    if (expect(reader.connections().size() == 1)) return 1;
    if (expect(reader.chunkInfos().size() == 1)) return 1;
    if (expect(reader.findConnectionByTopic("/raw") ==
               static_cast<int>(connection))) return 1;
    if (expect(reader.chunkInfos()[0].conn_counts.at(connection) == 1)) return 1;

    std::vector<bag::Message> messages;
    if (expect(reader.readMessagesByTopic(
            "/raw", [&](uint32_t conn_id, uint64_t time_ns,
                         const uint8_t* data, size_t size) {
                bag::Message message;
                message.conn_id = conn_id;
                message.time_ns = time_ns;
                message.data.assign(data, data + size);
                messages.push_back(std::move(message));
            }))) return 1;
    if (expect(messages.size() == 1)) return 1;
    if (expect(messages[0].conn_id == connection)) return 1;
    if (expect(messages[0].time_ns == 123)) return 1;
    if (expect(messages[0].data.size() == 1 &&
               messages[0].data[0] == payload)) return 1;
    reader.close();

    bool closed_write_threw = false;
    try {
        writer.writeMessage(connection, 456, &payload, sizeof(payload));
    } catch (...) {
        closed_write_threw = true;
    }
    if (expect(closed_write_threw)) return 1;

    BagWriter oversized_writer;
    const auto oversized_path = output.path() / "oversized.bag";
    if (expect(oversized_writer.open(oversized_path.string()))) return 1;
    const uint32_t oversized_connection = oversized_writer.addConnection(
        "/raw", "std_msgs/UInt8", "7c8164229e7d2c17eb95e9231617fdee",
        "uint8 data\n");
    bool oversized_write_threw = false;
    try {
        oversized_writer.writeMessage(
            oversized_connection, 789, &payload,
            static_cast<size_t>(std::numeric_limits<uint32_t>::max()) + 1ULL);
    } catch (...) {
        oversized_write_threw = true;
    }
    if (expect(oversized_write_threw)) return 1;
    oversized_writer.close();
    return 0;
}

static int testBagWriterEncodesRosV2FieldsIndependently() {
    ScopedStorageTestDirectory output("bag_writer_ros_v2_fields_test");
    const auto bag_path = output.path() / "fields.bag";
    constexpr uint64_t kLateTime = 5000000700ULL;
    constexpr uint64_t kEarlyTime = 2000000900ULL;
    const uint8_t late_payload = 31;
    const uint8_t early_payload = 41;
    const uint8_t equal_time_payload = 51;

    BagWriter writer;
    if (expect(writer.open(bag_path.string()))) return 1;
    const uint32_t connection = writer.addConnection(
        "/raw", "std_msgs/UInt8", "7c8164229e7d2c17eb95e9231617fdee",
        "uint8 data\n");
    writer.writeMessage(connection, kLateTime, &late_payload, 1);
    writer.writeMessage(connection, kEarlyTime, &early_payload, 1);
    writer.writeMessage(connection, kEarlyTime, &equal_time_payload, 1);
    writer.close();

    std::ifstream stream(bag_path, std::ios::binary);
    std::string version;
    std::getline(stream, version);
    if (expect(version == "#ROSBAG V2.0")) return 1;

    RawBagRecord file_header;
    if (expect(readRawBagRecord(stream, file_header))) return 1;
    if (expect(file_header.offset == 13)) return 1;
    if (expect(rawBagOp(file_header) == 0x03)) return 1;
    const auto* index_pos_field = rawBagField(file_header, "index_pos");
    const auto* connection_count_field = rawBagField(file_header, "conn_count");
    const auto* chunk_count_field = rawBagField(file_header, "chunk_count");
    if (expect(index_pos_field != nullptr && index_pos_field->size() == 8)) return 1;
    if (expect(connection_count_field != nullptr &&
               connection_count_field->size() == 4)) return 1;
    if (expect(chunk_count_field != nullptr && chunk_count_field->size() == 4)) return 1;
    uint64_t index_pos = 0;
    uint32_t connection_count = 0;
    uint32_t chunk_count = 0;
    if (expect(decodeRawBagU64(index_pos_field, index_pos))) return 1;
    if (expect(decodeRawBagU32(connection_count_field, connection_count))) return 1;
    if (expect(decodeRawBagU32(chunk_count_field, chunk_count))) return 1;
    if (expect(connection_count == 1)) return 1;
    if (expect(chunk_count == 1)) return 1;
    const size_t encoded_header_fields =
        4 + std::string("op=").size() + 1 +
        4 + std::string("index_pos=").size() + 8 +
        4 + std::string("conn_count=").size() + 4 +
        4 + std::string("chunk_count=").size() + 4;
    if (expect(encoded_header_fields + file_header.data.size() == 4096)) return 1;
    if (expect(static_cast<uint64_t>(stream.tellg()) == 13 + 4104)) return 1;

    RawBagRecord chunk;
    if (expect(readRawBagRecord(stream, chunk))) return 1;
    if (expect(rawBagOp(chunk) == 0x05)) return 1;
    const auto* chunk_size_field = rawBagField(chunk, "size");
    uint32_t chunk_size = 0;
    if (expect(chunk_size_field != nullptr && chunk_size_field->size() == 4)) return 1;
    if (expect(decodeRawBagU32(chunk_size_field, chunk_size))) return 1;
    if (expect(chunk_size == chunk.data.size())) return 1;

    std::vector<RawBagRecord> message_records;
    size_t chunk_offset = 0;
    while (chunk_offset < chunk.data.size()) {
        RawBagRecord message;
        if (expect(readRawBagRecord(chunk.data, chunk_offset, message))) return 1;
        if (expect(rawBagOp(message) == 0x02)) return 1;
        message_records.push_back(std::move(message));
    }
    if (expect(message_records.size() == 3)) return 1;
    const uint64_t expected_physical_times[] = {
        kLateTime, kEarlyTime, kEarlyTime};
    const uint8_t expected_payloads[] = {
        late_payload, early_payload, equal_time_payload};
    for (size_t i = 0; i < message_records.size(); ++i) {
        const auto* time_field = rawBagField(message_records[i], "time");
        uint32_t sec = 0;
        uint32_t nsec = 0;
        if (expect(time_field != nullptr && time_field->size() == 8)) return 1;
        if (expect(decodeRawBagTime(time_field, sec, nsec))) return 1;
        if (expect(sec == expected_physical_times[i] / 1000000000ULL)) return 1;
        if (expect(nsec == expected_physical_times[i] % 1000000000ULL)) return 1;
        if (expect(message_records[i].data.size() == 1 &&
                   message_records[i].data[0] == expected_payloads[i])) return 1;
    }

    RawBagRecord index_data;
    if (expect(readRawBagRecord(stream, index_data))) return 1;
    if (expect(rawBagOp(index_data) == 0x04)) return 1;
    uint32_t index_version = 0;
    uint32_t index_connection = 0;
    uint32_t index_count = 0;
    if (expect(rawBagField(index_data, "ver") != nullptr &&
               rawBagField(index_data, "ver")->size() == 4)) return 1;
    if (expect(decodeRawBagU32(rawBagField(index_data, "ver"),
                              index_version))) return 1;
    if (expect(decodeRawBagU32(rawBagField(index_data, "conn"),
                              index_connection))) return 1;
    if (expect(decodeRawBagU32(rawBagField(index_data, "count"),
                              index_count))) return 1;
    if (expect(index_version == 1)) return 1;
    if (expect(index_connection == connection)) return 1;
    if (expect(index_count == 3)) return 1;
    if (expect(index_data.data.size() == 3 * 12)) return 1;
    const uint64_t expected_index_times[] = {
        kEarlyTime, kEarlyTime, kLateTime};
    const uint32_t expected_index_offsets[] = {
        static_cast<uint32_t>(message_records[1].offset),
        static_cast<uint32_t>(message_records[2].offset),
        static_cast<uint32_t>(message_records[0].offset)};
    for (size_t i = 0; i < 3; ++i) {
        const size_t offset = i * 12;
        uint32_t sec = 0;
        uint32_t nsec = 0;
        uint32_t message_offset = 0;
        std::memcpy(&sec, index_data.data.data() + offset, 4);
        std::memcpy(&nsec, index_data.data.data() + offset + 4, 4);
        std::memcpy(&message_offset, index_data.data.data() + offset + 8, 4);
        if (expect(sec == expected_index_times[i] / 1000000000ULL)) return 1;
        if (expect(nsec == expected_index_times[i] % 1000000000ULL)) return 1;
        if (expect(message_offset == expected_index_offsets[i])) return 1;
    }
    if (expect(static_cast<uint64_t>(stream.tellg()) == index_pos)) return 1;

    RawBagRecord connection_record;
    if (expect(readRawBagRecord(stream, connection_record))) return 1;
    if (expect(rawBagOp(connection_record) == 0x07)) return 1;
    RawBagFields connection_fields;
    if (expect(parseRawBagFields(connection_record.data.data(),
                                 connection_record.data.size(),
                                 connection_fields))) return 1;
    const auto md5 = connection_fields.find("md5sum");
    if (expect(md5 != connection_fields.end())) return 1;
    if (expect(std::string(md5->second.begin(), md5->second.end()) ==
               "7c8164229e7d2c17eb95e9231617fdee")) return 1;

    RawBagRecord chunk_info;
    if (expect(readRawBagRecord(stream, chunk_info))) return 1;
    if (expect(rawBagOp(chunk_info) == 0x06)) return 1;
    uint32_t chunk_info_version = 0;
    uint64_t chunk_position = 0;
    if (expect(rawBagField(chunk_info, "ver") != nullptr &&
               rawBagField(chunk_info, "ver")->size() == 4)) return 1;
    if (expect(decodeRawBagU32(rawBagField(chunk_info, "ver"),
                              chunk_info_version))) return 1;
    if (expect(decodeRawBagU64(rawBagField(chunk_info, "chunk_pos"),
                              chunk_position))) return 1;
    if (expect(chunk_info_version == 1)) return 1;
    if (expect(chunk_position == chunk.offset)) return 1;
    uint32_t start_sec = 0;
    uint32_t start_nsec = 0;
    uint32_t end_sec = 0;
    uint32_t end_nsec = 0;
    if (expect(decodeRawBagTime(rawBagField(chunk_info, "start_time"),
                                start_sec, start_nsec))) return 1;
    if (expect(decodeRawBagTime(rawBagField(chunk_info, "end_time"),
                                end_sec, end_nsec))) return 1;
    if (expect(start_sec == 2 && start_nsec == 900)) return 1;
    if (expect(end_sec == 5 && end_nsec == 700)) return 1;

    bag::BagFileReader reader;
    if (expect(reader.open(bag_path.string()))) return 1;
    if (expect(reader.chunkInfos().size() == 1)) return 1;
    if (expect(reader.chunkInfos()[0].start_time == kEarlyTime)) return 1;
    if (expect(reader.chunkInfos()[0].end_time == kLateTime)) return 1;
    std::vector<uint64_t> reader_times;
    if (expect(reader.readMessagesByTopic(
            "/raw", [&](uint32_t, uint64_t time_ns,
                         const uint8_t*, size_t) {
                reader_times.push_back(time_ns);
            }))) return 1;
    const std::vector<uint64_t> expected_reader_times{
        kLateTime, kEarlyTime, kEarlyTime};
    if (expect(reader_times == expected_reader_times)) return 1;
    if (expect(ros_msg_defs::POSE_ARRAY_MD5 ==
               "916c28c5764443f268b296bb671b9d97")) return 1;
    if (expect(ros_msg_defs::POSE_STAMPED.rfind("PoseStamped\n", 0) != 0)) return 1;
    if (expect(ros_msg_defs::POINT_CLOUD2.rfind("PointCloud2\n", 0) != 0)) return 1;
    if (expect(ros_msg_defs::POSE_ARRAY.rfind("PoseArray\n", 0) != 0)) return 1;
    if (expect(ros_msg_defs::TF_MESSAGE.rfind(
                   "geometry_msgs/TransformStamped[] transforms\n", 0) == 0)) return 1;
    if (expect(ros_msg_defs::POINT_CLOUD2.find("uint8 FLOAT64=8\n") !=
               std::string::npos)) return 1;
    return 0;
}

static int testStorageWorkerRealtimePathReplacementPreservesBagOrder() {
    ScopedStorageTestDirectory output("storage_worker_path_order_test");
    const auto bag_path = output.path() / "ordered.bag";
    StorageWorker::Config config;
    config.mode = StorageWorker::Mode::Realtime;
    config.queue_max_bytes = 256ULL * 1024ULL * 1024ULL;
    config.bag_path_publish_hz = 1.0;
    config.bag_path = bag_path.string();
    config.enable_pcd = true;
    config.pcd_format = StorageWorker::PcdFormat::Binary;
    config.pcd_chunk_points = 750000;
    config.output_root = output.path().string();

    PointCloudXYZI first_blocker;
    PointCloudXYZI second_blocker;
    first_blocker.resize(config.pcd_chunk_points);
    second_blocker.resize(config.pcd_chunk_points);

    StorageWorker worker(config);
    if (expect(worker.start())) return 1;
    if (expect(worker.enqueuePcd(std::move(first_blocker)))) return 1;
    if (expect(waitUntil([&] { return worker.stats().busy; },
                         std::chrono::seconds(2)))) return 1;
    if (expect(worker.enqueuePcd(std::move(second_blocker)))) return 1;

    const std::vector<V3D> old_path{V3D(1.0, 0.0, 0.0)};
    const std::vector<V3D> new_path{
        V3D(1.0, 0.0, 0.0), V3D(2.0, 0.0, 0.0)};
    if (expect(worker.enqueuePath(1000000000ULL, old_path, "map"))) return 1;
    if (expect(worker.enqueueOdometryTf(
            2000000000ULL, V3D::Zero(), Eigen::Quaterniond::Identity()))) return 1;
    if (expect(worker.enqueuePath(3000000000ULL, new_path, "map"))) return 1;
    worker.stop(true);

    const StorageWorker::Stats stats = worker.stats();
    const StorageWorker::TimingStats timing = worker.timingStats();
    if (expect(stats.tasks_overwritten == 1)) return 1;
    if (expect(stats.tasks_dropped == 0)) return 1;
    if (expect(stats.tasks_cancelled == 0)) return 1;
    if (expect(stats.failures == 0)) return 1;
    if (expect(stats.tasks_written + stats.tasks_overwritten ==
               stats.tasks_enqueued)) return 1;
    if (expect(timing.write.total_samples == stats.tasks_written)) return 1;
    if (expect(timing.write.window_samples == stats.tasks_written)) return 1;
    if (expect(timing.bag_write.total_samples == 2)) return 1;
    if (expect(timing.pcd_write.total_samples == 2)) return 1;
    if (expect(timing.flush.total_samples == 1)) return 1;
    if (expect(timing.write.p99_ms >= timing.write.p95_ms)) return 1;
    if (expect(timing.flush.p99_ms >= timing.flush.p95_ms)) return 1;

    bag::BagFileReader reader;
    if (expect(reader.open(bag_path.string()))) return 1;
    std::vector<bag::Message> messages;
    if (expect(reader.readAllMessages(
            [&](uint32_t conn_id, uint64_t time_ns,
                const uint8_t* data, size_t size) {
                bag::Message message;
                message.conn_id = conn_id;
                message.time_ns = time_ns;
                message.data.assign(data, data + size);
                messages.push_back(std::move(message));
            }))) return 1;
    std::vector<std::pair<uint32_t, uint64_t>> ordered;
    for (const auto& message : messages) {
        // StorageWorker registers odometry=0, path=3, and TF=4.
        if (message.conn_id == 0 || message.conn_id == 3 ||
            message.conn_id == 4) {
            ordered.emplace_back(message.conn_id, message.time_ns);
        }
    }
    if (expect(ordered.size() == 3)) return 1;
    if (expect(ordered[0].first == 0 &&
               ordered[0].second == 2000000000ULL)) return 1;
    if (expect(ordered[1].first == 4 &&
               ordered[1].second == 2000000000ULL)) return 1;
    if (expect(ordered[2].first == 3 &&
               ordered[2].second == 3000000000ULL)) return 1;
    reader.close();
    return 0;
}

static int testStorageWorkerFlushWritesPartialPcdChunk() {
    ScopedStorageTestDirectory output("storage_worker_flush_test");
    StorageWorker::Config config;
    config.mode = StorageWorker::Mode::Realtime;
    config.queue_max_bytes = 1024 * 1024;
    config.pcd_format = StorageWorker::PcdFormat::Binary;
    config.pcd_chunk_points = 100;
    config.enable_pcd = true;
    config.output_root = output.path().string();

    StorageWorker worker(config);
    if (expect(worker.start())) return 1;

    PointCloudXYZI cloud;
    cloud.push_back(makeTestPoint(1.0f, 2.0f, 3.0f, 4.0f));
    cloud.push_back(makeTestPoint(5.0f, 6.0f, 7.0f, 8.0f));
    cloud.push_back(makeTestPoint(9.0f, 10.0f, 11.0f, 12.0f));
    if (expect(worker.enqueuePcd(std::move(cloud)))) return 1;
    if (expect(worker.flushFor(std::chrono::seconds(5)))) return 1;

    const StorageWorker::Stats flushed_stats = worker.stats();
    const StorageWorker::TimingStats flushed_timing = worker.timingStats();
    const auto files_before_stop = storagePcdFiles(output.path());
    worker.stop(true);
    const auto files_after_stop = storagePcdFiles(output.path());

    if (expect(flushed_stats.tasks_written == 1)) return 1;
    if (expect(flushed_stats.pcd_pending_points == 0)) return 1;
    if (expect(flushed_stats.pcd_pending_bytes == 0)) return 1;
    if (expect(flushed_stats.pcd_chunks_written == 1)) return 1;
    if (expect(flushed_timing.write.total_samples == 1)) return 1;
    if (expect(flushed_timing.write.window_samples == 1)) return 1;
    if (expect(flushed_timing.bag_write.total_samples == 0)) return 1;
    if (expect(flushed_timing.pcd_write.total_samples == 1)) return 1;
    if (expect(flushed_timing.flush.total_samples == 1)) return 1;
    if (expect(flushed_timing.flush.window_samples == 1)) return 1;
    if (expect(flushed_timing.flush.average_ms >= 0.0)) return 1;
    if (expect(files_before_stop.size() == 1)) return 1;
    if (expect(files_after_stop.size() == 1)) return 1;
    if (expect(files_after_stop.front().filename() == "scans_final_0.pcd")) return 1;

    PointCloudXYZI loaded;
    if (expect(pcl::io::loadPCDFile<PointType>(
                   files_after_stop.front().string(), loaded) == 0)) return 1;
    if (expect(loaded.size() == 3)) return 1;
    if (expect(near(loaded.points[0].x, 1.0f))) return 1;
    if (expect(near(loaded.points[2].intensity, 12.0f))) return 1;
    return 0;
}

static int testStorageWorkerFlushReportsPcdFailure() {
    ScopedStorageTestDirectory output("storage_worker_flush_failure_test");
    StorageWorker::Config config;
    config.mode = StorageWorker::Mode::Realtime;
    config.queue_max_bytes = 1024 * 1024;
    config.pcd_format = StorageWorker::PcdFormat::Binary;
    config.pcd_chunk_points = 100;
    config.enable_pcd = true;
    config.output_root = output.path().string();
    std::atomic<bool> injected{false};
    config.pcd_io_hook = [&injected](const char* operation) {
        if (std::strcmp(operation, "write") == 0 &&
            !injected.exchange(true)) {
            throw std::runtime_error("injected flush PCD failure");
        }
    };

    StorageWorker worker(config);
    if (expect(worker.start())) return 1;
    PointCloudXYZI cloud;
    cloud.resize(3);
    if (expect(worker.enqueuePcd(std::move(cloud)))) return 1;

    const bool first_flush = worker.flushFor(std::chrono::seconds(5));
    const bool later_flush = worker.flushFor(std::chrono::seconds(5));
    const StorageWorker::Stats stats = worker.stats();
    worker.stop(true);

    if (expect(!first_flush)) return 1;
    if (expect(!later_flush)) return 1;
    if (expect(injected.load())) return 1;
    if (expect(stats.failures == 1)) return 1;
    if (expect(stats.pcd_failures == 1)) return 1;
    if (expect(stats.pcd_disabled)) return 1;
    if (expect(stats.pcd_tasks_dropped == 1)) return 1;
    if (expect(stats.pcd_points_dropped == 3)) return 1;
    if (expect(storagePcdFiles(output.path()).empty())) return 1;
    return 0;
}

static int testStorageWorkerRealtimeFusesBagFailures() {
    {
        ScopedStorageTestDirectory output("storage_worker_bag_write_failure_test");
        StorageWorker::Config config;
        config.mode = StorageWorker::Mode::Realtime;
        config.queue_max_bytes = 1024 * 1024;
        config.bag_path = (output.path() / "write_failure.bag").string();
        config.enable_pcd = true;
        config.pcd_format = StorageWorker::PcdFormat::Binary;
        config.pcd_chunk_points = 100;
        config.output_root = output.path().string();
        std::atomic<bool> injected{false};
        config.bag_io_hook = [&injected](const char* operation) {
            if (std::strcmp(operation, "write_message") == 0 &&
                !injected.exchange(true)) {
                throw std::runtime_error("injected bag write failure");
            }
        };

        StorageWorker worker(config);
        if (expect(worker.start())) return 1;
        if (expect(worker.enqueueOdometryTf(
                1000000000ULL, V3D::Zero(),
                Eigen::Quaterniond::Identity()))) return 1;
        if (expect(waitUntil([&] { return worker.stats().bag_disabled; },
                             std::chrono::seconds(5)))) return 1;

        const StorageWorker::Stats failed_stats = worker.stats();
        if (expect(injected.load())) return 1;
        if (expect(!worker.isBagOpen())) return 1;
        if (expect(worker.isRunning())) return 1;
        if (expect(failed_stats.failures == 1)) return 1;
        if (expect(failed_stats.bag_failures == 1)) return 1;
        if (expect(failed_stats.tasks_dropped == 1)) return 1;
        if (expect(!failed_stats.failed)) return 1;
        if (expect(failed_stats.bag_disabled)) return 1;
        if (expect(!worker.enqueueOdometryTf(
                2000000000ULL, V3D::Zero(),
                Eigen::Quaterniond::Identity()))) return 1;

        PointCloudXYZI cloud;
        cloud.push_back(makeTestPoint(1.0f, 2.0f, 3.0f, 4.0f));
        if (expect(worker.enqueuePcd(std::move(cloud)))) return 1;
        if (expect(worker.flushFor(std::chrono::seconds(5)))) return 1;
        worker.stop(true);

        const StorageWorker::Stats stopped_stats = worker.stats();
        const auto pcd_files = storagePcdFiles(output.path());
        if (expect(!stopped_stats.failed)) return 1;
        if (expect(stopped_stats.bag_disabled)) return 1;
        if (expect(stopped_stats.bag_failures == 1)) return 1;
        if (expect(stopped_stats.pcd_chunks_written == 1)) return 1;
        if (expect(pcd_files.size() == 1)) return 1;
    }

    ScopedStorageTestDirectory output("storage_worker_bag_close_failure_test");
    StorageWorker::Config config;
    config.mode = StorageWorker::Mode::Realtime;
    config.queue_max_bytes = 1024 * 1024;
    config.bag_path = (output.path() / "close_failure.bag").string();
    config.output_root = output.path().string();
    std::atomic<bool> injected{false};
    config.bag_io_hook = [&injected](const char* operation) {
        if (std::strcmp(operation, "close") == 0 &&
            !injected.exchange(true)) {
            throw std::runtime_error("injected bag close failure");
        }
    };

    StorageWorker worker(config);
    if (expect(worker.start())) return 1;
    if (expect(worker.enqueueOdometryTf(
            3000000000ULL, V3D::Zero(),
            Eigen::Quaterniond::Identity()))) return 1;
    if (expect(worker.flushFor(std::chrono::seconds(5)))) return 1;
    bool stop_threw = false;
    try {
        worker.stop(true);
    } catch (...) {
        stop_threw = true;
    }
    const StorageWorker::Stats stats = worker.stats();
    if (expect(!stop_threw)) return 1;
    if (expect(injected.load())) return 1;
    if (expect(!worker.isRunning())) return 1;
    if (expect(!worker.isBagOpen())) return 1;
    if (expect(stats.tasks_written == 1)) return 1;
    if (expect(stats.failures == 1)) return 1;
    if (expect(stats.bag_failures == 1)) return 1;
    if (expect(stats.bag_disabled)) return 1;
    if (expect(!stats.failed)) return 1;
    return 0;
}

static int testStorageWorkerStopFinalizesAndContainsWriteFailures() {
    {
        ScopedStorageTestDirectory output("storage_worker_stop_test");
        StorageWorker::Config config;
        config.queue_max_bytes = 1024 * 1024;
        config.pcd_format = StorageWorker::PcdFormat::Binary;
        config.pcd_chunk_points = 100;
        config.enable_pcd = true;
        config.output_root = output.path().string();

        StorageWorker worker(config);
        if (expect(worker.start())) return 1;
        PointCloudXYZI cloud;
        cloud.push_back(makeTestPoint(13.0f, 14.0f, 15.0f));
        if (expect(worker.enqueuePcd(std::move(cloud)))) return 1;

        bool stop_threw = false;
        try {
            worker.stop(true);
            worker.stop(true);
        } catch (...) {
            stop_threw = true;
        }
        const StorageWorker::Stats stats = worker.stats();
        const auto files = storagePcdFiles(output.path());
        if (expect(!stop_threw)) return 1;
        if (expect(!worker.isRunning())) return 1;
        if (expect(stats.tasks_written == 1)) return 1;
        if (expect(stats.pcd_chunks_written == 1)) return 1;
        if (expect(stats.pcd_pending_points == 0)) return 1;
        if (expect(!stats.failed)) return 1;
        if (expect(files.size() == 1)) return 1;
    }

    ScopedStorageTestDirectory failure_output("storage_worker_stop_failure_test");
    StorageWorker::Config failure_config;
    failure_config.queue_max_bytes = 1024 * 1024;
    failure_config.pcd_format = StorageWorker::PcdFormat::Binary;
    failure_config.pcd_chunk_points = 100;
    failure_config.enable_pcd = true;
    failure_config.output_root = failure_output.path().string();

    StorageWorker failure_worker(failure_config);
    if (expect(failure_worker.start())) return 1;
    std::filesystem::create_directory(
        failure_output.path() / "PCD" / "scans_final_0.pcd");
    PointCloudXYZI failure_cloud;
    failure_cloud.push_back(makeTestPoint(1.0f, 1.0f, 1.0f));
    if (expect(failure_worker.enqueuePcd(std::move(failure_cloud)))) return 1;

    bool failure_stop_threw = false;
    try {
        failure_worker.stop(true);
    } catch (...) {
        failure_stop_threw = true;
    }
    const StorageWorker::Stats failure_stats = failure_worker.stats();
    if (expect(!failure_stop_threw)) return 1;
    if (expect(!failure_worker.isRunning())) return 1;
    if (expect(failure_stats.failures == 1)) return 1;
    if (expect(failure_stats.failed)) return 1;
    return 0;
}

static int testStorageWorkerRealtimePcdFailurePreservesBagAndAccountsLoss() {
    ScopedStorageTestDirectory output("storage_worker_pcd_realtime_failure_test");
    StorageWorker::Config config;
    config.mode = StorageWorker::Mode::Realtime;
    config.queue_max_bytes = 1024 * 1024;
    config.bag_path = (output.path() / "healthy.bag").string();
    config.enable_pcd = true;
    config.pcd_format = StorageWorker::PcdFormat::Binary;
    config.pcd_chunk_points = 100;
    config.output_root = output.path().string();
    std::atomic<bool> injected{false};
    config.pcd_io_hook = [&injected](const char* operation) {
        if (std::strcmp(operation, "write") == 0 &&
            !injected.exchange(true)) {
            throw std::runtime_error("injected PCD write failure");
        }
    };

    StorageWorker worker(config);
    if (expect(worker.start())) return 1;
    PointCloudXYZI cloud;
    cloud.resize(150);
    if (expect(worker.enqueuePcd(std::move(cloud)))) return 1;
    if (expect(waitUntil([&] { return worker.stats().pcd_disabled; },
                         std::chrono::seconds(5)))) return 1;

    const auto failure_stats = worker.stats();
    if (expect(injected.load())) return 1;
    if (expect(!worker.isPcdEnabled())) return 1;
    if (expect(worker.isBagOpen())) return 1;
    if (expect(worker.isRunning())) return 1;
    if (expect(!failure_stats.failed)) return 1;
    if (expect(failure_stats.pcd_failures == 1)) return 1;
    if (expect(failure_stats.pcd_tasks_dropped == 1)) return 1;
    if (expect(failure_stats.pcd_points_dropped == 150)) return 1;
    if (expect(failure_stats.pcd_bytes_dropped ==
               150 * sizeof(PointType))) return 1;
    if (expect(failure_stats.pcd_pending_points == 0)) return 1;

    if (expect(worker.enqueueOdometryTf(
            1000000000ULL, V3D::Zero(),
            Eigen::Quaterniond::Identity()))) return 1;
    if (expect(worker.flushFor(std::chrono::seconds(5)))) return 1;
    worker.stop(true);
    if (expect(storagePcdFiles(output.path()).empty())) return 1;

    bag::BagFileReader reader;
    if (expect(reader.open(config.bag_path))) return 1;
    size_t messages = 0;
    if (expect(reader.readAllMessages(
            [&](uint32_t, uint64_t, const uint8_t*, size_t) {
                ++messages;
            }))) return 1;
    if (expect(messages == 2)) return 1;
    reader.close();
    return 0;
}

static int testStorageWorkerReliablePcdFailureStopsWithRemainderReported() {
    ScopedStorageTestDirectory output("storage_worker_pcd_reliable_failure_test");
    StorageWorker::Config config;
    config.mode = StorageWorker::Mode::Reliable;
    config.queue_max_bytes = 1024 * 1024;
    config.bag_path = (output.path() / "healthy.bag").string();
    config.enable_pcd = true;
    config.pcd_format = StorageWorker::PcdFormat::Binary;
    config.pcd_chunk_points = 100;
    config.output_root = output.path().string();

    std::mutex hook_mutex;
    std::condition_variable hook_cv;
    bool hook_entered = false;
    bool release_hook = false;
    config.pcd_io_hook = [&](const char*) {
        std::unique_lock<std::mutex> lock(hook_mutex);
        hook_entered = true;
        hook_cv.notify_all();
        hook_cv.wait(lock, [&] { return release_hook; });
        throw std::runtime_error("injected reliable PCD write failure");
    };

    StorageWorker worker(config);
    if (expect(worker.start())) return 1;
    PointCloudXYZI failing_cloud;
    failing_cloud.resize(150);
    if (expect(worker.enqueuePcd(std::move(failing_cloud)))) return 1;
    {
        std::unique_lock<std::mutex> lock(hook_mutex);
        if (expect(hook_cv.wait_for(lock, std::chrono::seconds(5),
                                    [&] { return hook_entered; }))) return 1;
    }

    PointCloudXYZI queued_cloud;
    queued_cloud.resize(7);
    if (expect(worker.enqueuePcd(std::move(queued_cloud)))) return 1;
    if (expect(worker.enqueueOdometryTf(
            2000000000ULL, V3D::Zero(),
            Eigen::Quaterniond::Identity()))) return 1;
    {
        std::lock_guard<std::mutex> lock(hook_mutex);
        release_hook = true;
    }
    hook_cv.notify_all();

    if (expect(waitUntil([&] { return worker.stats().worker_exited; },
                         std::chrono::seconds(5)))) return 1;
    const auto failed_stats = worker.stats();
    if (expect(failed_stats.failed)) return 1;
    if (expect(failed_stats.pcd_disabled)) return 1;
    if (expect(failed_stats.pcd_failures == 1)) return 1;
    if (expect(failed_stats.pcd_pending_points == 150)) return 1;
    if (expect(failed_stats.pcd_pending_tasks == 1)) return 1;
    if (expect(failed_stats.pcd_queue_points == 7)) return 1;
    if (expect(failed_stats.pcd_queue_tasks == 1)) return 1;
    if (expect(failed_stats.queue_tasks == 2)) return 1;
    if (expect(failed_stats.pcd_points_dropped == 0)) return 1;
    if (expect(worker.stopFor(std::chrono::seconds(1), true))) return 1;
    const auto stopped_stats = worker.stats();
    if (expect(stopped_stats.pcd_pending_points == 150)) return 1;
    if (expect(stopped_stats.pcd_queue_points == 7)) return 1;
    if (expect(stopped_stats.queue_tasks == 2)) return 1;
    if (expect(!worker.startFor(std::chrono::milliseconds(0)))) return 1;
    if (expect(worker.stopFor(std::chrono::seconds(1), false))) return 1;
    const auto cancelled_stats = worker.stats();
    if (expect(cancelled_stats.queue_tasks == 0)) return 1;
    if (expect(cancelled_stats.pcd_pending_points == 0)) return 1;
    if (expect(cancelled_stats.pcd_pending_capacity_points == 0)) return 1;
    if (expect(cancelled_stats.tasks_cancelled == 3)) return 1;
    if (expect(cancelled_stats.bag_tasks_cancelled == 1)) return 1;
    if (expect(cancelled_stats.pcd_tasks_cancelled == 2)) return 1;
    if (expect(cancelled_stats.pcd_points_cancelled == 157)) return 1;
    return 0;
}

static int testStorageWorkerBagStartupFailureLeavesPcdUsable() {
    ScopedStorageTestDirectory output("storage_worker_bag_start_failure_test");
    StorageWorker::Config config;
    config.mode = StorageWorker::Mode::Reliable;
    config.queue_max_bytes = 1024 * 1024;
    config.bag_path =
        (output.path() / "missing_parent" / "unavailable.bag").string();
    config.enable_pcd = true;
    config.pcd_format = StorageWorker::PcdFormat::Binary;
    config.pcd_chunk_points = 100;
    config.output_root = output.path().string();

    StorageWorker worker(config);
    if (expect(worker.start())) return 1;
    const auto startup_stats = worker.stats();
    if (expect(!worker.isBagOpen())) return 1;
    if (expect(worker.isPcdEnabled())) return 1;
    if (expect(worker.isRunning())) return 1;
    if (expect(startup_stats.bag_disabled)) return 1;
    if (expect(startup_stats.bag_failures == 1)) return 1;
    if (expect(!startup_stats.failed)) return 1;

    PointCloudXYZI cloud;
    cloud.resize(3);
    if (expect(worker.enqueuePcd(std::move(cloud)))) return 1;
    if (expect(worker.flushFor(std::chrono::seconds(5)))) return 1;
    worker.stop(true);
    const auto files = storagePcdFiles(output.path());
    if (expect(files.size() == 1)) return 1;
    PointCloudXYZI loaded;
    if (expect(pcl::io::loadPCDFile<PointType>(files.front().string(), loaded) == 0)) return 1;
    if (expect(loaded.size() == 3)) return 1;
    return 0;
}

static int testStorageWorkerReliableBagFuseLeavesPcdUsable() {
    ScopedStorageTestDirectory output("storage_worker_bag_reliable_fuse_test");
    StorageWorker::Config config;
    config.mode = StorageWorker::Mode::Reliable;
    config.queue_max_bytes = 1024 * 1024;
    config.bag_path = (output.path() / "failure.bag").string();
    config.enable_pcd = true;
    config.pcd_format = StorageWorker::PcdFormat::Binary;
    config.pcd_chunk_points = 100;
    config.output_root = output.path().string();
    std::atomic<bool> injected{false};
    config.bag_io_hook = [&injected](const char* operation) {
        if (std::strcmp(operation, "write_message") == 0 &&
            !injected.exchange(true)) {
            throw std::runtime_error("injected reliable Bag write failure");
        }
    };

    StorageWorker worker(config);
    if (expect(worker.start())) return 1;
    if (expect(worker.enqueueOdometryTf(
            1000000000ULL, V3D::Zero(),
            Eigen::Quaterniond::Identity()))) return 1;
    if (expect(waitUntil([&] { return worker.stats().bag_disabled; },
                         std::chrono::seconds(5)))) return 1;
    const auto bag_failure_stats = worker.stats();
    if (expect(!bag_failure_stats.failed)) return 1;
    if (expect(bag_failure_stats.tasks_dropped == 1)) return 1;
    if (expect(worker.isRunning())) return 1;
    if (expect(worker.isPcdEnabled())) return 1;

    PointCloudXYZI cloud;
    cloud.resize(4);
    if (expect(worker.enqueuePcd(std::move(cloud)))) return 1;
    if (expect(worker.flushFor(std::chrono::seconds(5)))) return 1;
    worker.stop(true);
    if (expect(storagePcdFiles(output.path()).size() == 1)) return 1;
    return 0;
}

static int testStorageWorkerPcdChunkFrameTrigger() {
    ScopedStorageTestDirectory output("storage_worker_pcd_frame_chunk_test");
    StorageWorker::Config config;
    config.queue_max_bytes = 1024 * 1024;
    config.enable_pcd = true;
    config.pcd_format = StorageWorker::PcdFormat::Binary;
    config.pcd_chunk_points = 100;
    config.pcd_chunk_frames = 2;
    config.output_root = output.path().string();

    StorageWorker worker(config);
    if (expect(worker.start())) return 1;
    for (int frame = 0; frame < 2; ++frame) {
        PointCloudXYZI cloud;
        cloud.resize(3);
        if (expect(worker.enqueuePcd(std::move(cloud)))) return 1;
    }
    if (expect(waitUntil([&] { return worker.stats().pcd_chunks_written == 1; },
                         std::chrono::seconds(5)))) return 1;
    worker.stop(true);
    const auto stats = worker.stats();
    const auto files = storagePcdFiles(output.path());
    if (expect(stats.pcd_chunk_frames_limit == 2)) return 1;
    if (expect(stats.pcd_pending_points == 0)) return 1;
    if (expect(files.size() == 1)) return 1;
    PointCloudXYZI loaded;
    if (expect(pcl::io::loadPCDFile<PointType>(files.front().string(), loaded) == 0)) return 1;
    if (expect(loaded.size() == 6)) return 1;
    return 0;
}

static int testStorageWorkerBoundedNonDrainStopCancelsQueue() {
    ScopedStorageTestDirectory output("storage_worker_cancel_test");
    StorageWorker::Config config;
    config.queue_max_bytes = 64ULL * 1024ULL * 1024ULL;
    config.pcd_format = StorageWorker::PcdFormat::Binary;
    config.pcd_chunk_points = 0;
    config.enable_pcd = true;
    config.output_root = output.path().string();

    StorageWorker worker(config);
    if (expect(worker.start())) return 1;
    bool all_accepted = true;
    for (int batch = 0; batch < 64; ++batch) {
        PointCloudXYZI cloud;
        cloud.resize(1024);
        for (auto& point : cloud.points) {
            point = makeTestPoint(static_cast<float>(batch), 0.0f, 0.0f);
        }
        all_accepted = worker.enqueuePcd(std::move(cloud)) && all_accepted;
    }

    const bool stopped = worker.stopFor(std::chrono::seconds(2), false);
    const auto stats = worker.stats();
    const auto files = storagePcdFiles(output.path());
    if (expect(all_accepted)) return 1;
    if (expect(stopped)) return 1;
    if (expect(stats.worker_exited)) return 1;
    if (expect(!stats.stopping)) return 1;
    if (expect(stats.stop_timeouts == 0)) return 1;
    if (expect(stats.tasks_written + stats.tasks_cancelled ==
               stats.tasks_enqueued)) return 1;
    if (expect(stats.queue_tasks == 0)) return 1;
    if (expect(stats.queue_bytes == 0)) return 1;
    if (expect(stats.pcd_pending_points == 0)) return 1;
    if (expect(stats.pcd_chunk_points_limit ==
               StorageWorker::Config::kDefaultPcdChunkPoints)) return 1;
    if (expect(files.empty())) return 1;
    return 0;
}

static int testStorageWorkerHardCapIncludesInflightTask() {
    ScopedStorageTestDirectory output("storage_worker_inflight_cap_test");
    constexpr size_t kPointCount = 8192;
    const size_t payload_bytes = kPointCount * sizeof(PointType);
    StorageWorker::Config config;
    config.mode = StorageWorker::Mode::Realtime;
    config.queue_max_bytes = payload_bytes * 3 / 2 + 4096;
    config.enable_pcd = true;
    config.pcd_format = StorageWorker::PcdFormat::Binary;
    config.pcd_chunk_points = kPointCount;
    config.output_root = output.path().string();

    std::mutex hook_mutex;
    std::condition_variable hook_cv;
    bool hook_entered = false;
    bool release_hook = false;
    config.pcd_io_hook = [&](const char*) {
        std::unique_lock<std::mutex> lock(hook_mutex);
        hook_entered = true;
        hook_cv.notify_all();
        hook_cv.wait_for(lock, std::chrono::seconds(5),
                         [&] { return release_hook; });
        throw std::runtime_error("finish inflight-cap test");
    };

    StorageWorker worker(config);
    if (expect(worker.start())) return 1;
    PointCloudXYZI first;
    first.resize(kPointCount);
    if (expect(worker.enqueuePcd(std::move(first)))) return 1;
    {
        std::unique_lock<std::mutex> lock(hook_mutex);
        if (expect(hook_cv.wait_for(lock, std::chrono::seconds(5),
                                    [&] { return hook_entered; }))) return 1;
    }

    const auto inflight_stats = worker.stats();
    if (expect(inflight_stats.queue_tasks == 0)) return 1;
    if (expect(inflight_stats.inflight_tasks == 1)) return 1;
    if (expect(inflight_stats.inflight_points == kPointCount)) return 1;
    if (expect(inflight_stats.inflight_bytes > payload_bytes)) return 1;
    if (expect(inflight_stats.hard_usage_bytes ==
               inflight_stats.inflight_bytes)) return 1;
    if (expect(inflight_stats.hard_usage_bytes <= config.queue_max_bytes)) return 1;
    if (expect(inflight_stats.pcd_pending_capacity_points >= kPointCount)) return 1;
    if (expect(inflight_stats.pcd_pending_bytes >= payload_bytes)) return 1;
    if (expect(inflight_stats.pcd_scratch_bytes >= payload_bytes)) return 1;

    PointCloudXYZI rejected;
    rejected.resize(kPointCount);
    if (expect(!worker.enqueuePcd(std::move(rejected)))) return 1;
    const auto rejected_stats = worker.stats();
    if (expect(rejected_stats.tasks_dropped == 1)) return 1;
    if (expect(rejected_stats.queue_tasks == 0)) return 1;
    if (expect(rejected_stats.hard_usage_bytes <= config.queue_max_bytes)) return 1;
    if (expect(rejected_stats.max_hard_usage_bytes <= config.queue_max_bytes)) return 1;

    {
        std::lock_guard<std::mutex> lock(hook_mutex);
        release_hook = true;
    }
    hook_cv.notify_all();
    if (expect(waitUntil([&] { return worker.stats().pcd_disabled; },
                         std::chrono::seconds(5)))) return 1;
    worker.stop(true);
    const auto stopped_stats = worker.stats();
    if (expect(stopped_stats.inflight_tasks == 0)) return 1;
    if (expect(stopped_stats.pcd_pending_capacity_points == 0)) return 1;
    if (expect(stopped_stats.pcd_pending_bytes == 0)) return 1;
    if (expect(stopped_stats.pcd_scratch_bytes == 0)) return 1;
    return 0;
}

static int testStorageWorkerNonDrainCancelsUncommittedPendingPcd() {
    ScopedStorageTestDirectory output("storage_worker_pending_cancel_test");
    StorageWorker::Config config;
    config.queue_max_bytes = 4ULL * 1024ULL * 1024ULL;
    config.enable_pcd = true;
    config.pcd_format = StorageWorker::PcdFormat::Binary;
    config.pcd_chunk_points = 100;
    config.output_root = output.path().string();

    StorageWorker worker(config);
    if (expect(worker.start())) return 1;
    PointCloudXYZI cloud;
    cloud.reserve(10000);
    cloud.resize(3);
    if (expect(worker.enqueuePcd(std::move(cloud)))) return 1;
    if (expect(waitUntil([&] {
            const auto stats = worker.stats();
            return stats.queue_tasks == 0 && !stats.busy &&
                   stats.pcd_pending_points == 3;
        }, std::chrono::seconds(5)))) return 1;
    const auto pending_stats = worker.stats();
    if (expect(pending_stats.tasks_written == 0)) return 1;
    if (expect(pending_stats.pcd_pending_tasks == 1)) return 1;
    if (expect(pending_stats.pcd_pending_capacity_points >= 10000)) return 1;

    if (expect(worker.stopFor(std::chrono::seconds(5), false))) return 1;
    const auto stopped_stats = worker.stats();
    if (expect(stopped_stats.tasks_written == 0)) return 1;
    if (expect(stopped_stats.tasks_cancelled == 1)) return 1;
    if (expect(stopped_stats.pcd_tasks_cancelled == 1)) return 1;
    if (expect(stopped_stats.pcd_points_cancelled == 3)) return 1;
    if (expect(stopped_stats.pcd_bytes_cancelled ==
               3 * sizeof(PointType))) return 1;
    if (expect(stopped_stats.pcd_pending_points == 0)) return 1;
    if (expect(stopped_stats.pcd_pending_capacity_points == 0)) return 1;
    if (expect(stopped_stats.pcd_pending_bytes == 0)) return 1;
    if (expect(storagePcdFiles(output.path()).empty())) return 1;
    return 0;
}

static int testStorageWorkerReliableAdmissionFailureStopsAndPreservesQueue() {
    ScopedStorageTestDirectory output("storage_worker_reliable_cap_test");
    constexpr size_t kPointCount = 8192;
    const size_t payload_bytes = kPointCount * sizeof(PointType);
    StorageWorker::Config config;
    config.mode = StorageWorker::Mode::Reliable;
    config.queue_max_bytes = payload_bytes * 5 / 2 + 4096;
    config.enable_pcd = true;
    config.pcd_format = StorageWorker::PcdFormat::Binary;
    config.pcd_chunk_points = kPointCount;
    config.output_root = output.path().string();

    std::mutex hook_mutex;
    std::condition_variable hook_cv;
    bool hook_entered = false;
    bool release_hook = false;
    config.pcd_io_hook = [&](const char*) {
        std::unique_lock<std::mutex> lock(hook_mutex);
        hook_entered = true;
        hook_cv.notify_all();
        hook_cv.wait_for(lock, std::chrono::seconds(5),
                         [&] { return release_hook; });
    };

    StorageWorker worker(config);
    if (expect(worker.start())) return 1;
    PointCloudXYZI first;
    first.resize(kPointCount);
    if (expect(worker.enqueuePcd(std::move(first)))) return 1;
    {
        std::unique_lock<std::mutex> lock(hook_mutex);
        if (expect(hook_cv.wait_for(lock, std::chrono::seconds(5),
                                    [&] { return hook_entered; }))) return 1;
    }
    PointCloudXYZI queued;
    queued.resize(kPointCount);
    if (expect(worker.enqueuePcd(std::move(queued)))) return 1;
    const auto within_cap = worker.stats();
    if (expect(within_cap.inflight_tasks == 1)) return 1;
    if (expect(within_cap.queue_tasks == 1)) return 1;
    if (expect(within_cap.hard_usage_bytes <= config.queue_max_bytes)) return 1;

    PointCloudXYZI rejected;
    rejected.resize(kPointCount);
    if (expect(!worker.enqueuePcd(std::move(rejected)))) return 1;
    const auto rejected_stats = worker.stats();
    if (expect(rejected_stats.failed)) return 1;
    if (expect(rejected_stats.tasks_dropped == 1)) return 1;
    if (expect(rejected_stats.queue_tasks == 1)) return 1;
    {
        std::lock_guard<std::mutex> lock(hook_mutex);
        release_hook = true;
    }
    hook_cv.notify_all();

    if (expect(waitUntil([&] { return worker.stats().worker_exited; },
                         std::chrono::seconds(5)))) return 1;
    const auto exited_stats = worker.stats();
    if (expect(exited_stats.tasks_written == 1)) return 1;
    if (expect(exited_stats.queue_tasks == 1)) return 1;
    if (expect(exited_stats.pcd_queue_points == kPointCount)) return 1;
    if (expect(exited_stats.inflight_tasks == 0)) return 1;
    if (expect(exited_stats.hard_usage_bytes == exited_stats.queue_bytes)) return 1;
    if (expect(worker.stopFor(std::chrono::seconds(1), true))) return 1;
    if (expect(worker.stats().queue_tasks == 1)) return 1;
    return 0;
}

static int testStorageWorkerStartForTimesOutWithoutFalseSuccess() {
    ScopedStorageTestDirectory output("storage_worker_start_timeout_test");
    StorageWorker::Config config;
    config.enable_pcd = true;
    config.output_root = output.path().string();
    std::mutex hook_mutex;
    std::condition_variable hook_cv;
    bool hook_entered = false;
    bool release_hook = false;
    config.startup_io_hook = [&](const char*) {
        std::unique_lock<std::mutex> lock(hook_mutex);
        hook_entered = true;
        hook_cv.notify_all();
        hook_cv.wait_for(lock, std::chrono::seconds(5),
                         [&] { return release_hook; });
    };

    StorageWorker worker(config);
    const auto begin = std::chrono::steady_clock::now();
    if (expect(!worker.startFor(std::chrono::milliseconds(50)))) return 1;
    const auto elapsed = std::chrono::steady_clock::now() - begin;
    if (expect(elapsed < std::chrono::seconds(1))) return 1;
    const auto timeout_stats = worker.stats();
    if (expect(timeout_stats.startup_timeouts == 1)) return 1;
    if (expect(timeout_stats.failed)) return 1;
    if (expect(timeout_stats.stopping)) return 1;
    if (expect(!worker.isRunning())) return 1;
    {
        std::unique_lock<std::mutex> lock(hook_mutex);
        if (expect(hook_cv.wait_for(lock, std::chrono::seconds(1),
                                    [&] { return hook_entered; }))) return 1;
        release_hook = true;
    }
    hook_cv.notify_all();
    if (expect(worker.stopFor(std::chrono::seconds(5), false))) return 1;
    const auto stopped_stats = worker.stats();
    if (expect(stopped_stats.worker_exited)) return 1;
    if (expect(!worker.isPcdEnabled())) return 1;
    if (expect(stopped_stats.startup_timeouts == 1)) return 1;
    if (expect(!worker.start())) return 1;
    return 0;
}

static int testStorageWorkerBagOnlyReliableFailureStopsConsumer() {
    ScopedStorageTestDirectory output("storage_worker_bag_only_fatal_test");
    StorageWorker::Config config;
    config.mode = StorageWorker::Mode::Reliable;
    config.queue_max_bytes = 1024 * 1024;
    config.bag_path = (output.path() / "fatal.bag").string();
    config.output_root = output.path().string();

    std::mutex hook_mutex;
    std::condition_variable hook_cv;
    bool hook_entered = false;
    bool release_hook = false;
    config.bag_io_hook = [&](const char* operation) {
        if (std::strcmp(operation, "write_message") != 0) return;
        std::unique_lock<std::mutex> lock(hook_mutex);
        hook_entered = true;
        hook_cv.notify_all();
        hook_cv.wait_for(lock, std::chrono::seconds(5),
                         [&] { return release_hook; });
        throw std::runtime_error("injected fatal Bag failure");
    };

    StorageWorker worker(config);
    if (expect(worker.start())) return 1;
    if (expect(worker.enqueueOdometryTf(
            1000000000ULL, V3D::Zero(),
            Eigen::Quaterniond::Identity()))) return 1;
    {
        std::unique_lock<std::mutex> lock(hook_mutex);
        if (expect(hook_cv.wait_for(lock, std::chrono::seconds(5),
                                    [&] { return hook_entered; }))) return 1;
    }
    if (expect(worker.enqueueOdometryTf(
            2000000000ULL, V3D::Zero(),
            Eigen::Quaterniond::Identity()))) return 1;
    {
        std::lock_guard<std::mutex> lock(hook_mutex);
        release_hook = true;
    }
    hook_cv.notify_all();

    if (expect(waitUntil([&] { return worker.stats().worker_exited; },
                         std::chrono::seconds(5)))) return 1;
    const auto stats = worker.stats();
    if (expect(stats.failed)) return 1;
    if (expect(stats.tasks_enqueued == 2)) return 1;
    if (expect(stats.tasks_written == 0)) return 1;
    if (expect(stats.tasks_dropped == 1)) return 1;
    if (expect(stats.tasks_cancelled == 1)) return 1;
    if (expect(stats.bag_tasks_cancelled == 1)) return 1;
    if (expect(stats.queue_tasks == 0)) return 1;
    if (expect(stats.inflight_tasks == 0)) return 1;
    if (expect(stats.tasks_written + stats.tasks_dropped +
               stats.tasks_cancelled == stats.tasks_enqueued)) return 1;
    if (expect(!worker.startFor(std::chrono::milliseconds(0)))) return 1;
    if (expect(worker.stopFor(std::chrono::seconds(1), true))) return 1;
    if (expect(!worker.start())) return 1;
    return 0;
}

static int testStorageWorkerRealtimePcdFuseAccountsQueuedRaceOnce() {
    ScopedStorageTestDirectory output("storage_worker_pcd_fuse_race_test");
    StorageWorker::Config config;
    config.mode = StorageWorker::Mode::Realtime;
    config.queue_max_bytes = 4ULL * 1024ULL * 1024ULL;
    config.bag_path = (output.path() / "healthy.bag").string();
    config.enable_pcd = true;
    config.pcd_format = StorageWorker::PcdFormat::Binary;
    config.pcd_chunk_points = 100;
    config.output_root = output.path().string();

    std::mutex hook_mutex;
    std::condition_variable hook_cv;
    bool hook_entered = false;
    bool release_hook = false;
    config.pcd_io_hook = [&](const char*) {
        std::unique_lock<std::mutex> lock(hook_mutex);
        hook_entered = true;
        hook_cv.notify_all();
        hook_cv.wait_for(lock, std::chrono::seconds(5),
                         [&] { return release_hook; });
        throw std::runtime_error("injected PCD fuse race");
    };

    StorageWorker worker(config);
    if (expect(worker.start())) return 1;
    PointCloudXYZI failing;
    failing.resize(150);
    if (expect(worker.enqueuePcd(std::move(failing)))) return 1;
    {
        std::unique_lock<std::mutex> lock(hook_mutex);
        if (expect(hook_cv.wait_for(lock, std::chrono::seconds(5),
                                    [&] { return hook_entered; }))) return 1;
    }
    PointCloudXYZI queued;
    queued.resize(7);
    if (expect(worker.enqueuePcd(std::move(queued)))) return 1;
    if (expect(worker.enqueueOdometryTf(
            1000000000ULL, V3D::Zero(),
            Eigen::Quaterniond::Identity()))) return 1;
    {
        std::lock_guard<std::mutex> lock(hook_mutex);
        release_hook = true;
    }
    hook_cv.notify_all();

    if (expect(waitUntil([&] {
            const auto stats = worker.stats();
            return stats.pcd_disabled && stats.tasks_written == 1 &&
                   stats.queue_tasks == 0;
        }, std::chrono::seconds(5)))) return 1;
    const auto stats = worker.stats();
    if (expect(stats.tasks_enqueued == 3)) return 1;
    if (expect(stats.tasks_written == 1)) return 1;
    if (expect(stats.tasks_dropped == 1)) return 1;
    if (expect(stats.tasks_cancelled == 1)) return 1;
    if (expect(stats.pcd_tasks_dropped == 1)) return 1;
    if (expect(stats.pcd_points_dropped == 150)) return 1;
    if (expect(stats.pcd_tasks_cancelled == 1)) return 1;
    if (expect(stats.pcd_points_cancelled == 7)) return 1;
    if (expect(stats.pcd_pending_points == 0)) return 1;
    if (expect(stats.pcd_pending_capacity_points == 0)) return 1;
    if (expect(stats.pcd_queue_tasks == 0)) return 1;
    if (expect(stats.tasks_written + stats.tasks_dropped +
               stats.tasks_cancelled == stats.tasks_enqueued)) return 1;
    worker.stop(true);
    return 0;
}

static int testStorageWorkerMultiChunkFailureCompactsRemainderOnce() {
    ScopedStorageTestDirectory output("storage_worker_multichunk_remainder_test");
    StorageWorker::Config config;
    config.mode = StorageWorker::Mode::Reliable;
    config.queue_max_bytes = 4ULL * 1024ULL * 1024ULL;
    config.enable_pcd = true;
    config.pcd_format = StorageWorker::PcdFormat::Binary;
    config.pcd_chunk_points = 100;
    config.output_root = output.path().string();
    std::atomic<int> writes{0};
    config.pcd_io_hook = [&writes](const char*) {
        if (writes.fetch_add(1) == 2) {
            throw std::runtime_error("injected third chunk failure");
        }
    };

    StorageWorker worker(config);
    if (expect(worker.start())) return 1;
    PointCloudXYZI cloud;
    cloud.resize(305);
    if (expect(worker.enqueuePcd(std::move(cloud)))) return 1;
    if (expect(waitUntil([&] { return worker.stats().worker_exited; },
                         std::chrono::seconds(5)))) return 1;
    const auto failed = worker.stats();
    if (expect(failed.failed)) return 1;
    if (expect(failed.pcd_chunks_written == 2)) return 1;
    if (expect(failed.tasks_written == 0)) return 1;
    if (expect(failed.pcd_pending_tasks == 1)) return 1;
    if (expect(failed.pcd_pending_points == 105)) return 1;
    if (expect(failed.pcd_pending_capacity_points >= 105)) return 1;
    if (expect(failed.pcd_pending_capacity_points < 305)) return 1;
    if (expect(failed.pcd_scratch_bytes == 0)) return 1;
    if (expect(storagePcdFiles(output.path()).size() == 2)) return 1;
    if (expect(worker.stopFor(std::chrono::seconds(1), true))) return 1;
    if (expect(worker.stopFor(std::chrono::seconds(1), false))) return 1;
    const auto cancelled = worker.stats();
    if (expect(cancelled.pcd_pending_points == 0)) return 1;
    if (expect(cancelled.pcd_pending_capacity_points == 0)) return 1;
    if (expect(cancelled.tasks_cancelled == 1)) return 1;
    if (expect(cancelled.pcd_tasks_cancelled == 1)) return 1;
    if (expect(cancelled.pcd_points_cancelled == 105)) return 1;
    return 0;
}

static int testGroupedConfigLegacyOverrideMigration() {
    YamlConfig legacy;
    char executable[] = "test";
    char disable_async[] = "async_full_map_publish=false";
    char enable_tiles[] = "publish_map_delta=true";
    char tile_rate[] = "map_output.tile_publish_hz=7";
    char path_bound[] = "storage.path_max_points=321";
    char point_filter[] = "point_filter_num=3";
    char* legacy_args[] = {
        executable, disable_async, enable_tiles, tile_rate, path_bound,
        point_filter};
    legacy.applyOverrides(6, legacy_args);
    if (expect(legacy.getConfig().map_output_mode == "tiled_incremental")) return 1;
    if (expect(legacy.getConfig().map_output_tile_publish_hz == 7)) return 1;
    if (expect(legacy.getConfig().storage_path_max_points == 321)) return 1;
    if (expect(legacy.getConfig().point_filter_num == 3)) return 1;
    std::string validation_error;
    if (expect(legacy.validate(validation_error))) return 1;

    YamlConfig explicit_mode;
    char executable2[] = "test";
    char enable_tiles2[] = "publish_map_delta=true";
    char explicit_hybrid[] = "map_output.mode=hybrid";
    char* explicit_args[] = {
        executable2, enable_tiles2, explicit_hybrid};
    explicit_mode.applyOverrides(3, explicit_args);
    if (expect(explicit_mode.getConfig().map_output_mode == "hybrid")) return 1;

    YamlConfig invalid_mode;
    char executable3[] = "test";
    char invalid_mode_override[] = "map_output.mode=typo";
    char* invalid_args[] = {executable3, invalid_mode_override};
    invalid_mode.applyOverrides(2, invalid_args);
    validation_error.clear();
    if (expect(!invalid_mode.validate(validation_error))) return 1;
    if (expect(!validation_error.empty())) return 1;

    YamlConfig invalid_grid;
    char executable4[] = "test";
    char invalid_voxel[] = "map_output.tile_voxel_leaf_m=3";
    char* invalid_grid_args[] = {executable4, invalid_voxel};
    invalid_grid.applyOverrides(2, invalid_grid_args);
    validation_error.clear();
    if (expect(!invalid_grid.validate(validation_error))) return 1;

    YamlConfig legacy_pcd_interval;
    char executable5[] = "test";
    char interval[] = "pcd_interval=17";
    char* pcd_args[] = {executable5, interval};
    legacy_pcd_interval.applyOverrides(2, pcd_args);
    if (expect(legacy_pcd_interval.getConfig().storage_pcd_chunk_frames == 17)) return 1;
    return 0;
}

int main(int argc, char** argv) {
    if (argc == 2 && std::string(argv[1]) == "--reader-sdk-only") {
        if (testPlaybackTerminalStatusAndLogSanitization()) return 1;
        if (testLvxOpenIgnoresEmptyTailFrame()) return 1;
        if (testLivoxAdapterStopForIsSafeWithoutInitializedSdk()) return 1;
        if (testLivoxAdapterContainsThrowingSdkCallbacks()) return 1;
        if (testLivoxAdapterStopForFromCallbackDoesNotSelfWait()) return 1;
        if (testBagReaderRejectsMalformedPointCountWithoutThrowing()) return 1;
        if (testBagReaderContainsThrowingFrameCallback()) return 1;
        if (testBagReaderStopForIsIdempotentBeforePlayback()) return 1;
        return 0;
    }
    if (argc == 2 && std::string(argv[1]) == "--storage-hardening-only") {
        if (testStorageWorkerRealtimeRejectsOversizedTask()) return 1;
        if (testStorageWorkerRealtimePathReplacementPreservesBagOrder()) return 1;
        if (testStorageWorkerFlushWritesPartialPcdChunk()) return 1;
        if (testStorageWorkerFlushReportsPcdFailure()) return 1;
        if (testStorageWorkerRealtimeFusesBagFailures()) return 1;
        if (testStorageWorkerStopFinalizesAndContainsWriteFailures()) return 1;
        if (testStorageWorkerRealtimePcdFailurePreservesBagAndAccountsLoss()) return 1;
        if (testStorageWorkerReliablePcdFailureStopsWithRemainderReported()) return 1;
        if (testStorageWorkerBagStartupFailureLeavesPcdUsable()) return 1;
        if (testStorageWorkerReliableBagFuseLeavesPcdUsable()) return 1;
        if (testStorageWorkerPcdChunkFrameTrigger()) return 1;
        if (testStorageWorkerBoundedNonDrainStopCancelsQueue()) return 1;
        if (testStorageWorkerHardCapIncludesInflightTask()) return 1;
        if (testStorageWorkerNonDrainCancelsUncommittedPendingPcd()) return 1;
        if (testStorageWorkerReliableAdmissionFailureStopsAndPreservesQueue()) return 1;
        if (testStorageWorkerStartForTimesOutWithoutFalseSuccess()) return 1;
        if (testStorageWorkerBagOnlyReliableFailureStopsConsumer()) return 1;
        if (testStorageWorkerRealtimePcdFuseAccountsQueuedRaceOnce()) return 1;
        if (testStorageWorkerMultiChunkFailureCompactsRemainderOnce()) return 1;
        return 0;
    }
    if (testBoundedTimingWindowKeepsLatestSamples()) return 1;
    if (testPlaybackTerminalStatusAndLogSanitization()) return 1;
    if (testLivoxSdkStubMatchesOfficialLayout()) return 1;

    LvxPoint src{};
    src.x = 1.0f;
    src.y = 2.0f;
    src.z = 3.0f;
    src.reflectivity = 42;
    src.tag = 9;
    src.line = 3;
    src.offset_time = 2500000;

    PointType pt = livoxToPointType(src);

    if (expect(pt.x == 1.0f)) return 1;
    if (expect(pt.y == 2.0f)) return 1;
    if (expect(pt.z == 3.0f)) return 1;
    if (expect(pt.intensity == 42.0f)) return 1;
    if (expect(pt.normal_x == 9.0f)) return 1;
    if (expect(pt.normal_y == 3.0f)) return 1;
    if (expect(pt.curvature == 2.5f)) return 1;

    if (expect(std::string(kFastLioMapFrame) == "map")) return 1;
    if (expect(std::string(kFastLioBodyFrame) == "base_link")) return 1;
    if (expect(std::string(kFastLioRegisteredCloudTopic) == "/cloud_registered")) return 1;
    if (expect(std::string(kFastLioMapTopic) == "/map")) return 1;
    if (expect(std::string(kFastLioMapDeltaTopic) == "/map_delta")) return 1;
    if (expect(std::string(kFastLioMapTilesTopic) == "/map_tiles")) return 1;
    if (expect(std::string(kFastLioImuTopic) == "/imu")) return 1;
    if (expect(std::string(kFastLioImuFrame) == "livox_imu")) return 1;

    YamlConfig config;
    if (expect(config.getConfig().point_filter_num == 3)) return 1;
    if (expect(config.getConfig().max_iteration == 3)) return 1;
    if (expect(config.getConfig().max_feature_points == 2000)) return 1;
    if (expect(config.getConfig().iekf_match_threads == 4)) return 1;
    if (expect(config.getConfig().cube_side_length == 1000)) return 1;
    if (expect(config.getConfig().runtime_pos_log == false)) return 1;
    if (expect(config.getConfig().publish_full_map == true)) return 1;
    if (expect(config.getConfig().async_full_map_publish == true)) return 1;
    if (expect(config.getConfig().full_map_publish_interval_ms == 1000)) return 1;
    if (expect(near(config.getConfig().full_map_voxel_size, 0.2))) return 1;
    if (expect(config.getConfig().bag_full_map_periodic == false)) return 1;
    if (expect(config.getConfig().publish_map_delta == false)) return 1;
    if (expect(config.getConfig().map_delta_max_pending_points == 200000)) return 1;
    if (expect(config.getConfig().foxglove_control_interval_ms == 20)) return 1;
    if (expect(config.getConfig().foxglove_backlog_size == 64)) return 1;
    if (expect(config.getConfig().map_output_mode == "full_async")) return 1;
    if (expect(config.getConfig().storage_pcd_chunk_frames == 0)) return 1;
    std::string default_config_error;
    if (expect(config.validate(default_config_error))) return 1;

    if (expect(!hasImuCoverageForLidarFrame(10.04, 10.05))) return 1;
    if (expect(hasImuCoverageForLidarFrame(10.05, 10.05))) return 1;
    if (expect(hasImuCoverageForLidarFrame(10.06, 10.05))) return 1;

    if (expect(kFastLioLaserPointCovariance == 0.001)) return 1;
    if (expect(fastLioPlaneResidualAccepted(0.0, 4.0))) return 1;
    if (expect(!fastLioPlaneResidualAccepted(1.0, 4.0))) return 1;

    if (testLvxParsesCartesianPackets()) return 1;
    if (testFoxglovePublishesImuJson()) return 1;
    if (testLvxParsesSphericalPackets()) return 1;
    if (testLvxParsesImuAndDiagnostics()) return 1;
    if (testLvxOpenIgnoresEmptyTailFrame()) return 1;
    if (testLivoxAdapterStopForIsSafeWithoutInitializedSdk()) return 1;
    if (testLivoxAdapterContainsThrowingSdkCallbacks()) return 1;
    if (testLivoxAdapterStopForFromCallbackDoesNotSelfWait()) return 1;
    if (testBagReaderRejectsMalformedPointCountWithoutThrowing()) return 1;
    if (testBagReaderContainsThrowingFrameCallback()) return 1;
    if (testBagReaderStopForIsIdempotentBeforePlayback()) return 1;
    if (testMapAccumulatorKeepsDistantFrames()) return 1;
    if (testMapAccumulatorReturnsOnlyNewVoxels()) return 1;
    if (testAsyncMapPublisherBuildsLatestSnapshot()) return 1;
    if (testAsyncMapPublisherResyncsAfterDeltaOverflow()) return 1;
    if (testAsyncMapPublisherBoundsFrameQueue()) return 1;
    if (testTiledMapStoreHandlesCoordinatesAndUpdatePolicies()) return 1;
    if (testTiledMapStoreMatchesFirstVoxelAccumulatorSnapshot()) return 1;
    if (testTiledMapStoreCoalescesDirtyVersionsAndHonorsLimits()) return 1;
    if (testTiledMapStoreBoundsSingleTileAndKeepsExistingVoxelsLive()) return 1;
    if (testTiledMapStoreBoundsAndCoalescesPendingDeletions()) return 1;
    if (testTiledMapStoreOnlyDeletesTilesConfirmedHandedOff()) return 1;
    if (testTiledMapStoreRejectsNonFiniteCoordinates()) return 1;
    if (testTiledMapStoreNeverCreatesTombstoneAboveMemoryLimit()) return 1;
    if (testTiledMapStoreCompactsOuterBucketsAfterGrowthRollback()) return 1;
    if (testMapBuildWorkerNormalizesTileCapacityToOutputLimits()) return 1;
    if (testFoxgloveOutputWorkerBoundsAndMergesTileQueue()) return 1;
    if (testFoxgloveOutputWorkerRejectsTileBatchesAtomically()) return 1;
    if (testFoxgloveOutputWorkerDoesNotStarveFullSnapshot()) return 1;
    if (testFoxgloveOutputWorkerRejectsStaleInflightTileVersion()) return 1;
    if (testFoxgloveOutputWorkerQueuesEqualInflightResyncVersion()) return 1;
    if (testFoxgloveOutputWorkerRetriesFailedDeletionInFixedSlot()) return 1;
    if (testFoxgloveOutputWorkerBoundedStopCancelsQueuedTiles()) return 1;
    if (testMapBuildWorkerContinuesWhileOutputCallbackIsBlocked()) return 1;
    if (testMapBuildWorkerResyncsTileAfterPublishFailure()) return 1;
    if (testMapBuildWorkerBoundedStopPropagatesOutputTimeout()) return 1;
    if (testMapBuildWorkerBoundsInputQueue()) return 1;
    if (testMapBuildWorkerKeepsFullRequestBehindWholeSourceFrame()) return 1;
    if (testMapBuildWorkerCancelsCoalescedTileInput()) return 1;
    if (testStorageWorkerRealtimeRejectsOversizedTask()) return 1;
    if (testBagWriterRoundTripsHeaderAndMessageThroughReader()) return 1;
    if (testBagWriterEncodesRosV2FieldsIndependently()) return 1;
    if (testStorageWorkerRealtimePathReplacementPreservesBagOrder()) return 1;
    if (testStorageWorkerFlushWritesPartialPcdChunk()) return 1;
    if (testStorageWorkerFlushReportsPcdFailure()) return 1;
    if (testStorageWorkerRealtimeFusesBagFailures()) return 1;
    if (testStorageWorkerStopFinalizesAndContainsWriteFailures()) return 1;
    if (testStorageWorkerRealtimePcdFailurePreservesBagAndAccountsLoss()) return 1;
    if (testStorageWorkerReliablePcdFailureStopsWithRemainderReported()) return 1;
    if (testStorageWorkerBagStartupFailureLeavesPcdUsable()) return 1;
    if (testStorageWorkerReliableBagFuseLeavesPcdUsable()) return 1;
    if (testStorageWorkerPcdChunkFrameTrigger()) return 1;
    if (testStorageWorkerBoundedNonDrainStopCancelsQueue()) return 1;
    if (testStorageWorkerHardCapIncludesInflightTask()) return 1;
    if (testStorageWorkerNonDrainCancelsUncommittedPendingPcd()) return 1;
    if (testStorageWorkerReliableAdmissionFailureStopsAndPreservesQueue()) return 1;
    if (testStorageWorkerStartForTimesOutWithoutFalseSuccess()) return 1;
    if (testStorageWorkerBagOnlyReliableFailureStopsConsumer()) return 1;
    if (testStorageWorkerRealtimePcdFuseAccountsQueuedRaceOnce()) return 1;
    if (testStorageWorkerMultiChunkFailureCompactsRemainderOnce()) return 1;
    if (testGroupedConfigLegacyOverrideMigration()) return 1;
    return 0;
}
