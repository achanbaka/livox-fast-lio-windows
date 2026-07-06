#include <cassert>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <fstream>
#include <iostream>
#include <vector>
#include <string>

#include "foxglove_publisher.h"
#include "fast_lio_observation.h"
#include "lidar_imu_sync.h"
#include "livox_sdk.h"
#include "livox_point_utils.h"
#include "lvx_reader.h"
#include "map_accumulator.h"
#include "yaml_config.h"

static int expectAt(bool condition, const char* expression, int line) {
    if (!condition) {
        std::cerr << "Expectation failed at line " << line << ": " << expression << std::endl;
        return 1;
    }
    return 0;
}

#define expect(condition) expectAt((condition), #condition, __LINE__)

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
    reader.close();

    std::remove(path);
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

int main() {
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

    YamlConfig config;
    if (expect(config.getConfig().point_filter_num == 3)) return 1;
    if (expect(config.getConfig().max_iteration == 3)) return 1;
    if (expect(config.getConfig().max_feature_points == 2000)) return 1;
    if (expect(config.getConfig().cube_side_length == 1000)) return 1;
    if (expect(config.getConfig().runtime_pos_log == false)) return 1;
    if (expect(config.getConfig().publish_full_map == true)) return 1;

    if (expect(!hasImuCoverageForLidarFrame(10.04, 10.05))) return 1;
    if (expect(hasImuCoverageForLidarFrame(10.05, 10.05))) return 1;
    if (expect(hasImuCoverageForLidarFrame(10.06, 10.05))) return 1;

    if (expect(kFastLioLaserPointCovariance == 0.001)) return 1;
    if (expect(fastLioPlaneResidualAccepted(0.0, 4.0))) return 1;
    if (expect(!fastLioPlaneResidualAccepted(1.0, 4.0))) return 1;

    if (testLvxParsesCartesianPackets()) return 1;
    if (testLvxParsesSphericalPackets()) return 1;
    if (testLvxParsesImuAndDiagnostics()) return 1;
    if (testLvxOpenIgnoresEmptyTailFrame()) return 1;
    if (testMapAccumulatorKeepsDistantFrames()) return 1;
    return 0;
}
