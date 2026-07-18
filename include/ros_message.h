#ifndef ROS_MESSAGE_H
#define ROS_MESSAGE_H

#include <cstdint>
#include <cstring>
#include <string>
#include <stdexcept>
#include <vector>
#include <array>
#include "types.h"

// ═══════════════════════════════════════════════════════════════════════
//  ROS 1 Binary Serialization Primitives
// ═══════════════════════════════════════════════════════════════════════

class RosSerializer {
public:
    RosSerializer() { buf_.reserve(256); }

    void writeUint8(uint8_t v)   { buf_.push_back(v); }
    void writeUint32(uint32_t v) { writeRaw(&v, 4); }
    void writeUint64(uint64_t v) { writeRaw(&v, 8); }
    void writeFloat32(float v)   { writeRaw(&v, 4); }
    void writeFloat64(double v)  { writeRaw(&v, 8); }
    void writeString(const std::string& s) {
        writeUint32(static_cast<uint32_t>(s.size()));
        writeRaw(s.data(), s.size());
    }
    void writeTime(uint32_t sec, uint32_t nsec) {
        writeUint32(sec);
        writeUint32(nsec);
    }
    void writeRaw(const void* data, size_t len) {
        auto* p = static_cast<const uint8_t*>(data);
        buf_.insert(buf_.end(), p, p + len);
    }

    const std::vector<uint8_t>& buffer() const { return buf_; }
    const uint8_t* data() const { return buf_.data(); }
    size_t size() const { return buf_.size(); }
    void clear() { buf_.clear(); }

private:
    std::vector<uint8_t> buf_;
};

class RosDeserializer {
public:
    RosDeserializer() : data_(nullptr), len_(0), pos_(0), good_(true) {}
    RosDeserializer(const uint8_t* data, size_t len)
        : data_(data), len_(len), pos_(0), good_(true) {}
    RosDeserializer(const std::vector<uint8_t>& buf)
        : data_(buf.data()), len_(buf.size()), pos_(0), good_(true) {}

    uint8_t readUint8()   { return readRawT<uint8_t>(); }
    uint32_t readUint32() { return readRawT<uint32_t>(); }
    uint64_t readUint64() { return readRawT<uint64_t>(); }
    float readFloat32()   { return readRawT<float>(); }
    double readFloat64()  { return readRawT<double>(); }

    std::string readString() {
        uint32_t len = readUint32();
        if (!good_ || len > remaining()) {
            good_ = false;
            return "";
        }
        std::string s(reinterpret_cast<const char*>(data_ + pos_), len);
        pos_ += len;
        return s;
    }

    void readTime(uint32_t& sec, uint32_t& nsec) {
        sec = readUint32();
        nsec = readUint32();
    }

    void readRaw(void* dst, size_t len) {
        if (len > remaining()) {
            good_ = false;
            return;
        }
        memcpy(dst, data_ + pos_, len);
        pos_ += len;
    }

    void skip(size_t bytes) {
        if (bytes > remaining()) {
            pos_ = len_;
            good_ = false;
            return;
        }
        pos_ += bytes;
    }
    size_t remaining() const { return (pos_ < len_) ? (len_ - pos_) : 0; }
    size_t tell() const { return pos_; }
    bool good() const { return good_ && pos_ <= len_; }

private:
    template<typename T>
    T readRawT() {
        if (sizeof(T) > remaining()) {
            good_ = false;
            return T{};
        }
        T v;
        memcpy(&v, data_ + pos_, sizeof(T));
        pos_ += sizeof(T);
        return v;
    }

    const uint8_t* data_;
    size_t len_;
    size_t pos_;
    bool good_;
};

// ═══════════════════════════════════════════════════════════════════════
//  ROS Standard Header
// ═══════════════════════════════════════════════════════════════════════

struct RosHeader {
    uint32_t seq = 0;
    uint32_t stamp_sec = 0;
    uint32_t stamp_nsec = 0;
    std::string frame_id;
};

inline void serializeHeader(RosSerializer& s, const RosHeader& h) {
    s.writeUint32(h.seq);
    s.writeTime(h.stamp_sec, h.stamp_nsec);
    s.writeString(h.frame_id);
}

inline RosHeader deserializeHeader(RosDeserializer& d) {
    RosHeader h;
    h.seq = d.readUint32();
    d.readTime(h.stamp_sec, h.stamp_nsec);
    h.frame_id = d.readString();
    return h;
}

// ═══════════════════════════════════════════════════════════════════════
//  Input Messages (from bag) — Livox CustomMsg
// ═══════════════════════════════════════════════════════════════════════

struct LivoxCustomPoint {
    uint32_t offset_time;   // ns from packet start
    float x, y, z;          // meters
    uint8_t reflectivity;
    uint8_t tag;
    uint8_t line;
};

struct LivoxCustomMsg {
    RosHeader header;
    uint64_t timebase;       // ns
    uint32_t point_num;
    uint8_t lidar_id;
    uint8_t rsvd[3];
    std::vector<LivoxCustomPoint> points;

    // Deserialize from ROS binary format
    static LivoxCustomMsg deserialize(RosDeserializer& d) {
        LivoxCustomMsg msg;
        msg.header = deserializeHeader(d);
        msg.timebase = d.readUint64();
        msg.point_num = d.readUint32();
        msg.lidar_id = d.readUint8();
        msg.rsvd[0] = d.readUint8();
        msg.rsvd[1] = d.readUint8();
        msg.rsvd[2] = d.readUint8();
        // Array: uint32 length prefix + data
        const uint32_t arr_len = d.readUint32();
        constexpr size_t kSerializedPointBytes =
            sizeof(uint32_t) + 3 * sizeof(float) + 3 * sizeof(uint8_t);
        if (!d.good() || arr_len != msg.point_num ||
            static_cast<size_t>(arr_len) > d.remaining() / kSerializedPointBytes) {
            throw std::runtime_error("invalid Livox CustomMsg point array");
        }
        msg.points.resize(arr_len);
        for (uint32_t i = 0; i < arr_len; i++) {
            msg.points[i].offset_time = d.readUint32();
            msg.points[i].x = d.readFloat32();
            msg.points[i].y = d.readFloat32();
            msg.points[i].z = d.readFloat32();
            msg.points[i].reflectivity = d.readUint8();
            msg.points[i].tag = d.readUint8();
            msg.points[i].line = d.readUint8();
        }
        if (!d.good()) {
            throw std::runtime_error("truncated Livox CustomMsg point array");
        }
        return msg;
    }

    double getTimestamp() const {
        return static_cast<double>(header.stamp_sec) +
               static_cast<double>(header.stamp_nsec) / 1e9;
    }
};

// ═══════════════════════════════════════════════════════════════════════
//  Input Messages (from bag) — sensor_msgs/Imu
// ═══════════════════════════════════════════════════════════════════════

struct RosImu {
    RosHeader header;
    double orientation[4] = {0,0,0,1}; // x,y,z,w
    double orientation_cov[9] = {};
    double angular_velocity[3] = {};
    double angular_velocity_cov[9] = {};
    double linear_acceleration[3] = {};
    double linear_acceleration_cov[9] = {};

    static RosImu deserialize(RosDeserializer& d) {
        RosImu msg;
        msg.header = deserializeHeader(d);
        for (int i = 0; i < 4; i++) msg.orientation[i] = d.readFloat64();
        for (int i = 0; i < 9; i++) msg.orientation_cov[i] = d.readFloat64();
        for (int i = 0; i < 3; i++) msg.angular_velocity[i] = d.readFloat64();
        for (int i = 0; i < 9; i++) msg.angular_velocity_cov[i] = d.readFloat64();
        for (int i = 0; i < 3; i++) msg.linear_acceleration[i] = d.readFloat64();
        for (int i = 0; i < 9; i++) msg.linear_acceleration_cov[i] = d.readFloat64();
        return msg;
    }

    double getTimestamp() const {
        return static_cast<double>(header.stamp_sec) +
               static_cast<double>(header.stamp_nsec) / 1e9;
    }

    ImuData toImuData() const {
        ImuData imu;
        imu.timestamp = getTimestamp();
        imu.gyro = V3D(angular_velocity[0], angular_velocity[1], angular_velocity[2]);
        imu.acc  = V3D(linear_acceleration[0], linear_acceleration[1], linear_acceleration[2]);
        return imu;
    }
};

// ═══════════════════════════════════════════════════════════════════════
//  Output Messages (to bag) — geometry_msgs/PoseStamped
// ═══════════════════════════════════════════════════════════════════════

inline std::vector<uint8_t> serializePoseStamped(
    const RosHeader& header,
    double px, double py, double pz,
    double ox, double oy, double oz, double ow)
{
    RosSerializer s;
    serializeHeader(s, header);
    // position
    s.writeFloat64(px); s.writeFloat64(py); s.writeFloat64(pz);
    // orientation
    s.writeFloat64(ox); s.writeFloat64(oy); s.writeFloat64(oz); s.writeFloat64(ow);
    return std::vector<uint8_t>(s.buffer());
}

// ═══════════════════════════════════════════════════════════════════════
//  Output Messages (to bag) — sensor_msgs/PointCloud2
// ═══════════════════════════════════════════════════════════════════════

inline std::vector<uint8_t> serializePointCloud2(
    const RosHeader& header,
    const PointCloudXYZI& cloud)
{
    RosSerializer s;
    serializeHeader(s, header);

    uint32_t num_points = static_cast<uint32_t>(cloud.size());
    s.writeUint32(1);            // height
    s.writeUint32(num_points);   // width

    // PointField definitions
    uint32_t num_fields = 4;
    s.writeUint32(num_fields);

    // x field
    s.writeString("x");
    s.writeUint32(0);    // offset
    s.writeUint8(7);     // FLOAT32
    s.writeUint32(1);    // count

    // y field
    s.writeString("y");
    s.writeUint32(4);
    s.writeUint8(7);
    s.writeUint32(1);

    // z field
    s.writeString("z");
    s.writeUint32(8);
    s.writeUint8(7);
    s.writeUint32(1);

    // intensity field
    s.writeString("intensity");
    s.writeUint32(12);
    s.writeUint8(7);
    s.writeUint32(1);

    s.writeUint8(0);                    // is_bigendian
    s.writeUint32(16);                  // point_step (4 floats × 4 bytes)
    s.writeUint32(16 * num_points);     // row_step
    // data array
    s.writeUint32(16 * num_points);     // array byte length
    for (uint32_t i = 0; i < num_points; i++) {
        s.writeFloat32(cloud.points[i].x);
        s.writeFloat32(cloud.points[i].y);
        s.writeFloat32(cloud.points[i].z);
        s.writeFloat32(cloud.points[i].intensity);
    }
    s.writeUint8(1); // is_dense

    return std::vector<uint8_t>(s.buffer());
}

// ═══════════════════════════════════════════════════════════════════════
//  Output Messages (to bag) — geometry_msgs/PoseArray
// ═══════════════════════════════════════════════════════════════════════

inline std::vector<uint8_t> serializePoseArray(
    const RosHeader& header,
    const std::vector<V3D>& path)
{
    RosSerializer s;
    serializeHeader(s, header);

    // Array length prefix (number of poses)
    s.writeUint32(static_cast<uint32_t>(path.size()));
    for (const auto& p : path) {
        // position
        s.writeFloat64(p.x()); s.writeFloat64(p.y()); s.writeFloat64(p.z());
        // orientation (identity)
        s.writeFloat64(0); s.writeFloat64(0); s.writeFloat64(0); s.writeFloat64(1);
    }

    return std::vector<uint8_t>(s.buffer());
}

// ═══════════════════════════════════════════════════════════════════════
//  Output Messages (to bag) — tf2_msgs/TFMessage
// ═══════════════════════════════════════════════════════════════════════

inline std::vector<uint8_t> serializeTFMessage(
    const RosHeader& header,
    const std::string& child_frame_id,
    double tx, double ty, double tz,
    double rx, double ry, double rz, double rw)
{
    RosSerializer s;
    // Array of 1 TransformStamped
    s.writeUint32(1);

    // TransformStamped.header
    serializeHeader(s, header);
    // child_frame_id
    s.writeString(child_frame_id);
    // transform.translation
    s.writeFloat64(tx); s.writeFloat64(ty); s.writeFloat64(tz);
    // transform.rotation
    s.writeFloat64(rx); s.writeFloat64(ry); s.writeFloat64(rz); s.writeFloat64(rw);

    return std::vector<uint8_t>(s.buffer());
}

// ═══════════════════════════════════════════════════════════════════════
//  Message definition strings (for bag CONNECTION records)
// ═══════════════════════════════════════════════════════════════════════

namespace ros_msg_defs {

const std::string POSE_STAMPED =
    "# A Pose with reference coordinate frame and timestamp\n"
    "Header header\n"
    "Pose pose\n"
    "================================================================================\n"
    "MSG: std_msgs/Header\n"
    "uint32 seq\n"
    "time stamp\n"
    "string frame_id\n"
    "================================================================================\n"
    "MSG: geometry_msgs/Pose\n"
    "Point position\n"
    "Quaternion orientation\n"
    "================================================================================\n"
    "MSG: geometry_msgs/Point\n"
    "float64 x\n"
    "float64 y\n"
    "float64 z\n"
    "================================================================================\n"
    "MSG: geometry_msgs/Quaternion\n"
    "float64 x\n"
    "float64 y\n"
    "float64 z\n"
    "float64 w\n";

const std::string POINT_CLOUD2 =
    "Header header\n"
    "uint32 height\n"
    "uint32 width\n"
    "PointField[] fields\n"
    "bool is_bigendian\n"
    "uint32 point_step\n"
    "uint32 row_step\n"
    "uint8[] data\n"
    "bool is_dense\n"
    "================================================================================\n"
    "MSG: std_msgs/Header\n"
    "uint32 seq\n"
    "time stamp\n"
    "string frame_id\n"
    "================================================================================\n"
    "MSG: sensor_msgs/PointField\n"
    "uint8 INT8=1\n"
    "uint8 UINT8=2\n"
    "uint8 INT16=3\n"
    "uint8 UINT16=4\n"
    "uint8 INT32=5\n"
    "uint8 UINT32=6\n"
    "uint8 FLOAT32=7\n"
    "uint8 FLOAT64=8\n"
    "string name\n"
    "uint32 offset\n"
    "uint8 datatype\n"
    "uint32 count\n";

const std::string POSE_ARRAY =
    "# An array of poses with a header for global reference.\n"
    "Header header\n"
    "Pose[] poses\n"
    "================================================================================\n"
    "MSG: geometry_msgs/Pose\n"
    "Point position\n"
    "Quaternion orientation\n"
    "================================================================================\n"
    "MSG: geometry_msgs/Point\n"
    "float64 x\n"
    "float64 y\n"
    "float64 z\n"
    "================================================================================\n"
    "MSG: geometry_msgs/Quaternion\n"
    "float64 x\n"
    "float64 y\n"
    "float64 z\n"
    "float64 w\n"
    "================================================================================\n"
    "MSG: std_msgs/Header\n"
    "uint32 seq\n"
    "time stamp\n"
    "string frame_id\n";

const std::string TF_MESSAGE =
    "geometry_msgs/TransformStamped[] transforms\n"
    "================================================================================\n"
    "MSG: geometry_msgs/TransformStamped\n"
    "Header header\n"
    "string child_frame_id\n"
    "Transform transform\n"
    "================================================================================\n"
    "MSG: geometry_msgs/Transform\n"
    "Vector3 translation\n"
    "Quaternion rotation\n"
    "================================================================================\n"
    "MSG: geometry_msgs/Vector3\n"
    "float64 x\n"
    "float64 y\n"
    "float64 z\n"
    "================================================================================\n"
    "MSG: geometry_msgs/Quaternion\n"
    "float64 x\n"
    "float64 y\n"
    "float64 z\n"
    "float64 w\n"
    "================================================================================\n"
    "MSG: std_msgs/Header\n"
    "uint32 seq\n"
    "time stamp\n"
    "string frame_id\n";

const std::string CUSTOM_MSG =
    "CustomMsg\n"
    "Header header\n"
    "uint64 timebase\n"
    "uint32 point_num\n"
    "uint8 lidar_id\n"
    "uint8[3] rsvd\n"
    "CustomPoint[] points\n"
    "================================================================================\n"
    "MSG: std_msgs/Header\n"
    "uint32 seq\n"
    "time stamp\n"
    "string frame_id\n"
    "================================================================================\n"
    "MSG: livox_ros_driver/CustomPoint\n"
    "uint32 offset_time\n"
    "float32 x\n"
    "float32 y\n"
    "float32 z\n"
    "uint8 reflectivity\n"
    "uint8 tag\n"
    "uint8 line\n";

const std::string SENSOR_IMU =
    "Imu\n"
    "Header header\n"
    "Quaternion orientation\n"
    "float64[9] orientation_covariance\n"
    "Vector3 angular_velocity\n"
    "float64[9] angular_velocity_covariance\n"
    "Vector3 linear_acceleration\n"
    "float64[9] linear_acceleration_covariance\n"
    "================================================================================\n"
    "MSG: std_msgs/Header\n"
    "uint32 seq\n"
    "time stamp\n"
    "string frame_id\n"
    "================================================================================\n"
    "MSG: geometry_msgs/Quaternion\n"
    "float64 x\n"
    "float64 y\n"
    "float64 z\n"
    "float64 w\n"
    "================================================================================\n"
    "MSG: geometry_msgs/Vector3\n"
    "float64 x\n"
    "float64 y\n"
    "float64 z\n";

// MD5 sums (standard ROS values)
const std::string POSE_STAMPED_MD5 = "d3812c3eb69e39177346ef37679b2065";
const std::string POINT_CLOUD2_MD5 = "1155d3dc355119d6dd3256612b16c678";
const std::string POSE_ARRAY_MD5   = "916c28c5764443f268b296bb671b9d97";
const std::string TF_MESSAGE_MD5   = "94810edda583a504dfda3824e5c96c1e";
const std::string CUSTOM_MSG_MD5   = "c852fac30a094420cc56e73c57f4292e";
const std::string SENSOR_IMU_MD5   = "a9c9cb14c2e80c6e3d624478e3a15b5c";

} // namespace ros_msg_defs

// ═══════════════════════════════════════════════════════════════════════
//  Time helpers
// ═══════════════════════════════════════════════════════════════════════

inline void doubleToRosTime(double t, uint32_t& sec, uint32_t& nsec) {
    sec = static_cast<uint32_t>(t);
    nsec = static_cast<uint32_t>((t - sec) * 1e9);
}

inline uint64_t doubleToNs(double t) {
    return static_cast<uint64_t>(t * 1e9);
}

inline double nsToDouble(uint64_t ns) {
    return static_cast<double>(ns) / 1e9;
}

#endif // ROS_MESSAGE_H
