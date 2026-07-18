#include "bag_writer.h"
#include <iostream>
#include <cstring>
#include <algorithm>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <utility>

// ═══════════════════════════════════════════════════════════════════════
//  Construction / Destruction
// ═══════════════════════════════════════════════════════════════════════

BagWriter::BagWriter(IoHook io_hook) : io_hook_(std::move(io_hook)) {}

BagWriter::~BagWriter() {
    try {
        close();
    } catch (...) {
        // Destructors must not terminate the process. Explicit close() callers
        // still receive the original I/O failure.
    }
}

// ═══════════════════════════════════════════════════════════════════════
//  Low-level write helpers
// ═══════════════════════════════════════════════════════════════════════

void BagWriter::writeRaw(const void* data, size_t len) {
    ensureStreamGood("write");
    if (len > static_cast<size_t>(std::numeric_limits<std::streamsize>::max())) {
        throw std::overflow_error("[BagWriter] write length exceeds stream limit: " +
                                  filepath_);
    }
    if (len == 0) return;
    file_.write(static_cast<const char*>(data), static_cast<std::streamsize>(len));
    ensureStreamGood("write");
}

void BagWriter::writeU32(uint32_t v) {
    writeRaw(&v, 4);
}

uint64_t BagWriter::tellPosition(const char* operation) {
    ensureStreamGood(operation);
    const std::streampos position = file_.tellp();
    if (position == std::streampos(-1)) {
        throw std::runtime_error(std::string("[BagWriter] ") + operation +
                                 " failed: " + filepath_);
    }
    return static_cast<uint64_t>(position);
}

void BagWriter::seekPosition(std::streamoff offset, const char* operation) {
    ensureStreamGood(operation);
    file_.seekp(offset);
    ensureStreamGood(operation);
}

void BagWriter::ensureStreamGood(const char* operation) const {
    if (!file_.is_open() || !file_) {
        throw std::runtime_error(std::string("[BagWriter] ") + operation +
                                 " failed: " + filepath_);
    }
}

void BagWriter::closeFileNoThrow() noexcept {
    if (!file_.is_open()) return;
    file_.clear();
    file_.close();
}

void BagWriter::invokeIoHook(const char* operation) {
    if (io_hook_) io_hook_(operation);
}

// Encode a field as [field_len:u32][key=value]
static std::vector<uint8_t> encodeField(const std::string& key, const std::string& value) {
    std::string field = key + "=" + value;
    std::vector<uint8_t> result;
    uint32_t len = static_cast<uint32_t>(field.size());
    result.resize(4 + field.size());
    memcpy(result.data(), &len, 4);
    memcpy(result.data() + 4, field.data(), field.size());
    return result;
}

template <typename T>
static std::string rawFieldValue(T value) {
    std::string result(sizeof(T), '\0');
    memcpy(result.data(), &value, sizeof(T));
    return result;
}

static std::string rosTimeFieldValue(uint64_t time_ns) {
    const uint64_t sec64 = time_ns / 1000000000ULL;
    if (sec64 > std::numeric_limits<uint32_t>::max()) {
        throw std::overflow_error("[BagWriter] ROS time seconds overflow uint32");
    }
    const uint32_t sec = static_cast<uint32_t>(sec64);
    const uint32_t nsec = static_cast<uint32_t>(time_ns % 1000000000ULL);
    std::string result(2 * sizeof(uint32_t), '\0');
    memcpy(result.data(), &sec, sizeof(sec));
    memcpy(result.data() + sizeof(sec), &nsec, sizeof(nsec));
    return result;
}

static void appendRosTime(std::vector<uint8_t>& output, uint64_t time_ns) {
    const std::string encoded = rosTimeFieldValue(time_ns);
    output.insert(output.end(), encoded.begin(), encoded.end());
}

static std::vector<uint8_t> encodeFileHeader(
    uint64_t index_pos, uint32_t connection_count, uint32_t chunk_count) {
    const uint8_t op = 0x03; // FILE_HEADER
    const std::vector<std::pair<std::string, std::string>> fields = {
        {"op", rawFieldValue(op)},
        {"index_pos", rawFieldValue(index_pos)},
        {"conn_count", rawFieldValue(connection_count)},
        {"chunk_count", rawFieldValue(chunk_count)},
    };
    std::vector<uint8_t> header;
    for (const auto& field : fields) {
        const auto encoded = encodeField(field.first, field.second);
        header.insert(header.end(), encoded.begin(), encoded.end());
    }
    return header;
}

static uint32_t fileHeaderPaddingSize(size_t header_size) {
    // ROS rosbag_storage defines FILE_HEADER_LENGTH as header bytes plus data
    // padding; the two uint32 length prefixes are outside that reservation.
    constexpr size_t kFileHeaderContentSize = 4096;
    if (header_size > kFileHeaderContentSize) {
        throw std::overflow_error("[BagWriter] FILE_HEADER exceeds reserved size");
    }
    return static_cast<uint32_t>(
        kFileHeaderContentSize - header_size);
}

// ═══════════════════════════════════════════════════════════════════════
//  Open / Close
// ═══════════════════════════════════════════════════════════════════════

bool BagWriter::open(const std::string& filepath) {
    filepath_ = filepath;
    file_.clear();
    file_.open(filepath, std::ios::binary | std::ios::trunc);
    if (!file_.is_open()) {
        std::cerr << "[BagWriter] Cannot open: " << filepath << std::endl;
        return false;
    }

    try {
        // Write version line
        const char* version = "#ROSBAG V2.0\n";
        writeRaw(version, strlen(version));

        // Write placeholder BAG_HEADER (will be rewritten on close)
        // The BAG_HEADER data is padded to 4096 bytes total
        writeFileHeader();
    } catch (...) {
        closeFileNoThrow();
        throw;
    }

    std::cout << "[BagWriter] Opened: " << filepath << std::endl;
    return true;
}

void BagWriter::writeFileHeader() {
    // We'll write a placeholder; the real values are written in close()
    // For now, just reserve space (4096 bytes total for the header record)
    uint64_t start_pos = tellPosition("tell initial header position");

    const std::vector<uint8_t> header = encodeFileHeader(0, 0, 0);
    const uint32_t padding_size = fileHeaderPaddingSize(header.size());

    // Header bytes plus its data padding are exactly 4096 bytes, matching ROS
    // bag v2. The two uint32 length prefixes make the complete record 4104
    // bytes, and data_len lets a normal reader advance without a fixed seek.
    writeU32(static_cast<uint32_t>(header.size()));
    writeRaw(header.data(), header.size());
    writeU32(padding_size);
    std::vector<char> padding(padding_size, ' ');
    writeRaw(padding.data(), padding.size());
    const uint64_t end_pos = tellPosition("tell padded header position");
    if (end_pos != start_pos + 4096 + 2 * sizeof(uint32_t)) {
        throw std::runtime_error("[BagWriter] FILE_HEADER padding size mismatch");
    }
}

void BagWriter::close() {
    if (!file_.is_open()) return;

    std::lock_guard<std::mutex> lock(mutex_);
    try {
        invokeIoHook("close");
        // Flush remaining chunk
        if (!chunk_buffer_.empty()) {
            flushChunk();
        }

        // Record the index position (where CONNECTION records start)
        uint64_t index_pos = tellPosition("tell index position");

        // Write CONNECTION records
        writeConnectionRecords();

        // Write CHUNK_INFO records
        writeChunkInfoRecords();

        // Rewrite BAG_HEADER with correct values
        seekPosition(13, "seek file header"); // After "#ROSBAG V2.0\n"

        if (connections_.size() > std::numeric_limits<uint32_t>::max() ||
            chunk_records_.size() > std::numeric_limits<uint32_t>::max()) {
            throw std::overflow_error("[BagWriter] FILE_HEADER count overflow");
        }
        const std::vector<uint8_t> header = encodeFileHeader(
            index_pos, static_cast<uint32_t>(connections_.size()),
            static_cast<uint32_t>(chunk_records_.size()));

        writeU32(static_cast<uint32_t>(header.size()));
        writeRaw(header.data(), header.size());
        // Preserve the original padding data; only its length field needs to
        // be rewritten alongside the fixed-size header.
        writeU32(fileHeaderPaddingSize(header.size()));

        file_.flush();
        ensureStreamGood("flush");
        file_.close();
        if (file_.fail()) {
            throw std::runtime_error("[BagWriter] close failed: " + filepath_);
        }
    } catch (...) {
        closeFileNoThrow();
        throw;
    }
    std::cout << "[BagWriter] Closed: " << filepath_
              << " (chunks=" << chunk_records_.size()
              << ", connections=" << connections_.size() << ")" << std::endl;
}

// ═══════════════════════════════════════════════════════════════════════
//  Connection management
// ═══════════════════════════════════════════════════════════════════════

uint32_t BagWriter::addConnection(const std::string& topic,
                                   const std::string& type,
                                   const std::string& md5sum,
                                   const std::string& msg_def) {
    std::lock_guard<std::mutex> lock(mutex_);
    ensureStreamGood("add connection");
    uint32_t id = next_conn_id_++;
    ConnectionInfo conn;
    conn.conn_id = id;
    conn.topic = topic;
    conn.type = type;
    conn.md5sum = md5sum;
    conn.msg_def = msg_def;
    connections_[id] = conn;
    seq_counters_[id] = 0;
    return id;
}

// ═══════════════════════════════════════════════════════════════════════
//  Write messages
// ═══════════════════════════════════════════════════════════════════════

void BagWriter::appendMsgToChunk(uint32_t conn_id, uint64_t time_ns,
                                  const uint8_t* data, size_t len) {
    if (len > std::numeric_limits<uint32_t>::max()) {
        throw std::overflow_error("[BagWriter] message exceeds ROS bag record limit");
    }
    // MSG_DATA sub-record within chunk:
    // [header_len:u32][op(1B) + conn(4B) + time(8B)][data_len:u32][message_data]

    std::vector<uint8_t> sub_header;

    // op = 0x02 as raw byte
    {
        std::string key_eq = "op=";
        uint32_t fl = static_cast<uint32_t>(key_eq.size() + 1);
        auto old_size = sub_header.size();
        sub_header.resize(old_size + 4 + fl);
        memcpy(sub_header.data() + old_size, &fl, 4);
        memcpy(sub_header.data() + old_size + 4, key_eq.data(), key_eq.size());
        sub_header[old_size + 4 + key_eq.size()] = 0x02;
    }
    // conn = conn_id as 4-byte raw
    {
        std::string key_eq = "conn=";
        uint32_t fl = static_cast<uint32_t>(key_eq.size() + 4);
        auto old_size = sub_header.size();
        sub_header.resize(old_size + 4 + fl);
        memcpy(sub_header.data() + old_size, &fl, 4);
        memcpy(sub_header.data() + old_size + 4, key_eq.data(), key_eq.size());
        memcpy(sub_header.data() + old_size + 4 + key_eq.size(), &conn_id, 4);
    }
    // ROS time is two little-endian uint32 values: seconds, nanoseconds.
    {
        std::string key_eq = "time=";
        uint32_t fl = static_cast<uint32_t>(key_eq.size() + 8);
        auto old_size = sub_header.size();
        sub_header.resize(old_size + 4 + fl);
        memcpy(sub_header.data() + old_size, &fl, 4);
        memcpy(sub_header.data() + old_size + 4, key_eq.data(), key_eq.size());
        const std::string encoded_time = rosTimeFieldValue(time_ns);
        memcpy(sub_header.data() + old_size + 4 + key_eq.size(),
               encoded_time.data(), encoded_time.size());
    }

    uint32_t offset = static_cast<uint32_t>(chunk_buffer_.size());

    // Write sub-header length + sub-header
    uint32_t sub_header_len = static_cast<uint32_t>(sub_header.size());
    chunk_buffer_.insert(chunk_buffer_.end(),
                         reinterpret_cast<uint8_t*>(&sub_header_len),
                         reinterpret_cast<uint8_t*>(&sub_header_len) + 4);
    chunk_buffer_.insert(chunk_buffer_.end(), sub_header.begin(), sub_header.end());

    // Write data length + data
    uint32_t data_len_u32 = static_cast<uint32_t>(len);
    chunk_buffer_.insert(chunk_buffer_.end(),
                         reinterpret_cast<uint8_t*>(&data_len_u32),
                         reinterpret_cast<uint8_t*>(&data_len_u32) + 4);
    chunk_buffer_.insert(chunk_buffer_.end(), data, data + len);

    // Track message entry
    ChunkMsgEntry entry;
    entry.conn_id = conn_id;
    entry.time_ns = time_ns;
    entry.offset_in_chunk = offset;
    chunk_messages_.push_back(entry);

    // Update chunk time range
    if (chunk_messages_.size() == 1) {
        chunk_start_time_ = time_ns;
        chunk_end_time_ = time_ns;
    } else {
        chunk_start_time_ = std::min(chunk_start_time_, time_ns);
        chunk_end_time_ = std::max(chunk_end_time_, time_ns);
    }

}

void BagWriter::writeMessage(uint32_t conn_id, uint64_t time_ns,
                              const uint8_t* data, size_t len) {
    std::lock_guard<std::mutex> lock(mutex_);
    ensureStreamGood("write message");
    invokeIoHook("write_message");

    appendMsgToChunk(conn_id, time_ns, data, len);

    // Flush chunk if it's large enough
    if (chunk_buffer_.size() >= CHUNK_MAX_SIZE) {
        flushChunk();
    }
}

// ═══════════════════════════════════════════════════════════════════════
//  Chunk management
// ═══════════════════════════════════════════════════════════════════════

void BagWriter::flushChunk() {
    if (chunk_buffer_.empty()) return;

    uint64_t chunk_file_pos = tellPosition("tell chunk position");
    if (chunk_buffer_.size() > std::numeric_limits<uint32_t>::max()) {
        throw std::overflow_error("[BagWriter] chunk exceeds ROS bag uint32 limit");
    }
    const uint32_t chunk_size = static_cast<uint32_t>(chunk_buffer_.size());

    // Write CHUNK record header
    uint8_t op = 0x05; // CHUNK
    std::vector<std::pair<std::string, std::string>> fields;
    fields.push_back({"op", std::string(1, static_cast<char>(op))});
    fields.push_back({"compression", "none"});
    fields.push_back({"size", rawFieldValue(chunk_size)});

    std::vector<uint8_t> header;
    for (const auto& f : fields) {
        auto encoded = encodeField(f.first, f.second);
        header.insert(header.end(), encoded.begin(), encoded.end());
    }

    writeU32(static_cast<uint32_t>(header.size()));
    writeRaw(header.data(), header.size());
    writeU32(chunk_size);
    writeRaw(chunk_buffer_.data(), chunk_buffer_.size());

    // Write INDEX_DATA records for each connection in this chunk
    std::map<uint32_t, std::vector<std::pair<uint64_t, uint32_t>>> chunk_conn_index;
    ChunkRecord record;
    record.file_pos = chunk_file_pos;
    record.start_time_ns = chunk_start_time_;
    record.end_time_ns = chunk_end_time_;

    for (const auto& msg : chunk_messages_) {
        chunk_conn_index[msg.conn_id].push_back({msg.time_ns, msg.offset_in_chunk});
        record.conn_counts[msg.conn_id]++;
    }

    for (const auto& pair : chunk_conn_index) {
        uint32_t conn_id = pair.first;
        auto entries = pair.second;
        std::sort(entries.begin(), entries.end(),
                  [](const auto& lhs, const auto& rhs) {
                      return lhs.first < rhs.first ||
                             (lhs.first == rhs.first && lhs.second < rhs.second);
                  });

        // INDEX_DATA header
        std::vector<std::pair<std::string, std::string>> idx_fields;
        uint8_t idx_op = 0x04; // INDEX_DATA
        idx_fields.push_back({"op", std::string(1, static_cast<char>(idx_op))});
        idx_fields.push_back({"ver", rawFieldValue(uint32_t{1})});

        // conn as 4-byte raw
        std::string conn_val(4, '\0');
        memcpy(&conn_val[0], &conn_id, 4);
        idx_fields.push_back({"conn", conn_val});

        // count as 4-byte raw
        uint32_t count = static_cast<uint32_t>(entries.size());
        std::string count_val(4, '\0');
        memcpy(&count_val[0], &count, 4);
        idx_fields.push_back({"count", count_val});

        std::vector<uint8_t> idx_header;
        for (const auto& f : idx_fields) {
            auto encoded = encodeField(f.first, f.second);
            idx_header.insert(idx_header.end(), encoded.begin(), encoded.end());
        }

        // INDEX_DATA data: count entries of (time: sec+nsec, offset:u32).
        std::vector<uint8_t> idx_data;
        for (const auto& e : entries) {
            uint32_t o = e.second;
            appendRosTime(idx_data, e.first);
            idx_data.insert(idx_data.end(),
                           reinterpret_cast<uint8_t*>(&o),
                           reinterpret_cast<uint8_t*>(&o) + 4);
        }

        writeU32(static_cast<uint32_t>(idx_header.size()));
        writeRaw(idx_header.data(), idx_header.size());
        writeU32(static_cast<uint32_t>(idx_data.size()));
        writeRaw(idx_data.data(), idx_data.size());
    }

    chunk_records_.push_back(record);

    // Clear chunk buffer
    chunk_buffer_.clear();
    chunk_messages_.clear();
}

// ═══════════════════════════════════════════════════════════════════════
//  Write CONNECTION records (in index section)
// ═══════════════════════════════════════════════════════════════════════

void BagWriter::writeConnectionRecords() {
    for (const auto& pair : connections_) {
        const auto& conn = pair.second;

        // CONNECTION header fields
        uint8_t op = 0x07; // CONNECTION
        std::vector<std::pair<std::string, std::string>> fields;
        fields.push_back({"op", std::string(1, static_cast<char>(op))});

        // conn as 4-byte raw
        std::string conn_val(4, '\0');
        memcpy(&conn_val[0], &conn.conn_id, 4);
        fields.push_back({"conn", conn_val});

        fields.push_back({"topic", conn.topic});

        // Build header bytes
        std::vector<uint8_t> header;
        for (const auto& f : fields) {
            auto encoded = encodeField(f.first, f.second);
            header.insert(header.end(), encoded.begin(), encoded.end());
        }

        // Connection data: nested header with type, md5sum, message_definition
        std::vector<uint8_t> conn_data;
        auto addField = [&](const std::string& key, const std::string& value) {
            std::string field = key + "=" + value;
            uint32_t fl = static_cast<uint32_t>(field.size());
            conn_data.insert(conn_data.end(),
                            reinterpret_cast<uint8_t*>(&fl),
                            reinterpret_cast<uint8_t*>(&fl) + 4);
            conn_data.insert(conn_data.end(), field.begin(), field.end());
        };

        addField("topic", conn.topic);
        addField("type", conn.type);
        addField("md5sum", conn.md5sum);
        addField("message_definition", conn.msg_def);

        // Write: header_len + header + data_len + data
        writeU32(static_cast<uint32_t>(header.size()));
        writeRaw(header.data(), header.size());
        writeU32(static_cast<uint32_t>(conn_data.size()));
        writeRaw(conn_data.data(), conn_data.size());
    }
}

// ═══════════════════════════════════════════════════════════════════════
//  Write CHUNK_INFO records
// ═══════════════════════════════════════════════════════════════════════

void BagWriter::writeChunkInfoRecords() {
    for (const auto& chunk : chunk_records_) {
        std::vector<std::pair<std::string, std::string>> fields;
        uint8_t op = 0x06; // CHUNK_INFO
        fields.push_back({"op", std::string(1, static_cast<char>(op))});
        fields.push_back({"ver", rawFieldValue(uint32_t{1})});

        // chunk_pos as 8-byte raw
        std::string pos_val(8, '\0');
        memcpy(&pos_val[0], &chunk.file_pos, 8);
        fields.push_back({"chunk_pos", pos_val});

        // start_time as 8-byte raw (sec + nsec)
        fields.push_back({"start_time", rosTimeFieldValue(chunk.start_time_ns)});

        // end_time as 8-byte raw
        fields.push_back({"end_time", rosTimeFieldValue(chunk.end_time_ns)});

        // count as 4-byte raw
        uint32_t count = static_cast<uint32_t>(chunk.conn_counts.size());
        std::string count_val(4, '\0');
        memcpy(&count_val[0], &count, 4);
        fields.push_back({"count", count_val});

        std::vector<uint8_t> header;
        for (const auto& f : fields) {
            auto encoded = encodeField(f.first, f.second);
            header.insert(header.end(), encoded.begin(), encoded.end());
        }

        // Data: count × (conn:u32, count:u32)
        std::vector<uint8_t> data;
        for (const auto& cc : chunk.conn_counts) {
            uint32_t conn_id = cc.first;
            uint32_t msg_count = cc.second;
            data.insert(data.end(),
                       reinterpret_cast<uint8_t*>(&conn_id),
                       reinterpret_cast<uint8_t*>(&conn_id) + 4);
            data.insert(data.end(),
                       reinterpret_cast<uint8_t*>(&msg_count),
                       reinterpret_cast<uint8_t*>(&msg_count) + 4);
        }

        writeU32(static_cast<uint32_t>(header.size()));
        writeRaw(header.data(), header.size());
        writeU32(static_cast<uint32_t>(data.size()));
        writeRaw(data.data(), data.size());
    }
}

// ═══════════════════════════════════════════════════════════════════════
//  Convenience write methods
// ═══════════════════════════════════════════════════════════════════════

void BagWriter::writeOdometry(uint64_t time_ns,
                               const V3D& position,
                               const Eigen::Quaterniond& orientation,
                               const std::string& frame_id) {
    uint32_t sec = static_cast<uint32_t>(time_ns / 1000000000ULL);
    uint32_t nsec = static_cast<uint32_t>(time_ns % 1000000000ULL);

    RosHeader header;
    header.seq = seq_counters_[0]++; // approximate
    header.stamp_sec = sec;
    header.stamp_nsec = nsec;
    header.frame_id = frame_id;

    auto data = serializePoseStamped(header,
        position.x(), position.y(), position.z(),
        orientation.x(), orientation.y(), orientation.z(), orientation.w());

    // Find conn_id for odometry topic
    for (const auto& pair : connections_) {
        if (pair.second.topic == "/odometry") {
            writeMessage(pair.first, time_ns, data.data(), data.size());
            return;
        }
    }
    throw std::runtime_error("[BagWriter] odometry connection is not registered");
}

void BagWriter::writePointCloud(uint64_t time_ns,
                                 const PointCloudXYZI& cloud,
                                 const std::string& frame_id,
                                 const std::string& topic) {
    uint32_t sec = static_cast<uint32_t>(time_ns / 1000000000ULL);
    uint32_t nsec = static_cast<uint32_t>(time_ns % 1000000000ULL);

    RosHeader header;
    header.seq = 0;
    header.stamp_sec = sec;
    header.stamp_nsec = nsec;
    header.frame_id = frame_id;

    auto data = serializePointCloud2(header, cloud);

    for (const auto& pair : connections_) {
        if (pair.second.topic == topic) {
            writeMessage(pair.first, time_ns, data.data(), data.size());
            return;
        }
    }
    throw std::runtime_error("[BagWriter] point cloud connection is not registered: " +
                             topic);
}

void BagWriter::writePath(uint64_t time_ns,
                           const std::vector<V3D>& path,
                           const std::string& frame_id) {
    uint32_t sec = static_cast<uint32_t>(time_ns / 1000000000ULL);
    uint32_t nsec = static_cast<uint32_t>(time_ns % 1000000000ULL);

    RosHeader header;
    header.seq = 0;
    header.stamp_sec = sec;
    header.stamp_nsec = nsec;
    header.frame_id = frame_id;

    auto data = serializePoseArray(header, path);

    for (const auto& pair : connections_) {
        if (pair.second.topic == "/path") {
            writeMessage(pair.first, time_ns, data.data(), data.size());
            return;
        }
    }
    throw std::runtime_error("[BagWriter] path connection is not registered");
}

void BagWriter::writeTF(uint64_t time_ns,
                         const V3D& translation,
                         const Eigen::Quaterniond& rotation,
                         const std::string& parent_frame,
                         const std::string& child_frame) {
    uint32_t sec = static_cast<uint32_t>(time_ns / 1000000000ULL);
    uint32_t nsec = static_cast<uint32_t>(time_ns % 1000000000ULL);

    RosHeader header;
    header.seq = 0;
    header.stamp_sec = sec;
    header.stamp_nsec = nsec;
    header.frame_id = parent_frame;

    auto data = serializeTFMessage(header, child_frame,
        translation.x(), translation.y(), translation.z(),
        rotation.x(), rotation.y(), rotation.z(), rotation.w());

    for (const auto& pair : connections_) {
        if (pair.second.topic == "/tf") {
            writeMessage(pair.first, time_ns, data.data(), data.size());
            return;
        }
    }
    throw std::runtime_error("[BagWriter] TF connection is not registered");
}
