#include "bag_writer.h"
#include <iostream>
#include <cstring>
#include <algorithm>
#include <sstream>

// ═══════════════════════════════════════════════════════════════════════
//  Construction / Destruction
// ═══════════════════════════════════════════════════════════════════════

BagWriter::BagWriter() {}

BagWriter::~BagWriter() {
    close();
}

// ═══════════════════════════════════════════════════════════════════════
//  Low-level write helpers
// ═══════════════════════════════════════════════════════════════════════

void BagWriter::writeRaw(const void* data, size_t len) {
    file_.write(static_cast<const char*>(data), static_cast<std::streamsize>(len));
}

void BagWriter::writeU32(uint32_t v) {
    writeRaw(&v, 4);
}

void BagWriter::writeU64(uint64_t v) {
    writeRaw(&v, 8);
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

// Encode a field with raw byte value (for op field)
static std::vector<uint8_t> encodeFieldRaw(const std::string& key, const void* value, size_t value_len) {
    std::string key_eq = key + "=";
    uint32_t field_len = static_cast<uint32_t>(key_eq.size() + value_len);
    std::vector<uint8_t> result(4 + field_len);
    memcpy(result.data(), &field_len, 4);
    memcpy(result.data() + 4, key_eq.data(), key_eq.size());
    memcpy(result.data() + 4 + key_eq.size(), value, value_len);
    return result;
}

void BagWriter::writeRecordHeader(const std::vector<std::pair<std::string, std::string>>& fields,
                                   uint32_t data_len) {
    // Build header bytes
    std::vector<uint8_t> header;
    for (const auto& f : fields) {
        auto encoded = encodeField(f.first, f.second);
        header.insert(header.end(), encoded.begin(), encoded.end());
    }

    // Write header_len + header + data_len
    writeU32(static_cast<uint32_t>(header.size()));
    writeRaw(header.data(), header.size());
    writeU32(data_len);
}

// ═══════════════════════════════════════════════════════════════════════
//  Open / Close
// ═══════════════════════════════════════════════════════════════════════

bool BagWriter::open(const std::string& filepath) {
    filepath_ = filepath;
    file_.open(filepath, std::ios::binary | std::ios::trunc);
    if (!file_.is_open()) {
        std::cerr << "[BagWriter] Cannot open: " << filepath << std::endl;
        return false;
    }

    // Write version line
    const char* version = "#ROSBAG V2.0\n";
    writeRaw(version, strlen(version));

    // Write placeholder BAG_HEADER (will be rewritten on close)
    // The BAG_HEADER data is padded to 4096 bytes total
    writeFileHeader();

    std::cout << "[BagWriter] Opened: " << filepath << std::endl;
    return true;
}

void BagWriter::writeFileHeader() {
    // We'll write a placeholder; the real values are written in close()
    // For now, just reserve space (4096 bytes total for the header record)
    uint64_t start_pos = static_cast<uint64_t>(file_.tellp());

    // Build header fields
    uint8_t op = 0x03; // FILE_HEADER
    std::vector<std::pair<std::string, std::string>> fields;
    fields.push_back({"op", std::string(1, static_cast<char>(op))});
    fields.push_back({"index_pos", "0"});      // placeholder
    fields.push_back({"conn_count", "0"});      // placeholder
    fields.push_back({"chunk_count", "0"});     // placeholder

    // Build header bytes
    std::vector<uint8_t> header;
    for (const auto& f : fields) {
        auto encoded = encodeField(f.first, f.second);
        header.insert(header.end(), encoded.begin(), encoded.end());
    }

    // Write: header_len + header + data_len(0)
    writeU32(static_cast<uint32_t>(header.size()));
    writeRaw(header.data(), header.size());
    writeU32(0); // data_len = 0 for FILE_HEADER

    // Pad to 4096 bytes
    uint64_t current_pos = static_cast<uint64_t>(file_.tellp());
    uint64_t header_end = start_pos + 4096;
    if (current_pos < header_end) {
        std::vector<char> padding(header_end - current_pos, 0);
        writeRaw(padding.data(), padding.size());
    }
}

void BagWriter::close() {
    if (!file_.is_open()) return;

    std::lock_guard<std::mutex> lock(mutex_);

    // Flush remaining chunk
    if (!chunk_buffer_.empty()) {
        flushChunk();
    }

    // Record the index position (where CONNECTION records start)
    uint64_t index_pos = static_cast<uint64_t>(file_.tellp());

    // Write CONNECTION records
    writeConnectionRecords();

    // Write CHUNK_INFO records
    writeChunkInfoRecords();

    // Rewrite BAG_HEADER with correct values
    file_.seekp(13); // After "#ROSBAG V2.0\n"

    uint8_t op = 0x03;
    std::vector<std::pair<std::string, std::string>> fields;
    fields.push_back({"op", std::string(1, static_cast<char>(op))});
    fields.push_back({"index_pos", std::to_string(index_pos)});
    fields.push_back({"conn_count", std::to_string(connections_.size())});
    fields.push_back({"chunk_count", std::to_string(chunk_records_.size())});

    std::vector<uint8_t> header;
    for (const auto& f : fields) {
        auto encoded = encodeField(f.first, f.second);
        header.insert(header.end(), encoded.begin(), encoded.end());
    }

    writeU32(static_cast<uint32_t>(header.size()));
    writeRaw(header.data(), header.size());
    writeU32(0); // data_len = 0

    file_.close();
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
    // MSG_DATA sub-record within chunk:
    // [header_len:u32][op(1B) + conn(4B) + time(8B)][data_len:u32][message_data]

    // Build sub-header
    std::vector<uint8_t> sub_header;
    // op field: 1 byte
    uint8_t op = 0x02; // MSG_DATA
    sub_header.push_back(op);
    // conn field: 4 bytes
    sub_header.insert(sub_header.end(),
                      reinterpret_cast<uint8_t*>(&conn_id),
                      reinterpret_cast<uint8_t*>(&conn_id) + 4);
    // time field: 8 bytes
    sub_header.insert(sub_header.end(),
                      reinterpret_cast<uint8_t*>(&time_ns),
                      reinterpret_cast<uint8_t*>(&time_ns) + 8);

    // But we need to encode as field=value format for sub-records too
    // Actually in chunk sub-records, the header uses the same format:
    // [field_len:u32][key=value] for each field
    // Let me re-encode properly:
    sub_header.clear();

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
    // time = time_ns as 8-byte raw
    {
        std::string key_eq = "time=";
        uint32_t fl = static_cast<uint32_t>(key_eq.size() + 8);
        auto old_size = sub_header.size();
        sub_header.resize(old_size + 4 + fl);
        memcpy(sub_header.data() + old_size, &fl, 4);
        memcpy(sub_header.data() + old_size + 4, key_eq.data(), key_eq.size());
        memcpy(sub_header.data() + old_size + 4 + key_eq.size(), &time_ns, 8);
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
    }
    chunk_end_time_ = time_ns;

    // Update global index
    connection_index_[conn_id].push_back({time_ns, offset});
}

void BagWriter::writeMessage(uint32_t conn_id, uint64_t time_ns,
                              const uint8_t* data, size_t len) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!file_.is_open()) return;

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

    uint64_t chunk_file_pos = static_cast<uint64_t>(file_.tellp());

    // Write CHUNK record header
    uint8_t op = 0x05; // CHUNK
    std::vector<std::pair<std::string, std::string>> fields;
    fields.push_back({"op", std::string(1, static_cast<char>(op))});
    fields.push_back({"compression", "none"});
    fields.push_back({"size", std::to_string(chunk_buffer_.size())});

    std::vector<uint8_t> header;
    for (const auto& f : fields) {
        auto encoded = encodeField(f.first, f.second);
        header.insert(header.end(), encoded.begin(), encoded.end());
    }

    writeU32(static_cast<uint32_t>(header.size()));
    writeRaw(header.data(), header.size());
    writeU32(static_cast<uint32_t>(chunk_buffer_.size()));
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
        const auto& entries = pair.second;

        // INDEX_DATA header
        std::vector<std::pair<std::string, std::string>> idx_fields;
        uint8_t idx_op = 0x04; // INDEX_DATA
        idx_fields.push_back({"op", std::string(1, static_cast<char>(idx_op))});
        idx_fields.push_back({"ver", "1"});

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

        // INDEX_DATA data: count × (time:u64, offset:u32)
        std::vector<uint8_t> idx_data;
        for (const auto& e : entries) {
            uint64_t t = e.first;
            uint32_t o = e.second;
            idx_data.insert(idx_data.end(),
                           reinterpret_cast<uint8_t*>(&t),
                           reinterpret_cast<uint8_t*>(&t) + 8);
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
        fields.push_back({"ver", "1"});

        // chunk_pos as 8-byte raw
        std::string pos_val(8, '\0');
        memcpy(&pos_val[0], &chunk.file_pos, 8);
        fields.push_back({"chunk_pos", pos_val});

        // start_time as 8-byte raw (sec + nsec)
        uint32_t start_sec = static_cast<uint32_t>(chunk.start_time_ns / 1000000000ULL);
        uint32_t start_nsec = static_cast<uint32_t>(chunk.start_time_ns % 1000000000ULL);
        std::string start_val(8, '\0');
        memcpy(&start_val[0], &start_sec, 4);
        memcpy(&start_val[4], &start_nsec, 4);
        fields.push_back({"start_time", start_val});

        // end_time as 8-byte raw
        uint32_t end_sec = static_cast<uint32_t>(chunk.end_time_ns / 1000000000ULL);
        uint32_t end_nsec = static_cast<uint32_t>(chunk.end_time_ns % 1000000000ULL);
        std::string end_val(8, '\0');
        memcpy(&end_val[0], &end_sec, 4);
        memcpy(&end_val[4], &end_nsec, 4);
        fields.push_back({"end_time", end_val});

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
}
