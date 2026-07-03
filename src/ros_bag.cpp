#include "ros_bag.h"
#include <iostream>
#include <cstring>
#include <algorithm>

#ifdef HAS_BZIP2
#include <bzlib.h>
#endif

#ifdef HAS_LZ4
#include <lz4.h>
#pragma comment(lib, "lz4.lib")
#endif

namespace bag {

// ═══════════════════════════════════════════════════════════════════════
//  Construction / Destruction
// ═══════════════════════════════════════════════════════════════════════

BagFileReader::BagFileReader() {}
BagFileReader::~BagFileReader() { close(); }

// ═══════════════════════════════════════════════════════════════════════
//  Header field parsing helpers
// ═══════════════════════════════════════════════════════════════════════

FieldMap BagFileReader::parseHeaderFields(const uint8_t* data, size_t len) {
    FieldMap fields;
    size_t pos = 0;
    while (pos + 4 <= len) {
        uint32_t field_len;
        memcpy(&field_len, data + pos, 4);
        pos += 4;
        if (field_len > len - pos) break;
        std::string field(reinterpret_cast<const char*>(data + pos), field_len);
        pos += field_len;
        auto eq = field.find('=');
        if (eq != std::string::npos) {
            fields[field.substr(0, eq)] = field.substr(eq + 1);
        }
    }
    return fields;
}

std::string BagFileReader::getField(const FieldMap& f, const std::string& key,
                                     const std::string& def) {
    auto it = f.find(key);
    return (it != f.end()) ? it->second : def;
}

uint32_t BagFileReader::getFieldU32(const FieldMap& f, const std::string& key,
                                     uint32_t def) {
    auto it = f.find(key);
    if (it == f.end() || it->second.empty()) return def;
    // Value is stored as raw bytes (not ASCII)
    if (it->second.size() >= 4) {
        uint32_t v;
        memcpy(&v, it->second.data(), 4);
        return v;
    }
    return def;
}

uint64_t BagFileReader::getFieldU64(const FieldMap& f, const std::string& key,
                                     uint64_t def) {
    auto it = f.find(key);
    if (it == f.end() || it->second.empty()) return def;
    if (it->second.size() >= 8) {
        uint64_t v;
        memcpy(&v, it->second.data(), 8);
        return v;
    }
    return def;
}

// ═══════════════════════════════════════════════════════════════════════
//  Record-level I/O
// ═══════════════════════════════════════════════════════════════════════

bool BagFileReader::readRecordHeader(FieldMap& fields, uint32_t& data_len) {
    // Read header_len (4 bytes)
    uint32_t header_len;
    file_.read(reinterpret_cast<char*>(&header_len), 4);
    if (!file_.good() || header_len > 10000000) return false;

    // Read header data
    std::vector<uint8_t> header_data(header_len);
    file_.read(reinterpret_cast<char*>(header_data.data()), header_len);
    if (!file_.good()) return false;

    fields = parseHeaderFields(header_data.data(), header_len);

    // Read data_len (4 bytes)
    file_.read(reinterpret_cast<char*>(&data_len), 4);
    if (!file_.good()) return false;

    return true;
}

bool BagFileReader::readRecordData(std::vector<uint8_t>& data, uint32_t len) {
    data.resize(len);
    if (len > 0) {
        file_.read(reinterpret_cast<char*>(data.data()), len);
        if (!file_.good()) return false;
    }
    return true;
}

bool BagFileReader::skipRecordData(uint32_t len) {
    file_.seekg(len, std::ios::cur);
    return file_.good();
}

// ═══════════════════════════════════════════════════════════════════════
//  Parse FILE_HEADER (op=0x03)
// ═══════════════════════════════════════════════════════════════════════

bool BagFileReader::parseFileHeader(const FieldMap& fields) {
    index_pos_  = getFieldU64(fields, "index_pos", 0);
    conn_count_ = getFieldU32(fields, "conn_count", 0);
    chunk_count_ = getFieldU32(fields, "chunk_count", 0);

    std::cout << "[BagReader] File header: index_pos=" << index_pos_
              << " conn_count=" << conn_count_
              << " chunk_count=" << chunk_count_ << std::endl;
    return (index_pos_ > 0);
}

// ═══════════════════════════════════════════════════════════════════════
//  Parse CONNECTION (op=0x07)
// ═══════════════════════════════════════════════════════════════════════

bool BagFileReader::parseConnection(const FieldMap& header_fields,
                                     const uint8_t* data, size_t data_len) {
    ConnectionInfo conn;
    conn.conn_id = getFieldU32(header_fields, "conn", 0);
    conn.topic   = getField(header_fields, "topic", "");

    // Data contains nested connection header (same format as record header)
    FieldMap conn_fields = parseHeaderFields(data, data_len);
    conn.type = getField(conn_fields, "type", "");
    conn.md5sum = getField(conn_fields, "md5sum", "");
    conn.message_definition = getField(conn_fields, "message_definition", "");

    connections_[conn.conn_id] = conn;
    return true;
}

// ═══════════════════════════════════════════════════════════════════════
//  Parse CHUNK_INFO (op=0x06)
// ═══════════════════════════════════════════════════════════════════════

bool BagFileReader::parseChunkInfo(const FieldMap& fields,
                                    const uint8_t* data, size_t data_len) {
    ChunkInfo ci;
    ci.chunk_pos  = getFieldU64(fields, "chunk_pos", 0);
    ci.start_time = getFieldU64(fields, "start_time", 0);
    ci.end_time   = getFieldU64(fields, "end_time", 0);

    // Data: count × (conn:u32, count:u32)
    uint32_t count = getFieldU32(fields, "count", 0);
    size_t pos = 0;
    for (uint32_t i = 0; i < count && pos + 8 <= data_len; i++) {
        uint32_t conn_id, msg_count;
        memcpy(&conn_id, data + pos, 4); pos += 4;
        memcpy(&msg_count, data + pos, 4); pos += 4;
        ci.conn_counts[conn_id] = msg_count;
    }

    chunk_infos_.push_back(ci);
    return true;
}

// ═══════════════════════════════════════════════════════════════════════
//  Find connection by topic
// ═══════════════════════════════════════════════════════════════════════

int BagFileReader::findConnectionByTopic(const std::string& topic) const {
    for (const auto& pair : connections_) {
        if (pair.second.topic == topic)
            return static_cast<int>(pair.first);
    }
    return -1;
}

// ═══════════════════════════════════════════════════════════════════════
//  Decompression
// ═══════════════════════════════════════════════════════════════════════

bool BagFileReader::decompressChunk(const std::vector<uint8_t>& compressed,
                                     const std::string& compression,
                                     uint32_t uncompressed_size,
                                     std::vector<uint8_t>& output) {
    if (compression == "none" || compression.empty()) {
        output = compressed;
        return true;
    }

#ifdef HAS_BZIP2
    if (compression == "bz2") {
        output.resize(uncompressed_size);
        unsigned int dest_len = uncompressed_size;
        int ret = BZ2_bzBuffToBuffDecompress(
            reinterpret_cast<char*>(output.data()), &dest_len,
            const_cast<char*>(reinterpret_cast<const char*>(compressed.data())),
            static_cast<unsigned int>(compressed.size()),
            0, 0);
        if (ret != BZ_OK) {
            std::cerr << "[BagReader] bzip2 decompression failed: " << ret << std::endl;
            return false;
        }
        output.resize(dest_len);
        return true;
    }
#endif

#ifdef HAS_LZ4
    if (compression == "lz4") {
        output.resize(uncompressed_size);
        int ret = LZ4_decompress_safe(
            reinterpret_cast<const char*>(compressed.data()),
            reinterpret_cast<char*>(output.data()),
            static_cast<int>(compressed.size()),
            static_cast<int>(uncompressed_size));
        if (ret < 0) {
            std::cerr << "[BagReader] lz4 decompression failed: " << ret << std::endl;
            return false;
        }
        output.resize(ret);
        return true;
    }
#endif

    std::cerr << "[BagReader] Unsupported compression: " << compression << std::endl;
    return false;
}

// ═══════════════════════════════════════════════════════════════════════
//  Process a CHUNK — decompress and parse sub-records
// ═══════════════════════════════════════════════════════════════════════

bool BagFileReader::processChunk(uint64_t chunk_file_pos, MessageCallback callback,
                                  int32_t filter_conn_id) {
    // Seek to chunk record
    file_.clear();
    file_.seekg(static_cast<std::streamoff>(chunk_file_pos));
    if (!file_.good()) return false;

    // Read chunk record header
    FieldMap fields;
    uint32_t data_len;
    if (!readRecordHeader(fields, data_len)) return false;

    uint8_t op = 0;
    auto op_it = fields.find("op");
    if (op_it != fields.end() && !op_it->second.empty())
        op = static_cast<uint8_t>(op_it->second[0]);

    if (op != OP_CHUNK) {
        std::cerr << "[BagReader] Expected CHUNK at " << chunk_file_pos
                  << " but got op=" << (int)op << std::endl;
        return false;
    }

    std::string compression = getField(fields, "compression", "none");
    uint32_t uncompressed_size = getFieldU32(fields, "size", data_len);

    // Read chunk data
    std::vector<uint8_t> compressed_data(data_len);
    if (data_len > 0) {
        file_.read(reinterpret_cast<char*>(compressed_data.data()), data_len);
        if (!file_.good()) return false;
    }

    // Decompress
    std::vector<uint8_t> chunk_data;
    if (!decompressChunk(compressed_data, compression, uncompressed_size, chunk_data))
        return false;

    // Parse sub-records within the chunk
    size_t pos = 0;
    while (pos + 8 <= chunk_data.size()) {
        // Sub-record header_len
        uint32_t sub_header_len;
        memcpy(&sub_header_len, chunk_data.data() + pos, 4);
        pos += 4;
        if (sub_header_len > chunk_data.size() - pos) break;

        // Parse sub-record header fields
        FieldMap sub_fields = parseHeaderFields(chunk_data.data() + pos, sub_header_len);
        pos += sub_header_len;

        // Sub-record data_len
        if (pos + 4 > chunk_data.size()) break;
        uint32_t sub_data_len;
        memcpy(&sub_data_len, chunk_data.data() + pos, 4);
        pos += 4;

        if (sub_data_len > chunk_data.size() - pos) break;

        uint8_t sub_op = 0;
        auto sub_op_it = sub_fields.find("op");
        if (sub_op_it != sub_fields.end() && !sub_op_it->second.empty())
            sub_op = static_cast<uint8_t>(sub_op_it->second[0]);

        if (sub_op == OP_CONNECTION) {
            // Connection record inside chunk
            parseConnection(sub_fields, chunk_data.data() + pos, sub_data_len);
        }
        else if (sub_op == OP_MSG_DATA) {
            uint32_t conn_id = getFieldU32(sub_fields, "conn", 0);
            uint64_t time_ns = getFieldU64(sub_fields, "time", 0);

            if (filter_conn_id < 0 || static_cast<int32_t>(conn_id) == filter_conn_id) {
                callback(conn_id, time_ns,
                         chunk_data.data() + pos, sub_data_len);
            }
        }

        pos += sub_data_len;
    }

    return true;
}

// ═══════════════════════════════════════════════════════════════════════
//  Open — parse file structure
// ═══════════════════════════════════════════════════════════════════════

bool BagFileReader::open(const std::string& filepath) {
    filepath_ = filepath;
    file_.open(filepath, std::ios::binary);
    if (!file_.is_open()) {
        std::cerr << "[BagReader] Cannot open: " << filepath << std::endl;
        return false;
    }

    // Read version line
    char version_buf[64];
    file_.getline(version_buf, sizeof(version_buf));
    std::string version(version_buf);
    if (version.find("#ROSBAG V2.0") == std::string::npos) {
        std::cerr << "[BagReader] Not a ROS bag v2.0 file: " << version << std::endl;
        file_.close();
        return false;
    }

    // Read FILE_HEADER record
    FieldMap fields;
    uint32_t data_len;
    if (!readRecordHeader(fields, data_len)) {
        std::cerr << "[BagReader] Failed to read file header" << std::endl;
        file_.close();
        return false;
    }

    if (!parseFileHeader(fields)) {
        std::cerr << "[BagReader] Failed to parse file header" << std::endl;
        file_.close();
        return false;
    }

    // Skip file header data (padded to 4096 bytes typically)
    skipRecordData(data_len);

    // Seek to index_pos to read CONNECTION and CHUNK_INFO records
    file_.clear();
    file_.seekg(static_cast<std::streamoff>(index_pos_));
    if (!file_.good()) {
        std::cerr << "[BagReader] Failed to seek to index_pos=" << index_pos_ << std::endl;
        file_.close();
        return false;
    }

    // Get file size for loop bound
    file_.clear();
    auto file_end = file_.seekg(0, std::ios::end).tellg();
    file_.clear();
    file_.seekg(static_cast<std::streamoff>(index_pos_));

    // Read index section records
    while (file_.good() && file_.tellg() < file_end) {
        auto pos_before = file_.tellg();

        FieldMap idx_fields;
        uint32_t idx_data_len;
        if (!readRecordHeader(idx_fields, idx_data_len)) break;

        uint8_t idx_op = 0;
        auto op_it = idx_fields.find("op");
        if (op_it != idx_fields.end() && !op_it->second.empty())
            idx_op = static_cast<uint8_t>(op_it->second[0]);

        std::vector<uint8_t> idx_data;
        if (!readRecordData(idx_data, idx_data_len)) break;

        if (idx_op == OP_CONNECTION) {
            parseConnection(idx_fields, idx_data.data(), idx_data.size());
        }
        else if (idx_op == OP_CHUNK_INFO) {
            parseChunkInfo(idx_fields, idx_data.data(), idx_data.size());
        }

        // Safety check to avoid infinite loop
        if (file_.tellg() <= pos_before) break;
    }

    std::cout << "[BagReader] Opened: " << filepath
              << " (connections=" << connections_.size()
              << ", chunks=" << chunk_infos_.size() << ")" << std::endl;

    // Log connections
    for (const auto& pair : connections_) {
        std::cout << "[BagReader]   conn " << pair.first
                  << ": " << pair.second.topic
                  << " (" << pair.second.type << ")" << std::endl;
    }

    return true;
}

void BagFileReader::close() {
    if (file_.is_open()) file_.close();
    connections_.clear();
    chunk_infos_.clear();
}

// ═══════════════════════════════════════════════════════════════════════
//  Read all messages
// ═══════════════════════════════════════════════════════════════════════

bool BagFileReader::readAllMessages(MessageCallback callback) {
    if (!file_.is_open()) return false;

    // Sort chunks by file position
    std::vector<ChunkInfo> sorted_chunks = chunk_infos_;
    std::sort(sorted_chunks.begin(), sorted_chunks.end(),
              [](const ChunkInfo& a, const ChunkInfo& b) {
                  return a.chunk_pos < b.chunk_pos;
              });

    uint32_t chunk_idx = 0;
    for (const auto& ci : sorted_chunks) {
        if (!processChunk(ci.chunk_pos, callback)) {
            std::cerr << "[BagReader] Failed to process chunk " << chunk_idx << std::endl;
        }
        chunk_idx++;
        if (chunk_idx % 100 == 0) {
            std::cout << "[BagReader] Processed chunk " << chunk_idx
                      << "/" << sorted_chunks.size() << std::endl;
        }
    }

    std::cout << "[BagReader] Finished reading " << chunk_idx << " chunks" << std::endl;
    return true;
}

// ═══════════════════════════════════════════════════════════════════════
//  Read messages for a specific topic
// ═══════════════════════════════════════════════════════════════════════

bool BagFileReader::readMessagesByTopic(const std::string& topic, MessageCallback callback) {
    int conn_id = findConnectionByTopic(topic);
    if (conn_id < 0) {
        std::cerr << "[BagReader] Topic not found: " << topic << std::endl;
        return false;
    }
    return readAllMessages([callback, conn_id](uint32_t msg_conn, uint64_t time_ns,
                                                const uint8_t* data, size_t len) {
        if (static_cast<int32_t>(msg_conn) == conn_id) {
            callback(msg_conn, time_ns, data, len);
        }
    });
}

} // namespace bag
