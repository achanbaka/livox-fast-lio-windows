#ifndef ROS_BAG_H
#define ROS_BAG_H

#include <cstdint>
#include <string>
#include <vector>
#include <map>
#include <functional>
#include <fstream>

// ═══════════════════════════════════════════════════════════════════════
//  ROS1 Bag File Format — Low-Level Parser
//
//  Op codes (from ROS source rosbag/constants.h):
//    0x03 = FILE_HEADER
//    0x02 = MSG_DATA
//    0x05 = CHUNK
//    0x07 = CONNECTION
//    0x04 = INDEX_DATA
//    0x06 = CHUNK_INFO
// ═══════════════════════════════════════════════════════════════════════

namespace bag {

// Op code constants
enum OpCode : uint8_t {
    OP_MSG_DATA    = 0x02,
    OP_FILE_HEADER = 0x03,
    OP_INDEX_DATA  = 0x04,
    OP_CHUNK       = 0x05,
    OP_CHUNK_INFO  = 0x06,
    OP_CONNECTION  = 0x07,
};

// Parsed header field map
using FieldMap = std::map<std::string, std::string>;

// Connection info
struct ConnectionInfo {
    uint32_t conn_id = 0;
    std::string topic;
    std::string type;         // e.g. "livox_ros_driver/CustomMsg"
    std::string md5sum;
    std::string message_definition;
};

// Chunk metadata (from CHUNK_INFO records)
struct ChunkInfo {
    uint64_t chunk_pos = 0;   // file offset of the CHUNK record
    uint64_t start_time = 0;  // ns
    uint64_t end_time = 0;    // ns
    std::map<uint32_t, uint32_t> conn_counts; // conn_id → message count
};

// A single message extracted from the bag
struct Message {
    uint32_t conn_id = 0;
    uint64_t time_ns = 0;     // timestamp in nanoseconds
    std::vector<uint8_t> data;
};

// ═══════════════════════════════════════════════════════════════════════
//  BagFileReader — reads and parses a ROS1 .bag file
// ═══════════════════════════════════════════════════════════════════════

class BagFileReader {
public:
    BagFileReader();
    ~BagFileReader();

    // Open and parse file structure (header, connections, chunk list)
    bool open(const std::string& filepath);
    void close();
    bool isOpen() const { return file_.is_open(); }

    // Access parsed metadata
    const std::map<uint32_t, ConnectionInfo>& connections() const { return connections_; }
    const std::vector<ChunkInfo>& chunkInfos() const { return chunk_infos_; }
    uint32_t chunkCount() const { return chunk_count_; }
    uint32_t connCount() const { return conn_count_; }

    // Find connection by topic name (returns conn_id or -1)
    int findConnectionByTopic(const std::string& topic) const;

    // Read all messages, calling callback for each one
    // callback(conn_id, time_ns, data_ptr, data_len)
    using MessageCallback = std::function<void(uint32_t conn_id, uint64_t time_ns,
                                               const uint8_t* data, size_t len)>;
    bool readAllMessages(MessageCallback callback);

    // Read messages for a specific topic only
    bool readMessagesByTopic(const std::string& topic, MessageCallback callback);

private:
    // Record-level parsing
    bool readRecordHeader(FieldMap& fields, uint32_t& data_len);
    bool readRecordData(std::vector<uint8_t>& data, uint32_t len);
    bool skipRecordData(uint32_t len);

    // Parse specific record types
    bool parseFileHeader(const FieldMap& fields);
    bool parseConnection(const FieldMap& header_fields,
                         const uint8_t* data, size_t data_len);
    bool parseChunkInfo(const FieldMap& fields, const uint8_t* data, size_t data_len);

    // Process a chunk: decompress and parse sub-records
    bool processChunk(uint64_t chunk_file_pos, MessageCallback callback,
                      int32_t filter_conn_id = -1);

    // Decompression
    bool decompressChunk(const std::vector<uint8_t>& compressed,
                         const std::string& compression,
                         uint32_t uncompressed_size,
                         std::vector<uint8_t>& output);

    // Header field parsing helpers
    static FieldMap parseHeaderFields(const uint8_t* data, size_t len);
    static std::string getField(const FieldMap& fields, const std::string& key,
                                const std::string& default_val = "");
    static uint32_t getFieldU32(const FieldMap& fields, const std::string& key,
                                 uint32_t default_val = 0);
    static uint64_t getFieldU64(const FieldMap& fields, const std::string& key,
                                 uint64_t default_val = 0);

    std::ifstream file_;
    std::string filepath_;

    // File structure
    uint64_t index_pos_ = 0;
    uint32_t conn_count_ = 0;
    uint32_t chunk_count_ = 0;

    // Parsed connections (conn_id → info)
    std::map<uint32_t, ConnectionInfo> connections_;

    // Chunk info records
    std::vector<ChunkInfo> chunk_infos_;
};

} // namespace bag

#endif // ROS_BAG_H
