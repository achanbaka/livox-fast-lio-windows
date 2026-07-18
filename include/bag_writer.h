#ifndef BAG_WRITER_H
#define BAG_WRITER_H

#include <string>
#include <fstream>
#include <functional>
#include <vector>
#include <map>
#include <cstdint>
#include <mutex>
#include "types.h"
#include "ros_message.h"

/**
 * BagWriter — writes SLAM results to a ROS1 bag file.
 * Uses uncompressed chunks (compression="none") for simplicity.
 */
class BagWriter {
public:
    using IoHook = std::function<void(const char* operation)>;

    explicit BagWriter(IoHook io_hook = {});
    ~BagWriter();

    bool open(const std::string& filepath);
    void close();
    bool isOpen() const { return file_.is_open(); }

    // Register a connection (topic) and return its conn_id
    uint32_t addConnection(const std::string& topic,
                           const std::string& type,
                           const std::string& md5sum,
                           const std::string& msg_def);

    // Write a serialized message
    void writeMessage(uint32_t conn_id, uint64_t time_ns,
                      const uint8_t* data, size_t len);

    // ── Convenience methods ───────────────────────────────────────

    void writeOdometry(uint64_t time_ns,
                       const V3D& position,
                       const Eigen::Quaterniond& orientation,
                       const std::string& frame_id = "world");

    void writePointCloud(uint64_t time_ns,
                         const PointCloudXYZI& cloud,
                         const std::string& frame_id = "world",
                         const std::string& topic = "/cloud_registered");

    void writePath(uint64_t time_ns,
                   const std::vector<V3D>& path,
                   const std::string& frame_id = "world");

    void writeTF(uint64_t time_ns,
                 const V3D& translation,
                 const Eigen::Quaterniond& rotation,
                 const std::string& parent_frame,
                 const std::string& child_frame);

private:
    // Internal structures
    struct ConnectionInfo {
        uint32_t conn_id;
        std::string topic;
        std::string type;
        std::string md5sum;
        std::string msg_def;
    };

    struct ChunkMsgEntry {
        uint32_t conn_id;
        uint64_t time_ns;
        uint32_t offset_in_chunk; // offset within the uncompressed chunk data
    };

    struct ChunkRecord {
        uint64_t file_pos;       // file offset of the CHUNK record
        uint64_t start_time_ns;
        uint64_t end_time_ns;
        std::map<uint32_t, uint32_t> conn_counts;
    };

    // Write helpers
    void writeRaw(const void* data, size_t len);
    void writeU32(uint32_t v);
    uint64_t tellPosition(const char* operation);
    void seekPosition(std::streamoff offset, const char* operation);
    void ensureStreamGood(const char* operation) const;
    void closeFileNoThrow() noexcept;
    void invokeIoHook(const char* operation);

    // Chunk management
    void flushChunk();
    void writeConnectionRecords();
    void writeChunkInfoRecords();
    void writeFileHeader();

    // Serialize a MSG_DATA sub-record into the chunk buffer
    void appendMsgToChunk(uint32_t conn_id, uint64_t time_ns,
                          const uint8_t* data, size_t len);

    std::ofstream file_;
    std::string filepath_;
    IoHook io_hook_;

    // Connections
    std::map<uint32_t, ConnectionInfo> connections_;
    uint32_t next_conn_id_ = 0;

    // Current chunk buffer
    std::vector<uint8_t> chunk_buffer_;
    std::vector<ChunkMsgEntry> chunk_messages_;
    uint64_t chunk_start_time_ = 0;
    uint64_t chunk_end_time_ = 0;
    static const size_t CHUNK_MAX_SIZE = 10 * 1024 * 1024; // 10 MB

    // Chunk records for CHUNK_INFO
    std::vector<ChunkRecord> chunk_records_;

    // Sequence counters per connection
    std::map<uint32_t, uint32_t> seq_counters_;

    std::mutex mutex_;
};

#endif // BAG_WRITER_H
