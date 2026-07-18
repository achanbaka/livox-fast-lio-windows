#ifndef YAML_CONFIG_H
#define YAML_CONFIG_H

#include <string>
#include <vector>
#include <yaml-cpp/yaml.h>
#include "types.h"

struct FastLioConfig
{
    // Common
    std::string lid_topic;
    std::string imu_topic;
    bool time_sync_en;
    double time_offset_lidar_to_imu;
    std::string livox_broadcast_code;
    double realtime_frame_sec;
    int realtime_frame_points;

    // Preprocess
    int lidar_type;
    int scan_line;
    double blind;
    bool feature_enabled;
    int point_filter_num;

    // Mapping
    double acc_cov;
    double gyr_cov;
    double b_acc_cov;
    double b_gyr_cov;
    double fov_degree;
    double det_range;
    bool extrinsic_est_en;
    V3D extrinsic_T;
    M3D extrinsic_R;

    // Publish
    bool path_en;
    bool scan_publish_en;
    bool dense_publish_en;
    bool scan_bodyframe_pub_en;
    bool publish_full_map;
    bool async_full_map_publish;
    int full_map_publish_interval_ms;
    double full_map_voxel_size;
    bool bag_full_map_periodic;
    bool publish_map_delta;
    int map_delta_max_pending_points;
    int foxglove_control_interval_ms;
    int foxglove_backlog_size;
    double foxglove_current_cloud_publish_hz;
    double foxglove_path_publish_hz;

    // Scalable map output pipeline. Legacy publish.* fields are still loaded
    // and are used as fallbacks when a corresponding map_output field is absent.
    std::string map_output_mode;
    int map_output_full_publish_interval_ms;
    double map_output_full_voxel_leaf_m;
    double map_output_tile_size_m;
    double map_output_tile_voxel_leaf_m;
    std::string map_output_voxel_update_policy;
    int map_output_tile_publish_hz;
    int map_output_max_tiles_per_update;
    int map_output_max_points_per_update;
    int map_output_input_queue_capacity;
    int map_output_input_queue_max_mb;
    int map_output_max_memory_mb;
    std::string map_output_memory_policy;

    // Storage pipeline policy. The worker implementation consumes these fields
    // incrementally; keeping them in config now makes old/new files migratable.
    std::string storage_mode;
    int storage_queue_max_mb;
    double storage_bag_path_publish_hz;
    int storage_path_max_points;
    std::string storage_pcd_format;
    int storage_pcd_chunk_points;
    int storage_pcd_chunk_frames;
    bool storage_save_final_ikdtree;

    // PCD save
    bool pcd_save_en;
    int pcd_interval;

    // Runtime
    int max_iteration;
    int max_feature_points;
    int iekf_match_threads;
    double filter_size_map_min;
    double filter_size_surf_min;
    int cube_side_length;
    bool runtime_pos_log;
    std::string root_dir;
};

class YamlConfig
{
public:
    YamlConfig();
    ~YamlConfig() = default;

    bool load(const std::string& filepath);
    const FastLioConfig& getConfig() const { return config_; }
    FastLioConfig& getConfig() { return config_; }

    // Parse command line overrides
    void applyOverrides(int argc, char* argv[]);
    bool validate(std::string& error) const;

private:
    FastLioConfig config_;
};

#endif
