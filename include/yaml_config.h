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

    // PCD save
    bool pcd_save_en;
    int pcd_interval;

    // Runtime
    int max_iteration;
    int max_feature_points;
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

private:
    FastLioConfig config_;
};

#endif
