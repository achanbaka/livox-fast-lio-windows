#include "yaml_config.h"
#include <algorithm>
#include <cmath>
#include <iostream>
#include <fstream>

YamlConfig::YamlConfig()
{
    // Set defaults
    config_.lid_topic = "/livox/lidar";
    config_.imu_topic = "/livox/imu";
    config_.time_sync_en = false;
    config_.time_offset_lidar_to_imu = 0.0;
    config_.livox_broadcast_code = "";
    config_.realtime_frame_sec = 0.10;
    config_.realtime_frame_points = 20000;
    config_.lidar_type = 1;
    config_.scan_line = 6;
    config_.blind = 4.0;
    config_.feature_enabled = false;
    config_.point_filter_num = 3;
    config_.acc_cov = 0.1;
    config_.gyr_cov = 0.1;
    config_.b_acc_cov = 0.0001;
    config_.b_gyr_cov = 0.0001;
    config_.fov_degree = 100.0;
    config_.det_range = 260.0;
    config_.extrinsic_est_en = true;
    config_.extrinsic_T = V3D(0.05512, 0.02226, -0.0297);
    config_.extrinsic_R = M3D::Identity();
    config_.path_en = true;
    config_.scan_publish_en = true;
    config_.dense_publish_en = true;
    config_.scan_bodyframe_pub_en = true;
    config_.publish_full_map = true;
    config_.async_full_map_publish = true;
    config_.full_map_publish_interval_ms = 1000;
    config_.full_map_voxel_size = 0.2;
    config_.bag_full_map_periodic = false;
    config_.publish_map_delta = false;
    config_.map_delta_max_pending_points = 200000;
    config_.foxglove_control_interval_ms = 20;
    config_.foxglove_backlog_size = 64;
    config_.foxglove_current_cloud_publish_hz = 20.0;
    config_.foxglove_path_publish_hz = 2.0;
    config_.map_output_mode = "full_async";
    config_.map_output_full_publish_interval_ms = 5000;
    config_.map_output_full_voxel_leaf_m = 0.2;
    config_.map_output_tile_size_m = 20.0;
    config_.map_output_tile_voxel_leaf_m = 0.2;
    config_.map_output_voxel_update_policy = "latest";
    config_.map_output_tile_publish_hz = 10;
    config_.map_output_max_tiles_per_update = 32;
    config_.map_output_max_points_per_update = 200000;
    config_.map_output_input_queue_capacity = 64;
    config_.map_output_input_queue_max_mb = 128;
    config_.map_output_max_memory_mb = 1024;
    config_.map_output_memory_policy = "stop_accumulating";
    config_.storage_mode = "realtime";
    config_.storage_queue_max_mb = 512;
    config_.storage_bag_path_publish_hz = 1.0;
    config_.storage_path_max_points = 100000;
    config_.storage_pcd_format = "binary_compressed";
    config_.storage_pcd_chunk_points = 1000000;
    config_.storage_pcd_chunk_frames = 0;
    config_.storage_save_final_ikdtree = true;
    config_.pcd_save_en = true;
    config_.pcd_interval = -1;
    config_.max_iteration = 3;
    config_.max_feature_points = 2000;
    config_.iekf_match_threads = 4;
    config_.filter_size_map_min = 0.5;
    config_.filter_size_surf_min = 0.5;
    config_.cube_side_length = 1000;
    config_.runtime_pos_log = false;
    config_.root_dir = ".";
}

bool YamlConfig::load(const std::string& filepath)
{
    try
    {
        YAML::Node root = YAML::LoadFile(filepath);
        
        // Common section
        if (root["common"])
        {
            auto common = root["common"];
            if (common["lid_topic"]) config_.lid_topic = common["lid_topic"].as<std::string>();
            if (common["imu_topic"]) config_.imu_topic = common["imu_topic"].as<std::string>();
            if (common["time_sync_en"]) config_.time_sync_en = common["time_sync_en"].as<bool>();
            if (common["time_offset_lidar_to_imu"]) config_.time_offset_lidar_to_imu = common["time_offset_lidar_to_imu"].as<double>();
            if (common["livox_broadcast_code"]) config_.livox_broadcast_code = common["livox_broadcast_code"].as<std::string>();
            if (common["realtime_frame_sec"]) config_.realtime_frame_sec = common["realtime_frame_sec"].as<double>();
            if (common["realtime_frame_points"]) config_.realtime_frame_points = common["realtime_frame_points"].as<int>();
        }

        // Preprocess section
        if (root["preprocess"])
        {
            auto pre = root["preprocess"];
            if (pre["lidar_type"]) config_.lidar_type = pre["lidar_type"].as<int>();
            if (pre["scan_line"]) config_.scan_line = pre["scan_line"].as<int>();
            if (pre["blind"]) config_.blind = pre["blind"].as<double>();
            if (pre["feature_enabled"]) config_.feature_enabled = pre["feature_enabled"].as<bool>();
            if (pre["point_filter_num"]) config_.point_filter_num = pre["point_filter_num"].as<int>();
        }

        // Mapping section
        if (root["mapping"])
        {
            auto map = root["mapping"];
            if (map["acc_cov"]) config_.acc_cov = map["acc_cov"].as<double>();
            if (map["gyr_cov"]) config_.gyr_cov = map["gyr_cov"].as<double>();
            if (map["b_acc_cov"]) config_.b_acc_cov = map["b_acc_cov"].as<double>();
            if (map["b_gyr_cov"]) config_.b_gyr_cov = map["b_gyr_cov"].as<double>();
            if (map["fov_degree"]) config_.fov_degree = map["fov_degree"].as<double>();
            if (map["det_range"]) config_.det_range = map["det_range"].as<double>();
            if (map["max_iteration"]) config_.max_iteration = map["max_iteration"].as<int>();
            if (map["max_feature_points"]) config_.max_feature_points = map["max_feature_points"].as<int>();
            if (map["iekf_match_threads"]) config_.iekf_match_threads = map["iekf_match_threads"].as<int>();
            if (map["filter_size_surf"]) config_.filter_size_surf_min = map["filter_size_surf"].as<double>();
            if (map["filter_size_map"]) config_.filter_size_map_min = map["filter_size_map"].as<double>();
            if (map["cube_side_length"]) config_.cube_side_length = map["cube_side_length"].as<int>();
            if (map["extrinsic_est_en"]) config_.extrinsic_est_en = map["extrinsic_est_en"].as<bool>();
            
            if (map["extrinsic_T"])
            {
                auto T = map["extrinsic_T"].as<std::vector<double>>();
                if (T.size() >= 3)
                    config_.extrinsic_T = V3D(T[0], T[1], T[2]);
            }
            
            if (map["extrinsic_R"])
            {
                auto R = map["extrinsic_R"].as<std::vector<double>>();
                if (R.size() >= 9)
                {
                    config_.extrinsic_R << R[0], R[1], R[2],
                                           R[3], R[4], R[5],
                                           R[6], R[7], R[8];
                }
            }
        }

        if (root["runtime"])
        {
            auto runtime = root["runtime"];
            if (runtime["runtime_pos_log_enable"]) config_.runtime_pos_log = runtime["runtime_pos_log_enable"].as<bool>();
        }

        // Publish section
        if (root["publish"])
        {
            auto pub = root["publish"];
            if (pub["path_en"]) config_.path_en = pub["path_en"].as<bool>();
            if (pub["scan_publish_en"]) config_.scan_publish_en = pub["scan_publish_en"].as<bool>();
            if (pub["dense_publish_en"]) config_.dense_publish_en = pub["dense_publish_en"].as<bool>();
            if (pub["scan_bodyframe_pub_en"]) config_.scan_bodyframe_pub_en = pub["scan_bodyframe_pub_en"].as<bool>();
            if (pub["publish_full_map"]) config_.publish_full_map = pub["publish_full_map"].as<bool>();
            if (pub["full_map_publish_en"]) config_.publish_full_map = pub["full_map_publish_en"].as<bool>();
            if (pub["async_full_map_publish"]) config_.async_full_map_publish = pub["async_full_map_publish"].as<bool>();
            if (pub["full_map_publish_interval_ms"]) config_.full_map_publish_interval_ms = pub["full_map_publish_interval_ms"].as<int>();
            if (pub["full_map_voxel_size"]) config_.full_map_voxel_size = pub["full_map_voxel_size"].as<double>();
            if (pub["bag_full_map_periodic"]) config_.bag_full_map_periodic = pub["bag_full_map_periodic"].as<bool>();
            if (pub["publish_map_delta"]) config_.publish_map_delta = pub["publish_map_delta"].as<bool>();
            if (pub["map_delta_max_pending_points"]) config_.map_delta_max_pending_points = pub["map_delta_max_pending_points"].as<int>();
            if (pub["foxglove_control_interval_ms"]) config_.foxglove_control_interval_ms = pub["foxglove_control_interval_ms"].as<int>();
            if (pub["foxglove_backlog_size"]) config_.foxglove_backlog_size = pub["foxglove_backlog_size"].as<int>();
        }

        // New grouped map-output config wins over legacy publish.* fields.
        // Start by applying the legacy mapping so existing YAML files preserve
        // their current behaviour.
        config_.map_output_full_publish_interval_ms =
            config_.full_map_publish_interval_ms;
        config_.map_output_full_voxel_leaf_m = config_.full_map_voxel_size;
        config_.map_output_max_points_per_update =
            config_.map_delta_max_pending_points;
        config_.map_output_mode = config_.async_full_map_publish
            ? (config_.publish_map_delta ? "hybrid" : "full_async")
            : (config_.publish_map_delta ? "tiled_incremental" : "full_async");

        if (root["map_output"])
        {
            auto map_output = root["map_output"];
            if (map_output["mode"]) config_.map_output_mode = map_output["mode"].as<std::string>();
            if (map_output["full_publish_interval_ms"]) config_.map_output_full_publish_interval_ms = map_output["full_publish_interval_ms"].as<int>();
            if (map_output["full_voxel_leaf_m"]) config_.map_output_full_voxel_leaf_m = map_output["full_voxel_leaf_m"].as<double>();
            if (map_output["tile_size_m"]) config_.map_output_tile_size_m = map_output["tile_size_m"].as<double>();
            if (map_output["tile_voxel_leaf_m"]) config_.map_output_tile_voxel_leaf_m = map_output["tile_voxel_leaf_m"].as<double>();
            if (map_output["voxel_update_policy"]) config_.map_output_voxel_update_policy = map_output["voxel_update_policy"].as<std::string>();
            if (map_output["tile_publish_hz"]) config_.map_output_tile_publish_hz = map_output["tile_publish_hz"].as<int>();
            if (map_output["max_tiles_per_update"]) config_.map_output_max_tiles_per_update = map_output["max_tiles_per_update"].as<int>();
            if (map_output["max_points_per_update"]) config_.map_output_max_points_per_update = map_output["max_points_per_update"].as<int>();
            if (map_output["input_queue_capacity"]) config_.map_output_input_queue_capacity = map_output["input_queue_capacity"].as<int>();
            if (map_output["input_queue_max_mb"]) config_.map_output_input_queue_max_mb = map_output["input_queue_max_mb"].as<int>();
            if (map_output["max_memory_mb"]) config_.map_output_max_memory_mb = map_output["max_memory_mb"].as<int>();
            if (map_output["memory_policy"]) config_.map_output_memory_policy = map_output["memory_policy"].as<std::string>();
        }

        if (root["foxglove"])
        {
            auto foxglove = root["foxglove"];
            if (foxglove["current_cloud_publish_hz"]) config_.foxglove_current_cloud_publish_hz = foxglove["current_cloud_publish_hz"].as<double>();
            if (foxglove["path_publish_hz"]) config_.foxglove_path_publish_hz = foxglove["path_publish_hz"].as<double>();
            if (foxglove["backlog_size"]) config_.foxglove_backlog_size = foxglove["backlog_size"].as<int>();
        }

        if (root["storage"])
        {
            auto storage = root["storage"];
            if (storage["mode"]) config_.storage_mode = storage["mode"].as<std::string>();
            if (storage["queue_max_mb"]) config_.storage_queue_max_mb = storage["queue_max_mb"].as<int>();
            if (storage["bag_path_publish_hz"]) config_.storage_bag_path_publish_hz = storage["bag_path_publish_hz"].as<double>();
            if (storage["path_max_points"]) config_.storage_path_max_points = storage["path_max_points"].as<int>();
            if (storage["pcd_format"]) config_.storage_pcd_format = storage["pcd_format"].as<std::string>();
            if (storage["pcd_chunk_points"]) config_.storage_pcd_chunk_points = storage["pcd_chunk_points"].as<int>();
            if (storage["pcd_chunk_frames"]) config_.storage_pcd_chunk_frames = storage["pcd_chunk_frames"].as<int>();
            if (storage["save_final_ikdtree"]) config_.storage_save_final_ikdtree = storage["save_final_ikdtree"].as<bool>();
        }

        // PCD save section
        if (root["pcd_save"])
        {
            auto pcd = root["pcd_save"];
            if (pcd["pcd_save_en"]) config_.pcd_save_en = pcd["pcd_save_en"].as<bool>();
            if (pcd["interval"]) config_.pcd_interval = pcd["interval"].as<int>();
        }

        if ((!root["storage"] || !root["storage"]["pcd_chunk_frames"]) &&
            config_.pcd_interval > 0)
        {
            config_.storage_pcd_chunk_frames = config_.pcd_interval;
            std::cout << "[YamlConfig] Migrated legacy pcd_save.interval="
                      << config_.pcd_interval
                      << " to storage.pcd_chunk_frames." << std::endl;
        }

        std::cout << "[YamlConfig] Loaded: " << filepath << std::endl;
        std::cout << "  lidar_type=" << config_.lidar_type
                  << " scan_line=" << config_.scan_line
                  << " blind=" << config_.blind
                  << " acc_cov=" << config_.acc_cov
                  << " gyr_cov=" << config_.gyr_cov << std::endl;
        std::cout << "  extrinsic_T: " << config_.extrinsic_T.transpose() << std::endl;
        return true;
    }
    catch (const std::exception& e)
    {
        std::cerr << "[YamlConfig] Error loading " << filepath << ": " << e.what() << std::endl;
        return false;
    }
}

void YamlConfig::applyOverrides(int argc, char* argv[])
{
    // Simple key=value override from command line
    for (int i = 1; i < argc; i++)
    {
        std::string arg = argv[i];
        size_t eq = arg.find('=');
        if (eq == std::string::npos) continue;
        
        std::string key = arg.substr(0, eq);
        std::string val = arg.substr(eq + 1);
        
        try
        {
            if (key == "blind") config_.blind = std::stod(val);
            else if (key == "point_filter_num") config_.point_filter_num = std::stoi(val);
            else if (key == "acc_cov") config_.acc_cov = std::stod(val);
            else if (key == "gyr_cov") config_.gyr_cov = std::stod(val);
            else if (key == "pcd_save_en") config_.pcd_save_en = (val == "true" || val == "1");
            else if (key == "pcd_interval") {
                config_.pcd_interval = std::stoi(val);
                config_.storage_pcd_chunk_frames =
                    std::max(0, config_.pcd_interval);
            }
            else if (key == "filter_size_map") config_.filter_size_map_min = std::stod(val);
            else if (key == "max_iter") config_.max_iteration = std::stoi(val);
            else if (key == "max_feature_points") config_.max_feature_points = std::stoi(val);
            else if (key == "iekf_match_threads") config_.iekf_match_threads = std::stoi(val);
            else if (key == "publish_full_map") config_.publish_full_map = (val == "true" || val == "1");
            else if (key == "async_full_map_publish") {
                config_.async_full_map_publish = (val == "true" || val == "1");
                config_.map_output_mode = config_.async_full_map_publish
                    ? (config_.publish_map_delta ? "hybrid" : "full_async")
                    : (config_.publish_map_delta ? "tiled_incremental" : "full_async");
            }
            else if (key == "full_map_publish_interval_ms") {
                config_.full_map_publish_interval_ms = std::stoi(val);
                config_.map_output_full_publish_interval_ms = config_.full_map_publish_interval_ms;
            }
            else if (key == "full_map_voxel_size") {
                config_.full_map_voxel_size = std::stod(val);
                config_.map_output_full_voxel_leaf_m = config_.full_map_voxel_size;
            }
            else if (key == "bag_full_map_periodic") config_.bag_full_map_periodic = (val == "true" || val == "1");
            else if (key == "publish_map_delta") {
                config_.publish_map_delta = (val == "true" || val == "1");
                config_.map_output_mode = config_.async_full_map_publish
                    ? (config_.publish_map_delta ? "hybrid" : "full_async")
                    : (config_.publish_map_delta ? "tiled_incremental" : "full_async");
            }
            else if (key == "map_delta_max_pending_points") {
                config_.map_delta_max_pending_points = std::stoi(val);
                config_.map_output_max_points_per_update = config_.map_delta_max_pending_points;
            }
            else if (key == "foxglove_control_interval_ms") config_.foxglove_control_interval_ms = std::stoi(val);
            else if (key == "foxglove_backlog_size") config_.foxglove_backlog_size = std::stoi(val);
            else if (key == "map_output.mode") config_.map_output_mode = val;
            else if (key == "map_output.full_publish_interval_ms") config_.map_output_full_publish_interval_ms = std::stoi(val);
            else if (key == "map_output.full_voxel_leaf_m") config_.map_output_full_voxel_leaf_m = std::stod(val);
            else if (key == "map_output.tile_size_m") config_.map_output_tile_size_m = std::stod(val);
            else if (key == "map_output.tile_voxel_leaf_m") config_.map_output_tile_voxel_leaf_m = std::stod(val);
            else if (key == "map_output.voxel_update_policy") config_.map_output_voxel_update_policy = val;
            else if (key == "map_output.tile_publish_hz") config_.map_output_tile_publish_hz = std::stoi(val);
            else if (key == "map_output.max_tiles_per_update") config_.map_output_max_tiles_per_update = std::stoi(val);
            else if (key == "map_output.max_points_per_update") config_.map_output_max_points_per_update = std::stoi(val);
            else if (key == "map_output.input_queue_capacity") config_.map_output_input_queue_capacity = std::stoi(val);
            else if (key == "map_output.input_queue_max_mb") config_.map_output_input_queue_max_mb = std::stoi(val);
            else if (key == "map_output.max_memory_mb") config_.map_output_max_memory_mb = std::stoi(val);
            else if (key == "map_output.memory_policy") config_.map_output_memory_policy = val;
            else if (key == "foxglove.current_cloud_publish_hz") config_.foxglove_current_cloud_publish_hz = std::stod(val);
            else if (key == "foxglove.path_publish_hz") config_.foxglove_path_publish_hz = std::stod(val);
            else if (key == "foxglove.backlog_size") config_.foxglove_backlog_size = std::stoi(val);
            else if (key == "storage.mode") config_.storage_mode = val;
            else if (key == "storage.queue_max_mb") config_.storage_queue_max_mb = std::stoi(val);
            else if (key == "storage.bag_path_publish_hz") config_.storage_bag_path_publish_hz = std::stod(val);
            else if (key == "storage.path_max_points") config_.storage_path_max_points = std::stoi(val);
            else if (key == "storage.pcd_format") config_.storage_pcd_format = val;
            else if (key == "storage.pcd_chunk_points") config_.storage_pcd_chunk_points = std::stoi(val);
            else if (key == "storage.pcd_chunk_frames") config_.storage_pcd_chunk_frames = std::stoi(val);
            else if (key == "storage.save_final_ikdtree") config_.storage_save_final_ikdtree = (val == "true" || val == "1");
            else if (key == "root_dir") config_.root_dir = val;
            else if (key == "livox_broadcast_code") config_.livox_broadcast_code = val;
            else if (key == "realtime_frame_sec") config_.realtime_frame_sec = std::stod(val);
            else if (key == "realtime_frame_points") config_.realtime_frame_points = std::stoi(val);
        }
        catch (...)
        {
            std::cerr << "[YamlConfig] Invalid override: " << arg << std::endl;
        }
    }
}

bool YamlConfig::validate(std::string& error) const
{
    const auto oneOf = [](const std::string& value,
                          std::initializer_list<const char*> allowed) {
        return std::any_of(
            allowed.begin(), allowed.end(),
            [&](const char* candidate) { return value == candidate; });
    };
    if (!oneOf(config_.map_output_mode,
               {"full_async", "tiled_incremental", "hybrid"}))
    {
        error = "map_output.mode must be full_async, tiled_incremental, or hybrid";
        return false;
    }
    if (!oneOf(config_.map_output_voxel_update_policy,
               {"first", "latest", "centroid"}))
    {
        error = "map_output.voxel_update_policy must be first, latest, or centroid";
        return false;
    }
    if (!oneOf(config_.map_output_memory_policy,
               {"stop_accumulating", "lru", "evict_far", "spill_to_disk"}))
    {
        error = "map_output.memory_policy must be stop_accumulating, lru, evict_far, or spill_to_disk";
        return false;
    }
    if (!oneOf(config_.storage_mode, {"realtime", "reliable"})) {
        error = "storage.mode must be realtime or reliable";
        return false;
    }
    if (!oneOf(config_.storage_pcd_format,
               {"binary", "binary_compressed"}))
    {
        error = "storage.pcd_format must be binary or binary_compressed";
        return false;
    }
    if (!(config_.map_output_tile_size_m > 0.0) ||
        !(config_.map_output_tile_voxel_leaf_m > 0.0) ||
        !std::isfinite(config_.map_output_tile_size_m) ||
        !std::isfinite(config_.map_output_tile_voxel_leaf_m))
    {
        error = "map_output tile and voxel sizes must be finite and positive";
        return false;
    }
    const double ratio = config_.map_output_tile_size_m /
        config_.map_output_tile_voxel_leaf_m;
    if (std::abs(ratio - std::round(ratio)) >
        1e-9 * std::max(1.0, std::abs(ratio)))
    {
        error = "map_output.tile_size_m must be an integer multiple of tile_voxel_leaf_m";
        return false;
    }
    if (config_.storage_pcd_chunk_frames < 0) {
        error = "storage.pcd_chunk_frames must be non-negative";
        return false;
    }
    return true;
}
