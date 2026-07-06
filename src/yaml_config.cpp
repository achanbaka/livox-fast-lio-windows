#include "yaml_config.h"
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
    config_.pcd_save_en = true;
    config_.pcd_interval = -1;
    config_.max_iteration = 3;
    config_.max_feature_points = 2000;
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
        }

        // PCD save section
        if (root["pcd_save"])
        {
            auto pcd = root["pcd_save"];
            if (pcd["pcd_save_en"]) config_.pcd_save_en = pcd["pcd_save_en"].as<bool>();
            if (pcd["interval"]) config_.pcd_interval = pcd["interval"].as<int>();
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
            else if (key == "acc_cov") config_.acc_cov = std::stod(val);
            else if (key == "gyr_cov") config_.gyr_cov = std::stod(val);
            else if (key == "pcd_save_en") config_.pcd_save_en = (val == "true" || val == "1");
            else if (key == "filter_size_map") config_.filter_size_map_min = std::stod(val);
            else if (key == "max_iter") config_.max_iteration = std::stoi(val);
            else if (key == "max_feature_points") config_.max_feature_points = std::stoi(val);
            else if (key == "publish_full_map") config_.publish_full_map = (val == "true" || val == "1");
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
