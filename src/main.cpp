/**
 * Livox FAST-LIO Windows - Native Windows SLAM Application
 * 
 * Usage:
 *   livox_fast_lio [config_path] [--lvx <lvx_file>] [key=value overrides]
 * 
 * Examples:
 *   livox_fast_lio                                    # Use default config, connect to real LiDAR
 *   livox_fast_lio config/horizon.yaml                # Use specific config
 *   livox_fast_lio --lvx data/test.lvx                # Replay .lvx file
 *   livox_fast_lio config/horizon.yaml blind=2        # Override blind distance
 */

#include <iostream>
#include <string>
#include <filesystem>
#include "yaml_config.h"
#include "laser_mapping.h"

namespace fs = std::filesystem;

void printUsage(const char* prog)
{
    std::cout << "Livox FAST-LIO Windows SLAM\n"
              << "Usage: " << prog << " [config_path] [--lvx <file>] [--bag <file>] [key=value]\n"
              << "\nOptions:\n"
              << "  config_path       Path to YAML config (default: config/horizon.yaml)\n"
              << "  --lvx <file>      Replay .lvx file instead of real-time LiDAR\n"
              << "  --bag <file>      Read from ROS1 bag file and write results to bag\n"
              << "  key=value         Override config parameters (e.g., blind=2 acc_cov=0.5)\n"
              << "\nExamples:\n"
              << "  " << prog << "\n"
              << "  " << prog << " config/horizon.yaml --lvx data/test.lvx\n"
              << "  " << prog << " --bag data/test.bag\n"
              << "  " << prog << " blind=2 max_iter=15\n"
              << std::endl;
}

int main(int argc, char* argv[])
{
    std::cout << "========================================\n"
              << "  Livox FAST-LIO Windows SLAM v1.0\n"
              << "========================================\n" << std::endl;

    // Parse arguments
    std::string config_path;
    std::string lvx_path;
    std::string bag_path;
    bool use_lvx = false;
    bool use_bag = false;
    std::vector<std::string> overrides;

    // Determine root directory (where the executable is or ROOT_DIR)
    std::string root_dir = ROOT_DIR;
    
    for (int i = 1; i < argc; i++)
    {
        std::string arg = argv[i];
        
        if (arg == "--help" || arg == "-h")
        {
            printUsage(argv[0]);
            return 0;
        }
        else if (arg == "--lvx")
        {
            if (i + 1 < argc)
            {
                lvx_path = argv[++i];
                use_lvx = true;
            }
            else
            {
                std::cerr << "Error: --lvx requires a file path" << std::endl;
                return 1;
            }
        }
        else if (arg == "--bag")
        {
            if (i + 1 < argc)
            {
                bag_path = argv[++i];
                use_bag = true;
            }
            else
            {
                std::cerr << "Error: --bag requires a file path" << std::endl;
                return 1;
            }
        }
        else if (arg.find('=') != std::string::npos)
        {
            overrides.push_back(arg);
        }
        else
        {
            // Assume it's a config path
            config_path = arg;
        }
    }

    // Default config path
    if (config_path.empty())
    {
        // Try relative to executable and ROOT_DIR
        if (fs::exists("config/horizon.yaml"))
            config_path = "config/horizon.yaml";
        else if (fs::exists(root_dir + "config/horizon.yaml"))
            config_path = root_dir + "config/horizon.yaml";
        else
        {
            std::cerr << "Error: config/horizon.yaml not found." << std::endl;
            std::cerr << "Specify config path: " << argv[0] << " <config_path>" << std::endl;
            return 1;
        }
    }

    // Load config
    YamlConfig yaml_config;
    if (!yaml_config.load(config_path))
    {
        std::cerr << "Error: Failed to load config from " << config_path << std::endl;
        return 1;
    }

    // Apply overrides
    yaml_config.applyOverrides(argc, argv);
    yaml_config.getConfig().root_dir = root_dir;

    // Validate input paths
    if (use_lvx)
    {
        if (!fs::exists(lvx_path))
        {
            std::cerr << "Error: .lvx file not found: " << lvx_path << std::endl;
            return 1;
        }
        std::cout << "Mode: Lvx Replay (" << lvx_path << ")" << std::endl;
    }
    else if (use_bag)
    {
        if (!fs::exists(bag_path))
        {
            std::cerr << "Error: .bag file not found: " << bag_path << std::endl;
            return 1;
        }
        std::cout << "Mode: Bag Replay (" << bag_path << ")" << std::endl;
    }
    else
    {
        std::cout << "Mode: Real-time LiDAR (Livox Horizon)" << std::endl;
    }

    // Create output directories
    fs::create_directories(root_dir + "Log");
    fs::create_directories(root_dir + "PCD");

    // Run SLAM
    runLaserMapping(yaml_config.getConfig(), use_lvx, lvx_path, use_bag, bag_path);

    std::cout << "\nExiting." << std::endl;
    return 0;
}
