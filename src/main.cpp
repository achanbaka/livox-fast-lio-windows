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

static std::string resolveOutputRoot(const std::string& configured_root)
{
    if (configured_root.empty() || configured_root == "." ||
        configured_root == "./" || configured_root == ".\\")
    {
        return fs::current_path().lexically_normal().string();
    }

    return fs::absolute(fs::path(configured_root)).lexically_normal().string();
}

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

    // ROOT_DIR is the source tree. Runtime output defaults to the current
    // working directory so ./Log and ./PCD match where the app is launched.
    std::string source_root_dir = ROOT_DIR;
    
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
        else if (fs::exists(fs::path(source_root_dir) / "config/horizon.yaml"))
            config_path = (fs::path(source_root_dir) / "config/horizon.yaml").string();
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
    std::string config_error;
    if (!yaml_config.validate(config_error)) {
        std::cerr << "Error: Invalid configuration: " << config_error << std::endl;
        return 1;
    }
    const std::string output_root = resolveOutputRoot(yaml_config.getConfig().root_dir);
    yaml_config.getConfig().root_dir = output_root;

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
    fs::create_directories(fs::path(output_root) / "Log");
    fs::create_directories(fs::path(output_root) / "PCD");
    std::cout << "Output root: " << output_root << std::endl;
    std::cout << "Log directory: " << (fs::path(output_root) / "Log").string() << std::endl;

    // Run SLAM
    runLaserMapping(yaml_config.getConfig(), use_lvx, lvx_path, use_bag, bag_path);

    std::cout << "\nExiting." << std::endl;
    return 0;
}
