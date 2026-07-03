#ifndef LASER_MAPPING_H
#define LASER_MAPPING_H

#include "yaml_config.h"
#include <string>

void runLaserMapping(FastLioConfig& config,
                     bool use_lvx = false, const std::string& lvx_path = "",
                     bool use_bag = false, const std::string& bag_path = "");

#endif
