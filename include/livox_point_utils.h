#ifndef LIVOX_POINT_UTILS_H
#define LIVOX_POINT_UTILS_H

#include "lvx_reader.h"
#include "types.h"

inline PointType livoxToPointType(const LvxPoint& lp) {
    PointType pt;
    pt.x = lp.x;
    pt.y = lp.y;
    pt.z = lp.z;
    pt.intensity = static_cast<float>(lp.reflectivity);
    pt.curvature = static_cast<float>(lp.offset_time) / 1e6f;
    pt.normal_x = static_cast<float>(lp.tag);
    pt.normal_y = static_cast<float>(lp.line);
    pt.normal_z = 0.0f;
    return pt;
}

#endif
