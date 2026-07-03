#ifndef FAST_LIO_OBSERVATION_H
#define FAST_LIO_OBSERVATION_H

#include <cmath>

constexpr double kFastLioInitTime = 0.1;
constexpr double kFastLioLaserPointCovariance = 0.001;

inline bool fastLioPlaneResidualAccepted(double signed_distance, double point_norm)
{
    if (point_norm <= 0.0) {
        return false;
    }
    const double score = 1.0 - 0.9 * std::fabs(signed_distance) / std::sqrt(point_norm);
    return score > 0.9;
}

#endif // FAST_LIO_OBSERVATION_H
