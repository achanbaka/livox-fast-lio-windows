#ifndef LIDAR_IMU_SYNC_H
#define LIDAR_IMU_SYNC_H

inline bool hasImuCoverageForLidarFrame(double last_imu_timestamp, double lidar_end_time)
{
    return last_imu_timestamp >= lidar_end_time;
}

#endif // LIDAR_IMU_SYNC_H
