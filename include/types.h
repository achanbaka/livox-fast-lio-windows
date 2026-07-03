#ifndef TYPES_H
#define TYPES_H

#include <Eigen/Core>
#include <deque>
#include <memory>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

// ─── Basic Eigen typedefs ──────────────────────────────────────────
typedef Eigen::Vector3d V3D;
typedef Eigen::Matrix3d M3D;
typedef Eigen::Vector3f V3F;
typedef Eigen::Matrix3f M3F;

#define MD(a,b)  Eigen::Matrix<double, (a), (b)>
#define VD(a)    Eigen::Matrix<double, (a), 1>
#define MF(a,b)  Eigen::Matrix<float, (a), (b)>
#define VF(a)    Eigen::Matrix<float, (a), 1>

// ─── Point types ───────────────────────────────────────────────────
typedef pcl::PointXYZINormal PointType;
typedef pcl::PointCloud<PointType> PointCloudXYZI;
typedef std::vector<PointType, Eigen::aligned_allocator<PointType>> PointVector;

// ─── IMU data (replaces sensor_msgs::Imu) ─────────────────────────
struct ImuData
{
    double timestamp;        // seconds (absolute)
    V3D acc;                 // linear acceleration (m/s^2, unnormalized)
    V3D gyro;                // angular velocity (rad/s)

    ImuData() : timestamp(0.0), acc(0, 0, 0), gyro(0, 0, 0) {}
    ImuData(double t, const V3D &a, const V3D &g) : timestamp(t), acc(a), gyro(g) {}
};
typedef std::shared_ptr<ImuData> ImuDataPtr;
typedef std::shared_ptr<const ImuData> ImuDataConstPtr;

// ─── Pose6D (replaces fast_lio::Pose6D) ───────────────────────────
struct Pose6D
{
    double offset_time;
    double acc[3];
    double gyr[3];
    double vel[3];
    double pos[3];
    double rot[9];

    Pose6D() : offset_time(0.0)
    {
        memset(acc, 0, sizeof(acc));
        memset(gyr, 0, sizeof(gyr));
        memset(vel, 0, sizeof(vel));
        memset(pos, 0, sizeof(pos));
        memset(rot, 0, sizeof(rot));
    }
};

// ─── MeasureGroup (replaces ROS-based version) ─────────────────────
struct MeasureGroup
{
    MeasureGroup()
    {
        lidar_beg_time = 0.0;
        lidar_end_time = 0.0;
        lidar.reset(new PointCloudXYZI());
    }
    double lidar_beg_time;
    double lidar_end_time;
    PointCloudXYZI::Ptr lidar;
    std::deque<ImuDataConstPtr> imu;
};

#endif // TYPES_H
