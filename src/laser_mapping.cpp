/**
 * laser_mapping.cpp – FAST-LIO2 core SLAM loop, ported to Windows without ROS.
 *
 * Implements the main iterated-EKF LiDAR-inertial odometry pipeline:
 *   1. IMU propagation + LiDAR undistortion
 *   2. IEKF observation update (point-to-plane)
 *   3. Incremental ikd-Tree map management with FOV splitting
 *   4. Foxglove WebSocket publishing (odometry, cloud, path, TF)
 */

#include "laser_mapping.h"
#include "IMU_Processing.hpp"
#include "preprocess.h"
#include "livox_adapter.h"
#include "lvx_reader.h"
#include "foxglove_publisher.h"
#include "fast_lio_observation.h"
#include "lidar_imu_sync.h"
#include "map_accumulator.h"
#include "bag_reader.h"
#include "bag_writer.h"
#include "ikd-Tree/ikd_Tree.h"
#include "use-ikfom.hpp"
#include "common_lib.h"

#include <windows.h>

#include <atomic>
#include <chrono>
#include <algorithm>
#include <cmath>
#include <cstring>
#include <condition_variable>
#include <csignal>
#include <deque>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <mutex>
#include <numeric>
#include <sstream>
#include <string>
#include <thread>
#include <unordered_map>
#include <vector>

#include <pcl/io/pcd_io.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>

// ═══════════════════════════════════════════════════════════════════════
//  Constants & using
// ═══════════════════════════════════════════════════════════════════════
using namespace std;
using namespace Eigen;

// ═══════════════════════════════════════════════════════════════════════
//  Global state
// ═══════════════════════════════════════════════════════════════════════

// --- EKF filter ---
esekfom::esekf<state_ikfom, 12, input_ikfom> kf;

// --- ikd-Tree map ---
KD_TREE<pcl::PointXYZINormal> ikdtree;

// --- Preprocessor ---
Preprocess p_pre;

// --- IMU processor ---
ImuProcess imu_proc;

// --- Current measure group ---
MeasureGroup Measures;

// --- State helpers ---
state_ikfom state_point;
vect3 pos_lid;
Eigen::Quaterniond geoQuat;
V3D euler_cur;

// --- Timing ---
double first_lidar_time = 0.0;

// --- Path for visualization ---
vector<V3D> path_vec;

// --- Exit flag (set by Ctrl-C handler) ---
atomic<bool> g_exit_flag{false};

// --- Map management helpers ---
vector<BoxPointType> cub_needrm;

// --- Config cache (set in runLaserMapping) ---
static double filter_size_surf_min = 0.5;
static double filter_size_map_min    = 0.5;
static double DET_RANGE              = 300.0;
static double cube_len               = 600.0;
static double fov_deg                = 180.0;
static int    max_iteration          = 4;
static int    max_feature_points     = 2000;
static bool   extrinsic_est_en       = true;
static bool   pcd_save_en            = false;
static int    pcd_interval           = 1;
static bool   runtime_pos_log        = false;
static bool   scan_publish_en        = true;
static bool   dense_publish_en       = true;
static bool   path_en                = true;
static bool   scan_bodyframe_pub_en  = false;
static bool   publish_full_map       = true;
static string root_dir;
static ofstream runtime_log;
static constexpr int MAP_PUBLISH_INTERVAL = 1;
static constexpr int FULL_MAP_PUBLISH_INTERVAL = 50;
static constexpr size_t MAX_REPLAY_LIDAR_QUEUE = 3;
static constexpr int FOXGLOVE_CLOUD_INTERVAL_MS = 100;
static constexpr int FOXGLOVE_MAP_INTERVAL_MS = 1000;
static constexpr int FOXGLOVE_PATH_INTERVAL_MS = 200;
static constexpr int VERBOSE_SLAM_LOG_INTERVAL = 20;
static atomic<bool> replay_backpressure_enabled{false};
static atomic<bool> replay_backpressure_stop{false};

static string resolveOutputRoot(const string& configured_root)
{
    namespace fs = std::filesystem;
    if (configured_root.empty() || configured_root == "." ||
        configured_root == "./" || configured_root == ".\\")
    {
        return fs::current_path().lexically_normal().string();
    }

    return fs::absolute(fs::path(configured_root)).lexically_normal().string();
}

static string outputPath(const string& relative_path)
{
    return (std::filesystem::path(root_dir) / relative_path).string();
}

static void writeRuntimeLog(const string& line)
{
    if (runtime_log.is_open())
    {
        runtime_log << line << '\n';
        runtime_log.flush();
    }
}

// ═══════════════════════════════════════════════════════════════════════
//  Data buffers – filled from callbacks, consumed by sync_packages
// ═══════════════════════════════════════════════════════════════════════
static deque<PointCloudXYZI::Ptr>  lidar_buffer;
static deque<double>               time_buffer;
static deque<ImuDataConstPtr>      imu_buffer;

static mutex mtx_buffer;
static condition_variable sig_buffer;
static double last_timestamp_imu = -1.0;

// ═══════════════════════════════════════════════════════════════════════
//  Point cloud for map publishing
// ═══════════════════════════════════════════════════════════════════════
static PointCloudXYZI::Ptr featsFromMap(new PointCloudXYZI());
static PointCloudXYZI::Ptr feats_undistort(new PointCloudXYZI());
static PointCloudXYZI::Ptr feats_down_body(new PointCloudXYZI());
static PointCloudXYZI::Ptr feats_down_world(new PointCloudXYZI());
static PointCloudXYZI::Ptr normvec(new PointCloudXYZI());
static PointCloudXYZI::Ptr laserCloudOri(new PointCloudXYZI());
static PointCloudXYZI::Ptr corr_normvect(new PointCloudXYZI());
static vector<PointVector> Nearest_Points;
static vector<uint8_t> point_selected_surf;
static vector<double> res_last;
static MapAccumulator full_map_accumulator;
static int effct_feat_num = 0;
static int feats_down_size = 0;
static bool flg_EKF_inited = false;
static double res_mean_last = 0.05;
static double total_residual = 0.0;

// ═══════════════════════════════════════════════════════════════════════
//  Signal handler (Windows Ctrl-C / Close)
// ═══════════════════════════════════════════════════════════════════════
static BOOL WINAPI console_ctrl_handler(DWORD dwCtrlType)
{
    switch (dwCtrlType)
    {
    case CTRL_C_EVENT:
    case CTRL_BREAK_EVENT:
    case CTRL_CLOSE_EVENT:
    case CTRL_LOGOFF_EVENT:
    case CTRL_SHUTDOWN_EVENT:
        g_exit_flag.store(true);
        return TRUE;
    default:
        return FALSE;
    }
}

// ═══════════════════════════════════════════════════════════════════════
//  Transform point from body (LiDAR) frame to world frame
// ═══════════════════════════════════════════════════════════════════════
void pointBodyToWorld(PointType const *const pi, PointType *const po)
{
    V3D p_body(pi->x, pi->y, pi->z);
    V3D p_global(state_point.rot * (state_point.offset_R_L_I * p_body + state_point.offset_T_L_I) + state_point.pos);

    po->x = p_global(0);
    po->y = p_global(1);
    po->z = p_global(2);
    po->intensity         = pi->intensity;
    po->curvature         = pi->curvature;
    po->normal_x          = pi->normal_x;
    po->normal_y          = pi->normal_y;
    po->normal_z          = pi->normal_z;
}

static void pointBodyToWorld(const V3D &p_body, V3D &p_global)
{
    p_global = state_point.rot * (state_point.offset_R_L_I * p_body + state_point.offset_T_L_I) + state_point.pos;
}

struct VoxelKey {
    int64_t x = 0;
    int64_t y = 0;
    int64_t z = 0;

    bool operator==(const VoxelKey& other) const
    {
        return x == other.x && y == other.y && z == other.z;
    }
};

struct VoxelKeyHash {
    size_t operator()(const VoxelKey& key) const
    {
        uint64_t h = 1469598103934665603ULL;
        mix(h, static_cast<uint64_t>(key.x));
        mix(h, static_cast<uint64_t>(key.y));
        mix(h, static_cast<uint64_t>(key.z));
        return static_cast<size_t>(h);
    }

    static void mix(uint64_t& hash, uint64_t value)
    {
        hash ^= value;
        hash *= 1099511628211ULL;
    }
};

static bool isUsablePoint(const PointType& point)
{
    constexpr float kMaxAbsCoordinate = 1000000.0f;
    return std::isfinite(point.x) && std::isfinite(point.y) && std::isfinite(point.z) &&
           std::abs(point.x) <= kMaxAbsCoordinate &&
           std::abs(point.y) <= kMaxAbsCoordinate &&
           std::abs(point.z) <= kMaxAbsCoordinate;
}

static void voxelDownsampleSafe(const PointCloudXYZI::Ptr& input,
                                PointCloudXYZI::Ptr& output,
                                double leaf_size)
{
    output->clear();
    if (!input || input->empty()) {
        return;
    }

    if (leaf_size <= 0.0) {
        output->points.reserve(input->points.size());
        for (const auto& point : input->points) {
            if (isUsablePoint(point)) {
                output->points.push_back(point);
            }
        }
    } else {
        const double inv_leaf = 1.0 / leaf_size;
        std::unordered_map<VoxelKey, PointType, VoxelKeyHash> voxels;
        voxels.reserve(input->points.size());
        for (const auto& point : input->points) {
            if (!isUsablePoint(point)) {
                continue;
            }
            const VoxelKey key{
                static_cast<int64_t>(std::floor(static_cast<double>(point.x) * inv_leaf)),
                static_cast<int64_t>(std::floor(static_cast<double>(point.y) * inv_leaf)),
                static_cast<int64_t>(std::floor(static_cast<double>(point.z) * inv_leaf)),
            };
            voxels.try_emplace(key, point);
        }

        output->points.reserve(voxels.size());
        for (const auto& entry : voxels) {
            output->points.push_back(entry.second);
        }
    }

    output->width = static_cast<uint32_t>(output->points.size());
    output->height = 1;
    output->is_dense = true;
}

// ═══════════════════════════════════════════════════════════════════════
//  Data callbacks – push into global buffers
// ═══════════════════════════════════════════════════════════════════════

/**
 * Called when a LiDAR frame is ready (from LvxReader or LivoxAdapter).
 * Points have already been preprocessed into PointCloudXYZI.
 */
static void pushLidarFrame(PointCloudXYZI::Ptr cloud, double timestamp)
{
    if (!cloud || cloud->empty()) return;
    unique_lock<mutex> lock(mtx_buffer);
    if (replay_backpressure_enabled.load())
    {
        sig_buffer.wait(lock, [] {
            return replay_backpressure_stop.load() ||
                   g_exit_flag.load() ||
                   lidar_buffer.size() < MAX_REPLAY_LIDAR_QUEUE;
        });
        if (replay_backpressure_stop.load() || g_exit_flag.load()) {
            return;
        }
    }
    lidar_buffer.push_back(cloud);
    time_buffer.push_back(timestamp);
    sig_buffer.notify_one();
}

/**
 * Called for every IMU sample (from LvxReader or LivoxAdapter).
 */
static void pushImuSample(const ImuData &imu)
{
    ImuDataPtr p(new ImuData(imu));
    {
        lock_guard<mutex> lock(mtx_buffer);
        if (last_timestamp_imu > 0.0 && imu.timestamp < last_timestamp_imu)
        {
            imu_buffer.clear();
        }
        last_timestamp_imu = imu.timestamp;
        imu_buffer.push_back(ImuDataConstPtr(p));
    }
    sig_buffer.notify_one();
}

static PointCloudXYZI::Ptr preprocessLivoxPoints(const vector<LvxPoint>& points, double timestamp)
{
    PointCloudXYZI::Ptr cloud(new PointCloudXYZI());
    if (points.empty()) {
        return cloud;
    }

    vector<float> x(points.size()), y(points.size()), z(points.size());
    vector<uint8_t> line(points.size()), tag(points.size()), reflectivity(points.size());
    vector<uint32_t> offset_time(points.size());

    for (size_t i = 0; i < points.size(); ++i)
    {
        x[i] = points[i].x;
        y[i] = points[i].y;
        z[i] = points[i].z;
        line[i] = points[i].line;
        tag[i] = points[i].tag;
        reflectivity[i] = points[i].reflectivity;
        offset_time[i] = points[i].offset_time;
    }

    p_pre.process_points(x.data(), y.data(), z.data(),
                         line.data(), tag.data(), offset_time.data(),
                         reflectivity.data(),
                         static_cast<uint32_t>(points.size()), timestamp, cloud);
    return cloud;
}

// ═══════════════════════════════════════════════════════════════════════
//  sync_packages – group LiDAR frames with matching IMU data
// ═══════════════════════════════════════════════════════════════════════

/**
 * Collect one LiDAR frame and all IMU data that overlaps with it.
 * Returns false when no data is available.
 */
bool sync_packages(MeasureGroup &meas)
{
    meas.lidar->clear();
    meas.imu.clear();

    PointCloudXYZI::Ptr lidar_frame;
    double lidar_time = 0.0;
    double lidar_end_time = 0.0;

    {
        unique_lock<mutex> lock(mtx_buffer);
        if (lidar_buffer.empty() || imu_buffer.empty())
        {
            sig_buffer.wait_for(lock, chrono::milliseconds(500),
                                []{ return !lidar_buffer.empty() && !imu_buffer.empty(); });
            if (lidar_buffer.empty() || imu_buffer.empty()) return false;
        }

        lidar_frame = lidar_buffer.front();
        lidar_time  = time_buffer.front();

        if (!lidar_frame || lidar_frame->points.empty())
        {
            lidar_buffer.pop_front();
            time_buffer.pop_front();
            sig_buffer.notify_all();
            return false;
        }

        double max_offset_ms = 0.0;
        for (const auto& point : lidar_frame->points)
        {
            if (point.curvature > max_offset_ms)
            {
                max_offset_ms = point.curvature;
            }
        }
        lidar_end_time = lidar_time + max_offset_ms / 1000.0;

        if (!hasImuCoverageForLidarFrame(last_timestamp_imu, lidar_end_time))
        {
            return false;
        }

        lidar_buffer.pop_front();
        time_buffer.pop_front();
        sig_buffer.notify_all();

        while (!imu_buffer.empty())
        {
            const double imu_time = imu_buffer.front()->timestamp;
            if (imu_time > lidar_end_time) break;
            meas.imu.push_back(imu_buffer.front());
            imu_buffer.pop_front();
        }
    }

    // Copy point cloud data
    *(meas.lidar) = *lidar_frame;
    meas.lidar_beg_time = lidar_time;
    meas.lidar_end_time = lidar_end_time;

    if (meas.imu.empty()) return false;

    // Points are sorted by curvature (offset time in ms)
    sort(meas.lidar->points.begin(), meas.lidar->points.end(), time_list);

    return true;
}

// ═══════════════════════════════════════════════════════════════════════
//  IEKF helper (separate function so we can use SEH __try)
// ═══════════════════════════════════════════════════════════════════════
static void iekf_update_wrapper(double &solve_time)
{
    __try {
        kf.update_iterated_dyn_share_modified(kFastLioLaserPointCovariance, solve_time);
    } __except (EXCEPTION_EXECUTE_HANDLER) {
        cout << "[SLAM] IEKF SEH EXCEPTION (access violation)" << flush << endl;
    }
}

// ═══════════════════════════════════════════════════════════════════════
//  h_share_model – IEKF observation model (point-to-plane residual)
//
//  For each downsampled feature point:
//    1. Transform to world frame
//    2. Search 5 nearest neighbours in ikd-Tree
//    3. Fit a plane; compute residual and Jacobian H
// ═══════════════════════════════════════════════════════════════════════
void h_share_model_legacy(state_ikfom &state, esekfom::dyn_share_datastruct<double> &ekfom_data)
{
    int feats_size = featsFromMap->points.size();
    if (feats_size < 1) return;

    ekfom_data.h_x = MatrixXd::Zero(feats_size, state_ikfom::DOF);
    ekfom_data.h_v = MatrixXd::Identity(feats_size, feats_size);
    ekfom_data.z.resize(feats_size);
    ekfom_data.h = Eigen::VectorXd::Zero(feats_size);
    ekfom_data.R = 0.0001 * MatrixXd::Identity(feats_size, feats_size);
    ekfom_data.valid = true;

    M3D R_wl  = state.rot.toRotationMatrix();
    M3D R_offset = state.offset_R_L_I.toRotationMatrix();
    V3D T_offset = state.offset_T_L_I;
    V3D t_wl  = state.pos;

    V3D x_world = state.pos;

    for (int i = 0; i < feats_size; i++)
    {
        // --- point in LiDAR frame ---
        PointType &point_body = featsFromMap->points[i];
        V3D P_lidar(point_body.x, point_body.y, point_body.z);

        // --- point in IMU (body) frame ---
        V3D P_imu = R_offset * P_lidar + T_offset;

        // --- point in world frame ---
        V3D P_world = R_wl * P_imu + t_wl;

        // --- nearest search in ikd-Tree ---
        PointType point_world_search;
        point_world_search.x = P_world(0);
        point_world_search.y = P_world(1);
        point_world_search.z = P_world(2);

        PointVector nearest_points;
        vector<float> pointSearchSqDis(NUM_MATCH_POINTS);
        ikdtree.Nearest_Search(point_world_search, NUM_MATCH_POINTS,
                               nearest_points, pointSearchSqDis, 5.0);

        // Need exactly NUM_MATCH_POINTS neighbours within threshold
        if (nearest_points.size() < NUM_MATCH_POINTS) continue;
        if (pointSearchSqDis[NUM_MATCH_POINTS - 1] > 5.0f) continue;

        // --- fit plane to nearest neighbours ---
        Matrix<double, 4, 1> pabcd;
        if (!esti_plane(pabcd, nearest_points, 0.1)) continue;

        // --- residual: signed distance to plane ---
        double pd2 = pabcd(0) * P_world(0) + pabcd(1) * P_world(1)
                   + pabcd(2) * P_world(2) + pabcd(3);

        // Gate: skip if too far from plane (adaptive threshold)
        double s = 1.0 - 0.9 * fabs(pd2) / sqrt(P_imu.norm());
        if (s < 0.96) continue;

        // --- Jacobians ---
        V3D normal(pabcd(0), pabcd(1), pabcd(2));
        V3D P_body_to_world = P_world - t_wl;   // R_wl * P_imu

        // State ordering: pos(0:2), rot(3:5), offset_R_L_I(6:8), offset_T_L_I(9:11), vel(12:14), bg(15:17), ba(18:20), grav(21:22)
        // ∂r / ∂(δt)  =  n^T  →  state indices 0-2
        ekfom_data.h_x(i, 0) = normal(0);
        ekfom_data.h_x(i, 1) = normal(1);
        ekfom_data.h_x(i, 2) = normal(2);

        // ∂r / ∂(δR_IMU)  =  -n^T * [P_world - t]_×  →  state indices 3-5
        ekfom_data.h_x(i, 3) = normal(1) * P_body_to_world(2) - normal(2) * P_body_to_world(1);
        ekfom_data.h_x(i, 4) = normal(2) * P_body_to_world(0) - normal(0) * P_body_to_world(2);
        ekfom_data.h_x(i, 5) = normal(0) * P_body_to_world(1) - normal(1) * P_body_to_world(0);

        if (extrinsic_est_en)
        {
            V3D P_L = P_lidar;
            V3D R_I_P_L = R_wl * R_offset * P_L;

            // ∂r / ∂(δR_L_I)  =  -n^T * [R_wl * R_L_I * P_L]_×
            ekfom_data.h_x(i, 6) = normal(1) * R_I_P_L(2) - normal(2) * R_I_P_L(1);
            ekfom_data.h_x(i, 7) = normal(2) * R_I_P_L(0) - normal(0) * R_I_P_L(2);
            ekfom_data.h_x(i, 8) = normal(0) * R_I_P_L(1) - normal(1) * R_I_P_L(0);

            // ∂r / ∂(δT_L_I)  =  n^T * R_wl
            ekfom_data.h_x(i, 9)  = normal(0) * R_wl(0, 0) + normal(1) * R_wl(1, 0) + normal(2) * R_wl(2, 0);
            ekfom_data.h_x(i, 10) = normal(0) * R_wl(0, 1) + normal(1) * R_wl(1, 1) + normal(2) * R_wl(2, 1);
            ekfom_data.h_x(i, 11) = normal(0) * R_wl(0, 2) + normal(1) * R_wl(1, 2) + normal(2) * R_wl(2, 2);
        }

        // h_v = I (noise Jacobian, used in K = P*H^T*(H*P*H^T + h_v*R*h_v^T)^{-1})

        // Measurement residual
        ekfom_data.z(i) = -pd2;
        ekfom_data.valid = true;
    }
}

// ═══════════════════════════════════════════════════════════════════════
//  map_incremental – add new points to ikd-Tree, delete FOV boxes
// ═══════════════════════════════════════════════════════════════════════
void h_share_model(state_ikfom &state, esekfom::dyn_share_datastruct<double> &ekfom_data)
{
    if (feats_down_size < 1) {
        ekfom_data.valid = false;
        return;
    }

    laserCloudOri->clear();
    corr_normvect->clear();
    total_residual = 0.0;

    for (int i = 0; i < feats_down_size; i++)
    {
        PointType &point_body = feats_down_body->points[i];
        PointType &point_world = feats_down_world->points[i];

        V3D p_body(point_body.x, point_body.y, point_body.z);
        V3D p_global(state.rot * (state.offset_R_L_I * p_body + state.offset_T_L_I) + state.pos);
        point_world.x = p_global(0);
        point_world.y = p_global(1);
        point_world.z = p_global(2);
        point_world.intensity = point_body.intensity;

        vector<float> pointSearchSqDis(NUM_MATCH_POINTS);
        auto &points_near = Nearest_Points[i];

        if (ekfom_data.converge)
        {
            ikdtree.Nearest_Search(point_world, NUM_MATCH_POINTS, points_near, pointSearchSqDis);
            point_selected_surf[i] =
                (points_near.size() >= NUM_MATCH_POINTS &&
                 pointSearchSqDis[NUM_MATCH_POINTS - 1] <= 5.0f) ? 1 : 0;
        }

        if (!point_selected_surf[i]) continue;

        Matrix<double, 4, 1> pabcd;
        point_selected_surf[i] = 0;
        if (esti_plane(pabcd, points_near, 0.1))
        {
            const double pd2 = pabcd(0) * point_world.x +
                               pabcd(1) * point_world.y +
                               pabcd(2) * point_world.z +
                               pabcd(3);

            if (fastLioPlaneResidualAccepted(pd2, p_body.norm()))
            {
                point_selected_surf[i] = 1;
                normvec->points[i].x = static_cast<float>(pabcd(0));
                normvec->points[i].y = static_cast<float>(pabcd(1));
                normvec->points[i].z = static_cast<float>(pabcd(2));
                normvec->points[i].intensity = static_cast<float>(pd2);
                res_last[i] = std::fabs(pd2);
            }
        }
    }

    effct_feat_num = 0;
    laserCloudOri->points.resize(feats_down_size);
    corr_normvect->points.resize(feats_down_size);

    for (int i = 0; i < feats_down_size; i++)
    {
        if (point_selected_surf[i])
        {
            laserCloudOri->points[effct_feat_num] = feats_down_body->points[i];
            corr_normvect->points[effct_feat_num] = normvec->points[i];
            total_residual += res_last[i];
            effct_feat_num++;
        }
    }

    if (effct_feat_num < 1)
    {
        ekfom_data.valid = false;
        static int no_effective_log_count = 0;
        no_effective_log_count++;
        if (runtime_pos_log &&
            no_effective_log_count % VERBOSE_SLAM_LOG_INTERVAL == 0) {
            cout << "[SLAM] No effective points for IEKF update" << '\n';
        }
        return;
    }

    res_mean_last = total_residual / effct_feat_num;
    ekfom_data.h_x = MatrixXd::Zero(effct_feat_num, 12);
    ekfom_data.h.resize(effct_feat_num);

    for (int i = 0; i < effct_feat_num; i++)
    {
        const PointType &laser_p = laserCloudOri->points[i];
        V3D point_this_be(laser_p.x, laser_p.y, laser_p.z);
        M3D point_be_crossmat;
        point_be_crossmat << SKEW_SYM_MATRX(point_this_be);
        V3D point_this = state.offset_R_L_I * point_this_be + state.offset_T_L_I;
        M3D point_crossmat;
        point_crossmat << SKEW_SYM_MATRX(point_this);

        const PointType &norm_p = corr_normvect->points[i];
        V3D norm_vec(norm_p.x, norm_p.y, norm_p.z);

        V3D C(state.rot.conjugate() * norm_vec);
        V3D A(point_crossmat * C);
        if (extrinsic_est_en)
        {
            V3D B(point_be_crossmat * state.offset_R_L_I.conjugate() * C);
            ekfom_data.h_x.block<1, 12>(i, 0)
                << norm_p.x, norm_p.y, norm_p.z,
                   VEC_FROM_ARRAY(A),
                   VEC_FROM_ARRAY(B),
                   VEC_FROM_ARRAY(C);
        }
        else
        {
            ekfom_data.h_x.block<1, 12>(i, 0)
                << norm_p.x, norm_p.y, norm_p.z,
                   VEC_FROM_ARRAY(A),
                   0.0, 0.0, 0.0,
                   0.0, 0.0, 0.0;
        }

        ekfom_data.h(i) = -norm_p.intensity;
    }
}

void map_incremental()
{
    PointVector pointsToAdd;
    PointCloudXYZI &cloud = *feats_down_body;
    int size = feats_down_size;

    // Transform each downsampled point to world frame and collect for insertion
    PointVector point_world(size);
    for (int i = 0; i < size; i++)
    {
        pointBodyToWorld(&(cloud.points[i]), &(point_world[i]));
    }

    // Add points to ikd-Tree
    pointsToAdd = point_world;
    if (ikdtree.size() == 0)
    {
        // First frame: Build() initialises Root_Node; Add_Points() would null-deref
        ikdtree.Build(pointsToAdd);
    }
    else
    {
        ikdtree.Add_Points(pointsToAdd, true);
    }

    // ─── FOV-based cube deletion ───
    // Determine cube centred on current LiDAR position in world frame
    V3D lidar_pos_w = state_point.pos + state_point.rot.toRotationMatrix() * state_point.offset_T_L_I;

    float cube_min[3], cube_max[3];
    float det = static_cast<float>(DET_RANGE);
    float half_len = static_cast<float>(cube_len) / 2.0f;

    // If FOV < 360 deg, use a directional cube in front of the sensor
    if (fov_deg < 360.0)
    {
        // Build a cube that covers the FOV cone around current position
        for (int k = 0; k < 3; k++)
        {
            cube_min[k] = lidar_pos_w(k) - det;
            cube_max[k] = lidar_pos_w(k) + det;
        }
    }
    else
    {
        for (int k = 0; k < 3; k++)
        {
            cube_min[k] = lidar_pos_w(k) - half_len;
            cube_max[k] = lidar_pos_w(k) + half_len;
        }
    }

    // Get ikd-Tree bounding box
    BoxPointType tree_range = ikdtree.tree_range();

    // For each axis, if the tree extends beyond the cube, create deletion boxes
    cub_needrm.clear();

    // Check if tree is much larger than cube → need to trim
    bool need_trim = false;
    for (int k = 0; k < 3; k++)
    {
        if (tree_range.vertex_min[k] < cube_min[k] - half_len * 0.1f ||
            tree_range.vertex_max[k] > cube_max[k] + half_len * 0.1f)
        {
            need_trim = true;
            break;
        }
    }

    if (need_trim && ikdtree.size() > 500)
    {
        // Create deletion boxes for regions outside the cube
        // Six potential boxes: ±x, ±y, ±z
        float min_xyz[3], max_xyz[3];
        min_xyz[0] = tree_range.vertex_min[0];
        min_xyz[1] = tree_range.vertex_min[1];
        min_xyz[2] = tree_range.vertex_min[2];
        max_xyz[0] = tree_range.vertex_max[0];
        max_xyz[1] = tree_range.vertex_max[1];
        max_xyz[2] = tree_range.vertex_max[2];

        // -x side
        if (min_xyz[0] < cube_min[0])
        {
            BoxPointType box;
            memcpy(box.vertex_min, min_xyz, 3 * sizeof(float));
            memcpy(box.vertex_max, max_xyz, 3 * sizeof(float));
            box.vertex_max[0] = cube_min[0];
            cub_needrm.push_back(box);
        }
        // +x side
        if (max_xyz[0] > cube_max[0])
        {
            BoxPointType box;
            memcpy(box.vertex_min, min_xyz, 3 * sizeof(float));
            memcpy(box.vertex_max, max_xyz, 3 * sizeof(float));
            box.vertex_min[0] = cube_max[0];
            cub_needrm.push_back(box);
        }
        // -y side
        if (min_xyz[1] < cube_min[1])
        {
            BoxPointType box;
            box.vertex_min[0] = max(min_xyz[0], cube_min[0]);
            box.vertex_min[1] = min_xyz[1];
            box.vertex_min[2] = min_xyz[2];
            box.vertex_max[0] = min(max_xyz[0], cube_max[0]);
            box.vertex_max[1] = cube_min[1];
            box.vertex_max[2] = max_xyz[2];
            cub_needrm.push_back(box);
        }
        // +y side
        if (max_xyz[1] > cube_max[1])
        {
            BoxPointType box;
            box.vertex_min[0] = max(min_xyz[0], cube_min[0]);
            box.vertex_min[1] = cube_max[1];
            box.vertex_min[2] = min_xyz[2];
            box.vertex_max[0] = min(max_xyz[0], cube_max[0]);
            box.vertex_max[1] = max_xyz[1];
            box.vertex_max[2] = max_xyz[2];
            cub_needrm.push_back(box);
        }
        // -z side
        if (min_xyz[2] < cube_min[2])
        {
            BoxPointType box;
            box.vertex_min[0] = max(min_xyz[0], cube_min[0]);
            box.vertex_min[1] = max(min_xyz[1], cube_min[1]);
            box.vertex_min[2] = min_xyz[2];
            box.vertex_max[0] = min(max_xyz[0], cube_max[0]);
            box.vertex_max[1] = min(max_xyz[1], cube_max[1]);
            box.vertex_max[2] = cube_min[2];
            cub_needrm.push_back(box);
        }
        // +z side
        if (max_xyz[2] > cube_max[2])
        {
            BoxPointType box;
            box.vertex_min[0] = max(min_xyz[0], cube_min[0]);
            box.vertex_min[1] = max(min_xyz[1], cube_min[1]);
            box.vertex_min[2] = cube_max[2];
            box.vertex_max[0] = min(max_xyz[0], cube_max[0]);
            box.vertex_max[1] = min(max_xyz[1], cube_max[1]);
            box.vertex_max[2] = max_xyz[2];
            cub_needrm.push_back(box);
        }

        // Execute deletions
        if (!cub_needrm.empty())
        {
            ikdtree.Delete_Point_Boxes(cub_needrm);
        }
    }
}

// ═══════════════════════════════════════════════════════════════════════
//  runLaserMapping – main SLAM loop
// ═══════════════════════════════════════════════════════════════════════
void runLaserMapping(FastLioConfig &config, bool use_lvx, const string &lvx_path,
                     bool use_bag, const string &bag_path)
{
    // ── Install signal handler ────────────────────────────────────────
    SetConsoleCtrlHandler(console_ctrl_handler, TRUE);

    // ── Cache config into module-level vars ──────────────────────────
    filter_size_surf_min   = config.filter_size_surf_min;
    filter_size_map_min    = config.filter_size_map_min;
    DET_RANGE              = config.det_range;
    cube_len               = static_cast<double>(config.cube_side_length);
    fov_deg                = config.fov_degree;
    max_iteration          = config.max_iteration;
    max_feature_points     = config.max_feature_points;
    extrinsic_est_en       = config.extrinsic_est_en;
    pcd_save_en            = config.pcd_save_en;
    pcd_interval           = config.pcd_interval;
    runtime_pos_log        = config.runtime_pos_log;
    scan_publish_en        = config.scan_publish_en;
    dense_publish_en       = config.dense_publish_en;
    path_en                = config.path_en;
    scan_bodyframe_pub_en  = config.scan_bodyframe_pub_en;
    publish_full_map       = config.publish_full_map;
    root_dir               = resolveOutputRoot(config.root_dir);
    replay_backpressure_enabled = use_lvx || use_bag;
    replay_backpressure_stop = false;

    // ── Initialise ikd-Tree ──────────────────────────────────────────
    ikdtree.set_downsample_param(static_cast<float>(filter_size_map_min));
    full_map_accumulator.clear();
    full_map_accumulator.setLeafSize(static_cast<float>(filter_size_map_min));

    // ── Initialise preprocessor ──────────────────────────────────────
    p_pre.set(config.feature_enabled, config.lidar_type, config.blind, config.point_filter_num);
    p_pre.lidar_type      = config.lidar_type;
    p_pre.N_SCANS          = config.scan_line;
    p_pre.given_offset_time = false;

    // ── Initialise IMU processor ─────────────────────────────────────
    imu_proc.lidar_type = config.lidar_type;
    imu_proc.set_gyr_cov(V3D(config.gyr_cov, config.gyr_cov, config.gyr_cov));
    imu_proc.set_acc_cov(V3D(config.acc_cov, config.acc_cov, config.acc_cov));
    imu_proc.set_gyr_bias_cov(V3D(config.b_gyr_cov, config.b_gyr_cov, config.b_gyr_cov));
    imu_proc.set_acc_bias_cov(V3D(config.b_acc_cov, config.b_acc_cov, config.b_acc_cov));

    V3D ext_T = config.extrinsic_T;
    M3D ext_R = config.extrinsic_R;
    imu_proc.set_extrinsic(ext_T, ext_R);

    // Open IMU debug log
    {
        // Ensure runtime output directories exist.
        string log_dir = outputPath("Log");
        std::filesystem::create_directories(log_dir);
        std::filesystem::create_directories(outputPath("PCD"));

        string imu_log_path = outputPath("Log/imu.txt");
        imu_proc.fout_imu.open(imu_log_path, ios::out);
        if (!imu_proc.fout_imu.is_open())
        {
            cerr << "[LaserMapping] Failed to open IMU log: " << imu_log_path << endl;
        }
        else
        {
            imu_proc.fout_imu << "timestamp pos_x pos_y pos_z vel_x vel_y vel_z "
                              << "acc_x acc_y acc_z gyro_x gyro_y gyro_z" << '\n';
        }

        string runtime_log_path = outputPath("Log/runtime.txt");
        runtime_log.open(runtime_log_path, ios::out);
        if (!runtime_log.is_open())
        {
            cerr << "[LaserMapping] Failed to open runtime log: " << runtime_log_path << endl;
        }
        else
        {
            runtime_log << "FAST-LIO runtime log" << '\n'
                        << "output_root=" << root_dir << '\n'
                        << "runtime_pos_log=" << (runtime_pos_log ? "true" : "false") << '\n';
            runtime_log.flush();
        }
    }

    // ── Foxglove publisher ───────────────────────────────────────────
    FoxglovePublisher foxglove;
    // ── Bag writer (only in bag mode) ─────────────────────────────────
    unique_ptr<BagWriter> bag_writer;
    if (use_bag) {
        bag_writer = make_unique<BagWriter>();
        // Output bag path: same as input but with _output suffix
        string output_bag = bag_path;
        auto dot_pos = output_bag.rfind('.');
        if (dot_pos != string::npos)
            output_bag = output_bag.substr(0, dot_pos) + "_output.bag";
        else
            output_bag += "_output.bag";

        if (!bag_writer->open(output_bag)) {
            cerr << "[LaserMapping] Failed to open output bag: " << output_bag << endl;
        } else {
            bag_writer->addConnection("/odometry", "geometry_msgs/PoseStamped",
                ros_msg_defs::POSE_STAMPED_MD5, ros_msg_defs::POSE_STAMPED);
            bag_writer->addConnection(kFastLioRegisteredCloudTopic, "sensor_msgs/PointCloud2",
                ros_msg_defs::POINT_CLOUD2_MD5, ros_msg_defs::POINT_CLOUD2);
            bag_writer->addConnection(kFastLioMapTopic, "sensor_msgs/PointCloud2",
                ros_msg_defs::POINT_CLOUD2_MD5, ros_msg_defs::POINT_CLOUD2);
            bag_writer->addConnection("/path", "geometry_msgs/PoseArray",
                ros_msg_defs::POSE_ARRAY_MD5, ros_msg_defs::POSE_ARRAY);
            bag_writer->addConnection("/tf", "tf2_msgs/TFMessage",
                ros_msg_defs::TF_MESSAGE_MD5, ros_msg_defs::TF_MESSAGE);
            cout << "[LaserMapping] Bag writer opened: " << output_bag << endl;
        }
    }

    // ── Data source setup ────────────────────────────────────────────
    unique_ptr<LvxReader>    lvx_reader;
    unique_ptr<LivoxAdapter> livox_adapter;
    unique_ptr<BagReader>    bag_reader;

    // Shared storage for real-time adapter point accumulation
    vector<LvxPoint>    adapter_accum_points;
    double              adapter_accum_time = 0.0;
    mutex               adapter_accum_mtx;
    const double        ACCUM_FRAME_SEC = max(0.01, config.realtime_frame_sec);
    const size_t        ACCUM_POINT_THRESHOLD =
        static_cast<size_t>(max(1000, config.realtime_frame_points));
    atomic<uint64_t>    playback_current_time_ns{0};
    atomic<double>      playback_speed{1.0};
    atomic<bool>        playback_paused{false};

    if (use_lvx)
    {
        lvx_reader = make_unique<LvxReader>();
        if (!lvx_reader->open(lvx_path))
        {
            cerr << "[LaserMapping] Cannot open lvx file: " << lvx_path << endl;
            return;
        }

        lvx_reader->setFrameCallback(
            [](const vector<LvxPoint> &points,
               const vector<ImuData> &imus,
               double timestamp)
            {
                PointCloudXYZI::Ptr cloud = preprocessLivoxPoints(points, timestamp);

                if (!cloud->empty())
                    pushLidarFrame(cloud, timestamp);

                for (auto &imu : imus)
                    pushImuSample(imu);
            });

    }
    else if (use_bag)
    {
        bag_reader = make_unique<BagReader>();
        if (!bag_reader->open(bag_path))
        {
            cerr << "[LaserMapping] Cannot open bag file: " << bag_path << endl;
            return;
        }

        bag_reader->setFrameCallback(
            [](const vector<LvxPoint> &points,
               const vector<ImuData> &imus,
               double timestamp)
            {
                if (!points.empty()) {
                    PointCloudXYZI::Ptr cloud = preprocessLivoxPoints(points, timestamp);
                    if (!cloud->empty())
                        pushLidarFrame(cloud, timestamp);
                }
                for (auto &imu : imus)
                    pushImuSample(imu);
            });

    }
    else
    {
        livox_adapter = make_unique<LivoxAdapter>();
        if (!livox_adapter->init())
        {
            cerr << "[LaserMapping] LivoxAdapter init failed." << endl;
            return;
        }

        livox_adapter->setLidarCallback(
            [&adapter_accum_points, &adapter_accum_time, &adapter_accum_mtx,
             ACCUM_FRAME_SEC, ACCUM_POINT_THRESHOLD]
            (const vector<LvxPoint> &points, double timestamp)
            {
                if (points.empty()) {
                    return;
                }

                vector<LvxPoint> ready_points;
                double ready_time = 0.0;

                {
                    lock_guard<mutex> lock(adapter_accum_mtx);
                    if (adapter_accum_points.empty()) {
                        adapter_accum_time = timestamp;
                    }

                    const double frame_dt = max(0.0, timestamp - adapter_accum_time);
                    const uint32_t offset_ns =
                        static_cast<uint32_t>(min(frame_dt * 1e9,
                                                  static_cast<double>(numeric_limits<uint32_t>::max())));

                    adapter_accum_points.reserve(adapter_accum_points.size() + points.size());
                    for (const auto &point : points) {
                        LvxPoint adjusted = point;
                        adjusted.offset_time = offset_ns;
                        adapter_accum_points.push_back(adjusted);
                    }

                    if (frame_dt >= ACCUM_FRAME_SEC ||
                        adapter_accum_points.size() >= ACCUM_POINT_THRESHOLD) {
                        ready_points.swap(adapter_accum_points);
                        ready_time = adapter_accum_time;
                        adapter_accum_time = timestamp;
                    }
                }

                if (!ready_points.empty()) {
                    PointCloudXYZI::Ptr frame = preprocessLivoxPoints(ready_points, ready_time);
                    if (!frame->empty()) {
                        pushLidarFrame(frame, ready_time);
                    }
                }
            });

        livox_adapter->setImuCallback(
            [](const ImuData &imu) { pushImuSample(imu); });

    }

    // ── State & covariance init ──────────────────────────────────────
    if (use_lvx && lvx_reader)
    {
        const uint64_t end_ns = std::max<uint64_t>(lvx_reader->getDurationNs(), 1);
        foxglove.setPlaybackControl(
            0, end_ns,
            [&](const FoxglovePublisher::PlaybackControlRequest& request) {
                const double speed = request.speed > 0.0f ? request.speed : 1.0f;
                playback_speed = speed;
                lvx_reader->setSpeed(speed);

                if (request.seek_time_ns) {
                    // Seek is temporarily disabled because Foxglove clears the rendered map on seek.
                } else if (request.command == FoxglovePublisher::PlaybackCommand::Pause) {
                    lvx_reader->pause();
                    playback_paused = true;
                } else {
                    lvx_reader->resume();
                    playback_paused = false;
                }

                return FoxglovePublisher::PlaybackState{
                    playback_paused ? FoxglovePublisher::PlaybackStatus::Paused
                                    : FoxglovePublisher::PlaybackStatus::Playing,
                    playback_current_time_ns.load(),
                    static_cast<float>(playback_speed.load()),
                    false};
            });
    }
    else if (use_bag && bag_reader && bag_reader->getEndTimeNs() > bag_reader->getStartTimeNs())
    {
        foxglove.setPlaybackControl(
            bag_reader->getStartTimeNs(), bag_reader->getEndTimeNs(),
            [&](const FoxglovePublisher::PlaybackControlRequest& request) {
                const double speed = request.speed > 0.0f ? request.speed : 1.0f;
                playback_speed = speed;
                bag_reader->setSpeed(speed);

                if (request.seek_time_ns) {
                    // Seek is temporarily disabled because Foxglove clears the rendered map on seek.
                } else if (request.command == FoxglovePublisher::PlaybackCommand::Pause) {
                    bag_reader->pause();
                    playback_paused = true;
                } else {
                    bag_reader->resume();
                    playback_paused = false;
                }

                return FoxglovePublisher::PlaybackState{
                    playback_paused ? FoxglovePublisher::PlaybackStatus::Paused
                                    : FoxglovePublisher::PlaybackStatus::Playing,
                    playback_current_time_ns.load(),
                    static_cast<float>(playback_speed.load()),
                    false};
            });
    }

    if (!foxglove.start("127.0.0.1", 8765))
    {
        cerr << "[LaserMapping] Failed to start Foxglove publisher." << endl;
    }
    else
    {
        cout << "[LaserMapping] Foxglove WebSocket: ws://localhost:"
             << foxglove.getPort() << endl;
    }

    if (use_lvx && lvx_reader)
    {
        lvx_reader->play(playback_speed.load());
        cout << "[LaserMapping] LVX playback started." << endl;
    }
    else if (use_bag && bag_reader)
    {
        bag_reader->play(playback_speed.load());
        cout << "[LaserMapping] Bag playback started." << endl;
    }
    else if (livox_adapter)
    {
        livox_adapter->start(config.livox_broadcast_code);
        cout << "[LaserMapping] Livox adapter started, waiting for data..." << endl;
    }

    state_ikfom init_state = kf.get_x();
    init_state.pos = vect3::Zero();
    init_state.rot = SO3(M3D::Identity());
    init_state.vel = vect3::Zero();
    init_state.bg  = vect3::Zero();
    init_state.ba  = vect3::Zero();
    init_state.offset_R_L_I = SO3(ext_R);
    init_state.offset_T_L_I = ext_T;
    kf.change_x(init_state);

    esekfom::esekf<state_ikfom, 12, input_ikfom>::cov init_P = kf.get_P();
    init_P.setIdentity();
    init_P(6,6)   = init_P(7,7)   = init_P(8,8)   = 0.00001;
    init_P(9,9)   = init_P(10,10)  = init_P(11,11)  = 0.00001;
    init_P(15,15) = init_P(16,16) = init_P(17,17) = 0.0001;
    init_P(18,18) = init_P(19,19) = init_P(20,20) = 0.001;
    init_P(21,21) = init_P(22,22) = 0.00001;
    kf.change_P(init_P);

    // ── Register IKFoM callbacks ──────────────────────────────────────
    double R_limit[24];
    for (int i = 0; i < 24; i++) R_limit[i] = 0.0001;
    kf.init_dyn_share(get_f, df_dx, df_dw, h_share_model, max_iteration, R_limit);

    // ── Timing & bookkeeping ─────────────────────────────────────────
    int  frame_id = 0;
    double T1, T2, s_plot, s_plot2, s_plot3, s_plot4, s_plot5, s_plot6;
    const auto foxglove_publish_start = chrono::steady_clock::now();
    auto last_foxglove_cloud_pub =
        foxglove_publish_start - chrono::milliseconds(FOXGLOVE_CLOUD_INTERVAL_MS);
    auto last_foxglove_map_pub =
        foxglove_publish_start - chrono::milliseconds(FOXGLOVE_MAP_INTERVAL_MS);
    auto last_foxglove_path_pub =
        foxglove_publish_start - chrono::milliseconds(FOXGLOVE_PATH_INTERVAL_MS);

    // PCD accumulation cloud
    PointCloudXYZI::Ptr pcl_wait_save(new PointCloudXYZI());

    // Ensure PCD directory exists
    {
        string pcd_dir = outputPath("PCD");
        std::filesystem::create_directories(pcd_dir);
    }

    cout << "=================================================" << endl;
    cout << "  FAST-LIO2 SLAM loop started.  Press Ctrl-C to stop." << endl;
    cout << "  Playback control build: seek requests disabled." << endl;
    cout << "=================================================" << endl;

    // ══════════════════════════════════════════════════════════════════
    //  MAIN LOOP
    // ══════════════════════════════════════════════════════════════════
    while (!g_exit_flag.load())
    {
        // Check EOF for lvx/bag — but only exit after all buffered data is processed
        if ((use_lvx || use_bag) && playback_paused.load())
        {
            if (foxglove.isRunning()) {
                foxglove.broadcastPlaybackState(FoxglovePublisher::PlaybackState{
                    FoxglovePublisher::PlaybackStatus::Paused,
                    playback_current_time_ns.load(),
                    static_cast<float>(playback_speed.load()),
                    false});
            }
            this_thread::sleep_for(chrono::milliseconds(20));
            continue;
        }

        if (use_lvx && lvx_reader && lvx_reader->isEOF())
        {
            lock_guard<mutex> lock(mtx_buffer);
            if (lidar_buffer.empty())
            {
                cout << "[LaserMapping] LVX playback finished (EOF)." << endl;
                break;
            }
        }
        if (use_bag && bag_reader && bag_reader->isEOF())
        {
            lock_guard<mutex> lock(mtx_buffer);
            if (lidar_buffer.empty())
            {
                cout << "[LaserMapping] Bag playback finished (EOF)." << endl;
                break;
            }
        }

        auto t_loop_start = chrono::steady_clock::now();

        // ── 1. Synchronise LiDAR + IMU ─────────────────────────────
        T1 = chrono::duration<double>(chrono::steady_clock::now().time_since_epoch()).count();
        if (!sync_packages(Measures))
        {
            static int sync_wait_count = 0;
            sync_wait_count++;
            if (sync_wait_count % 100 == 0)
            {
                writeRuntimeLog("event=sync_wait count=" + to_string(sync_wait_count));
            }
            this_thread::sleep_for(chrono::milliseconds(10));
            continue;
        }
        T2 = chrono::duration<double>(chrono::steady_clock::now().time_since_epoch()).count();
        s_plot = (T2 - T1) * 1000.0;  // ms

        if (Measures.lidar->points.empty())
        {
            ostringstream log_line;
            log_line << fixed << setprecision(6)
                     << "event=skip reason=empty_lidar"
                     << " lidar_beg=" << Measures.lidar_beg_time
                     << " lidar_end=" << Measures.lidar_end_time
                     << " imu=" << Measures.imu.size();
            writeRuntimeLog(log_line.str());
            continue;
        }

        // Record first LiDAR time
        if (first_lidar_time < 1.0)
            first_lidar_time = Measures.lidar_beg_time;
        flg_EKF_inited = (Measures.lidar_beg_time - first_lidar_time) >= kFastLioInitTime;

        // Diagnostic: log SLAM frame processing
        static int slam_frame_count = 0;
        slam_frame_count++;
        {
            ostringstream log_line;
            log_line << fixed << setprecision(6)
                     << "event=input slam_frame=" << slam_frame_count
                     << " lidar_points=" << Measures.lidar->points.size()
                     << " imu=" << Measures.imu.size()
                     << " lidar_beg=" << Measures.lidar_beg_time
                     << " lidar_end=" << Measures.lidar_end_time;
            writeRuntimeLog(log_line.str());
        }
        const bool verbose_slam_log =
            runtime_pos_log &&
            (slam_frame_count % VERBOSE_SLAM_LOG_INTERVAL == 0);
        if (verbose_slam_log) {
            cout << "[SLAM] Frame " << slam_frame_count
                 << " pts=" << Measures.lidar->points.size()
                 << " imu=" << Measures.imu.size()
                 << " t_beg=" << Measures.lidar_beg_time
                 << " t_end=" << Measures.lidar_end_time << '\n';
        }

        // ── 2. IMU propagation & undistortion ──────────────────────
        T1 = chrono::duration<double>(chrono::steady_clock::now().time_since_epoch()).count();
        imu_proc.Process(Measures, kf, feats_undistort);
        T2 = chrono::duration<double>(chrono::steady_clock::now().time_since_epoch()).count();
        s_plot2 = (T2 - T1) * 1000.0;

        // featsFromMap now holds the undistorted point cloud
        if (feats_undistort->points.empty())
        {
            static int no_undistorted_log_count = 0;
            no_undistorted_log_count++;
            if (runtime_pos_log &&
                no_undistorted_log_count % VERBOSE_SLAM_LOG_INTERVAL == 0) {
                cout << "[LaserMapping] No undistorted points, skipping frame."
                     << " (slam_frame=" << slam_frame_count << ")" << '\n';
            }
            ostringstream log_line;
            log_line << "event=skip reason=no_undistorted"
                     << " slam_frame=" << slam_frame_count
                     << " lidar_points=" << Measures.lidar->points.size()
                     << " imu=" << Measures.imu.size();
            writeRuntimeLog(log_line.str());
            continue;
        }
        if (verbose_slam_log) {
            cout << "[SLAM] Undistorted pts=" << feats_undistort->points.size() << '\n';
        }

        // ── 3. Downsample ───────────────────────────────────────────
        T1 = chrono::duration<double>(chrono::steady_clock::now().time_since_epoch()).count();

        voxelDownsampleSafe(feats_undistort, feats_down_body, filter_size_surf_min);
        T2 = chrono::duration<double>(chrono::steady_clock::now().time_since_epoch()).count();
        s_plot3 = (T2 - T1) * 1000.0;

        // Keep IEKF matrix size bounded; set max_feature_points <= 0 to disable.
        if (max_feature_points > 0 &&
            feats_down_body->points.size() > static_cast<size_t>(max_feature_points))
        {
            feats_down_body->points.resize(static_cast<size_t>(max_feature_points));
        }

        feats_down_size = static_cast<int>(feats_down_body->points.size());
        if (verbose_slam_log) {
            cout << "[SLAM] DS pts=" << feats_down_size << '\n';
        }

        if (feats_down_size < 5)
        {
            static int too_few_downsampled_log_count = 0;
            too_few_downsampled_log_count++;
            if (runtime_pos_log &&
                too_few_downsampled_log_count % VERBOSE_SLAM_LOG_INTERVAL == 0) {
                cout << "[LaserMapping] Too few downsampled points, skipping frame." << '\n';
            }
            ostringstream log_line;
            log_line << "event=skip reason=too_few_downsampled"
                     << " slam_frame=" << slam_frame_count
                     << " points=" << feats_down_size;
            writeRuntimeLog(log_line.str());
            continue;
        }

        // ── 4. IEKF measurement update ──────────────────────────────
        // Skip IEKF on first frame when ikd-Tree is empty (no map yet)
        if (ikdtree.size() == 0)
        {
            if (verbose_slam_log) {
                cout << "[SLAM] First frame, skip IEKF (empty map)" << '\n';
            }
        }
        else
        {
            T1 = chrono::duration<double>(chrono::steady_clock::now().time_since_epoch()).count();
            normvec->points.resize(feats_down_size);
            feats_down_world->points.resize(feats_down_size);
            Nearest_Points.resize(feats_down_size);
            point_selected_surf.assign(feats_down_size, 1);
            res_last.assign(feats_down_size, -1000.0);
            double solve_time = 0.0;
            iekf_update_wrapper(solve_time);
            T2 = chrono::duration<double>(chrono::steady_clock::now().time_since_epoch()).count();
            s_plot4 = (T2 - T1) * 1000.0;
            if (verbose_slam_log) {
                cout << "[SLAM] IEKF done effective=" << effct_feat_num << '\n';
            }
        }

        // Update cached state
        state_point = kf.get_x();
        euler_cur   = SO3ToEuler(state_point.rot);
        pos_lid     = state_point.pos + state_point.rot.toRotationMatrix() * state_point.offset_T_L_I;
        geoQuat     = Eigen::Quaterniond(state_point.rot.coeffs()[3],
                                          state_point.rot.coeffs()[0],
                                          state_point.rot.coeffs()[1],
                                          state_point.rot.coeffs()[2]);

        // ── 5. Map incremental ──────────────────────────────────────
        T1 = chrono::duration<double>(chrono::steady_clock::now().time_since_epoch()).count();
        map_incremental();
        T2 = chrono::duration<double>(chrono::steady_clock::now().time_since_epoch()).count();
        s_plot5 = (T2 - T1) * 1000.0;

        // ── 6. Visualization publishing ─────────────────────────────
        T1 = chrono::duration<double>(chrono::steady_clock::now().time_since_epoch()).count();
        double pub_time = Measures.lidar_end_time;
        const uint64_t pub_time_ns = doubleToNs(pub_time);
        playback_current_time_ns = pub_time_ns;
        const bool write_bag_output = (bag_writer && bag_writer->isOpen());
        const bool publish_foxglove = foxglove.isRunning();
        const auto publish_wall_now = chrono::steady_clock::now();

        // Publish odometry
        V3D pos_v3d(state_point.pos[0], state_point.pos[1], state_point.pos[2]);

        if (write_bag_output) {
            bag_writer->writeOdometry(pub_time_ns, pos_v3d, geoQuat, kFastLioMapFrame);
            bag_writer->writeTF(pub_time_ns, pos_v3d, geoQuat,
                                kFastLioMapFrame, kFastLioBodyFrame);
        }
        if (publish_foxglove) {
            foxglove.publishOdometry(pos_v3d, geoQuat, pub_time);
            foxglove.publishTransform(pos_v3d, geoQuat,
                                      kFastLioMapFrame, kFastLioBodyFrame, pub_time);
        }

        // Build world-frame cloud for publishing
        PointCloudXYZI cloud_world;
        if (scan_publish_en)
        {
            PointCloudXYZI::Ptr publish_cloud =
                dense_publish_en ? feats_undistort : feats_down_body;
            cloud_world.reserve(publish_cloud->points.size());
            PointType pt_world;
            for (size_t i = 0; i < publish_cloud->points.size(); i++)
            {
                pointBodyToWorld(&(publish_cloud->points[i]), &pt_world);
                cloud_world.push_back(pt_world);
            }
            if (write_bag_output) {
                bag_writer->writePointCloud(pub_time_ns, cloud_world, kFastLioMapFrame,
                                            kFastLioRegisteredCloudTopic);
            }
            const bool publish_foxglove_cloud =
                publish_foxglove &&
                chrono::duration_cast<chrono::milliseconds>(
                    publish_wall_now - last_foxglove_cloud_pub).count() >=
                    FOXGLOVE_CLOUD_INTERVAL_MS;
            if (publish_foxglove_cloud) {
                foxglove.publishPointCloud(cloud_world, pub_time);
                last_foxglove_cloud_pub = publish_wall_now;
            }
        }

        if (publish_full_map && !cloud_world.empty()) {
            full_map_accumulator.addFrame(cloud_world);
        }

        const int map_publish_interval =
            publish_full_map ? FULL_MAP_PUBLISH_INTERVAL : MAP_PUBLISH_INTERVAL;
        const bool publish_foxglove_map =
            publish_foxglove &&
            chrono::duration_cast<chrono::milliseconds>(
                publish_wall_now - last_foxglove_map_pub).count() >=
                FOXGLOVE_MAP_INTERVAL_MS;
        if ((publish_foxglove_map || write_bag_output) &&
            frame_id % map_publish_interval == 0)
        {
            PointCloudXYZI map_cloud;
            if (publish_full_map) {
                map_cloud = full_map_accumulator.snapshot();
            } else if (ikdtree.Root_Node != nullptr) {
                PointVector map_storage;
                ikdtree.flatten(ikdtree.Root_Node, map_storage, NOT_RECORD);
                map_cloud.points = map_storage;
            }

            if (!map_cloud.empty())
            {
                map_cloud.width = static_cast<uint32_t>(map_cloud.points.size());
                map_cloud.height = 1;
                map_cloud.is_dense = true;

                if (write_bag_output) {
                    bag_writer->writePointCloud(pub_time_ns, map_cloud, kFastLioMapFrame,
                                                kFastLioMapTopic);
                }
                if (publish_foxglove_map) {
                    foxglove.publishMap(map_cloud, pub_time);
                    last_foxglove_map_pub = publish_wall_now;
                }
            }
        }

        // Publish path
        if (path_en)
        {
            path_vec.push_back(pos_v3d);
            if (write_bag_output) {
                bag_writer->writePath(pub_time_ns, path_vec, kFastLioMapFrame);
            }
            const bool publish_foxglove_path =
                publish_foxglove &&
                chrono::duration_cast<chrono::milliseconds>(
                    publish_wall_now - last_foxglove_path_pub).count() >=
                    FOXGLOVE_PATH_INTERVAL_MS;
            if (publish_foxglove_path) {
                foxglove.publishPath(path_vec, pub_time);
                last_foxglove_path_pub = publish_wall_now;
            }
        }

        if (publish_foxglove) {
            foxglove.broadcastTime(pub_time);
            if (use_lvx || use_bag) {
                foxglove.broadcastPlaybackState(FoxglovePublisher::PlaybackState{
                    playback_paused ? FoxglovePublisher::PlaybackStatus::Paused
                                    : FoxglovePublisher::PlaybackStatus::Playing,
                    pub_time_ns,
                    static_cast<float>(playback_speed.load()),
                    false});
            }
        }

        T2 = chrono::duration<double>(chrono::steady_clock::now().time_since_epoch()).count();
        s_plot6 = (T2 - T1) * 1000.0;

        // ── 7. PCD saving ───────────────────────────────────────────
        if (pcd_save_en)
        {
            // Accumulate world-frame points
            PointType pt_world;
            for (size_t i = 0; i < feats_undistort->points.size(); i++)
            {
                pointBodyToWorld(&(feats_undistort->points[i]), &pt_world);
                pcl_wait_save->push_back(pt_world);
            }

            if (frame_id > 0 && (frame_id % pcd_interval == 0))
            {
                string pcd_path = outputPath("PCD/scans_" + to_string(frame_id) + ".pcd");
                pcl::io::savePCDFileASCII(pcd_path, *pcl_wait_save);
                cout << "[PCD] Saved: " << pcd_path
                     << " (" << pcl_wait_save->size() << " pts)" << endl;
            }
        }

        // ── 8. Timing log ───────────────────────────────────────────
        auto t_loop_end = chrono::steady_clock::now();
        double loop_ms = chrono::duration<double, milli>(t_loop_end - t_loop_start).count();

        if (runtime_pos_log)
        {
            printf("[Frame %04d] sync:%.1f imu:%.1f ds:%.1f ekf:%.1f map:%.1f pub:%.1f | total:%.1f ms | pts:%d tree:%d\n",
                   frame_id, s_plot, s_plot2, s_plot3, s_plot4, s_plot5, s_plot6,
                   loop_ms,
                   feats_down_size,
                   ikdtree.size());
        }
        else if (frame_id % 20 == 0)
        {
            printf("[Frame %04d] pos: (%.2f, %.2f, %.2f)  tree: %d  total: %.1f ms\n",
                   frame_id,
                   state_point.pos(0), state_point.pos(1), state_point.pos(2),
                   ikdtree.size(), loop_ms);
        }

        ostringstream log_line;
        log_line << fixed << setprecision(3)
                 << "event=processed frame=" << frame_id
                 << " slam_frame=" << slam_frame_count
                 << " sync_ms=" << s_plot
                 << " imu_ms=" << s_plot2
                 << " downsample_ms=" << s_plot3
                 << " ekf_ms=" << s_plot4
                 << " map_ms=" << s_plot5
                 << " publish_ms=" << s_plot6
                 << " total_ms=" << loop_ms
                 << " points=" << feats_down_size
                 << " tree_size=" << ikdtree.size()
                 << " pos=(" << state_point.pos(0) << ","
                 << state_point.pos(1) << ","
                 << state_point.pos(2) << ")";
        writeRuntimeLog(log_line.str());

        frame_id++;

        // Throttle slightly so playback thread doesn't run too far ahead
        if (use_lvx || use_bag)
            this_thread::sleep_for(chrono::milliseconds(1));
    }

    // ══════════════════════════════════════════════════════════════════
    //  Shutdown
    // ══════════════════════════════════════════════════════════════════
    cout << "\n[LaserMapping] Shutting down..." << endl;

    if (publish_full_map && (foxglove.isRunning() || (bag_writer && bag_writer->isOpen()))) {
        PointCloudXYZI final_map = full_map_accumulator.snapshot();
        if (!final_map.empty()) {
            final_map.width = static_cast<uint32_t>(final_map.points.size());
            final_map.height = 1;
            final_map.is_dense = true;
            const uint64_t final_time_ns = playback_current_time_ns.load();
            const double final_time = static_cast<double>(final_time_ns) / 1e9;
            if (bag_writer && bag_writer->isOpen()) {
                bag_writer->writePointCloud(final_time_ns, final_map, kFastLioMapFrame,
                                            kFastLioMapTopic);
            }
            if (foxglove.isRunning()) {
                foxglove.publishMap(final_map, final_time);
            }
        }
    }

    if ((use_lvx || use_bag) && foxglove.isRunning()) {
        foxglove.broadcastPlaybackState(FoxglovePublisher::PlaybackState{
            FoxglovePublisher::PlaybackStatus::Ended,
            playback_current_time_ns.load(),
            static_cast<float>(playback_speed.load()),
            false});
    }

    // Stop data sources
    replay_backpressure_stop = true;
    sig_buffer.notify_all();
    if (lvx_reader) lvx_reader->stop();
    if (livox_adapter) livox_adapter->stop();
    if (bag_reader) bag_reader->stop();

    // Save final PCD if enabled
    if (pcd_save_en && pcl_wait_save->size() > 0)
    {
        string pcd_path = outputPath("PCD/scans_final.pcd");
        pcl::io::savePCDFileASCII(pcd_path, *pcl_wait_save);
        cout << "[PCD] Final cloud saved: " << pcd_path
             << " (" << pcl_wait_save->size() << " pts)" << endl;
    }

    // Dump ikd-Tree map
    {
        PointVector map_storage;
        ikdtree.flatten(ikdtree.Root_Node, map_storage, NOT_RECORD);
        if (!map_storage.empty())
        {
            PointCloudXYZI map_cloud;
            map_cloud.points = map_storage;
            string map_path = outputPath("PCD/ikd_tree_map.pcd");
            pcl::io::savePCDFileASCII(map_path, map_cloud);
            cout << "[PCD] ikd-Tree map saved: " << map_path
                 << " (" << map_cloud.size() << " pts)" << endl;
        }
    }

    // Stop Foxglove / Bag writer
    if (bag_writer) bag_writer->close();
    foxglove.stop();

    // Close IMU log
    if (imu_proc.fout_imu.is_open())
        imu_proc.fout_imu.close();
    if (runtime_log.is_open())
    {
        runtime_log << "total_frames=" << frame_id << '\n';
        runtime_log.close();
    }

    cout << "[LaserMapping] Total frames processed: " << frame_id << endl;
    cout << "[LaserMapping] Done." << endl;
}
