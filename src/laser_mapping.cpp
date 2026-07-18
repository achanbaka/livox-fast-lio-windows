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
#include "bounded_timing_stats.h"
#include "map_build_worker.h"
#include "realtime_foxglove_worker.h"
#include "storage_worker.h"
#include "IMU_Processing.hpp"
#include "preprocess.h"
#include "livox_adapter.h"
#include "lvx_reader.h"
#include "foxglove_publisher.h"
#include "fast_lio_observation.h"
#include "lidar_imu_sync.h"
#include "bag_reader.h"
#include "playback_status.h"
#include "ikd-Tree/ikd_Tree.h"
#include "use-ikfom.hpp"
#include "common_lib.h"

#include <windows.h>
#include <psapi.h>
#include <intrin.h>

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
#include <memory>
#include <mutex>
#include <numeric>
#include <sstream>
#include <string>
#include <system_error>
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
// ikd-Tree owns a background rebuild thread whose third-party destructor
// performs an unbounded join.  A multi-million-point rebuild cannot be
// cancelled and has been observed to keep the process alive (or touch freed
// tree storage) after every application-owned worker, file, and socket was
// already shut down.  Keep this application-wide map alive for the lifetime
// of the Windows process; ExitProcess stops the rebuild thread and reclaims
// its storage after normal main()/CRT cleanup without invoking that unsafe
// destructor.
static KD_TREE<pcl::PointXYZINormal>& ikdtree =
    *new KD_TREE<pcl::PointXYZINormal>();

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
static int    iekf_match_threads     = 4;
static bool   extrinsic_est_en       = true;
static bool   pcd_save_en            = false;
static int    pcd_interval           = 1;
static bool   runtime_pos_log        = false;
static bool   scan_publish_en        = true;
static bool   dense_publish_en       = true;
static bool   path_en                = true;
static bool   scan_bodyframe_pub_en  = false;
static bool   publish_full_map       = true;
static bool   async_full_map_publish = true;
static bool   bag_full_map_periodic  = false;
static bool   publish_map_delta      = false;
static int    full_map_publish_interval_ms = 1000;
static double full_map_voxel_size    = 0.2;
static size_t map_delta_max_pending_points = 200000;
static int    foxglove_control_interval_ms = 20;
static size_t foxglove_backlog_size  = 64;
static string root_dir;
static ofstream runtime_log;
static constexpr size_t MAX_REPLAY_LIDAR_QUEUE = 3;
static constexpr size_t MAX_LIVE_LIDAR_QUEUE = 64;
static constexpr size_t MAX_IMU_QUEUE = 4096;
static atomic<uint64_t> input_lidar_frames_dropped{0};
static atomic<uint64_t> input_imu_samples_dropped{0};
static atomic<size_t> input_lidar_queue_high_water{0};
static atomic<size_t> input_imu_queue_high_water{0};
static int foxglove_cloud_interval_ms = 50;
static int foxglove_path_interval_ms = 500;
static constexpr int VERBOSE_SLAM_LOG_INTERVAL = 20;
static atomic<bool> replay_backpressure_enabled{false};
static atomic<bool> replay_backpressure_stop{false};

static void updateAtomicHighWater(atomic<size_t>& high_water,
                                  size_t candidate) noexcept
{
    size_t current = high_water.load(memory_order_relaxed);
    while (current < candidate &&
           !high_water.compare_exchange_weak(
               current, candidate,
               memory_order_relaxed, memory_order_relaxed)) {}
}

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

static size_t currentProcessRssBytes()
{
    PROCESS_MEMORY_COUNTERS_EX counters{};
    if (!GetProcessMemoryInfo(
            GetCurrentProcess(),
            reinterpret_cast<PROCESS_MEMORY_COUNTERS*>(&counters),
            sizeof(counters)))
    {
        return 0;
    }
    return static_cast<size_t>(counters.WorkingSetSize);
}

static void writeRuntimeLog(const string& line)
{
    if (runtime_log.is_open())
    {
        runtime_log << line << '\n';
        static uint64_t lines_since_flush = 0;
        if (++lines_since_flush >= 100) {
            runtime_log.flush();
            lines_since_flush = 0;
        }
    }
}

// This path is reserved for a violated shutdown bound. Do not add logging,
// allocation, stream flushing, or DLL teardown here: another thread may have
// been stopped while owning the corresponding lock.
[[noreturn]] static void terminateForShutdownTimeout() noexcept
{
    ::TerminateProcess(::GetCurrentProcess(), ERROR_TIMEOUT);
    __fastfail(FAST_FAIL_FATAL_APP_EXIT);
}

class ShutdownWatchdog
{
public:
    explicit ShutdownWatchdog(chrono::steady_clock::time_point deadline)
        : deadline_(deadline)
    {}

    ShutdownWatchdog(const ShutdownWatchdog&) = delete;
    ShutdownWatchdog& operator=(const ShutdownWatchdog&) = delete;

    bool start() noexcept
    {
        try {
            worker_ = thread([this] {
                try {
                    unique_lock<mutex> lock(mutex_);
                    const bool cancelled = cv_.wait_until(
                        lock, deadline_, [this] { return cancelled_; });
                    if (!cancelled) {
                        lock.unlock();
                        terminateForShutdownTimeout();
                    }
                } catch (...) {
                    terminateForShutdownTimeout();
                }
            });
            started_ = true;
            return true;
        } catch (...) {
            return false;
        }
    }

    void cancelAndJoin() noexcept
    {
        if (!started_) return;
        try {
            {
                lock_guard<mutex> lock(mutex_);
                if (chrono::steady_clock::now() >= deadline_) {
                    terminateForShutdownTimeout();
                }
                cancelled_ = true;
            }
            cv_.notify_all();
            if (worker_.joinable()) worker_.join();
            started_ = false;
        } catch (...) {
            terminateForShutdownTimeout();
        }
    }

    ~ShutdownWatchdog() noexcept
    {
        // Only an explicit, successful cancel is allowed to disarm the hard
        // bound. Exception unwinding during shutdown must not expose the
        // process to unbounded worker destructors.
        if (started_) terminateForShutdownTimeout();
    }

private:
    chrono::steady_clock::time_point deadline_;
    mutex mutex_;
    condition_variable cv_;
    thread worker_;
    bool cancelled_ = false;
    bool started_ = false;
};

struct PlaybackControlLifetimeState
{
    mutex reader_mutex;
    LvxReader* lvx_reader = nullptr;
    BagReader* bag_reader = nullptr;
    atomic<uint64_t> current_time_ns{0};
    atomic<double> speed{1.0};
    atomic<bool> paused{false};
    atomic<bool> pause_drain_pending{false};
};

class PlaybackReaderLifetimeGuard
{
public:
    explicit PlaybackReaderLifetimeGuard(
        shared_ptr<PlaybackControlLifetimeState> state)
        : state_(std::move(state))
    {}

    PlaybackReaderLifetimeGuard(const PlaybackReaderLifetimeGuard&) = delete;
    PlaybackReaderLifetimeGuard& operator=(
        const PlaybackReaderLifetimeGuard&) = delete;

    void setLvxReader(LvxReader* reader)
    {
        lock_guard<mutex> lock(state_->reader_mutex);
        state_->lvx_reader = reader;
        state_->bag_reader = nullptr;
    }

    void setBagReader(BagReader* reader)
    {
        lock_guard<mutex> lock(state_->reader_mutex);
        state_->bag_reader = reader;
        state_->lvx_reader = nullptr;
    }

    void disarm() noexcept
    {
        if (!state_) return;
        try {
            auto state = state_;
            {
                lock_guard<mutex> lock(state->reader_mutex);
                state->lvx_reader = nullptr;
                state->bag_reader = nullptr;
            }
            state_.reset();
        } catch (...) {
            terminateForShutdownTimeout();
        }
    }

    ~PlaybackReaderLifetimeGuard() noexcept
    {
        disarm();
    }

private:
    shared_ptr<PlaybackControlLifetimeState> state_;
};

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
static vector<vector<float>> Nearest_Point_Distances;
static vector<uint8_t> point_selected_surf;
static vector<double> res_last;
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
    } else {
        while (lidar_buffer.size() >= MAX_LIVE_LIDAR_QUEUE) {
            lidar_buffer.pop_front();
            if (!time_buffer.empty()) time_buffer.pop_front();
            input_lidar_frames_dropped.fetch_add(1);
        }
    }
    lidar_buffer.push_back(cloud);
    time_buffer.push_back(timestamp);
    updateAtomicHighWater(input_lidar_queue_high_water, lidar_buffer.size());
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
            input_imu_samples_dropped.fetch_add(imu_buffer.size());
            imu_buffer.clear();
        }
        last_timestamp_imu = imu.timestamp;
        while (imu_buffer.size() >= MAX_IMU_QUEUE) {
            imu_buffer.pop_front();
            input_imu_samples_dropped.fetch_add(1);
        }
        imu_buffer.push_back(ImuDataConstPtr(p));
        updateAtomicHighWater(input_imu_queue_high_water, imu_buffer.size());
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

#ifdef _OPENMP
#pragma omp parallel for schedule(static) num_threads(iekf_match_threads) if(iekf_match_threads > 1)
#endif
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

        auto &points_near = Nearest_Points[i];
        auto &pointSearchSqDis = Nearest_Point_Distances[i];

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
    if (config.iekf_match_threads > 0) {
        iekf_match_threads = config.iekf_match_threads;
    } else {
        const unsigned int hardware_threads = std::thread::hardware_concurrency();
        const unsigned int auto_threads = hardware_threads > 0
            ? std::max(1u, std::min(8u, hardware_threads / 2u))
            : 4u;
        iekf_match_threads = static_cast<int>(auto_threads);
    }
    extrinsic_est_en       = config.extrinsic_est_en;
    pcd_save_en            = config.pcd_save_en;
    pcd_interval           = config.pcd_interval;
    runtime_pos_log        = config.runtime_pos_log;
    scan_publish_en        = config.scan_publish_en;
    dense_publish_en       = config.dense_publish_en;
    path_en                = config.path_en;
    scan_bodyframe_pub_en  = config.scan_bodyframe_pub_en;
    const bool full_map_mode =
        config.map_output_mode == "full_async" ||
        config.map_output_mode == "hybrid";
    const bool tiled_mode =
        config.map_output_mode == "tiled_incremental" ||
        config.map_output_mode == "hybrid";
    // The grouped mode is authoritative. The legacy publish flags are mapped
    // into map_output.mode by YamlConfig before execution reaches this point.
    publish_full_map       = full_map_mode;
    publish_map_delta      = tiled_mode;
    const bool map_pipeline_enabled = publish_full_map || publish_map_delta;
    async_full_map_publish = config.async_full_map_publish;
    bag_full_map_periodic  = config.bag_full_map_periodic;
    const bool periodic_full_map_enabled = publish_full_map;
    full_map_publish_interval_ms = max(
        100, config.map_output_full_publish_interval_ms);
    full_map_voxel_size = config.map_output_full_voxel_leaf_m > 0.0
        ? max(filter_size_map_min, config.map_output_full_voxel_leaf_m)
        : filter_size_map_min;
    map_delta_max_pending_points = static_cast<size_t>(
        max(1000, config.map_output_max_points_per_update));
    // Foxglove uses broadcast time as its live playback clock. Keep it near the
    // lidar frame rate so odometry/TF motion remains smooth; disconnect
    // protection comes from avoiding repeated PlaybackState broadcasts and
    // from the larger server backlog, not from reducing the clock to 4 Hz.
    foxglove_control_interval_ms = max(10, config.foxglove_control_interval_ms);
    foxglove_backlog_size = static_cast<size_t>(
        max(8, config.foxglove_backlog_size));
    foxglove_cloud_interval_ms = max(
        1, static_cast<int>(lround(1000.0 /
            max(0.1, config.foxglove_current_cloud_publish_hz))));
    foxglove_path_interval_ms = max(
        1, static_cast<int>(lround(1000.0 /
            max(0.1, config.foxglove_path_publish_hz))));
    const size_t bag_path_max_points = static_cast<size_t>(
        max(1, config.storage_path_max_points));
    root_dir               = resolveOutputRoot(config.root_dir);
    replay_backpressure_enabled = use_lvx || use_bag;
    replay_backpressure_stop = false;
    path_vec.clear();
    {
        lock_guard<mutex> lock(mtx_buffer);
        lidar_buffer.clear();
        time_buffer.clear();
        imu_buffer.clear();
        last_timestamp_imu = -1.0;
    }
    input_lidar_frames_dropped = 0;
    input_imu_samples_dropped = 0;
    input_lidar_queue_high_water = 0;
    input_imu_queue_high_water = 0;
    if (use_bag) path_vec.reserve(min<size_t>(bag_path_max_points, 10000));

    // ── Initialise ikd-Tree ──────────────────────────────────────────
    ikdtree.set_downsample_param(static_cast<float>(filter_size_map_min));

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
                        << "runtime_pos_log=" << (runtime_pos_log ? "true" : "false") << '\n'
                        << "iekf_match_threads=" << iekf_match_threads << '\n'
                        << "async_full_map_publish="
                        << (async_full_map_publish ? "true" : "false") << '\n'
                        << "full_map_publish_interval_ms="
                        << full_map_publish_interval_ms << '\n'
                        << "full_map_voxel_size="
                        << full_map_voxel_size << '\n'
                        << "bag_full_map_periodic="
                        << (bag_full_map_periodic ? "true" : "false") << '\n'
                        << "map_output_mode=" << config.map_output_mode << '\n'
                        << "publish_full_map="
                        << (publish_full_map ? "true" : "false") << '\n'
                        << "publish_map_delta="
                        << (publish_map_delta ? "true" : "false") << '\n'
                        << "map_delta_max_pending_points="
                        << map_delta_max_pending_points << '\n'
                        << "foxglove_control_interval_ms="
                        << foxglove_control_interval_ms << '\n'
                        << "foxglove_backlog_size="
                        << foxglove_backlog_size << '\n'
                        << "storage_path_max_points="
                        << bag_path_max_points << '\n';
            runtime_log.flush();
        }
    }

    // ── Independent output consumers ──────────────────────────────────
    FoxglovePublisher foxglove;
    RealtimeFoxgloveWorker::Callbacks realtime_callbacks;
    realtime_callbacks.imu = [&foxglove](const ImuData& imu) {
        if (foxglove.getClientCount() > 0) foxglove.publishImu(imu);
    };
    realtime_callbacks.pose = [&foxglove](
        const V3D& position, const Eigen::Quaterniond& orientation,
        double timestamp) {
        if (foxglove.getClientCount() > 0) {
            foxglove.publishOdometry(position, orientation, timestamp);
            foxglove.publishTransform(position, orientation, kFastLioMapFrame,
                                      kFastLioBodyFrame, timestamp);
        }
    };
    realtime_callbacks.cloud = [&foxglove](
        const PointCloudXYZI& cloud, double timestamp) {
        if (foxglove.getClientCount() > 0) {
            foxglove.publishPointCloud(cloud, timestamp);
        }
    };
    realtime_callbacks.path = [&foxglove](
        const vector<V3D>& path, double timestamp) {
        if (foxglove.getClientCount() > 0) {
            foxglove.publishPath(path, timestamp);
        }
    };
    realtime_callbacks.time = [&foxglove](double timestamp) {
        if (foxglove.getClientCount() > 0) foxglove.broadcastTime(timestamp);
    };
    realtime_callbacks.playback = [&foxglove](
        const FoxglovePublisher::PlaybackState& state) {
        if (foxglove.getClientCount() > 0) {
            foxglove.broadcastPlaybackState(state);
        }
    };
    RealtimeFoxgloveWorker realtime_foxglove(
        std::move(realtime_callbacks));
    unique_ptr<StorageWorker> storage_worker;
    if (use_bag || pcd_save_en) {
        StorageWorker::Config storage_config;
        storage_config.mode = StorageWorker::parseMode(config.storage_mode);
        storage_config.queue_max_bytes = static_cast<size_t>(
            max(1, config.storage_queue_max_mb)) * 1024ULL * 1024ULL;
        storage_config.bag_path_publish_hz =
            max(0.0, config.storage_bag_path_publish_hz);
        storage_config.pcd_format =
            StorageWorker::parsePcdFormat(config.storage_pcd_format);
        storage_config.pcd_chunk_points = static_cast<size_t>(
            max(1000, config.storage_pcd_chunk_points));
        storage_config.pcd_chunk_frames = static_cast<size_t>(
            max(0, config.storage_pcd_chunk_frames));
        storage_config.output_root = root_dir;
        storage_config.enable_pcd = pcd_save_en;
        if (use_bag) {
            storage_config.bag_path = bag_path;
            const auto dot_pos = storage_config.bag_path.rfind('.');
            if (dot_pos != string::npos) {
                storage_config.bag_path =
                    storage_config.bag_path.substr(0, dot_pos) + "_output.bag";
            } else {
                storage_config.bag_path += "_output.bag";
            }
        }
        storage_worker = make_unique<StorageWorker>(std::move(storage_config));
        if (!storage_worker->startFor(chrono::seconds(30))) {
            cerr << "[LaserMapping] Storage worker failed to start; SLAM will continue."
                 << endl;
        }
    }

    atomic<bool> final_bag_map_requested{false};
    unique_ptr<MapBuildWorker> async_map_publisher;
    if (map_pipeline_enabled) {
        MapBuildWorker::Config map_config;
        map_config.full_voxel_leaf_m = full_map_voxel_size;
        map_config.store.tile_size_m = max(1.0, config.map_output_tile_size_m);
        map_config.store.voxel_leaf_m = max(
            0.001, config.map_output_tile_voxel_leaf_m);
        map_config.store.update_policy = TiledMapStore::parseUpdatePolicy(
            config.map_output_voxel_update_policy);
        map_config.store.max_memory_bytes = static_cast<size_t>(
            max(1, config.map_output_max_memory_mb)) * 1024ULL * 1024ULL;
        map_config.store.memory_policy = TiledMapStore::parseMemoryPolicy(
            config.map_output_memory_policy);
        map_config.input_queue_capacity = static_cast<size_t>(
            max(1, config.map_output_input_queue_capacity));
        map_config.input_queue_max_bytes = static_cast<size_t>(
            max(1, config.map_output_input_queue_max_mb)) * 1024ULL * 1024ULL;
        map_config.input_queue_max_points =
            map_config.input_queue_max_bytes / sizeof(PointType);
        map_config.max_tiles_per_update = static_cast<size_t>(
            max(1, config.map_output_max_tiles_per_update));
        map_config.max_points_per_update = static_cast<size_t>(
            max(1000, config.map_output_max_points_per_update));
        map_config.max_bytes_per_update =
            map_config.max_points_per_update * 256ULL;
        if (publish_map_delta) {
            const size_t max_tile_voxels_by_bytes =
                TileUpdate::maxPointCountForBytes(map_config.max_bytes_per_update);
            map_config.store.max_voxels_per_tile = std::min(
                map_config.max_points_per_update, max_tile_voxels_by_bytes);
        }
        // A compatibility /map snapshot may be as large as the bounded store.
        // Pure Tile mode never reserves that extra full-map capacity.
        map_config.output.max_pending_bytes = publish_full_map
            ? map_config.store.max_memory_bytes + map_config.max_bytes_per_update
            : map_config.max_bytes_per_update;
        map_config.output.max_pending_points =
            map_config.output.max_pending_bytes / sizeof(PointType);
        map_config.output.tile_publish_hz = static_cast<double>(
            max(1, config.map_output_tile_publish_hz));

        async_map_publisher = make_unique<MapBuildWorker>(
            std::move(map_config),
            publish_full_map
                ? MapBuildWorker::PublishCallback{
                      [&foxglove, &storage_worker, &final_bag_map_requested]
                      (PointCloudXYZI const& map, double timestamp) {
                          const bool write_final =
                              final_bag_map_requested.exchange(false);
                          if (storage_worker && storage_worker->isBagOpen() &&
                              (bag_full_map_periodic || write_final))
                          {
                              const bool accepted = storage_worker->enqueueCloud(
                                  doubleToNs(timestamp), PointCloudXYZI(map),
                                  kFastLioMapFrame, kFastLioMapTopic);
                              if (write_final && !accepted) {
                                  final_bag_map_requested = true;
                                  cerr << "[LaserMapping] Final Bag map was rejected by the "
                                          "bounded storage queue." << endl;
                              }
                          }
                          if (foxglove.getClientCount() > 0) {
                              foxglove.publishMap(map, timestamp);
                          }
                      }}
                : MapBuildWorker::PublishCallback{},
            publish_map_delta
                ? MapBuildWorker::TilePublishCallback{
                      [&foxglove, &config](const TileUpdate& tile, double timestamp) {
                           if (foxglove.getClientCount() > 0) {
                              foxglove.publishMapTile(
                                  tile, config.map_output_tile_voxel_leaf_m,
                                  timestamp);
                          }
                      }}
                : MapBuildWorker::TilePublishCallback{});
        async_map_publisher->start();
        cout << "[LaserMapping] Bounded asynchronous map pipeline enabled."
             << " mode=" << config.map_output_mode
             << " tile=" << config.map_output_tile_size_m << " m"
             << " voxel=" << config.map_output_tile_voxel_leaf_m << " m"
             << " input_queue=" << config.map_output_input_queue_capacity << endl;
    }

    // ── Data source setup ────────────────────────────────────────────
    // Shared storage for real-time adapter point accumulation
    vector<LvxPoint>    adapter_accum_points;
    double              adapter_accum_time = 0.0;
    mutex               adapter_accum_mtx;
    const double        ACCUM_FRAME_SEC = max(0.01, config.realtime_frame_sec);
    const size_t        ACCUM_POINT_THRESHOLD =
        static_cast<size_t>(max(1000, config.realtime_frame_points));
    auto playback_control_state =
        make_shared<PlaybackControlLifetimeState>();

    // Declare producers after every object captured by their callbacks. C++
    // destroys locals in reverse order, so this is a final safety net in
    // addition to the explicit stopFor() barrier below.
    unique_ptr<LvxReader>    lvx_reader;
    unique_ptr<LivoxAdapter> livox_adapter;
    unique_ptr<BagReader>    bag_reader;
    PlaybackReaderLifetimeGuard playback_reader_lifetime(
        playback_control_state);

    const auto stop_initialization_workers = [&]() noexcept {
        try {
            replay_backpressure_stop = true;
            sig_buffer.notify_all();
            playback_reader_lifetime.disarm();
            const auto deadline =
                chrono::steady_clock::now() + chrono::seconds(30);
            const auto remaining = [&]() {
                const auto now = chrono::steady_clock::now();
                if (now >= deadline) return chrono::milliseconds(0);
                return chrono::duration_cast<chrono::milliseconds>(
                    deadline - now);
            };
            const auto slice = [&](chrono::milliseconds maximum) {
                return min(remaining(), maximum);
            };
            if (lvx_reader &&
                !lvx_reader->stopFor(slice(chrono::seconds(5)))) {
                terminateForShutdownTimeout();
            }
            if (livox_adapter &&
                !livox_adapter->stopFor(slice(chrono::seconds(5)))) {
                terminateForShutdownTimeout();
            }
            if (bag_reader &&
                !bag_reader->stopFor(slice(chrono::seconds(5)))) {
                terminateForShutdownTimeout();
            }
            if (async_map_publisher &&
                !async_map_publisher->stopFor(
                    slice(chrono::seconds(10)), false)) {
                terminateForShutdownTimeout();
            }
            if (storage_worker &&
                !storage_worker->stopFor(
                    slice(chrono::seconds(10)), false)) {
                terminateForShutdownTimeout();
            }
        } catch (...) {
            terminateForShutdownTimeout();
        }
    };

    try
    {
    if (use_lvx)
    {
        lvx_reader = make_unique<LvxReader>();
        if (!lvx_reader->open(lvx_path))
        {
            stop_initialization_workers();
            cerr << "[LaserMapping] Cannot open lvx file: " << lvx_path << endl;
            return;
        }
        playback_reader_lifetime.setLvxReader(lvx_reader.get());

        lvx_reader->setFrameCallback(
            [&realtime_foxglove, &foxglove](const vector<LvxPoint> &points,
               const vector<ImuData> &imus,
               double timestamp)
            {
                PointCloudXYZI::Ptr cloud = preprocessLivoxPoints(points, timestamp);

                if (!cloud->empty())
                    pushLidarFrame(cloud, timestamp);

                for (auto &imu : imus) {
                    if (foxglove.getClientCount() > 0) {
                        realtime_foxglove.tryEnqueueImu(imu);
                    }
                    pushImuSample(imu);
                }
            });

    }
    else if (use_bag)
    {
        bag_reader = make_unique<BagReader>();
        if (!bag_reader->open(bag_path))
        {
            stop_initialization_workers();
            cerr << "[LaserMapping] Cannot open bag file: " << bag_path << endl;
            return;
        }
        playback_reader_lifetime.setBagReader(bag_reader.get());

        bag_reader->setFrameCallback(
            [&realtime_foxglove, &foxglove](const vector<LvxPoint> &points,
               const vector<ImuData> &imus,
               double timestamp)
            {
                if (!points.empty()) {
                    PointCloudXYZI::Ptr cloud = preprocessLivoxPoints(points, timestamp);
                    if (!cloud->empty())
                        pushLidarFrame(cloud, timestamp);
                }
                for (auto &imu : imus) {
                    if (foxglove.getClientCount() > 0) {
                        realtime_foxglove.tryEnqueueImu(imu);
                    }
                    pushImuSample(imu);
                }
            });

    }
    else
    {
        livox_adapter = make_unique<LivoxAdapter>();
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
            [&realtime_foxglove, &foxglove](const ImuData &imu) {
                if (foxglove.getClientCount() > 0) {
                    realtime_foxglove.tryEnqueueImu(imu);
                }
                pushImuSample(imu);
            });

        if (!livox_adapter->init())
        {
            stop_initialization_workers();
            cerr << "[LaserMapping] LivoxAdapter init failed." << endl;
            return;
        }

    }

    // ── State & covariance init ──────────────────────────────────────
    }
    catch (...)
    {
        stop_initialization_workers();
        throw;
    }

    if (use_lvx && lvx_reader)
    {
        const uint64_t end_ns = std::max<uint64_t>(lvx_reader->getDurationNs(), 1);
        foxglove.setPlaybackControl(
            0, end_ns,
            [playback_control_state]
            (const FoxglovePublisher::PlaybackControlRequest& request) {
                lock_guard<mutex> reader_lock(
                    playback_control_state->reader_mutex);
                LvxReader* reader = playback_control_state->lvx_reader;
                if (!reader) {
                    return FoxglovePublisher::PlaybackState{
                        FoxglovePublisher::PlaybackStatus::Ended,
                        playback_control_state->current_time_ns.load(),
                        static_cast<float>(
                            playback_control_state->speed.load()),
                        false};
                }
                const double speed = request.speed > 0.0f ? request.speed : 1.0f;
                playback_control_state->speed = speed;
                reader->setSpeed(speed);

                if (request.seek_time_ns) {
                    // Seek is temporarily disabled because Foxglove clears the rendered map on seek.
                } else if (request.command == FoxglovePublisher::PlaybackCommand::Pause) {
                    reader->pause();
                    if (!playback_control_state->paused.exchange(true)) {
                        playback_control_state->pause_drain_pending = true;
                    }
                } else {
                    reader->resume();
                    playback_control_state->paused = false;
                    playback_control_state->pause_drain_pending = false;
                }

                return FoxglovePublisher::PlaybackState{
                    playback_control_state->paused
                        ? FoxglovePublisher::PlaybackStatus::Paused
                        : FoxglovePublisher::PlaybackStatus::Playing,
                    playback_control_state->current_time_ns.load(),
                    static_cast<float>(playback_control_state->speed.load()),
                    false};
            });
    }
    else if (use_bag && bag_reader && bag_reader->getEndTimeNs() > bag_reader->getStartTimeNs())
    {
        foxglove.setPlaybackControl(
            bag_reader->getStartTimeNs(), bag_reader->getEndTimeNs(),
            [playback_control_state]
            (const FoxglovePublisher::PlaybackControlRequest& request) {
                lock_guard<mutex> reader_lock(
                    playback_control_state->reader_mutex);
                BagReader* reader = playback_control_state->bag_reader;
                if (!reader) {
                    return FoxglovePublisher::PlaybackState{
                        FoxglovePublisher::PlaybackStatus::Ended,
                        playback_control_state->current_time_ns.load(),
                        static_cast<float>(
                            playback_control_state->speed.load()),
                        false};
                }
                const double speed = request.speed > 0.0f ? request.speed : 1.0f;
                playback_control_state->speed = speed;
                reader->setSpeed(speed);

                if (request.seek_time_ns) {
                    // Seek is temporarily disabled because Foxglove clears the rendered map on seek.
                } else if (request.command == FoxglovePublisher::PlaybackCommand::Pause) {
                    reader->pause();
                    if (!playback_control_state->paused.exchange(true)) {
                        playback_control_state->pause_drain_pending = true;
                    }
                } else {
                    reader->resume();
                    playback_control_state->paused = false;
                    playback_control_state->pause_drain_pending = false;
                }

                return FoxglovePublisher::PlaybackState{
                    playback_control_state->paused
                        ? FoxglovePublisher::PlaybackStatus::Paused
                        : FoxglovePublisher::PlaybackStatus::Playing,
                    playback_control_state->current_time_ns.load(),
                    static_cast<float>(playback_control_state->speed.load()),
                    false};
            });
    }

    if (!foxglove.start("127.0.0.1", 8765, foxglove_backlog_size))
    {
        cerr << "[LaserMapping] Failed to start Foxglove publisher." << endl;
    }
    else
    {
        realtime_foxglove.start();
        cout << "[LaserMapping] Foxglove WebSocket: ws://localhost:"
             << foxglove.getPort() << endl;

        if (publish_map_delta) {
            cout << "[LaserMapping] Persistent tiled Scene topic enabled: "
                 << kFastLioMapTilesTopic << endl;
        }
    }

    // Start at zero so a client that connects immediately after server start
    // still triggers the same full/tile resynchronization as a later reconnect.
    uint64_t last_playback_client_generation = 0;

    if (use_lvx && lvx_reader)
    {
        lvx_reader->play(playback_control_state->speed.load());
        if (foxglove.getClientCount() > 0) {
            realtime_foxglove.tryEnqueuePlayback(FoxglovePublisher::PlaybackState{
                FoxglovePublisher::PlaybackStatus::Playing,
                playback_control_state->current_time_ns.load(),
                static_cast<float>(playback_control_state->speed.load()),
                false});
        }
        cout << "[LaserMapping] LVX playback started." << endl;
    }
    else if (use_bag && bag_reader)
    {
        bag_reader->play(playback_control_state->speed.load());
        if (foxglove.getClientCount() > 0) {
            realtime_foxglove.tryEnqueuePlayback(FoxglovePublisher::PlaybackState{
                FoxglovePublisher::PlaybackStatus::Playing,
                playback_control_state->current_time_ns.load(),
                static_cast<float>(playback_control_state->speed.load()),
                false});
        }
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
    uint64_t bag_path_points_trimmed = 0;
    double T1, T2, s_plot, s_plot2, s_plot3, s_plot4, s_plot5, s_plot6;
    const auto foxglove_publish_start = chrono::steady_clock::now();
    auto last_foxglove_cloud_pub =
        foxglove_publish_start - chrono::milliseconds(foxglove_cloud_interval_ms);
    auto last_foxglove_map_pub =
        foxglove_publish_start - chrono::milliseconds(
            full_map_publish_interval_ms);
    auto last_foxglove_path_pub =
        foxglove_publish_start - chrono::milliseconds(foxglove_path_interval_ms);
    auto last_foxglove_control_pub =
        foxglove_publish_start - chrono::milliseconds(foxglove_control_interval_ms);
    size_t process_rss_bytes = currentProcessRssBytes();
    size_t process_peak_rss_bytes = process_rss_bytes;
    BoundedTimingWindow<> map_accumulate_timing;
    BoundedTimingWindow<> map_schedule_timing;
    bool input_failure_reported = false;

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
        // A new client needs control state and a map resync.  This also covers
        // live Livox input, where there is no playback state to announce.
        if (foxglove.isRunning()) {
            const uint64_t client_generation =
                foxglove.getClientConnectionGeneration();
            if (client_generation != last_playback_client_generation &&
                foxglove.getClientCount() > 0) {
                last_playback_client_generation = client_generation;
                if (use_lvx || use_bag) {
                    realtime_foxglove.tryEnqueuePlayback(
                        FoxglovePublisher::PlaybackState{
                        playback_control_state->paused.load()
                            ? FoxglovePublisher::PlaybackStatus::Paused
                            : FoxglovePublisher::PlaybackStatus::Playing,
                        playback_control_state->current_time_ns.load(),
                        static_cast<float>(
                            playback_control_state->speed.load()),
                        false});
                }
                if (async_map_publisher && async_map_publisher->isRunning()) {
                    if (publish_full_map) {
                        async_map_publisher->request(
                            static_cast<double>(
                                playback_control_state->current_time_ns.load()) /
                                1e9);
                    }
                    if (publish_map_delta) {
                        async_map_publisher->requestTileResync();
                    }
                }
            }
        }

        // Check EOF for lvx/bag — but only exit after all buffered data is processed
        const bool lvx_eof = use_lvx && lvx_reader && lvx_reader->isEOF();
        const bool bag_eof = use_bag && bag_reader && bag_reader->isEOF();
        const bool lvx_failed =
            use_lvx && lvx_reader && lvx_reader->hasPlaybackFailure();
        const bool bag_failed =
            use_bag && bag_reader && bag_reader->hasPlaybackFailure();
        const bool livox_failed =
            livox_adapter && livox_adapter->hasCallbackFailure();
        const bool lvx_terminal = isPlaybackTerminal(lvx_eof, lvx_failed);
        const bool bag_terminal = isPlaybackTerminal(bag_eof, bag_failed);
        const bool playback_terminal = lvx_terminal || bag_terminal;

        if ((lvx_failed || bag_failed || livox_failed) &&
            !input_failure_reported)
        {
            const char* source =
                lvx_failed ? "lvx" : (bag_failed ? "bag" : "livox");
            uint64_t failure_count = 0;
            string error = "sdk_callback_exception";
            try {
                if (lvx_failed) {
                    const auto stats = lvx_reader->playbackStats();
                    failure_count = stats.failures;
                    error = stats.last_error;
                } else if (bag_failed) {
                    const auto stats = bag_reader->playbackStats();
                    failure_count = stats.failures;
                    error = stats.last_error;
                } else {
                    failure_count = livox_adapter->callbackFailureCount();
                }
                error = sanitizePlaybackErrorForLog(error);
            } catch (...) {
                error = "failure_snapshot_unavailable";
            }

            ostringstream failure_log;
            failure_log << "event=playback_failed"
                        << " source=" << source
                        << " failures=" << failure_count
                        << " error=" << error;
            writeRuntimeLog(failure_log.str());
            cerr << "[LaserMapping] Input producer failed: source="
                 << source << " error=" << error << endl;
            input_failure_reported = true;
        }

        // Live input has no finite tail to drain. Once its callback fuse
        // trips, enter the same bounded shutdown path immediately.
        if (livox_failed) break;

        if (!playback_terminal && (use_lvx || use_bag) &&
            playback_control_state->paused.load())
        {
            if (playback_control_state->pause_drain_pending.exchange(false) &&
                async_map_publisher && async_map_publisher->isRunning())
            {
                if (publish_full_map && foxglove.getClientCount() > 0) {
                    async_map_publisher->request(
                        static_cast<double>(
                            playback_control_state->current_time_ns.load()) /
                            1e9);
                }
                async_map_publisher->flush();
            }

            this_thread::sleep_for(chrono::milliseconds(20));
            continue;
        }

        if (playback_terminal)
        {
            size_t tail_frames_dropped = 0;
            size_t tail_imu_remaining = 0;
            double tail_lidar_end = 0.0;
            double tail_last_imu = 0.0;
            string tail_drop_reason;
            bool playback_finished = false;
            bool drop_only_front_frame = false;
            {
                lock_guard<mutex> lock(mtx_buffer);
                if (lidar_buffer.empty()) {
                    playback_finished = true;
                } else if (time_buffer.empty()) {
                    tail_drop_reason = "missing_lidar_timestamp";
                } else if (!lidar_buffer.front() ||
                           lidar_buffer.front()->points.empty()) {
                    // sync_packages cannot derive an end time for an empty
                    // cloud, and terminal input guarantees it cannot be repaired.
                    tail_drop_reason = "empty_lidar_frame";
                    drop_only_front_frame = true;
                    tail_frames_dropped = 1;
                    tail_imu_remaining = imu_buffer.size();
                    lidar_buffer.pop_front();
                    time_buffer.pop_front();
                    playback_finished = lidar_buffer.empty();
                    sig_buffer.notify_all();
                } else {
                    double max_offset_ms = 0.0;
                    for (const auto& point : lidar_buffer.front()->points) {
                        max_offset_ms = max(
                            max_offset_ms,
                            static_cast<double>(point.curvature));
                    }
                    tail_lidar_end =
                        time_buffer.front() + max_offset_ms / 1000.0;
                    tail_last_imu = last_timestamp_imu;
                    tail_imu_remaining = imu_buffer.size();
                    if (imu_buffer.empty()) {
                        tail_drop_reason = "no_final_imu";
                    } else if (!hasImuCoverageForLidarFrame(
                                   last_timestamp_imu, tail_lidar_end))
                    {
                        // No producer can add another IMU sample after a
                        // terminal reader state.
                        // Keeping this frame would make sync_packages wait
                        // forever, so explicitly account for the unusable tail.
                        tail_drop_reason = "no_final_imu_coverage";
                    }
                }

                if (!tail_drop_reason.empty() && !drop_only_front_frame) {
                    tail_frames_dropped = lidar_buffer.size();
                    lidar_buffer.clear();
                    time_buffer.clear();
                    imu_buffer.clear();
                    playback_finished = true;
                    sig_buffer.notify_all();
                }
            }

            if (!tail_drop_reason.empty()) {
                ostringstream tail_log;
                tail_log << fixed << setprecision(6)
                         << "event=playback_tail_dropped"
                         << " source=" << (lvx_terminal ? "lvx" : "bag")
                         << " reason=" << tail_drop_reason
                         << " lidar_frames=" << tail_frames_dropped
                         << " imu_samples=" << tail_imu_remaining
                         << " lidar_end=" << tail_lidar_end
                         << " last_imu=" << tail_last_imu;
                writeRuntimeLog(tail_log.str());
                cerr << "[LaserMapping] Dropped " << tail_frames_dropped
                     << " unprocessable tail LiDAR frame(s) at terminal input: "
                     << tail_drop_reason << endl;
            }
            if (playback_finished) {
                const bool failed = lvx_failed || bag_failed;
                cout << "[LaserMapping] "
                     << (lvx_terminal ? "LVX" : "Bag")
                     << (failed
                             ? " playback stopped after reader failure."
                             : " playback finished (EOF).")
                     << endl;
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
            Nearest_Point_Distances.resize(feats_down_size);
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
        playback_control_state->current_time_ns = pub_time_ns;
        const bool write_bag_output =
            storage_worker && storage_worker->isBagOpen();
        const bool publish_foxglove = realtime_foxglove.isRunning() &&
            foxglove.getClientCount() > 0;
        const auto publish_wall_now = chrono::steady_clock::now();
        const bool publish_foxglove_cloud =
            scan_publish_en && publish_foxglove &&
            chrono::duration_cast<chrono::milliseconds>(
                publish_wall_now - last_foxglove_cloud_pub).count() >=
                foxglove_cloud_interval_ms;

        // Publish odometry
        V3D pos_v3d(state_point.pos[0], state_point.pos[1], state_point.pos[2]);

        if (write_bag_output) {
            storage_worker->enqueueOdometryTf(pub_time_ns, pos_v3d, geoQuat);
        }
        if (publish_foxglove) {
            realtime_foxglove.tryEnqueuePose(pos_v3d, geoQuat, pub_time);
        }

        // Build the registered cloud only when a consumer needs this frame.
        // Map accumulation owns its separate sparse world-frame transform.
        if (scan_publish_en &&
            (write_bag_output || publish_foxglove_cloud))
        {
            PointCloudXYZI cloud_world;
            PointCloudXYZI::Ptr publish_cloud =
                dense_publish_en ? feats_undistort : feats_down_body;
            cloud_world.reserve(publish_cloud->points.size());
            PointType pt_world;
            for (size_t i = 0; i < publish_cloud->points.size(); i++)
            {
                pointBodyToWorld(&(publish_cloud->points[i]), &pt_world);
                cloud_world.push_back(pt_world);
            }
            cloud_world.width =
                static_cast<uint32_t>(cloud_world.points.size());
            cloud_world.height = 1;
            cloud_world.is_dense = true;
            if (write_bag_output) {
                if (publish_foxglove_cloud) {
                    storage_worker->enqueueCloud(
                        pub_time_ns, PointCloudXYZI(cloud_world),
                        kFastLioMapFrame, kFastLioRegisteredCloudTopic);
                } else {
                    storage_worker->enqueueCloud(
                        pub_time_ns, std::move(cloud_world),
                        kFastLioMapFrame, kFastLioRegisteredCloudTopic);
                }
            }
            if (publish_foxglove_cloud) {
                realtime_foxglove.tryEnqueueCloud(
                    std::move(cloud_world), pub_time);
                last_foxglove_cloud_pub = publish_wall_now;
            }
        }

        double map_accumulate_ms = 0.0;
        double map_schedule_ms = 0.0;
        int effective_full_map_interval_ms = full_map_publish_interval_ms;
        if (map_pipeline_enabled) {
            const auto map_accumulate_begin = chrono::steady_clock::now();
            PointCloudXYZI map_frame_world;

            // The dense registered cloud is intended for visualization only.
            // Reuse the bounded SLAM input for the accumulated map so HD scans
            // do not hash tens of thousands of redundant points on the main thread.
            map_frame_world.reserve(feats_down_body->points.size());
            PointType pt_world;
            for (const auto& point_body : feats_down_body->points) {
                pointBodyToWorld(&point_body, &pt_world);
                map_frame_world.push_back(pt_world);
            }
            map_frame_world.width =
                static_cast<uint32_t>(map_frame_world.points.size());
            map_frame_world.height = 1;
            map_frame_world.is_dense = true;

            if (async_map_publisher && async_map_publisher->isRunning()) {
                PointType current_position;
                current_position.x = static_cast<float>(state_point.pos(0));
                current_position.y = static_cast<float>(state_point.pos(1));
                current_position.z = static_cast<float>(state_point.pos(2));
                async_map_publisher->enqueueFrame(
                    std::move(map_frame_world), pub_time, &current_position);
            }
            map_accumulate_ms = chrono::duration<double, milli>(
                chrono::steady_clock::now() - map_accumulate_begin).count();
            map_accumulate_timing.record(map_accumulate_ms);
        }

        const auto map_schedule_begin = chrono::steady_clock::now();
        if (publish_full_map) {
            if (async_map_publisher) {
                const auto timing_stats = async_map_publisher->stats();
                const double last_full_output_ms =
                    timing_stats.last_full_snapshot_ms +
                    timing_stats.output.last_full_publish_ms;
                if (last_full_output_ms > 0.0) {
                    effective_full_map_interval_ms = max(
                        full_map_publish_interval_ms,
                        static_cast<int>(ceil(last_full_output_ms * 2.0)));
                }
            }
            const bool full_map_due = periodic_full_map_enabled &&
                chrono::duration_cast<chrono::milliseconds>(
                    publish_wall_now - last_foxglove_map_pub).count() >=
                effective_full_map_interval_ms;
            const bool async_foxglove_map =
                publish_foxglove && async_map_publisher &&
                async_map_publisher->isRunning();
            const bool sync_bag_map = write_bag_output && bag_full_map_periodic;

            if (full_map_due && (async_foxglove_map || sync_bag_map)) {
                if (async_map_publisher && async_map_publisher->isRunning()) {
                    async_map_publisher->request(pub_time);
                }
                last_foxglove_map_pub = publish_wall_now;
            }
        }
        map_schedule_ms = chrono::duration<double, milli>(
            chrono::steady_clock::now() - map_schedule_begin).count();
        map_schedule_timing.record(map_schedule_ms);

        // Publish path
        if (path_en)
        {
            if (write_bag_output) {
                if (path_vec.size() >= bag_path_max_points) {
                    const size_t trim_count = min(
                        path_vec.size(),
                        max<size_t>(1, bag_path_max_points / 10));
                    path_vec.erase(
                        path_vec.begin(),
                        path_vec.begin() +
                            static_cast<std::ptrdiff_t>(trim_count));
                    bag_path_points_trimmed += trim_count;
                }
                path_vec.push_back(pos_v3d);
                storage_worker->enqueuePath(pub_time_ns, path_vec, kFastLioMapFrame);
            }
            const bool publish_foxglove_path =
                publish_foxglove &&
                chrono::duration_cast<chrono::milliseconds>(
                    publish_wall_now - last_foxglove_path_pub).count() >=
                    foxglove_path_interval_ms;
            if (realtime_foxglove.isRunning()) {
                realtime_foxglove.tryEnqueuePathPoint(
                    pos_v3d, pub_time, publish_foxglove_path);
            }
            if (publish_foxglove_path) {
                last_foxglove_path_pub = publish_wall_now;
            }
        }

        const bool publish_foxglove_control =
            publish_foxglove &&
            chrono::duration_cast<chrono::milliseconds>(
                publish_wall_now - last_foxglove_control_pub).count() >=
                foxglove_control_interval_ms;
        if (publish_foxglove_control) {
            realtime_foxglove.tryEnqueueTime(pub_time);
            last_foxglove_control_pub = publish_wall_now;
        }

        T2 = chrono::duration<double>(chrono::steady_clock::now().time_since_epoch()).count();
        s_plot6 = (T2 - T1) * 1000.0;

        // ── 7. PCD saving ───────────────────────────────────────────
        if (pcd_save_en && storage_worker && storage_worker->isRunning() &&
            storage_worker->isPcdEnabled())
        {
            PointCloudXYZI pcd_frame;
            pcd_frame.reserve(feats_undistort->points.size());
            PointType pt_world;
            for (size_t i = 0; i < feats_undistort->points.size(); i++)
            {
                pointBodyToWorld(&(feats_undistort->points[i]), &pt_world);
                pcd_frame.push_back(pt_world);
            }
            pcd_frame.width = static_cast<uint32_t>(pcd_frame.size());
            pcd_frame.height = 1;
            pcd_frame.is_dense = true;
            storage_worker->enqueuePcd(std::move(pcd_frame));
        }

        // ── 8. Timing log ───────────────────────────────────────────
        auto t_loop_end = chrono::steady_clock::now();
        double loop_ms = chrono::duration<double, milli>(t_loop_end - t_loop_start).count();

        if (runtime_pos_log)
        {
            printf("[Frame %04d] sync:%.1f imu:%.1f ds:%.1f ekf:%.1f map:%.1f map_acc:%.1f pub:%.1f | total:%.1f ms | pts:%d tree:%d\n",
                   frame_id, s_plot, s_plot2, s_plot3, s_plot4, s_plot5,
                   map_accumulate_ms, s_plot6, loop_ms,
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

        MapBuildWorker::Stats async_map_stats;
        if (async_map_publisher) {
            async_map_stats = async_map_publisher->stats();
        }
        StorageWorker::Stats storage_stats;
        if (storage_worker) storage_stats = storage_worker->stats();
        RealtimeFoxgloveWorker::Stats realtime_foxglove_stats;
        if (realtime_foxglove.isRunning()) {
            realtime_foxglove_stats = realtime_foxglove.stats();
        }
        size_t input_lidar_queue_frames = 0;
        size_t input_imu_queue_samples = 0;
        {
            lock_guard<mutex> lock(mtx_buffer);
            input_lidar_queue_frames = lidar_buffer.size();
            input_imu_queue_samples = imu_buffer.size();
        }
        if (frame_id % 20 == 0) {
            process_rss_bytes = currentProcessRssBytes();
            process_peak_rss_bytes = max(process_peak_rss_bytes, process_rss_bytes);
        }
        const auto add_memory_saturated = [](size_t left, size_t right) {
            return right > numeric_limits<size_t>::max() - left
                ? numeric_limits<size_t>::max() : left + right;
        };
        const size_t storage_pcd_memory_bytes = add_memory_saturated(
            storage_stats.pcd_pending_bytes,
            storage_stats.pcd_scratch_bytes);
        size_t queue_memory_bytes = async_map_stats.input_queue_bytes;
        queue_memory_bytes = add_memory_saturated(
            queue_memory_bytes, async_map_stats.output.current_bytes);
        queue_memory_bytes = add_memory_saturated(
            queue_memory_bytes, async_map_stats.output.inflight_bytes);
        queue_memory_bytes = add_memory_saturated(
            queue_memory_bytes, realtime_foxglove_stats.current_bytes);
        queue_memory_bytes = add_memory_saturated(
            queue_memory_bytes, realtime_foxglove_stats.inflight_bytes);
        queue_memory_bytes = add_memory_saturated(
            queue_memory_bytes, realtime_foxglove_stats.path_history_bytes);
        queue_memory_bytes = add_memory_saturated(
            queue_memory_bytes, storage_stats.hard_usage_bytes);
        queue_memory_bytes = add_memory_saturated(
            queue_memory_bytes, storage_pcd_memory_bytes);

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
                 << " map_accumulate_ms=" << map_accumulate_ms
                 << " map_schedule_ms=" << map_schedule_ms
                 << " map_effective_interval_ms=" << effective_full_map_interval_ms
                 << " map_async_requested=" << async_map_stats.full_requested
                 << " map_async_published=" << async_map_stats.output.full_published
                 << " map_async_coalesced=" << async_map_stats.full_coalesced
                 << " tile_resync_requested="
                 << async_map_stats.tile_resync_requested
                 << " tile_resync_on_failure="
                 << async_map_stats.tile_resync_on_failure
                 << " tile_resync_completed="
                 << async_map_stats.tile_resync_completed
                 << " map_async_snapshot_ms=" << async_map_stats.last_full_snapshot_ms
                 << " map_async_send_ms=" << async_map_stats.output.last_full_publish_ms
                 << " map_async_points=" << async_map_stats.last_full_points
                 << " map_accumulator_points=" << async_map_stats.store.voxel_count
                 << " map_frames_enqueued=" << async_map_stats.frames_enqueued
                 << " map_frames_built=" << async_map_stats.frames_built
                 << " map_frames_dropped=" << async_map_stats.frames_dropped
                 << " map_input_incomplete=" << async_map_stats.input_incomplete
                 << " map_input_points_dropped="
                 << async_map_stats.input_points_dropped
                 << " map_input_bytes_dropped="
                 << async_map_stats.input_bytes_dropped
                 << " map_input_tile_changes_enqueued="
                 << async_map_stats.input_tile_changes_enqueued
                 << " map_input_tile_changes_merged="
                 << async_map_stats.input_tile_changes_merged
                 << " map_input_tile_changes_dropped="
                 << async_map_stats.input_tile_changes_dropped
                 << " map_input_tile_tasks_evicted="
                 << async_map_stats.input_tile_tasks_evicted
                 << " map_input_resync_requested="
                 << async_map_stats.input_resync_requested
                 << " map_incomplete="
                 << ((async_map_stats.incomplete ||
                      async_map_stats.store.incomplete) ? 1 : 0)
                 << " map_accumulation_stopped="
                 << (async_map_stats.store.accumulation_stopped ? 1 : 0)
                 << " map_points_rejected_memory="
                 << async_map_stats.store.points_rejected_memory
                 << " map_points_rejected_invalid="
                 << async_map_stats.store.points_rejected_invalid
                 << " map_frame_queue_current=" << async_map_stats.input_queue_tasks
                 << " map_frame_queue_points=" << async_map_stats.input_queue_points
                 << " map_frame_queue_bytes=" << async_map_stats.input_queue_bytes
                 << " map_frame_queue_max=" << async_map_stats.max_input_queue_tasks
                 << " map_frame_queue_max_points="
                 << async_map_stats.max_input_queue_points
                 << " map_frame_queue_max_bytes="
                 << async_map_stats.max_input_queue_bytes
                 << " map_frame_points=" << async_map_stats.last_frame_points
                 << " map_frame_build_ms=" << async_map_stats.last_build_ms
                 << " tile_count=" << async_map_stats.store.tile_count
                 << " tile_dirty=" << async_map_stats.store.dirty_count
                 << " tile_evicted=" << async_map_stats.store.tiles_evicted
                 << " tile_pending_deletions="
                 << async_map_stats.store.pending_deletion_count
                 << " tile_deletions_backpressured="
                 << async_map_stats.store.deletions_backpressured
                 << " tile_updates=" << async_map_stats.tile_updates
                 << " tile_extract_ms=" << async_map_stats.last_tile_extract_ms
                 << " map_memory_bytes=" << async_map_stats.store.estimated_bytes
                 << " foxglove_queue_tasks=" << async_map_stats.output.current_tasks
                 << " foxglove_queue_bytes=" << async_map_stats.output.current_bytes
                 << " foxglove_overwritten=" << async_map_stats.output.full_overwritten
                 << " tile_merged=" << async_map_stats.output.tiles_merged
                 << " tile_dropped=" << async_map_stats.output.tiles_dropped
                 << " tile_retried=" << async_map_stats.output.tiles_retried
                 << " realtime_queue_tasks="
                 << realtime_foxglove_stats.current_tasks
                 << " realtime_queue_points="
                 << realtime_foxglove_stats.current_points
                 << " realtime_queue_bytes="
                 << realtime_foxglove_stats.current_bytes
                 << " realtime_path_history_bytes="
                 << realtime_foxglove_stats.path_history_bytes
                 << " realtime_overwritten="
                 << (realtime_foxglove_stats.playback.overwritten +
                     realtime_foxglove_stats.time.overwritten +
                     realtime_foxglove_stats.pose.overwritten +
                     realtime_foxglove_stats.imu.overwritten +
                     realtime_foxglove_stats.cloud.overwritten +
                     realtime_foxglove_stats.path.overwritten)
                 << " realtime_dropped="
                 << (realtime_foxglove_stats.playback.dropped +
                     realtime_foxglove_stats.time.dropped +
                     realtime_foxglove_stats.pose.dropped +
                     realtime_foxglove_stats.imu.dropped +
                     realtime_foxglove_stats.cloud.dropped +
                     realtime_foxglove_stats.path.dropped)
                 << " realtime_failures=" << realtime_foxglove_stats.failures
                  << " storage_queue_tasks=" << storage_stats.queue_tasks
                  << " storage_queue_bytes=" << storage_stats.queue_bytes
                  << " storage_inflight_tasks=" << storage_stats.inflight_tasks
                  << " storage_inflight_points=" << storage_stats.inflight_points
                  << " storage_inflight_bytes=" << storage_stats.inflight_bytes
                  << " storage_hard_usage_bytes="
                  << storage_stats.hard_usage_bytes
                  << " storage_dropped=" << storage_stats.tasks_dropped
                  << " storage_overwritten=" << storage_stats.tasks_overwritten
                  << " bag_path_points=" << path_vec.size()
                  << " bag_path_points_trimmed=" << bag_path_points_trimmed
                 << " storage_write_ms=" << storage_stats.last_write_ms
                  << " pcd_queue_points=" << storage_stats.pcd_pending_points
                  << " pcd_queue_bytes=" << storage_stats.pcd_pending_bytes
                  << " pcd_pending_capacity_points="
                  << storage_stats.pcd_pending_capacity_points
                  << " pcd_scratch_bytes=" << storage_stats.pcd_scratch_bytes
                  << " storage_pcd_memory_bytes=" << storage_pcd_memory_bytes
                  << " queue_memory_bytes=" << queue_memory_bytes
                  << " process_rss_bytes=" << process_rss_bytes
                  << " process_peak_rss_bytes=" << process_peak_rss_bytes
                 << " map_worker_busy=" << (async_map_stats.busy ? 1 : 0)
                 << " input_lidar_queue_frames="
                 << input_lidar_queue_frames
                 << " input_lidar_queue_high_water="
                 << input_lidar_queue_high_water.load()
                 << " input_lidar_frames_dropped="
                 << input_lidar_frames_dropped.load()
                 << " input_imu_queue_samples="
                 << input_imu_queue_samples
                 << " input_imu_queue_high_water="
                 << input_imu_queue_high_water.load()
                 << " input_imu_samples_dropped="
                 << input_imu_samples_dropped.load()
                 << " total_ms=" << loop_ms
                 << " points=" << feats_down_size
                 << " tree_size=" << ikdtree.size()
                 << setprecision(9)
                 << " pos=(" << state_point.pos(0) << ","
                 << state_point.pos(1) << ","
                 << state_point.pos(2) << ")"
                 << " quat=(" << geoQuat.w() << ","
                 << geoQuat.x() << ","
                 << geoQuat.y() << ","
                 << geoQuat.z() << ")";
        writeRuntimeLog(log_line.str());

        frame_id++;

        // Throttle slightly so playback thread doesn't run too far ahead
        if (use_lvx || use_bag)
            this_thread::sleep_for(chrono::milliseconds(1));
    }

    // ══════════════════════════════════════════════════════════════════
    //  Shutdown
    // ══════════════════════════════════════════════════════════════════
    const auto shutdown_deadline =
        chrono::steady_clock::now() + chrono::seconds(30);
    ShutdownWatchdog shutdown_watchdog(shutdown_deadline);
    if (!shutdown_watchdog.start()) terminateForShutdownTimeout();

    cout << "\n[LaserMapping] Shutting down..." << endl;

    const uint64_t final_time_ns =
        playback_control_state->current_time_ns.load();
    const double final_time = static_cast<double>(final_time_ns) / 1e9;
    const auto remaining_shutdown = [&]() {
        const auto now = chrono::steady_clock::now();
        if (now >= shutdown_deadline) return chrono::milliseconds(0);
        return chrono::duration_cast<chrono::milliseconds>(
            shutdown_deadline - now);
    };
    const auto shutdown_slice = [&](chrono::milliseconds maximum) {
        return min(remaining_shutdown(), maximum);
    };

    // Stop producers before draining consumers so no new task can race the
    // shared shutdown deadline.
    replay_backpressure_stop = true;
    sig_buffer.notify_all();
    playback_reader_lifetime.disarm();
    if (lvx_reader &&
        !lvx_reader->stopFor(shutdown_slice(chrono::seconds(5))))
    {
        terminateForShutdownTimeout();
    }
    if (livox_adapter &&
        !livox_adapter->stopFor(shutdown_slice(chrono::seconds(5))))
    {
        terminateForShutdownTimeout();
    }
    if (bag_reader &&
        !bag_reader->stopFor(shutdown_slice(chrono::seconds(5))))
    {
        terminateForShutdownTimeout();
    }

    {
        lock_guard<mutex> lock(mtx_buffer);
        ostringstream input_summary;
        input_summary << "event=input_queue_summary"
                      << " lidar_current=" << lidar_buffer.size()
                      << " lidar_capacity="
                      << (replay_backpressure_enabled.load()
                              ? MAX_REPLAY_LIDAR_QUEUE
                              : MAX_LIVE_LIDAR_QUEUE)
                      << " lidar_high_water="
                      << input_lidar_queue_high_water.load()
                      << " lidar_dropped="
                      << input_lidar_frames_dropped.load()
                      << " imu_current=" << imu_buffer.size()
                      << " imu_capacity=" << MAX_IMU_QUEUE
                      << " imu_high_water="
                      << input_imu_queue_high_water.load()
                      << " imu_dropped="
                      << input_imu_samples_dropped.load();
        writeRuntimeLog(input_summary.str());
    }
    // Keep potentially expensive input-file close and message-buffer release
    // inside the watchdog window. Playback callbacks were disarmed before the
    // producer stop barrier, so no Foxglove request can reach these objects.
    lvx_reader.reset();
    livox_adapter.reset();
    bag_reader.reset();

    if ((use_lvx || use_bag) && foxglove.getClientCount() > 0 &&
        realtime_foxglove.isRunning())
    {
        realtime_foxglove.tryEnqueuePlayback(FoxglovePublisher::PlaybackState{
            FoxglovePublisher::PlaybackStatus::Ended,
            playback_control_state->current_time_ns.load(),
            static_cast<float>(playback_control_state->speed.load()),
            false});
    }

    if (async_map_publisher && async_map_publisher->isRunning() &&
        publish_full_map &&
        (foxglove.getClientCount() > 0 ||
         (storage_worker && storage_worker->isBagOpen())))
    {
        if (storage_worker && storage_worker->isBagOpen()) {
            final_bag_map_requested = true;
        }
        async_map_publisher->request(final_time);
    }

    if (realtime_foxglove.isRunning()) {
        if (!realtime_foxglove.stopFor(
                shutdown_slice(chrono::seconds(3)), true)) {
            terminateForShutdownTimeout();
        }
        const auto realtime_stats = realtime_foxglove.stats();
        ostringstream realtime_log;
        realtime_log << fixed << setprecision(3)
                     << "event=realtime_foxglove_summary"
                     << " queue_max_tasks=" << realtime_stats.max_tasks
                     << " queue_max_points=" << realtime_stats.max_points
                     << " queue_max_bytes=" << realtime_stats.max_bytes
                     << " failures=" << realtime_stats.failures
                     << " cancelled=" << realtime_stats.tasks_cancelled
                     << " stop_timeouts=" << realtime_stats.stop_timeouts
                     << " pose_published=" << realtime_stats.pose.published
                     << " pose_overwritten=" << realtime_stats.pose.overwritten
                     << " cloud_published=" << realtime_stats.cloud.published
                     << " cloud_overwritten=" << realtime_stats.cloud.overwritten
                     << " cloud_dropped=" << realtime_stats.cloud.dropped
                     << " path_published=" << realtime_stats.path.published
                     << " path_overwritten=" << realtime_stats.path.overwritten
                     << " path_points_received="
                     << realtime_stats.path_points_received
                     << " path_points_dropped="
                     << realtime_stats.path_points_dropped
                     << " path_points_trimmed="
                     << realtime_stats.path_points_trimmed
                     << " path_history_bytes="
                     << realtime_stats.path_history_bytes
                     << " control_overwritten="
                     << (realtime_stats.time.overwritten +
                         realtime_stats.playback.overwritten)
                     << " imu_overwritten=" << realtime_stats.imu.overwritten;
        writeRuntimeLog(realtime_log.str());
    }

    MapBuildWorker::TimingStats final_map_timing;
    if (async_map_publisher) {
        if (!async_map_publisher->stopFor(
                shutdown_slice(chrono::seconds(10)), true)) {
            terminateForShutdownTimeout();
        }
        const auto final_async_stats = async_map_publisher->stats();
        final_map_timing = async_map_publisher->timingStats();
        ostringstream async_log;
        async_log << fixed << setprecision(3)
                  << "event=async_map_summary"
                  << " requested=" << final_async_stats.full_requested
                  << " published=" << final_async_stats.output.full_published
                  << " coalesced=" << final_async_stats.full_coalesced
                  << " tile_resync_requested="
                  << final_async_stats.tile_resync_requested
                  << " tile_resync_on_failure="
                  << final_async_stats.tile_resync_on_failure
                  << " tile_resync_completed="
                  << final_async_stats.tile_resync_completed
                  << " failures=" << final_async_stats.failures
                  << " points=" << final_async_stats.last_full_points
                  << " snapshot_ms=" << final_async_stats.last_full_snapshot_ms
                  << " publish_ms=" << final_async_stats.output.last_full_publish_ms
                  << " frames_enqueued=" << final_async_stats.frames_enqueued
                  << " frames_built=" << final_async_stats.frames_built
                  << " frames_dropped=" << final_async_stats.frames_dropped
                  << " input_points_dropped="
                  << final_async_stats.input_points_dropped
                  << " input_bytes_dropped="
                  << final_async_stats.input_bytes_dropped
                  << " input_tile_changes_enqueued="
                  << final_async_stats.input_tile_changes_enqueued
                  << " input_tile_changes_merged="
                  << final_async_stats.input_tile_changes_merged
                  << " input_tile_changes_dropped="
                  << final_async_stats.input_tile_changes_dropped
                  << " input_tile_tasks_evicted="
                  << final_async_stats.input_tile_tasks_evicted
                  << " input_resync_requested="
                  << final_async_stats.input_resync_requested
                  << " frames_cancelled=" << final_async_stats.frames_cancelled
                  << " control_cancelled="
                  << final_async_stats.control_tasks_cancelled
                  << " stop_timeouts=" << final_async_stats.stop_timeouts
                  << " incomplete="
                  << ((final_async_stats.incomplete ||
                       final_async_stats.store.incomplete) ? 1 : 0)
                  << " frame_queue_max=" << final_async_stats.max_input_queue_tasks
                  << " frame_queue_max_points="
                  << final_async_stats.max_input_queue_points
                  << " frame_queue_max_bytes="
                  << final_async_stats.max_input_queue_bytes
                  << " frame_points=" << final_async_stats.last_frame_points
                  << " frame_build_ms=" << final_async_stats.last_build_ms
                  << " tile_count=" << final_async_stats.store.tile_count
                  << " tile_updates=" << final_async_stats.tile_updates
                  << " tile_merged=" << final_async_stats.output.tiles_merged
                  << " tile_dropped=" << final_async_stats.output.tiles_dropped
                  << " tile_retried=" << final_async_stats.output.tiles_retried
                  << " tile_rejected_capacity="
                  << final_async_stats.store.points_rejected_tile_capacity
                  << " points_rejected_memory="
                  << final_async_stats.store.points_rejected_memory
                  << " points_rejected_invalid="
                  << final_async_stats.store.points_rejected_invalid
                  << " accumulation_stopped="
                  << (final_async_stats.store.accumulation_stopped ? 1 : 0)
                  << " tiles_evicted="
                  << final_async_stats.store.tiles_evicted
                  << " pending_deletions="
                  << final_async_stats.store.pending_deletion_count
                  << " deletions_backpressured="
                  << final_async_stats.store.deletions_backpressured
                  << " tile_cancelled="
                  << final_async_stats.output.tiles_cancelled
                  << " full_cancelled="
                  << final_async_stats.output.full_cancelled
                  << " map_memory_bytes=" << final_async_stats.store.estimated_bytes;
        writeRuntimeLog(async_log.str());
        if (final_bag_map_requested.load()) {
            cerr << "[LaserMapping] Final Bag map could not be queued; "
                    "the storage summary will report the drop." << endl;
        }
    }

    const auto map_accumulate_summary = map_accumulate_timing.summary();
    const auto map_schedule_summary = map_schedule_timing.summary();
    const auto foxglove_map_timing = foxglove.mapTimingStats();
    ostringstream timing_log;
    timing_log << fixed << setprecision(3)
               << "event=timing_summary"
               << " window_capacity=" << BoundedTimingWindow<>::capacity();
    const auto append_timing = [&timing_log](
        const char* name, const TimingWindowSummary& timing) {
        timing_log << ' ' << name << "_samples=" << timing.total_samples
                   << ' ' << name << "_window_samples=" << timing.window_samples
                   << ' ' << name << "_avg_ms=" << timing.average_ms
                   << ' ' << name << "_p95_ms=" << timing.p95_ms
                   << ' ' << name << "_p99_ms=" << timing.p99_ms;
    };
    append_timing("map_accumulate", map_accumulate_summary);
    append_timing("map_schedule", map_schedule_summary);
    append_timing("map_build", final_map_timing.map_build);
    append_timing("tile_extract", final_map_timing.tile_extract);
    append_timing("tile_publish", final_map_timing.output.tile_publish);
    append_timing("full_snapshot", final_map_timing.full_snapshot);
    append_timing("full_publish", final_map_timing.output.full_publish);
    append_timing("full_serialize", foxglove_map_timing.serialize);
    append_timing("full_send", foxglove_map_timing.send);
    writeRuntimeLog(timing_log.str());

    if (storage_worker) {
        if (!storage_worker->stopFor(remaining_shutdown(), true)) {
            terminateForShutdownTimeout();
        }
        const auto storage_stats = storage_worker->stats();
        const auto storage_timing = storage_worker->timingStats();
        ostringstream storage_log;
        storage_log << fixed << setprecision(3)
                    << "event=storage_summary"
                    << " timing_window_capacity="
                    << BoundedTimingWindow<>::capacity()
                    << " enqueued=" << storage_stats.tasks_enqueued
                    << " written=" << storage_stats.tasks_written
                    << " dropped=" << storage_stats.tasks_dropped
                    << " overwritten=" << storage_stats.tasks_overwritten
                    << " cancelled=" << storage_stats.tasks_cancelled
                    << " stop_timeouts=" << storage_stats.stop_timeouts
                    << " startup_timeouts=" << storage_stats.startup_timeouts
                    << " failures=" << storage_stats.failures
                    << " failed=" << (storage_stats.failed ? 1 : 0)
                    << " worker_exited="
                    << (storage_stats.worker_exited ? 1 : 0)
                    << " bag_failures=" << storage_stats.bag_failures
                    << " bag_tasks_cancelled="
                    << storage_stats.bag_tasks_cancelled
                    << " bag_disabled="
                    << (storage_stats.bag_disabled ? 1 : 0)
                    << " pcd_failures=" << storage_stats.pcd_failures
                    << " pcd_disabled="
                    << (storage_stats.pcd_disabled ? 1 : 0)
                    << " pcd_tasks_dropped="
                    << storage_stats.pcd_tasks_dropped
                    << " pcd_points_dropped="
                    << storage_stats.pcd_points_dropped
                    << " pcd_bytes_dropped="
                    << storage_stats.pcd_bytes_dropped
                    << " pcd_tasks_cancelled="
                    << storage_stats.pcd_tasks_cancelled
                    << " pcd_points_cancelled="
                    << storage_stats.pcd_points_cancelled
                    << " pcd_bytes_cancelled="
                    << storage_stats.pcd_bytes_cancelled
                    << " queue_tasks=" << storage_stats.queue_tasks
                    << " queue_bytes=" << storage_stats.queue_bytes
                    << " inflight_tasks=" << storage_stats.inflight_tasks
                    << " inflight_points=" << storage_stats.inflight_points
                    << " inflight_bytes=" << storage_stats.inflight_bytes
                    << " hard_usage_bytes="
                    << storage_stats.hard_usage_bytes
                    << " queue_max_bytes=" << storage_stats.max_queue_bytes
                    << " max_hard_usage_bytes="
                    << storage_stats.max_hard_usage_bytes
                    << " pcd_pending_capacity_points="
                    << storage_stats.pcd_pending_capacity_points
                    << " pcd_pending_tasks="
                    << storage_stats.pcd_pending_tasks
                    << " pcd_pending_points="
                    << storage_stats.pcd_pending_points
                    << " pcd_pending_bytes="
                    << storage_stats.pcd_pending_bytes
                    << " pcd_scratch_bytes="
                    << storage_stats.pcd_scratch_bytes
                    << " pcd_queue_tasks=" << storage_stats.pcd_queue_tasks
                    << " pcd_queue_points=" << storage_stats.pcd_queue_points
                    << " pcd_queue_bytes=" << storage_stats.pcd_queue_bytes
                    << " max_pcd_pending_bytes="
                    << storage_stats.max_pcd_pending_bytes
                    << " max_pcd_scratch_bytes="
                    << storage_stats.max_pcd_scratch_bytes
                    << " pcd_chunks=" << storage_stats.pcd_chunks_written
                    << " pcd_chunk_points_limit="
                    << storage_stats.pcd_chunk_points_limit
                    << " pcd_chunk_frames_limit="
                    << storage_stats.pcd_chunk_frames_limit
                    << " bag_path_points=" << path_vec.size()
                    << " bag_path_points_trimmed=" << bag_path_points_trimmed
                    << " write_ms=" << storage_stats.last_write_ms
                    << " flush_ms=" << storage_stats.last_flush_ms;
        const auto append_storage_timing = [&storage_log](
            const char* name, const TimingWindowSummary& timing) {
            storage_log << ' ' << name << "_samples=" << timing.total_samples
                        << ' ' << name << "_window_samples="
                        << timing.window_samples
                        << ' ' << name << "_avg_ms=" << timing.average_ms
                        << ' ' << name << "_p95_ms=" << timing.p95_ms
                        << ' ' << name << "_p99_ms=" << timing.p99_ms;
        };
        append_storage_timing("write", storage_timing.write);
        append_storage_timing("bag_write", storage_timing.bag_write);
        append_storage_timing("pcd_write", storage_timing.pcd_write);
        append_storage_timing("flush", storage_timing.flush);
        writeRuntimeLog(storage_log.str());
    }

    // Their worker threads are joined; release bounded queues, tile storage,
    // bag handles, and pending PCD buffers before disarming the watchdog.
    async_map_publisher.reset();
    storage_worker.reset();

    // Flattening and PCD compression are shutdown-only, but both can still be
    // unexpectedly slow on a damaged/full disk. Run them behind the same
    // process-wide deadline. If they cannot finish, TerminateProcess is used before
    // stack unwinding so no detached thread can observe destroyed SLAM state.
    if (config.storage_save_final_ikdtree) {
        struct FinalExportState {
            mutex state_mutex;
            condition_variable done_cv;
            bool done = false;
            size_t points = 0;
            double flatten_ms = 0.0;
            double write_ms = 0.0;
            int write_result = 0;
            string map_path;
            string temporary_path;
            string failure;
        };
        auto export_state = make_shared<FinalExportState>();
        thread export_thread;
        try {
            export_thread = thread([export_state] {
                try {
                    const auto flatten_begin = chrono::steady_clock::now();
                    PointVector map_storage;
                    ikdtree.flatten(
                        ikdtree.Root_Node, map_storage, NOT_RECORD);
                    export_state->flatten_ms =
                        chrono::duration<double, milli>(
                            chrono::steady_clock::now() - flatten_begin)
                            .count();
                    export_state->points = map_storage.size();
                    if (!map_storage.empty()) {
                        PointCloudXYZI map_cloud;
                        map_cloud.points = std::move(map_storage);
                        map_cloud.width =
                            static_cast<uint32_t>(map_cloud.size());
                        map_cloud.height = 1;
                        map_cloud.is_dense = true;
                        export_state->map_path =
                            outputPath("PCD/ikd_tree_map.pcd");
                        export_state->temporary_path =
                            export_state->map_path + ".tmp";
                        const auto write_begin = chrono::steady_clock::now();
                        export_state->write_result =
                            pcl::io::savePCDFileBinaryCompressed(
                                export_state->temporary_path, map_cloud);
                        if (export_state->write_result == 0) {
                            if (!::MoveFileExA(
                                    export_state->temporary_path.c_str(),
                                    export_state->map_path.c_str(),
                                    MOVEFILE_REPLACE_EXISTING |
                                        MOVEFILE_WRITE_THROUGH))
                            {
                                const DWORD error = ::GetLastError();
                                ::DeleteFileA(
                                    export_state->temporary_path.c_str());
                                throw std::system_error(
                                    static_cast<int>(error),
                                    std::system_category(),
                                    "could not atomically replace final PCD");
                            }
                        } else {
                            ::DeleteFileA(export_state->temporary_path.c_str());
                        }
                        export_state->write_ms =
                            chrono::duration<double, milli>(
                                chrono::steady_clock::now() - write_begin)
                                .count();
                    }
                } catch (const std::exception& error) {
                    if (!export_state->temporary_path.empty()) {
                        ::DeleteFileA(export_state->temporary_path.c_str());
                    }
                    export_state->failure = error.what();
                } catch (...) {
                    if (!export_state->temporary_path.empty()) {
                        ::DeleteFileA(export_state->temporary_path.c_str());
                    }
                    export_state->failure = "unknown";
                }
                {
                    lock_guard<mutex> lock(export_state->state_mutex);
                    export_state->done = true;
                }
                export_state->done_cv.notify_all();
            });
        } catch (const std::exception& error) {
            export_state->failure = error.what();
            export_state->done = true;
        }

        if (export_thread.joinable()) {
            unique_lock<mutex> lock(export_state->state_mutex);
            const bool completed = export_state->done_cv.wait_until(
                lock, shutdown_deadline,
                [&] { return export_state->done; });
            lock.unlock();
            if (!completed) {
                terminateForShutdownTimeout();
            }
            export_thread.join();
        }

        if (!export_state->failure.empty()) {
            cerr << "[PCD] Final ikd-Tree export failed: "
                 << export_state->failure << endl;
            writeRuntimeLog(
                "event=final_ikdtree_export failure=\"" +
                export_state->failure + "\"");
        } else {
            if (export_state->points > 0 &&
                export_state->write_result == 0)
            {
                cout << "[PCD] ikd-Tree map saved: "
                     << export_state->map_path << " ("
                     << export_state->points << " pts)" << endl;
            } else if (export_state->write_result != 0) {
                cerr << "[PCD] Failed to save final ikd-Tree map: "
                     << export_state->map_path << " (error="
                     << export_state->write_result << ")" << endl;
            }
            ostringstream export_log;
            export_log << fixed << setprecision(3)
                       << "event=final_ikdtree_export"
                       << " points=" << export_state->points
                       << " flatten_ms=" << export_state->flatten_ms
                       << " write_ms=" << export_state->write_ms
                       << " result=" << export_state->write_result;
            writeRuntimeLog(export_log.str());
        }
    }

    process_rss_bytes = currentProcessRssBytes();
    process_peak_rss_bytes = max(process_peak_rss_bytes, process_rss_bytes);
    writeRuntimeLog(
        "event=memory_summary process_rss_bytes=" +
        to_string(process_rss_bytes) + " process_peak_rss_bytes=" +
        to_string(process_peak_rss_bytes));

    // Stop Foxglove after asynchronous output workers have drained, still
    // under the total shutdown deadline.
    struct FoxgloveStopState {
        mutex state_mutex;
        condition_variable done_cv;
        bool done = false;
        bool failed = false;
    };
    auto foxglove_stop_state = make_shared<FoxgloveStopState>();
    thread foxglove_stop_thread;
    try {
        foxglove_stop_thread = thread([&foxglove, foxglove_stop_state] {
            try {
                foxglove.stop();
            } catch (...) {
                foxglove_stop_state->failed = true;
            }
            {
                lock_guard<mutex> lock(foxglove_stop_state->state_mutex);
                foxglove_stop_state->done = true;
            }
            foxglove_stop_state->done_cv.notify_all();
        });
    } catch (...) {
        terminateForShutdownTimeout();
    }
    {
        unique_lock<mutex> lock(foxglove_stop_state->state_mutex);
        const bool completed = foxglove_stop_state->done_cv.wait_until(
            lock, shutdown_deadline,
            [&] { return foxglove_stop_state->done; });
        lock.unlock();
        if (!completed) terminateForShutdownTimeout();
    }
    foxglove_stop_thread.join();
    if (foxglove_stop_state->failed) terminateForShutdownTimeout();

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
    shutdown_watchdog.cancelAndJoin();
}
