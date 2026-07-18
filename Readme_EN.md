# Livox FAST-LIO Windows

[中文 README](README.md)

**Native Windows FAST-LIO2 SLAM application** — supports real-time mapping with Livox Horizon LiDAR, Livox SDK1 / LVX v1.1 playback, and ROS1 Bag playback.
![Demo](Screen_Record.gif)

Based on the [FAST-LIO2](https://github.com/hku-mars/FAST_LIO) algorithm, this project removes the ROS runtime dependency and runs natively on Windows. It can parse Livox SDK1 `.lvx` recordings directly, read and write ROS1 Bag files, and stream mapping results to [Foxglove Studio](https://foxglove.dev/) in real time. Long-running maps can use a bounded-memory Dirty Tile pipeline; full-map generation, live visualization, Bag output, and PCD output all run outside the SLAM thread.

---

## Table of Contents

- [Features](#features)
- [System Requirements](#system-requirements)
- [Build from Source](#build-from-source)
  - [Install Dependencies](#install-dependencies)
  - [Configure and Build with CMake](#configure-and-build-with-cmake)
- [Usage](#usage)
  - [Command-Line Arguments](#command-line-arguments)
  - [Real-Time LiDAR Mode](#real-time-lidar-mode)
  - [LVX Playback Mode](#lvx-playback-mode)
  - [Bag Playback Mode](#bag-playback-mode)
  - [Foxglove Visualization](#foxglove-visualization)
  - [Map Output Modes](#map-output-modes)
- [Configuration File](#configuration-file)
- [Project Structure](#project-structure)
- [Output Files](#output-files)
- [Connecting a Real Livox LiDAR](#connecting-a-real-livox-lidar)
- [FAQ](#faq)

---

## Features

- **FAST-LIO2 algorithm** — tightly coupled LiDAR-IMU odometry based on IEKF (Iterated Error-State Kalman Filter)
- **ikd-Tree** — incremental k-d tree for efficient nearest-neighbor search and map maintenance
- **Livox Horizon support** — receives real-time point cloud and IMU data through Livox SDK
- **Livox SDK1 LVX v1.1 playback** — replays recorded `.lvx` files without hardware, with synchronized point cloud and IMU data
- **ROS1 Bag file read/write** — reads `.bag` files exported by Livox Viewer as input and writes SLAM results to an output bag file
- **Scalable map output** — `full_async`, `tiled_incremental`, and `hybrid` modes; the high-density configuration publishes persistent Dirty Tiles together with low-rate full PointCloud snapshots by default
- **Bounded asynchronous pipeline** — map building, Foxglove sends, Bag writes, and compressed PCD writes use separate workers with explicit queue and memory limits, keeping heavy I/O out of IEKF
- **Foxglove Studio visualization** — view the `/map` full map or persistent `/map_tiles`, current frame cloud, odometry, path, and TF transforms through `ws://localhost:8765`
- **Playback control** — Foxglove playback time is synchronized with the current SLAM time; pause, resume, and playback speed changes are supported, while seek by dragging is not currently available
- **Asynchronous chunked PCD output** — binary and binary-compressed formats with point- or frame-based chunking, avoiding an unbounded single in-memory PCD buffer
- **Observability and bounded shutdown** — logs queue peaks, drops/merges, memory, and average/P95/P99 timing; workers use bounded drain and shutdown deadlines
- **YAML configuration** — flexible parameter configuration with command-line overrides
- **No ROS dependency** — fully native Windows application, no ROS installation required

---

## System Requirements

| Item | Requirement |
|------|-------------|
| Operating system | Windows 10/11 (x64) |
| Compiler | MSVC (Visual Studio 2019+) with C++17 support |
| CMake | >= 3.15 |
| vcpkg | Latest version, located at `third_party/vcpkg` in this project |
| Memory | >= 8 GB; 16 GB+ recommended for large-scale mapping |

---

## Build from Source

### Install Dependencies

This project uses [vcpkg](https://vcpkg.io/) to manage core C++ dependencies. See [`vcpkg.json`](vcpkg.json) for the dependency manifest. The Foxglove C++ SDK is included as a redistributable package under `third_party/foxglove-sdk/`.

| Dependency | Purpose |
|------------|---------|
| **PCL** | Point cloud processing (common, io, filters, kdtree) |
| **Eigen3** | Linear algebra and matrix operations |
| **yaml-cpp** | YAML configuration parsing |
| **Foxglove C++ SDK** | Official Foxglove WebSocket and protobuf schema publishing, located at `third_party/foxglove-sdk/` |
| **bzip2** | bzip2 decompression support for bag files |
| **lz4** | lz4 decompression support for bag files |
| **Livox SDK source** | Real-time Horizon connection and LVX SDK1 data structure reference; referenced as a submodule at `third_party/Livox-SDK`, and CMake builds and links `livox_sdk_static` automatically |

Installation steps:

```powershell
# 1. Enter the project directory
cd D:\Projects\livox-fast-lio-windows

# 2. Initialize vcpkg on the first build
cd third_party\vcpkg
.\bootstrap-vcpkg.bat
cd ..\..

# 3. Install dependencies through vcpkg manifest mode
# Dependencies are installed automatically during the CMake configure stage
```

> **Note:** Installing PCL and its dependency chain, including boost, flann, and qhull, can take a long time, usually about 15-30 minutes.

### Configure and Build with CMake

```powershell
# Configure the project
cmake -B build -S . -DCMAKE_TOOLCHAIN_FILE=third_party/vcpkg/scripts/buildsystems/vcpkg.cmake

# Build with multiple threads; adjust -j according to your CPU core count
cmake --build build --config Release -j16

# Run automated tests
ctest --test-dir build -C Release --output-on-failure

# Optional: create a redistributable installation directory
cmake --install build --config Release --prefix dist
```

After a successful build, the executable is located at:

```text
build\Release\livox_fast_lio.exe
```

The installed executable is `dist\bin\livox_fast_lio.exe`, and installed
configuration files are under `dist\config\`.

> **Tip:** CMake automatically copies `config/` to `build\config/` and deploys PCL, Foxglove SDK, and other runtime DLLs next to the executable. The installation also includes `pcl_io.dll` and `foxglove_cpp.dll`; no manual copying from vcpkg is required.
> If `third_party/Livox-SDK` exists, CMake builds `sdk_core` and links the real Livox SDK automatically. It falls back to `src/livox_sdk_stub.cpp` only when the SDK is absent.

---

## Usage

### Command-Line Arguments

```text
livox_fast_lio.exe [config_path] [--lvx <file>] [--bag <file>] [key=value ...]
```

| Argument | Description |
|----------|-------------|
| `config_path` | Path to the YAML configuration file. Default: `config/horizon.yaml` |
| `--lvx <file>` | Path to the `.lvx` playback file |
| `--bag <file>` | Path to the `.bag` playback file exported by Livox Viewer as a ROS1 bag |
| `key=value` | Override configuration values, such as `blind=2` or `acc_cov=0.5` |
| `--help` / `-h` | Show help information |

**Examples:**

```powershell
# Use the default configuration and connect to a real-time LiDAR
.\livox_fast_lio.exe

# Specify a configuration file
.\livox_fast_lio.exe D:\path\to\horizon.yaml

# Use the high-density Horizon configuration (hybrid: Tiles + full PointCloud snapshots)
.\livox_fast_lio.exe config\horizon_hd.yaml pcd_save_en=false

# Replay an LVX file. For visualization/performance-only runs, disable both
# chunked PCD output and the final ikd-Tree export.
.\livox_fast_lio.exe config\horizon_hd.yaml --lvx "D:\data\recording.lvx" `
  pcd_save_en=false storage.save_final_ikdtree=false

# Replay a bag file exported by Livox Viewer
.\livox_fast_lio.exe --bag D:\data\recording.bag

# Replay with an explicit configuration
.\livox_fast_lio.exe config\horizon.yaml --lvx "D:\data\test.lvx"

# Override parameters: blind distance 2 m, accelerometer covariance 0.5
.\livox_fast_lio.exe blind=2 acc_cov=0.5

# Override multiple parameters
.\livox_fast_lio.exe blind=2 max_iter=15 filter_size_map=0.3

# Temporarily switch the map output mode
.\livox_fast_lio.exe config\horizon_hd.yaml --lvx "D:\data\test.lvx" `
  map_output.mode=full_async
```

`key=value` accepts both legacy top-level override names and grouped names such
as `map_output.mode`, `storage.mode`, and
`foxglove.current_cloud_publish_hz`. If an option appears more than once, the
last command-line value wins.

### Real-Time LiDAR Mode

The default mode connects to a Livox Horizon device and runs real-time SLAM:

```powershell
cd build\Release
.\livox_fast_lio.exe ..\config\horizon.yaml
```

After startup, the program will:

1. Load the `config/horizon.yaml` configuration
2. Start the Foxglove WebSocket server on port 8765
3. Initialize Livox SDK, listen for Livox broadcasts on the LAN, and connect to one Horizon device
4. Receive LiDAR + IMU data after sampling starts and feed real-time frames into FAST-LIO2
5. Output pose estimation and the point cloud map in real time

The live path is currently designed for a single Livox Horizon device. If multiple Livox devices are present on the same LAN, use `livox_broadcast_code=...` to lock onto one device and avoid mixing data streams.

### LVX Playback Mode

You can test the full SLAM pipeline without hardware by replaying `.lvx` files recorded with Livox Viewer or Livox SDK1:

```powershell
.\livox_fast_lio.exe config\horizon.yaml --lvx "D:\data\my_recording.lvx" pcd_save_en=false
```

LVX playback reads the file header, device information, and the number of valid non-empty frames. Empty frames at the end of the file are treated as normal EOF. During playback, point cloud and IMU data are pushed according to LVX frame intervals and packet timestamps to approximate the original acquisition rhythm. The common processing path remains `LvxPoint/ImuData -> Preprocess -> Sync -> IEKF`; downstream workers handle map building, Foxglove, and PCD output.

The current LVX parser targets Livox SDK1 / LVX v1.1 data and supports the following `data_type` values:

| data_type | Data type |
|-----------|-----------|
| 0 | Cartesian |
| 1 | Spherical |
| 2 | Extended Cartesian |
| 3 | Extended Spherical |
| 4 | Dual Extended Cartesian |
| 5 | Dual Extended Spherical |
| 6 | IMU |
| 7 | Triple Extended Cartesian |
| 8 | Triple Extended Spherical |

Notes:

- Spherical-coordinate points are converted to metric XYZ points.
- Dual-return and triple-return points are split into independent points before mapping.
- FAST-LIO requires IMU data. If the LVX file contains no IMU packets, the program will report this in the log.
- When unsupported or truncated packets are encountered, the program prints diagnostic information to help identify recording or format issues.

### Bag Playback Mode

Use a ROS1 `.bag` file exported by Livox Viewer to run SLAM:

```powershell
.\livox_fast_lio.exe --bag D:\data\my_recording.bag
```

The program will:

1. Read `/livox/lidar` or `/livox_lidar` Livox `CustomMsg` data, and `/livox/imu` or `/livox_imu` `sensor_msgs/Imu` data from the bag file
2. Sort messages by sensor timestamp, replay them at 1.0x real-time speed, and run FAST-LIO2
3. Follow the ROS FAST-LIO synchronization policy by waiting until IMU data covers the LiDAR frame end time before processing, preventing LiDAR frames from running ahead
4. Start the Foxglove WebSocket by default at `ws://localhost:8765`
5. Publish protobuf schema data through the official Foxglove C++ SDK
6. Use the Storage Worker to write SLAM results asynchronously to `<input_file_name>_output.bag` when the output path is writable

The Windows version applies the same key processing chain as ROS FAST-LIO for Livox `CustomMsg`:

- Each Livox `CustomMsg` is treated as one LiDAR frame
- Preprocessing applies `point_filter_num=3`, blind-zone filtering, tag filtering, reflectivity-to-intensity mapping, and `offset_time` to point-time mapping
- LiDAR/IMU synchronization waits for `last_imu_time >= lidar_end_time`
- IEKF uses the original FAST-LIO-style `h_share_model` and `update_iterated_dyn_share_modified(0.001, solve_time)`
- `/cloud_registered` publishes the current registered frame cloud. The map topic is selected by `map_output.mode`; both `/map` and `/map_tiles` come from an independent bounded accumulated map and are not affected by local ikd-Tree pruning in FAST-LIO

### Foxglove Visualization

#### Real-Time Visualization over WebSocket

LVX and Bag playback start the Foxglove WebSocket server by default:

```powershell
.\livox_fast_lio.exe config\horizon.yaml --lvx "D:\data\my_recording.lvx" pcd_save_en=false
.\livox_fast_lio.exe config\horizon.yaml --bag "D:\data\my_recording.bag"
```

1. Open Foxglove Studio and select **Open Connection**
2. Select **Foxglove WebSocket**
3. Enter `ws://localhost:8765`
4. Add a 3D panel and set the Fixed frame to `map`
5. With `horizon_hd.yaml`, display `/map_tiles` and `/cloud_registered`; a low-rate `/map` PointCloud snapshot is also retained. The other default configurations use `/map`. Add `/odometry`, `/path`, and `/tf` as needed

Foxglove playback time follows the SLAM processing time broadcast. LVX and Bag playback support pause, resume, and playback speed adjustment in Foxglove. Seeking by dragging the progress bar to a specific time is not currently available because FAST-LIO state, the local ikd-Tree, the path, and the accumulated map all depend on historical LiDAR/IMU integration order and cannot be updated by only moving the file read position.

Real-time WebSocket topics:

| Topic | Foxglove schema | Description |
|-------|-----------------|-------------|
| `/map` | `foxglove.PointCloud` | Full accumulated map snapshots in `full_async` / `hybrid`, with frame `map` |
| `/map_tiles` | `foxglove.SceneUpdate` | Persistent Tiles in `tiled_incremental` / `hybrid`; the same Tile ID is updated in place, eviction sends a deletion, and a new client triggers a current-Tile resync |
| `/map_delta` | `foxglove.PointCloud` | Legacy incremental topic; new configurations should use `map_output.mode` and `/map_tiles` |
| `/cloud_registered` | `foxglove.PointCloud` | Current registered frame cloud, with frame `map` |
| `/odometry` | `foxglove.Odometry` | SLAM odometry, with body frame `base_link` |
| `/path` | `foxglove.PosesInFrame` | Motion trajectory |
| `/tf` | `foxglove.FrameTransforms` | Coordinate transform from `base_link` to `map` |
| `/imu` | `livox_fast_lio/Imu` (JSON) | IMU data, with frame `livox_imu` |

### Map Output Modes

`map_output.mode` is authoritative for map output behavior. Legacy
`publish.*` map fields are still mapped to compatible behavior when the
`map_output` group is absent.

| Mode | Foxglove map output | Recommended use |
|------|----------------------|-----------------|
| `full_async` | `/map` only | Legacy layout compatibility, smaller maps, and tools that require one complete PointCloud |
| `tiled_incremental` | `/map_tiles` only | Long-running or high-density mapping; each update scales with changed regions |
| `hybrid` | `/map` + `/map_tiles` | Migration and comparison; highest resource cost |

All modes accumulate a bounded voxel map in the background. Tile mode sends
Dirty Tile updates and merges queued work for the same Tile; full-map mode
builds snapshots at the configured interval. In `hybrid` mode, the number of
consecutive Tile outputs is bounded so a pending `/map` snapshot cannot be
starved by continuously updated Tiles. A slow or disconnected network client
does not make the SLAM thread serialize point clouds or perform network writes.

---

## Configuration File

Default configuration file: [`config/horizon.yaml`](config/horizon.yaml)

High-density configuration: [`config/horizon_hd.yaml`](config/horizon_hd.yaml). It reduces `filter_size_surf` and `filter_size_map` from `0.5 m` to `0.05 m`, sets `point_filter_num` to `1`, raises the live frame point threshold to `30000`, sets the IEKF iteration limit to `4`, raises the feature limit to `4000`, and uses `hybrid` by default. `/map_tiles` continuously publishes Dirty Tiles, while `/map` retains a full `foxglove.PointCloud` snapshot with a 5-second minimum interval and immediately resynchronizes a newly connected client. If snapshot construction or network delivery is slow, the runtime lengthens the actual full-map interval to avoid output backlog. This improves map density but increases CPU and memory usage. Visualization uses an independent `0.2 m` Tile voxel so SLAM precision does not directly become network load.

### common — General Settings

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `lid_topic` | string | `/livox/lidar` | LiDAR data topic. Reserved field; the Windows version does not use ROS topics |
| `imu_topic` | string | `/livox/imu` | IMU data topic |
| `livox_broadcast_code` | string | `""` | Optional Livox broadcast-code filter in live mode. Empty string means connect to the discovered device automatically |
| `realtime_frame_sec` | double | `0.10` | Live-mode accumulation window, in seconds, for one FAST-LIO frame |
| `realtime_frame_points` | int | `20000` | Live-mode point threshold that can flush one FAST-LIO frame early |
| `time_sync_en` | bool | `false` | Whether to enable external time synchronization, only used when hardware synchronization is unavailable |
| `time_offset_lidar_to_imu` | double | `0.0` | LiDAR-IMU time offset in seconds, calibrated by tools such as LI-Init |

### preprocess — Preprocessing Parameters

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `lidar_type` | int | `1` | LiDAR type: 1=Livox, 2=Velodyne, 3=Ouster |
| `scan_line` | int | `6` | Number of scan lines. Horizon = 6 |
| `blind` | double | `4` | Blind-zone distance in meters; points closer than this are filtered out |
| `feature_enabled` | bool | `false` | Whether to enable feature extraction. Usually kept false for Livox Horizon bag playback |
| `point_filter_num` | int | `3` | Livox point filtering step, aligned with ROS FAST-LIO Horizon launch parameters |

### mapping — SLAM Mapping Parameters

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `acc_cov` | double | `0.1` | Accelerometer noise covariance |
| `gyr_cov` | double | `0.1` | Gyroscope noise covariance |
| `b_acc_cov` | double | `0.0001` | Accelerometer bias covariance |
| `b_gyr_cov` | double | `0.0001` | Gyroscope bias covariance |
| `max_iteration` | int | `3` | Maximum IEKF iterations, aligned with ROS FAST-LIO Horizon launch parameters |
| `max_feature_points` | int | `2000` | Maximum number of downsampled feature points per frame entering IEKF. `<=0` means unlimited |
| `iekf_match_threads` | int | `4` | IEKF point-matching workers. `0` uses half the logical CPUs (capped at 8); use `1` for troubleshooting |
| `filter_size_surf` | double | `0.5` | Voxel downsampling leaf size for the current frame cloud, in meters |
| `filter_size_map` | double | `0.5` | Incremental downsampling leaf size for the ikd-Tree map, in meters |
| `cube_side_length` | int | `1000` | Local map cube side length, in meters |
| `fov_degree` | double | `100` | Field of view, in degrees |
| `det_range` | double | `260.0` | Maximum detection range, in meters |
| `extrinsic_est_en` | bool | `true` | Whether to estimate IMU-LiDAR extrinsics online |
| `extrinsic_T` | [3] | `[0.055, 0.022, -0.030]` | Extrinsic translation from LiDAR to IMU, in meters |
| `extrinsic_R` | [9] | Identity matrix | Extrinsic rotation from LiDAR to IMU, row-major 3x3 |

### publish — Publishing Settings

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `path_en` | bool | `true` | Whether to publish the motion path |
| `scan_publish_en` | bool | `true` | Whether to publish point clouds. `false` disables all point cloud output |
| `dense_publish_en` | bool | `true` | Whether to publish dense point clouds. `false` enables downsampling |
| `scan_bodyframe_pub_en` | bool | `true` | Whether to publish point clouds in the IMU body frame |
| `publish_full_map` | bool | `true` | Legacy compatibility field; `map_output.mode` controls whether new configurations publish `/map` |
| `async_full_map_publish` | bool | `true` | Legacy compatibility field; full maps are always built and published in the background |
| `full_map_publish_interval_ms` | int | `1000` | Legacy full-map interval; maps to `map_output.full_publish_interval_ms` |
| `full_map_voxel_size` | double | `0.2` | Legacy full-map output voxel; maps to `map_output.full_voxel_leaf_m` |
| `bag_full_map_periodic` | bool | `false` | Periodically writes full `/map` messages to the output bag; false writes only the final map |
| `publish_map_delta` | bool | `false` | Legacy compatibility field; new configurations use `map_output.mode=tiled_incremental` |
| `map_delta_max_pending_points` | int | `200000` | Legacy incremental limit; maps to `map_output.max_points_per_update` |
| `foxglove_control_interval_ms` | int | `20` | Foxglove playback-clock and playback-state update interval in milliseconds; clamped to at least `10` at runtime |
| `foxglove_backlog_size` | int | `64` | Legacy compatibility field; maps to `foxglove.backlog_size` and controls the WebSocket message backlog; clamped to at least `8` at runtime |

### map_output — Scalable Map Output

| Parameter | Type | Horizon default | Description |
|-----------|------|-----------------|-------------|
| `mode` | string | `full_async` | `full_async`, `tiled_incremental`, or `hybrid`; `horizon_hd.yaml` defaults to `hybrid` |
| `full_publish_interval_ms` | int | `1000` | Minimum `/map` snapshot interval in milliseconds; clamped to at least 100 at runtime |
| `full_voxel_leaf_m` | double | `0.2` | Full-map output voxel leaf size in meters, independent of ikd-Tree precision |
| `tile_size_m` | double | `20.0` | Spatial Tile side length in meters |
| `tile_voxel_leaf_m` | double | `0.2` | Tile voxel size in meters; `tile_size_m` must be an integer multiple |
| `voxel_update_policy` | string | `latest` | Use `first`, `latest`, or `centroid` for repeated updates to one voxel |
| `tile_publish_hz` | int | `10` | Dirty Tile extraction and publication rate |
| `max_tiles_per_update` | int | `32` | Maximum Tiles per publication batch |
| `max_points_per_update` | int | `200000` | Maximum points per Tile publication batch |
| `input_queue_capacity` | int | `64` | Maximum map-build input Tile tasks; queued work for the same Tile can merge |
| `input_queue_max_mb` | int | `128` | Hard map-build input queue byte limit in MiB |
| `max_memory_mb` | int | `1024` | Estimated bounded accumulated-map memory limit in MiB |
| `memory_policy` | string | `stop_accumulating` | On the limit use `stop_accumulating`, `lru`, or `evict_far`; `spill_to_disk` is not implemented and warns before falling back to `stop_accumulating` |

### foxglove — Live Output Queue

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `current_cloud_publish_hz` | double | `20` | Maximum `/cloud_registered` publication rate |
| `path_publish_hz` | double | `2` | Maximum `/path` publication rate |
| `backlog_size` | int | `64` | Foxglove WebSocket message backlog |

Odometry, TF, the current cloud, path, and IMU use a bounded live-output
worker. Replaceable data follows latest-wins behavior; cloud overload is
counted as a drop instead of propagating network backpressure into IEKF.

### storage — Bag and PCD Worker

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `mode` | string | `realtime` | `realtime` drops older tasks on overload and keeps SLAM running; `reliable` never drops silently and marks storage failed after capacity or write failure |
| `queue_max_mb` | int | `512` | Byte limit in MiB for the shared Bag/PCD queue plus in-flight payload |
| `bag_path_publish_hz` | double | `1` | `/path` write rate in the output Bag |
| `path_max_points` | int | `100000` | Maximum rolling Path points in memory and Bag; odometry is still retained per frame |
| `pcd_format` | string | `binary_compressed` | `binary` or `binary_compressed` |
| `pcd_chunk_points` | int | `1000000` | Point-count trigger for each PCD chunk |
| `pcd_chunk_frames` | int | `0` | Optional frame-count trigger; `0` disables it |
| `save_final_ikdtree` | bool | `true` | Also export `PCD/ikd_tree_map.pcd` during shutdown; independent of `pcd_save_en` |

### pcd_save — Point Cloud Saving

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `pcd_save_en` | bool | `true` | Asynchronously write registered frames to `PCD/scans_*.pcd` chunks |
| `interval` | int | `-1` | Legacy frame-based chunk setting; migrated only when `storage.pcd_chunk_frames` is absent and the value is greater than 0 |

Setting only `pcd_save_en=false` does not disable the final ikd-Tree export.
For playback performance tests, usually also set
`storage.save_final_ikdtree=false`.

### runtime — Runtime Logging

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `runtime_pos_log_enable` | bool | `false` | Whether to write the additional legacy position text log; structured `Log/runtime.txt` is always generated |

`Log/runtime.txt` contains per-frame timing, current and peak input/output
queues, Tile merge/drop/resync counters, process RSS, and average/P95/P99
summaries for map building, serialization, sends, Bag, PCD, and flushes.

---

## Project Structure

```text
livox-fast-lio-windows/
├── CMakeLists.txt              # CMake build configuration
├── vcpkg.json                  # vcpkg dependency manifest (manifest mode)
├── config/
│   ├── avia.yaml               # Livox Avia configuration
│   ├── horizon.yaml            # Default Horizon configuration (0.5 m SLAM voxels)
│   ├── horizon_hd.yaml         # High-density Horizon (0.05 m, Tile + PointCloud snapshots)
│   └── mid360.yaml             # Livox Mid-360 configuration
├── cmake/
│   └── deploy_runtime_dlls.cmake # Post-build deployment of PCL, Foxglove, and other runtime DLLs
├── include/
│   ├── bounded_timing_stats.h  # Fixed-capacity timing windows and P95/P99 summaries
│   ├── common_lib.h            # Common utility functions
│   ├── Exp_mat.h               # Exponential map / SO3 math utilities
│   ├── so3_math.h              # SO(3) group operations
│   ├── types.h                 # Basic type definitions (V3D, M3D, ImuData, MeasureGroup, etc.)
│   ├── use-ikfom.hpp           # IKFoM manifold state definition and process model
│   ├── IMU_Processing.hpp      # IMU preprocessing and backward compensation
│   ├── laser_mapping.h         # SLAM main loop entry point
│   ├── preprocess.h            # Point cloud preprocessing for Livox, Velodyne, and Ouster
│   ├── fast_lio_observation.h  # FAST-LIO point-to-plane observation constants and residual thresholds
│   ├── lidar_imu_sync.h        # LiDAR/IMU frame-end coverage synchronization check
│   ├── playback_status.h        # Playback terminal states and safe log error text
│   ├── map_accumulator.h       # Full accumulated map cache for Foxglove/Bag output
│   ├── async_map_publisher.h    # Legacy full-map asynchronous publisher retained for compatibility tests
│   ├── tiled_map_store.h       # Bounded Tile/voxel map with dirty and eviction tracking
│   ├── map_build_worker.h      # Tile input coalescing, map building, and snapshot barriers
│   ├── foxglove_output_worker.h # Bounded map network-output queue
│   ├── realtime_foxglove_worker.h # Pose, current cloud, path, and IMU output queue
│   ├── storage_worker.h        # Asynchronous Bag and chunked-PCD storage queue
│   ├── livox_adapter.h         # Livox SDK adapter
│   ├── livox_sdk.h             # Livox SDK compatibility header; forwards to the official header when available
│   ├── lvx_reader.h            # LVX file reading and playback
│   ├── foxglove_publisher.h    # Foxglove WebSocket publisher
│   ├── ros_message.h           # ROS message serialization/deserialization, header-only
│   ├── ros_bag.h               # Low-level ROS1 Bag parser
│   ├── bag_reader.h            # High-level bag file reader interface
│   ├── bag_writer.h            # Bag file writer
│   ├── yaml_config.h           # YAML configuration loader
│   └── ikd-Tree/               # Incremental k-d tree
│       ├── ikd_Tree.h
│       └── ikd_Tree.cpp
├── src/
│   ├── main.cpp                # Program entry point and argument parsing
│   ├── laser_mapping.cpp       # FAST-LIO2 core SLAM loop
│   ├── preprocess.cpp          # Point cloud preprocessing implementation
│   ├── livox_adapter.cpp       # Livox SDK adapter implementation
│   ├── livox_sdk_stub.cpp      # Livox SDK stub, used only when the official SDK is absent
│   ├── lvx_reader.cpp          # LVX playback implementation
│   ├── async_map_publisher.cpp  # Legacy full-map asynchronous publisher retained for compatibility tests
│   ├── foxglove_publisher.cpp  # Official Foxglove SDK WebSocket publishing implementation
│   ├── tiled_map_store.cpp     # Bounded Tile map implementation
│   ├── map_build_worker.cpp    # Asynchronous map building
│   ├── foxglove_output_worker.cpp # Map publication worker
│   ├── realtime_foxglove_worker.cpp # Live-topic publication worker
│   ├── storage_worker.cpp      # Bag/PCD storage worker
│   ├── ros_bag.cpp             # Low-level bag parsing: decompression and record parsing
│   ├── bag_reader.cpp          # High-level bag reading: topic routing and message dispatch
│   ├── bag_writer.cpp          # Bag writing: chunks, connections, and indexes
│   └── yaml_config.cpp         # Configuration loading implementation
├── tests/
│   ├── test_livox_fast_lio.cpp # Configuration, playback, map, storage, and output-pipeline tests
│   └── test_realtime_foxglove_worker.cpp # Realtime Foxglove Worker tests
├── third_party/
│   ├── foxglove-sdk/           # Foxglove C++ SDK redistributable
│   ├── IKFoM_toolkit/          # Iterated Kalman filter on manifold toolkit
│   ├── Livox-SDK/              # Official Livox SDK source for LVX SDK1 format reference
│   └── vcpkg/                  # vcpkg package manager
├── Log/                        # Runtime log output directory
└── PCD/                        # Point cloud output directory
```

---

## Output Files

By default, runtime output is created under the **current working directory at
startup**. Use the command-line override
`root_dir=D:\path\to\output` to select an independent output root. The startup
log prints the resolved `Output root`.

| Path | Content |
|------|---------|
| `Log/runtime.txt` | Structured per-frame log plus input, map, Foxglove, Storage, timing, and memory summaries |
| `Log/imu.txt` | IMU debug data |
| `PCD/scans_*.pcd` | Registered-cloud chunks from the Storage Worker; a final partial chunk is named `scans_final_*.pcd` |
| `PCD/ikd_tree_map.pcd` | Final local ikd-Tree map, atomically written during shutdown when `storage.save_final_ikdtree=true` |

In Bag mode, the output bag file is generated in the same directory as the input file:

| Input file | Output file |
|------------|-------------|
| `D:\data\test.bag` | `D:\data\test_output.bag` |

PCD files can be viewed with:

- [CloudCompare](https://www.cloudcompare.org/) — open-source point cloud viewer
- [MeshLab](http://www.meshlab.net/) — 3D mesh processing
- PCL Viewer — `pcl_viewer map.pcd`

---

## Connecting a Real Livox LiDAR

The repository references `third_party/Livox-SDK` as a git submodule. CMake builds the official SDK1 `sdk_core` target and links it into `livox_fast_lio.exe` automatically. The live connection flow is:

1. Initialize Livox SDK
2. Listen for Livox device broadcasts on the LAN
3. Add the target Horizon with `AddLidarToConnect(...)`
4. Start sampling with `LidarStartSampling(...)` after the device reaches `Normal`
5. Feed live point cloud and IMU data into FAST-LIO2

If your local checkout is missing `third_party/Livox-SDK`, initialize the official SDK submodule:

```powershell
git submodule update --init --recursive third_party/Livox-SDK
```

Then reconfigure and rebuild the main project:

```powershell
cmake -B build -S . -DCMAKE_TOOLCHAIN_FILE=third_party/vcpkg/scripts/buildsystems/vcpkg.cmake
cmake --build build --config Release -j16
```

**Network setup:** Make sure the PC network adapter and Horizon are on the same subnet, and that the firewall allows Livox SDK UDP broadcast and data ports. Use `ping <device-ip>` first to confirm basic connectivity. For a single-device setup, no IP option is required; the SDK discovers the device through broadcasts.

Live mapping command:

```powershell
.\build\Release\livox_fast_lio.exe .\config\horizon.yaml pcd_save_en=false
```

High-density mapping command:

```powershell
.\build\Release\livox_fast_lio.exe .\config\horizon_hd.yaml pcd_save_en=false
```

The sampling path is up when the log contains:

```text
[LivoxAdapter] Broadcast: code=...
[LivoxAdapter] Added LiDAR handle=...
[LivoxAdapter] Start sampling callback status=0 ... response=0
```

---

## FAQ

### Q: Runtime reports "config/horizon.yaml not found"

Run the program from the correct working directory, or specify the full path:

```powershell
# Option 1: Run from the project root with an explicit configuration
.\build\Release\livox_fast_lio.exe .\config\horizon.yaml

# Option 2: From build/Release, use the copy under build/config
cd build\Release
.\livox_fast_lio.exe ..\config\horizon.yaml
```

### Q: It reports "SDK init failed"

This usually means the program did not link or initialize the real Livox SDK successfully. Check:

1. `third_party/Livox-SDK` exists and contains `sdk_core/include/livox_sdk.h`
2. The CMake configure log contains `Livox SDK headers found; building sdk_core from source.`
3. The build produced `build\Livox-SDK\sdk_core\Release\livox_sdk_static.lib`
4. You re-ran `cmake -B build -S . ...` and `cmake --build build --config Release`

LVX and Bag playback do not depend on a real-time Livox SDK connection and can be used offline normally.

### Q: Foxglove Studio cannot connect to the WebSocket

1. Confirm that the program is running. The WebSocket server starts after SLAM starts.
2. Check whether port 8765 is blocked by the firewall.
3. Try connecting with `ws://127.0.0.1:8765`.
4. Confirm that the startup log contains `Foxglove WebSocket: ws://localhost:8765`.
5. If the port is already in use, close the program occupying port 8765 and restart this application.

### Q: Foxglove can display point clouds, but the cloud is messy or the publish rate is wrong

1. First confirm that Foxglove is connected to this program's live connection, `ws://localhost:8765`, not an old or incomplete output bag file.
2. Set the 3D panel Fixed frame to `map`.
3. Check `map_output_mode` in the startup log: select `/map` for `full_async`, `/map_tiles` for `tiled_incremental`, or either for `hybrid`. `/cloud_registered` is only the current registered frame.
4. Normal Livox Horizon bag playback should be close to a 10 Hz LiDAR frame rate. Logs usually show about `19-21` IMU samples per frame. For LVX v1.1 playback, the number of points and IMU packets per frame depends on the recording frame interval; a common 50 ms Horizon frame may contain about `10` IMU packets.
5. If the log repeatedly shows `No undistorted points`, `No effective points`, or the IMU count stays at 0 for a long time, there is likely still an issue with LiDAR/IMU timestamps, bag topics, or LVX packet parsing.

The current Windows version follows the ROS FAST-LIO reference chain for bag and LVX playback: Livox `CustomMsg` or LVX internal point/IMU preprocessing, LiDAR/IMU frame-end coverage synchronization, and original FAST-LIO-style IEKF point-to-plane observation updates. Map output publishes background full snapshots, Dirty Tiles, or both according to `map_output.mode`.

### Q: Startup reports a missing `pcl_io.dll` or another DLL

Do not copy `livox_fast_lio.exe` by itself. Rebuilding deploys runtime DLLs to
`build\Release\`:

```powershell
cmake --build build --config Release
```

Use the installation directory when redistributing the application:

```powershell
cmake --install build --config Release --prefix dist
.\dist\bin\livox_fast_lio.exe --help
```

The current CMake install rules include `pcl_io.dll`, `foxglove_cpp.dll`, and
the remaining PCL dependencies. If the build directory is old, rerun CMake
configure first so a new EXE is not mixed with old DLLs.

### Q: Playback exits with an invalid memory access or the process never exits

Older builds could trigger unsafe destruction of the third-party ikd-Tree
background rebuild thread during Windows process teardown. The current build
keeps that tree for the process lifetime and stops map, Foxglove, Bag, and PCD
workers with bounded deadlines before teardown. Rebuild the main executable
instead of continuing to run an old EXE.

### Q: Can the Foxglove playback bar pause or seek?

The WebSocket connection for LVX and Bag playback declares the Foxglove `PlaybackControl` capability. Currently supported:

- Playback time advances with the current SLAM processing time
- Pause and resume playback in Foxglove
- Adjust playback speed

Seeking to a specific time by dragging the playback progress bar is not currently supported. True seek requires resetting SLAM state, clearing old caches and maps, and rebuilding a consistent filter state and map from the target time after a seek request is received.

### Q: Can an LVX file without IMU be used for mapping?

No. FAST-LIO depends on LiDAR and IMU fusion, so the LVX file must contain IMU packets with `data_type=6`. If IMU data is missing, the program logs a warning. Record a new LVX with IMU included, or use a bag file that contains `/livox/imu`.

### Q: Why does the frame count in the LVX startup log not exactly match the file tail record?

The program checks frame offsets and counts valid non-empty frames. Empty frames at the tail are ignored. This kind of empty tail frame is normal in some Livox Viewer recordings and does not affect playback of valid earlier data.

### Q: C4819 Unicode warnings appear during compilation

Make sure all source files are saved as UTF-8 without BOM. In Visual Studio: File -> Save As -> Encoding -> choose "UTF-8 (without signature)".

### Q: PCL installation is very slow

The PCL dependency chain includes boost, flann, qhull, and other packages. The first installation usually takes about 15-30 minutes. This is normal, and later CMake configure runs will not reinstall everything.

---

## License

This project is based on the original FAST-LIO2 code and follows the [GPLv2](https://github.com/hku-mars/FAST_LIO/blob/master/LICENSE) license.

## Acknowledgements

- [FAST-LIO2](https://github.com/hku-mars/FAST_LIO) — HKU MARS Lab
- [ikd-Tree](https://github.com/hku-mars/ikd-Tree) — incremental k-d tree
- [IKFoM](https://github.com/hku-mars/IKFoM) — iterated Kalman filter on manifolds
- [Livox SDK](https://github.com/Livox-SDK/Livox-SDK) — Livoxtech
- [Foxglove](https://foxglove.dev/) — robotics data visualization platform
- [ROS Bag Format](http://wiki.ros.org/Bags/Format/2.0) — ROS1 bag file format specification
