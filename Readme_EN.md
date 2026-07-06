# Livox FAST-LIO Windows

[中文 README](README.md)

**Native Windows FAST-LIO2 SLAM application** — supports real-time mapping with Livox Horizon LiDAR, Livox SDK1 / LVX v1.1 playback, and ROS1 Bag playback.
![Demo](Screen_Record.gif)

Based on the [FAST-LIO2](https://github.com/hku-mars/FAST_LIO) algorithm, this project removes the ROS dependency and runs natively on Windows. It can parse Livox SDK1 `.lvx` recordings directly, read and write ROS1 Bag files, and stream mapping results to [Foxglove Studio](https://foxglove.dev/) in real time.

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
- **Foxglove Studio visualization** — view the accumulated map, current frame cloud, odometry, path, and TF transforms in real time through `ws://localhost:8765`
- **Playback control** — Foxglove playback time is synchronized with the current SLAM time; pause, resume, and playback speed changes are supported, while seek by dragging is not currently available
- **PCD point cloud saving** — automatically saves the global map as PCD files
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
| **Livox SDK source** | Real-time Horizon connection and LVX SDK1 data structure reference; included under `third_party/Livox-SDK`, and CMake builds and links `livox_sdk_static` automatically |

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
```

After a successful build, the executable is located at:

```text
build\Release\livox_fast_lio.exe
```

> **Tip:** CMake automatically copies the `config/` directory to `build\config/` and deploys all dependency DLLs to `build\Release/`.
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

# Use the high-density Horizon configuration
.\livox_fast_lio.exe config\horizon_hd.yaml pcd_save_en=false

# Replay an LVX file. For long recordings, disable PCD saving first.
.\livox_fast_lio.exe config\horizon.yaml --lvx "D:\data\recording.lvx" pcd_save_en=false

# Replay a bag file exported by Livox Viewer
.\livox_fast_lio.exe --bag D:\data\recording.bag

# Replay with an explicit configuration
.\livox_fast_lio.exe config\horizon.yaml --lvx "D:\data\test.lvx"

# Override parameters: blind distance 2 m, accelerometer covariance 0.5
.\livox_fast_lio.exe blind=2 acc_cov=0.5

# Override multiple parameters
.\livox_fast_lio.exe blind=2 max_iter=15 filter_size_map=0.3
```

### Real-Time LiDAR Mode

The default mode connects to a Livox Horizon device and runs real-time SLAM:

```powershell
cd build\Release
.\livox_fast_lio.exe
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

LVX playback reads the file header, device information, and the number of valid non-empty frames. Empty frames at the end of the file are treated as normal EOF. During playback, point cloud and IMU data are pushed according to LVX frame intervals and packet timestamps to approximate the original acquisition rhythm. The downstream pipeline remains the same: `LvxPoint/ImuData -> Preprocess -> Sync -> IEKF -> Foxglove/PCD`.

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
6. Write SLAM results to an output bag file named `<input_file_name>_output.bag` when the output path is writable

The Windows version applies the same key processing chain as ROS FAST-LIO for Livox `CustomMsg`:

- Each Livox `CustomMsg` is treated as one LiDAR frame
- Preprocessing applies `point_filter_num=3`, blind-zone filtering, tag filtering, reflectivity-to-intensity mapping, and `offset_time` to point-time mapping
- LiDAR/IMU synchronization waits for `last_imu_time >= lidar_end_time`
- IEKF uses the original FAST-LIO-style `h_share_model` and `update_iterated_dyn_share_modified(0.001, solve_time)`
- `/cloud_registered` publishes the current registered frame cloud. When `publish_full_map=true`, `/map` publishes an independent accumulated global map that is no longer affected by local ikd-Tree pruning in FAST-LIO

**How to get a bag file:**

1. Record an `.lvx` file with Livox Viewer
2. In Livox Viewer, choose **File -> Export** and select ROS Bag format
3. Use the exported `.bag` file as input to this program

### Foxglove Visualization

#### Option 1: Real-Time Visualization over WebSocket (Recommended)

LVX and Bag playback start the Foxglove WebSocket server by default:

```powershell
.\livox_fast_lio.exe config\horizon.yaml --lvx "D:\data\my_recording.lvx" pcd_save_en=false
.\livox_fast_lio.exe config\horizon.yaml --bag "D:\data\my_recording.bag"
```

1. Open Foxglove Studio and select **Open Connection**
2. Select **Foxglove WebSocket**
3. Enter `ws://localhost:8765`
4. Add a 3D panel and set the Fixed frame to `map`
5. Prefer displaying `/map` and `/cloud_registered` first, then add `/odometry`, `/path`, and `/tf` as needed

Foxglove playback time follows the SLAM processing time broadcast. LVX and Bag playback support pause, resume, and playback speed adjustment in Foxglove. Seeking by dragging the progress bar to a specific time is not currently available because FAST-LIO state, the local ikd-Tree, the path, and the accumulated map all depend on historical LiDAR/IMU integration order and cannot be updated by only moving the file read position.

Real-time WebSocket topics:

| Topic | Foxglove schema | Description |
|-------|-----------------|-------------|
| `/map` | `foxglove.PointCloud` | Fully accumulated FAST-LIO map, with frame `map`; by default it does not delete historical regions when the local ikd-Tree is pruned |
| `/cloud_registered` | `foxglove.PointCloud` | Current registered frame cloud, with frame `map` |
| `/odometry` | `foxglove.Odometry` | SLAM odometry, with body frame `base_link` |
| `/path` | `foxglove.PosesInFrame` | Motion trajectory |
| `/tf` | `foxglove.FrameTransforms` | Coordinate transform from `base_link` to `map` |

#### Option 2: Offline Visualization from Bag Files

After SLAM finishes, open the output bag file directly in Foxglove Studio:

1. Download and install [Foxglove Studio](https://foxglove.dev/download)
2. Open Foxglove Studio and select **File -> Open File**
3. Select `test2_output.bag`, the SLAM output file

The output bag contains the following topics:

| Topic | Type | Description |
|-------|------|-------------|
| `/odometry` | `geometry_msgs/PoseStamped` | SLAM odometry, including position and quaternion orientation |
| `/map` | `sensor_msgs/PointCloud2` | Fully accumulated FAST-LIO map |
| `/cloud_registered` | `sensor_msgs/PointCloud2` | Current registered frame cloud |
| `/path` | `geometry_msgs/PoseArray` | Motion trajectory |
| `/tf` | `tf2_msgs/TFMessage` | Coordinate transform from `base_link` to `map` |

---

## Configuration File

Default configuration file: [`config/horizon.yaml`](config/horizon.yaml)

High-density configuration: [`config/horizon_hd.yaml`](config/horizon_hd.yaml). It reduces `filter_size_surf` and `filter_size_map` from `0.5 m` to `0.2 m`, and sets `point_filter_num` to `1` for denser mapping. This improves map density but increases CPU and memory usage.

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
| `publish_full_map` | bool | `true` | Whether `/map` publishes the fully accumulated map. When false, it falls back to the local ikd-Tree map |

### pcd_save — Point Cloud Saving

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `pcd_save_en` | bool | `true` | Whether to save PCD files |
| `interval` | int | `-1` | Number of frames saved per PCD file. `-1` means all frames are saved to one file |

### runtime — Runtime Logging

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `runtime_pos_log_enable` | bool | `false` | Whether to output more detailed per-frame timing, point count, and map size logs |

---

## Project Structure

```text
livox-fast-lio-windows/
├── CMakeLists.txt              # CMake build configuration
├── vcpkg.json                  # vcpkg dependency manifest (manifest mode)
├── config/
│   ├── horizon.yaml            # Default Livox Horizon configuration (0.5 m voxels)
│   └── horizon_hd.yaml         # High-density Livox Horizon configuration (0.2 m voxels)
├── include/
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
│   ├── map_accumulator.h       # Full accumulated map cache for Foxglove/Bag output
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
│   ├── foxglove_publisher.cpp  # Official Foxglove SDK WebSocket publishing implementation
│   ├── ros_bag.cpp             # Low-level bag parsing: decompression and record parsing
│   ├── bag_reader.cpp          # High-level bag reading: topic routing and message dispatch
│   ├── bag_writer.cpp          # Bag writing: chunks, connections, and indexes
│   └── yaml_config.cpp         # Configuration loading implementation
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

At runtime, the program creates the following directories under `ROOT_DIR`, the source root directory:

| Path | Content |
|------|---------|
| `Log/` | Runtime logs, including IMU data and pose estimates |
| `PCD/` | Global map PCD files |

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

The repository includes `third_party/Livox-SDK`. CMake builds the official SDK1 `sdk_core` target and links it into `livox_fast_lio.exe` automatically. The live connection flow is:

1. Initialize Livox SDK
2. Listen for Livox device broadcasts on the LAN
3. Add the target Horizon with `AddLidarToConnect(...)`
4. Start sampling with `LidarStartSampling(...)` after the device reaches `Normal`
5. Feed live point cloud and IMU data into FAST-LIO2

If your local checkout is missing `third_party/Livox-SDK`, fetch the official SDK:

```powershell
cd third_party
git clone https://github.com/Livox-SDK/Livox-SDK.git
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
# Option 1: Run from build/Release, where config has been copied automatically
cd build\Release
.\livox_fast_lio.exe

# Option 2: Specify the configuration file path
.\livox_fast_lio.exe D:\Projects\livox-fast-lio-windows\config\horizon.yaml
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
3. During mapping, inspect `/map` first. With the default configuration it is the fully accumulated map. `/cloud_registered` is the current registered frame cloud, not the full accumulated map.
4. Normal Livox Horizon bag playback should be close to a 10 Hz LiDAR frame rate. Logs usually show about `19-21` IMU samples per frame. For LVX v1.1 playback, the number of points and IMU packets per frame depends on the recording frame interval; a common 50 ms Horizon frame may contain about `10` IMU packets.
5. If the log repeatedly shows `No undistorted points`, `No effective points`, or the IMU count stays at 0 for a long time, there is likely still an issue with LiDAR/IMU timestamps, bag topics, or LVX packet parsing.

The current Windows version follows the ROS FAST-LIO reference chain for bag and LVX playback: Livox `CustomMsg` or LVX internal point/IMU preprocessing, LiDAR/IMU frame-end coverage synchronization, original FAST-LIO-style IEKF point-to-plane observation updates, and `/map` publishing the full accumulated map every frame.

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
