# Livox FAST-LIO Windows

[English README](Readme_EN.md)

**Windows 原生 FAST-LIO2 SLAM 应用** — 支持 Livox Horizon LiDAR 实时建图、Livox SDK1 / LVX v1.1 回放与 ROS1 Bag 回放。
![运行演示](Screen_Record.gif)

基于 [FAST-LIO2](https://github.com/hku-mars/FAST_LIO) 算法，去除 ROS 运行时依赖，原生运行于 Windows 平台。支持直接解析 Livox SDK1 `.lvx` 录制文件、读取/写入 ROS1 Bag 文件，并可通过 [Foxglove Studio](https://foxglove.dev/) 实时查看建图过程。长时间建图可使用有界内存的 Dirty Tile 增量地图管线，完整地图、实时可视化、Bag 与 PCD 写入均在 SLAM 主线程之外执行。

---

## 目录

- [功能特性](#功能特性)
- [系统要求](#系统要求)
- [从源码构建](#从源码构建)
  - [依赖安装](#依赖安装)
  - [CMake 配置与编译](#cmake-配置与编译)
- [使用方法](#使用方法)
  - [命令行参数](#命令行参数)
  - [实时 LiDAR 模式](#实时-lidar-模式)
  - [LVX 回放模式](#lvx-回放模式)
  - [Bag 回放模式](#bag-回放模式)
  - [Foxglove 可视化](#foxglove-可视化)
  - [地图输出模式](#地图输出模式)
- [配置文件说明](#配置文件说明)
- [项目结构](#项目结构)
- [输出文件](#输出文件)
- [连接真实 Livox LiDAR](#连接真实-livox-lidar)
- [常见问题](#常见问题)

---

## 功能特性

- **FAST-LIO2 算法** — 基于 IEKF（迭代误差状态卡尔曼滤波）的紧耦合 LiDAR-IMU 里程计
- **ikd-Tree** — 增量式 k-d 树，支持高效近邻搜索与地图维护
- **Livox Horizon 支持** — 通过 Livox SDK 接收实时点云与 IMU 数据
- **Livox SDK1 LVX v1.1 回放** — 无需硬件，直接回放录制的 `.lvx` 文件，支持点云与 IMU 同步
- **ROS1 Bag 文件读写** — 读取 Livox Viewer 导出的 `.bag` 文件作为数据源，SLAM 结果写入输出 bag 文件
- **可扩展地图输出** — 支持 `full_async`、`tiled_incremental`、`hybrid` 三种模式；高密度配置默认同时发布持久化 Dirty Tile 和低频完整 PointCloud 快照
- **有界异步管线** — 地图构建、Foxglove 发送、Bag 与压缩 PCD 写入均使用独立 Worker 和显式内存/队列上限，不在 IEKF 主线程执行重 I/O
- **Foxglove Studio 可视化** — 通过 `ws://localhost:8765` 查看 `/map` 完整地图或 `/map_tiles` 持久化 Tile，以及当前帧点云、里程计、路径、TF 变换
- **回放播放控制** — Foxglove 播放条可同步当前时间，支持暂停、继续和倍速播放；拖动 seek 暂不可用
- **异步 PCD 分块保存** — 支持 binary / binary_compressed 格式，按点数或帧数分块，避免长时间运行时无限累计单个内存缓冲区
- **可观测性与安全退出** — 日志记录队列峰值、丢弃/合并、内存、平均/P95/P99 耗时；Worker 使用有界 drain 和退出超时
- **YAML 配置** — 灵活的参数配置，支持命令行覆盖
- **零 ROS 依赖** — 完全原生 Windows 应用，无需安装 ROS

---

## 系统要求

| 项目 | 要求 |
|------|------|
| 操作系统 | Windows 10/11 (x64) |
| 编译器 | MSVC (Visual Studio 2019+) 支持 C++17 |
| CMake | >= 3.15 |
| vcpkg | 最新版（项目内 `third_party/vcpkg`） |
| 内存 | >= 8 GB（大规模建图建议 16 GB+） |

---

## 从源码构建

### 依赖安装

项目使用 [vcpkg](https://vcpkg.io/) 管理基础 C++ 依赖。依赖清单见 [`vcpkg.json`](vcpkg.json)，Foxglove C++ SDK 以 redistributable 形式放在 `third_party/foxglove-sdk/`：

| 依赖 | 用途 |
|------|------|
| **PCL** | 点云处理 (common, io, filters, kdtree) |
| **Eigen3** | 线性代数与矩阵运算 |
| **yaml-cpp** | YAML 配置文件解析 |
| **Foxglove C++ SDK** | 官方 Foxglove WebSocket 与 protobuf schema 发布（位于 `third_party/foxglove-sdk/`） |
| **bzip2** | Bag 文件 bzip2 解压支持 |
| **lz4** | Bag 文件 lz4 解压支持 |
| **Livox SDK 源码** | 实时 Horizon 连接与 LVX SDK1 数据结构对照；通过 submodule 引用到 `third_party/Livox-SDK`，CMake 会自动构建并链接 `livox_sdk_static` |

安装步骤：

```powershell
# 1. 进入项目目录
cd D:\Projects\livox-fast-lio-windows

# 2. 初始化 vcpkg（首次构建时）
cd third_party\vcpkg
.\bootstrap-vcpkg.bat
cd ..\..

# 3. 安装依赖（使用 manifest 模式自动安装）
# 依赖会在 CMake Configure 阶段自动安装
```

> **注意：** PCL 及其依赖链（boost、flann、qhull 等）安装时间较长（约 15-30 分钟），请耐心等待。

### CMake 配置与编译

```powershell
# 配置项目
cmake -B build -S . -DCMAKE_TOOLCHAIN_FILE=third_party/vcpkg/scripts/buildsystems/vcpkg.cmake

# 多线程编译（根据你的 CPU 核心数调整 -j 参数）
cmake --build build --config Release -j16

# 运行自动化测试
ctest --test-dir build -C Release --output-on-failure

# 可选：生成可直接分发的安装目录
cmake --install build --config Release --prefix dist
```

编译成功后，可执行文件位于：

```
build\Release\livox_fast_lio.exe
```

安装后的程序位于 `dist\bin\livox_fast_lio.exe`，配置位于 `dist\config\`。

> **提示：** CMake 会自动将 `config/` 目录复制到 `build\config/`，并将 PCL、Foxglove SDK 等运行时 DLL 部署到可执行文件旁边；安装目录也包含 `pcl_io.dll` 和 `foxglove_cpp.dll`，无需手工从 vcpkg 复制 DLL。
> 如果 `third_party/Livox-SDK` 存在，CMake 会自动构建 `sdk_core` 并链接真实 Livox SDK；只有 SDK 不存在时才会回退到 `src/livox_sdk_stub.cpp`。

---

## 使用方法

### 命令行参数

```
livox_fast_lio.exe [config_path] [--lvx <file>] [--bag <file>] [key=value ...]
```

| 参数 | 说明 |
|------|------|
| `config_path` | YAML 配置文件路径（默认：`config/horizon.yaml`） |
| `--lvx <file>` | 指定 .lvx 回放文件路径 |
| `--bag <file>` | 指定 .bag 回放文件路径（Livox Viewer 导出的 ROS1 bag） |
| `key=value` | 覆盖配置参数（如 `blind=2`, `acc_cov=0.5`） |
| `--help` / `-h` | 显示帮助信息 |

**示例：**

```powershell
# 使用默认配置，连接实时 LiDAR
.\livox_fast_lio.exe

# 指定配置文件
.\livox_fast_lio.exe D:\path\to\horizon.yaml

# 使用高精度 Horizon 配置（默认 hybrid：Tile + 完整 PointCloud 快照）
.\livox_fast_lio.exe config\horizon_hd.yaml pcd_save_en=false

# 回放 LVX 文件；仅做可视化/性能测试时同时关闭分块 PCD 和最终 ikd-Tree 导出
.\livox_fast_lio.exe config\horizon_hd.yaml --lvx "D:\data\recording.lvx" `
  pcd_save_en=false storage.save_final_ikdtree=false

# 回放 Bag 文件（Livox Viewer 导出）
.\livox_fast_lio.exe --bag D:\data\recording.bag

# 回放 + 指定配置
.\livox_fast_lio.exe config\horizon.yaml --lvx "D:\data\test.lvx"

# 覆盖参数（盲区距离 2m，加速度计协方差 0.5）
.\livox_fast_lio.exe blind=2 acc_cov=0.5

# 覆盖多个参数
.\livox_fast_lio.exe blind=2 max_iter=15 filter_size_map=0.3

# 临时切换地图输出模式
.\livox_fast_lio.exe config\horizon_hd.yaml --lvx "D:\data\test.lvx" `
  map_output.mode=full_async
```

`key=value` 支持旧版顶层覆盖名，也支持 `map_output.mode`、`storage.mode`、
`foxglove.current_cloud_publish_hz` 等分组配置名。若同一选项重复出现，后面的
命令行覆盖值生效。

### 实时 LiDAR 模式

默认模式，连接 Livox Horizon 进行实时 SLAM：

```powershell
cd build\Release
.\livox_fast_lio.exe ..\config\horizon.yaml
```

程序启动后会：
1. 加载 `config/horizon.yaml` 配置
2. 启动 Foxglove WebSocket 服务器（端口 8765）
3. 初始化 Livox SDK，监听内网设备广播并自动连接单台 Horizon
4. 进入采样后接收 LiDAR + IMU 数据，按实时建帧窗口推入 FAST-LIO2
5. 实时输出位姿估计与点云地图

默认按单台 Livox Horizon 设备设计；如果内网里存在多台 Livox 设备，建议使用 `livox_broadcast_code=...` 锁定目标设备，避免多台数据混入同一个建图输入流。

### LVX 回放模式

无需硬件，直接使用 Livox Viewer 或 Livox SDK1 录制的 `.lvx` 文件测试完整 SLAM 流程：

```powershell
.\livox_fast_lio.exe config\horizon.yaml --lvx "D:\data\my_recording.lvx" pcd_save_en=false
```

LVX 回放会读取文件头、设备信息和有效非空帧数；文件尾部的空帧会被当作正常 EOF 处理。回放时按 LVX 的帧间隔和包时间戳以接近真实采集节奏推送点云与 IMU，后续仍走同一套 `LvxPoint/ImuData -> Preprocess -> Sync -> IEKF` 流程；地图构建、Foxglove 和 PCD 输出由后续异步 Worker 处理。

当前 LVX 解析器面向 Livox SDK1 / LVX v1.1 数据，支持以下 `data_type`：

| data_type | 数据类型 |
|-----------|----------|
| 0 | Cartesian |
| 1 | Spherical |
| 2 | Extended Cartesian |
| 3 | Extended Spherical |
| 4 | Dual Extended Cartesian |
| 5 | Dual Extended Spherical |
| 6 | IMU |
| 7 | Triple Extended Cartesian |
| 8 | Triple Extended Spherical |

说明：

- 球坐标点会转换为米制 XYZ 点。
- 双回波/三回波点会拆分为独立点参与建图。
- FAST-LIO 需要 IMU；如果 LVX 中没有 IMU 包，程序会在日志中提示。
- 遇到不支持或截断的数据包时，程序会输出诊断信息，便于判断录制文件或格式问题。

### Bag 回放模式

使用 Livox Viewer 导出的 ROS1 `.bag` 文件进行 SLAM：

```powershell
.\livox_fast_lio.exe --bag D:\data\my_recording.bag
```

程序会：
1. 读取 bag 文件中的 `/livox/lidar` 或 `/livox_lidar`（Livox `CustomMsg`）以及 `/livox/imu` 或 `/livox_imu`（`sensor_msgs/Imu`）数据
2. 按传感器时间戳排序，以 1.0 倍实时速度回放并运行 FAST-LIO2 算法
3. 按 ROS FAST-LIO 的同步策略等待 IMU 覆盖到 LiDAR 帧尾后再处理，避免 LiDAR 帧抢跑
4. 默认启动 Foxglove WebSocket：`ws://localhost:8765`
5. 通过官方 Foxglove C++ SDK 发布 protobuf schema 数据
6. 由 Storage Worker 将 SLAM 结果异步写入输出 bag 文件（`<输入文件名>_output.bag`，如果输出路径可写）

Windows 版会对 Livox `CustomMsg` 执行与 ROS FAST-LIO 相同的关键处理链路：

- 每个 Livox `CustomMsg` 作为一帧 LiDAR 输入
- 预处理执行 `point_filter_num=3`、盲区过滤、tag 过滤、反射率映射到 intensity、`offset_time` 映射到点时间
- LiDAR/IMU 同步等待 `last_imu_time >= lidar_end_time`
- IEKF 使用 FAST-LIO 原版风格的 `h_share_model` 与 `update_iterated_dyn_share_modified(0.001, solve_time)`
- `/cloud_registered` 发布当前帧配准点云；地图 topic 由 `map_output.mode` 决定，`/map` 与 `/map_tiles` 都来自独立的有界累计地图，不受 FAST-LIO 局部 ikd-Tree 裁剪影响

### Foxglove 可视化

#### WebSocket 实时可视化

LVX 和 Bag 回放都会默认启动 Foxglove WebSocket 服务器：

```powershell
.\livox_fast_lio.exe config\horizon.yaml --lvx "D:\data\my_recording.lvx" pcd_save_en=false
.\livox_fast_lio.exe config\horizon.yaml --bag "D:\data\my_recording.bag"
```

1. 打开 Foxglove Studio → **Open Connection**
2. 选择 **Foxglove WebSocket**
3. 输入 `ws://localhost:8765`
4. 添加 3D 面板，将 Fixed frame 设为 `map`
5. `horizon_hd.yaml` 优先显示 `/map_tiles` 与 `/cloud_registered`，同时保留低频 `/map` PointCloud 快照；其他默认配置显示 `/map`。再按需添加 `/odometry`、`/path`、`/tf`

Foxglove 的播放时间会跟随 SLAM 处理时间广播。LVX 和 Bag 回放支持在 Foxglove 中暂停、继续和调整播放速度。当前拖动进度条 seek 到指定时间暂不可用；原因是 FAST-LIO 状态、局部 ikd-Tree、路径和完整累计地图都依赖历史 LiDAR/IMU 顺序积分，不能只移动文件读取位置。

实时 WebSocket topics：

| Topic | Foxglove schema | 说明 |
|-------|-----------------|------|
| `/map` | `foxglove.PointCloud` | `full_async` / `hybrid` 模式的完整累计地图快照（frame：`map`） |
| `/map_tiles` | `foxglove.SceneUpdate` | `tiled_incremental` / `hybrid` 模式的持久化 Tile；相同 Tile ID 原位更新，淘汰时发送删除事件，新客户端连接会重同步当前 Tile |
| `/map_delta` | `foxglove.PointCloud` | 旧版兼容增量 topic；新配置应使用 `map_output.mode` 与 `/map_tiles` |
| `/cloud_registered` | `foxglove.PointCloud` | 当前帧配准点云（frame：`map`） |
| `/odometry` | `foxglove.Odometry` | SLAM 里程计（body frame：`base_link`） |
| `/path` | `foxglove.PosesInFrame` | 运动轨迹 |
| `/tf` | `foxglove.FrameTransforms` | 坐标变换（`base_link` → `map`） |
| `/imu` | `livox_fast_lio/Imu`（JSON） | IMU 数据（frame：`livox_imu`） |

### 地图输出模式

`map_output.mode` 是地图输出行为的权威配置。旧版 `publish.*` 地图字段仍会
在缺少 `map_output` 分组时映射为兼容行为。

| 模式 | Foxglove 地图输出 | 适用场景 |
|------|-------------------|----------|
| `full_async` | 仅 `/map` | 兼容旧布局、小型地图、需要单个完整 PointCloud 的工具 |
| `tiled_incremental` | 仅 `/map_tiles` | 长时间或高密度建图；单次输出量由变化区域决定 |
| `hybrid` | `/map` + `/map_tiles` | 迁移和对照验证，资源开销最高 |

所有模式都在后台累计有界体素地图。Tile 模式按 Dirty Tile 发送更新并对同一
Tile 合并积压；完整地图模式按配置间隔在后台生成快照。`hybrid` 模式会限制
连续 Tile 输出次数，确保等待中的 `/map` 快照不会被持续更新的 Tile 饿死。
网络慢或客户端断开不会让 SLAM 主线程执行点云序列化或网络写入。

## 配置文件说明

默认配置文件：[`config/horizon.yaml`](config/horizon.yaml)

另提供高精度配置：[`config/horizon_hd.yaml`](config/horizon_hd.yaml)。它将 `filter_size_surf` 和 `filter_size_map` 从 `0.5m` 降到 `0.05m`，将 `point_filter_num` 调为 `1`、实时建帧点数阈值提高到 `30000`、IEKF 最大迭代次数调为 `4`、特征点上限调为 `4000`，并默认使用 `hybrid`：`/map_tiles` 持续发布 Dirty Tile，`/map` 以 5 秒为最小间隔保留一个完整 `foxglove.PointCloud` 快照，新客户端连接时还会立即重同步。若快照构建或网络发送较慢，运行时会自动延长实际完整地图间隔，避免输出积压。该配置适合需要更高点云密度的现场建图，但会增加 CPU 和内存压力。地图可视化仍独立使用 `0.2m` Tile 体素，避免把 SLAM 内部精度直接转化为网络负载。

### common — 通用设置

| 参数 | 类型 | 默认值 | 说明 |
|------|------|--------|------|
| `lid_topic` | string | `/livox/lidar` | LiDAR 数据话题（保留字段，Windows 版不使用 ROS 话题） |
| `imu_topic` | string | `/livox/imu` | IMU 数据话题 |
| `livox_broadcast_code` | string | `""` | 实时模式下可选的 Livox 设备广播码过滤；空字符串表示自动连接发现到的设备 |
| `realtime_frame_sec` | double | `0.10` | 实时模式下累计多少秒点云组成一帧 FAST-LIO 输入 |
| `realtime_frame_points` | int | `20000` | 实时模式下累计点数达到该值时提前组成一帧 |
| `time_sync_en` | bool | `false` | 是否启用外部时间同步（仅在无法硬件同步时开启） |
| `time_offset_lidar_to_imu` | double | `0.0` | LiDAR-IMU 时间偏移（秒），由 LI-Init 等工具标定 |

### preprocess — 预处理参数

| 参数 | 类型 | 默认值 | 说明 |
|------|------|--------|------|
| `lidar_type` | int | `1` | LiDAR 类型：1=Livox, 2=Velodyne, 3=Ouster |
| `scan_line` | int | `6` | 扫描线数（Horizon = 6） |
| `blind` | double | `4` | 盲区距离（米），小于此距离的点被过滤 |
| `feature_enabled` | bool | `false` | 是否启用特征提取；Livox Horizon bag 回放通常保持 false |
| `point_filter_num` | int | `3` | Livox 点过滤步长，与 ROS FAST-LIO Horizon 启动参数保持一致 |

### mapping — SLAM 建图参数

| 参数 | 类型 | 默认值 | 说明 |
|------|------|--------|------|
| `acc_cov` | double | `0.1` | 加速度计噪声协方差 |
| `gyr_cov` | double | `0.1` | 陀螺仪噪声协方差 |
| `b_acc_cov` | double | `0.0001` | 加速度计偏置协方差 |
| `b_gyr_cov` | double | `0.0001` | 陀螺仪偏置协方差 |
| `max_iteration` | int | `3` | IEKF 最大迭代次数，与 ROS FAST-LIO Horizon 启动参数保持一致 |
| `max_feature_points` | int | `2000` | 每帧进入 IEKF 的降采样特征点上限；`<=0` 表示不限制 |
| `iekf_match_threads` | int | `4` | IEKF 逐点邻域匹配线程数；`0` 自动使用一半逻辑核心（最多 8），排障时可设为 `1` |
| `filter_size_surf` | double | `0.5` | 当前帧点云体素降采样叶尺寸（米） |
| `filter_size_map` | double | `0.5` | ikd-Tree 地图增量降采样叶尺寸（米） |
| `cube_side_length` | int | `1000` | 局部地图立方体边长（米） |
| `fov_degree` | double | `100` | 视场角（度） |
| `det_range` | double | `260.0` | 最大探测距离（米） |
| `extrinsic_est_en` | bool | `true` | 是否在线估计 IMU-LiDAR 外参 |
| `extrinsic_T` | [3] | `[0.055, 0.022, -0.030]` | 外参平移（LiDAR→IMU，米） |
| `extrinsic_R` | [9] | 单位矩阵 | 外参旋转（LiDAR→IMU，行优先 3x3） |

### publish — 发布设置

| 参数 | 类型 | 默认值 | 说明 |
|------|------|--------|------|
| `path_en` | bool | `true` | 是否发布运动路径 |
| `scan_publish_en` | bool | `true` | 是否发布点云（false 关闭所有点云输出） |
| `dense_publish_en` | bool | `true` | 是否发布密集点云（false 降采样） |
| `scan_bodyframe_pub_en` | bool | `true` | 是否以 IMU 体坐标系发布点云 |
| `publish_full_map` | bool | `true` | 旧版兼容字段；新配置由 `map_output.mode` 决定是否发布 `/map` |
| `async_full_map_publish` | bool | `true` | 旧版兼容字段；完整地图始终在后台构建和发布 |
| `full_map_publish_interval_ms` | int | `1000` | 旧版完整地图间隔；对应 `map_output.full_publish_interval_ms` |
| `full_map_voxel_size` | double | `0.2` | 旧版完整地图输出体素；对应 `map_output.full_voxel_leaf_m` |
| `bag_full_map_periodic` | bool | `false` | 是否在运行中向输出 Bag 周期写入完整 `/map`；false 时仅在结束时写最终地图 |
| `publish_map_delta` | bool | `false` | 旧版兼容字段；新配置使用 `map_output.mode=tiled_incremental` |
| `map_delta_max_pending_points` | int | `200000` | 旧版增量上限；对应 `map_output.max_points_per_update` |
| `foxglove_control_interval_ms` | int | `20` | Foxglove 回放时钟与播放状态的更新间隔（毫秒）；运行时最小 `10` |
| `foxglove_backlog_size` | int | `64` | 旧版兼容字段；对应 `foxglove.backlog_size`，控制 WebSocket 消息 backlog（运行时最小 `8`） |

### map_output — 可扩展地图输出

| 参数 | 类型 | Horizon 默认值 | 说明 |
|------|------|----------------|------|
| `mode` | string | `full_async` | `full_async`、`tiled_incremental` 或 `hybrid`；`horizon_hd.yaml` 默认为 `hybrid` |
| `full_publish_interval_ms` | int | `1000` | `/map` 完整快照最小发布间隔（毫秒，运行时最小 100） |
| `full_voxel_leaf_m` | double | `0.2` | 完整地图输出体素叶尺寸（米），独立于 ikd-Tree 精度 |
| `tile_size_m` | double | `20.0` | 空间 Tile 边长（米） |
| `tile_voxel_leaf_m` | double | `0.2` | Tile 内体素尺寸（米）；`tile_size_m` 必须是其整数倍 |
| `voxel_update_policy` | string | `latest` | 同一体素使用 `first`、`latest` 或 `centroid` 更新 |
| `tile_publish_hz` | int | `10` | Dirty Tile 提取/发布频率 |
| `max_tiles_per_update` | int | `32` | 单个发布批次最多包含的 Tile 数 |
| `max_points_per_update` | int | `200000` | 单个 Tile 发布批次的点数上限 |
| `input_queue_capacity` | int | `64` | 地图构建输入队列最大 Tile 任务数；同 Tile 任务可合并 |
| `input_queue_max_mb` | int | `128` | 地图构建输入队列字节硬上限（MiB） |
| `max_memory_mb` | int | `1024` | 有界累计地图估算内存上限（MiB） |
| `memory_policy` | string | `stop_accumulating` | 达到上限时使用 `stop_accumulating`、`lru` 或 `evict_far`；`spill_to_disk` 尚未实现，会告警并回退为 `stop_accumulating` |

### foxglove — 实时输出队列

| 参数 | 类型 | 默认值 | 说明 |
|------|------|--------|------|
| `current_cloud_publish_hz` | double | `20` | `/cloud_registered` 最大发布频率 |
| `path_publish_hz` | double | `2` | `/path` 最大发布频率 |
| `backlog_size` | int | `64` | Foxglove WebSocket 消息 backlog 大小 |

里程计、TF、当前帧点云、路径和 IMU 使用有界实时 Worker。可覆盖的数据采用
latest-wins，点云过载会记录丢弃计数，不会把网络背压传入 IEKF。

### storage — Bag 与 PCD Worker

| 参数 | 类型 | 默认值 | 说明 |
|------|------|--------|------|
| `mode` | string | `realtime` | `realtime` 在过载时丢弃旧任务并继续 SLAM；`reliable` 不静默丢弃，容量或写入失败会将存储标记为失败 |
| `queue_max_mb` | int | `512` | Bag/PCD 共用队列及 in-flight 负载的字节上限（MiB） |
| `bag_path_publish_hz` | double | `1` | 输出 Bag 中 `/path` 的写入频率 |
| `path_max_points` | int | `100000` | 内存和 Bag 中滚动 Path 的最大点数；Odom 仍逐帧保留 |
| `pcd_format` | string | `binary_compressed` | `binary` 或 `binary_compressed` |
| `pcd_chunk_points` | int | `1000000` | 每个 PCD 分块的点数触发值 |
| `pcd_chunk_frames` | int | `0` | 可选帧数触发值；`0` 表示关闭按帧触发 |
| `save_final_ikdtree` | bool | `true` | 退出时另外导出 `PCD/ikd_tree_map.pcd`；与 `pcd_save_en` 独立 |

### pcd_save — 点云保存

| 参数 | 类型 | 默认值 | 说明 |
|------|------|--------|------|
| `pcd_save_en` | bool | `true` | 是否把配准帧异步写成 `PCD/scans_*.pcd` 分块 |
| `interval` | int | `-1` | 旧版按帧分块参数；仅当未配置 `storage.pcd_chunk_frames` 且值大于 0 时迁移 |

仅设置 `pcd_save_en=false` 不会关闭最终 ikd-Tree 导出。纯回放性能测试通常同时
设置 `storage.save_final_ikdtree=false`。

### runtime — 运行时日志

| 参数 | 类型 | 默认值 | 说明 |
|------|------|--------|------|
| `runtime_pos_log_enable` | bool | `false` | 是否额外输出旧版位置文本日志；结构化 `Log/runtime.txt` 始终生成 |

`Log/runtime.txt` 包含逐帧耗时、输入/输出队列当前值和峰值、Tile 合并/丢弃/
重同步、进程 RSS，以及地图构建、序列化、发送、Bag、PCD 和 flush 的
平均值、P95、P99 汇总。

---

## 项目结构

```
livox-fast-lio-windows/
├── CMakeLists.txt              # CMake 构建配置
├── vcpkg.json                  # vcpkg 依赖清单 (manifest mode)
├── config/
│   ├── avia.yaml               # Livox Avia 配置
│   ├── horizon.yaml            # Livox Horizon 默认配置（0.5m SLAM 体素）
│   ├── horizon_hd.yaml         # Horizon 高精度配置（0.05m，默认 hybrid：Tile + PointCloud 快照）
│   └── mid360.yaml             # Livox Mid-360 配置
├── cmake/
│   └── deploy_runtime_dlls.cmake # 构建后部署 PCL/Foxglove 等运行时 DLL
├── include/
│   ├── bounded_timing_stats.h  # 固定容量耗时窗口与 P95/P99 汇总
│   ├── common_lib.h            # 通用工具函数
│   ├── Exp_mat.h               # 指数映射 / SO3 数学工具
│   ├── so3_math.h              # SO(3) 群运算
│   ├── types.h                 # 基础类型定义 (V3D, M3D, ImuData, MeasureGroup 等)
│   ├── use-ikfom.hpp           # IKFoM 流形状态定义与过程模型
│   ├── IMU_Processing.hpp      # IMU 预处理与反向补偿
│   ├── laser_mapping.h         # SLAM 主循环入口
│   ├── preprocess.h            # 点云预处理（Livox/Velodyne/Ouster）
│   ├── fast_lio_observation.h  # FAST-LIO 点到面观测常量与残差门限
│   ├── lidar_imu_sync.h        # LiDAR/IMU 帧尾覆盖同步判定
│   ├── playback_status.h        # 回放结束状态与日志错误文本处理
│   ├── map_accumulator.h       # Foxglove/Bag 输出用完整累计地图缓存
│   ├── async_map_publisher.h    # 旧版完整地图异步发布器（兼容测试保留）
│   ├── tiled_map_store.h       # 有界 Tile/体素地图、Dirty 与淘汰管理
│   ├── map_build_worker.h      # Tile 输入合并、地图构建和快照屏障
│   ├── foxglove_output_worker.h # 有界地图网络输出队列
│   ├── realtime_foxglove_worker.h # 位姿、当前帧、路径和 IMU 输出队列
│   ├── storage_worker.h        # Bag 与分块 PCD 异步存储队列
│   ├── livox_adapter.h         # Livox SDK 适配器
│   ├── livox_sdk.h             # Livox SDK 兼容头；真实 SDK 存在时转发到官方头文件
│   ├── lvx_reader.h            # LVX 文件读取与回放
│   ├── foxglove_publisher.h    # Foxglove WebSocket 发布器
│   ├── ros_message.h           # ROS 消息序列化/反序列化（header-only）
│   ├── ros_bag.h               # ROS1 Bag 底层解析器
│   ├── bag_reader.h            # Bag 文件高层读取接口
│   ├── bag_writer.h            # Bag 文件写入器
│   ├── yaml_config.h           # YAML 配置加载器
│   └── ikd-Tree/               # 增量式 k-d 树
│       ├── ikd_Tree.h
│       └── ikd_Tree.cpp
├── src/
│   ├── main.cpp                # 程序入口（参数解析）
│   ├── laser_mapping.cpp       # FAST-LIO2 核心 SLAM 循环
│   ├── preprocess.cpp          # 点云预处理实现
│   ├── livox_adapter.cpp       # Livox SDK 适配实现
│   ├── livox_sdk_stub.cpp      # Livox SDK 存根（未找到官方 SDK 时使用）
│   ├── lvx_reader.cpp          # LVX 回放实现
│   ├── async_map_publisher.cpp  # 旧版完整地图异步发布器实现（兼容测试保留）
│   ├── foxglove_publisher.cpp  # 官方 Foxglove SDK WebSocket 发布实现
│   ├── tiled_map_store.cpp     # 有界 Tile 地图实现
│   ├── map_build_worker.cpp    # 异步地图构建实现
│   ├── foxglove_output_worker.cpp # 地图发布 Worker
│   ├── realtime_foxglove_worker.cpp # 实时 topic 发布 Worker
│   ├── storage_worker.cpp      # Bag/PCD 存储 Worker
│   ├── ros_bag.cpp             # Bag 底层解析（解压、record 解析）
│   ├── bag_reader.cpp          # Bag 高层读取（topic 路由、消息分发）
│   ├── bag_writer.cpp          # Bag 写入（chunk、connection、index）
│   └── yaml_config.cpp         # 配置加载实现
├── tests/
│   ├── test_livox_fast_lio.cpp # 配置、回放、地图、存储和输出管线测试
│   └── test_realtime_foxglove_worker.cpp # 实时 Foxglove Worker 测试
├── third_party/
│   ├── foxglove-sdk/           # Foxglove C++ SDK redistributable
│   ├── IKFoM_toolkit/          # 迭代卡尔曼滤波流形工具包
│   ├── Livox-SDK/              # 官方 Livox SDK 源码，用于 LVX SDK1 格式对照
│   └── vcpkg/                  # vcpkg 包管理器
├── Log/                        # 运行时日志输出目录
└── PCD/                        # 点云保存目录
```

---

## 输出文件

程序默认在**启动时的当前工作目录**下创建输出；可通过命令行
`root_dir=D:\path\to\output` 指定独立输出根目录。启动日志会打印最终解析后的
`Output root`。

| 路径 | 内容 |
|------|------|
| `Log/runtime.txt` | 结构化逐帧日志以及输入、地图、Foxglove、Storage、耗时和内存汇总 |
| `Log/imu.txt` | IMU 调试数据 |
| `PCD/scans_*.pcd` | Storage Worker 生成的配准点云分块；结尾不足一块时使用 `scans_final_*.pcd` |
| `PCD/ikd_tree_map.pcd` | `storage.save_final_ikdtree=true` 时退出阶段原子写入的最终局部 ikd-Tree 地图 |

Bag 模式下，输出 bag 文件生成在输入文件同目录：

| 输入文件 | 输出文件 |
|----------|----------|
| `D:\data\test.bag` | `D:\data\test_output.bag` |

PCD 文件可使用以下工具查看：
- [CloudCompare](https://www.cloudcompare.org/) — 开源点云查看器
- [MeshLab](http://www.meshlab.net/) — 3D 网格处理
- PCL Viewer — `pcl_viewer map.pcd`

---

## 连接真实 Livox LiDAR

仓库通过 git submodule 引用 `third_party/Livox-SDK`，CMake 会自动构建官方 SDK1 的 `sdk_core` 并链接到 `livox_fast_lio.exe`。实时模式的连接流程为：

1. 初始化 Livox SDK
2. 监听内网中的 Livox 设备广播
3. 调用 `AddLidarToConnect(...)` 添加目标 Horizon
4. 设备进入 `Normal` 状态后调用 `LidarStartSampling(...)`
5. 将实时点云和 IMU 数据送入 FAST-LIO2

如果你的本地仓库缺少 `third_party/Livox-SDK`，可以初始化官方 SDK submodule：

```powershell
git submodule update --init --recursive third_party/Livox-SDK
```

然后重新配置和编译主工程：

```powershell
cmake -B build -S . -DCMAKE_TOOLCHAIN_FILE=third_party/vcpkg/scripts/buildsystems/vcpkg.cmake
cmake --build build --config Release -j16
```

**网络配置：** 确保 PC 网卡和 Horizon 在同一网段，且防火墙允许 Livox SDK 的 UDP 广播与数据端口。你可以先用 `ping <设备IP>` 确认基础网络连通。单台设备场景下无需指定 IP，SDK 会通过广播自动发现设备。

实时建图命令：

```powershell
.\build\Release\livox_fast_lio.exe .\config\horizon.yaml pcd_save_en=false
```

高精度建图命令：

```powershell
.\build\Release\livox_fast_lio.exe .\config\horizon_hd.yaml pcd_save_en=false
```

启动日志中出现以下内容时，表示设备采样链路已经起来：

```text
[LivoxAdapter] Broadcast: code=...
[LivoxAdapter] Added LiDAR handle=...
[LivoxAdapter] Start sampling callback status=0 ... response=0
```

---

## 常见问题

### Q: 运行时提示 "config/horizon.yaml not found"

确保从正确的工作目录运行，或指定完整路径：

```powershell
# 方式 1：从项目根目录运行并显式指定配置
.\build\Release\livox_fast_lio.exe .\config\horizon.yaml

# 方式 2：从 build/Release 使用已复制到 build/config 的配置
cd build\Release
.\livox_fast_lio.exe ..\config\horizon.yaml
```

### Q: 提示 "SDK init failed"

这通常表示程序没有成功链接或初始化真实 Livox SDK。请检查：

1. `third_party/Livox-SDK` 是否存在，且包含 `sdk_core/include/livox_sdk.h`
2. CMake 配置日志中是否出现 `Livox SDK headers found; building sdk_core from source.`
3. 构建产物中是否生成 `build\Livox-SDK\sdk_core\Release\livox_sdk_static.lib`
4. 是否重新执行过 `cmake -B build -S . ...` 和 `cmake --build build --config Release`

LVX 和 Bag 回放不依赖实时 Livox SDK 连接，可正常离线使用。

### Q: Foxglove Studio 无法连接 WebSocket

1. 确认程序正在运行（WebSocket 服务器在 SLAM 启动后开启）
2. 检查端口 8765 是否被防火墙阻止
3. 尝试使用 `ws://127.0.0.1:8765` 连接
4. 确认启动日志中出现 `Foxglove WebSocket: ws://localhost:8765`
5. 如果端口已被占用，关闭占用 8765 的程序后重新启动

### Q: Foxglove 能显示点云，但点云杂乱或发布频率不对

1. 先确认打开的是本程序的实时连接 `ws://localhost:8765`，不是旧的、不完整的输出 bag 文件
2. 3D 面板的 Fixed frame 应设为 `map`
3. 查看启动日志中的 `map_output_mode`：`full_async` 选择 `/map`，`tiled_incremental` 选择 `/map_tiles`，`hybrid` 两者都可；`/cloud_registered` 只是当前帧配准点云
4. 正常 Livox Horizon bag 回放应接近 10Hz LiDAR 帧率；日志中每帧通常会出现约 `19-21` 条 IMU 样本。LVX v1.1 回放的单帧点数和 IMU 数量取决于录制帧间隔，Horizon 常见 50ms 帧可见约 `10` 个 IMU 包
5. 若日志中反复出现 `No undistorted points`、`No effective points` 或 IMU 数量长期为 0，说明 LiDAR/IMU 时间戳、bag topic 或 LVX 数据包解析仍有问题

当前 Windows 版已按 ROS FAST-LIO 参考链路处理 bag 与 LVX 回放：Livox `CustomMsg` 或 LVX 内部点/IMU 数据预处理、LiDAR/IMU 帧尾覆盖同步、FAST-LIO 原版风格 IEKF 点到面观测更新；地图按 `map_output.mode` 在后台发布完整快照、Dirty Tile 或两者。

### Q: 启动时提示找不到 `pcl_io.dll` 或其他 DLL

不要只复制 `livox_fast_lio.exe`。重新执行构建会把运行时 DLL 部署到
`build\Release\`：

```powershell
cmake --build build --config Release
```

需要分发时使用安装目录：

```powershell
cmake --install build --config Release --prefix dist
.\dist\bin\livox_fast_lio.exe --help
```

当前 CMake 安装规则会连同 `pcl_io.dll`、`foxglove_cpp.dll` 及其他 PCL
依赖一起安装。如果使用旧构建目录，建议先重新运行 CMake configure，避免
新 EXE 与旧 DLL 混用。

### Q: 回放结束时出现非法内存访问或进程不退出

旧版本会在 Windows 进程退出阶段触发第三方 ikd-Tree 后台 rebuild 的不安全
析构。当前版本已把该树作为进程生命周期对象，并在它之前有界停止地图、
Foxglove、Bag 和 PCD Worker。请重新编译主程序，不要继续运行旧 EXE。

### Q: Foxglove 播放条能暂停或拖动进度吗?

LVX 和 Bag 回放的 WebSocket 连接会声明 Foxglove `PlaybackControl` 能力。当前支持：

- 播放条时间随 SLAM 当前处理时间前进
- 在 Foxglove 中暂停和继续回放
- 调整播放倍速

当前暂不支持拖动播放进度条 seek 到指定时间点。要实现真正 seek，需要在收到 seek 请求后重置 SLAM 状态、清空旧缓存和地图，并从目标时间重新建立一致的滤波状态与地图。

### Q: LVX 文件没有 IMU 能建图吗?

不能。FAST-LIO 依赖 LiDAR 与 IMU 融合，LVX 文件需要包含 `data_type=6` 的 IMU 包。缺少 IMU 时，程序会记录提示，建议重新录制包含 IMU 的 LVX，或使用包含 `/livox/imu` 的 bag。

### Q: LVX 启动日志中的帧数为什么和文件尾部记录不完全一致?

程序会按帧偏移检查并统计有效非空帧，尾部空帧会被忽略。这类空尾帧在部分 Livox Viewer 录制文件中是正常现象，不会影响前面有效数据的回放。

### Q: 编译时出现 C4819 Unicode 警告

确保所有源文件保存为 UTF-8 编码（无 BOM）。在 Visual Studio 中：文件 → 另存为 → 编码选择 "UTF-8 (无签名)"。

### Q: PCL 安装非常慢

PCL 依赖链包含 boost（66 个包）、flann、qhull 等，首次安装约需 15-30 分钟。这是正常现象，后续 CMake Configure 不会重复安装。

---

## 许可证

本项目基于 FAST-LIO2 原始代码，遵循 [GPLv2](https://github.com/hku-mars/FAST_LIO/blob/master/LICENSE) 许可证。

## 致谢

- [FAST-LIO2](https://github.com/hku-mars/FAST_LIO) — 香港大学 MARS 实验室
- [ikd-Tree](https://github.com/hku-mars/ikd-Tree) — 增量式 k-d 树
- [IKFoM](https://github.com/hku-mars/IKFoM) — 流形上的迭代卡尔曼滤波
- [Livox SDK](https://github.com/Livox-SDK/Livox-SDK) — 览沃科技
- [Foxglove](https://foxglove.dev/) — foxglove机器人数据可视化平台
- [ROS Bag Format](http://wiki.ros.org/Bags/Format/2.0) — ROS1 bag 文件格式规范
