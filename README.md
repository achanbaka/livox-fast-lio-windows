# Livox FAST-LIO Windows

[English README](Readme_EN.md)

**Windows 原生 FAST-LIO2 SLAM 应用** — 支持 Livox Horizon LiDAR 实时建图、Livox SDK1 / LVX v1.1 回放与 ROS1 Bag 回放。
![运行演示](Screen_Record.gif)

基于 [FAST-LIO2](https://github.com/hku-mars/FAST_LIO) 算法，去除 ROS 依赖，原生运行于 Windows 平台。支持直接解析 Livox SDK1 `.lvx` 录制文件、读取/写入 ROS1 Bag 文件，并可通过 [Foxglove Studio](https://foxglove.dev/) 实时查看建图过程。

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
- **Foxglove Studio 可视化** — 回放时通过 `ws://localhost:8765` 实时查看完整累计地图、当前帧点云、里程计、路径、TF 变换
- **回放播放控制** — Foxglove 播放条可同步当前时间，支持暂停、继续和倍速播放；拖动 seek 暂不可用
- **PCD 点云保存** — 自动保存全局地图为 PCD 文件
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
```

编译成功后，可执行文件位于：

```
build\Release\livox_fast_lio.exe
```

> **提示：** CMake 会自动将 `config/` 目录复制到 `build\config/`，并将所有依赖 DLL 部署到 `build\Release/`。
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

# 使用高精度 Horizon 配置
.\livox_fast_lio.exe config\horizon_hd.yaml pcd_save_en=false

# 回放 LVX 文件，长时间数据建议先关闭 PCD 保存
.\livox_fast_lio.exe config\horizon.yaml --lvx "D:\data\recording.lvx" pcd_save_en=false

# 回放 Bag 文件（Livox Viewer 导出）
.\livox_fast_lio.exe --bag D:\data\recording.bag

# 回放 + 指定配置
.\livox_fast_lio.exe config\horizon.yaml --lvx "D:\data\test.lvx"

# 覆盖参数（盲区距离 2m，加速度计协方差 0.5）
.\livox_fast_lio.exe blind=2 acc_cov=0.5

# 覆盖多个参数
.\livox_fast_lio.exe blind=2 max_iter=15 filter_size_map=0.3
```

### 实时 LiDAR 模式

默认模式，连接 Livox Horizon 进行实时 SLAM：

```powershell
cd build\Release
.\livox_fast_lio.exe
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

LVX 回放会读取文件头、设备信息和有效非空帧数；文件尾部的空帧会被当作正常 EOF 处理。回放时按 LVX 的帧间隔和包时间戳以接近真实采集节奏推送点云与 IMU，后续仍走同一套 `LvxPoint/ImuData -> Preprocess -> Sync -> IEKF -> Foxglove/PCD` 流程。

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
6. 将 SLAM 结果写入输出 bag 文件（`<输入文件名>_output.bag`，如果输出路径可写）

Windows 版会对 Livox `CustomMsg` 执行与 ROS FAST-LIO 相同的关键处理链路：

- 每个 Livox `CustomMsg` 作为一帧 LiDAR 输入
- 预处理执行 `point_filter_num=3`、盲区过滤、tag 过滤、反射率映射到 intensity、`offset_time` 映射到点时间
- LiDAR/IMU 同步等待 `last_imu_time >= lidar_end_time`
- IEKF 使用 FAST-LIO 原版风格的 `h_share_model` 与 `update_iterated_dyn_share_modified(0.001, solve_time)`
- `/cloud_registered` 发布当前帧配准点云；默认 `publish_full_map=true` 时，`/map` 发布独立累计的完整全局地图，不再受 FAST-LIO 局部 ikd-Tree 裁剪影响

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
5. 优先显示 `/map` 与 `/cloud_registered`，再按需添加 `/odometry`、`/path`、`/tf`

Foxglove 的播放时间会跟随 SLAM 处理时间广播。LVX 和 Bag 回放支持在 Foxglove 中暂停、继续和调整播放速度。当前拖动进度条 seek 到指定时间暂不可用；原因是 FAST-LIO 状态、局部 ikd-Tree、路径和完整累计地图都依赖历史 LiDAR/IMU 顺序积分，不能只移动文件读取位置。

实时 WebSocket topics：

| Topic | Foxglove schema | 说明 |
|-------|-----------------|------|
| `/map` | `foxglove.PointCloud` | 完整累计 FAST-LIO 地图（frame：`map`，默认不随局部 ikd-Tree 裁剪删除历史区域） |
| `/cloud_registered` | `foxglove.PointCloud` | 当前帧配准点云（frame：`map`） |
| `/odometry` | `foxglove.Odometry` | SLAM 里程计（body frame：`base_link`） |
| `/path` | `foxglove.PosesInFrame` | 运动轨迹 |
| `/tf` | `foxglove.FrameTransforms` | 坐标变换（`base_link` → `map`） |

## 配置文件说明

默认配置文件：[`config/horizon.yaml`](config/horizon.yaml)

另提供高精度配置：[`config/horizon_hd.yaml`](config/horizon_hd.yaml)。它将 `filter_size_surf` 和 `filter_size_map` 从 `0.5m` 降到 `0.2m`，并将 `point_filter_num` 调为 `1`，适合需要更高点云密度的现场建图，但会增加 CPU 和内存压力。

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
| `publish_full_map` | bool | `true` | `/map` 是否发布完整累计地图；false 时回退为局部 ikd-Tree 地图 |

### pcd_save — 点云保存

| 参数 | 类型 | 默认值 | 说明 |
|------|------|--------|------|
| `pcd_save_en` | bool | `true` | 是否保存 PCD 文件 |
| `interval` | int | `-1` | 每个 PCD 文件保存的帧数。`-1` = 所有帧保存到一个文件 |

### runtime — 运行时日志

| 参数 | 类型 | 默认值 | 说明 |
|------|------|--------|------|
| `runtime_pos_log_enable` | bool | `false` | 是否输出更详细的每帧耗时、点数和地图大小日志 |

---

## 项目结构

```
livox-fast-lio-windows/
├── CMakeLists.txt              # CMake 构建配置
├── vcpkg.json                  # vcpkg 依赖清单 (manifest mode)
├── config/
│   ├── horizon.yaml            # Livox Horizon 默认配置（0.5m 体素）
│   └── horizon_hd.yaml         # Livox Horizon 高精度配置（0.2m 体素）
├── include/
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
│   ├── map_accumulator.h       # Foxglove/Bag 输出用完整累计地图缓存
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
│   ├── foxglove_publisher.cpp  # 官方 Foxglove SDK WebSocket 发布实现
│   ├── ros_bag.cpp             # Bag 底层解析（解压、record 解析）
│   ├── bag_reader.cpp          # Bag 高层读取（topic 路由、消息分发）
│   ├── bag_writer.cpp          # Bag 写入（chunk、connection、index）
│   └── yaml_config.cpp         # 配置加载实现
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

程序运行时会在 `ROOT_DIR`（源码根目录）下创建：

| 路径 | 内容 |
|------|------|
| `Log/` | 运行日志（IMU 数据、位姿估计等） |
| `PCD/` | 全局地图 PCD 文件 |

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
# 方式 1：从 build/Release 运行（config 已自动复制）
cd build\Release
.\livox_fast_lio.exe

# 方式 2：指定配置文件路径
.\livox_fast_lio.exe D:\Projects\livox-fast-lio-windows\config\horizon.yaml
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
3. 建图过程优先查看 `/map`；默认配置下它是完整累计地图。`/cloud_registered` 是当前帧配准点云，不是完整累计地图
4. 正常 Livox Horizon bag 回放应接近 10Hz LiDAR 帧率；日志中每帧通常会出现约 `19-21` 条 IMU 样本。LVX v1.1 回放的单帧点数和 IMU 数量取决于录制帧间隔，Horizon 常见 50ms 帧可见约 `10` 个 IMU 包
5. 若日志中反复出现 `No undistorted points`、`No effective points` 或 IMU 数量长期为 0，说明 LiDAR/IMU 时间戳、bag topic 或 LVX 数据包解析仍有问题

当前 Windows 版已按 ROS FAST-LIO 参考链路处理 bag 与 LVX 回放：Livox `CustomMsg` 或 LVX 内部点/IMU 数据预处理、LiDAR/IMU 帧尾覆盖同步、FAST-LIO 原版风格 IEKF 点到面观测更新、`/map` 每帧发布完整累计地图。

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
