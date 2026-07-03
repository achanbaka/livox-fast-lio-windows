# Livox SDK 1 LVX v1.1 回放建图设计

日期：2026-06-25
状态：已选择方案 A，等待用户审阅设计

## 目标

支持 Livox SDK 1 / LVX v1.1 录制的 `.lvx` 文件回放，并复用当前已经验证正常的 bag 建图链路。目标设备范围包括 Horizon、Mid-40、Mid-70 等 LVX v1.1 标准支持的 Livox SDK 1 设备；暂不要求支持 Livox SDK 2 / livox_ros_driver2 规范的设备或文件：

```powershell
livox_fast_lio.exe config\horizon.yaml --lvx "D:\data\my_recording.lvx"
```

程序启动后默认开启 Foxglove WebSocket：`ws://localhost:8765`。Foxglove Studio 连接后，应能像 bag 回放一样实时看到 FAST-LIO 建图过程，包括 `/map`、`/cloud_registered`、`/odometry`、`/path`、`/tf`。

## 范围

第一阶段支持 Livox SDK 1 时代的 LVX v1.1 文件，而不是只支持 Horizon。FAST-LIO 建图仍要求 LVX 文件中存在可用 IMU 数据；仅有点云、没有 IMU 的文件应给出清晰提示，不进入误导性的建图流程。

必须支持：

- LVX v1.1 文件头、设备信息、frame header、packet header 解析。
- LVX v1.1 标准点云包类型识别与归一化，覆盖 Livox SDK 1 设备常见数据类型。
- IMU 包 `data_type=6`，用于 FAST-LIO LiDAR/IMU 紧耦合建图。
- 1.0x 实时回放。
- 与 bag 模式一致的 FAST-LIO 主链路和 Foxglove 输出。

明确不支持：

- Livox SDK 2 / livox_ros_driver2 规范的设备或文件格式。
- 非 LVX v1.1 容器格式。
- 缺少 IMU 的 LVX 文件直接进行 FAST-LIO 建图。
- 在第一阶段实现 LVX 到 bag 的导出工具。

## 当前发现

项目已有 `LvxReader`，并且 `laser_mapping.cpp` 中已经把 LVX 输入接入到和 bag 相同的主链路：

```text
LvxReader
  -> LvxPoint / ImuData
  -> preprocessLivoxPoints
  -> pushLidarFrame / pushImuSample
  -> sync_packages
  -> FAST-LIO IEKF
  -> FoxglovePublisher
```

bag 模式已经验证正常，因此 LVX 第一阶段不应新建一条独立 SLAM 链路。需要把 LVX reader 修成“等价于 Livox SDK 1 点云包 + IMU 输入”的数据源，让后续同步、预处理、IEKF 和 Foxglove 发布都复用已验证代码。

当前 LVX 代码的主要风险点：

- 帧时间和点时间的基准需要严格一致，否则 `sync_packages` 会因为 IMU 未覆盖 LiDAR 帧尾而无法稳定处理。
- LVX v1.1 有多种点云 `data_type`，不同设备和回波模式的 payload 布局不同，不能只按 Horizon Extended Cartesian 解析。
- 点格式需要保留 `reflectivity`，有 `tag` 时保留 `tag`，并补齐适合 FAST-LIO 去畸变使用的 `offset_time`。
- LVX 点包没有 bag `CustomPoint.line` 的同等字段时，第一阶段使用 `line=0`，但必须确认不会被预处理的 `scan_line=6` 过滤掉。
- 如果 LVX 文件中没有 IMU 包，应明确报错或提示无法建图，而不是静默播放杂乱点云。

## 设计方案

采用方案 A：LVX 回放完全复用 bag FAST-LIO 主链路。

核心思路：

```text
LVX 文件
  -> Livox SDK 1 / LVX v1.1 packet parser
  -> 归一化 LvxPoint + ImuData
  -> 现有 preprocessLivoxPoints
  -> 现有 LiDAR/IMU 帧尾覆盖同步
  -> 现有 FAST-LIO 官方风格 IEKF 更新
  -> 现有 Foxglove SDK WebSocket 输出
```

这样可以避免 bag 与 LVX 的建图行为分叉。后续如果要支持更多 Livox 设备，也应优先扩展 `LvxReader` 的输入归一化能力，而不是复制 SLAM 主循环。

## LVX 文件解析

`LvxReader::open` 负责读取并校验：

- public header 签名为 `livox_tech`。
- magic code 为 `0xAC0EA767`。
- private header 中的 `frame_duration` 和 `device_count`。
- 每个 `LvxDeviceInfo` 的设备类型、外参标记和序列号。

`LvxReader::readFrame` 负责：

- 按 `LvxFrameHeader.current_offset` 和 `next_offset` 读取一个 LVX frame。
- 对异常 frame size 做边界检查。
- 将 frame data 交给 `parseFrameData`。
- 如果 frame 中解析出点云或 IMU，则触发 `frame_cb_`。

`LvxReader::parseFrameData` 第一阶段按 LVX v1.1 标准识别 SDK 1 packet 类型，并将可用于建图的点云包归一化为 `LvxPoint`：

| data_type | 名称 | 用途 |
|-----------|------|------|
| `0` | Cartesian | Mid-40 等 SDK 1 设备常见单回波笛卡尔点云 |
| `1` | Spherical | SDK 1 球坐标点云，需转换为笛卡尔点 |
| `2` | Extended Cartesian | Horizon、Mid-70 等 SDK 1 设备常见扩展笛卡尔点云 |
| `3` | Extended Spherical | SDK 1 扩展球坐标点云，需转换为笛卡尔点 |
| `4` | Dual Cartesian | 双回波笛卡尔点云，拆分为多个 `LvxPoint` |
| `5` | Dual Spherical | 双回波球坐标点云，转换并拆分为多个 `LvxPoint` |
| `6` | IMU | FAST-LIO IMU 输入 |
| `7` | Triple Cartesian | 三回波笛卡尔点云，拆分为多个 `LvxPoint` |
| `8` | Triple Spherical | 三回波球坐标点云，转换并拆分为多个 `LvxPoint` |

未知 data type 可以跳过并记录统计，不作为错误中断整个文件；但如果整段 LVX 中没有任何可归一化点云包，应返回明确错误。

## 点云归一化

LVX v1.1 点云包统一解析为 `LvxPoint`：

- 笛卡尔格式：`x/y/z` 由 `int32 mm` 转为 `float meters`。
- 球坐标格式：按 SDK 1/LVX v1.1 定义将距离和角度转换为 `x/y/z` 米制坐标。
- `reflectivity`：映射到 FAST-LIO/PCL intensity。
- `tag`：有 tag 字段时保留 Livox tag，交给预处理做 tag 过滤；没有 tag 字段时填 `0`。
- `line`：LVX v1.1 点包若没有可靠 line 字段，第一阶段统一填 `0`。
- 多回波格式：每个回波拆成独立 `LvxPoint`，保留对应回波的坐标和反射率。
- `offset_time`：相对当前 LiDAR 帧起点的纳秒偏移。

`offset_time` 的原则：

- frame 起点使用该 LVX frame 内第一条非 IMU 点云包的 packet timestamp。
- packet 内点偏移使用设备类型和 data type 对应的点频率推算；无法精确判断时使用 LVX frame 内 packet timestamp 与点序号的稳定线性分布。
- 最终 `offset_time` 必须是当前 LiDAR 帧内的非负值，并且同一帧内整体单调。

预处理仍使用现有 `preprocessLivoxPoints`，以保持与 bag 模式一致：

- `point_filter_num=3`
- 盲区过滤
- tag 过滤
- reflectivity 到 intensity
- `offset_time` 到 curvature

## IMU 归一化

LVX IMU 包解析为 `ImuData`：

- `timestamp`：和 LiDAR 使用同一个 LVX 时间基准，单位为秒。
- `gyro`：按 LVX payload 中的 float 顺序读取。
- `acc`：按 LVX payload 中的 float 顺序读取。

所有 IMU 样本通过现有 `pushImuSample` 进入 `imu_buffer`。`sync_packages` 会等待：

```text
last_imu_time >= lidar_end_time
```

只有满足覆盖条件后才处理 LiDAR 帧。这样 LVX 与 bag 使用同一套 LiDAR/IMU 同步语义。

## 时间与回放节奏

LVX 回放第一阶段采用 1.0x 实时节奏。

优先级：

1. 使用 LVX packet timestamp 推导帧间时间。
2. 如果 frame 内时间不足以推导，使用 private header 的 `frame_duration`。
3. 对异常时间跳变记录警告，并保持单调回放。

FAST-LIO 和 Foxglove 发布使用 LiDAR 帧尾时间：

- `/cloud_registered`
- `/map`
- `/odometry`
- `/path`
- `/tf`
- Foxglove broadcast time

## Foxglove 输出

LVX 模式不单独实现发布逻辑，完全沿用 bag 模式已经验证的 `FoxglovePublisher`：

| Topic | 内容 |
|-------|------|
| `/map` | 累积 ikd-Tree 地图，每个处理后的 LiDAR 帧发布 |
| `/cloud_registered` | 当前帧配准点云 |
| `/odometry` | FAST-LIO 位姿 |
| `/path` | 运动轨迹 |
| `/tf` | `map -> base_link` 变换 |

Foxglove 使用方式与 bag 模式一致：

- 连接 `ws://localhost:8765`
- 3D 面板 Fixed frame 设置为 `map`
- 优先观察 `/map` 和 `/cloud_registered`

## 日志与错误处理

启动 LVX 模式时输出：

- LVX 文件路径
- LVX version
- frame duration
- device count
- SDK 1 设备信息，包括 device type、broadcast code、外参标记
- 是否发现点云包
- 是否发现 IMU 包
- Foxglove WebSocket URL

异常处理：

- 文件头不合法：打开失败并打印明确错误。
- 未发现任何 LVX v1.1 / SDK 1 可归一化点云包：打开或播放阶段报错，提示当前只支持 SDK 1 LVX v1.1 点云包。
- 未发现 IMU 包：提示该 LVX 文件无法进行 FAST-LIO LiDAR/IMU 建图。
- packet 截断：跳过当前 frame 并记录警告；连续异常过多时停止播放。
- 时间戳回退：记录警告，并清理同步缓冲，避免把错序 IMU/LiDAR 送入 FAST-LIO。

## 测试策略

自动测试：

- LVX packet parser 能识别 SDK 1 / LVX v1.1 标准 `data_type=0..8`。
- 点云归一化覆盖 `data_type=0/2/4/7` 笛卡尔格式。
- 球坐标格式 `data_type=1/3/5/8` 按 LVX v1.1 标准转换为笛卡尔点。
- 点云单位从 mm 转为 m。
- `reflectivity` 保留并最终映射到 intensity。
- `offset_time` 在单帧内非负且单调。
- LVX IMU 和 LiDAR 时间使用同一时间基准。
- 缺少 IMU 时返回明确错误状态或日志标志。

手动验证：

1. 使用 Livox SDK 1 / LVX v1.1 文件运行：

   ```powershell
   .\build\Release\livox_fast_lio.exe .\config\horizon.yaml --lvx "D:\data\sdk1_recording.lvx" pcd_save_en=false
   ```

2. 连接 Foxglove：`ws://localhost:8765`。
3. Fixed frame 设置为 `map`。
4. 确认 `/cloud_registered` 和 `/map` 实时更新。
5. 确认日志中 LiDAR 帧率接近 LVX 原始帧率，且有 IMU 的文件每帧 IMU 数量稳定。
6. 确认 FAST-LIO 能进入 IEKF 更新，并出现有效点数量。

## 验收标准

第一阶段完成时应满足：

- Livox SDK 1 / LVX v1.1 文件可通过 `--lvx` 打开并播放。
- Foxglove 可通过 `ws://localhost:8765` 实时看到建图过程。
- `/map` 与 `/cloud_registered` 的行为和 bag 模式一致。
- 日志不再出现长期 IMU 为 0、LiDAR 抢跑或无法覆盖帧尾的问题。
- 有 IMU 的 SDK 1 LVX 文件能稳定进入 FAST-LIO IEKF 更新。
- 缺少 IMU、非 LVX v1.1 或 SDK2 数据时给出清晰提示，而不是输出误导性的杂乱点云。

## 非目标

- 不支持 Livox SDK 2 / livox_ros_driver2 规范。
- 不新增 ROS 运行时依赖。
- 不重写 FAST-LIO 主循环。
- 不改变 bag 模式已经验证正常的行为。
- 不在第一阶段实现 LVX 转 bag 工具。
