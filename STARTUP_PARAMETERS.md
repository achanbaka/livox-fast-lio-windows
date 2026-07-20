# Livox FAST-LIO Windows 启动参数使用说明

本文档对应当前版本的 `livox_fast_lio.exe`。启动参数以
`src/main.cpp` 和 `src/yaml_config.cpp` 中的实际解析逻辑为准。

核对基线：Git 提交 `d1a5507`，文档更新日期 `2026-07-21`。

## 1. 命令格式

```text
livox_fast_lio.exe [config_path] [--lvx <file>] [--bag <file>] [key=value ...]
```

| 参数 | 是否必需 | 说明 |
|---|---:|---|
| `config_path` | 否 | YAML 配置文件路径。未指定时查找 `config/horizon.yaml` |
| `--lvx <file>` | 否 | 回放 Livox SDK1 `.lvx` 文件 |
| `--bag <file>` | 否 | 回放 ROS1 `.bag` 文件，并生成对应的输出 Bag |
| `key=value` | 否 | 在 YAML 加载完成后临时覆盖一个受支持的配置项 |
| `--help`、`-h` | 否 | 输出程序内置帮助并退出 |

不要同时指定 `--lvx` 和 `--bag`。当前解析器没有显式拒绝这种组合，但两种输入源并不是设计为同时使用的。

## 2. 从哪里启动

### 从项目根目录启动

```powershell
cd D:\Projects\livox-fast-lio-windows
.\build\Release\livox_fast_lio.exe .\config\horizon.yaml
```

### 从 `build\Release` 启动

```powershell
cd D:\Projects\livox-fast-lio-windows\build\Release
.\livox_fast_lio.exe ..\..\config\horizon.yaml
```

未设置 `root_dir` 时，`Log` 和 `PCD` 输出目录位于程序启动时的当前工作目录，而不是固定在可执行文件目录。

路径包含空格时必须使用双引号：

```powershell
--lvx "D:\Livox Horizon\record files\test.lvx"
```

PowerShell 多行命令使用反引号 `` ` `` 续行。反引号必须是该行最后一个字符，后面不能有空格。

## 3. 输入模式

### 3.1 实时 Livox Horizon

不指定 `--lvx` 或 `--bag` 时进入实时传感器模式：

```powershell
.\build\Release\livox_fast_lio.exe .\config\horizon.yaml
```

如果同一网络中存在多台 Livox 设备，可通过广播码选择设备：

```powershell
.\build\Release\livox_fast_lio.exe .\config\horizon.yaml `
  livox_broadcast_code=1HDDH3200100031
```

实时建帧由两个阈值共同控制，满足任意一个即可提前形成 FAST-LIO 输入帧：

```powershell
realtime_frame_sec=0.10 realtime_frame_points=20000
```

### 3.2 LVX 回放

```powershell
.\build\Release\livox_fast_lio.exe .\config\horizon.yaml `
  --lvx "D:\data\recording.lvx"
```

高密度配置：

```powershell
.\build\Release\livox_fast_lio.exe .\config\horizon_hd.yaml `
  --lvx "D:\data\recording.lvx"
```

### 3.3 ROS1 Bag 回放与输出

```powershell
.\build\Release\livox_fast_lio.exe .\config\horizon.yaml `
  --bag "D:\data\recording.bag"
```

输入文件为 `recording.bag` 时，输出 Bag 默认写到同目录下的
`recording_output.bag`。不要把输入文件命名为已经带 `_output` 的同一路径。

## 4. 常用启动方案

### 标准 Horizon 建图

```powershell
.\build\Release\livox_fast_lio.exe .\config\horizon.yaml `
  --lvx "D:\data\recording.lvx"
```

`horizon.yaml` 默认使用 `map_output.mode=full_async`，通过 `/map` 发布后台构建的完整地图。

### Horizon HD 建图

```powershell
.\build\Release\livox_fast_lio.exe .\config\horizon_hd.yaml `
  --lvx "D:\data\recording.lvx"
```

`horizon_hd.yaml` 默认使用 `hybrid`，同时发布：

- `/map_tiles`：Dirty Tile 增量地图，消息类型为 `foxglove.SceneUpdate`；
- `/map`：低频完整地图，消息类型为 `foxglove.PointCloud`。

### 长时间建图，优先降低网络和完整地图开销

```powershell
.\build\Release\livox_fast_lio.exe .\config\horizon_hd.yaml `
  --lvx "D:\data\recording.lvx" `
  map_output.mode=tiled_incremental
```

该模式只发布 `/map_tiles`，不再周期构建兼容 `/map` 快照，适合长距离和高密度建图。使用前应确认 Foxglove 布局已经添加 Scene 面板并订阅 `/map_tiles`。

### 兼容传统 `/map` 显示

```powershell
.\build\Release\livox_fast_lio.exe .\config\horizon_hd.yaml `
  --lvx "D:\data\recording.lvx" `
  map_output.mode=full_async `
  map_output.full_publish_interval_ms=5000
```

### 纯回放性能测试

```powershell
.\build\Release\livox_fast_lio.exe .\config\horizon_hd.yaml `
  --lvx "D:\Livox Horizon\Livox Viewer 0.11.0\data\record files\2026-03-03 20-20-05.lvx" `
  pcd_save_en=false `
  storage.save_final_ikdtree=false `
  bag_full_map_periodic=false `
  map_output.mode=tiled_incremental
```

注意：`pcd_save_en=false` 只关闭分块扫描 PCD，不能关闭程序退出时的最终 ikd-Tree 地图导出；纯性能测试应同时设置 `storage.save_final_ikdtree=false`。

### 指定 8C/16T CPU 的 IEKF 匹配线程

```powershell
iekf_match_threads=4
```

当前 `horizon_hd.yaml` 默认值为 `4`。设置为 `0` 时自动使用一半逻辑处理器、最多 `8` 个线程；设置为 `1` 可用于排查并行相关问题。编译命令中的 `-j 16` 只影响编译并行度，与运行时 `iekf_match_threads` 无关。

## 5. `key=value` 解析规则

1. 参数必须写成 `key=value`，不要写成 `--key=value`。
2. 参数顺序基本不受限制，但建议按“配置文件、输入文件、覆盖参数”的顺序书写。
3. 同一个键出现多次时，后面的值覆盖前面的值。
4. 键名区分大小写，且只支持下表列出的白名单。未知键目前会被静默忽略。
5. 布尔值建议只使用 `true` 和 `false`。解析器将 `true` 或 `1` 视为真，其他字符串均视为假。
6. 数值格式错误时会输出 `Invalid override`；最终配置仍会继续经过合法性校验。
7. 带点号的分组键是完整键名，例如 `map_output.mode=hybrid`，不能省略前缀。
8. 启动参数只对本次运行生效，不会改写 YAML 文件。

## 6. 当前支持的命令行覆盖参数

### 6.1 实时输入与 SLAM

| 参数 | 类型 | Horizon 默认值 | 作用 |
|---|---|---:|---|
| `livox_broadcast_code` | string | 空 | 实时模式下选择指定 Livox 设备 |
| `realtime_frame_sec` | double | `0.10` | 实时点云累计时间阈值，单位秒 |
| `realtime_frame_points` | int | `20000` | 实时点云累计点数阈值；HD 默认 `30000` |
| `blind` | double | `4` | LiDAR 盲区距离，单位米 |
| `point_filter_num` | int | `3` | Livox 点过滤步长；HD 默认 `1` |
| `acc_cov` | double | `0.1` | 加速度计噪声协方差 |
| `gyr_cov` | double | `0.1` | 陀螺仪噪声协方差 |
| `filter_size_map` | double | `0.5` | ikd-Tree 地图体素叶尺寸；HD 默认 `0.05` 米 |
| `max_iter` | int | `3` | IEKF 最大迭代次数；对应 YAML 的 `mapping.max_iteration`，HD 默认 `4` |
| `max_feature_points` | int | `2000` | 每帧进入 IEKF 的特征点上限；HD 默认 `4000`，`<=0` 表示不限制 |
| `iekf_match_threads` | int | `4` | IEKF 最近邻匹配线程数；`0` 表示自动选择 |
| `root_dir` | string | `.` | `Log` 和 `PCD` 输出根目录；相对路径以当前工作目录为基准 |

`filter_size_surf`、外参、IMU 偏移等其他 YAML 字段当前不能通过命令行覆盖，需要修改或复制 YAML 配置文件。

### 6.2 地图输出

| 参数 | 类型 | Horizon 默认值 | 作用 |
|---|---|---:|---|
| `map_output.mode` | enum | `full_async` | `full_async`、`tiled_incremental` 或 `hybrid`；HD 默认 `hybrid` |
| `map_output.full_publish_interval_ms` | int | `1000` | `/map` 完整快照最小间隔；运行时至少 `100 ms`，HD 默认 `5000` |
| `map_output.full_voxel_leaf_m` | double | `0.2` | 完整 `/map` 的输出体素尺寸 |
| `map_output.tile_size_m` | double | `20.0` | Tile 空间边长，单位米 |
| `map_output.tile_voxel_leaf_m` | double | `0.2` | Tile 内部体素尺寸；`tile_size_m` 必须是它的整数倍 |
| `map_output.voxel_update_policy` | enum | `latest` | 同一体素的更新方式：`first`、`latest` 或 `centroid` |
| `map_output.tile_publish_hz` | int | `10` | Dirty Tile 提取和发布频率 |
| `map_output.max_tiles_per_update` | int | `32` | 单个批次最多处理的 Tile 数 |
| `map_output.max_points_per_update` | int | `200000` | 单个 Tile 输出任务的点数上限 |
| `map_output.input_queue_capacity` | int | `64` | 地图构建输入队列任务数上限 |
| `map_output.input_queue_max_mb` | int | `128` | 地图构建输入队列内存上限，单位 MiB |
| `map_output.max_memory_mb` | int | `1024` | 后台累计地图的估算内存上限，单位 MiB |
| `map_output.memory_policy` | enum | `stop_accumulating` | `stop_accumulating`、`lru`、`evict_far`；`spill_to_disk` 尚未实现，会回退到停止累计 |

地图模式的实际行为：

| 模式 | `/map` | `/map_tiles` | 建议用途 |
|---|---:|---:|---|
| `full_async` | 是 | 否 | 兼容传统完整 PointCloud 显示 |
| `tiled_incremental` | 否 | 是 | 长时间、高密度建图 |
| `hybrid` | 是 | 是 | 迁移、对照和兼容，资源开销最大 |

### 6.3 Foxglove 输出

| 参数 | 类型 | 默认值 | 作用 |
|---|---|---:|---|
| `foxglove.current_cloud_publish_hz` | double | `20` | `/cloud_registered` 最大发布频率 |
| `foxglove.path_publish_hz` | double | `2` | `/path` 最大发布频率 |
| `foxglove.backlog_size` | int | `64` | WebSocket 消息 backlog；运行时至少为 `8` |
| `foxglove_control_interval_ms` | int | `20` | 回放时钟广播间隔；运行时至少为 `10 ms` |
| `foxglove_backlog_size` | int | `64` | `foxglove.backlog_size` 的旧版覆盖名 |

Foxglove 地址固定为：

```text
ws://localhost:8765
```

当前版本没有启动参数用于修改监听地址或端口。

### 6.4 PCD 与 Bag 存储

| 参数 | 类型 | 默认值 | 作用 |
|---|---|---:|---|
| `pcd_save_en` | bool | `true` | 是否后台保存扫描 PCD 分块 |
| `pcd_interval` | int | `-1` | 旧版按帧分块参数；正数会映射到 `storage.pcd_chunk_frames`，`-1`/`0` 表示不按帧触发 |
| `bag_full_map_periodic` | bool | `false` | Bag 运行期间是否周期写入完整 `/map`；为假时只尝试写最终地图 |
| `storage.mode` | enum | `realtime` | `realtime` 过载时丢弃旧任务；`reliable` 在容量或写入失败时标记存储失败 |
| `storage.queue_max_mb` | int | `512` | Bag/PCD 后台队列内存上限，单位 MiB |
| `storage.bag_path_publish_hz` | double | `1` | 输出 Bag 中 `/path` 的最大写入频率 |
| `storage.path_max_points` | int | `100000` | Bag 路径消息保留的最大轨迹点数 |
| `storage.pcd_format` | enum | `binary_compressed` | `binary` 或 `binary_compressed` |
| `storage.pcd_chunk_points` | int | `1000000` | 每个扫描 PCD 分块的点数上限，运行时至少 `1000` |
| `storage.pcd_chunk_frames` | int | `0` | 每个扫描 PCD 分块的帧数上限；`0` 表示只按点数触发 |
| `storage.save_final_ikdtree` | bool | `true` | 退出时是否导出最终 ikd-Tree 地图 |

PCD 默认输出示例：

```text
PCD/scans_0.pcd
PCD/scans_1.pcd
PCD/scans_final_2.pcd
PCD/ikd_tree_map.pcd
```

运行指标和 IMU 调试文件写入：

```text
Log/runtime.txt
Log/imu.txt
```

## 7. 旧版兼容覆盖参数

以下参数仍被解析，但新配置文件已经包含 `map_output` 分组，建议不要再用于新命令：

| 旧参数 | 推荐替代参数 | 说明 |
|---|---|---|
| `publish_full_map` | `map_output.mode` | 单独覆盖它不会改变当前 `map_output.mode`，不建议使用 |
| `async_full_map_publish` | `map_output.mode` | 旧版模式映射开关，不代表能够把完整地图改回主线程同步发布 |
| `full_map_publish_interval_ms` | `map_output.full_publish_interval_ms` | 当前仍会同步更新新字段 |
| `full_map_voxel_size` | `map_output.full_voxel_leaf_m` | 当前仍会同步更新新字段 |
| `publish_map_delta` | `map_output.mode=tiled_incremental` 或 `hybrid` | 当前增量实现使用 `/map_tiles`，不是旧 `/map_delta` |
| `map_delta_max_pending_points` | `map_output.max_points_per_update` | 当前仍会同步更新新字段 |

尤其不要依赖 `async_full_map_publish` 与 `publish_map_delta` 的组合顺序来切换模式，直接设置一次 `map_output.mode` 更清晰且结果确定。

## 8. 配置合法性要求

程序启动时会拒绝以下无效配置：

- `map_output.mode` 不是 `full_async`、`tiled_incremental`、`hybrid` 之一；
- `map_output.voxel_update_policy` 不是 `first`、`latest`、`centroid` 之一；
- `map_output.memory_policy` 不在允许列表内；
- `storage.mode` 不是 `realtime` 或 `reliable`；
- `storage.pcd_format` 不是 `binary` 或 `binary_compressed`；
- Tile 尺寸或 Tile 体素尺寸不是有限正数；
- `map_output.tile_size_m` 不是 `map_output.tile_voxel_leaf_m` 的整数倍；
- `storage.pcd_chunk_frames` 小于 `0`。

## 9. 常见问题

### 启动后提示找不到配置文件

显式传入配置文件路径：

```powershell
.\build\Release\livox_fast_lio.exe .\config\horizon.yaml
```

### 参数写了但没有效果

依次检查：

1. 是否写成了准确的 `key=value`；
2. 是否误写成 `--key=value`；
3. 参数是否在本文档的命令行覆盖白名单中；
4. 是否混用了旧版地图开关和新的 `map_output.mode`；
5. 是否把 PowerShell 反引号后的空格也复制进了命令。

### Foxglove 无法连接

确认程序日志显示 WebSocket 已启动，然后连接：

```text
ws://127.0.0.1:8765
```

如果端口被占用，需要关闭占用 `8765` 的程序后重新启动；当前版本不能通过 CLI 修改端口。

### 如何停止程序

在终端按 `Ctrl+C`。程序会停止输入源、处理后台队列，并根据配置完成 Bag/PCD 和最终地图收尾。大地图导出可能需要一定时间，不建议直接结束进程。
