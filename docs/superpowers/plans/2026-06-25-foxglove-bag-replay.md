# Foxglove Bag Replay Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 让 `--bag test2.bag` 回放 FAST-LIO 建图时，Foxglove Studio 可以通过 `ws://localhost:8765` 实时查看 `/cloud_registered`、`/odometry`、`/path` 和 `/tf`。

**Architecture:** 保留现有 FAST-LIO 主循环，把 Livox 输入语义标准化，修正反射率/标签/线号字段，再用 Foxglove 官方 C++ SDK 替换手写 WebSocket 协议实现。`laser_mapping.cpp` 同时驱动 Foxglove live output 和可选 bag writer，避免 bag 模式二选一。

**Tech Stack:** C++17、CMake、PCL、Eigen、yaml-cpp、Livox ROS driver message semantics、Foxglove C++ SDK `sdk/v0.25.2`、Windows/MSVC。

---

## 文件结构

- Modify: `CMakeLists.txt`
  - 接入 Foxglove SDK `third_party/foxglove-sdk/v0.25.2/foxglove`
  - 添加轻量测试可执行文件 `livox_fast_lio_tests`
  - 将 `foxglove.dll` 或 wrapper 运行时文件复制到 `build/Release`
- Modify: `include/lvx_reader.h`
  - 给 `LvxPoint` 增加 `reflectivity`
- Modify: `src/lvx_reader.cpp`
  - LVX 解析时保存官方 Livox reflectivity 字段
- Modify: `src/bag_reader.cpp`
  - ROS1 bag `livox_ros_driver/CustomMsg` 解析时保存 `CustomPoint.reflectivity`
- Create: `include/livox_point_utils.h`
  - 提供 `LvxPoint -> PointType` 的唯一转换函数
  - `reflectivity -> intensity`
  - `tag/line` 保留到 `normal_x/normal_y`
- Modify: `src/laser_mapping.cpp`
  - 使用 `livox_point_utils.h`
  - bag 模式也启动 Foxglove
  - bag writer 和 Foxglove publisher 同时输出
  - 默认 frame 使用 `map` 和 `base_link`
- Modify: `include/foxglove_publisher.h`
  - 保留对外接口，增加 `broadcastTime`
- Replace: `src/foxglove_publisher.cpp`
  - 删除手写 socket 协议实现
  - 使用 `foxglove::WebSocketServer`
  - 使用 `foxglove::messages::*Channel`
- Create: `tests/test_livox_fast_lio.cpp`
  - 测试 reflectivity/tag/line 语义和 frame 常量

## Task 1: 接入测试框架并写第一个失败测试

**Files:**
- Modify: `CMakeLists.txt`
- Create: `tests/test_livox_fast_lio.cpp`

- [ ] **Step 1: 写失败测试**

`tests/test_livox_fast_lio.cpp`:

```cpp
#include <cassert>
#include <string>
#include "foxglove_publisher.h"
#include "livox_point_utils.h"
#include "lvx_reader.h"

int main() {
    LvxPoint src{};
    src.x = 1.0f;
    src.y = 2.0f;
    src.z = 3.0f;
    src.reflectivity = 42;
    src.tag = 9;
    src.line = 3;
    src.offset_time = 2500000;

    PointType pt = livoxToPointType(src);

    assert(pt.x == 1.0f);
    assert(pt.y == 2.0f);
    assert(pt.z == 3.0f);
    assert(pt.intensity == 42.0f);
    assert(pt.normal_x == 9.0f);
    assert(pt.normal_y == 3.0f);
    assert(pt.curvature == 2.5f);

    assert(std::string(kFastLioMapFrame) == "map");
    assert(std::string(kFastLioBodyFrame) == "base_link");
    return 0;
}
```

- [ ] **Step 2: 修改 CMake 注册测试可执行文件**

在 `CMakeLists.txt` 末尾 `message(...)` 前加入：

```cmake
enable_testing()

add_executable(livox_fast_lio_tests
    tests/test_livox_fast_lio.cpp
)
target_include_directories(livox_fast_lio_tests PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${PCL_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIR}
)
target_link_libraries(livox_fast_lio_tests PRIVATE
    ${PCL_LIBRARIES}
)
add_test(NAME livox_fast_lio_tests COMMAND livox_fast_lio_tests)
```

- [ ] **Step 3: 运行测试确认失败**

Run:

```powershell
cmake --build build --config Release --target livox_fast_lio_tests
```

Expected: 编译失败，原因包含 `livox_point_utils.h` 不存在或 `LvxPoint` 没有 `reflectivity`。

## Task 2: 修正 Livox 点字段语义

**Files:**
- Modify: `include/lvx_reader.h`
- Modify: `src/lvx_reader.cpp`
- Modify: `src/bag_reader.cpp`
- Create: `include/livox_point_utils.h`
- Modify: `src/laser_mapping.cpp`

- [ ] **Step 1: 修改 `LvxPoint`**

在 `include/lvx_reader.h` 的 `struct LvxPoint` 中改成：

```cpp
struct LvxPoint {
    float x, y, z;             // meters
    uint8_t reflectivity = 0;  // Livox reflectivity, maps to intensity
    uint8_t line = 0;
    uint8_t tag = 0;
    uint32_t offset_time = 0;  // ns from frame start
};
```

- [ ] **Step 2: 新增唯一转换工具**

Create `include/livox_point_utils.h`:

```cpp
#ifndef LIVOX_POINT_UTILS_H
#define LIVOX_POINT_UTILS_H

#include "lvx_reader.h"
#include "types.h"

inline PointType livoxToPointType(const LvxPoint& lp) {
    PointType pt;
    pt.x = lp.x;
    pt.y = lp.y;
    pt.z = lp.z;
    pt.intensity = static_cast<float>(lp.reflectivity);
    pt.curvature = static_cast<float>(lp.offset_time) / 1e6f; // ns -> ms
    pt.normal_x = static_cast<float>(lp.tag);
    pt.normal_y = static_cast<float>(lp.line);
    pt.normal_z = 0.0f;
    return pt;
}

#endif // LIVOX_POINT_UTILS_H
```

- [ ] **Step 3: 修正 bag reader 字段映射**

在 `src/bag_reader.cpp` 的 CustomPoint 转 `LvxPoint` 处使用：

```cpp
lp.x = pt.x;
lp.y = pt.y;
lp.z = pt.z;
lp.reflectivity = pt.reflectivity;
lp.tag = pt.tag;
lp.line = pt.line;
lp.offset_time = pt.offset_time;
```

- [ ] **Step 4: 修正 LVX reader 字段映射**

在 `src/lvx_reader.cpp` 的 extend cartesian 解析处设置：

```cpp
p.reflectivity = pt[12];
p.tag = pt[13];
```

在普通 cartesian 解析处设置：

```cpp
p.reflectivity = pt[12];
p.tag = 0;
```

- [ ] **Step 5: 修改 laser mapping 统一使用转换函数**

在 `src/laser_mapping.cpp` include 区加入：

```cpp
#include "livox_point_utils.h"
```

将 LVX callback 和 bag callback 中手写构造 `PointType` 的循环改成：

```cpp
for (auto& lp : points)
{
    cloud->push_back(livoxToPointType(lp));
}
```

- [ ] **Step 6: 运行测试确认通过**

Run:

```powershell
cmake --build build --config Release --target livox_fast_lio_tests
ctest --test-dir build -C Release --output-on-failure
```

Expected: `livox_fast_lio_tests` 通过。

## Task 3: 接入 Foxglove 官方 SDK

**Files:**
- Modify: `CMakeLists.txt`

- [ ] **Step 1: 增加 SDK 路径与 wrapper library**

在 `CMakeLists.txt` 的依赖区加入：

```cmake
set(FOXGLOVE_SDK_DIR "${CMAKE_CURRENT_SOURCE_DIR}/third_party/foxglove-sdk/v0.25.2/foxglove")
find_package(foxglove-sdk CONFIG REQUIRED
    PATHS "${FOXGLOVE_SDK_DIR}"
    NO_DEFAULT_PATH)

foxglove_sdk_add_cpp_library(foxglove_cpp
    TYPE STATIC
    REMOTE_ACCESS OFF
)
```

- [ ] **Step 2: 链接主程序**

在 `target_link_libraries(livox_fast_lio ...)` 中加入：

```cmake
    foxglove_cpp
```

- [ ] **Step 3: 链接测试程序**

在 `target_link_libraries(livox_fast_lio_tests ...)` 中加入：

```cmake
    foxglove_cpp
```

- [ ] **Step 4: 配置并构建确认 SDK 可用**

Run:

```powershell
cmake -B build -S . -DCMAKE_TOOLCHAIN_FILE=third_party/vcpkg/scripts/buildsystems/vcpkg.cmake
cmake --build build --config Release --target livox_fast_lio_tests
```

Expected: CMake 找到 `foxglove-sdkConfig.cmake`，测试目标能链接。

## Task 4: 用官方 SDK 替换 FoxglovePublisher 实现

**Files:**
- Modify: `include/foxglove_publisher.h`
- Replace: `src/foxglove_publisher.cpp`

- [ ] **Step 1: 更新 header 接口和 frame 常量**

`include/foxglove_publisher.h` 保留现有 public API，并加入：

```cpp
constexpr const char* kFastLioMapFrame = "map";
constexpr const char* kFastLioBodyFrame = "base_link";
```

新增方法：

```cpp
void broadcastTime(double timestamp);
```

- [ ] **Step 2: 实现 SDK-backed publisher**

`src/foxglove_publisher.cpp` 使用这些 SDK 类型：

```cpp
#include "foxglove_publisher.h"
#include "ros_message.h"

#include <foxglove/websocket.hpp>
#include <foxglove/messages.hpp>

#include <array>
#include <cstring>
#include <iostream>
#include <memory>
#include <optional>

namespace fg = foxglove;
namespace fgm = foxglove::messages;
```

实现私有状态：

```cpp
struct FoxglovePublisherImpl {
    std::optional<fg::WebSocketServer> server;
    std::optional<fgm::PointCloudChannel> cloud;
    std::optional<fgm::OdometryChannel> odom;
    std::optional<fgm::PosesInFrameChannel> path;
    std::optional<fgm::FrameTransformsChannel> tf;
};
```

创建 timestamp helper：

```cpp
static fgm::Timestamp toFoxgloveTimestamp(double t) {
    uint32_t sec = 0;
    uint32_t nsec = 0;
    doubleToRosTime(t, sec, nsec);
    return fgm::Timestamp{sec, nsec};
}
```

创建 pose helper：

```cpp
static fgm::Pose makePose(const V3D& p, const Eigen::Quaterniond& q) {
    fgm::Pose pose;
    pose.position = fgm::Vector3{p.x(), p.y(), p.z()};
    pose.orientation = fgm::Quaternion{q.x(), q.y(), q.z(), q.w()};
    return pose;
}
```

- [ ] **Step 3: `start` 创建官方 server 和 channels**

`start` 的核心逻辑：

```cpp
auto impl = std::make_unique<FoxglovePublisherImpl>();

fg::WebSocketServerOptions options;
options.name = "livox-fast-lio";
options.host = host;
options.port = port;
options.capabilities = fg::WebSocketServerCapabilities::Time;
options.supported_encodings = {"protobuf"};

auto server_result = fg::WebSocketServer::create(std::move(options));
if (!server_result) {
    std::cerr << "[Foxglove] Failed to start WebSocket server: "
              << static_cast<int>(server_result.error()) << std::endl;
    return false;
}
impl->server = std::move(server_result.value());

auto cloud_result = fgm::PointCloudChannel::create("/cloud_registered");
auto odom_result = fgm::OdometryChannel::create("/odometry");
auto path_result = fgm::PosesInFrameChannel::create("/path");
auto tf_result = fgm::FrameTransformsChannel::create("/tf");
if (!cloud_result || !odom_result || !path_result || !tf_result) {
    std::cerr << "[Foxglove] Failed to create one or more channels" << std::endl;
    return false;
}
impl->cloud = std::move(cloud_result.value());
impl->odom = std::move(odom_result.value());
impl->path = std::move(path_result.value());
impl->tf = std::move(tf_result.value());
```

Then store `server_impl_ = impl.release(); running_ = true;`.

- [ ] **Step 4: `publishPointCloud` 使用 packed fields**

每点 stride:

```cpp
struct PackedPoint {
    float x;
    float y;
    float z;
    float intensity;
    uint8_t tag;
    uint8_t line;
    uint8_t pad0;
    uint8_t pad1;
};
```

fields:

```cpp
msg.fields = {
    {"x", 0, fgm::PackedElementField::NumericType::FLOAT32},
    {"y", 4, fgm::PackedElementField::NumericType::FLOAT32},
    {"z", 8, fgm::PackedElementField::NumericType::FLOAT32},
    {"intensity", 12, fgm::PackedElementField::NumericType::FLOAT32},
    {"tag", 16, fgm::PackedElementField::NumericType::UINT8},
    {"line", 17, fgm::PackedElementField::NumericType::UINT8},
};
```

点数据中 `tag` 从 `normal_x` 取，`line` 从 `normal_y` 取。

- [ ] **Step 5: 实现 odometry/path/tf/time**

Odometry:

```cpp
fgm::Odometry msg;
msg.timestamp = toFoxgloveTimestamp(timestamp);
msg.frame_id = kFastLioMapFrame;
msg.body_frame_id = kFastLioBodyFrame;
msg.pose = makePose(position, orientation);
impl->odom->log(msg, doubleToNs(timestamp));
```

Path:

```cpp
fgm::PosesInFrame msg;
msg.timestamp = toFoxgloveTimestamp(timestamp);
msg.frame_id = kFastLioMapFrame;
for (const auto& p : path) {
    msg.poses.push_back(makePose(p, Eigen::Quaterniond::Identity()));
}
impl->path->log(msg, doubleToNs(timestamp));
```

TF:

```cpp
fgm::FrameTransform transform;
transform.timestamp = toFoxgloveTimestamp(timestamp);
transform.parent_frame_id = parent_frame;
transform.child_frame_id = child_frame;
transform.translation = fgm::Vector3{translation.x(), translation.y(), translation.z()};
transform.rotation = fgm::Quaternion{rotation.x(), rotation.y(), rotation.z(), rotation.w()};
fgm::FrameTransforms msg;
msg.transforms.push_back(transform);
impl->tf->log(msg, doubleToNs(timestamp));
```

Time:

```cpp
void FoxglovePublisher::broadcastTime(double timestamp) {
    auto* impl = static_cast<FoxglovePublisherImpl*>(server_impl_);
    if (!impl || !impl->server || !running_) return;
    impl->server->broadcastTime(doubleToNs(timestamp));
}
```

- [ ] **Step 6: 构建确认 API 使用正确**

Run:

```powershell
cmake --build build --config Release --target livox_fast_lio_tests
cmake --build build --config Release --target livox_fast_lio
```

Expected: 两个 target 都构建成功。

## Task 5: 让 bag 模式实时发布到 Foxglove

**Files:**
- Modify: `src/laser_mapping.cpp`

- [ ] **Step 1: Foxglove 不再排除 bag 模式**

将 `if (!use_bag)` 包裹的启动逻辑改成始终尝试启动：

```cpp
FoxglovePublisher foxglove;
const bool foxglove_ok = foxglove.start("127.0.0.1", 8765);
if (!foxglove_ok) {
    cerr << "[LaserMapping] Failed to start Foxglove publisher." << endl;
} else {
    cout << "[LaserMapping] Foxglove WebSocket: ws://localhost:"
         << foxglove.getPort() << endl;
}
```

- [ ] **Step 2: bag writer 只在打开成功时使用**

发布时判断：

```cpp
const bool write_bag_output = (bag_writer && bag_writer->isOpen());
```

不要用 `if (bag_writer) ... else foxglove...`，改为：

```cpp
if (write_bag_output) {
    bag_writer->writeOdometry(time_ns, pos_v3d, geoQuat, kFastLioMapFrame);
    bag_writer->writeTF(time_ns, pos_v3d, geoQuat, kFastLioMapFrame, kFastLioBodyFrame);
}
if (foxglove.isRunning()) {
    foxglove.publishOdometry(pos_v3d, geoQuat, pub_time);
    foxglove.publishTransform(pos_v3d, geoQuat, kFastLioMapFrame, kFastLioBodyFrame, pub_time);
    foxglove.broadcastTime(pub_time);
}
```

Point cloud and path 同样改成 bag writer 和 Foxglove 各自独立输出。

- [ ] **Step 3: 停止逻辑按运行状态处理**

函数末尾改成：

```cpp
if (bag_writer) bag_writer->close();
foxglove.stop();
```

- [ ] **Step 4: 构建**

Run:

```powershell
cmake --build build --config Release --target livox_fast_lio
```

Expected: 构建成功。

## Task 6: 验证和文档微调

**Files:**
- Modify: `README.md`

- [ ] **Step 1: 更新 README 的 Foxglove bag replay 说明**

把 bag 回放说明改成：

```markdown
Bag replay starts the Foxglove WebSocket server by default.

1. Run:
   `livox_fast_lio.exe config\horizon.yaml --bag "D:\path\to\test2.bag"`
2. Open Foxglove Studio.
3. Connect to `ws://localhost:8765`.
4. Add 3D panel topics `/cloud_registered`, `/odometry`, `/path`, `/tf`.
```

- [ ] **Step 2: 运行自动验证**

Run:

```powershell
ctest --test-dir build -C Release --output-on-failure
cmake --build build --config Release --target livox_fast_lio
```

Expected: 测试通过，主程序构建成功。

- [ ] **Step 3: 用用户提供的 bag 做短时验证**

Run:

```powershell
& 'D:\Projects\livox-fast-lio-windows\build\Release\livox_fast_lio.exe' `
  'D:\Projects\livox-fast-lio-windows\config\horizon.yaml' `
  --bag 'D:\Livox Horizon\Livox Viewer 0.11.0\data\record files\test2.bag' `
  pcd_save_en=false
```

Expected within startup logs:

```text
[BagReader] Found LiDAR topic: /livox/lidar
[BagReader] Found IMU topic: /livox/imu
[LaserMapping] Foxglove WebSocket: ws://localhost:8765
[SLAM] Frame
```

- [ ] **Step 4: 仓库状态说明**

Run:

```powershell
git rev-parse --is-inside-work-tree
```

Expected: 当前目录不是 git repo，因此不能提交。最终汇报列出已改文件和验证结果。
