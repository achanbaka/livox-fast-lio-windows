# Foxglove Bag Replay Design

Date: 2026-06-25
Status: Approved design, pending implementation plan

## Goal

Enable live FAST-LIO mapping visualization in Foxglove while replaying a Livox ROS1 bag:

```powershell
livox_fast_lio.exe config\horizon.yaml --bag "D:\Livox Horizon\Livox Viewer 0.11.0\data\record files\test2.bag"
```

Foxglove Studio connects to `ws://localhost:8765` and displays the mapping process in real time.

## Current Findings

The supplied `test2.bag` is readable by the existing parser. It contains:

- `/livox/lidar` with type `livox_ros_driver/CustomMsg`
- `/livox/imu` with type `sensor_msgs/Imu`

The SLAM pipeline starts and processes frames. The blocking design flaw is that bag mode currently disables the Foxglove WebSocket publisher, so `--bag` playback has no live Foxglove endpoint. The existing hand-written WebSocket publisher also uses non-standard text JSON publish frames, so it should not be extended.

## Standards To Follow

Implementation must prefer official definitions over project-local interpretations.

- Livox input follows Livox ROS driver and Livox SDK semantics:
  - `livox_ros_driver/CustomMsg`
  - `livox_ros_driver/CustomPoint`
  - Livox SDK point unit and field definitions
- Foxglove output uses the official Foxglove C++ SDK:
  - `foxglove::WebSocketServer`
  - Foxglove built-in schemas where available
  - Foxglove SDK time broadcast for replay clock
- ROS topic/message semantics follow official ROS1/ROS2 message definitions and REP coordinate conventions:
  - `sensor_msgs/PointCloud2`
  - `sensor_msgs/Imu`
  - `nav_msgs/Odometry`
  - `nav_msgs/Path`
  - `tf2_msgs/TFMessage`
  - REP 103 and REP 105 frame naming and coordinate conventions

Reference sources:

- Livox CustomMsg: https://raw.githubusercontent.com/Livox-SDK/livox_ros_driver/master/livox_ros_driver/msg/CustomMsg.msg
- Livox CustomPoint: https://raw.githubusercontent.com/Livox-SDK/livox_ros_driver/master/livox_ros_driver/msg/CustomPoint.msg
- Livox SDK definitions: https://raw.githubusercontent.com/Livox-SDK/Livox-SDK/master/sdk_core/include/livox_def.h
- Foxglove C++ SDK: https://docs.foxglove.dev/docs/getting-started/cpp
- Foxglove WebSocket server: https://docs.foxglove.dev/docs/sdk/websocket-server
- Foxglove PointCloud schema: https://docs.foxglove.dev/docs/sdk/schemas/point-cloud
- ROS REP 103: https://raw.githubusercontent.com/ros-infrastructure/rep/master/rep-0103.rst
- ROS REP 105: https://raw.githubusercontent.com/ros-infrastructure/rep/master/rep-0105.rst

## Architecture

Keep the existing SLAM pipeline shape, but separate three concerns that are currently mixed:

1. Input decoding
2. Internal normalized sensor/mapping data
3. Output publishing and bag writing

The flow becomes:

```text
ROS1 bag reader
  -> Livox official message decoder
  -> normalized point/IMU frames
  -> FAST-LIO processing
  -> standard output messages
  -> Foxglove SDK WebSocket server
  -> optional ROS1 output bag writer
```

The existing `FoxglovePublisher` public interface can remain as the adapter used by `laser_mapping.cpp`, but its implementation should be backed by the official Foxglove SDK instead of a custom socket protocol.

## Livox Input Model

Create or formalize a normalized point representation with official Livox semantics:

- `x`, `y`, `z` in meters
- `intensity` from Livox `reflectivity`
- `tag` preserved as Livox tag metadata
- `line` preserved as Livox scan line metadata
- per-point offset time preserved from `offset_time`
- frame timestamp derived from the official Livox message header/timebase semantics

Current code maps `tag` to PCL intensity in bag replay. That is incorrect for official Livox semantics and must be changed to `reflectivity`.

For ROS bag `livox_ros_driver/CustomMsg`, parse `CustomPoint` using the official field order. For real Livox SDK data, normalize SDK point units to meters before handing data to the shared model.

## Foxglove Live Output

Use the official Foxglove C++ SDK to start a WebSocket server on port `8765`.

Behavior:

- Start the Foxglove server in bag mode and live LiDAR mode.
- Keep the server alive for the duration of playback.
- Publish data from the SLAM output stage, not directly from raw bag input.
- Broadcast replay time using the bag/SLAM timestamp so Foxglove's time controls track playback.

Preferred channels:

- `/cloud_registered`: Foxglove `PointCloud`, equivalent to ROS `sensor_msgs/PointCloud2` semantics
- `/odometry`: Foxglove `Odometry`, equivalent to ROS `nav_msgs/Odometry`
- `/path`: Foxglove `PosesInFrame`, equivalent to ROS `nav_msgs/Path`
- `/tf`: Foxglove `FrameTransforms`, equivalent to ROS `tf2_msgs/TFMessage`

Point cloud data should be packed binary point data, not an array of JSON arrays. Fields must include `x`, `y`, `z`, `intensity`, `tag`, and `line`. If the selected SDK version does not expose a typed helper for one preferred schema, use the official SDK `RawChannel` with the corresponding Foxglove schema name instead of changing the topic semantics.

## ROS Semantics And Frames

Use ROS frame names by default:

- `map` for the global mapping frame
- `odom` if a local odometry frame is needed
- `base_link` for the body frame
- sensor-specific frames can be added later only when the extrinsic chain is represented explicitly

Existing `world` and `body` names can be retained as compatibility aliases only if needed, but new Foxglove and ROS-style output should default to REP 103/105 names.

Use ROS message semantics for optional output bag writing:

- Replace or supplement the current `/path` `geometry_msgs/PoseArray` output with `nav_msgs/Path`.
- Prefer `nav_msgs/Odometry` for `/odometry` if the output writer is updated.
- Keep `/cloud_registered` compatible with `sensor_msgs/PointCloud2`.
- Keep `/tf` compatible with `tf2_msgs/TFMessage`.

## Build And Dependency Plan

Add the official Foxglove C++ SDK as a pinned dependency. Prefer a reproducible local third-party dependency under `third_party/foxglove-sdk` or a pinned CMake package location. The selected version must be recorded in build documentation.

Because the workspace currently has restricted network access, downloading the SDK release asset will require explicit approval when implementation begins.

The existing hand-written WebSocket server code should either be removed after replacement or kept behind a compile-time fallback only if the official SDK cannot be found. The default build must use the official SDK.

## Error Handling

Startup should fail loudly if live Foxglove output is requested but the SDK/server cannot start. For `--bag` mode, the program should print:

- selected bag path
- detected Livox/IMU topics and message types
- Foxglove WebSocket URL
- output channels
- whether optional output bag writing is enabled

If port 8765 is unavailable, the program should report the port conflict clearly.

## Testing

Tests should cover:

- Livox `CustomMsg` decoding maps `reflectivity` to intensity and preserves `tag`/`line`.
- Bag mode starts the Foxglove publisher path instead of disabling it.
- Frame names default to `map` and `base_link`.
- The Foxglove adapter creates expected channels for cloud, odometry, path, and transforms.
- Optional ROS bag writer output stays compatible with ROS message definitions when touched.

Manual verification uses the supplied `test2.bag`:

1. Start `livox_fast_lio.exe ... --bag test2.bag`.
2. Connect Foxglove Studio to `ws://localhost:8765`.
3. Confirm live updates on `/cloud_registered`, `/odometry`, `/path`, and `/tf`.
4. Confirm timestamps advance according to replay data.
5. Confirm logs show Livox topics, Foxglove server startup, and SLAM frame processing.

## Out Of Scope

- Replacing the FAST-LIO algorithm.
- Implementing ROS middleware runtime on Windows.
- Supporting arbitrary non-Livox point cloud bags in this pass.
- Building a custom Foxglove protocol implementation.
