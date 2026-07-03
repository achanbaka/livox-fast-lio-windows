# Livox SDK1 LVX Playback Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Implement first-phase Livox SDK 1 / LVX v1.1 playback parsing so `.lvx` files normalize SDK1 point and IMU packets into the existing FAST-LIO/Foxglove chain.

**Architecture:** Keep `laser_mapping.cpp`, bag replay, and Foxglove publishing behavior unchanged. Extend `LvxReader` so LVX frame data is parsed into `LvxPoint` and `ImuData` for SDK1 `data_type=0..8`, with reader-level diagnostics for missing points or IMU.

**Tech Stack:** C++17, CMake, existing single executable test target `livox_fast_lio_tests`, PCL/Eigen/yaml-cpp already configured by the project.

---

### Task 1: Parser Test Surface

**Files:**
- Modify: `include/lvx_reader.h`
- Modify: `tests/test_livox_fast_lio.cpp`
- Modify: `CMakeLists.txt`

- [x] **Step 1: Write failing tests for direct LVX packet parsing**

Add synthetic LVX packet builders to `tests/test_livox_fast_lio.cpp` and assertions that call a public test-friendly parser API:

```cpp
LvxFrameParseResult parsed = reader.parsePacketsForTest(frame_bytes);
if (expect(parsed.points.size() == 100)) return 1;
if (expect(parsed.has_points)) return 1;
if (expect(parsed.unsupported_packet_count == 0)) return 1;
```

- [x] **Step 2: Run test to verify it fails**

Run: `cmake --build build --config Release --target livox_fast_lio_tests`

Expected: compile failure because `LvxFrameParseResult` and `parsePacketsForTest` do not exist yet.

- [x] **Step 3: Add minimal parser test surface**

Declare in `include/lvx_reader.h`:

```cpp
struct LvxFrameParseResult {
    std::vector<LvxPoint> points;
    std::vector<ImuData> imus;
    double frame_time = 0.0;
    bool has_points = false;
    bool has_imu = false;
    uint32_t unsupported_packet_count = 0;
    uint32_t truncated_packet_count = 0;
};

LvxFrameParseResult parsePacketsForTest(const std::vector<uint8_t>& data);
```

- [x] **Step 4: Link the test target with parser implementation**

Add `src/lvx_reader.cpp` to the `livox_fast_lio_tests` source list in `CMakeLists.txt`.

- [x] **Step 5: Run test to verify the parser test surface compiles**

Run: `cmake --build build --config Release --target livox_fast_lio_tests`

Expected: build succeeds only after a stub parser result is available; behavior tests still fail until Task 2.

### Task 2: SDK1 LVX Packet Normalization

**Files:**
- Modify: `src/lvx_reader.cpp`
- Modify: `include/lvx_reader.h`
- Modify: `tests/test_livox_fast_lio.cpp`

- [x] **Step 1: Write failing normalization tests for cartesian packets**

Assert `data_type=0`, `2`, `4`, and `7` convert millimeters to meters, preserve reflectivity and tag where present, set `line=0`, and split dual/triple returns into separate `LvxPoint` entries.

- [x] **Step 2: Run focused test to verify failure**

Run: `build\Release\livox_fast_lio_tests.exe`

Expected: non-zero exit because only existing type 0 and type 2 behavior is partially implemented.

- [x] **Step 3: Implement cartesian normalization**

Use one shared helper in `src/lvx_reader.cpp` for reading little-endian integers and appending `LvxPoint` values. Handle packet sizes:

```text
0: 100 * 13 bytes
2: 96 * 14 bytes
4: 48 * 28 bytes, two returns per raw point
7: 30 * 42 bytes, three returns per raw point
```

- [x] **Step 4: Write failing normalization tests for spherical packets**

Assert `data_type=1`, `3`, `5`, and `8` convert `depth mm`, `theta centidegree`, and `phi centidegree` into meter cartesian coordinates and split dual/triple returns.

- [x] **Step 5: Implement spherical normalization**

Use the SDK1 spherical conversion:

```cpp
x = depth_m * std::sin(theta_rad) * std::cos(phi_rad);
y = depth_m * std::sin(theta_rad) * std::sin(phi_rad);
z = depth_m * std::cos(theta_rad);
```

- [x] **Step 6: Write and pass IMU/time tests**

Assert `data_type=6` creates one `ImuData` sample, uses the same relative seconds base as LiDAR frame time, and preserves gyro/acc float order.

- [x] **Step 7: Verify all focused parser tests pass**

Run: `build\Release\livox_fast_lio_tests.exe`

Expected: exit code 0.

### Task 3: Reader Diagnostics

**Files:**
- Modify: `include/lvx_reader.h`
- Modify: `src/lvx_reader.cpp`
- Modify: `tests/test_livox_fast_lio.cpp`

- [x] **Step 1: Write failing diagnostics tests**

Assert `parsePacketsForTest` reports `has_points`, `has_imu`, `unsupported_packet_count`, and `truncated_packet_count` correctly for frames containing known point packets, IMU packets, unknown packet types, and truncated payloads.

- [x] **Step 2: Implement diagnostics counters**

Accumulate reader-level `seen_points_`, `seen_imu_`, `unsupported_packet_count_`, and `truncated_packet_count_`. Log clear warnings during playback when frames have no supported point packets or no IMU packets yet.

- [x] **Step 3: Preserve existing playback callback behavior**

Keep callback shape unchanged:

```cpp
frame_cb_(points, imus, frame_time);
```

Only invoke it when a frame has points or IMU, as before.

### Task 4: Verification

**Files:**
- Modify: none unless verification exposes defects.

- [x] **Step 1: Run the focused executable test**

Run: `build\Release\livox_fast_lio_tests.exe`

Expected: exit code 0.

- [x] **Step 2: Run CTest**

Run: `ctest --test-dir build -C Release --output-on-failure`

Expected: `100% tests passed`.

- [x] **Step 3: Build the main executable**

Run: `cmake --build build --config Release --target livox_fast_lio`

Expected: build exit code 0.

---

**Execution note:** The current workspace contains an empty `.git` directory, and `git status` reports “not a git repository.” This plan will be executed without commit steps unless Git metadata is restored.
