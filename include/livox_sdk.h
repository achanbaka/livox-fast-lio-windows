/**
 * Minimal Livox SDK header for Windows FAST-LIO port.
 * Contains only the data types and function declarations used by our code.
 * 
 * When the full Livox-SDK is available, replace this with the real SDK headers.
 * The actual SDK binary (livox_sdk_static.lib) provides the implementations.
 */
#ifndef LIVOX_SDK_H
#define LIVOX_SDK_H

#include <cstdint>
#include <cstddef>

#ifdef __cplusplus
extern "C" {
#endif

// ─── Data type enums ───────────────────────────────────────────────
typedef enum {
    kCartesian              = 0,
    kSpherical              = 1,
    kExtendCartesian        = 2,
    kExtendSpherical        = 3,
    kDualExtendCartesian    = 4,
    kDualExtendSpherical    = 5,
    kImu                    = 6,
    kTripleExtendCartesian  = 7,
    kTripleExtendSpherical  = 8,
} LivoxDataType;

// ─── Device state ──────────────────────────────────────────────────
typedef enum {
    kConnectStateOff    = 0,
    kConnectStateConfig = 1,
    kConnectStateSampling = 2,
} LivoxConnectState;

// ─── Device type ───────────────────────────────────────────────────
typedef enum {
    kDeviceTypeHub      = 0,
    kDeviceTypeLidarMid40  = 1,
    kDeviceTypeLidarTele   = 2,
    kDeviceTypeLidarHorizon = 3,
    kDeviceTypeLidarMid70  = 6,
    kDeviceTypeLidarAvia   = 7,
} LivoxDeviceType;

// ─── Point structures ──────────────────────────────────────────────
#pragma pack(push, 1)

typedef struct {
    int32_t x;            // x axis, unit: mm
    int32_t y;            // y axis, unit: mm
    int32_t z;            // z axis, unit: mm
    uint8_t reflectivity; // reflectivity
} LivoxRawPoint;

typedef struct {
    uint32_t depth;       // depth, unit: mm
    uint16_t theta;       // zenith angle, unit: 0.01 degree
    uint16_t phi;         // azimuth, unit: 0.01 degree
    uint8_t reflectivity; // reflectivity
} LivoxSpherPoint;

typedef struct {
    int32_t x;            // x axis, unit: mm
    int32_t y;            // y axis, unit: mm
    int32_t z;            // z axis, unit: mm
    uint8_t reflectivity; // reflectivity
    uint8_t tag;          // tag
} LivoxExtendRawPoint;

typedef struct {
    uint32_t depth;       // depth, unit: mm
    uint16_t theta;       // zenith angle, unit: 0.01 degree
    uint16_t phi;         // azimuth, unit: 0.01 degree
    uint8_t reflectivity; // reflectivity
    uint8_t tag;          // tag
} LivoxExtendSpherPoint;

typedef struct {
    int32_t x1;
    int32_t y1;
    int32_t z1;
    uint8_t reflectivity1;
    uint8_t tag1;
    int32_t x2;
    int32_t y2;
    int32_t z2;
    uint8_t reflectivity2;
    uint8_t tag2;
} LivoxDualExtendRawPoint;

typedef struct {
    uint16_t theta;
    uint16_t phi;
    uint32_t depth1;
    uint8_t reflectivity1;
    uint8_t tag1;
    uint32_t depth2;
    uint8_t reflectivity2;
    uint8_t tag2;
} LivoxDualExtendSpherPoint;

typedef struct {
    int32_t x1;
    int32_t y1;
    int32_t z1;
    uint8_t reflectivity1;
    uint8_t tag1;
    int32_t x2;
    int32_t y2;
    int32_t z2;
    uint8_t reflectivity2;
    uint8_t tag2;
    int32_t x3;
    int32_t y3;
    int32_t z3;
    uint8_t reflectivity3;
    uint8_t tag3;
} LivoxTripleExtendRawPoint;

typedef struct {
    uint16_t theta;
    uint16_t phi;
    uint32_t depth1;
    uint8_t reflectivity1;
    uint8_t tag1;
    uint32_t depth2;
    uint8_t reflectivity2;
    uint8_t tag2;
    uint32_t depth3;
    uint8_t reflectivity3;
    uint8_t tag3;
} LivoxTripleExtendSpherPoint;

typedef struct {
    float x;              // x axis, unit: m
    float y;              // y axis, unit: m
    float z;              // z axis, unit: m
    uint8_t reflectivity; // reflectivity
} LivoxCartesianPoint;

typedef struct {
    float x;              // x axis, unit: m
    float y;              // y axis, unit: m
    float z;              // z axis, unit: m
    uint8_t reflectivity; // reflectivity
    uint8_t tag;          // tag
} LivoxExtendCartesianPoint;

typedef struct {
    union {
        struct {
            float gyro_x;
            float gyro_y;
            float gyro_z;
        };
        float gyro[3];    // gyroscope, unit: rad/s
    };
    union {
        struct {
            float acc_x;
            float acc_y;
            float acc_z;
        };
        float acc[3];     // accelerometer, unit: g in the Livox SDK definition
    };
} LivoxImuPacket;

typedef struct {
    float gyro[3];        // gyroscope, unit: rad/s
    float acc[3];         // accelerometer, unit: m/s^2
    uint8_t tag;          // tag
} LivoxExtendImuPacket;

// ─── Ethernet packet ───────────────────────────────────────────────
typedef struct {
    uint32_t version;         // protocol version
    uint8_t  slot;            // slot number
    uint8_t  id;              // device id
    uint8_t  rsvd;            // reserved
    uint8_t  error_info;      // error info
    uint32_t timestamp_point; // timestamp of the point (ns)
    uint8_t  data_type;       // data type (LivoxDataType)
    uint8_t  data[1440];      // data payload
    uint32_t length;          // actual data length
    uint32_t crc;             // CRC32
} LivoxEthPacket;

#pragma pack(pop)

// ─── Callback typedefs ─────────────────────────────────────────────
typedef void (*DeviceConnectCallback)(const uint8_t dev_type, const char* serial_num, const char* user_data);
typedef void (*DeviceStateUpdateCallback)(const uint8_t dev_type, const char* serial_num, uint8_t state);
typedef void (*DataCallback)(uint8_t handle, LivoxEthPacket* data, uint32_t data_num, void* client_data);

// ─── SDK API functions ─────────────────────────────────────────────
bool Init();
void UninitLivoxSdk();

void SetBroadcastCallback(DeviceConnectCallback cb);
void SetDeviceStateUpdateCallback(DeviceStateUpdateCallback cb);

bool StartSample();
void StopSample();

uint8_t AddDeviceToConnect(const char* broadcast_code, uint8_t* handle);
bool SetDataCallback(uint8_t handle, DataCallback cb, void* client_data);

// ─── Query functions ───────────────────────────────────────────────
bool GetLidarFirmwareVersion(uint8_t handle, uint8_t* version, uint8_t size);

#ifdef __cplusplus
}
#endif

#endif // LIVOX_SDK_H
