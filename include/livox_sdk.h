#ifndef LIVOX_SDK_COMPAT_H
#define LIVOX_SDK_COMPAT_H

#if defined(LIVOX_USE_REAL_SDK) && defined(LIVOX_REAL_SDK_HEADER)
#include LIVOX_REAL_SDK_HEADER
typedef LivoxImuPoint LivoxImuPacket;
#else

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define kMaxLidarCount 32
#define kBroadcastCodeSize 16

typedef enum {
  kDeviceTypeHub = 0,
  kDeviceTypeLidarMid40 = 1,
  kDeviceTypeLidarTele = 2,
  kDeviceTypeLidarHorizon = 3,
  kDeviceTypeLidarMid70 = 6,
  kDeviceTypeLidarAvia = 7
} DeviceType;

typedef enum {
  kLidarStateInit = 0,
  kLidarStateNormal = 1,
  kLidarStatePowerSaving = 2,
  kLidarStateStandBy = 3,
  kLidarStateError = 4,
  kLidarStateUnknown = 5
} LidarState;

typedef enum {
  kStatusSendFailed = -9,
  kStatusHandlerImplNotExist = -8,
  kStatusInvalidHandle = -7,
  kStatusChannelNotExist = -6,
  kStatusNotEnoughMemory = -5,
  kStatusTimeout = -4,
  kStatusNotSupported = -3,
  kStatusNotConnected = -2,
  kStatusFailure = -1,
  kStatusSuccess = 0
} LivoxStatus;

typedef int32_t livox_status;

typedef enum {
  kEventConnect = 0,
  kEventDisconnect = 1,
  kEventStateChange = 2,
  kEventHubConnectionChange = 3
} DeviceEvent;

typedef enum {
  kTimestampTypeNoSync = 0,
  kTimestampTypePtp = 1,
  kTimestampTypeRsvd = 2,
  kTimestampTypePpsGps = 3,
  kTimestampTypePps = 4,
  kTimestampTypeUnknown = 5
} TimestampType;

typedef enum {
  kCartesian = 0,
  kSpherical = 1,
  kExtendCartesian = 2,
  kExtendSpherical = 3,
  kDualExtendCartesian = 4,
  kDualExtendSpherical = 5,
  kImu = 6,
  kTripleExtendCartesian = 7,
  kTripleExtendSpherical = 8,
  kMaxPointDataType = 9
} PointDataType;

typedef enum {
  kFirstReturn = 0,
  kStrongestReturn = 1,
  kDualReturn = 2,
  kTripleReturn = 3
} PointCloudReturnMode;

typedef enum {
  kImuFreq0Hz = 0,
  kImuFreq200Hz = 1
} ImuFreq;

#pragma pack(push, 1)

typedef struct {
  int major;
  int minor;
  int patch;
} LivoxSdkVersion;

typedef struct {
  int32_t x;
  int32_t y;
  int32_t z;
  uint8_t reflectivity;
} LivoxRawPoint;

typedef struct {
  uint32_t depth;
  uint16_t theta;
  uint16_t phi;
  uint8_t reflectivity;
} LivoxSpherPoint;

typedef struct {
  int32_t x;
  int32_t y;
  int32_t z;
  uint8_t reflectivity;
  uint8_t tag;
} LivoxExtendRawPoint;

typedef struct {
  uint32_t depth;
  uint16_t theta;
  uint16_t phi;
  uint8_t reflectivity;
  uint8_t tag;
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
  float gyro_x;
  float gyro_y;
  float gyro_z;
  float acc_x;
  float acc_y;
  float acc_z;
} LivoxImuPoint;

typedef LivoxImuPoint LivoxImuPacket;

typedef struct {
  uint32_t error_code;
} ErrorMessage;

typedef union {
  uint32_t progress;
  ErrorMessage status_code;
} StatusUnion;

typedef struct {
  char broadcast_code[kBroadcastCodeSize];
  uint8_t handle;
  uint8_t slot;
  uint8_t id;
  uint8_t type;
  uint16_t data_port;
  uint16_t cmd_port;
  uint16_t sensor_port;
  uint8_t state;
  StatusUnion status;
  uint8_t feature;
} DeviceInfo;

typedef struct {
  char broadcast_code[kBroadcastCodeSize];
  uint8_t dev_type;
  uint16_t reserved;
} BroadcastDeviceInfo;

typedef struct {
  uint8_t firmware_version[4];
} DeviceInformationResponse;

typedef struct {
  uint8_t version;
  uint8_t slot;
  uint8_t id;
  uint8_t rsvd;
  uint32_t err_code;
  uint8_t timestamp_type;
  uint8_t data_type;
  uint8_t timestamp[8];
  uint8_t data[1440];
} LivoxEthPacket;

#pragma pack(pop)

typedef void (*DeviceBroadcastCallback)(const BroadcastDeviceInfo *info);
typedef void (*DeviceStateUpdateCallback)(const DeviceInfo *device, DeviceEvent type);
typedef void (*DeviceInformationCallback)(livox_status status,
                                          uint8_t handle,
                                          DeviceInformationResponse *response,
                                          void *client_data);
typedef void (*DataCallback)(uint8_t handle, LivoxEthPacket *data, uint32_t data_num, void *client_data);
typedef void (*CommonCommandCallback)(livox_status status, uint8_t handle, uint8_t response, void *client_data);
typedef void (*ErrorMessageCallback)(livox_status status, uint8_t handle, ErrorMessage *message);

void GetLivoxSdkVersion(LivoxSdkVersion *version);
void DisableConsoleLogger();
bool Init();
bool Start();
void Uninit();
void SaveLoggerFile();
void SetBroadcastCallback(DeviceBroadcastCallback cb);
void SetDeviceStateUpdateCallback(DeviceStateUpdateCallback cb);
livox_status AddLidarToConnect(const char *broadcast_code, uint8_t *handle);
livox_status QueryDeviceInformation(uint8_t handle, DeviceInformationCallback cb, void *client_data);
void SetDataCallback(uint8_t handle, DataCallback cb, void *client_data);
livox_status SetErrorMessageCallback(uint8_t handle, ErrorMessageCallback cb);
livox_status SetCartesianCoordinate(uint8_t handle, CommonCommandCallback cb, void *client_data);
livox_status LidarStartSampling(uint8_t handle, CommonCommandCallback cb, void *client_data);
livox_status LidarStopSampling(uint8_t handle, CommonCommandCallback cb, void *client_data);
livox_status LidarSetImuPushFrequency(uint8_t handle, ImuFreq freq, CommonCommandCallback cb, void *client_data);
livox_status LidarSetPointCloudReturnMode(uint8_t handle,
                                          PointCloudReturnMode mode,
                                          CommonCommandCallback cb,
                                          void *client_data);

#ifdef __cplusplus
}
#endif

#endif

#endif
