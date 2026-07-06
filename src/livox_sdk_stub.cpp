/*
 * Stub Livox SDK implementation used only when third_party/Livox-SDK is absent.
 * Real hardware builds link the official SDK and do not compile this file.
 */
#include "livox_sdk.h"
#include <cstdio>

static DeviceBroadcastCallback g_broadcast_cb = nullptr;
static DeviceStateUpdateCallback g_state_cb = nullptr;

void GetLivoxSdkVersion(LivoxSdkVersion *version)
{
    if (!version) return;
    version->major = 0;
    version->minor = 0;
    version->patch = 0;
}

void DisableConsoleLogger() {}

bool Init()
{
    std::printf("[LivoxSDK-Stub] Init() called; no real SDK is linked.\n");
    return false;
}

bool Start()
{
    return false;
}

void Uninit()
{
    g_broadcast_cb = nullptr;
    g_state_cb = nullptr;
}

void SaveLoggerFile() {}

void SetBroadcastCallback(DeviceBroadcastCallback cb)
{
    g_broadcast_cb = cb;
}

void SetDeviceStateUpdateCallback(DeviceStateUpdateCallback cb)
{
    g_state_cb = cb;
}

livox_status AddLidarToConnect(const char *, uint8_t *handle)
{
    if (handle) *handle = 0;
    return kStatusFailure;
}

livox_status QueryDeviceInformation(uint8_t, DeviceInformationCallback, void *)
{
    return kStatusFailure;
}

void SetDataCallback(uint8_t, DataCallback, void *) {}

livox_status SetErrorMessageCallback(uint8_t, ErrorMessageCallback)
{
    return kStatusFailure;
}

livox_status SetCartesianCoordinate(uint8_t, CommonCommandCallback, void *)
{
    return kStatusFailure;
}

livox_status LidarStartSampling(uint8_t, CommonCommandCallback, void *)
{
    return kStatusFailure;
}

livox_status LidarStopSampling(uint8_t, CommonCommandCallback, void *)
{
    return kStatusFailure;
}

livox_status LidarSetImuPushFrequency(uint8_t, ImuFreq, CommonCommandCallback, void *)
{
    return kStatusFailure;
}

livox_status LidarSetPointCloudReturnMode(uint8_t, PointCloudReturnMode, CommonCommandCallback, void *)
{
    return kStatusFailure;
}
