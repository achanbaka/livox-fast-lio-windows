/**
 * Livox SDK stub implementations for Windows FAST-LIO port.
 * 
 * These stubs allow the project to link without the actual Livox SDK library.
 * When a real Livox Horizon device is connected, replace with the full SDK.
 * 
 * All functions return failure/empty values -- the livox_adapter will detect
 * this and report "SDK init failed" gracefully.
 */
#include "livox_sdk.h"
#include <cstdio>

// Stored callbacks (unused in stub mode)
static DeviceConnectCallback      g_broadcast_cb      = nullptr;
static DeviceStateUpdateCallback  g_state_update_cb   = nullptr;

bool Init()
{
    std::printf("[LivoxSDK-Stub] Init() called -- no real SDK, returning false\n");
    return false;
}

void UninitLivoxSdk()
{
    g_broadcast_cb    = nullptr;
    g_state_update_cb = nullptr;
}

void SetBroadcastCallback(DeviceConnectCallback cb)
{
    g_broadcast_cb = cb;
}

void SetDeviceStateUpdateCallback(DeviceStateUpdateCallback cb)
{
    g_state_update_cb = cb;
}

bool StartSample()
{
    return false;
}

void StopSample()
{
}

uint8_t AddDeviceToConnect(const char* broadcast_code, uint8_t* handle)
{
    if (handle) *handle = 0;
    return 0;  // success code, but nothing actually happens
}

bool SetDataCallback(uint8_t handle, DataCallback cb, void* client_data)
{
    return false;
}

bool GetLidarFirmwareVersion(uint8_t handle, uint8_t* version, uint8_t size)
{
    return false;
}
