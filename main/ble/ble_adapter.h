#pragma once

// -----------------------------------------------------------
// BLE Unified Protocol - Adapter for migrating from old BLE API
// This wraps the old callbacks to work with the new unified protocol
// -----------------------------------------------------------

#include "ble_unified.h"

// Helper to bridge old callbacks to new unified protocol
// This allows gradual migration without rewriting all callback functions

namespace BleAdapter {

// Current state stored here for building full status responses
static int8_t s_eqBass = 0;
static int8_t s_eqMid = 0;
static int8_t s_eqTreble = 0;
static uint8_t s_control = 0;
static uint8_t s_ledSettings[10] = {0};
static uint8_t s_soundStatus = 0;
static char s_deviceName[64] = {0};
static char s_fwVersion[32] = {0};

// Store callbacks from old API style
static BleUnifiedService::EqCallback s_eqCb = nullptr;
static BleUnifiedService::ControlCallback s_controlCb = nullptr;
static BleUnifiedService::NameCallback s_nameCb = nullptr;
static BleUnifiedService::LedCallback s_ledCb = nullptr;
static BleUnifiedService::LedEffectCallback s_ledEffectCb = nullptr;
static BleUnifiedService::LedBrightnessCallback s_ledBrightCb = nullptr;
static BleUnifiedService::SoundMuteCallback s_soundMuteCb = nullptr;
static BleUnifiedService::SoundDeleteCallback s_soundDeleteCb = nullptr;
static BleUnifiedService::SoundDataCallback s_soundDataCb = nullptr;
static BleUnifiedService::OtaCallback s_otaCb = nullptr;

// EQ preset definitions (matching app presets)
static const int8_t EQ_PRESETS[][3] = {
    {0, 0, 0},       // 0: Balanced
    {6, -2, -2},     // 1: Deep Bass
    {-2, 3, 1},      // 2: Clear Vocals
    {-3, -1, 5},     // 3: Bright
    {3, 1, 3},       // 4: Rock
    {4, 2, 4},       // 5: Party
    {-2, 0, 4},      // 6: Podcast
    {1, -1, 2},      // 7: Jazz
    {-1, 2, 0},      // 8: Acoustic
    {5, -2, -1},     // 9: Hip Hop
    {3, 0, 3},       // 10: Electronic
    {2, 4, 1},       // 11: R&B
};

// EQ preset callback - converts preset ID to bass/mid/treble values
static void onEqPreset(uint8_t presetId) {
    if (presetId >= sizeof(EQ_PRESETS) / sizeof(EQ_PRESETS[0])) {
        presetId = 0;  // Default to Balanced
    }
    
    int8_t bass = EQ_PRESETS[presetId][0];
    int8_t mid = EQ_PRESETS[presetId][1];
    int8_t treble = EQ_PRESETS[presetId][2];
    
    if (s_eqCb) {
        s_eqCb(bass, mid, treble);
    }
}

}  // namespace BleAdapter
