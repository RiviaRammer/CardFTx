#pragma once

#include <Arduino.h>

static constexpr int kAudioSampleRate = 12000;

enum class FtxAutoMode : uint8_t {
    Manual,
    AutoAnswer,
    AutoCQ
};

static constexpr FtxAutoMode kFtxAutoMode = FtxAutoMode::Manual; // Manual, AutoAnswer, AutoCQ
static constexpr const char* kFtxTxSlot = "Even"; // Odd, Even
static constexpr float kFtxDefaultTxToneHz = 1000.0f;

static constexpr const char* kFtxStationCallsign = "BG6WRI";
static constexpr const char* kFtxStationGrid = "ON80";
static constexpr int kFtxFallbackReportDb = -10;

static constexpr const char* kDefaultWifiSsid = "CMCC-WTF";
static constexpr const char* kDefaultWifiPassword = "WTFWTFWTF";
