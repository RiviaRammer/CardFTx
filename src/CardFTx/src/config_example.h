#pragma once

#include <Arduino.h>

static constexpr int kAudioSampleRate = 12000;

static constexpr int kPinCodecSda = 8;
static constexpr int kPinCodecScl = 9;
static constexpr int kPinI2sBclk = 41;   // ES8311 SCLK
static constexpr int kPinI2sDin = 46;    // ES8311 ASDOUT -> ESP32-S3 RX
static constexpr int kPinI2sLrck = 43;
static constexpr int kPinI2sDout = 42;   // ESP32-S3 TX -> ES8311 DSDIN
static constexpr int kPinAmpEnable = -1; // Set to AMP_EN GPIO when known.

static constexpr uint8_t kEs8311Address = 0x18;
static constexpr float kFt8DefaultToneHz = 1000.0f;

static constexpr const char* kDefaultWifiSsid = "CMCC-WTF";
static constexpr const char* kDefaultWifiPassword = "WTFWTFWTF";
