#include "Ft8ArduinoApp.h"

#include "config.h"
#include "M5Cardputer.h"

#include <WiFi.h>
#include <esp_heap_caps.h>
#include <math.h>
#include <string.h>
#include <sys/time.h>
#include <time.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

extern "C" {
#include "common/monitor.h"
#include "ft8/constants.h"
#include "ft8/decode.h"
#include "ft8/encode.h"
#include "ft8/message.h"
}

static constexpr int kTimeOsr = 1;
static constexpr int kFreqOsr = 1;
static constexpr int kMinScore = 10;
static constexpr int kMaxCandidates = 140;
static constexpr int kLdpcIterations = 25;
static constexpr int kMaxDecodedMessages = 50;
static constexpr int kCallsignHashTableSize = 256;
static constexpr int kSpeakerChunkSamples = 512;
static constexpr int kSpeakerChannel = 0;
static constexpr uint32_t kDecodeTaskStackBytes = 16 * 1024;
static constexpr int kFt8BlockSamples = static_cast<int>(kAudioSampleRate * FT8_SYMBOL_PERIOD);
static constexpr bool kDrawRxWaterfall = true;
static constexpr uint8_t kDisplayTextSize = 2;
static constexpr uint8_t kDisplaySmallTextSize = 1;
static constexpr int kDisplayLineHeight = 18;
static constexpr int kCommandMaxLength = 63;
static constexpr float kFt8SymbolBt = 2.0f;
static constexpr float kGfskConstK = 5.336446f;

static SemaphoreHandle_t radioMutex;
static char txMessage[FTX_MAX_MESSAGE_LENGTH] = "CQ BG6WRI ON80";
static float txToneHz = kFt8DefaultToneHz;
static int16_t speakerBuffers[3][kSpeakerChunkSamples];
static uint8_t speakerBufferIndex = 0;
static bool isTransmitting = false;
static bool isReceiving = false;
static uint32_t lastDisplayMs = 0;
static uint32_t displayHoldUntilMs = 0;
static String keyboardCommand;
static int16_t rxPcm[kFt8BlockSamples];
static float rxFrame[kFt8BlockSamples];
static int32_t rxPeak = 0;
static uint64_t rxAbsSum = 0;
static uint32_t rxSampleTotal = 0;
static char wifiSsid[33];
static char wifiPassword[65];

static struct {
    char callsign[12];
    uint32_t hash;
} callsignHashtable[kCallsignHashTableSize];

static int callsignHashtableSize = 0;

static void hashtableInit()
{
    callsignHashtableSize = 0;
    memset(callsignHashtable, 0, sizeof(callsignHashtable));
}

static void hashtableCleanup(uint8_t maxAge)
{
    for (auto& item : callsignHashtable) {
        if (item.callsign[0] == '\0') {
            continue;
        }
        uint8_t age = static_cast<uint8_t>(item.hash >> 24);
        if (age > maxAge) {
            item.callsign[0] = '\0';
            item.hash = 0;
            callsignHashtableSize--;
        } else {
            item.hash = (static_cast<uint32_t>(age + 1) << 24) | (item.hash & 0x3fffffu);
        }
    }
}

static void hashtableAdd(const char* callsign, uint32_t hash)
{
    uint16_t hash10 = (hash >> 12) & 0x03ffu;
    int idx = (hash10 * 23) % kCallsignHashTableSize;
    for (int probe = 0; probe < kCallsignHashTableSize; ++probe) {
        if (callsignHashtable[idx].callsign[0] == '\0') {
            callsignHashtableSize++;
            strncpy(callsignHashtable[idx].callsign, callsign, 11);
            callsignHashtable[idx].callsign[11] = '\0';
            callsignHashtable[idx].hash = hash;
            return;
        }
        if (((callsignHashtable[idx].hash & 0x3fffffu) == hash) &&
            strcmp(callsignHashtable[idx].callsign, callsign) == 0) {
            callsignHashtable[idx].hash &= 0x3fffffu;
            return;
        }
        idx = (idx + 1) % kCallsignHashTableSize;
    }

    Serial.println("Callsign hash table full");
}

static bool hashtableLookup(ftx_callsign_hash_type_t hashType, uint32_t hash, char* callsign)
{
    uint8_t hashShift = (hashType == FTX_CALLSIGN_HASH_10_BITS) ? 12 :
        (hashType == FTX_CALLSIGN_HASH_12_BITS ? 10 : 0);
    uint16_t hash10 = (hash >> (12 - hashShift)) & 0x03ffu;
    int idx = (hash10 * 23) % kCallsignHashTableSize;

    for (int probe = 0; probe < kCallsignHashTableSize; ++probe) {
        if (callsignHashtable[idx].callsign[0] == '\0') {
            break;
        }
        if (((callsignHashtable[idx].hash & 0x3fffffu) >> hashShift) == hash) {
            strcpy(callsign, callsignHashtable[idx].callsign);
            return true;
        }
        idx = (idx + 1) % kCallsignHashTableSize;
    }

    callsign[0] = '\0';
    return false;
}

static ftx_callsign_hash_interface_t hashIf = { hashtableLookup, hashtableAdd };

static void* appHeapMalloc(size_t size)
{
    void* ptr = heap_caps_malloc(size, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (ptr == nullptr) {
        ptr = heap_caps_malloc(size, MALLOC_CAP_8BIT);
    }
    return ptr;
}

static void* appHeapCalloc(size_t count, size_t size)
{
    size_t total = count * size;
    void* ptr = appHeapMalloc(total);
    if (ptr != nullptr) {
        memset(ptr, 0, total);
    }
    return ptr;
}

static bool timeIsSynced()
{
    time_t now = time(nullptr);
    return now > 1700000000;
}

static void formatUtcTime(char* out, size_t outSize)
{
    time_t now = time(nullptr);
    if (!timeIsSynced()) {
        snprintf(out, outSize, "--:--:--");
        return;
    }

    struct tm utc;
    gmtime_r(&now, &utc);
    snprintf(out, outSize, "%02d:%02d:%02d", utc.tm_hour, utc.tm_min, utc.tm_sec);
}

static int secondsToNextFt8Slot()
{
    if (!timeIsSynced()) {
        return -1;
    }

    time_t now = time(nullptr);
    int sec = static_cast<int>(now % 15);
    return (15 - sec) % 15;
}

static void renderCommandLine()
{
    auto& display = M5Cardputer.Display;
    int y = static_cast<int>(display.height()) - 17;
    int maxChars = max(4, static_cast<int>(display.width()) / 12);
    String text = ">";
    text += keyboardCommand;
    if (text.length() > static_cast<unsigned>(maxChars)) {
        text = ">";
        text += keyboardCommand.substring(keyboardCommand.length() - (maxChars - 1));
    }

    display.fillRect(0, y - 2, display.width(), display.height() - y + 2, TFT_BLACK);
    display.drawFastHLine(4, y - 3, display.width() - 8, TFT_DARKGREY);
    display.setTextSize(kDisplayTextSize);
    display.setTextColor(TFT_WHITE, TFT_BLACK);
    display.setCursor(4, y);
    display.printf("%s", text.c_str());
}

static void renderStatus(bool force = false)
{
    uint32_t nowMs = millis();
    if (!force && displayHoldUntilMs != 0 && static_cast<int32_t>(displayHoldUntilMs - nowMs) > 0) {
        return;
    }
    if (displayHoldUntilMs != 0 && static_cast<int32_t>(nowMs - displayHoldUntilMs) >= 0) {
        displayHoldUntilMs = 0;
    }
    if (!force && (nowMs - lastDisplayMs < 1000)) {
        return;
    }
    lastDisplayMs = nowMs;

    char utcText[16];
    formatUtcTime(utcText, sizeof(utcText));
    int waitSec = secondsToNextFt8Slot();
    const char* statusText = isTransmitting ? "TX" : (isReceiving ? "RX" : "Idle");
    int screenW = M5Cardputer.Display.width();
    int statusX = screenW - static_cast<int>(strlen(statusText)) * 12 - 4;

    M5Cardputer.Display.fillScreen(TFT_BLACK);
    M5Cardputer.Display.setTextColor(TFT_GREEN, TFT_BLACK);
    M5Cardputer.Display.setTextSize(kDisplayTextSize);
    M5Cardputer.Display.setCursor(4, 4);
    M5Cardputer.Display.printf("FT8");
    M5Cardputer.Display.setTextColor(isTransmitting ? TFT_ORANGE : (isReceiving ? TFT_CYAN : TFT_WHITE), TFT_BLACK);
    M5Cardputer.Display.setCursor(max(52, statusX), 4);
    M5Cardputer.Display.printf("%s", statusText);
    M5Cardputer.Display.drawFastHLine(4, 23, screenW - 8, TFT_DARKGREY);
    M5Cardputer.Display.setTextColor(TFT_WHITE, TFT_BLACK);
    M5Cardputer.Display.setCursor(4, 29);
    M5Cardputer.Display.printf("UTC %s", utcText);
    M5Cardputer.Display.setCursor(4, 29 + kDisplayLineHeight);
    M5Cardputer.Display.printf("Slot ");
    if (waitSec < 0) {
        M5Cardputer.Display.printf("--");
    } else {
        M5Cardputer.Display.printf("%ds", waitSec);
    }
    M5Cardputer.Display.drawFastHLine(4, 70, screenW - 8, TFT_DARKGREY);
    M5Cardputer.Display.setCursor(4, 76);
    M5Cardputer.Display.printf("Freq %.0fHz", txToneHz);
    char displayMessage[19];
    strncpy(displayMessage, txMessage, sizeof(displayMessage) - 1);
    displayMessage[sizeof(displayMessage) - 1] = '\0';
    M5Cardputer.Display.setCursor(4, 76 + kDisplayLineHeight);
    M5Cardputer.Display.printf("%s", displayMessage);
    renderCommandLine();
}

static void returnHomeForCommandInput()
{
    if (displayHoldUntilMs == 0) {
        return;
    }

    displayHoldUntilMs = 0;
    renderStatus(true);
}

static uint8_t waterfallMagnitudeToByte(const WF_ELEM_T& value)
{
#ifdef WATERFALL_USE_PHASE
    int scaled = static_cast<int>(2.0f * value.mag + 240.0f);
    return static_cast<uint8_t>(scaled < 0 ? 0 : (scaled > 255 ? 255 : scaled));
#else
    return value;
#endif
}

static uint16_t waterfallColor(uint8_t mag)
{
    if (mag < 50) {
        return TFT_BLACK;
    }
    if (mag < 90) {
        return TFT_NAVY;
    }
    if (mag < 130) {
        return TFT_BLUE;
    }
    if (mag < 170) {
        return TFT_CYAN;
    }
    if (mag < 215) {
        return TFT_YELLOW;
    }
    return TFT_ORANGE;
}

static void renderRxWaterfall(const monitor_t& monitor)
{
    if (!kDrawRxWaterfall || monitor.wf.num_blocks == 0 || monitor.wf.mag == nullptr) {
        return;
    }

    auto& display = M5Cardputer.Display;
    int screenW = display.width();
    int screenH = display.height();
    const int left = 6;
    const int top = 32;
    const int right = screenW - 5;
    const int bottom = screenH - 36;
    const int plotW = max(1, right - left + 1);
    const int plotH = max(1, bottom - top + 1);

    if (monitor.wf.num_blocks == 1) {
        display.fillRect(0, 0, screenW, screenH, TFT_BLACK);
        display.drawRect(left - 1, top - 1, plotW + 2, plotH + 2, TFT_DARKGREY);
        for (int x = 1; x < 4; ++x) {
            int gx = left + (plotW * x) / 4;
            display.drawFastVLine(gx, top, plotH, TFT_DARKGREY);
        }
        display.setTextSize(kDisplaySmallTextSize);
        display.setTextColor(TFT_DARKGREY, TFT_BLACK);
        display.setCursor(left, bottom + 4);
        display.printf("%dHz", static_cast<int>(monitor.min_bin / monitor.symbol_period));
        display.setCursor(screenW - 43, bottom + 4);
        display.printf("%dHz", static_cast<int>((monitor.max_bin - 1) / monitor.symbol_period));
    }

    display.fillRect(0, 0, screenW, top - 1, TFT_BLACK);
    display.setTextSize(kDisplayTextSize);
    char progressText[12];
    snprintf(progressText, sizeof(progressText), "%d/%d", monitor.wf.num_blocks, monitor.wf.max_blocks);
    int rxX = screenW - 30;
    int progressX = rxX - static_cast<int>(strlen(progressText)) * 12 - 8;
    display.setTextColor(TFT_WHITE, TFT_BLACK);
    display.setCursor(max(4, progressX), 4);
    display.printf("%s", progressText);
    display.setTextColor(TFT_GREEN, TFT_BLACK);
    display.setCursor(rxX, 4);
    display.printf("RX");
    display.drawFastHLine(4, top - 4, screenW - 8, TFT_DARKGREY);

    int block = monitor.wf.num_blocks - 1;
    int y0 = top + (block * plotH) / monitor.wf.max_blocks;
    int y1 = top + ((block + 1) * plotH) / monitor.wf.max_blocks;
    int rowH = max(1, y1 - y0);
    const int blockOffset = block * monitor.wf.block_stride;

    uint8_t rowMin = 255;
    uint8_t rowMax = 0;
    for (int i = 0; i < monitor.wf.block_stride; ++i) {
        uint8_t mag = waterfallMagnitudeToByte(monitor.wf.mag[blockOffset + i]);
        if (mag < rowMin) {
            rowMin = mag;
        }
        if (mag > rowMax) {
            rowMax = mag;
        }
    }
    int rowSpan = max(12, static_cast<int>(rowMax) - static_cast<int>(rowMin));

    for (int x = 0; x < plotW; ++x) {
        int firstBin = (x * monitor.wf.num_bins) / plotW;
        int lastBin = ((x + 1) * monitor.wf.num_bins) / plotW;
        if (lastBin <= firstBin) {
            lastBin = firstBin + 1;
        }

        uint8_t columnMag = 0;
        for (int timeSub = 0; timeSub < monitor.wf.time_osr; ++timeSub) {
            for (int freqSub = 0; freqSub < monitor.wf.freq_osr; ++freqSub) {
                int offset = blockOffset;
                offset += ((timeSub * monitor.wf.freq_osr) + freqSub) * monitor.wf.num_bins;
                for (int bin = firstBin; bin < lastBin && bin < monitor.wf.num_bins; ++bin) {
                    uint8_t mag = waterfallMagnitudeToByte(monitor.wf.mag[offset + bin]);
                    if (mag > columnMag) {
                        columnMag = mag;
                    }
                }
            }
        }

        int normalized = ((static_cast<int>(columnMag) - static_cast<int>(rowMin)) * 255) / rowSpan;
        normalized = normalized < 0 ? 0 : (normalized > 255 ? 255 : normalized);
        display.fillRect(left + x, y0, 1, rowH, waterfallColor(static_cast<uint8_t>(normalized)));
    }
    renderCommandLine();
}

static bool connectWifi(const char* ssid, const char* password, uint32_t timeoutMs = 15000)
{
    if (ssid == nullptr || ssid[0] == '\0') {
        Serial.println("WiFi SSID is empty. Use: set SSID your_wifi_name");
        return false;
    }

    Serial.printf("Connecting WiFi: %s\n", ssid);
    WiFi.mode(WIFI_STA);
    WiFi.begin(ssid, password);

    uint32_t start = millis();
    while (WiFi.status() != WL_CONNECTED && millis() - start < timeoutMs) {
        delay(250);
        Serial.print(".");
        renderStatus(true);
    }
    Serial.println();

    if (WiFi.status() != WL_CONNECTED) {
        Serial.println("WiFi connect failed");
        renderStatus(true);
        return false;
    }

    Serial.print("WiFi IP: ");
    Serial.println(WiFi.localIP());
    renderStatus(true);
    return true;
}

static bool syncTime(uint32_t timeoutMs = 15000)
{
    if (WiFi.status() != WL_CONNECTED) {
        Serial.println("WiFi is not connected");
        return false;
    }

    Serial.println("Syncing UTC time with NTP...");
    configTime(0, 0, "pool.ntp.org", "time.nist.gov", "time.google.com");

    uint32_t start = millis();
    while (!timeIsSynced() && millis() - start < timeoutMs) {
        delay(250);
        Serial.print(".");
    }
    Serial.println();

    if (!timeIsSynced()) {
        Serial.println("NTP sync failed");
        renderStatus(true);
        return false;
    }

    char utcText[16];
    formatUtcTime(utcText, sizeof(utcText));
    Serial.printf("UTC synced: %s\n", utcText);
    renderStatus(true);
    return true;
}

static bool waitForFt8Slot()
{
    if (!timeIsSynced()) {
        Serial.println("Time is not synced. Use: set SSID ..., set PASS ..., then sync");
        renderStatus(true);
        return false;
    }

    struct timeval tvStart;
    gettimeofday(&tvStart, nullptr);
    time_t target = ((tvStart.tv_sec / 15) + 1) * 15;
    int waitSec = static_cast<int>(target - tvStart.tv_sec);
    Serial.printf("Waiting for next FT8 slot: %d s\n", waitSec);

    while (true) {
        struct timeval tv;
        gettimeofday(&tv, nullptr);
        int64_t remainingUs = (static_cast<int64_t>(target - tv.tv_sec) * 1000000LL) - tv.tv_usec;
        if (remainingUs <= 0) {
            break;
        }

        M5Cardputer.update();
        if (remainingUs > 300000) {
            renderStatus();
            delay(50);
        } else {
            delay(5);
        }
    }

    return true;
}

static void ensureSpeakerReady()
{
    if (!M5Cardputer.Speaker.isEnabled()) {
        M5Cardputer.Speaker.begin();
        M5Cardputer.Speaker.setVolume(255);
    }
}

static void ensureMicReady()
{
    if (M5Cardputer.Speaker.isEnabled()) {
        M5Cardputer.Speaker.stop();
        M5Cardputer.Speaker.end();
    }

    if (M5Cardputer.Mic.isEnabled()) {
        while (M5Cardputer.Mic.isRecording()) {
            delay(1);
        }
        M5Cardputer.Mic.end();
    }

    if (!M5Cardputer.Mic.isEnabled()) {
        auto cfg = M5Cardputer.Mic.config();
        cfg.sample_rate = kAudioSampleRate;
        cfg.left_channel = 1;
        cfg.stereo = 0;
        cfg.over_sampling = 1;
        cfg.noise_filter_level = 0;
        cfg.magnification = 64;
        cfg.dma_buf_len = 256;
        cfg.dma_buf_count = 8;
        M5Cardputer.Mic.config(cfg);
        M5Cardputer.Mic.begin();
    }
}

static void stopMicAndRestoreSpeaker()
{
    while (M5Cardputer.Mic.isRecording()) {
        delay(1);
    }
    if (M5Cardputer.Mic.isEnabled()) {
        M5Cardputer.Mic.end();
    }
    M5Cardputer.Speaker.stop();
    M5Cardputer.Speaker.end();
}

static void decodeWaterfall(const monitor_t& monitor)
{
    const ftx_waterfall_t* wf = &monitor.wf;
    ftx_candidate_t* candidateList = static_cast<ftx_candidate_t*>(
        appHeapMalloc(sizeof(ftx_candidate_t) * kMaxCandidates));
    ftx_message_t* decoded = static_cast<ftx_message_t*>(
        appHeapMalloc(sizeof(ftx_message_t) * kMaxDecodedMessages));
    ftx_message_t** decodedHashtable = static_cast<ftx_message_t**>(
        appHeapCalloc(kMaxDecodedMessages, sizeof(ftx_message_t*)));

    if (candidateList == nullptr || decoded == nullptr || decodedHashtable == nullptr) {
        Serial.println("RX decode allocation failed");
        M5Cardputer.Display.setTextSize(kDisplayTextSize);
        M5Cardputer.Display.setTextColor(TFT_RED, TFT_BLACK);
        M5Cardputer.Display.printf("Decode alloc fail\n");
        renderCommandLine();
        free(decodedHashtable);
        free(decoded);
        free(candidateList);
        return;
    }

    int numCandidates = ftx_find_candidates(wf, kMaxCandidates, candidateList, kMinScore);
    int numDecoded = 0;

    Serial.printf("RX candidates: %d, max_mag=%.1f dB\n", numCandidates, monitor.max_mag);

    M5Cardputer.Display.fillScreen(TFT_BLACK);
    M5Cardputer.Display.setCursor(4, 4);
    M5Cardputer.Display.setTextSize(kDisplayTextSize);
    M5Cardputer.Display.setTextColor(TFT_GREEN, TFT_BLACK);
    M5Cardputer.Display.printf("RX decode");
    M5Cardputer.Display.drawFastHLine(4, 23, M5Cardputer.Display.width() - 8, TFT_DARKGREY);
    M5Cardputer.Display.setTextColor(TFT_WHITE, TFT_BLACK);
    M5Cardputer.Display.setCursor(4, 30);
    M5Cardputer.Display.printf("Cand %d", numCandidates);
    M5Cardputer.Display.drawFastHLine(4, 51, M5Cardputer.Display.width() - 8, TFT_DARKGREY);

    for (int idx = 0; idx < numCandidates; ++idx) {
        const ftx_candidate_t* candidate = &candidateList[idx];
        ftx_message_t message;
        ftx_decode_status_t status = {};
        if (!ftx_decode_candidate(wf, candidate, kLdpcIterations, &message, &status)) {
            continue;
        }

        if (numDecoded >= kMaxDecodedMessages) {
            Serial.println("RX decoded table full");
            break;
        }

        int hashIndex = message.hash % kMaxDecodedMessages;
        bool foundEmpty = false;
        bool foundDuplicate = false;
        for (int probe = 0; probe < kMaxDecodedMessages && !foundEmpty && !foundDuplicate; ++probe) {
            if (decodedHashtable[hashIndex] == nullptr) {
                foundEmpty = true;
            } else if ((decodedHashtable[hashIndex]->hash == message.hash) &&
                memcmp(decodedHashtable[hashIndex]->payload, message.payload, sizeof(message.payload)) == 0) {
                foundDuplicate = true;
            } else {
                hashIndex = (hashIndex + 1) % kMaxDecodedMessages;
            }
        }

        if (!foundEmpty) {
            continue;
        }

        memcpy(&decoded[hashIndex], &message, sizeof(message));
        decodedHashtable[hashIndex] = &decoded[hashIndex];
        numDecoded++;

        float freqHz = (monitor.min_bin + candidate->freq_offset +
            static_cast<float>(candidate->freq_sub) / wf->freq_osr) / monitor.symbol_period;
        float timeSec = (candidate->time_offset +
            static_cast<float>(candidate->time_sub) / wf->time_osr) * monitor.symbol_period;
        float snr = candidate->score * 0.5f;

        char text[FTX_MAX_MESSAGE_LENGTH];
        ftx_message_offsets_t offsets;
        ftx_message_rc_t unpack = ftx_message_decode(&message, &hashIf, text, &offsets);
        if (unpack != FTX_MESSAGE_RC_OK) {
            snprintf(text, sizeof(text), "unpack error %d", static_cast<int>(unpack));
        }

        Serial.printf("FT8 %+05.1f dB %+4.2f s %4.0f Hz ~ %s\n", snr, timeSec, freqHz, text);
        if (numDecoded <= 4) {
            char shortText[15];
            strncpy(shortText, text, sizeof(shortText) - 1);
            shortText[sizeof(shortText) - 1] = '\0';
            M5Cardputer.Display.setCursor(4, 56 + (numDecoded - 1) * kDisplayLineHeight);
            M5Cardputer.Display.printf("%4.0f %s", freqHz, shortText);
        }
    }

    Serial.printf("RX decoded: %d, callsign hashes: %d\n", numDecoded, callsignHashtableSize);
    if (numDecoded == 0) {
        M5Cardputer.Display.setCursor(4, 56);
        M5Cardputer.Display.setTextColor(TFT_ORANGE, TFT_BLACK);
        M5Cardputer.Display.printf("No decode");
    }
    renderCommandLine();
    displayHoldUntilMs = millis() + 15000;
    hashtableCleanup(10);

    free(decodedHashtable);
    free(decoded);
    free(candidateList);
}

struct DecodeTaskContext {
    const monitor_t* monitor;
    SemaphoreHandle_t done;
};

static void decodeWaterfallTask(void* arg)
{
    DecodeTaskContext* context = static_cast<DecodeTaskContext*>(arg);
    decodeWaterfall(*context->monitor);
    xSemaphoreGive(context->done);
    vTaskDelete(nullptr);
}

static void runDecodeWaterfall(const monitor_t& monitor)
{
    SemaphoreHandle_t done = xSemaphoreCreateBinary();
    if (done == nullptr) {
        Serial.println("RX decode semaphore allocation failed");
        M5Cardputer.Display.setTextSize(kDisplayTextSize);
        M5Cardputer.Display.setTextColor(TFT_RED, TFT_BLACK);
        M5Cardputer.Display.printf("Decode semaphore fail\n");
        renderCommandLine();
        return;
    }

    DecodeTaskContext context = { &monitor, done };
    BaseType_t created = xTaskCreatePinnedToCore(
        decodeWaterfallTask,
        "ft8Decode",
        kDecodeTaskStackBytes,
        &context,
        1,
        nullptr,
        1);

    if (created != pdPASS) {
        Serial.println("RX decode task allocation failed");
        M5Cardputer.Display.setTextSize(kDisplayTextSize);
        M5Cardputer.Display.setTextColor(TFT_RED, TFT_BLACK);
        M5Cardputer.Display.printf("Decode task fail\n");
        renderCommandLine();
    } else {
        xSemaphoreTake(done, portMAX_DELAY);
    }

    vSemaphoreDelete(done);
}

static bool receiveOnce()
{
    if (!timeIsSynced()) {
        Serial.println("Time is not synced. Use: set SSID ..., set PASS ..., then sync");
        renderStatus(true);
        return false;
    }

    stopMicAndRestoreSpeaker();
    M5Cardputer.Speaker.end();

    monitor_config_t config;
    config.f_min = 200.0f;
    config.f_max = 3000.0f;
    config.sample_rate = kAudioSampleRate;
    config.time_osr = kTimeOsr;
    config.freq_osr = kFreqOsr;
    config.protocol = FTX_PROTOCOL_FT8;

    int maxBlocks = static_cast<int>(FT8_SLOT_TIME / FT8_SYMBOL_PERIOD);
    int minBin = static_cast<int>(config.f_min * FT8_SYMBOL_PERIOD);
    int maxBin = static_cast<int>(config.f_max * FT8_SYMBOL_PERIOD) + 1;
    int numBins = maxBin - minBin;
    size_t waterfallBytes = static_cast<size_t>(maxBlocks) * config.time_osr * config.freq_osr * numBins;
    size_t fftBytes = static_cast<size_t>(config.sample_rate * FT8_SYMBOL_PERIOD * config.freq_osr) *
        (sizeof(float) * 3 + sizeof(kiss_fft_cpx));
    Serial.printf("RX alloc estimate: waterfall=%u, fft~= %u, heap=%u\n",
        static_cast<unsigned>(waterfallBytes),
        static_cast<unsigned>(fftBytes),
        heap_caps_get_free_size(MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT));

    monitor_t monitor;
    monitor_init(&monitor, &config);
    if (monitor.window == nullptr || monitor.last_frame == nullptr ||
        monitor.fft_timedata == nullptr || monitor.fft_freqdata == nullptr ||
        monitor.fft_work == nullptr || monitor.wf.mag == nullptr) {
        Serial.println("RX monitor allocation failed");
        monitor_free(&monitor);
        ensureSpeakerReady();
        return false;
    }

    Serial.printf("RX monitor ready: block=%d, bins=%d, heap=%u\n",
        monitor.block_size, monitor.wf.num_bins,
        heap_caps_get_free_size(MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT));

    isReceiving = true;
    renderStatus(true);

    ensureMicReady();
    if (!M5Cardputer.Mic.isEnabled()) {
        Serial.println("Mic begin failed");
        monitor_free(&monitor);
        stopMicAndRestoreSpeaker();
        isReceiving = false;
        renderStatus(true);
        return false;
    }

    if (!waitForFt8Slot()) {
        stopMicAndRestoreSpeaker();
        monitor_free(&monitor);
        isReceiving = false;
        renderStatus(true);
        return false;
    }

    Serial.println("RX capture start");
    rxPeak = 0;
    rxAbsSum = 0;
    rxSampleTotal = 0;
    M5Cardputer.Display.fillScreen(TFT_BLACK);
    M5Cardputer.Display.setCursor(4, 4);
    M5Cardputer.Display.setTextSize(kDisplayTextSize);
    M5Cardputer.Display.setTextColor(TFT_RED, TFT_BLACK);
    M5Cardputer.Display.printf("RX capture");
    M5Cardputer.Display.drawFastHLine(4, 23, M5Cardputer.Display.width() - 8, TFT_DARKGREY);
    renderCommandLine();

    while (monitor.wf.num_blocks < monitor.wf.max_blocks) {
        if (!M5Cardputer.Mic.record(rxPcm, monitor.block_size, kAudioSampleRate, false)) {
            delay(1);
            continue;
        }

        for (int i = 0; i < monitor.block_size; ++i) {
            int32_t sample = rxPcm[i];
            int32_t mag = abs(sample);
            if (mag > rxPeak) {
                rxPeak = mag;
            }
            rxAbsSum += mag;
            rxFrame[i] = sample / 32768.0f;
        }
        rxSampleTotal += monitor.block_size;

        monitor_process(&monitor, rxFrame);
        renderRxWaterfall(monitor);
        if ((monitor.wf.num_blocks % 10) == 0) {
            Serial.print(".");
        }
        M5Cardputer.update();
    }
    Serial.println();
    Serial.println("RX capture done");
    Serial.printf("RX audio peak=%ld avg_abs=%lu samples=%lu\n",
        static_cast<long>(rxPeak),
        static_cast<unsigned long>(rxSampleTotal ? rxAbsSum / rxSampleTotal : 0),
        static_cast<unsigned long>(rxSampleTotal));

    stopMicAndRestoreSpeaker();
    runDecodeWaterfall(monitor);
    monitor_free(&monitor);

    isReceiving = false;
    return true;
}

static float gfskPulseValue(int index, int samplesPerSymbol)
{
    float t = index / static_cast<float>(samplesPerSymbol) - 1.5f;
    float arg1 = kGfskConstK * kFt8SymbolBt * (t + 0.5f);
    float arg2 = kGfskConstK * kFt8SymbolBt * (t - 0.5f);
    return (erff(arg1) - erff(arg2)) * 0.5f;
}

static int16_t synthSample(const uint8_t* tones, int sampleIndex, float baseToneHz, float& phase)
{
    constexpr int numSymbols = FT8_NN;
    int samplesPerSymbol = static_cast<int>(0.5f + kAudioSampleRate * FT8_SYMBOL_PERIOD);
    int waveSamples = numSymbols * samplesPerSymbol;
    float dphi = 2.0f * static_cast<float>(M_PI) * baseToneHz / kAudioSampleRate;
    float dphiPeak = 2.0f * static_cast<float>(M_PI) / samplesPerSymbol;
    int shapedIndex = sampleIndex + samplesPerSymbol;

    int firstSymbol = max(0, shapedIndex / samplesPerSymbol - 2);
    int lastSymbol = min(numSymbols - 1, shapedIndex / samplesPerSymbol);
    for (int sym = firstSymbol; sym <= lastSymbol; ++sym) {
        int pulseIndex = shapedIndex - sym * samplesPerSymbol;
        if (pulseIndex >= 0 && pulseIndex < 3 * samplesPerSymbol) {
            dphi += dphiPeak * tones[sym] * gfskPulseValue(pulseIndex, samplesPerSymbol);
        }
    }

    if (shapedIndex < 2 * samplesPerSymbol) {
        dphi += dphiPeak * tones[0] * gfskPulseValue(shapedIndex + samplesPerSymbol, samplesPerSymbol);
    }
    if (shapedIndex >= waveSamples) {
        dphi += dphiPeak * tones[numSymbols - 1] *
            gfskPulseValue(shapedIndex - waveSamples, samplesPerSymbol);
    }

    float sample = sinf(phase);
    phase = fmodf(phase + dphi, 2.0f * static_cast<float>(M_PI));

    int rampSamples = samplesPerSymbol / 8;
    if (sampleIndex < rampSamples) {
        sample *= (1.0f - cosf(static_cast<float>(M_PI) * sampleIndex / rampSamples)) * 0.5f;
    } else if (sampleIndex >= waveSamples - rampSamples) {
        int rampIndex = waveSamples - 1 - sampleIndex;
        sample *= (1.0f - cosf(static_cast<float>(M_PI) * rampIndex / rampSamples)) * 0.5f;
    }

    return static_cast<int16_t>(sample * 12000.0f);
}

static void speakerPlayChunk(const int16_t* samples, int sampleCount, bool stopCurrent = false)
{
    ensureSpeakerReady();
    while (M5Cardputer.Speaker.isPlaying(kSpeakerChannel) == 2) {
        delay(1);
    }

    int16_t* target = speakerBuffers[speakerBufferIndex];
    speakerBufferIndex = (speakerBufferIndex + 1) % 3;
    memcpy(target, samples, sampleCount * sizeof(target[0]));
    M5Cardputer.Speaker.playRaw(target, sampleCount, kAudioSampleRate, false, 1, kSpeakerChannel, stopCurrent);
}

static void writeSilenceSamples(int sampleCount)
{
    int16_t silence[kSpeakerChunkSamples] = {};
    bool first = false;
    while (sampleCount > 0) {
        int count = min(sampleCount, kSpeakerChunkSamples);
        speakerPlayChunk(silence, count, first);
        first = false;
        sampleCount -= count;
    }
}

static void playTestTone(float frequencyHz, int durationMs)
{
    Serial.printf("Playing test tone %.1f Hz for %d ms\n", frequencyHz, durationMs);
    int totalFrames = static_cast<int>((durationMs / 1000.0f) * kAudioSampleRate);
    int16_t chunk[kSpeakerChunkSamples];
    float phase = 0.0f;
    float dphi = 2.0f * static_cast<float>(M_PI) * frequencyHz / kAudioSampleRate;

    M5Cardputer.Speaker.stop();
    int frameIndex = 0;
    bool firstChunk = true;
    while (frameIndex < totalFrames) {
        int count = min(kSpeakerChunkSamples, totalFrames - frameIndex);
        for (int i = 0; i < count; ++i) {
            int16_t sample = static_cast<int16_t>(sinf(phase) * 18000.0f);
            phase = fmodf(phase + dphi, 2.0f * static_cast<float>(M_PI));
            chunk[i] = sample;
        }
        speakerPlayChunk(chunk, count, firstChunk);
        firstChunk = false;
        frameIndex += count;
    }
    writeSilenceSamples(kAudioSampleRate / 10);
    while (M5Cardputer.Speaker.isPlaying(kSpeakerChannel)) {
        delay(1);
    }
    M5Cardputer.Speaker.end();
    Serial.println("Test tone done");
}

static bool transmitFt8(const char* text, float baseToneHz, bool waitForSlot)
{
    ftx_message_t message;
    ftx_message_rc_t rc = ftx_message_encode(&message, &hashIf, text);
    if (rc != FTX_MESSAGE_RC_OK) {
        Serial.printf("TX parse failed, rc=%d\n", static_cast<int>(rc));
        renderStatus(true);
        return false;
    }

    uint8_t tones[FT8_NN];
    ft8_encode(message.payload, tones);

    int samplesPerSymbol = static_cast<int>(0.5f + kAudioSampleRate * FT8_SYMBOL_PERIOD);
    int waveSamples = FT8_NN * samplesPerSymbol;
    int slotSamples = static_cast<int>(FT8_SLOT_TIME * kAudioSampleRate);
    int silenceSamples = max(0, (slotSamples - waveSamples) / 2);

    Serial.printf("Encoding FT8: %s\n", text);
    Serial.print("Tones: ");
    for (int i = 0; i < FT8_NN; ++i) {
        Serial.print(tones[i]);
    }
    Serial.println();
    Serial.printf("Ready at %.1f Hz\n", baseToneHz);

    if (waitForSlot && !waitForFt8Slot()) {
        return false;
    }

    isTransmitting = true;
    if (!waitForSlot) {
        renderStatus(true);
    }
    Serial.printf("TX audio at %.1f Hz\n", baseToneHz);
    M5Cardputer.Speaker.stop();
    writeSilenceSamples(silenceSamples);

    int16_t chunk[kSpeakerChunkSamples];
    float phase = 0.0f;
    int sampleIndex = 0;
    while (sampleIndex < waveSamples) {
        int count = min(kSpeakerChunkSamples, waveSamples - sampleIndex);
        for (int i = 0; i < count; ++i) {
            chunk[i] = synthSample(tones, sampleIndex + i, baseToneHz, phase);
        }

        speakerPlayChunk(chunk, count);
        sampleIndex += count;
    }

    writeSilenceSamples(silenceSamples);
    while (M5Cardputer.Speaker.isPlaying(kSpeakerChannel)) {
        delay(1);
    }
    Serial.println("TX done");
    M5Cardputer.Speaker.end();
    isTransmitting = false;
    renderStatus(true);
    return true;
}

static void setTxMessage(const String& message)
{
    String trimmed = message;
    trimmed.trim();
    if (trimmed.length() == 0) {
        Serial.println("Message unchanged");
        return;
    }

    ftx_message_t testMessage;
    ftx_message_rc_t rc = ftx_message_encode(&testMessage, &hashIf, trimmed.c_str());
    if (rc != FTX_MESSAGE_RC_OK) {
        Serial.printf("Message parse failed, rc=%d\n", static_cast<int>(rc));
        return;
    }

    trimmed.toCharArray(txMessage, sizeof(txMessage));
    Serial.printf("Message set: %s\n", txMessage);
}

static String readSerialLine()
{
    static String line;
    while (Serial.available() > 0) {
        char ch = static_cast<char>(Serial.read());
        if (ch == '\r') {
            continue;
        }
        returnHomeForCommandInput();
        if (ch == '\n') {
            String out = line;
            line = "";
            out.trim();
            return out;
        }
        line += ch;
    }
    return String();
}

static String readKeyboardLine()
{
    if (!M5Cardputer.Keyboard.isChange() || !M5Cardputer.Keyboard.isPressed()) {
        return String();
    }

    Keyboard_Class::KeysState status = M5Cardputer.Keyboard.keysState();
    bool changed = false;
    bool hasCommandInput = status.enter || (status.del && keyboardCommand.length() > 0) ||
        (status.space && keyboardCommand.length() < kCommandMaxLength);
    if (!hasCommandInput) {
        for (char ch : status.word) {
            if (keyboardCommand.length() < kCommandMaxLength && ch >= 32 && ch <= 126) {
                hasCommandInput = true;
                break;
            }
        }
    }
    if (hasCommandInput) {
        returnHomeForCommandInput();
    }

    for (char ch : status.word) {
        if (keyboardCommand.length() < kCommandMaxLength && ch >= 32 && ch <= 126) {
            keyboardCommand += ch;
            changed = true;
        }
    }
    if (status.space && status.word.empty() && keyboardCommand.length() < kCommandMaxLength) {
        keyboardCommand += ' ';
        changed = true;
    }
    if (status.del && keyboardCommand.length() > 0) {
        keyboardCommand.remove(keyboardCommand.length() - 1);
        changed = true;
    }
    if (status.enter) {
        String out = keyboardCommand;
        out.trim();
        keyboardCommand = "";
        renderCommandLine();
        return out;
    }
    if (changed) {
        renderCommandLine();
    }
    return String();
}

static void printCommandHelp()
{
    Serial.println("CardFTx commands:");
    Serial.println("  set freq 1000");
    Serial.println("  set msg CQ BG6WRI ON80");
    Serial.println("  set SSID your_wifi_name");
    Serial.println("  set PASS your_wifi_password");
    Serial.println("  sync");
    Serial.println("  tx");
    Serial.println("  rx | rx once");
    Serial.println("  show");
}

static void handleCommand(const String& input)
{
    String line = input;
    line.trim();
    if (line.length() == 0) {
        return;
    }

    String lower = line;
    lower.toLowerCase();

    if (lower == "help" || lower == "?") {
        printCommandHelp();
    } else if (lower == "sync") {
        if (connectWifi(wifiSsid, wifiPassword)) {
            syncTime();
        }
    } else if (lower.startsWith("set msg ")) {
        setTxMessage(line.substring(8));
        renderStatus(true);
    } else if (lower.startsWith("set freq ")) {
        float value = line.substring(9).toFloat();
        if (value < 200.0f || value > 3000.0f) {
            Serial.println("Frequency must be 200..3000 Hz");
        } else {
            txToneHz = value;
            Serial.printf("TX audio frequency=%.1f Hz\n", txToneHz);
            renderStatus(true);
        }
    } else if (lower.startsWith("set ssid ")) {
        String value = line.substring(9);
        value.trim();
        value.toCharArray(wifiSsid, sizeof(wifiSsid));
        Serial.printf("WiFi SSID set: %s\n", wifiSsid);
        renderStatus(true);
    } else if (lower.startsWith("set pass ")) {
        String value = line.substring(9);
        value.trim();
        value.toCharArray(wifiPassword, sizeof(wifiPassword));
        Serial.println("WiFi password set");
        renderStatus(true);
    } else if (lower == "show") {
        char utcText[16];
        formatUtcTime(utcText, sizeof(utcText));
        Serial.printf("Message: %s\n", txMessage);
        Serial.printf("TX audio frequency: %.1f Hz\n", txToneHz);
        Serial.printf("WiFi SSID: %s\n", wifiSsid);
        Serial.printf("UTC: %s, synced=%s\n", utcText, timeIsSynced() ? "yes" : "no");
        Serial.printf("WiFi: %s\n", WiFi.status() == WL_CONNECTED ? "connected" : "disconnected");
        renderStatus(true);
    } else if (lower == "tx") {
        xSemaphoreTake(radioMutex, portMAX_DELAY);
        transmitFt8(txMessage, txToneHz, true);
        xSemaphoreGive(radioMutex);
    } else if (lower == "rx" || lower == "rx once") {
        xSemaphoreTake(radioMutex, portMAX_DELAY);
        receiveOnce();
        xSemaphoreGive(radioMutex);
    } else {
        Serial.println("Unknown command. Type: help");
    }
}

bool ft8AppBegin()
{
    Serial.printf("Free internal heap: %u bytes\n", heap_caps_get_free_size(MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT));
    Serial.printf("Free PSRAM heap: %u bytes\n", heap_caps_get_free_size(MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT));

    radioMutex = xSemaphoreCreateMutex();
    if (radioMutex == nullptr) {
        Serial.println("radio mutex allocation failed");
        return false;
    }

    Serial.println("TX audio uses M5Cardputer.Speaker");
    Serial.println("RX audio uses M5Cardputer.Mic");

    strncpy(wifiSsid, kDefaultWifiSsid, sizeof(wifiSsid) - 1);
    wifiSsid[sizeof(wifiSsid) - 1] = '\0';
    strncpy(wifiPassword, kDefaultWifiPassword, sizeof(wifiPassword) - 1);
    wifiPassword[sizeof(wifiPassword) - 1] = '\0';

    hashtableInit();
    renderStatus(true);

    if (wifiSsid[0] != '\0' && connectWifi(wifiSsid, wifiPassword)) {
        syncTime();
    }

    Serial.println("CardFTx ready.");
    printCommandHelp();
    return true;
}

void ft8AppLoop()
{
    String line = readSerialLine();
    if (line.length() == 0) {
        line = readKeyboardLine();
    }
    if (line.length() == 0) {
        if (!isTransmitting) {
            renderStatus();
        }
        delay(10);
        return;
    }

    handleCommand(line);
}
