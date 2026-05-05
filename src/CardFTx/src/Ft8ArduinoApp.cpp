#include "Ft8ArduinoApp.h"

#include "config.h"
#include "M5Cardputer.h"

#include <WiFi.h>
#include <esp_heap_caps.h>
#include <math.h>
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
static constexpr int kSpeakerChunkSamples = 512;
static constexpr int kSpeakerChannel = 0;
static constexpr int kFt8BlockSamples = static_cast<int>(kAudioSampleRate * FT8_SYMBOL_PERIOD);
static constexpr float kFt8SymbolBt = 2.0f;
static constexpr float kGfskConstK = 5.336446f;

static SemaphoreHandle_t radioMutex;
static char txMessage[FTX_MAX_MESSAGE_LENGTH] = "CQ TEST AB12";
static float txToneHz = kFt8DefaultToneHz;
static int16_t speakerBuffers[3][kSpeakerChunkSamples];
static uint8_t speakerBufferIndex = 0;
static bool isTransmitting = false;
static bool isReceiving = false;
static uint32_t lastDisplayMs = 0;
static uint32_t displayHoldUntilMs = 0;
static int16_t rxPcm[kFt8BlockSamples];
static float rxFrame[kFt8BlockSamples];
static int32_t rxPeak = 0;
static uint64_t rxAbsSum = 0;
static uint32_t rxSampleTotal = 0;
static bool micUseLeftChannel = true;
static uint8_t micMagnification = 64;

static struct {
    char callsign[12];
    uint32_t hash;
} callsignHashtable[256];

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
    int idx = (hash10 * 23) % 256;
    while (callsignHashtable[idx].callsign[0] != '\0') {
        if (((callsignHashtable[idx].hash & 0x3fffffu) == hash) &&
            strcmp(callsignHashtable[idx].callsign, callsign) == 0) {
            callsignHashtable[idx].hash &= 0x3fffffu;
            return;
        }
        idx = (idx + 1) % 256;
    }

    callsignHashtableSize++;
    strncpy(callsignHashtable[idx].callsign, callsign, 11);
    callsignHashtable[idx].callsign[11] = '\0';
    callsignHashtable[idx].hash = hash;
}

static bool hashtableLookup(ftx_callsign_hash_type_t hashType, uint32_t hash, char* callsign)
{
    uint8_t hashShift = (hashType == FTX_CALLSIGN_HASH_10_BITS) ? 12 :
        (hashType == FTX_CALLSIGN_HASH_12_BITS ? 10 : 0);
    uint16_t hash10 = (hash >> (12 - hashShift)) & 0x03ffu;
    int idx = (hash10 * 23) % 256;

    while (callsignHashtable[idx].callsign[0] != '\0') {
        if (((callsignHashtable[idx].hash & 0x3fffffu) >> hashShift) == hash) {
            strcpy(callsign, callsignHashtable[idx].callsign);
            return true;
        }
        idx = (idx + 1) % 256;
    }

    callsign[0] = '\0';
    return false;
}

static ftx_callsign_hash_interface_t hashIf = { hashtableLookup, hashtableAdd };

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

    M5Cardputer.Display.fillScreen(TFT_BLACK);
    M5Cardputer.Display.setTextColor(TFT_GREEN, TFT_BLACK);
    M5Cardputer.Display.setTextSize(1);
    M5Cardputer.Display.setCursor(4, 4);
    M5Cardputer.Display.printf("CardFTx FT8 TX\n");
    M5Cardputer.Display.setTextColor(TFT_WHITE, TFT_BLACK);
    M5Cardputer.Display.printf("UTC: %s\n", utcText);
    M5Cardputer.Display.printf("WiFi: %s\n", WiFi.status() == WL_CONNECTED ? "OK" : "OFF");
    M5Cardputer.Display.printf("Slot: ");
    if (waitSec < 0) {
        M5Cardputer.Display.printf("no sync\n");
    } else {
        M5Cardputer.Display.printf("%ds\n", waitSec);
    }
    M5Cardputer.Display.printf("Freq: %.0f Hz\n", txToneHz);
    M5Cardputer.Display.printf("Msg:\n%s\n", txMessage);
    M5Cardputer.Display.setTextColor(isTransmitting ? TFT_ORANGE : TFT_CYAN, TFT_BLACK);
    if (isTransmitting) {
        M5Cardputer.Display.printf("TX running\n");
    } else if (isReceiving) {
        M5Cardputer.Display.printf("RX running\n");
    } else {
        M5Cardputer.Display.printf("Idle\n");
    }
}

static bool connectWifi(const char* ssid, const char* password, uint32_t timeoutMs = 15000)
{
    if (ssid == nullptr || ssid[0] == '\0') {
        Serial.println("WiFi SSID is empty. Use: wifi SSID PASSWORD");
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
        Serial.println("Time is not synced. Use: wifi SSID PASSWORD, then sync");
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
        cfg.left_channel = micUseLeftChannel ? 1 : 0;
        cfg.stereo = 0;
        cfg.over_sampling = 1;
        cfg.noise_filter_level = 0;
        cfg.magnification = micMagnification;
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
    ftx_candidate_t candidateList[kMaxCandidates];
    int numCandidates = ftx_find_candidates(wf, kMaxCandidates, candidateList, kMinScore);
    ftx_message_t decoded[kMaxDecodedMessages];
    ftx_message_t* decodedHashtable[kMaxDecodedMessages] = {};
    int numDecoded = 0;

    Serial.printf("RX candidates: %d, max_mag=%.1f dB\n", numCandidates, monitor.max_mag);

    M5Cardputer.Display.fillScreen(TFT_BLACK);
    M5Cardputer.Display.setCursor(4, 4);
    M5Cardputer.Display.setTextColor(TFT_GREEN, TFT_BLACK);
    M5Cardputer.Display.printf("RX decode\n");
    M5Cardputer.Display.setTextColor(TFT_WHITE, TFT_BLACK);
    M5Cardputer.Display.printf("Cand: %d\n", numCandidates);

    for (int idx = 0; idx < numCandidates; ++idx) {
        const ftx_candidate_t* candidate = &candidateList[idx];
        ftx_message_t message;
        ftx_decode_status_t status;
        if (!ftx_decode_candidate(wf, candidate, kLdpcIterations, &message, &status)) {
            continue;
        }

        int hashIndex = message.hash % kMaxDecodedMessages;
        bool foundEmpty = false;
        bool foundDuplicate = false;
        do {
            if (decodedHashtable[hashIndex] == nullptr) {
                foundEmpty = true;
            } else if ((decodedHashtable[hashIndex]->hash == message.hash) &&
                memcmp(decodedHashtable[hashIndex]->payload, message.payload, sizeof(message.payload)) == 0) {
                foundDuplicate = true;
            } else {
                hashIndex = (hashIndex + 1) % kMaxDecodedMessages;
            }
        } while (!foundEmpty && !foundDuplicate);

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
        if (numDecoded <= 5) {
            M5Cardputer.Display.printf("%4.0fHz %s\n", freqHz, text);
        }
    }

    Serial.printf("RX decoded: %d, callsign hashes: %d\n", numDecoded, callsignHashtableSize);
    if (numDecoded == 0) {
        M5Cardputer.Display.printf("No decode\n");
    }
    displayHoldUntilMs = millis() + 15000;
    hashtableCleanup(10);
}

static bool receiveOnce()
{
    if (!timeIsSynced()) {
        Serial.println("Time is not synced. Use: wifi SSID PASSWORD, then sync");
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
    M5Cardputer.Display.setTextColor(TFT_RED, TFT_BLACK);
    M5Cardputer.Display.printf("RX capture\n");

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
    decodeWaterfall(monitor);
    monitor_free(&monitor);

    isReceiving = false;
    return true;
}

static void micTest(uint32_t durationMs = 2000)
{
    xSemaphoreTake(radioMutex, portMAX_DELAY);
    Serial.printf("Mic test: channel=%s magnification=%u\n",
        micUseLeftChannel ? "left" : "right", micMagnification);
    ensureMicReady();
    if (!M5Cardputer.Mic.isEnabled()) {
        Serial.println("Mic begin failed");
        xSemaphoreGive(radioMutex);
        return;
    }

    uint32_t start = millis();
    int32_t peak = 0;
    uint64_t absSum = 0;
    uint32_t total = 0;
    while (millis() - start < durationMs) {
        if (!M5Cardputer.Mic.record(rxPcm, kFt8BlockSamples, kAudioSampleRate, false)) {
            delay(1);
            continue;
        }
        for (int i = 0; i < kFt8BlockSamples; ++i) {
            int32_t mag = abs(static_cast<int32_t>(rxPcm[i]));
            if (mag > peak) {
                peak = mag;
            }
            absSum += mag;
        }
        total += kFt8BlockSamples;
    }

    stopMicAndRestoreSpeaker();
    Serial.printf("Mic test peak=%ld avg_abs=%lu samples=%lu\n",
        static_cast<long>(peak),
        static_cast<unsigned long>(total ? absSum / total : 0),
        static_cast<unsigned long>(total));
    xSemaphoreGive(radioMutex);
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
    Serial.printf("Playing at %.1f Hz\n", baseToneHz);
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
    Serial.println("Play done");
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

    hashtableInit();
    renderStatus(true);

    if (kDefaultWifiSsid[0] != '\0' && connectWifi(kDefaultWifiSsid, kDefaultWifiPassword)) {
        syncTime();
    }

    Serial.println("CardFTx ready. Commands: wifi SSID PASS | sync | msg CQ TEST AB12 | freq 1000 | play | playnow | tx MESSAGE | txnow MESSAGE | beep");
    return true;
}

void ft8AppLoop()
{
    String line = readSerialLine();
    if (line.length() == 0) {
        if (!isTransmitting) {
            renderStatus();
        }
        delay(10);
        return;
    }

    if (line.startsWith("wifi ")) {
        String args = line.substring(5);
        int split = args.indexOf(' ');
        if (split < 0) {
            Serial.println("Use: wifi SSID PASSWORD");
        } else {
            String ssid = args.substring(0, split);
            String password = args.substring(split + 1);
            password.trim();
            if (connectWifi(ssid.c_str(), password.c_str())) {
                syncTime();
            }
        }
    } else if (line == "sync") {
        syncTime();
    } else if (line.startsWith("msg ")) {
        setTxMessage(line.substring(4));
        renderStatus(true);
    } else if (line.startsWith("freq ")) {
        float value = line.substring(5).toFloat();
        if (value < 200.0f || value > 3000.0f) {
            Serial.println("Frequency must be 200..3000 Hz");
        } else {
            txToneHz = value;
            Serial.printf("TX audio frequency=%.1f Hz\n", txToneHz);
            renderStatus(true);
        }
    } else if (line == "show") {
        char utcText[16];
        formatUtcTime(utcText, sizeof(utcText));
        Serial.printf("Message: %s\n", txMessage);
        Serial.printf("TX audio frequency: %.1f Hz\n", txToneHz);
        Serial.printf("UTC: %s, synced=%s\n", utcText, timeIsSynced() ? "yes" : "no");
        Serial.printf("WiFi: %s\n", WiFi.status() == WL_CONNECTED ? "connected" : "disconnected");
        renderStatus(true);
    } else if (line == "beep") {
        xSemaphoreTake(radioMutex, portMAX_DELAY);
        playTestTone(1000.0f, 3000);
        xSemaphoreGive(radioMutex);
    } else if (line.startsWith("beep ")) {
        float value = line.substring(5).toFloat();
        if (value < 50.0f || value > 6000.0f) {
            Serial.println("Beep frequency must be 50..6000 Hz");
        } else {
            xSemaphoreTake(radioMutex, portMAX_DELAY);
            playTestTone(value, 3000);
            xSemaphoreGive(radioMutex);
        }
    } else if (line == "play") {
        xSemaphoreTake(radioMutex, portMAX_DELAY);
        transmitFt8(txMessage, txToneHz, true);
        xSemaphoreGive(radioMutex);
    } else if (line == "playnow") {
        xSemaphoreTake(radioMutex, portMAX_DELAY);
        transmitFt8(txMessage, txToneHz, false);
        xSemaphoreGive(radioMutex);
    } else if (line == "rxonce" || line == "mictest" || line == "esmic" ||
        line == "mic left" || line == "mic right" || line.startsWith("micgain ")) {
        Serial.println("RX/mic commands are disabled in this TX-only build.");
    } else if (line == "rx on") {
        Serial.println("RX is disabled in this TX-only build.");
    } else if (line == "rx off") {
        Serial.println("RX is disabled in this TX-only build.");
    } else if (line.startsWith("tx ")) {
        String message = line.substring(3);
        xSemaphoreTake(radioMutex, portMAX_DELAY);
        transmitFt8(message.c_str(), txToneHz, true);
        xSemaphoreGive(radioMutex);
    } else if (line.startsWith("txnow ")) {
        String message = line.substring(6);
        xSemaphoreTake(radioMutex, portMAX_DELAY);
        transmitFt8(message.c_str(), txToneHz, false);
        xSemaphoreGive(radioMutex);
    } else if (line.startsWith("vol ")) {
        uint8_t value = static_cast<uint8_t>(line.substring(4).toInt());
        M5Cardputer.Speaker.setVolume(value);
        Serial.printf("volume=%u\n", value);
    } else {
        Serial.println("Unknown command. Use: wifi SSID PASS | sync | msg MESSAGE | freq HZ | play | playnow | tx MESSAGE | txnow MESSAGE | beep");
    }
}
