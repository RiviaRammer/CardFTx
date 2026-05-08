#include "Ft8ArduinoApp.h"

#include "config.h"
#include "M5Cardputer.h"

#include <WiFi.h>
#include <ctype.h>
#include <esp_heap_caps.h>
#include <freertos/semphr.h>
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
static constexpr uint32_t kRadioTaskStackBytes = 24 * 1024;
static constexpr int kFt8BlockSamples = static_cast<int>(kAudioSampleRate * FT8_SYMBOL_PERIOD);
static constexpr uint8_t kDisplayTextSize = 2;
static constexpr uint8_t kDisplaySmallTextSize = 1;
static constexpr int kDisplayLineHeight = 18;
static constexpr int kCommandMaxLength = 63;
static constexpr float kFt8SymbolBt = 2.0f;
static constexpr float kGfskConstK = 5.336446f;
static constexpr int kDecodeHistorySize = 8;
static constexpr uint8_t kHidKeyEscape = 0x29;

enum class RadioPhase : uint8_t {
    Boot,
    WaitSync,
    WaitRxSlot,
    RxCapture,
    RxDecode,
    WaitTxSlot,
    TxTransmit,
    Error
};

enum class UiView : uint8_t {
    Home,
    TxSetting,
    Message,
    Waterfall
};

struct DecodeHistoryItem {
    char text[FTX_MAX_MESSAGE_LENGTH];
    float snr;
    float freqHz;
    bool slotOdd;
    bool transmitted;
    uint32_t utc;
};

static TaskHandle_t radioTaskHandle;
static SemaphoreHandle_t displayMutex;
static portMUX_TYPE appStateMux = portMUX_INITIALIZER_UNLOCKED;
static char txMessage[FTX_MAX_MESSAGE_LENGTH];
static float txToneHz = kFtxDefaultTxToneHz;
static bool txSlotOdd = false;
static int16_t speakerBuffers[3][kSpeakerChunkSamples];
static uint8_t speakerBufferIndex = 0;
static bool isTransmitting = false;
static bool isReceiving = false;
static bool txRequested = false;
static bool txCancelRequested = false;
static bool autoTxPending = false;
static char autoTxMessage[FTX_MAX_MESSAGE_LENGTH];
static char activeTxMessage[FTX_MAX_MESSAGE_LENGTH];
static char autoQsoCallsign[12];
static uint8_t autoQsoStage = 0;
static RadioPhase radioPhase = RadioPhase::Boot;
static UiView uiView = UiView::Home;
static int latestRxDecodedCount = 0;
static int unreadDecodedCount = 0;
static DecodeHistoryItem decodeHistory[kDecodeHistorySize];
static int decodeHistoryCount = 0;
static uint32_t lastDisplayMs = 0;
static uint32_t displayHoldUntilMs = 0;
static bool waterfallFrameDrawn = false;
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

static bool configTxSlotIsOdd()
{
    char slot[8];
    strncpy(slot, kFtxTxSlot, sizeof(slot) - 1);
    slot[sizeof(slot) - 1] = '\0';
    for (char* cursor = slot; *cursor != '\0'; ++cursor) {
        *cursor = static_cast<char>(toupper(static_cast<unsigned char>(*cursor)));
    }
    return strcmp(slot, "ODD") == 0;
}

static const char* ftxAutoModeText()
{
    switch (kFtxAutoMode) {
    case FtxAutoMode::AutoCQ:
        return "Auto CQ";
    case FtxAutoMode::AutoAnswer:
        return "Auto Answer";
    case FtxAutoMode::Manual:
    default:
        return "Manual";
    }
}

static bool ftxAutoEnabled()
{
    return kFtxAutoMode != FtxAutoMode::Manual;
}

static void formatStandardTxMessage(int index, char* out, size_t outSize)
{
    char reportText[8];
    snprintf(reportText, sizeof(reportText), "R%+03d", kFtxFallbackReportDb);

    switch (index) {
    case 0:
        snprintf(out, outSize, "CQ %s %s", kFtxStationCallsign, kFtxStationGrid);
        break;
    case 1:
        snprintf(out, outSize, "%s %s", kFtxStationCallsign, reportText);
        break;
    case 2:
        snprintf(out, outSize, "%s RR73", kFtxStationCallsign);
        break;
    default:
        snprintf(out, outSize, "%s 73", kFtxStationCallsign);
        break;
    }
}

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

static void formatUtcTimestamp(uint32_t timestamp, char* out, size_t outSize)
{
    if (timestamp == 0) {
        snprintf(out, outSize, "--:--:--");
        return;
    }

    time_t value = static_cast<time_t>(timestamp);
    struct tm utc;
    gmtime_r(&value, &utc);
    snprintf(out, outSize, "%02d:%02d:%02d", utc.tm_hour, utc.tm_min, utc.tm_sec);
}

static void logUtcEvent(const char* event)
{
    char utcText[16];
    formatUtcTime(utcText, sizeof(utcText));
    Serial.printf("[%s] %s\n", utcText, event);
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

static bool currentSlotIsOdd()
{
    if (!timeIsSynced()) {
        return false;
    }

    int sec = static_cast<int>(time(nullptr) % 60);
    return sec < 15 || (sec >= 30 && sec < 45);
}

static bool nextSlotIsOdd()
{
    if (!timeIsSynced()) {
        return false;
    }

    time_t nextSlot = ((time(nullptr) / 15) + 1) * 15;
    int sec = static_cast<int>(nextSlot % 60);
    return sec == 0 || sec == 30;
}

static int currentSlotElapsedSeconds()
{
    if (!timeIsSynced()) {
        return 0;
    }

    return static_cast<int>(time(nullptr) % 15);
}

static const char* phaseText(RadioPhase phase)
{
    switch (phase) {
    case RadioPhase::Boot:
        return "Boot";
    case RadioPhase::WaitSync:
        return "Sync";
    case RadioPhase::WaitRxSlot:
        return "Wait RX";
    case RadioPhase::RxCapture:
        return "RX";
    case RadioPhase::RxDecode:
        return "Decode";
    case RadioPhase::WaitTxSlot:
        return "Wait TX";
    case RadioPhase::TxTransmit:
        return "TX";
    case RadioPhase::Error:
        return "Error";
    }
    return "--";
}

static const char* displayPhaseText(RadioPhase phase)
{
    switch (phase) {
    case RadioPhase::WaitRxSlot:
    case RadioPhase::RxCapture:
    case RadioPhase::RxDecode:
    case RadioPhase::WaitTxSlot:
        return "RX";
    case RadioPhase::TxTransmit:
        return "TX";
    default:
        return phaseText(phase);
    }
}

static const char* nextActionText(RadioPhase phase, bool pendingTx)
{
    if (pendingTx && phase != RadioPhase::TxTransmit && phase != RadioPhase::WaitTxSlot) {
        return "TX next slot";
    }

    switch (phase) {
    case RadioPhase::WaitSync:
        return "Need UTC sync";
    case RadioPhase::WaitRxSlot:
    case RadioPhase::RxCapture:
    case RadioPhase::RxDecode:
        return pendingTx ? "TX next slot" : "RX next slot";
    case RadioPhase::WaitTxSlot:
        return "Return RX";
    case RadioPhase::TxTransmit:
        return "Return RX";
    case RadioPhase::Error:
        return "Check serial";
    default:
        return "RX";
    }
}

static const char* displayNextActionText(RadioPhase phase, bool txNext)
{
    if (phase == RadioPhase::TxTransmit) {
        return "RX";
    }

    if (phase == RadioPhase::WaitTxSlot) {
        return "TX";
    }

    if (txNext) {
        return "TX";
    }

    switch (phase) {
    case RadioPhase::WaitSync:
        return "UTC";
    case RadioPhase::WaitRxSlot:
    case RadioPhase::RxCapture:
    case RadioPhase::RxDecode:
        return "RX";
    case RadioPhase::Error:
        return "Serial";
    default:
        return "RX";
    }
}

static void setRadioPhase(RadioPhase phase)
{
    portENTER_CRITICAL(&appStateMux);
    radioPhase = phase;
    isReceiving = phase == RadioPhase::WaitRxSlot || phase == RadioPhase::RxCapture || phase == RadioPhase::RxDecode;
    isTransmitting = phase == RadioPhase::WaitTxSlot || phase == RadioPhase::TxTransmit;
    portEXIT_CRITICAL(&appStateMux);
}

static void addMessageHistory(const char* text, float snr, float freqHz, bool slotOdd, uint32_t slotEndUtc, bool transmitted)
{
    portENTER_CRITICAL(&appStateMux);
    for (int i = kDecodeHistorySize - 1; i > 0; --i) {
        decodeHistory[i] = decodeHistory[i - 1];
    }
    strncpy(decodeHistory[0].text, text, sizeof(decodeHistory[0].text) - 1);
    decodeHistory[0].text[sizeof(decodeHistory[0].text) - 1] = '\0';
    decodeHistory[0].snr = snr;
    decodeHistory[0].freqHz = freqHz;
    decodeHistory[0].slotOdd = slotOdd;
    decodeHistory[0].transmitted = transmitted;
    decodeHistory[0].utc = slotEndUtc;
    if (decodeHistoryCount < kDecodeHistorySize) {
        decodeHistoryCount++;
    }
    if (!transmitted) {
        latestRxDecodedCount++;
        unreadDecodedCount++;
    }
    portEXIT_CRITICAL(&appStateMux);
}

static void addDecodeHistory(const char* text, float snr, float freqHz, bool slotOdd, uint32_t slotEndUtc)
{
    addMessageHistory(text, snr, freqHz, slotOdd, slotEndUtc, false);
}

static void addTransmitHistory(const char* text, bool slotOdd, uint32_t slotEndUtc)
{
    addMessageHistory(text, 0.0f, txToneHz, slotOdd, slotEndUtc, true);
}

static void queueAutoTx(const char* text, const char* callsign, uint8_t stage)
{
    portENTER_CRITICAL(&appStateMux);
    strncpy(autoTxMessage, text, sizeof(autoTxMessage) - 1);
    autoTxMessage[sizeof(autoTxMessage) - 1] = '\0';
    if (callsign != nullptr) {
        strncpy(autoQsoCallsign, callsign, sizeof(autoQsoCallsign) - 1);
        autoQsoCallsign[sizeof(autoQsoCallsign) - 1] = '\0';
    }
    autoQsoStage = stage;
    autoTxPending = true;
    portEXIT_CRITICAL(&appStateMux);
    Serial.printf("Auto TX queued: %s\n", text);
}

static bool isReportToken(const char* token, bool requireR)
{
    if (token == nullptr || token[0] == '\0') {
        return false;
    }

    int index = 0;
    if (token[index] == 'R') {
        if (!requireR) {
            return false;
        }
        index++;
    } else if (requireR) {
        return false;
    }

    if (token[index] != '+' && token[index] != '-') {
        return false;
    }
    index++;
    return token[index] >= '0' && token[index] <= '9' && token[index + 1] >= '0' && token[index + 1] <= '9' &&
        token[index + 2] == '\0';
}

static int clampReportDb(float snr)
{
    int report = static_cast<int>(roundf(snr));
    if (report < -50 || report > 49) {
        report = kFtxFallbackReportDb;
    }
    return max(-50, min(49, report));
}

static void formatSignalReport(char* out, size_t outSize, float snr, bool roger)
{
    int report = clampReportDb(snr);
    snprintf(out, outSize, roger ? "R%+03d" : "%+03d", report);
}

static void maybeQueueAutoReply(const char* decodedText, float snr)
{
    if (!ftxAutoEnabled() || decodedText == nullptr || decodedText[0] == '\0') {
        if (kFtxAutoMode == FtxAutoMode::Manual) {
            Serial.println("Auto ignored: Manual mode");
        }
        return;
    }

    char text[FTX_MAX_MESSAGE_LENGTH];
    strncpy(text, decodedText, sizeof(text) - 1);
    text[sizeof(text) - 1] = '\0';
    for (char* cursor = text; *cursor != '\0'; ++cursor) {
        *cursor = static_cast<char>(toupper(static_cast<unsigned char>(*cursor)));
    }

    char* tokens[4] = {};
    int tokenCount = 0;
    char* token = strtok(text, " ");
    while (token != nullptr && tokenCount < 4) {
        tokens[tokenCount++] = token;
        token = strtok(nullptr, " ");
    }
    if (tokenCount == 0) {
        Serial.println("Auto ignored: empty tokens");
        return;
    }

    char reply[FTX_MAX_MESSAGE_LENGTH];
    char otherCall[12] = {};
    char reportText[8];
    const char* myCall = kFtxStationCallsign;
    char myCallCompare[12];
    strncpy(myCallCompare, kFtxStationCallsign, sizeof(myCallCompare) - 1);
    myCallCompare[sizeof(myCallCompare) - 1] = '\0';
    for (char* cursor = myCallCompare; *cursor != '\0'; ++cursor) {
        *cursor = static_cast<char>(toupper(static_cast<unsigned char>(*cursor)));
    }

    if (kFtxAutoMode == FtxAutoMode::AutoAnswer && tokenCount >= 2 && strcmp(tokens[0], "CQ") == 0) {
        strncpy(otherCall, tokens[1], sizeof(otherCall) - 1);
        snprintf(reply, sizeof(reply), "%s %s %s", otherCall, myCall, kFtxStationGrid);
        queueAutoTx(reply, otherCall, 1);
        return;
    }

    if (tokenCount < 3) {
        Serial.println("Auto ignored: not enough tokens");
        return;
    }

    const bool toMe = strcmp(tokens[0], myCallCompare) == 0;
    const bool fromMe = strcmp(tokens[1], myCallCompare) == 0;
    if (!toMe && !fromMe) {
        Serial.println("Auto ignored: not addressed to me");
        return;
    }

    const char* peer = toMe ? tokens[1] : tokens[0];
    strncpy(otherCall, peer, sizeof(otherCall) - 1);

    if (toMe && kFtxAutoMode == FtxAutoMode::AutoCQ && !isReportToken(tokens[2], false) &&
        !isReportToken(tokens[2], true) && strcmp(tokens[2], "RRR") != 0 && strcmp(tokens[2], "RR73") != 0 &&
        strcmp(tokens[2], "73") != 0) {
        formatSignalReport(reportText, sizeof(reportText), snr, false);
        snprintf(reply, sizeof(reply), "%s %s %s", otherCall, myCall, reportText);
        queueAutoTx(reply, otherCall, 1);
    } else if (toMe && isReportToken(tokens[2], false)) {
        formatSignalReport(reportText, sizeof(reportText), snr, true);
        snprintf(reply, sizeof(reply), "%s %s %s", otherCall, myCall, reportText);
        queueAutoTx(reply, otherCall, 2);
    } else if (toMe && isReportToken(tokens[2], true)) {
        snprintf(reply, sizeof(reply), "%s %s RRR", otherCall, myCall);
        queueAutoTx(reply, otherCall, 3);
    } else if (toMe && (strcmp(tokens[2], "RRR") == 0 || strcmp(tokens[2], "RR73") == 0)) {
        snprintf(reply, sizeof(reply), "%s %s 73", otherCall, myCall);
        queueAutoTx(reply, otherCall, 4);
    } else if (toMe && strcmp(tokens[2], "73") == 0) {
        portENTER_CRITICAL(&appStateMux);
        autoTxPending = false;
        autoQsoStage = 0;
        autoQsoCallsign[0] = '\0';
        portEXIT_CRITICAL(&appStateMux);
    }
}

static void setUiView(UiView view)
{
    portENTER_CRITICAL(&appStateMux);
    uiView = view;
    if (view == UiView::Message) {
        unreadDecodedCount = 0;
    }
    portEXIT_CRITICAL(&appStateMux);
    displayHoldUntilMs = 0;
    lastDisplayMs = 0;
    waterfallFrameDrawn = false;
}

static void toggleUiView()
{
    UiView view;
    portENTER_CRITICAL(&appStateMux);
    switch (uiView) {
    case UiView::Home:
        view = UiView::TxSetting;
        break;
    case UiView::TxSetting:
        view = UiView::Message;
        break;
    case UiView::Message:
        view = UiView::Waterfall;
        break;
    default:
        view = UiView::Home;
        break;
    }
    uiView = view;
    if (view == UiView::Message) {
        unreadDecodedCount = 0;
    }
    portEXIT_CRITICAL(&appStateMux);
    displayHoldUntilMs = 0;
    lastDisplayMs = 0;
    waterfallFrameDrawn = false;
}

static void renderStatus(bool force);

static bool isUiView(UiView view)
{
    bool matches;
    portENTER_CRITICAL(&appStateMux);
    matches = uiView == view;
    portEXIT_CRITICAL(&appStateMux);
    return matches;
}

static bool currentViewShowsCommandLine()
{
    bool shows;
    portENTER_CRITICAL(&appStateMux);
    shows = uiView != UiView::Message && uiView != UiView::Waterfall;
    portEXIT_CRITICAL(&appStateMux);
    return shows;
}

static bool isTxCancelRequested()
{
    bool cancel;
    portENTER_CRITICAL(&appStateMux);
    cancel = txCancelRequested;
    portEXIT_CRITICAL(&appStateMux);
    return cancel;
}

static bool isWaterfallCaptureVisible()
{
    bool visible;
    portENTER_CRITICAL(&appStateMux);
    visible = uiView == UiView::Waterfall && radioPhase == RadioPhase::RxCapture;
    portEXIT_CRITICAL(&appStateMux);
    return visible;
}

static void clearTxCancelRequest()
{
    portENTER_CRITICAL(&appStateMux);
    txCancelRequested = false;
    portEXIT_CRITICAL(&appStateMux);
}

static void setActiveTxMessage(const char* text)
{
    portENTER_CRITICAL(&appStateMux);
    if (text == nullptr) {
        activeTxMessage[0] = '\0';
    } else {
        strncpy(activeTxMessage, text, sizeof(activeTxMessage) - 1);
        activeTxMessage[sizeof(activeTxMessage) - 1] = '\0';
    }
    portEXIT_CRITICAL(&appStateMux);
}

static void requestTxCancel()
{
    bool wasPending;
    bool wasTransmitting;
    portENTER_CRITICAL(&appStateMux);
    wasPending = txRequested;
    wasTransmitting = radioPhase == RadioPhase::WaitTxSlot || radioPhase == RadioPhase::TxTransmit;
    txRequested = false;
    txCancelRequested = true;
    portEXIT_CRITICAL(&appStateMux);

    if (wasPending || wasTransmitting) {
        M5Cardputer.Speaker.stop();
        if (radioTaskHandle != nullptr) {
            xTaskNotifyGive(radioTaskHandle);
        }
        Serial.println("TX cancelled");
    } else {
        Serial.println("No TX to cancel");
    }
    renderStatus(true);
}

static void lockDisplay()
{
    if (displayMutex != nullptr) {
        xSemaphoreTake(displayMutex, portMAX_DELAY);
    }
}

static void unlockDisplay()
{
    if (displayMutex != nullptr) {
        xSemaphoreGive(displayMutex);
    }
}

static void renderCommandLineUnlocked()
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

static void renderCommandLine()
{
    lockDisplay();
    unlockDisplay();
}

static void drawRightText(const char* text, int y, uint8_t textSize, uint16_t color, int minX = 4)
{
    int screenW = M5Cardputer.Display.width();
    int textW = static_cast<int>(strlen(text)) * 6 * textSize;
    int x = max(minX, screenW - textW - 4);
    M5Cardputer.Display.setTextSize(textSize);
    M5Cardputer.Display.setTextColor(color, TFT_BLACK);
    M5Cardputer.Display.setCursor(x, y);
    M5Cardputer.Display.printf("%s", text);
}

static void drawSlotDivider(uint32_t slotEndUtc, int y)
{
    char timeText[16];
    formatUtcTimestamp(slotEndUtc, timeText, sizeof(timeText));

    char label[24];
    snprintf(label, sizeof(label), "%s end", timeText);

    int screenW = M5Cardputer.Display.width();
    int labelW = static_cast<int>(strlen(label)) * 6 * kDisplaySmallTextSize;
    int arrowW = 7;
    int labelX = max(4, screenW - labelW - arrowW - 6);
    int lineEndX = max(4, labelX - 4);

    M5Cardputer.Display.setTextSize(kDisplaySmallTextSize);
    M5Cardputer.Display.setTextColor(TFT_DARKGREY, TFT_BLACK);
    if (lineEndX > 4) {
        M5Cardputer.Display.drawFastHLine(4, y + 4, lineEndX - 4, TFT_DARKGREY);
    }
    M5Cardputer.Display.setCursor(labelX, y);
    M5Cardputer.Display.printf("%s", label);

    int arrowX = labelX + labelW + 4;
    int arrowY = y + 2;
    M5Cardputer.Display.drawLine(arrowX, arrowY, arrowX + 3, arrowY + 4, TFT_DARKGREY);
    M5Cardputer.Display.drawLine(arrowX + 6, arrowY, arrowX + 3, arrowY + 4, TFT_DARKGREY);
}

static void renderStatus(bool force = false)
{
    uint32_t nowMs = millis();
    if (!force && (nowMs - lastDisplayMs < 1000)) {
        return;
    }
    lastDisplayMs = nowMs;

    RadioPhase phase;
    UiView view;
    bool pendingTx;
    bool autoPending;
    int unreadCount;
    int historyCount;
    int decodedCount;
    DecodeHistoryItem history[kDecodeHistorySize];
    float toneHz;
    bool configuredTxSlotOdd;
    uint8_t qsoStage;
    char txPreview[FTX_MAX_MESSAGE_LENGTH];

    portENTER_CRITICAL(&appStateMux);
    phase = radioPhase;
    view = uiView;
    pendingTx = txRequested;
    autoPending = autoTxPending;
    unreadCount = unreadDecodedCount;
    historyCount = decodeHistoryCount;
    decodedCount = latestRxDecodedCount;
    memcpy(history, decodeHistory, sizeof(history));
    toneHz = txToneHz;
    configuredTxSlotOdd = txSlotOdd;
    qsoStage = autoQsoStage;
    txPreview[0] = '\0';
    if (phase == RadioPhase::WaitTxSlot || phase == RadioPhase::TxTransmit) {
        strncpy(txPreview, activeTxMessage, sizeof(txPreview) - 1);
        txPreview[sizeof(txPreview) - 1] = '\0';
    }
    portEXIT_CRITICAL(&appStateMux);

    char utcText[16];
    formatUtcTime(utcText, sizeof(utcText));
    int waitSec = secondsToNextFt8Slot();
    bool nextOdd = nextSlotIsOdd();
    bool txNext = pendingTx || autoPending ||
        (kFtxAutoMode == FtxAutoMode::AutoCQ && qsoStage == 0 && nextOdd == configuredTxSlotOdd);
    const char* statusText = displayPhaseText(phase);
    char transitionText[32];
    snprintf(transitionText, sizeof(transitionText), "%s->%s", statusText, displayNextActionText(phase, txNext));

    lockDisplay();
    int screenW = M5Cardputer.Display.width();
    int statusX = screenW - static_cast<int>(strlen(statusText)) * 12 - 4;

    M5Cardputer.Display.fillScreen(TFT_BLACK);

    if (view == UiView::Message) {
        M5Cardputer.Display.setTextSize(kDisplaySmallTextSize);
        int y = 4;
        uint32_t lastSlotEnd = 0;
        for (int i = 0; i < historyCount && y < M5Cardputer.Display.height() - 4; ++i) {
            if (i == 0 || history[i].utc != lastSlotEnd) {
                drawSlotDivider(history[i].utc, y);
                y += 10;
                lastSlotEnd = history[i].utc;
            }

            char shortText[28];
            strncpy(shortText, history[i].text, sizeof(shortText) - 1);
            shortText[sizeof(shortText) - 1] = '\0';
            M5Cardputer.Display.setCursor(4, y);
            M5Cardputer.Display.setTextColor(history[i].transmitted ? TFT_RED : (i == 0 ? TFT_CYAN : TFT_WHITE),
                TFT_BLACK);
            if (history[i].transmitted) {
                M5Cardputer.Display.printf("TX       %s", shortText);
            } else {
                M5Cardputer.Display.printf("%4.0f %+.0f %s", history[i].freqHz, history[i].snr, shortText);
            }
            y += 11;
        }
        if (historyCount == 0) {
            M5Cardputer.Display.setCursor(4, 4);
            M5Cardputer.Display.setTextColor(TFT_DARKGREY, TFT_BLACK);
            M5Cardputer.Display.printf("No RX history");
        }
        unlockDisplay();
        return;
    }

    if (view == UiView::TxSetting) {
        M5Cardputer.Display.setTextSize(kDisplayTextSize);
        M5Cardputer.Display.setTextColor(TFT_CYAN, TFT_BLACK);
        M5Cardputer.Display.setCursor(4, 4);
        M5Cardputer.Display.printf("TX Settying");
        drawRightText(ftxAutoModeText(), 9, kDisplaySmallTextSize, TFT_WHITE, 104);
        M5Cardputer.Display.drawFastHLine(4, 27, screenW - 8, TFT_DARKGREY);

        M5Cardputer.Display.setTextSize(kDisplaySmallTextSize);
        M5Cardputer.Display.setTextColor(TFT_WHITE, TFT_BLACK);
        M5Cardputer.Display.setCursor(4, 35);
        M5Cardputer.Display.printf("%.0fHz | %s", toneHz, configuredTxSlotOdd ? "Odd" : "Even");

        M5Cardputer.Display.drawFastHLine(4, 50, screenW - 8, TFT_DARKGREY);
        M5Cardputer.Display.setTextColor(TFT_DARKGREY, TFT_BLACK);
        M5Cardputer.Display.setCursor(4, 58);
        M5Cardputer.Display.printf("Preset standard messages");
        char presetText[FTX_MAX_MESSAGE_LENGTH];
        M5Cardputer.Display.setTextColor(TFT_WHITE, TFT_BLACK);
        formatStandardTxMessage(0, presetText, sizeof(presetText));
        M5Cardputer.Display.setCursor(4, 72);
        M5Cardputer.Display.printf("%s", presetText);
        formatStandardTxMessage(1, presetText, sizeof(presetText));
        M5Cardputer.Display.setCursor(4, 84);
        M5Cardputer.Display.printf("%s", presetText);
        formatStandardTxMessage(2, presetText, sizeof(presetText));
        M5Cardputer.Display.setCursor(4, 96);
        M5Cardputer.Display.printf("%s", presetText);
        renderCommandLineUnlocked();
        unlockDisplay();
        return;
    }

    M5Cardputer.Display.setTextColor(TFT_GREEN, TFT_BLACK);
    M5Cardputer.Display.setTextSize(kDisplayTextSize);
    M5Cardputer.Display.setCursor(4, 4);
    M5Cardputer.Display.printf("FT8");
    M5Cardputer.Display.setTextColor(strcmp(statusText, "TX") == 0 ? TFT_RED :
        (strcmp(statusText, "RX") == 0 ? TFT_GREEN : TFT_WHITE), TFT_BLACK);
    M5Cardputer.Display.setCursor(max(52, statusX), 4);
    M5Cardputer.Display.printf("%s", statusText);
    M5Cardputer.Display.drawFastHLine(4, 23, screenW - 8, TFT_DARKGREY);

    M5Cardputer.Display.setTextSize(kDisplayTextSize);
    M5Cardputer.Display.setTextColor(TFT_WHITE, TFT_BLACK);
    M5Cardputer.Display.setCursor(4, 29);
    M5Cardputer.Display.printf("UTC %s", utcText);
    drawRightText(transitionText, 34, kDisplaySmallTextSize, TFT_DARKGREY, 150);

    char slotText[16];
    if (waitSec < 0) {
        snprintf(slotText, sizeof(slotText), "Slot --");
    } else {
        snprintf(slotText, sizeof(slotText), "%s %02ds", currentSlotIsOdd() ? "Odd" : "Even", waitSec);
    }

    M5Cardputer.Display.setTextSize(kDisplaySmallTextSize);
    M5Cardputer.Display.setTextColor(TFT_WHITE, TFT_BLACK);
    int slotTextW = static_cast<int>(strlen(slotText)) * 6 * kDisplaySmallTextSize;
    M5Cardputer.Display.setCursor(max(4, (screenW - slotTextW) / 2), 48);
    M5Cardputer.Display.printf("%s", slotText);

    M5Cardputer.Display.drawFastHLine(4, 60, screenW - 8, TFT_DARKGREY);

    if (view == UiView::Waterfall) {
        M5Cardputer.Display.setTextSize(kDisplayTextSize);
        M5Cardputer.Display.setTextColor(TFT_CYAN, TFT_BLACK);
        M5Cardputer.Display.setCursor(4, 64);
        M5Cardputer.Display.printf("Waterfall");
        M5Cardputer.Display.setTextSize(kDisplaySmallTextSize);
        M5Cardputer.Display.setTextColor(TFT_DARKGREY, TFT_BLACK);
        M5Cardputer.Display.setCursor(4, 88);
        if (phase == RadioPhase::RxCapture) {
            M5Cardputer.Display.printf("RX capture starting");
        } else {
            M5Cardputer.Display.printf("Shows during RX capture");
        }
    } else {
        M5Cardputer.Display.setTextSize(kDisplayTextSize);
        M5Cardputer.Display.setTextColor(TFT_WHITE, TFT_BLACK);
        M5Cardputer.Display.setCursor(4, 64);
        if (txPreview[0] != '\0') {
            char shortTx[17];
            strncpy(shortTx, txPreview, sizeof(shortTx) - 1);
            shortTx[sizeof(shortTx) - 1] = '\0';
            M5Cardputer.Display.printf("TX:%s", shortTx);
        } else {
            M5Cardputer.Display.printf("TX:");
        }
        M5Cardputer.Display.drawFastHLine(4, 88, screenW - 8, TFT_DARKGREY);
        M5Cardputer.Display.setTextSize(kDisplayTextSize);
        M5Cardputer.Display.setCursor(4, 97);
        if (unreadCount > 0) {
            M5Cardputer.Display.setTextColor(TFT_CYAN, TFT_BLACK);
            M5Cardputer.Display.printf("RX new %d total %d", unreadCount, decodedCount);
        } else if (historyCount > 0) {
            M5Cardputer.Display.setTextColor(TFT_DARKGREY, TFT_BLACK);
            M5Cardputer.Display.printf("RX received %d", decodedCount);
        } else {
            M5Cardputer.Display.setTextColor(TFT_DARKGREY, TFT_BLACK);
            M5Cardputer.Display.printf("RX listening");
        }
    }
    if (view != UiView::Message && view != UiView::Waterfall) {
        renderCommandLineUnlocked();
    }
    unlockDisplay();
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
    if (monitor.wf.num_blocks == 0 || monitor.wf.mag == nullptr) {
        return;
    }

    lockDisplay();
    auto& display = M5Cardputer.Display;
    int screenW = display.width();
    int screenH = display.height();
    const int left = 6;
    const int top = 32;
    const int right = screenW - 5;
    const int bottom = screenH - 36;
    const int plotW = max(1, right - left + 1);
    const int plotH = max(1, bottom - top + 1);

    if (!waterfallFrameDrawn || monitor.wf.num_blocks == 1) {
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
        waterfallFrameDrawn = true;
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
    unlockDisplay();
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

static bool waitForFt8Slot(bool abortOnTxRequest = false)
{
    if (!timeIsSynced()) {
        Serial.println("Time is not synced. Use: set SSID ..., set PASS ..., then sync");
        setRadioPhase(RadioPhase::WaitSync);
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
        if (!abortOnTxRequest && isTxCancelRequested()) {
            return false;
        }
        if (abortOnTxRequest) {
            bool pendingTx;
            portENTER_CRITICAL(&appStateMux);
            pendingTx = txRequested;
            portEXIT_CRITICAL(&appStateMux);
            if (pendingTx) {
                return false;
            }
        }

        if (remainingUs > 300000) {
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

static void decodeWaterfall(const monitor_t& monitor, bool slotOdd, uint32_t slotEndUtc)
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
        free(decodedHashtable);
        free(decoded);
        free(candidateList);
        return;
    }

    int numCandidates = ftx_find_candidates(wf, kMaxCandidates, candidateList, kMinScore);
    int numDecoded = 0;

    Serial.printf("RX candidates: %d, max_mag=%.1f dB\n", numCandidates, monitor.max_mag);
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
        addDecodeHistory(text, snr, freqHz, slotOdd, slotEndUtc);
        maybeQueueAutoReply(text, snr);
    }

    Serial.printf("RX decoded: %d, callsign hashes: %d\n", numDecoded, callsignHashtableSize);
    hashtableCleanup(10);

    free(decodedHashtable);
    free(decoded);
    free(candidateList);
}

static void runDecodeWaterfall(const monitor_t& monitor, bool slotOdd, uint32_t slotEndUtc)
{
    decodeWaterfall(monitor, slotOdd, slotEndUtc);
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
        setRadioPhase(RadioPhase::Error);
        return false;
    }

    Serial.printf("RX monitor ready: block=%d, bins=%d, heap=%u\n",
        monitor.block_size, monitor.wf.num_bins,
        heap_caps_get_free_size(MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT));

    setRadioPhase(RadioPhase::WaitRxSlot);

    ensureMicReady();
    if (!M5Cardputer.Mic.isEnabled()) {
        Serial.println("Mic begin failed");
        monitor_free(&monitor);
        stopMicAndRestoreSpeaker();
        setRadioPhase(RadioPhase::Error);
        return false;
    }

    if (!waitForFt8Slot(true)) {
        stopMicAndRestoreSpeaker();
        monitor_free(&monitor);
        setRadioPhase(timeIsSynced() ? RadioPhase::WaitTxSlot : RadioPhase::WaitSync);
        return false;
    }
    setRadioPhase(RadioPhase::RxCapture);
    bool captureSlotOdd = currentSlotIsOdd();
    uint32_t captureSlotEndUtc = static_cast<uint32_t>(((time(nullptr) / 15) + 1) * 15);

    logUtcEvent("RX capture start");
    rxPeak = 0;
    rxAbsSum = 0;
    rxSampleTotal = 0;

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
        if (isUiView(UiView::Waterfall)) {
            renderRxWaterfall(monitor);
        }
        if ((monitor.wf.num_blocks % 10) == 0) {
            Serial.print(".");
        }
    }
    Serial.println();
    logUtcEvent("RX capture done");
    Serial.printf("RX audio peak=%ld avg_abs=%lu samples=%lu\n",
        static_cast<long>(rxPeak),
        static_cast<unsigned long>(rxSampleTotal ? rxAbsSum / rxSampleTotal : 0),
        static_cast<unsigned long>(rxSampleTotal));
    stopMicAndRestoreSpeaker();
    setRadioPhase(RadioPhase::RxDecode);
    logUtcEvent("RX decode start");
    runDecodeWaterfall(monitor, captureSlotOdd, captureSlotEndUtc);
    logUtcEvent("RX decode done");
    monitor_free(&monitor);

    setRadioPhase(RadioPhase::WaitRxSlot);
    logUtcEvent("RX wait next slot");
    return true;
}

static bool waitForConfiguredTxSlot(bool abortOnTxRequest = false)
{
    while (true) {
        if (!timeIsSynced()) {
            Serial.println("Time is not synced. Use: set SSID ..., set PASS ..., then sync");
            setRadioPhase(RadioPhase::WaitSync);
            return false;
        }

        if (nextSlotIsOdd() == txSlotOdd) {
            return waitForFt8Slot(abortOnTxRequest);
        }

        Serial.printf("Waiting for %s TX slot\n", txSlotOdd ? "Odd" : "Even");
        if (!waitForFt8Slot(abortOnTxRequest)) {
            return false;
        }
    }
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

static bool writeSilenceSamples(int sampleCount, bool abortOnTxCancel = false)
{
    int16_t silence[kSpeakerChunkSamples] = {};
    bool first = false;
    while (sampleCount > 0) {
        if (abortOnTxCancel && isTxCancelRequested()) {
            return false;
        }
        int count = min(sampleCount, kSpeakerChunkSamples);
        speakerPlayChunk(silence, count, first);
        first = false;
        sampleCount -= count;
    }
    return true;
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

static bool transmitFt8(const char* text, float baseToneHz, bool waitForSlot, bool configuredSlotOnly = false)
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

    setActiveTxMessage(text);
    setRadioPhase(waitForSlot ? RadioPhase::WaitTxSlot : RadioPhase::TxTransmit);
    if (waitForSlot) {
        logUtcEvent("TX wait slot");
    }

    if (waitForSlot && !(configuredSlotOnly ? waitForConfiguredTxSlot() : waitForFt8Slot())) {
        if (isTxCancelRequested()) {
            Serial.println("TX cancelled before slot");
            clearTxCancelRequest();
            setRadioPhase(RadioPhase::WaitRxSlot);
        } else {
            setRadioPhase(RadioPhase::WaitSync);
        }
        setActiveTxMessage(nullptr);
        return false;
    }

    setRadioPhase(RadioPhase::TxTransmit);
    logUtcEvent("TX start");
    addTransmitHistory(text, currentSlotIsOdd(), static_cast<uint32_t>(((time(nullptr) / 15) + 1) * 15));
    Serial.printf("TX audio at %.1f Hz\n", baseToneHz);
    M5Cardputer.Speaker.stop();
    if (!writeSilenceSamples(silenceSamples, true)) {
        M5Cardputer.Speaker.stop();
        M5Cardputer.Speaker.end();
        clearTxCancelRequest();
        setRadioPhase(RadioPhase::WaitRxSlot);
        setActiveTxMessage(nullptr);
        return false;
    }

    int16_t chunk[kSpeakerChunkSamples];
    float phase = 0.0f;
    int sampleIndex = 0;
    while (sampleIndex < waveSamples) {
        if (isTxCancelRequested()) {
            M5Cardputer.Speaker.stop();
            M5Cardputer.Speaker.end();
            clearTxCancelRequest();
            setRadioPhase(RadioPhase::WaitRxSlot);
            setActiveTxMessage(nullptr);
            return false;
        }
        int count = min(kSpeakerChunkSamples, waveSamples - sampleIndex);
        for (int i = 0; i < count; ++i) {
            chunk[i] = synthSample(tones, sampleIndex + i, baseToneHz, phase);
        }

        speakerPlayChunk(chunk, count);
        sampleIndex += count;
    }

    if (!writeSilenceSamples(silenceSamples, true)) {
        M5Cardputer.Speaker.stop();
        M5Cardputer.Speaker.end();
        clearTxCancelRequest();
        setRadioPhase(RadioPhase::WaitRxSlot);
        setActiveTxMessage(nullptr);
        return false;
    }
    while (M5Cardputer.Speaker.isPlaying(kSpeakerChannel)) {
        if (isTxCancelRequested()) {
            M5Cardputer.Speaker.stop();
            M5Cardputer.Speaker.end();
            clearTxCancelRequest();
            setRadioPhase(RadioPhase::WaitRxSlot);
            setActiveTxMessage(nullptr);
            return false;
        }
        delay(1);
    }
    Serial.println("TX done");
    logUtcEvent("TX done");
    M5Cardputer.Speaker.end();
    clearTxCancelRequest();
    setRadioPhase(RadioPhase::WaitRxSlot);
    setActiveTxMessage(nullptr);
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

    char messageCopy[FTX_MAX_MESSAGE_LENGTH];
    trimmed.toCharArray(messageCopy, sizeof(messageCopy));
    portENTER_CRITICAL(&appStateMux);
    strncpy(txMessage, messageCopy, sizeof(txMessage) - 1);
    txMessage[sizeof(txMessage) - 1] = '\0';
    portEXIT_CRITICAL(&appStateMux);
    Serial.printf("Message set: %s\n", messageCopy);
}

static String readSerialLine()
{
    static String line;
    while (Serial.available() > 0) {
        char ch = static_cast<char>(Serial.read());
        if (ch == '\r') {
            continue;
        }
        if (ch == 27) {
            line = "";
            requestTxCancel();
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
    bool escapePressed = false;
    for (uint8_t hid : status.hid_keys) {
        if (hid == kHidKeyEscape) {
            escapePressed = true;
            break;
        }
    }
    for (char ch : status.word) {
        if (ch == 27 || (status.fn && ch == '`')) {
            escapePressed = true;
            break;
        }
    }
    if (escapePressed) {
        keyboardCommand = "";
        requestTxCancel();
        if (currentViewShowsCommandLine()) {
            renderCommandLine();
        }
        return String();
    }

    bool changed = false;
    if (keyboardCommand.length() == 0) {
        for (char ch : status.word) {
            if (ch == '/') {
                toggleUiView();
                renderStatus(true);
                return String();
            }
        }
    }
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
        if (currentViewShowsCommandLine()) {
            renderCommandLine();
        }
        return out;
    }
    if (changed && currentViewShowsCommandLine()) {
        renderCommandLine();
    }
    return String();
}

static void printCommandHelp()
{
    Serial.println("CardFTx commands:");
    Serial.println("  set freq 1000");
    Serial.println("  set slot odd|even");
    char defaultText[FTX_MAX_MESSAGE_LENGTH];
    formatStandardTxMessage(0, defaultText, sizeof(defaultText));
    Serial.printf("  set msg %s\n", defaultText);
    Serial.println("  set SSID your_wifi_name");
    Serial.println("  set PASS your_wifi_password");
    Serial.println("  sync");
    Serial.println("  tx");
    Serial.println("  esc cancels queued/current TX");
    Serial.println("  home | txset | message | waterfall");
    Serial.println("  show");
    Serial.println("  / cycles home/txset/message/waterfall on the keyboard");
}

static void requestTxNextSlot()
{
    portENTER_CRITICAL(&appStateMux);
    txRequested = true;
    txCancelRequested = false;
    portEXIT_CRITICAL(&appStateMux);
    if (radioTaskHandle != nullptr) {
        xTaskNotifyGive(radioTaskHandle);
    }
}

static bool setTxSlotParity(const String& value)
{
    String lower = value;
    lower.trim();
    lower.toLowerCase();

    if (lower == "odd") {
        portENTER_CRITICAL(&appStateMux);
        txSlotOdd = true;
        portEXIT_CRITICAL(&appStateMux);
        Serial.println("TX slot set: Odd");
        return true;
    }
    if (lower == "even") {
        portENTER_CRITICAL(&appStateMux);
        txSlotOdd = false;
        portEXIT_CRITICAL(&appStateMux);
        Serial.println("TX slot set: Even");
        return true;
    }

    Serial.println("TX slot must be odd or even");
    return false;
}

static void radioTask(void*)
{
    char txText[FTX_MAX_MESSAGE_LENGTH];
    float toneHz = kFtxDefaultTxToneHz;

    while (true) {
        if (!timeIsSynced()) {
            setRadioPhase(RadioPhase::WaitSync);
            ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(250));
            continue;
        }

        bool shouldTx = false;
        bool autoTx = false;
        bool autoTxNow = false;
        bool autoTxWaiting = false;
        bool autoCqRxSlot = false;
        uint8_t autoTxStage = 0;
        bool currentOdd = currentSlotIsOdd();
        bool nextOdd = nextSlotIsOdd();
        int currentElapsed = currentSlotElapsedSeconds();
        portENTER_CRITICAL(&appStateMux);
        if (txRequested) {
            shouldTx = true;
            txRequested = false;
            strncpy(txText, txMessage, sizeof(txText) - 1);
            txText[sizeof(txText) - 1] = '\0';
            toneHz = txToneHz;
        } else if (ftxAutoEnabled() && autoTxPending && currentOdd == txSlotOdd && currentElapsed <= 1) {
            shouldTx = true;
            autoTx = true;
            autoTxNow = true;
            autoTxPending = false;
            strncpy(txText, autoTxMessage, sizeof(txText) - 1);
            txText[sizeof(txText) - 1] = '\0';
            toneHz = txToneHz;
            autoTxStage = autoQsoStage;
        } else if (ftxAutoEnabled() && autoTxPending && nextOdd == txSlotOdd) {
            shouldTx = true;
            autoTx = true;
            autoTxPending = false;
            strncpy(txText, autoTxMessage, sizeof(txText) - 1);
            txText[sizeof(txText) - 1] = '\0';
            toneHz = txToneHz;
            autoTxStage = autoQsoStage;
        } else if (kFtxAutoMode == FtxAutoMode::AutoCQ && autoQsoStage == 0 && currentOdd == txSlotOdd &&
            currentElapsed <= 1) {
            shouldTx = true;
            autoTx = true;
            autoTxNow = true;
            formatStandardTxMessage(0, txText, sizeof(txText));
            toneHz = txToneHz;
        } else if (kFtxAutoMode == FtxAutoMode::AutoCQ && autoQsoStage == 0 && nextOdd == txSlotOdd) {
            shouldTx = true;
            autoTx = true;
            formatStandardTxMessage(0, txText, sizeof(txText));
            toneHz = txToneHz;
        } else if (kFtxAutoMode == FtxAutoMode::AutoCQ && autoQsoStage == 0) {
            autoCqRxSlot = true;
        } else if (ftxAutoEnabled() && autoTxPending) {
            autoTxWaiting = true;
        }
        portEXIT_CRITICAL(&appStateMux);

        if (autoTxWaiting) {
            Serial.printf("Auto TX pending, waiting for %s slot\n", txSlotOdd ? "Odd" : "Even");
        } else if (autoCqRxSlot) {
            Serial.printf("Auto CQ RX slot, TX slot=%s\n", txSlotOdd ? "Odd" : "Even");
        }

        if (shouldTx) {
            if (autoTx) {
                Serial.printf(autoTxNow ? "Auto TX now: %s\n" : "Auto TX: %s\n", txText);
            }
            transmitFt8(txText, toneHz, !autoTxNow, autoTx);
            if (autoTx && autoTxStage >= 4) {
                portENTER_CRITICAL(&appStateMux);
                autoQsoStage = 0;
                autoQsoCallsign[0] = '\0';
                portEXIT_CRITICAL(&appStateMux);
            }
            continue;
        }

        receiveOnce();
        delay(1);
    }
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
            portENTER_CRITICAL(&appStateMux);
            txToneHz = value;
            portEXIT_CRITICAL(&appStateMux);
            Serial.printf("TX audio frequency=%.1f Hz\n", txToneHz);
            renderStatus(true);
        }
    } else if (lower.startsWith("set slot ")) {
        if (setTxSlotParity(line.substring(9))) {
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
        char messageCopy[FTX_MAX_MESSAGE_LENGTH];
        bool pendingTx;
        RadioPhase phase;
        portENTER_CRITICAL(&appStateMux);
        strncpy(messageCopy, txMessage, sizeof(messageCopy) - 1);
        messageCopy[sizeof(messageCopy) - 1] = '\0';
        pendingTx = txRequested;
        phase = radioPhase;
        portEXIT_CRITICAL(&appStateMux);
        Serial.printf("FT8 %s TX=%s\n", displayPhaseText(phase), pendingTx ? "queued" : "idle");
        Serial.printf("%s\n", messageCopy);
        renderStatus(true);
    } else if (lower == "tx") {
        requestTxNextSlot();
        Serial.println("TX queued for the next FT8 slot");
        renderStatus(true);
    } else if (lower == "esc" || lower == "cancel" || lower == "stop") {
        requestTxCancel();
    } else if (lower == "rx" || lower == "rx once") {
        Serial.println("Continuous RX is already running");
    } else if (lower == "home") {
        setUiView(UiView::Home);
        renderStatus(true);
    } else if (lower == "txset" || lower == "tx setting" || lower == "txsetting") {
        setUiView(UiView::TxSetting);
        renderStatus(true);
    } else if (lower == "message" || lower == "msg" || lower == "history" || lower == "hist") {
        setUiView(UiView::Message);
        renderStatus(true);
    } else if (lower == "waterfall" || lower == "wf") {
        setUiView(UiView::Waterfall);
        renderStatus(true);
    } else {
        Serial.println("Unknown command. Type: help");
    }
}

bool ft8AppBegin()
{
    Serial.printf("Free internal heap: %u bytes\n", heap_caps_get_free_size(MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT));
    Serial.printf("Free PSRAM heap: %u bytes\n", heap_caps_get_free_size(MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT));

    Serial.println("TX audio uses M5Cardputer.Speaker");
    Serial.println("RX audio uses M5Cardputer.Mic");

    strncpy(wifiSsid, kDefaultWifiSsid, sizeof(wifiSsid) - 1);
    wifiSsid[sizeof(wifiSsid) - 1] = '\0';
    strncpy(wifiPassword, kDefaultWifiPassword, sizeof(wifiPassword) - 1);
    wifiPassword[sizeof(wifiPassword) - 1] = '\0';
    formatStandardTxMessage(0, txMessage, sizeof(txMessage));
    txSlotOdd = configTxSlotIsOdd();

    displayMutex = xSemaphoreCreateMutex();
    if (displayMutex == nullptr) {
        Serial.println("display mutex allocation failed");
        setRadioPhase(RadioPhase::Error);
        return false;
    }

    hashtableInit();
    renderStatus(true);

    if (wifiSsid[0] != '\0' && connectWifi(wifiSsid, wifiPassword)) {
        syncTime();
    }

    BaseType_t created = xTaskCreatePinnedToCore(
        radioTask,
        "ft8Radio",
        kRadioTaskStackBytes,
        nullptr,
        1,
        &radioTaskHandle,
        0);
    if (created != pdPASS) {
        Serial.println("radio task allocation failed");
        setRadioPhase(RadioPhase::Error);
        return false;
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
        if (!isWaterfallCaptureVisible()) {
            renderStatus();
        }
        delay(10);
        return;
    }

    handleCommand(line);
}
