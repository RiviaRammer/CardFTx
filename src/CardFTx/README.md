# CardFTx on Cardputer Adv

This sketch adapts `ft8_lib` to an ESP32-S3 Arduino project using the ES8311 codec.

## Pins

| ES8311 | ESP32-S3 GPIO |
| --- | --- |
| SDA | G8 |
| SCL | G9 |
| SCLK / BCLK | G41 |
| ASDOUT / ADC data | G46 |
| LRCK / WS | G43 |
| DSDIN / DAC data | G42 |

## Build Notes

- Select an ESP32-S3 board profile in Arduino IDE.
- If boot prints `quad_psram: PSRAM chip is not connected, or wrong PSRAM`, set PSRAM to Disabled or try the board package's exact Cardputer Adv PSRAM option. The sketch now falls back to internal RAM when PSRAM is unavailable.
- Use a large app partition if the default sketch partition is tight.
- The decode task uses a 16 KB stack. Large FFT buffers are allocated on heap/PSRAM instead of the task stack.
- Copy `src/config_example.h` to `src/config.h` for local Wi-Fi credentials and private settings. Keep `src/config.h` out of git.

## Commands

Open Serial Monitor at `115200`, or type the same commands on the Cardputer keyboard. The bottom line of the screen shows the current keyboard command; press Enter to run it.

- `set SSID your_wifi_name` sets the Wi-Fi SSID.
- `set PASS your_wifi_password` sets the Wi-Fi password.
- `sync` connects Wi-Fi with the current SSID/PASS and runs NTP sync.
- `set msg CQ TEST AB12` edits the stored FT8 message and validates it locally.
- `set freq 1000` sets the FT8 base audio tone in Hz.
- `set slot odd` or `set slot even` sets the configured TX slot parity for future Auto TX.
- `tx` queues the stored message for the next UTC 15 second FT8 boundary. After TX, the radio task returns to continuous RX.
- `esc`, `cancel`, or `stop` cancels queued TX or stops the current TX.
- `rx` or `rx once` is retained as a harmless status command; continuous RX is already running by default.
- `home` shows the live dashboard.
- `txset`, `tx setting`, or `txsetting` shows TX settings.
- `message`, `msg`, `history`, or `hist` shows recent decoded messages.
- `waterfall` or `wf` shows the live RX waterfall during capture.
- On the Cardputer keyboard, `/` cycles through home, TX Setting, Message, and waterfall. ESC cancels queued/current TX.
- `show` prints one status line followed by the stored message.
- `help` prints the command list.

The receiver starts automatically after boot once UTC time is synced. It captures an FT8 window at 12 kHz with `M5Cardputer.Mic` and prints decoded candidates:

```text
FT8 +12.5 dB +0.80 s 1000 Hz ~ CQ TEST AB12
```

## Current Scope

The TX path uses `M5Cardputer.Speaker` and schedules playback on UTC 00/15/30/45 second boundaries after NTP sync. RX uses `M5Cardputer.Mic`, so speaker playback and microphone capture are switched rather than used simultaneously.

Standard TX messages are generated from `kFtxStationCallsign` and `kFtxStationGrid` in `src/config.h`; report examples use `kFtxFallbackReportDb`. Auto behavior is configured with `kFtxAutoMode`, TX slot parity with `kFtxTxSlot`, and the default transmit audio tone with `kFtxDefaultTxToneHz`.

Auto mode uses simple FT8 keyword matching. `AutoCQ` sends `CQ <mycall> <grid>` on the configured TX slot when idle. `AutoAnswer` replies to `CQ <call> <grid>` with `<call> <mycall> <grid>`. Directed reports such as `<mycall> <call> -10` are answered with `<call> <mycall> R<actual-snr>`, `RRR`/`RR73` are answered with `73`, and `73` marks the QSO complete. `Manual` mode still continuously receives and only transmits when the `tx` command is issued.

For two-board Auto tests, configure one board as `AutoCQ` and the other as `AutoAnswer`, with opposite TX slots (`Odd` vs `Even`). If a decoded message does not trigger a reply, the serial log prints the Auto ignore reason.

In this UI, `Odd` means UTC second windows `00-15` and `30-45`; `Even` means `15-30` and `45-60`.

## Runtime Design

CardFTx now treats RX as the default operating mode. Single-shot RX/TX commands are avoided in the main UI path: the radio task continuously waits for the next FT8 slot, captures RX audio, decodes, and loops. The `tx` command only marks the current stored message as pending for the next slot; the radio task performs the scheduled TX and then goes back to RX.

The task split is intentionally simple:

- Arduino `loop()` remains on the UI side. It calls `M5Cardputer.update()`, reads the keyboard/serial command line, and redraws the screen once per second.
- `ft8Radio` is pinned to core 0. It owns microphone capture, FT8 decode, scheduled speaker TX, and audio device switching.
- Shared state is copied through a small critical section. The radio task only draws directly while the waterfall view is visible during RX capture.

## Screen Design

The home screen is a compact dashboard rather than a full waterfall:

```text
FT8                         RX
UTC 12:34:08          RX->RX
Odd 07s

TX:

----------------------
RX new 2 total 5
>command
```

- The top-left protocol label is `FT8`; `FT4` is reserved for future support.
- The top-right status matches the current status shown on the UTC row.
- The UTC time is enlarged; slot parity and countdown sit below it.
- The UTC row also shows the current action and predicted next action on the right.
- The TX area only shows `TX:` on Home; TX details live on the TX Setting page.
- The RX area is separated from TX and uses larger text for received/new/listening state.

Press `/` to cycle Home -> TX Setting -> Message -> Waterfall. The TX Setting page shows the transmit frequency, configured TX slot parity, and preset standard messages for future Auto mode. The Message page uses the full screen for decoded and transmitted lines, without the Home header, UTC band, or bottom command line. RX lines keep their original colors; TX lines are red. Each decoded slot starts with a divider whose right side shows that slot's UTC end timestamp and a drawn arrow. Opening Message clears the unread count. The waterfall screen draws the live RX waterfall while a capture is in progress and does not show the bottom command line.
