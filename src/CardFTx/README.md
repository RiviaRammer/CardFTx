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

## Serial Commands

Open Serial Monitor at `115200`.

- `wifi SSID PASSWORD` connects Wi-Fi and starts NTP sync. SSIDs with spaces are not supported by this simple parser yet.
- `sync` runs NTP sync again after Wi-Fi is connected.
- `msg CQ TEST AB12` edits the stored test message and validates it locally.
- `freq 1000` sets the FT8 base audio tone in Hz.
- `beep` plays a 1 kHz stereo test tone for hardware audio bring-up.
- `beep 2000` plays a 2 kHz stereo test tone.
- `play` encodes the stored message, waits for the next UTC 15 second FT8 boundary, and plays it.
- `playnow` plays immediately for bench testing.
- `tx CQ TEST AB12` encodes one message, waits for the next FT8 boundary, and plays it without changing the stored test message.
- `txnow CQ TEST AB12` plays immediately for bench testing.
- `rxonce` turns off the speaker, enables the built-in microphone, captures one FT8 window at the next UTC 15 second boundary, and decodes it.
- `mictest` captures a short microphone window and prints peak/average sample levels.
- `mic left` / `mic right` selects the `M5Cardputer.Mic` channel used for RX.
- `micgain 64` changes the `M5Cardputer.Mic` magnification used for RX.
- `show` prints the stored message, frequency, Wi-Fi state, and UTC sync state.
- `vol 255` sets the M5Cardputer speaker volume.

The receiver captures a 15 second FT8 window at 12 kHz with `M5Cardputer.Mic` and prints decoded candidates:

```text
FT8 +12.5 dB +0.80 s 1000 Hz ~ CQ TEST AB12
```

## Current Scope

The TX path uses `M5Cardputer.Speaker` and schedules playback on UTC 00/15/30/45 second boundaries after NTP sync. `rxonce` uses `M5Cardputer.Mic`, so speaker playback and microphone capture are switched rather than used simultaneously.
