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
- `tx` encodes the stored message, waits for the next UTC 15 second FT8 boundary, and plays it.
- `rx` or `rx once` turns off the speaker, enables the built-in microphone, captures one FT8 window at the next UTC 15 second boundary, and decodes it.
- `show` prints the stored message, frequency, Wi-Fi state, and UTC sync state.
- `help` prints the command list.

The receiver captures a 15 second FT8 window at 12 kHz with `M5Cardputer.Mic` and prints decoded candidates:

```text
FT8 +12.5 dB +0.80 s 1000 Hz ~ CQ TEST AB12
```

## Current Scope

The TX path uses `M5Cardputer.Speaker` and schedules playback on UTC 00/15/30/45 second boundaries after NTP sync. `rx` uses `M5Cardputer.Mic`, so speaker playback and microphone capture are switched rather than used simultaneously.
