# ESP32 App Layout

ESP32-S3 firmware for the Wheel-Legged Robot wireless tuning bridge.

```text
esp32_app/
├── CMakeLists.txt          # ESP-IDF project entry
├── sdkconfig.defaults      # Default ESP-IDF options
├── data/                   # SPIFFS web assets
└── main/                   # ESP-IDF main component
    ├── main.c              # app_main entrypoint
    ├── app/                # Runtime orchestration
    ├── bsp/                # Board pin/peripheral configuration
    ├── communication/      # STM32 UART protocol and telemetry bridge
    ├── module/             # Auxiliary hardware features
    │   ├── fan/            # Fan PWM module
    │   ├── light/          # Light PWM module
    │   └── screen/         # Local screen UI module
    └── network/            # WiFi AP, HTTP server, WebSocket, SPIFFS
```

Keep feature modules under `main/module/<feature>/` with their `.c` and `.h`
files together. Shared board-level pin and peripheral defaults belong in
`main/bsp/`, while protocol/network code should stay outside feature modules.

## Round TFT Video

The ST77916 360x360 round TFT is driven over QSPI. Boot and loop animations are
stored in SPIFFS as WVJ files generated from the source MOV files:

```bash
python3 esp32_app/tools/convert_screen_video.py
```

Useful tuning options:

```bash
python3 esp32_app/tools/convert_screen_video.py --fps 15 --quality 8
python3 esp32_app/tools/convert_screen_video.py --fps 18 --quality 10
```

Higher `--fps` is smoother but costs more decode time and flash space. Higher
`--quality` values in ffmpeg `-q:v` make smaller JPEG frames.
