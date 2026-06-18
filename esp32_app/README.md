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

## Round TFT RoboEyes

The ST77916 360x360 round TFT is driven over QSPI. After panel init, firmware
starts the external FluxGarage RoboEyes library and drives its real-time program
animation from the app tick loop.

The build pins RoboEyes to `b42f8e596535234932be3514ac7a813d4ced0046`. By
default CMake clones it under `build/_deps/`; for offline builds, point CMake at
an existing checkout:

```bash
idf.py -DROBOEYES_SOURCE_DIR="/path/to/FluxGarage RoboEyes" build
```
