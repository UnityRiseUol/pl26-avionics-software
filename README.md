# PL-26 Flight Computer Software

Embedded avionics firmware for **PL-26**, developed for the **Unity Rise University of Liverpool Rocket Team** for the **2025-26 launch**.

This repository is the software component of a **Year 3 MEng Individual Project (ELEC440)** for the **MEng Avionic Systems** degree.

## Project Context

- **Author:** Joseph Wood
- **Module:** ELEC440 MEng Individual Project
- **Project Title:** LASER - Rocket Flight Tracking and Control
- **Rocket:** PL-26 (Unity Rise, 2025-26)

## What This Firmware Does

From `src/main.cpp`, the flight computer provides:

- Sensor acquisition from BMP390 (barometer), BNO085 (IMU/orientation), ICM20948 (IMU) and u-blox MAX-M10S GNSS
- INS/Dead reckoning algorithm integration via `lib/INS_Model` (`INS_Model_C`), generated using the Simulink C++ Coder
- SD card CSV logging using `SD_MMC`
- LoRa telemetry transmission at 868 MHz
- Wi-Fi SoftAP + HTTP server for configuration and file access
- FreeRTOS task-based software for task and core management with mutex-protected shared state

## Runtime Architecture

Core tasks created in `setup()`:

- `highFrequencySensorTask` - reads BMP3XX and BNO08x data and computes filtered vertical speed
- `gpsTask` - reads GNSS PVT data and updates GPS fields
- `insTask` - maps sensor inputs into `INS_Model_C` and updates INS outputs
- `loggingTask` - writes periodic CSV rows to SD card and flushes once per second
- `loraTask` - sends packed binary telemetry packets over LoRa
- `webServerTask` - services HTTP requests
- `buttonTask` - toggles logging pause/resume with debounce

Synchronization primitives:

- `xSensorDataMutex` protects shared sensor/INS data
- `xSpiMutex` arbitrates SPI access between sensors and LoRa

## Data Outputs

### SD Logging

At boot, firmware creates an incremented log file:

- `/Flight_Data_<n>.csv`

CSV columns include:

- Time (ms)
- BMP: temperature, pressure, altitude, vertical speed
- BNO: linear acceleration, gravity, quaternion
- GNSS: latitude, longitude, altitude, speed, heading
- INS: `X_Estimate`, `Y_Estimate`, `Z_Estimate`, `Lat_Estimate`, `Long_Estimate`

### LoRa Telemetry

LoRa payload uses a packed `TelemetryPacket` (44 bytes) containing:

- `altitude`, `vSpeed`
- `lat`, `lon`
- `qR`, `qI`, `qJ`, `qK`
- `insX`, `insY`, `insZ`

Configured in code for:

- Band: `868E6`
- CRC enabled
- Bandwidth `500E3`, spreading factor `7`, coding rate `4/5`

### VEGA Raspberry Pi UART Link

LIFTSv2 now includes a dedicated UART link for the VEGA Raspberry Pi recorder.

- UART TX: GPIO `17`
- UART RX: GPIO `18`
- Baud rate: `115200`

Flight-phase events trigger outgoing commands:

- `VEGA_STARTED` when launch is detected
- `STOP_VEGA` when landing/mission end is detected

Incoming VEGA status lines are stored in the main flight CSV and mirrored to a backup event log:

- Main log columns now include `vegaOn`, `vegaStatus`, `vegaLastMessage`, `vegaLastMessageMillis`
- Backup event log: `/vega_uart_<n>.csv`

Expected inbound status messages include the Pi-side handshake and shutdown flow, for example:

- `VEGA_START_ACKNOWLEDGED`
- `VEGA_RECORDING_STARTED`
- `VEGA_ACTIVE_T+10s`
- `VEGA_STOP_ACKNOWLEDGE`
- `VEGA_SAVING_DATA`
- `VEGA_SAVED_AND_STOPPED`
- `VEGA_DATA_SECURE_INITIATING_SHUTDOWN`

If you need to rewire the Pi link, adjust the `VEGA_UART_*` constants near the top of `src/main.cpp`.

## Configuration

Configuration is saved on SD card at:

- `/config.txt`

Current contents:

```text
pressure=1013.25
```

- `pressure` maps to `seaLevelPressureHpa` for barometric altitude reference.

## Repository Structure

```text
pl26-avionics-software/
|- include/
|- lib/
|  |- INS_Model/
|- src/
|  |- main.cpp
|- test/
|- platformio.ini
|- README.md
|- LICENSE.md
```

## Build and Flash

This project uses PlatformIO, MCU is ESP32-S3-Mini-1-N8.

```bash
platformio run
platformio run --target upload
platformio device monitor -b 115200
```

## Related Repositories

- Test code: https://github.com/UnityRiseUol/pl26-avionics-test-code
- Altium Designer hardware files: https://github.com/UnityRiseUol/pl26-avionics-hardware

## Safety Notice

This software is developed for educational rocketry use. Please complete thorough testing and validation before flight.

## License

Licensed under the MIT Licence. See `LICENSE.md`.
