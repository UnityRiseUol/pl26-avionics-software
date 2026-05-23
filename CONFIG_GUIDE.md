# Configuration Guide

## Overview
Your avionics system stores all settings in a `config.txt` file on the SD card. You can edit this file directly or update settings through the web interface.

## Editing config.txt on the SD Card

### Via MSC Mode (Easiest)
1. **Enter MSC Mode**: Hold the button during device startup until the LED turns blue
2. **Connect to PC**: Plug the device into your computer via USB
3. **Access Files**: The SD card will appear as a USB drive on your PC
4. **Edit config.txt**: Open `/config.txt` with any text editor (Notepad, VS Code, etc.)
5. **Save and Eject**: Save the file and safely eject the USB drive
6. **Restart**: Disconnect and restart the device to apply changes

### Format of config.txt
The file uses **simple key=value pairs**, one per line. No JSON required for direct editing.

## All Available Settings

### Sensor & Data Logging

| Setting | Type | Default | Range | Description |
|---------|------|---------|-------|-------------|
| `sensorSampleRateMs` | int | 10 | 5-10000 | IMU/BMP/MAG sampling rate in milliseconds |
| `gpsSampleRateMs` | int | 1000 | 100-60000 | GPS polling interval in milliseconds |
| `logFlushIntervalMs` | int | 1000 | 500-60000 | How often to flush data to SD card |

### Pressure & Altitude

| Setting | Type | Default | Range | Description |
|---------|------|---------|-------|-------------|
| `pressure` | float | 1013.25 | 800-1200 | Sea level atmospheric pressure (hPa) |

### LED Brightness (0-255)

| Setting | Type | Default | Range | Description |
|---------|------|---------|-------|-------------|
| `ledBrightnessHeartbeat` | int | 40 | 0-255 | Brightness of heartbeat LED during logging |
| `ledBrightnessIdle` | int | 20 | 0-255 | Brightness of idle indicator LED |

### Flight Detection Thresholds

| Setting | Type | Default | Range | Description |
|---------|------|---------|-------|-------------|
| `launchThresholdMultiplier` | float | 5.0 | 1.0-20.0 | Launch acceleration threshold (x baseline std dev) |
| `burnoutThresholdMultiplier` | float | 2.0 | 0.5-10.0 | Motor burnout threshold (x baseline std dev) |
| `launchStreakRequired` | int | 2 | 1-50 | Consecutive samples for launch detection |
| `burnoutStreakRequired` | int | 5 | 1-100 | Consecutive samples for burnout detection |
| `apogeeStreakRequired` | int | 8 | 1-100 | Consecutive descent samples to detect apogee |
| `landStreakRequired` | int | 10 | 1-100 | Consecutive samples for landing detection |
| `landingAltitudeThresholdFt` | float | 15.0 | 1.0-500.0 | Altitude threshold for landing (feet) |

### WiFi Credentials

| Setting | Type | Default | Description |
|---------|------|---------|-------------|
| `wifiSSID` | string | LIFTSv2 | WiFi access point name |
| `wifiPassword` | string | 123456789 | WiFi access point password |

## Example config.txt File

```
pressure=1013.25
sensorSampleRateMs=10
gpsSampleRateMs=1000
logFlushIntervalMs=1000
ledBrightnessHeartbeat=40
ledBrightnessIdle=20
launchThresholdMultiplier=5.0
burnoutThresholdMultiplier=2.0
launchStreakRequired=2
burnoutStreakRequired=5
apogeeStreakRequired=8
landStreakRequired=10
landingAltitudeThresholdFt=15.0
wifiSSID=LIFTSv2
wifiPassword=123456789
```

## Updating Settings via Web Interface

You can also update settings through the web interface:

1. **Connect to WiFi**: Connect your device to the WiFi network (default: `LIFTSv2`)
2. **Open Web Browser**: Navigate to `http://192.168.4.1/`
3. **Access Config**: Send a POST request to `/updateconfig` with JSON body:

```json
{
  "pressure": 1013.25,
  "sensorSampleRateMs": 10,
  "gpsSampleRateMs": 1000,
  "ledBrightnessHeartbeat": 40,
  "launchThresholdMultiplier": 5.0,
  "wifiSSID": "MyNetwork"
}
```

Or use curl:
```bash
curl -X POST http://192.168.4.1/updateconfig \
  -H "Content-Type: application/json" \
  -d '{"pressure": 1013.25, "sensorSampleRateMs": 10}'
```

## Notes

- **Validation**: All values have minimum and maximum bounds. Invalid values are rejected.
- **Auto-Creation**: If `config.txt` doesn't exist, it's automatically created with defaults on first boot.
- **Changes Apply On**: Configuration changes take effect when the device restarts or when you enter/exit logging mode.
- **No Special Characters**: Keep WiFi SSID/password simple (alphanumeric recommended).
- **Backup**: Before editing, it's good practice to backup your `config.txt`.

## Tuning Flight Detection

For rocket launches, you may need to adjust detection thresholds:

- **Increase `launchThresholdMultiplier`** (higher value) → More stable, less false launches
- **Decrease `launchThresholdMultiplier`** (lower value) → More sensitive, catches low-acceleration launches
- **Increase `burnoutStreakRequired`** → More stable burnout detection
- **Decrease `launchStreakRequired`** → Faster launch detection (but more noise-sensitive)

## Troubleshooting

**Settings not saving?**
- Ensure SD card is properly formatted and has write permissions
- Check that file isn't corrupt - try deleting and letting it recreate

**LED too bright/dim?**
- Adjust `ledBrightnessHeartbeat` and `ledBrightnessIdle` (0 = off, 255 = maximum)

**WiFi not connecting?**
- Restart the device
- Check SSID/password don't contain special characters
- Try resetting to defaults


