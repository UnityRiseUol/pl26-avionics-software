/*
 * File:        main.cpp
 * Author:      Joseph Wood
 * Last Update: 21/10/2025
 *
 * Project:   ELEC440 MEng Individual Project
 * Title:     "LASER - Rocket Flight Tracking and Control"
 */

// --- Includes ---
#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <LoRa.h>
#include <Adafruit_Sensor.h>
#include "Adafruit_BMP3XX.h"
// #include "ICM_20948.h"
#include "Adafruit_BNO08x.h"
#include <SparkFun_u-blox_GNSS_Arduino_Library.h>
#include "FS.h"
#include "SD_MMC.h"
#include <cstdio>
#include <cstring>
#include <WiFi.h>
#include <WebServer.h>
#include <ArduinoJson.h>
#include <INS_Model_C.h>
#include <cmath>

// --- Definitions ---
#define BMP_CS 18
// #define ICM_CS 2
#define BNO08X_CS 4
#define BNO08X_INT 17
#define BNO08X_RESET 1

// LoRa Definitions
#define RFM95_CS    7
#define RFM95_RST   5
#define RFM95_INT   6
#define BAND 868E6

// --- WiFi & Web Server ---
auto ssid = "LIFTSv2";
auto password = "123456789";
WebServer server(80);

// --- Sensor Objects ---
Adafruit_BMP3XX bmp;
// ICM_20948_SPI icm;
Adafruit_BNO08x bno08x(BNO08X_RESET);
sh2_SensorValue_t bnoSensorValue;
SFE_UBLOX_GNSS myGNSS;

// --- SD Card and File Handling ---
char logFileName[35];
File dataFile;

// --- SDIO Pin Definitions ---
int pinSdioClk = 38;
int pinSdioCmd = 34;
int pinSdioD0 = 39;
int pinSdioD1 = 40;
int pinSdioD2 = 47;
int pinSdioD3 = 33;

// --- Global State Variables ---
volatile bool loggingEnabled = true;
constexpr int buttonPin = 15;

// --- Vertical Speed Calculation ---
#define VERTICAL_SPEED_SAMPLES 10
float altitudeSamples[VERTICAL_SPEED_SAMPLES] = {0};
int sampleIndex = 0;
float lastAltitude = 0;
unsigned long lastVsTime = 0;

// --- Configuration Struct ---
struct SystemConfig {
    float seaLevelPressureHpa;
};
SystemConfig config = {1013.25};

// --- Shared Data Structure (Full Resolution for SD Card) ---
struct AllSensorData {
    float bmpTemperature, bmpPressure, bmpAltitude, bmpVerticalSpeed;
    // float icmAccX, icmAccY, icmAccZ;
    // float icmGyrX, icmGyrY, icmGyrZ;
    // float icmMagX, icmMagY, icmMagZ;
    float bnoLinearAccX, bnoLinearAccY, bnoLinearAccZ;
    float bnoGravityX, bnoGravityY, bnoGravityZ;
    float bnoQuatR, bnoQuatI, bnoQuatJ, bnoQuatK;
    float gpsLatitude, gpsLongitude, gpsAltitude;
    float gpsSpeed, gpsHeading;
    bool gpsValid;
};

// --- Telemetry Packet Structure (Compressed for LoRa) ---
// Total Size: 44 bytes
struct __attribute__((packed)) TelemetryPacket {
    float altitude;   // 4 bytes
    float vSpeed;    // 4 bytes
    float lat;        // 4 bytes
    float lon;        // 4 bytes
    float qR;         // 4 bytes
    float qI;         // 4 bytes
    float qJ;         // 4 bytes
    float qK;         // 4 bytes
    float insX;       // 4 bytes
    float insY;       // 4 bytes
    float insZ;       // 4 bytes
};

// Global instances
AllSensorData sensorData = {0};
INS_Model_C insModel;
INS_Model_C::ExtU_INS_Model_C_T insInputs;
INS_Model_C::ExtY_INS_Model_C_T insOutputs;
bool insRefInitialized = false;

// --- RTOS Synchronization ---
SemaphoreHandle_t xSensorDataMutex;
SemaphoreHandle_t xSpiMutex;

// --- Task Function Prototypes ---
void highFrequencySensorTask(void *pvParameters);
void insTask(void *pvParameters);
void gpsTask(void *pvParameters);
void loggingTask(void *pvParameters);
void buttonTask(void *pvParameters);
void webServerTask(void *pvParameters);
void loraTask(void *pvParameters);
void flightPhaseTask(void *pvParameters);

// ─────────────────────────────────────────────────────────────────────────────
// Flight Phase Detection
// Ported from State_Prediction.py — no look-ahead, processes one sample at a time.
//
// Phase IDs:
//   0 = IDLE  (pre-launch baseline)
//   1 = LAUNCH
//   2 = MOTOR BURNOUT
//   3 = APOGEE
//   4 = PARACHUTE DEPLOYED
//   5 = LANDED
// ─────────────────────────────────────────────────────────────────────────────

static constexpr const char* const FLIGHT_PHASE_NAMES[] = {
    "IDLE",
    "LAUNCH",
    "MOTOR BURNOUT",
    "APOGEE",
    "PARACHUTE DEPLOYED",
    "LANDED"
};

class FlightDetector {
public:
    static constexpr int BASELINE_N    = 30;
    static constexpr int CHUTE_BUF_SZ  = 20;

    FlightDetector()
        : _phase(0), _blAccelSum(0.0f), _blAccelSumSq(0.0f), _blAltSum(0.0f), _blN(0),
          _baselineDone(false), _blAccelMean(0.0f), _blAccelStd(1.0f), _blAltMean(0.0f),
          _launchStreak(0), _burnoutStreak(0), _apogeePeak(0.0f), _apogeeStreak(0),
          _chuteBuf{}, _chuteHead(0), _chuteCount(0), _chuteWarmup(0), _landStreak(0) {
    }

    void reset() {
        _phase        = 0;
        _blAccelSum   = 0.0f; _blAccelSumSq = 0.0f; _blAltSum = 0.0f; _blN = 0;
        _baselineDone = false;
        _blAccelMean  = 0.0f; _blAccelStd = 1.0f; _blAltMean = 0.0f;
        _launchStreak = 0; _burnoutStreak = 0;
        _apogeePeak   = 0.0f; _apogeeStreak = 0;
        for (auto& val : _chuteBuf) val = 0.0f;
        _chuteHead = 0; _chuteCount = 0; _chuteWarmup = 0;
        _landStreak = 0;
    }

    // Feed one sample. Returns new phase (0-5) on transition, -1 otherwise.
    // accel_mag : total acceleration magnitude (m/s²)
    // alt_ft    : barometric altitude (feet)
    // accel_baro: d(vertical_speed)/dt (m/s²)
    int feed(float accel_mag, float alt_ft, float accel_baro) {
        const int prev = _phase;

        // ── Baseline collection ──────────────────────────────────────────────
        if (!_baselineDone) {
            if (_blN >= 3) {
                const float runningMean = _blAccelSum / static_cast<float>(_blN);
                if (accel_mag > runningMean * 5.0f) {
                    // Looks like ignition — finalise without this sample, fall through
                    _finaliseBaseline();
                } else {
                    _accumulate(accel_mag, alt_ft);
                    if (_blN >= BASELINE_N) _finaliseBaseline();
                    return -1;
                }
            } else {
                _accumulate(accel_mag, alt_ft);
                return -1;
            }
        }

        // ── State machine ────────────────────────────────────────────────────

        if (_phase == 0) {
            // IDLE → LAUNCH: accel_mag > baseline + 5σ, sustained 2+ samples
            const float thresh = _blAccelMean + 5.0f * _blAccelStd;
            _launchStreak = (accel_mag > thresh) ? (_launchStreak + 1) : 0;
            if (_launchStreak >= 2) _phase = 1;
        }
        else if (_phase == 1) {
            // LAUNCH → MOTOR BURNOUT: accel_mag back within baseline + 2σ for 5 samples
            const float thresh = _blAccelMean + 2.0f * _blAccelStd;
            _burnoutStreak = (accel_mag < thresh) ? (_burnoutStreak + 1) : 0;
            if (_burnoutStreak >= 5) _phase = 2;
        }
        else if (_phase == 2) {
            // MOTOR BURNOUT → APOGEE: altitude below rolling peak for 8 samples
            if (alt_ft >= _apogeePeak) { _apogeePeak = alt_ft; _apogeeStreak = 0; }
            else                       { _apogeeStreak++; }
            if (_apogeeStreak >= 8) _phase = 3;
        }
        else if (_phase == 3) {
            // APOGEE → PARACHUTE: statistical spike in accel_baro vs recent noise
            _chuteBuf[_chuteHead] = accel_baro;
            _chuteHead = (_chuteHead + 1) % CHUTE_BUF_SZ;
            if (_chuteCount < CHUTE_BUF_SZ) _chuteCount++;
            _chuteWarmup++;

            if (_chuteWarmup >= 15 && _chuteCount >= 15) {
                float mean = 0.0f;
                for (int i = 0; i < _chuteCount; ++i) mean += _chuteBuf[i];
                mean /= static_cast<float>(_chuteCount);
                float var = 0.0f;
                for (int i = 0; i < _chuteCount; ++i) {
                    const float d = _chuteBuf[i] - mean;
                    var += d * d;
                }
                const float sd = sqrtf(var / static_cast<float>(_chuteCount));
                const float eff = (sd > 1.0f) ? sd : 1.0f;
                if (fabsf(accel_baro - mean) > 4.0f * eff) _phase = 4;
            }
        }
        else if (_phase == 4) {
            // PARACHUTE → LANDED: altitude within 15 ft of ground baseline for 10 samples
            _landStreak = (fabsf(alt_ft - _blAltMean) < 15.0f) ? (_landStreak + 1) : 0;
            if (_landStreak >= 10) _phase = 5;
        }
        // Phase 5 (LANDED) is terminal

        return (_phase != prev) ? _phase : -1;
    }

    int phase() const { return _phase; }

private:
    int   _phase;
    float _blAccelSum, _blAccelSumSq, _blAltSum;
    int   _blN;
    bool  _baselineDone;
    float _blAccelMean, _blAccelStd, _blAltMean;
    int   _launchStreak, _burnoutStreak;
    float _apogeePeak;
    int   _apogeeStreak;
    float _chuteBuf[CHUTE_BUF_SZ];
    int   _chuteHead, _chuteCount, _chuteWarmup;
    int   _landStreak;

    void _accumulate(float am, float af) {
        _blAccelSum += am; _blAccelSumSq += am * am; _blAltSum += af; _blN++;
    }
    void _finaliseBaseline() {
        _blAccelMean = _blAccelSum / static_cast<float>(_blN);
        float var    = (_blAccelSumSq / static_cast<float>(_blN)) - (_blAccelMean * _blAccelMean);
        if (var < 0.0f) var = 0.0f;
        const float sd = sqrtf(var);
        _blAccelStd  = (sd > 0.5f) ? sd : 0.5f;
        _blAltMean   = _blAltSum / static_cast<float>(_blN);
        _baselineDone = true;
    }
};

void flightPhaseTask(void *pvParameters) {
    FlightDetector detector;
    float prevVelMs  = 0.0f;
    unsigned long prevTimeMs = 0;

    for (;;) {
        AllSensorData local{};
        if (xSemaphoreTake(xSensorDataMutex, portMAX_DELAY) == pdTRUE) {
            memcpy(&local, &sensorData, sizeof(AllSensorData));
            xSemaphoreGive(xSensorDataMutex);
        }

        // Total acceleration magnitude: BNO linear accel + gravity vectors
        const float ax = local.bnoLinearAccX + local.bnoGravityX;
        const float ay = local.bnoLinearAccY + local.bnoGravityY;
        const float az = local.bnoLinearAccZ + local.bnoGravityZ;
        const float accelMag = sqrtf(ax*ax + ay*ay + az*az);

        // Altitude in feet (bmpAltitude is metres)
        const float altFt = local.bmpAltitude * 3.28084f;

        // Barometric acceleration: d(verticalSpeed)/dt
        const unsigned long now = millis();
        float accelBaro = 0.0f;
        if (prevTimeMs > 0) {
            const float dt = static_cast<float>(now - prevTimeMs) / 1000.0f;
            if (dt > 0.0f) accelBaro = (local.bmpVerticalSpeed - prevVelMs) / dt;
        }
        prevVelMs  = local.bmpVerticalSpeed;
        prevTimeMs = now;

        const int transition = detector.feed(accelMag, altFt, accelBaro);
        if (transition >= 0) {
            Serial.printf("[FLIGHT PHASE] %d: %s\n",
                          transition, FLIGHT_PHASE_NAMES[transition]);
        }

        vTaskDelay(pdMS_TO_TICKS(20));
    }
}

// --- Configuration Functions ---
void saveConfiguration() {
    File configFile = SD_MMC.open("/config.txt", "w");
    if (configFile) {
        configFile.print("pressure=");
        configFile.println(config.seaLevelPressureHpa);
        configFile.close();
        Serial.println("Configuration saved to /config.txt");
    }
}

void loadConfiguration() {
    File configFile = SD_MMC.open("/config.txt", "r");
    if (!configFile) {
        Serial.println("config.txt not found. Creating default.");
        saveConfiguration();
    } else {
        while (configFile.available()) {
            String line = configFile.readStringUntil('\n');
            line.trim();
            if (line.startsWith("pressure=")) {
                config.seaLevelPressureHpa = line.substring(line.indexOf('=') + 1).toFloat();
            }
        }
        configFile.close();
        Serial.println("Configuration loaded.");
    }
}

// --- Web Server Handlers ---
void handleRoot() {
  File file = SD_MMC.open("/index.html", "r");
  if (!file) {
    server.send(500, "text/plain", "ERROR: index.html not found.");
    return;
  }
  server.streamFile(file, "text/html");
  file.close();
}

void handleGetCsvList() {
  String json = "[";
  File root = SD_MMC.open("/");
  if (root) {
    File file = root.openNextFile();
    bool firstFile = true;
    while(file) {
      const char* fileName = file.name();
      if (strstr(fileName, ".csv")) {
        if (!firstFile) json += ",";
        json += "\"";
        if (fileName[0] == '/') json += (fileName + 1);
        else json += fileName;
        json += "\"";
        firstFile = false;
      }
      file.close();
      file = root.openNextFile();
    }
    root.close();
  }
  json += "]";
  server.send(200, "application/json", json);
}

void handleFileDownload() {
  if (server.hasArg("file")) {
    const String fileName = server.arg("file");
    if (fileName.indexOf('/') != -1 || fileName.indexOf("..") != -1) {
      server.send(400, "text/plain", "Invalid filename.");
      return;
    }
    String path = "/" + fileName;
    File file = SD_MMC.open(path, "r");
    if (!file) {
      server.send(404, "text/plain", "File not found.");
      return;
    }
    server.sendHeader("Content-Disposition", "attachment; filename=" + fileName);
    server.streamFile(file, "text/csv");
    file.close();
  } else {
    server.send(400, "text/plain", "Bad Request: 'file' missing.");
  }
}

void handleGetConfig() {
    JsonDocument doc;
    doc["pressure"] = String(config.seaLevelPressureHpa, 2);
    String output;
    serializeJson(doc, output);
    server.send(200, "application/json", output);
}

void handleUpdateConfig() {
    JsonDocument doc;
    const DeserializationError error = deserializeJson(doc, server.arg("plain"));
    if (error) {
        server.send(400, "text/plain", "Invalid JSON.");
        return;
    }
    if (doc["pressure"].is<float>()) {
        float newPressure = doc["pressure"];
        if (newPressure >= 800.0 && newPressure <= 1200.0) {
            config.seaLevelPressureHpa = newPressure;
            saveConfiguration();
            server.send(200, "text/plain", "Config updated.");
        } else {
            server.send(400, "text/plain", "Invalid pressure.");
        }
    }
}

void handleNotFound() {
  server.send(404, "text/plain", "Not Found");
}

// --- SETUP ---
void setup() {
    Serial.begin(115200);
    pinMode(buttonPin, INPUT);

    // Init SD Card
    if (!SD_MMC.setPins(pinSdioClk, pinSdioCmd, pinSdioD0, pinSdioD1, pinSdioD2, pinSdioD3)) {
        Serial.println("SDIO pin error"); while(true);
    }
    if (!SD_MMC.begin()) {
        Serial.println("Card Mount Failed"); while(true);
    }

    loadConfiguration();

    // Log File Creation
    int maxLogNum = 0;
    File root = SD_MMC.open("/");
    if (root) {
        File file = root.openNextFile();
        while(file) {
            const char* fileName = file.name();
            if (strstr(fileName, "Flight_Data_") && strstr(fileName, ".csv")) {
                const char* p = fileName;
                if (p[0] == '/') p++;
                if (strncmp(p, "Flight_Data_", 12) == 0) {
                    char* end;
                    long val = strtol(p + 12, &end, 10);
                    if (end != p + 12 && strcmp(end, ".csv") == 0) {
                        int currentNum = 0;
                        currentNum = static_cast<int>(val);
                        if (currentNum > maxLogNum) { maxLogNum = currentNum; }
                    }
                }
            }
            file.close();
            file = root.openNextFile();
        }
        root.close();
    }
    int newLogNum = maxLogNum + 1;
    sprintf(logFileName, "/Flight_Data_%d.csv", newLogNum);

    dataFile = SD_MMC.open(logFileName, FILE_WRITE);
    if (dataFile) {
        dataFile.println("Time(ms),Temp(C),Pressure(hPa),Altitude(m),V_Speed(m/s),"
                         // "ICM_AccX,ICM_AccY,ICM_AccZ,ICM_GyrX,ICM_GyrY,ICM_GyrZ,"
                         // "ICM_MagX,ICM_MagY,ICM_MagZ,"
                         "BNO_LinAccX,BNO_LinAccY,BNO_LinAccZ,"
                         "BNO_GravX,BNO_GravY,BNO_GravZ,"
                         "BNO_QuatR,BNO_QuatI,BNO_QuatJ,BNO_QuatK,"
                         "GPS_Lat,GPS_Lon,GPS_Alt,GPS_Speed,GPS_Heading,"
                         "INS_X,INS_Y,INS_Z,INS_Lat,INS_Lon");
        dataFile.flush();
    }

    // Mutexes
    xSensorDataMutex = xSemaphoreCreateMutex();
    xSpiMutex = xSemaphoreCreateMutex();

    // Sensors
    Wire.begin();
    SPI.begin();

    if (!bmp.begin_SPI(BMP_CS)) { Serial.println("BMP Init Failed"); while (true); }
    // icm.begin(ICM_CS, SPI);
    // if (icm.status != ICM_20948_Stat_Ok) { Serial.println("ICM Init Failed"); while (true); }
    if (!bno08x.begin_SPI(BNO08X_CS, BNO08X_INT)) { Serial.println("BNO Init Failed"); while (true); }
    if (!myGNSS.begin()) { Serial.println("GNSS Init Failed"); while (true); }

    bmp.setTemperatureOversampling(BMP3_OVERSAMPLING_8X);
    bmp.setPressureOversampling(BMP3_OVERSAMPLING_4X);
    bmp.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);
    bmp.setOutputDataRate(BMP3_ODR_50_HZ);

    bno08x.enableReport(SH2_LINEAR_ACCELERATION, 5);
    bno08x.enableReport(SH2_GRAVITY, 10);
    bno08x.enableReport(SH2_GEOMAGNETIC_ROTATION_VECTOR, 10);
    myGNSS.setAutoPVT(true);

    Serial.println("Sensors Initialised.");

    // Model Initialisation
    insModel.initialize();
    Serial.println("INS Model Initialised.");

    // --- LoRa Setup ---
    LoRa.setSPI(SPI);
    LoRa.setPins(RFM95_CS, RFM95_RST, RFM95_INT);
    if (!LoRa.begin(BAND)) {
        Serial.println("LoRa Failed!");
    } else {
        LoRa.setTxPower(20);
        LoRa.setSignalBandwidth(500E3); // 500kHz
        LoRa.setSpreadingFactor(7);     // SF7
        LoRa.setCodingRate4(5);         // CR 4/5
        Serial.println("LoRa High Speed (500kHz) Ready.");
    }

    // WiFi & Server
    WiFi.softAP(ssid, password);
    server.on("/", HTTP_GET, handleRoot);
    server.on("/list-csv", HTTP_GET, handleGetCsvList);
    server.on("/download", HTTP_GET, handleFileDownload);
    server.on("/getconfig", HTTP_GET, handleGetConfig);
    server.on("/updateconfig", HTTP_POST, handleUpdateConfig);
    server.onNotFound(handleNotFound);
    server.begin();
    Serial.println("Web Server Started.");

    // Tasks
    xTaskCreatePinnedToCore(highFrequencySensorTask, "SensTask", 4096, nullptr, 3, nullptr, 1);
    xTaskCreatePinnedToCore(insTask,                 "INSTask",  8192, nullptr, 3, nullptr, 1);
    xTaskCreatePinnedToCore(gpsTask,                 "GPSTask",  4096, nullptr, 1, nullptr, 1);
    xTaskCreatePinnedToCore(loggingTask,             "LogTask",  4096, nullptr, 2, nullptr, 0);
    xTaskCreatePinnedToCore(buttonTask,              "BtnTask",  2048, nullptr, 2, nullptr, 0);
    xTaskCreatePinnedToCore(webServerTask,           "WebTask",  4096, nullptr, 2, nullptr, 0);
    xTaskCreatePinnedToCore(loraTask,                "LoRaTask",  4096, nullptr, 3, nullptr, 1);
    xTaskCreatePinnedToCore(flightPhaseTask,         "PhaseTask", 4096, nullptr, 2, nullptr, 0);
}

// --- TASKS ---

void highFrequencySensorTask(void *pvParameters) {
    for (;;) {
        if (xSemaphoreTake(xSensorDataMutex, portMAX_DELAY) == pdTRUE) {

            if (xSemaphoreTake(xSpiMutex, portMAX_DELAY) == pdTRUE) {
                // BMP Reading
                if (bmp.performReading()) {
                    sensorData.bmpTemperature = static_cast<float>(bmp.temperature);
                    sensorData.bmpPressure = static_cast<float>(bmp.pressure / 100.0);
                    sensorData.bmpAltitude = bmp.readAltitude(config.seaLevelPressureHpa);

                    const unsigned long currentTime = millis();
                    if (lastVsTime > 0) {
                        const float dt = static_cast<float>(currentTime - lastVsTime) / 1000.0f;
                        if (dt > 0) {
                            const float rawVs = (sensorData.bmpAltitude - lastAltitude) / dt;
                            altitudeSamples[sampleIndex] = rawVs;
                            sampleIndex = (sampleIndex + 1) % VERTICAL_SPEED_SAMPLES;
                            float totalVs = 0;
                            for (float altitudeSample : altitudeSamples) {
                                totalVs += altitudeSample;
                            }
                            sensorData.bmpVerticalSpeed = totalVs / VERTICAL_SPEED_SAMPLES;
                        }
                    }
                    lastAltitude = sensorData.bmpAltitude;
                    lastVsTime = currentTime;
                }

                // ICM Reading
                /*
                if(icm.dataReady()) {
                    constexpr float G_MPS2 = 9.80665f;
                    icm.getAGMT();
                    sensorData.icmAccX = (icm.accX() / 1000.0f) * G_MPS2;
                    sensorData.icmAccY = (icm.accY() / 1000.0f) * G_MPS2;
                    sensorData.icmAccZ = (icm.accZ() / 1000.0f) * G_MPS2;
                    sensorData.icmGyrX = icm.gyrX();
                    sensorData.icmGyrY = icm.gyrY();
                    sensorData.icmGyrZ = icm.gyrZ();
                    sensorData.icmMagX = icm.magX();
                    sensorData.icmMagY = icm.magY();
                    sensorData.icmMagZ = icm.magZ();
                }
                */

                // BNO Reading
                if (bno08x.getSensorEvent(&bnoSensorValue)) {
                    switch (bnoSensorValue.sensorId) {
                        case SH2_LINEAR_ACCELERATION:
                            sensorData.bnoLinearAccX = bnoSensorValue.un.linearAcceleration.x;
                            sensorData.bnoLinearAccY = bnoSensorValue.un.linearAcceleration.y;
                            sensorData.bnoLinearAccZ = bnoSensorValue.un.linearAcceleration.z;
                            break;
                        case SH2_GRAVITY:
                            sensorData.bnoGravityX = bnoSensorValue.un.gravity.x;
                            sensorData.bnoGravityY = bnoSensorValue.un.gravity.y;
                            sensorData.bnoGravityZ = bnoSensorValue.un.gravity.z;
                            break;
                        case SH2_GEOMAGNETIC_ROTATION_VECTOR:
                            sensorData.bnoQuatR = bnoSensorValue.un.geoMagRotationVector.real;
                            sensorData.bnoQuatI = bnoSensorValue.un.geoMagRotationVector.i;
                            sensorData.bnoQuatJ = bnoSensorValue.un.geoMagRotationVector.j;
                            sensorData.bnoQuatK = bnoSensorValue.un.geoMagRotationVector.k;
                            break;
                        default: ;
                    }
                }
                xSemaphoreGive(xSpiMutex);
            }

            xSemaphoreGive(xSensorDataMutex);
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

void insTask(void *pvParameters) {
    for (;;) {
        if (xSemaphoreTake(xSensorDataMutex, portMAX_DELAY) == pdTRUE) {
            // --- INS Model Input Map ---
            insInputs.GPS_LL[0] = sensorData.gpsLatitude;
            insInputs.GPS_LL[1] = sensorData.gpsLongitude;
            insInputs.BMP_Altitude = sensorData.bmpAltitude;

            insInputs.BNO_Quaternion[0] = sensorData.bnoQuatR;
            insInputs.BNO_Quaternion[1] = sensorData.bnoQuatI;
            insInputs.BNO_Quaternion[2] = sensorData.bnoQuatJ;
            insInputs.BNO_Quaternion[3] = sensorData.bnoQuatK;

            insInputs.BNO_Lin_Accel[0] = sensorData.bnoLinearAccX;
            insInputs.BNO_Lin_Accel[1] = sensorData.bnoLinearAccY;
            insInputs.BNO_Lin_Accel[2] = sensorData.bnoLinearAccZ;

            // --- REFERENCE CHECK ---
            // Only lock the reference if GPS is valid, and it hasn't locked it yet
            if (!insRefInitialized && sensorData.gpsValid) {
                // Don't lock to (0,0) coordinates
                if (abs(sensorData.gpsLatitude) > 0.001f) {
                    insInputs.LL_ref[0] = sensorData.gpsLatitude;
                    insInputs.LL_ref[1] = sensorData.gpsLongitude;
                    insInputs.h_ref = sensorData.bmpAltitude;
                    insRefInitialized = true;
                    Serial.println("INS: Launch Reference Locked.");
                }
            }

            // If not initialised, ensure we don't pass garbage data
            if (!insRefInitialized) {
                 insInputs.LL_ref[0] = 0.0;
                 insInputs.LL_ref[1] = 0.0;
                 insInputs.h_ref = 0.0;
            }

            // --- NaN Protection ---
            // Calculate Magnitude of Quaternion: R^2 + I^2 + J^2 + K^2
            const float quatNorm = (sensorData.bnoQuatR * sensorData.bnoQuatR) +
                             (sensorData.bnoQuatI * sensorData.bnoQuatI) +
                             (sensorData.bnoQuatJ * sensorData.bnoQuatJ) +
                             (sensorData.bnoQuatK * sensorData.bnoQuatK);

            // Only step the model if Reference is Locked and Quaternion is valid
            if (insRefInitialized && quatNorm > 0.001f) {
                insModel.setExternalInputs(&insInputs);
                insModel.step();
                insOutputs = insModel.getExternalOutputs();
            }

            xSemaphoreGive(xSensorDataMutex);
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

void gpsTask(void *pvParameters) {
    for (;;) {
        if (myGNSS.getPVT()) {
            if (xSemaphoreTake(xSensorDataMutex, portMAX_DELAY) == pdTRUE) {
                const long rawLat = myGNSS.getLatitude();
                const long rawLon = myGNSS.getLongitude();
                if (rawLat != 0 || rawLon != 0) {
                    sensorData.gpsLatitude = static_cast<float>(rawLat) / 10000000.0f;
                    sensorData.gpsLongitude = static_cast<float>(rawLon) / 10000000.0f;
                    sensorData.gpsAltitude = static_cast<float>(myGNSS.getAltitude()) / 1000.0f;
                    sensorData.gpsSpeed = static_cast<float>(myGNSS.getGroundSpeed()) / 1000.0f;
                    sensorData.gpsHeading = static_cast<float>(myGNSS.getHeading()) / 100000.0f;
                    sensorData.gpsValid = true;
                }
                xSemaphoreGive(xSensorDataMutex);
            }
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

void loggingTask(void *pvParameters) {
    unsigned long lastFlush = 0;

    for (;;) {
        AllSensorData local = {};
        INS_Model_C::ExtY_INS_Model_C_T localIns = {};
        
        if (xSemaphoreTake(xSensorDataMutex, portMAX_DELAY) == pdTRUE) {
            memcpy(&local, &sensorData, sizeof(AllSensorData));
            localIns = insOutputs;
            xSemaphoreGive(xSensorDataMutex);
        }

        if (loggingEnabled) {
            if (dataFile) {
                dataFile.print(millis());
                dataFile.print(","); dataFile.print(local.bmpTemperature, 2);
                dataFile.print(","); dataFile.print(local.bmpPressure, 2);
                dataFile.print(","); dataFile.print(local.bmpAltitude, 2);
                dataFile.print(","); dataFile.print(local.bmpVerticalSpeed, 2);
                /*
                dataFile.print(","); dataFile.print(local.icmAccX, 3);
                dataFile.print(","); dataFile.print(local.icmAccY, 3);
                dataFile.print(","); dataFile.print(local.icmAccZ, 3);
                dataFile.print(","); dataFile.print(local.icmGyrX, 2);
                dataFile.print(","); dataFile.print(local.icmGyrY, 2);
                dataFile.print(","); dataFile.print(local.icmGyrZ, 2);
                dataFile.print(","); dataFile.print(local.icmMagX, 1);
                dataFile.print(","); dataFile.print(local.icmMagY, 1);
                dataFile.print(","); dataFile.print(local.icmMagZ, 1);
                */
                dataFile.print(","); dataFile.print(local.bnoLinearAccX, 3);
                dataFile.print(","); dataFile.print(local.bnoLinearAccY, 3);
                dataFile.print(","); dataFile.print(local.bnoLinearAccZ, 3);
                dataFile.print(","); dataFile.print(local.bnoGravityX, 3);
                dataFile.print(","); dataFile.print(local.bnoGravityY, 3);
                dataFile.print(","); dataFile.print(local.bnoGravityZ, 3);
                dataFile.print(","); dataFile.print(local.bnoQuatR, 4);
                dataFile.print(","); dataFile.print(local.bnoQuatI, 4);
                dataFile.print(","); dataFile.print(local.bnoQuatJ, 4);
                dataFile.print(","); dataFile.print(local.bnoQuatK, 4);
                dataFile.print(","); dataFile.print(local.gpsLatitude, 6);
                dataFile.print(","); dataFile.print(local.gpsLongitude, 6);
                dataFile.print(","); dataFile.print(local.gpsAltitude, 2);
                dataFile.print(","); dataFile.print(local.gpsSpeed, 2);
                dataFile.print(","); dataFile.print(local.gpsHeading, 2);
                dataFile.print(","); dataFile.print(localIns.X_Estimate, 2);
                dataFile.print(","); dataFile.print(localIns.Y_Estimate, 2);
                dataFile.print(","); dataFile.print(localIns.Z_Estimate, 2);
                dataFile.print(","); dataFile.print(localIns.Lat_Estimate, 7);
                dataFile.println(localIns.Long_Estimate, 7);
            }

            if (millis() - lastFlush >= 1000) {
                lastFlush = millis();
                if(dataFile) dataFile.flush();
            }
        }
        
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

void buttonTask(void *pvParameters) {
    int lastState = LOW;
    unsigned long lastDebounce = 0;
    for (;;) {
        const int reading = digitalRead(buttonPin);
        if (reading != lastState) lastDebounce = millis();

        if ((millis() - lastDebounce) > 50) {
            if (reading == HIGH) {
                loggingEnabled = !loggingEnabled;
                if (loggingEnabled) {
                    Serial.println("Log Resumed");
                } else {
                    if(dataFile) {
                        dataFile.flush();
                    }
                    Serial.println("Log Stopped");
                }
                // Wait for release
                while(digitalRead(buttonPin) == HIGH) vTaskDelay(10);
            }
        }
        lastState = reading;
        vTaskDelay(pdMS_TO_TICKS(20));
    }
}

void webServerTask(void *pvParameters) {
    for (;;) {
        server.handleClient();
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

void loraTask(void *pvParameters) {
    // Enable Hardware CRC to filter noise at the radio level
    if (xSemaphoreTake(xSpiMutex, portMAX_DELAY) == pdTRUE) {
        LoRa.enableCrc();
        xSemaphoreGive(xSpiMutex);
    }

    vTaskDelay(pdMS_TO_TICKS(2000));

    for (;;) {
        TelemetryPacket packet{};

        // 1. Thread-safe data copy
        if (xSemaphoreTake(xSensorDataMutex, portMAX_DELAY) == pdTRUE) {
            packet.altitude = sensorData.bmpAltitude;
            packet.vSpeed   = sensorData.bmpVerticalSpeed;
            packet.lat      = sensorData.gpsLatitude;
            packet.lon      = sensorData.gpsLongitude;
            packet.qR       = sensorData.bnoQuatR;
            packet.qI       = sensorData.bnoQuatI;
            packet.qJ       = sensorData.bnoQuatJ;
            packet.qK       = sensorData.bnoQuatK;
            packet.insX     = static_cast<float>(insOutputs.X_Estimate);
            packet.insY     = static_cast<float>(insOutputs.Y_Estimate);
            packet.insZ     = static_cast<float>(insOutputs.Z_Estimate);
            xSemaphoreGive(xSensorDataMutex);
        }

        // 2. Transmit Binary with synchronous blocking
        if (xSemaphoreTake(xSpiMutex, portMAX_DELAY) == pdTRUE) {
            if (LoRa.beginPacket()) {
                LoRa.write(reinterpret_cast<uint8_t *>(&packet), sizeof(packet));
                LoRa.endPacket();
            }
            xSemaphoreGive(xSpiMutex);
        }

        vTaskDelay(pdMS_TO_TICKS(20));
    }
}

void loop() {
    // Empty
}