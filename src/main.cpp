// ReSharper disable All
#include "ICM42688.h"
#include <Wire.h>
#include <SPI.h>
#include <SD.h>
#include <FS.h>
#include <Adafruit_Sensor.h>
#include "Adafruit_BMP5xx.h"
#include <SparkFun_MMC5983MA_Arduino_Library.h>
#include <SparkFun_u-blox_GNSS_Arduino_Library.h>
#include <LoRa.h>

#include <FreeRTOS.h>
#include <task.h>
#include <semphr.h>
#include <WiFi.h>
#include <WebServer.h>
#include <ArduinoJson.h>

#include "USB.h"
#include "USBMSC.h"

extern "C" {
  #include "driver/sdspi_host.h"
  #include "driver/spi_common.h"
  #include "sdmmc_cmd.h"
  #include "esp_system.h"
}

#include "esp_sleep.h"

constexpr int CS_ICM = 3;
constexpr int CS_BMP = 41;
constexpr int CS_MMC = 45;
constexpr int RFM95_CS = 5;
constexpr int RFM95_RST = 14;
constexpr int RFM95_INT = 15;
constexpr int CS_SD = 40;
constexpr int SD_MOSI = 39;
constexpr int SD_MISO = 38;
constexpr int SD_CLK = 47;
constexpr long BAND = 868E6;
constexpr int BUTTON_PIN = 16;
constexpr int RED_LED = 26;
constexpr int GREEN_LED = 48;
constexpr int BLUE_LED = 21;
constexpr int VEGA_UART_TX = 43;
constexpr int VEGA_UART_RX = 44;
constexpr uint32_t VEGA_UART_BAUD = 115200;

bool inMSCMode = false;
bool loggingActive = false;
RTC_DATA_ATTR uint32_t bootToMSC = 0;
bool longPressHandled = false;

constexpr uint32_t AUTO_SHUTDOWN_AFTER_LANDING_MS = 2UL * 60UL * 1000UL;
constexpr uint32_t AUTO_SHUTDOWN_AFTER_LAUNCH_MS = 10UL * 60UL * 1000UL;
constexpr uint32_t AUTO_SHUTDOWN_FROM_POWER_ON_MS = 60UL * 60UL * 1000UL;
volatile uint32_t launchDetectedMillis = 0;
volatile uint32_t landedDetectedMillis = 0;
volatile bool autoShutdownIssued = false;

SPIClass* sdcardSPI = nullptr;

ICM42688* IMU = nullptr;
Adafruit_BMP5xx bmp;
SFE_MMC5983MA myMag;
SFE_UBLOX_GNSS myGNSS;

String ap_ssid = "LIFTSv2";
String ap_password = "123456789";
WebServer webServer(80);

float seaLevelPressureHpa = 1013.25f;

uint32_t sensorSampleRateMs = 10;
uint32_t gpsSampleRateMs = 1000;
uint32_t logFlushIntervalMs = 1000;

uint8_t ledBrightnessHeartbeat = 40;
uint8_t ledBrightnessIdle = 20;

float launchThresholdMultiplier = 5.0f;
float burnoutThresholdMultiplier = 2.0f;
int launchStreakRequired = 2;
int burnoutStreakRequired = 5;
int apogeeStreakRequired = 8;
int landStreakRequired = 10;
float landingAltitudeThresholdFt = 15.0f;

SemaphoreHandle_t sensorSpiMutex = nullptr;
SemaphoreHandle_t sdSpiMutex = nullptr;
SemaphoreHandle_t countMutex = nullptr;
SemaphoreHandle_t gpsDataMutex = nullptr;
SemaphoreHandle_t telemetryMutex = nullptr;
SemaphoreHandle_t vegaMutex = nullptr;

volatile uint32_t pollCount = 0;
volatile bool stopSensorTask = false;
volatile bool stopSdTask = false;
volatile bool stopGpsTask = false;
volatile bool stopFlightPhaseTask = false;
volatile bool sensorTaskStopped = false;
volatile bool sdLogTaskStopped = false;
volatile bool gpsTaskStopped = false;
volatile bool flightPhaseTaskStopped = false;
volatile bool stopVegaTask = false;
volatile bool vegaTaskStopped = false;
volatile bool vegaStartCommandSent = false;
volatile bool vegaStopCommandSent = false;
volatile bool vegaOn = false;

USBMSC msc;
sdmmc_card_t* mscCard = nullptr;

int lastButtonState = LOW;
unsigned long pressStartTime = 0;
constexpr unsigned long debounceLockout = 200;

struct GpsSample {
  uint32_t fixMillis;
  bool valid;
  long rawLat, rawLon, rawAlt, rawSpeed, rawHeading;
  float latitude, longitude, altitude, speed, heading;
};

struct SensorSample {
  uint32_t sampleMillis;
  float accelX, accelY, accelZ;
  float gyroX, gyroY, gyroZ;
  float imuTemp;
  float bmpTemp, bmpPressure, bmpAltitude, bmpVSpeed;
  uint32_t magX, magY, magZ;
  GpsSample gps;
  int flightPhase;
};

struct __attribute__((packed)) TelemetryPacket {
  float altitude;
  float vSpeed;
  float lat;
  float lon;
  float qR;
  float qI;
  float qJ;
  float qK;
  float insX;
  float insY;
  float insZ;
  int32_t flightPhase;
};

static_assert(sizeof(TelemetryPacket) == 48, "TelemetryPacket must remain 48 bytes");

QueueHandle_t logQueue = nullptr;
File logFile;
char currentFilename[32];
GpsSample latestGpsSample{};
SensorSample latestSensorSample{};
bool haveLastBmpAltitude = false;
float lastBmpAltitude = 0.0f;
uint32_t lastBmpAltitudeMillis = 0;
char vegaStatusText[32] = "OFF";
char vegaLastUartMessage[48] = "NONE";
uint32_t vegaLastUartMillis = 0;

bool initGPS();
bool initLoRa();
void startLogging();
void stopLogging();
void VegaUartTask(void* pvParameters);


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
          _chuteHead(0), _chuteCount(0), _chuteWarmup(0), _landStreak(0) {
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

    int feed(float accel_mag, float alt_ft, float accel_baro) {
        const int prev = _phase;

        if (!_baselineDone) {
            if (_blN >= 3) {
                const float runningMean = _blAccelSum / static_cast<float>(_blN);
                if (accel_mag > runningMean * 5.0f) {
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

        if (_phase == 0) {
            const float thresh = _blAccelMean + launchThresholdMultiplier * _blAccelStd;
            _launchStreak = (accel_mag > thresh) ? (_launchStreak + 1) : 0;
            if (_launchStreak >= launchStreakRequired) _phase = 1;
        }
        else if (_phase == 1) {
            const float thresh = _blAccelMean + burnoutThresholdMultiplier * _blAccelStd;
            _burnoutStreak = (accel_mag < thresh) ? (_burnoutStreak + 1) : 0;
            if (_burnoutStreak >= burnoutStreakRequired) _phase = 2;
        }
        else if (_phase == 2) {
            if (alt_ft >= _apogeePeak) { _apogeePeak = alt_ft; _apogeeStreak = 0; }
            else                       { _apogeeStreak++; }
            if (_apogeeStreak >= apogeeStreakRequired) _phase = 3;
        }
        else if (_phase == 3) {
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
            _landStreak = (fabsf(alt_ft - _blAltMean) < landingAltitudeThresholdFt) ? (_landStreak + 1) : 0;
            if (_landStreak >= landStreakRequired) _phase = 5;
        }

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
    float _chuteBuf[CHUTE_BUF_SZ]{};
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

FlightDetector flightDetector{};
SemaphoreHandle_t flightPhaseMutex = nullptr;

namespace {

template <int N> struct PriorityTag : PriorityTag<N - 1> {};
template <> struct PriorityTag<0> {};

#define DEFINE_SENSOR_READER(FuncName, Member1, Member2, Member3, Member4) \
  template <typename T> \
  auto FuncName##_impl(T& obj, PriorityTag<8>) -> decltype((void)obj.Member1(), float()) { return obj.Member1(); } \
  template <typename T> \
  auto FuncName##_impl(T& obj, PriorityTag<7>) -> decltype((void)obj.Member1, float()) { return obj.Member1; } \
  template <typename T> \
  auto FuncName##_impl(T& obj, PriorityTag<6>) -> decltype((void)obj.Member2(), float()) { return obj.Member2(); } \
  template <typename T> \
  auto FuncName##_impl(T& obj, PriorityTag<5>) -> decltype((void)obj.Member2, float()) { return obj.Member2; } \
  template <typename T> \
  auto FuncName##_impl(T& obj, PriorityTag<4>) -> decltype((void)obj.Member3(), float()) { return obj.Member3(); } \
  template <typename T> \
  auto FuncName##_impl(T& obj, PriorityTag<3>) -> decltype((void)obj.Member3, float()) { return obj.Member3; } \
  template <typename T> \
  auto FuncName##_impl(T& obj, PriorityTag<2>) -> decltype((void)obj.Member4(), float()) { return obj.Member4(); } \
  template <typename T> \
  auto FuncName##_impl(T& obj, PriorityTag<1>) -> decltype((void)obj.Member4, float()) { return obj.Member4; } \
  inline float FuncName##_impl(...) { return 0.0f; } \
  template <typename T> \
  float FuncName(T& obj) { return FuncName##_impl(obj, PriorityTag<8>{}); }

DEFINE_SENSOR_READER(readImuAccelX, accX, accelX, aX, AccelX)
DEFINE_SENSOR_READER(readImuAccelY, accY, accelY, aY, AccelY)
DEFINE_SENSOR_READER(readImuAccelZ, accZ, accelZ, aZ, AccelZ)
DEFINE_SENSOR_READER(readImuGyroX, gyrX, gyroX, gX, GyroX)
DEFINE_SENSOR_READER(readImuGyroY, gyrY, gyroY, gY, GyroY)
DEFINE_SENSOR_READER(readImuGyroZ, gyrZ, gyroZ, gZ, GyroZ)
DEFINE_SENSOR_READER(readImuTemp, temp, temperature, Temperature, Temp)

#undef DEFINE_SENSOR_READER

}

void SensorPollTask(void* pvParameters);
void SdLogTask(void* pvParameters);
void GpsTask(void* pvParameters);
void LoRaTask(void* pvParameters);
void FlightPhaseTask(void* pvParameters);
void PrintTask(void* pvParameters);
void ControlTask(void* pvParameters);
void setupLoggerMode();
void setupMSCMode();
void webServerTask(void* pvParameters);

void handleRoot();
void handleGetCsvList();
void handleFileDownload();
void handleGetConfig();
void handleUpdateConfig();
void handleNotFound();

void saveConfiguration();
void loadConfiguration();

bool hasElapsed(const uint32_t now, const uint32_t start, const uint32_t duration) {
  return static_cast<uint32_t>(now - start) >= duration;
}

void setRGB(uint8_t r, uint8_t g, uint8_t b) {
  analogWrite(RED_LED, r);
  analogWrite(GREEN_LED, g);
  analogWrite(BLUE_LED, b);
}

void ledHeartbeat() {
  unsigned long t = millis() % 2000;
  unsigned long brightness_ul = (t < 1000UL) ? (t * static_cast<unsigned long>(ledBrightnessHeartbeat) / 1000UL) : ((2000UL - t) * static_cast<unsigned long>(ledBrightnessHeartbeat) / 1000UL);
  setRGB(0, static_cast<uint8_t>(brightness_ul), 0);
}

void flashYellow() {
  setRGB(150, 80, 0);
  delay(250);
}

[[noreturn]] void blinkRedForever() {
  for (;;) {
    setRGB(150, 0, 0);
    delay(300);
    setRGB(0, 0, 0);
    delay(300);
  }
}

uint32_t findNextLogFileIndex() {
  uint32_t fileIndex = 0;
  char candidate[32];
  while (true) {
    snprintf(candidate, sizeof(candidate), "/log_%lu.csv", static_cast<unsigned long>(fileIndex));
    if (!SD.exists(candidate)) return fileIndex;
    fileIndex++;
  }
}

void initVegaUart();
void setVegaStateLocked(bool on, const char* statusText, const char* lastMessage, uint32_t messageMillis);
void readVegaStateSnapshot(bool& on, char* statusText, size_t statusTextSize, char* lastMessage, size_t lastMessageSize, uint32_t& messageMillis);
void logVegaEvent(uint32_t whenMillis, const char* direction, const char* message);
void processVegaMessage(const char* message, uint32_t whenMillis);
void sendVegaCommand(const char* command, bool markStart, bool markStop, const char* statusText);
void requestVegaStart(uint32_t whenMillis);
void requestVegaStop(uint32_t whenMillis);

bool initGPS() {
  Wire.begin();

  if (myGNSS.begin() == false) {
    Serial.println(F("u-blox GNSS not detected at default I2C address. Please check wiring. Freezing."));
    return false;
  }

  Serial.println(F("u-blox GNSS Initialized Successfully."));
  return true;
}

bool initLoRa() {
  pinMode(RFM95_CS, OUTPUT);
  digitalWrite(RFM95_CS, HIGH);
  pinMode(RFM95_RST, OUTPUT);
  digitalWrite(RFM95_RST, HIGH);
  pinMode(RFM95_INT, INPUT);

  LoRa.setSPI(SPI);
  LoRa.setPins(RFM95_CS, RFM95_RST, RFM95_INT);

  if (!LoRa.begin(BAND)) {
    Serial.println(F("LoRa initialization failed. Check wiring/PCB."));
    return false;
  }

  LoRa.setTxPower(20);
  LoRa.setSignalBandwidth(500E3);
  LoRa.setSpreadingFactor(7);
  LoRa.setCodingRate4(5);
  LoRa.enableCrc();

  Serial.println(F("LoRa Initialized. Beginning telemetry transmission."));
  return true;
}

bool initSDForMSC() {
  spi_bus_config_t bus_cfg = {};
  bus_cfg.mosi_io_num = SD_MOSI;
  bus_cfg.miso_io_num = SD_MISO;
  bus_cfg.sclk_io_num = SD_CLK;
  bus_cfg.quadwp_io_num = -1;
  bus_cfg.quadhd_io_num = -1;
  bus_cfg.max_transfer_sz = 4096;

  esp_err_t ret = spi_bus_initialize(SPI3_HOST, &bus_cfg, SDSPI_DEFAULT_DMA);
  if (ret != ESP_OK && ret != ESP_ERR_INVALID_STATE) {
    Serial.printf("spi_bus_initialize failed: %s\n", esp_err_to_name(ret));
    return false;
  }

  ret = sdspi_host_init();
  if (ret != ESP_OK && ret != ESP_ERR_INVALID_STATE) {
    Serial.printf("sdspi_host_init failed: %s\n", esp_err_to_name(ret));
    return false;
  }

  sdspi_device_config_t dev_cfg = SDSPI_DEVICE_CONFIG_DEFAULT();
  dev_cfg.gpio_cs = static_cast<gpio_num_t>(CS_SD);
  dev_cfg.host_id = SPI3_HOST;

  sdspi_dev_handle_t handle;
  ret = sdspi_host_init_device(&dev_cfg, &handle);
  if (ret != ESP_OK) {
    Serial.printf("sdspi_host_init_device failed: %s\n", esp_err_to_name(ret));
    return false;
  }

  sdmmc_host_t host = SDSPI_HOST_DEFAULT();
  host.slot = handle;
  host.max_freq_khz = 20000;

  mscCard = static_cast<sdmmc_card_t*>(malloc(sizeof(sdmmc_card_t)));
  if (!mscCard) return false;

  ret = sdmmc_card_init(&host, mscCard);
  if (ret != ESP_OK) {
    Serial.printf("sdmmc_card_init failed: %s\n", esp_err_to_name(ret));
    free(mscCard);
    mscCard = nullptr;
    return false;
  }

  Serial.printf("SD card initialised for MSC: %llu sectors of %u bytes\n", static_cast<unsigned long long>(mscCard->csd.capacity), static_cast<unsigned>(mscCard->csd.sector_size));
  return true;
}

static int32_t mscOnRead(uint32_t lba, uint32_t offset, void* buffer, uint32_t bufsize) {
  if (!mscCard) return -1;
  if (sdmmc_read_sectors(mscCard, buffer, lba, bufsize / 512) != ESP_OK) return -1;
  return static_cast<int32_t>(bufsize);
}

static int32_t mscOnWrite(uint32_t lba, uint32_t offset, const uint8_t* buffer, uint32_t bufsize) {
  if (!mscCard) return -1;
  if (sdmmc_write_sectors(mscCard, buffer, lba, bufsize / 512) != ESP_OK) return -1;
  return static_cast<int32_t>(bufsize);
}

static bool mscOnStartStop(uint8_t power_condition, bool start, bool load_eject) {
  (void)power_condition; (void)start; (void)load_eject;
  return true;
}

void initVegaUart() {
  if (!vegaMutex) {
    vegaMutex = xSemaphoreCreateMutex();
    if (!vegaMutex) {
      Serial.println("Failed to create VEGA mutex");
      setRGB(150, 0, 0);
      while (true) delay(10);
    }
  }

  Serial1.begin(VEGA_UART_BAUD, SERIAL_8N1, VEGA_UART_RX, VEGA_UART_TX);
  Serial.printf("VEGA UART initialised at %lu baud on RX=%d TX=%d\n",
                static_cast<unsigned long>(VEGA_UART_BAUD), VEGA_UART_RX, VEGA_UART_TX);

  setVegaStateLocked(false, "OFF", "NONE", 0);
  vegaStartCommandSent = false;
  vegaStopCommandSent = false;
}

void setVegaStateLocked(bool on, const char* statusText, const char* lastMessage, uint32_t messageMillis) {
  if (!vegaMutex) return;
  if (xSemaphoreTake(vegaMutex, portMAX_DELAY) != pdTRUE) return;

  vegaOn = on;
  snprintf(vegaStatusText, sizeof(vegaStatusText), "%s", statusText ? statusText : "OFF");
  if (lastMessage && lastMessage[0] != '\0') {
    snprintf(vegaLastUartMessage, sizeof(vegaLastUartMessage), "%s", lastMessage);
    vegaLastUartMillis = messageMillis;
  }

  xSemaphoreGive(vegaMutex);
}

void readVegaStateSnapshot(bool& on, char* statusText, size_t statusTextSize, char* lastMessage, size_t lastMessageSize, uint32_t& messageMillis) {
  on = false;
  snprintf(statusText, statusTextSize, "%s", "OFF");
  snprintf(lastMessage, lastMessageSize, "%s", "NONE");
  messageMillis = 0;

  if (!vegaMutex) return;
  if (xSemaphoreTake(vegaMutex, portMAX_DELAY) != pdTRUE) return;

  on = vegaOn;
  snprintf(statusText, statusTextSize, "%s", vegaStatusText);
  snprintf(lastMessage, lastMessageSize, "%s", vegaLastUartMessage);
  messageMillis = vegaLastUartMillis;

  xSemaphoreGive(vegaMutex);
}

void logVegaEvent(uint32_t whenMillis, const char* direction, const char* message) {
  (void)whenMillis;
  (void)direction;
  (void)message;
}

void processVegaMessage(const char* message, uint32_t whenMillis) {
  if (!message || message[0] == '\0') return;

  logVegaEvent(whenMillis, "RX", message);

  bool on = true;
  const char* statusText = "UNKNOWN";

  if (strstr(message, "VEGA_SAVED_AND_STOPPED") || strstr(message, "VEGA_DATA_SECURE_INITIATING_SHUTDOWN") || strstr(message, "VEGA_OFF")) {
    on = false;
    statusText = "OFF";
  } else if (strstr(message, "VEGA_STOP_ACKNOWLEDGE") || strstr(message, "VEGA_STOP_ACKNOWLEDGED")) {
    on = true;
    statusText = "STOPPING";
  } else if (strstr(message, "VEGA_SAVING_DATA")) {
    on = true;
    statusText = "SAVING";
  } else if (strstr(message, "VEGA_FAILSAFE_TIMEOUT_STOP")) {
    on = true;
    statusText = "TIMEOUT_STOPPING";
  } else if (strstr(message, "VEGA_INITIALISING_CAMERAS")) {
    on = true;
    statusText = "CAM_INIT";
  } else if (strstr(message, "VEGA_RECORDING_STARTED") || strstr(message, "VEGA_RECORDING") || strstr(message, "VEGA_ACTIVE_T+")) {
    on = true;
    statusText = "RECORDING";
  } else if (strstr(message, "VEGA_START_ACKNOWLEDGED") || strstr(message, "VEGA_FAILSAFE_LAUNCH_OVERRIDE") || strstr(message, "VEGA_STARTED")) {
    on = true;
    statusText = "START_PENDING";
  } else {
    statusText = "UNRECOGNISED";
  }

  setVegaStateLocked(on, statusText, message, whenMillis);
}

void sendVegaCommand(const char* command, bool markStart, bool markStop, const char* statusText) {
  if (!command || command[0] == '\0') return;

  if (markStart && vegaStartCommandSent) return;
  if (markStop && vegaStopCommandSent) return;

  Serial.print("UART->VEGA: ");
  Serial.println(command);
  Serial1.println(command);
  Serial1.flush();

  const uint32_t now = millis();
  logVegaEvent(now, "TX", command);

  if (markStart) vegaStartCommandSent = true;
  if (markStop) vegaStopCommandSent = true;

  setVegaStateLocked(true, statusText ? statusText : "PENDING", command, now);
}

void requestVegaStart(uint32_t whenMillis) {
  (void)whenMillis;
  if (vegaStartCommandSent) return;
  sendVegaCommand("VEGA_STARTED", true, false, "START_SENT");
}

void requestVegaStop(uint32_t whenMillis) {
  (void)whenMillis;
  if (vegaStopCommandSent || (!vegaStartCommandSent && !vegaOn)) return;
  sendVegaCommand("STOP_VEGA", false, true, "STOP_SENT");
}

void setupMSCMode() {
  Serial.println("=== Entering MSC mode ===");
  setRGB(50, 25, 0);

  if (!initSDForMSC()) {
    Serial.println("SD initialisation for MSC failed");
    setRGB(150, 0, 0);
    blinkRedForever();
  }

  msc.vendorID("LIFTSv2");
  msc.productID("Logger");
  msc.productRevision("1.0");
  msc.onRead(mscOnRead);
  msc.onWrite(reinterpret_cast<msc_write_cb>(mscOnWrite));
  msc.onStartStop(mscOnStartStop);
  msc.mediaPresent(true);
  msc.begin(mscCard->csd.capacity, mscCard->csd.sector_size);

  USB.begin();

  Serial.println("MSC ready. Plug into host PC to access files.");
  Serial.println("Hold during boot to enter MSC. Restart device to exit MSC mode.");
  setRGB(0, 0, 80);
}

void setupLoggerMode() {
  Serial.println("=== Entering LOGGER mode ===");
  setRGB(50, 25, 0);

  pinMode(CS_ICM, OUTPUT); digitalWrite(CS_ICM, HIGH);
  pinMode(CS_BMP, OUTPUT); digitalWrite(CS_BMP, HIGH);
  pinMode(CS_MMC, OUTPUT); digitalWrite(CS_MMC, HIGH);
  pinMode(RFM95_CS, OUTPUT); digitalWrite(RFM95_CS, HIGH);
  pinMode(CS_SD, OUTPUT); digitalWrite(CS_SD, HIGH);

  SPI.begin();

  sdcardSPI = new SPIClass(HSPI);
  sdcardSPI->begin(SD_CLK, SD_MISO, SD_MOSI, CS_SD);
  if (!SD.begin(CS_SD, *sdcardSPI, 4000000)) {
    Serial.println("Warning: SD card initialisation unsuccessful (web server may be limited). Will retry when logging starts.");
  } else {
    loadConfiguration();
  }
  if (!IMU) IMU = new ICM42688(SPI, CS_ICM);
  int status = IMU->begin();
  if (status < 0) {
    Serial.println("Failed: ICM42688 initialisation unsuccessful.");
    setRGB(150, 0, 0);
    blinkRedForever();
  }

  if (!bmp.begin(CS_BMP, &SPI)) {
    Serial.println("Failed: BMP5xx initialisation unsuccessful.");
    setRGB(150, 0, 0);
    blinkRedForever();
  }

  bmp.setTemperatureOversampling(BMP5XX_OVERSAMPLING_2X);
  bmp.setPressureOversampling(BMP5XX_OVERSAMPLING_16X);
  bmp.setIIRFilterCoeff(BMP5XX_IIR_FILTER_COEFF_3);
  bmp.setOutputDataRate(BMP5XX_ODR_100_2_HZ);
  bmp.setPowerMode(BMP5XX_POWERMODE_CONTINUOUS);

  if (myMag.begin(CS_MMC) == false) {
    Serial.println("Failed: MMC5983MA initialisation unsuccessful.");
    setRGB(150, 0, 0);
    blinkRedForever();
  }
  myMag.softReset();
  delay(20);
  myMag.enableAutomaticSetReset();
  myMag.setContinuousModeFrequency(100);
  myMag.enableContinuousMode();

  if (!initGPS()) {
    Serial.println("Failed: GNSS initialisation unsuccessful.");
    setRGB(150, 0, 0);
    blinkRedForever();
  }

  const bool loraReady = initLoRa();
  if (!loraReady) {
    Serial.println("Warning: LoRa initialisation unsuccessful. Telemetry task not started.");
  }

  Serial.println("Success: All sensors initialised okay.");
  sensorSpiMutex = xSemaphoreCreateMutex();
  sdSpiMutex = xSemaphoreCreateMutex();
  countMutex = xSemaphoreCreateMutex();
  gpsDataMutex = xSemaphoreCreateMutex();
  telemetryMutex = xSemaphoreCreateMutex();
  flightPhaseMutex = xSemaphoreCreateMutex();
  if (!sensorSpiMutex || !sdSpiMutex || !countMutex || !gpsDataMutex || !telemetryMutex || !flightPhaseMutex) {
    Serial.println("Failed to create mutexes");
    setRGB(150, 0, 0);
    while (true) delay(10);
  }

  initVegaUart();

  BaseType_t r2 = xTaskCreate(PrintTask, "Printer", 2048, nullptr, tskIDLE_PRIORITY + 1, nullptr);
  if (r2 != pdPASS) {
    Serial.println("Failed to create PrintTask");
    setRGB(150, 0, 0);
    while (true) delay(10);
  }

  if (loraReady) {
    BaseType_t rlora = xTaskCreate(LoRaTask, "LoRaTx", 4096, nullptr, tskIDLE_PRIORITY + 1, nullptr);
    if (rlora != pdPASS) {
      Serial.println("Failed to create LoRaTask");
    }
  }

  webServer.on("/", HTTP_GET, handleRoot);
  webServer.on("/list-csv", HTTP_GET, handleGetCsvList);
  webServer.on("/download", HTTP_GET, handleFileDownload);
  webServer.on("/getconfig", HTTP_GET, handleGetConfig);
  webServer.on("/updateconfig", HTTP_POST, handleUpdateConfig);
  webServer.onNotFound(handleNotFound);

  WiFi.softAP(ap_ssid, ap_password);
  webServer.begin();
  BaseType_t rws = xTaskCreate(webServerTask, "WebSrv", 4096, nullptr, tskIDLE_PRIORITY + 1, nullptr);
  if (rws != pdPASS) {
    Serial.println("Failed to create WebServer task");
  } else {
    Serial.println("Web server started (AP mode). Connect to WiFi to access files.");
  }

  startLogging();

  Serial.println("Press button to stop/start logging, or hold button during boot for MSC.");
}

void startLogging() {
  if (loggingActive) return;

  Serial.println("Starting logging: initialising SD and creating tasks...");
  flashYellow();

  if (!SD.begin(CS_SD, *sdcardSPI, 4000000)) {
    Serial.println("Failed: SD card initialisation unsuccessful.");
    setRGB(150, 0, 0);
    blinkRedForever();
  }

  uint32_t fileIndex = findNextLogFileIndex();
  snprintf(currentFilename, sizeof(currentFilename), "/log_%lu.csv", static_cast<unsigned long>(fileIndex));

  logFile = SD.open(currentFilename, FILE_WRITE);
  if (!logFile) {
    Serial.println("Failed to open log file for writing.");
    setRGB(150, 0, 0);
    return;
  }

  logFile.println("millis,accelX,accelY,accelZ,gyroX,gyroY,gyroZ,imuTemp,bmpTemp,bmpPressure,bmpAltitude,bmpVSpeed,magX,magY,magZ,gpsFixMillis,gpsValid,gpsLatitude,gpsLongitude,gpsAltitude,gpsSpeed,gpsHeading,flightPhase,vegaOn,vegaStatus,vegaLastMessage,vegaLastMessageMillis");
  logFile.flush();
  Serial.print("Logging to file: ");
  Serial.println(currentFilename);

  logQueue = xQueueCreate(128, sizeof(SensorSample));
  if (logQueue == nullptr) {
    Serial.println("Failed to create logging queue");
    if (logFile) {
      logFile.close();
    }
    setRGB(150, 0, 0);
    return;
  }

  latestGpsSample = {};
  latestSensorSample = {};
  flightDetector.reset();
  haveLastBmpAltitude = false;
  lastBmpAltitude = 0.0f;
  lastBmpAltitudeMillis = 0;
  launchDetectedMillis = 0;
  landedDetectedMillis = 0;
  autoShutdownIssued = false;
  stopVegaTask = false;
  vegaTaskStopped = false;
  vegaStartCommandSent = false;
  vegaStopCommandSent = false;
  setVegaStateLocked(false, "OFF", "NONE", 0);
  stopSensorTask = false;
  stopSdTask = false;
  stopGpsTask = false;
  stopFlightPhaseTask = false;
  sensorTaskStopped = false;
  sdLogTaskStopped = false;
  gpsTaskStopped = false;
  flightPhaseTaskStopped = false;

  BaseType_t r0 = xTaskCreate(GpsTask, "GpsPoll", 4096, nullptr, tskIDLE_PRIORITY + 1, nullptr);
  BaseType_t r1 = xTaskCreate(SensorPollTask, "SensorPoll", 2048, nullptr, tskIDLE_PRIORITY + 2, nullptr);
  BaseType_t r3 = xTaskCreate(SdLogTask, "SDLog", 4096, nullptr, tskIDLE_PRIORITY + 1, nullptr);
  BaseType_t r4 = xTaskCreate(FlightPhaseTask, "FlightPhase", 2048, nullptr, tskIDLE_PRIORITY + 1, nullptr);
  BaseType_t r5 = xTaskCreate(VegaUartTask, "VegaUART", 3072, nullptr, tskIDLE_PRIORITY + 1, nullptr);

  if (r0 != pdPASS || r1 != pdPASS || r3 != pdPASS || r4 != pdPASS || r5 != pdPASS) {
    Serial.println("Failed to create FreeRTOS tasks");
    if (logFile) {
      logFile.close();
    }
    setRGB(150, 0, 0);
    return;
  }

  loggingActive = true;
}

void stopLogging() {
  if (!loggingActive) return;

  Serial.println("Stopping logging and closing file...");
  flashYellow();

  stopSensorTask = true;
  unsigned long t0 = millis();
  while (!sensorTaskStopped && (millis() - t0) < 2000) delay(10);

  stopGpsTask = true;
  t0 = millis();
  while (!gpsTaskStopped && (millis() - t0) < 3000) delay(10);

  stopFlightPhaseTask = true;
  t0 = millis();
  while (!flightPhaseTaskStopped && (millis() - t0) < 1000) delay(10);

  requestVegaStop(millis());
  stopVegaTask = true;
  t0 = millis();
  while (!vegaTaskStopped && (millis() - t0) < 2000) delay(10);

  stopSdTask = true;
  t0 = millis();
  while (!sdLogTaskStopped && (millis() - t0) < 3000) delay(10);

  SD.end();

  if (logQueue) {
    vQueueDelete(logQueue);
    logQueue = nullptr;
  }


  loggingActive = false;
  Serial.println("Logging stopped.");
  setRGB(0, 40, 0);
}

void setup() {
  pinMode(BUTTON_PIN, INPUT);
  pinMode(RED_LED, OUTPUT);
  pinMode(GREEN_LED, OUTPUT);
  pinMode(BLUE_LED, OUTPUT);
  setRGB(0, 0, 0);

  Serial.begin(115200);
  delay(200);
  USB.begin();

  Serial.println();
  Serial.print("Boot: checking button for MSC hold...");
  bool heldAtBoot = false;
  if (digitalRead(BUTTON_PIN) == HIGH) {
    delay(500);
    if (digitalRead(BUTTON_PIN) == HIGH) heldAtBoot = true;
  }

  if (bootToMSC || heldAtBoot) {
    inMSCMode = true;
    bootToMSC = 0;
  }

  if (inMSCMode) {
    Serial.println(" held -> MSC mode");
    setupMSCMode();
  } else {
    Serial.println(" -> Logger mode");
    setupLoggerMode();
  }

  xTaskCreatePinnedToCore(ControlTask, "CtrlTask", 4096, nullptr, 2, nullptr, 0);
}

void loop() {
}

void ControlTask(void* pvParameters) {
  for (;;) {
    int btn = digitalRead(BUTTON_PIN);
    unsigned long now = millis();
    if (btn == HIGH && lastButtonState == LOW) {
      pressStartTime = now;
      lastButtonState = HIGH;
      longPressHandled = false;
    } else if (btn == HIGH && lastButtonState == HIGH) {
      if (!longPressHandled && (now - pressStartTime >= 2000)) {
        longPressHandled = true;
        Serial.println("Long press detected: stopping logging and rebooting into MSC...");
        if (loggingActive) stopLogging();
        bootToMSC = 1;
        Serial.println("Rebooting now...");
        Serial.flush();
        esp_restart();
      }
    } else if (btn == LOW && lastButtonState == HIGH) {
      unsigned long pressDuration = now - pressStartTime;
      lastButtonState = LOW;
      if (pressDuration >= debounceLockout) {
        if (inMSCMode) {
          Serial.println("Button released while in MSC mode (ignored). Restart to exit MSC.");
        } else {
          if (!longPressHandled) {
            if (loggingActive) stopLogging(); else startLogging();
          }
        }
      }
    }

    if (inMSCMode) {
      setRGB(0, 0, 80);
    } else {
      if (loggingActive) ledHeartbeat(); else setRGB(0, ledBrightnessIdle, 0);

      if (!autoShutdownIssued) {
        const uint32_t nowMillis = millis();
        bool shutdownDue = false;

        // Priority: landed timeout first, launch fallback second, power-on fallback last.
        if (landedDetectedMillis != 0) {
          shutdownDue = hasElapsed(nowMillis, landedDetectedMillis, AUTO_SHUTDOWN_AFTER_LANDING_MS);
        } else if (launchDetectedMillis != 0) {
          shutdownDue = hasElapsed(nowMillis, launchDetectedMillis, AUTO_SHUTDOWN_AFTER_LAUNCH_MS);
        } else {
          shutdownDue = hasElapsed(nowMillis, 0, AUTO_SHUTDOWN_FROM_POWER_ON_MS);
        }

        if (shutdownDue) {
          autoShutdownIssued = true;
          Serial.println("Auto shutdown timeout reached. Entering deep sleep...");
          if (loggingActive) {
            stopLogging();
          }
          Serial.flush();
          esp_deep_sleep_start();
        }
      }
    }

    vTaskDelay(pdMS_TO_TICKS(20));
  }
}

void SensorPollTask(void* pvParameters) {
  TickType_t xFrequency = pdMS_TO_TICKS(sensorSampleRateMs);
  TickType_t xLastWakeTime = xTaskGetTickCount();

  for (;;) {
    if (stopSensorTask) {
      sensorTaskStopped = true;
      vTaskDelete(nullptr);
    }

    vTaskDelayUntil(&xLastWakeTime, xFrequency);

    SensorSample sample{};

    if (xSemaphoreTake(sensorSpiMutex, portMAX_DELAY) == pdTRUE) {
      if (IMU) IMU->getAGT();
      if (bmp.performReading()) {
        sample.accelX = IMU ? readImuAccelX(*IMU) : 0.0f;
        sample.accelY = IMU ? readImuAccelY(*IMU) : 0.0f;
        sample.accelZ = IMU ? readImuAccelZ(*IMU) : 0.0f;
        sample.gyroX = IMU ? readImuGyroX(*IMU) : 0.0f;
        sample.gyroY = IMU ? readImuGyroY(*IMU) : 0.0f;
        sample.gyroZ = IMU ? readImuGyroZ(*IMU) : 0.0f;
        sample.imuTemp = IMU ? readImuTemp(*IMU) : 0.0f;
        sample.bmpTemp = bmp.temperature;
        sample.bmpPressure = static_cast<float>(bmp.pressure) / 100.0f;
        sample.bmpAltitude = bmp.readAltitude(seaLevelPressureHpa);

        const uint32_t nowMillis = millis();
        sample.bmpVSpeed = 0.0f;
        if (haveLastBmpAltitude) {
          const uint32_t deltaMillis = nowMillis - lastBmpAltitudeMillis;
          if (deltaMillis > 0) {
            sample.bmpVSpeed = (sample.bmpAltitude - lastBmpAltitude) / (static_cast<float>(deltaMillis) / 1000.0f);
          }
        }
        lastBmpAltitude = sample.bmpAltitude;
        lastBmpAltitudeMillis = nowMillis;
        haveLastBmpAltitude = true;
      }
      myMag.readFieldsXYZ(&sample.magX, &sample.magY, &sample.magZ);
      myMag.clearMeasDoneInterrupt();
      xSemaphoreGive(sensorSpiMutex);
    }

    if (xSemaphoreTake(gpsDataMutex, portMAX_DELAY) == pdTRUE) {
      sample.gps = latestGpsSample;
      xSemaphoreGive(gpsDataMutex);
    }

    sample.sampleMillis = millis();

    if (telemetryMutex && xSemaphoreTake(telemetryMutex, portMAX_DELAY) == pdTRUE) {
      latestSensorSample = sample;
      xSemaphoreGive(telemetryMutex);
    }

    if (xSemaphoreTake(flightPhaseMutex, portMAX_DELAY) == pdTRUE) {
      sample.flightPhase = flightDetector.phase();
      xSemaphoreGive(flightPhaseMutex);
    }

    if (logQueue != nullptr) {
      xQueueSend(logQueue, &sample, pdMS_TO_TICKS(50));
    }

    if (xSemaphoreTake(countMutex, portMAX_DELAY) == pdTRUE) {
      pollCount++;
      xSemaphoreGive(countMutex);
    }
  }
}

void GpsTask(void* pvParameters) {
  TickType_t xFrequency = pdMS_TO_TICKS(gpsSampleRateMs);
  TickType_t xLastWakeTime = xTaskGetTickCount();

  for (;;) {
    if (stopGpsTask) {
      gpsTaskStopped = true;
      vTaskDelete(nullptr);
    }

    vTaskDelayUntil(&xLastWakeTime, xFrequency);

    if (myGNSS.getPVT()) {
      GpsSample sampleGps{};
      sampleGps.fixMillis = millis();
      sampleGps.rawLat = myGNSS.getLatitude();
      sampleGps.rawLon = myGNSS.getLongitude();
      sampleGps.rawAlt = myGNSS.getAltitude();
      sampleGps.rawSpeed = myGNSS.getGroundSpeed();
      sampleGps.rawHeading = myGNSS.getHeading();
      sampleGps.latitude = static_cast<float>(sampleGps.rawLat) / 10000000.0f;
      sampleGps.longitude = static_cast<float>(sampleGps.rawLon) / 10000000.0f;
      sampleGps.altitude = static_cast<float>(sampleGps.rawAlt) / 1000.0f;
      sampleGps.speed = static_cast<float>(sampleGps.rawSpeed) / 1000.0f;
      sampleGps.heading = static_cast<float>(sampleGps.rawHeading) / 100000.0f;
      sampleGps.valid = true;

      Serial.printf("GPS sample: lat=%.7f, lon=%.7f\n",
                    static_cast<double>(sampleGps.latitude),
                    static_cast<double>(sampleGps.longitude));

      if (xSemaphoreTake(gpsDataMutex, portMAX_DELAY) == pdTRUE) {
        latestGpsSample = sampleGps;
        xSemaphoreGive(gpsDataMutex);
      }
    }
    else {
      GpsSample sampleGps{};
      if (xSemaphoreTake(gpsDataMutex, portMAX_DELAY) == pdTRUE) {
        latestGpsSample = sampleGps;
        xSemaphoreGive(gpsDataMutex);
      }
    }
  }
}

void LoRaTask(void* pvParameters) {
  constexpr TickType_t txPeriod = pdMS_TO_TICKS(100);
  TickType_t lastWakeTime = xTaskGetTickCount();

  for (;;) {
    vTaskDelayUntil(&lastWakeTime, txPeriod);

    TelemetryPacket packet{};

    if (telemetryMutex && xSemaphoreTake(telemetryMutex, portMAX_DELAY) == pdTRUE) {
      packet.altitude = latestSensorSample.bmpAltitude;
      packet.vSpeed = latestSensorSample.bmpVSpeed;
      packet.flightPhase = static_cast<int32_t>(latestSensorSample.flightPhase);
      xSemaphoreGive(telemetryMutex);
    }

    if (xSemaphoreTake(gpsDataMutex, portMAX_DELAY) == pdTRUE) {
      if (latestGpsSample.valid) {
        packet.lat = latestGpsSample.latitude;
        packet.lon = latestGpsSample.longitude;
      }
      xSemaphoreGive(gpsDataMutex);
    }

    if (xSemaphoreTake(sensorSpiMutex, portMAX_DELAY) == pdTRUE) {
      if (LoRa.beginPacket()) {
        LoRa.write(reinterpret_cast<uint8_t*>(&packet), sizeof(packet));
        LoRa.endPacket();
      }
      xSemaphoreGive(sensorSpiMutex);
    }
  }
}

void FlightPhaseTask(void* pvParameters) {
  float prevVelMs  = 0.0f;
  unsigned long prevTimeMs = 0;

  for (;;) {
    if (stopFlightPhaseTask) {
      flightPhaseTaskStopped = true;
      vTaskDelete(nullptr);
    }

    float ax = 0.0f, ay = 0.0f, az = 0.0f;
    if (xSemaphoreTake(sensorSpiMutex, portMAX_DELAY) == pdTRUE) {
      if (IMU) IMU->getAGT();
      ax = IMU ? readImuAccelX(*IMU) : 0.0f;
      ay = IMU ? readImuAccelY(*IMU) : 0.0f;
      az = IMU ? readImuAccelZ(*IMU) : 0.0f;
      xSemaphoreGive(sensorSpiMutex);
    }
    const float accelMag = sqrtf(ax*ax + ay*ay + az*az);

     float altFt = 0.0f;
     float verticalSpeed = 0.0f;
     if (xSemaphoreTake(telemetryMutex, portMAX_DELAY) == pdTRUE) {
       altFt = latestSensorSample.bmpAltitude * 3.28084f;  // Convert meters to feet
       verticalSpeed = latestSensorSample.bmpVSpeed * 3.28084f;  // Convert m/s to ft/s
       xSemaphoreGive(telemetryMutex);
     }
     if (xSemaphoreTake(countMutex, portMAX_DELAY) == pdTRUE) {
       xSemaphoreGive(countMutex);
     }

    const unsigned long now = millis();
    float accelBaro = 0.0f;
    if (prevTimeMs > 0) {
      const float dt = static_cast<float>(now - prevTimeMs) / 1000.0f;
      if (dt > 0.0f) accelBaro = (verticalSpeed - prevVelMs) / dt;
    }
    prevVelMs  = verticalSpeed;
    prevTimeMs = now;

    const int transition = flightDetector.feed(accelMag, altFt, accelBaro);

    if (xSemaphoreTake(flightPhaseMutex, portMAX_DELAY) == pdTRUE) {
      xSemaphoreGive(flightPhaseMutex);
    }

    if (transition >= 0) {
      const uint32_t transitionMillis = millis();
      if (transition == 1 && launchDetectedMillis == 0) {
        launchDetectedMillis = transitionMillis;
        requestVegaStart(transitionMillis);
      } else if (transition == 5 && landedDetectedMillis == 0) {
        landedDetectedMillis = transitionMillis;
        requestVegaStop(transitionMillis);
      }

      Serial.printf("[FLIGHT PHASE] %d: %s\n",
                    transition, FLIGHT_PHASE_NAMES[transition]);
    }

    vTaskDelay(pdMS_TO_TICKS(20));
  }
}

void SdLogTask(void* pvParameters) {
  TickType_t lastFlush = xTaskGetTickCount();
  SensorSample sample{};

  auto writeSample = [&]() {
    logFile.print(sample.sampleMillis);          logFile.print(',');
    logFile.print(sample.accelX, 6);             logFile.print(',');
    logFile.print(sample.accelY, 6);             logFile.print(',');
    logFile.print(sample.accelZ, 6);             logFile.print(',');
    logFile.print(sample.gyroX, 6);              logFile.print(',');
    logFile.print(sample.gyroY, 6);              logFile.print(',');
    logFile.print(sample.gyroZ, 6);              logFile.print(',');
    logFile.print(sample.imuTemp, 6);            logFile.print(',');
    logFile.print(sample.bmpTemp, 6);            logFile.print(',');
    logFile.print(sample.bmpPressure, 6);        logFile.print(',');
    logFile.print(sample.bmpAltitude, 6);        logFile.print(',');
    logFile.print(sample.bmpVSpeed, 6);          logFile.print(',');
    logFile.print(sample.magX);                  logFile.print(',');
    logFile.print(sample.magY);                  logFile.print(',');
    logFile.print(sample.magZ);                  logFile.print(',');
    logFile.print(sample.gps.fixMillis);         logFile.print(',');
    logFile.print(sample.gps.valid ? 1 : 0);     logFile.print(',');
    logFile.print(sample.gps.latitude, 7);       logFile.print(',');
    logFile.print(sample.gps.longitude, 7);      logFile.print(',');
    logFile.print(sample.gps.altitude, 3);       logFile.print(',');
    logFile.print(sample.gps.speed, 3);          logFile.print(',');
    logFile.print(sample.gps.heading, 3);        logFile.print(',');
    logFile.print(sample.flightPhase);            logFile.print(',');

    bool vegaOnSnapshot = false;
    char vegaStatusSnapshot[32];
    char vegaLastMessageSnapshot[48];
    uint32_t vegaLastMessageMillisSnapshot = 0;
    readVegaStateSnapshot(vegaOnSnapshot, vegaStatusSnapshot, sizeof(vegaStatusSnapshot), vegaLastMessageSnapshot, sizeof(vegaLastMessageSnapshot), vegaLastMessageMillisSnapshot);

    logFile.print(vegaOnSnapshot ? 1 : 0);        logFile.print(',');
    logFile.print(vegaStatusSnapshot);            logFile.print(',');
    logFile.print(vegaLastMessageSnapshot);       logFile.print(',');
    logFile.println(vegaLastMessageMillisSnapshot);
  };

  for (;;) {
    if (stopSdTask) {
      while (xQueueReceive(logQueue, &sample, 0) == pdTRUE) {
        if (xSemaphoreTake(sdSpiMutex, portMAX_DELAY) == pdTRUE) {
          writeSample();
          xSemaphoreGive(sdSpiMutex);
        }
      }
      if (xSemaphoreTake(sdSpiMutex, portMAX_DELAY) == pdTRUE) {
        logFile.flush();
        logFile.close();
        xSemaphoreGive(sdSpiMutex);
      }
      sdLogTaskStopped = true;
      vTaskDelete(nullptr);
    }

    if (xQueueReceive(logQueue, &sample, pdMS_TO_TICKS(100)) == pdTRUE) {
      do {
        if (xSemaphoreTake(sdSpiMutex, portMAX_DELAY) == pdTRUE) {
          writeSample();
          xSemaphoreGive(sdSpiMutex);
        }
      } while (xQueueReceive(logQueue, &sample, 0) == pdTRUE);
    }

    if ((xTaskGetTickCount() - lastFlush) >= pdMS_TO_TICKS(logFlushIntervalMs)) {
      if (xSemaphoreTake(sdSpiMutex, portMAX_DELAY) == pdTRUE) {
        logFile.flush();
        xSemaphoreGive(sdSpiMutex);
      }
      lastFlush = xTaskGetTickCount();
    }
  }
}

void VegaUartTask(void* pvParameters) {
  (void)pvParameters;
  constexpr TickType_t pollDelay = pdMS_TO_TICKS(20);
  char line[96];
  size_t lineLength = 0;

  for (;;) {
    if (stopVegaTask) {
      vegaTaskStopped = true;
      vTaskDelete(nullptr);
    }

    while (Serial1.available() > 0) {
      const char ch = static_cast<char>(Serial1.read());
      if (ch == '\r' || ch == '\n') {
        if (lineLength > 0) {
          line[lineLength] = '\0';
          processVegaMessage(line, millis());
          lineLength = 0;
        }
      } else if (lineLength < sizeof(line) - 1) {
        line[lineLength++] = ch;
      }
    }

    vTaskDelay(pollDelay);
  }
}

void PrintTask(void* pvParameters) {
  constexpr TickType_t printDelay = pdMS_TO_TICKS(5000);
  for (;;) {
    vTaskDelay(printDelay);
    uint32_t count = 0;
    if (xSemaphoreTake(countMutex, portMAX_DELAY) == pdTRUE) {
      count = pollCount;
      pollCount = 0;
      xSemaphoreGive(countMutex);
    }
    const float averageFreq = static_cast<float>(count) / 5.0f;
    Serial.print("Average Polling Frequency: ");
    Serial.print(averageFreq);
    Serial.println(" Hz");
  }
}

void webServerTask(void* pvParameters) {
  for (;;) {
    webServer.handleClient();
    vTaskDelay(pdMS_TO_TICKS(10));
  }
}

void handleRoot() {
  File file = SD.open("/index.html");
  if (!file) {
    webServer.send(500, "text/plain", "ERROR: index.html not found on SD");
    return;
  }
  webServer.streamFile(file, "text/html");
  file.close();
}

void handleGetCsvList() {
  String json = "[";
  File root = SD.open("/");
  if (root) {
    File file = root.openNextFile();
    bool firstFile = true;
    while (file) {
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
  webServer.send(200, "application/json", json);
}

void handleFileDownload() {
  if (webServer.hasArg("file")) {
    const String fileName = webServer.arg("file");
    if (fileName.indexOf('/') != -1 || fileName.indexOf("..") != -1) {
      webServer.send(400, "text/plain", "Invalid filename.");
      return;
    }
    String path = "/" + fileName;
    File file = SD.open(path);
    if (!file) {
      webServer.send(404, "text/plain", "File not found.");
      return;
    }
    webServer.sendHeader("Content-Disposition", "attachment; filename=" + fileName);
    webServer.streamFile(file, "text/csv");
    file.close();
  } else {
    webServer.send(400, "text/plain", "Bad Request: 'file' missing.");
  }
}

void handleGetConfig() {
  JsonDocument doc;
  doc["pressure"] = String(seaLevelPressureHpa, 2);
  doc["sensorSampleRateMs"] = sensorSampleRateMs;
  doc["gpsSampleRateMs"] = gpsSampleRateMs;
  doc["logFlushIntervalMs"] = logFlushIntervalMs;
  doc["ledBrightnessHeartbeat"] = ledBrightnessHeartbeat;
  doc["ledBrightnessIdle"] = ledBrightnessIdle;
  doc["launchThresholdMultiplier"] = String(launchThresholdMultiplier, 2);
  doc["burnoutThresholdMultiplier"] = String(burnoutThresholdMultiplier, 2);
  doc["launchStreakRequired"] = launchStreakRequired;
  doc["burnoutStreakRequired"] = burnoutStreakRequired;
  doc["apogeeStreakRequired"] = apogeeStreakRequired;
  doc["landStreakRequired"] = landStreakRequired;
  doc["landingAltitudeThresholdFt"] = String(landingAltitudeThresholdFt, 2);
  doc["wifiSSID"] = ap_ssid;
  doc["wifiPassword"] = ap_password;

  String output;
  serializeJson(doc, output);
  webServer.send(200, "application/json", output);
}


void handleUpdateConfig() {
  String body = webServer.arg("plain");
  if (body.length() == 0) {
    webServer.send(400, "text/plain", "Empty body");
    return;
  }

  JsonDocument doc;
  DeserializationError error = deserializeJson(doc, body);
  if (error) {
    webServer.send(400, "text/plain", "Invalid JSON");
    return;
  }

   if (doc["pressure"].is<float>()) {
     auto p = doc["pressure"].as<float>();
     if (p >= 800.0f && p <= 1200.0f) seaLevelPressureHpa = p;
     else { webServer.send(400, "text/plain", "Invalid pressure value"); return; }
   }
   if (doc["sensorSampleRateMs"].is<uint32_t>()) {
     auto val = doc["sensorSampleRateMs"].as<uint32_t>();
     if (val >= 5 && val <= 10000) sensorSampleRateMs = val;
   }
   if (doc["gpsSampleRateMs"].is<uint32_t>()) {
     auto val = doc["gpsSampleRateMs"].as<uint32_t>();
     if (val >= 100 && val <= 60000) gpsSampleRateMs = val;
   }
   if (doc["logFlushIntervalMs"].is<uint32_t>()) {
     auto val = doc["logFlushIntervalMs"].as<uint32_t>();
     if (val >= 500 && val <= 60000) logFlushIntervalMs = val;
   }
   if (doc["ledBrightnessHeartbeat"].is<uint8_t>()) {
     ledBrightnessHeartbeat = doc["ledBrightnessHeartbeat"].as<uint8_t>();
   }
   if (doc["ledBrightnessIdle"].is<uint8_t>()) {
     ledBrightnessIdle = doc["ledBrightnessIdle"].as<uint8_t>();
   }
   if (doc["launchThresholdMultiplier"].is<float>()) {
     auto val = doc["launchThresholdMultiplier"].as<float>();
     if (val >= 1.0f && val <= 20.0f) launchThresholdMultiplier = val;
   }
   if (doc["burnoutThresholdMultiplier"].is<float>()) {
     auto val = doc["burnoutThresholdMultiplier"].as<float>();
     if (val >= 0.5f && val <= 10.0f) burnoutThresholdMultiplier = val;
   }
   if (doc["launchStreakRequired"].is<int>()) {
     int val = doc["launchStreakRequired"].as<int>();
     if (val >= 1 && val <= 50) launchStreakRequired = val;
   }
   if (doc["burnoutStreakRequired"].is<int>()) {
     int val = doc["burnoutStreakRequired"].as<int>();
     if (val >= 1 && val <= 100) burnoutStreakRequired = val;
   }
   if (doc["apogeeStreakRequired"].is<int>()) {
     int val = doc["apogeeStreakRequired"].as<int>();
     if (val >= 1 && val <= 100) apogeeStreakRequired = val;
   }
   if (doc["landStreakRequired"].is<int>()) {
     int val = doc["landStreakRequired"].as<int>();
     if (val >= 1 && val <= 100) landStreakRequired = val;
   }
   if (doc["landingAltitudeThresholdFt"].is<float>()) {
     auto val = doc["landingAltitudeThresholdFt"].as<float>();
     if (val >= 1.0f && val <= 500.0f) landingAltitudeThresholdFt = val;
   }
   if (doc["wifiSSID"].is<String>()) {
     ap_ssid = doc["wifiSSID"].as<String>();
   }
   if (doc["wifiPassword"].is<String>()) {
     ap_password = doc["wifiPassword"].as<String>();
   }

  saveConfiguration();
  webServer.send(200, "text/plain", "Config updated");
}

void handleNotFound() {
  webServer.send(404, "text/plain", "Not Found");
}

void saveConfiguration() {
  File f = SD.open("/config.txt", FILE_WRITE);
  if (!f) return;

  f.print("pressure=");
  f.println(String(seaLevelPressureHpa, 2));

  f.print("sensorSampleRateMs=");
  f.println(sensorSampleRateMs);

  f.print("gpsSampleRateMs=");
  f.println(gpsSampleRateMs);

  f.print("logFlushIntervalMs=");
  f.println(logFlushIntervalMs);

  f.print("ledBrightnessHeartbeat=");
  f.println(ledBrightnessHeartbeat);

  f.print("ledBrightnessIdle=");
  f.println(ledBrightnessIdle);

  f.print("launchThresholdMultiplier=");
  f.println(String(launchThresholdMultiplier, 2));

  f.print("burnoutThresholdMultiplier=");
  f.println(String(burnoutThresholdMultiplier, 2));

  f.print("launchStreakRequired=");
  f.println(launchStreakRequired);

  f.print("burnoutStreakRequired=");
  f.println(burnoutStreakRequired);

  f.print("apogeeStreakRequired=");
  f.println(apogeeStreakRequired);

  f.print("landStreakRequired=");
  f.println(landStreakRequired);

  f.print("landingAltitudeThresholdFt=");
  f.println(String(landingAltitudeThresholdFt, 2));

  f.print("wifiSSID=");
  f.println(ap_ssid);

  f.print("wifiPassword=");
  f.println(ap_password);

  f.close();
}

void loadConfiguration() {
  File f = SD.open("/config.txt");
  if (!f) {
    Serial.println("config.txt not found on SD (using defaults). Creating default.");
    saveConfiguration();
    return;
  }

  while (f.available()) {
    String line = f.readStringUntil('\n');
    line.trim();

    if (line.startsWith("pressure=")) {
      String val = line.substring(line.indexOf('=') + 1);
      float p = val.toFloat();
      if (p >= 800.0f && p <= 1200.0f) seaLevelPressureHpa = p;
    }
    else if (line.startsWith("sensorSampleRateMs=")) {
      sensorSampleRateMs = line.substring(line.indexOf('=') + 1).toInt();
      if (sensorSampleRateMs < 5) sensorSampleRateMs = 5;
    }
    else if (line.startsWith("gpsSampleRateMs=")) {
      gpsSampleRateMs = line.substring(line.indexOf('=') + 1).toInt();
      if (gpsSampleRateMs < 100) gpsSampleRateMs = 100;
    }
    else if (line.startsWith("logFlushIntervalMs=")) {
      logFlushIntervalMs = line.substring(line.indexOf('=') + 1).toInt();
      if (logFlushIntervalMs < 500) logFlushIntervalMs = 500;
    }
    else if (line.startsWith("ledBrightnessHeartbeat=")) {
      ledBrightnessHeartbeat = line.substring(line.indexOf('=') + 1).toInt();
    }
    else if (line.startsWith("ledBrightnessIdle=")) {
      ledBrightnessIdle = line.substring(line.indexOf('=') + 1).toInt();
    }
    else if (line.startsWith("launchThresholdMultiplier=")) {
      launchThresholdMultiplier = line.substring(line.indexOf('=') + 1).toFloat();
      if (launchThresholdMultiplier < 1.0f) launchThresholdMultiplier = 1.0f;
    }
    else if (line.startsWith("burnoutThresholdMultiplier=")) {
      burnoutThresholdMultiplier = line.substring(line.indexOf('=') + 1).toFloat();
      if (burnoutThresholdMultiplier < 0.5f) burnoutThresholdMultiplier = 0.5f;
    }
    else if (line.startsWith("launchStreakRequired=")) {
      launchStreakRequired = line.substring(line.indexOf('=') + 1).toInt();
      if (launchStreakRequired < 1) launchStreakRequired = 1;
    }
    else if (line.startsWith("burnoutStreakRequired=")) {
      burnoutStreakRequired = line.substring(line.indexOf('=') + 1).toInt();
      if (burnoutStreakRequired < 1) burnoutStreakRequired = 1;
    }
    else if (line.startsWith("apogeeStreakRequired=")) {
      apogeeStreakRequired = line.substring(line.indexOf('=') + 1).toInt();
      if (apogeeStreakRequired < 1) apogeeStreakRequired = 1;
    }
    else if (line.startsWith("landStreakRequired=")) {
      landStreakRequired = line.substring(line.indexOf('=') + 1).toInt();
      if (landStreakRequired < 1) landStreakRequired = 1;
    }
    else if (line.startsWith("landingAltitudeThresholdFt=")) {
      landingAltitudeThresholdFt = line.substring(line.indexOf('=') + 1).toFloat();
      if (landingAltitudeThresholdFt < 1.0f) landingAltitudeThresholdFt = 1.0f;
    }
    else if (line.startsWith("wifiSSID=")) {
      ap_ssid = line.substring(line.indexOf('=') + 1);
    }
    else if (line.startsWith("wifiPassword=")) {
      ap_password = line.substring(line.indexOf('=') + 1);
    }
  }

  f.close();
  Serial.printf("Loaded configuration: pressure=%.2f hPa, sensorRate=%lu ms, gpsRate=%lu ms\n",
                static_cast<double>(seaLevelPressureHpa),
                static_cast<unsigned long>(sensorSampleRateMs),
                static_cast<unsigned long>(gpsSampleRateMs));
}