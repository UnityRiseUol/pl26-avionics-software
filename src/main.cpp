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

bool inMSCMode = false;
bool loggingActive = false;
RTC_DATA_ATTR uint32_t bootToMSC = 0;
bool longPressHandled = false;

SPIClass* sdcardSPI = nullptr;

ICM42688* IMU = nullptr;
Adafruit_BMP5xx bmp;
SFE_MMC5983MA myMag;
SFE_UBLOX_GNSS myGNSS;

static auto ap_ssid = "LIFTSv2";
static auto ap_password = "123456789";
WebServer webServer(80);

float seaLevelPressureHpa = 1013.25f;

SemaphoreHandle_t sensorSpiMutex = nullptr;
SemaphoreHandle_t sdSpiMutex = nullptr;
SemaphoreHandle_t countMutex = nullptr;
SemaphoreHandle_t gpsDataMutex = nullptr;
SemaphoreHandle_t telemetryMutex = nullptr;

volatile uint32_t pollCount = 0;
volatile bool stopSensorTask = false;
volatile bool stopSdTask = false;
volatile bool stopGpsTask = false;
volatile bool sensorTaskStopped = false;
volatile bool sdLogTaskStopped = false;
volatile bool gpsTaskStopped = false;

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
};

// Must remain exactly 44 bytes for the ground station decoder.
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
};

static_assert(sizeof(TelemetryPacket) == 44, "TelemetryPacket must remain 44 bytes");

QueueHandle_t logQueue = nullptr;
File logFile;
char currentFilename[32];
GpsSample latestGpsSample{};
SensorSample latestSensorSample{};
bool haveLastBmpAltitude = false;
float lastBmpAltitude = 0.0f;
uint32_t lastBmpAltitudeMillis = 0;

bool initGPS();
bool initLoRa();
void startLogging();
void stopLogging();

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
bool parsePressureFromBody(const String& body, float& outPressure);

void setRGB(uint8_t r, uint8_t g, uint8_t b) {
  analogWrite(RED_LED, r);
  analogWrite(GREEN_LED, g);
  analogWrite(BLUE_LED, b);
}

void ledHeartbeat() {
  unsigned long t = millis() % 2000;
  unsigned long brightness_ul = (t < 1000UL) ? (t * 40UL / 1000UL) : ((2000UL - t) * 40UL / 1000UL);
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
  if (!sensorSpiMutex || !sdSpiMutex || !countMutex || !gpsDataMutex || !telemetryMutex) {
    Serial.println("Failed to create mutexes");
    setRGB(150, 0, 0);
    while (true) delay(10);
  }

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

  logFile.println("millis,accelX,accelY,accelZ,gyroX,gyroY,gyroZ,imuTemp,bmpTemp,bmpPressure,bmpAltitude,bmpVSpeed,magX,magY,magZ,gpsFixMillis,gpsValid,gpsLatitude,gpsLongitude,gpsAltitude,gpsSpeed,gpsHeading");
  logFile.flush();
  Serial.print("Logging to file: ");
  Serial.println(currentFilename);

  logQueue = xQueueCreate(128, sizeof(SensorSample));
  if (logQueue == nullptr) {
    Serial.println("Failed to create logging queue");
    setRGB(150, 0, 0);
    return;
  }

  latestGpsSample = {};
  latestSensorSample = {};
  haveLastBmpAltitude = false;
  lastBmpAltitude = 0.0f;
  lastBmpAltitudeMillis = 0;
  stopSensorTask = false;
  stopSdTask = false;
  stopGpsTask = false;
  sensorTaskStopped = false;
  sdLogTaskStopped = false;
  gpsTaskStopped = false;

  BaseType_t r0 = xTaskCreate(GpsTask, "GpsPoll", 4096, nullptr, tskIDLE_PRIORITY + 1, nullptr);
  BaseType_t r1 = xTaskCreate(SensorPollTask, "SensorPoll", 2048, nullptr, tskIDLE_PRIORITY + 2, nullptr);
  BaseType_t r3 = xTaskCreate(SdLogTask, "SDLog", 4096, nullptr, tskIDLE_PRIORITY + 1, nullptr);

  if (r0 != pdPASS || r1 != pdPASS || r3 != pdPASS) {
    Serial.println("Failed to create FreeRTOS tasks");
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
      if (loggingActive) ledHeartbeat(); else setRGB(0, 20, 0);
    }

    vTaskDelay(pdMS_TO_TICKS(20));
  }
}

void SensorPollTask(void* pvParameters) {
  constexpr TickType_t xFrequency = pdMS_TO_TICKS(10);
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
  constexpr TickType_t xFrequency = pdMS_TO_TICKS(1000);
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
      sampleGps.latitude = sampleGps.rawLat / 10000000.0f;
      sampleGps.longitude = sampleGps.rawLon / 10000000.0f;
      sampleGps.altitude = sampleGps.rawAlt / 1000.0f;
      sampleGps.speed = sampleGps.rawSpeed / 1000.0f;
      sampleGps.heading = sampleGps.rawHeading / 100000.0f;
      sampleGps.valid = true;

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
      xSemaphoreGive(telemetryMutex);
    }

    if (xSemaphoreTake(gpsDataMutex, portMAX_DELAY) == pdTRUE) {
      if (latestGpsSample.valid) {
        packet.lat = latestGpsSample.latitude;
        packet.lon = latestGpsSample.longitude;
      }
      xSemaphoreGive(gpsDataMutex);
    }

    // Quaternion and INS estimates are not available in this build yet, so they remain zero.
    // Packet is intentionally zero-initialised to preserve the ground-station layout.
    if (xSemaphoreTake(sensorSpiMutex, portMAX_DELAY) == pdTRUE) {
      if (LoRa.beginPacket()) {
        LoRa.write(reinterpret_cast<uint8_t*>(&packet), sizeof(packet));
        LoRa.endPacket();
      }
      xSemaphoreGive(sensorSpiMutex);
    }
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
    logFile.println(sample.gps.heading, 3);
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

    if ((xTaskGetTickCount() - lastFlush) >= pdMS_TO_TICKS(1000)) {
      if (xSemaphoreTake(sdSpiMutex, portMAX_DELAY) == pdTRUE) {
        logFile.flush();
        xSemaphoreGive(sdSpiMutex);
      }
      lastFlush = xTaskGetTickCount();
    }
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
  float newPressure = NAN;
  if (!parsePressureFromBody(body, newPressure) || newPressure < 800.0f || newPressure > 1200.0f) {
    webServer.send(400, "text/plain", "Invalid pressure value");
    return;
  }
  seaLevelPressureHpa = newPressure;
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
  f.close();
}

void loadConfiguration() {
  File f = SD.open("/config.txt");
  if (!f) {
    Serial.println("config.txt not found on SD (using default sea level pressure). Creating default.");
    saveConfiguration();
    return;
  }
  while (f.available()) {
    String line = f.readStringUntil('\n');
    line.trim();
    if (line.startsWith("pressure=")) {
      String val = line.substring(line.indexOf('=') + 1);
      float p = val.toFloat();
      if (p >= 800.0f && p <= 1200.0f) {
        seaLevelPressureHpa = p;
      }
    }
  }
  f.close();
  Serial.printf("Loaded sea level pressure: %.2f hPa\n", static_cast<double>(seaLevelPressureHpa));
}

bool parsePressureFromBody(const String& body, float& outPressure) {
  String s = body;
  s.trim();

  int keyPos = s.indexOf("pressure");
  if (keyPos >= 0) {
    int sep = s.indexOf('=', keyPos);
    if (sep < 0) sep = s.indexOf(':', keyPos);
    if (sep >= 0) {
      String num = s.substring(sep + 1);
      num.trim();
      if (num.startsWith("\"")) num.remove(0, 1);
      if (num.endsWith("\"")) num.remove(num.length() - 1);
      while (num.length() > 0) {
        const char c = num.charAt(num.length() - 1);
        const bool numericTail = (c >= '0' && c <= '9') || c == '.' || c == '-' || c == '+' || c == 'e' || c == 'E';
        if (numericTail) break;
        num.remove(num.length() - 1);
      }
      outPressure = num.toFloat();
      return !isnan(outPressure);
    }
  }

  outPressure = s.toFloat();
  return !isnan(outPressure);
}

