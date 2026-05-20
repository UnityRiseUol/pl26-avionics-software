#include "ICM42688.h"
#include <Wire.h>
#include <SPI.h>
#include <SD.h>
#include <FS.h>
#include <Adafruit_Sensor.h>
#include "Adafruit_BMP5xx.h"
#include <SparkFun_MMC5983MA_Arduino_Library.h>

#include <FreeRTOS.h>
#include <task.h>
#include <semphr.h>

// Sensor SPI Configuration (Standard Pins - FSPI / default SPI)
const int CS_ICM = 3;
const int CS_BMP = 41;
const int CS_MMC = 45;
const int CS_OTHER = 5;

// SD Card SPI Configuration (Custom Pins - HSPI)
const int CS_SD   = 40;
const int SD_MOSI = 39;
const int SD_MISO = 38;
const int SD_CLK  = 47;

// Separate SPI instance for SD card (initialized in setup)
SPIClass* sdcardSPI = nullptr;

ICM42688 IMU(SPI, CS_ICM);
Adafruit_BMP5xx bmp;
SFE_MMC5983MA myMag;

SemaphoreHandle_t sensorSpiMutex = nullptr;
SemaphoreHandle_t sdSpiMutex = nullptr;
volatile uint32_t pollCount = 0;
SemaphoreHandle_t countMutex = nullptr;

struct SensorSample {
  uint32_t sampleMillis;
  float accelX;
  float accelY;
  float accelZ;
  float gyroX;
  float gyroY;
  float gyroZ;
  float imuTemp;
  float bmpTemp;
  float bmpPressure;
  uint32_t magX;
  uint32_t magY;
  uint32_t magZ;
};

QueueHandle_t logQueue = nullptr;
File logFile;
char currentFilename[32];

namespace {

template <int N>
struct PriorityTag : PriorityTag<N - 1> {};

template <>
struct PriorityTag<0> {};

#define DEFINE_SENSOR_READER(FuncName, Member1, Member2, Member3, Member4) \
  template <typename T> \
  auto FuncName##_impl(T& obj, PriorityTag<4>) -> decltype((void)obj.Member1, float()) { return obj.Member1; } \
  template <typename T> \
  auto FuncName##_impl(T& obj, PriorityTag<3>) -> decltype((void)obj.Member2, float()) { return obj.Member2; } \
  template <typename T> \
  auto FuncName##_impl(T& obj, PriorityTag<2>) -> decltype((void)obj.Member3, float()) { return obj.Member3; } \
  template <typename T> \
  auto FuncName##_impl(T& obj, PriorityTag<1>) -> decltype((void)obj.Member4, float()) { return obj.Member4; } \
  inline float FuncName##_impl(...) { return 0.0f; } \
  template <typename T> \
  float FuncName(T& obj) { return FuncName##_impl(obj, PriorityTag<4>{}); }

DEFINE_SENSOR_READER(readImuAccelX, accX, accelX, aX, AccelX)
DEFINE_SENSOR_READER(readImuAccelY, accY, accelY, aY, AccelY)
DEFINE_SENSOR_READER(readImuAccelZ, accZ, accelZ, aZ, AccelZ)
DEFINE_SENSOR_READER(readImuGyroX, gyrX, gyroX, gX, GyroX)
DEFINE_SENSOR_READER(readImuGyroY, gyrY, gyroY, gY, GyroY)
DEFINE_SENSOR_READER(readImuGyroZ, gyrZ, gyroZ, gZ, GyroZ)
DEFINE_SENSOR_READER(readImuTemp, temp, temperature, Temperature, Temp)

#undef DEFINE_SENSOR_READER

}  // namespace

void SensorPollTask(void* pvParameters);
void PrintTask(void* pvParameters);
void SdLogTask(void* pvParameters);

uint32_t findNextLogFileIndex() {
  uint32_t fileIndex = 0;
  char candidate[32];

  while (true) {
    snprintf(candidate, sizeof(candidate), "/log_%lu.csv", (unsigned long)fileIndex);
    if (!SD.exists(candidate)) {
      return fileIndex;
    }
    fileIndex++;
  }
}

void setup() {
  // Sensor CS pins (default SPI bus)
  pinMode(CS_ICM, OUTPUT); digitalWrite(CS_ICM, HIGH);
  pinMode(CS_BMP, OUTPUT); digitalWrite(CS_BMP, HIGH);
  pinMode(CS_MMC, OUTPUT); digitalWrite(CS_MMC, HIGH);
  pinMode(CS_OTHER, OUTPUT); digitalWrite(CS_OTHER, HIGH);

  // SD Card CS pin (HSPI - Custom pins)
  pinMode(CS_SD, OUTPUT); digitalWrite(CS_SD, HIGH);

  Serial.begin(115200);

  Serial.println("Starting Sensor Initialization...");

  // Initialize sensor SPI bus (default SPI with default pins)
  SPI.begin();

  // Create and initialize SD card SPI bus (HSPI with custom pins 47/38/39/40)
  sdcardSPI = new SPIClass(HSPI);
  sdcardSPI->begin(SD_CLK, SD_MISO, SD_MOSI, CS_SD);

  int status = IMU.begin();
  if (status < 0) {
    Serial.println("Failed: ICM42688 initialization unsuccessful.");
    while (true) delay(10);
  }

  if (!bmp.begin(CS_BMP, &SPI)) {
    Serial.println("Failed: BMP5xx initialization unsuccessful.");
    while (true) delay(10);
  }
  bmp.setTemperatureOversampling(BMP5XX_OVERSAMPLING_2X);
  bmp.setPressureOversampling(BMP5XX_OVERSAMPLING_16X);
  bmp.setIIRFilterCoeff(BMP5XX_IIR_FILTER_COEFF_3);
  bmp.setOutputDataRate(BMP5XX_ODR_100_2_HZ);
  bmp.setPowerMode(BMP5XX_POWERMODE_CONTINUOUS);

  if (myMag.begin(CS_MMC) == false) {
    Serial.println("Failed: MMC5983MA initialization unsuccessful.");
    while (true) delay(10);
  }
  myMag.softReset();
  delay(20);
  myMag.enableAutomaticSetReset();

  myMag.setContinuousModeFrequency(100);
  myMag.enableContinuousMode();

  Serial.println("Success: All 3 sensors initialized okay.");

  if (!SD.begin(CS_SD, *sdcardSPI, 4000000)) {
    Serial.println("Failed: SD card initialization unsuccessful.");
    while (true) delay(10);
  }

  uint32_t fileIndex = findNextLogFileIndex();
  snprintf(currentFilename, sizeof(currentFilename), "/log_%lu.csv", (unsigned long)fileIndex);

  logFile = SD.open(currentFilename, FILE_WRITE);
  if (!logFile) {
    Serial.println("Failed to open log file for writing.");
    while (true) delay(10);
  }

  logFile.println("millis,accelX,accelY,accelZ,gyroX,gyroY,gyroZ,imuTemp,bmpTemp,bmpPressure,magX,magY,magZ");
  logFile.flush();
  Serial.print("Logging to file: ");
  Serial.println(currentFilename);

  sensorSpiMutex = xSemaphoreCreateMutex();
  if (sensorSpiMutex == nullptr) {
    Serial.println("Failed to create sensor SPI mutex");
    while (true) delay(10);
  }

  sdSpiMutex = xSemaphoreCreateMutex();
  if (sdSpiMutex == nullptr) {
    Serial.println("Failed to create SD SPI mutex");
    while (true) delay(10);
  }

  countMutex = xSemaphoreCreateMutex();
  if (countMutex == nullptr) {
    Serial.println("Failed to create counter mutex");
    while (true) delay(10);
  }

  logQueue = xQueueCreate(128, sizeof(SensorSample));
  if (logQueue == nullptr) {
    Serial.println("Failed to create logging queue");
    while (true) delay(10);
  }

  BaseType_t r1 = xTaskCreate(
    SensorPollTask,
    "SensorPoll",
    2048,
    nullptr,
    tskIDLE_PRIORITY + 2,
    nullptr
  );

  BaseType_t r2 = xTaskCreate(
    PrintTask,
    "Printer",
    1024,
    nullptr,
    tskIDLE_PRIORITY + 1,
    nullptr
  );

  BaseType_t r3 = xTaskCreate(
    SdLogTask,
    "SDLog",
    4096,
    nullptr,
    tskIDLE_PRIORITY + 1,
    nullptr
  );

  if (r1 != pdPASS || r2 != pdPASS || r3 != pdPASS) {
    Serial.println("Failed to create FreeRTOS tasks");
    while (true) delay(10);
  }
}

void loop() {
  vTaskDelay(pdMS_TO_TICKS(1000));
}

void SensorPollTask(void* pvParameters) {
  const TickType_t xFrequency = pdMS_TO_TICKS(10);
  TickType_t xLastWakeTime = xTaskGetTickCount();

  for (;;) {
    vTaskDelayUntil(&xLastWakeTime, xFrequency);

    SensorSample sample{};

    if (xSemaphoreTake(sensorSpiMutex, portMAX_DELAY) == pdTRUE) {
      IMU.getAGT();
      bmp.performReading();
      sample.accelX = readImuAccelX(IMU);
      sample.accelY = readImuAccelY(IMU);
      sample.accelZ = readImuAccelZ(IMU);
      sample.gyroX = readImuGyroX(IMU);
      sample.gyroY = readImuGyroY(IMU);
      sample.gyroZ = readImuGyroZ(IMU);
      sample.imuTemp = readImuTemp(IMU);
      sample.bmpTemp = bmp.temperature;
      sample.bmpPressure = bmp.pressure;
      myMag.readFieldsXYZ(&sample.magX, &sample.magY, &sample.magZ);
      myMag.clearMeasDoneInterrupt();
      xSemaphoreGive(sensorSpiMutex);
    }

    sample.sampleMillis = millis();

    if (logQueue != nullptr) {
      xQueueSend(logQueue, &sample, portMAX_DELAY);
    }

    if (xSemaphoreTake(countMutex, portMAX_DELAY) == pdTRUE) {
      pollCount++;
      xSemaphoreGive(countMutex);
    }
  }
}

void SdLogTask(void* pvParameters) {
  TickType_t lastFlush = xTaskGetTickCount();
  SensorSample sample{};

  for (;;) {
    if (xQueueReceive(logQueue, &sample, pdMS_TO_TICKS(100)) == pdTRUE) {
      do {
        if (xSemaphoreTake(sdSpiMutex, portMAX_DELAY) == pdTRUE) {
          logFile.print(sample.sampleMillis);
          logFile.print(',');
          logFile.print(sample.accelX, 6); logFile.print(',');
          logFile.print(sample.accelY, 6); logFile.print(',');
          logFile.print(sample.accelZ, 6); logFile.print(',');
          logFile.print(sample.gyroX, 6); logFile.print(',');
          logFile.print(sample.gyroY, 6); logFile.print(',');
          logFile.print(sample.gyroZ, 6); logFile.print(',');
          logFile.print(sample.imuTemp, 6); logFile.print(',');
          logFile.print(sample.bmpTemp, 6); logFile.print(',');
          logFile.print(sample.bmpPressure, 6); logFile.print(',');
          logFile.print(sample.magX); logFile.print(',');
          logFile.print(sample.magY); logFile.print(',');
          logFile.println(sample.magZ);
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
  const TickType_t printDelay = pdMS_TO_TICKS(5000);
  for (;;) {
    vTaskDelay(printDelay);
    uint32_t count = 0;
    if (xSemaphoreTake(countMutex, portMAX_DELAY) == pdTRUE) {
      count = pollCount;
      pollCount = 0;
      xSemaphoreGive(countMutex);
    }

    float averageFreq = (float)count / 5.0f;
    Serial.print("Average Polling Frequency: ");
    Serial.print(averageFreq);
    Serial.println(" Hz");
  }
}
