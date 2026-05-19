#include "ICM42688.h"
#include <Wire.h>
#include <SPI.h>
#include <Adafruit_Sensor.h>
#include "Adafruit_BMP5xx.h"
#include <SparkFun_MMC5983MA_Arduino_Library.h>

#include <FreeRTOS.h>
#include <task.h>
#include <semphr.h>

const int CS_ICM = 3;
const int CS_BMP = 41;
const int CS_MMC = 45;
const int CS_OTHER = 5;

ICM42688 IMU(SPI, CS_ICM);
Adafruit_BMP5xx bmp;
SFE_MMC5983MA myMag;

SemaphoreHandle_t spiMutex = NULL;
volatile uint32_t pollCount = 0;
SemaphoreHandle_t countMutex = NULL;

void SensorPollTask(void* pvParameters);
void PrintTask(void* pvParameters);

void setup() {
  pinMode(CS_ICM, OUTPUT); digitalWrite(CS_ICM, HIGH);
  pinMode(CS_BMP, OUTPUT); digitalWrite(CS_BMP, HIGH);
  pinMode(CS_MMC, OUTPUT); digitalWrite(CS_MMC, HIGH);
  pinMode(CS_OTHER, OUTPUT); digitalWrite(CS_OTHER, HIGH);

  Serial.begin(115200);
  while (!Serial) delay(10);

  Serial.println("Starting Sensor Initialization...");

  SPI.begin();

  int status = IMU.begin();
  if (status < 0) {
    Serial.println("Failed: ICM42688 initialization unsuccessful.");
    while(1) delay(10);
  }

  if (!bmp.begin(CS_BMP, &SPI)) {
    Serial.println("Failed: BMP5xx initialization unsuccessful.");
    while (1) delay(10);
  }
  bmp.setTemperatureOversampling(BMP5XX_OVERSAMPLING_2X);
  bmp.setPressureOversampling(BMP5XX_OVERSAMPLING_16X);
  bmp.setIIRFilterCoeff(BMP5XX_IIR_FILTER_COEFF_3);
  bmp.setOutputDataRate(BMP5XX_ODR_100_2_HZ);
  bmp.setPowerMode(BMP5XX_POWERMODE_CONTINUOUS);

  if (myMag.begin(CS_MMC) == false) {
    Serial.println("Failed: MMC5983MA initialization unsuccessful.");
    while(1) delay(10);
  }
  myMag.softReset();
  delay(20);
  myMag.enableAutomaticSetReset();

  myMag.setContinuousModeFrequency(100);
  myMag.enableContinuousMode();

  Serial.println("Success: All 3 sensors initialized okay.");

  spiMutex = xSemaphoreCreateMutex();
  if (spiMutex == NULL) {
    Serial.println("Failed to create SPI mutex");
    while (1) delay(10);
  }

  countMutex = xSemaphoreCreateMutex();
  if (countMutex == NULL) {
    Serial.println("Failed to create counter mutex");
    while (1) delay(10);
  }

  BaseType_t r1 = xTaskCreate(
    SensorPollTask,
    "SensorPoll",
    2048,
    NULL,
    tskIDLE_PRIORITY + 2,
    NULL
  );

  BaseType_t r2 = xTaskCreate(
    PrintTask,
    "Printer",
    1024,
    NULL,
    tskIDLE_PRIORITY + 1,
    NULL
  );

  if (r1 != pdPASS || r2 != pdPASS) {
    Serial.println("Failed to create FreeRTOS tasks");
    while (1) delay(10);
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

    if (xSemaphoreTake(spiMutex, portMAX_DELAY) == pdTRUE) {
      IMU.getAGT();
      bmp.performReading();
      uint32_t currentX = 0, currentY = 0, currentZ = 0;
      myMag.readFieldsXYZ(&currentX, &currentY, &currentZ);
      myMag.clearMeasDoneInterrupt();
      xSemaphoreGive(spiMutex);
    }

    if (xSemaphoreTake(countMutex, portMAX_DELAY) == pdTRUE) {
      pollCount++;
      xSemaphoreGive(countMutex);
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