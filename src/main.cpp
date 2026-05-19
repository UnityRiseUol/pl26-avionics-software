#include "ICM42688.h"
#include <Wire.h>
#include <SPI.h>
#include <Adafruit_Sensor.h>
#include "Adafruit_BMP5xx.h"
#include <SparkFun_MMC5983MA_Arduino_Library.h>

// --- CS Pin Definitions ---
const int CS_ICM = 3;
const int CS_BMP = 41;
const int CS_MMC = 45;
const int CS_OTHER = 5;

// --- Sensor Objects ---
ICM42688 IMU(SPI, CS_ICM);
Adafruit_BMP5xx bmp;
SFE_MMC5983MA myMag;

// --- Timing Variables ---
unsigned long lastPollTime = 0;
const unsigned long pollInterval = 10000; // 100 Hz = 10,000 microseconds

unsigned long lastPrintTime = 0;
const unsigned long printInterval = 5000000; // 5 seconds = 5,000,000 microseconds
unsigned long pollCount = 0;

void setup() {
  // 1. Deselect all SPI devices to prevent bus collisions during initialization
  pinMode(CS_ICM, OUTPUT); digitalWrite(CS_ICM, HIGH);
  pinMode(CS_BMP, OUTPUT); digitalWrite(CS_BMP, HIGH);
  pinMode(CS_MMC, OUTPUT); digitalWrite(CS_MMC, HIGH);
  pinMode(CS_OTHER, OUTPUT); digitalWrite(CS_OTHER, HIGH);

  Serial.begin(115200);
  while (!Serial) delay(10);

  Serial.println("Starting Sensor Initialization...");

  SPI.begin();

  // --- Initialize ICM42688 ---
  int status = IMU.begin();
  if (status < 0) {
    Serial.println("Failed: ICM42688 initialization unsuccessful.");
    while(1) delay(10);
  }

  // --- Initialize BMP5xx ---
  if (!bmp.begin(CS_BMP, &SPI)) {
    Serial.println("Failed: BMP5xx initialization unsuccessful.");
    while (1) delay(10);
  }
  // Optimize BMP5xx for ~100Hz operation
  bmp.setTemperatureOversampling(BMP5XX_OVERSAMPLING_2X);
  bmp.setPressureOversampling(BMP5XX_OVERSAMPLING_16X);
  bmp.setIIRFilterCoeff(BMP5XX_IIR_FILTER_COEFF_3);
  bmp.setOutputDataRate(BMP5XX_ODR_100_2_HZ); // Output rate ~100 Hz
  bmp.setPowerMode(BMP5XX_POWERMODE_CONTINUOUS);

  // --- Initialize MMC5983MA ---
  if (myMag.begin(CS_MMC) == false) {
    Serial.println("Failed: MMC5983MA initialization unsuccessful.");
    while(1) delay(10);
  }
  myMag.softReset();
  delay(20);
  myMag.enableAutomaticSetReset();

  // *** NEW: Put MMC5983MA into Continuous Mode ***
  // Valid frequencies: 1000, 200, 100, 50, 20, 10, 1, 0 (off)
  myMag.setContinuousModeFrequency(100);
  myMag.enableContinuousMode();

  Serial.println("Success: All 3 sensors initialized okay.");

  // Prime the timers
  lastPollTime = micros();
  lastPrintTime = micros();
}

void loop() {
  unsigned long currentMicros = micros();

  // Trigger exactly every 10,000 microseconds (100 Hz)
  if (currentMicros - lastPollTime >= pollInterval) {
    // Add interval to lastPollTime rather than setting it to currentMicros
    // to strictly prevent timing drift over time.
    lastPollTime += pollInterval;

    // 1. Poll ICM42688
    IMU.getAGT();

    // 2. Poll BMP5xx
    bmp.performReading();

    // 3. Poll MMC5983MA (Non-blocking continuous read)
    uint32_t currentX, currentY, currentZ;

    // Instantly grabs whatever is in the registers without waiting
    myMag.readFieldsXYZ(&currentX, &currentY, &currentZ);

    // Clear the hardware flag so the sensor knows it can push the next reading
    myMag.clearMeasDoneInterrupt();

    pollCount++;
  }

  // Trigger every 5,000,000 microseconds (5 seconds)
  if (currentMicros - lastPrintTime >= printInterval) {
    float averageFreq = (float)pollCount / 5.0;

    Serial.print("Average Polling Frequency: ");
    Serial.print(averageFreq);
    Serial.println(" Hz");

    // Reset counters
    pollCount = 0;
    lastPrintTime = currentMicros;
  }
}