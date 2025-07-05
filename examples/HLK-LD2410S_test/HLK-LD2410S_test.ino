#include <Arduino.h>
#include "HLK-LD2410S.h"

// Pin definitions
constexpr int RX_PIN = 16;
constexpr int TX_PIN = 17;
constexpr size_t SERIAL_NUMBER_BUFFER_SIZE = 16;

// Sensor object
LD2410S ld2410s(Serial1, RX_PIN, TX_PIN);

// Firmware version variables
uint16_t firmwareMajor = 0;
uint16_t firmwareMinor = 0;
uint16_t firmwarePatch = 0;

// Serial number buffer
char serialNumberBuffer[SERIAL_NUMBER_BUFFER_SIZE];

// Common parameters
uint32_t farDistance = 0;
uint32_t nearDistance = 0;
uint32_t delayTime = 0;
uint32_t freqStatus = 0;
uint32_t freqDistance = 0;
uint32_t responseTime = 0;

// Timing variables
unsigned long lastStatusMillis = 0;

void setup() {
  Serial.begin(115200);
  ld2410s.begin();
  delay(1000);

  Serial.println("Starting");

  // Read firmware version
  if (ld2410s.readFirmwareVersion(firmwareMajor, firmwareMinor, firmwarePatch)) {
    Serial.printf("Firmware Version: %d.%d.%d\n", firmwareMajor, firmwareMinor, firmwarePatch);
  } else {
    Serial.println("Failed to read firmware version");
  }

  // Read serial number
  if (ld2410s.readSerialNumber(serialNumberBuffer, sizeof(serialNumberBuffer))) {
    Serial.printf("Serial Number: %s\n", serialNumberBuffer);
  } else {
    Serial.println("Failed to read serial number.");
  }

  // Read common parameters
  if (ld2410s.readCommonParametersCommand(farDistance, nearDistance, delayTime, freqStatus, freqDistance, responseTime)) {
    Serial.println("Read common parameters success");
  } else {
    Serial.println("Failed to read common parameters");
  }

  lastStatusMillis = millis();
}

void loop() {
  ld2410s.loop();

  // Print status every second
  if (millis() - lastStatusMillis >= 1000) {
    Serial.printf("Target Distance: %d cm\n", ld2410s.getDistance());
    Serial.println(ld2410s.isMotionDetected() ? "Motion" : "Static");
    lastStatusMillis = millis();
  }
}