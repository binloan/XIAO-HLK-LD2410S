# HLK-LD2410S
Arduino library for **low power human presence sensor**  module **HLK-LD2410S**

[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](LICENSE)
[![GitHub stars](https://img.shields.io/github/stars/phuongnamzz/HLK-LD2410S)](https://github.com/phuongnamzz/HLK-LD2410S/stargazers)
[![GitHub forks](https://img.shields.io/github/forks/phuongnamzz/HLK-LD2410S)](https://github.com/phuongnamzz/HLK-LD2410S/network)

![Platforms](https://img.shields.io/badge/Platforms-ESP32%20%7C%20AVR%20%7C%20STM32-green)
![Version](https://img.shields.io/github/v/release/phuongnamzz/HLK-LD2410S?color=brightgreen)
![Arduino Library](https://img.shields.io/badge/Arduino-Library-yellow)
![PlatformIO](https://img.shields.io/badge/PlatformIO-Library-orange)

📁 [📄 Official Documents & Driver Files](https://drive.google.com/drive/folders/1lCQv3mfHJ3XKXzweeHPFnJ_8_D_EWEKk)

---

## 🧭 Module Types
![Example pinout module](https://raw.githubusercontent.com/phuongnamzz/HLK-LD2410S/main/resources/HLK-2410S_pinout.png)

---

## Overview
The **HLK-LD2410S** is a battery-powered ultra-low-power milimeter-wave sensor based on Hi-Link's milimeter-wave sensor chip.
- Intergrated intelligent milimeter wave sensor SoC 24GHz 
- 

## ⚠️ Important Notes
- Power supply 3.0 -3.6V , recommend 3.3V
- Average working current max 0.6mA 
- Max range detection range 10m

- Interface with sensor
    Basic procedure:
    - 1.Enter the command mode. 
    - 2.Set parameter commands or obtain parameter commands. 
    - 3.Exit the commandhttps://www.openhardware.io/explore mode. 
    - HLK-LD2410S data communication uses a small-endian format, and all data in the following table is
    hexadecimal.

## 🧷 Pin Description

![Pin description](https://raw.githubusercontent.com/phuongnamzz/HLK-LD2410S/main/resources/pinout.png)


## 🔗 ESP32 Connection Table

| 🆔 No | 📟 LD6002 Pin | ⚙️ Function         | 📲 ESP32 Pin   |
|:----:|:-------------:|:------------------:|:-------------:|
|  1   | **3V3**       | ⚡ Power Input      | **3V3**        |
|  2   | **GND**       | 🛑 Ground           | **GND**        |
|  3   | **TX0**       | 📤 Serial TX        | **GPIO16**     |
|  4   | **RX0**       | 📥 Serial RX        | **GPIO17**     |
|  5   | **OT2**       | 📥IO                | **NC**         |
---


## 📟 Serial Protocols

Baudrate: 115200_8N1

### - Switch the module output mode
![Switch the module output mode](https://raw.githubusercontent.com/phuongnamzz/HLK-LD2410S/main/resources/Switch the module output mode.png)


### - Switch the module output mode
![Switch the module output mode](https://raw.githubusercontent.com/phuongnamzz/HLK-LD2410S/main/resources/pinout.png)

### - Switch the module output mode
![Switch the module output mode](https://raw.githubusercontent.com/phuongnamzz/HLK-LD2410S/main/resources/pinout.png)


### - Switch the module output mode
![Switch the module output mode](https://raw.githubusercontent.com/phuongnamzz/HLK-LD2410S/main/resources/pinout.png)



### - Switch the module output mode
![Switch the module output mode](https://raw.githubusercontent.com/phuongnamzz/HLK-LD2410S/main/resources/pinout.png)



### - Switch the module output mode
![Switch the module output mode](https://raw.githubusercontent.com/phuongnamzz/HLK-LD2410S/main/resources/pinout.png)



### - Switch the module output mode
![Switch the module output mode](https://raw.githubusercontent.com/phuongnamzz/HLK-LD2410S/main/resources/pinout.png)



### - Switch the module output mode
![Switch the module output mode](https://raw.githubusercontent.com/phuongnamzz/HLK-LD2410S/main/resources/pinout.png)


### - Switch the module output mode
![Switch the module output mode](https://raw.githubusercontent.com/phuongnamzz/HLK-LD2410S/main/resources/pinout.png)


### - Switch the module output mode
![Switch the module output mode](https://raw.githubusercontent.com/phuongnamzz/HLK-LD2410S/main/resources/pinout.png)

## 📦 Example Code (ESP32)
``` cpp

#include <Arduino.h>
#include "LD2410S.h"

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
```

```
✅ Result:
```

```
Starting
Firmware Version: 1.1.1
Serial Number: 12345678
Read common parameters success
Target Distance: 0 cm
Static
Target Distance: 0 cm
Static
Target Distance: 0 cm
Static
Target Distance: 35 cm
Motion
Target Distance: 35 cm
Motion
Target Distance: 35 cm
Motion
Target Distance: 35 cm
Motion
Target Distance: 35 cm
Motion
Target Distance: 35 cm
Motion
```