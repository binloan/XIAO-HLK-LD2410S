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

<p float="left">
  <img src="https://raw.githubusercontent.com/phuongnamzz/HLK-LD2410S/main/resources/HLK-2410S_pinout.png" width="45%" />
  <img src="https://raw.githubusercontent.com/phuongnamzz/HLK-LD2410S/main/resources/LD2410S.jpg" width="45%" />
</p>

---


{:toc}
## Overview
The **HLK-LD2410S** is a battery-powered ultra-low-power milimeter-wave sensor based on Hi-Link's milimeter-wave sensor chip.
- Intergrated intelligent milimeter wave sensor SoC 24GHz 
- Uses millimeter wave sensor distance measurement technology and ICL1112 chip
advanced proprietary radar signal processing and low power control technology to achieve accurate
perception of moving, micromotion and standing human body.

## ⚠️ Important Notes
- Power supply 3.0 -3.6V , recommend 3.3V
- Average working current max 0.6mA 
- Max range detection range 10m

- Interface with sensor
    Basic procedure:
    - 1.Enter the command mode. 
    - 2.Set parameter commands or obtain parameter commands. 
    - 3.Exit the command mode. 
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

### *- Baudrate: 115200_8N1*

### - Switch the module output mode
![Switch the module output mode](https://raw.githubusercontent.com/phuongnamzz/HLK-LD2410S/main/resources/Switch%20the%20module%20output%20mode.png)

``` cpp

bool LD2410S::switchToMinimalMode()
{
    sendFrameHead();

    _serial.write(0x08);
    _serial.write(0x00);
    _serial.write(0x7A);
    _serial.write(0x00);

    const uint8_t param[6] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
    _serial.write(param, sizeof(param));

    sendFrameEnd();

    unsigned long start = millis();
    while (millis() - start < 1000)
    {
        if (_serial.available() >= 14)
        {
            uint8_t ack[14];
            _serial.readBytes(ack, sizeof(ack));
            LD2410S_LOG_LINE("Switch to minimal output mode data:");
            for (uint8_t i = 0; i < sizeof(ack); ++i)
            {
                LD2410S_LOG("%02X ", ack[i]);
            }
            LD2410S_LOG_LINE("");
            if (ack[0] == 0xFD && ack[1] == 0xFC && ack[2] == 0xFB && ack[3] == 0xFA && ack[6] == 0x7A && ack[7] == 0x01)
            {
                LD2410S_LOG_LINE("Change to minimal data output mode success");
                return true;
            }
        }
    }
    return false;
}
```

### - Read the firmware version command
![Read the firmware version command](https://raw.githubusercontent.com/phuongnamzz/HLK-LD2410S/main/resources/Read%20the%20firmware%20version%20command.png)

``` cpp

bool LD2410S::readFirmwareVersion(uint16_t &major, uint16_t &minor, uint16_t &patch)
{
    while (_serial.available())
        _serial.read();

    sendFrameHead();
    const uint8_t cmd[] = {0x02, 0x00, 0x00, 0x00};
    _serial.write(cmd, sizeof(cmd));
    sendFrameEnd();
    _serial.flush();

    unsigned long start = millis();
    while (millis() - start < 1000)
    {
        if (_serial.available() >= 24)
        {
            uint8_t ack[24];
            _serial.readBytes(ack, sizeof(ack));
            LD2410S_LOG_LINE("Firmware version data:");
            for (int i = 0; i < 24; i++)
            {
                LD2410S_LOG("%02X ", ack[i]);
            }
            LD2410S_LOG_LINE();

            if (ack[0] == 0xFD && ack[1] == 0xFC && ack[2] == 0xFB && ack[3] == 0xFA &&
                ack[6] == 0x00 && ack[7] == 0x01)
            {
                major = (static_cast<uint16_t>(ack[15]) << 8) | ack[14];
                minor = (static_cast<uint16_t>(ack[17]) << 8) | ack[16];
                patch = (static_cast<uint16_t>(ack[19]) << 8) | ack[18];
                LD2410S_LOG_LINE("Read firmware version  Ok");
                LD2410S_LOG("\nmajor = %d, minor = %d, patch = %d", major, minor, patch);
                delay(50);
                return true;
            }
        }
    }
    delay(50);
    return false;
}
```

### - Enable configuration command
![Enable configuration command](https://raw.githubusercontent.com/phuongnamzz/HLK-LD2410S/main/resources/Enable%20configuration%20command.png)

``` cpp
bool LD2410S::enterConfigMode()
{
    sendFrameHead();
    const uint8_t cmd[] = {0x04, 0x00, 0xFF, 0x00, 0x01, 0x00};
    _serial.write(cmd, sizeof(cmd));
    sendFrameEnd();

    unsigned long start = millis();

    while (millis() - start < 500)
    {
        if (_serial.available() >= 18)
        {
            uint8_t ack[18];
            _serial.readBytes(ack, sizeof(ack));
            if (ack[0] == 0xFD && ack[1] == 0xFC && ack[2] == 0xFB && ack[3] == 0xFA &&
                ack[6] == 0xFF && ack[7] == 0x01 && ack[10] == 0x03 && ack[11] == 0x00)
            {
                LD2410S_LOG_LINE("Enter config mode Ok");
                return true;
            }
        }
    }
    return false;
}
```
### - End configuration command
![End configuration command](https://raw.githubusercontent.com/phuongnamzz/HLK-LD2410S/main/resources/End%20configuration%20command.png)

``` cpp
bool LD2410S::exitConfigMode()
{
    sendFrameHead();
    const uint8_t cmd[] = {0x02, 0x00, 0xFE, 0x00};
    _serial.write(cmd, sizeof(cmd));
    sendFrameEnd();

    unsigned long start = millis();
    while (millis() - start < 500)
    {
        if (_serial.available() >= 14)
        {
            uint8_t ack[14];
            _serial.readBytes(ack, sizeof(ack));
            if (ack[0] == 0xFD && ack[1] == 0xFC && ack[2] == 0xFB && ack[3] == 0xFA &&
                ack[6] == 0xFE && ack[7] == 0x01)
            {
                LD2410S_LOG_LINE("Exit config mode Ok");
                return true;
            }
        }
    }
    return false;
}
```

### - Write Serial number command
![Write Serial number command](https://raw.githubusercontent.com/phuongnamzz/HLK-LD2410S/main/resources/Write%20Serial%20number%20command.png)

``` cpp

```

### - Read serial number command
![Read serial number command](https://raw.githubusercontent.com/phuongnamzz/HLK-LD2410S/main/resources/Read%20serial%20number%20command.png)

``` cpp
bool LD2410S::readSerialNumber(char *buffer, size_t len)
{
    while (_serial.available())
        _serial.read();

    if (len < 9)
        return false;

    delay(100);

    sendFrameHead();
    const uint8_t cmd[] = {0x02, 0x00, 0x11, 0x00};
    _serial.write(cmd, sizeof(cmd));
    sendFrameEnd();
    _serial.flush();

    unsigned long start = millis();
    while (millis() - start < 1000)
    {
        if (_serial.available() >= 24)
        {
            uint8_t ack[24];
            _serial.readBytes(ack, sizeof(ack));

            LD2410S_LOG_LINE("Serial number data: ");
            for (int i = 0; i < 24; ++i)
                LD2410S_LOG("%02X ", ack[i]);
            LD2410S_LOG_LINE();

            if (ack[0] == 0xFD && ack[1] == 0xFC && ack[2] == 0xFB && ack[3] == 0xFA &&
                ack[6] == 0x11 && ack[7] == 0x01 &&
                ack[8] == 0x00 && ack[9] == 0x00)
            {
                for (uint8_t i = 0; i < 8; i++)
                    buffer[i] = static_cast<char>(ack[12 + i]);
                buffer[8] = '\0';
                delay(50);
                return true;
            }
        }
    }
    delay(50);
    return false;
}


```


### - Write generic parameter commands
![Write generic parameter commands](https://raw.githubusercontent.com/phuongnamzz/HLK-LD2410S/main/resources/Write%20generic%20parameter%20commands.png)

``` cpp

bool LD2410S::writeGenericParametersCommand()
{
    const uint8_t farthest_distance[6] = {0x05, 0x00, 0x01, 0x00, 0x00, 0x00};    // 1m
    const uint8_t nearest_distance[6] = {0x0A, 0x00, 0x00, 0x00, 0x00, 0x00};     // 0.0m
    const uint8_t delay_time[6] = {0x06, 0x00, 0x0A, 0x00, 0x00, 0x00};           // 10s
    const uint8_t freq_status_report[6] = {0x02, 0x00, 0x28, 0x00, 0x00, 0x00};   // 4.0 Hz
    const uint8_t freq_distance_report[6] = {0x0C, 0x00, 0x28, 0x00, 0x00, 0x00}; // 4.0Hz
    const uint8_t response_speed[6] = {0x0B, 0x00, 0x0A, 0x00, 0x00, 0x00};       // response normal

    sendFrameHead();

    const uint8_t cmd[] = {0x26, 0x00, 0x70, 0x00};
    _serial.write(cmd, sizeof(cmd));
    _serial.write(farthest_distance, sizeof(farthest_distance));
    _serial.write(nearest_distance, sizeof(nearest_distance));
    _serial.write(delay_time, sizeof(delay_time));
    _serial.write(freq_status_report, sizeof(freq_status_report));
    _serial.write(freq_distance_report, sizeof(freq_distance_report));
    _serial.write(response_speed, sizeof(response_speed));

    sendFrameEnd();
    _serial.flush();

    unsigned long start = millis();

    while (millis() - start < 1000)
    {
        if (_serial.available() >= 14)
        {
            uint8_t ack[14];
            _serial.readBytes(ack, sizeof(ack));
            LD2410S_LOG_LINE("Generic parameters data: ");
            for (int i = 0; i < 14; i++)
            {
                LD2410S_LOG("%02X ", ack[i]);
            }
            LD2410S_LOG_LINE("");
            if (ack[0] == 0xFD && ack[1] == 0xFC && ack[2] == 0xFB && ack[3] == 0xFA &&
                ack[6] == 0x70 && ack[7] == 0x01)
            {
                LD2410S_LOG_LINE("Write generic parameters Ok");
                return true;
            }
        }
    }
    return false;
}
```


### - Read common parameter commands
![Read common parameter commands](https://raw.githubusercontent.com/phuongnamzz/HLK-LD2410S/main/resources/Read%20common%20parameter%20commands.png)


``` cpp

bool LD2410S::readCommonParametersCommand(uint32_t &farthest_distance, uint32_t &nearest_distance, uint32_t &delay_time, uint32_t &freq_status, uint32_t &freq_distance, uint32_t &respond_speed)
{
    sendFrameHead();

    const uint8_t cmd[] = {
        0x0E, 0x00, 0x71, 0x00,
        0x05, 0x00, 0x0A, 0x00,
        0x06, 0x00, 0x02, 0x00,
        0x0C, 0x00, 0x0B, 0x00};
    _serial.write(cmd, sizeof(cmd));
    sendFrameEnd();

    unsigned long start = millis();

    while (millis() - start < 1000)
    {
        if (_serial.available() >= 38)
        {
            uint8_t ack[38];
            _serial.readBytes(ack, sizeof(ack));

            LD2410S_LOG_LINE("Commond parameters data: ");
            for (int i = 0; i < 38; i++)
            {
                LD2410S_LOG("%02X ", ack[i]);
            }

            if (ack[0] == 0xFD && ack[1] == 0xFC && ack[2] == 0xFB && ack[3] == 0xFA &&
                ack[6] == 0x71 && ack[7] == 0x01 && ack[8] == 0x00 && ack[9] == 0x00)
            {
                farthest_distance = parseLEUint32(&ack[10]);
                nearest_distance = parseLEUint32(&ack[14]);
                delay_time = parseLEUint32(&ack[18]);
                freq_status = parseLEUint32(&ack[22]);
                freq_distance = parseLEUint32(&ack[26]);
                respond_speed = parseLEUint32(&ack[30]);

                LD2410S_LOG("\nfarthest_distance = %d", farthest_distance);
                LD2410S_LOG("\nnearest_distance = %d", nearest_distance);
                LD2410S_LOG("\ndelay_time = %d", delay_time);
                LD2410S_LOG("\nfreq_status = %d", freq_status);
                LD2410S_LOG("\nfreq_distance = %d", freq_distance);
                LD2410S_LOG("\nrespond_speed = %d\n", respond_speed);

                return true;
            }
        }
    }
    return false;
}
```

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