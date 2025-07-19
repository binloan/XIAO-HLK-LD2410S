/**
 * @file LD2410S.h
 * @brief Driver library for LD2410S radar sensor (motion & distance)
 * @author PhuongNam720
 * @version 1.0
 * @date 2025-07-03
 */

#pragma once
#include <Arduino.h>

/**
 * @def LD2410S_ENABLE_LOG
 * @brief Enable or disable Serial debug logging (1 = enabled, 0 = disabled)
 */
#define LD2410S_ENABLE_LOG 0

#if LD2410S_ENABLE_LOG
#define LD2410S_LOG(...) Serial.printf(__VA_ARGS__)
#define LD2410S_LOG_LINE(x) Serial.println(x)
#else
#define LD2410S_LOG(...)
#define LD2410S_LOG_LINE(x)
#endif

/**
 * @class LD2410S
 * @brief Class to communicate and configure the LD2410S radar sensor.
 */
class LD2410S
{
public:
    /**
     * @brief Constructor
     * @param serial Reference to HardwareSerial object
     */
    LD2410S(Uart &serial);

    /**
     * @brief Destructor
     */
    ~LD2410S();

    /**
     * @brief Initialize serial communication and enter config mode
     * @param baudrate Serial baudrate (default = 115200)
     * @return true if successful
     */
    bool begin(uint32_t baudrate = 115200); // set default baudrate 115200

    /**
     * @brief Read and process incoming frames (non-blocking)
     */
    void loop();

    /**
     * @brief Enter configuration mode
     * @return true if success
     */
    bool enterConfigMode();

    /**
     * @brief Exit configuration mode
     * @return true if success
     */
    bool exitConfigMode();

    /**
     * @brief Switch to standard output mode (detailed)
     * @return true if success
     */
    bool switchToStandardMode();

    /**
     * @brief Switch to minimal output mode (shorter packets)
     * @return true if success
     */
    bool switchToMinimalMode();

    /**
     * @brief Read firmware version of the sensor
     * @param major Major version
     * @param minor Minor version
     * @param patch Patch version
     * @return true if response received
     */
    bool readFirmwareVersion(uint16_t &major, uint16_t &minor, uint16_t &patch);

    /**
     * @brief Read configuration parameters from sensor
     * @param farthest_distance Max detection distance
     * @param nearest_distance Min detection distance
     * @param delay_time Delay before entering static state
     * @param freq_status Report frequency for motion/static state
     * @param freq_distance Report frequency for distance
     * @param respond_speed Sensor response speed (normal/fast)
     * @return true if successful
     */
    bool readCommonParametersCommand(uint32_t &farthest_distance, uint32_t &nearest_distance, uint32_t &delay_time, uint32_t &freq_status, uint32_t &freq_distance, uint32_t &respond_speed);

    /**
     * @brief Read 8-byte serial number
     * @param buffer Buffer to store null-terminated serial number
     * @param len Buffer size (must be at least 9)
     * @return true if successful
     */
    bool readSerialNumber(char *buffer, size_t len);

    /**
     * @brief Check if motion is currently detected
     * @return true if motion detected
     */
    bool isMotionDetected() const;

    /**
     * @brief Get target distance
     * @return Distance in cm
     */
    uint16_t getDistance() const;

private:
    /**
     * @brief Parse minimal output frame
     * @param frame Data frame
     * @param len Length
     * @return true if successfully parsed
     */
    bool parseMinimalFrame(uint8_t *frame, size_t len);

    /**
     * @brief Write frame header: FD FC FB FA
     */
    void sendFrameHead();

    /**
     * @brief Write frame footer: 04 03 02 01
     */
    void sendFrameEnd();

    /**
     * @brief Write default configuration parameters to sensor
     * @return true if ACK received
     */
    bool writeGenericParametersCommand();

    Uart &_serial; ///< Reference to serial port
    int _rxPin;              ///< GPIO used for sensor TX
    int _txPin;              ///< GPIO used for sensor RX
    bool _motion = false;    ///< Internal motion state
    uint16_t _distance = 0;  ///< Last known distance
};