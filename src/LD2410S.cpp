#include "LD2410S.h"

/*
    Command frame : FD FC FB FA xx xx ... xx 04 03 02 01
*/


/**
 * @brief Constructs an LD2410S object with the specified serial interface and pin configuration.
 *
 * Initializes the LD2410S sensor driver, associating it with the provided HardwareSerial instance
 * and configuring the RX and TX pins for communication.
 *
 * @param serial Reference to a HardwareSerial object used for UART communication with the sensor.
 * @param rxPin GPIO pin number used for receiving data from the sensor.
 * @param txPin GPIO pin number used for transmitting data to the sensor.
 */
LD2410S::LD2410S(HardwareSerial &serial, int rxPin, int txPin)
    : _serial(serial), _rxPin(rxPin), _txPin(txPin), _motion(false), _distance(0) {}


/**
 * @brief Destructor for the LD2410S class.
 *
 * Cleans up any resources allocated by the LD2410S instance.
 * As this destructor is defaulted, no custom cleanup is performed.
 */
LD2410S::~LD2410S() = default;


/**
 * @brief Initializes the LD2410S sensor and configures its communication parameters.
 *
 * This function sets up the serial communication with the specified baud rate and pin configuration,
 * attempts to enter configuration mode (with up to two retries), writes generic parameters to the sensor,
 * and switches the sensor to minimal data output mode. Delays are used to ensure proper timing between operations.
 *
 * @param baudrate The baud rate for serial communication with the LD2410S sensor.
 * @return true if initialization and configuration succeed; false otherwise.
 */
bool LD2410S::begin(uint32_t baudrate)
{
    _serial.begin(baudrate, SERIAL_8N1, _rxPin, _txPin);
    delay(100);

    for (int i = 0; i < 2; ++i)
    {
        if (enterConfigMode())
            break;
        LD2410S_LOG_LINE("Enter config mode failed!. Try again");
        delay(100);
        if (i == 1)
            return false;
    }

    delay(100);
    if (!writeGenericParametersCommand())
    {
        LD2410S_LOG_LINE("Write generic parameters failed!");
        return false;
    }

    if (!switchToMinimalMode())
    {
        LD2410S_LOG_LINE("Set to minimal data output mode failed!");
    }
    delay(100);
    return true;
}

/**
 * @brief Switches the LD2410S device to standard output mode.
 *
 * This function sends the appropriate command sequence to the LD2410S device
 * over the configured serial interface to switch it into standard data output mode.
 * It waits for an acknowledgment frame from the device to confirm the mode change.
 *
 * @note The function waits up to 1 second for the acknowledgment response.
 *
 * @return true if the device successfully switches to standard output mode, false otherwise.
 */
bool LD2410S::switchToStandardMode()
{
    sendFrameHead();

    _serial.write(0x08);
    _serial.write(0x00);
    _serial.write(0x7A);
    _serial.write(0x00);

    const uint8_t param[6] = {0x00, 0x00, 0x01, 0x00, 0x00, 0x00};
    _serial.write(param, sizeof(param));

    sendFrameEnd();

    unsigned long start = millis();
    while (millis() - start < 1000)
    {
        if (_serial.available() >= 14)
        {
            uint8_t ack[14];
            _serial.readBytes(ack, sizeof(ack));
            LD2410S_LOG_LINE("Switch to standard output mode data:");
            for (uint8_t i = 0; i < sizeof(ack); ++i)
            {
                LD2410S_LOG("%02X ", ack[i]);
            }
            if (ack[0] == 0xFD && ack[1] == 0xFC && ack[2] == 0xFB && ack[3] == 0xFA && ack[6] == 0x7A && ack[7] == 0x01)
            {
                LD2410S_LOG_LINE("Change to standard data output mode success");
                return true;
            }
        }
    }
    return false;
}

/**
 * @brief Switches the LD2410S device to minimal output mode.
 *
 * This function sends a command frame to the LD2410S device to change its output mode to minimal.
 * It writes the appropriate command and parameters to the serial interface, then waits for an
 * acknowledgment response from the device. The function checks the response for specific header
 * and command bytes to confirm the mode change was successful.
 *
 * @return true if the device successfully switches to minimal output mode, false otherwise.
 */
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

/**
 * @brief read data serial continously
 */
/**
 * @brief Processes incoming serial data to detect and parse minimal data frames from the LD2410S sensor.
 *
 * This function should be called repeatedly (typically in the main loop) to read bytes from the serial interface.
 * It accumulates bytes into a buffer when a frame start byte (0x6E) is detected, and continues until a frame end
 * byte (0x62) is found or the maximum frame size is reached. Upon detecting a complete frame, it logs the frame
 * contents and invokes the minimal frame parser.
 *
 * The function is non-blocking and processes all available bytes in the serial buffer during each call.
 *
 * @note The function uses static variables to maintain state between calls, allowing it to handle frames that
 *       span multiple invocations.
 */
void LD2410S::loop()
{
    static constexpr size_t MAX_FRAME_SIZE = 64;
    static uint8_t buffer[MAX_FRAME_SIZE];
    static size_t pos = 0;
    static bool inFrame = false;

    while (_serial.available())
    {
        uint8_t byte = _serial.read();

        if (!inFrame)
        {
            if (byte == 0x6E)
            {
                inFrame = true;
                pos = 0;
                buffer[pos++] = byte;
            }
        }
        else
        {
            buffer[pos++] = byte;

            if (byte == 0x62 || pos >= MAX_FRAME_SIZE)
            {
                LD2410S_LOG_LINE("Minimal Frame:");
                for (size_t i = 0; i < pos; i++)
                {

                        LD2410S_LOG("%02X ", buffer[i]);
                }
                LD2410S_LOG_LINE();

                parseMinimalFrame(buffer, pos);

                inFrame = false;
                pos = 0;
            }
        }
    }
}

/**
 * @brief extract minimal frame 6E ... 62
 * @param frame
 * @param len
 * @return true
 * @return false
 */
/**
 * @brief Parses a minimal data frame from the LD2410S sensor and updates motion and distance state.
 *
 * This function checks the validity of the provided frame based on expected header and footer bytes.
 * It then interprets the target state to determine if motion is detected, and extracts the distance value.
 *
 * @param frame Pointer to the buffer containing the frame data.
 * @param len Length of the frame buffer. Must be at least 5 bytes.
 * @return true if the frame is valid and successfully parsed; false otherwise.
 *
 * The function updates the internal motion state (`_motion`) and distance (`_distance`) members.
 * - Motion is set to true if the target state is 2 or 3, false if 0 or 1.
 * - Distance is extracted from bytes 2 and 3 (little-endian).
 */
bool LD2410S::parseMinimalFrame(uint8_t *frame, size_t len)
{
    if (len < 5)
        return false;

    if (frame[0] != 0x6E || frame[4] != 0x62)
        return false;

    uint8_t target_state = frame[1];

    if (target_state == 0 || target_state == 1)
    {
        _motion = false;
    }
    else if (target_state == 2 || target_state == 3)
    {
        _motion = true;
    }
    else
    {
        return false;
    }

    _distance = (static_cast<uint16_t>(frame[3]) << 8) | frame[2];
    return true;
}


/**
 * @brief Checks if motion has been detected by the LD2410S sensor.
 *
 * @return true if motion is currently detected; false otherwise.
 */
bool LD2410S::isMotionDetected() const
{
    return _motion;
}


/**
 * @brief Retrieves the last measured distance value.
 *
 * This method returns the most recent distance measurement obtained by the LD2410S sensor.
 *
 * @return The distance value in units defined by the sensor (typically millimeters).
 */
uint16_t LD2410S::getDistance() const
{
    return _distance;
}


/**
 * @brief Sends the frame header sequence to the connected serial device.
 *
 * This function writes a predefined 4-byte header (0xFD, 0xFC, 0xFB, 0xFA)
 * to the serial interface. The header is typically used to indicate the
 * start of a new data frame in communication with the LD2410S device.
 */
void LD2410S::sendFrameHead()
{
    static const uint8_t head[4] = {0xFD, 0xFC, 0xFB, 0xFA};
    _serial.write(head, sizeof(head));
}


/**
 * @brief Sends the end-of-frame sequence to the connected device.
 *
 * This function writes a predefined 4-byte tail sequence (0x04, 0x03, 0x02, 0x01)
 * to the serial interface, indicating the end of a data frame for the LD2410S device.
 */
void LD2410S::sendFrameEnd()
{
    static const uint8_t tail[4] = {0x04, 0x03, 0x02, 0x01};
    _serial.write(tail, sizeof(tail));
}


/**
 * @brief Sends the command to enter configuration mode on the LD2410S device.
 *
 * This function transmits a specific command frame to the LD2410S device via the serial interface
 * to request entry into configuration mode. It waits for up to 500 milliseconds for an 18-byte
 * acknowledgment frame from the device. The function checks specific bytes in the response to
 * verify successful entry into configuration mode.
 *
 * @return true if the device successfully enters configuration mode and the expected acknowledgment is received; false otherwise.
 */
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


/**
 * @brief Exits the configuration mode of the LD2410S device.
 *
 * Sends the appropriate command to the device to exit configuration mode and waits for an acknowledgment.
 * The function waits up to 500 milliseconds for a response from the device. If a valid acknowledgment
 * frame is received, it logs a success message and returns true. Otherwise, it returns false.
 *
 * @return true if the device successfully exits configuration mode, false otherwise.
 */
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


/**
 * @brief Sends a command to write generic parameters to the LD2410S device.
 *
 * This function constructs and sends a command frame to the LD2410S sensor to set various generic parameters,
 * including farthest and nearest detection distances, delay time, status report frequency, distance report frequency,
 * and response speed. The function waits for an acknowledgment from the device and verifies the response.
 *
 * @return true if the parameters were successfully written and acknowledged by the device; false otherwise.
 */
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


/**
 * @brief Parses a 32-bit unsigned integer from a little-endian byte array.
 *
 * This function takes a pointer to a 4-byte array and interprets the bytes as a
 * 32-bit unsigned integer in little-endian order (least significant byte first).
 *
 * @param data Pointer to an array of at least 4 bytes containing the little-endian data.
 * @return The parsed 32-bit unsigned integer.
 */
static uint32_t parseLEUint32(const uint8_t *data)
{
    return static_cast<uint32_t>(data[0]) |
           (static_cast<uint32_t>(data[1]) << 8) |
           (static_cast<uint32_t>(data[2]) << 16) |
           (static_cast<uint32_t>(data[3]) << 24);
}


/**
 * @brief Reads the common parameters from the LD2410S sensor.
 *
 * This function sends a command to the LD2410S sensor to request its common parameters,
 * waits for a response, and parses the returned data. The parameters retrieved include
 * the farthest and nearest detection distances, delay time, frequency status, frequency
 * distance, and response speed. The function returns true if the parameters are successfully
 * read and parsed, otherwise returns false.
 *
 * @param[out] farthest_distance The farthest detection distance reported by the sensor.
 * @param[out] nearest_distance The nearest detection distance reported by the sensor.
 * @param[out] delay_time The delay time parameter from the sensor.
 * @param[out] freq_status The frequency status parameter from the sensor.
 * @param[out] freq_distance The frequency distance parameter from the sensor.
 * @param[out] respond_speed The response speed parameter from the sensor.
 * @return true if the parameters were successfully read and parsed, false otherwise.
 */
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


/**
 * @brief Reads the firmware version from the LD2410S device.
 *
 * This function sends a command to the LD2410S device to request its firmware version.
 * It waits for a response and parses the major, minor, and patch version numbers from the received data.
 *
 * @param[out] major Reference to a uint16_t variable where the major version will be stored.
 * @param[out] minor Reference to a uint16_t variable where the minor version will be stored.
 * @param[out] patch Reference to a uint16_t variable where the patch version will be stored.
 * @return true if the firmware version was successfully read and parsed, false otherwise.
 */
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


/**
 * @brief Reads the serial number from the LD2410S device via the serial interface.
 *
 * This function sends a command to the LD2410S device to request its serial number,
 * waits for the response, and extracts the serial number from the received data.
 * The serial number is expected to be 8 bytes long and will be stored as a null-terminated
 * string in the provided buffer.
 *
 * The function first clears the serial buffer, sends the serial number request command,
 * and waits up to 1 second for a valid response. If a valid response is received,
 * the serial number is extracted and copied into the buffer.
 *
 * @param[out] buffer Pointer to a character array where the serial number will be stored.
 *                    The buffer must be at least 9 bytes long to accommodate the 8-byte
 *                    serial number and the null terminator.
 * @param[in] len     The length of the provided buffer.
 * @return true if the serial number was successfully read and stored in the buffer,
 *         false otherwise (e.g., if the buffer is too small or a valid response was not received).
 */
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
