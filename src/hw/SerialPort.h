#pragma once
// Minimal POSIX serial port (Linux): raw 8N1, non-blocking.

#include <string>
#include <vector>

namespace pegasus {

struct SerialPortInfo {
    std::string path;        // e.g. /dev/ttyACM0
    std::string description; // e.g. "Arduino Uno (usb-Arduino__www.arduino.cc__0043_...)"
    bool likelyArduino = false;
};

class SerialPort {
public:
    SerialPort() = default;
    ~SerialPort() { close(); }
    SerialPort(const SerialPort&) = delete;
    SerialPort& operator=(const SerialPort&) = delete;

    // Serial devices present on this machine (USB CDC / USB-serial adapters, plus the Pegasus
    // Arduino simulator if it is running). Arduino-like devices first.
    static std::vector<SerialPortInfo> list();

    // Opens and configures the port. On failure `err` holds a user-facing (Turkish) explanation.
    bool open(const std::string& path, int baud, std::string& err);
    void close();
    bool isOpen() const { return fd_ >= 0; }
    const std::string& path() const { return path_; }

    // Non-blocking. Returns bytes read (0 = nothing available), -1 when the device is gone.
    int read(char* buf, int size);
    // Writes everything (short blocking on a full buffer). False when the device is gone.
    bool write(const std::string& data);

private:
    int fd_ = -1;
    std::string path_;
};

} // namespace pegasus
