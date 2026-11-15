// Host-side stand-in for the Arduino core, just enough to compile pegasus_arm.ino on Linux
// (see sim_main.cpp). Serial talks to a pseudo terminal, so the Pegasus app can connect to it
// exactly like to a real board.
#pragma once
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <string>

using std::fabs;

unsigned long millis();

class SimSerial {
public:
    void begin(unsigned long) {}
    int available();
    int read();
    void print(const char* s) { out_ += s; flush(); }
    void print(char c) { out_ += c; flush(); }
    void print(int v) { print(long(v)); }
    void print(long v) { out_ += std::to_string(v); flush(); }
    void println() { print("\r\n"); }
    void println(const char* s) { print(s); println(); }
    void println(long v) { print(v); println(); }
    void println(int v) { print(long(v)); println(); }

    // sim side
    int fd = -1;
    std::string in_, out_;
    void flush();
    void pump(); // read from the pty
    bool hungUp = false;
};
extern SimSerial Serial;
