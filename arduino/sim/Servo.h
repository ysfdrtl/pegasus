#pragma once
#include "Arduino.h"

class Servo {
public:
    void attach(int pin) { pin_ = pin; attached_ = true; }
    void detach() { attached_ = false; }
    void writeMicroseconds(int us) { us_ = us; }
    void write(int deg) { us_ = 544 + deg * (2400 - 544) / 180; }
    bool attached() const { return attached_; }
    int pin_ = -1, us_ = 1500;
    bool attached_ = false;
};
