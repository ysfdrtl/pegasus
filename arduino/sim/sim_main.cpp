// Pegasus Arduino simulator: runs the real firmware (pegasus_arm.ino) on the PC behind a pseudo
// terminal. Usage:  pegasus_arduino_sim [link-path]   (default /tmp/pegasus-arduino-sim)
// Then connect from the app to that path. Like a real Uno, the "board" resets (prints READY again)
// whenever the port is opened.
#include "Arduino.h"

#include <cerrno>
#include <chrono>
#include <csignal>
#include <cstdio>
#include <fcntl.h>
#include <poll.h>
#include <termios.h>
#include <thread>
#include <unistd.h>

SimSerial Serial;

static const auto t0 = std::chrono::steady_clock::now();
unsigned long millis() {
    return (unsigned long)std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - t0).count();
}

int SimSerial::available() { return int(in_.size()); }
int SimSerial::read() {
    if (in_.empty()) return -1;
    const int c = (unsigned char)in_[0];
    in_.erase(0, 1);
    return c;
}
void SimSerial::flush() {
    if (fd < 0 || out_.empty() || hungUp) {
        if (hungUp) out_.clear();
        return;
    }
    const ssize_t n = ::write(fd, out_.data(), out_.size());
    if (n > 0) out_.erase(0, size_t(n));
}
void SimSerial::pump() {
    char buf[256];
    while (true) {
        const ssize_t n = ::read(fd, buf, sizeof(buf));
        if (n > 0) {
            hungUp = false;
            in_.append(buf, size_t(n));
            continue;
        }
        if (n < 0 && errno == EIO) hungUp = true; // no program has the port open
        break;
    }
}

// the firmware
#include "../pegasus_arm/pegasus_arm.ino"

static volatile bool running = true;
static void onSignal(int) { running = false; }

int main(int argc, char** argv) {
    const std::string link = argc > 1 ? argv[1] : "/tmp/pegasus-arduino-sim";
    const int m = posix_openpt(O_RDWR | O_NOCTTY);
    if (m < 0 || grantpt(m) != 0 || unlockpt(m) != 0) {
        std::perror("posix_openpt");
        return 1;
    }
    fcntl(m, F_SETFL, fcntl(m, F_GETFL) | O_NONBLOCK);
    const std::string slave = ptsname(m);
    { // raw line discipline (no echo), like a real USB serial port
        const int sfd = open(slave.c_str(), O_RDWR | O_NOCTTY);
        struct termios t;
        if (sfd >= 0 && tcgetattr(sfd, &t) == 0) {
            cfmakeraw(&t);
            tcsetattr(sfd, TCSANOW, &t);
        }
        if (sfd >= 0) close(sfd);
    }
    unlink(link.c_str());
    if (symlink(slave.c_str(), link.c_str()) != 0) std::perror("symlink");
    std::printf("[sim] Pegasus Arduino simulatoru: %s -> %s\n", link.c_str(), slave.c_str());
    std::fflush(stdout);
    std::signal(SIGINT, onSignal);
    std::signal(SIGTERM, onSignal);
    Serial.fd = m;
    Serial.hungUp = true;
    bool wasOpen = false;
    while (running) {
        Serial.pump();
        // a program opened the port: reset the board like the Uno's DTR auto-reset does
        struct pollfd p{m, POLLIN, 0};
        poll(&p, 1, 0);
        const bool open = !(p.revents & POLLHUP);
        if (open && !wasOpen) {
            Serial.hungUp = false;
            Serial.in_.clear();
            std::this_thread::sleep_for(std::chrono::milliseconds(300)); // bootloader delay
            Serial.pump();
            Serial.in_.clear();
            setup();
            std::printf("[sim] port acildi, kart sifirlandi\n");
            std::fflush(stdout);
        }
        if (!open && wasOpen) {
            std::printf("[sim] port kapandi\n");
            std::fflush(stdout);
        }
        wasOpen = open;
        if (open) loop();
        Serial.flush();
        std::this_thread::sleep_for(std::chrono::microseconds(500));
    }
    unlink(link.c_str());
    return 0;
}
