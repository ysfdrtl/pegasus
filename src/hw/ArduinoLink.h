#pragma once
// Connection to the Pegasus Arduino firmware (arduino/pegasus_arm/pegasus_arm.ino).
//
// All serial I/O runs on a worker thread so the UI never blocks: connecting (the Uno resets when
// the port is opened and needs ~1.5 s), heart-beat (PING every 0.5 s; no answer for 2 s = link
// lost), streaming servo targets and reading replies. The editor only sets the latest targets and
// reads a status snapshot.
//
// Streaming: with streaming enabled the worker sends "M <seq> <ms> a1..a6" at up to 50 Hz whenever
// the targets change, using the free queue slots reported in the firmware's "A" replies for flow
// control (latest target wins, nothing piles up).

#include "hw/SerialPort.h"

#include <array>
#include <atomic>
#include <chrono>
#include <deque>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

namespace pegasus {

constexpr int kServoChannels = 6;

enum class LinkState { Disconnected = 0, Connecting = 1, Connected = 2, Error = 3 };

struct LinkStatus {
    LinkState state = LinkState::Disconnected;
    std::string port;
    std::string message;   // what is going on / why it failed (Turkish)
    std::string firmware;  // e.g. "pegasus-arm 1.0"
    float rttMs = 0.0f;    // PING round trip
    float lastRxAgeMs = 0.0f;
    long framesSent = 0, acks = 0, errors = 0, resets = 0;
    int freeSlots = 0;
    bool streaming = false;
    bool havePositions = false, moving = false;
    // false until the user sends HOME: the board does not drive the servos before that (firmware 1.1+)
    bool homed = false;
    std::array<float, kServoChannels> servoDeg{}; // positions reported by the board (degrees)
    std::chrono::steady_clock::time_point connectedSince{};
};

class ArduinoLink {
public:
    ArduinoLink();
    ~ArduinoLink();
    ArduinoLink(const ArduinoLink&) = delete;
    ArduinoLink& operator=(const ArduinoLink&) = delete;

    void connect(const std::string& port, int baud);
    void disconnect();
    LinkStatus status() const;
    bool connected() const { return status().state == LinkState::Connected; }

    // Latest servo targets in degrees (0..180). Only sent while streaming is on.
    void setTargets(const std::array<float, kServoChannels>& deg);
    void setStreaming(bool on);
    // Raw protocol line (e.g. "HOME", "S 3 900", "V 60"). Queued, sent in order once connected.
    void send(const std::string& line);
    // Emergency stop: drops queued commands and streaming, sends STOP right away.
    void stop();

    // Communication log (newest last): "> PING", "< PONG", "! ..." for notes.
    std::vector<std::string> log() const;
    void clearLog();
    bool logStreamFrames = false; // also log every M / A line (noisy)

private:
    void run();
    void handleLine(const std::string& line);
    void addLog(const std::string& s);
    bool writeLine(const std::string& s);
    void fail(const std::string& msg);

    mutable std::mutex mtx_;
    LinkStatus st_;
    std::deque<std::string> log_;
    std::deque<std::string> outbox_;
    std::array<float, kServoChannels> targets_{};
    bool targetsSet_ = false, targetsDirty_ = false, stopRequested_ = false;
    std::string reqPort_;
    int reqBaud_ = 115200;
    bool reqConnect_ = false, reqDisconnect_ = false;

    // worker-only state
    SerialPort port_;
    std::string rx_;
    std::chrono::steady_clock::time_point connectStart_, lastRx_, lastPing_, pingSent_, lastFrame_, lastGet_;
    bool pingPending_ = false, gotReady_ = false, infoSent_ = false;
    int seq_ = 0, inflight_ = 0;
    std::array<float, kServoChannels> lastSent_{};
    bool sentOnce_ = false;

    std::atomic<bool> quit_{false};
    std::thread thread_;
};

} // namespace pegasus
