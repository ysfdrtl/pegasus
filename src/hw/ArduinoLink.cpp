#include "hw/ArduinoLink.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <sstream>

namespace pegasus {

namespace {
using Clock = std::chrono::steady_clock;
float msSince(Clock::time_point t) { return std::chrono::duration<float, std::milli>(Clock::now() - t).count(); }

constexpr float kConnectTimeoutMs = 4500.0f;
constexpr float kReadyWaitMs = 2200.0f;  // Uno bootloader after the DTR reset
constexpr float kPingEveryMs = 500.0f;
constexpr float kLostAfterMs = 2000.0f;
constexpr float kFrameEveryMs = 20.0f;   // streaming: max 50 frames/s
constexpr float kGetEveryMs = 250.0f;
} // namespace

ArduinoLink::ArduinoLink() { thread_ = std::thread([this] { run(); }); }

ArduinoLink::~ArduinoLink() {
    quit_ = true;
    if (thread_.joinable()) thread_.join();
}

void ArduinoLink::connect(const std::string& port, int baud) {
    std::lock_guard<std::mutex> lk(mtx_);
    reqPort_ = port;
    reqBaud_ = baud;
    reqConnect_ = true;
    reqDisconnect_ = false;
    st_.state = LinkState::Connecting;
    st_.port = port;
    st_.message = "Bağlanıyor...";
}

void ArduinoLink::disconnect() {
    std::lock_guard<std::mutex> lk(mtx_);
    reqDisconnect_ = true;
    reqConnect_ = false;
}

LinkStatus ArduinoLink::status() const {
    std::lock_guard<std::mutex> lk(mtx_);
    LinkStatus s = st_;
    if (s.state == LinkState::Connected) s.lastRxAgeMs = msSince(lastRx_);
    return s;
}

void ArduinoLink::setTargets(const std::array<float, kServoChannels>& deg) {
    std::lock_guard<std::mutex> lk(mtx_);
    if (!targetsSet_ || deg != targets_) targetsDirty_ = true;
    targets_ = deg;
    targetsSet_ = true;
}

void ArduinoLink::setStreaming(bool on) {
    std::lock_guard<std::mutex> lk(mtx_);
    if (on && !st_.streaming) targetsDirty_ = true;
    st_.streaming = on;
}

void ArduinoLink::send(const std::string& line) {
    std::lock_guard<std::mutex> lk(mtx_);
    outbox_.push_back(line);
}

void ArduinoLink::stop() {
    std::lock_guard<std::mutex> lk(mtx_);
    outbox_.clear();
    st_.streaming = false;
    stopRequested_ = true;
}

std::vector<std::string> ArduinoLink::log() const {
    std::lock_guard<std::mutex> lk(mtx_);
    return {log_.begin(), log_.end()};
}

void ArduinoLink::clearLog() {
    std::lock_guard<std::mutex> lk(mtx_);
    log_.clear();
}

void ArduinoLink::addLog(const std::string& s) { // mtx_ held
    log_.push_back(s);
    while (log_.size() > 400) log_.pop_front();
}

bool ArduinoLink::writeLine(const std::string& s) { // worker thread, mtx_ NOT held
    const bool quiet = s[0] == 'M' || s == "PING" || s == "GET";
    {
        std::lock_guard<std::mutex> lk(mtx_);
        if (!quiet || logStreamFrames) addLog("> " + s);
    }
    return port_.write(s + "\n");
}

void ArduinoLink::fail(const std::string& msg) { // worker thread, mtx_ NOT held
    port_.close();
    std::lock_guard<std::mutex> lk(mtx_);
    st_.state = LinkState::Error;
    st_.message = msg;
    st_.streaming = false;
    st_.havePositions = false;
    addLog("! " + msg);
}

void ArduinoLink::handleLine(const std::string& line) { // worker thread
    lastRx_ = Clock::now();
    std::istringstream is(line);
    std::string tag;
    is >> tag;
    std::lock_guard<std::mutex> lk(mtx_);
    const bool quiet = tag == "A" || tag == "PONG" || tag == "POS";
    if (!quiet || logStreamFrames) addLog("< " + line);
    if (tag == "READY") {
        gotReady_ = true;
        st_.homed = false; // a (re)started board leaves the servos free until HOME
        if (st_.state == LinkState::Connected) {
            // the board restarted while connected (power dip when the servos start, reset button)
            st_.resets++;
            addLog("! Kart yeniden başladı (servo beslemesi yetersiz olabilir: harici 5V kaynağı kontrol edin).");
            inflight_ = 0;
            sentOnce_ = false;
            targetsDirty_ = true;
        }
    } else if (tag == "PONG") {
        if (pingPending_) st_.rttMs = msSince(pingSent_);
        pingPending_ = false;
    } else if (tag == "INFO") {
        std::string name, ver;
        is >> name >> ver;
        if (name == "pegasus-arm") {
            st_.firmware = name + " " + ver;
            // 1.1+: "homed=0/1"; 1.0 drove the servos from power-on
            st_.homed = ver == "1.0";
            for (std::string kv; is >> kv;)
                if (kv.rfind("homed=", 0) == 0) st_.homed = kv.substr(6) == "1";
            if (st_.state == LinkState::Connecting) {
                st_.state = LinkState::Connected;
                st_.message = "Bağlı";
                st_.connectedSince = Clock::now();
                st_.freeSlots = 8;
                addLog("! Bağlandı: " + st_.firmware);
            }
        }
    } else if (tag == "A") {
        long seq = 0;
        int freeSlots = 0;
        is >> seq >> freeSlots;
        st_.acks++;
        st_.freeSlots = freeSlots;
        inflight_ = std::max(0, inflight_ - 1);
    } else if (tag == "POS") {
        int moving = 0;
        is >> moving;
        for (int i = 0; i < kServoChannels; ++i) {
            int v = 0;
            is >> v;
            st_.servoDeg[size_t(i)] = float(v) / 10.0f;
        }
        st_.moving = moving != 0;
        st_.havePositions = true;
        int freeSlots = -1;
        if (is >> freeSlots) { // queue state also arrives with every position report (no stall when the queue was full)
            st_.freeSlots = freeSlots;
            if (freeSlots == 8) inflight_ = 0;
        }
        int homed = 0;
        if (is >> homed) st_.homed = homed != 0;
    } else if (tag == "ERR") {
        st_.errors++;
        int code = 0;
        is >> code;
        if (code == 3) inflight_ = std::max(0, inflight_ - 1); // queue full: frame dropped
        if (code == 5) st_.streaming = false;                   // watchdog stopped the arm
        if (code == 6) {                                        // not homed yet: nothing moves
            st_.streaming = false;
            st_.homed = false;
        }
    }
}

void ArduinoLink::run() {
    char buf[512];
    while (!quit_) {
        // ---- requests from the UI
        bool doConnect = false, doDisconnect = false, doStop = false;
        std::string port;
        int baud = 115200;
        {
            std::lock_guard<std::mutex> lk(mtx_);
            doConnect = reqConnect_;
            doDisconnect = reqDisconnect_;
            doStop = stopRequested_;
            reqConnect_ = reqDisconnect_ = stopRequested_ = false;
            port = reqPort_;
            baud = reqBaud_;
        }
        if (doDisconnect) {
            if (port_.isOpen()) {
                port_.write("STOP\n");
                port_.close();
            }
            std::lock_guard<std::mutex> lk(mtx_);
            st_.state = LinkState::Disconnected;
            st_.message = "Bağlantı kesildi";
            st_.streaming = false;
            st_.havePositions = false;
            addLog("! Bağlantı kesildi");
        }
        if (doConnect) {
            port_.close();
            std::string err;
            if (!port_.open(port, baud, err)) {
                fail(err);
            } else {
                rx_.clear();
                connectStart_ = lastRx_ = lastPing_ = lastFrame_ = lastGet_ = Clock::now();
                pingPending_ = gotReady_ = infoSent_ = sentOnce_ = false;
                inflight_ = 0;
                std::lock_guard<std::mutex> lk(mtx_);
                st_ = LinkStatus{};
                st_.state = LinkState::Connecting;
                st_.port = port;
                st_.message = "Port açıldı, kartın açılması bekleniyor...";
                addLog("! " + port + " açıldı (" + std::to_string(baud) + " baud)");
            }
        }
        if (!port_.isOpen()) {
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
            continue;
        }

        // ---- read
        while (true) {
            const int n = port_.read(buf, sizeof(buf));
            if (n < 0) {
                fail("Bağlantı koptu: port kapandı (kablo çekildi ya da kart yeniden bağlandı).");
                break;
            }
            if (n == 0) break;
            rx_.append(buf, size_t(n));
        }
        if (!port_.isOpen()) continue;
        size_t nl;
        while ((nl = rx_.find('\n')) != std::string::npos) {
            std::string line = rx_.substr(0, nl);
            rx_.erase(0, nl + 1);
            if (!line.empty() && line.back() == '\r') line.pop_back();
            if (!line.empty()) handleLine(line);
        }
        if (rx_.size() > 4096) rx_.clear(); // garbage (wrong baud rate)

        LinkState state;
        {
            std::lock_guard<std::mutex> lk(mtx_);
            state = st_.state;
        }

        // ---- handshake: wait for READY (board reset), then INFO
        if (state == LinkState::Connecting) {
            if (!infoSent_ && (gotReady_ || msSince(connectStart_) > kReadyWaitMs)) {
                writeLine("INFO"); // boards without auto-reset never print READY
                infoSent_ = true;
            }
            if (msSince(connectStart_) > kConnectTimeoutMs) {
                fail("Arduino yanıt vermedi. Karta 'arduino/pegasus_arm/pegasus_arm.ino' yüklü mü, baud hızı 115200 mü, "
                     "doğru port mu seçildi?");
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
            continue;
        }
        if (state != LinkState::Connected) {
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
            continue;
        }

        // ---- emergency stop first
        if (doStop) {
            writeLine("STOP");
            inflight_ = 0;
        }

        // ---- queued commands
        std::deque<std::string> cmds;
        {
            std::lock_guard<std::mutex> lk(mtx_);
            cmds.swap(outbox_);
        }
        for (const auto& c : cmds)
            if (!writeLine(c)) break;

        // ---- streaming
        bool streaming, dirty;
        std::array<float, kServoChannels> tgt;
        int freeSlots;
        {
            std::lock_guard<std::mutex> lk(mtx_);
            streaming = st_.streaming && targetsSet_;
            dirty = targetsDirty_;
            tgt = targets_;
            freeSlots = st_.freeSlots;
        }
        const float sinceFrame = msSince(lastFrame_);
        if (streaming && dirty && sinceFrame >= kFrameEveryMs && inflight_ < 2 && freeSlots > 1) {
            bool changed = !sentOnce_;
            for (int i = 0; i < kServoChannels && !changed; ++i) changed = std::abs(tgt[size_t(i)] - lastSent_[size_t(i)]) >= 0.05f;
            if (changed) {
                // duration = time since the previous frame, so the arm moves at the speed the targets change
                // after an idle gap the target jumped "now": do not stretch the step over the gap
                const int dur = !sentOnce_ ? 0 : sinceFrame > 100.0f ? int(kFrameEveryMs) : int(std::clamp(sinceFrame, kFrameEveryMs, 50.0f));
                char line[128];
                int len = std::snprintf(line, sizeof(line), "M %d %d", seq_ = (seq_ + 1) % 30000, dur);
                for (int i = 0; i < kServoChannels; ++i)
                    len += std::snprintf(line + len, sizeof(line) - size_t(len), " %d",
                                         int(std::lround(std::clamp(tgt[size_t(i)], 0.0f, 180.0f) * 10.0f)));
                writeLine(line);
                inflight_++;
                lastSent_ = tgt;
                sentOnce_ = true;
                lastFrame_ = Clock::now();
                std::lock_guard<std::mutex> lk(mtx_);
                st_.framesSent++;
                st_.freeSlots = std::max(0, st_.freeSlots - 1);
            }
            std::lock_guard<std::mutex> lk(mtx_);
            if (targets_ == tgt) targetsDirty_ = false;
        }
        if (inflight_ > 0 && sinceFrame > 1000.0f) inflight_ = 0; // lost replies: do not stall forever

        // ---- heart beat + positions
        if (msSince(lastPing_) > kPingEveryMs) {
            if (!pingPending_) pingSent_ = Clock::now();
            pingPending_ = true;
            writeLine("PING");
            lastPing_ = Clock::now();
        }
        if (msSince(lastGet_) > kGetEveryMs) {
            writeLine("GET");
            lastGet_ = Clock::now();
        }
        if (msSince(lastRx_) > kLostAfterMs) fail("Bağlantı koptu: Arduino 2 saniyedir yanıt vermiyor.");

        std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }
    if (port_.isOpen()) {
        port_.write("STOP\n");
        port_.close();
    }
}

} // namespace pegasus
