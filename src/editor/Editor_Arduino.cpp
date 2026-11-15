// Arduino window: serial connection to the robot arm (port, baud, state), servo channel mapping /
// calibration, live following of the model and a protocol console.

#include "editor/Editor.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <sstream>

namespace fs = std::filesystem;

namespace pegasus {

namespace {
const char* kChannelNames[kServoChannels] = {"Bel", "Omuz", "Dirsek", "Bilek dönüş", "Bilek eğim", "Kıskaç"};
const int kChannelPins[kServoChannels] = {5, 6, 7, 8, 9, 10};
const int kBauds[] = {9600, 57600, 115200, 230400};

ImVec4 stateColor(LinkState s) {
    switch (s) {
    case LinkState::Connected: return ImVec4(0.35f, 0.85f, 0.4f, 1.0f);
    case LinkState::Connecting: return ImVec4(1.0f, 0.8f, 0.3f, 1.0f);
    case LinkState::Error: return ImVec4(1.0f, 0.4f, 0.35f, 1.0f);
    default: return ImVec4(0.6f, 0.6f, 0.6f, 1.0f);
    }
}
const char* stateName(LinkState s) {
    switch (s) {
    case LinkState::Connected: return "Bağlı";
    case LinkState::Connecting: return "Bağlanıyor";
    case LinkState::Error: return "Hata";
    default: return "Bağlı değil";
    }
}
} // namespace

std::string Editor::arduinoConfigPath() const {
    if (!arduinoCfgOverride_.empty()) return arduinoCfgOverride_;
    return robot_ ? (fs::path(robot_->folder) / "arduino.pegasus").string() : std::string("arduino.pegasus");
}

void Editor::resetServoChannels() {
    const std::vector<int> drivers = robot_ ? robot_->drivers() : std::vector<int>{};
    for (int i = 0; i < kServoChannels; ++i) {
        ServoChannel& c = servoChannels_[size_t(i)];
        c = ServoChannel{};
        c.joint = i < int(drivers.size()) ? drivers[size_t(i)] : -1;
        c.jog = 90.0f;
    }
}

bool Editor::loadArduinoConfig() {
    resetServoChannels();
    std::ifstream f(arduinoConfigPath());
    if (!f) return false;
    std::string line;
    while (std::getline(f, line)) {
        std::istringstream is(line);
        std::string key;
        is >> key;
        if (key == "port") {
            std::string p;
            std::getline(is >> std::ws, p);
            std::snprintf(portBuf_, sizeof(portBuf_), "%s", p.c_str());
        } else if (key == "baud") {
            is >> baud_;
        } else if (key == "speed") {
            is >> servoSpeed_;
        } else if (key == "channel") {
            int ch = 0;
            ServoChannel c;
            std::string jointName;
            is >> ch >> c.center >> c.dir >> c.minDeg >> c.maxDeg;
            std::getline(is >> std::ws, jointName);
            if (ch < 1 || ch > kServoChannels) continue;
            c.joint = -1;
            if (robot_)
                for (size_t j = 0; j < robot_->joints.size(); ++j)
                    if (robot_->joints[j].name == jointName) c.joint = int(j);
            c.jog = c.center;
            servoChannels_[size_t(ch - 1)] = c;
        }
    }
    return true;
}

bool Editor::saveArduinoConfig(std::string& err) const {
    const std::string path = arduinoConfigPath();
    std::ofstream f(path);
    if (!f) {
        err = "Yazılamadı: " + path;
        return false;
    }
    f << "# Pegasus Arduino ayarlari\n";
    f << "# channel <no> <merkez (servo derecesi, eklem 0 iken)> <yon +1/-1> <min> <max> <eklem adi>\n";
    f << "port " << portBuf_ << "\n";
    f << "baud " << baud_ << "\n";
    f << "speed " << servoSpeed_ << "\n";
    for (int i = 0; i < kServoChannels; ++i) {
        const ServoChannel& c = servoChannels_[size_t(i)];
        const std::string jn = robot_ && c.joint >= 0 && c.joint < int(robot_->joints.size()) ? robot_->joints[size_t(c.joint)].name : "-";
        f << "channel " << (i + 1) << " " << c.center << " " << c.dir << " " << c.minDeg << " " << c.maxDeg << " " << jn << "\n";
    }
    return bool(f);
}

float Editor::servoTarget(int ch) const {
    const ServoChannel& c = servoChannels_[size_t(ch)];
    float deg = c.center;
    if (robot_ && c.joint >= 0 && c.joint < int(robot_->joints.size())) deg = c.center + c.dir * robot_->joints[size_t(c.joint)].angleDeg;
    return std::clamp(deg, c.minDeg, c.maxDeg);
}

void Editor::sendServoLimits() {
    if (!link_) return;
    char buf[64];
    std::snprintf(buf, sizeof(buf), "V %d", int(std::lround(std::clamp(servoSpeed_, 1.0f, 600.0f))));
    link_->send(buf);
    for (int i = 0; i < kServoChannels; ++i) {
        const ServoChannel& c = servoChannels_[size_t(i)];
        std::snprintf(buf, sizeof(buf), "L %d %d %d", i + 1, int(std::lround(c.minDeg * 10.0f)), int(std::lround(c.maxDeg * 10.0f)));
        link_->send(buf);
    }
}

void Editor::emergencyStop() {
    const bool wasPlaying = playing_;
    const bool connected = link_ && link_->connected();
    followRobot_ = false;
    if (connected) link_->stop(); // board: queue cleared, arm holds where it is
    if (playing_) stopRun();
    if (connected) notify("ACİL DURDUR: kol olduğu yerde durdu, canlı takip kapatıldı.", true);
    else if (wasPlaying) notify("ACİL DURDUR: hareket animasyonu durduruldu.", true);
    else notify("ACİL DURDUR (Space): durdurulacak bir hareket yok.", true);
}

void Editor::updateArduino() {
    if (!link_) return;
    const LinkStatus st = link_->status();
    if (st.state == LinkState::Connected && lastLinkState_ != LinkState::Connected) {
        sendServoLimits();
        notify("Arduino bağlandı: " + st.port + " (" + st.firmware + ")");
    }
    if (st.state == LinkState::Error && lastLinkState_ != LinkState::Error) {
        followRobot_ = false;
        notify("Arduino: " + st.message, true);
    }
    lastLinkState_ = st.state;
    if (st.state != LinkState::Connected) return;
    if (followRobot_ && !st.homed) {
        followRobot_ = false; // board reset / servos released: the arm must be homed again
        notify("Servolar serbest: canlı takip kapatıldı. Devam etmek için 'Home konumuna al'.", true);
    }
    // manual sliders show where the servos really are (no jump on the first touch)
    if (st.havePositions && !jogActive_)
        for (int i = 0; i < kServoChannels; ++i) servoChannels_[size_t(i)].jog = st.servoDeg[size_t(i)];
    std::array<float, kServoChannels> t{};
    for (int i = 0; i < kServoChannels; ++i) t[size_t(i)] = servoTarget(i);
    link_->setTargets(t);
    link_->setStreaming(followRobot_);
}

void Editor::drawArduinoWindow() {
    if (!showArduino_) return;
    if (!link_) link_ = std::make_unique<ArduinoLink>();
    if (!arduinoCfgLoaded_) {
        loadArduinoConfig();
        arduinoCfgLoaded_ = true;
        ports_ = SerialPort::list();
        if (portBuf_[0] == 0 && !ports_.empty()) std::snprintf(portBuf_, sizeof(portBuf_), "%s", ports_.front().path.c_str());
    }
    const ImVec2 vs = ImGui::GetMainViewport()->Size;
    ImGui::SetNextWindowSize(ImVec2(std::min(vs.x * 0.9f, ImGui::GetFontSize() * 50.0f), std::min(vs.y * 0.9f, ImGui::GetFontSize() * 44.0f)),
                             ImGuiCond_FirstUseEver);
    ImGui::SetNextWindowPos(ImGui::GetMainViewport()->GetCenter(), ImGuiCond_FirstUseEver, ImVec2(0.5f, 0.5f));
    if (!ImGui::Begin("Arduino bağlantısı", &showArduino_, ImGuiWindowFlags_NoDocking)) {
        ImGui::End();
        return;
    }
    const LinkStatus st = link_->status();
    const bool busy = st.state == LinkState::Connecting || st.state == LinkState::Connected;

    // ---- connection
    ImGui::SeparatorText("Bağlantı");
    {
        const ImVec2 p = ImGui::GetCursorScreenPos();
        const float r = ImGui::GetFontSize() * 0.32f;
        ImGui::GetWindowDrawList()->AddCircleFilled(ImVec2(p.x + r, p.y + ImGui::GetTextLineHeight() * 0.5f), r,
                                                    ImGui::ColorConvertFloat4ToU32(stateColor(st.state)));
        ImGui::Dummy(ImVec2(r * 2.0f, ImGui::GetTextLineHeight()));
    }
    ImGui::SameLine();
    ImGui::TextColored(stateColor(st.state), "%s", stateName(st.state));
    if (st.state == LinkState::Connected) {
        ImGui::SameLine();
        ImGui::TextDisabled("%s  -  %s", st.port.c_str(), st.firmware.c_str());
    } else if (st.state == LinkState::Connecting) {
        ImGui::SameLine();
        ImGui::TextDisabled("%s", st.message.c_str());
    }
    if (st.state == LinkState::Error) ImGui::TextColored(ImVec4(1.0f, 0.55f, 0.45f, 1.0f), "%s", st.message.c_str());

    ImGui::BeginDisabled(busy);
    ImGui::SetNextItemWidth(ImGui::GetFontSize() * 22.0f);
    if (ImGui::BeginCombo("Port", portBuf_[0] ? portBuf_ : "(port yok)")) {
        for (const auto& p : ports_) {
            const std::string label = p.path + "   " + p.description;
            if (ImGui::Selectable(label.c_str(), p.path == portBuf_)) std::snprintf(portBuf_, sizeof(portBuf_), "%s", p.path.c_str());
        }
        if (ports_.empty()) ImGui::TextDisabled("Seri cihaz bulunamadı. Arduino USB kablosu takılı mı?");
        ImGui::EndCombo();
    }
    ImGui::SameLine();
    if (ImGui::Button("Yenile")) ports_ = SerialPort::list();
    ImGui::SetNextItemWidth(ImGui::GetFontSize() * 22.0f);
    ImGui::InputText("Port yolu", portBuf_, sizeof(portBuf_));
#ifdef _WIN32
    ImGui::SetItemTooltip("Listede olmayan bir port elle yazılabilir (ör. COM4).");
#else
    ImGui::SetItemTooltip("Listede olmayan bir port elle yazılabilir (ör. /dev/ttyUSB0).");
#endif
    ImGui::SetNextItemWidth(ImGui::GetFontSize() * 8.0f);
    if (ImGui::BeginCombo("Baud", std::to_string(baud_).c_str())) {
        for (int b : kBauds)
            if (ImGui::Selectable(std::to_string(b).c_str(), b == baud_)) baud_ = b;
        ImGui::EndCombo();
    }
    ImGui::SetItemTooltip("Firmware 115200 baud kullanır.");
    ImGui::EndDisabled();
    ImGui::SameLine();
    if (!busy) {
        ImGui::BeginDisabled(portBuf_[0] == 0);
        if (ImGui::Button("Bağlan", ImVec2(ImGui::GetFontSize() * 7.0f, 0))) link_->connect(portBuf_, baud_);
        ImGui::EndDisabled();
    } else if (ImGui::Button("Bağlantıyı kes", ImVec2(ImGui::GetFontSize() * 7.0f, 0))) {
        followRobot_ = false;
        link_->disconnect();
    }
    if (ports_.empty() && !busy) ImGui::TextDisabled("Port listesi boş: Arduino'yu takıp 'Yenile'ye basın.");

    if (st.state == LinkState::Connected) {
        const float up = std::chrono::duration<float>(std::chrono::steady_clock::now() - st.connectedSince).count();
        ImGui::TextDisabled("Gecikme %.1f ms  |  son yanıt %.0f ms önce  |  %.0f s bağlı", st.rttMs, st.lastRxAgeMs, up);
        ImGui::TextDisabled("Gönderilen hareket %ld, onay %ld, hata %ld, kuyrukta boş %d%s", st.framesSent, st.acks, st.errors, st.freeSlots,
                            st.resets ? "  |  kart yeniden başladı!" : "");
    }

    // ---- control
    ImGui::SeparatorText("Kontrol");
    const bool connected = st.state == LinkState::Connected;
    if (connected && st.firmware == "pegasus-arm 1.0")
        ImGui::TextColored(ImVec4(1.0f, 0.6f, 0.3f, 1.0f),
                           "Karttaki firmware eski (1.0): açılışta servoları sürüyor. arduino/pegasus_arm/pegasus_arm.ino'yu yeniden yükleyin.");
    if (connected && !st.homed) {
        ImGui::TextColored(ImVec4(1.0f, 0.8f, 0.35f, 1.0f), "Servolar serbest: kol hiçbir komutla hareket etmez.");
        ImGui::TextWrapped("Kolu elle başlangıç pozuna yakın getirin, sonra 'Home konumuna al'a basın. Konum bilinmediği için "
                           "servolar başlangıç pozuna kendi hızlarıyla gider.");
        ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.2f, 0.45f, 0.75f, 1.0f));
        if (ImGui::Button("Home konumuna al")) link_->send("HOME");
        ImGui::PopStyleColor();
        ImGui::SetItemTooltip("Servoları ilk kez sürer ve kolu başlangıç pozuna (90/150/35/140/85/80) alır.");
    }
    ImGui::BeginDisabled(!connected || !st.homed);
    if (ImGui::Checkbox("Robotu takip et (canlı)", &followRobot_) && followRobot_)
        notify("Gerçek kol modelin pozuna gidiyor (hız sınırı " + std::to_string(int(servoSpeed_)) + " °/s).");
    ImGui::SetItemTooltip("Açıkken gerçek kol, ekrandaki modelin eklem açılarını izler: eklem kaydırıcıları,\n"
                          "IK ve 'Çalıştır' animasyonu doğrudan servolara gider. Önce kalibrasyonu kontrol edin!");
    ImGui::SameLine();
    ImGui::SetNextItemWidth(ImGui::GetFontSize() * 10.0f);
    if (ImGui::SliderFloat("Hız sınırı", &servoSpeed_, 5.0f, 300.0f, "%.0f °/s")) {}
    if (ImGui::IsItemDeactivatedAfterEdit()) link_->send("V " + std::to_string(int(std::lround(servoSpeed_))));
    if (ImGui::Button("Başlangıç pozuna git")) {
        followRobot_ = false;
        link_->send("HOME");
    }
    ImGui::SetItemTooltip("Firmware'deki başlangıç pozuna hız sınırıyla gider (örnek koddaki 90/150/35/140/85/80).");
    ImGui::SameLine();
    if (ImGui::Button("Servoları serbest bırak")) {
        followRobot_ = false;
        link_->send("DETACH");
    }
    ImGui::SetItemTooltip("Servolara darbe gönderimi durur; kol elle hareket ettirilebilir (yerçekimiyle düşebilir!).\n"
                          "Sonrasında yeniden 'Home konumuna al' gerekir.");
    ImGui::EndDisabled();
    ImGui::SameLine();
    ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.75f, 0.12f, 0.1f, 1.0f));
    ImGui::PushStyleColor(ImGuiCol_ButtonHovered, ImVec4(0.9f, 0.2f, 0.15f, 1.0f));
    if (ImGui::Button("ACİL DURDUR (Space)")) emergencyStop();
    ImGui::PopStyleColor(2);
    ImGui::SetItemTooltip("Space tuşu uygulamanın her yerinde aynı işi yapar (metin kutusuna yazarken hariç).");
    if (followRobot_) ImGui::TextColored(ImVec4(0.45f, 0.85f, 1.0f, 1.0f), "Canlı takip açık: modelde yaptığınız her hareket gerçek kola gider.");

    // ---- channels
    ImGui::SeparatorText("Servo kanalları ve kalibrasyon");
    ImGui::TextDisabled("servo açısı = merkez + yön × eklem açısı  (min..max ile sınırlanır)");
    const ImGuiTableFlags tf = ImGuiTableFlags_Borders | ImGuiTableFlags_RowBg | ImGuiTableFlags_SizingFixedFit | ImGuiTableFlags_ScrollX;
    if (ImGui::BeginTable("##servos", 9, tf, ImVec2(0.0f, ImGui::GetFrameHeightWithSpacing() * 7.6f))) {
        ImGui::TableSetupScrollFreeze(1, 1);
        ImGui::TableSetupColumn("Kanal");
        ImGui::TableSetupColumn("Eklem");
        ImGui::TableSetupColumn("Model");
        ImGui::TableSetupColumn("Hedef");
        ImGui::TableSetupColumn("Kart");
        ImGui::TableSetupColumn("Merkez");
        ImGui::TableSetupColumn("Yön");
        ImGui::TableSetupColumn("Min / Max");
        ImGui::TableSetupColumn("Elle");
        ImGui::TableHeadersRow();
        bool limitsChanged = false;
        jogActive_ = false;
        for (int i = 0; i < kServoChannels; ++i) {
            ServoChannel& c = servoChannels_[size_t(i)];
            ImGui::PushID(i);
            ImGui::TableNextRow();
            ImGui::TableNextColumn();
            ImGui::Text("%d %s (D%d)", i + 1, kChannelNames[i], kChannelPins[i]);
            ImGui::TableNextColumn();
            ImGui::SetNextItemWidth(ImGui::GetFontSize() * 6.0f);
            const char* jn = robot_ && c.joint >= 0 && c.joint < int(robot_->joints.size()) ? robot_->joints[size_t(c.joint)].name.c_str() : "-";
            if (ImGui::BeginCombo("##joint", jn)) {
                if (ImGui::Selectable("-", c.joint < 0)) c.joint = -1;
                if (robot_)
                    for (int j : robot_->drivers())
                        if (ImGui::Selectable(robot_->joints[size_t(j)].name.c_str(), c.joint == j)) c.joint = j;
                ImGui::EndCombo();
            }
            const bool hasJoint = robot_ && c.joint >= 0 && c.joint < int(robot_->joints.size());
            ImGui::TableNextColumn();
            if (hasJoint) ImGui::Text("%+.1f°", robot_->joints[size_t(c.joint)].angleDeg);
            else ImGui::TextDisabled("-");
            ImGui::TableNextColumn();
            const float tgt = servoTarget(i);
            const float raw = hasJoint ? c.center + c.dir * robot_->joints[size_t(c.joint)].angleDeg : c.center;
            if (std::abs(raw - tgt) > 0.05f) ImGui::TextColored(ImVec4(1, 0.6f, 0.3f, 1), "%.1f°", tgt);
            else ImGui::Text("%.1f°", tgt);
            if (std::abs(raw - tgt) > 0.05f) ImGui::SetItemTooltip("Servo sınırına takıldı (istenen %.1f°)", raw);
            ImGui::TableNextColumn();
            if (st.havePositions && st.homed) ImGui::Text("%.1f°", st.servoDeg[size_t(i)]);
            else if (st.havePositions) ImGui::TextDisabled("serbest"); // not driven: the real position is unknown
            else ImGui::TextDisabled("-");
            ImGui::TableNextColumn();
            ImGui::SetNextItemWidth(ImGui::GetFontSize() * 4.0f);
            ImGui::DragFloat("##center", &c.center, 0.2f, 0.0f, 180.0f, "%.1f");
            ImGui::SetItemTooltip("Eklem açısı 0 iken (modelin başlangıç pozu) servonun açısı.");
            ImGui::TableNextColumn();
            if (ImGui::SmallButton(c.dir > 0 ? "+1" : "-1")) c.dir = -c.dir;
            ImGui::SetItemTooltip("Servo modelle ters yönde dönüyorsa çevirin.");
            ImGui::TableNextColumn();
            ImGui::SetNextItemWidth(ImGui::GetFontSize() * 5.0f);
            float mm[2] = {c.minDeg, c.maxDeg};
            if (ImGui::DragFloat2("##lim", mm, 0.5f, 0.0f, 180.0f, "%.0f")) {
                c.minDeg = std::clamp(std::min(mm[0], mm[1]), 0.0f, 180.0f);
                c.maxDeg = std::clamp(std::max(mm[0], mm[1]), 0.0f, 180.0f);
            }
            if (ImGui::IsItemDeactivatedAfterEdit()) limitsChanged = true;
            ImGui::TableNextColumn();
            ImGui::BeginDisabled(st.state != LinkState::Connected || followRobot_ || !st.homed);
            ImGui::SetNextItemWidth(ImGui::GetFontSize() * 5.0f);
            if (ImGui::SliderFloat("##jog", &c.jog, c.minDeg, c.maxDeg, "%.0f°") && link_)
                link_->send("S " + std::to_string(i + 1) + " " + std::to_string(int(std::lround(c.jog * 10.0f))));
            jogActive_ |= ImGui::IsItemActive();
            ImGui::SetItemTooltip("Servoyu doğrudan sürer (takip kapalıyken). Kalibrasyon için: modeldeki eklemi 0°'ye,\n"
                                  "gerçek servoyu bu kaydırıcıyla aynı duruşa getirip 'Merkez yap'a basın.");
            ImGui::SameLine();
            if (ImGui::SmallButton("Merkez yap")) {
                const float a = hasJoint ? robot_->joints[size_t(c.joint)].angleDeg : 0.0f;
                c.center = std::clamp(c.jog - c.dir * a, 0.0f, 180.0f);
            }
            ImGui::EndDisabled();
            ImGui::PopID();
        }
        ImGui::EndTable();
        if (limitsChanged && st.state == LinkState::Connected) sendServoLimits();
    }
    if (ImGui::Button("Ayarları kaydet")) {
        std::string err;
        if (saveArduinoConfig(err)) notify("Arduino ayarları kaydedildi: " + arduinoConfigPath());
        else notify(err, true);
    }
    ImGui::SameLine();
    if (ImGui::Button("Yeniden yükle")) {
        if (loadArduinoConfig()) notify("Arduino ayarları yüklendi: " + arduinoConfigPath());
        else notify("Kayıtlı ayar yok, varsayılanlar kullanılıyor.");
    }
    ImGui::SameLine();
    if (ImGui::Button("Varsayılanlar")) resetServoChannels();
    ImGui::SameLine();
    ImGui::TextDisabled("%s", arduinoConfigPath().c_str());

    // ---- console
    ImGui::SeparatorText("Haberleşme günlüğü");
    bool verbose = link_->logStreamFrames;
    if (ImGui::Checkbox("Akış satırlarını da göster (M / A / PING / GET)", &verbose)) link_->logStreamFrames = verbose;
    ImGui::SameLine();
    if (ImGui::SmallButton("Temizle")) link_->clearLog();
    const float logH = std::max(ImGui::GetContentRegionAvail().y - ImGui::GetFrameHeightWithSpacing() * 1.2f, ImGui::GetFontSize() * 5.0f);
    if (ImGui::BeginChild("##arduinolog", ImVec2(0, logH), ImGuiChildFlags_Borders)) {
        for (const auto& l : link_->log()) {
            if (l[0] == '!') ImGui::TextColored(ImVec4(1.0f, 0.8f, 0.4f, 1.0f), "%s", l.c_str());
            else if (l[0] == '<') ImGui::TextColored(l.rfind("< ERR", 0) == 0 ? ImVec4(1, 0.45f, 0.4f, 1) : ImVec4(0.6f, 0.9f, 0.6f, 1), "%s", l.c_str());
            else ImGui::TextDisabled("%s", l.c_str());
        }
        if (ImGui::GetScrollY() >= ImGui::GetScrollMaxY() - 4.0f) ImGui::SetScrollHereY(1.0f);
    }
    ImGui::EndChild();
    ImGui::BeginDisabled(st.state != LinkState::Connected);
    ImGui::SetNextItemWidth(-ImGui::GetFontSize() * 6.0f);
    const bool enter = ImGui::InputTextWithHint("##cmd", "komut (ör. GET, HOME, S 1 900)", cmdBuf_, sizeof(cmdBuf_), ImGuiInputTextFlags_EnterReturnsTrue);
    ImGui::SameLine();
    if ((ImGui::Button("Gönder") || enter) && cmdBuf_[0]) {
        link_->send(cmdBuf_);
        cmdBuf_[0] = 0;
    }
    ImGui::EndDisabled();
    ImGui::End();
}

} // namespace pegasus
