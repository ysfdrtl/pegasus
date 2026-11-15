// Robot arm integration of the editor: loading, forward kinematics UI, joint editing, overlays.
#include "editor/Editor.h"

#include "robot/JointDetect.h"

#include <imgui_internal.h>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <filesystem>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>

namespace pegasus {

namespace fs = std::filesystem;

namespace {

const glm::vec4 kDriverAxisColor{1.0f, 0.82f, 0.15f, 1.0f};
const glm::vec4 kMimicAxisColor{0.45f, 0.75f, 1.0f, 0.7f};
const glm::vec4 kSelectedAxisColor{1.0f, 0.45f, 0.05f, 1.0f};

std::string lowerStr(std::string s) {
    for (auto& c : s) c = char(std::tolower(static_cast<unsigned char>(c)));
    return s;
}

// ZYX (yaw-pitch-roll) angles in degrees of a rotation matrix.
glm::vec3 rpyDegrees(const glm::mat3& R) {
    const float pitch = std::asin(std::clamp(-R[0][2], -1.0f, 1.0f));
    const float roll = std::atan2(R[1][2], R[2][2]);
    const float yaw = std::atan2(R[0][1], R[0][0]);
    return glm::degrees(glm::vec3(roll, pitch, yaw)) + glm::vec3(0.0f); // + 0 avoids "-0.0"
}

} // namespace

// =============================================================================================
// Loading / syncing
// =============================================================================================

bool Editor::loadRobot(const std::string& folder) {
    std::vector<std::string> log;
    std::string err;
    std::unique_ptr<Robot> r;
    const fs::path cfg = fs::path(folder) / "robot.pegasus";
    std::error_code ec;
    if (fs::exists(cfg, ec)) {
        r = Robot::load(cfg.string(), folder, err);
        if (r) {
            r->computeIgnoredPairs();
            log.push_back("Kayıtlı robot tanımı yüklendi (robot.pegasus). 'Yeniden algıla' ile STL'lerden tekrar çıkarılabilir.");
        } else {
            log.push_back("robot.pegasus okunamadı (" + err + "), STL'lerden otomatik algılama yapıldı.");
        }
    }
    if (!r) r = detectRobot(folder, err, log);
    if (!r) {
        notify(err, true);
        return false;
    }
    removeRobot();
    robot_ = std::move(r);
    arduinoCfgLoaded_ = false; // servo mapping follows the robot (arduino.pegasus in its folder)
    if (showArduino_ || link_) loadArduinoConfig(), arduinoCfgLoaded_ = true;
    robotLog_ = std::move(log);

    // governing driver of each part (for colouring)
    const std::vector<int> drivers = robot_->drivers();
    auto governing = [&](int p) {
        for (int q = p; q >= 0; q = robot_->parts[q].parent) {
            const int j = robot_->parts[q].joint;
            if (j >= 0 && robot_->joints[j].type == JointType::Revolute) return robot_->joints[j].mimic >= 0 ? robot_->joints[j].mimic : j;
        }
        return -1;
    };
    const glm::vec4 orange(0.93f, 0.52f, 0.16f, 1.0f), light(0.86f, 0.87f, 0.89f, 1.0f), dark(0.33f, 0.35f, 0.39f, 1.0f),
        servo(0.20f, 0.38f, 0.85f, 1.0f);
    for (int i = 0; i < int(robot_->parts.size()); ++i) {
        RobotPart& part = robot_->parts[i];
        part.mesh->setFeatureAngle(featureAngle_);
        SceneObject* obj = scene_.add(part.mesh, part.name);
        obj->robotPart = i;
        obj->matrixOverride = true;
        obj->sourcePath = (fs::path(folder) / part.file).string();
        const int g = governing(i);
        const auto it = std::find(drivers.begin(), drivers.end(), g);
        const int k = it == drivers.end() ? -1 : int(it - drivers.begin());
        obj->color = k < 0 ? dark : (k % 2 == 0 ? orange : light);
        if (lowerStr(part.name).find("servo") != std::string::npos) obj->color = servo;
        part.objectId = obj->id;
    }
    syncRobot();
    selectedJoint_ = drivers.empty() ? -1 : drivers[0];
    activeObject_ = 0;
    camera_.setView(-50.0f, 22.0f);
    frameAll();
    focusRobotPanel_ = true;
    robotDirty_ = false;
    char buf[256];
    std::snprintf(buf, sizeof(buf), "Robot yüklendi: %zu parça, %zu bağımsız eklem", robot_->parts.size(), drivers.size());
    notify(buf);
    for (const auto& l : robotLog_) std::printf("[robot] %s\n", l.c_str());
    return true;
}

void Editor::removeRobot() {
    if (!robot_) return;
    std::vector<uint32_t> ids;
    for (const auto& o : scene_.objects())
        if (o->robotPart >= 0) ids.push_back(o->id);
    for (uint32_t id : ids) {
        selection_.erase(std::remove_if(selection_.begin(), selection_.end(), [&](const ElementRef& r) { return r.objectId == id; }),
                         selection_.end());
        if (activeObject_ == id) activeObject_ = 0;
        if (dragObject_ == id) dragCandidate_ = dragging_ = false;
        scene_.remove(id);
    }
    hover_ = {};
    hoverObject_ = 0;
    robot_.reset();
    selectedJoint_ = -1;
}

void Editor::syncRobot() {
    if (!robot_) return;
    robot_->solveFK();
    for (int i = 0; i < int(robot_->parts.size()); ++i) {
        if (SceneObject* o = scene_.find(robot_->parts[i].objectId)) {
            o->worldOverride = robot_->partWorld(i);
            o->matrixOverride = true;
        }
    }
    scene_.markDirty();
}

bool Editor::robotColliding() const {
    if (!robot_ || !collision_.enabled) return false;
    for (const auto& part : robot_->parts) {
        const SceneObject* o = scene_.find(part.objectId);
        if (o && collision_.collidesAt(scene_, *o, o->matrix())) return true;
    }
    return false;
}

void Editor::setRobotAngle(int j, float deg) {
    if (!robot_ || j < 0 || j >= int(robot_->joints.size())) return;
    RobotJoint& J = robot_->joints[j];
    const float old = J.angleDeg;
    const float target = std::clamp(deg, J.minDeg, J.maxDeg);
    if (target == old) return;
    const bool check = blockOnCollision_ && collision_.enabled && !robotColliding();
    J.angleDeg = target + 0.0f;
    syncRobot();
    if (check && robotColliding()) {
        float lo = 0.0f, hi = 1.0f;
        for (int i = 0; i < 10; ++i) {
            const float mid = 0.5f * (lo + hi);
            J.angleDeg = old + (target - old) * mid;
            syncRobot();
            if (robotColliding()) hi = mid;
            else lo = mid;
        }
        J.angleDeg = old + (target - old) * lo;
        syncRobot();
        blockedFlash_ = 1.2f;
    }
}

void Editor::refreshRobotFolders() {
    robotFolders_.clear();
    robotFoldersScanned_ = true;
    std::error_code ec;
    for (const auto& d : fs::directory_iterator(fs::current_path(ec), ec)) {
        if (!d.is_directory()) continue;
        const std::string name = d.path().filename().string();
        if (name.empty() || name[0] == '.' || name == "build") continue;
        int stl = 0;
        std::error_code ec2;
        for (const auto& f : fs::directory_iterator(d.path(), ec2)) {
            if (lowerStr(f.path().extension().string()) == ".stl" && ++stl >= 2) break;
        }
        if (stl >= 2) robotFolders_.push_back(d.path().string());
    }
    std::sort(robotFolders_.begin(), robotFolders_.end());
}

// =============================================================================================
// Axis from the current element selection
// =============================================================================================

bool Editor::axisFromSelection(glm::vec3& originW, glm::vec3& axisW, std::string& how) const {
    if (selection_.empty()) {
        how = "Önce Nokta / Kenar / Yüzey aracıyla geometri seçin.";
        return false;
    }
    if (selKind_ == Tool::Vertex || selKind_ == Tool::Edge) {
        std::vector<glm::vec3> pts;
        auto addUnique = [&](const glm::vec3& p) {
            for (const auto& q : pts)
                if (glm::length(q - p) < 1e-4f) return;
            pts.push_back(p);
        };
        if (selKind_ == Tool::Vertex) {
            for (const auto& r : selection_) addUnique(vertexWorld(r));
        } else {
            if (selection_.size() == 1) {
                glm::vec3 a, b;
                edgeWorld(selection_[0], a, b);
                originW = 0.5f * (a + b);
                axisW = glm::normalize(b - a);
                how = "Eksen: seçili kenar doğrultusu";
                return glm::length(b - a) > 0.0f;
            }
            for (const auto& r : selection_) {
                glm::vec3 a, b;
                edgeWorld(r, a, b);
                addUnique(a);
                addUnique(b);
            }
        }
        if (pts.size() == 2) {
            originW = pts[0];
            axisW = glm::normalize(pts[1] - pts[0]);
            how = "Eksen: iki noktadan geçen doğru";
            return true;
        }
        float radius = 0.0f;
        if (pts.size() >= 3 && fitCircle(pts, originW, axisW, radius)) {
            char buf[96];
            std::snprintf(buf, sizeof(buf), "Eksen: %zu noktadan daire (r = %.2f mm) merkezi ve normali", pts.size(), radius);
            how = buf;
            return true;
        }
        how = "Seçili noktalardan eksen çıkarılamadı (en az 2 nokta ya da daire üzerinde 3+ nokta gerekli).";
        return false;
    }
    // faces
    std::vector<glm::vec3> pts, normals;
    std::vector<float> weights;
    for (const auto& r : selection_) {
        const SceneObject* o = scene_.find(r.objectId);
        if (!o) continue;
        std::vector<uint32_t> tris;
        appendFaceTriangles(*o->mesh, selFaceMode_, r.index, tris);
        const glm::mat4 m = o->matrix();
        for (uint32_t t : tris) {
            const glm::vec3 a = glm::vec3(m * glm::vec4(o->mesh->triVertex(t, 0), 1.0f));
            const glm::vec3 b = glm::vec3(m * glm::vec4(o->mesh->triVertex(t, 1), 1.0f));
            const glm::vec3 c = glm::vec3(m * glm::vec4(o->mesh->triVertex(t, 2), 1.0f));
            const glm::vec3 cr = glm::cross(b - a, c - a);
            const float len = glm::length(cr);
            if (len <= 0.0f) continue;
            for (const glm::vec3& p : {a, b, c}) {
                pts.push_back(p);
                normals.push_back(cr / len);
                weights.push_back(len / 6.0f);
            }
        }
    }
    bool cyl = false;
    if (!fitAxisFromSurface(pts, normals, weights, originW, axisW, cyl)) {
        how = "Seçili yüzeyden eksen çıkarılamadı.";
        return false;
    }
    how = cyl ? "Eksen: silindirik yüzeyin ekseni" : "Eksen: düz yüzeyin normali (merkezden)";
    return true;
}

// =============================================================================================
// Panels
// =============================================================================================

void Editor::drawRobotPanel() {
    if (focusRobotPanel_) {
        ImGui::SetNextWindowFocus();
        focusRobotPanel_ = false;
    }
    if (!ImGui::Begin("Robot")) {
        ImGui::End();
        return;
    }
    if (!robot_) {
        ImGui::TextWrapped("Robot kol yüklemek için her parçası ayrı STL dosyası olan (montaj koordinatlarında dışa aktarılmış) bir klasör seçin. Eklemler otomatik algılanır.");
        if (ImGui::Button("Klasör seç...")) folderBrowser_.open();
        if (!robotFoldersScanned_) refreshRobotFolders();
        if (!robotFolders_.empty()) {
            ImGui::SeparatorText("Çalışma klasöründe bulunanlar");
            for (const auto& f : robotFolders_) {
                const std::string label = fs::path(f).filename().string();
                if (ImGui::Button(label.c_str())) pendingRobot_ = f;
            }
        }
        if (ImGui::SmallButton("Listeyi yenile")) refreshRobotFolders();
        ImGui::End();
        return;
    }

    Robot& R = *robot_;
    ImGui::PushItemWidth(-ImGui::GetFontSize() * 6.0f);
    ImGui::Text("%s  (%zu parça)", R.name.c_str(), R.parts.size());
    if (robotDirty_) {
        ImGui::SameLine();
        ImGui::TextColored(ImVec4(1.0f, 0.75f, 0.3f, 1.0f), "* kaydedilmedi");
    }

    // ---------------- forward kinematics
    ImGui::SeparatorText("İleri kinematik");
    const std::vector<int> drivers = R.drivers();
    for (int d : drivers) {
        RobotJoint& J = R.joints[d];
        ImGui::PushID(d);
        const bool sel = selectedJoint_ == d;
        if (sel) ImGui::PushStyleColor(ImGuiCol_FrameBg, ImVec4(0.45f, 0.28f, 0.08f, 1.0f));
        float a = J.angleDeg + 0.0f;
        if (ImGui::SliderFloat("##a", &a, J.minDeg, J.maxDeg, "%.1f°")) setRobotAngle(d, a);
        if (ImGui::IsItemActivated()) selectedJoint_ = d;
        if (sel) ImGui::PopStyleColor();
        ImGui::SameLine();
        if (ImGui::SmallButton("0")) setRobotAngle(d, 0.0f);
        ImGui::SameLine();
        if (ImGui::Selectable(J.name.c_str(), sel, 0, ImVec2(0, 0))) selectedJoint_ = d;
        ImGui::PopID();
    }
    if (drivers.empty()) ImGui::TextDisabled("Bağımsız eklem yok.");
    if (ImGui::Button("Ev konumu (tüm açılar 0)")) {
        for (int d : drivers) R.joints[d].angleDeg = std::clamp(0.0f, R.joints[d].minDeg, R.joints[d].maxDeg);
        syncRobot();
    }

    if (showTcp_ || true) {
        const glm::mat4 tcp = R.tcpWorld();
        const glm::vec3 pw = glm::vec3(tcp[3]);
        const glm::mat4 inv = glm::inverse(R.mount);
        const glm::vec3 pb = glm::vec3(inv * glm::vec4(pw, 1.0f));
        const glm::vec3 rpy = rpyDegrees(glm::mat3(inv) * glm::mat3(tcp));
        ImGui::SeparatorText("Uç nokta (TCP)");
        ImGui::Text("Tabana göre:  X %.1f  Y %.1f  Z %.1f mm", pb.x, pb.y, pb.z);
        ImGui::Text("Dünya:        X %.1f  Y %.1f  Z %.1f mm", pw.x, pw.y, pw.z);
        ImGui::Text("Yönelim (R/P/Y): %.1f°  %.1f°  %.1f°", rpy.x, rpy.y, rpy.z);
    }

    // ---------------- display
    ImGui::SeparatorText("Görünüm");
    ImGui::Checkbox("Eklem eksenleri", &showJointAxes_);
    ImGui::SameLine();
    ImGui::Checkbox("Bağımlılar", &showAllJoints_);
    ImGui::SameLine();
    ImGui::Checkbox("TCP", &showTcp_);

    // ---------------- joint editor
    if (ImGui::CollapsingHeader("Eklem düzenleme", ImGuiTreeNodeFlags_DefaultOpen)) drawJointEditor();

    // ---------------- placement
    if (ImGui::CollapsingHeader("Robot yerleşimi")) {
        Transform t = Transform::fromMatrix(R.mount);
        bool changed = ImGui::DragFloat3("Konum (mm)", glm::value_ptr(t.position), 0.5f, 0, 0, "%.1f");
        changed |= ImGui::DragFloat3("Dönüş (°)", glm::value_ptr(t.rotationDeg), 0.5f, 0, 0, "%.1f");
        if (changed) {
            t.scale = glm::vec3(1.0f);
            R.mount = t.matrix();
            syncRobot();
            robotDirty_ = true;
        }
        int up = R.upAxis;
        if (ImGui::Combo("Model yukarı ekseni", &up, "X\0Y\0Z\0")) {
            R.upAxis = up;
            syncRobot();
            robotDirty_ = true;
        }
        ImGui::SetItemTooltip("STL dosyalarında yukarıyı gösteren eksen (SolidWorks: Y).");
    }

    // ---------------- actions
    ImGui::Separator();
    if (ImGui::Button("Kaydet")) {
        std::string err;
        const std::string path = (fs::path(R.folder) / "robot.pegasus").string();
        if (R.save(path, err)) {
            robotDirty_ = false;
            notify("Robot kaydedildi: " + path);
        } else notify(err, true);
    }
    ImGui::SetItemTooltip("Eklem tanımlarını STL klasörüne robot.pegasus olarak kaydeder;\nbir sonraki yüklemede bu dosya kullanılır.");
    ImGui::SameLine();
    if (ImGui::Button("Yeniden algıla")) ImGui::OpenPopup("Yeniden algıla?");
    ImGui::SameLine();
    if (ImGui::Button("Robotu kaldır")) {
        removeRobot();
        ImGui::PopItemWidth();
        ImGui::End();
        return;
    }
    if (ImGui::BeginPopupModal("Yeniden algıla?", nullptr, ImGuiWindowFlags_AlwaysAutoResize)) {
        ImGui::TextUnformatted("Eklemler STL dosyalarından tekrar algılanacak; elle yapılan düzenlemeler kaybolur.");
        if (ImGui::Button("Algıla")) {
            std::string err;
            std::vector<std::string> log;
            const std::string folder = R.folder;
            const glm::mat4 mount = R.mount;
            auto fresh = detectRobot(folder, err, log);
            if (fresh) {
                // reuse loaded meshes / scene objects: same files, same order
                if (fresh->parts.size() == R.parts.size()) {
                    for (size_t i = 0; i < R.parts.size(); ++i) {
                        fresh->parts[i].mesh = R.parts[i].mesh;
                        fresh->parts[i].objectId = R.parts[i].objectId;
                    }
                    fresh->mount = mount;
                    fresh->computeIgnoredPairs();
                    robot_ = std::move(fresh);
                    robotLog_ = std::move(log);
                    selectedJoint_ = robot_->drivers().empty() ? -1 : robot_->drivers()[0];
                    syncRobot();
                    robotDirty_ = true;
                    notify("Eklemler yeniden algılandı.");
                } else {
                    pendingRobot_ = folder; // part list changed: full reload
                }
            } else notify(err, true);
            ImGui::CloseCurrentPopup();
        }
        ImGui::SameLine();
        if (ImGui::Button("Vazgeç")) ImGui::CloseCurrentPopup();
        ImGui::EndPopup();
    }

    if (robot_ && ImGui::CollapsingHeader("Algılama raporu")) {
        for (const auto& l : robotLog_) ImGui::TextWrapped("%s", l.c_str());
    }
    ImGui::PopItemWidth();
    ImGui::End();
}

void Editor::drawJointEditor() {
    Robot& R = *robot_;
    if (R.joints.empty()) return;
    if (selectedJoint_ < 0 || selectedJoint_ >= int(R.joints.size())) selectedJoint_ = 0;

    // ordered list: drivers, mimics, fixed
    std::vector<int> order = R.drivers();
    for (int j = 0; j < int(R.joints.size()); ++j)
        if (R.joints[j].type == JointType::Revolute && R.joints[j].mimic >= 0) order.push_back(j);
    for (int j = 0; j < int(R.joints.size()); ++j)
        if (R.joints[j].type == JointType::Fixed) order.push_back(j);
    if (ImGui::BeginCombo("Eklem", R.joints[selectedJoint_].name.c_str())) {
        for (int j : order) {
            if (ImGui::Selectable((R.joints[j].name + "##j" + std::to_string(j)).c_str(), j == selectedJoint_)) selectedJoint_ = j;
        }
        ImGui::EndCombo();
    }
    RobotJoint& J = R.joints[selectedJoint_];
    const int jIdx = selectedJoint_;
    bool changed = false, structural = false;

    char name[128];
    std::snprintf(name, sizeof(name), "%s", J.name.c_str());
    if (ImGui::InputText("Ad", name, sizeof(name))) {
        J.name = name;
        robotDirty_ = true;
    }
    RobotPart& part = R.parts[J.part];
    ImGui::Text("Hareket eden parça: %s", part.name.c_str());
    if (ImGui::SmallButton("Parçayı seç")) activeObject_ = part.objectId;

    // parent part
    const char* parentName = part.parent >= 0 ? R.parts[part.parent].name.c_str() : "(yok)";
    if (ImGui::BeginCombo("Bağlı olduğu parça", parentName)) {
        for (int p = 0; p < int(R.parts.size()); ++p) {
            if (p == J.part || R.isAncestor(J.part, p)) continue; // no cycles
            if (ImGui::Selectable((R.parts[p].name + "##p" + std::to_string(p)).c_str(), p == part.parent)) {
                part.parent = p;
                changed = structural = true;
            }
        }
        ImGui::EndCombo();
    }

    // type
    int type = J.type == JointType::Fixed ? 0 : (J.mimic >= 0 ? 2 : 1);
    if (ImGui::Combo("Tip", &type, "Sabit\0Döner (bağımsız)\0Döner (bağımlı / mimic)\0")) {
        const std::vector<int> drv = R.drivers();
        if (type == 2) {
            int master = -1;
            for (int d : drv)
                if (d != jIdx) {
                    master = d;
                    break;
                }
            if (master < 0) {
                notify("Bağımlı eklem için takip edilecek başka bir bağımsız eklem yok.", true);
                type = J.type == JointType::Fixed ? 0 : 1;
            }
            if (master >= 0) {
                // joints that followed this one now follow the new master
                for (auto& o : R.joints)
                    if (o.mimic == jIdx) {
                        o.mimic = master;
                        o.multiplier *= J.multiplier;
                    }
                J.type = JointType::Revolute;
                J.mimic = master;
                J.multiplier = 1.0f;
                J.offsetDeg = 0.0f;
            }
        } else {
            if (J.isDriver() && type == 0)
                for (auto& o : R.joints)
                    if (o.mimic == jIdx) o.mimic = -1; // followers become independent
            J.type = type == 0 ? JointType::Fixed : JointType::Revolute;
            J.mimic = -1;
            if (J.type == JointType::Revolute) {
                J.minDeg = std::min(J.minDeg, 0.0f);
                J.maxDeg = std::max(J.maxDeg, 0.0f);
                if (glm::length(J.axis) < 1e-6f) J.axis = glm::vec3(0, 0, 1);
            }
        }
        changed = structural = true;
    }

    if (J.type == JointType::Revolute) {
        ImGui::TextDisabled("Eksen, montaj (STL) koordinatlarında, mm:");
        changed |= ImGui::DragFloat3("Eksen noktası", glm::value_ptr(J.origin), 0.05f, 0, 0, "%.2f");
        changed |= ImGui::DragFloat3("Eksen yönü", glm::value_ptr(J.axis), 0.005f, -1.0f, 1.0f, "%.3f");
        if (ImGui::IsItemDeactivatedAfterEdit() && glm::length(J.axis) > 1e-6f) J.axis = glm::normalize(J.axis);
        if (ImGui::SmallButton("X")) J.axis = glm::vec3(1, 0, 0), changed = true;
        ImGui::SameLine();
        if (ImGui::SmallButton("Y")) J.axis = glm::vec3(0, 1, 0), changed = true;
        ImGui::SameLine();
        if (ImGui::SmallButton("Z")) J.axis = glm::vec3(0, 0, 1), changed = true;
        ImGui::SameLine();
        if (ImGui::SmallButton("Yönü ters çevir")) {
            if (J.isDriver()) R.flipDriverDirection(jIdx);
            else J.axis = -J.axis;
            changed = true;
        }
        ImGui::SameLine();
        if (ImGui::SmallButton("Seçimden al")) {
            glm::vec3 o, a;
            std::string how;
            if (axisFromSelection(o, a, how)) {
                const int parent = part.parent;
                J.origin = R.worldToPartModel(parent, o);
                J.axis = R.worldDirToPartModel(parent, a);
                changed = true;
                notify(how);
            } else notify(how, true);
        }
        ImGui::SetItemTooltip("Mevcut eleman seçiminden ekseni hesaplar:\n"
                              "  Nokta: daire üzerinde 3+ nokta -> daire merkezi ve normali; 2 nokta -> doğru\n"
                              "  Kenar: tek kenar -> doğrultusu; bir deliğin kenarları -> daire ekseni\n"
                              "  Yüzey: silindirik yüzey (mil/delik) -> silindir ekseni; düz yüzey -> normali");

        if (J.isDriver()) {
            if (ImGui::DragFloatRange2("Limit (°)", &J.minDeg, &J.maxDeg, 0.5f, -360.0f, 360.0f, "min %.0f°", "max %.0f°")) {
                J.angleDeg = std::clamp(J.angleDeg, J.minDeg, J.maxDeg);
                changed = true;
            }
            if (ImGui::Button("Limitleri otomatik bul")) {
                R.autoLimits(jIdx);
                changed = true;
                char buf[128];
                std::snprintf(buf, sizeof(buf), "%s limitleri: %.0f° .. %.0f° (kendi kendine çarpışma / zemin)", J.name.c_str(), J.minDeg, J.maxDeg);
                notify(buf);
            }
            ImGui::SetItemTooltip("Eklemi +-90° arasında tarar; robotun kendisiyle veya zeminle\nçarpıştığı ilk açıdan önceki değerleri limit yapar.");
        } else {
            const std::vector<int> drv = R.drivers();
            const char* masterName = J.mimic >= 0 ? R.joints[J.mimic].name.c_str() : "-";
            if (ImGui::BeginCombo("Takip ettiği eklem", masterName)) {
                for (int d : drv)
                    if (d != jIdx && ImGui::Selectable((R.joints[d].name + "##m" + std::to_string(d)).c_str(), d == J.mimic)) {
                        J.mimic = d;
                        changed = structural = true;
                    }
                ImGui::EndCombo();
            }
            changed |= ImGui::DragFloat("Çarpan", &J.multiplier, 0.01f, -10.0f, 10.0f, "%.3f");
            changed |= ImGui::DragFloat("Ofset (°)", &J.offsetDeg, 0.1f, -180.0f, 180.0f, "%.1f");
            ImGui::TextDisabled("açı = çarpan x %s + ofset", masterName);
        }
    }

    if (changed) {
        if (std::string v = R.validate(); !v.empty()) notify(v, true);
        if (structural) R.computeIgnoredPairs();
        syncRobot();
        robotDirty_ = true;
    }
}

void Editor::drawRobotPartProperties(SceneObject& o) {
    Robot& R = *robot_;
    const RobotPart& part = R.parts[o.robotPart];
    ImGui::Text("Robot parçası: %s", o.name.c_str());
    ImGui::TextDisabled("%s", part.file.c_str());
    const int j = part.joint;
    if (j < 0) ImGui::TextUnformatted("Taban (sabit kök parça)");
    else {
        const RobotJoint& J = R.joints[j];
        if (J.type == JointType::Fixed) ImGui::Text("Bağlantı: sabit -> %s", R.parts[part.parent].name.c_str());
        else if (J.mimic >= 0) ImGui::Text("Eklem: %s (bağımlı, x%.2f)", J.name.c_str(), J.multiplier);
        else ImGui::Text("Eklem: %s  (%.1f°)", J.name.c_str(), J.angleDeg);
        if (ImGui::SmallButton("Bu eklemi düzenle")) {
            selectedJoint_ = j;
            focusRobotPanel_ = true;
        }
    }
    ImGui::ColorEdit3("Renk", glm::value_ptr(o.color), ImGuiColorEditFlags_NoInputs);
    ImGui::SameLine();
    if (ImGui::Checkbox("Görünür", &o.visible)) scene_.markDirty();
    if (ImGui::Checkbox("Çarpışma kontrolüne dahil", &o.collidable)) scene_.markDirty();
    ImGui::SeparatorText("Geometri");
    const AABB wb = o.worldBounds();
    ImGui::Text("Üçgen: %zu   Nokta: %zu", o.mesh->triangles.size(), o.mesh->positions.size());
    ImGui::Text("Boyut (dünya): %.1f x %.1f x %.1f mm", wb.size().x, wb.size().y, wb.size().z);
    ImGui::Spacing();
    ImGui::TextDisabled("Robot parçaları sürüklenince / gizmo ile tüm robot taşınır.");
    if (ImGui::Button("Robotu zemine indir")) dropToGround(o);
    ImGui::SameLine();
    if (ImGui::Button("Robotu kaldır")) removeRobot();
}

// =============================================================================================
// Render data and overlay
// =============================================================================================

void Editor::addRobotRenderData(FrameScene& fs) {
    if (!robot_) return;
    Robot& R = *robot_;
    AABB rb;
    for (const auto& p : R.parts) rb.expand(p.mesh->bounds);
    const float L = std::max(15.0f, rb.diagonal() * 0.06f);

    if (showJointAxes_) {
        LineBatch drv, mim, sel;
        drv.color = kDriverAxisColor;
        drv.width = 2.0f;
        mim.color = kMimicAxisColor;
        mim.width = 1.5f;
        sel.color = kSelectedAxisColor;
        sel.width = 3.5f;
        for (int j = 0; j < int(R.joints.size()); ++j) {
            const RobotJoint& J = R.joints[j];
            if (J.type != JointType::Revolute) continue;
            const bool isSel = j == selectedJoint_;
            if (!isSel && J.mimic >= 0 && !showAllJoints_) continue;
            const glm::vec3 o = R.jointOriginWorld(j), a = R.jointAxisWorld(j);
            const float len = J.mimic >= 0 ? L * 0.5f : L;
            LineBatch& b = isSel ? sel : (J.mimic >= 0 ? mim : drv);
            b.points.insert(b.points.end(), {o - a * len, o + a * len});
        }
        for (LineBatch* b : {&mim, &drv, &sel}) {
            b->depthTest = false;
            if (!b->points.empty()) fs.overlayLines.push_back(std::move(*b));
        }
    }
    if (showTcp_) {
        const glm::mat4 t = R.tcpWorld();
        const glm::vec3 p = glm::vec3(t[3]);
        const glm::vec3 cols[3] = {{0.95f, 0.25f, 0.25f}, {0.3f, 0.9f, 0.3f}, {0.3f, 0.55f, 1.0f}};
        for (int k = 0; k < 3; ++k) {
            LineBatch b;
            b.color = glm::vec4(cols[k], 1.0f);
            b.width = 2.5f;
            b.depthTest = false;
            b.points = {p, p + glm::normalize(glm::vec3(t[k])) * L * 0.8f};
            fs.overlayLines.push_back(std::move(b));
        }
    }
}

void Editor::drawRobotOverlay(ImDrawList* dl) {
    if (!robot_) return;
    Robot& R = *robot_;
    const glm::vec2 size(vpSize_.x, vpSize_.y);
    auto label = [&](const glm::vec3& w, const char* text, ImU32 col) {
        glm::vec2 s;
        if (!camera_.worldToViewport(w, size, s)) return;
        const ImVec2 p(vpPos_.x + s.x + 8, vpPos_.y + s.y - 8);
        const ImVec2 ts = ImGui::CalcTextSize(text);
        dl->AddRectFilled(ImVec2(p.x - 3, p.y - 1), ImVec2(p.x + ts.x + 3, p.y + ts.y + 1), IM_COL32(15, 17, 20, 170), 3.0f);
        dl->AddText(p, col, text);
        dl->AddCircleFilled(ImVec2(vpPos_.x + s.x, vpPos_.y + s.y), 3.5f, col);
    };
    if (showJointAxes_) {
        for (int d : R.drivers()) {
            char buf[96];
            const float shown = std::round(R.joints[d].angleDeg) + 0.0f;
            std::snprintf(buf, sizeof(buf), "%s %.0f°", R.joints[d].name.c_str(), shown);
            label(R.jointOriginWorld(d), buf, d == selectedJoint_ ? IM_COL32(255, 140, 30, 255) : IM_COL32(255, 215, 60, 255));
        }
        if (selectedJoint_ >= 0 && selectedJoint_ < int(R.joints.size()) && !R.joints[selectedJoint_].isDriver() &&
            R.joints[selectedJoint_].type == JointType::Revolute)
            label(R.jointOriginWorld(selectedJoint_), R.joints[selectedJoint_].name.c_str(), IM_COL32(255, 140, 30, 255));
    }
    if (showTcp_) label(glm::vec3(R.tcpWorld()[3]), "TCP", IM_COL32(230, 230, 240, 255));
}

} // namespace pegasus
