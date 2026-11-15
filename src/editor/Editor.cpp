#include "editor/Editor.h"

#include "scene/Primitives.h"
#include "scene/StlLoader.h"

#include <imgui_internal.h>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <filesystem>
#include <set>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>

namespace pegasus {

namespace {

const glm::vec4 kSelectColor{1.0f, 0.55f, 0.10f, 1.0f};
const glm::vec4 kHoverColor{0.30f, 0.80f, 1.00f, 1.0f};
const glm::vec4 kCollisionColor{0.95f, 0.18f, 0.15f, 1.0f};

ImU32 toU32(const glm::vec4& c, float alpha = -1.0f) {
    return ImGui::ColorConvertFloat4ToU32(ImVec4(c.r, c.g, c.b, alpha < 0.0f ? c.a : alpha));
}

const char* toolName(Tool t) {
    switch (t) {
    case Tool::Vertex: return "Nokta";
    case Tool::Edge: return "Kenar";
    case Tool::Face: return "Yüzey";
    default: return "Nesne";
    }
}

const char* faceModeName(FaceMode m) {
    switch (m) {
    case FaceMode::Triangle: return "Tek üçgen";
    case FaceMode::Smooth: return "Yumuşak yüzey";
    default: return "Düzlemsel yüzey";
    }
}

bool sameLinearPart(const glm::mat4& a, const glm::mat4& b) {
    for (int c = 0; c < 3; ++c)
        for (int r = 0; r < 3; ++r)
            if (std::abs(a[c][r] - b[c][r]) > 1e-5f) return false;
    return true;
}

glm::vec3 transformPoint(const glm::mat4& m, const glm::vec3& p) { return glm::vec3(m * glm::vec4(p, 1.0f)); }

} // namespace

Editor::Editor() {
    camera_.yawDeg = -55.0f;
    camera_.pitchDeg = 28.0f;
    collision_.ignorePair = [this](const SceneObject& a, const SceneObject& b) {
        return robot_ && a.robotPart >= 0 && b.robotPart >= 0 && robot_->ignores(a.robotPart, b.robotPart);
    };
}

// =============================================================================================
// Actions
// =============================================================================================

void Editor::notify(const std::string& msg, bool error) {
    status_ = msg;
    statusError_ = error;
    statusTimer_ = error ? 8.0f : 4.0f;
    std::fprintf(error ? stderr : stdout, "[pegasus] %s\n", msg.c_str());
}

bool Editor::openStl(const std::string& path) {
    std::string err;
    MeshPtr mesh = loadStl(path, err);
    if (!mesh) {
        notify(err, true);
        return false;
    }
    if (autoPlace_) {
        const AABB b = mesh->bounds;
        mesh->translateVertices(glm::vec3(b.center().x, b.center().y, b.min.z)); // pivot = bottom centre
    }
    mesh->setFeatureAngle(featureAngle_);
    SceneObject* obj = scene_.add(mesh, mesh->name);
    obj->sourcePath = path;
    if (autoPlace_) placeBesideScene(*obj);
    activeObject_ = obj->id;
    char buf[512];
    std::snprintf(buf, sizeof(buf), "Yüklendi: %s  (%zu üçgen, %zu nokta)", obj->name.c_str(), mesh->triangles.size(),
                  mesh->positions.size());
    notify(buf);
    if (scene_.objects().size() == 1) frameAll();
    return true;
}

void Editor::addPrimitive(int kind, const glm::vec3& boxSize) {
    MeshPtr mesh;
    switch (kind) {
    case 0: mesh = makeBox(boxSize); break;
    case 1: mesh = makeCylinder(25.0f, 90.0f); break;
    default: mesh = makeSphere(35.0f); break;
    }
    mesh->setFeatureAngle(featureAngle_);
    SceneObject* obj = scene_.add(mesh, mesh->name);
    placeBesideScene(*obj);
    activeObject_ = obj->id;
    if (scene_.objects().size() == 1) frameAll();
}

void Editor::placeBesideScene(SceneObject& obj) {
    AABB others;
    for (const auto& o : scene_.objects())
        if (o.get() != &obj && o->visible) others.expand(o->worldBounds());
    const AABB lb = obj.mesh->bounds.transformed(Transform{glm::vec3(0.0f), obj.transform.rotationDeg, obj.transform.scale}.matrix());
    glm::vec3 pos(-lb.center().x, -lb.center().y, -lb.min.z);
    if (others.valid()) {
        const float gap = std::max(15.0f, lb.diagonal() * 0.15f);
        pos.x = others.max.x + gap - lb.min.x;
        pos.y = others.center().y - lb.center().y;
    }
    obj.transform.position = pos;
    scene_.markDirty();
}

void Editor::dropToGround(SceneObject& obj) {
    float minZ = std::numeric_limits<float>::max();
    for (const auto& o : scene_.objects()) {
        const bool sameBody = o.get() == &obj || (obj.robotPart >= 0 && o->robotPart >= 0);
        if (!sameBody) continue;
        const glm::mat4 m = o->matrix();
        for (const auto& p : o->mesh->positions) minZ = std::min(minZ, transformPoint(m, p).z);
    }
    if (minZ == std::numeric_limits<float>::max()) return;
    applyTransform(obj, glm::translate(glm::mat4(1.0f), glm::vec3(0.0f, 0.0f, -minZ)) * obj.matrix());
}

void Editor::deleteObject(uint32_t id) {
    if (const SceneObject* o = scene_.find(id); o && o->robotPart >= 0) {
        removeRobot();
        return;
    }
    selection_.erase(std::remove_if(selection_.begin(), selection_.end(), [&](const ElementRef& r) { return r.objectId == id; }),
                     selection_.end());
    if (activeObject_ == id) activeObject_ = 0;
    if (dragObject_ == id) dragCandidate_ = dragging_ = false;
    hover_ = {};
    hoverObject_ = 0;
    scene_.remove(id);
}

void Editor::duplicateActive() {
    if (SceneObject* src = scene_.find(activeObject_)) {
        if (src->robotPart >= 0) {
            notify("Robot parçaları çoğaltılamaz.", true);
            return;
        }
        SceneObject* obj = scene_.duplicate(src->id);
        obj->transform.position.x += src->worldBounds().size().x + 15.0f;
        scene_.markDirty();
        activeObject_ = obj->id;
    }
}

void Editor::setTool(Tool t) {
    if (t == tool_) return;
    tool_ = t;
    hover_ = {};
    hoverObject_ = 0;
    lastHoverVersion_ = 0;
}

void Editor::clearSelection() {
    selection_.clear();
    hover_ = {};
}

void Editor::frameAll() {
    const AABB b = scene_.bounds();
    if (b.valid()) camera_.frame(b);
    else {
        camera_.target = glm::vec3(0, 0, 50);
        camera_.distance = 600.0f;
    }
}

void Editor::frameSelection() {
    if (const SceneObject* o = scene_.find(activeObject_)) camera_.frame(o->worldBounds());
    else frameAll();
}

bool Editor::applyTransform(SceneObject& obj, const glm::mat4& target) {
    if (obj.robotPart >= 0 && robot_) {
        // Moving any robot part moves the whole robot (its mount). Scale is not allowed.
        glm::mat4 delta = target * glm::inverse(obj.matrix());
        for (int c = 0; c < 3; ++c) delta[c] = glm::vec4(glm::normalize(glm::vec3(delta[c])), 0.0f);
        delta[1] = glm::vec4(glm::normalize(glm::cross(glm::vec3(delta[2]), glm::vec3(delta[0]))), 0.0f);
        delta[2] = glm::vec4(glm::cross(glm::vec3(delta[0]), glm::vec3(delta[1])), 0.0f);
        const glm::mat4 oldMount = robot_->mount;
        const bool wasColliding = robotColliding();
        robot_->mount = delta * oldMount;
        syncRobot();
        if (blockOnCollision_ && collision_.enabled && !wasColliding && robotColliding()) {
            robot_->mount = oldMount;
            syncRobot();
            blockedFlash_ = 1.2f;
            return false;
        }
        robotDirty_ = true;
        return true;
    }
    glm::mat4 final = target;
    if (blockOnCollision_ && collision_.enabled && obj.collidable && obj.visible && !collision_.isColliding(obj.id) &&
        collision_.collidesAt(scene_, obj, target)) {
        const glm::mat4 cur = obj.matrix();
        blockedFlash_ = 1.2f;
        if (!sameLinearPart(cur, target)) return false; // rotation / scale into collision: reject
        // Pure translation: move as far as possible towards the target without touching.
        float lo = 0.0f, hi = 1.0f;
        glm::mat4 m = cur;
        for (int i = 0; i < 12; ++i) {
            const float mid = 0.5f * (lo + hi);
            m[3] = glm::mix(cur[3], target[3], mid);
            if (collision_.collidesAt(scene_, obj, m)) hi = mid;
            else lo = mid;
        }
        if (lo <= 0.0f) return false;
        final = cur;
        final[3] = glm::mix(cur[3], target[3], lo);
    }
    obj.transform = Transform::fromMatrix(final);
    scene_.markDirty();
    return true;
}

// =============================================================================================
// Frame update
// =============================================================================================

void Editor::update(float dt) {
    fps_ = fps_ <= 0.0f ? 1.0f / std::max(dt, 1e-4f) : fps_ * 0.95f + 0.05f / std::max(dt, 1e-4f);
    statusTimer_ -= dt;
    blockedFlash_ -= dt;

    if (!pendingFiles_.empty()) {
        auto files = std::move(pendingFiles_);
        pendingFiles_.clear();
        bool any = false;
        for (const auto& f : files) any |= openStl(f);
        if (any && files.size() > 1) frameAll();
    }

    if (!pendingRobot_.empty()) {
        const std::string folder = pendingRobot_;
        pendingRobot_.clear();
        loadRobot(folder);
    }

    updateMotion(dt);
    updateArduino();

    collision_.update(scene_);
    const AABB sb = scene_.bounds();
    if (sb.valid()) camera_.sceneRadius = std::max(500.0f, sb.diagonal() + glm::length(sb.center()));

    drawMenuBar();
    drawStatusBar();

    ImGuiViewport* mainVp = ImGui::GetMainViewport();
    const ImGuiID dockId = ImGui::GetID("PegasusDockSpace");
    if (!layoutBuilt_) setupDockLayout(dockId);
    ImGui::DockSpaceOverViewport(dockId, mainVp, ImGuiDockNodeFlags_PassthruCentralNode);
    if (ImGuiDockNode* central = ImGui::DockBuilderGetCentralNode(dockId)) {
        vpPos_ = central->Pos;
        vpSize_ = ImVec2(std::max(central->Size.x, 1.0f), std::max(central->Size.y, 1.0f));
    }

    drawToolsPanel();
    drawScenePanel();
    drawPropertiesPanel();
    drawSelectionPanel();
    drawCollisionPanel();
    drawRobotPanel();
    drawMotionPanel();
    drawViewport();
    handleShortcuts();
    if (showHelp_) drawHelpWindow();
    drawArduinoWindow();

    std::string picked;
    if (fileBrowser_.draw(picked)) openStl(picked);
    if (folderBrowser_.draw(picked)) loadRobot(picked);

    collision_.update(scene_); // reflect this frame's edits in the rendered colours
}

void Editor::setupDockLayout(ImGuiID dockId) {
    layoutBuilt_ = true;
    ImGuiViewport* vp = ImGui::GetMainViewport();
    ImGui::DockBuilderRemoveNode(dockId);
    ImGui::DockBuilderAddNode(dockId, ImGuiDockNodeFlags(ImGuiDockNodeFlags_DockSpace) | ImGuiDockNodeFlags_PassthruCentralNode);
    ImGui::DockBuilderSetNodeSize(dockId, vp->WorkSize);
    ImGuiID center = dockId;
    ImGuiID left = ImGui::DockBuilderSplitNode(center, ImGuiDir_Left, 0.21f, nullptr, &center);
    ImGuiID right = ImGui::DockBuilderSplitNode(center, ImGuiDir_Right, 0.27f, nullptr, &center);
    ImGuiID leftBottom;
    ImGuiID leftTop = ImGui::DockBuilderSplitNode(left, ImGuiDir_Up, 0.55f, nullptr, &leftBottom);
    ImGuiID rightBottom;
    ImGuiID rightTop = ImGui::DockBuilderSplitNode(right, ImGuiDir_Up, 0.66f, nullptr, &rightBottom);
    ImGui::DockBuilderDockWindow("Araçlar", leftTop);
    ImGui::DockBuilderDockWindow("Sahne", leftBottom);
    ImGui::DockBuilderDockWindow("Robot", rightTop);
    ImGui::DockBuilderDockWindow("Hareket", rightTop);
    ImGui::DockBuilderDockWindow("Özellikler", rightTop);
    ImGui::DockBuilderDockWindow("Seçim", rightBottom);
    ImGui::DockBuilderDockWindow("Çarpışmalar", rightBottom);
    ImGui::DockBuilderFinish(dockId);
}

// =============================================================================================
// Menus, status bar, shortcuts
// =============================================================================================

void Editor::drawMenuBar() {
    if (!ImGui::BeginMainMenuBar()) return;
    SceneObject* active = scene_.find(activeObject_);
    if (ImGui::BeginMenu("Dosya")) {
        if (ImGui::MenuItem("STL Aç...", "Ctrl+O")) fileBrowser_.open();
        if (ImGui::MenuItem("Robot Kol Yükle (klasör)...")) folderBrowser_.open();
        if (ImGui::MenuItem("Robotu kaydet", nullptr, false, robot_ != nullptr)) {
            std::string err;
            const std::string path = robot_->folder + "/robot.pegasus";
            if (robot_->save(path, err)) {
                robotDirty_ = false;
                notify("Robot kaydedildi: " + path);
            } else notify(err, true);
        }
        if (ImGui::MenuItem("Sahneyi temizle", nullptr, false, !scene_.objects().empty())) {
            robot_.reset();
            selectedJoint_ = -1;
            scene_.clear();
            clearSelection();
            activeObject_ = 0;
        }
        ImGui::Separator();
        if (ImGui::MenuItem("Çıkış", "Ctrl+Q")) quitRequested_ = true;
        ImGui::EndMenu();
    }
    if (ImGui::BeginMenu("Ekle")) {
        if (ImGui::MenuItem("Kutu")) addPrimitive(0);
        if (ImGui::MenuItem("Silindir")) addPrimitive(1);
        if (ImGui::MenuItem("Küre")) addPrimitive(2);
        ImGui::Separator();
        if (ImGui::MenuItem("Robot kol (klasörden)...")) folderBrowser_.open();
        ImGui::EndMenu();
    }
    if (ImGui::BeginMenu("Düzen")) {
        if (ImGui::MenuItem("Çoğalt", "Ctrl+D", false, active != nullptr)) duplicateActive();
        if (ImGui::MenuItem("Sil", "Delete", false, active != nullptr)) deleteObject(activeObject_);
        if (ImGui::MenuItem("Zemine indir", "G", false, active != nullptr)) dropToGround(*active);
        if (ImGui::MenuItem("Seçimi temizle", "Esc")) clearSelection();
        ImGui::EndMenu();
    }
    if (ImGui::BeginMenu("Görünüm")) {
        if (ImGui::MenuItem("Tümünü sığdır", "Home")) frameAll();
        if (ImGui::MenuItem("Seçili nesneye odaklan", "F")) frameSelection();
        ImGui::Separator();
        if (ImGui::MenuItem("Üstten")) camera_.setView(-90.0f, 89.9f);
        if (ImGui::MenuItem("Önden")) camera_.setView(-90.0f, 0.0f);
        if (ImGui::MenuItem("Sağdan")) camera_.setView(0.0f, 0.0f);
        if (ImGui::MenuItem("İzometrik")) camera_.setView(-45.0f, 35.26f);
        ImGui::Separator();
        ImGui::MenuItem("Izgara", nullptr, &showGrid_);
        ImGui::MenuItem("Kenar çizgileri", nullptr, &showEdges_);
        ImGui::MenuItem("Temas üçgenleri", nullptr, &showContacts_);
        ImGui::EndMenu();
    }
    if (ImGui::BeginMenu("Arduino")) {
        ImGui::MenuItem("Bağlantı penceresi", nullptr, &showArduino_);
        if (link_ && link_->connected()) {
            ImGui::MenuItem("Robotu takip et (canlı)", nullptr, &followRobot_);
        }
        if (ImGui::MenuItem("Acil durdur", "Space")) emergencyStop();
        ImGui::EndMenu();
    }
    if (ImGui::BeginMenu("Yardım")) {
        ImGui::MenuItem("Kısayollar", "F1", &showHelp_);
        ImGui::EndMenu();
    }
    ImGui::EndMainMenuBar();
}

void Editor::drawStatusBar() {
    const ImGuiWindowFlags flags = ImGuiWindowFlags_NoScrollbar | ImGuiWindowFlags_NoSavedSettings | ImGuiWindowFlags_MenuBar;
    if (ImGui::BeginViewportSideBar("##StatusBar", ImGui::GetMainViewport(), ImGuiDir_Down, ImGui::GetFrameHeight(), flags)) {
        if (ImGui::BeginMenuBar()) {
            ImGui::Text("Araç: %s", toolName(tool_));
            ImGui::Separator();
            ImGui::Text("Nesne: %zu", scene_.objects().size());
            ImGui::Separator();
            {
                const LinkStatus ls = link_ ? link_->status() : LinkStatus{};
                const ImVec4 col = ls.state == LinkState::Connected ? ImVec4(0.4f, 0.9f, 0.45f, 1) : ls.state == LinkState::Error ? ImVec4(1, 0.45f, 0.4f, 1)
                                 : ls.state == LinkState::Connecting ? ImVec4(1, 0.8f, 0.3f, 1) : ImVec4(0.6f, 0.6f, 0.6f, 1);
                const char* txt = ls.state == LinkState::Connected ? (!ls.homed ? "Arduino: bağlı, servolar serbest"
                                                                     : followRobot_ ? "Arduino: bağlı, canlı takip" : "Arduino: bağlı")
                                : ls.state == LinkState::Error ? "Arduino: hata" : ls.state == LinkState::Connecting ? "Arduino: bağlanıyor" : "Arduino: bağlı değil";
                ImGui::PushStyleColor(ImGuiCol_Text, col);
                if (ImGui::Selectable(txt, false, ImGuiSelectableFlags_None, ImVec2(ImGui::CalcTextSize(txt).x, 0))) showArduino_ = true;
                ImGui::PopStyleColor();
                ImGui::SetItemTooltip("Arduino bağlantı penceresini aç");
            }
            ImGui::Separator();
            if (!collision_.enabled) ImGui::TextDisabled("Çarpışma kontrolü kapalı");
            else if (collision_.collisions().empty()) ImGui::TextColored(ImVec4(0.5f, 0.9f, 0.5f, 1), "Çarpışma yok");
            else ImGui::TextColored(ImVec4(1.0f, 0.4f, 0.35f, 1), "%zu çarpışma!", collision_.collisions().size());
            ImGui::Separator();
            if (statusTimer_ > 0.0f) {
                if (statusError_) ImGui::TextColored(ImVec4(1.0f, 0.45f, 0.4f, 1), "%s", status_.c_str());
                else ImGui::TextUnformatted(status_.c_str());
            } else {
                switch (tool_) {
                case Tool::Object: ImGui::TextDisabled("Sol tık: seç / sürükle (Shift: dikey)  |  Sağ tık: döndür  |  Shift+Sağ: kaydır  |  Teker: yakınlaş"); break;
                case Tool::Vertex: ImGui::TextDisabled("Sol tık: nokta seç  |  Shift+tık: seçime ekle/çıkar"); break;
                case Tool::Edge: ImGui::TextDisabled("Sol tık: kenar seç  |  Shift+tık: seçime ekle/çıkar"); break;
                case Tool::Face: ImGui::TextDisabled("Sol tık: yüzey seç  |  Shift+tık: seçime ekle/çıkar"); break;
                }
            }
            const float w = ImGui::CalcTextSize("000.0 FPS").x;
            ImGui::SameLine(ImGui::GetWindowWidth() - w - 16.0f);
            ImGui::TextDisabled("%.1f FPS", fps_);
            ImGui::EndMenuBar();
        }
    }
    ImGui::End();
}

void Editor::handleShortcuts() {
    ImGuiIO& io = ImGui::GetIO();
    // emergency stop first: works with any window, dialog or tool active (only typing in a text box
    // keeps Space for the text)
    if (!io.WantTextInput && ImGui::IsKeyPressed(ImGuiKey_Space, false)) emergencyStop();
    if (io.WantTextInput || ImGui::IsPopupOpen(nullptr, ImGuiPopupFlags_AnyPopupId)) return;
    if (ImGui::IsKeyChordPressed(ImGuiMod_Ctrl | ImGuiKey_O)) fileBrowser_.open();
    if (ImGui::IsKeyChordPressed(ImGuiMod_Ctrl | ImGuiKey_Q)) quitRequested_ = true;
    if (ImGui::IsKeyChordPressed(ImGuiMod_Ctrl | ImGuiKey_D)) duplicateActive();
    if (tool_ == Tool::Face) {
        if (ImGui::IsKeyChordPressed(ImGuiMod_Ctrl | ImGuiKey_L)) selectLinkedFromSelection();
        if (ImGui::IsKeyChordPressed(ImGuiMod_Ctrl | ImGuiKey_Equal) || ImGui::IsKeyChordPressed(ImGuiMod_Ctrl | ImGuiKey_KeypadAdd) ||
            ImGui::IsKeyChordPressed(ImGuiMod_Ctrl | ImGuiMod_Shift | ImGuiKey_Equal))
            growShrinkSelection(true);
        if (ImGui::IsKeyChordPressed(ImGuiMod_Ctrl | ImGuiKey_Minus) || ImGui::IsKeyChordPressed(ImGuiMod_Ctrl | ImGuiKey_KeypadSubtract) ||
            ImGui::IsKeyChordPressed(ImGuiMod_Ctrl | ImGuiKey_Slash))
            growShrinkSelection(false);
        if (ImGui::IsKeyChordPressed(ImGuiMod_Ctrl | ImGuiKey_I)) invertFaceSelection();
    }
    if (io.KeyCtrl || io.KeyAlt) return;
    if (ImGui::IsKeyPressed(ImGuiKey_Q, false)) setTool(Tool::Object);
    if (ImGui::IsKeyPressed(ImGuiKey_1, false)) setTool(Tool::Vertex);
    if (ImGui::IsKeyPressed(ImGuiKey_2, false)) setTool(Tool::Edge);
    if (ImGui::IsKeyPressed(ImGuiKey_3, false)) setTool(Tool::Face);
    if (ImGui::IsKeyPressed(ImGuiKey_W, false)) gizmoOp_ = ImGuizmo::TRANSLATE;
    if (ImGui::IsKeyPressed(ImGuiKey_E, false)) gizmoOp_ = ImGuizmo::ROTATE;
    if (ImGui::IsKeyPressed(ImGuiKey_R, false)) gizmoOp_ = ImGuizmo::SCALE;
    if (ImGui::IsKeyPressed(ImGuiKey_L, false)) {
        if (tool_ == Tool::Face) {
            if (vpHovered_) selectLinkedUnderCursor(io.KeyShift);
        } else {
            gizmoMode_ = gizmoMode_ == ImGuizmo::WORLD ? ImGuizmo::LOCAL : ImGuizmo::WORLD;
        }
    }
    if (tool_ == Tool::Face && ImGui::IsKeyPressed(ImGuiKey_C, false)) brushMode_ = !brushMode_;
    if (tool_ == Tool::Face && ImGui::IsKeyPressed(ImGuiKey_A, false)) selectAllFaces(hoverObject_ ? hoverObject_ : activeObject_);
    if (ImGui::IsKeyPressed(ImGuiKey_F, false)) frameSelection();
    if (ImGui::IsKeyPressed(ImGuiKey_Home, false)) frameAll();
    if (ImGui::IsKeyPressed(ImGuiKey_F1, false)) showHelp_ = !showHelp_;
    if (ImGui::IsKeyPressed(ImGuiKey_Escape, false)) {
        if (brushMode_ && tool_ == Tool::Face) brushMode_ = false;
        else clearSelection();
    }
    if (SceneObject* a = scene_.find(activeObject_)) {
        if (ImGui::IsKeyPressed(ImGuiKey_Delete, false)) deleteObject(a->id);
        else if (ImGui::IsKeyPressed(ImGuiKey_G, false)) dropToGround(*a);
    }
}

void Editor::drawHelpWindow() {
    ImGui::SetNextWindowSize(ImVec2(460, 0), ImGuiCond_Appearing);
    if (ImGui::Begin("Kısayollar", &showHelp_, ImGuiWindowFlags_NoDocking)) {
        if (ImGui::BeginTable("keys", 2, ImGuiTableFlags_RowBg | ImGuiTableFlags_BordersInnerV)) {
            const char* rows[][2] = {
                {"Q / 1 / 2 / 3", "Nesne / Nokta / Kenar / Yüzey aracı"},
                {"W / E / R", "Gizmo: Taşı / Döndür / Ölçekle"},
                {"L", "Gizmo dünya / yerel eksen (Yüzey aracında: bağlıyı seç)"},
                {"Yüzey: L / Shift+L", "İmleç altındaki bağlı yüzeyi seç / çıkar"},
                {"Yüzey: Ctrl+L", "Seçimi bağlı yüzeye genişlet"},
                {"Yüzey: Ctrl + / Ctrl -", "Seçimi bir halka büyüt / küçült"},
                {"Yüzey: Alt + tık", "Döngü seçimi (Shift ile ekle)"},
                {"Yüzey: C", "Fırça seçimi (sürükle; Shift çıkarır, teker boyut)"},
                {"Yüzey: A / Ctrl+I", "Nesnenin tüm yüzeyi / seçimi ters çevir"},
                {"Sol tık + sürükle", "Nesneyi zeminde (XY) taşı"},
                {"Shift + sol sürükle", "Nesneyi dikey (Z) taşı"},
                {"Ctrl (sürüklerken)", "Adımlı (snap) hareket"},
                {"Shift + tık", "Seçime ekle / çıkar"},
                {"Sağ / orta tık sürükle", "Kamerayı döndür (Alt+sol da olur)"},
                {"Shift + sağ/orta sürükle", "Kamerayı kaydır"},
                {"Fare tekeri", "Yakınlaş / uzaklaş"},
                {"F / Home", "Seçili nesneye / tümüne odaklan"},
                {"G", "Seçili nesneyi zemine indir"},
                {"Ctrl+D / Delete", "Çoğalt / sil"},
                {"Ctrl+O", "STL aç (dosyayı pencereye sürükleyip bırakmak da olur)"},
                {"Esc", "Seçimi temizle"},
                {"Space", "ACİL DURDUR: gerçek kol ve hareket animasyonu durur, canlı takip kapanır"},
            };
            for (auto& r : rows) {
                ImGui::TableNextRow();
                ImGui::TableNextColumn();
                ImGui::TextUnformatted(r[0]);
                ImGui::TableNextColumn();
                ImGui::TextUnformatted(r[1]);
            }
            ImGui::EndTable();
        }
    }
    ImGui::End();
}

// =============================================================================================
// Panels
// =============================================================================================

void Editor::drawToolsPanel() {
    if (!ImGui::Begin("Araçlar")) {
        ImGui::End();
        return;
    }
    ImGui::PushItemWidth(-ImGui::GetFontSize() * 8.0f);
    ImGui::SeparatorText("Seçim aracı");
    const Tool tools[] = {Tool::Object, Tool::Vertex, Tool::Edge, Tool::Face};
    const char* keys[] = {"Q", "1", "2", "3"};
    for (int i = 0; i < 4; ++i) {
        if (i > 0) ImGui::SameLine();
        const bool on = tool_ == tools[i];
        if (on) ImGui::PushStyleColor(ImGuiCol_Button, ImGui::GetStyleColorVec4(ImGuiCol_ButtonActive));
        char label[64];
        std::snprintf(label, sizeof(label), "%s##tool%d", toolName(tools[i]), i);
        if (ImGui::Button(label)) setTool(tools[i]);
        if (on) ImGui::PopStyleColor();
        ImGui::SetItemTooltip("Kısayol: %s", keys[i]);
    }
    if (tool_ == Tool::Face) {
        int fm = int(faceMode_);
        if (ImGui::Combo("Yüzey tipi", &fm, "Tek üçgen\0Düzlemsel yüzey\0Yumuşak yüzey\0")) {
            faceMode_ = FaceMode(fm);
            if (selKind_ == Tool::Face) clearSelection();
        }
        ImGui::SetItemTooltip("Düzlemsel: aynı düzlemdeki bitişik üçgenler.\nYumuşak: keskin kenarla ayrılmamış bölge (ör. silindir yüzeyi).");
        drawFaceSelectionTools();
    }
    if (tool_ == Tool::Edge) {
        int es = int(edgeSet_);
        if (ImGui::Combo("Kenar kümesi", &es, "Keskin kenarlar\0Tüm kenarlar (üçgen ağı)\0")) edgeSet_ = EdgeSet(es);
    }
    ImGui::SliderFloat("Seçim yarıçapı", &pickRadius_, 4.0f, 30.0f, "%.0f px");

    ImGui::SeparatorText("Taşıma (gizmo)");
    ImGui::Checkbox("Gizmo göster", &gizmoEnabled_);
    if (ImGui::RadioButton("Taşı (W)", gizmoOp_ == ImGuizmo::TRANSLATE)) gizmoOp_ = ImGuizmo::TRANSLATE;
    ImGui::SameLine();
    if (ImGui::RadioButton("Döndür (E)", gizmoOp_ == ImGuizmo::ROTATE)) gizmoOp_ = ImGuizmo::ROTATE;
    ImGui::SameLine();
    if (ImGui::RadioButton("Ölçek (R)", gizmoOp_ == ImGuizmo::SCALE)) gizmoOp_ = ImGuizmo::SCALE;
    if (ImGui::RadioButton("Dünya", gizmoMode_ == ImGuizmo::WORLD)) gizmoMode_ = ImGuizmo::WORLD;
    ImGui::SameLine();
    if (ImGui::RadioButton("Yerel (L)", gizmoMode_ == ImGuizmo::LOCAL)) gizmoMode_ = ImGuizmo::LOCAL;
    ImGui::Checkbox("Adımlı hareket (snap)", &snap_);
    ImGui::SetItemTooltip("Kapalıyken Ctrl basılı tutularak da kullanılabilir.");
    if (snap_ || ImGui::GetIO().KeyCtrl) {
        ImGui::DragFloat("Öteleme adımı", &snapTranslate_, 0.5f, 0.1f, 1000.0f, "%.1f mm");
        ImGui::DragFloat("Açı adımı", &snapRotate_, 0.5f, 1.0f, 90.0f, "%.0f°");
        ImGui::DragFloat("Ölçek adımı", &snapScale_, 0.01f, 0.01f, 1.0f, "%.2f");
    }

    ImGui::SeparatorText("Görünüm");
    ImGui::Checkbox("Izgara", &showGrid_);
    ImGui::SameLine();
    ImGui::Checkbox("Kenarlar", &showEdges_);
    ImGui::SliderFloat("Keskin kenar açısı", &featureAngle_, 5.0f, 90.0f, "%.0f°");
    if (ImGui::IsItemDeactivatedAfterEdit()) {
        std::set<Mesh*> done;
        for (auto& o : scene_.objects())
            if (done.insert(o->mesh.get()).second) o->mesh->setFeatureAngle(featureAngle_);
        if (selKind_ == Tool::Edge || (selKind_ == Tool::Face && selFaceMode_ == FaceMode::Smooth)) clearSelection();
        lastHoverVersion_ = 0;
    }
    ImGui::SetItemTooltip("Bu açıdan keskin kenarlar 'keskin kenar' sayılır ve yumuşak yüzeyleri ayırır.");

    ImGui::SeparatorText("İçe aktarma");
    ImGui::Checkbox("Otomatik yerleştir", &autoPlace_);
    ImGui::SetItemTooltip("Açık: parça pivotu tabanın ortasına alınır ve sahnedeki\nnesnelerin yanına, zemine oturtulur.\nKapalı: STL koordinatları aynen korunur.");
    if (ImGui::Button("STL Aç...")) fileBrowser_.open();
    ImGui::SameLine();
    if (ImGui::Button("Kutu")) addPrimitive(0);
    ImGui::SameLine();
    if (ImGui::Button("Silindir")) addPrimitive(1);
    ImGui::SameLine();
    if (ImGui::Button("Küre")) addPrimitive(2);
    ImGui::PopItemWidth();
    ImGui::End();
}

void Editor::drawScenePanel() {
    if (!ImGui::Begin("Sahne")) {
        ImGui::End();
        return;
    }
    if (scene_.objects().empty()) {
        ImGui::TextWrapped("Sahne boş. STL dosyasını pencereye sürükleyip bırakın ya da Dosya > STL Aç kullanın.");
    }
    uint32_t toDelete = 0, toDuplicate = 0;
    bool robotOpen = false;
    if (robot_) {
        ImGui::SetNextItemOpen(false, ImGuiCond_Once);
        robotOpen = ImGui::TreeNodeEx("##robotnode", ImGuiTreeNodeFlags_SpanAvailWidth, "Robot: %s (%zu parça)",
                                      robot_->name.c_str(), robot_->parts.size());
        if (ImGui::IsItemClicked() && !robot_->parts.empty()) {
            activeObject_ = robot_->parts[robot_->root()].objectId;
            focusRobotPanel_ = true;
        }
    }
    for (auto& up : scene_.objects()) {
        SceneObject& o = *up;
        if (o.robotPart >= 0) {
            if (!robotOpen) continue;
            ImGui::PushID(int(o.id));
            const bool colliding = collision_.isColliding(o.id);
            if (colliding) ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(1.0f, 0.45f, 0.4f, 1.0f));
            char label[300];
            std::snprintf(label, sizeof(label), "%s%s", o.name.c_str(), colliding ? "  (çarpışıyor)" : "");
            if (ImGui::Selectable(label, activeObject_ == o.id)) {
                activeObject_ = o.id;
                selectedJoint_ = robot_->jointOfPart(o.robotPart);
            }
            if (colliding) ImGui::PopStyleColor();
            ImGui::PopID();
            continue;
        }
        ImGui::PushID(int(o.id));
        if (ImGui::Checkbox("##vis", &o.visible)) scene_.markDirty();
        ImGui::SetItemTooltip("Görünür");
        ImGui::SameLine();
        ImGui::ColorButton("##col", ImVec4(o.color.r, o.color.g, o.color.b, 1.0f), ImGuiColorEditFlags_NoTooltip,
                           ImVec2(ImGui::GetFrameHeight(), ImGui::GetFrameHeight()));
        ImGui::SameLine();
        const bool colliding = collision_.isColliding(o.id);
        if (colliding) ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(1.0f, 0.45f, 0.4f, 1.0f));
        char label[300];
        std::snprintf(label, sizeof(label), "%s%s", o.name.c_str(), colliding ? "  (çarpışıyor)" : "");
        if (ImGui::Selectable(label, activeObject_ == o.id)) activeObject_ = o.id;
        if (colliding) ImGui::PopStyleColor();
        if (ImGui::BeginPopupContextItem()) {
            activeObject_ = o.id;
            if (ImGui::MenuItem("Odaklan")) camera_.frame(o.worldBounds());
            if (ImGui::MenuItem("Çoğalt")) toDuplicate = o.id;
            if (ImGui::MenuItem("Zemine indir")) dropToGround(o);
            if (ImGui::MenuItem("Sil")) toDelete = o.id;
            ImGui::EndPopup();
        }
        ImGui::PopID();
    }
    if (robotOpen) ImGui::TreePop();
    if (toDuplicate) {
        activeObject_ = toDuplicate;
        duplicateActive();
    }
    if (toDelete) deleteObject(toDelete);
    ImGui::End();
}

void Editor::drawPropertiesPanel() {
    if (!ImGui::Begin("Özellikler")) {
        ImGui::End();
        return;
    }
    SceneObject* o = scene_.find(activeObject_);
    if (!o) {
        ImGui::TextDisabled("Seçili nesne yok.");
        ImGui::End();
        return;
    }
    if (o->robotPart >= 0 && robot_) {
        drawRobotPartProperties(*o);
        ImGui::End();
        return;
    }
    ImGui::PushItemWidth(-ImGui::GetFontSize() * 6.5f);
    char name[256];
    std::snprintf(name, sizeof(name), "%s", o->name.c_str());
    if (ImGui::InputText("Ad", name, sizeof(name))) o->name = name;
    if (!o->sourcePath.empty()) ImGui::TextDisabled("%s", o->sourcePath.c_str());
    ImGui::ColorEdit3("Renk", glm::value_ptr(o->color), ImGuiColorEditFlags_NoInputs);
    ImGui::SameLine();
    if (ImGui::Checkbox("Görünür", &o->visible)) scene_.markDirty();
    if (ImGui::Checkbox("Çarpışma kontrolüne dahil", &o->collidable)) scene_.markDirty();

    ImGui::SeparatorText("Dönüşüm");
    Transform t = o->transform;
    bool changed = false;
    changed |= ImGui::DragFloat3("Konum (mm)", glm::value_ptr(t.position), 0.5f, 0, 0, "%.2f");
    changed |= ImGui::DragFloat3("Dönüş (°)", glm::value_ptr(t.rotationDeg), 0.5f, 0, 0, "%.1f");
    changed |= ImGui::DragFloat3("Ölçek", glm::value_ptr(t.scale), 0.005f, 0.001f, 1000.0f, "%.3f");
    float uniform = t.scale.x;
    if (ImGui::DragFloat("Tekdüze ölçek", &uniform, 0.005f, 0.001f, 1000.0f, "%.3f")) {
        t.scale = glm::vec3(uniform);
        changed = true;
    }
    if (changed) applyTransform(*o, t.matrix());
    if (ImGui::Button("Zemine indir")) dropToGround(*o);
    ImGui::SameLine();
    if (ImGui::Button("Dönüşü sıfırla")) {
        Transform r = o->transform;
        r.rotationDeg = glm::vec3(0.0f);
        r.scale = glm::vec3(1.0f);
        applyTransform(*o, r.matrix());
    }
    ImGui::SameLine();
    if (ImGui::Button("Orijine taşı")) {
        Transform r = o->transform;
        r.position = glm::vec3(0.0f);
        applyTransform(*o, r.matrix());
    }

    ImGui::SeparatorText("Geometri");
    const Mesh& m = *o->mesh;
    const AABB wb = o->worldBounds();
    ImGui::Text("Üçgen: %zu   Nokta: %zu   Kenar: %zu", m.triangles.size(), m.positions.size(), m.edges.size());
    ImGui::Text("Keskin kenar: %zu   Düzlemsel yüzey: %u", m.featureEdges.size(), m.planarRegions.count());
    ImGui::Text("Boyut (dünya): %.2f x %.2f x %.2f mm", wb.size().x, wb.size().y, wb.size().z);
    ImGui::Text("Yüzey alanı: %.1f mm²", m.totalArea);
    size_t open = 0, nonManifold = 0;
    for (const auto& e : m.edges) {
        if (e.faceCount == 1) open++;
        else if (e.faceCount > 2) nonManifold++;
    }
    if (open || nonManifold)
        ImGui::TextColored(ImVec4(1.0f, 0.75f, 0.3f, 1.0f), "Uyarı: %zu açık, %zu manifold olmayan kenar\n(içi dolu çarpışma testi güvenilir olmayabilir)",
                           open, nonManifold);
    else ImGui::TextColored(ImVec4(0.5f, 0.85f, 0.5f, 1.0f), "Kapalı (su geçirmez) ağ");

    ImGui::Spacing();
    if (ImGui::Button("Çoğalt")) duplicateActive();
    ImGui::SameLine();
    if (ImGui::Button("Sil")) deleteObject(o->id);
    ImGui::PopItemWidth();
    ImGui::End();
}

void Editor::drawSelectionPanel() {
    if (!ImGui::Begin("Seçim")) {
        ImGui::End();
        return;
    }
    if (selection_.empty()) {
        ImGui::TextDisabled("Eleman seçili değil.");
        ImGui::TextWrapped("Nokta (1), Kenar (2) veya Yüzey (3) aracıyla nesne üzerinde seçim yapın. Shift ile çoklu seçim.");
        ImGui::End();
        return;
    }
    const char* kindName = selKind_ == Tool::Vertex ? "nokta" : selKind_ == Tool::Edge ? "kenar" : selFaceMode_ == FaceMode::Triangle ? "üçgen" : "yüzey";
    ImGui::Text("%zu %s seçili", selection_.size(), kindName);
    if (selKind_ == Tool::Face) ImGui::TextDisabled("Yüzey tipi: %s", faceModeName(selFaceMode_));
    ImGui::SameLine(ImGui::GetContentRegionAvail().x - 60);
    if (ImGui::SmallButton("Temizle")) {
        clearSelection();
        ImGui::End();
        return;
    }

    // Per-element data (world space)
    struct FaceInfo {
        float area = 0;
        glm::vec3 normal{0}, centroid{0};
    };
    auto faceInfo = [&](const ElementRef& r) {
        FaceInfo fi;
        const SceneObject* o = scene_.find(r.objectId);
        if (!o) return fi;
        std::vector<uint32_t> tris;
        appendFaceTriangles(*o->mesh, selFaceMode_, r.index, tris);
        const glm::mat4 m = o->matrix();
        for (uint32_t t : tris) {
            const glm::vec3 a = transformPoint(m, o->mesh->triVertex(t, 0));
            const glm::vec3 b = transformPoint(m, o->mesh->triVertex(t, 1));
            const glm::vec3 c = transformPoint(m, o->mesh->triVertex(t, 2));
            const glm::vec3 cr = glm::cross(b - a, c - a);
            const float ar = 0.5f * glm::length(cr);
            fi.area += ar;
            fi.normal += cr * 0.5f;
            fi.centroid += (a + b + c) / 3.0f * ar;
        }
        if (fi.area > 0) fi.centroid /= fi.area;
        const float nl = glm::length(fi.normal);
        if (nl > 0) fi.normal /= nl;
        return fi;
    };

    float totalLen = 0.0f, totalArea = 0.0f;
    if (ImGui::BeginChild("##sellist", ImVec2(0, ImGui::GetContentRegionAvail().y * 0.6f), ImGuiChildFlags_Borders)) {
        int shown = 0;
        for (const auto& r : selection_) {
            const SceneObject* o = scene_.find(r.objectId);
            if (!o) continue;
            if (selKind_ == Tool::Vertex) {
                const glm::vec3 p = vertexWorld(r);
                if (shown++ < 200) ImGui::Text("%s  v%u:  (%.2f, %.2f, %.2f)", o->name.c_str(), r.index, p.x, p.y, p.z);
            } else if (selKind_ == Tool::Edge) {
                glm::vec3 a, b;
                edgeWorld(r, a, b);
                const float len = glm::length(b - a);
                totalLen += len;
                if (shown++ < 200)
                    ImGui::Text("%s  e%u:  L = %.3f mm  (%.0f°)", o->name.c_str(), r.index, len,
                                o->mesh->edges[r.index].angleDeg);
            } else {
                const FaceInfo fi = faceInfo(r);
                totalArea += fi.area;
                if (shown++ < 200)
                    ImGui::Text("%s  f%u:  A = %.2f mm²  n = (%.3f, %.3f, %.3f)", o->name.c_str(), r.index, fi.area,
                                fi.normal.x, fi.normal.y, fi.normal.z);
            }
        }
    }
    ImGui::EndChild();

    ImGui::SeparatorText("Ölçüm");
    if (selKind_ == Tool::Vertex) {
        if (selection_.size() >= 2) {
            const glm::vec3 a = vertexWorld(selection_[selection_.size() - 2]);
            const glm::vec3 b = vertexWorld(selection_.back());
            const glm::vec3 d = b - a;
            ImGui::Text("Son iki nokta arası: %.3f mm", glm::length(d));
            ImGui::Text("dX %.3f  dY %.3f  dZ %.3f", d.x, d.y, d.z);
        }
        if (selection_.size() >= 1) {
            glm::vec3 c(0.0f);
            for (const auto& r : selection_) c += vertexWorld(r);
            c /= float(selection_.size());
            ImGui::Text("Ağırlık merkezi: (%.2f, %.2f, %.2f)", c.x, c.y, c.z);
        }
    } else if (selKind_ == Tool::Edge) {
        ImGui::Text("Toplam uzunluk: %.3f mm", totalLen);
        if (selection_.size() >= 2) {
            glm::vec3 a0, a1, b0, b1;
            edgeWorld(selection_[selection_.size() - 2], a0, a1);
            edgeWorld(selection_.back(), b0, b1);
            const float c = std::abs(glm::dot(glm::normalize(a1 - a0), glm::normalize(b1 - b0)));
            ImGui::Text("Son iki kenar arası açı: %.2f°", glm::degrees(std::acos(std::clamp(c, 0.0f, 1.0f))));
        }
    } else {
        ImGui::Text("Toplam alan: %.2f mm²", totalArea);
        if (selection_.size() >= 2) {
            const FaceInfo a = faceInfo(selection_[selection_.size() - 2]);
            const FaceInfo b = faceInfo(selection_.back());
            const float ang = glm::degrees(std::acos(std::clamp(glm::dot(a.normal, b.normal), -1.0f, 1.0f)));
            ImGui::Text("Son iki yüzey normali arası açı: %.2f°", ang);
            if (std::abs(glm::dot(a.normal, b.normal)) > 0.9999f)
                ImGui::Text("Paralel düzlemler arası mesafe: %.3f mm", std::abs(glm::dot(b.centroid - a.centroid, a.normal)));
        }
    }
    ImGui::End();
}

void Editor::drawCollisionPanel() {
    if (!ImGui::Begin("Çarpışmalar")) {
        ImGui::End();
        return;
    }
    if (ImGui::Checkbox("Çarpışma kontrolü", &collision_.enabled)) collision_.update(scene_, true);
    ImGui::Checkbox("Çarpışmada hareketi engelle", &blockOnCollision_);
    ImGui::SetItemTooltip("Açıkken, çarpışmayan bir nesne başka bir nesnenin içine taşınamaz;\nsürükleme temas noktasında durur.");
    ImGui::Checkbox("Temas üçgenlerini göster", &showContacts_);
    if (collision_.enabled) {
        ImGui::TextDisabled("Son kontrol: %.2f ms", collision_.lastUpdateMs());
        ImGui::Separator();
        if (collision_.collisions().empty()) ImGui::TextColored(ImVec4(0.5f, 0.9f, 0.5f, 1.0f), "Çarpışma yok.");
        for (size_t i = 0; i < collision_.collisions().size(); ++i) {
            const CollisionInfo& c = collision_.collisions()[i];
            const SceneObject* a = scene_.find(c.a);
            const SceneObject* b = scene_.find(c.b);
            if (!a || !b) continue;
            char label[600];
            if (c.containment) std::snprintf(label, sizeof(label), "%s  <->  %s : biri diğerinin içinde##c%zu", a->name.c_str(), b->name.c_str(), i);
            else
                std::snprintf(label, sizeof(label), "%s  <->  %s : %zu%s üçgen çifti##c%zu", a->name.c_str(), b->name.c_str(),
                              c.triPairs.size(), c.truncated ? "+" : "", i);
            ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(1.0f, 0.5f, 0.45f, 1.0f));
            if (ImGui::Selectable(label, activeObject_ == c.a)) activeObject_ = c.a;
            ImGui::PopStyleColor();
        }
    }
    ImGui::End();
}

// =============================================================================================
// Viewport: gizmo, input, hover, overlay
// =============================================================================================

void Editor::drawViewport() {
    ImGui::SetNextWindowPos(vpPos_);
    ImGui::SetNextWindowSize(vpSize_);
    ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2(0, 0));
    ImGui::PushStyleVar(ImGuiStyleVar_WindowBorderSize, 0.0f);
    const ImGuiWindowFlags flags = ImGuiWindowFlags_NoDecoration | ImGuiWindowFlags_NoInputs | ImGuiWindowFlags_NoBackground |
                                   ImGuiWindowFlags_NoSavedSettings | ImGuiWindowFlags_NoFocusOnAppearing |
                                   ImGuiWindowFlags_NoBringToFrontOnFocus | ImGuiWindowFlags_NoNav | ImGuiWindowFlags_NoDocking |
                                   ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoResize;
    ImGui::Begin("##Viewport3D", nullptr, flags);
    ImGui::PopStyleVar(2);

    camera_.aspect = vpSize_.x / vpSize_.y;
    glm::mat4 view = camera_.view();
    const glm::mat4 proj = camera_.projection();

    ImGuizmo::SetDrawlist();
    ImGuizmo::SetOrthographic(false);
    ImGuizmo::SetRect(vpPos_.x, vpPos_.y, vpSize_.x, vpSize_.y);

    SceneObject* active = scene_.find(activeObject_);
    if (gizmoEnabled_ && tool_ == Tool::Object && active && active->visible && !dragging_) {
        glm::mat4 m = active->matrix();
        const bool snapping = snap_ || ImGui::GetIO().KeyCtrl;
        float snapVals[3];
        const float s = gizmoOp_ == ImGuizmo::TRANSLATE ? snapTranslate_ : gizmoOp_ == ImGuizmo::ROTATE ? snapRotate_ : snapScale_;
        snapVals[0] = snapVals[1] = snapVals[2] = s;
        if (ImGuizmo::Manipulate(glm::value_ptr(view), glm::value_ptr(proj), gizmoOp_, gizmoMode_, glm::value_ptr(m), nullptr,
                                 snapping ? snapVals : nullptr))
            applyTransform(*active, m);
    }

    // View cube (top-right corner) – rotates the orbit camera.
    const float cube = 110.0f;
    glm::mat4 viewCopy = view;
    ImGuizmo::ViewManipulate(glm::value_ptr(viewCopy), camera_.distance, ImVec2(vpPos_.x + vpSize_.x - cube - 8, vpPos_.y + 8),
                             ImVec2(cube, cube), 0x10101010);
    if (viewCopy != view) {
        const glm::mat4 inv = glm::inverse(viewCopy);
        const glm::vec3 eye = glm::vec3(inv[3]);
        const glm::vec3 fwd = -glm::vec3(inv[2]);
        camera_.target = eye + fwd * camera_.distance;
        const glm::vec3 dir = glm::normalize(eye - camera_.target);
        camera_.pitchDeg = std::clamp(glm::degrees(std::asin(std::clamp(dir.z, -1.0f, 1.0f))), -89.9f, 89.9f);
        if (std::abs(dir.x) + std::abs(dir.y) > 1e-4f) camera_.yawDeg = glm::degrees(std::atan2(dir.y, dir.x));
    }

    handleViewportInput();
    drawOverlay(ImGui::GetWindowDrawList());
    if (brushMode_ && tool_ == Tool::Face && vpHovered_) {
        const ImVec2 mp = ImGui::GetIO().MousePos;
        const ImU32 col = brushActive_ && brushErase_ ? IM_COL32(255, 120, 90, 230) : IM_COL32(255, 255, 255, 220);
        ImGui::GetWindowDrawList()->AddCircle(mp, brushRadius_, col, 64, 1.5f);
        ImGui::GetWindowDrawList()->AddCircleFilled(mp, 2.0f, col);
    }
    ImGui::End();
}

bool Editor::selectionContains(const ElementRef& r) const {
    return std::find(selection_.begin(), selection_.end(), r) != selection_.end();
}

glm::vec3 Editor::vertexWorld(const ElementRef& r) const {
    const SceneObject* o = scene_.find(r.objectId);
    if (!o || r.index >= o->mesh->positions.size()) return glm::vec3(0.0f);
    return transformPoint(o->matrix(), o->mesh->positions[r.index]);
}

void Editor::edgeWorld(const ElementRef& r, glm::vec3& a, glm::vec3& b) const {
    a = b = glm::vec3(0.0f);
    const SceneObject* o = scene_.find(r.objectId);
    if (!o || r.index >= o->mesh->edges.size()) return;
    const glm::mat4 m = o->matrix();
    a = transformPoint(m, o->mesh->positions[o->mesh->edges[r.index].v[0]]);
    b = transformPoint(m, o->mesh->positions[o->mesh->edges[r.index].v[1]]);
}

void Editor::handleViewportInput() {
    ImGuiIO& io = ImGui::GetIO();
    const glm::vec2 mouse(io.MousePos.x - vpPos_.x, io.MousePos.y - vpPos_.y);
    const glm::vec2 vpSize(vpSize_.x, vpSize_.y);
    const bool inside = mouse.x >= 0 && mouse.y >= 0 && mouse.x < vpSize.x && mouse.y < vpSize.y;
    vpHovered_ = inside && !io.WantCaptureMouse;
    const bool gizmoBusy = ImGuizmo::IsUsingAny() || ImGuizmo::IsOver() || ImGuizmo::IsViewManipulateHovered();

    // ---- camera
    const bool brushOn = brushMode_ && tool_ == Tool::Face;
    if (vpHovered_ && io.MouseWheel != 0.0f) {
        if (brushOn) brushRadius_ = std::clamp(brushRadius_ * (io.MouseWheel > 0 ? 1.15f : 1.0f / 1.15f), 4.0f, 200.0f);
        else camera_.zoom(io.MouseWheel);
    }
    // Alt + click in the Face tool: face loop (on empty space it still orbits the camera)
    bool loopClick = false;
    if (vpHovered_ && !gizmoBusy && tool_ == Tool::Face && io.KeyAlt && ImGui::IsMouseClicked(ImGuiMouseButton_Left)) {
        PickContext lc;
        lc.scene = &scene_;
        lc.camera = &camera_;
        lc.viewportSize = vpSize;
        lc.mouse = mouse;
        const SceneHit h = pickSurface(lc);
        const SceneObject* ho = h.hit() ? scene_.find(h.objectId) : nullptr;
        if (ho && ho->robotPart < 0) loopClick = loopSelect(h, io.KeyShift);
    }
    if (vpHovered_ && !gizmoBusy && !loopClick &&
        (ImGui::IsMouseClicked(ImGuiMouseButton_Right) || ImGui::IsMouseClicked(ImGuiMouseButton_Middle) ||
         (io.KeyAlt && ImGui::IsMouseClicked(ImGuiMouseButton_Left))))
        cameraDrag_ = true;
    if (cameraDrag_) {
        const bool down = ImGui::IsMouseDown(ImGuiMouseButton_Right) || ImGui::IsMouseDown(ImGuiMouseButton_Middle) ||
                          ImGui::IsMouseDown(ImGuiMouseButton_Left);
        if (!down) cameraDrag_ = false;
        else if (io.MouseDelta.x != 0.0f || io.MouseDelta.y != 0.0f) {
            if (io.KeyShift) camera_.pan(io.MouseDelta.x, io.MouseDelta.y, vpSize.y);
            else camera_.orbit(io.MouseDelta.x, io.MouseDelta.y);
        }
    }

    PickContext ctx;
    ctx.scene = &scene_;
    ctx.camera = &camera_;
    ctx.viewportSize = vpSize;
    ctx.mouse = mouse;
    ctx.radiusPx = pickRadius_;

    // ---- brush (circle) selection
    if (brushActive_) updateBrush(mouse);
    if (brushOn && !cameraDrag_ && vpHovered_ && !gizmoBusy && !io.KeyAlt && ImGui::IsMouseClicked(ImGuiMouseButton_Left)) {
        beginBrush(io.KeyShift || io.KeyCtrl);
        updateBrush(mouse);
    }

    // ---- left click: selection / start drag
    if (!brushOn && !cameraDrag_ && vpHovered_ && !gizmoBusy && !io.KeyAlt && ImGui::IsMouseClicked(ImGuiMouseButton_Left)) {
        pressPos_ = io.MousePos;
        if (tool_ == Tool::Object) {
            const SceneHit hit = pickSurface(ctx);
            if (hit.hit()) {
                activeObject_ = hit.objectId;
                SceneObject* o = scene_.find(hit.objectId);
                dragCandidate_ = true;
                dragging_ = false;
                dragObject_ = hit.objectId;
                grabPoint_ = hit.point;
                dragStartPos_ = o->transform.position;
                dragStartMatrix_ = o->matrix();
                if (robot_ && o->robotPart >= 0) {
                    selectedJoint_ = robot_->jointOfPart(o->robotPart);
                    dragStartMatrix_ = o->matrix();
                }
                dragVertical_ = io.KeyShift;
                if (dragVertical_) {
                    glm::vec3 f = camera_.forward();
                    f.z = 0.0f;
                    dragPlaneN_ = glm::length(f) > 1e-4f ? glm::normalize(f) : glm::vec3(1, 0, 0);
                } else {
                    dragPlaneN_ = glm::vec3(0, 0, 1);
                }
            } else {
                activeObject_ = 0;
            }
        } else {
            ElementRef r;
            if (tool_ == Tool::Vertex) r = pickVertex(ctx);
            else if (tool_ == Tool::Edge) r = pickEdge(ctx, edgeSet_);
            else r = pickFace(ctx, faceMode_);

            // Shift + click on a region while the selection is made of triangles (brush, linked ...):
            // add / remove the region's triangles instead of starting over
            if (tool_ == Tool::Face && selKind_ == Tool::Face && selFaceMode_ == FaceMode::Triangle && faceMode_ != FaceMode::Triangle &&
                io.KeyShift && r.valid()) {
                const SceneObject* ro = scene_.find(r.objectId);
                std::vector<uint32_t> ts;
                appendFaceTriangles(*ro->mesh, faceMode_, r.index, ts);
                const SceneHit hh = pickSurface(ctx);
                const bool has = hh.hit() && selectionContains({hh.objectId, hh.tri});
                addTriangles(r.objectId, ts, has);
                r = {};
                goto pickDone;
            }
            {
            const bool kindChanged = selKind_ != tool_ || (tool_ == Tool::Face && selFaceMode_ != faceMode_);
            if (kindChanged) selection_.clear();
            selKind_ = tool_;
            selFaceMode_ = faceMode_;
            if (r.valid()) {
                activeObject_ = r.objectId;
                if (io.KeyShift) {
                    auto it = std::find(selection_.begin(), selection_.end(), r);
                    if (it != selection_.end()) selection_.erase(it);
                    else selection_.push_back(r);
                } else {
                    selection_ = {r};
                }
            } else if (!io.KeyShift) {
                selection_.clear();
            }
            }
        pickDone:;
        }
    }

    // ---- free dragging (object tool)
    if (dragCandidate_) {
        SceneObject* o = scene_.find(dragObject_);
        if (!o || !ImGui::IsMouseDown(ImGuiMouseButton_Left)) {
            dragCandidate_ = dragging_ = false;
        } else {
            if (!dragging_) {
                const float dx = io.MousePos.x - pressPos_.x, dy = io.MousePos.y - pressPos_.y;
                if (dx * dx + dy * dy > 16.0f) dragging_ = true;
            }
            if (dragging_) {
                const Ray ray = camera_.rayFromViewport(mouse, vpSize);
                const float denom = glm::dot(ray.dir, dragPlaneN_);
                if (std::abs(denom) > 1e-5f) {
                    const float t = glm::dot(grabPoint_ - ray.origin, dragPlaneN_) / denom;
                    if (t > 0.0f) {
                        glm::vec3 delta = ray.origin + ray.dir * t - grabPoint_;
                        if (dragVertical_) delta = glm::vec3(0.0f, 0.0f, delta.z);
                        else delta.z = 0.0f;
                        if (snap_ || io.KeyCtrl) delta = glm::round(delta / snapTranslate_) * snapTranslate_;
                        applyTransform(*o, glm::translate(glm::mat4(1.0f), delta) * dragStartMatrix_);
                    }
                }
            }
        }
    }

    // ---- hover
    if (vpHovered_ && !dragging_ && !cameraDrag_ && !ImGuizmo::IsUsingAny()) updateHover();
    else {
        hover_ = {};
        hoverObject_ = 0;
        lastHoverVersion_ = 0;
    }
}

void Editor::updateHover() {
    const ImGuiIO& io = ImGui::GetIO();
    const glm::vec2 mouse(io.MousePos.x - vpPos_.x, io.MousePos.y - vpPos_.y);
    const glm::mat4 view = camera_.view();
    if (mouse == lastHoverMouse_ && scene_.version() == lastHoverVersion_ && view == lastHoverView_ && tool_ == lastHoverTool_)
        return;
    lastHoverMouse_ = mouse;
    lastHoverVersion_ = scene_.version();
    lastHoverView_ = view;
    lastHoverTool_ = tool_;

    PickContext ctx;
    ctx.scene = &scene_;
    ctx.camera = &camera_;
    ctx.viewportSize = glm::vec2(vpSize_.x, vpSize_.y);
    ctx.mouse = mouse;
    ctx.radiusPx = pickRadius_;
    hover_ = {};
    hoverObject_ = 0;
    switch (tool_) {
    case Tool::Object: hoverObject_ = pickSurface(ctx).objectId; break;
    case Tool::Vertex: hover_ = pickVertex(ctx); break;
    case Tool::Edge: hover_ = pickEdge(ctx, edgeSet_); break;
    case Tool::Face: hover_ = pickFace(ctx, faceMode_); break;
    }
}

void Editor::drawOverlay(ImDrawList* dl) {
    const glm::vec2 size(vpSize_.x, vpSize_.y);
    auto toScreen = [&](const glm::vec3& p, ImVec2& out) {
        glm::vec2 s;
        if (!camera_.worldToViewport(p, size, s)) return false;
        out = ImVec2(vpPos_.x + s.x, vpPos_.y + s.y);
        return true;
    };

    // Vertex markers
    if (selKind_ == Tool::Vertex) {
        for (const auto& r : selection_) {
            ImVec2 s;
            if (!toScreen(vertexWorld(r), s)) continue;
            dl->AddCircleFilled(s, 5.5f, toU32(kSelectColor));
            dl->AddCircle(s, 5.5f, IM_COL32(20, 20, 20, 255), 0, 1.5f);
        }
    }
    if (tool_ == Tool::Vertex && hover_.valid()) {
        ImVec2 s;
        if (toScreen(vertexWorld(hover_), s)) {
            dl->AddCircle(s, 8.0f, toU32(kHoverColor), 0, 2.0f);
            const glm::vec3 p = vertexWorld(hover_);
            char buf[96];
            std::snprintf(buf, sizeof(buf), "(%.2f, %.2f, %.2f)", p.x, p.y, p.z);
            dl->AddText(ImVec2(s.x + 10, s.y - 18), IM_COL32(220, 240, 255, 255), buf);
        }
    }
    if (tool_ == Tool::Edge && hover_.valid()) {
        glm::vec3 a, b;
        edgeWorld(hover_, a, b);
        ImVec2 s;
        if (toScreen((a + b) * 0.5f, s)) {
            char buf[64];
            std::snprintf(buf, sizeof(buf), "%.2f mm", glm::length(b - a));
            dl->AddText(ImVec2(s.x + 10, s.y - 18), IM_COL32(220, 240, 255, 255), buf);
        }
    }

    drawRobotOverlay(dl);
    drawMotionOverlay(dl);

    // Info text
    char info[128];
    std::snprintf(info, sizeof(info), "Araç: %s%s%s", toolName(tool_), tool_ == Tool::Face ? " - " : "",
                  tool_ == Tool::Face ? faceModeName(faceMode_) : "");
    dl->AddText(ImVec2(vpPos_.x + 10, vpPos_.y + 8), IM_COL32(230, 230, 230, 200), info);
    if (blockedFlash_ > 0.0f) {
        const char* msg = "Çarpışma: hareket engellendi";
        const ImVec2 ts = ImGui::CalcTextSize(msg);
        const ImVec2 p(vpPos_.x + (vpSize_.x - ts.x) * 0.5f, vpPos_.y + 12);
        dl->AddRectFilled(ImVec2(p.x - 8, p.y - 4), ImVec2(p.x + ts.x + 8, p.y + ts.y + 4), IM_COL32(150, 30, 25, 220), 4.0f);
        dl->AddText(p, IM_COL32(255, 255, 255, 255), msg);
    }
}

// =============================================================================================
// Render data
// =============================================================================================

void Editor::buildFrameScene(FrameScene& fs, const glm::vec2& fbScale) {
    fs.view = camera_.view();
    fs.proj = camera_.vulkanProjParams();
    fs.viewport[0] = vpPos_.x * fbScale.x;
    fs.viewport[1] = vpPos_.y * fbScale.y;
    fs.viewport[2] = vpSize_.x * fbScale.x;
    fs.viewport[3] = vpSize_.y * fbScale.y;

    // ---- objects
    for (const auto& up : scene_.objects()) {
        const SceneObject& o = *up;
        if (!o.visible) continue;
        const glm::mat4 m = o.matrix();
        glm::vec4 color = o.color;
        if (collision_.isColliding(o.id)) color = glm::mix(color, kCollisionColor, 0.55f);
        if (tool_ == Tool::Object && hoverObject_ == o.id && activeObject_ != o.id) color = glm::mix(color, glm::vec4(1.0f), 0.18f);
        if (o.id == activeObject_) color = glm::mix(color, kSelectColor, 0.12f);
        color.a = 1.0f;
        fs.meshes.push_back({o.mesh, m, color});

        const bool isActive = o.id == activeObject_;
        const bool wire = tool_ == Tool::Vertex || (tool_ == Tool::Edge && edgeSet_ == EdgeSet::All);
        if (wire) fs.edges.push_back({o.mesh, m, glm::vec4(0.05f, 0.05f, 0.07f, 0.35f), true, 1.0f});
        if (isActive) fs.edges.push_back({o.mesh, m, kSelectColor, false, 1.5f});
        else if (showEdges_ || tool_ == Tool::Edge) fs.edges.push_back({o.mesh, m, glm::vec4(0.06f, 0.06f, 0.08f, 0.85f), false, 1.0f});
    }

    // ---- grid
    if (showGrid_) {
        const float minor = std::pow(10.0f, std::floor(std::log10(std::max(camera_.distance * 0.04f, 0.01f))));
        const float major = minor * 10.0f;
        const float extent = major * 10.0f;
        const glm::vec2 c = glm::round(glm::vec2(camera_.target) / major) * major;
        LineBatch minorL, majorL, axisX, axisY, axisZ;
        minorL.color = glm::vec4(0.55f, 0.57f, 0.62f, 0.16f);
        majorL.color = glm::vec4(0.60f, 0.62f, 0.68f, 0.38f);
        const int n = int(std::lround(extent / minor));
        const long long kx0 = std::llround(c.x / minor), ky0 = std::llround(c.y / minor);
        for (int i = -n; i <= n; ++i) {
            const long long kx = kx0 + i, ky = ky0 + i;
            const float x = float(kx) * minor, y = float(ky) * minor;
            if (kx != 0) {
                LineBatch& b = (kx % 10 == 0) ? majorL : minorL;
                b.points.insert(b.points.end(), {{x, c.y - extent, 0}, {x, c.y + extent, 0}});
            }
            if (ky != 0) {
                LineBatch& b = (ky % 10 == 0) ? majorL : minorL;
                b.points.insert(b.points.end(), {{c.x - extent, y, 0}, {c.x + extent, y, 0}});
            }
        }
        axisX.color = glm::vec4(0.90f, 0.30f, 0.30f, 0.85f);
        axisY.color = glm::vec4(0.35f, 0.85f, 0.35f, 0.85f);
        axisZ.color = glm::vec4(0.35f, 0.55f, 1.00f, 0.85f);
        axisX.width = axisY.width = axisZ.width = 1.5f;
        axisX.points = {{c.x - extent, 0, 0}, {c.x + extent, 0, 0}};
        axisY.points = {{0, c.y - extent, 0}, {0, c.y + extent, 0}};
        axisZ.points = {{0, 0, 0}, {0, 0, major}};
        fs.lines.push_back(std::move(minorL));
        fs.lines.push_back(std::move(majorL));
        fs.lines.push_back(std::move(axisX));
        fs.lines.push_back(std::move(axisY));
        fs.lines.push_back(std::move(axisZ));
    }

    // ---- face highlights
    auto addFaceHighlight = [&](const ElementRef& r, FaceMode mode, const glm::vec4& color) {
        const SceneObject* o = scene_.find(r.objectId);
        if (!o || !o->visible) return;
        std::vector<uint32_t> tris;
        appendFaceTriangles(*o->mesh, mode, r.index, tris);
        TriHighlight h{o->mesh, o->matrix(), color, {}};
        h.indices.reserve(tris.size() * 3);
        for (uint32_t t : tris) {
            const glm::uvec3& tri = o->mesh->triangles[t];
            h.indices.insert(h.indices.end(), {tri.x, tri.y, tri.z});
        }
        fs.highlights.push_back(std::move(h));
    };
    if (selKind_ == Tool::Face) {
        // group selected faces by object to keep draw calls low
        std::vector<ElementRef> sorted = selection_;
        std::sort(sorted.begin(), sorted.end(), [](const ElementRef& a, const ElementRef& b) { return a.objectId < b.objectId; });
        for (size_t i = 0; i < sorted.size();) {
            const SceneObject* o = scene_.find(sorted[i].objectId);
            size_t j = i;
            std::vector<uint32_t> tris;
            while (j < sorted.size() && sorted[j].objectId == sorted[i].objectId) {
                if (o) appendFaceTriangles(*o->mesh, selFaceMode_, sorted[j].index, tris);
                ++j;
            }
            if (o && o->visible) {
                TriHighlight h{o->mesh, o->matrix(), glm::vec4(kSelectColor.r, kSelectColor.g, kSelectColor.b, 0.6f), {}};
                for (uint32_t t : tris) {
                    const glm::uvec3& tri = o->mesh->triangles[t];
                    h.indices.insert(h.indices.end(), {tri.x, tri.y, tri.z});
                }
                fs.highlights.push_back(std::move(h));
            }
            i = j;
        }
    }
    if (tool_ == Tool::Face && hover_.valid() && !(selKind_ == Tool::Face && selFaceMode_ == faceMode_ && selectionContains(hover_)))
        addFaceHighlight(hover_, faceMode_, glm::vec4(kHoverColor.r, kHoverColor.g, kHoverColor.b, 0.45f));

    // ---- collision contact triangles
    if (showContacts_ && collision_.enabled) {
        for (const auto& c : collision_.collisions()) {
            for (int side = 0; side < 2; ++side) {
                const SceneObject* o = scene_.find(side == 0 ? c.a : c.b);
                if (!o) continue;
                std::vector<uint32_t> tris;
                tris.reserve(c.triPairs.size());
                for (const auto& p : c.triPairs) tris.push_back(side == 0 ? p.first : p.second);
                std::sort(tris.begin(), tris.end());
                tris.erase(std::unique(tris.begin(), tris.end()), tris.end());
                TriHighlight h{o->mesh, o->matrix(), glm::vec4(1.0f, 0.12f, 0.08f, 0.9f), {}};
                for (uint32_t t : tris) {
                    const glm::uvec3& tri = o->mesh->triangles[t];
                    h.indices.insert(h.indices.end(), {tri.x, tri.y, tri.z});
                }
                if (!h.indices.empty()) fs.highlights.push_back(std::move(h));
            }
        }
    }

    addRobotRenderData(fs);
    addMotionRenderData(fs);

    // ---- edge highlights (drawn on top)
    if (selKind_ == Tool::Edge && !selection_.empty()) {
        LineBatch sel;
        sel.color = kSelectColor;
        sel.width = 3.0f;
        sel.depthTest = false;
        for (const auto& r : selection_) {
            glm::vec3 a, b;
            edgeWorld(r, a, b);
            sel.points.insert(sel.points.end(), {a, b});
        }
        fs.overlayLines.push_back(std::move(sel));
    }
    if (tool_ == Tool::Edge && hover_.valid()) {
        LineBatch hov;
        hov.color = kHoverColor;
        hov.width = 2.5f;
        hov.depthTest = false;
        glm::vec3 a, b;
        edgeWorld(hover_, a, b);
        hov.points = {a, b};
        fs.overlayLines.push_back(std::move(hov));
    }
}

} // namespace pegasus
