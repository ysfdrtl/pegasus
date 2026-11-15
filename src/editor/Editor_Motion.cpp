// Motion paths: tool heads, seam from selected edges, formula library, IK preview and playback.
#include "editor/Editor.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <filesystem>
#include <glm/gtc/type_ptr.hpp>

namespace pegasus {

namespace fs = std::filesystem;

namespace {
const char* kFormulaLabels[CompiledMotion::Count] = {"x (yan)", "y (ilerleme)", "z (yüzey normali)", "Yan eğim (°)",
                                                       "İlerleme eğimi (°)"};
const glm::vec4 kPathOk{0.25f, 1.0f, 0.45f, 1.0f};
const glm::vec4 kPathBad{1.0f, 0.2f, 0.2f, 1.0f};
const glm::vec4 kPathNoRobot{0.3f, 0.85f, 1.0f, 1.0f};
const glm::vec4 kTraceColor{1.0f, 0.6f, 0.1f, 1.0f};
} // namespace

// =============================================================================================
// State
// =============================================================================================

IkTool Editor::currentTool() const {
    IkTool t;
    if (toolIndex_ >= 0 && toolIndex_ < int(library_.tools.size())) t.tipOffset = library_.tools[toolIndex_].tipOffset;
    return t;
}

void Editor::setMotionMode(int mode) {
    if (mode == motionMode_) return;
    stopRun();
    modeIndex_[motionMode_] = motionIndex_;
    motionMode_ = mode;
    motionIndex_ = -1;
    if (!activeDefs().empty()) selectMotion(std::clamp(modeIndex_[mode], 0, int(activeDefs().size()) - 1));
    planDirty_ = true;
}

void Editor::selectMotion(int index) {
    if (index < 0 || index >= int(activeDefs().size())) return;
    motionIndex_ = index;
    motion_ = activeDefs()[index];
    const std::string* src[CompiledMotion::Count] = {&motion_.fx, &motion_.fy, &motion_.fz, &motion_.fWork, &motion_.fTravel};
    for (int i = 0; i < CompiledMotion::Count; ++i) std::snprintf(formulaBuf_[i], sizeof(formulaBuf_[i]), "%s", src[i]->c_str());
    std::snprintf(motionNameBuf_, sizeof(motionNameBuf_), "%s", motion_.name.c_str());
    motionEdited_ = false;
    recompileMotion();
}

void Editor::recompileMotion() {
    motion_.fx = formulaBuf_[0];
    motion_.fy = formulaBuf_[1];
    motion_.fz = formulaBuf_[2];
    motion_.fWork = formulaBuf_[3];
    motion_.fTravel = formulaBuf_[4];
    motionValid_ = compiled_.compile(motion_, formulaErr_);
    planDirty_ = true;
}

std::vector<SeamEdgeGeom> Editor::seamGeometry() const {
    std::vector<SeamEdgeGeom> out;
    for (const auto& r : seamEdges_) {
        const SceneObject* o = scene_.find(r.objectId);
        if (!o || r.index >= o->mesh->edges.size()) continue;
        const Mesh::Edge& e = o->mesh->edges[r.index];
        const glm::mat4 m = o->matrix();
        const glm::mat3 nm = glm::transpose(glm::inverse(glm::mat3(m)));
        SeamEdgeGeom g;
        g.a = glm::vec3(m * glm::vec4(o->mesh->positions[e.v[0]], 1.0f));
        g.b = glm::vec3(m * glm::vec4(o->mesh->positions[e.v[1]], 1.0f));
        for (int k = 0; k < 2; ++k)
            if (e.tri[k] != kInvalid) g.faceNormals.push_back(glm::normalize(nm * o->mesh->triNormals[e.tri[k]]));
        out.push_back(std::move(g));
    }
    return out;
}

void Editor::updateObstacles() {
    obstacles_.clear();
    for (const auto& o : scene_.objects())
        if (o->visible && o->robotPart < 0 && o->mesh) obstacles_.push_back({o->mesh.get(), o->matrix()});
    planSettings_.obstacles = &obstacles_;
    scanSettings_.obstacles = &obstacles_;
}

std::vector<ScanFace> Editor::scanGeometry() const {
    // one ScanFace per object with all its selected triangles (shared vertices): the planner then
    // treats connected triangles as one continuous surface instead of polygon by polygon
    std::vector<uint32_t> objectOrder;
    std::unordered_map<uint32_t, std::vector<uint32_t>> trisOf;
    for (const auto& r : scanFaces_) {
        const SceneObject* o = scene_.find(r.objectId);
        if (!o) continue;
        auto [it, inserted] = trisOf.try_emplace(r.objectId);
        if (inserted) objectOrder.push_back(r.objectId);
        appendFaceTriangles(*o->mesh, scanFaceMode_, r.index, it->second);
    }
    std::vector<ScanFace> out;
    for (uint32_t id : objectOrder) {
        const SceneObject* o = scene_.find(id);
        std::vector<uint32_t>& tris = trisOf[id];
        std::sort(tris.begin(), tris.end());
        tris.erase(std::unique(tris.begin(), tris.end()), tris.end());
        const glm::mat4 m = o->matrix();
        const bool mirrored = glm::determinant(glm::mat3(m)) < 0.0f;
        ScanFace f;
        std::unordered_map<uint32_t, uint32_t> remap;
        for (uint32_t t : tris) {
            glm::uvec3 idx;
            for (int k = 0; k < 3; ++k) {
                const uint32_t v = o->mesh->triangles[t][k];
                auto [it, inserted] = remap.try_emplace(v, uint32_t(f.verts.size()));
                if (inserted) f.verts.push_back(glm::vec3(m * glm::vec4(o->mesh->positions[v], 1.0f)));
                idx[k] = it->second;
            }
            if (mirrored) std::swap(idx.y, idx.z);
            f.tris.push_back(idx);
        }
        if (!f.tris.empty()) out.push_back(std::move(f));
    }
    return out;
}

bool Editor::scanFromSelection() {
    if (selKind_ != Tool::Face || selection_.empty()) {
        notify("Tarama için Yüzey aracıyla (3) bir veya daha fazla yüz seçin (Shift ile çoklu). Düz yüzler için 'Düzlemsel yüzey' tipi önerilir.", true);
        return false;
    }
    scanFaces_ = selection_;
    scanFaceMode_ = selFaceMode_;
    scanFaceMats_.clear();
    planDirty_ = true;
    char buf[128];
    std::snprintf(buf, sizeof(buf), "Tarama alanı: %zu %s", scanFaces_.size(), scanFaceMode_ == FaceMode::Triangle ? "üçgen" : "yüz");
    notify(buf);
    return true;
}

bool Editor::seamFromSelection() {
    if (selKind_ != Tool::Edge || selection_.empty() || selection_.size() > 2) {
        notify("Yol için Kenar aracıyla (2) 1 veya 2 kenar seçin (iki parça arasındaki dikiş için iki kenar).", true);
        return false;
    }
    seamEdges_ = selection_;
    seamReverse_ = false;
    Seam s;
    std::string err;
    if (!buildSeam(seamGeometry(), seamReverse_, camera_.forward(), s, err)) {
        notify(err, true);
        seamEdges_.clear();
        return false;
    }
    seam_ = s;
    planDirty_ = true;
    char buf[160];
    std::snprintf(buf, sizeof(buf), "Yol oluşturuldu: %.2f mm%s", s.length, s.edgeCount == 2 ? "" : " (tek kenar)");
    notify(buf);
    return true;
}

void Editor::recomputePlan() {
    planDirty_ = false;
    planPoints_.clear();
    planError_.clear();
    planReachable_ = 0;
    planMaxPosErr_ = planMaxDirErr_ = 0.0f;
    scanStats_ = ScanStats{};
    if (motionMode_ == 0 && !seam_.valid) return;
    if (motionMode_ == 1 && scanFaces_.empty()) return;
    if (!motionValid_) {
        planError_ = "Formülde hata var.";
        return;
    }
    if (motionMode_ == 0) {
        if (!samplePath(seam_, motion_, compiled_, planSettings_, planPoints_, planError_)) return;
    } else {
        // closed objects (a whole sphere, a figure) are painted from the robot's side
        glm::vec3 from(0.0f, 0.0f, 250.0f);
        if (robot_) from = glm::vec3(robot_->mount[3]) + glm::vec3(0.0f, 0.0f, 250.0f);
        updateObstacles();
        if (!planSurfaceScan(scanGeometry(), motion_, compiled_, scanSettings_, planPoints_, scanStats_, planError_, &from, &obstacles_)) return;
    }
    if (robot_ && ik_.setup(*robot_)) {
        robot_->solveFK();
        planReachable_ = solvePath(ik_, *robot_, currentTool(), ik_.currentAngles(*robot_), planPoints_, activeSettings(),
                                   selfTestDir_.empty() ? 200.0f : -1.0f);
        for (const auto& p : planPoints_) {
            planMaxPosErr_ = std::max(planMaxPosErr_, p.posErr);
            planMaxDirErr_ = std::max(planMaxDirErr_, p.dirErr);
        }
        planMount_ = robot_->mount;
    }
}

void Editor::startRun() {
    if (!robot_) {
        notify("Önce robotu yükleyin.", true);
        return;
    }
    if (planPoints_.empty()) {
        const bool haveInput = motionMode_ == 0 ? seam_.valid : !scanFaces_.empty();
        notify(haveInput ? "Yol oluşturulamadı: " + planError_
                         : (motionMode_ == 0 ? "Önce seçili kenarlardan yol oluşturun." : "Önce seçili yüzeylerden tarama alanı oluşturun."),
               true);
        return;
    }
    if (!ik_.setup(*robot_)) {
        notify("Robotta TCP'ye giden eklem zinciri yok.", true);
        return;
    }
    std::string err;
    updateObstacles();
    if (!buildTrajectory(ik_, *robot_, currentTool(), ik_.currentAngles(*robot_), planPoints_, activeSettings(), traj_, err)) {
        notify(err, true);
        return;
    }
    runWarning_ = err;
    playing_ = true;
    paused_ = false;
    playTime_ = 0.0f;
    trace_.clear();
    char buf[200];
    std::snprintf(buf, sizeof(buf), "Çalışıyor: %s, %.1f s%s", motion_.name.c_str(), traj_.duration, err.empty() ? "" : "  (uyarı: erişilemeyen noktalar var)");
    notify(buf, !err.empty());
}

void Editor::stopRun() {
    playing_ = false;
    paused_ = false;
}

void Editor::updateMotion(float dt) {
    if (!libraryLoaded_) {
        libraryLoaded_ = true;
        library_.load(MotionLibrary::findLibraryDir());
        for (const auto& w : library_.warnings) std::fprintf(stderr, "[kutuphane] %s\n", w.c_str());
        if (!library_.motions.empty()) selectMotion(0);
    }

    // The seam follows the parts: rebuild it from the referenced edges every frame.
    if (!seamEdges_.empty()) {
        seamEdges_.erase(std::remove_if(seamEdges_.begin(), seamEdges_.end(), [&](const ElementRef& r) { return !scene_.find(r.objectId); }),
                         seamEdges_.end());
        Seam s;
        seamError_.clear();
        if (seamEdges_.empty()) s = Seam{};
        else if (!buildSeam(seamGeometry(), seamReverse_, camera_.forward(), s, seamError_)) s = Seam{};
        if (!(s == seam_)) {
            seam_ = s;
            planDirty_ = true;
        }
    } else if (seam_.valid) {
        seam_ = Seam{};
        planDirty_ = true;
    }
    // scan faces follow their objects too
    if (!scanFaces_.empty()) {
        scanFaces_.erase(std::remove_if(scanFaces_.begin(), scanFaces_.end(), [&](const ElementRef& r) { return !scene_.find(r.objectId); }),
                         scanFaces_.end());
        std::vector<glm::mat4> mats;
        for (const auto& r : scanFaces_) mats.push_back(scene_.find(r.objectId)->matrix());
        if (mats != scanFaceMats_) {
            scanFaceMats_ = mats;
            if (motionMode_ == 1) planDirty_ = true;
        }
    }
    if (robot_ && robot_->mount != planMount_ && !playing_) planDirty_ = true;
    if (planDirty_ && !playing_) recomputePlan();

    // playback
    if (playing_ && !paused_ && robot_ && traj_.valid()) {
        playTime_ = std::min(playTime_ + dt * playSpeed_, traj_.duration);
        int idx = 0;
        const std::vector<float> q = traj_.sample(playTime_, &idx);
        ik_.applyAngles(*robot_, q);
        syncRobot();
        const glm::vec3 tip = glm::vec3(ik_.toolFrame(*robot_, q, currentTool())[3]);
        if (trace_.empty() || glm::length(trace_.back() - tip) > 0.05f) trace_.push_back(tip);
        if (stopOnCollision_ && robotColliding()) {
            paused_ = true;
            notify("Çarpışma algılandı: hareket duraklatıldı.", true);
        } else if (playTime_ >= traj_.duration) {
            playing_ = false;
            notify("Hareket tamamlandı.");
        }
    }
}

// =============================================================================================
// Panel
// =============================================================================================

void Editor::drawMotionPanel() {
    if (focusMotionPanel_) {
        ImGui::SetNextWindowFocus();
        focusMotionPanel_ = false;
    }
    if (!ImGui::Begin("Hareket")) {
        ImGui::End();
        return;
    }
    ImGui::PushItemWidth(-ImGui::GetFontSize() * 7.5f);

    // ---------------- tool head
    ImGui::SeparatorText("Başlık (takım)");
    if (!library_.tools.empty()) {
        toolIndex_ = std::clamp(toolIndex_, 0, int(library_.tools.size()) - 1);
        if (ImGui::BeginCombo("Başlık", library_.tools[toolIndex_].name.c_str())) {
            for (int i = 0; i < int(library_.tools.size()); ++i)
                if (ImGui::Selectable(library_.tools[i].name.c_str(), i == toolIndex_)) {
                    toolIndex_ = i;
                    planDirty_ = true;
                }
            ImGui::EndCombo();
        }
        const ToolDef& t = library_.tools[toolIndex_];
        if (!t.description.empty()) ImGui::TextWrapped("%s", t.description.c_str());
        ImGui::TextDisabled("Uç ofseti (TCP'ye göre): %.0f, %.0f, %.0f mm", t.tipOffset.x, t.tipOffset.y, t.tipOffset.z);
    }

    // ---------------- mode
    ImGui::SeparatorText("İş tipi");
    if (ImGui::RadioButton("Kenar yolu (kaynak, çizim)", motionMode_ == 0)) setMotionMode(0);
    if (ImGui::RadioButton("Yüzey tarama (sprey boya)", motionMode_ == 1)) setMotionMode(1);

    if (motionMode_ == 1) {
        ImGui::SeparatorText("Tarama alanı (yüzeyler)");
        if (ImGui::Button("Seçili yüzeylerden tarama alanı oluştur")) scanFromSelection();
        ImGui::SetItemTooltip("Yüzey aracıyla (3) bir veya daha fazla yüz seçin (Shift ile çoklu).\n"
                              "Düz yüzler için 'Düzlemsel yüzey', eğri yüzler için 'Yumuşak yüzey' tipini kullanın.\n"
                              "Her yüz kendi düzleminde, sırayla taranır.");
        if (!scanFaces_.empty()) {
            ImGui::Text("%d üçgen -> %d yüzey parçası, boyanan alan %.0f mm²", scanStats_.inputTriangles, scanStats_.faces, scanStats_.area);
            ImGui::SetItemTooltip("Bağlı üçgenler tek bir sürekli yüzey olarak taranır; yalnızca 'Kırılma açısı'ndan\nkeskin kenarlar yüzeyi ayrı parçalara böler.");
            if (scanStats_.passes > 0)
                ImGui::Text("%d tarama çizgisi, toplam %.0f mm", scanStats_.passes, scanStats_.processLength);
            for (const auto& w : scanStats_.warnings) ImGui::TextColored(ImVec4(1, 0.7f, 0.3f, 1), "%s", w.c_str());
            if (ImGui::SmallButton("Tarama alanını temizle")) {
                scanFaces_.clear();
                stopRun();
                planDirty_ = true;
            }
        } else {
            ImGui::TextDisabled("Tarama alanı yok. Yüzey aracıyla yüz(ler) seçip butona basın.");
        }
    }

    // ---------------- seam
    if (motionMode_ == 0) {
    ImGui::SeparatorText("Yol (dikiş)");
    if (ImGui::Button("Seçili kenarlardan yol oluştur")) seamFromSelection();
    ImGui::SetItemTooltip("Kenar aracıyla (2) seçin:\n  2 kenar: iki parçanın arasındaki dikiş; yol tam ortadan geçer\n  1 kenar: yol kenar boyunca ilerler");
    if (seam_.valid) {
        ImGui::Text("Uzunluk: %.2f mm%s", seam_.length, seam_.edgeCount == 2 ? "" : "   (tek kenar)");
        if (seam_.edgeCount == 2) ImGui::Text("Kenarlar arası boşluk: %.2f mm", seam_.gap);
        ImGui::Text("Başlangıç: (%.1f, %.1f, %.1f)", seam_.start.x, seam_.start.y, seam_.start.z);
        ImGui::TextDisabled("Normal (z): (%.2f, %.2f, %.2f)", seam_.N.x, seam_.N.y, seam_.N.z);
        if (ImGui::SmallButton("Yönü ters çevir")) {
            seamReverse_ = !seamReverse_;
            planDirty_ = true;
        }
        ImGui::SameLine();
        if (ImGui::SmallButton("Yolu temizle")) {
            seamEdges_.clear();
            stopRun();
        }
    } else if (!seamEdges_.empty()) {
        ImGui::TextColored(ImVec4(1, 0.5f, 0.4f, 1), "%s", seamError_.c_str());
    } else {
        ImGui::TextDisabled("Yol yok. Kenar aracıyla kenar(lar) seçip butona basın.");
    }
    } // edge path mode

    // ---------------- motion formula
    ImGui::SeparatorText(motionMode_ == 1 ? "Tarama deseni (kütüphane)" : "Hareket formülü (kütüphane)");
    std::vector<MotionDef>& defs = activeDefs();
    if (!defs.empty()) {
        const char* preview = motionIndex_ >= 0 && motionIndex_ < int(defs.size()) ? defs[motionIndex_].name.c_str() : "-";
        if (ImGui::BeginCombo(motionMode_ == 1 ? "Desen" : "Hareket", preview)) {
            for (int i = 0; i < int(defs.size()); ++i)
                if (ImGui::Selectable(defs[i].name.c_str(), i == motionIndex_)) selectMotion(i);
            ImGui::EndCombo();
        }
        if (motionMode_ == 1)
            ImGui::TextDisabled("Desen: %s, bağlantı: %s, yön: %s", motion_.pattern.c_str(), motion_.link.c_str(), motion_.orient.c_str());
    } else {
        ImGui::TextColored(ImVec4(1, 0.6f, 0.3f, 1), "Kütüphane boş: %s", library_.dir.c_str());
    }
    ImGui::SameLine();
    if (ImGui::SmallButton("Yenile")) {
        const std::string current = motion_.id;
        library_.load(MotionLibrary::findLibraryDir());
        int idx = 0;
        for (int i = 0; i < int(activeDefs().size()); ++i)
            if (activeDefs()[i].id == current) idx = i;
        if (!activeDefs().empty()) selectMotion(idx);
    }
    ImGui::SetItemTooltip("library/ klasöründeki .pmotion ve .ptool dosyalarını yeniden okur.");
    if (!motion_.description.empty()) ImGui::TextWrapped("%s", motion_.description.c_str());

    // parameters
    int removeParam = -1;
    for (int i = 0; i < int(motion_.params.size()); ++i) {
        MotionParam& p = motion_.params[i];
        ImGui::PushID(i);
        char label[160];
        std::snprintf(label, sizeof(label), "%s (%s)%s%s", p.label.c_str(), p.name.c_str(), p.unit.empty() ? "" : " ", p.unit.c_str());
        float v = float(p.value);
        const float speed = float(std::max(0.001, (p.maxValue - p.minValue) / 400.0));
        if (ImGui::DragFloat("##v", &v, speed, float(p.minValue), float(p.maxValue), "%.3f")) {
            p.value = v;
            planDirty_ = true;
        }
        ImGui::SameLine();
        ImGui::TextUnformatted(label);
        if (ImGui::BeginPopupContextItem("##pctx")) {
            if (ImGui::MenuItem("Varsayılana döndür")) {
                p.value = p.defaultValue;
                planDirty_ = true;
            }
            if (ImGui::MenuItem("Varsayılan yap (kaydetmek için)")) {
                p.defaultValue = p.value;
                motionEdited_ = true;
            }
            if (ImGui::MenuItem("Parametreyi sil")) removeParam = i;
            ImGui::EndPopup();
        }
        ImGui::PopID();
    }
    if (removeParam >= 0) {
        motion_.params.erase(motion_.params.begin() + removeParam);
        motionEdited_ = true;
        recompileMotion();
    }

    if (ImGui::TreeNode("Formülleri düzenle")) {
        ImGui::TextDisabled("Yerel eksenler: x = yan, y = ilerleme, z = yüzey normali (mm)");
        ImGui::TextDisabled(motionMode_ == 1 ? "Değişkenler: s, u, L (çizgi boyunca), t, v, k (çizgi no), n (çizgi sayısı)"
                                             : "Değişkenler: s (yol boyunca mm), u (0..1), L (uzunluk), t (sn), v (hız)");
        ImGui::TextDisabled("Fonksiyonlar: sin cos tri saw sq pulse abs min max if clamp ...");
        for (int i = 0; i < CompiledMotion::Count; ++i) {
            ImGui::PushID(100 + i);
            if (ImGui::InputText(kFormulaLabels[i], formulaBuf_[i], sizeof(formulaBuf_[i]))) {
                motionEdited_ = true;
                recompileMotion();
            }
            if (!formulaErr_[i].empty()) ImGui::TextColored(ImVec4(1, 0.45f, 0.4f, 1), "  %s", formulaErr_[i].c_str());
            ImGui::PopID();
        }
        ImGui::SetNextItemWidth(ImGui::GetFontSize() * 5);
        ImGui::InputText("##newparam", newParamBuf_, sizeof(newParamBuf_));
        ImGui::SameLine();
        if (ImGui::Button("Parametre ekle") && newParamBuf_[0]) {
            MotionParam p;
            p.name = p.label = newParamBuf_;
            p.value = p.defaultValue = 1.0;
            p.minValue = -100.0;
            p.maxValue = 100.0;
            motion_.params.push_back(p);
            newParamBuf_[0] = 0;
            motionEdited_ = true;
            recompileMotion();
        }
        ImGui::InputText("Ad", motionNameBuf_, sizeof(motionNameBuf_));
        if (ImGui::Button("Kütüphaneye kaydet")) {
            MotionDef m = motion_;
            m.name = motionNameBuf_;
            for (auto& p : m.params) p.defaultValue = p.value;
            const std::string id = MotionLibrary::fileIdFromName(m.name);
            const bool scanFile = motionMode_ == 1;
            const std::string path =
                (fs::path(library_.dir) / (scanFile ? "scans" : "motions") / (id + (scanFile ? ".pscan" : ".pmotion"))).string();
            std::string err;
            if (MotionLibrary::saveMotion(m, path, err, scanFile)) {
                library_.load(library_.dir);
                for (int i = 0; i < int(activeDefs().size()); ++i)
                    if (activeDefs()[i].id == id) selectMotion(i);
                notify("Kütüphaneye kaydedildi: " + path);
            } else notify(err, true);
        }
        ImGui::SetItemTooltip("Aynı ada sahip bir hareket varsa üzerine yazılır.");
        ImGui::TreePop();
    }

    // ---------------- settings
    if (ImGui::TreeNode("Yol ayarları")) {
        bool c = false;
        c |= ImGui::DragFloat("Örnekleme adımı", &activeSettings().step, 0.01f, 0.05f, 20.0f, "%.2f mm");
        c |= ImGui::DragFloat("İlerleme hızı", &activeSettings().speed, 0.1f, 0.1f, 200.0f, "%.1f mm/s");
        c |= ImGui::DragFloat("Yaklaşma mesafesi", &activeSettings().approach, 0.5f, 0.0f, 200.0f, "%.1f mm");
        c |= ImGui::DragFloat("Yaklaşma hızı", &activeSettings().approachSpeed, 0.5f, 1.0f, 500.0f, "%.0f mm/s");
        c |= ImGui::DragFloat("Eklem hızı", &activeSettings().jointSpeed, 1.0f, 5.0f, 360.0f, "%.0f °/s");
        c |= ImGui::DragFloat("Konum toleransı", &activeSettings().posTolerance, 0.005f, 0.005f, 5.0f, "%.3f mm");
        if (c) planDirty_ = true;
        ImGui::TreePop();
    }

    // ---------------- status + run
    ImGui::SeparatorText("Durum");
    if (!planError_.empty()) ImGui::TextColored(ImVec4(1, 0.5f, 0.4f, 1), "%s", planError_.c_str());
    if (!planPoints_.empty()) {
        if (!robot_) ImGui::TextColored(ImVec4(0.4f, 0.8f, 1, 1), "%zu nokta (erişim kontrolü için robot yükleyin)", planPoints_.size());
        else {
            const bool all = planReachable_ == int(planPoints_.size());
            ImGui::TextColored(all ? ImVec4(0.45f, 0.95f, 0.5f, 1) : ImVec4(1, 0.5f, 0.4f, 1), "Erişilebilir: %d / %zu nokta", planReachable_,
                               planPoints_.size());
            ImGui::TextDisabled("En büyük IK hatası: %.3f mm, %.2f°", planMaxPosErr_, planMaxDirErr_);
            int unsolved = 0;
            for (const auto& p : planPoints_) unsolved += !p.solved;
            if (unsolved > 0)
                ImGui::TextColored(ImVec4(0.4f, 0.8f, 1, 1), "%d nokta önizlemede kontrol edilmedi (süre sınırı); Çalıştır hepsini çözer.", unsolved);
        }
        const float workLen = motionMode_ == 1 ? scanStats_.processLength : seam_.length;
        ImGui::TextDisabled("Süre (çalışma kısmı): %.1f s", workLen / std::max(activeSettings().speed, 0.01f));
    }
    const bool canRun = robot_ && !planPoints_.empty();
    ImGui::BeginDisabled(!canRun);
    if (!playing_) {
        if (ImGui::Button("Çalıştır  >", ImVec2(ImGui::GetFontSize() * 7, 0))) startRun();
    } else {
        if (ImGui::Button(paused_ ? "Devam  >" : "Duraklat  ||", ImVec2(ImGui::GetFontSize() * 7, 0))) paused_ = !paused_;
    }
    ImGui::EndDisabled();
    ImGui::SameLine();
    ImGui::BeginDisabled(!playing_);
    if (ImGui::Button("Durdur")) stopRun();
    ImGui::EndDisabled();
    ImGui::SameLine();
    ImGui::SetNextItemWidth(ImGui::GetFontSize() * 4);
    ImGui::DragFloat("##speed", &playSpeed_, 0.05f, 0.1f, 20.0f, "x%.1f");
    ImGui::SetItemTooltip("Oynatma hızı çarpanı");
    if (traj_.valid()) {
        float t = playTime_;
        if (ImGui::SliderFloat("Zaman", &t, 0.0f, traj_.duration, "%.2f s") && robot_) {
            playTime_ = t;
            if (!playing_) {
                playing_ = true;
                paused_ = true;
            }
            const std::vector<float> q = traj_.sample(playTime_);
            ik_.applyAngles(*robot_, q);
            syncRobot();
        }
    }
    if (!runWarning_.empty() && traj_.valid()) ImGui::TextColored(ImVec4(1, 0.7f, 0.3f, 1), "%s", runWarning_.c_str());
    ImGui::Checkbox("Çarpışmada durdur", &stopOnCollision_);
    ImGui::SameLine();
    ImGui::Checkbox("İz", &showTrace_);
    ImGui::SameLine();
    ImGui::Checkbox("Takım yönleri", &showToolTicks_);
    ImGui::PopItemWidth();
    ImGui::End();
}

// =============================================================================================
// Drawing
// =============================================================================================

void Editor::addMotionRenderData(FrameScene& fs) {
    auto batch = [&](const glm::vec4& c, float w) {
        LineBatch b;
        b.color = c;
        b.width = w;
        b.depthTest = false;
        return b;
    };
    // tool head geometry
    if (robot_ && toolIndex_ >= 0 && toolIndex_ < int(library_.tools.size())) {
        const ToolDef& t = library_.tools[toolIndex_];
        if (glm::length(t.tipOffset) > 0.01f) {
            const glm::mat4 F = robot_->tcpWorld();
            const glm::vec3 base = glm::vec3(F[3]);
            const glm::vec3 tip = base + glm::mat3(F) * t.tipOffset;
            LineBatch b = batch(glm::vec4(t.color, 1.0f), std::max(2.0f, t.radius));
            b.depthTest = true;
            b.points = {base, tip};
            const glm::vec3 ax = glm::normalize(tip - base);
            glm::vec3 u = glm::normalize(glm::cross(ax, std::abs(ax.z) < 0.9f ? glm::vec3(0, 0, 1) : glm::vec3(1, 0, 0)));
            const glm::vec3 v = glm::cross(ax, u);
            const float r = t.radius * 0.8f;
            for (const glm::vec3& d : {u, v}) b.points.insert(b.points.end(), {tip - ax * r * 2.0f + d * r, tip, tip - ax * r * 2.0f - d * r, tip});
            fs.lines.push_back(std::move(b));
        }
    }
    // surface scan: highlight the faces to be painted
    if (motionMode_ == 1) {
        for (const auto& r : scanFaces_) {
            const SceneObject* o = scene_.find(r.objectId);
            if (!o || !o->visible) continue;
            std::vector<uint32_t> tris;
            appendFaceTriangles(*o->mesh, scanFaceMode_, r.index, tris);
            TriHighlight h{o->mesh, o->matrix(), glm::vec4(0.35f, 0.6f, 1.0f, 0.35f), {}};
            for (uint32_t t : tris) {
                const glm::uvec3& tri = o->mesh->triangles[t];
                h.indices.insert(h.indices.end(), {tri.x, tri.y, tri.z});
            }
            fs.highlights.push_back(std::move(h));
        }
    }

    // edge path: seam base line + local frame at the start
    const float fl = seam_.valid ? std::clamp(seam_.length * 0.35f, 4.0f, 25.0f) : 10.0f;
    if (motionMode_ == 0 && seam_.valid) {
        LineBatch base = batch(glm::vec4(1, 1, 1, 0.55f), 1.0f);
        base.points = {seam_.start, seam_.end};
        fs.overlayLines.push_back(std::move(base));
        const glm::vec3 axes[3] = {seam_.X, seam_.Y, seam_.N};
        const glm::vec4 cols[3] = {{1, 0.3f, 0.3f, 1}, {0.3f, 1, 0.3f, 1}, {0.35f, 0.6f, 1, 1}};
        for (int k = 0; k < 3; ++k) {
            LineBatch a = batch(cols[k], 2.0f);
            a.points = {seam_.start, seam_.start + axes[k] * fl};
            fs.overlayLines.push_back(std::move(a));
        }
    }
    if (planPoints_.empty()) return;

    // planned path, coloured by reachability; links (tool off) in grey
    LineBatch ok = batch(kPathOk, 2.5f), bad = batch(kPathBad, 3.0f), nor = batch(kPathNoRobot, 2.5f);
    LineBatch links = batch(glm::vec4(0.75f, 0.75f, 0.8f, 0.45f), 1.0f);
    LineBatch ticks = batch(glm::vec4(0.8f, 0.8f, 0.85f, 0.6f), 1.0f);
    const int tickEvery = std::max(1, int(planPoints_.size() / 30));
    const float tickLen = motionMode_ == 1 ? 12.0f : fl * 0.6f;
    for (size_t i = 0; i + 1 < planPoints_.size(); ++i) {
        const PathPoint &a = planPoints_[i], &b = planPoints_[i + 1];
        LineBatch& L = (a.phase == PathPhase::Link || b.phase == PathPhase::Link)
                           ? links
                           : (!a.solved ? nor : (a.reachable && b.reachable ? ok : bad));
        if (&L == &links && a.solved && b.solved && (!a.reachable || !b.reachable)) {
            bad.points.insert(bad.points.end(), {a.pos, b.pos});
            continue;
        }
        L.points.insert(L.points.end(), {a.pos, b.pos});
    }
    if (showToolTicks_)
        for (size_t i = 0; i < planPoints_.size(); i += size_t(tickEvery))
            if (planPoints_[i].phase == PathPhase::Process)
                ticks.points.insert(ticks.points.end(), {planPoints_[i].pos, planPoints_[i].pos - planPoints_[i].dir * tickLen});
    {
        LineBatch app = batch(glm::vec4(0.7f, 0.7f, 0.7f, 0.5f), 1.0f);
        app.points = {planPoints_.front().pos, planPoints_.front().pos - planPoints_.front().dir * activeSettings().approach};
        fs.overlayLines.push_back(std::move(app));
    }
    for (LineBatch* b : {&links, &ticks, &nor, &ok, &bad})
        if (!b->points.empty()) fs.overlayLines.push_back(std::move(*b));

    if (showTrace_ && trace_.size() >= 2) {
        LineBatch tr = batch(kTraceColor, 2.0f);
        for (size_t i = 0; i + 1 < trace_.size(); ++i) tr.points.insert(tr.points.end(), {trace_[i], trace_[i + 1]});
        fs.overlayLines.push_back(std::move(tr));
    }

    // spray cone while painting
    if (playing_ && robot_ && traj_.valid() && motionMode_ == 1) {
        int idx = 0;
        traj_.sample(playTime_, &idx);
        const PathPhase ph = traj_.keys[size_t(idx)].phase;
        const bool painting = ph == PathPhase::Process && idx + 1 < int(traj_.keys.size()) && traj_.keys[size_t(idx) + 1].phase == PathPhase::Process;
        if (painting) {
            const glm::mat4 F = ik_.toolFrame(*robot_, ik_.currentAngles(*robot_), currentTool());
            const glm::vec3 tip = glm::vec3(F[3]), ax = glm::normalize(glm::vec3(F[2]));
            const float len = std::max(5.0f, float(motion_.paramValue("standoff", 50.0)));
            const float rad = len * std::tan(glm::radians(12.0f));
            const glm::vec3 u = glm::normalize(glm::vec3(F[0])), v = glm::normalize(glm::vec3(F[1]));
            LineBatch cone = batch(glm::vec4(0.4f, 0.95f, 0.5f, 0.55f), 1.0f);
            const int segs = 16;
            for (int k = 0; k < segs; ++k) {
                const float a0 = 2.0f * float(M_PI) * float(k) / segs, a1 = 2.0f * float(M_PI) * float(k + 1) / segs;
                const glm::vec3 p0 = tip + ax * len + (u * std::cos(a0) + v * std::sin(a0)) * rad;
                const glm::vec3 p1 = tip + ax * len + (u * std::cos(a1) + v * std::sin(a1)) * rad;
                cone.points.insert(cone.points.end(), {p0, p1});
                if (k % 4 == 0) cone.points.insert(cone.points.end(), {tip, p0});
            }
            fs.overlayLines.push_back(std::move(cone));
        }
    }
}

void Editor::drawMotionOverlay(ImDrawList* dl) {
    if (motionMode_ == 1) {
        if (planPoints_.empty()) return;
        const glm::vec2 size(vpSize_.x, vpSize_.y);
        glm::vec2 s;
        if (camera_.worldToViewport(planPoints_.front().pos, size, s))
            dl->AddText(ImVec2(vpPos_.x + s.x + 6, vpPos_.y + s.y - 6), IM_COL32(230, 240, 230, 255), "Başlangıç");
        return;
    }
    if (!seam_.valid) return;
    const glm::vec2 size(vpSize_.x, vpSize_.y);
    auto label = [&](const glm::vec3& w, const char* text, ImU32 col) {
        glm::vec2 s;
        if (!camera_.worldToViewport(w, size, s)) return;
        dl->AddText(ImVec2(vpPos_.x + s.x + 6, vpPos_.y + s.y - 6), col, text);
    };
    const float fl = std::clamp(seam_.length * 0.35f, 4.0f, 25.0f);
    label(seam_.start + seam_.X * fl, "x", IM_COL32(255, 90, 90, 255));
    label(seam_.start + seam_.Y * fl, "y", IM_COL32(90, 255, 90, 255));
    label(seam_.start + seam_.N * fl, "z", IM_COL32(110, 160, 255, 255));
    if (!planPoints_.empty()) {
        label(planPoints_.front().pos, "Başlangıç", IM_COL32(230, 240, 230, 255));
        label(planPoints_.back().pos, "Bitiş", IM_COL32(230, 240, 230, 255));
    }
}

} // namespace pegasus
