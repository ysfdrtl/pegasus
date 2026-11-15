// Automated end-to-end UI test (run with: pegasus --selftest <output-dir>).
// Drives the real input path by injecting mouse events into ImGui, then checks editor state
// and saves screenshots of each step.
#include "editor/Editor.h"

#include <chrono>
#include <csignal>
#include <filesystem>
#ifndef _WIN32
#include <sys/wait.h>
#include <unistd.h>
#endif

#include <algorithm>
#include <cmath>
#include <cstdio>

namespace pegasus {

void Editor::selfTestInject() {
    if (selfTestDir_.empty()) return;
    if (robotSelfTest_) {
        selfTestRobotInject();
        return;
    }
    if (motionSelfTest_) {
        selfTestMotionInject();
        return;
    }
    if (scanSelfTest_) {
        selfTestScanInject();
        return;
    }
    if (selectSelfTest_) {
        selfTestSelectInject();
        return;
    }
    if (arduinoSelfTest_) {
        selfTestArduinoInject();
        return;
    }
    ImGuiIO& io = ImGui::GetIO();
    const int f = selfTestFrame_++;

    auto screen = [&](const glm::vec3& w) {
        glm::vec2 s(0.0f);
        camera_.worldToViewport(w, glm::vec2(vpSize_.x, vpSize_.y), s);
        return ImVec2(vpPos_.x + s.x, vpPos_.y + s.y);
    };
    auto obj = [&](size_t i) { return scene_.objects().size() > i ? scene_.objects()[i].get() : nullptr; };
    auto move = [&](const glm::vec3& w) {
        const ImVec2 p = screen(w);
        io.AddMousePosEvent(p.x, p.y);
    };
    auto capture = [&](const char* name) { captureRequest_ = selfTestDir_ + "/" + name; };
    auto check = [&](bool ok, const char* what) {
        std::printf("[selftest] %-55s %s\n", what, ok ? "OK" : "FAIL");
        if (!ok) selfTestFailures_++;
    };
    // Box: 80x60x40, cylinder r25 h90, sphere r35 (pivots at the bottom centre)
    SceneObject* box = obj(0);
    SceneObject* cyl = obj(1);
    SceneObject* sph = obj(2);
    const glm::vec3 bp = box ? box->transform.position : glm::vec3(0);

    static uint32_t u_box = 0, u_cyl = 0, u_sph = 0;
    static glm::vec3 u_cylStart(0.0f), u_sphStart(0.0f), u_cylAfter(0.0f), u_sphAfter(0.0f);
    switch (f) {
    case 2:
        addPrimitive(0, glm::vec3(80.0f, 60.0f, 40.0f));
        addPrimitive(1);
        addPrimitive(2);
        camera_.setView(-60.0f, 30.0f);
        frameAll();
        camera_.distance *= 0.8f;
        break;
    case 10: capture("01_sahne.png"); break;

    // ---- face selection
    case 12: setTool(Tool::Face); move(bp + glm::vec3(5, 5, 40)); break;
    case 14: io.AddMouseButtonEvent(0, true); break;
    case 15: io.AddMouseButtonEvent(0, false); break;
    case 16: move(bp + glm::vec3(0, -30, 20)); break; // hover the front face
    case 19:
        check(selKind_ == Tool::Face && selection_.size() == 1 && selection_[0].objectId == box->id, "yuzey secimi (kutu ust yuzu)");
        if (!selection_.empty())
            check(std::abs(faceArea(*box->mesh, FaceMode::Planar, selection_[0].index) - 80.0f * 60.0f) < 1.0f, "secilen yuzey alani = 4800 mm2");
        check(hover_.valid() && hover_ != (selection_.empty() ? ElementRef{} : selection_[0]), "farkli yuzey uzerinde hover");
        capture("02_yuzey.png");
        break;

    // ---- edge selection
    case 22: setTool(Tool::Edge); move(bp + glm::vec3(10, -30, 40)); break;
    case 24: io.AddMouseButtonEvent(0, true); break;
    case 25: io.AddMouseButtonEvent(0, false); break;
    case 28: {
        bool ok = selKind_ == Tool::Edge && selection_.size() == 1;
        if (ok) {
            glm::vec3 a, b;
            edgeWorld(selection_[0], a, b);
            ok = std::abs(glm::length(b - a) - 80.0f) < 0.01f && std::abs(a.y - (bp.y - 30)) < 0.01f && std::abs(a.z - 40) < 0.01f;
        }
        check(ok, "kenar secimi (on ust kenar, 80 mm)");
        capture("03_kenar.png");
        break;
    }

    // ---- vertex selection (+ shift multi select)
    case 30: setTool(Tool::Vertex); move(bp + glm::vec3(40, -30, 40)); break;
    case 32: io.AddMouseButtonEvent(0, true); break;
    case 33: io.AddMouseButtonEvent(0, false); break;
    case 34: move(bp + glm::vec3(-40, -30, 40)); io.AddKeyEvent(ImGuiMod_Shift, true); break;
    case 36: io.AddMouseButtonEvent(0, true); break;
    case 37: io.AddMouseButtonEvent(0, false); io.AddKeyEvent(ImGuiMod_Shift, false); break;
    case 40: {
        bool ok = selKind_ == Tool::Vertex && selection_.size() == 2;
        if (ok) ok = std::abs(glm::length(vertexWorld(selection_[0]) - vertexWorld(selection_[1])) - 80.0f) < 0.01f;
        check(ok, "iki nokta secimi (Shift), mesafe 80 mm");
        capture("04_nokta.png");
        break;
    }

    // ---- free drag: cylinder into the box -> collision
    case 42:
        setTool(Tool::Object);
        clearSelection();
        activeObject_ = 0;
        move(cyl->transform.position + glm::vec3(0, -25, 45));
        break;
    case 44: io.AddMouseButtonEvent(0, true); break;
    case 70: io.AddMouseButtonEvent(0, false); break;
    case 74: {
        const bool colliding = collision_.isColliding(cyl->id) && collision_.isColliding(box->id);
        check(dragging_ == false && colliding, "silindir kutuya suruklendi -> carpisma algilandi");
        capture("05_carpisma.png");
        break;
    }

    // ---- blocked drag: sphere towards the box stops at contact
    case 76:
        blockOnCollision_ = true;
        move(sph->transform.position + glm::vec3(0, -35, 35));
        break;
    case 78: io.AddMouseButtonEvent(0, true); break;
    case 104: io.AddMouseButtonEvent(0, false); break;
    case 108: {
        check(!collision_.isColliding(sph->id), "engelleme acik: kure kutuya girmedi");
        const float gap = sph->worldBounds().min.x - box->worldBounds().max.x;
        std::printf("[selftest] kure-kutu x araligi: %.3f mm\n", gap);
        capture("06_engelleme.png");
        break;
    }

    // ---- smooth surface selection on the sphere
    case 110:
        setTool(Tool::Face);
        faceMode_ = FaceMode::Smooth;
        move(sph->transform.position + glm::vec3(0, 0, 70));
        break;
    case 112: io.AddMouseButtonEvent(0, true); break;
    case 113: io.AddMouseButtonEvent(0, false); break;
    case 116:
        check(selection_.size() == 1 && selection_[0].objectId == sph->id, "yumusak yuzey secimi (kure)");
        capture("07_kure_yuzey.png");
        break;
    // ---- undo / redo with the real keyboard shortcuts
    case 120: {
        u_box = box->id;
        u_cyl = cyl->id;
        u_sph = sph->id;
        u_cylAfter = cyl->transform.position;
        u_sphAfter = sph->transform.position;
        check(glm::length(u_cylAfter - u_cylStart) > 1.0f, "silindir surukleme ile yer degistirdi (geri alinacak)");
        io.AddKeyEvent(ImGuiMod_Ctrl, true);
        io.AddKeyEvent(ImGuiKey_Z, true);
        break;
    }
    case 121:
        io.AddKeyEvent(ImGuiKey_Z, false);
        io.AddKeyEvent(ImGuiMod_Ctrl, false);
        break;
    case 124: { // last edit = the sphere drag (if it moved at all) -> undo once more for the cylinder
        const SceneObject* s2 = scene_.find(u_sph);
        const bool sphMoved = glm::length(u_sphAfter - u_sphStart) > 0.01f;
        if (sphMoved) check(s2 && glm::length(s2->transform.position - u_sphStart) < 1e-4f, "Ctrl+Z: kure suruklemesi geri alindi");
        io.AddKeyEvent(ImGuiMod_Ctrl, true);
        io.AddKeyEvent(ImGuiKey_Z, true);
        break;
    }
    case 125:
        io.AddKeyEvent(ImGuiKey_Z, false);
        io.AddKeyEvent(ImGuiMod_Ctrl, false);
        break;
    case 128: {
        const SceneObject* c = scene_.find(u_cyl);
        check(c && glm::length(c->transform.position - u_cylStart) < 1e-4f, "Ctrl+Z: silindir suruklemeden onceki yerine dondu");
        check(!collision_.isColliding(u_cyl), "geri alininca carpisma da kalkti");
        capture("08_geri_al.png");
        io.AddKeyEvent(ImGuiMod_Ctrl, true);
        io.AddKeyEvent(ImGuiKey_Y, true);
        break;
    }
    case 129:
        io.AddKeyEvent(ImGuiKey_Y, false);
        io.AddKeyEvent(ImGuiMod_Ctrl, false);
        break;
    case 132: {
        const SceneObject* c = scene_.find(u_cyl);
        check(c && glm::length(c->transform.position - u_cylAfter) < 1e-4f, "Ctrl+Y: silindir surukleme yinelendi");
        activeObject_ = u_box;
        io.AddKeyEvent(ImGuiKey_Delete, true);
        break;
    }
    case 133: io.AddKeyEvent(ImGuiKey_Delete, false); break;
    case 136:
        check(scene_.find(u_box) == nullptr && scene_.objects().size() == 2, "Delete: kutu silindi");
        io.AddKeyEvent(ImGuiMod_Ctrl, true);
        io.AddKeyEvent(ImGuiKey_Z, true);
        break;
    case 137:
        io.AddKeyEvent(ImGuiKey_Z, false);
        io.AddKeyEvent(ImGuiMod_Ctrl, false);
        break;
    case 140: {
        const SceneObject* b = scene_.find(u_box);
        check(b && scene_.objects().size() == 3 && glm::length(b->transform.position - bp) < 1e-4f, "Ctrl+Z: silinen kutu ayni kimlik ve yerle geri geldi");
        // redo of the delete, then undo again: history in both directions
        redo();
        check(scene_.find(u_box) == nullptr, "yinele: kutu yeniden silindi");
        undo();
        check(scene_.find(u_box) != nullptr, "geri al: kutu tekrar geldi");
        // an empty history says so instead of doing anything
        while (!undoStack_.empty()) undo();
        const size_t n = scene_.objects().size();
        undo();
        check(scene_.objects().size() == n, "bos gecmiste geri al sahneyi bozmaz");
        break;
    }
    case 142:
        std::printf("[selftest] %s (%d hata)\n", selfTestFailures_ ? "BASARISIZ" : "TUM TESTLER GECTI", selfTestFailures_);
        quitRequested_ = true;
        break;
    default: break;
    }
    if (f == 42 && cyl && sph) {
        u_cylStart = cyl->transform.position;
        u_sphStart = sph->transform.position;
    }
    // Interpolated drags (world positions captured when the drag starts)
    static glm::vec3 dragFrom, dragTo, sphereStart;
    if (f == 44 && cyl && box) {
        dragFrom = cyl->transform.position + glm::vec3(0, -25, 45);
        dragTo = bp + glm::vec3(0, -25, 45);
    }
    if (f == 78 && sph && box) {
        dragFrom = sph->transform.position + glm::vec3(0, -35, 35);
        dragTo = bp + glm::vec3(0, -35, 35);
        sphereStart = sph->transform.position;
    }
    if ((f > 44 && f < 70) || (f > 78 && f < 104)) {
        const float t = f < 70 ? float(f - 44) / 25.0f : float(f - 78) / 25.0f;
        move(glm::mix(dragFrom, dragTo, std::min(t, 1.0f)));
    }
    if (f == 108 && sph) check(glm::length(sph->transform.position - sphereStart) > 1.0f, "kure suruklenince yer degistirdi");
}

// Robot test: pegasus --selftest-robot <stl-folder> <output-dir>
void Editor::selfTestRobotInject() {
    const int f = selfTestFrame_++;
    auto capture = [&](const char* name) { captureRequest_ = selfTestDir_ + "/" + name; };
    auto check = [&](bool ok, const std::string& what) {
        std::printf("[robot-test] %-62s %s\n", what.c_str(), ok ? "OK" : "FAIL");
        if (!ok) selfTestFailures_++;
    };
    auto driver = [&](const char* name) {
        if (robot_)
            for (int d : robot_->drivers())
                if (robot_->joints[d].name == name) return d;
        return -1;
    };
    auto partByName = [&](const char* needle) {
        if (robot_)
            for (int i = 0; i < int(robot_->parts.size()); ++i)
                if (robot_->parts[i].name.find(needle) != std::string::npos) return i;
        return -1;
    };
    auto near = [](const glm::mat4& a, const glm::mat4& b, float tol) {
        for (int c = 0; c < 4; ++c)
            for (int r = 0; r < 4; ++r)
                if (std::abs(a[c][r] - b[c][r]) > tol) return false;
        return true;
    };
    static glm::vec3 tcp0;
    static glm::mat4 mBase, mWaist, mArm1, mArm2;
    static float elbowToTcp = 0.0f, fingerGap0 = 0.0f;
    static int obstacleDir = 0;

    if (f > 2 && !robot_) {
        check(false, "robot yuklenemedi");
        quitRequested_ = true;
        return;
    }
    Robot* R = robot_.get();
    switch (f) {
    case 3: {
        const auto drv = R->drivers();
        check(drv.size() == 6, "6 bagimsiz eklem bulundu (" + std::to_string(drv.size()) + ")");
        const char* names[] = {"Bel", "Omuz", "Dirsek", "Bilek dönüş", "Bilek eğim", "Kıskaç"};
        bool namesOk = drv.size() == 6;
        for (size_t k = 0; namesOk && k < 6; ++k) namesOk = R->joints[drv[k]].name == names[k];
        check(namesOk, "eklem sirasi: Bel, Omuz, Dirsek, Bilek donus, Bilek egim, Kiskac");
        int mimics = 0;
        for (const auto& j : R->joints) mimics += j.mimic == driver("Kıskaç");
        check(mimics == 7, "kiskac mekanizmasi: 7 bagimli eklem (" + std::to_string(mimics) + ")");
        const int servo = partByName("Servo");
        check(servo >= 0 && R->joints[R->parts[servo].joint].type == JointType::Fixed, "9g servo govdeye sabit");
        check(!robotColliding(), "ev konumunda yanlis carpisma yok");
        capture("10_robot.png");
        break;
    }
    case 5: {
        tcp0 = glm::vec3(R->tcpWorld()[3]);
        const int bel = driver("Bel");
        const glm::vec3 o = R->jointOriginWorld(bel), a = R->jointAxisWorld(bel);
        setRobotAngle(bel, 90.0f);
        const glm::vec3 tcp1 = glm::vec3(R->tcpWorld()[3]);
        const glm::vec3 v0 = tcp0 - o, v1 = tcp1 - o;
        const glm::vec3 p0 = v0 - glm::dot(v0, a) * a, p1 = v1 - glm::dot(v1, a) * a;
        const float ang = glm::degrees(std::acos(std::clamp(glm::dot(glm::normalize(p0), glm::normalize(p1)), -1.0f, 1.0f)));
        check(std::abs(glm::dot(v0, a) - glm::dot(v1, a)) < 0.01f && std::abs(glm::length(p0) - glm::length(p1)) < 0.01f &&
                  std::abs(ang - 90.0f) < 0.05f,
              "Bel 90 derece: TCP dikey eksen etrafinda 90 derece dondu");
        std::printf("[robot-test] TCP ev: (%.1f %.1f %.1f)  Bel=90: (%.1f %.1f %.1f)\n", tcp0.x, tcp0.y, tcp0.z, tcp1.x, tcp1.y, tcp1.z);
        capture("11_bel90.png");
        break;
    }
    case 8: {
        setRobotAngle(driver("Bel"), 0.0f);
        mBase = R->partWorld(partByName("Base"));
        mWaist = R->partWorld(partByName("Waist"));
        mArm1 = R->partWorld(partByName("Arm 01"));
        mArm2 = R->partWorld(partByName("Arm 02"));
        const int dirsek = driver("Dirsek");
        elbowToTcp = glm::length(glm::vec3(R->tcpWorld()[3]) - R->jointOriginWorld(dirsek));
        setRobotAngle(dirsek, -60.0f);
        check(near(mBase, R->partWorld(partByName("Base")), 1e-4f) && near(mWaist, R->partWorld(partByName("Waist")), 1e-4f) &&
                  near(mArm1, R->partWorld(partByName("Arm 01")), 1e-4f),
              "Dirsek donunce taban/bel/kol1 yerinde kaldi");
        check(!near(mArm2, R->partWorld(partByName("Arm 02")), 1e-3f), "Dirsek donunce kol2 hareket etti");
        check(std::abs(glm::length(glm::vec3(R->tcpWorld()[3]) - R->jointOriginWorld(dirsek)) - elbowToTcp) < 0.01f,
              "dirsek-TCP mesafesi korundu (rijit zincir)");
        break;
    }
    case 10:
        setRobotAngle(driver("Omuz"), 30.0f);
        setRobotAngle(driver("Bilek eğim"), 40.0f);
        setRobotAngle(driver("Bilek dönüş"), 45.0f);
        setRobotAngle(driver("Bel"), -35.0f);
        check(!robotColliding(), "ornek pozda carpisma yok");
        capture("12_poz.png");
        break;
    case 12: {
        for (int d : R->drivers()) setRobotAngle(d, 0.0f);
        const int f1 = partByName("Gripper 1-1"), f2 = partByName("Gripper 1-2"), gb = partByName("Gripper base");
        auto gap = [&]() {
            return glm::length(glm::vec3(R->partWorld(f1) * glm::vec4(R->parts[f1].mesh->bounds.center(), 1.0f)) -
                               glm::vec3(R->partWorld(f2) * glm::vec4(R->parts[f2].mesh->bounds.center(), 1.0f)));
        };
        fingerGap0 = gap();
        const int k = driver("Kıskaç");
        setRobotAngle(k, R->joints[k].maxDeg);
        const glm::mat3 rb = glm::mat3(R->partWorld(gb));
        const bool translateOnly = near(glm::mat4(glm::transpose(rb) * glm::mat3(R->partWorld(f1))), glm::mat4(1.0f), 1e-4f) &&
                                   near(glm::mat4(glm::transpose(rb) * glm::mat3(R->partWorld(f2))), glm::mat4(1.0f), 1e-4f);
        check(translateOnly, "kiskac parmaklari donmeden oteleniyor (paralelkenar)");
        std::printf("[robot-test] parmak merkez araligi: ev %.2f mm, kapali (%.0f deg) %.2f mm\n", fingerGap0, R->joints[k].maxDeg, gap());
        check(gap() < fingerGap0 - 1.0f, "pozitif kiskac acisi = kapanma");
        check(!robotColliding(), "kiskac kapali limitte carpisma yok");
        camera_.frame(scene_.find(R->parts[gb].objectId)->worldBounds());
        camera_.distance *= 0.9f;
        capture("13_kiskac_kapali.png");
        break;
    }
    case 14: {
        const int k = driver("Kıskaç");
        setRobotAngle(k, R->joints[k].minDeg);
        capture("14_kiskac_acik.png");
        break;
    }
    case 16: {
        setRobotAngle(driver("Kıskaç"), 10.0f);
        setRobotAngle(driver("Omuz"), -25.0f);
        std::string err;
        const std::string path = selfTestDir_ + "/robot.pegasus";
        check(R->save(path, err), "robot.pegasus kaydedildi");
        auto loaded = Robot::load(path, R->folder, err);
        bool same = loaded && loaded->joints.size() == R->joints.size() && loaded->parts.size() == R->parts.size();
        if (same) {
            loaded->mount = R->mount;
            loaded->solveFK();
            for (int i = 0; same && i < int(R->parts.size()); ++i) same = near(loaded->partWorld(i), R->partWorld(i), 1e-3f);
        }
        check(same, "kaydet/yukle sonrasi ayni kinematik");
        for (int d : R->drivers()) setRobotAngle(d, 0.0f);
        frameAll();
        break;
    }
    case 18: {
        // obstacle: tall box next to the robot in the shoulder's swing plane
        addPrimitive(0, glm::vec3(80.0f, 60.0f, 40.0f));
        SceneObject* box = scene_.objects().back().get();
        box->transform = Transform{glm::vec3(0.0f), glm::vec3(0.0f), glm::vec3(1.0f, 1.0f, 5.0f)};
        const int omuz = driver("Omuz");
        R->setDriverAngle(omuz, 90.0f);
        syncRobot();
        const glm::vec3 tip = glm::vec3(R->tcpWorld()[3]);
        R->setDriverAngle(omuz, 0.0f);
        syncRobot();
        glm::vec3 dir = tip;
        dir.z = 0.0f;
        dir = glm::normalize(dir);
        box->transform.position = dir * 260.0f;
        scene_.markDirty();
        collision_.update(scene_, true);
        check(!robotColliding(), "engel yerlestirildi, ev konumunda carpisma yok");
        blockOnCollision_ = true;
        obstacleDir = 1;
        setRobotAngle(omuz, 90.0f);
        const float a = R->joints[omuz].angleDeg;
        std::printf("[robot-test] engel varken omuz %.1f derecede durdu\n", a);
        check(a > 5.0f && a < 89.0f && !robotColliding(), "carpismada engelle: omuz engele degmeden durdu");
        capture("15_engel.png");
        break;
    }
    case 21: { // undo / redo of joint angles (robot parts are placed by forward kinematics)
        const int omuz = driver("Omuz");
        blockOnCollision_ = false;
        const float before = R->joints[omuz].angleDeg;
        const glm::mat4 armBefore = R->partWorld(partByName("Arm 02"));
        setRobotAngle(omuz, before - 20.0f);
        updateUndo();
        check(std::abs(R->joints[omuz].angleDeg - (before - 20.0f)) < 1e-3f, "omuz 20 derece cevrildi");
        undo();
        check(std::abs(R->joints[omuz].angleDeg - before) < 1e-4f && near(R->partWorld(partByName("Arm 02")), armBefore, 1e-4f),
              "geri al: eklem acisi ve kol parcalari eski yerinde");
        redo();
        check(std::abs(R->joints[omuz].angleDeg - (before - 20.0f)) < 1e-3f, "yinele: eklem acisi tekrar uygulandi");
        break;
    }
    case 23:
        std::printf("[robot-test] %s (%d hata)\n", selfTestFailures_ ? "BASARISIZ" : "TUM TESTLER GECTI", selfTestFailures_);
        (void)obstacleDir;
        quitRequested_ = true;
        break;
    default: break;
    }
}

// Motion path test: pegasus --selftest-motion <robot-folder> <output-dir>
void Editor::selfTestMotionInject() {
    const int f = selfTestFrame_++;
    auto capture = [&](const char* name) { captureRequest_ = selfTestDir_ + "/" + name; };
    auto check = [&](bool ok, const std::string& what) {
        std::printf("[motion-test] %-62s %s\n", what.c_str(), ok ? "OK" : "FAIL");
        if (!ok) selfTestFailures_++;
    };
    auto selectByName = [&](const char* id) {
        for (int i = 0; i < int(library_.motions.size()); ++i)
            if (library_.motions[i].id == id) {
                selectMotion(i);
                recomputePlan();
                return true;
            }
        return false;
    };
    static uint32_t pedestal = 0, cubeA = 0, cubeB = 0;
    static glm::vec3 towardRobot(0.0f);
    static int runFrames = 0;
    static bool midCaptured = false;

    // place pedestal + two 10 mm cubes with a 0.5 mm gap in direction `dir`, select the two
    // vertical edges of the gap on the face looking at the robot and build the seam
    static float dist = 175.0f, height = 100.0f;
    // look at the seam from the robot side, slightly from the left and above
    auto viewSeam = [&](float distance) {
        const glm::vec3 e = glm::normalize(seam_.N * 0.8f + seam_.X * 0.55f + glm::vec3(0, 0, 0.45f));
        camera_.yawDeg = glm::degrees(std::atan2(e.y, e.x));
        camera_.pitchDeg = glm::degrees(std::asin(e.z));
        camera_.target = seam_.start + seam_.Y * (seam_.length * 0.5f);
        camera_.distance = distance;
    };
    auto place = [&](const glm::vec3& dir) {
        const glm::vec3 side = glm::vec3(-dir.y, dir.x, 0.0f);
        SceneObject* P = scene_.find(pedestal);
        SceneObject* A = scene_.find(cubeA);
        SceneObject* B = scene_.find(cubeB);
        const float yaw = glm::degrees(std::atan2(dir.y, dir.x));
        P->transform = Transform{dir * (dist + 25.0f), glm::vec3(0, 0, yaw), glm::vec3(1.0f, 1.0f, height / 100.0f)};
        A->transform = Transform{dir * dist - side * 5.25f + glm::vec3(0, 0, height), glm::vec3(0, 0, yaw), glm::vec3(1.0f)};
        B->transform = Transform{dir * dist + side * 5.25f + glm::vec3(0, 0, height), glm::vec3(0, 0, yaw), glm::vec3(1.0f)};
        scene_.markDirty();
        auto findEdge = [&](SceneObject* o, float sideSign) {
            const glm::mat4 m = o->matrix();
            const glm::vec3 target = dir * (dist - 5.0f) + side * (0.25f * sideSign);
            for (uint32_t e = 0; e < o->mesh->edges.size(); ++e) {
                const glm::vec3 a = glm::vec3(m * glm::vec4(o->mesh->positions[o->mesh->edges[e].v[0]], 1.0f));
                const glm::vec3 b = glm::vec3(m * glm::vec4(o->mesh->positions[o->mesh->edges[e].v[1]], 1.0f));
                if (std::abs(std::abs(a.z - b.z) - 10.0f) > 1e-3f) continue;
                if (glm::length(glm::vec2(a) - glm::vec2(target)) < 1e-3f && glm::length(glm::vec2(b) - glm::vec2(target)) < 1e-3f)
                    return ElementRef{o->id, e};
            }
            return ElementRef{};
        };
        selKind_ = Tool::Edge;
        selection_ = {findEdge(A, -1.0f), findEdge(B, 1.0f)};
        towardRobot = -dir;
        if (!selection_[0].valid() || !selection_[1].valid()) return -1;
        if (!seamFromSelection()) return -1;
        recomputePlan();
        return planReachable_;
    };

    if (f > 2 && !robot_) {
        check(false, "robot yuklenemedi");
        quitRequested_ = true;
        return;
    }
    switch (f) {
    case 3: {
        check(library_.motions.size() >= 1 && library_.tools.size() >= 2, "kutuphane yuklendi (" + std::to_string(library_.motions.size()) +
                                                                                " hareket, " + std::to_string(library_.tools.size()) + " baslik)");
        for (int i = 0; i < int(library_.tools.size()); ++i)
            if (library_.tools[i].id == "kaynak_torcu") toolIndex_ = i;
        check(library_.tools[toolIndex_].id == "kaynak_torcu", "kaynak torcu secildi");
        check(selectByName("duz") && motion_.name == "Optimum noktadan düz ilerle", "hareket: Optimum noktadan duz ilerle");
        addPrimitive(0, glm::vec3(80.0f, 60.0f, 100.0f));
        pedestal = scene_.objects().back()->id;
        addPrimitive(0);
        cubeA = scene_.objects().back()->id;
        addPrimitive(0);
        cubeB = scene_.objects().back()->id;
        check(scene_.find(cubeA)->mesh->bounds.size() == glm::vec3(10.0f), "standart kutu 10 mm kup");
        break;
    }
    case 4: {
        planSettings_.step = 0.25f;
        const glm::vec3 dirs[4] = {{1, 0, 0}, {-1, 0, 0}, {0, 1, 0}, {0, -1, 0}};
        int best = 0, bestReach = -2;
        float bestDist = 0, bestHeight = 0;
        for (float d : {360.0f, 340.0f, 380.0f})
            for (float h : {60.0f, 100.0f, 140.0f})
                for (int i = 0; i < 4; ++i) {
                    dist = d;
                    height = h;
                    const int r = place(dirs[i]);
                    if (r > bestReach) {
                        bestReach = r;
                        best = i;
                        bestDist = d;
                        bestHeight = h;
                    }
                }
        dist = bestDist;
        height = bestHeight;
        place(dirs[best]);
        std::printf("[motion-test] secilen yerlesim: yon %d, uzaklik %.0f mm, yukseklik %.0f mm -> %d / %zu erisilebilir\n", best, dist,
                    height, bestReach, planPoints_.size());
        check(seam_.valid && std::abs(seam_.length - 10.0f) < 1e-3f, "dikis uzunlugu 10 mm");
        check(std::abs(seam_.gap - 0.5f) < 1e-3f, "kenarlar arasi bosluk 0.5 mm");
        check(glm::dot(seam_.N, towardRobot) > 0.999f, "yuzey normali robota bakan yuze dik");
        check(seam_.Y.z > 0.999f, "ilerleme alttan uste");
        const glm::vec3 expectStart = scene_.find(cubeA)->worldBounds().center() * 0.5f + scene_.find(cubeB)->worldBounds().center() * 0.5f;
        check(std::abs(seam_.start.z - height) < 1e-3f && glm::length(glm::vec2(seam_.start) - glm::vec2(expectStart) - glm::vec2(towardRobot) * 5.0f) < 1e-3f,
              "baslangic: iki kenarin tam ortasi, alt uc");
        check(!planPoints_.empty() && glm::length(planPoints_.front().pos - (seam_.start + seam_.N * 1.0f)) < 1e-4f &&
                  glm::length(planPoints_.back().pos - (seam_.end + seam_.N * 1.0f)) < 1e-4f,
              "duz yol: yuzeyden 1 mm yukarida, baslangictan bitise");
        check(planReachable_ == int(planPoints_.size()), "tum yol noktalari robotca erisilebilir (IK)");
        std::printf("[motion-test] en buyuk IK hatasi: %.4f mm, %.3f derece\n", planMaxPosErr_, planMaxDirErr_);
        clearSelection();
        viewSeam(90.0f);
        break;
    }
    case 6: capture("20_yol_duz.png"); break;
    case 8: {
        selectByName("sinus");
        float maxX = 0.0f;
        for (const auto& p : planPoints_) maxX = std::max(maxX, std::abs(glm::dot(p.pos - seam_.start, seam_.X)));
        check(std::abs(maxX - 1.5f) < 0.02f, "sinus: yana +-1.5 mm (A)");
        check(planReachable_ == int(planPoints_.size()), "sinus yolu erisilebilir");
        viewSeam(70.0f);
        break;
    }
    case 10: capture("21_sinus.png"); break;
    case 12: {
        selectByName("zikzak");
        // change a parameter live: amplitude 2 mm
        for (auto& p : motion_.params)
            if (p.name == "A") p.value = 2.0;
        recomputePlan();
        float maxX = 0.0f;
        for (const auto& p : planPoints_) maxX = std::max(maxX, std::abs(glm::dot(p.pos - seam_.start, seam_.X)));
        check(std::abs(maxX - 2.0f) < 0.02f, "zikzak: parametre degisince yol guncellendi (A = 2 mm)");
        break;
    }
    case 14: capture("22_zikzak.png"); break;
    case 16: {
        selectByName("gagalama");
        float minZ = 1e9f, maxZ = -1e9f;
        for (const auto& p : planPoints_) {
            const float z = glm::dot(p.pos - seam_.start, seam_.N);
            minZ = std::min(minZ, z);
            maxZ = std::max(maxZ, z);
        }
        check(std::abs(minZ - 1.0f) < 0.01f && std::abs(maxZ - 5.0f) < 0.05f, "gagalama: z 1..5 mm arasi");
        break;
    }
    case 18: capture("23_gagalama.png"); break;
    case 20: {
        // custom formula typed by the user
        std::snprintf(formulaBuf_[0], sizeof(formulaBuf_[0]), "0.8*sin(2*pi*s/2.5) + q");
        recompileMotion();
        check(!motionValid_ && !formulaErr_[0].empty(), "hatali formul yakalandi: " + formulaErr_[0]);
        std::snprintf(formulaBuf_[0], sizeof(formulaBuf_[0]), "0.8*sin(2*pi*s/2.5)");
        recompileMotion();
        check(motionValid_, "kullanici formulu derlendi");
        selectByName("duz");
        frameAll();
        camera_.distance *= 0.7f;
        startRun();
        playSpeed_ = 3.0f;
        check(playing_ && traj_.valid(), "Calistir: yorunge olusturuldu (" + std::to_string(traj_.keys.size()) + " anahtar)");
        break;
    }
    default: break;
    }

    if (f > 20 && runFrames >= 0) {
        ++runFrames;
        int idx = 0;
        if (playing_) traj_.sample(playTime_, &idx);
        if (playing_ && !midCaptured && idx > 0 && traj_.keys[idx].phase == PathPhase::Process && traj_.keys[idx].s > seam_.length * 0.5f) {
            midCaptured = true;
            viewSeam(420.0f);
            camera_.target = 0.5f * (seam_.start + glm::vec3(0, 0, 150));
            capture("24_calisiyor.png");
        }
        if (!playing_ || runFrames > 1500) {
            check(!playing_, "hareket tamamlandi");
            float worst = 0.0f;
            for (const auto& p : planPoints_) {
                float best = 1e9f;
                for (size_t i = 0; i + 1 < trace_.size(); ++i) {
                    const glm::vec3 a = trace_[i], b = trace_[i + 1];
                    const glm::vec3 ab = b - a;
                    const float t = glm::dot(ab, ab) > 0 ? std::clamp(glm::dot(p.pos - a, ab) / glm::dot(ab, ab), 0.0f, 1.0f) : 0.0f;
                    best = std::min(best, glm::length(a + ab * t - p.pos));
                }
                worst = std::max(worst, best);
            }
            std::printf("[motion-test] takim ucu izinin planli yola en buyuk uzakligi: %.3f mm (%zu iz noktasi)\n", worst, trace_.size());
            check(worst < 0.3f, "robot takim ucu planli yolu izledi");
            check(!robotColliding(), "hareket boyunca/sonunda carpisma yok");
            check(runWarning_.empty(), "yaklasma/geri cekilme dahil tum yorunge erisilebilir");
            viewSeam(80.0f);
            capture("25_bitti.png");
            runFrames = -100000;
        }
    }
    if (runFrames < -99990 && ++runFrames > -99995) {
        std::printf("[motion-test] %s (%d hata)\n", selfTestFailures_ ? "BASARISIZ" : "TUM TESTLER GECTI", selfTestFailures_);
        quitRequested_ = true;
    }
}

// Surface scan test: pegasus --selftest-scan <robot-folder> <output-dir>
// Samples the whole trajectory finely: does the tool tip ever enter one of the given objects, and
// does the arm swing (> 90 degrees between two keys) while the tool is on?
static void trajectoryChecks(const Trajectory& traj, const IkSolver& ik, const Robot& robot, const IkTool& tool, const Scene& scene,
                             const std::vector<uint32_t>& ids, int& insideSamples, int& swingsToolOn) {
    insideSamples = swingsToolOn = 0;
    for (size_t k = 1; k < traj.keys.size(); ++k) {
        const PathPoint &a = traj.keys[k - 1], &b = traj.keys[k];
        float dq = 0.0f;
        for (size_t j = 0; j < a.q.size() && j < b.q.size(); ++j) dq = std::max(dq, std::abs(a.q[j] - b.q[j]));
        if (dq > 90.0f && a.phase == PathPhase::Process && b.phase == PathPhase::Process) ++swingsToolOn;
        const int n = std::max(4, int(dq / 2.0f));
        for (int s = 0; s <= n; ++s) {
            const float t = a.time + (b.time - a.time) * float(s) / float(n);
            const glm::vec3 tip = glm::vec3(ik.toolFrame(robot, traj.sample(t), tool)[3]);
            for (uint32_t id : ids) {
                const SceneObject* o = scene.find(id);
                if (!o) continue;
                const glm::vec3 l = glm::vec3(glm::inverse(o->matrix()) * glm::vec4(tip, 1.0f));
                const AABB& bb = o->mesh->bounds;
                if (glm::all(glm::greaterThan(l, bb.min)) && glm::all(glm::lessThan(l, bb.max)) && o->mesh->containsPoint(l)) ++insideSamples;
            }
        }
    }
}

void Editor::selfTestScanInject() {
    const int f = selfTestFrame_++;
    auto capture = [&](const char* name) { captureRequest_ = selfTestDir_ + "/" + name; };
    auto check = [&](bool ok, const std::string& what) {
        std::printf("[scan-test] %-64s %s\n", what.c_str(), ok ? "OK" : "FAIL");
        if (!ok) selfTestFailures_++;
    };
    auto selectPattern = [&](const char* id) {
        for (int i = 0; i < int(library_.scans.size()); ++i)
            if (library_.scans[i].id == id) {
                selectMotion(i);
                recomputePlan();
                return true;
            }
        return false;
    };
    auto setParam = [&](const char* name, double v) {
        for (auto& p : motion_.params)
            if (p.name == name) p.value = v;
        recomputePlan();
    };
    static uint32_t panelA = 0, panelB = 0;
    static glm::vec3 dirToPanels(1, 0, 0);
    static int runFrames = 0;
    static bool midCaptured = false;
    static float dist = 400.0f, height = 60.0f;

    // two 10 x 80 x 60 mm panels in front of the robot, both turned towards it
    auto place = [&]() {
        const glm::vec3 d = dirToPanels, side(-d.y, d.x, 0.0f);
        const float yaw = glm::degrees(std::atan2(d.y, d.x));
        SceneObject* A = scene_.find(panelA);
        SceneObject* B = scene_.find(panelB);
        A->transform = Transform{d * dist - side * 50.0f + glm::vec3(0, 0, height), glm::vec3(0, 0, yaw - 15.0f), glm::vec3(1.0f)};
        B->transform = Transform{d * dist + side * 50.0f + glm::vec3(0, 0, height), glm::vec3(0, 0, yaw + 15.0f), glm::vec3(1.0f)};
        scene_.markDirty();
        // select the planar face of each panel that looks at the robot
        selection_.clear();
        for (SceneObject* o : {A, B}) {
            const glm::mat4 m = o->matrix();
            const Mesh& mesh = *o->mesh;
            int best = -1;
            float bestDot = -2.0f;
            for (uint32_t r = 0; r < mesh.planarRegions.count(); ++r) {
                const glm::vec3 n = glm::normalize(glm::mat3(m) * mesh.triNormals[*mesh.planarRegions.begin(r)]);
                const glm::vec3 toRobot = glm::normalize(-glm::vec3(m[3].x, m[3].y, 0.0f));
                if (glm::dot(n, toRobot) > bestDot) {
                    bestDot = glm::dot(n, toRobot);
                    best = int(r);
                }
            }
            selection_.push_back({o->id, uint32_t(best)});
        }
        selKind_ = Tool::Face;
        selFaceMode_ = FaceMode::Planar;
        scanFromSelection();
        recomputePlan();
        return planReachable_ - int(planPoints_.size());
    };
    auto viewPanels = [&](float distance) {
        const glm::vec3 c = dirToPanels * dist + glm::vec3(0, 0, height + 30.0f);
        const glm::vec3 e = glm::normalize(-dirToPanels * 0.8f + glm::vec3(-dirToPanels.y, dirToPanels.x, 0) * 0.35f + glm::vec3(0, 0, 0.5f));
        camera_.yawDeg = glm::degrees(std::atan2(e.y, e.x));
        camera_.pitchDeg = glm::degrees(std::asin(e.z));
        camera_.target = c;
        camera_.distance = distance;
    };
    auto processStats = [&](int& passes, float& minEdge, float& maxStandoffErr) {
        passes = scanStats_.passes;
        minEdge = 1e9f;
        maxStandoffErr = 0.0f;
        for (const auto& p : planPoints_) {
            if (p.phase != PathPhase::Process) continue;
            // nearest panel front face: distance from the face plane must be the standoff
            float best = 1e9f, edge = 0.0f;
            for (uint32_t id : {panelA, panelB}) {
                const SceneObject* o = scene_.find(id);
                const glm::mat4 inv = glm::inverse(o->matrix());
                const glm::vec3 l = glm::vec3(inv * glm::vec4(p.pos, 1.0f)); // local: box 10 x 80 x 60, front face x = -5
                const float plane = std::abs(-5.0f - l.x);
                if (plane < best) {
                    best = plane;
                    edge = std::min({l.y + 40.0f, 40.0f - l.y, l.z, 60.0f - l.z});
                }
            }
            maxStandoffErr = std::max(maxStandoffErr, std::abs(best - float(motion_.paramValue("standoff", 0))));
            minEdge = std::min(minEdge, edge);
        }
    };

    if (f > 2 && !robot_) {
        check(false, "robot yuklenemedi");
        quitRequested_ = true;
        return;
    }
    switch (f) {
    case 3: {
        setMotionMode(1);
        check(library_.scans.size() >= 4, "tarama kutuphanesi yuklendi (" + std::to_string(library_.scans.size()) + " desen)");
        for (int i = 0; i < int(library_.tools.size()); ++i)
            if (library_.tools[i].id == "sprey") toolIndex_ = i;
        check(library_.tools[toolIndex_].id == "sprey", "sprey tabancasi secildi");
        check(selectPattern("zikzak_tarama"), "desen: Zikzak tarama");
        setParam("standoff", 40.0);
        addPrimitive(0, glm::vec3(10.0f, 80.0f, 60.0f));
        panelA = scene_.objects().back()->id;
        addPrimitive(0, glm::vec3(10.0f, 80.0f, 60.0f));
        panelB = scene_.objects().back()->id;
        break;
    }
    case 4: {
        const glm::vec3 dirs[4] = {{1, 0, 0}, {-1, 0, 0}, {0, 1, 0}, {0, -1, 0}};
        int bestMiss = -1000000;
        glm::vec3 bd = dirs[0];
        float bdist = 400, bh = 60;
        for (float d : {400.0f, 430.0f, 370.0f, 460.0f})
            for (float h : {40.0f, 80.0f, 0.0f})
                for (const auto& dir : dirs) {
                    dirToPanels = dir;
                    dist = d;
                    height = h;
                    const int miss = place();
                    if (miss > bestMiss) {
                        bestMiss = miss;
                        bd = dir;
                        bdist = d;
                        bh = h;
                    }
                }
        dirToPanels = bd;
        dist = bdist;
        height = bh;
        place();
        std::printf("[scan-test] yerlesim: uzaklik %.0f mm, yukseklik %.0f mm -> %d / %zu nokta erisilebilir\n", dist, height, planReachable_,
                    planPoints_.size());
        int passes;
        float minEdge, sErr;
        processStats(passes, minEdge, sErr);
        check(scanStats_.faces == 2, "iki yuz secildi ve taraniyor");
        check(passes == 12, "yuz basina 6 cizgi (60 - 2x5 = 50 mm / 10 mm), toplam 12 (" + std::to_string(passes) + ")");
        check(std::abs(minEdge - 5.0f) < 0.05f, "kenara en yakin boya noktasi 5 mm (kenar ofseti)");
        check(sErr < 0.01f, "tum boya noktalari yuzeyden 40 mm uzakta");
        check(std::abs(scanStats_.processLength - 12 * 70.0f) < 0.5f, "toplam boya yolu 12 x 70 mm");
        check(planReachable_ == int(planPoints_.size()), "tum noktalar robotca erisilebilir");
        clearSelection();
        viewPanels(330.0f);
        break;
    }
    case 6: capture("30_tarama_zikzak.png"); break;
    case 8: {
        setParam("spacing", 20.0);
        check(scanStats_.passes == 6, "aralik 20 mm: yuz basina 3 cizgi (canli guncelleme)");
        setParam("offset", 12.0);
        int passes;
        float minEdge, sErr;
        processStats(passes, minEdge, sErr);
        check(std::abs(minEdge - 12.0f) < 0.05f, "kenar ofseti 12 mm: en yakin nokta 12 mm");
        setParam("standoff", 60.0);
        processStats(passes, minEdge, sErr);
        check(sErr < 0.01f, "uzaklik 60 mm: tum noktalar 60 mm");
        selectPattern("capraz_tarama");
        setParam("standoff", 40.0);
        setParam("spacing", 10.0);
        check(scanStats_.passes == 2 * (6 + 8), "capraz tarama: yuz basina 6 + 8 cizgi");
        break;
    }
    case 10: capture("31_tarama_capraz.png"); break;
    case 12:
        selectPattern("dalgali_tarama");
        setParam("standoff", 40.0);
        check(motionValid_ && !planPoints_.empty(), "dalgali tarama (formul) yolu olustu");
        break;
    case 14: capture("32_tarama_dalgali.png"); break;
    case 16:
        selectPattern("zikzak_tarama");
        setParam("standoff", 40.0);
        startRun();
        playSpeed_ = 10.0f;
        check(playing_ && traj_.valid(), "Calistir: yorunge (" + std::to_string(traj_.keys.size()) + " anahtar, " +
                                             std::to_string(int(traj_.duration)) + " s)");
        break;
    default: break;
    }
    if (f > 16 && runFrames >= 0) {
        ++runFrames;
        int idx = 0;
        if (playing_) traj_.sample(playTime_, &idx);
        if (playing_ && !midCaptured && playTime_ > traj_.duration * 0.55f && traj_.keys[size_t(idx)].phase == PathPhase::Process) {
            midCaptured = true;
            viewPanels(520.0f);
            camera_.target = dirToPanels * dist * 0.5f + glm::vec3(0, 0, 120.0f);
            capture("33_boyaniyor.png");
        }
        if (!playing_ || runFrames > 4000) {
            check(!playing_, "boyama hareketi tamamlandi");
            // frame-rate independent: sample the trajectory finely and compare the tool tip with the
            // planned polyline of the working segment it belongs to
            float worst = 0.0f;
            for (size_t k = 0; k + 1 < traj_.keys.size(); ++k) {
                const PathPoint &ka = traj_.keys[k], &kb = traj_.keys[k + 1];
                if (ka.phase != PathPhase::Process || kb.phase != PathPhase::Process) continue;
                for (int j = 0; j <= 8; ++j) {
                    const float t = ka.time + (kb.time - ka.time) * float(j) / 8.0f;
                    const glm::vec3 tip = glm::vec3(ik_.toolFrame(*robot_, traj_.sample(t), currentTool())[3]);
                    const glm::vec3 ab = kb.pos - ka.pos;
                    const float u = glm::dot(ab, ab) > 0 ? std::clamp(glm::dot(tip - ka.pos, ab) / glm::dot(ab, ab), 0.0f, 1.0f) : 0.0f;
                    worst = std::max(worst, glm::length(ka.pos + ab * u - tip));
                }
            }
            check(worst < 0.3f, "robot boya yolunu izledi (yorunge boyunca sapma < 0.3 mm)");
            int inside = 0, swings = 0;
            trajectoryChecks(traj_, ik_, *robot_, currentTool(), scene_, {panelA, panelB}, inside, swings);
            check(inside == 0, "takim ucu hicbir anda panellerin icine girmedi (" + std::to_string(inside) + ")");
            check(swings == 0, "takim acikken kol ters donmedi");
            check(!robotColliding(), "carpisma yok");
            viewPanels(330.0f);
            capture("34_bitti.png");
            runFrames = -100000;
        }
    }
    if (runFrames < -99990 && ++runFrames > -99995) {
        std::printf("[scan-test] %s (%d hata)\n", selfTestFailures_ ? "BASARISIZ" : "TUM TESTLER GECTI", selfTestFailures_);
        quitRequested_ = true;
    }
}


// Face selection helpers (L / Ctrl +- / Alt+click / brush) on a sphere, then a surface scan of
// the whole sphere as one continuous surface and a robot run.
void Editor::selfTestSelectInject() {
    ImGuiIO& io = ImGui::GetIO();
    const int f = selfTestFrame_++ - 10; // let the window, robot and layout settle first
    auto check = [&](bool ok, const std::string& what) {
        std::printf("[select-test] %-64s %s\n", what.c_str(), ok ? "OK" : "FAIL");
        if (!ok) selfTestFailures_++;
    };
    auto capture = [&](const char* name) { captureRequest_ = selfTestDir_ + "/" + name; };
    auto screen = [&](const glm::vec3& w) {
        glm::vec2 sp(0.0f);
        camera_.worldToViewport(w, glm::vec2(vpSize_.x, vpSize_.y), sp);
        return ImVec2(vpPos_.x + sp.x, vpPos_.y + sp.y);
    };
    auto move = [&](const glm::vec3& w) {
        const ImVec2 p = screen(w);
        io.AddMousePosEvent(p.x, p.y);
    };
    static uint32_t sphereId = 0;
    static glm::vec3 dir(1, 0, 0);
    static float dist = 460.0f, height = 80.0f;
    static size_t loopCount = 0;
    static int runFrames = 0;
    const float R = 35.0f;
    SceneObject* sph = scene_.find(sphereId);
    const glm::vec3 center = sph ? glm::vec3(sph->matrix()[3]) + glm::vec3(0, 0, R) : glm::vec3(0.0f);
    const glm::vec3 side = glm::normalize(glm::vec3(-dir.y, dir.x, 0.0f));
    auto view = [&](float distance) {
        const glm::vec3 e = glm::normalize(-dir * 0.85f + side * 0.3f + glm::vec3(0, 0, 0.45f));
        camera_.yawDeg = glm::degrees(std::atan2(e.y, e.x));
        camera_.pitchDeg = glm::degrees(std::asin(e.z));
        camera_.target = sph ? glm::vec3(sph->matrix()[3]) + glm::vec3(0, 0, R) : center;
        camera_.distance = distance;
    };
    auto place = [&]() {
        sph->transform = Transform{dir * dist + glm::vec3(0, 0, height), glm::vec3(0.0f), glm::vec3(1.0f)};
        scene_.markDirty();
        scanFromSelection();
        recomputePlan();
        return planReachable_;
    };

    if (f > 2 && !robot_) {
        check(false, "robot yuklenemedi");
        quitRequested_ = true;
        return;
    }
    switch (f) {
    case 3:
        addPrimitive(2);
        sphereId = scene_.objects().back()->id;
        scene_.find(sphereId)->transform = Transform{dir * dist + glm::vec3(0, 0, height), glm::vec3(0.0f), glm::vec3(1.0f)};
        scene_.markDirty();
        setTool(Tool::Face);
        faceMode_ = FaceMode::Planar;
        break;
    case 4: view(260.0f); break;
    case 6:
    case 7: move(center - dir * R * 0.2f); break; // over the sphere, facing the camera
    case 8: io.AddKeyEvent(ImGuiKey_L, true); break;
    case 9: io.AddKeyEvent(ImGuiKey_L, false); break;
    case 11: {
        const size_t n = sph->mesh->triangles.size();
        check(selKind_ == Tool::Face && selFaceMode_ == FaceMode::Triangle && selection_.size() == n,
              "L: imlec altindaki kurenin tamami tek tusla secildi (" + std::to_string(selection_.size()) + " / " + std::to_string(n) + " ucgen)");
        capture("40_bagli_secim.png");
        break;
    }
    // ---- Alt + click: face loop
    case 13:
        io.AddKeyEvent(ImGuiKey_Escape, true);
        break;
    case 14:
        io.AddKeyEvent(ImGuiKey_Escape, false);
        move(center - dir * R * 0.9f + side * R * 0.3f);
        break;
    case 16: io.AddKeyEvent(ImGuiMod_Alt, true); break;
    case 17: io.AddMouseButtonEvent(0, true); break;
    case 18: io.AddMouseButtonEvent(0, false); break;
    case 19: io.AddKeyEvent(ImGuiMod_Alt, false); break;
    case 21: {
        loopCount = selection_.size();
        // UV sphere 48 x 24: a latitude ring has 48 quads (96 triangles), a meridian 22 quads (44)
        check(loopCount == 96 || loopCount == 44, "Alt+tik: dongu secimi (" + std::to_string(loopCount) + " ucgen = tam halka)");
        check(!cameraDrag_, "Alt+tik nesne uzerinde kamerayi dondurmedi");
        capture("41_dongu_secimi.png");
        growShrinkSelection(true);
        const size_t grown = selection_.size();
        check(grown > loopCount, "Ctrl +: secim buyudu (" + std::to_string(grown) + ")");
        growShrinkSelection(false);
        check(selection_.size() == loopCount, "Ctrl -: secim tekrar halkaya kucultuldu");
        break;
    }
    // ---- brush
    case 23:
        clearSelection();
        brushMode_ = true;
        brushRadius_ = 25.0f;
        move(center - dir * R + side * R * 0.5f);
        break;
    case 25: io.AddMouseButtonEvent(0, true); break;
    case 26: move(center - dir * R); break;
    case 27: move(center - dir * R - side * R * 0.5f); break;
    case 28: io.AddMouseButtonEvent(0, false); break;
    case 30: {
        const size_t painted = selection_.size();
        loopCount = painted;
        check(painted > 20 && painted < sph->mesh->triangles.size() / 2, "Firca: surukleyerek " + std::to_string(painted) + " ucgen secildi");
        capture("42_firca_secimi.png");
        io.AddKeyEvent(ImGuiMod_Shift, true);
        io.AddMouseButtonEvent(0, true);
        break;
    }
    case 31: io.AddMouseButtonEvent(0, false); break;
    case 32: io.AddKeyEvent(ImGuiMod_Shift, false); break;
    case 34:
        check(selection_.size() < loopCount, "Shift + firca: secimden cikarildi (" + std::to_string(selection_.size()) + ")");
        brushMode_ = false;
        clearSelection();
        move(center - dir * R * 0.2f);
        break;
    case 36: io.AddKeyEvent(ImGuiKey_L, true); break;
    case 37: io.AddKeyEvent(ImGuiKey_L, false); break;
    // ---- scan the whole sphere as one surface
    case 39: {
        setMotionMode(1);
        for (int i = 0; i < int(library_.tools.size()); ++i)
            if (library_.tools[i].id == "sprey") toolIndex_ = i;
        for (int i = 0; i < int(library_.scans.size()); ++i)
            if (library_.scans[i].id == "zikzak_tarama") selectMotion(i);
        for (auto& p : motion_.params) {
            if (p.name == "standoff") p.value = 40.0;
            if (p.name == "spacing") p.value = 10.0;
        }
        check(selection_.size() == sph->mesh->triangles.size(), "kurenin tamami tekrar secili");
        const glm::vec3 dirs[4] = {{1, 0, 0}, {-1, 0, 0}, {0, 1, 0}, {0, -1, 0}};
        int best = -1;
        glm::vec3 bd = dir;
        float bdist = dist, bh = height;
        for (float d : {450.0f, 480.0f, 420.0f})
            for (float h : {60.0f, 100.0f, 20.0f})
                for (const auto& dd : dirs) {
                    dir = dd;
                    dist = d;
                    height = h;
                    const int ok = place();
                    if (ok > best) {
                        best = ok;
                        bd = dd;
                        bdist = d;
                        bh = h;
                    }
                }
        dir = bd;
        dist = bdist;
        height = bh;
        place();
        std::printf("[select-test] yerlesim: uzaklik %.0f, yukseklik %.0f -> %d / %zu nokta erisilebilir\n", dist, height, planReachable_,
                    planPoints_.size());
        const glm::vec3 c = glm::vec3(sph->matrix()[3]) + glm::vec3(0, 0, R);
        int proc = 0, procReach = 0;
        float maxAimErr = 0.0f, maxTiltSeen = 0.0f, maxStandoffErr = 0.0f;
        for (const auto& p : planPoints_) {
            if (p.phase != PathPhase::Process) continue;
            ++proc;
            if (!p.reachable) continue;
            ++procReach;
            // the spray axis hits the sphere exactly 40 mm in front of the gun ...
            const glm::vec3 hitPt = p.pos + p.dir * 40.0f;
            maxAimErr = std::max(maxAimErr, std::abs(glm::length(hitPt - c) - R));
            maxStandoffErr = std::max(maxStandoffErr, std::abs(glm::length(hitPt - p.pos) - 40.0f));
            // ... and leans at most `tilt` away from the surface normal there
            maxTiltSeen = std::max(maxTiltSeen, glm::degrees(std::acos(std::clamp(glm::dot(-p.dir, glm::normalize(hitPt - c)), -1.0f, 1.0f))));
        }
        const int reachTilt = planReachable_;
        std::printf("[select-test] %d ucgen -> %d yuzey, %d cizgi, %d boya noktasi (%d erisilebilir), atlanan arka taraf %.0f mm2\n",
                    scanStats_.inputTriangles, scanStats_.faces, scanStats_.passes, proc, procReach, scanStats_.skippedArea);
        std::printf("[select-test] nisan hatasi %.3f mm, en buyuk egim %.1f derece\n", maxAimErr, maxTiltSeen);
        check(scanStats_.faces == 1, "2208 ucgenli kure tek surekli yuzey olarak taraniyor");
        check(scanStats_.passes > 3 && scanStats_.passes < 20, "tarama cizgisi sayisi ucgen sayisindan bagimsiz (" + std::to_string(scanStats_.passes) + ")");
        check(scanStats_.skippedArea > 0.0f, "robota bakmayan arka taraf atlandi");
        check(maxAimErr < 0.5f && maxStandoffErr < 0.01f, "tabanca ekseni kure yuzeyine 40 mm'den nisan aliyor (faset payi < 0.5 mm)");
        check(maxTiltSeen <= 30.05f, "egim en fazla 30 derece (desen parametresi)");
        for (auto& p : motion_.params)
            if (p.name == "tilt") p.value = 0.0;
        recomputePlan();
        const int reachStrict = planReachable_;
        for (auto& p : motion_.params)
            if (p.name == "tilt") p.value = 30.0;
        recomputePlan();
        std::printf("[select-test] erisilebilir nokta: egimsiz %d, 30 derece egimle %d / %zu\n", reachStrict, reachTilt, planPoints_.size());
        check(reachTilt > 2 * reachStrict, "egim toleransi 5 eksenli kolun erisimini artiriyor");
        clearSelection();
        view(300.0f);
        break;
    }
    case 42: capture("43_kure_tarama.png"); break;
    case 44:
        startRun();
        playSpeed_ = 10.0f;
        check(playing_ && traj_.valid(), "Calistir: kure boyaniyor");
        break;
    default: break;
    }
    if (f > 44 && runFrames >= 0) {
        ++runFrames;
        if (!playing_ || runFrames > 6000) {
            float worst = 0.0f;
            for (size_t k = 0; k + 1 < traj_.keys.size(); ++k) {
                const PathPoint &ka = traj_.keys[k], &kb = traj_.keys[k + 1];
                if (ka.phase != PathPhase::Process || kb.phase != PathPhase::Process) continue;
                for (int j = 0; j <= 8; ++j) {
                    const float t = ka.time + (kb.time - ka.time) * float(j) / 8.0f;
                    const glm::vec3 tip = glm::vec3(ik_.toolFrame(*robot_, traj_.sample(t), currentTool())[3]);
                    const glm::vec3 ab = kb.pos - ka.pos;
                    const float u = glm::dot(ab, ab) > 0 ? std::clamp(glm::dot(tip - ka.pos, ab) / glm::dot(ab, ab), 0.0f, 1.0f) : 0.0f;
                    worst = std::max(worst, glm::length(ka.pos + ab * u - tip));
                }
            }
            check(!playing_, "boyama hareketi tamamlandi" + (runWarning_.empty() ? std::string() : " (" + runWarning_ + ")"));
            int on = 0;
            for (const auto& k : traj_.keys) on += k.phase == PathPhase::Process && !k.reachable;
            check(on == 0, "erisilemeyen noktalarda tabanca kapali");
            check(worst < 0.5f, "tabanca acikken robot boya yolunu izledi (sapma " + std::to_string(worst) + " mm < 0.5)");
            int inside = 0, swings = 0;
            trajectoryChecks(traj_, ik_, *robot_, currentTool(), scene_, {sphereId}, inside, swings);
            check(inside == 0, "takim ucu hicbir anda kurenin icine girmedi (" + std::to_string(inside) + ")");
            check(swings == 0, "takim acikken kol ters donmedi");
            check(!robotColliding(), "carpisma yok");
            view(300.0f);
            capture("44_kure_bitti.png");
            runFrames = -100000;
        }
    }
    if (runFrames < -99990 && ++runFrames > -99995) {
        std::printf("[select-test] %s (%d hata)\n", selfTestFailures_ ? "BASARISIZ" : "TUM TESTLER GECTI", selfTestFailures_);
        quitRequested_ = true;
    }
}


// Arduino link against the firmware simulator (the real pegasus_arm.ino compiled for the PC).
void Editor::selfTestArduinoInject() {
#ifdef _WIN32
    // the firmware simulator needs a POSIX pseudo terminal; on Windows test with a real board
    // through the Arduino window instead
    std::printf("[arduino-test] Windows'ta desteklenmiyor (simulator POSIX gerektirir)\n");
    quitRequested_ = true;
#else
    using Clock = std::chrono::steady_clock;
    static int step = 0;
    static Clock::time_point t0 = Clock::now(), stepStart = Clock::now();
    static pid_t sim = -1;
    static std::string simPort;
    static float lastBel = 0.0f;
    auto check = [&](bool ok, const std::string& what) {
        std::printf("[arduino-test] %-64s %s\n", what.c_str(), ok ? "OK" : "FAIL");
        if (!ok) selfTestFailures_++;
    };
    auto capture = [&](const char* name) { captureRequest_ = selfTestDir_ + "/" + name; };
    auto elapsed = [&] { return std::chrono::duration<float>(Clock::now() - stepStart).count(); };
    auto next = [&] {
        ++step;
        stepStart = Clock::now();
    };
    auto jointByName = [&](const char* n) {
        if (robot_)
            for (size_t j = 0; j < robot_->joints.size(); ++j)
                if (robot_->joints[j].name == n) return int(j);
        return -1;
    };
    auto boardMatches = [&](float tol) {
        const LinkStatus st = link_->status();
        if (!st.havePositions) return false;
        for (int i = 0; i < kServoChannels; ++i)
            if (std::abs(st.servoDeg[size_t(i)] - servoTarget(i)) > tol) return false;
        return true;
    };
    const int f = selfTestFrame_++;
    if (f < 5) return;
    if (!robot_) {
        check(false, "robot yuklenemedi");
        quitRequested_ = true;
        return;
    }
    const float total = std::chrono::duration<float>(Clock::now() - t0).count();
    if (total > 90.0f) {
        check(false, "zaman asimi (adim " + std::to_string(step) + ")");
        step = 100;
    }
    switch (step) {
    case 0: {
        if (const char* hw = getenv("PEGASUS_TEST_PORT")) { // a real board instead of the simulator
            simPort = hw;
            arduinoCfgOverride_ = selfTestDir_ + "/arduino.pegasus";
            std::filesystem::remove(arduinoCfgOverride_);
            showArduino_ = true;
            std::printf("[arduino-test] gercek kart: %s\n", hw);
            next();
            break;
        }
        // start the simulator next to our executable
        char exe[4096] = {};
        const ssize_t n = readlink("/proc/self/exe", exe, sizeof(exe) - 1);
        const std::string dir = n > 0 ? std::filesystem::path(std::string(exe, size_t(n))).parent_path().string() : ".";
        const std::string bin = dir + "/pegasus_arduino_sim";
        simPort = selfTestDir_ + "/sim-port";
        std::filesystem::remove(simPort);
        sim = fork();
        if (sim == 0) {
            execl(bin.c_str(), bin.c_str(), simPort.c_str(), (char*)nullptr);
            _exit(127);
        }
        arduinoCfgOverride_ = selfTestDir_ + "/arduino.pegasus";
        std::filesystem::remove(arduinoCfgOverride_);
        showArduino_ = true;
        next();
        break;
    }
    case 1:
        if (std::filesystem::exists(simPort)) {
            std::snprintf(portBuf_, sizeof(portBuf_), "%s", simPort.c_str());
            link_->connect(portBuf_, 115200);
            next();
        } else if (elapsed() > 3.0f) {
            check(false, "simulator baslamadi (" + simPort + ")");
            step = 100;
        }
        break;
    case 2: {
        const LinkStatus st = link_->status();
        if (st.state == LinkState::Connected) {
            check(true, "baglandi: " + st.port);
            check(st.firmware == "pegasus-arm 1.1", "firmware tanindi (" + st.firmware + ")");
            check(elapsed() < 3.0f, "kartin acilmasi (READY) + el sikisma < 3 s (" + std::to_string(elapsed()) + " s)");
            next();
        } else if (st.state == LinkState::Error || elapsed() > 6.0f) {
            check(false, "baglanti kurulamadi: " + st.message);
            step = 100;
        }
        break;
    }
    case 3: // power-on / connect: the servos must stay free until HOME
        if (elapsed() > 0.6f) {
            const LinkStatus st = link_->status();
            check(st.havePositions && !st.homed, "baglaninca servolar serbest (HOME verilmedi, hicbir sey surulmuyor)");
            followRobot_ = true; // must be refused by the app
            link_->send("S 1 1200"); // must be refused by the board
            step = 30;
            stepStart = Clock::now();
        }
        break;
    case 30:
        if (elapsed() > 0.5f) {
            const LinkStatus st = link_->status();
            check(!followRobot_, "HOME'dan once canli takip acilamiyor");
            check(st.errors >= 1 && !st.homed && std::abs(st.servoDeg[0] - 90.0f) < 0.05f, "HOME'dan once S komutu reddedildi (ERR 6), servo hareket etmedi");
            check(st.framesSent == 0, "HOME'dan once karta hic hareket gonderilmedi");
            capture("49_arduino_serbest.png");
            link_->send("HOME");
            step = 31;
            stepStart = Clock::now();
        }
        break;
    case 31: {
        const LinkStatus st = link_->status();
        if (st.homed && st.havePositions) {
            check(std::abs(st.servoDeg[0] - 90.0f) < 0.05f && std::abs(st.servoDeg[1] - 150.0f) < 0.05f && std::abs(st.servoDeg[5] - 80.0f) < 0.05f,
                  "HOME: servolar baslangic pozunda (90/150/35/140/85/80)");
            arduinoErrBase_ = st.errors; // the refused S before HOME was an expected error
            step = 32;
            stepStart = Clock::now();
        } else if (elapsed() > 3.0f) {
            check(false, "HOME sonrasi homed bildirilmedi");
            step = 100;
        }
        break;
    }
    case 32:
        if (elapsed() > 0.3f) {
            const LinkStatus st = link_->status();
            check(st.havePositions, "kart konumlari okunuyor (GET)");
            check(st.rttMs > 0.0f && st.rttMs < 100.0f, "PING gecikmesi " + std::to_string(st.rttMs) + " ms");
            capture("50_arduino_bagli.png");
            // map: default channels = drivers in base -> tip order
            check(servoChannels_[0].joint == jointByName("Bel") && servoChannels_[5].joint == jointByName("Kıskaç"),
                  "kanal 1 = Bel, kanal 6 = Kiskac (varsayilan eslesme)");
            setRobotAngle(jointByName("Bel"), 30.0f);
            setRobotAngle(jointByName("Omuz"), -20.0f);
            setRobotAngle(jointByName("Kıskaç"), 10.0f);
            servoSpeed_ = 120.0f;
            sendServoLimits();
            followRobot_ = true;
            if (getenv("ARDDBG")) link_->logStreamFrames = true;
            step = 4;
            stepStart = Clock::now();
        }
        break;
    case 4:
        if (boardMatches(0.15f)) {
            check(true, "canli takip: kart servo acilari modele esit (" + std::to_string(elapsed()) + " s)");
            const LinkStatus st = link_->status();
            check(std::abs(st.servoDeg[0] - 120.0f) < 0.2f && std::abs(st.servoDeg[1] - 70.0f) < 0.2f,
                  "Bel 30 -> servo 120, Omuz -20 -> servo 70 (merkez 90, yon +1)");
            capture("51_arduino_takip.png");
            next();
        } else if (elapsed() > 6.0f) {
            check(false, "canli takip: kart konumlari modele ulasmadi");
            next();
        }
        break;
    case 5: // continuous motion: sweep Bel and make sure the board follows closely
        if (elapsed() < 2.0f) {
            lastBel = 30.0f * std::cos(elapsed() * 1.5f);
            setRobotAngle(jointByName("Bel"), lastBel);
        } else if (elapsed() > 2.6f) {
            check(boardMatches(0.15f), "surekli hareket (50 Hz akis) sonunda kart modelle ayni");
            const LinkStatus st = link_->status();
            check(st.framesSent > 50 && st.errors == arduinoErrBase_,
                  "akis: " + std::to_string(st.framesSent) + " hareket, " + std::to_string(st.errors - arduinoErrBase_) + " hata");
            if (getenv("ARDDBG")) for (const auto& l : link_->log()) std::printf("LOG %s\n", l.c_str());
            // emergency stop during a long move
            servoSpeed_ = 40.0f;
            sendServoLimits();
            setRobotAngle(jointByName("Bel"), -80.0f);
            next();
        }
        break;
    case 6:
        // emergency stop through the real keyboard path: press and release Space
        {
            static int spacePhase = 0;
            if (spacePhase == 0 && elapsed() > 0.5f) {
                ImGui::GetIO().AddKeyEvent(ImGuiKey_Space, true);
                spacePhase = 1;
            } else if (spacePhase == 1) {
                ImGui::GetIO().AddKeyEvent(ImGuiKey_Space, false);
                spacePhase = 2;
            } else if (spacePhase == 2 && elapsed() > 0.9f) {
                check(!followRobot_, "Space: acil durdurma kisayolu takibi kapatti");
                spacePhase = 3;
            }
        }
        if (elapsed() > 1.3f) {
            const LinkStatus a = link_->status();
            static float firstReading = -1.0f;
            if (firstReading < 0.0f) {
                firstReading = a.servoDeg[0];
            } else if (elapsed() > 1.9f) {
                check(!a.moving && std::abs(a.servoDeg[0] - firstReading) < 0.05f && a.servoDeg[0] > 15.0f,
                      "ACIL DURDUR: kol yolun ortasinda durdu (Bel servo " + std::to_string(a.servoDeg[0]) + ")");
                link_->send("S 6 1200");
                next();
            }
        }
        break;
    case 7:
        if (link_->status().servoDeg[5] > 119.9f && link_->status().servoDeg[5] < 120.1f) {
            check(true, "elle surme (S 6 1200): kiskac servosu 120 derece");
            // calibration round trip
            servoChannels_[0].center = 95.0f;
            servoChannels_[2].dir = -1.0f;
            servoChannels_[4].minDeg = 20.0f;
            std::string err;
            check(saveArduinoConfig(err), "ayarlar kaydedildi");
            resetServoChannels();
            loadArduinoConfig();
            check(servoChannels_[0].center == 95.0f && servoChannels_[2].dir == -1.0f && servoChannels_[4].minDeg == 20.0f &&
                      servoChannels_[0].joint == jointByName("Bel"),
                  "ayarlar dosyadan geri yuklendi (merkez, yon, sinir, eklem)");
            // limits are enforced on the board too
            link_->send("L 1 600 1200");
            link_->send("S 1 1700");
            next();
        } else if (elapsed() > 4.0f) {
            check(false, "elle surme: kiskac servosu hedefe gitmedi");
            next();
        }
        break;
    case 8:
        if (elapsed() > 3.2f) {
            check(std::abs(link_->status().servoDeg[0] - 120.0f) < 0.1f, "kart tarafinda sinir: S 1 1700 -> 120 derecede kaldi");
            capture("52_arduino_kalibrasyon.png");
            link_->send("L 1 0 1800");
            link_->send("DETACH");
            step = 21;
            stepStart = Clock::now();
        }
        break;
    case 21:
        if (elapsed() > 0.6f) {
            const LinkStatus st = link_->status();
            check(!st.homed, "DETACH sonrasi servolar serbest, yeniden HOME gerekiyor");
            followRobot_ = true;
            updateArduino();
            check(!followRobot_, "DETACH sonrasi canli takip acilamiyor");
            if (sim > 0) {
                kill(sim, SIGTERM); // "cable pulled"
                step = 9;
                stepStart = Clock::now();
            } else { // real board: we cannot pull the cable, disconnect instead
                link_->disconnect();
                step = 20;
                stepStart = Clock::now();
            }
        }
        break;
    case 9: {
        const LinkStatus st = link_->status();
        if (st.state == LinkState::Error) {
            check(st.message.find("koptu") != std::string::npos, "kablo cekildi: algilandi (" + st.message + ")");
            check(elapsed() < 3.0f, "kopma " + std::to_string(elapsed()) + " s icinde fark edildi");
            check(!followRobot_, "kopunca canli takip kapandi");
            capture("53_arduino_koptu.png");
            link_->connect("/dev/pegasus-yok", 115200);
            next();
        } else if (elapsed() > 5.0f) {
            check(false, "kopma algilanmadi");
            next();
        }
        break;
    }
    case 10: {
        const LinkStatus st = link_->status();
        if (st.state == LinkState::Error) {
            check(st.message.find("bulunamad") != std::string::npos, "olmayan port: anlasilir hata (" + st.message + ")");
            // a port we may not open (permission) - only if the machine has one
            if (access("/dev/ttyS0", F_OK) == 0 && access("/dev/ttyS0", R_OK | W_OK) != 0) {
                link_->connect("/dev/ttyS0", 115200);
                next();
            } else {
                step = 12;
            }
        } else if (elapsed() > 3.0f) {
            check(false, "olmayan port hatasi gelmedi");
            step = 100;
        }
        break;
    }
    case 11: {
        const LinkStatus st = link_->status();
        if (st.state == LinkState::Error && elapsed() > 0.2f) {
            check(st.message.find("dialout") != std::string::npos, "izin yok: dialout grubu aciklamasi gosteriliyor");
            capture("54_arduino_izin.png");
            step = 12;
        } else if (elapsed() > 3.0f) {
            check(false, "izin hatasi gelmedi");
            step = 12;
        }
        break;
    }
    case 12:
        step = 100;
        break;
    case 20:
        if (link_->status().state == LinkState::Disconnected) {
            check(true, "baglanti kesildi (gercek kart)");
            check(!followRobot_, "takip kapali");
            link_->connect("/dev/pegasus-yok", 115200);
            step = 10;
            stepStart = Clock::now();
        } else if (elapsed() > 3.0f) {
            check(false, "baglanti kesilemedi");
            step = 100;
        }
        break;
    default:
        break;
    }
    if (step == 100) {
        if (sim > 0) {
            kill(sim, SIGTERM);
            waitpid(sim, nullptr, 0);
            sim = -1;
        }
        std::printf("[arduino-test] %s (%d hata)\n", selfTestFailures_ ? "BASARISIZ" : "TUM TESTLER GECTI", selfTestFailures_);
        quitRequested_ = true;
        step = 101;
    }
#endif
}

} // namespace pegasus
