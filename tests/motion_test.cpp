// Motion library files, compiled formulas and edge path sampling.
#include "check.h"

#include "motion/MotionLibrary.h"
#include "motion/PathPlanner.h"
#include "motion/Seam.h"

#include <cmath>

using namespace pegasus;

int main() {
    section("1) kutuphane dosyalari");
    MotionLibrary lib;
    const bool ok = lib.load(PEGASUS_LIBRARY_DIR);
    CHECK(ok, std::string("library/ yuklendi: ") + PEGASUS_LIBRARY_DIR);
    CHECK(lib.motions.size() >= 5, "kenar hareketleri: " + std::to_string(lib.motions.size()));
    CHECK(lib.scans.size() >= 4, "tarama desenleri: " + std::to_string(lib.scans.size()));
    CHECK(lib.tools.size() >= 4, "takim basliklari: " + std::to_string(lib.tools.size()));
    CHECK(lib.warnings.empty(), "uyari yok" + (lib.warnings.empty() ? std::string() : ": " + lib.warnings.front()));
    int compiled = 0;
    for (auto* set : {&lib.motions, &lib.scans})
        for (const auto& m : *set) {
            CompiledMotion cm;
            std::string errs[CompiledMotion::Count];
            if (cm.compile(m, errs)) ++compiled;
            else CHECK(false, m.id + ": formul derlenemedi: " + errs[0] + errs[1] + errs[2]);
        }
    CHECK(compiled == int(lib.motions.size() + lib.scans.size()), "tum formuller derleniyor (" + std::to_string(compiled) + ")");

    section("2) iki kup arasi dikis (0.5 mm aralik)");
    // two vertical edges 0.5 mm apart, faces looking towards -Y and outwards
    std::vector<SeamEdgeGeom> edges(2);
    edges[0].a = {0, 0, 0};
    edges[0].b = {0, 0, 10};
    edges[0].faceNormals = {{0, -1, 0}, {-1, 0, 0}};
    edges[1].a = {0.5f, 0, 0};
    edges[1].b = {0.5f, 0, 10};
    edges[1].faceNormals = {{0, -1, 0}, {1, 0, 0}};
    Seam seam;
    std::string err;
    CHECK(buildSeam(edges, false, glm::vec3(0, 1, 0), seam, err), "dikis olustu " + err);
    CHECK(std::abs(seam.length - 10.0f) < 1e-4f && std::abs(seam.gap - 0.5f) < 1e-4f, "uzunluk 10 mm, aralik 0.5 mm");
    CHECK(std::abs(seam.start.x - 0.25f) < 1e-4f && seam.start.z < 1e-4f && seam.Y.z > 0.99f, "tam ortadan, alttan uste");
    CHECK(glm::dot(seam.N, glm::vec3(0, -1, 0)) > 0.99f, "normal yuzlerden disa (-Y)");

    section("3) duz hareket: 1 mm yukaridan gecen dogru");
    const MotionDef* duz = nullptr;
    for (const auto& m : lib.motions)
        if (m.id == "duz") duz = &m;
    CHECK(duz != nullptr, "duz.pmotion bulundu");
    if (duz) {
        CompiledMotion cm;
        std::string errs[CompiledMotion::Count];
        cm.compile(*duz, errs);
        PlanSettings ps;
        ps.step = 0.25f;
        std::vector<PathPoint> pts;
        CHECK(samplePath(seam, *duz, cm, ps, pts, err), "yol orneklendi");
        float maxErr = 0.0f;
        for (const auto& p : pts) {
            const glm::vec3 want = seam.start + seam.Y * p.s + seam.N * float(duz->paramValue("h", 1.0));
            maxErr = std::max(maxErr, glm::length(p.pos - want));
        }
        CHECK(pts.size() == 41 && maxErr < 1e-4f, "41 nokta, hepsi dikisin " + std::to_string(duz->paramValue("h", 1.0)) + " mm ustunde");
    }

    section("4) sinus formulu");
    {
        MotionDef d;
        d.fx = "A * sin(2 * pi * s / W)";
        MotionParam a, w;
        a.name = "A";
        a.value = 2.0;
        w.name = "W";
        w.value = 5.0;
        d.params = {a, w};
        CompiledMotion cm;
        std::string errs[CompiledMotion::Count];
        CHECK(cm.compile(d, errs), "A*sin(2*pi*s/W) derlendi");
        glm::vec3 off;
        float wd, td;
        cm.evaluate(d, 1.25, 0.0, 10.0, 5.0, off, wd, td);
        CHECK(std::abs(off.x - 2.0f) < 1e-5f, "s = W/4: x = A");
    }
    return pegasus_test::finish("motion_test");
}
