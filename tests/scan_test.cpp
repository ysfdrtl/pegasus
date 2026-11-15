// Surface scan planner (spray painting): pass layout, edge offset, standoff, curved and closed
// surfaces, patches / creases and obstacle-free links.
#include "check.h"

#include "motion/SurfaceScan.h"
#include "scene/Mesh.h"
#include "scene/Primitives.h"

#include <algorithm>
#include <cmath>
#include <functional>
#include <map>
#include <set>

using namespace pegasus;

namespace {

// grid of square cells (size h) in the XY plane, normal +Z
ScanFace gridFace(float w, float hgt, float h, const std::function<bool(float, float)>& keep) {
    ScanFace f;
    std::map<std::pair<int, int>, uint32_t> idx;
    auto vid = [&](int i, int j) {
        auto it = idx.find({i, j});
        if (it != idx.end()) return it->second;
        f.verts.push_back(glm::vec3(float(i) * h, float(j) * h, 0.0f));
        return idx[{i, j}] = uint32_t(f.verts.size() - 1);
    };
    for (int i = 0; i < int(w / h); ++i)
        for (int j = 0; j < int(hgt / h); ++j) {
            if (!keep((float(i) + 0.5f) * h, (float(j) + 0.5f) * h)) continue;
            const uint32_t a = vid(i, j), b = vid(i + 1, j), c = vid(i + 1, j + 1), d = vid(i, j + 1);
            f.tris.push_back({a, b, c});
            f.tris.push_back({a, c, d});
        }
    return f;
}

// UV sphere (outward winding); latMin > -90 keeps only the part above that latitude
ScanFace sphere(float R, int nu, int nv, float latMin = -90.0f) {
    ScanFace f;
    f.verts.push_back({0, 0, -R});
    for (int j = 1; j < nv; ++j)
        for (int i = 0; i < nu; ++i) {
            const float th = float(M_PI) * (-0.5f + float(j) / float(nv)), ph = 2.0f * float(M_PI) * float(i) / float(nu);
            f.verts.push_back({R * std::cos(th) * std::cos(ph), R * std::cos(th) * std::sin(ph), R * std::sin(th)});
        }
    f.verts.push_back({0, 0, R});
    const uint32_t top = uint32_t(f.verts.size() - 1);
    auto id = [&](int j, int i) { return uint32_t(1 + (j - 1) * nu + ((i % nu) + nu) % nu); };
    auto lat = [&](int j) { return -90.0f + 180.0f * float(j) / float(nv); };
    for (int i = 0; i < nu; ++i) {
        if (lat(1) > latMin) f.tris.push_back({0, id(1, i + 1), id(1, i)});
        for (int j = 1; j < nv - 1; ++j) {
            if (lat(j) < latMin - 1e-3f) continue;
            f.tris.push_back({id(j, i), id(j, i + 1), id(j + 1, i + 1)});
            f.tris.push_back({id(j, i), id(j + 1, i + 1), id(j + 1, i)});
        }
        f.tris.push_back({id(nv - 1, i), id(nv - 1, i + 1), top});
    }
    return f;
}

MotionDef makeDef(float offset, float spacing, float standoff, const char* pattern = "zigzag", const char* fx = "0") {
    MotionDef d;
    d.pattern = pattern;
    d.fx = fx;
    auto P = [&](const char* n, double v) {
        MotionParam p;
        p.name = n;
        p.value = v;
        d.params.push_back(p);
    };
    P("offset", offset);
    P("spacing", spacing);
    P("standoff", standoff);
    P("angle", 0);
    P("lift", 15);
    return d;
}
void setParam(MotionDef& d, const char* n, double v) {
    for (auto& p : d.params)
        if (p.name == n) {
            p.value = v;
            return;
        }
    MotionParam p;
    p.name = n;
    p.value = v;
    d.params.push_back(p);
}

struct Run {
    std::vector<PathPoint> pts;
    ScanStats st;
    bool ok = false;
    std::string err;
};
Run run(const std::vector<ScanFace>& faces, const MotionDef& d, const glm::vec3* from = nullptr,
        const std::vector<ScanObstacle>* obstacles = nullptr) {
    CompiledMotion cm;
    std::string errs[CompiledMotion::Count];
    cm.compile(d, errs);
    PlanSettings ps;
    ps.step = 0.5f;
    Run r;
    r.ok = planSurfaceScan(faces, d, cm, ps, r.pts, r.st, r.err, from, obstacles);
    return r;
}
float rectInsideDist(float x, float y, float x0, float y0, float x1, float y1) { return std::min({x - x0, x1 - x, y - y0, y1 - y}); }

} // namespace

int main() {
    char buf[200];

    section("1) dikdortgen 80x60, ofset 5, aralik 10, uzaklik 30");
    {
        const ScanFace f = gridFace(80, 60, 5, [](float, float) { return true; });
        const Run r = run({f}, makeDef(5, 10, 30));
        std::set<int> ys;
        float minEdge = 1e9f, maxZerr = 0.0f;
        for (const auto& p : r.pts) {
            if (p.phase != PathPhase::Process) continue;
            ys.insert(int(std::lround(p.pos.y * 100)));
            minEdge = std::min(minEdge, rectInsideDist(p.pos.x, p.pos.y, 0, 0, 80, 60));
            maxZerr = std::max(maxZerr, std::abs(p.pos.z - 30.0f));
            if (glm::length(p.dir - glm::vec3(0, 0, -1)) > 1e-4f) maxZerr = 99.0f;
        }
        CHECK(r.ok && r.st.passes == 6, "6 gecis (60 - 2x5 = 50 mm, 10 mm aralik)");
        CHECK(ys.size() == 6 && *ys.begin() == 500 && *ys.rbegin() == 5500, "cizgiler y = 5 .. 55, esit aralikli");
        CHECK(std::abs(minEdge - 5.0f) < 0.01f, "kenara en yakin nokta tam 5 mm");
        CHECK(maxZerr < 1e-3f, "tum noktalar yuzeyden 30 mm, takim yuzeye dik");
        CHECK(std::abs(r.st.processLength - 6 * 70.0f) < 0.05f, "gecis uzunlugu 6 x 70 mm");
    }

    section("2) L sekilli yuz, ofset 5");
    {
        auto inL = [](float x, float y) { return y < 40 || x < 40; };
        const Run r = run({gridFace(100, 100, 5, inL)}, makeDef(5, 8, 20));
        float minD = 1e9f;
        bool armA = false, armB = false, outside = false;
        for (const auto& p : r.pts) {
            if (p.phase != PathPhase::Process) continue;
            const float x = p.pos.x, y = p.pos.y;
            if (!inL(x, y)) outside = true;
            float d = std::min({x, y, 100 - x, 100 - y});
            if (x > 40) d = std::min(d, 40 - y);
            if (y > 40) d = std::min(d, 40 - x);
            const float cx = x - 40, cy = y - 40;
            if (cx > 0 && cy > 0) d = std::min(d, std::sqrt(cx * cx + cy * cy));
            minD = std::min(minD, d);
            armA |= x > 60;
            armB |= y > 60;
        }
        CHECK(r.ok && !outside, "tum noktalar L icinde");
        CHECK(minD > 5.0f - 0.02f, "her nokta sinira >= 5 mm (ic kose dahil)");
        CHECK(armA && armB, "iki kol da taraniyor");
    }

    section("3) ortasi delikli kare (100x100, delik 30x30), ofset 4");
    {
        auto keep = [](float x, float y) { return !(x > 35 && x < 65 && y > 35 && y < 65); };
        const Run r = run({gridFace(100, 100, 5, keep)}, makeDef(4, 6, 20));
        float minHole = 1e9f;
        int links = 0;
        for (const auto& p : r.pts) {
            if (p.phase == PathPhase::Link) {
                ++links;
                continue;
            }
            const float dx = std::max({35 - p.pos.x, 0.0f, p.pos.x - 65}), dy = std::max({35 - p.pos.y, 0.0f, p.pos.y - 65});
            minHole = std::min(minHole, std::sqrt(dx * dx + dy * dy));
        }
        CHECK(r.ok && minHole > 4.0f - 0.02f, "delige >= 4 mm mesafe");
        CHECK(links > 0, "delik uzerinden takim kapali gecis var");
    }

    section("4) yarim silindir R=50, uzaklik 25");
    {
        ScanFace f;
        const int na = 36, nl = 20;
        for (int i = 0; i <= na; ++i)
            for (int j = 0; j <= nl; ++j) {
                const float a = float(M_PI) * float(i) / float(na);
                f.verts.push_back({50 * std::cos(a), float(j) * 5.0f, 50 * std::sin(a)});
            }
        for (int i = 0; i < na; ++i)
            for (int j = 0; j < nl; ++j) {
                const uint32_t a = uint32_t(i * (nl + 1) + j), b = uint32_t((i + 1) * (nl + 1) + j), c = b + 1, d = a + 1;
                f.tris.push_back({a, d, c});
                f.tris.push_back({a, c, b});
            }
        {
            const glm::uvec3 t = f.tris[0];
            const glm::vec3 n = glm::cross(f.verts[t.y] - f.verts[t.x], f.verts[t.z] - f.verts[t.x]);
            const glm::vec3 c = (f.verts[t.x] + f.verts[t.y] + f.verts[t.z]) / 3.0f;
            if (glm::dot(n, glm::vec3(c.x, 0, c.z)) < 0)
                for (auto& tr : f.tris) std::swap(tr.y, tr.z);
        }
        const Run r = run({f}, makeDef(3, 8, 25));
        float maxErr = 0.0f, maxDir = 0.0f;
        for (const auto& p : r.pts) {
            if (p.phase != PathPhase::Process) continue;
            const glm::vec3 radial(p.pos.x, 0, p.pos.z);
            maxErr = std::max(maxErr, std::abs(glm::length(radial) - 75.0f));
            maxDir = std::max(maxDir, glm::length(p.dir + glm::normalize(radial)));
        }
        std::snprintf(buf, sizeof(buf), "yuzeyi takip ediyor: eksene uzaklik R + 25 (hata %.3f mm)", maxErr);
        CHECK(r.ok && maxErr < 0.6f, buf);
        CHECK(maxDir < 0.1f, "takim yerel yuzey normaline dik");
    }

    section("5) capraz desen ve dalgali formul");
    {
        const ScanFace f = gridFace(80, 60, 5, [](float, float) { return true; });
        const Run a = run({f}, makeDef(5, 10, 30)), b = run({f}, makeDef(5, 10, 30, "crosshatch"));
        CHECK(b.st.passes == a.st.passes + 8, "capraz: 6 + 8 gecis");
        const Run w = run({f}, makeDef(5, 10, 30, "zigzag", "2*sin(2*pi*s/10)"));
        float maxDev = 0.0f;
        for (const auto& p : w.pts) {
            if (p.phase != PathPhase::Process) continue;
            float dy = std::fmod(p.pos.y - 5.0f + 100.0f, 10.0f);
            dy = std::min(dy, 10 - dy);
            maxDev = std::max(maxDev, dy);
        }
        CHECK(std::abs(maxDev - 2.0f) < 0.05f, "x = 2 sin(...): gecis cizgisinden +-2 mm sapma");
    }

    section("6) iki yuz, farkli duzlemler");
    {
        const ScanFace f1 = gridFace(40, 40, 5, [](float, float) { return true; });
        ScanFace f2 = f1;
        for (auto& v : f2.verts) v = glm::vec3(v.x + 100, 0, v.y);
        for (auto& t : f2.tris) std::swap(t.y, t.z);
        const Run r = run({f1, f2}, makeDef(5, 10, 20));
        bool hasZ = false, hasY = false;
        for (const auto& p : r.pts)
            if (p.phase == PathPhase::Process) {
                hasZ |= std::abs(p.pos.z - 20) < 1e-3f;
                hasY |= std::abs(p.pos.y - 20) < 1e-3f;
            }
        CHECK(r.ok && r.st.faces == 2 && hasZ && hasY, "iki yuz de kendi normali yonunde 20 mm uzaktan taraniyor");
    }

    section("7) cok kucuk yuz");
    {
        const Run r = run({gridFace(10, 10, 5, [](float, float) { return true; })}, makeDef(6, 5, 20));
        CHECK(!r.ok && !r.err.empty(), "ofset alan birakmiyor: anlasilir hata (" + r.err + ")");
    }

    section("8) kapali kure R=40 (2208 ucgen), robot +X yonunde");
    {
        const ScanFace f = sphere(40, 48, 24);
        const glm::vec3 from(400, 0, 100);
        const Run r = run({f}, makeDef(3, 8, 30), &from);
        float maxR = 0, minR = 1e9f, maxDir = 0, maxTurn = 0;
        bool back = false;
        const PathPoint* prev = nullptr;
        for (const auto& p : r.pts) {
            if (p.phase != PathPhase::Process) {
                prev = nullptr;
                continue;
            }
            const float rr = glm::length(p.pos);
            maxR = std::max(maxR, rr);
            minR = std::min(minR, rr);
            maxDir = std::max(maxDir, glm::length(p.dir + glm::normalize(p.pos)));
            back |= glm::dot(p.pos, glm::normalize(from)) < -1e-3f;
            if (prev) maxTurn = std::max(maxTurn, glm::degrees(std::acos(std::clamp(glm::dot(prev->dir, p.dir), -1.0f, 1.0f))));
            prev = &p;
        }
        CHECK(r.ok && r.st.faces == 1, "tum kure tek yuzey olarak taraniyor (ucgen ucgen degil)");
        CHECK(r.st.passes < 20, "gecis sayisi ucgen sayisindan bagimsiz (" + std::to_string(r.st.passes) + ")");
        CHECK(!back && r.st.skippedArea > 0, "robota bakmayan arka yari atlandi");
        CHECK(minR > 69.5f && maxR < 70.05f, "tabanca merkeze 40 + 30 mm uzakta");
        CHECK(maxDir < 0.03f, "tabanca her noktada yuzeye dik (yumusatilmis normal)");
        std::snprintf(buf, sizeof(buf), "ardisik noktalar arasinda yon sicramasi yok (en fazla %.2f derece)", maxTurn);
        CHECK(maxTurn < 3.0f, buf);
    }

    section("9) kubbe (yarim kure R=50): yuzey uzerinde esit aralik");
    {
        const Run r = run({sphere(50, 64, 32, 0.0f)}, makeDef(5, 10, 20));
        std::vector<glm::vec3> tops;
        float minElev = 1e9f, cur = -1.0f;
        glm::vec3 best(0.0f);
        for (const auto& p : r.pts) {
            if (p.phase != PathPhase::Process) {
                if (cur >= 0) tops.push_back(best);
                cur = -1.0f;
                continue;
            }
            const glm::vec3 q = glm::normalize(p.pos) * 50.0f;
            minElev = std::min(minElev, q.z);
            if (q.z > cur) {
                cur = q.z;
                best = q;
            }
        }
        if (cur >= 0) tops.push_back(best);
        std::sort(tops.begin(), tops.end(), [](const glm::vec3& a, const glm::vec3& b) { return std::atan2(a.y, a.z) < std::atan2(b.y, b.z); });
        float minG = 1e9f, maxG = 0.0f;
        for (size_t i = 1; i < tops.size(); ++i) {
            const float g = 50.0f * std::acos(std::clamp(glm::dot(glm::normalize(tops[i]), glm::normalize(tops[i - 1])), -1.0f, 1.0f));
            minG = std::min(minG, g);
            maxG = std::max(maxG, g);
        }
        CHECK(r.ok && r.st.faces == 1, "kubbe tek yuzey");
        std::snprintf(buf, sizeof(buf), "komsu cizgiler yuzey uzerinde %.1f .. %.1f mm (bosluk yok, <= ~10)", minG, maxG);
        CHECK(minG > 6.0f && maxG < 10.6f, buf);
        CHECK(minElev > 4.0f, "ekvator kenarina >= ~5 mm");
    }

    section("10) puruzlu duzlem (0.3 mm gurultu) 80x60");
    {
        ScanFace f = gridFace(80, 60, 2, [](float, float) { return true; });
        unsigned seed = 7;
        auto rnd = [&]() {
            seed = seed * 1103515245u + 12345u;
            return float((seed >> 8) & 0xFFFF) / 65535.0f - 0.5f;
        };
        for (auto& v : f.verts) v.z += 0.6f * rnd();
        const Run r = run({f}, makeDef(5, 10, 30));
        float maxTilt = 0.0f;
        for (const auto& p : r.pts)
            if (p.phase == PathPhase::Process) maxTilt = std::max(maxTilt, glm::degrees(std::acos(std::clamp(-p.dir.z, -1.0f, 1.0f))));
        CHECK(r.ok && r.st.faces == 1 && r.st.passes == 6, "parcalanmiyor: 1 yuzey, 6 gecis");
        std::snprintf(buf, sizeof(buf), "takim fasetlere gore sallanmiyor (egim en fazla %.1f derece < 8)", maxTilt);
        CHECK(maxTilt < 8.0f, buf);
    }

    section("11) ayni nesnede 90 derece kose");
    {
        ScanFace f = gridFace(40, 40, 5, [](float, float) { return true; });
        const size_t nv = f.verts.size();
        ScanFace g = f;
        for (auto& v : g.verts) v = glm::vec3(v.x, 0, -v.y);
        for (auto& t : g.tris) std::swap(t.y, t.z);
        for (const auto& v : g.verts) f.verts.push_back(v);
        for (auto t : g.tris) {
            for (int k = 0; k < 3; ++k) {
                const glm::vec3 p = g.verts[t[k]];
                uint32_t w = uint32_t(nv + t[k]);
                if (std::abs(p.z) < 1e-4f)
                    for (size_t q = 0; q < nv; ++q)
                        if (glm::length(f.verts[q] - p) < 1e-4f) {
                            w = uint32_t(q);
                            break;
                        }
                t[k] = w;
            }
            f.tris.push_back(t);
        }
        const Run a = run({f}, makeDef(5, 10, 20));
        MotionDef d = makeDef(5, 10, 20);
        setParam(d, "crease", 180);
        const Run b = run({f}, d);
        CHECK(a.ok && a.st.faces == 2, "keskin kose: iki ayri yuzey parcasi");
        CHECK(b.ok && b.st.faces == 1, "kirilma acisi 180: tek parca");
    }

    section("12) kupun bitisik iki yuzu: gecis yolu kupun disinda");
    for (int vv = 0; vv < 16; ++vv) {
        const int variant = vv % 4;
        const float standoff = (vv / 4) % 2 == 0 ? 20.0f : 6.0f;
        const bool twoSides = vv >= 8;
        MeshPtr box = makeBox(glm::vec3(40.0f));
        const glm::vec3 sideN[4] = {{1, 0, 0}, {-1, 0, 0}, {0, 1, 0}, {0, -1, 0}};
        const glm::vec3 other = twoSides ? sideN[(variant + 2) % 4] : glm::vec3(0, 0, 1);
        ScanFace f;
        std::map<uint32_t, uint32_t> remap;
        for (uint32_t t = 0; t < box->triangles.size(); ++t) {
            const glm::vec3 n = box->triNormals[t];
            if (glm::dot(n, other) < 0.9f && glm::dot(n, sideN[variant]) < 0.9f) continue;
            glm::uvec3 idx;
            for (int k = 0; k < 3; ++k) {
                const uint32_t v = box->triangles[t][k];
                auto it = remap.find(v);
                if (it == remap.end()) {
                    it = remap.emplace(v, uint32_t(f.verts.size())).first;
                    f.verts.push_back(box->positions[v]);
                }
                idx[k] = it->second;
            }
            f.tris.push_back(idx);
        }
        const std::vector<ScanObstacle> obs = {{box.get(), glm::mat4(1.0f)}};
        const Run r = run({f}, makeDef(5, 10, standoff), nullptr, &obs);
        float worst = 1e9f;
        bool inside = false;
        for (size_t i = 1; i < r.pts.size(); ++i)
            for (int j = 0; j <= 10; ++j) {
                const glm::vec3 p = glm::mix(r.pts[i - 1].pos, r.pts[i].pos, float(j) / 10.0f);
                const float dx = std::max(std::abs(p.x) - 20.0f, 0.0f), dy = std::max(std::abs(p.y) - 20.0f, 0.0f);
                const float dz = std::max({-p.z, p.z - 40.0f, 0.0f});
                worst = std::min(worst, std::sqrt(dx * dx + dy * dy + dz * dz));
                inside |= std::abs(p.x) < 20 && std::abs(p.y) < 20 && p.z > 0 && p.z < 40;
            }
        std::snprintf(buf, sizeof(buf), "%s %d, uzaklik %2.0f: tum yol kupun disinda (en yakin %.1f mm)", twoSides ? "yan+yan" : "ust+yan", variant,
                      standoff, worst);
        CHECK(r.ok && r.st.faces == 2 && !inside && worst > (standoff < 10 ? 3.5f : 4.0f), buf);
    }
    return pegasus_test::finish("scan_test");
}
