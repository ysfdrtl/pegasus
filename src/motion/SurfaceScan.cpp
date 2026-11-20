#include "motion/SurfaceScan.h"

#include "scene/Mesh.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <numeric>
#include <unordered_map>

namespace pegasus {

namespace {

uint64_t edgeKey(uint32_t a, uint32_t b) {
    if (a > b) std::swap(a, b);
    return (uint64_t(a) << 32) | b;
}

struct UnionFind {
    std::vector<uint32_t> p;
    explicit UnionFind(size_t n) : p(n) { std::iota(p.begin(), p.end(), 0u); }
    uint32_t find(uint32_t x) {
        while (p[x] != x) x = p[x] = p[p[x]];
        return x;
    }
    void unite(uint32_t a, uint32_t b) { p[find(a)] = find(b); }
};

// One continuous surface piece to be painted.
struct Patch {
    std::vector<glm::vec3> V, VN;  // vertices, smoothed vertex normals
    std::vector<glm::uvec3> T;
    std::vector<glm::vec3> TN;     // unit triangle normals
    std::vector<float> TA;         // triangle areas
    std::vector<std::pair<glm::vec3, glm::vec3>> boundary;
    glm::vec3 N{0, 0, 1}, C{0.0f};
    float area = 0.0f;
};

// Uniform grid over the boundary segments for "closer than offset?" queries.
class BoundaryGrid {
public:
    void build(const std::vector<std::pair<glm::vec3, glm::vec3>>& segs, float cell) {
        segs_ = &segs;
        cell_ = std::max(cell, 0.5f);
        for (uint32_t i = 0; i < segs.size(); ++i) {
            const glm::ivec3 lo = cellOf(glm::min(segs[i].first, segs[i].second));
            const glm::ivec3 hi = cellOf(glm::max(segs[i].first, segs[i].second));
            for (int x = lo.x; x <= hi.x; ++x)
                for (int y = lo.y; y <= hi.y; ++y)
                    for (int z = lo.z; z <= hi.z; ++z) cells_[key(x, y, z)].push_back(i);
        }
    }
    bool closerThan(const glm::vec3& p, float r) const {
        if (r <= 0.0f || !segs_) return false;
        const glm::ivec3 lo = cellOf(p - glm::vec3(r)), hi = cellOf(p + glm::vec3(r));
        const float rr = std::max(0.0f, r - 1e-4f), r2 = rr * rr;
        for (int x = lo.x; x <= hi.x; ++x)
            for (int y = lo.y; y <= hi.y; ++y)
                for (int z = lo.z; z <= hi.z; ++z) {
                    auto it = cells_.find(key(x, y, z));
                    if (it == cells_.end()) continue;
                    for (uint32_t i : it->second) {
                        const glm::vec3 a = (*segs_)[i].first, ab = (*segs_)[i].second - a;
                        const float l2 = glm::dot(ab, ab);
                        const float t = l2 > 0.0f ? std::clamp(glm::dot(p - a, ab) / l2, 0.0f, 1.0f) : 0.0f;
                        const glm::vec3 d = p - (a + ab * t);
                        if (glm::dot(d, d) < r2) return true;
                    }
                }
        return false;
    }

private:
    glm::ivec3 cellOf(const glm::vec3& p) const { return glm::ivec3(glm::floor(p / cell_)); }
    static int64_t key(int x, int y, int z) {
        return (int64_t(x & 0x1FFFFF) << 42) | (int64_t(y & 0x1FFFFF) << 21) | int64_t(z & 0x1FFFFF);
    }
    const std::vector<std::pair<glm::vec3, glm::vec3>>* segs_ = nullptr;
    float cell_ = 1.0f;
    std::unordered_map<int64_t, std::vector<uint32_t>> cells_;
};

// Surface curve (intersection of a plane with the patch), parameterised by arc length.
struct Curve {
    std::vector<glm::vec3> P, Nn;
    std::vector<float> S;
    bool closed = false;
    float length() const { return S.empty() ? 0.0f : S.back(); }
    void finish() {
        if (closed && !P.empty()) {
            P.push_back(P.front());
            Nn.push_back(Nn.front());
        }
        S.assign(P.size(), 0.0f);
        for (size_t i = 1; i < P.size(); ++i) S[i] = S[i - 1] + glm::length(P[i] - P[i - 1]);
    }
    float wrap(float s) const {
        const float L = length();
        if (L <= 0.0f) return 0.0f;
        if (closed) {
            s = std::fmod(s, L);
            if (s < 0.0f) s += L;
            return s;
        }
        return std::clamp(s, 0.0f, L);
    }
    void eval(float s, glm::vec3& pos, glm::vec3& n) const {
        s = wrap(s);
        size_t i = size_t(std::upper_bound(S.begin(), S.end(), s) - S.begin());
        i = std::clamp<size_t>(i, 1, P.size() - 1);
        const float seg = S[i] - S[i - 1];
        const float t = seg > 0.0f ? (s - S[i - 1]) / seg : 0.0f;
        pos = glm::mix(P[i - 1], P[i], t);
        n = glm::mix(Nn[i - 1], Nn[i], t);
        const float l = glm::length(n);
        n = l > 1e-9f ? n / l : glm::vec3(0, 0, 1);
    }
};

// All curves of the plane dot(p, D) = h.
std::vector<Curve> slice(const Patch& P, const glm::vec3& D, float h) {
    std::vector<float> sd(P.V.size());
    for (size_t i = 0; i < P.V.size(); ++i) {
        float s = glm::dot(P.V[i], D) - h;
        if (std::abs(s) < 1e-6f) s = 1e-6f; // no vertex exactly on the plane
        sd[i] = s;
    }
    std::unordered_map<uint64_t, uint32_t> pointOf;
    std::vector<glm::vec3> pos, nrm;
    std::vector<std::vector<uint32_t>> adj;
    auto crossing = [&](uint32_t a, uint32_t b) {
        const uint64_t k = edgeKey(a, b);
        auto [it, inserted] = pointOf.try_emplace(k, uint32_t(pos.size()));
        if (inserted) {
            const float t = sd[a] / (sd[a] - sd[b]);
            pos.push_back(glm::mix(P.V[a], P.V[b], t));
            nrm.push_back(glm::mix(P.VN[a], P.VN[b], t));
            adj.emplace_back();
        }
        return it->second;
    };
    for (const glm::uvec3& t : P.T) {
        uint32_t cut[2];
        int n = 0;
        for (int k = 0; k < 3; ++k) {
            const uint32_t a = t[k], b = t[(k + 1) % 3];
            if ((sd[a] > 0.0f) != (sd[b] > 0.0f)) cut[n++] = crossing(a, b);
        }
        if (n != 2 || cut[0] == cut[1]) continue;
        adj[cut[0]].push_back(cut[1]);
        adj[cut[1]].push_back(cut[0]);
    }
    std::vector<Curve> curves;
    std::vector<char> used(pos.size(), 0);
    auto walk = [&](uint32_t start, bool closedHint) {
        Curve c;
        uint32_t prev = 0xFFFFFFFFu, cur = start;
        while (true) {
            used[cur] = 1;
            c.P.push_back(pos[cur]);
            c.Nn.push_back(nrm[cur]);
            uint32_t next = 0xFFFFFFFFu;
            for (uint32_t q : adj[cur])
                if (q != prev && !used[q]) {
                    next = q;
                    break;
                }
            if (next == 0xFFFFFFFFu) {
                c.closed = closedHint && std::find(adj[cur].begin(), adj[cur].end(), start) != adj[cur].end() && c.P.size() > 2;
                break;
            }
            prev = cur;
            cur = next;
        }
        if (c.P.size() >= 2) {
            c.finish();
            if (c.length() > 1e-4f) curves.push_back(std::move(c));
        }
    };
    for (uint32_t i = 0; i < pos.size(); ++i)
        if (!used[i] && adj[i].size() == 1) walk(i, false); // open curves start at the patch boundary
    for (uint32_t i = 0; i < pos.size(); ++i)
        if (!used[i] && !adj[i].empty()) walk(i, true); // remaining: closed loops
    return curves;
}

// Splits the input into patches (crease angle, connectivity), merges tiny pieces, and prepares
// normals / boundaries. Triangles facing away from the chosen spray direction are dropped.
std::vector<Patch> buildPatches(const std::vector<ScanFace>& faces, float creaseDeg, float minArea, const glm::vec3* sprayFrom,
                                ScanStats& stats) {
    std::vector<Patch> patches;
    const float cosCrease = std::cos(glm::radians(std::clamp(creaseDeg, 0.0f, 180.0f)));
    for (const ScanFace& f : faces) {
        const size_t nt = f.tris.size();
        stats.inputTriangles += int(nt);
        std::vector<glm::vec3> tn(nt);
        std::vector<float> ta(nt);
        for (size_t t = 0; t < nt; ++t) {
            const glm::uvec3 tr = f.tris[t];
            const glm::vec3 c = glm::cross(f.verts[tr.y] - f.verts[tr.x], f.verts[tr.z] - f.verts[tr.x]);
            const float l = glm::length(c);
            ta[t] = 0.5f * l;
            tn[t] = l > 0.0f ? c / l : glm::vec3(0.0f);
        }
        std::unordered_map<uint64_t, std::vector<uint32_t>> edgeTris;
        edgeTris.reserve(nt * 2);
        for (uint32_t t = 0; t < nt; ++t)
            for (int k = 0; k < 3; ++k) edgeTris[edgeKey(f.tris[t][k], f.tris[t][(k + 1) % 3])].push_back(t);
        UnionFind uf(nt);
        for (const auto& [k, ts] : edgeTris)
            if (ts.size() == 2 && glm::dot(tn[ts[0]], tn[ts[1]]) >= cosCrease - 1e-6f) uf.unite(ts[0], ts[1]);
        // merge pieces that are too small to be painted on their own into a neighbour
        auto compAreas = [&] {
            std::unordered_map<uint32_t, float> a;
            for (uint32_t t = 0; t < nt; ++t) a[uf.find(t)] += ta[t];
            return a;
        };
        for (int pass = 0; pass < 3; ++pass) {
            const auto areas = compAreas();
            bool changed = false;
            for (const auto& [k, ts] : edgeTris) {
                if (ts.size() != 2) continue;
                const uint32_t a = uf.find(ts[0]), b = uf.find(ts[1]);
                if (a == b) continue;
                const float aa = areas.at(a), ab = areas.at(b);
                if (std::min(aa, ab) < minArea) {
                    uf.unite(a, b);
                    changed = true;
                }
            }
            if (!changed) break;
        }
        std::unordered_map<uint32_t, std::vector<uint32_t>> groups;
        for (uint32_t t = 0; t < nt; ++t)
            if (ta[t] > 0.0f) groups[uf.find(t)].push_back(t);
        std::vector<std::vector<uint32_t>> ordered;
        for (auto& [r, ts] : groups) ordered.push_back(std::move(ts));
        std::sort(ordered.begin(), ordered.end(), [](const auto& a, const auto& b) { return a.front() < b.front(); });

        for (const auto& ts : ordered) {
            glm::vec3 nsum(0.0f), csum(0.0f);
            float area = 0.0f;
            for (uint32_t t : ts) {
                const glm::uvec3 tr = f.tris[t];
                nsum += tn[t] * ta[t];
                csum += (f.verts[tr.x] + f.verts[tr.y] + f.verts[tr.z]) / 3.0f * ta[t];
                area += ta[t];
            }
            if (area <= 0.0f) continue;
            Patch P;
            P.C = csum / area;
            if (glm::length(nsum) >= 0.35f * area) {
                P.N = glm::normalize(nsum);
            } else if (sprayFrom && glm::length(*sprayFrom - P.C) > 1e-3f) {
                P.N = glm::normalize(*sprayFrom - P.C); // closed / wrapped surface: paint the side facing the robot
            } else if (glm::length(nsum) > 1e-6f * area) {
                P.N = glm::normalize(nsum);
            } else {
                P.N = glm::vec3(0, 0, 1);
            }
            // keep the triangles that can be sprayed from the N side
            std::unordered_map<uint32_t, uint32_t> remap;
            for (uint32_t t : ts) {
                if (glm::dot(tn[t], P.N) <= 0.05f) {
                    stats.skippedArea += ta[t];
                    continue;
                }
                glm::uvec3 idx;
                for (int k = 0; k < 3; ++k) {
                    const uint32_t v = f.tris[t][k];
                    auto [it, inserted] = remap.try_emplace(v, uint32_t(P.V.size()));
                    if (inserted) P.V.push_back(f.verts[v]);
                    idx[k] = it->second;
                }
                P.T.push_back(idx);
                P.TN.push_back(tn[t]);
                P.TA.push_back(ta[t]);
                P.area += ta[t];
            }
            if (P.T.empty()) continue;
            // smoothed vertex normals (area weighted)
            P.VN.assign(P.V.size(), glm::vec3(0.0f));
            for (size_t t = 0; t < P.T.size(); ++t)
                for (int k = 0; k < 3; ++k) P.VN[P.T[t][k]] += P.TN[t] * P.TA[t];
            for (auto& n : P.VN) {
                const float l = glm::length(n);
                n = l > 0.0f ? n / l : P.N;
            }
            // boundary of the kept piece
            std::unordered_map<uint64_t, int> cnt;
            for (const auto& t : P.T)
                for (int k = 0; k < 3; ++k) cnt[edgeKey(t[k], t[(k + 1) % 3])]++;
            for (const auto& [k, c] : cnt)
                if (c == 1) P.boundary.push_back({P.V[uint32_t(k >> 32)], P.V[uint32_t(k & 0xFFFFFFFFu)]});
            patches.push_back(std::move(P));
        }
    }
    return patches;
}

// Average normal over a short window of the curve (removes facet-to-facet wobble).
glm::vec3 smoothNormal(const Curve& c, float s, float w) {
    glm::vec3 sum(0.0f), p, n;
    for (int i = -2; i <= 2; ++i) {
        c.eval(s + w * 0.5f * float(i), p, n);
        sum += n;
    }
    const float l = glm::length(sum);
    if (l > 1e-6f) return sum / l;
    c.eval(s, p, n);
    return n;
}

} // namespace

bool planSurfaceScan(const std::vector<ScanFace>& faces, const MotionDef& def, const CompiledMotion& motion,
                     const PlanSettings& settings, std::vector<PathPoint>& out, ScanStats& stats, std::string& err,
                     const glm::vec3* sprayFrom, const std::vector<ScanObstacle>* obstacles) {
    out.clear();
    stats = ScanStats{};
    if (faces.empty()) {
        err = "Önce Yüzey aracıyla taranacak yüz(ler)i seçip tarama alanını oluşturun.";
        return false;
    }
    const float offset = std::max(0.0f, float(def.paramValue("offset", 5.0)));
    const float spacing = std::max(0.05f, float(def.paramValue("spacing", 10.0)));
    const float standoff = float(def.paramValue("standoff", 50.0));
    const float angle = float(def.paramValue("angle", 0.0));
    const float lift = std::max(0.0f, float(def.paramValue("lift", 20.0)));
    const float crease = float(def.paramValue("crease", 60.0));
    const float tilt = std::clamp(float(def.paramValue("tilt", 0.0)), 0.0f, 89.0f);
    const bool follow = def.orient != "fixed";
    const float step = std::max(settings.step, 0.05f);
    const float smoothW = std::clamp(0.5f * spacing, 1.0f, 10.0f);

    std::vector<Patch> patches = buildPatches(faces, crease, spacing * spacing, sprayFrom, stats);
    if (stats.skippedArea > 1.0f) {
        char buf[200];
        std::snprintf(buf, sizeof(buf), "Ana püskürtme yönüne bakmayan %.0f mm² (arka taraf) taranmadı.", stats.skippedArea);
        stats.warnings.push_back(buf);
    }

    // links (tool off) between pieces of work
    auto addLine = [&](const glm::vec3& a, const glm::vec3& b, const glm::vec3& da, const glm::vec3& db) {
        const int n = std::max(1, int(std::ceil(glm::length(b - a) / (step * 4.0f))));
        for (int i = 1; i < n; ++i) {
            PathPoint p;
            p.phase = PathPhase::Link;
            p.maxTilt = tilt > 0.0f ? 180.0f : 0.0f; // tool off: only the position matters
            const float t = float(i) / float(n);
            p.pos = glm::mix(a, b, t);
            p.dir = glm::normalize(glm::mix(da, db, t));
            out.push_back(p);
        }
    };
    // ---- obstacle-aware links: a segment is free if it (and 4 parallel copies `clear` mm around it)
    // does not cross any obstacle surface and does not start inside one
    const float clear = std::max(5.0f, 0.25f * lift);
    auto segmentFree = [&](const glm::vec3& a, const glm::vec3& b) { return !obstacles || segmentClear(*obstacles, a, b, clear); };
    auto pathFree = [&](const std::vector<glm::vec3>& pts) {
        for (size_t i = 1; i < pts.size(); ++i)
            if (!segmentFree(pts[i - 1], pts[i])) return false;
        return true;
    };
    auto pathLen = [](const std::vector<glm::vec3>& pts) {
        float l = 0.0f;
        for (size_t i = 1; i < pts.size(); ++i) l += glm::length(pts[i] - pts[i - 1]);
        return l;
    };
    // shortest free route up0 -> ... -> up1 through one or two detour waypoints
    auto route = [&](const glm::vec3& up0, const glm::vec3& up1, const glm::vec3& n0, const glm::vec3& n1) {
        std::vector<glm::vec3> best = {up0, up1};
        if (pathFree(best)) return best;
        std::vector<glm::vec3> dirs;
        auto addDir = [&](glm::vec3 d) {
            if (glm::length(d) < 0.1f) return;
            d = glm::normalize(d);
            for (const auto& e : dirs)
                if (glm::dot(e, d) > 0.98f) return;
            dirs.push_back(d);
        };
        addDir(n0 + n1);
        addDir(n0);
        addDir(n1);
        addDir(glm::vec3(0, 0, 1));
        if (sprayFrom) addDir(*sprayFrom - 0.5f * (up0 + up1));
        const glm::vec3 mid = 0.5f * (up0 + up1);
        const float base = std::max({lift, clear * 2.0f, 10.0f});
        float bestLen = 1e30f;
        bool found = false;
        for (float k : {1.0f, 2.0f, 3.0f, 5.0f, 8.0f, 12.0f}) {
            for (const auto& d : dirs) {
                std::vector<glm::vec3> c1 = {up0, mid + d * (base * k), up1};
                if (pathFree(c1) && pathLen(c1) < bestLen) {
                    best = c1;
                    bestLen = pathLen(c1);
                    found = true;
                }
                std::vector<glm::vec3> c2 = {up0, up0 + d * (base * k), up1 + d * (base * k), up1};
                if (pathFree(c2) && pathLen(c2) < bestLen) {
                    best = c2;
                    bestLen = pathLen(c2);
                    found = true;
                }
            }
            if (found) break; // smallest detour distance that works
        }
        if (found) stats.detours++;
        else if (stats.warnings.empty() || stats.warnings.back().rfind("Geçiş", 0) != 0)
            stats.warnings.push_back("Geçiş yolu için engelsiz bir rota bulunamadı; geçişte çarpışma olabilir.");
        return best;
    };
    // `from` by value: callers pass out.back(), and push_back below may reallocate `out`
    auto link = [&](const PathPoint from, const PathPoint& to, bool lifted) {
        if (!lifted && segmentFree(from.pos, to.pos)) {
            addLine(from.pos, to.pos, from.dir, to.dir);
            return;
        }
        const glm::vec3 up0 = from.pos - from.dir * lift, up1 = to.pos - to.dir * lift;
        const std::vector<glm::vec3> r = route(up0, up1, -from.dir, -to.dir);
        PathPoint p;
        p.phase = PathPhase::Link;
        p.maxTilt = tilt > 0.0f ? 180.0f : 0.0f;
        addLine(from.pos, up0, from.dir, from.dir);
        p.pos = up0;
        p.dir = from.dir;
        out.push_back(p);
        const float total = std::max(pathLen(r), 1e-3f);
        float acc = 0.0f;
        for (size_t i = 1; i < r.size(); ++i) {
            const float l = glm::length(r[i] - r[i - 1]);
            const glm::vec3 da = glm::normalize(glm::mix(from.dir, to.dir, acc / total));
            acc += l;
            const glm::vec3 db = glm::normalize(glm::mix(from.dir, to.dir, acc / total));
            addLine(r[i - 1], r[i], da, db);
            p.pos = r[i];
            p.dir = db;
            out.push_back(p);
        }
        addLine(up1, to.pos, to.dir, to.dir);
    };

    int passIndex = 0;
    for (size_t pi = 0; pi < patches.size(); ++pi) {
        const Patch& P = patches[pi];
        const glm::vec3 N = P.N;
        stats.area += P.area;
        stats.faces++;
        BoundaryGrid grid;
        if (offset > 0.0f) grid.build(P.boundary, offset);
        auto valid = [&](const Curve& c, float s) {
            glm::vec3 p, n;
            c.eval(s, p, n);
            return !grid.closerThan(p, offset);
        };

        // pass direction: longest extent of the patch across N (PCA), then rotated by `angle`
        // exact second moment of the triangle areas (independent of how the surface is triangulated)
        glm::mat3 cov(0.0f);
        for (size_t t = 0; t < P.T.size(); ++t) {
            glm::vec3 d[3];
            for (int k = 0; k < 3; ++k) {
                d[k] = P.V[P.T[t][k]] - P.C;
                d[k] -= glm::dot(d[k], N) * N;
            }
            const glm::vec3 sum = d[0] + d[1] + d[2];
            cov += (glm::outerProduct(d[0], d[0]) + glm::outerProduct(d[1], d[1]) + glm::outerProduct(d[2], d[2]) +
                    glm::outerProduct(sum, sum)) * (P.TA[t] / 12.0f);
        }
        const glm::vec3 p1 = std::abs(N.z) < 0.9f ? glm::normalize(glm::cross(glm::vec3(0, 0, 1), N)) : glm::normalize(glm::cross(glm::vec3(1, 0, 0), N));
        const glm::vec3 p2 = glm::cross(N, p1);
        glm::vec3 U0 = glm::normalize(p1 + 0.618f * p2); // generic start: never exactly an eigenvector of a box
        for (int it = 0; it < 60; ++it) {                  // power iteration for the main axis
            glm::vec3 nu = cov * U0;
            nu -= glm::dot(nu, N) * N;
            if (glm::length(nu) < 1e-9f) break;
            U0 = glm::normalize(nu);
        }
        if (U0.x + U0.y * 0.01f + U0.z * 0.001f < 0.0f) U0 = -U0; // passes run left->right where possible

        std::vector<float> angles = {angle};
        if (def.pattern == "crosshatch") angles.push_back(angle + 90.0f);
        for (float a : angles) {
            bool firstOfPatch = true; // lift into each patch / each crosshatch direction
            const float r = glm::radians(a);
            const glm::vec3 U = glm::normalize(std::cos(r) * U0 + std::sin(r) * glm::cross(N, U0));
            const glm::vec3 D = glm::cross(N, U);
            float hmin = 1e30f, hmax = -1e30f;
            for (const auto& v : P.V) {
                const float h = glm::dot(v, D);
                hmin = std::min(hmin, h);
                hmax = std::max(hmax, h);
            }
            // adaptive, centred layout: the plane step shrinks where the surface is steep along D
            const float vlo = hmin + offset, vhi = hmax - offset;
            std::vector<float> H;
            auto slopeFactor = [&](const std::vector<Curve>& cs) {
                std::vector<float> f;
                for (const Curve& c : cs)
                    for (size_t i = 0; i < c.P.size(); ++i) {
                        if (offset > 0.0f && grid.closerThan(c.P[i], offset)) continue;
                        const float nd = glm::dot(glm::normalize(c.Nn[i]), D);
                        f.push_back(std::sqrt(std::max(0.0f, 1.0f - nd * nd)));
                    }
                if (f.empty()) return 1.0f;
                std::nth_element(f.begin(), f.begin() + f.size() / 5, f.end());
                return std::clamp(f[f.size() / 5], 0.25f, 1.0f);
            };
            if (vhi >= vlo - 1e-5f) {
                float h = vlo, lastF = 1.0f;
                for (int guard = 0; guard < 20000 && h <= vhi + 1e-4f * spacing; ++guard) {
                    H.push_back(h);
                    lastF = slopeFactor(slice(P, D, h));
                    // midpoint rule: surface distance to the next curve ~ spacing
                    const float fMid = slopeFactor(slice(P, D, h + 0.5f * spacing * lastF));
                    h += spacing * std::min(lastF, fMid);
                }
                const float shift = 0.5f * std::max(0.0f, vhi - H.back());
                for (float& x : H) x += shift;
                (void)lastF;
            }
            // curves and the intervals on them that respect the edge offset
            struct Piece {
                const Curve* curve;
                float s0, s1;
                float key; // ordering along U
            };
            std::vector<std::vector<Curve>> lineCurves(H.size());
            std::vector<std::vector<Piece>> pieces(H.size());
            const float sampleStep = std::clamp(std::min(step, spacing * 0.25f), 0.05f, 5.0f);
            int piecesHere = 0;
            for (size_t li = 0; li < H.size(); ++li) {
                lineCurves[li] = slice(P, D, H[li]);
                for (Curve& c : lineCurves[li]) {
                    // orientation: open curves run along +U, closed loops counter-clockwise about D
                    if (!c.closed) {
                        if (glm::dot(c.P.back() - c.P.front(), U) < 0.0f) {
                            std::reverse(c.P.begin(), c.P.end());
                            std::reverse(c.Nn.begin(), c.Nn.end());
                            c.finish();
                        }
                    }
                }
                for (const Curve& c : lineCurves[li]) {
                    const float L = c.length();
                    const int n = std::max(2, int(std::ceil(L / sampleStep)));
                    std::vector<char> ok(size_t(n) + 1);
                    float sStart = 0.0f;
                    if (c.closed) { // start a closed loop inside an invalid part so intervals do not wrap
                        for (int i = 0; i < n; ++i)
                            if (!valid(c, L * float(i) / float(n))) {
                                sStart = L * float(i) / float(n);
                                break;
                            }
                    }
                    auto at = [&](int i) { return sStart + L * float(i) / float(n); };
                    for (int i = 0; i <= n; ++i) ok[size_t(i)] = valid(c, at(i)) ? 1 : 0;
                    auto refine = [&](float a, float b) {
                        const bool va = valid(c, a);
                        for (int it = 0; it < 18; ++it) {
                            const float m = 0.5f * (a + b);
                            if (valid(c, m) == va) a = m;
                            else b = m;
                        }
                        return va ? a : b;
                    };
                    bool prev = false;
                    float start = 0.0f;
                    for (int i = 0; i <= n; ++i) {
                        const bool cur = ok[size_t(i)];
                        if (cur && !prev) start = i == 0 ? at(0) : refine(at(i - 1), at(i));
                        if (!cur && prev) {
                            const float end = refine(at(i - 1), at(i));
                            if (end - start > 1e-3f) pieces[li].push_back({&c, start, end, 0.0f});
                        }
                        prev = cur;
                    }
                    if (prev && at(n) - start > 1e-3f) pieces[li].push_back({&c, start, at(n), 0.0f});
                    // close gaps where the curve only grazes the offset limit (noise, faceting): the
                    // tool may pass if it stays within `tol` of the offset there
                    if (offset > 0.0f && !pieces[li].empty()) {
                        auto& pv = pieces[li];
                        const float tol = std::clamp(0.1f * offset, 0.2f, 1.0f);
                        size_t first = 0;
                        while (first < pv.size() && pv[first].curve != &c) ++first;
                        for (size_t i = first + 1; i < pv.size();) {
                            Piece& a = pv[i - 1];
                            const Piece& b = pv[i];
                            bool graze = true;
                            for (float s = a.s1; s <= b.s0 && graze; s += sampleStep) {
                                glm::vec3 p, nn;
                                c.eval(s, p, nn);
                                if (grid.closerThan(p, offset - tol)) graze = false;
                            }
                            if (graze) {
                                a.s1 = b.s1;
                                pv.erase(pv.begin() + std::ptrdiff_t(i));
                            } else {
                                ++i;
                            }
                        }
                        pv.erase(std::remove_if(pv.begin() + std::ptrdiff_t(first), pv.end(),
                                                [&](const Piece& q) { return q.s1 - q.s0 < std::min(1.0f, 0.1f * spacing); }),
                                 pv.end());
                    }
                }
                for (Piece& pc : pieces[li]) {
                    glm::vec3 p, n;
                    pc.curve->eval(pc.s0, p, n);
                    pc.key = glm::dot(p, U);
                }
                std::sort(pieces[li].begin(), pieces[li].end(), [](const Piece& a, const Piece& b) { return a.key < b.key; });
                piecesHere += int(pieces[li].size());
            }
            if (piecesHere == 0) {
                stats.warnings.push_back("Yüzey " + std::to_string(pi + 1) + ": kenar ofseti (" + std::to_string(int(offset)) +
                                         " mm) nedeniyle taranacak alan kalmadı.");
                continue;
            }
            const int nLines = int(H.size());
            int lineNo = 0;
            for (int li = 0; li < nLines; ++li) {
                auto& iv = pieces[size_t(li)];
                if (iv.empty()) continue;
                const float dirSign = (def.pattern == "oneway" || lineNo % 2 == 0) ? 1.0f : -1.0f;
                ++lineNo;
                if (dirSign < 0.0f) std::reverse(iv.begin(), iv.end());
                for (const Piece& pc : iv) {
                    const Curve& c = *pc.curve;
                    const float L = pc.s1 - pc.s0;
                    const int m = std::max(1, int(std::ceil(L / step)));
                    std::vector<PathPoint> pass;
                    for (int i = 0; i <= m; ++i) {
                        const float s = L * float(i) / float(m);
                        glm::vec3 off;
                        float work, travel;
                        motion.evaluate(def, s, L > 0 ? s / L : 0.0, L, settings.speed, off, work, travel, double(passIndex),
                                        double(std::max(1, nLines)));
                        float sc = (dirSign > 0 ? pc.s0 + s : pc.s1 - s) + dirSign * off.y;
                        if (!c.closed) sc = std::clamp(sc, 0.0f, c.length());
                        glm::vec3 surf, n0;
                        c.eval(sc, surf, n0);
                        glm::vec3 n = follow ? smoothNormal(c, sc, smoothW) : N;
                        // tangent of the curve in the travel direction
                        glm::vec3 pa, pb, tmp;
                        const float ds = std::max(0.05f, step * 0.5f);
                        c.eval(sc - ds * dirSign, pa, tmp);
                        c.eval(sc + ds * dirSign, pb, tmp);
                        glm::vec3 T = pb - pa;
                        T -= glm::dot(T, n) * n;
                        if (glm::length(T) < 1e-6f) T = U * dirSign - glm::dot(U * dirSign, n) * n;
                        T = glm::normalize(T);
                        Seam frame;
                        frame.N = n;
                        frame.Y = T;
                        frame.X = glm::cross(T, n);
                        PathPoint p;
                        p.phase = PathPhase::Process;
                        p.s = s;
                        p.pos = surf + frame.X * off.x + n * (standoff + off.z);
                        p.dir = toolDirection(frame, work, travel);
                        p.aim = standoff + off.z; // the spray axis stays aimed at this surface point
                        p.maxTilt = tilt;
                        pass.push_back(p);
                    }
                    if (!out.empty()) {
                        const bool lifted = firstOfPatch || def.link == "lift" || def.pattern == "oneway";
                        link(out.back(), pass.front(), lifted);
                    }
                    firstOfPatch = false;
                    out.insert(out.end(), pass.begin(), pass.end());
                    stats.processLength += L;
                    stats.passes++;
                    ++passIndex;
                }
            }
        }
    }
    if (out.empty()) {
        err = stats.warnings.empty() ? "Taranacak alan bulunamadı." : stats.warnings.back();
        return false;
    }
    return true;
}

} // namespace pegasus
