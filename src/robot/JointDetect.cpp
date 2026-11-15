#include "robot/JointDetect.h"

#include "geom/Collision.h"
#include "scene/StlLoader.h"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstdarg>
#include <cstdio>
#include <filesystem>
#include <map>
#include <numeric>

namespace pegasus {

namespace fs = std::filesystem;

namespace {

std::string fmt(const char* f, ...) __attribute__((format(printf, 1, 2)));
std::string fmt(const char* f, ...) {
    char buf[512];
    va_list ap;
    va_start(ap, f);
    std::vsnprintf(buf, sizeof(buf), f, ap);
    va_end(ap);
    return buf;
}

// Jacobi eigen decomposition of a symmetric 3x3 matrix. Columns of `vecs` are eigenvectors,
// values sorted ascending.
void symmetricEigen(const glm::dmat3& m, glm::dvec3& vals, glm::dmat3& vecs) {
    double a[3][3];
    double v[3][3] = {{1, 0, 0}, {0, 1, 0}, {0, 0, 1}};
    for (int c = 0; c < 3; ++c)
        for (int r = 0; r < 3; ++r) a[r][c] = m[c][r];
    for (int sweep = 0; sweep < 50; ++sweep) {
        const double off = std::abs(a[0][1]) + std::abs(a[0][2]) + std::abs(a[1][2]);
        if (off < 1e-14) break;
        for (int p = 0; p < 2; ++p)
            for (int q = p + 1; q < 3; ++q) {
                if (std::abs(a[p][q]) < 1e-300) continue;
                const double theta = (a[q][q] - a[p][p]) / (2.0 * a[p][q]);
                const double t = (theta >= 0 ? 1.0 : -1.0) / (std::abs(theta) + std::sqrt(theta * theta + 1.0));
                const double c = 1.0 / std::sqrt(t * t + 1.0), s = t * c;
                for (int k = 0; k < 3; ++k) {
                    const double akp = a[k][p], akq = a[k][q];
                    a[k][p] = c * akp - s * akq;
                    a[k][q] = s * akp + c * akq;
                }
                for (int k = 0; k < 3; ++k) {
                    const double apk = a[p][k], aqk = a[q][k];
                    a[p][k] = c * apk - s * aqk;
                    a[q][k] = s * apk + c * aqk;
                }
                for (int k = 0; k < 3; ++k) {
                    const double vkp = v[k][p], vkq = v[k][q];
                    v[k][p] = c * vkp - s * vkq;
                    v[k][q] = s * vkp + c * vkq;
                }
            }
    }
    int idx[3] = {0, 1, 2};
    std::sort(idx, idx + 3, [&](int x, int y) { return a[x][x] < a[y][y]; });
    for (int i = 0; i < 3; ++i) {
        vals[i] = a[idx[i]][idx[i]];
        vecs[i] = glm::dvec3(v[0][idx[i]], v[1][idx[i]], v[2][idx[i]]);
    }
}

// Least squares circle in 2D (Kasa fit).
bool fitCircle2D(const std::vector<glm::dvec2>& p, glm::dvec2& c, double& r) {
    if (p.size() < 3) return false;
    glm::dmat3 A(0.0);
    glm::dvec3 b(0.0);
    for (const auto& q : p) {
        const glm::dvec3 row(q.x, q.y, 1.0);
        const double rhs = -(q.x * q.x + q.y * q.y);
        A += glm::outerProduct(row, row);
        b += row * rhs;
    }
    if (std::abs(glm::determinant(A)) < 1e-18) return false;
    const glm::dvec3 s = glm::inverse(A) * b;
    c = glm::dvec2(-s.x / 2.0, -s.y / 2.0);
    const double r2 = glm::dot(c, c) - s.z;
    if (r2 <= 0.0) return false;
    r = std::sqrt(r2);
    return true;
}

glm::dvec3 anyPerpendicular(const glm::dvec3& n) {
    const glm::dvec3 t = std::abs(n.x) < 0.9 ? glm::dvec3(1, 0, 0) : glm::dvec3(0, 1, 0);
    return glm::normalize(glm::cross(n, t));
}

struct Line {
    glm::vec3 p{0.0f}, d{0.0f, 0.0f, 1.0f};
    float radius = 0.0f;
};

bool sameLine(const Line& a, const glm::vec3& p, const glm::vec3& d, float tol) {
    if (std::abs(glm::dot(a.d, d)) < 0.999f) return false;
    const glm::vec3 off = p - a.p;
    return glm::length(off - glm::dot(off, a.d) * a.d) <= tol;
}

struct Connection {
    int a = -1, b = -1;
    std::vector<Line> lines;
};

// Weighted union-find: angle(i) = w[i] * angle(root(i)).
struct MimicGroups {
    std::vector<int> parent;
    std::vector<float> w;
    explicit MimicGroups(size_t n) : parent(n), w(n, 1.0f) { std::iota(parent.begin(), parent.end(), 0); }
    int find(int i) {
        if (parent[i] == i) return i;
        const int r = find(parent[i]);
        w[i] *= w[parent[i]];
        parent[i] = r;
        return r;
    }
    // angle(b) = k * angle(a). Returns false if already related.
    bool relate(int a, int b, float k) {
        const int ra = find(a), rb = find(b);
        if (ra == rb) return false;
        w[rb] = k * w[a] / w[b];
        parent[rb] = ra;
        return true;
    }
};

float containedFraction(const AABB& inner, const AABB& outer) {
    const glm::vec3 lo = glm::max(inner.min, outer.min), hi = glm::min(inner.max, outer.max);
    const glm::vec3 s = glm::max(hi - lo, glm::vec3(0.0f));
    const glm::vec3 is = inner.size();
    const float vol = std::max(is.x, 1e-3f) * std::max(is.y, 1e-3f) * std::max(is.z, 1e-3f);
    return s.x * s.y * s.z / vol;
}

} // namespace

// =============================================================================================

std::vector<Circle> findCircles(const Mesh& m) {
    std::vector<Circle> out;
    std::vector<std::vector<uint32_t>> adj(m.positions.size());
    for (uint32_t ei : m.featureEdges) {
        adj[m.edges[ei].v[0]].push_back(ei);
        adj[m.edges[ei].v[1]].push_back(ei);
    }
    std::vector<char> used(m.edges.size(), 0);
    auto other = [&](uint32_t ei, uint32_t v) { return m.edges[ei].v[0] == v ? m.edges[ei].v[1] : m.edges[ei].v[0]; };

    for (uint32_t start : m.featureEdges) {
        if (used[start]) continue;
        used[start] = 1;
        std::vector<uint32_t> chain = {m.edges[start].v[0], m.edges[start].v[1]};
        bool closed = false;
        while (true) { // walk forward through vertices of degree 2
            const uint32_t v = chain.back();
            if (adj[v].size() != 2) break;
            uint32_t next = kInvalid;
            for (uint32_t ei : adj[v])
                if (!used[ei]) next = ei;
            if (next == kInvalid) break;
            used[next] = 1;
            const uint32_t w = other(next, v);
            if (w == chain.front()) {
                closed = true;
                break;
            }
            chain.push_back(w);
        }
        if (!closed || chain.size() < 8) continue;

        glm::dvec3 c(0.0);
        for (uint32_t v : chain) c += glm::dvec3(m.positions[v]);
        c /= double(chain.size());
        glm::dvec3 n(0.0);
        for (size_t i = 0; i < chain.size(); ++i)
            n += glm::cross(glm::dvec3(m.positions[chain[i]]) - c, glm::dvec3(m.positions[chain[(i + 1) % chain.size()]]) - c);
        if (glm::length(n) < 1e-12) continue;
        n = glm::normalize(n);
        double r = 0.0;
        for (uint32_t v : chain) r += glm::length(glm::dvec3(m.positions[v]) - c);
        r /= double(chain.size());
        double dev = 0.0, planar = 0.0;
        for (uint32_t v : chain) {
            const glm::dvec3 d = glm::dvec3(m.positions[v]) - c;
            dev = std::max(dev, std::abs(glm::length(d) - r));
            planar = std::max(planar, std::abs(glm::dot(d, n)));
        }
        if (r < 0.75 || dev > 0.03 * r + 0.02 || planar > 0.03 * r + 0.02) continue;
        // canonical direction: dominant component positive
        const int k = std::abs(n.x) >= std::abs(n.y) && std::abs(n.x) >= std::abs(n.z) ? 0 : (std::abs(n.y) >= std::abs(n.z) ? 1 : 2);
        if (n[k] < 0.0) n = -n;
        out.push_back({glm::vec3(c), glm::vec3(n), float(r)});
    }
    return out;
}

std::vector<std::string> prettyPartNames(const std::vector<std::string>& stems) {
    if (stems.size() < 2) return stems;
    size_t common = stems[0].size();
    for (const auto& s : stems) {
        size_t i = 0;
        while (i < common && i < s.size() && s[i] == stems[0][i]) ++i;
        common = i;
    }
    // cut back to a word boundary
    while (common > 0 && stems[0][common - 1] != ' ' && stems[0][common - 1] != '-' && stems[0][common - 1] != '_') --common;
    std::vector<std::string> out;
    for (const auto& s : stems) {
        std::string n = s.substr(common);
        while (!n.empty() && (n[0] == ' ' || n[0] == '-' || n[0] == '_')) n.erase(0, 1);
        out.push_back(n.empty() ? s : n);
    }
    return out;
}

bool fitCircle(const std::vector<glm::vec3>& pts, glm::vec3& center, glm::vec3& normal, float& radius) {
    if (pts.size() < 3) return false;
    glm::dvec3 mean(0.0);
    for (const auto& p : pts) mean += glm::dvec3(p);
    mean /= double(pts.size());
    glm::dmat3 cov(0.0);
    for (const auto& p : pts) {
        const glm::dvec3 d = glm::dvec3(p) - mean;
        cov += glm::outerProduct(d, d);
    }
    glm::dvec3 vals;
    glm::dmat3 vecs;
    symmetricEigen(cov, vals, vecs);
    if (vals[1] < 1e-9 * std::max(vals[2], 1e-30)) return false; // collinear
    const glm::dvec3 n = glm::normalize(vecs[0]);
    const glm::dvec3 u = glm::normalize(vecs[2]), v = glm::cross(n, u);
    std::vector<glm::dvec2> p2;
    for (const auto& p : pts) {
        const glm::dvec3 d = glm::dvec3(p) - mean;
        p2.emplace_back(glm::dot(d, u), glm::dot(d, v));
    }
    glm::dvec2 c2;
    double r;
    if (!fitCircle2D(p2, c2, r)) return false;
    center = glm::vec3(mean + u * c2.x + v * c2.y);
    normal = glm::vec3(n);
    radius = float(r);
    return true;
}

bool fitAxisFromSurface(const std::vector<glm::vec3>& pts, const std::vector<glm::vec3>& normals,
                        const std::vector<float>& weights, glm::vec3& origin, glm::vec3& axis, bool& cylindrical) {
    if (pts.empty() || normals.size() != pts.size()) return false;
    glm::dmat3 M(0.0);
    glm::dvec3 centroid(0.0);
    double wsum = 0.0;
    for (size_t i = 0; i < pts.size(); ++i) {
        const double w = i < weights.size() ? weights[i] : 1.0;
        const glm::dvec3 n = glm::dvec3(normals[i]);
        M += w * glm::outerProduct(n, n);
        centroid += w * glm::dvec3(pts[i]);
        wsum += w;
    }
    if (wsum <= 0.0) return false;
    centroid /= wsum;
    glm::dvec3 vals;
    glm::dmat3 vecs;
    symmetricEigen(M, vals, vecs);
    // Cylinder: normals span a plane -> one eigenvalue ~ 0 and the other two comparable.
    cylindrical = vals[0] < 0.02 * vals[2] && vals[1] > 0.05 * vals[2];
    if (!cylindrical) {
        axis = glm::vec3(glm::normalize(vecs[2]));
        origin = glm::vec3(centroid);
        return true;
    }
    const glm::dvec3 a = glm::normalize(vecs[0]);
    const glm::dvec3 u = anyPerpendicular(a), v = glm::cross(a, u);
    std::vector<glm::dvec2> p2;
    for (const auto& p : pts) {
        const glm::dvec3 d = glm::dvec3(p) - centroid;
        p2.emplace_back(glm::dot(d, u), glm::dot(d, v));
    }
    glm::dvec2 c2;
    double r;
    if (!fitCircle2D(p2, c2, r)) return false;
    origin = glm::vec3(centroid + u * c2.x + v * c2.y);
    axis = glm::vec3(a);
    return true;
}

// =============================================================================================
// Structure detection
// =============================================================================================

void detectRobotStructure(Robot& robot, std::vector<std::string>& log) {
    const int n = int(robot.parts.size());
    robot.joints.clear();
    for (auto& p : robot.parts) {
        p.parent = -1;
        p.joint = -1;
    }
    if (n == 0) return;

    std::vector<std::vector<Circle>> circles(n);
    std::vector<AABB> box(n);
    AABB all;
    for (int i = 0; i < n; ++i) {
        circles[i] = findCircles(*robot.parts[i].mesh);
        box[i] = robot.parts[i].mesh->bounds;
        all.expand(box[i]);
    }

    // ---- up axis and root (lowest part along the longest assembly direction)
    const glm::vec3 ext = all.size();
    robot.upAxis = ext.y >= ext.x && ext.y >= ext.z ? 1 : (ext.x >= ext.z ? 0 : 2);
    const int up = robot.upAxis;
    int root = 0;
    for (int i = 1; i < n; ++i) {
        const float a = box[i].min[up], b = box[root].min[up];
        if (a < b - 1e-3f || (std::abs(a - b) <= 1e-3f && box[i].diagonal() > box[root].diagonal())) root = i;
    }
    log.push_back(fmt("Yukarı ekseni: %c, taban parçası: %s", "XYZ"[up], robot.parts[root].name.c_str()));

    // ---- coaxial circle connections between neighbouring parts
    std::vector<Connection> conns;
    std::vector<std::vector<int>> connOf(n);
    for (int a = 0; a < n; ++a) {
        for (int b = a + 1; b < n; ++b) {
            AABB ea = box[a];
            ea.min -= glm::vec3(2.0f);
            ea.max += glm::vec3(2.0f);
            if (!ea.overlaps(box[b])) continue;
            Connection c{a, b, {}};
            for (const Circle& ca : circles[a]) {
                for (const Circle& cb : circles[b]) {
                    if (std::abs(glm::dot(ca.normal, cb.normal)) < 0.999f) continue;
                    const glm::vec3 d = ca.normal;
                    const glm::vec3 off = cb.center - ca.center;
                    const float along = glm::dot(off, d);
                    const float perp = glm::length(off - along * d);
                    const float tol = std::max(0.5f, 0.08f * std::min(ca.radius, cb.radius));
                    if (perp > tol || std::abs(along) > 30.0f) continue;
                    const glm::vec3 mid = 0.5f * (ca.center + cb.center);
                    bool merged = false;
                    for (Line& l : c.lines)
                        if (sameLine(l, mid, d, tol)) {
                            l.radius = std::max(l.radius, std::max(ca.radius, cb.radius));
                            merged = true;
                            break;
                        }
                    if (!merged) c.lines.push_back({mid, d, std::max(ca.radius, cb.radius)});
                }
            }
            if (c.lines.empty()) continue;
            connOf[a].push_back(int(conns.size()));
            connOf[b].push_back(int(conns.size()));
            conns.push_back(std::move(c));
        }
    }

    // ---- spanning tree (BFS from the base)
    std::vector<int> treeConn(n, -1);
    std::vector<char> visited(n, 0);
    std::vector<int> queue = {root};
    visited[root] = 1;
    robot.joints.reserve(n);
    for (size_t qi = 0; qi < queue.size(); ++qi) {
        const int p = queue[qi];
        std::vector<int> cs = connOf[p];
        std::sort(cs.begin(), cs.end(), [&](int x, int y) {
            const int ox = conns[x].a == p ? conns[x].b : conns[x].a, oy = conns[y].a == p ? conns[y].b : conns[y].a;
            return ox < oy;
        });
        for (int ci : cs) {
            const Connection& c = conns[ci];
            const int q = c.a == p ? c.b : c.a;
            if (visited[q]) continue;
            visited[q] = 1;
            queue.push_back(q);
            robot.parts[q].parent = p;
            treeConn[q] = ci;
            RobotJoint j;
            j.part = q;
            const float inside = containedFraction(box[q], box[p]);
            if (c.lines.size() >= 2 || inside >= 0.8f) {
                j.type = JointType::Fixed;
            } else {
                j.type = JointType::Revolute;
                j.origin = c.lines[0].p;
                j.axis = glm::normalize(c.lines[0].d);
            }
            robot.parts[q].joint = int(robot.joints.size());
            robot.joints.push_back(j);
        }
    }
    // Parts without any shaft/pin connection: fix them to the visited part they overlap most.
    for (bool progress = true; progress;) {
        progress = false;
        for (int q = 0; q < n; ++q) {
            if (visited[q]) continue;
            int best = -1;
            float bestVol = 0.0f;
            for (int p = 0; p < n; ++p) {
                if (!visited[p]) continue;
                AABB e = box[p];
                e.min -= glm::vec3(1.0f);
                e.max += glm::vec3(1.0f);
                const glm::vec3 s = glm::max(glm::min(e.max, box[q].max) - glm::max(e.min, box[q].min), glm::vec3(0.0f));
                const float vol = s.x * s.y * s.z + 1e-6f;
                if (e.overlaps(box[q]) && vol > bestVol) {
                    bestVol = vol;
                    best = p;
                }
            }
            if (best < 0) continue;
            visited[q] = 1;
            robot.parts[q].parent = best;
            RobotJoint j;
            j.part = q;
            j.type = JointType::Fixed;
            robot.parts[q].joint = int(robot.joints.size());
            robot.joints.push_back(j);
            progress = true;
            log.push_back(fmt("%s: mil/pim bulunamadı, %s parçasına sabitlendi", robot.parts[q].name.c_str(),
                              robot.parts[best].name.c_str()));
        }
    }
    for (int q = 0; q < n; ++q)
        if (!visited[q]) { // isolated part: attach to the base so the hierarchy stays valid
            robot.parts[q].parent = root;
            RobotJoint j;
            j.part = q;
            j.type = JointType::Fixed;
            robot.parts[q].joint = int(robot.joints.size());
            robot.joints.push_back(j);
            log.push_back(fmt("%s: diğer parçalara temas etmiyor, tabana sabitlendi", robot.parts[q].name.c_str()));
        }

    auto jointOf = [&](int part) { return robot.parts[part].joint; };
    auto isRev = [&](int part) { const int j = jointOf(part); return j >= 0 && robot.joints[j].type == JointType::Revolute; };

    // ---- motorised joints: the axis coincides with a shaft of a fixed sibling part (servo)
    for (const Connection& c : conns) {
        for (int side = 0; side < 2; ++side) {
            const int q = side == 0 ? c.a : c.b, f = side == 0 ? c.b : c.a;
            if (!isRev(q) || jointOf(f) < 0 || robot.joints[jointOf(f)].type != JointType::Fixed) continue;
            if (robot.parts[f].parent != robot.parts[q].parent) continue;
            RobotJoint& j = robot.joints[jointOf(q)];
            for (const Line& l : c.lines)
                if (sameLine(Line{j.origin, j.axis, 0}, l.p, l.d, 0.6f)) j.motorized = true;
        }
    }

    // ---- closed loops -> mimic relations
    MimicGroups groups(robot.joints.size());
    auto perp = [](const glm::vec3& v, const glm::vec3& d) { return v - glm::dot(v, d) * d; };
    for (int ci = 0; ci < int(conns.size()); ++ci) {
        const Connection& c = conns[ci];
        if (treeConn[c.a] == ci || treeConn[c.b] == ci || c.lines.size() != 1) continue;
        for (int side = 0; side < 2; ++side) {
            const int X = side == 0 ? c.a : c.b, B = side == 0 ? c.b : c.a;
            const int A = robot.parts[X].parent;
            if (A < 0 || A == B || !isRev(X) || !isRev(A) || !isRev(B)) continue;
            if (robot.parts[B].parent != robot.parts[A].parent) continue;
            const RobotJoint &jX = robot.joints[jointOf(X)], &jA = robot.joints[jointOf(A)], &jB = robot.joints[jointOf(B)];
            const glm::vec3 d = jA.axis;
            if (std::abs(glm::dot(d, jB.axis)) < 0.999f || std::abs(glm::dot(d, jX.axis)) < 0.999f) continue;
            const glm::vec3 vA = perp(jX.origin - jA.origin, d);
            const glm::vec3 vB = perp(c.lines[0].p - jB.origin, d);
            if (glm::length(vA - vB) > 0.05f * glm::length(vA) + 0.8f) continue;
            // Parallelogram: B turns with A, X keeps its orientation (translates).
            const float sB = glm::dot(jB.axis, jA.axis) > 0 ? 1.0f : -1.0f;
            const float sX = glm::dot(jX.axis, jA.axis) > 0 ? 1.0f : -1.0f;
            const bool r1 = groups.relate(jointOf(A), jointOf(B), sB);
            const bool r2 = groups.relate(jointOf(A), jointOf(X), -sX);
            if (r1 || r2)
                log.push_back(fmt("Paralelkenar bağlantı: %s + %s, %s ötelenerek hareket eder", robot.parts[A].name.c_str(),
                                  robot.parts[B].name.c_str(), robot.parts[X].name.c_str()));
            break;
        }
    }
    // gear pairs: sibling revolute joints on different parallel axes whose parts touch
    for (int a = 0; a < n; ++a) {
        for (int b = a + 1; b < n; ++b) {
            if (!isRev(a) || !isRev(b) || robot.parts[a].parent != robot.parts[b].parent) continue;
            const int ja = jointOf(a), jb = jointOf(b);
            const RobotJoint &JA = robot.joints[ja], &JB = robot.joints[jb];
            if (std::abs(glm::dot(JA.axis, JB.axis)) < 0.999f) continue;
            if (sameLine(Line{JA.origin, JA.axis, 0}, JB.origin, JB.axis, 1.0f)) continue;
            if (groups.find(ja) == groups.find(jb)) continue;
            if (!box[a].overlaps(box[b])) continue;
            if (!CollisionSystem::touching(*robot.parts[a].mesh, glm::mat4(1.0f), *robot.parts[b].mesh, glm::mat4(1.0f), 0.3f)) continue;
            const float s = glm::dot(JA.axis, JB.axis) > 0 ? -1.0f : 1.0f;
            groups.relate(ja, jb, s);
            log.push_back(fmt("Dişli çifti: %s <-> %s (ters yönde, oran 1:1)", robot.parts[a].name.c_str(), robot.parts[b].name.c_str()));
        }
    }
    // choose one driver per group and express the others as mimics of it
    std::map<int, std::vector<int>> members;
    for (int j = 0; j < int(robot.joints.size()); ++j)
        if (robot.joints[j].type == JointType::Revolute) members[groups.find(j)].push_back(j);
    for (auto& [r, ms] : members) {
        if (ms.size() < 2) continue;
        int driver = ms[0];
        for (int j : ms) {
            const RobotJoint &a = robot.joints[j], &b = robot.joints[driver];
            const int da = robot.depth(a.part), db = robot.depth(b.part);
            if ((a.motorized && !b.motorized) || (a.motorized == b.motorized && da < db)) driver = j;
        }
        groups.find(driver);
        for (int j : ms) {
            if (j == driver) continue;
            groups.find(j);
            robot.joints[j].mimic = driver;
            robot.joints[j].multiplier = groups.w[j] / groups.w[driver];
        }
    }

    // ---- names and limits
    const std::vector<int> drv = robot.drivers();
    static const char* six[] = {"Bel", "Omuz", "Dirsek", "Bilek dönüş", "Bilek eğim", "Kıskaç"};
    for (size_t k = 0; k < drv.size(); ++k) {
        RobotJoint& j = robot.joints[drv[k]];
        j.name = drv.size() == 6 ? six[k] : fmt("Eklem %zu", k + 1);
        j.minDeg = -90.0f;
        j.maxDeg = 90.0f;
    }
    for (auto& j : robot.joints) {
        if (j.type == JointType::Fixed) j.name = "Sabit: " + robot.parts[j.part].name;
        else if (j.mimic >= 0) {
            j.name = robot.joints[j.mimic].name + " -> " + robot.parts[j.part].name;
            j.minDeg = -360.0f;
            j.maxDeg = 360.0f;
        }
    }

    // ---- TCP: centre of the tip region of the last driver's mechanism
    if (!drv.empty()) {
        const int last = robot.joints[drv.back()].part;
        const int G = robot.parts[last].parent >= 0 ? robot.parts[last].parent : last;
        glm::vec3 dir = box[G].center() - box[root].center();
        if (glm::length(dir) < 1e-3f) dir = glm::vec3(0.0f);
        if (glm::length(dir) < 1e-3f) dir[up] = 1.0f;
        dir = glm::normalize(dir);
        float maxProj = -std::numeric_limits<float>::max();
        std::vector<int> sub;
        for (int i = 0; i < n; ++i)
            if (i == G || robot.isAncestor(G, i)) sub.push_back(i);
        for (int i : sub)
            for (const auto& p : robot.parts[i].mesh->positions) maxProj = std::max(maxProj, glm::dot(p, dir));
        glm::dvec3 acc(0.0);
        size_t cnt = 0;
        for (int i : sub)
            for (const auto& p : robot.parts[i].mesh->positions)
                if (glm::dot(p, dir) >= maxProj - 10.0f) {
                    acc += glm::dvec3(p);
                    ++cnt;
                }
        robot.tcpPart = G;
        robot.tcpPoint = cnt ? glm::vec3(acc / double(cnt)) : box[G].center();
    }

    // ---- report
    int revolute = 0, fixed = 0, mimic = 0;
    for (const auto& j : robot.joints) {
        if (j.type == JointType::Fixed) ++fixed;
        else if (j.mimic >= 0) ++mimic;
        else ++revolute;
    }
    log.push_back(fmt("%d bağımsız eklem, %d bağımlı eklem, %d sabit bağlantı bulundu.", revolute, mimic, fixed));
    for (int j : drv) {
        const RobotJoint& J = robot.joints[j];
        log.push_back(fmt("  %-12s %s -> %s  eksen (%.2f %.2f %.2f) @ (%.1f %.1f %.1f)%s", J.name.c_str(),
                          robot.parts[robot.parts[J.part].parent].name.c_str(), robot.parts[J.part].name.c_str(), J.axis.x,
                          J.axis.y, J.axis.z, J.origin.x, J.origin.y, J.origin.z, J.motorized ? " [motor]" : ""));
    }
}

std::unique_ptr<Robot> detectRobot(const std::string& folder, std::string& err, std::vector<std::string>& log) {
    std::vector<fs::path> files;
    std::error_code ec;
    for (const auto& e : fs::directory_iterator(folder, ec)) {
        std::string ext = e.path().extension().string();
        for (auto& c : ext) c = char(std::tolower(static_cast<unsigned char>(c)));
        if (e.is_regular_file() && ext == ".stl") files.push_back(e.path());
    }
    if (ec) {
        err = "Klasör okunamadı: " + folder;
        return nullptr;
    }
    std::sort(files.begin(), files.end());
    if (files.size() < 2) {
        err = "Klasörde en az iki STL parçası olmalı: " + folder;
        return nullptr;
    }
    auto robot = std::make_unique<Robot>();
    robot->folder = folder;
    robot->name = fs::path(folder).filename().string();
    std::vector<std::string> stems;
    for (const auto& f : files) stems.push_back(f.stem().string());
    const auto names = prettyPartNames(stems);
    for (size_t i = 0; i < files.size(); ++i) {
        std::string e;
        RobotPart p;
        p.file = files[i].filename().string();
        p.name = names[i];
        p.mesh = loadStl(files[i].string(), e);
        if (!p.mesh) {
            err = e;
            return nullptr;
        }
        robot->parts.push_back(std::move(p));
    }
    detectRobotStructure(*robot, log);
    if (std::string v = robot->validate(); !v.empty()) {
        err = v;
        return nullptr;
    }
    robot->solveFK();
    robot->computeIgnoredPairs();
    for (int d : robot->drivers()) robot->autoLimits(d);
    // gripper-like last mechanism: make positive angles close it
    const auto drv = robot->drivers();
    if (!drv.empty() && robot->tcpPart >= 0) {
        const int d = drv.back();
        int members = 0;
        for (const auto& j : robot->joints) members += j.mimic == d;
        if (members >= 2) {
            // closing = parts of the mechanism move towards the gripper centre line
            const int r = robot->root();
            glm::vec3 dir = robot->parts[robot->tcpPart].mesh->bounds.center() - robot->parts[r].mesh->bounds.center();
            dir = glm::length(dir) > 1e-3f ? glm::normalize(dir) : glm::vec3(0, 1, 0);
            auto spread = [&]() {
                robot->solveFK();
                float s = 0.0f;
                const glm::vec3 tcp = robot->tcpPoint;
                for (int i = 0; i < int(robot->parts.size()); ++i) {
                    const int j = robot->parts[i].joint;
                    if (j >= 0 && robot->joints[j].mimic == d) {
                        glm::vec3 v = glm::vec3(robot->partModel(i) * glm::vec4(robot->parts[i].mesh->bounds.center(), 1.0f)) - tcp;
                        v -= glm::dot(v, dir) * dir;
                        s += glm::length(v);
                    }
                }
                return s;
            };
            const float s0 = spread();
            robot->joints[d].angleDeg = 5.0f;
            const float s1 = spread();
            robot->joints[d].angleDeg = 0.0f;
            if (s1 > s0) robot->flipDriverDirection(d);
            robot->solveFK();
            log.push_back("Kıskaç: pozitif açı = kapanma yönü.");
        }
    }
    for (int d : drv)
        log.push_back(fmt("  %-12s limit: %.0f° .. %.0f°", robot->joints[d].name.c_str(), robot->joints[d].minDeg, robot->joints[d].maxDeg));
    return robot;
}

} // namespace pegasus
