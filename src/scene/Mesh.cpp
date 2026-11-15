#include "scene/Mesh.h"

#include <algorithm>
#include <atomic>
#include <cmath>
#include <unordered_map>

namespace pegasus {

namespace {

std::atomic<uint64_t> g_meshUid{1};

struct QuantKey {
    int64_t x, y, z;
    bool operator==(const QuantKey& o) const { return x == o.x && y == o.y && z == o.z; }
};
struct QuantKeyHash {
    size_t operator()(const QuantKey& k) const {
        uint64_t h = uint64_t(k.x) * 0x9E3779B97F4A7C15ull;
        h ^= uint64_t(k.y) * 0xC2B2AE3D27D4EB4Full + (h << 6) + (h >> 2);
        h ^= uint64_t(k.z) * 0x165667B19E3779F9ull + (h << 6) + (h >> 2);
        return size_t(h);
    }
};

inline uint64_t edgeKey(uint32_t a, uint32_t b) {
    if (a > b) std::swap(a, b);
    return (uint64_t(a) << 32) | uint64_t(b);
}

// +1 if triangle traverses (a -> b), -1 if (b -> a)
inline int edgeDirection(const glm::uvec3& t, uint32_t a, uint32_t b) {
    for (int k = 0; k < 3; ++k) {
        if (t[k] == a && t[(k + 1) % 3] == b) return 1;
        if (t[k] == b && t[(k + 1) % 3] == a) return -1;
    }
    return 0;
}

} // namespace

// ---------------------------------------------------------------------------------------------
// BVH
// ---------------------------------------------------------------------------------------------

void BVH::build(const Mesh& mesh) {
    nodes_.clear();
    triOrder_.clear();
    const uint32_t n = uint32_t(mesh.triangles.size());
    if (n == 0) return;
    triOrder_.resize(n);
    std::vector<glm::vec3> centroids(n);
    for (uint32_t i = 0; i < n; ++i) {
        triOrder_[i] = i;
        centroids[i] = mesh.triCentroid(i);
    }
    nodes_.reserve(size_t(n) * 2);
    Node root;
    root.leftOrFirst = 0;
    root.count = n;
    nodes_.push_back(root);
    subdivide(mesh, centroids, 0);
    nodes_.shrink_to_fit();
}

void BVH::subdivide(const Mesh& mesh, const std::vector<glm::vec3>& centroids, uint32_t nodeIdx) {
    Node& node = nodes_[nodeIdx];
    const uint32_t first = node.leftOrFirst, count = node.count;
    AABB box, cbox;
    for (uint32_t i = first; i < first + count; ++i) {
        const uint32_t t = triOrder_[i];
        for (int k = 0; k < 3; ++k) box.expand(mesh.triVertex(t, k));
        cbox.expand(centroids[t]);
    }
    node.box = box;
    if (count <= 4) return;

    const glm::vec3 ext = cbox.size();
    int axis = 0;
    if (ext.y > ext[axis]) axis = 1;
    if (ext.z > ext[axis]) axis = 2;
    if (ext[axis] <= 0.0f) return; // all centroids coincide -> keep as leaf

    const uint32_t mid = first + count / 2;
    std::nth_element(triOrder_.begin() + first, triOrder_.begin() + mid, triOrder_.begin() + first + count,
                     [&](uint32_t a, uint32_t b) { return centroids[a][axis] < centroids[b][axis]; });

    const uint32_t left = uint32_t(nodes_.size());
    Node l, r;
    l.leftOrFirst = first;
    l.count = mid - first;
    r.leftOrFirst = mid;
    r.count = first + count - mid;
    nodes_.push_back(l);
    nodes_.push_back(r);
    // `node` reference may be invalidated by push_back
    nodes_[nodeIdx].leftOrFirst = left;
    nodes_[nodeIdx].count = 0;
    subdivide(mesh, centroids, left);
    subdivide(mesh, centroids, left + 1);
}

// ---------------------------------------------------------------------------------------------
// Mesh construction
// ---------------------------------------------------------------------------------------------

std::shared_ptr<Mesh> Mesh::fromSoup(const std::vector<glm::vec3>& soup, std::string name) {
    auto mesh = std::make_shared<Mesh>();
    mesh->name = std::move(name);
    mesh->uid = g_meshUid.fetch_add(1);

    AABB box;
    for (const auto& p : soup) box.expand(p);
    const float diag = box.diagonal();
    const double q = diag > 0.0f ? double(diag) * 1e-7 : 1e-9;

    // Weld vertices closer than `tol`: hash grid with cell size 2*tol, checking neighbour cells
    // so that points straddling a cell boundary are still merged.
    const double tol = q;
    const double cell = tol * 2.0;
    std::unordered_map<QuantKey, std::vector<uint32_t>, QuantKeyHash> grid;
    grid.reserve(soup.size());
    std::vector<uint32_t> remap(soup.size());
    for (size_t i = 0; i < soup.size(); ++i) {
        const glm::vec3& p = soup[i];
        const int64_t cx = int64_t(std::floor(p.x / cell)), cy = int64_t(std::floor(p.y / cell)), cz = int64_t(std::floor(p.z / cell));
        uint32_t found = kInvalid;
        for (int dx = -1; dx <= 1 && found == kInvalid; ++dx)
            for (int dy = -1; dy <= 1 && found == kInvalid; ++dy)
                for (int dz = -1; dz <= 1 && found == kInvalid; ++dz) {
                    auto it = grid.find(QuantKey{cx + dx, cy + dy, cz + dz});
                    if (it == grid.end()) continue;
                    for (uint32_t vi : it->second) {
                        const glm::dvec3 d = glm::dvec3(mesh->positions[vi]) - glm::dvec3(p);
                        if (glm::dot(d, d) <= tol * tol) {
                            found = vi;
                            break;
                        }
                    }
                }
        if (found == kInvalid) {
            found = uint32_t(mesh->positions.size());
            mesh->positions.push_back(p);
            grid[QuantKey{cx, cy, cz}].push_back(found);
        }
        remap[i] = found;
    }

    const float minArea2 = float(q * q) * 1e-4f;
    mesh->triangles.reserve(soup.size() / 3);
    for (size_t i = 0; i + 2 < soup.size(); i += 3) {
        const glm::uvec3 t(remap[i], remap[i + 1], remap[i + 2]);
        if (t.x == t.y || t.y == t.z || t.x == t.z) continue;
        const glm::vec3 c = glm::cross(mesh->positions[t.y] - mesh->positions[t.x],
                                       mesh->positions[t.z] - mesh->positions[t.x]);
        if (glm::dot(c, c) <= minArea2) continue;
        mesh->triangles.push_back(t);
    }

    mesh->buildTopology();
    return mesh;
}

glm::vec3 Mesh::triCentroid(uint32_t tri) const {
    return (positions[triangles[tri].x] + positions[triangles[tri].y] + positions[triangles[tri].z]) / 3.0f;
}

void Mesh::buildEdges() {
    const uint32_t nt = uint32_t(triangles.size());
    edges.clear();
    triEdges.assign(nt, glm::uvec3(kInvalid));
    std::unordered_map<uint64_t, uint32_t> edgeMap;
    edgeMap.reserve(size_t(nt) * 2);
    for (uint32_t t = 0; t < nt; ++t) {
        for (int k = 0; k < 3; ++k) {
            const uint32_t a = triangles[t][k], b = triangles[t][(k + 1) % 3];
            auto [it, inserted] = edgeMap.try_emplace(edgeKey(a, b), uint32_t(edges.size()));
            if (inserted) {
                Edge e;
                e.v[0] = std::min(a, b);
                e.v[1] = std::max(a, b);
                edges.push_back(e);
            }
            Edge& e = edges[it->second];
            if (e.faceCount < 2) e.tri[e.faceCount] = t;
            e.faceCount++;
            triEdges[t][k] = it->second;
        }
    }
}

// Makes triangle winding consistent across manifold edges (BFS per connected component) and
// flips each closed component so that its signed volume is positive (normals point outward).
void Mesh::orientConsistently() {
    const uint32_t nt = uint32_t(triangles.size());
    std::vector<uint32_t> comp(nt, kInvalid);
    std::vector<uint8_t> flip(nt, 0);
    std::vector<uint32_t> stack;
    bool conflict = false, open = false;
    uint32_t compCount = 0;
    for (const Edge& e : edges)
        if (e.faceCount != 2) open = true;

    // orientation of triangle t after the pending flip
    auto dir = [&](uint32_t t, uint32_t a, uint32_t b) {
        const int d = edgeDirection(triangles[t], a, b);
        return flip[t] ? -d : d;
    };
    for (uint32_t seed = 0; seed < nt; ++seed) {
        if (comp[seed] != kInvalid) continue;
        const uint32_t c = compCount++;
        comp[seed] = c;
        stack.assign(1, seed);
        while (!stack.empty()) {
            const uint32_t t = stack.back();
            stack.pop_back();
            for (int k = 0; k < 3; ++k) {
                const Edge& e = edges[triEdges[t][k]];
                if (e.faceCount != 2) continue;
                const uint32_t nb = e.tri[0] == t ? e.tri[1] : e.tri[0];
                const bool consistent = dir(t, e.v[0], e.v[1]) != dir(nb, e.v[0], e.v[1]);
                if (comp[nb] == kInvalid) {
                    comp[nb] = c;
                    if (!consistent) flip[nb] ^= 1;
                    stack.push_back(nb);
                } else if (!consistent) {
                    conflict = true; // non-orientable surface
                }
            }
        }
    }
    // signed volume per component
    std::vector<double> vol(compCount, 0.0);
    for (uint32_t t = 0; t < nt; ++t) {
        glm::dvec3 p0 = positions[triangles[t].x], p1 = positions[triangles[t].y], p2 = positions[triangles[t].z];
        if (flip[t]) std::swap(p1, p2);
        vol[comp[t]] += glm::dot(p0, glm::cross(p1, p2)) / 6.0;
    }
    flippedTriangles = 0;
    for (uint32_t t = 0; t < nt; ++t) {
        const bool f = (flip[t] != 0) != (vol[comp[t]] < 0.0);
        if (f) {
            std::swap(triangles[t].y, triangles[t].z);
            flippedTriangles++;
        }
    }
    outwardOriented = !open && !conflict;
}

void Mesh::buildTopology() {
    const uint32_t nt = uint32_t(triangles.size());
    bounds = AABB{};
    for (const auto& p : positions) bounds.expand(p);

    buildEdges();
    orientConsistently();
    if (flippedTriangles > 0) buildEdges(); // triEdges slot order follows the winding

    triNormals.resize(nt);
    triAreas.resize(nt);
    totalArea = 0.0f;
    for (uint32_t i = 0; i < nt; ++i) {
        const glm::vec3 c = glm::cross(triVertex(i, 1) - triVertex(i, 0), triVertex(i, 2) - triVertex(i, 0));
        const float len = glm::length(c);
        triNormals[i] = len > 0.0f ? c / len : glm::vec3(0, 0, 1);
        triAreas[i] = 0.5f * len;
        totalArea += triAreas[i];
    }

    // Dihedral angles, robust to (remaining) inconsistent triangle winding.
    for (Edge& e : edges) {
        if (e.faceCount != 2) {
            e.angleDeg = 180.0f;
            continue;
        }
        glm::vec3 n0 = triNormals[e.tri[0]], n1 = triNormals[e.tri[1]];
        const int d0 = edgeDirection(triangles[e.tri[0]], e.v[0], e.v[1]);
        const int d1 = edgeDirection(triangles[e.tri[1]], e.v[0], e.v[1]);
        if (d0 == d1) n1 = -n1;
        e.angleDeg = glm::degrees(std::acos(std::clamp(glm::dot(n0, n1), -1.0f, 1.0f)));
    }

    bvh.build(*this);
    buildRegions(planarRegions, 0.5f, true);
    setFeatureAngle(featureAngleDeg_);
}

void Mesh::setFeatureAngle(float degrees) {
    featureAngleDeg_ = degrees;
    featureEdges.clear();
    for (uint32_t i = 0; i < edges.size(); ++i)
        if (edges[i].faceCount != 2 || edges[i].angleDeg >= degrees) featureEdges.push_back(i);
    buildRegions(smoothRegions, degrees, false);
    gpuVersion++;
}

void Mesh::buildRegions(Regions& out, float angleTolDeg, bool compareToSeed) const {
    const uint32_t nt = uint32_t(triangles.size());
    out.regionOfTri.assign(nt, kInvalid);
    out.offsets.clear();
    out.tris.clear();
    out.tris.reserve(nt);
    const float seedCos = std::cos(glm::radians(angleTolDeg * 2.0f));

    std::vector<uint32_t> stack;
    uint32_t regionCount = 0;
    for (uint32_t seed = 0; seed < nt; ++seed) {
        if (out.regionOfTri[seed] != kInvalid) continue;
        const uint32_t rid = regionCount++;
        out.offsets.push_back(uint32_t(out.tris.size()));
        const glm::vec3 seedN = triNormals[seed];
        stack.clear();
        stack.push_back(seed);
        out.regionOfTri[seed] = rid;
        while (!stack.empty()) {
            const uint32_t t = stack.back();
            stack.pop_back();
            out.tris.push_back(t);
            for (int k = 0; k < 3; ++k) {
                const Edge& e = edges[triEdges[t][k]];
                if (e.faceCount != 2 || e.angleDeg >= angleTolDeg) continue;
                const uint32_t nb = e.tri[0] == t ? e.tri[1] : e.tri[0];
                if (out.regionOfTri[nb] != kInvalid) continue;
                if (compareToSeed && std::abs(glm::dot(triNormals[nb], seedN)) < seedCos) continue;
                out.regionOfTri[nb] = rid;
                stack.push_back(nb);
            }
        }
    }
    out.offsets.push_back(uint32_t(out.tris.size()));
}

void Mesh::translateVertices(const glm::vec3& offset) {
    for (auto& p : positions) p -= offset;
    bounds.min -= offset;
    bounds.max -= offset;
    bvh.build(*this);
    gpuVersion++;
}

// ---------------------------------------------------------------------------------------------
// Queries
// ---------------------------------------------------------------------------------------------

bool Mesh::raycast(const Ray& ray, RayHit& hit, float tMax) const {
    if (bvh.empty()) return false;
    const glm::vec3 invDir = 1.0f / ray.dir;
    const auto& nodes = bvh.nodes();
    const auto& order = bvh.triOrder();
    hit = RayHit{};
    hit.t = tMax;

    uint32_t stack[96];
    int sp = 0;
    float tn;
    if (!rayAabb(ray.origin, invDir, nodes[0].box, hit.t, tn)) return false;
    stack[sp++] = 0;
    while (sp > 0) {
        const BVH::Node& node = nodes[stack[--sp]];
        if (node.isLeaf()) {
            for (uint32_t i = node.leftOrFirst; i < node.leftOrFirst + node.count; ++i) {
                const uint32_t t = order[i];
                float tt, u, v;
                if (rayTriangle(ray, triVertex(t, 0), triVertex(t, 1), triVertex(t, 2), tt, u, v) && tt < hit.t) {
                    hit.t = tt;
                    hit.tri = t;
                    hit.u = u;
                    hit.v = v;
                }
            }
            continue;
        }
        const uint32_t l = node.leftOrFirst, r = l + 1;
        float tl, tr;
        const bool hl = rayAabb(ray.origin, invDir, nodes[l].box, hit.t, tl);
        const bool hr = rayAabb(ray.origin, invDir, nodes[r].box, hit.t, tr);
        if (hl && hr) {
            // push far child first so the near child is processed first
            if (tl < tr) { stack[sp++] = r; stack[sp++] = l; }
            else { stack[sp++] = l; stack[sp++] = r; }
        } else if (hl) stack[sp++] = l;
        else if (hr) stack[sp++] = r;
    }
    return hit.hit();
}

int Mesh::countHits(const Ray& ray) const {
    if (bvh.empty()) return 0;
    const glm::vec3 invDir = 1.0f / ray.dir;
    const auto& nodes = bvh.nodes();
    const auto& order = bvh.triOrder();
    int count = 0;
    uint32_t stack[96];
    int sp = 0;
    stack[sp++] = 0;
    const float inf = std::numeric_limits<float>::max();
    while (sp > 0) {
        const BVH::Node& node = nodes[stack[--sp]];
        float tn;
        if (!rayAabb(ray.origin, invDir, node.box, inf, tn)) continue;
        if (node.isLeaf()) {
            for (uint32_t i = node.leftOrFirst; i < node.leftOrFirst + node.count; ++i) {
                const uint32_t t = order[i];
                float tt, u, v;
                if (rayTriangle(ray, triVertex(t, 0), triVertex(t, 1), triVertex(t, 2), tt, u, v)) count++;
            }
        } else {
            stack[sp++] = node.leftOrFirst;
            stack[sp++] = node.leftOrFirst + 1;
        }
    }
    return count;
}

bool Mesh::containsPoint(const glm::vec3& p) const {
    if (!bounds.valid()) return false;
    if (p.x < bounds.min.x || p.y < bounds.min.y || p.z < bounds.min.z || p.x > bounds.max.x ||
        p.y > bounds.max.y || p.z > bounds.max.z)
        return false;
    // Two skewed directions, majority vote is overkill; require both to agree to reduce
    // false positives caused by rays grazing edges.
    const Ray r1{p, glm::normalize(glm::vec3(0.5773f, 0.5791f, 0.5755f))};
    const Ray r2{p, glm::normalize(glm::vec3(-0.6123f, 0.3311f, -0.7180f))};
    return (countHits(r1) & 1) && (countHits(r2) & 1);
}

} // namespace pegasus
