#include "geom/Collision.h"

#include <algorithm>
#include <chrono>

namespace pegasus {

namespace {

float boxVolume(const AABB& b) {
    const glm::vec3 s = b.size();
    return s.x * s.y * s.z;
}

// Exact touching (within eps) is "contact", not collision: objects resting on each other are fine.
// But coincident faces whose outward normals point the same way mean the volumes overlap
// (e.g. two equal boxes slid into each other), so those count as penetration.
bool trianglesPenetrate(const glm::vec3 ta[3], const glm::vec3 tb[3], float eps, float bNormalSign, bool oriented) {
    if (triTriOverlap(ta, tb, eps)) return true;
    glm::vec3 na = glm::cross(ta[1] - ta[0], ta[2] - ta[0]);
    glm::vec3 nb = glm::cross(tb[1] - tb[0], tb[2] - tb[0]) * bNormalSign;
    const float la = glm::length(na), lb = glm::length(nb);
    if (la <= 0.0f || lb <= 0.0f) return false;
    na /= la;
    nb /= lb;
    const float d = glm::dot(na, nb);
    if (std::abs(d) < 0.99999f) return false;
    for (int k = 0; k < 3; ++k)
        if (std::abs(glm::dot(tb[k] - ta[0], na)) > eps) return false;
    if (oriented && d < 0.0f) return false; // opposite faces touching: contact only
    return triTriCoplanarOverlap(ta, tb, eps);
}

bool surfaceIntersect(const Mesh& A, const Mesh& B, const glm::mat4& bToA, float eps,
                      std::vector<std::pair<uint32_t, uint32_t>>* pairs, size_t maxPairs, bool* truncated,
                      bool touchMode = false) {
    const float bSign = glm::determinant(glm::mat3(bToA)) < 0.0f ? -1.0f : 1.0f;
    const bool oriented = A.outwardOriented && B.outwardOriented;
    if (A.bvh.empty() || B.bvh.empty()) return false;
    const auto& na = A.bvh.nodes();
    const auto& nb = B.bvh.nodes();
    const auto& oa = A.bvh.triOrder();
    const auto& ob = B.bvh.triOrder();

    bool found = false;
    std::vector<std::pair<uint32_t, uint32_t>> stack;
    stack.reserve(256);
    stack.emplace_back(0u, 0u);
    glm::vec3 ta[3], tb[3];

    while (!stack.empty()) {
        const auto [ia, ib] = stack.back();
        stack.pop_back();
        const BVH::Node& nodeA = na[ia];
        const BVH::Node& nodeB = nb[ib];
        AABB boxB = nodeB.box.transformed(bToA);
        if (touchMode) {
            boxB.min += eps; // eps is negative in touch mode: grow the box by the tolerance
            boxB.max -= eps;
        }
        if (!nodeA.box.overlaps(boxB)) continue;

        if (nodeA.isLeaf() && nodeB.isLeaf()) {
            for (uint32_t j = nodeB.leftOrFirst; j < nodeB.leftOrFirst + nodeB.count; ++j) {
                const uint32_t tbIdx = ob[j];
                for (int k = 0; k < 3; ++k) tb[k] = glm::vec3(bToA * glm::vec4(B.triVertex(tbIdx, k), 1.0f));
                AABB tbBox;
                for (int k = 0; k < 3; ++k) tbBox.expand(tb[k]);
                if (touchMode) {
                    tbBox.min += eps;
                    tbBox.max -= eps;
                }
                if (!nodeA.box.overlaps(tbBox)) continue;
                for (uint32_t i = nodeA.leftOrFirst; i < nodeA.leftOrFirst + nodeA.count; ++i) {
                    const uint32_t taIdx = oa[i];
                    for (int k = 0; k < 3; ++k) ta[k] = A.triVertex(taIdx, k);
                    if (touchMode ? !triTriOverlap(ta, tb, eps) : !trianglesPenetrate(ta, tb, eps, bSign, oriented)) continue;
                    found = true;
                    if (!pairs) return true;
                    if (pairs->size() >= maxPairs) {
                        if (truncated) *truncated = true;
                        return true;
                    }
                    pairs->emplace_back(taIdx, tbIdx);
                }
            }
            continue;
        }
        const bool descendA = nodeB.isLeaf() || (!nodeA.isLeaf() && boxVolume(nodeA.box) >= boxVolume(boxB));
        if (descendA) {
            stack.emplace_back(nodeA.leftOrFirst, ib);
            stack.emplace_back(nodeA.leftOrFirst + 1, ib);
        } else {
            stack.emplace_back(ia, nodeB.leftOrFirst);
            stack.emplace_back(ia, nodeB.leftOrFirst + 1);
        }
    }
    return found;
}

} // namespace

bool CollisionSystem::testPair(const Mesh& a, const glm::mat4& ma, const Mesh& b, const glm::mat4& mb,
                               std::vector<std::pair<uint32_t, uint32_t>>* pairs, size_t maxPairs, bool* truncated,
                               bool* containment) {
    if (containment) *containment = false;
    const glm::mat4 bToA = glm::inverse(ma) * mb;
    const float scale = std::max(a.bounds.diagonal(), b.bounds.transformed(bToA).diagonal());
    const float eps = std::max(scale * 1e-5f, 1e-7f);
    if (surfaceIntersect(a, b, bToA, eps, pairs, maxPairs, truncated)) return true;

    // No surface contact: check whether one body lies completely inside the other.
    if (!a.positions.empty() && !b.positions.empty()) {
        const glm::mat4 aToB = glm::inverse(bToA);
        const glm::vec3 pa = glm::vec3(aToB * glm::vec4(a.positions[0], 1.0f));
        const glm::vec3 pb = glm::vec3(bToA * glm::vec4(b.positions[0], 1.0f));
        if (b.containsPoint(pa) || a.containsPoint(pb)) {
            if (containment) *containment = true;
            return true;
        }
    }
    return false;
}

bool CollisionSystem::touching(const Mesh& a, const glm::mat4& ma, const Mesh& b, const glm::mat4& mb, float tolerance) {
    const glm::mat4 bToA = glm::inverse(ma) * mb;
    return surfaceIntersect(a, b, bToA, -std::max(tolerance, 1e-6f), nullptr, 0, nullptr, true);
}

void CollisionSystem::update(const Scene& scene, bool force) {
    if (!enabled) {
        if (!collisions_.empty()) clear();
        lastVersion_ = 0;
        return;
    }
    if (!force && scene.version() == lastVersion_) return;
    lastVersion_ = scene.version();
    const auto t0 = std::chrono::steady_clock::now();

    collisions_.clear();
    collidingIds_.clear();
    const auto& objs = scene.objects();
    std::vector<const SceneObject*> active;
    std::vector<AABB> boxes;
    std::vector<glm::mat4> mats;
    for (const auto& o : objs) {
        if (!o->visible || !o->collidable) continue;
        active.push_back(o.get());
        mats.push_back(o->matrix());
        boxes.push_back(o->mesh->bounds.transformed(mats.back()));
    }
    for (size_t i = 0; i < active.size(); ++i) {
        for (size_t j = i + 1; j < active.size(); ++j) {
            if (!boxes[i].overlaps(boxes[j])) continue;
            if (ignorePair && ignorePair(*active[i], *active[j])) continue;
            CollisionInfo info;
            info.a = active[i]->id;
            info.b = active[j]->id;
            if (testPair(*active[i]->mesh, mats[i], *active[j]->mesh, mats[j], &info.triPairs, maxTriPairsPerPair,
                         &info.truncated, &info.containment)) {
                collidingIds_.push_back(info.a);
                collidingIds_.push_back(info.b);
                collisions_.push_back(std::move(info));
            }
        }
    }
    std::sort(collidingIds_.begin(), collidingIds_.end());
    collidingIds_.erase(std::unique(collidingIds_.begin(), collidingIds_.end()), collidingIds_.end());
    lastMs_ = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count();
}

void CollisionSystem::clear() {
    collisions_.clear();
    collidingIds_.clear();
}

bool CollisionSystem::isColliding(uint32_t id) const {
    return std::binary_search(collidingIds_.begin(), collidingIds_.end(), id);
}

bool CollisionSystem::collidesAt(const Scene& scene, const SceneObject& obj, const glm::mat4& m) const {
    if (!obj.collidable || !obj.visible) return false;
    const AABB box = obj.mesh->bounds.transformed(m);
    for (const auto& o : scene.objects()) {
        if (o.get() == &obj || !o->visible || !o->collidable) continue;
        if (ignorePair && ignorePair(obj, *o)) continue;
        const glm::mat4 mo = o->matrix();
        if (!box.overlaps(o->mesh->bounds.transformed(mo))) continue;
        if (testPair(*obj.mesh, m, *o->mesh, mo, nullptr, 0, nullptr, nullptr)) return true;
    }
    return false;
}

} // namespace pegasus
