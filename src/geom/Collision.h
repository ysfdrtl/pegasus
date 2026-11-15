#pragma once
// Mesh-vs-mesh collision detection (exact triangle level, BVH accelerated).
// Broad phase: world AABBs. Narrow phase: BVH-vs-BVH traversal + triangle SAT test.
// Full containment (one closed body completely inside another) is detected with a
// point-in-mesh parity test.

#include "scene/Scene.h"

#include <functional>
#include <utility>
#include <vector>

namespace pegasus {

struct CollisionInfo {
    uint32_t a = 0, b = 0;                               // object ids
    std::vector<std::pair<uint32_t, uint32_t>> triPairs;  // intersecting triangles (a-tri, b-tri), capped
    bool truncated = false;                              // triPairs hit the cap
    bool containment = false;                            // no surface contact, one body inside the other
};

class CollisionSystem {
public:
    bool enabled = true;
    size_t maxTriPairsPerPair = 4096;
    // Optional filter: return true for object pairs that must never be reported
    // (e.g. robot parts that are permanently in contact in the assembly).
    std::function<bool(const SceneObject&, const SceneObject&)> ignorePair;

    // Recomputes all pairs if the scene changed since the last call (or `force`).
    void update(const Scene& scene, bool force = false);
    void clear();

    const std::vector<CollisionInfo>& collisions() const { return collisions_; }
    bool isColliding(uint32_t id) const;
    double lastUpdateMs() const { return lastMs_; }

    // Would `obj` collide with any other collidable object if it had world matrix `m`?
    bool collidesAt(const Scene& scene, const SceneObject& obj, const glm::mat4& m) const;

    // Exact test between two meshes with world matrices. If `pairs` is null returns at the first
    // intersecting triangle pair. Containment is only checked when no surface contact is found.
    static bool testPair(const Mesh& a, const glm::mat4& ma, const Mesh& b, const glm::mat4& mb,
                         std::vector<std::pair<uint32_t, uint32_t>>* pairs, size_t maxPairs, bool* truncated,
                         bool* containment);

    // True if the two meshes touch or overlap (gap <= tolerance). Used for assembly analysis.
    static bool touching(const Mesh& a, const glm::mat4& ma, const Mesh& b, const glm::mat4& mb, float tolerance);

private:
    std::vector<CollisionInfo> collisions_;
    std::vector<uint32_t> collidingIds_;
    uint64_t lastVersion_ = 0;
    double lastMs_ = 0.0;
};

} // namespace pegasus
