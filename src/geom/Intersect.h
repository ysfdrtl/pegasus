#pragma once
// Basic geometric primitives and intersection tests.

#include <glm/glm.hpp>
#include <limits>

namespace pegasus {

struct Ray {
    glm::vec3 origin{0.0f};
    glm::vec3 dir{0.0f, 0.0f, 1.0f}; // not necessarily normalized; t is measured in units of |dir|
};

struct AABB {
    glm::vec3 min{std::numeric_limits<float>::max()};
    glm::vec3 max{-std::numeric_limits<float>::max()};

    bool valid() const { return min.x <= max.x && min.y <= max.y && min.z <= max.z; }
    void expand(const glm::vec3& p) { min = glm::min(min, p); max = glm::max(max, p); }
    void expand(const AABB& b) { min = glm::min(min, b.min); max = glm::max(max, b.max); }
    glm::vec3 center() const { return (min + max) * 0.5f; }
    glm::vec3 extent() const { return (max - min) * 0.5f; }
    glm::vec3 size() const { return max - min; }
    float diagonal() const { return valid() ? glm::length(max - min) : 0.0f; }
    bool overlaps(const AABB& o) const {
        return min.x <= o.max.x && max.x >= o.min.x && min.y <= o.max.y && max.y >= o.min.y &&
               min.z <= o.max.z && max.z >= o.min.z;
    }
    // Conservative AABB of this box after an affine transform.
    AABB transformed(const glm::mat4& m) const;
};

// Möller–Trumbore, double sided. Returns t and barycentrics (u, v) on hit.
bool rayTriangle(const Ray& ray, const glm::vec3& a, const glm::vec3& b, const glm::vec3& c,
                 float& t, float& u, float& v);

// Slab test. invDir = 1/dir. Returns entry distance in tNear.
bool rayAabb(const glm::vec3& origin, const glm::vec3& invDir, const AABB& box, float tMax, float& tNear);

// Separating axis test between two triangles. Contacts within `eps` (touching faces/edges)
// are NOT reported as overlap so that objects resting on each other are not "colliding".
bool triTriOverlap(const glm::vec3 a[3], const glm::vec3 b[3], float eps);

// For two (nearly) coplanar triangles: do they overlap with positive area (beyond `eps`)?
bool triTriCoplanarOverlap(const glm::vec3 a[3], const glm::vec3 b[3], float eps);

// Closest points between the infinite line (p, p + d) and the segment [a, b].
// Returns the parameter s in [0,1] along the segment.
float closestSegmentParamToLine(const glm::vec3& p, const glm::vec3& d, const glm::vec3& a, const glm::vec3& b);

// Distance from point p to 2D segment [a, b], also returns the param along the segment.
float pointSegmentDistance2D(const glm::vec2& p, const glm::vec2& a, const glm::vec2& b, float& s);

} // namespace pegasus
