#include "geom/Intersect.h"

#include <algorithm>
#include <cmath>

namespace pegasus {

AABB AABB::transformed(const glm::mat4& m) const {
    if (!valid()) return {};
    const glm::vec3 c = glm::vec3(m * glm::vec4(center(), 1.0f));
    const glm::vec3 e = extent();
    glm::vec3 ne;
    for (int r = 0; r < 3; ++r)
        ne[r] = std::abs(m[0][r]) * e.x + std::abs(m[1][r]) * e.y + std::abs(m[2][r]) * e.z;
    AABB out;
    out.min = c - ne;
    out.max = c + ne;
    return out;
}

bool rayTriangle(const Ray& ray, const glm::vec3& a, const glm::vec3& b, const glm::vec3& c, float& t,
                 float& u, float& v) {
    const glm::vec3 e1 = b - a;
    const glm::vec3 e2 = c - a;
    const glm::vec3 p = glm::cross(ray.dir, e2);
    const float det = glm::dot(e1, p);
    if (std::abs(det) < 1e-20f) return false;
    const float inv = 1.0f / det;
    const glm::vec3 s = ray.origin - a;
    u = glm::dot(s, p) * inv;
    if (u < 0.0f || u > 1.0f) return false;
    const glm::vec3 q = glm::cross(s, e1);
    v = glm::dot(ray.dir, q) * inv;
    if (v < 0.0f || u + v > 1.0f) return false;
    t = glm::dot(e2, q) * inv;
    return t > 0.0f;
}

bool rayAabb(const glm::vec3& origin, const glm::vec3& invDir, const AABB& box, float tMax, float& tNear) {
    float t0 = 0.0f, t1 = tMax;
    for (int i = 0; i < 3; ++i) {
        float tn = (box.min[i] - origin[i]) * invDir[i];
        float tf = (box.max[i] - origin[i]) * invDir[i];
        if (tn > tf) std::swap(tn, tf);
        // NaN-safe comparisons (0 * inf)
        t0 = tn > t0 ? tn : t0;
        t1 = tf < t1 ? tf : t1;
        if (t0 > t1) return false;
    }
    tNear = t0;
    return true;
}

namespace {
inline void project(const glm::vec3 tri[3], const glm::vec3& axis, float& mn, float& mx) {
    const float d0 = glm::dot(tri[0], axis), d1 = glm::dot(tri[1], axis), d2 = glm::dot(tri[2], axis);
    mn = std::min(d0, std::min(d1, d2));
    mx = std::max(d0, std::max(d1, d2));
}
} // namespace

bool triTriOverlap(const glm::vec3 a[3], const glm::vec3 b[3], float eps) {
    const glm::vec3 ea[3] = {a[1] - a[0], a[2] - a[1], a[0] - a[2]};
    const glm::vec3 eb[3] = {b[1] - b[0], b[2] - b[1], b[0] - b[2]};
    const glm::vec3 na = glm::cross(ea[0], ea[1]);
    const glm::vec3 nb = glm::cross(eb[0], eb[1]);

    // Returns true if `axis` separates the triangles (or is degenerate -> ignored).
    auto separated = [&](glm::vec3 axis, float refLen2) {
        const float len2 = glm::dot(axis, axis);
        if (len2 <= 1e-12f * refLen2 || len2 <= 0.0f) return false;
        axis *= 1.0f / std::sqrt(len2);
        float amin, amax, bmin, bmax;
        project(a, axis, amin, amax);
        project(b, axis, bmin, bmax);
        return amax <= bmin + eps || bmax <= amin + eps;
    };

    const float la[3] = {glm::dot(ea[0], ea[0]), glm::dot(ea[1], ea[1]), glm::dot(ea[2], ea[2])};
    const float lb[3] = {glm::dot(eb[0], eb[0]), glm::dot(eb[1], eb[1]), glm::dot(eb[2], eb[2])};

    if (separated(na, la[0] * la[1])) return false;
    if (separated(nb, lb[0] * lb[1])) return false;
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j)
            if (separated(glm::cross(ea[i], eb[j]), la[i] * lb[j])) return false;
    // In-plane edge normals (needed for coplanar configurations).
    const float lna = glm::dot(na, na), lnb = glm::dot(nb, nb);
    for (int i = 0; i < 3; ++i) {
        if (separated(glm::cross(na, ea[i]), lna * la[i])) return false;
        if (separated(glm::cross(nb, eb[i]), lnb * lb[i])) return false;
    }
    return true;
}

bool triTriCoplanarOverlap(const glm::vec3 a[3], const glm::vec3 b[3], float eps) {
    const glm::vec3 ea[3] = {a[1] - a[0], a[2] - a[1], a[0] - a[2]};
    const glm::vec3 eb[3] = {b[1] - b[0], b[2] - b[1], b[0] - b[2]};
    glm::vec3 n = glm::cross(ea[0], ea[1]);
    const float nl = glm::length(n);
    if (nl <= 0.0f) return false;
    n /= nl;
    for (int i = 0; i < 6; ++i) {
        glm::vec3 axis = glm::cross(n, i < 3 ? ea[i] : eb[i - 3]);
        const float len = glm::length(axis);
        if (len <= 0.0f) continue;
        axis /= len;
        float amin, amax, bmin, bmax;
        project(a, axis, amin, amax);
        project(b, axis, bmin, bmax);
        if (amax <= bmin + eps || bmax <= amin + eps) return false;
    }
    return true;
}

float closestSegmentParamToLine(const glm::vec3& p, const glm::vec3& d, const glm::vec3& a, const glm::vec3& b) {
    const glm::vec3 u = b - a;
    const glm::vec3 w = a - p;
    const float A = glm::dot(u, u), B = glm::dot(u, d), C = glm::dot(d, d);
    const float D = glm::dot(u, w), E = glm::dot(d, w);
    const float denom = A * C - B * B;
    float s = 0.0f;
    if (denom > 1e-12f * A * C) s = (B * E - C * D) / denom;
    else if (A > 0.0f) s = -D / A;
    return std::clamp(s, 0.0f, 1.0f);
}

float pointSegmentDistance2D(const glm::vec2& p, const glm::vec2& a, const glm::vec2& b, float& s) {
    const glm::vec2 ab = b - a;
    const float len2 = glm::dot(ab, ab);
    s = len2 > 0.0f ? std::clamp(glm::dot(p - a, ab) / len2, 0.0f, 1.0f) : 0.0f;
    return glm::length(p - (a + ab * s));
}

} // namespace pegasus
