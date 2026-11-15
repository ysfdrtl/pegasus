#include "motion/Seam.h"

#include <cmath>

namespace pegasus {

glm::mat4 Seam::frameAt(float s) const {
    glm::mat4 m(1.0f);
    m[0] = glm::vec4(X, 0.0f);
    m[1] = glm::vec4(Y, 0.0f);
    m[2] = glm::vec4(N, 0.0f);
    m[3] = glm::vec4(pointAt(s), 1.0f);
    return m;
}

bool Seam::operator==(const Seam& o) const {
    auto eq = [](const glm::vec3& a, const glm::vec3& b) { return glm::length(a - b) < 1e-5f; };
    return valid == o.valid && eq(start, o.start) && eq(end, o.end) && eq(X, o.X) && eq(N, o.N) && edgeCount == o.edgeCount;
}

bool buildSeam(const std::vector<SeamEdgeGeom>& edges, bool reverse, const glm::vec3& viewDir, Seam& out, std::string& err) {
    out = Seam{};
    if (edges.empty() || edges.size() > 2) {
        err = "Yol için 1 veya 2 kenar seçin.";
        return false;
    }
    glm::vec3 start, end, N(0.0f);
    float gap = 0.0f;
    if (edges.size() == 1) {
        start = edges[0].a;
        end = edges[0].b;
        for (const auto& n : edges[0].faceNormals) N += n;
    } else {
        SeamEdgeGeom A = edges[0], B = edges[1];
        if (glm::dot(A.b - A.a, B.b - B.a) < 0.0f) std::swap(B.a, B.b);
        start = 0.5f * (A.a + B.a);
        end = 0.5f * (A.b + B.b);
        const glm::vec3 lateral = 0.5f * (B.a + B.b) - 0.5f * (A.a + A.b);
        gap = glm::length(lateral);
        const glm::vec3 latDir = gap > 1e-6f ? lateral / gap : glm::vec3(0.0f);
        // Work surface: the adjacent faces that are not the faces bounding the gap itself,
        // choosing the pair that agrees best between the two edges.
        float best = -2.0f;
        for (const auto& na : A.faceNormals)
            for (const auto& nb : B.faceNormals) {
                if (gap > 1e-6f && (std::abs(glm::dot(na, latDir)) > 0.7f || std::abs(glm::dot(nb, latDir)) > 0.7f)) continue;
                const float d = glm::dot(na, nb);
                if (d > best) {
                    best = d;
                    N = na + nb;
                }
            }
        out.gap = gap;
        if (glm::length(N) < 1e-6f && gap > 1e-6f) N = glm::cross(glm::normalize(end - start), latDir);
    }
    const float len = glm::length(end - start);
    if (len < 1e-4f) {
        err = "Seçilen kenarlar çok kısa veya çakışık.";
        return false;
    }
    glm::vec3 Y = (end - start) / len;
    // default direction: from the lower end upwards; horizontal seams keep the edge order
    const bool flipUp = std::abs(Y.z) > 1e-3f && Y.z < 0.0f;
    if (flipUp != reverse) {
        std::swap(start, end);
        Y = -Y;
    }
    N -= glm::dot(N, Y) * Y;
    if (glm::length(N) < 1e-5f) {
        // no usable face normal: face the viewer
        N = -viewDir - glm::dot(-viewDir, Y) * Y;
        if (glm::length(N) < 1e-5f) N = glm::abs(Y.z) < 0.9f ? glm::vec3(0, 0, 1) : glm::vec3(1, 0, 0);
        N -= glm::dot(N, Y) * Y;
    }
    N = glm::normalize(N);
    out.valid = true;
    out.start = start;
    out.end = end;
    out.Y = Y;
    out.N = N;
    out.X = glm::cross(Y, N); // right handed: X x Y = N
    out.length = len;
    out.gap = gap;
    out.edgeCount = int(edges.size());
    return true;
}

} // namespace pegasus
