#include "scene/Primitives.h"

#include <cmath>

namespace pegasus {

namespace {
void quad(std::vector<glm::vec3>& s, glm::vec3 a, glm::vec3 b, glm::vec3 c, glm::vec3 d) {
    s.insert(s.end(), {a, b, c, a, c, d});
}
} // namespace

MeshPtr makeBox(const glm::vec3& size, const std::string& name) {
    const glm::vec3 h(size.x * 0.5f, size.y * 0.5f, size.z);
    const glm::vec3 p[8] = {{-h.x, -h.y, 0}, {h.x, -h.y, 0}, {h.x, h.y, 0}, {-h.x, h.y, 0},
                            {-h.x, -h.y, h.z}, {h.x, -h.y, h.z}, {h.x, h.y, h.z}, {-h.x, h.y, h.z}};
    std::vector<glm::vec3> s;
    quad(s, p[0], p[3], p[2], p[1]); // bottom
    quad(s, p[4], p[5], p[6], p[7]); // top
    quad(s, p[0], p[1], p[5], p[4]);
    quad(s, p[1], p[2], p[6], p[5]);
    quad(s, p[2], p[3], p[7], p[6]);
    quad(s, p[3], p[0], p[4], p[7]);
    return Mesh::fromSoup(s, name);
}

MeshPtr makeCylinder(float r, float height, int seg, const std::string& name) {
    std::vector<glm::vec3> s;
    const glm::vec3 cb(0, 0, 0), ct(0, 0, height);
    for (int i = 0; i < seg; ++i) {
        const float a0 = 2.0f * float(M_PI) * float(i) / float(seg);
        const float a1 = 2.0f * float(M_PI) * float(i + 1) / float(seg);
        const glm::vec3 b0(r * std::cos(a0), r * std::sin(a0), 0), b1(r * std::cos(a1), r * std::sin(a1), 0);
        const glm::vec3 t0 = b0 + glm::vec3(0, 0, height), t1 = b1 + glm::vec3(0, 0, height);
        s.insert(s.end(), {cb, b1, b0});
        s.insert(s.end(), {ct, t0, t1});
        quad(s, b0, b1, t1, t0);
    }
    return Mesh::fromSoup(s, name);
}

MeshPtr makeSphere(float r, int rings, int seg, const std::string& name) {
    std::vector<glm::vec3> s;
    auto pt = [&](int i, int j) {
        const float th = float(M_PI) * float(i) / float(rings);
        const float ph = 2.0f * float(M_PI) * float(j) / float(seg);
        return glm::vec3(r * std::sin(th) * std::cos(ph), r * std::sin(th) * std::sin(ph), r - r * std::cos(th));
    };
    for (int i = 0; i < rings; ++i)
        for (int j = 0; j < seg; ++j) {
            const glm::vec3 a = pt(i, j), b = pt(i + 1, j), c = pt(i + 1, j + 1), d = pt(i, j + 1);
            if (i != 0) s.insert(s.end(), {a, b, d});
            if (i != rings - 1) s.insert(s.end(), {b, c, d});
        }
    return Mesh::fromSoup(s, name);
}

} // namespace pegasus
