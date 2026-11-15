#pragma once
// Work path ("seam") built from selected edges, with its local frame:
//   X = lateral (across the seam)   Y = travel direction   N (Z) = surface normal, pointing out
// Two edges -> the seam runs along the middle between them (e.g. the 0.5 mm gap between two
// cubes), starting at the optimal point = midpoint of the two edge starts.
// One edge  -> the path runs along the edge, normal = bisector of the adjacent faces.

#include <glm/glm.hpp>

#include <string>
#include <vector>

namespace pegasus {

struct SeamEdgeGeom {
    glm::vec3 a{0.0f}, b{0.0f};
    std::vector<glm::vec3> faceNormals; // outward normals of the adjacent faces (world)
};

struct Seam {
    bool valid = false;
    glm::vec3 start{0.0f}, end{0.0f};
    glm::vec3 X{1, 0, 0}, Y{0, 1, 0}, N{0, 0, 1};
    float length = 0.0f;
    float gap = 0.0f; // distance between the two edges (0 for a single edge)
    int edgeCount = 0;

    glm::vec3 pointAt(float s) const { return start + Y * s; }
    glm::mat4 frameAt(float s) const;
    bool operator==(const Seam& o) const;
};

// `reverse` flips the travel direction (default: from the lower end upwards).
bool buildSeam(const std::vector<SeamEdgeGeom>& edges, bool reverse, const glm::vec3& viewDir, Seam& out, std::string& err);

} // namespace pegasus
