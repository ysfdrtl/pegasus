#pragma once
// Triangle mesh with the topology needed by the editor:
//  - welded vertices (STL files are triangle soups)
//  - unique edges with adjacent triangles and dihedral angles
//  - "feature" edges (sharp / boundary edges)
//  - face regions: planar regions and smooth surfaces (for face selection)
//  - BVH for ray casting and collision queries

#include "geom/Intersect.h"

#include <glm/glm.hpp>

#include <cstdint>
#include <memory>
#include <string>
#include <vector>

namespace pegasus {

constexpr uint32_t kInvalid = 0xFFFFFFFFu;

class Mesh;

class BVH {
public:
    struct Node {
        AABB box;
        uint32_t leftOrFirst = 0; // interior: index of left child (right = left + 1); leaf: first index in triOrder
        uint32_t count = 0;       // > 0 for leaves
        bool isLeaf() const { return count > 0; }
    };

    void build(const Mesh& mesh);
    bool empty() const { return nodes_.empty(); }
    const std::vector<Node>& nodes() const { return nodes_; }
    const std::vector<uint32_t>& triOrder() const { return triOrder_; }

private:
    void subdivide(const Mesh& mesh, const std::vector<glm::vec3>& centroids, uint32_t nodeIdx);
    std::vector<Node> nodes_;
    std::vector<uint32_t> triOrder_;
};

struct RayHit {
    float t = std::numeric_limits<float>::max();
    uint32_t tri = kInvalid;
    float u = 0.0f, v = 0.0f;
    bool hit() const { return tri != kInvalid; }
};

class Mesh {
public:
    struct Edge {
        uint32_t v[2] = {kInvalid, kInvalid};
        uint32_t tri[2] = {kInvalid, kInvalid}; // first two adjacent triangles
        uint32_t faceCount = 0;                 // 1 = boundary, 2 = manifold, >2 = non-manifold
        float angleDeg = 0.0f;                  // angle between adjacent face normals (0 = flat)
    };

    // Partition of triangles into connected regions (CSR layout).
    struct Regions {
        std::vector<uint32_t> regionOfTri;
        std::vector<uint32_t> offsets; // size = count + 1
        std::vector<uint32_t> tris;
        uint32_t count() const { return offsets.empty() ? 0u : uint32_t(offsets.size() - 1); }
        uint32_t size(uint32_t r) const { return offsets[r + 1] - offsets[r]; }
        const uint32_t* begin(uint32_t r) const { return tris.data() + offsets[r]; }
        const uint32_t* end(uint32_t r) const { return tris.data() + offsets[r + 1]; }
    };

    // Build from a raw triangle soup (3 positions per triangle). Welds vertices,
    // removes degenerate triangles and computes all derived data.
    static std::shared_ptr<Mesh> fromSoup(const std::vector<glm::vec3>& soup, std::string name);

    // Recompute feature edges and smooth regions for a new sharp-edge angle threshold.
    void setFeatureAngle(float degrees);
    float featureAngle() const { return featureAngleDeg_; }

    // Move all vertices by -offset (used to re-center imported parts).
    void translateVertices(const glm::vec3& offset);

    // Queries in mesh-local space.
    bool raycast(const Ray& ray, RayHit& hit, float tMax = std::numeric_limits<float>::max()) const;
    int countHits(const Ray& ray) const; // number of crossings, for point-in-mesh tests
    bool containsPoint(const glm::vec3& p) const;

    glm::vec3 triVertex(uint32_t tri, int k) const { return positions[triangles[tri][k]]; }
    glm::vec3 triCentroid(uint32_t tri) const;

    std::string name;
    uint64_t uid = 0;
    uint32_t gpuVersion = 1; // bumped when GPU buffers (e.g. feature edges) must be rebuilt

    std::vector<glm::vec3> positions;
    std::vector<glm::uvec3> triangles;
    std::vector<glm::vec3> triNormals;
    std::vector<float> triAreas;
    std::vector<Edge> edges;
    std::vector<glm::uvec3> triEdges;      // 3 edge indices per triangle
    std::vector<uint32_t> featureEdges;    // indices into edges
    Regions planarRegions;                 // coplanar connected triangles
    Regions smoothRegions;                 // connected triangles not separated by feature edges
    AABB bounds;
    BVH bvh;
    float totalArea = 0.0f;
    // True when the mesh is closed, manifold and consistently oriented with outward normals
    // (triangles are re-oriented on load when possible).
    bool outwardOriented = false;
    size_t flippedTriangles = 0;

private:
    void buildTopology();
    void buildEdges();
    void orientConsistently();
    void buildRegions(Regions& out, float angleTolDeg, bool compareToSeed) const;
    float featureAngleDeg_ = 30.0f;
};

using MeshPtr = std::shared_ptr<Mesh>;

} // namespace pegasus
