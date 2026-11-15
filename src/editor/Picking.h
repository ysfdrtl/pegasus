#pragma once
// Mouse picking of objects and sub-elements (vertex / edge / face) in the scene.

#include "editor/Camera.h"
#include "scene/Scene.h"

namespace pegasus {

enum class FaceMode { Triangle = 0, Planar = 1, Smooth = 2 };
enum class EdgeSet { Feature = 0, All = 1 };

struct ElementRef {
    uint32_t objectId = 0;
    uint32_t index = kInvalid;
    bool valid() const { return objectId != 0 && index != kInvalid; }
    bool operator==(const ElementRef& o) const { return objectId == o.objectId && index == o.index; }
    bool operator!=(const ElementRef& o) const { return !(*this == o); }
};

struct SceneHit {
    uint32_t objectId = 0;
    uint32_t tri = kInvalid;
    float t = 0.0f;
    glm::vec3 point{0.0f};
    bool hit() const { return objectId != 0; }
};

struct PickContext {
    const Scene* scene = nullptr;
    const Camera* camera = nullptr;
    glm::vec2 viewportSize{1.0f};
    glm::vec2 mouse{0.0f}; // viewport relative pixels
    float radiusPx = 12.0f;
};

SceneHit raycastScene(const Scene& scene, const Ray& worldRay, float tMax = std::numeric_limits<float>::max());
bool isPointVisible(const Scene& scene, const glm::vec3& eye, const glm::vec3& p);

SceneHit pickSurface(const PickContext& ctx);
ElementRef pickVertex(const PickContext& ctx);
ElementRef pickEdge(const PickContext& ctx, EdgeSet set);
ElementRef pickFace(const PickContext& ctx, FaceMode mode);

uint32_t faceIndexForTri(const Mesh& mesh, FaceMode mode, uint32_t tri);
void appendFaceTriangles(const Mesh& mesh, FaceMode mode, uint32_t face, std::vector<uint32_t>& outTris);
float faceArea(const Mesh& mesh, FaceMode mode, uint32_t face);

} // namespace pegasus
