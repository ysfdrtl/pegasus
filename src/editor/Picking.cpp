#include "editor/Picking.h"

#include <algorithm>

namespace pegasus {

namespace {

struct Candidate {
    float distPx;
    uint32_t objectId;
    uint32_t index;
    glm::vec3 world;
};

constexpr int kMaxVisibilityTests = 10;

// Projects all vertices of a mesh into clip space.
void projectVertices(const Mesh& mesh, const glm::mat4& mvp, std::vector<glm::vec4>& out) {
    out.resize(mesh.positions.size());
    for (size_t i = 0; i < mesh.positions.size(); ++i) out[i] = mvp * glm::vec4(mesh.positions[i], 1.0f);
}

glm::vec2 clipToViewport(const glm::vec4& c, const glm::vec2& size) {
    return glm::vec2((c.x / c.w * 0.5f + 0.5f) * size.x, (1.0f - (c.y / c.w * 0.5f + 0.5f)) * size.y);
}

ElementRef resolveVisible(const PickContext& ctx, std::vector<Candidate>& cands) {
    std::sort(cands.begin(), cands.end(), [](const Candidate& a, const Candidate& b) { return a.distPx < b.distPx; });
    const glm::vec3 eye = ctx.camera->eye();
    const int n = std::min<int>(int(cands.size()), kMaxVisibilityTests);
    for (int i = 0; i < n; ++i)
        if (isPointVisible(*ctx.scene, eye, cands[i].world)) return {cands[i].objectId, cands[i].index};
    return {};
}

} // namespace

SceneHit raycastScene(const Scene& scene, const Ray& ray, float tMax) {
    SceneHit best;
    best.t = tMax;
    for (const auto& o : scene.objects()) {
        if (!o->visible) continue;
        const glm::mat4 m = o->matrix();
        // quick reject on the world AABB
        const AABB wb = o->mesh->bounds.transformed(m);
        float tn;
        if (!rayAabb(ray.origin, 1.0f / ray.dir, wb, best.t, tn)) continue;
        const glm::mat4 inv = glm::inverse(m);
        Ray local;
        local.origin = glm::vec3(inv * glm::vec4(ray.origin, 1.0f));
        local.dir = glm::mat3(inv) * ray.dir; // unnormalized: keeps the world t parametrisation
        RayHit h;
        if (o->mesh->raycast(local, h, best.t)) {
            best.objectId = o->id;
            best.tri = h.tri;
            best.t = h.t;
        }
    }
    if (best.hit()) best.point = ray.origin + ray.dir * best.t;
    return best;
}

bool isPointVisible(const Scene& scene, const glm::vec3& eye, const glm::vec3& p) {
    Ray r{eye, p - eye}; // t = 1 at p
    const SceneHit h = raycastScene(scene, r, 1.0f);
    return !h.hit() || h.t >= 1.0f - 2e-3f;
}

SceneHit pickSurface(const PickContext& ctx) {
    return raycastScene(*ctx.scene, ctx.camera->rayFromViewport(ctx.mouse, ctx.viewportSize));
}

ElementRef pickVertex(const PickContext& ctx) {
    const glm::mat4 vp = ctx.camera->projection() * ctx.camera->view();
    const float nearW = ctx.camera->nearPlane();
    std::vector<Candidate> cands;
    static thread_local std::vector<glm::vec4> clip;
    for (const auto& o : ctx.scene->objects()) {
        if (!o->visible) continue;
        const glm::mat4 m = o->matrix();
        projectVertices(*o->mesh, vp * m, clip);
        for (uint32_t i = 0; i < clip.size(); ++i) {
            if (clip[i].w <= nearW) continue;
            const float d = glm::length(clipToViewport(clip[i], ctx.viewportSize) - ctx.mouse);
            if (d <= ctx.radiusPx) cands.push_back({d, o->id, i, glm::vec3(m * glm::vec4(o->mesh->positions[i], 1.0f))});
        }
    }
    return resolveVisible(ctx, cands);
}

ElementRef pickEdge(const PickContext& ctx, EdgeSet set) {
    const glm::mat4 vp = ctx.camera->projection() * ctx.camera->view();
    const float nearW = ctx.camera->nearPlane();
    const Ray ray = ctx.camera->rayFromViewport(ctx.mouse, ctx.viewportSize);
    std::vector<Candidate> cands;
    static thread_local std::vector<glm::vec4> clip;

    for (const auto& o : ctx.scene->objects()) {
        if (!o->visible) continue;
        const Mesh& mesh = *o->mesh;
        const glm::mat4 m = o->matrix();
        projectVertices(mesh, vp * m, clip);

        auto test = [&](uint32_t ei) {
            const Mesh::Edge& e = mesh.edges[ei];
            glm::vec4 a = clip[e.v[0]], b = clip[e.v[1]];
            if (a.w <= nearW && b.w <= nearW) return;
            // clip the segment against the near plane (w = near)
            if (a.w <= nearW) a = glm::mix(a, b, (nearW - a.w) / (b.w - a.w));
            else if (b.w <= nearW) b = glm::mix(b, a, (nearW - b.w) / (a.w - b.w));
            float s;
            const float d = pointSegmentDistance2D(ctx.mouse, clipToViewport(a, ctx.viewportSize),
                                                   clipToViewport(b, ctx.viewportSize), s);
            if (d > ctx.radiusPx) return;
            const glm::vec3 wa = glm::vec3(m * glm::vec4(mesh.positions[e.v[0]], 1.0f));
            const glm::vec3 wb = glm::vec3(m * glm::vec4(mesh.positions[e.v[1]], 1.0f));
            const float ws = closestSegmentParamToLine(ray.origin, ray.dir, wa, wb);
            cands.push_back({d, o->id, ei, glm::mix(wa, wb, ws)});
        };
        if (set == EdgeSet::All)
            for (uint32_t ei = 0; ei < mesh.edges.size(); ++ei) test(ei);
        else
            for (uint32_t ei : mesh.featureEdges) test(ei);
    }
    return resolveVisible(ctx, cands);
}

uint32_t faceIndexForTri(const Mesh& mesh, FaceMode mode, uint32_t tri) {
    switch (mode) {
    case FaceMode::Planar: return mesh.planarRegions.regionOfTri[tri];
    case FaceMode::Smooth: return mesh.smoothRegions.regionOfTri[tri];
    default: return tri;
    }
}

void appendFaceTriangles(const Mesh& mesh, FaceMode mode, uint32_t face, std::vector<uint32_t>& out) {
    if (mode == FaceMode::Triangle) {
        if (face < mesh.triangles.size()) out.push_back(face);
        return;
    }
    const Mesh::Regions& r = mode == FaceMode::Planar ? mesh.planarRegions : mesh.smoothRegions;
    if (face >= r.count()) return;
    out.insert(out.end(), r.begin(face), r.end(face));
}

float faceArea(const Mesh& mesh, FaceMode mode, uint32_t face) {
    std::vector<uint32_t> tris;
    appendFaceTriangles(mesh, mode, face, tris);
    float a = 0.0f;
    for (uint32_t t : tris) a += mesh.triAreas[t];
    return a;
}

ElementRef pickFace(const PickContext& ctx, FaceMode mode) {
    const SceneHit h = pickSurface(ctx);
    if (!h.hit()) return {};
    const SceneObject* o = ctx.scene->find(h.objectId);
    return {h.objectId, faceIndexForTri(*o->mesh, mode, h.tri)};
}

} // namespace pegasus
