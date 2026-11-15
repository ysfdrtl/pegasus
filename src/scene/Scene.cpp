#include "scene/Scene.h"

#include <imgui.h>
#include <ImGuizmo.h>

#include <algorithm>
#include <glm/gtc/type_ptr.hpp>

namespace pegasus {

namespace {
const glm::vec4 kPalette[] = {
    {0.62f, 0.68f, 0.78f, 1.0f}, {0.85f, 0.67f, 0.42f, 1.0f}, {0.55f, 0.75f, 0.58f, 1.0f},
    {0.75f, 0.58f, 0.78f, 1.0f}, {0.52f, 0.74f, 0.80f, 1.0f}, {0.82f, 0.78f, 0.52f, 1.0f},
};
}

glm::mat4 Transform::matrix() const {
    glm::mat4 m(1.0f);
    ImGuizmo::RecomposeMatrixFromComponents(glm::value_ptr(position), glm::value_ptr(rotationDeg),
                                            glm::value_ptr(scale), glm::value_ptr(m));
    return m;
}

Transform Transform::fromMatrix(const glm::mat4& m) {
    Transform t;
    ImGuizmo::DecomposeMatrixToComponents(glm::value_ptr(m), glm::value_ptr(t.position),
                                          glm::value_ptr(t.rotationDeg), glm::value_ptr(t.scale));
    return t;
}

SceneObject* Scene::add(MeshPtr mesh, const std::string& name) {
    auto obj = std::make_unique<SceneObject>();
    obj->id = nextId_++;
    obj->name = name;
    obj->mesh = std::move(mesh);
    obj->color = kPalette[colorIndex_++ % (sizeof(kPalette) / sizeof(kPalette[0]))];
    objects_.push_back(std::move(obj));
    markDirty();
    return objects_.back().get();
}

SceneObject* Scene::duplicate(uint32_t id) {
    const SceneObject* src = find(id);
    if (!src) return nullptr;
    SceneObject copy = *src;
    SceneObject* obj = add(copy.mesh, copy.name + " (kopya)");
    obj->sourcePath = copy.sourcePath;
    obj->transform = copy.transform;
    obj->collidable = copy.collidable;
    return obj;
}

void Scene::remove(uint32_t id) {
    objects_.erase(std::remove_if(objects_.begin(), objects_.end(), [&](const auto& o) { return o->id == id; }),
                   objects_.end());
    markDirty();
}

void Scene::clear() {
    objects_.clear();
    markDirty();
}

SceneObject* Scene::find(uint32_t id) {
    for (auto& o : objects_)
        if (o->id == id) return o.get();
    return nullptr;
}

const SceneObject* Scene::find(uint32_t id) const {
    for (auto& o : objects_)
        if (o->id == id) return o.get();
    return nullptr;
}

AABB Scene::bounds() const {
    AABB b;
    for (auto& o : objects_)
        if (o->visible) b.expand(o->worldBounds());
    return b;
}

} // namespace pegasus
