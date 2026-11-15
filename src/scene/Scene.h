#pragma once
// Scene: a flat list of objects (mesh instance + transform). Units are millimetres, Z is up.

#include "scene/Mesh.h"

#include <glm/glm.hpp>

#include <memory>
#include <string>
#include <vector>

namespace pegasus {

struct Transform {
    glm::vec3 position{0.0f};
    glm::vec3 rotationDeg{0.0f}; // Euler angles (ImGuizmo convention)
    glm::vec3 scale{1.0f};

    glm::mat4 matrix() const;
    static Transform fromMatrix(const glm::mat4& m);
};

struct SceneObject {
    uint32_t id = 0;
    std::string name;
    std::string sourcePath;
    MeshPtr mesh;
    Transform transform;
    glm::vec4 color{0.7f, 0.72f, 0.75f, 1.0f};
    bool visible = true;
    bool collidable = true;

    // Robot parts get their world matrix from forward kinematics instead of `transform`.
    int robotPart = -1;
    bool matrixOverride = false;
    glm::mat4 worldOverride{1.0f};

    glm::mat4 matrix() const { return matrixOverride ? worldOverride : transform.matrix(); }
    AABB worldBounds() const { return mesh->bounds.transformed(matrix()); }
};

class Scene {
public:
    SceneObject* add(MeshPtr mesh, const std::string& name);
    SceneObject* duplicate(uint32_t id);
    void remove(uint32_t id);
    void clear();
    // Replaces all objects with copies of `objs`, keeping their ids (undo / redo).
    void restore(const std::vector<SceneObject>& objs);

    SceneObject* find(uint32_t id);
    const SceneObject* find(uint32_t id) const;

    std::vector<std::unique_ptr<SceneObject>>& objects() { return objects_; }
    const std::vector<std::unique_ptr<SceneObject>>& objects() const { return objects_; }

    AABB bounds() const;

    // Must be called whenever geometry / transforms / visibility change (drives collision updates).
    void markDirty() { ++version_; }
    uint64_t version() const { return version_; }

private:
    std::vector<std::unique_ptr<SceneObject>> objects_;
    uint32_t nextId_ = 1;
    uint32_t colorIndex_ = 0;
    uint64_t version_ = 1;
};

} // namespace pegasus
