#pragma once
// Orbit camera, Z-up, millimetre units. Matrices use the OpenGL convention (also what
// ImGuizmo expects); `vulkanProjParams()` converts for our shaders.

#include "geom/Intersect.h"

#include <glm/glm.hpp>

namespace pegasus {

class Camera {
public:
    glm::vec3 target{0.0f, 0.0f, 50.0f};
    float distance = 600.0f;
    float yawDeg = -50.0f;
    float pitchDeg = 28.0f;
    float fovYDeg = 40.0f;
    float aspect = 1.0f;

    glm::vec3 eye() const;
    glm::vec3 forward() const;
    glm::mat4 view() const;
    glm::mat4 projection() const; // OpenGL clip conventions
    float nearPlane() const;
    float farPlane() const;
    glm::vec4 vulkanProjParams() const;

    void orbit(float dxPixels, float dyPixels);
    void pan(float dxPixels, float dyPixels, float viewportHeight);
    void zoom(float wheelSteps);
    void frame(const AABB& box);
    void setView(float yaw, float pitch) { yawDeg = yaw; pitchDeg = pitch; }

    // Ray through a point given in viewport-relative pixels.
    Ray rayFromViewport(const glm::vec2& px, const glm::vec2& viewportSize) const;
    // World -> viewport pixels. Returns false if behind the camera.
    bool worldToViewport(const glm::vec3& p, const glm::vec2& viewportSize, glm::vec2& out) const;

    float sceneRadius = 1000.0f; // used for clip planes
};

} // namespace pegasus
