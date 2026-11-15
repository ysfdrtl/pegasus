#include "editor/Camera.h"

#include <algorithm>
#include <cmath>
#include <glm/gtc/matrix_transform.hpp>

namespace pegasus {

glm::vec3 Camera::forward() const {
    const float y = glm::radians(yawDeg), p = glm::radians(pitchDeg);
    // direction from eye towards target
    return -glm::vec3(std::cos(p) * std::cos(y), std::cos(p) * std::sin(y), std::sin(p));
}

glm::vec3 Camera::eye() const { return target - forward() * distance; }

glm::mat4 Camera::view() const { return glm::lookAt(eye(), target, glm::vec3(0, 0, 1)); }

float Camera::nearPlane() const { return std::max(distance * 0.002f, 0.01f); }
float Camera::farPlane() const { return distance * 20.0f + sceneRadius * 4.0f + 1000.0f; }

glm::mat4 Camera::projection() const {
    return glm::perspective(glm::radians(fovYDeg), aspect, nearPlane(), farPlane());
}

glm::vec4 Camera::vulkanProjParams() const {
    const glm::mat4 p = projection();
    // y flipped, z remapped from [-1,1] to [0,1]: z' = 0.5 z + 0.5 w, with w = -z_view
    return glm::vec4(p[0][0], -p[1][1], 0.5f * p[2][2] - 0.5f, 0.5f * p[3][2]);
}

void Camera::orbit(float dx, float dy) {
    yawDeg -= dx * 0.3f;
    pitchDeg = std::clamp(pitchDeg + dy * 0.3f, -89.0f, 89.0f);
}

void Camera::pan(float dx, float dy, float viewportHeight) {
    const float worldPerPixel = 2.0f * distance * std::tan(glm::radians(fovYDeg) * 0.5f) / std::max(viewportHeight, 1.0f);
    const glm::vec3 f = forward();
    const glm::vec3 right = glm::normalize(glm::cross(f, glm::vec3(0, 0, 1)));
    const glm::vec3 up = glm::cross(right, f);
    target += (-dx * right + dy * up) * worldPerPixel;
}

void Camera::zoom(float steps) {
    distance = std::clamp(distance * std::pow(0.88f, steps), 0.5f, 1.0e6f);
}

void Camera::frame(const AABB& box) {
    if (!box.valid()) return;
    target = box.center();
    const float radius = std::max(box.diagonal() * 0.5f, 1.0f);
    distance = radius / std::sin(glm::radians(fovYDeg) * 0.5f) * 1.1f;
    sceneRadius = std::max(sceneRadius, radius);
}

Ray Camera::rayFromViewport(const glm::vec2& px, const glm::vec2& size) const {
    const glm::vec2 ndc(2.0f * px.x / size.x - 1.0f, 1.0f - 2.0f * px.y / size.y);
    const glm::mat4 inv = glm::inverse(projection() * view());
    glm::vec4 n = inv * glm::vec4(ndc, -1.0f, 1.0f);
    glm::vec4 f = inv * glm::vec4(ndc, 1.0f, 1.0f);
    n /= n.w;
    f /= f.w;
    Ray r;
    r.origin = eye();
    r.dir = glm::normalize(glm::vec3(f) - glm::vec3(n));
    return r;
}

bool Camera::worldToViewport(const glm::vec3& p, const glm::vec2& size, glm::vec2& out) const {
    const glm::vec4 c = projection() * view() * glm::vec4(p, 1.0f);
    if (c.w <= 1e-6f) return false;
    out = glm::vec2((c.x / c.w * 0.5f + 0.5f) * size.x, (1.0f - (c.y / c.w * 0.5f + 0.5f)) * size.y);
    return true;
}

} // namespace pegasus
