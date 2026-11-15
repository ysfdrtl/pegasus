#pragma once
#include "scene/Mesh.h"

namespace pegasus {

// Simple test bodies (in millimetres). Their local origin is at the bottom centre.
MeshPtr makeBox(const glm::vec3& size, const std::string& name = "Kutu");
MeshPtr makeCylinder(float radius, float height, int segments = 48, const std::string& name = "Silindir");
MeshPtr makeSphere(float radius, int rings = 24, int segments = 48, const std::string& name = "Küre");

} // namespace pegasus
