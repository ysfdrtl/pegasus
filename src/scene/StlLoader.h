#pragma once
#include "scene/Mesh.h"

#include <string>

namespace pegasus {

// Loads binary or ASCII STL files. Returns nullptr and fills `error` on failure.
MeshPtr loadStl(const std::string& path, std::string& error);

} // namespace pegasus
