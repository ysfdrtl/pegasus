#include "scene/StlLoader.h"

#include <cmath>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <sstream>

namespace pegasus {

namespace {

bool parseAscii(const std::string& text, std::vector<glm::vec3>& soup) {
    std::istringstream in(text);
    std::string tok;
    while (in >> tok) {
        if (tok == "vertex") {
            glm::vec3 p;
            if (!(in >> p.x >> p.y >> p.z)) return false;
            soup.push_back(p);
        }
    }
    return !soup.empty() && soup.size() % 3 == 0;
}

} // namespace

MeshPtr loadStl(const std::string& path, std::string& error) {
    std::ifstream f(path, std::ios::binary);
    if (!f) {
        error = "Dosya açılamadı: " + path;
        return nullptr;
    }
    std::string data((std::istreambuf_iterator<char>(f)), std::istreambuf_iterator<char>());
    const std::string name = std::filesystem::path(path).stem().string();

    std::vector<glm::vec3> soup;
    bool binary = false;
    if (data.size() >= 84) {
        uint32_t count = 0;
        std::memcpy(&count, data.data() + 80, 4);
        binary = data.size() == 84ull + 50ull * count;
        if (binary) {
            soup.reserve(size_t(count) * 3);
            const char* p = data.data() + 84;
            for (uint32_t i = 0; i < count; ++i, p += 50) {
                float v[12];
                std::memcpy(v, p, sizeof(v)); // normal (ignored) + 3 vertices
                for (int k = 0; k < 3; ++k) soup.emplace_back(v[3 + k * 3], v[4 + k * 3], v[5 + k * 3]);
            }
        }
    }
    if (!binary) {
        if (data.compare(0, 5, "solid") != 0 || !parseAscii(data, soup)) {
            error = "Geçersiz veya desteklenmeyen STL dosyası: " + path;
            return nullptr;
        }
    }
    for (const auto& p : soup) {
        if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z)) {
            error = "STL dosyası geçersiz (NaN/Inf) koordinat içeriyor.";
            return nullptr;
        }
    }
    auto mesh = Mesh::fromSoup(soup, name);
    if (mesh->triangles.empty()) {
        error = "STL dosyasında geçerli üçgen yok.";
        return nullptr;
    }
    return mesh;
}

} // namespace pegasus
