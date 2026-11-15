#pragma once
#include <cstdint>
#include <string>
#include <vector>

namespace pegasus {
// Writes an 8-bit RGBA PNG (uncompressed deflate; small dependency-free writer for screenshots).
bool writePng(const std::string& path, uint32_t width, uint32_t height, const std::vector<uint8_t>& rgba);
}
