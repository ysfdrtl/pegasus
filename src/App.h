#pragma once

namespace pegasus {

// Owns the window, Vulkan context, renderer, ImGui and the editor; runs the main loop.
int runApp(int argc, char** argv);

} // namespace pegasus
