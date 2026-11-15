#pragma once
// Scene renderer. The editor fills a FrameScene each frame (pure data); the renderer
// uploads what is needed and records the Vulkan commands (3D scene + ImGui).

#include "render/VulkanContext.h"
#include "scene/Mesh.h"

#include <glm/glm.hpp>

#include <unordered_map>
#include <vector>

struct ImDrawData;

namespace pegasus {

struct MeshDraw {
    MeshPtr mesh;
    glm::mat4 model{1.0f};
    glm::vec4 color{1.0f};
};

struct EdgeDraw {
    MeshPtr mesh;
    glm::mat4 model{1.0f};
    glm::vec4 color{1.0f};
    bool allEdges = false; // false: feature edges only
    float width = 1.0f;
};

struct TriHighlight {
    MeshPtr mesh;
    glm::mat4 model{1.0f};
    glm::vec4 color{1.0f};
    std::vector<uint32_t> indices; // vertex indices, 3 per triangle
};

struct LineBatch {
    std::vector<glm::vec3> points; // world space, 2 per segment
    glm::vec4 color{1.0f};
    float width = 1.0f;
    bool depthTest = true;
};

struct FrameScene {
    glm::mat4 view{1.0f};
    glm::vec4 proj{1.0f}; // see shaders: (P00, P11, P22, P32) Vulkan-style
    float viewport[4] = {0, 0, 0, 0}; // x, y, w, h in framebuffer pixels
    float clearColor[4] = {0.16f, 0.17f, 0.19f, 1.0f};

    std::vector<MeshDraw> meshes;
    std::vector<EdgeDraw> edges;
    std::vector<LineBatch> lines;          // drawn after meshes (grid etc.)
    std::vector<TriHighlight> highlights;  // translucent overlays on top of meshes
    std::vector<LineBatch> overlayLines;   // drawn last
};

class Renderer {
public:
    bool init(VulkanContext* ctx);
    void shutdown();
    // Returns false if the frame was skipped.
    bool render(const FrameScene& scene, ImDrawData* imgui);

private:
    struct GpuMesh {
        std::weak_ptr<Mesh> mesh;
        uint32_t version = 0;
        Buffer vertices, triIndices, allEdgeIndices, featureEdgeIndices;
        uint32_t triIndexCount = 0, allEdgeIndexCount = 0, featureEdgeIndexCount = 0;
    };
    struct Retired {
        uint64_t frame;
        std::vector<Buffer> buffers;
    };

    GpuMesh* gpuMesh(const MeshPtr& mesh);
    void retire(GpuMesh& gm);
    void collectGarbage(bool all);
    VkPipeline createPipeline(VkShaderModule vs, VkShaderModule fs, VkPrimitiveTopology topo, bool depthTest,
                              bool depthWrite, VkCompareOp cmp, bool blend, bool depthBias);

    VulkanContext* ctx_ = nullptr;
    VkPipelineLayout layout_ = VK_NULL_HANDLE;
    VkPipeline meshPipe_ = VK_NULL_HANDLE, highlightPipe_ = VK_NULL_HANDLE;
    VkPipeline linePipe_ = VK_NULL_HANDLE, lineOverlayPipe_ = VK_NULL_HANDLE;

    std::unordered_map<uint64_t, GpuMesh> meshes_;
    std::vector<Retired> retired_;
    Buffer dynamic_[VulkanContext::kFramesInFlight];
    uint64_t frameCounter_ = 0;
};

} // namespace pegasus
