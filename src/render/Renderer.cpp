#include "render/Renderer.h"

#include <imgui.h>
#include <imgui_impl_vulkan.h>

#include <algorithm>
#include <cstdio>
#include <cstring>
#include <stdexcept>

namespace pegasus {

namespace {

struct PushConstants {
    glm::mat4 modelView;
    glm::vec4 color;
    glm::vec4 proj;
};
static_assert(sizeof(PushConstants) == 96);

constexpr VkBufferUsageFlags kDynUsage = VK_BUFFER_USAGE_VERTEX_BUFFER_BIT | VK_BUFFER_USAGE_INDEX_BUFFER_BIT;
constexpr VkMemoryPropertyFlags kHostMem = VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT | VK_MEMORY_PROPERTY_HOST_COHERENT_BIT;

} // namespace

bool Renderer::init(VulkanContext* ctx) {
    ctx_ = ctx;
    try {
        const std::string dir = PEGASUS_SHADER_DIR;
        VkShaderModule meshVs = ctx->loadShader(dir + "/mesh.vert.spv");
        VkShaderModule meshFs = ctx->loadShader(dir + "/mesh.frag.spv");
        VkShaderModule lineVs = ctx->loadShader(dir + "/line.vert.spv");
        VkShaderModule lineFs = ctx->loadShader(dir + "/line.frag.spv");

        VkPushConstantRange range{VK_SHADER_STAGE_VERTEX_BIT | VK_SHADER_STAGE_FRAGMENT_BIT, 0, sizeof(PushConstants)};
        VkPipelineLayoutCreateInfo li{VK_STRUCTURE_TYPE_PIPELINE_LAYOUT_CREATE_INFO};
        li.pushConstantRangeCount = 1;
        li.pPushConstantRanges = &range;
        if (vkCreatePipelineLayout(ctx->device, &li, nullptr, &layout_) != VK_SUCCESS) return false;

        meshPipe_ = createPipeline(meshVs, meshFs, VK_PRIMITIVE_TOPOLOGY_TRIANGLE_LIST, true, true,
                                   VK_COMPARE_OP_LESS, false, true);
        highlightPipe_ = createPipeline(meshVs, meshFs, VK_PRIMITIVE_TOPOLOGY_TRIANGLE_LIST, true, false,
                                        VK_COMPARE_OP_LESS_OR_EQUAL, true, false);
        linePipe_ = createPipeline(lineVs, lineFs, VK_PRIMITIVE_TOPOLOGY_LINE_LIST, true, false,
                                   VK_COMPARE_OP_LESS_OR_EQUAL, true, false);
        lineOverlayPipe_ = createPipeline(lineVs, lineFs, VK_PRIMITIVE_TOPOLOGY_LINE_LIST, false, false,
                                          VK_COMPARE_OP_ALWAYS, true, false);

        for (auto m : {meshVs, meshFs, lineVs, lineFs}) vkDestroyShaderModule(ctx->device, m, nullptr);
        for (auto& d : dynamic_) d = ctx->createBuffer(1 << 20, kDynUsage, kHostMem);
    } catch (const std::exception& e) {
        std::fprintf(stderr, "Renderer init hatasi: %s\n", e.what());
        return false;
    }
    return true;
}

VkPipeline Renderer::createPipeline(VkShaderModule vs, VkShaderModule fs, VkPrimitiveTopology topo, bool depthTest,
                                    bool depthWrite, VkCompareOp cmp, bool blend, bool depthBias) {
    VkPipelineShaderStageCreateInfo stages[2]{};
    stages[0] = {VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO, nullptr, 0, VK_SHADER_STAGE_VERTEX_BIT, vs, "main", nullptr};
    stages[1] = {VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO, nullptr, 0, VK_SHADER_STAGE_FRAGMENT_BIT, fs, "main", nullptr};

    VkVertexInputBindingDescription binding{0, sizeof(glm::vec3), VK_VERTEX_INPUT_RATE_VERTEX};
    VkVertexInputAttributeDescription attr{0, 0, VK_FORMAT_R32G32B32_SFLOAT, 0};
    VkPipelineVertexInputStateCreateInfo vi{VK_STRUCTURE_TYPE_PIPELINE_VERTEX_INPUT_STATE_CREATE_INFO};
    vi.vertexBindingDescriptionCount = 1;
    vi.pVertexBindingDescriptions = &binding;
    vi.vertexAttributeDescriptionCount = 1;
    vi.pVertexAttributeDescriptions = &attr;

    VkPipelineInputAssemblyStateCreateInfo ia{VK_STRUCTURE_TYPE_PIPELINE_INPUT_ASSEMBLY_STATE_CREATE_INFO};
    ia.topology = topo;

    VkPipelineViewportStateCreateInfo vp{VK_STRUCTURE_TYPE_PIPELINE_VIEWPORT_STATE_CREATE_INFO};
    vp.viewportCount = 1;
    vp.scissorCount = 1;

    VkPipelineRasterizationStateCreateInfo rs{VK_STRUCTURE_TYPE_PIPELINE_RASTERIZATION_STATE_CREATE_INFO};
    rs.polygonMode = VK_POLYGON_MODE_FILL;
    rs.cullMode = VK_CULL_MODE_NONE;
    rs.frontFace = VK_FRONT_FACE_COUNTER_CLOCKWISE;
    rs.lineWidth = 1.0f;
    rs.depthBiasEnable = depthBias;
    rs.depthBiasConstantFactor = 1.0f;
    rs.depthBiasSlopeFactor = 1.0f;

    VkPipelineMultisampleStateCreateInfo ms{VK_STRUCTURE_TYPE_PIPELINE_MULTISAMPLE_STATE_CREATE_INFO};
    ms.rasterizationSamples = ctx_->msaaSamples;

    VkPipelineDepthStencilStateCreateInfo ds{VK_STRUCTURE_TYPE_PIPELINE_DEPTH_STENCIL_STATE_CREATE_INFO};
    ds.depthTestEnable = depthTest;
    ds.depthWriteEnable = depthWrite;
    ds.depthCompareOp = cmp;

    VkPipelineColorBlendAttachmentState cba{};
    cba.colorWriteMask = VK_COLOR_COMPONENT_R_BIT | VK_COLOR_COMPONENT_G_BIT | VK_COLOR_COMPONENT_B_BIT | VK_COLOR_COMPONENT_A_BIT;
    cba.blendEnable = blend;
    cba.srcColorBlendFactor = VK_BLEND_FACTOR_SRC_ALPHA;
    cba.dstColorBlendFactor = VK_BLEND_FACTOR_ONE_MINUS_SRC_ALPHA;
    cba.colorBlendOp = VK_BLEND_OP_ADD;
    cba.srcAlphaBlendFactor = VK_BLEND_FACTOR_ONE;
    cba.dstAlphaBlendFactor = VK_BLEND_FACTOR_ONE_MINUS_SRC_ALPHA;
    cba.alphaBlendOp = VK_BLEND_OP_ADD;
    VkPipelineColorBlendStateCreateInfo cb{VK_STRUCTURE_TYPE_PIPELINE_COLOR_BLEND_STATE_CREATE_INFO};
    cb.attachmentCount = 1;
    cb.pAttachments = &cba;

    const VkDynamicState dyn[] = {VK_DYNAMIC_STATE_VIEWPORT, VK_DYNAMIC_STATE_SCISSOR, VK_DYNAMIC_STATE_LINE_WIDTH};
    VkPipelineDynamicStateCreateInfo dy{VK_STRUCTURE_TYPE_PIPELINE_DYNAMIC_STATE_CREATE_INFO};
    dy.dynamicStateCount = 3;
    dy.pDynamicStates = dyn;

    VkGraphicsPipelineCreateInfo pi{VK_STRUCTURE_TYPE_GRAPHICS_PIPELINE_CREATE_INFO};
    pi.stageCount = 2;
    pi.pStages = stages;
    pi.pVertexInputState = &vi;
    pi.pInputAssemblyState = &ia;
    pi.pViewportState = &vp;
    pi.pRasterizationState = &rs;
    pi.pMultisampleState = &ms;
    pi.pDepthStencilState = &ds;
    pi.pColorBlendState = &cb;
    pi.pDynamicState = &dy;
    pi.layout = layout_;
    pi.renderPass = ctx_->renderPass;
    VkPipeline p;
    if (vkCreateGraphicsPipelines(ctx_->device, VK_NULL_HANDLE, 1, &pi, nullptr, &p) != VK_SUCCESS)
        throw std::runtime_error("vkCreateGraphicsPipelines failed");
    return p;
}

void Renderer::retire(GpuMesh& gm) {
    Retired r;
    r.frame = frameCounter_ + VulkanContext::kFramesInFlight + 1;
    for (Buffer* b : {&gm.vertices, &gm.triIndices, &gm.allEdgeIndices, &gm.featureEdgeIndices})
        if (b->buffer) r.buffers.push_back(*b);
    gm.vertices = gm.triIndices = gm.allEdgeIndices = gm.featureEdgeIndices = Buffer{};
    retired_.push_back(std::move(r));
}

void Renderer::collectGarbage(bool all) {
    for (auto it = meshes_.begin(); it != meshes_.end();) {
        if (it->second.mesh.expired()) {
            retire(it->second);
            it = meshes_.erase(it);
        } else ++it;
    }
    for (auto it = retired_.begin(); it != retired_.end();) {
        if (all || it->frame <= frameCounter_) {
            for (auto& b : it->buffers) ctx_->destroyBuffer(b);
            it = retired_.erase(it);
        } else ++it;
    }
}

Renderer::GpuMesh* Renderer::gpuMesh(const MeshPtr& mesh) {
    if (!mesh || mesh->triangles.empty()) return nullptr;
    GpuMesh& gm = meshes_[mesh->uid];
    if (gm.version == mesh->gpuVersion && gm.vertices.buffer) return &gm;
    if (gm.vertices.buffer) retire(gm);
    gm.mesh = mesh;
    gm.version = mesh->gpuVersion;

    gm.vertices = ctx_->createDeviceLocalBuffer(mesh->positions.data(), mesh->positions.size() * sizeof(glm::vec3),
                                                VK_BUFFER_USAGE_VERTEX_BUFFER_BIT);
    gm.triIndexCount = uint32_t(mesh->triangles.size() * 3);
    gm.triIndices = ctx_->createDeviceLocalBuffer(mesh->triangles.data(), gm.triIndexCount * sizeof(uint32_t),
                                                  VK_BUFFER_USAGE_INDEX_BUFFER_BIT);
    std::vector<uint32_t> idx;
    idx.reserve(mesh->edges.size() * 2);
    for (const auto& e : mesh->edges) idx.insert(idx.end(), {e.v[0], e.v[1]});
    gm.allEdgeIndexCount = uint32_t(idx.size());
    if (!idx.empty())
        gm.allEdgeIndices = ctx_->createDeviceLocalBuffer(idx.data(), idx.size() * 4, VK_BUFFER_USAGE_INDEX_BUFFER_BIT);
    idx.clear();
    for (uint32_t ei : mesh->featureEdges) idx.insert(idx.end(), {mesh->edges[ei].v[0], mesh->edges[ei].v[1]});
    gm.featureEdgeIndexCount = uint32_t(idx.size());
    if (!idx.empty())
        gm.featureEdgeIndices = ctx_->createDeviceLocalBuffer(idx.data(), idx.size() * 4, VK_BUFFER_USAGE_INDEX_BUFFER_BIT);
    return &gm;
}

bool Renderer::render(const FrameScene& fs, ImDrawData* imgui) {
    VkCommandBuffer cmd = ctx_->beginFrame();
    if (!cmd) return false;
    frameCounter_++;
    collectGarbage(false);

    // ---- Upload dynamic data (highlight indices + line vertices) ------------------------------
    size_t bytes = 0;
    for (const auto& h : fs.highlights) bytes += h.indices.size() * 4;
    for (const auto& l : fs.lines) bytes += l.points.size() * sizeof(glm::vec3);
    for (const auto& l : fs.overlayLines) bytes += l.points.size() * sizeof(glm::vec3);
    Buffer& dyn = dynamic_[ctx_->frameIndex()];
    if (bytes > dyn.size) {
        ctx_->destroyBuffer(dyn); // safe: this frame slot's fence has been waited on
        dyn = ctx_->createBuffer(VkDeviceSize(bytes * 3 / 2), kDynUsage, kHostMem);
    }
    std::vector<VkDeviceSize> hlOffsets, lineOffsets, overlayOffsets;
    size_t off = 0;
    auto put = [&](const void* src, size_t n) {
        std::memcpy(static_cast<char*>(dyn.mapped) + off, src, n);
        const size_t at = off;
        off += n;
        return VkDeviceSize(at);
    };
    for (const auto& h : fs.highlights) hlOffsets.push_back(put(h.indices.data(), h.indices.size() * 4));
    for (const auto& l : fs.lines) lineOffsets.push_back(put(l.points.data(), l.points.size() * sizeof(glm::vec3)));
    for (const auto& l : fs.overlayLines) overlayOffsets.push_back(put(l.points.data(), l.points.size() * sizeof(glm::vec3)));

    // Make sure all meshes are resident before recording.
    for (const auto& d : fs.meshes) gpuMesh(d.mesh);
    for (const auto& d : fs.edges) gpuMesh(d.mesh);
    for (const auto& d : fs.highlights) gpuMesh(d.mesh);

    ctx_->beginRenderPass(cmd, fs.clearColor);

    const float fbW = float(ctx_->extent.width), fbH = float(ctx_->extent.height);
    const float x0 = std::clamp(fs.viewport[0], 0.0f, fbW), y0 = std::clamp(fs.viewport[1], 0.0f, fbH);
    const float x1 = std::clamp(fs.viewport[0] + fs.viewport[2], 0.0f, fbW);
    const float y1 = std::clamp(fs.viewport[1] + fs.viewport[3], 0.0f, fbH);

    if (x1 - x0 >= 1.0f && y1 - y0 >= 1.0f) {
        // Use the unclamped rect for the viewport (keeps projection consistent) but clamp the scissor.
        VkViewport vp{fs.viewport[0], fs.viewport[1], fs.viewport[2], fs.viewport[3], 0.0f, 1.0f};
        VkRect2D sc{{int32_t(x0), int32_t(y0)}, {uint32_t(x1 - x0), uint32_t(y1 - y0)}};
        vkCmdSetViewport(cmd, 0, 1, &vp);
        vkCmdSetScissor(cmd, 0, 1, &sc);
        vkCmdSetLineWidth(cmd, 1.0f);

        auto push = [&](const glm::mat4& model, const glm::vec4& color) {
            PushConstants pc{fs.view * model, color, fs.proj};
            vkCmdPushConstants(cmd, layout_, VK_SHADER_STAGE_VERTEX_BIT | VK_SHADER_STAGE_FRAGMENT_BIT, 0, sizeof(pc), &pc);
        };
        auto lineWidth = [&](float w) { vkCmdSetLineWidth(cmd, ctx_->wideLines ? std::max(1.0f, w) : 1.0f); };
        const VkDeviceSize zero = 0;

        // Opaque meshes
        vkCmdBindPipeline(cmd, VK_PIPELINE_BIND_POINT_GRAPHICS, meshPipe_);
        for (const auto& d : fs.meshes) {
            GpuMesh* gm = gpuMesh(d.mesh);
            if (!gm) continue;
            push(d.model, d.color);
            vkCmdBindVertexBuffers(cmd, 0, 1, &gm->vertices.buffer, &zero);
            vkCmdBindIndexBuffer(cmd, gm->triIndices.buffer, 0, VK_INDEX_TYPE_UINT32);
            vkCmdDrawIndexed(cmd, gm->triIndexCount, 1, 0, 0, 0);
        }

        // Mesh edges
        vkCmdBindPipeline(cmd, VK_PIPELINE_BIND_POINT_GRAPHICS, linePipe_);
        for (const auto& d : fs.edges) {
            GpuMesh* gm = gpuMesh(d.mesh);
            if (!gm) continue;
            const Buffer& ib = d.allEdges ? gm->allEdgeIndices : gm->featureEdgeIndices;
            const uint32_t n = d.allEdges ? gm->allEdgeIndexCount : gm->featureEdgeIndexCount;
            if (!ib.buffer || n == 0) continue;
            lineWidth(d.width);
            push(d.model, d.color);
            vkCmdBindVertexBuffers(cmd, 0, 1, &gm->vertices.buffer, &zero);
            vkCmdBindIndexBuffer(cmd, ib.buffer, 0, VK_INDEX_TYPE_UINT32);
            vkCmdDrawIndexed(cmd, n, 1, 0, 0, 0);
        }

        // World lines (grid, axes, ...)
        for (size_t i = 0; i < fs.lines.size(); ++i) {
            const auto& l = fs.lines[i];
            if (l.points.empty()) continue;
            vkCmdBindPipeline(cmd, VK_PIPELINE_BIND_POINT_GRAPHICS, l.depthTest ? linePipe_ : lineOverlayPipe_);
            lineWidth(l.width);
            push(glm::mat4(1.0f), l.color);
            vkCmdBindVertexBuffers(cmd, 0, 1, &dyn.buffer, &lineOffsets[i]);
            vkCmdDraw(cmd, uint32_t(l.points.size()), 1, 0, 0);
        }

        // Translucent triangle highlights
        vkCmdBindPipeline(cmd, VK_PIPELINE_BIND_POINT_GRAPHICS, highlightPipe_);
        for (size_t i = 0; i < fs.highlights.size(); ++i) {
            const auto& h = fs.highlights[i];
            GpuMesh* gm = gpuMesh(h.mesh);
            if (!gm || h.indices.empty()) continue;
            push(h.model, h.color);
            vkCmdBindVertexBuffers(cmd, 0, 1, &gm->vertices.buffer, &zero);
            vkCmdBindIndexBuffer(cmd, dyn.buffer, hlOffsets[i], VK_INDEX_TYPE_UINT32);
            vkCmdDrawIndexed(cmd, uint32_t(h.indices.size()), 1, 0, 0, 0);
        }

        // Overlay lines
        for (size_t i = 0; i < fs.overlayLines.size(); ++i) {
            const auto& l = fs.overlayLines[i];
            if (l.points.empty()) continue;
            vkCmdBindPipeline(cmd, VK_PIPELINE_BIND_POINT_GRAPHICS, l.depthTest ? linePipe_ : lineOverlayPipe_);
            lineWidth(l.width);
            push(glm::mat4(1.0f), l.color);
            vkCmdBindVertexBuffers(cmd, 0, 1, &dyn.buffer, &overlayOffsets[i]);
            vkCmdDraw(cmd, uint32_t(l.points.size()), 1, 0, 0);
        }
    }

    if (imgui) ImGui_ImplVulkan_RenderDrawData(imgui, cmd);
    ctx_->endFrame();
    return true;
}

void Renderer::shutdown() {
    if (!ctx_ || !ctx_->device) return;
    vkDeviceWaitIdle(ctx_->device);
    for (auto& [id, gm] : meshes_) retire(gm);
    meshes_.clear();
    collectGarbage(true);
    for (auto& d : dynamic_) ctx_->destroyBuffer(d);
    for (VkPipeline p : {meshPipe_, highlightPipe_, linePipe_, lineOverlayPipe_})
        if (p) vkDestroyPipeline(ctx_->device, p, nullptr);
    if (layout_) vkDestroyPipelineLayout(ctx_->device, layout_, nullptr);
}

} // namespace pegasus
