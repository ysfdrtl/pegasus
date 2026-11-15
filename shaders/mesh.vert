#version 450
// Mesh vertex shader. Only positions are uploaded; flat normals are derived
// per-fragment from screen-space derivatives (ideal for STL triangle soups).
layout(location = 0) in vec3 inPos;

layout(push_constant) uniform PushConstants {
    mat4 modelView;
    vec4 color;
    vec4 proj; // (P00, P11, P22, P32) of a Vulkan-style perspective matrix
} pc;

layout(location = 0) out vec3 vViewPos;

void main() {
    vec4 v = pc.modelView * vec4(inPos, 1.0);
    vViewPos = v.xyz;
    gl_Position = vec4(pc.proj.x * v.x, pc.proj.y * v.y, pc.proj.z * v.z + pc.proj.w, -v.z);
}
