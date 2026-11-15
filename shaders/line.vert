#version 450
layout(location = 0) in vec3 inPos;

layout(push_constant) uniform PushConstants {
    mat4 modelView;
    vec4 color;
    vec4 proj;
} pc;

void main() {
    vec4 v = pc.modelView * vec4(inPos, 1.0);
    gl_Position = vec4(pc.proj.x * v.x, pc.proj.y * v.y, pc.proj.z * v.z + pc.proj.w, -v.z);
}
