#version 450
layout(location = 0) in vec3 vViewPos;
layout(location = 0) out vec4 outColor;

layout(push_constant) uniform PushConstants {
    mat4 modelView;
    vec4 color;
    vec4 proj;
} pc;

void main() {
    vec3 n = normalize(cross(dFdx(vViewPos), dFdy(vViewPos)));
    vec3 V = normalize(-vViewPos);
    if (dot(n, V) < 0.0) n = -n; // double sided: STL normals are often unreliable

    vec3 keyDir = normalize(vec3(-0.35, 0.55, 0.75));
    float head = max(dot(n, V), 0.0);
    float key = max(dot(n, keyDir), 0.0);
    float spec = pow(max(dot(reflect(-keyDir, n), V), 0.0), 32.0);

    vec3 c = pc.color.rgb * (0.20 + 0.55 * head + 0.30 * key) + vec3(0.12 * spec);
    outColor = vec4(c, pc.color.a);
}
