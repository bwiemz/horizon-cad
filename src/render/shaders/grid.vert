#version 330 core

layout(location = 0) in vec3 aPos;

out vec3 vNearPoint;
out vec3 vFarPoint;

uniform mat4 uInvViewProj;

// Unproject a clip-space point to world space.
vec3 unprojectPoint(vec3 ndc) {
    vec4 world = uInvViewProj * vec4(ndc, 1.0);
    return world.xyz / world.w;
}

void main() {
    // The input quad covers [-1,1] in NDC xy.
    // We compute world-space rays through each vertex by unprojecting
    // the near and far planes.
    vNearPoint = unprojectPoint(vec3(aPos.xy, -1.0));
    vFarPoint  = unprojectPoint(vec3(aPos.xy,  1.0));
    gl_Position = vec4(aPos.xy, 0.0, 1.0);
}
