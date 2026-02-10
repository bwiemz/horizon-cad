#version 330 core

layout(location = 0) in vec3 aPos;
layout(location = 1) in vec3 aNormal;

out vec3 vWorldPos;
out vec3 vWorldNormal;

uniform mat4 uMVP;
uniform mat4 uModel;
uniform mat4 uNormalMatrix;

void main() {
    vec4 worldPos = uModel * vec4(aPos, 1.0);
    vWorldPos = worldPos.xyz;

    // Transform the normal to world space using the inverse-transpose matrix.
    vWorldNormal = normalize((uNormalMatrix * vec4(aNormal, 0.0)).xyz);

    gl_Position = uMVP * vec4(aPos, 1.0);
}
