#version 330 core

out vec4 FragColor;

uniform vec3 uLineColor;

void main() {
    FragColor = vec4(uLineColor, 1.0);
}
