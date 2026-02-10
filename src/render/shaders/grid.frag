#version 330 core

in vec3 vNearPoint;
in vec3 vFarPoint;

out vec4 FragColor;

uniform mat4 uViewProj;
uniform float uNear;
uniform float uFar;

// Compute grid pattern at a given scale.
// Returns an RGBA color with alpha representing line visibility.
vec4 grid(vec3 fragPos3D, float scale) {
    vec2 coord = fragPos3D.xy * scale;
    vec2 derivative = fwidth(coord);
    vec2 grid = abs(fract(coord - 0.5) - 0.5) / derivative;
    float lineVal = min(grid.x, grid.y);
    float minimumz = min(derivative.y, 1.0);
    float minimumx = min(derivative.x, 1.0);

    vec4 color = vec4(0.35, 0.35, 0.35, 1.0 - min(lineVal, 1.0));

    // Highlight X axis (red line along Y=0)
    if (fragPos3D.y > -minimumz * 0.5 && fragPos3D.y < minimumz * 0.5) {
        color = vec4(0.8, 0.2, 0.2, 1.0 - min(lineVal, 1.0));
        color.a = max(color.a, 0.5);
    }
    // Highlight Y axis (green line along X=0)
    if (fragPos3D.x > -minimumx * 0.5 && fragPos3D.x < minimumx * 0.5) {
        color = vec4(0.2, 0.8, 0.2, 1.0 - min(lineVal, 1.0));
        color.a = max(color.a, 0.5);
    }

    return color;
}

// Compute clip-space depth for correct depth buffer writes.
float computeDepth(vec3 pos) {
    vec4 clipPos = uViewProj * vec4(pos, 1.0);
    return (clipPos.z / clipPos.w) * 0.5 + 0.5;
}

// Compute linear depth for fade-out effect.
float computeLinearDepth(vec3 pos) {
    vec4 clipPos = uViewProj * vec4(pos, 1.0);
    float clipDepth = clipPos.z / clipPos.w;
    clipDepth = clipDepth * 0.5 + 0.5;
    float linearDepth = (2.0 * uNear * uFar) /
                        (uFar + uNear - (2.0 * clipDepth - 1.0) * (uFar - uNear));
    return linearDepth / uFar;
}

void main() {
    // Find the ray-plane intersection with the XY plane (Z = 0).
    float t = -vNearPoint.z / (vFarPoint.z - vNearPoint.z);

    // Discard fragments behind the camera or that don't hit the plane.
    if (t < 0.0) discard;

    vec3 fragPos3D = vNearPoint + t * (vFarPoint - vNearPoint);

    // Write the correct depth so 3D objects properly occlude the grid.
    gl_FragDepth = computeDepth(fragPos3D);

    float linearDepth = computeLinearDepth(fragPos3D);
    float fadeFactor = max(0.0, 1.0 - linearDepth);

    // Two scales of grid lines: fine (1 unit) and coarse (10 units).
    vec4 gridSmall = grid(fragPos3D, 1.0);
    vec4 gridLarge = grid(fragPos3D, 0.1);

    // Start with the fine grid
    FragColor = gridSmall;
    FragColor.a *= fadeFactor;

    // Blend in the coarse grid
    FragColor = mix(FragColor, gridLarge, gridLarge.a * 0.6);
    FragColor.a *= fadeFactor;

    // Discard nearly-invisible fragments for performance.
    if (FragColor.a < 0.01) discard;
}
