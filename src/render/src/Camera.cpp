#include "horizon/render/Camera.h"

#include "horizon/math/Constants.h"
#include "horizon/math/Vec4.h"

#include <algorithm>
#include <cmath>

namespace hz::render {

Camera::Camera() = default;

void Camera::setPerspective(double fovY, double aspect, double nearPlane, double farPlane) {
    m_projType = ProjectionType::Perspective;
    m_fov = fovY;
    m_aspect = aspect;
    m_near = nearPlane;
    m_far = farPlane;
}

void Camera::setOrthographic(double width, double height, double nearPlane, double farPlane) {
    m_projType = ProjectionType::Orthographic;
    m_orthoWidth = width;
    m_orthoHeight = height;
    m_near = nearPlane;
    m_far = farPlane;
}

void Camera::lookAt(const math::Vec3& eye, const math::Vec3& target, const math::Vec3& up) {
    m_eye = eye;
    m_target = target;
    math::Vec3 n = up.normalized();
    m_up = (n.lengthSquared() < 1e-10) ? math::Vec3(0.0, 0.0, 1.0) : n;
}

void Camera::orbit(double deltaYaw, double deltaPitch) {
    // Orbit around m_target. Z is up (CAD convention).
    math::Vec3 offset = m_eye - m_target;
    double radius = offset.length();
    if (radius < 1e-10) return;

    // Convert to spherical coordinates (physics convention with Z up)
    // theta = angle from Z axis (polar), phi = angle in XY plane (azimuth)
    double theta = std::acos(std::clamp(offset.z / radius, -1.0, 1.0));
    double phi = std::atan2(offset.y, offset.x);

    phi += deltaYaw;
    theta -= deltaPitch;  // negative so that dragging up looks upward

    // Clamp theta to avoid gimbal lock at poles
    constexpr double kMinTheta = 0.01;
    constexpr double kMaxTheta = math::kPi - 0.01;
    theta = std::clamp(theta, kMinTheta, kMaxTheta);

    // Convert back to Cartesian
    m_eye.x = m_target.x + radius * std::sin(theta) * std::cos(phi);
    m_eye.y = m_target.y + radius * std::sin(theta) * std::sin(phi);
    m_eye.z = m_target.z + radius * std::cos(theta);
}

void Camera::pan(double deltaX, double deltaY) {
    // Pan in the camera's local right and up directions
    math::Vec3 forward = (m_target - m_eye).normalized();
    math::Vec3 right = forward.cross(m_up).normalized();
    math::Vec3 up = right.cross(forward).normalized();

    math::Vec3 offset = right * deltaX + up * deltaY;
    m_eye += offset;
    m_target += offset;
}

void Camera::zoom(double factor) {
    // Move eye closer to / farther from target
    math::Vec3 dir = m_eye - m_target;
    double dist = dir.length();
    double newDist = dist * factor;
    // Clamp minimum distance
    newDist = std::max(newDist, 0.01);

    if (dist > 1e-10) {
        m_eye = m_target + dir * (newDist / dist);
    }

    // Also adjust ortho size if in ortho mode
    if (m_projType == ProjectionType::Orthographic) {
        m_orthoWidth *= factor;
        m_orthoHeight *= factor;
    }
}

void Camera::fitAll(const math::BoundingBox& bbox) {
    if (!bbox.isValid()) return;

    math::Vec3 center = bbox.center();
    double diag = bbox.diagonal();
    if (diag < 1e-10) diag = 1.0;

    // Keep the current viewing direction but adjust distance
    math::Vec3 dir = (m_eye - m_target).normalized();
    if (dir.length() < 1e-10) {
        dir = math::Vec3(1.0, 1.0, 1.0).normalized();
    }

    // Calculate distance to fit the bounding box in view
    double distance;
    if (m_projType == ProjectionType::Perspective) {
        double halfFovRad = (m_fov * math::kDegToRad) * 0.5;
        double tanHalf = std::tan(halfFovRad);
        if (tanHalf < 1e-10) tanHalf = 1e-10;  // Guard against zero/tiny FOV.
        distance = (diag * 0.5) / tanHalf;
        distance *= 1.2;  // add some margin
    } else {
        distance = diag * 1.5;
        m_orthoWidth = diag * 1.2;
        m_orthoHeight = m_orthoWidth / m_aspect;
    }

    m_target = center;
    m_eye = center + dir * distance;
}

void Camera::setFrontView() {
    // Looking along -Y towards origin, Z up
    double dist = (m_eye - m_target).length();
    if (dist < 1e-10) dist = 10.0;
    m_eye = m_target + math::Vec3(0.0, -dist, 0.0);
    m_up = math::Vec3(0.0, 0.0, 1.0);
}

void Camera::setTopView() {
    // Looking along -Z down, Y up on screen
    double dist = (m_eye - m_target).length();
    if (dist < 1e-10) dist = 10.0;
    m_eye = m_target + math::Vec3(0.0, 0.0, dist);
    m_up = math::Vec3(0.0, 1.0, 0.0);
}

void Camera::setRightView() {
    // Looking along -X, Z up
    double dist = (m_eye - m_target).length();
    if (dist < 1e-10) dist = 10.0;
    m_eye = m_target + math::Vec3(dist, 0.0, 0.0);
    m_up = math::Vec3(0.0, 0.0, 1.0);
}

void Camera::setIsometricView() {
    double dist = (m_eye - m_target).length();
    if (dist < 1e-10) dist = 10.0;
    // Standard isometric direction
    double d = dist / std::sqrt(3.0);
    m_eye = m_target + math::Vec3(d, d, d);
    m_up = math::Vec3(0.0, 0.0, 1.0);
}

math::Mat4 Camera::viewMatrix() const {
    return math::Mat4::lookAt(m_eye, m_target, m_up);
}

math::Mat4 Camera::projectionMatrix() const {
    if (m_projType == ProjectionType::Perspective) {
        return math::Mat4::perspective(m_fov * math::kDegToRad, m_aspect, m_near, m_far);
    } else {
        double hw = m_orthoWidth * 0.5;
        double hh = m_orthoHeight * 0.5;
        return math::Mat4::ortho(-hw, hw, -hh, hh, m_near, m_far);
    }
}

math::Mat4 Camera::viewProjectionMatrix() const {
    return projectionMatrix() * viewMatrix();
}

std::pair<math::Vec3, math::Vec3> Camera::screenToRay(double screenX, double screenY,
                                                       int vpW, int vpH) const {
    // Convert screen coords to NDC [-1, 1]
    double ndcX = (2.0 * screenX / vpW) - 1.0;
    double ndcY = 1.0 - (2.0 * screenY / vpH);  // flip Y

    math::Mat4 invVP = viewProjectionMatrix().inverse();

    // Near point
    math::Vec4 nearNDC(ndcX, ndcY, -1.0, 1.0);
    math::Vec4 nearWorld = invVP * nearNDC;
    math::Vec3 nearPt = nearWorld.perspectiveDivide();

    // Far point
    math::Vec4 farNDC(ndcX, ndcY, 1.0, 1.0);
    math::Vec4 farWorld = invVP * farNDC;
    math::Vec3 farPt = farWorld.perspectiveDivide();

    math::Vec3 dir = (farPt - nearPt).normalized();
    return {nearPt, dir};
}

math::Vec3 Camera::unproject(double screenX, double screenY, double depth,
                              int vpW, int vpH) const {
    double ndcX = (2.0 * screenX / vpW) - 1.0;
    double ndcY = 1.0 - (2.0 * screenY / vpH);
    // Map depth from [0,1] to NDC [-1,1]
    double ndcZ = 2.0 * depth - 1.0;

    math::Mat4 invVP = viewProjectionMatrix().inverse();
    math::Vec4 clipPt(ndcX, ndcY, ndcZ, 1.0);
    math::Vec4 worldPt = invVP * clipPt;
    return worldPt.perspectiveDivide();
}

}  // namespace hz::render
