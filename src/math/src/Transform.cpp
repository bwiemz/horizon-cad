#include "horizon/math/Transform.h"

namespace hz::math {

const Transform Transform::Identity = Transform();

Transform::Transform() : m_translation(Vec3::Zero), m_rotation(Quaternion::Identity), m_scale({1.0, 1.0, 1.0}) {}

Transform::Transform(const Vec3& translation, const Quaternion& rotation, const Vec3& scale)
    : m_translation(translation), m_rotation(rotation), m_scale(scale) {}

Mat4 Transform::toMatrix() const {
    Mat4 s = Mat4::scale(m_scale);
    Mat4 r = m_rotation.toMatrix();
    Mat4 t = Mat4::translation(m_translation);
    return t * r * s;
}

Transform Transform::inverse() const {
    Quaternion invRot = m_rotation.inverse();
    Vec3 invScale = {1.0 / m_scale.x, 1.0 / m_scale.y, 1.0 / m_scale.z};
    Vec3 invTrans = invRot.rotate(Vec3{-m_translation.x * invScale.x,
                                        -m_translation.y * invScale.y,
                                        -m_translation.z * invScale.z});
    return {invTrans, invRot, invScale};
}

Transform Transform::operator*(const Transform& rhs) const {
    Vec3 newScale = {m_scale.x * rhs.m_scale.x, m_scale.y * rhs.m_scale.y,
                     m_scale.z * rhs.m_scale.z};
    Quaternion newRot = m_rotation * rhs.m_rotation;
    Vec3 newTrans = m_translation + m_rotation.rotate(
        Vec3{m_scale.x * rhs.m_translation.x, m_scale.y * rhs.m_translation.y,
             m_scale.z * rhs.m_translation.z});
    return {newTrans, newRot, newScale};
}

Vec3 Transform::transformPoint(const Vec3& p) const {
    Vec3 scaled = {p.x * m_scale.x, p.y * m_scale.y, p.z * m_scale.z};
    return m_translation + m_rotation.rotate(scaled);
}

Vec3 Transform::transformDirection(const Vec3& d) const {
    Vec3 scaled = {d.x * m_scale.x, d.y * m_scale.y, d.z * m_scale.z};
    return m_rotation.rotate(scaled);
}

}  // namespace hz::math
