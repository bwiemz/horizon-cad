#pragma once

#include "horizon/math/Mat4.h"
#include "horizon/math/Quaternion.h"
#include "horizon/math/Vec3.h"

namespace hz::math {

class Transform {
public:
    Transform();
    Transform(const Vec3& translation, const Quaternion& rotation, const Vec3& scale);

    void setTranslation(const Vec3& t) { m_translation = t; }
    void setRotation(const Quaternion& r) { m_rotation = r; }
    void setScale(const Vec3& s) { m_scale = s; }

    const Vec3& translation() const { return m_translation; }
    const Quaternion& rotation() const { return m_rotation; }
    const Vec3& scale() const { return m_scale; }

    Mat4 toMatrix() const;
    Transform inverse() const;
    Transform operator*(const Transform& rhs) const;

    Vec3 transformPoint(const Vec3& p) const;
    Vec3 transformDirection(const Vec3& d) const;

    static const Transform Identity;

private:
    Vec3 m_translation;
    Quaternion m_rotation;
    Vec3 m_scale;
};

}  // namespace hz::math
