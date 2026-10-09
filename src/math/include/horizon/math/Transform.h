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

    /// The transform that undoes this one.
    ///
    /// A transform is a translation, a rotation and a scale, in that order,
    /// and the composition of those three does not always come back out as
    /// three of them: undoing a scale and a rotation together needs a shear,
    /// which this has nowhere to put. So this is exact where the three can
    /// hold the answer, and that is a uniform scale (mirrored or not) or a
    /// scale with no rotation in it, which is every case a placement or a
    /// mirror makes. Where it cannot, the rotation and scale are returned
    /// undivided rather than wrong in a way that cannot be seen: Mat4's
    /// own inverse is exact there, and toMatrix() hands it to you.
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
