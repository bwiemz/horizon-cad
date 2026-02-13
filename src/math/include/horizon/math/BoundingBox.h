#pragma once

#include "horizon/math/Vec3.h"

namespace hz::math {

class BoundingBox {
public:
    BoundingBox();
    BoundingBox(const Vec3& min, const Vec3& max);

    void expand(const Vec3& point);
    void expand(const BoundingBox& other);
    void reset();

    bool contains(const Vec3& point) const;
    bool contains(const BoundingBox& other) const;
    bool intersects(const BoundingBox& other) const;
    bool isValid() const;

    Vec3 center() const;
    Vec3 size() const;
    double diagonal() const;

    const Vec3& min() const { return m_min; }
    const Vec3& max() const { return m_max; }

private:
    Vec3 m_min;
    Vec3 m_max;
    bool m_valid;
};

}  // namespace hz::math
