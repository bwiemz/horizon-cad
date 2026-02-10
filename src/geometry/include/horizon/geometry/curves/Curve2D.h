#pragma once

#include "horizon/math/Vec2.h"
#include "horizon/math/BoundingBox.h"
#include <vector>

namespace hz::geo {

class Curve2D {
public:
    virtual ~Curve2D() = default;
    virtual math::Vec2 evaluate(double t) const = 0;
    virtual math::Vec2 derivative(double t, int order = 1) const = 0;
    virtual double tMin() const = 0;
    virtual double tMax() const = 0;
    virtual bool isClosed() const = 0;
    virtual double length() const;
    virtual double project(const math::Vec2& point) const;
    virtual std::vector<math::Vec2> tessellate(int segments = 64) const;
};

}  // namespace hz::geo
