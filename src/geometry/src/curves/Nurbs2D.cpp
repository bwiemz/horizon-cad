#include "horizon/geometry/curves/Nurbs2D.h"

#include <algorithm>
#include <cmath>
#include <utility>

namespace hz::geo {

namespace {
std::vector<math::Vec3> lift(const std::vector<math::Vec2>& pts) {
    std::vector<math::Vec3> out;
    out.reserve(pts.size());
    for (const math::Vec2& p : pts) {
        out.emplace_back(p.x, p.y, 0.0);
    }
    return out;
}
}  // namespace

Nurbs2D::Nurbs2D(std::vector<math::Vec2> controlPoints, std::vector<double> weights,
                 std::vector<double> knots, int degree)
    : curve_(lift(controlPoints), std::move(weights), std::move(knots), degree) {}

math::Vec2 Nurbs2D::evaluate(double t) const {
    const math::Vec3 p = curve_.evaluate(t);
    return {p.x, p.y};
}

math::Vec2 Nurbs2D::derivative(double t, int order) const {
    const math::Vec3 d = curve_.derivative(t, order);
    return {d.x, d.y};
}

double Nurbs2D::tMin() const {
    return curve_.tMin();
}

double Nurbs2D::tMax() const {
    return curve_.tMax();
}

bool Nurbs2D::isClosed() const {
    const math::Vec2 a = evaluate(tMin());
    const math::Vec2 b = evaluate(tMax());
    const double scale = std::max({1.0, std::abs(a.x), std::abs(a.y)});
    return std::hypot(a.x - b.x, a.y - b.y) <= 1e-12 * scale;
}

}  // namespace hz::geo
