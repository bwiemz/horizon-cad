#include "horizon/topology/ExactTolerance.h"

#include <algorithm>
#include <cmath>
#include <limits>

#include "horizon/geometry/curves/Curve3.h"

namespace hz::topo {

namespace {

constexpr int kEdgeSteps = 16;  // steps along each exact edge; its points are their ends

}  // namespace

bool hasExactFaces(const Solid& solid) {
    return std::any_of(solid.faces().begin(), solid.faces().end(),
                       [](const Face& f) { return f.alive && f.exactSurface; });
}

double exactExtent(const Solid& solid) {
    constexpr double kInf = std::numeric_limits<double>::infinity();
    math::Vec3 lo(kInf, kInf, kInf);
    math::Vec3 hi(-kInf, -kInf, -kInf);
    const auto include = [&lo, &hi](const math::Vec3& p) {
        lo = {std::min(lo.x, p.x), std::min(lo.y, p.y), std::min(lo.z, p.z)};
        hi = {std::max(hi.x, p.x), std::max(hi.y, p.y), std::max(hi.z, p.z)};
    };
    for (const Vertex& v : solid.vertices()) {
        if (v.alive) include(v.point);
    }
    for (const Edge& e : solid.edges()) {
        if (!e.alive || !e.exactCurve) continue;
        for (int k = 0; k <= kEdgeSteps; ++k) {
            const double f = static_cast<double>(k) / kEdgeSteps;
            include(e.exactCurve->evaluate(e.curveStart + (e.curveEnd - e.curveStart) * f));
        }
    }
    const double diagonal = (hi - lo).length();
    return std::isfinite(diagonal) ? diagonal : 0.0;
}

math::ModelTolerance exactTolerance(const Solid& solid) {
    return math::ModelTolerance::forExtent(exactExtent(solid));
}

}  // namespace hz::topo
