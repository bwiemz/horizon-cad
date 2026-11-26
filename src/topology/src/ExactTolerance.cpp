#include "horizon/topology/ExactTolerance.h"

#include <algorithm>
#include <cmath>
#include <limits>

#include "horizon/geometry/curves/Curve3.h"

namespace hz::topo {

namespace {

constexpr int kEdgeSteps = 16;  // steps along each exact edge; its points are their ends

/// The box round the points exactExtent measures a solid by.
class Box {
public:
    /// @p solid's live vertices, and points along its live exact edges, added to the box.
    void include(const Solid& solid) {
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
    }

    /// Its diagonal; 0 for a box round nothing, or one not finite.
    double diagonal() const {
        const double length = (m_hi - m_lo).length();
        return std::isfinite(length) ? length : 0.0;
    }

private:
    static constexpr double kInf = std::numeric_limits<double>::infinity();

    void include(const math::Vec3& p) {
        m_lo = {std::min(m_lo.x, p.x), std::min(m_lo.y, p.y), std::min(m_lo.z, p.z)};
        m_hi = {std::max(m_hi.x, p.x), std::max(m_hi.y, p.y), std::max(m_hi.z, p.z)};
    }

    math::Vec3 m_lo{kInf, kInf, kInf};
    math::Vec3 m_hi{-kInf, -kInf, -kInf};
};

}  // namespace

bool hasExactFaces(const Solid& solid) {
    return std::any_of(solid.faces().begin(), solid.faces().end(),
                       [](const Face& f) { return f.alive && f.exactSurface; });
}

double exactExtent(const Solid& solid) {
    Box box;
    box.include(solid);
    return box.diagonal();
}

double exactExtent(const std::vector<const Solid*>& solids) {
    Box box;
    for (const Solid* solid : solids) {
        if (solid != nullptr) box.include(*solid);
    }
    return box.diagonal();
}

math::ModelTolerance exactTolerance(const Solid& solid) {
    return math::ModelTolerance::forExtent(exactExtent(solid));
}

ToleranceGrowth::ToleranceGrowth(double extent)
    : resolution(math::ModelTolerance::forExtent(extent)),
      sized(1e-3 * extent > resolution.linear),
      limit(sized ? 1e-3 * extent : resolution.linear) {}

bool ToleranceGrowth::grow(double& tolerance, double gap) const {
    if (!(gap > resolution.linear)) return true;
    if (!(gap <= limit)) return false;
    tolerance = std::max(tolerance, gap);
    return true;
}

}  // namespace hz::topo
