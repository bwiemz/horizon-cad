#include "horizon/geometry/surfaces/SurfaceBuilder.h"

#include <cmath>
#include <vector>

namespace hz::geo {

using hz::math::Vec3;

namespace {

bool sameKnots(const std::vector<double>& a, const std::vector<double>& b) {
    if (a.size() != b.size()) return false;
    for (size_t i = 0; i < a.size(); ++i) {
        if (std::abs(a[i] - b[i]) > 1e-12) return false;
    }
    return true;
}

bool unitWeights(const NurbsCurve& c) {
    for (double w : c.weights()) {
        if (std::abs(w - 1.0) > 1e-12) return false;
    }
    return true;
}

}  // namespace

std::optional<NurbsSurface> SurfaceBuilder::coonsPatch(const NurbsCurve& c0, const NurbsCurve& c1,
                                                       const NurbsCurve& c2, const NurbsCurve& c3,
                                                       double cornerTol) {
    // First-slice compatibility: matched degree/knots per direction, all
    // boundaries non-rational (degree elevation / knot refinement staged).
    if (c0.degree() != c2.degree() || c1.degree() != c3.degree() ||
        !sameKnots(c0.knots(), c2.knots()) || !sameKnots(c1.knots(), c3.knots()) ||
        !unitWeights(c0) || !unitWeights(c1) || !unitWeights(c2) || !unitWeights(c3)) {
        return std::nullopt;
    }

    const auto& bottom = c0.controlPoints();  // u direction at v = 0
    const auto& top = c2.controlPoints();     // u direction at v = 1
    const auto& left = c1.controlPoints();    // v direction at u = 0
    const auto& right = c3.controlPoints();   // v direction at u = 1
    const size_t nu = bottom.size();
    const size_t nv = left.size();
    if (nu < 2 || nv < 2 || top.size() != nu || right.size() != nv) return std::nullopt;

    // Corners must meet: bottom/left, bottom/right, top/left, top/right.
    const Vec3& p00 = bottom.front();
    const Vec3& p10 = bottom.back();
    const Vec3& p01 = top.front();
    const Vec3& p11 = top.back();
    if (left.front().distanceTo(p00) > cornerTol || right.front().distanceTo(p10) > cornerTol ||
        left.back().distanceTo(p01) > cornerTol || right.back().distanceTo(p11) > cornerTol) {
        return std::nullopt;
    }

    // Discrete Coons formula on the control net: the boundary rows/columns
    // are the input control points verbatim (so the surface reproduces all
    // four curves exactly); interior points blend the boundaries bilinearly
    // and subtract the doubly-counted corner term.
    std::vector<std::vector<Vec3>> net(nu, std::vector<Vec3>(nv));
    std::vector<std::vector<double>> weights(nu, std::vector<double>(nv, 1.0));
    for (size_t i = 0; i < nu; ++i) {
        const double s = static_cast<double>(i) / static_cast<double>(nu - 1);
        for (size_t j = 0; j < nv; ++j) {
            const double t = static_cast<double>(j) / static_cast<double>(nv - 1);
            const Vec3 ruled =
                bottom[i] * (1.0 - t) + top[i] * t + left[j] * (1.0 - s) + right[j] * s;
            const Vec3 corners = p00 * ((1.0 - s) * (1.0 - t)) + p10 * (s * (1.0 - t)) +
                                 p01 * ((1.0 - s) * t) + p11 * (s * t);
            net[i][j] = ruled - corners;
        }
    }

    return NurbsSurface(std::move(net), std::move(weights), c0.knots(), c1.knots(), c0.degree(),
                        c1.degree());
}

}  // namespace hz::geo
