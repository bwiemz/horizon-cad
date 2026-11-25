// Typed surfaces and curves as NURBS (exact kernel, stage 3): each typed surface and curve is a
// rational NURBS over a parameter box or range, exactly, its corners or ends the typed one's; and
// the two NURBS surfaces a sketch is extruded and revolved into.

#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <limits>
#include <memory>
#include <numbers>
#include <stdexcept>
#include <string>
#include <vector>

#include "horizon/geometry/Frame.h"
#include "horizon/geometry/curves/AnalyticCurves.h"
#include "horizon/geometry/curves/NurbsCurve.h"
#include "horizon/geometry/surfaces/AnalyticSurfaces.h"
#include "horizon/geometry/surfaces/NurbsSurface.h"
#include "horizon/math/Mat4.h"
#include "horizon/math/Vec2.h"
#include "horizon/math/Vec3.h"

namespace {

using hz::geo::CircleCurve;
using hz::geo::Cone;
using hz::geo::Curve3;
using hz::geo::Cylinder;
using hz::geo::EllipseCurve;
using hz::geo::Frame;
using hz::geo::LineCurve;
using hz::geo::NurbsCurve;
using hz::geo::NurbsSurface;
using hz::geo::Plane;
using hz::geo::Sphere;
using hz::geo::Surface;
using hz::geo::Torus;
using hz::math::Mat4;
using hz::math::Vec2;
using hz::math::Vec3;

constexpr double kPi = std::numbers::pi;
constexpr int kGrid = 20;  // 20 x 20 = 400 samples

/// A tilted frame away from the origin, so nothing is axis-aligned; mirrored, it is left-handed.
Frame tilted(bool mirrored) {
    const Frame f = Frame::fromAxis({1.0, -2.0, 3.0}, Vec3{1.0, 2.0, 2.0} * (1.0 / 3.0), {0, 0, 1});
    return mirrored ? f.transformed(Mat4::scale({-1.0, 1.0, 1.0})) : f;
}

/// Whether @p x lies in [lo, hi], allowing @p slack, taken modulo @p period when it is not 0.
bool within(double x, double lo, double hi, double period, double slack) {
    if (period > 0.0) {
        x = lo + std::fmod(std::fmod(x - lo, period) + period, period);
        if (x > hi + slack && x - period >= lo - slack) x -= period;
    }
    return x >= lo - slack && x <= hi + slack;
}

struct SurfaceCase {
    std::string what;
    std::shared_ptr<const Surface> surface;
    double u0, u1, v0, v1;
};

std::vector<SurfaceCase> surfaceCases(bool mirrored) {
    const Frame f = tilted(mirrored);
    return {
        {"plane", std::make_shared<Plane>(f), -2.0, 3.0, -1.0, 4.0},
        {"cylinder", std::make_shared<Cylinder>(f, 3.0), 0.3, 2.9, -1.0, 2.0},
        {"cone", std::make_shared<Cone>(f, 2.0, 0.3), 0.5, 5.5, 0.0, 3.0},
        {"sphere", std::make_shared<Sphere>(f, 2.0), 1.0, 4.0, -1.2, 0.9},
        {"torus", std::make_shared<Torus>(f, 5.0, 1.0), 0.2, 4.0, -2.0, 1.5},
    };
}

}  // namespace

// Each typed surface over a box inside one period: 400 samples of its NURBS lie on it, inside the
// box, within 1e-12 of its size; the NURBS's corners are its corners; and its normal du x dv faces
// as the typed one's, so a face keeps its sense. In a right- and a left-handed frame.
TEST(ToNurbs, EachTypedSurfaceIsItsNurbs) {
    for (const bool mirrored : {false, true}) {
        for (const SurfaceCase& c : surfaceCases(mirrored)) {
            SCOPED_TRACE(c.what + (mirrored ? " (mirrored)" : ""));
            const Surface& s = *c.surface;
            const auto nurbs = s.toNurbs(c.u0, c.u1, c.v0, c.v1);
            ASSERT_NE(nurbs, nullptr);
            // Its size: the box round the typed surface's points over the box.
            Vec3 lo = s.evaluate(c.u0, c.v0);
            Vec3 hi = lo;
            for (int i = 0; i <= kGrid; ++i) {
                for (int j = 0; j <= kGrid; ++j) {
                    const Vec3 p = s.evaluate(c.u0 + (c.u1 - c.u0) * i / kGrid,
                                              c.v0 + (c.v1 - c.v0) * j / kGrid);
                    lo = {std::min(lo.x, p.x), std::min(lo.y, p.y), std::min(lo.z, p.z)};
                    hi = {std::max(hi.x, p.x), std::max(hi.y, p.y), std::max(hi.z, p.z)};
                }
            }
            const double size = (hi - lo).length();
            const double tol = 1e-12 * size;
            const double pu = s.periodicU() ? s.uRange().max - s.uRange().min : 0.0;
            const double pv = s.periodicV() ? s.vRange().max - s.vRange().min : 0.0;
            for (int i = 0; i < kGrid; ++i) {
                for (int j = 0; j < kGrid; ++j) {
                    const double u =
                        nurbs->uMin() + (nurbs->uMax() - nurbs->uMin()) * (i + 0.5) / kGrid;
                    const double v =
                        nurbs->vMin() + (nurbs->vMax() - nurbs->vMin()) * (j + 0.5) / kGrid;
                    const Vec3 p = nurbs->evaluate(u, v);
                    const Vec2 uv = s.invert(p);
                    EXPECT_LE((s.evaluate(uv.x, uv.y) - p).length(), tol) << u << ", " << v;
                    EXPECT_TRUE(within(uv.x, c.u0, c.u1, pu, 1e-9)) << "u " << uv.x;
                    EXPECT_TRUE(within(uv.y, c.v0, c.v1, pv, 1e-9)) << "v " << uv.y;
                    const auto dn = nurbs->derivatives(u, v);
                    const auto dt = s.derivatives(uv.x, uv.y);
                    EXPECT_GT(dn.du.cross(dn.dv).dot(dt.du.cross(dt.dv)), 0.0) << u << ", " << v;
                }
            }
            const struct {
                double nu, nv, u, v;
            } corners[] = {{nurbs->uMin(), nurbs->vMin(), c.u0, c.v0},
                           {nurbs->uMax(), nurbs->vMin(), c.u1, c.v0},
                           {nurbs->uMin(), nurbs->vMax(), c.u0, c.v1},
                           {nurbs->uMax(), nurbs->vMax(), c.u1, c.v1}};
            for (const auto& k : corners) {
                EXPECT_LE((nurbs->evaluate(k.nu, k.nv) - s.evaluate(k.u, k.v)).length(), tol)
                    << "corner " << k.u << ", " << k.v;
            }
        }
    }
}

// A whole turn of a periodic surface closes its NURBS where the typed one has its seam.
TEST(ToNurbs, AWholeTurnClosesTheNurbs) {
    const Frame f = tilted(false);
    const Cylinder cylinder(f, 3.0);
    const auto band = cylinder.toNurbs(0.0, 2.0 * kPi, -1.0, 2.0);
    ASSERT_NE(band, nullptr);
    EXPECT_TRUE(band->closedU());
    EXPECT_FALSE(band->closedV());
    const Torus torus(f, 5.0, 1.0);
    const auto ring = torus.toNurbs(0.0, 2.0 * kPi, 0.0, 2.0 * kPi);
    ASSERT_NE(ring, nullptr);
    EXPECT_TRUE(ring->closedU());
    EXPECT_TRUE(ring->closedV());
    EXPECT_LE((ring->evaluate(0.3, 1.1) - ring->evaluate(0.3 + 2.0 * kPi, 1.1)).length(), 1e-12);
}

// A box no NURBS can be, an empty or endless one, one more than a period round, or one outside a
// NURBS surface's domain, is refused.
TEST(ToNurbs, ABoxThatIsNoPatchIsRefused) {
    const Frame f = tilted(false);
    const Cylinder cylinder(f, 3.0);
    EXPECT_THROW((void)cylinder.toNurbs(1.0, 1.0, 0.0, 1.0), std::invalid_argument);
    EXPECT_THROW((void)cylinder.toNurbs(0.0, 7.0, 0.0, 1.0), std::invalid_argument);
    const Plane plane(f);
    EXPECT_THROW((void)plane.toNurbs(0.0, std::numeric_limits<double>::infinity(), 0.0, 1.0),
                 std::invalid_argument);
    // A NURBS surface that does not close has nothing beyond its domain.
    const NurbsSurface patch = NurbsSurface::makePlane({0, 0, 0}, {1, 0, 0}, {0, 1, 0}, 2.0, 3.0);
    EXPECT_THROW(
        (void)patch.toNurbs(patch.uMax() + 1.0, patch.uMax() + 2.0, patch.vMin(), patch.vMax()),
        std::invalid_argument);
}

// Each typed curve over a range: 400 samples of its NURBS lie on it, inside the range, within
// 1e-12 of its size, and its ends are the curve's at t0 and t1.
TEST(ToNurbs, EachTypedCurveIsItsNurbs) {
    for (const bool mirrored : {false, true}) {
        const Frame f = tilted(mirrored);
        const struct {
            std::string what;
            std::shared_ptr<const Curve3> curve;
            double t0, t1;
        } cases[] = {
            {"line", std::make_shared<LineCurve>(f.origin, f.x * 2.0 + f.y, 1.5), -2.0, 3.0},
            {"circle", std::make_shared<CircleCurve>(f, 2.5), 0.4, 5.1},
            {"whole circle", std::make_shared<CircleCurve>(f, 2.5), 1.0, 1.0 + 2.0 * kPi},
            {"ellipse", std::make_shared<EllipseCurve>(f, 4.0, 1.5), -1.0, 3.7},
        };
        for (const auto& c : cases) {
            SCOPED_TRACE(c.what + (mirrored ? " (mirrored)" : ""));
            const auto nurbs = c.curve->toNurbs(c.t0, c.t1);
            ASSERT_NE(nurbs, nullptr);
            double size = 0.0;
            for (int i = 0; i <= kGrid * kGrid; ++i) {
                const double t = c.t0 + (c.t1 - c.t0) * i / (kGrid * kGrid);
                size = std::max(size, (c.curve->evaluate(t) - c.curve->evaluate(c.t0)).length());
            }
            const double tol = 1e-12 * size;
            const double period = c.curve->periodic() ? 2.0 * kPi : 0.0;
            for (int i = 0; i < kGrid * kGrid; ++i) {
                const double t =
                    nurbs->tMin() + (nurbs->tMax() - nurbs->tMin()) * (i + 0.5) / (kGrid * kGrid);
                const Vec3 p = nurbs->evaluate(t);
                const double on = c.curve->closestParameter(p);
                EXPECT_LE((c.curve->evaluate(on) - p).length(), tol) << t;
                EXPECT_TRUE(within(on, c.t0, c.t1, period, 1e-9)) << "t " << on;
            }
            EXPECT_LE((nurbs->evaluate(nurbs->tMin()) - c.curve->evaluate(c.t0)).length(), tol);
            EXPECT_LE((nurbs->evaluate(nurbs->tMax()) - c.curve->evaluate(c.t1)).length(), tol);
            // It runs the curve's way.
            const double mid = 0.5 * (nurbs->tMin() + nurbs->tMax());
            const double on = c.curve->closestParameter(nurbs->evaluate(mid));
            EXPECT_GT(nurbs->derivatives(mid).d1.dot(c.curve->derivatives(on).d1), 0.0);
        }
    }
}

// A NURBS surface is its own NURBS: over a box inside its domain, the part of it there, cut by
// knot insertion and keeping its parameters; over its whole domain, a copy.
TEST(ToNurbs, ANurbsSurfaceIsCutToTheBox) {
    const NurbsSurface torus = NurbsSurface::makeTorus({1, 2, 3}, {0, 0, 1}, 5.0, 1.5);
    const double u0 = torus.uMin() + 0.2 * (torus.uMax() - torus.uMin());
    const double u1 = torus.uMin() + 0.7 * (torus.uMax() - torus.uMin());
    const double v0 = torus.vMin() + 0.1 * (torus.vMax() - torus.vMin());
    const double v1 = torus.vMin() + 0.45 * (torus.vMax() - torus.vMin());
    const auto part = torus.toNurbs(u0, u1, v0, v1);
    ASSERT_NE(part, nullptr);
    EXPECT_DOUBLE_EQ(part->uMin(), u0);
    EXPECT_DOUBLE_EQ(part->uMax(), u1);
    EXPECT_DOUBLE_EQ(part->vMin(), v0);
    EXPECT_DOUBLE_EQ(part->vMax(), v1);
    for (int i = 0; i <= kGrid; ++i) {
        for (int j = 0; j <= kGrid; ++j) {
            const double u = u0 + (u1 - u0) * i / kGrid;
            const double v = v0 + (v1 - v0) * j / kGrid;
            EXPECT_LE((part->evaluate(u, v) - torus.evaluate(u, v)).length(), 1e-12 * 13.0);
        }
    }
    const auto whole = torus.toNurbs(torus.uMin(), torus.uMax(), torus.vMin(), torus.vMax());
    ASSERT_NE(whole, nullptr);
    ASSERT_EQ(whole->controlPointCountU(), torus.controlPointCountU());
    ASSERT_EQ(whole->controlPointCountV(), torus.controlPointCountV());
    for (int i = 0; i < torus.controlPointCountU(); ++i) {
        for (int j = 0; j < torus.controlPointCountV(); ++j) {
            const Vec3 a = whole->controlPoints()[i][j];
            const Vec3 b = torus.controlPoints()[i][j];
            EXPECT_TRUE(a.x == b.x && a.y == b.y && a.z == b.z) << i << ", " << j;
        }
    }
    EXPECT_EQ(whole->weights(), torus.weights());
    EXPECT_EQ(whole->knotsU(), torus.knotsU());
    EXPECT_EQ(whole->knotsV(), torus.knotsV());
}

// Across the seam of a NURBS surface that closes, the box is the part from u0 round to u1: the
// surface cut to close at u0 instead, its parameters running from u0.
TEST(ToNurbs, ANurbsSurfaceIsCutRoundItsSeam) {
    const NurbsSurface cylinder = NurbsSurface::makeCylinder({0, 0, 0}, {0, 0, 1}, 2.0, 3.0);
    ASSERT_TRUE(cylinder.closedU());
    const double period = cylinder.uMax() - cylinder.uMin();
    const double u0 = cylinder.uMax() - 0.3 * period;
    const double u1 = cylinder.uMax() + 0.2 * period;
    const auto part = cylinder.toNurbs(u0, u1, cylinder.vMin(), cylinder.vMax());
    ASSERT_NE(part, nullptr);
    EXPECT_NEAR(part->uMin(), u0, 1e-12);
    EXPECT_NEAR(part->uMax(), u1, 1e-12);
    for (int i = 0; i <= kGrid; ++i) {
        const double u = u0 + (u1 - u0) * i / kGrid;
        const double v = cylinder.vMin() + 0.37 * (cylinder.vMax() - cylinder.vMin());
        EXPECT_LE((part->evaluate(u, v) - cylinder.evaluate(u, v)).length(), 1e-12 * 5.0) << u;
    }
}

// A NURBS curve's is the part of it over the range.
TEST(ToNurbs, ANurbsCurveIsItsSegment) {
    const NurbsCurve arc = NurbsCurve::makeArc({1, 2, 3}, 2.0, 0.2, 4.0);
    const double t0 = arc.tMin() + 0.25 * (arc.tMax() - arc.tMin());
    const double t1 = arc.tMin() + 0.8 * (arc.tMax() - arc.tMin());
    const auto part = arc.toNurbs(t0, t1);
    ASSERT_NE(part, nullptr);
    EXPECT_DOUBLE_EQ(part->tMin(), t0);
    EXPECT_DOUBLE_EQ(part->tMax(), t1);
    for (int i = 0; i <= kGrid; ++i) {
        const double t = t0 + (t1 - t0) * i / kGrid;
        EXPECT_LE((part->evaluate(t) - arc.evaluate(t)).length(), 1e-12 * 5.0) << t;
    }
}

// A rational quarter circle extruded is a quarter of a cylinder, exactly.
TEST(NurbsSurface, AnExtrusionIsExact) {
    const NurbsCurve quarter = NurbsCurve::makeArc({0, 0, 0}, 1.0, 0.0, kPi / 2.0);
    const NurbsSurface wall = NurbsSurface::makeExtrusion(quarter, {0, 0, 4});
    EXPECT_EQ(wall.degreeU(), quarter.degree());
    EXPECT_EQ(wall.degreeV(), 1);
    for (int i = 0; i < kGrid; ++i) {
        for (int j = 0; j < kGrid; ++j) {
            const double u = wall.uMin() + (wall.uMax() - wall.uMin()) * (i + 0.5) / kGrid;
            const double v = wall.vMin() + (wall.vMax() - wall.vMin()) * (j + 0.5) / kGrid;
            const Vec3 p = wall.evaluate(u, v);
            EXPECT_NEAR(std::hypot(p.x, p.y), 1.0, 1e-12) << u << ", " << v;
            EXPECT_GE(p.z, 0.0);
            EXPECT_LE(p.z, 4.0);
        }
    }
    // u along the curve, v along the offset.
    EXPECT_LE((wall.evaluate(wall.uMin(), wall.vMin()) - Vec3{1, 0, 0}).length(), 1e-15);
    EXPECT_LE((wall.evaluate(wall.uMax(), wall.vMax()) - Vec3{0, 1, 4}).length(), 1e-15);
}

// A quarter circle revolved about an axis through its centre is part of a sphere: every sample
// at the circle's radius from the centre. u runs round the axis, from the profile at u = 0 to it
// turned by the angle (right-handed about the axis) at the end; a whole turn closes.
TEST(NurbsSurface, ARevolutionIsExact) {
    const Vec3 centre{1, 2, 3};
    const Vec3 axis = Vec3{1, 1, 0} * (1.0 / std::sqrt(2.0));
    const Vec3 across = Vec3{1, -1, 1} * (1.0 / std::sqrt(3.0));  // perpendicular to the axis
    const double r = 2.5;
    const double w = std::sqrt(0.5);
    const NurbsCurve quarter({centre + across * r, centre + (across + axis) * r, centre + axis * r},
                             {1.0, w, 1.0}, {0, 0, 0, 1, 1, 1}, 2);
    for (const double angle : {1.3, 2.0 * kPi, -0.8}) {
        SCOPED_TRACE(angle);
        const NurbsSurface s = NurbsSurface::makeRevolution(quarter, centre, axis, angle);
        EXPECT_NEAR(s.uMin(), 0.0, 1e-15);
        EXPECT_NEAR(s.uMax(), std::abs(angle), 1e-15);
        for (int i = 0; i < kGrid; ++i) {
            for (int j = 0; j < kGrid; ++j) {
                const double u = s.uMin() + (s.uMax() - s.uMin()) * (i + 0.5) / kGrid;
                const double v = s.vMin() + (s.vMax() - s.vMin()) * (j + 0.5) / kGrid;
                EXPECT_NEAR((s.evaluate(u, v) - centre).length(), r, 1e-12) << u << ", " << v;
            }
        }
        // The profile at u = 0, turned by the angle at the end.
        const Vec3 start = quarter.evaluate(0.5);
        EXPECT_LE((s.evaluate(s.uMin(), 0.5) - start).length(), 1e-12);
        const Vec3 radial = start - centre - axis * (start - centre).dot(axis);
        const Vec3 turned = centre + axis * (start - centre).dot(axis) + radial * std::cos(angle) +
                            axis.cross(radial) * std::sin(angle);
        EXPECT_LE((s.evaluate(s.uMax(), 0.5) - turned).length(), 1e-12);
        EXPECT_EQ(s.closedU(), angle == 2.0 * kPi);
    }
}
