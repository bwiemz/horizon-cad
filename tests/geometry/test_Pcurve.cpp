#include <gtest/gtest.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "horizon/geometry/Pcurve.h"
#include "horizon/geometry/curves/AnalyticCurves.h"
#include "horizon/geometry/curves/Circle2D.h"
#include "horizon/geometry/curves/Line2D.h"
#include "horizon/geometry/curves/Nurbs2D.h"
#include "horizon/geometry/curves/NurbsCurve.h"
#include "horizon/geometry/surfaces/AnalyticSurfaces.h"
#include "horizon/geometry/surfaces/NurbsSurface.h"

using namespace hz::geo;
namespace math = hz::math;

namespace {

constexpr double kPi = 3.14159265358979323846;
constexpr double kTwoPi = 2.0 * kPi;
/// A length that stands for the model's size in the relative bounds.
constexpr double kSize = 10.0;

Frame tilted() {
    return Frame::fromAxis({1.0, -2.0, 0.5}, {0.3, -0.5, 1.0}, {1.0, 0.2, 0.0});
}

/// The pcurve lands on the curve: its ends map onto C(t0) and C(t1), every point of it
/// stays on the curve, and its own deviation is as small as a closed form's.
void expectFollows(const Surface& s, const Curve3& c, double t0, double t1, const Pcurve& pc,
                   double bound) {
    ASSERT_TRUE(pc.problem.empty()) << pc.problem;
    ASSERT_NE(pc.curve, nullptr);
    EXPECT_LE(pc.deviation, bound);
    const math::Vec2 a = pc.curve->evaluate(pc.start);
    const math::Vec2 b = pc.curve->evaluate(pc.end);
    EXPECT_LE((s.evaluate(a.x, a.y) - c.evaluate(t0)).length(), bound);
    EXPECT_LE((s.evaluate(b.x, b.y) - c.evaluate(t1)).length(), bound);
    for (int k = 0; k <= 40; ++k) {
        const double f = k / 40.0;
        const math::Vec2 uv = pc.curve->evaluate(pc.start + f * (pc.end - pc.start));
        const math::Vec3 p = s.evaluate(uv.x, uv.y);
        const math::Vec3 q = c.evaluate(c.closestParameter(p));
        EXPECT_LE((p - q).length(), bound);
    }
}

template <class T>
const T* as(const Pcurve& pc) {
    return dynamic_cast<const T*>(pc.curve.get());
}

/// The distance from @p p to the curve, found independently of the code under test: the nearest
/// of a dense table of the curve's points, polished by Newton's method.
class DenseCurve {
public:
    explicit DenseCurve(const Curve3& c, int count = 600) : c_(c) {
        const Interval r = c.range();
        lo_ = r.min;
        hi_ = r.max;
        for (int i = 0; i <= count; ++i) {
            ts_.push_back(lo_ + (hi_ - lo_) * i / count);
            pts_.push_back(c.evaluate(ts_.back()));
        }
    }
    double distance(const math::Vec3& p) const {
        size_t best = 0;
        for (size_t i = 1; i < pts_.size(); ++i) {
            if ((pts_[i] - p).lengthSquared() < (pts_[best] - p).lengthSquared()) best = i;
        }
        double t = ts_[best];
        for (int it = 0; it < 40; ++it) {
            const CurveDerivatives d = c_.derivatives(t);
            const double denom = d.d1.dot(d.d1) + (d.point - p).dot(d.d2);
            if (!(denom > 0.0)) break;
            const double next = std::clamp(t - (d.point - p).dot(d.d1) / denom, lo_, hi_);
            const bool done = std::abs(next - t) <= 1e-13 * (hi_ - lo_);
            t = next;
            if (done) break;
        }
        return std::min((c_.evaluate(t) - p).length(), (pts_[best] - p).length());
    }

private:
    const Curve3& c_;
    double lo_ = 0.0;
    double hi_ = 0.0;
    std::vector<double> ts_;
    std::vector<math::Vec3> pts_;
};

/// The largest distance from the pcurve's image (8000 points of it) to the curve.
double denseDeviation(const Surface& s, const Curve3& c, const Pcurve& pc) {
    const DenseCurve table(c);
    double worst = 0.0;
    constexpr int kN = 8000;
    for (int k = 0; k <= kN; ++k) {
        const math::Vec2 uv = pc.curve->evaluate(pc.start + (pc.end - pc.start) * k / kN);
        worst = std::max(worst, table.distance(s.evaluate(uv.x, uv.y)));
    }
    return worst;
}

/// v stays inside the surface's v range all along the pcurve.
void expectVInRange(const Surface& s, const Pcurve& pc) {
    const Interval r = s.vRange();
    for (int k = 0; k <= 50; ++k) {
        const double v = pc.curve->evaluate(pc.start + (pc.end - pc.start) * k / 50.0).y;
        EXPECT_GE(v, r.min - 1e-12);
        EXPECT_LE(v, r.max + 1e-12);
    }
}

Frame leftHanded(Frame f) {
    f.y = -f.y;
    return f;
}

}  // namespace

TEST(Pcurve, EachClosedFormIsExact) {
    const double bound = 1e-12 * kSize;
    const double tol = 1e-6;
    const Frame f = tilted();

    {  // A line on a plane, of scale 1.5.
        const Plane s(f, 1.5);
        const math::Vec3 a = s.evaluate(0.5, -1.0);
        const math::Vec3 b = s.evaluate(3.0, 2.0);
        const LineCurve c(a, b - a, 1.0);
        const double len = (b - a).length();
        const Pcurve pc = pcurveOn(s, c, 0.0, len, tol);
        EXPECT_NE(as<Line2D>(pc), nullptr);
        expectFollows(s, c, 0.0, len, pc, bound);
    }
    for (const double dir : {1.0, -1.0}) {  // A circle on a plane, either way round.
        const Plane s(f, 1.5);
        const CircleCurve c(Frame::fromAxis(s.evaluate(1.0, 2.0), f.z * dir, {0.3, 1.0, 0.1}), 3.0);
        const Pcurve pc = pcurveOn(s, c, 0.4, 5.0, tol);
        EXPECT_NE(as<Circle2D>(pc), nullptr);
        expectFollows(s, c, 0.4, 5.0, pc, bound);
    }
    {  // An ellipse on a plane.
        const Plane s(f, 1.5);
        const EllipseCurve c(Frame::fromAxis(s.evaluate(1.0, 2.0), -f.z, {0.3, 1.0, 0.1}), 4.0,
                             2.0);
        const Pcurve pc = pcurveOn(s, c, 0.4, 5.5, tol);
        EXPECT_NE(as<Nurbs2D>(pc), nullptr);
        expectFollows(s, c, 0.4, 5.5, pc, bound);
    }
    {  // A generator of a cylinder, with an axial scale.
        const Cylinder s(f, 2.0, 1.5);
        const math::Vec3 a = s.evaluate(0.7, -1.0);
        const math::Vec3 b = s.evaluate(0.7, 2.0);
        const LineCurve c(a, b - a, 1.0);
        const double len = (b - a).length();
        const Pcurve pc = pcurveOn(s, c, 0.0, len, tol);
        EXPECT_NE(as<Line2D>(pc), nullptr);
        expectFollows(s, c, 0.0, len, pc, bound);
    }
    for (const double dir : {1.0, -1.0}) {  // A parallel of a cylinder.
        const Cylinder s(f, 2.0, 1.5);
        const CircleCurve c(Frame::fromAxis(f.origin + f.z * 1.5, f.z * dir, {0.2, 1.0, 0.0}), 2.0);
        const Pcurve pc = pcurveOn(s, c, 0.3, 4.0, tol);
        EXPECT_NE(as<Line2D>(pc), nullptr);
        expectFollows(s, c, 0.3, 4.0, pc, bound);
    }
    {  // A generator of a cone.
        const Cone s(f, 2.0, 0.4, 1.0);
        const math::Vec3 a = s.evaluate(2.0, 0.0);
        const math::Vec3 b = s.evaluate(2.0, 3.0);
        const LineCurve c(a, b - a, 1.0);
        const double len = (b - a).length();
        const Pcurve pc = pcurveOn(s, c, 0.0, len, tol);
        EXPECT_NE(as<Line2D>(pc), nullptr);
        expectFollows(s, c, 0.0, len, pc, bound);
    }
    {  // A parallel of a cone, at v = 1.5.
        const Cone s(f, 2.0, 0.4, 1.0);
        const CircleCurve c(Frame::fromAxis(f.origin + f.z * 1.5, f.z, {1.0, 0.0, 0.3}),
                            2.0 + 1.5 * std::tan(0.4));
        const Pcurve pc = pcurveOn(s, c, 0.0, 6.0, tol);
        EXPECT_NE(as<Line2D>(pc), nullptr);
        expectFollows(s, c, 0.0, 6.0, pc, bound);
    }
    {  // A parallel of a sphere.
        const Sphere s(f, 3.0);
        const CircleCurve c(
            Frame::fromAxis(f.origin + f.z * (3.0 * std::sin(0.5)), -f.z, {0.0, 1.0, 0.3}),
            3.0 * std::cos(0.5));
        const Pcurve pc = pcurveOn(s, c, 1.0, 4.0, tol);
        EXPECT_NE(as<Line2D>(pc), nullptr);
        expectFollows(s, c, 1.0, 4.0, pc, bound);
    }
    for (const double t0 : {0.4, kPi / 2.0}) {  // A meridian of a sphere: over the pole, and
                                                // leaving the south pole (in range).
        const Sphere s(f, 3.0);
        const double u0 = 0.7;
        const math::Vec3 e1 = f.x * std::cos(u0) + f.y * std::sin(u0);
        const CircleCurve c(Frame::fromAxis(f.origin, f.z.cross(e1), e1), 3.0);
        const Pcurve pc = pcurveOn(s, c, t0, t0 + 2.0, tol);
        const Line2D* line = as<Line2D>(pc);
        ASSERT_NE(line, nullptr);
        EXPECT_NEAR(line->start().x, line->end().x, 1e-13);
        expectFollows(s, c, t0, t0 + 2.0, pc, bound);
        if (t0 > 1.0) expectVInRange(s, pc);
    }
    {  // A parallel of a torus.
        const Torus s(f, 4.0, 1.5);
        const CircleCurve c(Frame::fromAxis(f.origin + f.z * (1.5 * std::sin(1.0)), f.z, {1, 0, 0}),
                            4.0 + 1.5 * std::cos(1.0));
        const Pcurve pc = pcurveOn(s, c, 0.5, 3.0, tol);
        EXPECT_NE(as<Line2D>(pc), nullptr);
        expectFollows(s, c, 0.5, 3.0, pc, bound);
    }
    {  // A meridian of a torus.
        const Torus s(f, 4.0, 1.5);
        const double u0 = 2.0;
        const math::Vec3 e1 = f.x * std::cos(u0) + f.y * std::sin(u0);
        const CircleCurve c(Frame::fromAxis(f.origin + e1 * 4.0, f.z.cross(e1), e1), 1.5);
        const Pcurve pc = pcurveOn(s, c, 0.2, 5.0, tol);
        const Line2D* line = as<Line2D>(pc);
        ASSERT_NE(line, nullptr);
        EXPECT_NEAR(line->start().x, line->end().x, 1e-13);
        expectFollows(s, c, 0.2, 5.0, pc, bound);
    }
}

TEST(Pcurve, ARimAcrossTheSeamIsContinuous) {
    const Cylinder s(Frame::fromAxis({0, 0, 0}, {0, 0, 1}), 2.0);
    const CircleCurve c(Frame::fromAxis({0, 0, 3}, {0, 0, 1}), 2.0);
    const Pcurve pc = pcurveOn(s, c, 3.0 * kPi / 2.0, 5.0 * kPi / 2.0, 1e-6);
    expectFollows(s, c, 3.0 * kPi / 2.0, 5.0 * kPi / 2.0, pc, 1e-11);
    double prev = pc.curve->evaluate(pc.start).x;
    EXPECT_NEAR(prev, 3.0 * kPi / 2.0, 1e-12);
    for (int k = 1; k <= 100; ++k) {
        const double u = pc.curve->evaluate(pc.start + (pc.end - pc.start) * k / 100.0).x;
        EXPECT_NEAR(u - prev, kPi / 100.0, 1e-12);
        prev = u;
    }
    EXPECT_NEAR(prev, 5.0 * kPi / 2.0, 1e-12);
}

TEST(Pcurve, ASeamGeneratorSitsAtTheWindowOfItsStartPoint) {
    // The window is the start point's inverted u, wrapped into [0, 2 pi): a generator on the
    // seam is at u = 0, or at 2 pi where the start point inverts there.
    const Cylinder s(Frame::fromAxis({0, 0, 0}, {0, 0, 1}), 2.0);
    const LineCurve c({2.0, 0.0, 0.0}, {0, 0, 1}, 1.0);
    const Pcurve pc = pcurveOn(s, c, 0.0, 5.0, 1e-6);
    const Line2D* line = as<Line2D>(pc);
    ASSERT_NE(line, nullptr);
    const double u = line->start().x;
    EXPECT_EQ(u, s.invert(c.evaluate(0.0)).x);
    EXPECT_TRUE(std::abs(u) < 1e-12 || std::abs(u - kTwoPi) < 1e-9) << u;
    EXPECT_EQ(line->end().x, u);
    expectFollows(s, c, 0.0, 5.0, pc, 1e-11);
}

namespace {

/// A bi-quadratic patch, gently curved.
NurbsSurface makePatch() {
    const double z[3][3] = {{0.0, 0.3, 0.1}, {0.2, 0.8, 0.2}, {0.0, 0.4, 0.3}};
    std::vector<std::vector<math::Vec3>> pts(3);
    std::vector<std::vector<double>> w(3, std::vector<double>(3, 1.0));
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            pts[static_cast<size_t>(i)].emplace_back(2.0 * i, 2.0 * j, z[i][j]);
        }
    }
    const std::vector<double> k = {0, 0, 0, 1, 1, 1};
    return NurbsSurface(pts, w, k, k, 2, 2);
}

math::Vec2 curvedUV(double l) {
    return {0.15 + 0.6 * l + 0.15 * l * l, 0.2 + 0.3 * l + 0.3 * l * l};
}

/// The degree-8 Bezier that the patch's image of curvedUV is, exactly, found by interpolating
/// it at nine parameters.
std::vector<math::Vec3> imageControlPoints(const NurbsSurface& s) {
    constexpr int kN = 9;
    auto bern = [](int i, double l) {
        double c = 1.0;
        for (int k = 0; k < i; ++k) c = c * (kN - 1 - k) / (k + 1);
        return c * std::pow(l, i) * std::pow(1.0 - l, kN - 1 - i);
    };
    std::array<std::array<double, kN + 3>, kN> m{};
    for (int r = 0; r < kN; ++r) {
        const double l = r / double(kN - 1);
        for (int i = 0; i < kN; ++i) m[r][i] = bern(i, l);
        const math::Vec2 uv = curvedUV(l);
        const math::Vec3 p = s.evaluate(uv.x, uv.y);
        m[r][kN] = p.x;
        m[r][kN + 1] = p.y;
        m[r][kN + 2] = p.z;
    }
    for (int c = 0; c < kN; ++c) {
        int piv = c;
        for (int r = c + 1; r < kN; ++r) {
            if (std::abs(m[r][c]) > std::abs(m[piv][c])) piv = r;
        }
        std::swap(m[c], m[piv]);
        for (int r = 0; r < kN; ++r) {
            if (r == c) continue;
            const double f = m[r][c] / m[c][c];
            for (int k = c; k < kN + 3; ++k) m[r][k] -= f * m[c][k];
        }
    }
    std::vector<math::Vec3> out;
    for (int r = 0; r < kN; ++r) {
        out.emplace_back(m[r][kN] / m[r][r], m[r][kN + 1] / m[r][r], m[r][kN + 2] / m[r][r]);
    }
    return out;
}

NurbsCurve bezier(const std::vector<math::Vec3>& pts) {
    std::vector<double> knots(9, 0.0);
    knots.insert(knots.end(), 9, 1.0);
    return NurbsCurve(pts, std::vector<double>(pts.size(), 1.0), knots, 8);
}

}  // namespace

TEST(Pcurve, ASplineOnASplineIsWithinTolerance) {
    const NurbsSurface s = makePatch();
    const NurbsCurve c = bezier(imageControlPoints(s));
    const double tol = 1e-6;
    for (const double t0 : {0.0, 1.0}) {
        const double t1 = 1.0 - t0;
        const Pcurve pc = pcurveOn(s, c, t0, t1, tol);
        EXPECT_NE(as<Nurbs2D>(pc), nullptr);
        EXPECT_LE(pc.deviation, tol);
        expectFollows(s, c, t0, t1, pc, tol);
        EXPECT_LE(denseDeviation(s, c, pc), tol);
        const math::Vec2 a = pc.curve->evaluate(pc.start);
        const math::Vec2 want = curvedUV(t0);
        EXPECT_NEAR(a.x, want.x, 1e-5);
        EXPECT_NEAR(a.y, want.y, 1e-5);
    }
}

TEST(Pcurve, ACurveOffTheSurfaceSaysSo) {
    const NurbsSurface s = makePatch();
    const double tol = 1e-6;
    std::vector<math::Vec3> pts = imageControlPoints(s);
    for (math::Vec3& p : pts) p.z += 10.0 * tol;
    const NurbsCurve c = bezier(pts);
    const Pcurve pc = pcurveOn(s, c, 0.0, 1.0, tol);
    EXPECT_FALSE(pc.problem.empty());
    EXPECT_NE(pc.problem.find("deviation"), std::string::npos) << pc.problem;
    EXPECT_GT(pc.deviation, tol);
}

TEST(Pcurve, AReversedRangeRunsTheOtherWay) {
    const Cylinder s(tilted(), 2.0);
    const CircleCurve c(Frame::fromAxis(tilted().origin + tilted().z * 1.0, tilted().z), 2.0);
    const Pcurve fwd = pcurveOn(s, c, 1.0, 2.5, 1e-6);
    const Pcurve rev = pcurveOn(s, c, 2.5, 1.0, 1e-6);
    expectFollows(s, c, 1.0, 2.5, fwd, 1e-11);
    expectFollows(s, c, 2.5, 1.0, rev, 1e-11);
    EXPECT_GT(rev.curve->evaluate(rev.start).x, rev.curve->evaluate(rev.end).x);
    EXPECT_LT(fwd.curve->evaluate(fwd.start).x, fwd.curve->evaluate(fwd.end).x);

    // The same on the general path.
    const NurbsSurface p = makePatch();
    const NurbsCurve g = bezier(imageControlPoints(p));
    const Pcurve back = pcurveOn(p, g, 1.0, 0.0, 1e-6);
    expectFollows(p, g, 1.0, 0.0, back, 2e-6);
}

TEST(Pcurve, AGeneralRimOverTheSeamIsUnwrappedAndContinuous) {
    // A NURBS circle is no closed form: sampled, inverted from each previous point, fitted.
    const Cylinder cyl(Frame::fromAxis({0, 0, 0}, {0, 0, 1}), 2.0);
    const NurbsCurve rim = NurbsCurve::makeCircle({0, 0, 3}, 2.0);
    const Interval r = rim.range();
    // From 0.3 of the circle's range to its end.
    const double t0 = r.min + 0.3 * (r.max - r.min);
    const Pcurve pc = pcurveOn(cyl, rim, t0, r.max, 1e-6);
    expectFollows(cyl, rim, t0, r.max, pc, 2e-6);
    double prev = pc.curve->evaluate(pc.start).x;
    for (int k = 1; k <= 200; ++k) {
        const double u = pc.curve->evaluate(pc.start + (pc.end - pc.start) * k / 200.0).x;
        EXPECT_LT(std::abs(u - prev), 0.2);
        prev = u;
    }
    // From half way round to the end: u runs from pi to 2 pi.
    const Pcurve full = pcurveOn(cyl, rim, r.min + 0.5 * (r.max - r.min), r.max, 1e-6);
    EXPECT_TRUE(full.problem.empty()) << full.problem;
    const double u1 = full.curve->evaluate(full.end).x;
    const double u0 = full.curve->evaluate(full.start).x;
    EXPECT_NEAR(u0, kPi, 1e-5);
    EXPECT_NEAR(u1, kTwoPi, 1e-5);
}

TEST(Pcurve, AGeneralCurveOnASphereAndATorusFollows) {
    const Frame f = tilted();
    const Sphere sph(f, 3.0);
    // A tilted small circle on the sphere: not a parallel, not a meridian.
    const math::Vec3 axis = (f.z + f.x * 0.5).normalized();
    const double h = 1.2;
    const math::Vec3 center = f.origin + axis * h;
    const NurbsCurve small = NurbsCurve::makeCircle(center, std::sqrt(9.0 - h * h), axis);
    const Interval r = small.range();
    const Pcurve a = pcurveOn(sph, small, r.min, r.max, 1e-6);
    expectFollows(sph, small, r.min, r.max, a, 2e-6);

    const Torus tor(f, 4.0, 1.5);
    // The torus's own outer equator as a NURBS circle.
    const NurbsCurve eq = NurbsCurve::makeCircle(f.origin, 5.5, f.z);
    const Interval q = eq.range();
    const Pcurve b = pcurveOn(tor, eq, q.min, q.max, 1e-6);
    expectFollows(tor, eq, q.min, q.max, b, 2e-6);
}

namespace {

/// A circle through the sphere's poles: from the south pole, along the meridian at azimuth u0
/// (on the side @p dir of e1), to the north pole in t = 0..pi.
CircleCurve poleToPole(const Frame& f, double u0, double dir, double radius) {
    const math::Vec3 e1 = f.x * std::cos(u0) + f.y * std::sin(u0);
    Frame c;
    c.origin = f.origin;
    c.x = -f.z;
    c.y = e1 * dir;
    c.z = c.x.cross(c.y);
    return CircleCurve(c, radius);
}

}  // namespace

TEST(Pcurve, ASphereMeridianLeavingAPoleStaysInRange) {
    const double tol = 1e-6;
    for (const bool lh : {false, true}) {
        const Frame f = lh ? leftHanded(tilted()) : tilted();
        const Sphere s(f, 3.0);
        for (const double dir : {1.0, -1.0}) {
            const CircleCurve c = poleToPole(f, 0.0, dir, 3.0);
            // The seam, south pole to north pole, the way it runs and the way back; and a quarter.
            for (const auto& [t0, t1] : {std::pair{0.0, kPi}, std::pair{kPi, 0.0},
                                         std::pair{0.0, kPi / 2.0}, std::pair{kPi / 2.0, kPi}}) {
                const Pcurve pc = pcurveOn(s, c, t0, t1, tol);
                ASSERT_NE(as<Line2D>(pc), nullptr);
                expectFollows(s, c, t0, t1, pc, 1e-11);
                expectVInRange(s, pc);
            }
        }
    }
}

TEST(Pcurve, AConeGeneratorFromTheApexIsALine) {
    const double tol = 1e-6;
    const Frame f = tilted();
    const Cone s(f, 2.0, 0.4, 1.0);
    const double apexV = -2.0 / std::tan(0.4);
    for (const double u0 : {0.0, 2.0}) {  // on the seam, and off it
        const math::Vec3 apex = s.evaluate(u0, apexV);
        const math::Vec3 base = s.evaluate(u0, 3.0);
        const LineCurve c(apex, base - apex, 1.0);
        const double len = (base - apex).length();
        for (const auto& [t0, t1] : {std::pair{0.0, len}, std::pair{len, 0.0}}) {
            const Pcurve pc = pcurveOn(s, c, t0, t1, tol);
            const Line2D* line = as<Line2D>(pc);
            ASSERT_NE(line, nullptr);
            EXPECT_TRUE(std::abs(line->start().x - u0) < 1e-9 ||
                        std::abs(line->start().x - u0 - kTwoPi) < 1e-9)
                << line->start().x;
            EXPECT_EQ(line->start().x, line->end().x);
            expectFollows(s, c, t0, t1, pc, 1e-11);
        }
    }
}

namespace {

/// A helix on the cylinder of radius 5 and a pitch of 1.5: (R cos t, R sin t, pitch t / 2 pi).
class Helix : public Curve3 {
public:
    CurveKind kind() const override { return CurveKind::Nurbs; }
    math::Vec3 evaluate(double t) const override {
        return {5.0 * std::cos(t), 5.0 * std::sin(t), 1.5 * t / kTwoPi};
    }
    CurveDerivatives derivatives(double t) const override {
        return {evaluate(t),
                {-5.0 * std::sin(t), 5.0 * std::cos(t), 1.5 / kTwoPi},
                {-5.0 * std::cos(t), -5.0 * std::sin(t), 0.0}};
    }
    Interval range() const override { return {0.0, 40.0 * kTwoPi}; }
    bool periodic() const override { return false; }
    double closestParameter(const math::Vec3&) const override { return 0.0; }
    std::unique_ptr<Curve3> transformed(const math::Mat4&) const override { return nullptr; }
};

}  // namespace

TEST(Pcurve, AHelixOfManyTurnsKeepsItsTurnsApart) {
    const Cylinder s(Frame::fromAxis({0, 0, 0}, {0, 0, 1}), 5.0);
    const Helix h;
    const double tol = 1e-6;
    for (const int turns : {10, 20}) {
        const double t1 = turns * kTwoPi;
        const Pcurve pc = pcurveOn(s, h, 0.0, t1, tol);
        ASSERT_TRUE(pc.problem.empty()) << turns << ": " << pc.problem;
        ASSERT_NE(pc.curve, nullptr);
        EXPECT_LE(pc.deviation, tol);
        // On the cylinder, the helix at parameter u is at height pitch u / 2 pi: the distance from
        // the pcurve's image to it is how far v is from that.
        double worst = 0.0;
        double prevU = pc.curve->evaluate(pc.start).x;
        for (int k = 0; k <= 40000; ++k) {
            const math::Vec2 uv = pc.curve->evaluate(pc.start + (pc.end - pc.start) * k / 40000.0);
            worst = std::max(worst, std::abs(uv.y - 1.5 * uv.x / kTwoPi));
            EXPECT_LT(std::abs(uv.x - prevU), 0.05);
            prevU = uv.x;
        }
        EXPECT_LE(worst, tol);
        EXPECT_NEAR(pc.curve->evaluate(pc.end).x, t1, 1e-6);
    }
}

namespace {

/// A wavy planar Bezier of degree 6, @p length long, in the plane z = 0.
NurbsCurve wavy(double length) {
    std::vector<math::Vec3> pts;
    for (int i = 0; i <= 6; ++i) {
        pts.emplace_back(length * i / 6.0,
                         0.1 * length * ((i % 2) ? 1.0 : -0.6) * (i % 3 ? 1 : 0.4), 0.0);
    }
    std::vector<double> knots(7, 0.0);
    knots.insert(knots.end(), 7, 1.0);
    return NurbsCurve(pts, std::vector<double>(pts.size(), 1.0), knots, 6);
}

}  // namespace

TEST(Pcurve, TheReportedDeviationNeverReadsBelowTheTrueOne) {
    const Plane plane(Frame::fromAxis({0, 0, 0}, {0, 0, 1}, {1, 0, 0}));
    for (const auto& [length, tol] :
         {std::pair{100.0, 1e-7}, std::pair{1000.0, 1e-6}, std::pair{10000.0, 1e-7}}) {
        const NurbsCurve c = wavy(length);
        const Pcurve pc = pcurveOn(plane, c, 0.0, 1.0, tol);
        const double truth = denseDeviation(plane, c, pc);
        EXPECT_TRUE(pc.problem.empty()) << length << ": " << pc.problem;
        EXPECT_LE(truth, tol) << length;
        EXPECT_GE(pc.deviation, truth * 0.999) << length;
    }
}

TEST(Pcurve, AnExactLongStraightSplineNeedsFewControlPoints) {
    const Plane plane(Frame::fromAxis({0, 0, 0}, {0, 0, 1}, {1, 0, 0}));
    for (const double length : {1000.0, 5000.0, 10000.0}) {
        const NurbsCurve c({{0, 0, 0}, {length, 0.3 * length, 0}}, {1.0, 1.0}, {0, 0, 1, 1}, 1);
        const Pcurve pc = pcurveOn(plane, c, 0.0, 1.0, 1e-7);
        EXPECT_TRUE(pc.problem.empty()) << length << ": " << pc.problem;
        ASSERT_NE(as<Nurbs2D>(pc), nullptr);
        EXPECT_LE(as<Nurbs2D>(pc)->curve().controlPointCount(), 40);
        EXPECT_LE(denseDeviation(plane, c, pc), 1e-7);
    }
}

TEST(Pcurve, MirroredFramesKeepTheirClosedForms) {
    const double tol = 1e-6;
    const double bound = 1e-12 * kSize;
    for (const bool surfaceLH : {false, true}) {
        for (const bool circleLH : {false, true}) {
            const Frame f = surfaceLH ? leftHanded(tilted()) : tilted();
            auto circle = [&](const math::Vec3& center, const math::Vec3& axis,
                              const math::Vec3& hint, double r) {
                Frame cf = Frame::fromAxis(center, axis, hint);
                return CircleCurve(circleLH ? leftHanded(cf) : cf, r);
            };
            {
                const Plane s(f, 1.5);
                const CircleCurve c = circle(s.evaluate(1.0, 2.0), f.z, {0.3, 1.0, 0.1}, 3.0);
                const Pcurve pc = pcurveOn(s, c, 0.4, 5.0, tol);
                EXPECT_NE(as<Circle2D>(pc), nullptr) << surfaceLH << circleLH;
                expectFollows(s, c, 0.4, 5.0, pc, bound);
            }
            {
                const Cylinder s(f, 2.0, 1.5);
                const CircleCurve c = circle(f.origin + f.z * 1.5, f.z, {0.2, 1.0, 0.0}, 2.0);
                const Pcurve pc = pcurveOn(s, c, 0.3, 4.0, tol);
                EXPECT_NE(as<Line2D>(pc), nullptr) << surfaceLH << circleLH;
                expectFollows(s, c, 0.3, 4.0, pc, bound);
            }
            {
                const Sphere s(f, 3.0);
                const math::Vec3 e1 = f.x * std::cos(0.7) + f.y * std::sin(0.7);
                const CircleCurve c = circle(f.origin, f.z.cross(e1), e1, 3.0);
                const Pcurve pc = pcurveOn(s, c, 0.4, 2.4, tol);
                EXPECT_NE(as<Line2D>(pc), nullptr) << surfaceLH << circleLH;
                expectFollows(s, c, 0.4, 2.4, pc, bound);
            }
            {
                const Torus s(f, 4.0, 1.5);
                const CircleCurve c = circle(f.origin + f.z * (1.5 * std::sin(1.0)), f.z, {1, 0, 0},
                                             4.0 + 1.5 * std::cos(1.0));
                const Pcurve pc = pcurveOn(s, c, 0.5, 3.0, tol);
                EXPECT_NE(as<Line2D>(pc), nullptr) << surfaceLH << circleLH;
                expectFollows(s, c, 0.5, 3.0, pc, bound);
            }
        }
    }
}

TEST(Pcurve, AFailureHasNoCurveAndAnInfiniteDeviation) {
    const Plane plane(Frame::fromAxis({0, 0, 0}, {0, 0, 1}));
    const LineCurve c({0, 0, 0}, {1, 0, 0});
    const Pcurve bad = pcurveOn(plane, c, 0.0, 1.0, -1.0);
    EXPECT_EQ(bad.curve, nullptr);
    EXPECT_FALSE(bad.problem.empty());
    EXPECT_EQ(bad.problem.find("finite"), std::string::npos);
    EXPECT_TRUE(std::isinf(bad.deviation));
    const Pcurve nan = pcurveOn(plane, c, 0.0, std::nan(""), 1e-6);
    EXPECT_EQ(nan.curve, nullptr);
    EXPECT_TRUE(std::isinf(nan.deviation));
}

// A rim running back across a closed B-spline's seam, against u: its samples, a period below
// the range, are measured where they are on the surface, not clamped onto the seam (which read
// as the curve standing off the surface by the height of the cylinder).
TEST(Pcurve, ARimRunningBackAcrossAClosedSplinesSeamFollows) {
    const NurbsSurface cyl = NurbsSurface::makeCylinder({0, 0, 0}, {0, 0, 1}, 1.0, 2.0);
    ASSERT_TRUE(cyl.periodicU());
    const math::Vec3 seam = cyl.evaluate(cyl.uMin(), cyl.vMax());
    const Frame fr = Frame::fromAxis({0, 0, 2}, {0, 0, 1}, {seam.x, seam.y, 0});
    for (const Frame& frame : {fr, Frame::fromAxis(fr.origin, {0, 0, -1}, fr.x)}) {
        const CircleCurve rim(frame, 1.0);
        for (const auto& [t0, t1] : {std::pair{kTwoPi, 0.0}, std::pair{0.0, kTwoPi}}) {
            const Pcurve pc = pcurveOn(cyl, rim, t0, t1, 1e-7);
            EXPECT_TRUE(pc.problem.empty()) << pc.problem;
            expectFollows(cyl, rim, t0, t1, pc, 2e-7);
        }
    }
}

// A straight edge along a patch's own edge (a box's, as Horizon writes one) follows it, either
// way: the search for each sample, from the one before along that edge, finds it.
TEST(Pcurve, ALineAlongAPatchsEdgeFollowsIt) {
    const NurbsSurface patch({{{0, 0, 0}, {0, 20, 0}}, {{10, 0, 0}, {10, 20, 0}}}, {{1, 1}, {1, 1}},
                             {0, 0, 1, 1}, {0, 0, 1, 1}, 1, 1);
    const NurbsCurve edge({{0, 20, 0}, {0, 0, 0}}, {1, 1}, {0, 0, 1, 1}, 1);
    for (const auto& [t0, t1] : {std::pair{0.0, 1.0}, std::pair{1.0, 0.0}}) {
        const Pcurve pc = pcurveOn(patch, edge, t0, t1, 1e-7);
        expectFollows(patch, edge, t0, t1, pc, 1e-7);
    }
}

// A curve that stands off its surface says how far it stands off, apart from how well any fit
// follows it: no pcurve comes nearer than that, so a caller that may allow the gap (an imported
// edge, its tolerance grown to cover it) fits again beyond it rather than chasing it. A closed
// form the same: a rim 1e-3 off its cylinder.
TEST(Pcurve, ACurveStandingOffItsSurfaceSaysHowFar) {
    const NurbsSurface s = makePatch();
    const double tol = 1e-6;
    std::vector<math::Vec3> pts = imageControlPoints(s);
    for (math::Vec3& p : pts) p.z += 10.0 * tol;
    const NurbsCurve c = bezier(pts);
    const Pcurve off = pcurveOn(s, c, 0.0, 1.0, tol);
    EXPECT_FALSE(off.problem.empty());
    EXPECT_GT(off.standOff, 5.0 * tol);
    EXPECT_LE(off.standOff, 10.0 * tol * (1.0 + 1e-9));
    EXPECT_LE(off.standOff, off.deviation);
    const Pcurve beyond = pcurveOn(s, c, 0.0, 1.0, 2.0 * off.standOff + tol);
    EXPECT_TRUE(beyond.problem.empty()) << beyond.problem;
    EXPECT_GE(beyond.deviation, off.standOff * (1.0 - 1e-9));
    EXPECT_LE(beyond.deviation, 2.0 * off.standOff + tol);

    const Cylinder cyl(tilted(), 2.0);
    const CircleCurve rim(Frame::fromAxis(tilted().origin + tilted().z * 1.5, tilted().z), 2.001);
    const Pcurve rimOff = pcurveOn(cyl, rim, 0.0, kTwoPi, tol);
    EXPECT_FALSE(rimOff.problem.empty());
    EXPECT_NEAR(rimOff.standOff, 1e-3, 1e-9);
    const Pcurve rimBeyond = pcurveOn(cyl, rim, 0.0, kTwoPi, 2.0 * rimOff.standOff + tol);
    EXPECT_TRUE(rimBeyond.problem.empty()) << rimBeyond.problem;
    EXPECT_NE(as<Line2D>(rimBeyond), nullptr) << "a parallel still, its closed form";
    EXPECT_NEAR(rimBeyond.deviation, 1e-3, 1e-9);
    EXPECT_NEAR(rimBeyond.standOff, 1e-3, 1e-9);
}
