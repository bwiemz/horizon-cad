#include "horizon/geometry/Pcurve.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdio>
#include <functional>
#include <limits>
#include <optional>
#include <utility>
#include <vector>

#include "AngleWrap.h"
#include "NurbsKnots.h"
#include "horizon/geometry/curves/AnalyticCurves.h"
#include "horizon/geometry/curves/Circle2D.h"
#include "horizon/geometry/curves/Line2D.h"
#include "horizon/geometry/curves/Nurbs2D.h"
#include "horizon/geometry/curves/NurbsCurve.h"
#include "horizon/geometry/surfaces/AnalyticSurfaces.h"
#include "horizon/geometry/surfaces/NurbsSurface.h"
#include "horizon/math/Constants.h"

namespace hz::geo {

namespace {

using math::Vec2;
using math::Vec3;

/// The most samples a fitted pcurve may use.
constexpr size_t kMaxSamples = 2048;
/// The most samples the curvature criteria alone may ask for, leaving the rest for refinement.
constexpr size_t kMaxCriteriaSamples = 1024;
constexpr int kInitialIntervals = 32;
/// The most steps the curve's turning may ask for at the start.
constexpr int kMaxInitial = 512;
/// How far the tangents or the normals may turn across one sample interval, as a cosine
/// (about 17 degrees).
constexpr double kTurnCos = 0.955;
/// The fraction of a period one sample step may cover in a periodic parameter: far below the
/// half period at which the step to the nearest representative would be ambiguous.
constexpr double kPeriodStep = 0.1;
/// How nearly parallel two unit directions must be for a closed form to be tried (the sine of
/// the angle; the measured deviation has the last word).
constexpr double kParallel = 1e-3;

double sgn(double x) {
    return x < 0.0 ? -1.0 : 1.0;
}

/// An angle from atan2, in [0, 2 pi).
double wrapTwoPi(double a) {
    return detail::wrapAngle(a);
}

double nearestRepresentative(double x, double ref, double period) {
    return x + period * std::round((ref - x) / period);
}

// ---------------------------------------------------------------------------------------------
// Peaks between samples (peakOfGap)
// ---------------------------------------------------------------------------------------------

/// The part of a bracket's larger side a golden-section probe takes: (3 - sqrt 5) / 2.
constexpr double kGolden = 0.3819660112501051;
/// The most probes one peak's search makes: pinning a peak to rounding takes some 40 at most (a
/// bracket shrinks by 0.618 a probe, and only to the root of rounding's share of the peak's
/// height), so this only stops a gap that is not a number.
constexpr int kMostPeakProbes = 64;
/// How many units in the last place of the points a gap is measured between pin its peak: two
/// searches of one peak from different samples then part by rounding alone.
constexpr double kPeakUlps = 4.0;

/// What a gap's points' magnitude @p reach rounds a gap measured between them to.
double peakRounding(double reach) {
    const double r = kPeakUlps * std::numeric_limits<double>::epsilon() * reach;
    return std::isfinite(r) ? r : 0.0;
}

/// The k for which @p x lies between @p at[k] and @p at[k + 1], @p at in order either way (at
/// least two of them); held to the first and last pair.
size_t bracketOf(const std::vector<double>& at, double x) {
    const bool rising = at.back() >= at.front();
    const auto it = rising ? std::upper_bound(at.begin(), at.end(), x)
                           : std::upper_bound(at.begin(), at.end(), x, std::greater<>());
    const auto k = static_cast<size_t>(std::max<std::ptrdiff_t>(it - at.begin() - 1, 0));
    return std::min(k, at.size() - 2);
}

struct Peak {
    double at = 0.0;   ///< where it was found
    double gap = 0.0;  ///< its gap there
};

/// The peaks peakOfGap searches for, highest first: each where it was found and its gap, which is
/// never below its sample's. @p above is a gap known beside the samples' own (the searches that
/// cannot lift a peak past it, or past @p within, are not made).
template <class GapAt>
std::vector<Peak> searchPeaks(const std::vector<double>& at, const std::vector<double>& gaps,
                              const GapAt& gapAt, double rounding, double above, double within) {
    std::vector<Peak> found;
    const size_t n = std::min(at.size(), gaps.size());
    if (n < 2) return found;
    double best = std::isfinite(above) ? above : 0.0;
    for (size_t i = 0; i < n; ++i) best = std::max(best, gaps[i]);  // a NaN is passed over
    struct Rising {
        size_t i;
        double height;  ///< how far it rises above the lower of the samples about it
        double reach;   ///< the most its peak may come to: its height again above it (a smooth
                        ///< peak rises at most a quarter of it above an inner sample, an eighth
                        ///< above an end one)
    };
    std::vector<Rising> rising;
    for (size_t i = 0; i < n; ++i) {
        const double g = gaps[i];
        if (!std::isfinite(g)) continue;
        if ((i > 0 && !(g >= gaps[i - 1])) || (i + 1 < n && !(g >= gaps[i + 1]))) continue;
        // Its height over the samples about it: an inner sample's two neighbours; an end
        // sample's two on its inner side, for the one beside it alone bounds nothing of a peak
        // near the far side of the end interval (its rise there can be all but nothing).
        double low = g;
        if (i > 0) low = std::min(low, gaps[i - 1]);
        if (i + 1 < n) low = std::min(low, gaps[i + 1]);
        if (i == 0 && n > 2) low = std::min(low, gaps[2]);
        if (i + 1 == n && n > 2) low = std::min(low, gaps[n - 3]);
        const double height = g - low;
        // A rise within rounding has no peak to pin beyond it.
        if (height > rounding) rising.push_back({i, height, g + height});
    }
    std::sort(rising.begin(), rising.end(),
              [](const Rising& a, const Rising& b) { return a.reach > b.reach; });
    for (const Rising& r : rising) {
        if (!(r.reach > std::max(best, within))) break;  // nor can any after it
        double a = at[r.i == 0 ? 0 : r.i - 1];
        double b = at[r.i + 1 == n ? n - 1 : r.i + 1];
        if (a > b) std::swap(a, b);
        double c = at[r.i];
        double fc = gaps[r.i];
        // A smooth peak falls off as the square of the way from it: once the bracket is this
        // narrow, its best point is within rounding of the peak (its height bounds its curvature,
        // to 4 times its height over the bracket's width squared about an inner sample and 1/2
        // at an end, wherever between the samples it lies; 8 covers both).
        const double settled = (b - a) * std::sqrt(rounding / (8.0 * r.height));
        for (int probe = 0; probe < kMostPeakProbes && b - a > settled; ++probe) {
            const double x = b - c > c - a ? c + kGolden * (b - c) : c - kGolden * (c - a);
            if (!(x > a && x < b) || x == c) break;  // no room left between them
            const double fx = gapAt(x);
            if (fx > fc) {
                (x > c ? a : b) = c;
                c = x;
                fc = fx;
            } else {
                (x > c ? b : a) = x;
            }
        }
        found.push_back({c, fc});
        best = std::max(best, fc);
    }
    return found;
}

// ---------------------------------------------------------------------------------------------
// Closed forms
// ---------------------------------------------------------------------------------------------

struct Form {
    std::shared_ptr<const Curve2D> curve;
    double start = 0.0;
    double end = 0.0;
    /// Whether the pcurve parameter is affine in the curve's t (so s(t) is known).
    bool affine = true;
    /// A bound on the deviation, known from how the form was built, never below the true one;
    /// negative where the form is measured instead.
    double bound = -1.0;
};

/// What rounding may add to a form found by mapping points (mapping them into (u, v), and
/// evaluating the pcurve, the surface and the curve), as a fraction of how far from the origin
/// those points lie: some 450 units in the last place, well above what a curve of any usual
/// degree loses.
constexpr double kRounding = 1e-13;

/// The pcurve of a B-spline curve in a plane whose (u, v) is affine: its control points mapped
/// by @p toUV (along the plane's normal onto it, then into (u, v)), its weights and knots kept,
/// so it runs with the curve's own parameter and its image is the curve projected onto the
/// plane. With @p straightAsLine, a straight curve of two points of one weight, over a range
/// inside its own, is the Line2D between its ends instead. The curve is a convex combination of
/// its control points, so it strays from the plane no farther than the farthest of them
/// (@p offPlane): that, and what rounding may add (kRounding of the farthest of them from the
/// origin, and of @p reach, how far the map's own origin and axes reach), is the bound. @p hull
/// receives the (u, v) points the pcurve is a convex combination of.
template <class ToUV, class OffPlane>
Form nurbsInPlane(const NurbsCurve& c, double t0, double t1, const ToUV& toUV,
                  const OffPlane& offPlane, double reach, bool straightAsLine,
                  std::vector<Vec2>& hull) {
    Form f;
    hull.clear();
    const Interval r = c.range();
    const bool inside = t0 >= r.min && t0 <= r.max && t1 >= r.min && t1 <= r.max;
    if (straightAsLine && c.degree() == 1 && c.controlPointCount() == 2 &&
        c.weights()[0] == c.weights()[1] && inside) {
        // Straight, at a speed that does not change: its points at t0 and t1 end it.
        const Vec3 a = c.evaluate(t0);
        const Vec3 b = c.evaluate(t1);
        hull = {toUV(a), toUV(b)};
        f.curve = std::make_shared<Line2D>(hull[0], hull[1]);
        f.start = 0.0;
        f.end = 1.0;
        f.bound = std::max(offPlane(a), offPlane(b)) +
                  kRounding * (reach + std::max(a.length(), b.length()));
        return f;
    }
    std::vector<Vec2> pts;
    pts.reserve(c.controlPoints().size());
    double bound = 0.0;
    double far = 0.0;
    for (const Vec3& p : c.controlPoints()) {
        pts.push_back(toUV(p));
        bound = std::max(bound, offPlane(p));
        far = std::max(far, p.length());
    }
    hull = pts;
    f.curve = std::make_shared<Nurbs2D>(std::move(pts), c.weights(), c.knots(), c.degree());
    f.start = t0;
    f.end = t1;
    f.bound = bound + kRounding * (reach + far);
    return f;
}

std::shared_ptr<const Curve2D> ellipseArc(const Vec2& c, const Vec2& a, const Vec2& b, double th0,
                                          double dth, int segments);

/// A curve on a bilinear patch that is a parallelogram: the patch is S = o + U a + V b + U V w
/// over its domain (U, V in [0, 1] across it, w its twist, P11 - P10 - P01 + P00), so its
/// (u, v) is affine on the plane of o, a and b, up to U V w. The pcurve of a line, or of a
/// B-spline curve, is the one of nurbsInPlane through that affine map, and its bound adds what
/// the twist can make of it (|w| times the largest |U| and |V| of the points the pcurve is a
/// convex combination of) and where those points lie outside the domain by rounding, what
/// evaluating the patch there (it clamps) moves. A pcurve that leaves the domain by more is no
/// closed form: the patch ends there.
std::optional<Form> patchForm(const NurbsSurface& s, const Curve3& c, double t0, double t1) {
    if (s.degreeU() != 1 || s.degreeV() != 1 || s.controlPointCountU() != 2 ||
        s.controlPointCountV() != 2) {
        return std::nullopt;
    }
    const auto& w = s.weights();
    if (!(w[0][0] == w[0][1] && w[0][0] == w[1][0] && w[0][0] == w[1][1])) return std::nullopt;
    const auto& net = s.controlPoints();
    const Vec3 o = net[0][0];
    const Vec3 a = net[1][0] - o;
    const Vec3 b = net[0][1] - o;
    const Vec3 twist = net[1][1] - net[1][0] - net[0][1] + o;
    const Vec3 cross = a.cross(b);
    const double area = cross.length();
    if (!std::isfinite(area) || !(area > 0.0)) return std::nullopt;
    const Vec3 n = cross * (1.0 / area);
    // The dual basis of a and b in their plane: fa . a = 1, fa . b = 0, and fb the other way.
    const Vec3 fa = b.cross(n) * (1.0 / area);
    const Vec3 fb = n.cross(a) * (1.0 / area);
    const double u0 = s.uMin();
    const double du = s.uMax() - s.uMin();
    const double v0 = s.vMin();
    const double dv = s.vMax() - s.vMin();
    const auto toUV = [&](const Vec3& p) {
        const Vec3 d = p - o;
        return Vec2(u0 + d.dot(fa) * du, v0 + d.dot(fb) * dv);
    };
    const auto offPlane = [&](const Vec3& p) { return std::abs((p - o).dot(n)); };

    Form f;
    std::vector<Vec2> hull;
    const double reach = o.length() + a.length() + b.length() + twist.length();
    if (c.kind() == CurveKind::Line) {
        const Vec3 p0 = c.evaluate(t0);
        const Vec3 p1 = c.evaluate(t1);
        hull = {toUV(p0), toUV(p1)};
        f.curve = std::make_shared<Line2D>(hull[0], hull[1]);
        f.start = 0.0;
        f.end = 1.0;
        f.bound = std::max(offPlane(p0), offPlane(p1)) +
                  kRounding * (reach + std::max(p0.length(), p1.length()));
    } else if (const auto* nc = dynamic_cast<const NurbsCurve*>(&c)) {
        // Horizon's own straight edges, the commonest here, are kept as cheap to evaluate as
        // they can be.
        f = nurbsInPlane(*nc, t0, t1, toUV, offPlane, reach, true, hull);
    } else if (c.kind() == CurveKind::Circle || c.kind() == CurveKind::Ellipse) {
        // C(th) = centre + ra cos th x + rb sin th y: projected into (u, v), the ellipse
        // q + A cos th + B sin th, the exact rational arc of it. Off the plane it strays by
        // d0 + ra cos th (x . n) + rb sin th (y . n), never more than |d0| + |(ra x.n, rb y.n)|.
        const auto* cc = dynamic_cast<const CircleCurve*>(&c);
        const auto* ec = dynamic_cast<const EllipseCurve*>(&c);
        if (cc == nullptr && ec == nullptr) return std::nullopt;
        const Frame& cf = cc != nullptr ? cc->frame() : ec->frame();
        const double ra = cc != nullptr ? cc->radius() : ec->majorRadius();
        const double rb = cc != nullptr ? cc->radius() : ec->minorRadius();
        if (cf.z.cross(n).length() > kParallel) return std::nullopt;
        const double span = t1 - t0;
        if (!(std::abs(span) <= 64.0 * math::kPi)) return std::nullopt;
        const int segments =
            std::max(1, static_cast<int>(std::ceil(std::abs(span) / math::kHalfPi)));
        const auto linear = [&](const Vec3& d) { return Vec2(d.dot(fa) * du, d.dot(fb) * dv); };
        f.curve =
            ellipseArc(toUV(cf.origin), linear(cf.x * ra), linear(cf.y * rb), t0, span, segments);
        f.start = 0.0;
        f.end = static_cast<double>(segments);
        f.affine = false;
        f.bound = offPlane(cf.origin) + std::hypot(ra * cf.x.dot(n), rb * cf.y.dot(n)) +
                  kRounding * (reach + cf.origin.length() + std::max(ra, rb));
        // A rational arc of positive weights lies in the hull of its control points.
        const auto& arc = static_cast<const Nurbs2D&>(*f.curve).curve().controlPoints();
        hull.reserve(arc.size());
        for (const Vec3& p : arc) hull.emplace_back(p.x, p.y);
    } else {
        return std::nullopt;
    }
    // Rounding at the domain's edges (an edge along the patch's own edge) is allowed this much.
    constexpr double kSlack = 1e-9;
    double mostU = 0.0;
    double mostV = 0.0;
    double outU = 0.0;
    double outV = 0.0;
    for (const Vec2& uv : hull) {
        const double cu = (uv.x - u0) / du;
        const double cv = (uv.y - v0) / dv;
        if (!(cu >= -kSlack && cu <= 1.0 + kSlack && cv >= -kSlack && cv <= 1.0 + kSlack)) {
            return std::nullopt;
        }
        mostU = std::max(mostU, std::abs(cu));
        mostV = std::max(mostV, std::abs(cv));
        outU = std::max({outU, -cu, cu - 1.0});
        outV = std::max({outV, -cv, cv - 1.0});
    }
    const double tw = twist.length();
    f.bound += tw * mostU * mostV + outU * (a.length() + tw) + outV * (b.length() + tw);
    return f;
}

/// The rational quadratic Bezier chain of the arc P(th) = c + A cos th + B sin th from
/// @p th0 over @p dth, exact as the affine image of a circle's rational arc.
std::shared_ptr<const Curve2D> ellipseArc(const Vec2& c, const Vec2& a, const Vec2& b, double th0,
                                          double dth, int segments) {
    const double alpha = dth / segments;
    const double w = std::cos(alpha / 2.0);
    auto at = [&](double th, double scale) {
        return Vec2(c.x + scale * (a.x * std::cos(th) + b.x * std::sin(th)),
                    c.y + scale * (a.y * std::cos(th) + b.y * std::sin(th)));
    };
    const auto count = static_cast<size_t>(2 * segments + 1);
    std::vector<Vec2> pts;
    std::vector<double> weights;
    pts.reserve(count);
    weights.reserve(count);
    for (int i = 0; i < segments; ++i) {
        pts.push_back(at(th0 + i * alpha, 1.0));
        weights.push_back(1.0);
        pts.push_back(at(th0 + (i + 0.5) * alpha, 1.0 / w));
        weights.push_back(w);
    }
    pts.push_back(at(th0 + dth, 1.0));
    weights.push_back(1.0);
    std::vector<double> knots(3, 0.0);
    knots.reserve(count + 3);
    for (int i = 1; i < segments; ++i) {
        knots.push_back(i);
        knots.push_back(i);
    }
    knots.insert(knots.end(), 3, static_cast<double>(segments));
    return std::make_shared<Nurbs2D>(std::move(pts), std::move(weights), std::move(knots), 2);
}

/// Whether @p p is on the axis of @p fr, to within @p tolerance (or rounding).
bool onAxis(const Frame& fr, const Vec3& p, double tolerance) {
    const Vec3 l = p - fr.origin;
    return std::hypot(l.dot(fr.x), l.dot(fr.y)) <= std::max(1e-9 * (1.0 + l.length()), tolerance);
}

/// A line: on a plane its two ends, on a cylinder or cone (@p axis, the surface's frame) a
/// generator, which keeps one u. A generator's u window is its start point's inverted u,
/// wrapped into [0, 2 pi); at the apex u means nothing, so the end that is off the axis gives it.
std::optional<Form> lineForm(const Surface& s, const Curve3& c, double t0, double t1,
                             const Frame* axis, double tolerance) {
    const Vec3 p0 = c.evaluate(t0);
    const Vec3 p1 = c.evaluate(t1);
    const Vec2 a = s.invert(p0);
    const Vec2 b = s.invert(p1);
    Form f;
    f.start = 0.0;
    f.end = 1.0;
    if (axis == nullptr) {
        f.curve = std::make_shared<Line2D>(a, b);
        return f;
    }
    double u = a.x;
    if (onAxis(*axis, p0, tolerance)) {
        if (onAxis(*axis, p1, tolerance)) return std::nullopt;
        u = b.x;
    }
    f.curve = std::make_shared<Line2D>(Vec2(u, a.y), Vec2(u, b.y));
    return f;
}

std::optional<Form> planeForm(const Plane& s, const Curve3& c, double t0, double t1,
                              double tolerance) {
    const Frame& pf = s.frame();
    const Vec3 normal = pf.x.cross(pf.y);
    if (c.kind() == CurveKind::Line) return lineForm(s, c, t0, t1, nullptr, tolerance);
    if (const auto* cc = dynamic_cast<const CircleCurve*>(&c)) {
        if (cc->frame().z.cross(normal).length() > kParallel) return std::nullopt;
        const Vec2 c2 = s.invert(cc->frame().origin);
        const Vec2 uv0 = s.invert(c.evaluate(t0));
        const double phi0 = std::atan2(uv0.y - c2.y, uv0.x - c2.x);
        // The circle turns about x cross y, which is -z in a left-handed frame.
        const double sigma = sgn(cc->frame().x.cross(cc->frame().y).dot(normal));
        Form f;
        f.curve = std::make_shared<Circle2D>(c2, cc->radius() / s.scale());
        f.start = phi0;
        f.end = phi0 + sigma * (t1 - t0);
        return f;
    }
    if (const auto* ec = dynamic_cast<const EllipseCurve*>(&c)) {
        if (ec->frame().z.cross(normal).length() > kParallel) return std::nullopt;
        const double span = t1 - t0;
        if (!(std::abs(span) <= 64.0 * math::kPi)) return std::nullopt;
        const int segments =
            std::max(1, static_cast<int>(std::ceil(std::abs(span) / math::kHalfPi)));
        const Frame& ef = ec->frame();
        const Vec2 c2 = s.invert(ef.origin);
        const double k = 1.0 / s.scale();
        const Vec2 a(ec->majorRadius() * ef.x.dot(pf.x) * k,
                     ec->majorRadius() * ef.x.dot(pf.y) * k);
        const Vec2 b(ec->minorRadius() * ef.y.dot(pf.x) * k,
                     ec->minorRadius() * ef.y.dot(pf.y) * k);
        Form f;
        f.curve = ellipseArc(c2, a, b, t0, span, segments);
        f.start = 0.0;
        f.end = static_cast<double>(segments);
        f.affine = false;
        return f;
    }
    if (const auto* nc = dynamic_cast<const NurbsCurve*>(&c)) {
        const double len = normal.length();
        if (!(len > 0.0)) return std::nullopt;
        const Vec3 unit = normal * (1.0 / len);
        // On a Plane the mapped control points are kept as they are, a straight curve's too: its
        // pcurve is the curve, in (u, v).
        std::vector<Vec2> hull;
        return nurbsInPlane(
            *nc, t0, t1, [&s](const Vec3& p) { return s.invert(p); },
            [&](const Vec3& p) { return std::abs((p - pf.origin).dot(unit)); },
            pf.origin.length() + s.scale(), false, hull);
    }
    return std::nullopt;
}

/// A circle on a surface of revolution: a parallel (its axis is the surface's) or, on a sphere
/// or a torus, a meridian (its plane holds the surface's axis).
std::optional<Form> circleOnRevolved(const Surface& s, const Frame& fr, const CircleCurve& cc,
                                     double t0, double t1, double tolerance) {
    // The axis the circle turns about: x cross y, which is -z in a left-handed frame.
    const Vec3 n = cc.frame().x.cross(cc.frame().y);
    const double d = n.dot(fr.z);
    const Vec3 start = cc.evaluate(t0);
    if (std::sqrt(std::max(0.0, 1.0 - d * d)) <= kParallel) {
        // A parallel: v is that of its start point, u turns with t.
        const Vec2 uv0 = s.invert(start);
        const double sigma = sgn(n.dot(fr.x.cross(fr.y)));
        Form f;
        f.curve = std::make_shared<Line2D>(uv0, Vec2(uv0.x + sigma * (t1 - t0), uv0.y));
        f.start = 0.0;
        f.end = 1.0;
        return f;
    }
    if (std::abs(d) > kParallel) return std::nullopt;
    const auto* sphere = dynamic_cast<const Sphere*>(&s);
    const auto* torus = dynamic_cast<const Torus*>(&s);
    if (sphere == nullptr && torus == nullptr) return std::nullopt;
    // A meridian: u is that of the circle's plane, on the side of the start point (of the
    // circle's centre, for a torus), so it does not come from inverting a pole.
    const Vec3 h = n.cross(fr.z);
    Vec3 ref = (sphere != nullptr ? start : cc.frame().origin) - fr.origin;
    const double scaleRef = sphere != nullptr ? sphere->radius() : torus->majorRadius();
    if (sphere != nullptr && onAxis(fr, start, tolerance)) {
        // The arc leaves a pole: it goes the way its middle is, which keeps v in range for an
        // arc that does not cross a pole.
        ref = cc.evaluate(0.5 * (t0 + t1)) - fr.origin;
    }
    const double ra = ref.dot(fr.x);
    const double rb = ref.dot(fr.y);
    const double hx = h.dot(fr.x);
    const double hy = h.dot(fr.y);
    double side = 1.0;
    if (std::hypot(ra, rb) > 1e-9 * scaleRef && ra * hx + rb * hy < 0.0) side = -1.0;
    const double u0 = wrapTwoPi(std::atan2(side * hy, side * hx));
    const Vec3 e1 = fr.x * std::cos(u0) + fr.y * std::sin(u0);
    const Vec3 p = start - fr.origin;
    double psi0 = 0.0;
    if (sphere != nullptr) {
        psi0 = std::atan2(p.dot(fr.z), p.dot(e1));
    } else {
        psi0 = wrapTwoPi(std::atan2(p.dot(fr.z), p.dot(e1) - torus->majorRadius()));
    }
    const double sigma = sgn(n.dot(e1.cross(fr.z)));
    Form f;
    f.curve = std::make_shared<Line2D>(Vec2(u0, psi0), Vec2(u0, psi0 + sigma * (t1 - t0)));
    f.start = 0.0;
    f.end = 1.0;
    return f;
}

/// The closed form @p c has on @p s, if any (still to be measured or bounded).
std::optional<Form> closedFormCandidate(const Surface& s, const Curve3& c, double t0, double t1,
                                        double tolerance) {
    switch (s.kind()) {
        case SurfaceKind::Plane:
            return planeForm(static_cast<const Plane&>(s), c, t0, t1, tolerance);
        case SurfaceKind::Cylinder:
        case SurfaceKind::Cone: {
            const Frame& fr = s.kind() == SurfaceKind::Cylinder
                                  ? static_cast<const Cylinder&>(s).frame()
                                  : static_cast<const Cone&>(s).frame();
            if (c.kind() == CurveKind::Line) return lineForm(s, c, t0, t1, &fr, tolerance);
            if (const auto* cc = dynamic_cast<const CircleCurve*>(&c)) {
                if (std::abs(cc->frame().z.dot(fr.z)) < 1.0 - kParallel * kParallel) {
                    return std::nullopt;
                }
                return circleOnRevolved(s, fr, *cc, t0, t1, tolerance);
            }
            return std::nullopt;
        }
        case SurfaceKind::Sphere:
        case SurfaceKind::Torus: {
            const Frame& fr = s.kind() == SurfaceKind::Sphere
                                  ? static_cast<const Sphere&>(s).frame()
                                  : static_cast<const Torus&>(s).frame();
            if (const auto* cc = dynamic_cast<const CircleCurve*>(&c)) {
                return circleOnRevolved(s, fr, *cc, t0, t1, tolerance);
            }
            return std::nullopt;
        }
        case SurfaceKind::Nurbs:
            if (const auto* nurbs = dynamic_cast<const NurbsSurface*>(&s)) {
                return patchForm(*nurbs, c, t0, t1);
            }
            return std::nullopt;
    }
    return std::nullopt;
}

/// The largest distance between the closed form's image and the curve, at its ends and along
/// it: pcurveGap's, the validator's measure, at 49 points and at the peaks between them that could
/// pass @p within; and where the form's parameter is affine in the curve's, each of those points'
/// distance from the curve's point at the same parameter too, which a form running the wrong way
/// or at the wrong speed cannot pass.
double measureForm(const Surface& s, const Curve3& c, const Form& f, double t0, double t1,
                   double within) {
    constexpr int kChecks = 48;
    std::vector<double> at;
    at.reserve(kChecks + 1);
    double dev = 0.0;
    for (int k = 0; k <= kChecks; ++k) {
        const double lambda = static_cast<double>(k) / kChecks;
        at.push_back(f.start + lambda * (f.end - f.start));
        if (f.affine) {
            const Vec2 uv = f.curve->evaluate(at.back());
            const Vec3 p = s.evaluate(uv.x, uv.y);
            dev = std::max(dev, (p - c.evaluate(t0 + lambda * (t1 - t0))).length());
        }
    }
    dev = std::max(dev, pcurveGap(s, *f.curve, at, c, within).strays);
    const Vec2 a = f.curve->evaluate(f.start);
    const Vec2 b = f.curve->evaluate(f.end);
    dev = std::max(dev, (s.evaluate(a.x, a.y) - c.evaluate(t0)).length());
    dev = std::max(dev, (s.evaluate(b.x, b.y) - c.evaluate(t1)).length());
    return dev;
}

// ---------------------------------------------------------------------------------------------
// The general method
// ---------------------------------------------------------------------------------------------

struct Sample {
    double t = 0.0;
    /// C(t) and C'(t), kept while the sample is placed again.
    Vec3 point;
    Vec3 tangent;
    Vec2 uv;
    /// |S(uv) - C(t)|: how far the curve stands off the surface here.
    double gap = 0.0;
    /// The surface's unit normal at uv (where du x dv vanishes, Surface::normal's limit).
    Vec3 normal;
    /// |dS/du| and |dS/dv| at uv: how far a step in u or v moves across the surface.
    double speedU = 0.0;
    double speedV = 0.0;
    bool singular = false;
    /// On a NURBS surface, the surface where it was evaluated at @c evaluatedAt (its point and
    /// first derivatives, evaluateWithDerivatives'): the first step of Newton's method from this
    /// sample's (u, v), while that is still uv, to the last bit.
    SurfacePoint surface;
    Vec2 evaluatedAt{std::numeric_limits<double>::quiet_NaN(),
                     std::numeric_limits<double>::quiet_NaN()};
};

/// Whether @p a and @p b are the same (u, v), bit for bit (equal and of one sign; a NaN is equal
/// to nothing).
bool sameBits(const Vec2& a, const Vec2& b) {
    return a.x == b.x && a.y == b.y && std::signbit(a.x) == std::signbit(b.x) &&
           std::signbit(a.y) == std::signbit(b.y);
}

/// A (u, v) a pcurve's first sample is placed from (pcurveOn's startNear), and how near its point
/// it must land to be taken.
struct StartNear {
    Vec2 uv;
    double within = 0.0;
};

/// Places curve points on the surface, each from the (u, v) of the sample before it; one
/// evaluation of the surface there gives the gap, whether u means anything, and the normal.
class Inverter {
public:
    Inverter(const Surface& s, const Curve3& c)
        : s_(s), c_(c), nurbs_(dynamic_cast<const NurbsSurface*>(&s)) {}

    /// A sample at @p t, placed from @p prev.
    Sample at(double t, const Sample* prev) const { return at(t, c_.derivatives(t), prev); }

    /// A sample at @p t, whose point and tangent @p d already has, placed from @p prev; the first
    /// (no @p prev) from @p near, where it is given and lands (place).
    Sample at(double t, const CurveDerivatives& d, const Sample* prev,
              const StartNear* near = nullptr) const {
        Sample out;
        out.t = t;
        out.point = d.point;
        out.tangent = d.d1;
        place(out, prev, near);
        return out;
    }

    /// The (u, v) of @p smp's point near @p prev's: from prev's, by Newton on a NURBS surface,
    /// else the nearest representative of the inverted point. A first sample (no @p prev) on a
    /// NURBS surface is placed from @p near by Newton too, as a later one is from the sample
    /// before it, where that lands within @p near's tolerance of its point: no point of the
    /// surface is nearer it by more, so the search of the whole surface (invert) would find the
    /// same point (or, where the surface passes it twice, at a seam or a pole, the same point by
    /// another (u, v)). Elsewhere it is searched for.
    void place(Sample& smp, const Sample* prev, const StartNear* near = nullptr) const {
        const Vec3& p = smp.point;
        Vec2 uv;
        if (nurbs_ != nullptr && prev != nullptr) {
            // Its first step from the surface prev was placed on, already evaluated there.
            const SurfacePoint* at =
                sameBits(prev->evaluatedAt, prev->uv) ? &prev->surface : nullptr;
            const auto r = nurbs_->project(p, prev->uv.x, prev->uv.y, at);
            uv = Vec2(r.first, r.second);
        } else if (nurbs_ == nullptr || near == nullptr || !landsFrom(*near, p, uv)) {
            uv = s_.invert(p);
        }
        if (prev != nullptr) {
            if (s_.periodicU()) {
                const Interval r = s_.uRange();
                uv.x = nearestRepresentative(uv.x, prev->uv.x, r.max - r.min);
            }
            if (s_.periodicV()) {
                const Interval r = s_.vRange();
                uv.y = nearestRepresentative(uv.y, prev->uv.y, r.max - r.min);
            }
        }
        Local here = local(uv);
        // At a pole or an apex u means nothing: take the neighbour's.
        smp.singular = here.du.length() <= 1e-9 * std::max(1.0, p.length());
        if (smp.singular && prev != nullptr && uv.x != prev->uv.x) {
            uv.x = prev->uv.x;
            here = local(uv);
        }
        smp.uv = uv;
        if (nurbs_ != nullptr) {
            smp.surface = {here.point, here.du, here.dv};
            smp.evaluatedAt = uv;
        }
        smp.gap = (here.point - p).length();
        smp.speedU = here.du.length();
        smp.speedV = here.dv.length();
        const Vec3 cross = here.du.cross(here.dv);
        const double len = cross.length();
        const double scale = std::max(here.du.length(), here.dv.length());
        smp.normal =
            len > 1e-9 * scale * scale && len > 0.0 ? cross * (1.0 / len) : s_.normal(uv.x, uv.y);
    }

private:
    struct Local {
        Vec3 point;
        Vec3 du;
        Vec3 dv;
    };

    /// Whether Newton's method from @p near's (u, v) lands within its tolerance of @p p, the
    /// (u, v) it lands on in @p uv.
    bool landsFrom(const StartNear& near, const Vec3& p, Vec2& uv) const {
        const auto r = nurbs_->project(p, near.uv.x, near.uv.y);
        uv = Vec2(r.first, r.second);
        return (nurbs_->evaluate(uv.x, uv.y) - p).length() <= near.within;
    }

    /// The surface at @p uv, to first derivatives (a NURBS surface without its second ones).
    Local local(const Vec2& uv) const {
        if (nurbs_ != nullptr) {
            const SurfacePoint sp = nurbs_->evaluateWithDerivatives(uv.x, uv.y);
            return {sp.point, sp.du, sp.dv};
        }
        const SurfaceDerivatives d = s_.derivatives(uv.x, uv.y);
        return {d.point, d.du, d.dv};
    }

    const Surface& s_;
    const Curve3& c_;
    const NurbsSurface* nurbs_;
};

/// Whether the interval between @p a and @p b turns the curve or the surface too far, or steps
/// a periodic parameter too far, to take its straight chord on trust.
bool needsSplit(const Surface& s, const Sample& a, const Sample& b) {
    if (!(std::abs(b.t - a.t) > 1e-12 * (1.0 + std::abs(a.t)))) return false;
    const double la = a.tangent.length();
    const double lb = b.tangent.length();
    if (la > 0.0 && lb > 0.0 && a.tangent.dot(b.tangent) < kTurnCos * la * lb) return true;
    if (a.normal.length() > 0.5 && b.normal.length() > 0.5 && a.normal.dot(b.normal) < kTurnCos) {
        return true;
    }
    if (s.periodicU()) {
        const Interval r = s.uRange();
        if (std::abs(b.uv.x - a.uv.x) > kPeriodStep * (r.max - r.min)) return true;
    }
    if (s.periodicV()) {
        const Interval r = s.vRange();
        if (std::abs(b.uv.y - a.uv.y) > kPeriodStep * (r.max - r.min)) return true;
    }
    return false;
}

double chord(const Vec2& a, const Vec2& b) {
    return std::hypot(a.x - b.x, a.y - b.y);
}

bool tooClose(const Vec2& a, const Vec2& b) {
    return chord(a, b) <= 1e-12 * (1.0 + std::abs(a.x) + std::abs(a.y));
}

/// The p + 1 B-spline basis functions that are non-zero at @p u, in @p span (A2.2).
std::array<double, 4> basisFunctions(int span, double u, int p, const std::vector<double>& knots) {
    std::array<double, 4> n{};
    std::array<double, 4> left{};
    std::array<double, 4> right{};
    n[0] = 1.0;
    for (int j = 1; j <= p; ++j) {
        left[static_cast<size_t>(j)] = u - knots[static_cast<size_t>(span + 1 - j)];
        right[static_cast<size_t>(j)] = knots[static_cast<size_t>(span + j)] - u;
        double saved = 0.0;
        for (int r = 0; r < j; ++r) {
            const double denom =
                right[static_cast<size_t>(r + 1)] + left[static_cast<size_t>(j - r)];
            const double temp = n[static_cast<size_t>(r)] / denom;
            n[static_cast<size_t>(r)] = saved + right[static_cast<size_t>(r + 1)] * temp;
            saved = left[static_cast<size_t>(j - r)] * temp;
        }
        n[static_cast<size_t>(j)] = saved;
    }
    return n;
}

/// A cubic (lower where there are fewer points) B-spline through the samples, before it is made
/// a Nurbs2D: its control points and knots, and the chord-length parameter of each sample.
struct Fit {
    std::vector<Vec2> ctrl;
    std::vector<double> knots;
    int degree = 0;
    std::vector<double> params;
    bool ok = false;

    /// The fit at @p s, by the steps NurbsCurve::evaluate takes for the Nurbs2D made of it: its
    /// points (x, y, 0) of weight 1 in homogeneous coordinates, through the same detail::knotSpan
    /// and detail::deBoor, so the same point by construction, without making the Nurbs2D.
    Vec2 at(double s) const {
        const int p = degree;
        const auto n = static_cast<int>(ctrl.size());
        const double t =
            std::clamp(s, knots[static_cast<size_t>(p)], knots[static_cast<size_t>(n)]);
        const int k = detail::knotSpan(knots.data(), n, p, t);
        std::array<detail::Homogeneous, 4> d{};
        for (int j = 0; j <= p; ++j) {
            const Vec2& c = ctrl[static_cast<size_t>(k - p + j)];
            d[static_cast<size_t>(j)] = {c.x * 1.0, c.y * 1.0, 0.0 * 1.0, 1.0};
        }
        detail::deBoor(d.data(), knots.data(), k, p, t);
        const auto& h = d[static_cast<size_t>(p)];
        const double wInv = 1.0 / h[3];
        return {h[0] * wInv, h[1] * wInv};
    }

    std::shared_ptr<const Nurbs2D> curve() const {
        return std::make_shared<Nurbs2D>(ctrl, std::vector<double>(ctrl.size(), 1.0), knots,
                                         degree);
    }
};

/// The cubic (lower where there are fewer points) B-spline through the sample points, at
/// chord-length parameters with averaged knots; the collocation matrix is banded, so it is
/// solved in O(n) without pivoting (it is totally positive).
Fit fitCubic(const std::vector<Sample>& samples) {
    Fit fit;
    const int n = static_cast<int>(samples.size()) - 1;
    const int p = std::min(3, n);
    fit.params.assign(samples.size(), 0.0);
    for (size_t i = 1; i < samples.size(); ++i) {
        fit.params[i] = fit.params[i - 1] + chord(samples[i].uv, samples[i - 1].uv);
    }
    const auto count = static_cast<size_t>(n) + 1;
    std::vector<double> knots(count + static_cast<size_t>(p) + 1, 0.0);
    for (int i = 0; i <= p; ++i) {
        knots[static_cast<size_t>(i)] = fit.params.front();
        knots[count + static_cast<size_t>(i)] = fit.params.back();
    }
    for (int j = 1; j <= n - p; ++j) {
        double sum = 0.0;
        for (int i = j; i < j + p; ++i) sum += fit.params[static_cast<size_t>(i)];
        knots[static_cast<size_t>(j + p)] = sum / p;
    }

    const auto width = static_cast<size_t>(2 * p + 1);
    std::vector<double> band(count * width, 0.0);
    std::vector<Vec2> rhs(count);
    auto entry = [&](int row, int col) -> double& {
        return band[static_cast<size_t>(row) * width + static_cast<size_t>(col - row + p)];
    };
    for (int i = 0; i <= n; ++i) {
        const double u = fit.params[static_cast<size_t>(i)];
        const int span = detail::knotSpan(knots.data(), n + 1, p, u);
        const std::array<double, 4> basis = basisFunctions(span, u, p, knots);
        for (int r = 0; r <= p; ++r) {
            const int col = span - p + r;
            if (col - i < -p || col - i > p) return fit;
            entry(i, col) = basis[static_cast<size_t>(r)];
        }
        rhs[static_cast<size_t>(i)] = samples[static_cast<size_t>(i)].uv;
    }
    for (int i = 0; i <= n; ++i) {
        const double pivot = entry(i, i);
        if (!(std::abs(pivot) > 1e-14)) return fit;
        for (int k = i + 1; k <= std::min(i + p, n); ++k) {
            const double f = entry(k, i) / pivot;
            if (f == 0.0) continue;
            for (int j = i; j <= std::min(i + p, n); ++j) entry(k, j) -= f * entry(i, j);
            rhs[static_cast<size_t>(k)].x -= f * rhs[static_cast<size_t>(i)].x;
            rhs[static_cast<size_t>(k)].y -= f * rhs[static_cast<size_t>(i)].y;
        }
    }
    std::vector<Vec2> ctrl(count);
    for (int i = n; i >= 0; --i) {
        Vec2 v = rhs[static_cast<size_t>(i)];
        for (int j = i + 1; j <= std::min(i + p, n); ++j) {
            v.x -= entry(i, j) * ctrl[static_cast<size_t>(j)].x;
            v.y -= entry(i, j) * ctrl[static_cast<size_t>(j)].y;
        }
        ctrl[static_cast<size_t>(i)] = Vec2(v.x / entry(i, i), v.y / entry(i, i));
    }
    for (const Vec2& v : ctrl) {
        if (!std::isfinite(v.x) || !std::isfinite(v.y)) return fit;
    }
    fit.ctrl = std::move(ctrl);
    fit.knots = std::move(knots);
    fit.degree = p;
    fit.ok = true;
    return fit;
}

/// The distance from @p p to the curve over t in [lo, hi]: a coarse scan, golden section about
/// the nearest cell, then Newton's method on (C - p) . C' to full precision (golden section
/// alone leaves a point on the curve about 1e-7 of a sample spacing away).
double distanceToCurve(const Curve3& c, const Vec3& p, double lo, double hi) {
    constexpr int kScan = 8;
    if (lo > hi) std::swap(lo, hi);
    double best = std::numeric_limits<double>::infinity();
    double bestT = lo;
    int bi = 0;
    for (int i = 0; i <= kScan; ++i) {
        const double t = lo + (hi - lo) * i / kScan;
        const double d = (c.evaluate(t) - p).length();
        if (d < best) {
            best = d;
            bestT = t;
            bi = i;
        }
    }
    double a = lo + (hi - lo) * std::max(0, bi - 1) / kScan;
    double b = lo + (hi - lo) * std::min(kScan, bi + 1) / kScan;
    constexpr double kInvPhi = 0.6180339887498949;
    double x1 = b - kInvPhi * (b - a);
    double x2 = a + kInvPhi * (b - a);
    double f1 = (c.evaluate(x1) - p).length();
    double f2 = (c.evaluate(x2) - p).length();
    for (int it = 0; it < 20; ++it) {
        if (f1 < f2) {
            b = x2;
            x2 = x1;
            f2 = f1;
            x1 = b - kInvPhi * (b - a);
            f1 = (c.evaluate(x1) - p).length();
        } else {
            a = x1;
            x1 = x2;
            f1 = f2;
            x2 = a + kInvPhi * (b - a);
            f2 = (c.evaluate(x2) - p).length();
        }
    }
    if (f1 < best) {
        best = f1;
        bestT = x1;
    }
    if (f2 < best) {
        best = f2;
        bestT = x2;
    }
    double t = bestT;
    for (int it = 0; it < 12; ++it) {
        const CurveDerivatives d = c.derivatives(t);
        const Vec3 r = d.point - p;
        const double denom = d.d1.dot(d.d1) + r.dot(d.d2);
        if (!(denom > 0.0)) break;
        const double next = std::clamp(t - r.dot(d.d1) / denom, lo, hi);
        const double dist = (c.evaluate(next) - p).length();
        if (!(dist < best)) break;
        best = dist;
        t = next;
    }
    return best;
}

/// The distance from @p p to the point of the curve that Newton's method on (C - p) . C' finds
/// from @p guess, kept in [lo, hi]: the distance to a point of the curve, so never below the
/// distance to its nearest. From the parameter that matches a point of the fit (where the fit
/// passes near the curve) it is the nearest, in a step or two, where distanceToCurve's search
/// takes some forty evaluations. It stops once the distance is within @p settle: nearer still
/// would decide nothing, and what it reads is still never below the truth.
double distanceNear(const Curve3& c, const Vec3& p, double guess, double lo, double hi,
                    double settle) {
    if (lo > hi) std::swap(lo, hi);
    double t = std::clamp(guess, lo, hi);
    // The point alone first (a curve's derivatives' point is its evaluate's, to the last bit):
    // where it settles there, as from a matched parameter it mostly does, its derivatives are not
    // needed.
    double best = (c.evaluate(t) - p).length();
    for (int it = 0; it < 8 && best > settle; ++it) {
        const CurveDerivatives d = c.derivatives(t);
        const Vec3 r = d.point - p;
        const double denom = d.d1.dot(d.d1) + r.dot(d.d2);
        if (!(denom > 0.0)) break;
        const double next = std::clamp(t - r.dot(d.d1) / denom, lo, hi);
        if (next == t) break;
        const double dist = (c.evaluate(next) - p).length();
        if (!(dist < best)) break;
        const bool still = best - dist <= 1e-9 * best;
        best = dist;
        t = next;
        if (still) break;
    }
    return best;
}

/// Points checked inside each sample interval: the interpolation error peaks between the
/// samples, and the middle thirds alone read it low.
constexpr int kChecksPerInterval = 7;
/// Points the quick look checks inside each interval, to steer refinement: a quarter, half and
/// three quarters of the way, each also one of the seven.
constexpr int kQuickChecks = 3;
/// The checks can miss the peak of the interpolation error by about 3%: what a fit strays
/// beyond the curve's own gap from the surface is counted with this much more.
constexpr double kPeakMargin = 0.03;
/// A check whose quick distance comes within this fraction of the target is searched for in
/// full as well (the nearer of the two counts): Newton from the matched parameter may find a
/// point of the curve that is not its nearest, which reads high, never low.
constexpr double kSearchAbove = 0.5;
/// Newton stops once a check is within this fraction of the target: nearer decides nothing (a
/// reader grows an edge's tolerance only past the target), and it still reads high, never low.
constexpr double kSettle = 0.01;

struct Measure {
    std::vector<double> interval;  ///< the fit's deviation inside each sample interval
    double fitDeviation = 0.0;
    double maxGap = 0.0;
    bool strict = false;
};

/// A pcurve's deviation inside each sample interval, the pcurve given by @p at at the samples'
/// parameters @p params. Strict: at seven points of each, each distance found from the matched
/// parameter and, where it nears @p target, in full, and what the pcurve strays beyond the
/// curve's own gap counted kPeakMargin more; and where the distance rises to a point among those,
/// its peak searched for between them (peakOfGap's search, each point from its matched parameter),
/// so that an interval reads no lower than the peak found in it, whatever its margin made of the
/// points; this is what a returned pcurve is measured by. Peaks within the settling distance
/// (kSettle of the target) decide nothing, and are not searched for. Otherwise a quick look at
/// three points, from the matched parameter alone, to steer refinement. A fit passes through its
/// samples, so its distance there is the samples' gap, already counted; a pcurve that does not (a
/// parameter line, which its samples may stand off by up to the tolerance) is measured at each
/// sample as well, in the interval it starts (the last sample in the last interval): where
/// @p atSamples gives a sample a bound (not below 0), by that, else checked as the points between
/// are.
template <class At>
Measure measureAlong(const Surface& s, const Curve3& c, const std::vector<Sample>& samples,
                     const std::vector<double>& params, const At& at, bool strict, double target,
                     const std::vector<double>* atSamples) {
    Measure m;
    m.strict = strict;
    m.interval.assign(samples.size() - 1, 0.0);
    for (const Sample& smp : samples) m.maxGap = std::max(m.maxGap, smp.gap);
    const int checks = strict ? kChecksPerInterval : kQuickChecks;
    // Strict, every point measured, in order along the pcurve, for the peaks between them.
    std::vector<double> checkAt;
    std::vector<double> checkGap;
    double reach = 0.0;
    if (strict) {
        checkAt.reserve(samples.size() * (kChecksPerInterval + 1));
        checkGap.reserve(samples.size() * (kChecksPerInterval + 1));
    }
    const auto around = [&samples](size_t i) {
        return std::pair<double, double>{samples[i == 0 ? 0 : i - 1].t,
                                         samples[std::min(i + 2, samples.size() - 1)].t};
    };
    for (size_t i = 0; i + 1 < samples.size(); ++i) {
        const std::pair<double, double> range = around(i);
        const double lo = range.first;
        const double hi = range.second;
        // At samples, each once: an interval's start, and the last interval's end too.
        const bool ends = atSamples != nullptr;
        const int lastK = ends && i + 2 == samples.size() ? checks + 1 : checks;
        for (int k = ends ? 0 : 1; k <= lastK; ++k) {
            if (ends && (k == 0 || k == checks + 1)) {
                const double bound = (*atSamples)[k == 0 ? i : i + 1];
                if (bound >= 0.0) {
                    m.interval[i] = std::max(m.interval[i], bound);
                    continue;
                }
            }
            const double f = static_cast<double>(k) / (checks + 1);
            const double sp = params[i] + f * (params[i + 1] - params[i]);
            const Vec2 uv = at(sp);
            const Vec3 p = s.evaluate(uv.x, uv.y);
            const double guess = samples[i].t + f * (samples[i + 1].t - samples[i].t);
            double d = distanceNear(c, p, guess, lo, hi, kSettle * target);
            if (strict && d > kSearchAbove * target) d = std::min(d, distanceToCurve(c, p, lo, hi));
            m.interval[i] = std::max(m.interval[i], d);
            if (strict) {
                checkAt.push_back(sp);
                checkGap.push_back(d);
                reach = std::max(reach, p.length());
            }
        }
        if (strict) {
            const double gap = std::max(samples[i].gap, samples[i + 1].gap);
            m.interval[i] += kPeakMargin * std::max(0.0, m.interval[i] - gap);
        }
        m.fitDeviation = std::max(m.fitDeviation, m.interval[i]);
    }
    if (!strict || checkAt.size() < 2) return m;
    const auto pointAt = [&](double sp) {
        const Vec2 uv = at(sp);
        return s.evaluate(uv.x, uv.y);
    };
    // A point between the checks, measured as they are: from its matched parameter.
    const auto matched = [&](double sp) {
        const size_t j = bracketOf(params, sp);
        const double span = params[j + 1] - params[j];
        const double f = span != 0.0 ? (sp - params[j]) / span : 0.0;
        return std::pair<size_t, double>{j, samples[j].t + f * (samples[j + 1].t - samples[j].t)};
    };
    const auto gapAt = [&](double sp) {
        const auto found = matched(sp);
        const auto range = around(found.first);
        return distanceNear(c, pointAt(sp), found.second, range.first, range.second,
                            kSettle * target);
    };
    for (const Peak& peak : searchPeaks(checkAt, checkGap, gapAt, peakRounding(reach),
                                        m.fitDeviation, kSettle * target)) {
        const size_t j = matched(peak.at).first;
        double d = peak.gap;
        if (d > kSearchAbove * target) {
            const auto range = around(j);
            d = std::min(d, distanceToCurve(c, pointAt(peak.at), range.first, range.second));
        }
        m.interval[j] = std::max(m.interval[j], d);
        m.fitDeviation = std::max(m.fitDeviation, m.interval[j]);
    }
    return m;
}

/// measureAlong for a fit: at its samples' chord-length parameters, by Fit::at.
Measure measureFit(const Surface& s, const Curve3& c, const std::vector<Sample>& samples,
                   const Fit& fit, bool strict, double target) {
    return measureAlong(
        s, c, samples, fit.params, [&fit](double sp) { return fit.at(sp); }, strict, target,
        nullptr);
}

/// A parameter line of the surface the samples keep to, and each sample's place along it.
struct ParameterLine {
    std::shared_ptr<const Line2D> curve;
    std::vector<double> params;
    /// How far each sample is off the line across the surface, to first order: its step in the
    /// parameter that stays, times the surface's speed along that parameter there.
    std::vector<double> off;
    /// That step itself, in the parameter, over the size of the parameter (1 + |it|).
    std::vector<double> step;
};

/// The line of the surface the samples keep to, if they do: each sample's v (or u) within
/// @p tolerance, across the surface, of the first sample's, while the other parameter runs one
/// way. Then the Line2D from the first sample's (u, v) to the last's along that line, and each
/// sample's fraction of the way. A rim of a surface of revolution written as a B-spline surface
/// keeps one v, a generator one u, an edge along a patch's own edge either.
std::optional<ParameterLine> parameterLineOf(const std::vector<Sample>& samples, double tolerance) {
    if (samples.size() < 2) return std::nullopt;
    const Vec2 a = samples.front().uv;
    bool oneV = true;  // u runs, v stays
    bool oneU = true;  // v runs, u stays
    for (const Sample& smp : samples) {
        if (!(std::abs(smp.uv.y - a.y) * smp.speedV <= tolerance)) oneV = false;
        if (!(std::abs(smp.uv.x - a.x) * smp.speedU <= tolerance)) oneU = false;
        if (!oneV && !oneU) return std::nullopt;
    }
    if (oneV == oneU) return std::nullopt;  // it stays at one point, or the surface is singular
    const auto run = [oneV](const Vec2& p) { return oneV ? p.x : p.y; };
    const double span = run(samples.back().uv) - run(a);
    if (!std::isfinite(span) || span == 0.0) return std::nullopt;
    ParameterLine line;
    line.params.reserve(samples.size());
    line.off.reserve(samples.size());
    line.step.reserve(samples.size());
    for (size_t i = 0; i < samples.size(); ++i) {
        if (i > 0 && !((run(samples[i].uv) - run(samples[i - 1].uv)) * span > 0.0)) {
            return std::nullopt;
        }
        line.params.push_back((run(samples[i].uv) - run(a)) / span);
        const Sample& smp = samples[i];
        const double stay = oneV ? a.y : a.x;
        const double d = std::abs((oneV ? smp.uv.y : smp.uv.x) - stay);
        line.off.push_back(d * (oneV ? smp.speedV : smp.speedU));
        line.step.push_back(d / (1.0 + std::abs(stay)));
    }
    line.params.back() = 1.0;
    const Vec2 b = oneV ? Vec2(samples.back().uv.x, a.y) : Vec2(a.x, samples.back().uv.y);
    line.curve = std::make_shared<Line2D>(a, b);
    return line;
}

/// The problem of a pcurve that strays @p deviation, beyond @p tolerance: where the curve itself
/// stands off the surface by more (@p standOff), by how much, since a fit that stops there has
/// not converged and its deviation is no measure of the gap.
std::string describe(double deviation, double tolerance, size_t samples, double standOff) {
    std::array<char, 240> buf{};
    if (standOff > tolerance) {
        std::snprintf(buf.data(), buf.size(),
                      "pcurve deviation %.3g exceeds tolerance %.3g (the curve stands %.3g off "
                      "the surface, %zu samples)",
                      deviation, tolerance, standOff, samples);
    } else {
        std::snprintf(buf.data(), buf.size(),
                      "pcurve deviation %.3g exceeds tolerance %.3g (the fit cannot follow it, "
                      "%zu samples)",
                      deviation, tolerance, samples);
    }
    return std::string(buf.data());
}

Pcurve failed(const std::string& why) {
    Pcurve pc;
    pc.deviation = std::numeric_limits<double>::infinity();
    pc.standOff = std::numeric_limits<double>::infinity();
    pc.problem = why;
    return pc;
}

/// How many equal steps of t to start from, and the curve at each where they are already known.
struct Start {
    int intervals = kInitialIntervals;
    /// The curve's derivatives at the kInitialIntervals + 1 starting parameters, when those are
    /// the start; else empty.
    std::vector<CurveDerivatives> at;
};

/// How many equal steps of t to start from: enough that the curve's tangent turns at most
/// about 0.25 rad in one (a helix of many turns gets several steps a turn, so no step spans half
/// a period of u, where the nearest representative of u would be ambiguous). The turning is
/// added up over kInitialIntervals steps, and over 512 only where those may hide some: a step
/// that turns more than 0.5 rad, whose tangent vanishes, or whose chord falls short of the way
/// its ends' speeds would carry it (a loop that comes back round between two samples turns the
/// tangent back to where it was, but not the chord).
Start initialIntervals(const Curve3& c, double t0, double t1) {
    constexpr int kPre = 512;
    constexpr double kStepTurn = 0.25;
    const auto parameter = [&](int k, int steps) {
        return t0 + (t1 - t0) * static_cast<double>(k) / steps;
    };
    std::vector<CurveDerivatives> coarse;
    coarse.reserve(static_cast<size_t>(kInitialIntervals) + 1);
    double turned = 0.0;
    bool closer = false;
    const double step = std::abs(t1 - t0) / kInitialIntervals;
    for (int k = 0; k <= kInitialIntervals; ++k) {
        coarse.push_back(c.derivatives(parameter(k, kInitialIntervals)));
        if (k == 0) continue;
        const CurveDerivatives& a = coarse[coarse.size() - 2];
        const CurveDerivatives& b = coarse.back();
        const double la = a.d1.length();
        const double lb = b.d1.length();
        if (!(la > 0.0 && lb > 0.0)) {
            closer = true;
            continue;
        }
        const double turn = std::acos(std::clamp(a.d1.dot(b.d1) / (la * lb), -1.0, 1.0));
        turned += turn;
        const double run = 0.5 * (la + lb) * step;
        if (turn > 2.0 * kStepTurn || (b.point - a.point).length() < 0.8 * run) closer = true;
    }
    if (closer) {
        turned = 0.0;
        Vec3 prev;
        for (int k = 0; k <= kPre; ++k) {
            const Vec3 d = c.derivatives(parameter(k, kPre)).d1;
            const double l = d.length();
            const double lp = prev.length();
            if (k > 0 && l > 0.0 && lp > 0.0) {
                turned += std::acos(std::clamp(d.dot(prev) / (l * lp), -1.0, 1.0));
            }
            prev = d;
        }
    }
    Start start;
    if (std::isfinite(turned)) {
        const double want =
            std::min(std::ceil(turned / kStepTurn), static_cast<double>(kMaxInitial));
        start.intervals = std::max(kInitialIntervals, static_cast<int>(want));
    }
    if (start.intervals == kInitialIntervals) start.at = std::move(coarse);
    return start;
}

/// Gives a pole or an apex at the start the first regular sample's u.
void fixSingularStart(std::vector<Sample>& samples) {
    const auto regular = std::find_if(samples.begin(), samples.end(),
                                      [](const Sample& smp) { return !smp.singular; });
    for (auto it = samples.begin(); it != regular && regular != samples.end(); ++it) {
        it->uv.x = regular->uv.x;
    }
}

/// Places every sample again in order, each from the one before it, so that u and v stay
/// continuous with the samples between them, and gives a pole or an apex at the start the first
/// regular u.
void rechain(const Inverter& inv, std::vector<Sample>& samples) {
    for (size_t i = 1; i < samples.size(); ++i) inv.place(samples[i], &samples[i - 1]);
    fixSingularStart(samples);
}

/// The most fit-and-measure rounds the general method takes.
constexpr int kMaxRounds = 64;

/// How far the curve stands off the surface: the largest of the samples' gaps, and of their peaks
/// between them (peakOfGap's search, each point between placed on the surface from the sample
/// before it). A peak that could not pass @p within (a gap within it needs nothing) is not
/// searched for.
double standOffOf(const Inverter& inv, const std::vector<Sample>& samples, double within) {
    std::vector<double> ts;
    std::vector<double> gaps;
    ts.reserve(samples.size());
    gaps.reserve(samples.size());
    double most = 0.0;
    double reach = 0.0;
    for (const Sample& smp : samples) {
        ts.push_back(smp.t);
        gaps.push_back(smp.gap);
        most = std::max(most, smp.gap);
        reach = std::max(reach, smp.point.length());
    }
    if (samples.size() < 2) return most;
    const auto gapAt = [&](double t) { return inv.at(t, &samples[bracketOf(ts, t)]).gap; };
    for (const Peak& p : searchPeaks(ts, gaps, gapAt, peakRounding(reach), 0.0, within)) {
        most = std::max(most, p.gap);
    }
    return most;
}

Pcurve generalPcurve(const Surface& s, const Curve3& c, double t0, double t1, double tolerance,
                     double limit, const Vec2* startNear) {
    const Inverter inv(s, c);
    std::vector<Sample> samples;
    samples.reserve(kMaxSamples);
    const Start start = initialIntervals(c, t0, t1);
    const int initial = start.intervals;
    // The first sample from the (u, v) it starts near, where that is given; each after it from
    // the one before.
    StartNear near;
    const StartNear* first = nullptr;
    if (startNear != nullptr) {
        near = {*startNear, tolerance};
        first = &near;
    }
    for (int k = 0; k <= initial; ++k) {
        const double t = t0 + (t1 - t0) * static_cast<double>(k) / initial;
        const Sample* prev = samples.empty() ? nullptr : &samples.back();
        const CurveDerivatives d =
            start.at.empty() ? c.derivatives(t) : start.at[static_cast<size_t>(k)];
        samples.push_back(inv.at(t, d, prev, k == 0 ? first : nullptr));
    }
    // Each was placed from the one before it, as rechain would place it again: only a pole at
    // the start is left to give its u.
    fixSingularStart(samples);

    // Subdivide where the curve or the surface turns, or a periodic parameter steps far.
    for (int pass = 0; pass < 16 && samples.size() < kMaxCriteriaSamples; ++pass) {
        std::vector<Sample> next;
        next.reserve(samples.size() * 2);
        bool split = false;
        for (size_t i = 0; i < samples.size(); ++i) {
            next.push_back(samples[i]);
            if (i + 1 < samples.size() && next.size() + samples.size() - i < kMaxCriteriaSamples &&
                needsSplit(s, samples[i], samples[i + 1])) {
                next.push_back(inv.at(0.5 * (samples[i].t + samples[i + 1].t), &samples[i]));
                split = true;
            }
        }
        samples = std::move(next);
        if (!split) break;
        // The new samples show how far the old ones really stepped: unwrap them all again.
        rechain(inv, samples);
    }

    double baseGap = 0.0;
    for (const Sample& smp : samples) {
        if (!std::isfinite(smp.uv.x) || !std::isfinite(smp.uv.y) || !std::isfinite(smp.gap)) {
            return failed(
                "pcurve deviation is unbounded: the curve cannot be inverted onto the "
                "surface");
        }
        baseGap = std::max(baseGap, smp.gap);
    }
    // What the fit is held to: the tolerance; where the curve stands off the surface by a gap
    // within the limit, its gap (no pcurve comes nearer than that).
    double target = tolerance;
    const auto allow = [&](double gap) {
        if (gap > target && gap <= limit) {
            target = std::max(target, std::min(limit, 2.0 * gap + tolerance));
        }
    };
    allow(baseGap);
    // Drop samples that sit on their neighbour in (u, v): they would make a zero chord.
    std::vector<Sample> kept;
    kept.reserve(samples.size());
    for (size_t i = 0; i < samples.size(); ++i) {
        if (!kept.empty() && tooClose(samples[i].uv, kept.back().uv)) {
            if (i + 1 == samples.size()) kept.back() = samples[i];
            continue;
        }
        kept.push_back(samples[i]);
    }
    samples = std::move(kept);

    Pcurve out;
    if (samples.size() < 2) {
        // The curve stays at one (u, v).
        out.curve = std::make_shared<Line2D>(samples.front().uv, samples.front().uv);
        out.start = 0.0;
        out.end = 1.0;
        out.deviation = baseGap;
        out.standOff = baseGap;
        if (baseGap > target) out.problem = describe(baseGap, target, 1, baseGap);
        return out;
    }

    // On a NURBS surface, samples that keep to one parameter line (a rim or a generator of a
    // surface of revolution written as a B-spline surface, an edge along a patch's own edge) are
    // tried as that line: measured as strictly as a fit, at these samples, and taken as a fit
    // would be (an error within the surface to the tolerance, a gap off it to the limit).
    // Otherwise the samples go on to be fitted.
    if (dynamic_cast<const NurbsSurface*>(&s) != nullptr) {
        if (const std::optional<ParameterLine> line = parameterLineOf(samples, tolerance)) {
            // The line does not pass through its samples: measured at them too. A sample on it
            // and on its surface to rounding (a rim exactly on its surface: its step off the
            // line a hair of its parameter, and that step and its gap each within a hundredth of
            // the target) bounds the line's distance there, as tightly as a check would find
            // it: the sample's own gap, its step off the line with 1% more for the surface's
            // speed over so small a step, and rounding (1e-12 of the point's reach). The rest,
            // a sample that stands off among them, are checked as the points between are.
            std::vector<double> atSamples(samples.size(), -1.0);
            for (size_t i = 0; i < samples.size(); ++i) {
                if (line->step[i] <= 1e-12 && line->off[i] <= kSettle * target &&
                    samples[i].gap <= kSettle * target) {
                    atSamples[i] = samples[i].gap + 1.01 * line->off[i] +
                                   1e-12 * (1.0 + samples[i].point.length());
                }
            }
            const Measure lm = measureAlong(
                s, c, samples, line->params,
                [&line](double sp) { return line->curve->evaluate(sp); }, true, target, &atSamples);
            allow(lm.maxGap);
            // The curve's stand-off at its peaks between the samples too, which no pcurve comes
            // nearer than: allowed for as the samples' gaps are, and counted in the deviation.
            const double standOff = std::max(standOffOf(inv, samples, tolerance), baseGap);
            allow(standOff);
            if (!(lm.maxGap > target) &&
                std::max({lm.fitDeviation, lm.maxGap, standOff}) <= target) {
                out.curve = line->curve;
                out.start = 0.0;
                out.end = 1.0;
                out.deviation = std::max({lm.fitDeviation, lm.maxGap, standOff});
                out.standOff = std::max(lm.maxGap, standOff);
                return out;
            }
        }
    }

    // The first fit is measured strictly (on the samples the curve's turning asked for, it is
    // often the last); refinement is steered by the quick measure, a fit that passes it is
    // measured strictly, and refined further where that finds it strays.
    Fit fit;
    Measure m;
    bool fitted = false;
    bool strict = true;
    for (int round = 0; round < kMaxRounds; ++round) {
        if (!fitted) {
            fit = fitCubic(samples);
            if (!fit.ok) return failed("pcurve fit failed: degenerate samples");
            fitted = true;
        }
        m = measureFit(s, c, samples, fit, strict, target);
        // Samples cannot cure a curve that stands off the surface itself; within the limit,
        // its gap is what the fit is held to.
        allow(m.maxGap);
        if (m.maxGap > target) break;
        if (std::max(m.fitDeviation, m.maxGap) <= target) {
            if (m.strict) break;
            strict = true;  // the quick look passes it: measure the same fit strictly
            continue;
        }
        if (samples.size() >= kMaxSamples) break;

        std::vector<std::pair<double, size_t>> bad;
        for (size_t i = 0; i < m.interval.size(); ++i) {
            if (m.interval[i] > target) bad.emplace_back(m.interval[i], i);
        }
        std::sort(bad.begin(), bad.end(), std::greater<>());
        const size_t room = kMaxSamples - samples.size();
        if (bad.size() > room) bad.resize(room);
        std::vector<std::pair<size_t, Sample>> inserts;
        inserts.reserve(bad.size());
        for (const auto& [dev, i] : bad) {
            const Sample mid = inv.at(0.5 * (samples[i].t + samples[i + 1].t), &samples[i]);
            if (!std::isfinite(mid.uv.x) || !std::isfinite(mid.uv.y)) continue;
            if (tooClose(mid.uv, samples[i].uv) || tooClose(mid.uv, samples[i + 1].uv)) continue;
            inserts.emplace_back(i, mid);
        }
        if (inserts.empty()) break;
        std::sort(inserts.begin(), inserts.end(),
                  [](const auto& a, const auto& b) { return a.first < b.first; });
        std::vector<Sample> next;
        next.reserve(samples.size() + inserts.size());
        size_t k = 0;
        for (size_t i = 0; i < samples.size(); ++i) {
            next.push_back(samples[i]);
            if (k < inserts.size() && inserts[k].first == i) next.push_back(inserts[k++].second);
        }
        samples = std::move(next);
        fitted = false;
        strict = false;
    }
    // What is returned is measured strictly, whichever way the rounds ended.
    if (!fitted) {
        fit = fitCubic(samples);
        if (!fit.ok) return failed("pcurve fit failed: degenerate samples");
        m.strict = false;
    }
    if (!m.strict) {
        m = measureFit(s, c, samples, fit, true, target);
        allow(m.maxGap);
    }

    // The curve's stand-off at its peaks between the samples too, which no pcurve comes nearer
    // than: allowed for as the samples' gaps are (the fit was held to them; a gap that peaks
    // higher between them is the curve's, not the fit's), and counted in the deviation, so that a
    // curve standing off past the limit between its samples says so.
    const double standOff = std::max({standOffOf(inv, samples, tolerance), m.maxGap, baseGap});
    allow(standOff);
    out.curve = fit.curve();
    out.start = out.curve->tMin();
    out.end = out.curve->tMax();
    out.deviation = std::max({m.fitDeviation, m.maxGap, baseGap, standOff});
    out.standOff = standOff;
    if (out.deviation > target) {
        out.problem = describe(out.deviation, target, samples.size(), out.standOff);
    }
    return out;
}

}  // namespace

Pcurve pcurveOn(const Surface& surface, const Curve3& curve, double t0, double t1, double tolerance,
                double standOffLimit, const math::Vec2* startNear) {
    if (!std::isfinite(t0) || !std::isfinite(t1) || !std::isfinite(tolerance)) {
        return failed("pcurve deviation is unbounded: the range or the tolerance is not finite");
    }
    if (!(tolerance > 0.0)) {
        return failed("pcurve deviation cannot be within a tolerance that is not positive");
    }
    // How far the curve may stand off the surface and still be taken.
    const double limit =
        std::isfinite(standOffLimit) && standOffLimit > tolerance ? standOffLimit : tolerance;
    if (const std::optional<Form> form = closedFormCandidate(surface, curve, t0, t1, tolerance)) {
        // A bounded form is taken within the tolerance: a curve that stands off its plane by
        // more is measured on the general path, which finds its gap rather than bounding it.
        const double dev = form->bound >= 0.0
                               ? form->bound
                               : measureForm(surface, curve, *form, t0, t1, tolerance);
        if (std::isfinite(dev) && dev <= (form->bound >= 0.0 ? tolerance : limit)) {
            Pcurve pc;
            pc.curve = form->curve;
            pc.start = form->start;
            pc.end = form->end;
            pc.deviation = dev;
            pc.standOff = dev;
            return pc;
        }
    }
    return generalPcurve(surface, curve, t0, t1, tolerance, limit, startNear);
}

double peakOfGap(const std::vector<double>& at, const std::vector<double>& gaps,
                 const std::function<double(double)>& gapAt, double rounding, double within) {
    double most = 0.0;
    for (size_t i = 0; i < std::min(at.size(), gaps.size()); ++i) most = std::max(most, gaps[i]);
    for (const Peak& p : searchPeaks(at, gaps, gapAt, rounding, 0.0, within)) {
        most = std::max(most, p.gap);
    }
    return most;
}

PcurveGap pcurveGap(const Surface& surface, const Curve2D& pcurve, const std::vector<double>& at,
                    const Curve3& curve, double within) {
    PcurveGap out;
    out.params.reserve(at.size());
    std::vector<double> gaps;
    gaps.reserve(at.size());
    double reach = 0.0;
    std::vector<Vec3> points;
    points.reserve(at.size());
    for (const double s : at) {
        const Vec2 uv = pcurve.evaluate(s);
        points.push_back(surface.evaluate(uv.x, uv.y));
    }
    // The pcurve and the curve need not share parameters: measured to the nearest point
    // (closestParameter; a B-spline's, its look along the curve made once for every sample).
    const auto* nurbs = dynamic_cast<const NurbsCurve*>(&curve);
    std::vector<double> nearest;
    if (nurbs != nullptr) nearest = nurbs->closestPoints(points);
    for (size_t i = 0; i < points.size(); ++i) {
        const Vec3& p = points[i];
        const double t = nurbs != nullptr ? nearest[i] : curve.closestParameter(p);
        const double d = (curve.evaluate(t) - p).length();
        out.params.push_back(t);
        gaps.push_back(d);
        out.strays = std::max(out.strays, d);
        reach = std::max(reach, p.length());
    }
    if (at.size() < 2) return out;
    // A point between two samples is measured by Newton's method from the parameter between
    // their nearest points (a closed curve's taken the short way round its period), held to the
    // curve's range where it does not close.
    const bool periodic = curve.periodic();
    const Interval range = curve.range();
    const double period = range.max - range.min;
    const bool wraps = periodic && std::isfinite(period) && period > 0.0;
    const double lo = wraps ? -std::numeric_limits<double>::infinity() : range.min;
    const double hi = wraps ? std::numeric_limits<double>::infinity() : range.max;
    const auto pointAt = [&](double s) {
        const Vec2 uv = pcurve.evaluate(s);
        return surface.evaluate(uv.x, uv.y);
    };
    const auto gapAt = [&](double s) {
        const size_t k = bracketOf(at, s);
        const double t0 = out.params[k];
        double t1 = out.params[k + 1];
        if (wraps) t1 = nearestRepresentative(t1, t0, period);
        const double span = at[k + 1] - at[k];
        const double f = span != 0.0 ? (s - at[k]) / span : 0.0;
        return distanceNear(curve, pointAt(s), t0 + f * (t1 - t0), lo, hi, 0.0);
    };
    for (const Peak& p : searchPeaks(at, gaps, gapAt, peakRounding(reach), 0.0, within)) {
        // The peak's point measured as the samples are, too: Newton's method found a point of the
        // curve, never nearer than its nearest, so the nearer of the two is the truer.
        const Vec3 q = pointAt(p.at);
        const double nearest = (curve.evaluate(curve.closestParameter(q)) - q).length();
        out.strays = std::max(out.strays, std::min(p.gap, nearest));
    }
    return out;
}

}  // namespace hz::geo
