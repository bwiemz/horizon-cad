#include "horizon/geometry/Pcurve.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdio>
#include <functional>
#include <limits>
#include <optional>
#include <utility>
#include <vector>

#include "horizon/geometry/curves/AnalyticCurves.h"
#include "horizon/geometry/curves/Circle2D.h"
#include "horizon/geometry/curves/Line2D.h"
#include "horizon/geometry/curves/Nurbs2D.h"
#include "horizon/geometry/surfaces/AnalyticSurfaces.h"
#include "horizon/geometry/surfaces/NurbsSurface.h"
#include "horizon/math/Constants.h"

namespace hz::geo {

namespace {

using math::Vec2;
using math::Vec3;

constexpr double kTwoPi = math::kTwoPi;
/// The most samples a fitted pcurve may use.
constexpr size_t kMaxSamples = 2048;
/// The most samples the curvature criteria alone may ask for, leaving the rest for refinement.
constexpr size_t kMaxCriteriaSamples = 1024;
constexpr int kInitialIntervals = 32;
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

double wrapTwoPi(double a) {
    double w = std::fmod(a, kTwoPi);
    if (w < 0.0) w += kTwoPi;
    return w;
}

double nearestRepresentative(double x, double ref, double period) {
    return x + period * std::round((ref - x) / period);
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
};

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

std::optional<Form> lineForm(const Surface& s, const Curve3& c, double t0, double t1, bool onlyU) {
    const Vec2 a = s.invert(c.evaluate(t0));
    const Vec2 b = s.invert(c.evaluate(t1));
    Form f;
    f.start = 0.0;
    f.end = 1.0;
    // A generator keeps the u of its start point (so its window is the start's, wrapped).
    f.curve = std::make_shared<Line2D>(a, onlyU ? Vec2(a.x, b.y) : b);
    return f;
}

std::optional<Form> planeForm(const Plane& s, const Curve3& c, double t0, double t1) {
    const Frame& pf = s.frame();
    const Vec3 normal = pf.x.cross(pf.y);
    if (c.kind() == CurveKind::Line) return lineForm(s, c, t0, t1, false);
    if (const auto* cc = dynamic_cast<const CircleCurve*>(&c)) {
        if (cc->frame().z.cross(normal).length() > kParallel) return std::nullopt;
        const Vec2 c2 = s.invert(cc->frame().origin);
        const Vec2 uv0 = s.invert(c.evaluate(t0));
        const double phi0 = std::atan2(uv0.y - c2.y, uv0.x - c2.x);
        const double sigma = sgn(cc->frame().z.dot(normal));
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
    return std::nullopt;
}

/// A circle on a surface of revolution: a parallel (its axis is the surface's) or, on a sphere
/// or a torus, a meridian (its plane holds the surface's axis).
std::optional<Form> circleOnRevolved(const Surface& s, const Frame& fr, const CircleCurve& cc,
                                     double t0, double t1) {
    const Vec3 n = cc.frame().z;
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
    const Vec3 ref = (sphere != nullptr ? start : cc.frame().origin) - fr.origin;
    const double scaleRef = sphere != nullptr ? sphere->radius() : torus->majorRadius();
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

std::optional<Form> closedFormCandidate(const Surface& s, const Curve3& c, double t0, double t1) {
    switch (s.kind()) {
        case SurfaceKind::Plane:
            return planeForm(static_cast<const Plane&>(s), c, t0, t1);
        case SurfaceKind::Cylinder:
        case SurfaceKind::Cone: {
            const Frame& fr = s.kind() == SurfaceKind::Cylinder
                                  ? static_cast<const Cylinder&>(s).frame()
                                  : static_cast<const Cone&>(s).frame();
            if (c.kind() == CurveKind::Line) return lineForm(s, c, t0, t1, true);
            if (const auto* cc = dynamic_cast<const CircleCurve*>(&c)) {
                if (std::abs(cc->frame().z.dot(fr.z)) < 1.0 - kParallel * kParallel) {
                    return std::nullopt;
                }
                return circleOnRevolved(s, fr, *cc, t0, t1);
            }
            return std::nullopt;
        }
        case SurfaceKind::Sphere:
        case SurfaceKind::Torus: {
            const Frame& fr = s.kind() == SurfaceKind::Sphere
                                  ? static_cast<const Sphere&>(s).frame()
                                  : static_cast<const Torus&>(s).frame();
            if (const auto* cc = dynamic_cast<const CircleCurve*>(&c)) {
                return circleOnRevolved(s, fr, *cc, t0, t1);
            }
            return std::nullopt;
        }
        case SurfaceKind::Nurbs:
            return std::nullopt;
    }
    return std::nullopt;
}

/// The largest distance between the closed form's image and the curve, along it and at its
/// ends.
double measureForm(const Surface& s, const Curve3& c, const Form& f, double t0, double t1) {
    constexpr int kChecks = 48;
    double dev = 0.0;
    for (int k = 0; k <= kChecks; ++k) {
        const double lambda = static_cast<double>(k) / kChecks;
        const Vec2 uv = f.curve->evaluate(f.start + lambda * (f.end - f.start));
        const Vec3 p = s.evaluate(uv.x, uv.y);
        double d = 0.0;
        if (f.affine) {
            d = (p - c.evaluate(t0 + lambda * (t1 - t0))).length();
        } else {
            d = (p - c.evaluate(c.closestParameter(p))).length();
        }
        dev = std::max(dev, d);
    }
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
    Vec2 uv;
    /// |S(uv) - C(t)|: how far the curve stands off the surface here.
    double gap = 0.0;
    bool singular = false;
};

class Inverter {
public:
    explicit Inverter(const Surface& s) : s_(s), nurbs_(dynamic_cast<const NurbsSurface*>(&s)) {}

    /// The (u, v) of @p p near @p hint: from the hint's, by Newton on a NURBS surface, else
    /// the nearest representative of the inverted point.
    Sample at(const Curve3& c, double t, const Sample* prev) const {
        const Vec3 p = c.evaluate(t);
        Sample out;
        out.t = t;
        Vec2 uv;
        if (nurbs_ != nullptr && prev != nullptr) {
            const auto r = nurbs_->project(p, prev->uv.x, prev->uv.y);
            uv = Vec2(r.first, r.second);
        } else {
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
        // At a pole or an apex u means nothing: take the neighbour's.
        const double du = s_.derivatives(uv.x, uv.y).du.length();
        out.singular = du <= 1e-9 * std::max(1.0, p.length());
        if (out.singular && prev != nullptr) uv.x = prev->uv.x;
        out.uv = uv;
        out.gap = (s_.evaluate(uv.x, uv.y) - p).length();
        return out;
    }

private:
    const Surface& s_;
    const NurbsSurface* nurbs_;
};

/// Whether the interval between @p a and @p b turns the curve or the surface too far, or steps
/// a periodic parameter too far, to take its straight chord on trust.
bool needsSplit(const Surface& s, const Curve3& c, const Sample& a, const Sample& b) {
    if (!(std::abs(b.t - a.t) > 1e-12 * (1.0 + std::abs(a.t)))) return false;
    const Vec3 da = c.derivatives(a.t).d1;
    const Vec3 db = c.derivatives(b.t).d1;
    const double la = da.length();
    const double lb = db.length();
    if (la > 0.0 && lb > 0.0 && da.dot(db) < kTurnCos * la * lb) return true;
    const Vec3 na = s.normal(a.uv.x, a.uv.y);
    const Vec3 nb = s.normal(b.uv.x, b.uv.y);
    if (na.length() > 0.5 && nb.length() > 0.5 && na.dot(nb) < kTurnCos) return true;
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

struct Fit {
    std::shared_ptr<const Nurbs2D> curve;
    std::vector<double> params;  ///< the chord-length parameter of each sample
};

int findSpan(int n, int p, double u, const std::vector<double>& knots) {
    if (u >= knots[static_cast<size_t>(n) + 1]) return n;
    if (u <= knots[static_cast<size_t>(p)]) return p;
    int lo = p;
    int hi = n + 1;
    int mid = (lo + hi) / 2;
    while (u < knots[static_cast<size_t>(mid)] || u >= knots[static_cast<size_t>(mid) + 1]) {
        if (u < knots[static_cast<size_t>(mid)]) {
            hi = mid;
        } else {
            lo = mid;
        }
        mid = (lo + hi) / 2;
    }
    return mid;
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
        const int span = findSpan(n, p, u, knots);
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
    fit.curve = std::make_shared<Nurbs2D>(std::move(ctrl), std::vector<double>(count, 1.0),
                                          std::move(knots), p);
    return fit;
}

/// The distance from @p p to the curve over t in [lo, hi]: a coarse scan, then golden section
/// about the nearest cell.
double distanceToCurve(const Curve3& c, const Vec3& p, double lo, double hi) {
    constexpr int kScan = 8;
    if (lo > hi) std::swap(lo, hi);
    double best = std::numeric_limits<double>::infinity();
    int bi = 0;
    for (int i = 0; i <= kScan; ++i) {
        const double d = (c.evaluate(lo + (hi - lo) * i / kScan) - p).length();
        if (d < best) {
            best = d;
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
    for (int it = 0; it < 30; ++it) {
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
    return std::min({best, f1, f2});
}

struct Measure {
    std::vector<double> interval;  ///< the fit's deviation inside each sample interval
    double fitDeviation = 0.0;
    double maxGap = 0.0;
};

Measure measureFit(const Surface& s, const Curve3& c, const std::vector<Sample>& samples,
                   const Fit& fit) {
    Measure m;
    m.interval.assign(samples.size() - 1, 0.0);
    for (const Sample& smp : samples) m.maxGap = std::max(m.maxGap, smp.gap);
    for (size_t i = 0; i + 1 < samples.size(); ++i) {
        const double lo = samples[i == 0 ? 0 : i - 1].t;
        const double hi = samples[std::min(i + 2, samples.size() - 1)].t;
        for (const double f : {1.0 / 3.0, 2.0 / 3.0}) {
            const double sp = fit.params[i] + f * (fit.params[i + 1] - fit.params[i]);
            const Vec2 uv = fit.curve->evaluate(sp);
            const Vec3 p = s.evaluate(uv.x, uv.y);
            m.interval[i] = std::max(m.interval[i], distanceToCurve(c, p, lo, hi));
        }
        m.fitDeviation = std::max(m.fitDeviation, m.interval[i]);
    }
    return m;
}

std::string describe(double deviation, double tolerance, size_t samples, bool offSurface) {
    std::array<char, 200> buf{};
    std::snprintf(
        buf.data(), buf.size(), "pcurve deviation %.3g exceeds tolerance %.3g (%s, %zu samples)",
        deviation, tolerance,
        offSurface ? "the curve stands off the surface" : "the fit cannot follow it", samples);
    return std::string(buf.data());
}

Pcurve failed(const std::string& why) {
    Pcurve pc;
    pc.problem = why;
    return pc;
}

Pcurve generalPcurve(const Surface& s, const Curve3& c, double t0, double t1, double tolerance) {
    const Inverter inv(s);
    std::vector<Sample> samples;
    samples.reserve(kMaxSamples);
    for (int k = 0; k <= kInitialIntervals; ++k) {
        const double t = t0 + (t1 - t0) * static_cast<double>(k) / kInitialIntervals;
        samples.push_back(inv.at(c, t, samples.empty() ? nullptr : &samples.back()));
    }
    // A pole or an apex at the start has no u of its own: the first regular one's.
    const auto regular = std::find_if(samples.begin(), samples.end(),
                                      [](const Sample& smp) { return !smp.singular; });
    for (auto it = samples.begin(); it != regular && regular != samples.end(); ++it) {
        it->uv.x = regular->uv.x;
    }

    // Subdivide where the curve or the surface turns, or a periodic parameter steps far.
    for (int pass = 0; pass < 16 && samples.size() < kMaxCriteriaSamples; ++pass) {
        std::vector<Sample> next;
        next.reserve(samples.size() * 2);
        bool split = false;
        for (size_t i = 0; i < samples.size(); ++i) {
            next.push_back(samples[i]);
            if (i + 1 < samples.size() && next.size() + samples.size() - i < kMaxCriteriaSamples &&
                needsSplit(s, c, samples[i], samples[i + 1])) {
                next.push_back(inv.at(c, 0.5 * (samples[i].t + samples[i + 1].t), &samples[i]));
                split = true;
            }
        }
        samples = std::move(next);
        if (!split) break;
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
        if (baseGap > tolerance) out.problem = describe(baseGap, tolerance, 1, true);
        return out;
    }

    Fit fit;
    Measure m;
    for (int round = 0; round < 64; ++round) {
        fit = fitCubic(samples);
        if (fit.curve == nullptr) return failed("pcurve fit failed: degenerate samples");
        m = measureFit(s, c, samples, fit);
        const double worst = std::max(m.fitDeviation, m.maxGap);
        // Samples cannot cure a curve that stands off the surface itself.
        if (worst <= tolerance || m.maxGap > tolerance || samples.size() >= kMaxSamples) break;

        std::vector<std::pair<double, size_t>> bad;
        for (size_t i = 0; i < m.interval.size(); ++i) {
            if (m.interval[i] > tolerance) bad.emplace_back(m.interval[i], i);
        }
        std::sort(bad.begin(), bad.end(), std::greater<>());
        const size_t room = kMaxSamples - samples.size();
        if (bad.size() > room) bad.resize(room);
        std::vector<std::pair<size_t, Sample>> inserts;
        inserts.reserve(bad.size());
        for (const auto& [dev, i] : bad) {
            const Sample mid = inv.at(c, 0.5 * (samples[i].t + samples[i + 1].t), &samples[i]);
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
    }

    out.curve = fit.curve;
    out.start = fit.curve->tMin();
    out.end = fit.curve->tMax();
    out.deviation = std::max({m.fitDeviation, m.maxGap, baseGap});
    if (out.deviation > tolerance) {
        out.problem = describe(out.deviation, tolerance, samples.size(), m.maxGap > tolerance);
    }
    return out;
}

}  // namespace

Pcurve pcurveOn(const Surface& surface, const Curve3& curve, double t0, double t1,
                double tolerance) {
    if (!std::isfinite(t0) || !std::isfinite(t1) || !std::isfinite(tolerance) ||
        !(tolerance > 0.0)) {
        return failed("pcurve deviation is unbounded: the range or the tolerance is not finite");
    }
    if (const std::optional<Form> form = closedFormCandidate(surface, curve, t0, t1)) {
        const double dev = measureForm(surface, curve, *form, t0, t1);
        if (std::isfinite(dev) && dev <= tolerance) {
            Pcurve pc;
            pc.curve = form->curve;
            pc.start = form->start;
            pc.end = form->end;
            pc.deviation = dev;
            return pc;
        }
    }
    return generalPcurve(surface, curve, t0, t1, tolerance);
}

}  // namespace hz::geo
