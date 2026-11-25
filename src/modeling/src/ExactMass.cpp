// Exact mass properties (stage 2 of the exact kernel): each exact face's share of the ten volume
// integrals and of the area, integrated on its own surface over its own trimmed region in (u, v).
//
// Coordinates are taken from a point near the solid and divided by its size, so every integrand
// is of the order of the face's area whatever the part's size and place, and one relative
// accuracy serves them all; the sums are scaled back at the end.

#include "ExactMass.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <exception>
#include <limits>
#include <set>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

#include "MassIntegrals.h"
#include "horizon/geometry/curves/Curve2D.h"
#include "horizon/geometry/curves/Line2D.h"
#include "horizon/geometry/curves/Nurbs2D.h"
#include "horizon/geometry/surfaces/NurbsSurface.h"
#include "horizon/geometry/surfaces/Surface.h"
#include "horizon/math/BoundingBox.h"
#include "horizon/math/Constants.h"
#include "horizon/math/Vec2.h"
#include "horizon/math/Vec3.h"
#include "horizon/modeling/Naming.h"
#include "horizon/topology/ExactTolerance.h"
#include "horizon/topology/HalfEdge.h"
#include "horizon/topology/LoopUnwrap.h"

namespace hz::model::detail {

using math::Vec2;
using math::Vec3;
using topo::Face;
using topo::HalfEdge;

namespace {

/// The ten volume integrands, {1, x, y, z, x², y², z², xy, yz, zx} each as the flux of a field
/// whose divergence it is, and the area's.
constexpr std::size_t kTerms = 11;
constexpr std::size_t kArea = 10;
using Terms = std::array<double, kTerms>;

/// Each face's boundary integral is taken to this, relative to the face's own size: the largest
/// integral of the absolute value of any of its integrands along its boundary.
constexpr double kRelativeAccuracy = 1e-11;

/// The budget of one face. A face from a file is untrusted, so every refinement is bounded: the
/// stretches of boundary measured (a pcurve's knot spans and halvings), the surface evaluations
/// made, and how often one stretch may be halved. Past any of them the face says its integral
/// did not settle; nothing is guessed.
constexpr std::size_t kMaxStretches = std::size_t{1} << 14;
constexpr std::size_t kMaxEvaluations = 4'000'000;
constexpr int kMaxHalvings = 48;

/// An unwrapped loop lies within one period of its surface; a parameter this many periods from
/// its loop's start is no loop of the face, and is not integrated.
constexpr double kMaxPeriods = 4.0;

/// The most points a Gauss–Legendre rule here has.
constexpr int kMaxGaussPoints = 32;

/// The points of the 15-point Gauss–Kronrod rule on [-1, 1] (QUADPACK's qk15), largest first:
/// the odd ones, and the middle, are the 7-point Gauss rule's.
constexpr std::array<double, 8> kKronrodNodes = {
    0.991455371120812639206854697526329, 0.949107912342758524526189684047851,
    0.864864423359769072789712788640926, 0.741531185599394439863864773280788,
    0.586087235467691130294144845693013, 0.405845151377397166906606412076961,
    0.207784955007898467600689403773245, 0.000000000000000000000000000000000};
constexpr std::array<double, 8> kKronrodWeights = {
    0.022935322010529224963732008058970, 0.063092092629978553290700663189204,
    0.104790010322250183839876322541518, 0.140653259715525918745189590510238,
    0.169004726639267902826583426598550, 0.190350578064785409913256402421014,
    0.204432940075298892414161999234649, 0.209482141084727828012999174891714};
constexpr std::array<double, 4> kGaussWeights = {
    0.129484966168869693270611432679082, 0.279705391489276667901467771423780,
    0.381830050505118944950369775488975, 0.417959183673469387755102040816327};

struct GaussRule {
    std::vector<double> nodes;  ///< in [-1, 1]
    std::vector<double> weights;
};

/// The n-point Gauss–Legendre rule: the roots of P_n by Newton from Tricomi's estimates.
GaussRule makeGaussRule(int n) {
    GaussRule rule;
    rule.nodes.assign(static_cast<std::size_t>(n), 0.0);
    rule.weights.assign(static_cast<std::size_t>(n), 0.0);
    for (int i = 0; i < (n + 1) / 2; ++i) {
        double x = std::cos(math::kPi * (i + 0.75) / (n + 0.5));
        double slope = 1.0;
        for (int step = 0; step < 100; ++step) {
            double p0 = 1.0;
            double p1 = x;
            for (int k = 2; k <= n; ++k) {
                const double p2 = ((2.0 * k - 1.0) * x * p1 - (k - 1.0) * p0) / k;
                p0 = p1;
                p1 = p2;
            }
            slope = n * (x * p1 - p0) / (x * x - 1.0);
            const double dx = p1 / slope;
            x -= dx;
            if (std::fabs(dx) <= 1e-16) break;
        }
        const double w = 2.0 / ((1.0 - x * x) * slope * slope);
        rule.nodes[static_cast<std::size_t>(i)] = -x;
        rule.nodes[static_cast<std::size_t>(n - 1 - i)] = x;
        rule.weights[static_cast<std::size_t>(i)] = w;
        rule.weights[static_cast<std::size_t>(n - 1 - i)] = w;
    }
    return rule;
}

/// The rules of 1 to kMaxGaussPoints points, made once (read-only after).
const GaussRule& gaussRule(int n) {
    static const std::vector<GaussRule> rules = [] {
        std::vector<GaussRule> made(kMaxGaussPoints + 1);
        for (int k = 1; k <= kMaxGaussPoints; ++k)
            made[static_cast<std::size_t>(k)] = makeGaussRule(k);
        return made;
    }();
    return rules[static_cast<std::size_t>(std::clamp(n, 1, kMaxGaussPoints))];
}

/// The widest stretch of an angle over which an n-point Gauss–Legendre rule integrates a
/// trigonometric polynomial of degree 4 to 1e-16 of it, by the rule's error term
/// h (4h)^2n (n!)^4 / ((2n + 1) ((2n)!)^3). A plane's, cylinder's, cone's, sphere's and torus's
/// integrands, taken from a point and divided by a size, are such polynomials in u: the point is
/// of degree 1 in (cos u, sin u), so its cube is of degree 3, and S_u × S_v of degree 1.
const std::array<double, kMaxGaussPoints + 1>& angleSpans() {
    static const std::array<double, kMaxGaussPoints + 1> spans = [] {
        std::array<double, 2 * kMaxGaussPoints + 2> logFactorial{};
        for (std::size_t k = 2; k < logFactorial.size(); ++k) {
            logFactorial[k] = logFactorial[k - 1] + std::log(static_cast<double>(k));
        }
        std::array<double, kMaxGaussPoints + 1> made{};
        for (int n = 1; n <= kMaxGaussPoints; ++n) {
            const auto un = static_cast<std::size_t>(n);
            const double bound = std::log(1e-16) + std::log(2.0 * n + 1.0) +
                                 3.0 * logFactorial[2 * un] - 4.0 * logFactorial[un];
            made[un] = std::exp(bound / (2.0 * n)) / 4.0;
        }
        return made;
    }();
    return spans;
}

/// The face's name, for a problem.
std::string nameOf(const Face& face) {
    return face.topoId.isValid() ? face.topoId.tag() : "face " + std::to_string(face.id);
}

std::string num(double v) {
    std::ostringstream out;
    out.precision(4);
    out << v;
    return out.str();
}

bool finite(const Vec2& p) {
    return std::isfinite(p.x) && std::isfinite(p.y);
}

/// The distinct values of @p knots, in order.
std::vector<double> distinct(const std::vector<double>& knots) {
    std::vector<double> out;
    for (double k : knots) {
        if (std::isfinite(k) && (out.empty() || k > out.back())) out.push_back(k);
    }
    return out;
}

/// The breaks of one parameter of a surface: where its integrands are not smooth (a B-spline's
/// knots), repeated every period where it wraps.
struct Breaks {
    std::vector<double> knots;  ///< distinct, in order; empty where the integrands are smooth
    double period = 0.0;        ///< 0 where the parameter does not wrap

    /// Appends the breaks strictly between @p lo and @p hi (lo < hi), in order. False when the
    /// stretch reaches more than kMaxPeriods round, or would be cut into more than @p cap. Each
    /// call costs a search of the knots and the breaks it appends, however many knots there are.
    bool between(double lo, double hi, std::size_t cap, std::vector<double>& out) const {
        if (knots.empty()) return true;
        const auto append = [&](double from, double to, double offset) {
            auto k = std::upper_bound(knots.begin(), knots.end(), from);
            for (; k != knots.end() && *k < to; ++k) {
                if (out.size() >= cap) return false;
                out.push_back(*k + offset);
            }
            return true;
        };
        if (period <= 0.0) return append(lo, hi, 0.0);
        const double base = knots.front();
        if (!((hi - lo) / period <= kMaxPeriods)) return false;
        const double first = std::floor((lo - base) / period);
        const double last = std::floor((hi - base) / period);
        // Whole turns, counted exactly: a parameter so far out that adding one turn is lost to
        // rounding is no parameter of a loop.
        if (!(std::fabs(first) < 1e9 && std::fabs(last) < 1e9)) return false;
        for (double turn = first; turn <= last; turn += 1.0) {
            const double offset = turn * period;
            // The period's end is the next one's start: each turn's knots are [base, base + P).
            const double from = std::max(lo - offset, base - 1.0);
            const double to = std::min(hi - offset, base + period);
            if (from == base - 1.0) {
                if (base + offset > lo && base + offset < hi) {
                    if (out.size() >= cap) return false;
                    out.push_back(base + offset);
                }
                if (!append(base, to, offset)) return false;
            } else if (!append(from, to, offset)) {
                return false;
            }
        }
        return true;
    }
};

/// How a face's surface is integrated in its inner direction, from the reference to a point.
struct InnerRule {
    enum class Kind {
        Polynomial,  ///< one Gauss–Legendre rule over the whole stretch, exact for the integrands
        Angle,       ///< a trigonometric polynomial: as many points as the stretch's length needs
        Spans,       ///< piecewise (a B-spline): a rule between each pair of knots
    };
    Kind kind = Kind::Polynomial;
    int points = 3;
    Breaks breaks;  ///< Spans: the knots in the inner direction
};

/// One face's surface, and the integrands it gives.
class FaceSurface {
public:
    FaceSurface(const geo::Surface& surface, const Vec3& origin, double size)
        : surface_(surface),
          nurbs_(dynamic_cast<const geo::NurbsSurface*>(&surface)),
          origin_(origin),
          inverse_(1.0 / size),
          inverse2_(1.0 / (size * size)) {}

    /// The integrands at (u, v) into @p out: the point p taken from the origin and divided by
    /// the size, the normal n = S_u × S_v divided by its square. False when one is not finite.
    bool terms(double u, double v, Terms& out) {
        ++evaluations;
        Vec3 point;
        Vec3 du;
        Vec3 dv;
        if (nurbs_ != nullptr) {
            const geo::SurfacePoint sp = nurbs_->evaluateWithDerivatives(u, v);
            point = sp.point;
            du = sp.du;
            dv = sp.dv;
        } else {
            const geo::SurfaceDerivatives d = surface_.derivatives(u, v);
            point = d.point;
            du = d.du;
            dv = d.dv;
        }
        const Vec3 p = (point - origin_) * inverse_;
        const Vec3 n = du.cross(dv) * inverse2_;
        // The fields are the polygon formulas' (Eberly's), whose divergences are the ten
        // integrands: each sum is right only over a closed boundary, so a solid of exact and
        // plain faces needs one field for each integral on every face.
        out[0] = p.x * n.x;
        out[1] = 0.5 * p.x * p.x * n.x;
        out[2] = 0.5 * p.y * p.y * n.y;
        out[3] = 0.5 * p.z * p.z * n.z;
        out[4] = p.x * p.x * p.x * n.x / 3.0;
        out[5] = p.y * p.y * p.y * n.y / 3.0;
        out[6] = p.z * p.z * p.z * n.z / 3.0;
        out[7] = 0.5 * p.x * p.x * p.y * n.x;
        out[8] = 0.5 * p.y * p.y * p.z * n.y;
        out[9] = 0.5 * p.z * p.z * p.x * n.z;
        out[kArea] = n.length();
        return std::all_of(out.begin(), out.end(), [](double x) { return std::isfinite(x); });
    }

    const geo::NurbsSurface* nurbs() const { return nurbs_; }

    std::size_t evaluations = 0;

private:
    const geo::Surface& surface_;
    const geo::NurbsSurface* nurbs_;
    Vec3 origin_;
    double inverse_;
    double inverse2_;
};

/// What one face's integration came to.
struct FaceResult {
    Terms flux{};   ///< its share of the ten volume integrals and the area, divided by sizes
    Terms error{};  ///< their estimated errors
    std::string problem;
};

/// The 15-point Gauss–Kronrod estimate of one stretch of a face's boundary integral.
struct Span {
    std::size_t stretch = 0;
    double t0 = 0.0;
    double t1 = 0.0;
    int halvings = 0;
    Terms value{};
    Terms error{};
    Terms magnitude{};  ///< the integral of the absolute value
    double worst = 0.0;
};

/// A piece of a loop's boundary in unwrapped (u, v): a pcurve moved by its shift, or a straight
/// jump between two pcurves' ends.
struct Stretch {
    const geo::Curve2D* curve = nullptr;  ///< null for a straight jump
    Vec2 shift;
    Vec2 from;  ///< a jump's ends
    Vec2 to;
    double reference = 0.0;  ///< where its loop's inner integrals start
};

/// The boundary integral of one exact face.
class FaceIntegral {
public:
    FaceIntegral(const Face& face, const Vec3& origin, double size,
                 const math::ModelTolerance& tolerance, const std::atomic<bool>* cancelled)
        : face_(face),
          surface_(*face.exactSurface, origin, size),
          tolerance_(tolerance),
          cancelled_(cancelled) {}

    FaceResult run() {
        FaceResult result;
        if (!chooseRule(result.problem) || !collectLoops(result.problem)) return result;
        integrate(result);
        return result;
    }

private:
    /// Picks the inner direction and its rule: along u, except on a cylinder or a cone, whose
    /// integrands are polynomials along the axis, and on a B-spline whose v has the lower
    /// degree.
    bool chooseRule(std::string& problem) {
        const geo::Surface& s = *face_.exactSurface;
        switch (s.kind()) {
            case geo::SurfaceKind::Plane:
                rule_.kind = InnerRule::Kind::Polynomial;  // degree 3 in each parameter
                rule_.points = 3;
                break;
            case geo::SurfaceKind::Cylinder:
            case geo::SurfaceKind::Cone:
                alongV_ = true;  // degree 3 (a cylinder) or 4 (a cone) along the axis
                rule_.kind = InnerRule::Kind::Polynomial;
                rule_.points = 3;
                break;
            case geo::SurfaceKind::Sphere:
            case geo::SurfaceKind::Torus:
                rule_.kind = InnerRule::Kind::Angle;
                break;
            case geo::SurfaceKind::Nurbs: {
                const geo::NurbsSurface* n = surface_.nurbs();
                if (n == nullptr) {
                    problem = "its B-spline surface cannot be read";
                    return false;
                }
                // Rational unless every weight is the first.
                bool rational = false;
                const double first = n->weights().empty() || n->weights().front().empty()
                                         ? 1.0
                                         : n->weights().front().front();
                for (const auto& row : n->weights()) {
                    for (double w : row) rational = rational || w != first;
                }
                alongV_ = n->degreeV() < n->degreeU();
                const int degree = alongV_ ? n->degreeV() : n->degreeU();
                // Non-rational, the integrands are of degree 5q - 1 along a direction of degree
                // q, which ceil(5q / 2) points integrate exactly; one more for the area's root,
                // and four more for a rational surface's quotients.
                rule_.kind = InnerRule::Kind::Spans;
                rule_.points =
                    std::clamp((5 * degree + 1) / 2 + 1 + (rational ? 4 : 0), 3, kMaxGaussPoints);
                rule_.breaks = breaksOf(!alongV_);
                otherBreaks_ = breaksOf(alongV_);
                break;
            }
        }
        return true;
    }

    /// A B-spline surface's knots along u (@p u) or v.
    Breaks breaksOf(bool u) const {
        const geo::NurbsSurface& n = *surface_.nurbs();
        Breaks b;
        b.knots = distinct(u ? n.knotsU() : n.knotsV());
        const geo::Interval range = u ? n.uRange() : n.vRange();
        const bool periodic = u ? n.periodicU() : n.periodicV();
        // Only the knots of the domain: the clamped ends repeat its first and last.
        std::vector<double> inside;
        for (double k : b.knots) {
            if (k >= range.min && k <= range.max) inside.push_back(k);
        }
        b.knots = std::move(inside);
        b.period = periodic ? range.max - range.min : 0.0;
        return b;
    }

    /// Every loop of the face as stretches of boundary, each loop unwrapped as the validator
    /// unwraps it and checked to run as the face's sense says.
    bool collectLoops(std::string& problem) {
        if (face_.outerLoop == nullptr || face_.outerLoop->halfEdge == nullptr) {
            problem = "it has no loop";
            return false;
        }
        std::vector<const topo::Wire*> wires{face_.outerLoop};
        wires.insert(wires.end(), face_.innerLoops.begin(), face_.innerLoops.end());
        const geo::Surface& s = *face_.exactSurface;
        const double periodU = topo::uPeriod(s);
        const double periodV = topo::vPeriod(s);
        for (std::size_t w = 0; w < wires.size(); ++w) {
            const bool outer = w == 0;
            if (wires[w] == nullptr || wires[w]->halfEdge == nullptr) {
                problem = "one of its inner loops is empty";
                return false;
            }
            std::vector<const HalfEdge*> ring;
            if (!topo::loopHalfEdges(face_, *wires[w], kMaxStretches, ring)) {
                problem = "a loop of it does not return to its start within " +
                          std::to_string(kMaxStretches) + " half-edges";
                return false;
            }
            std::vector<double> allowances;
            allowances.reserve(ring.size());
            for (const HalfEdge* h : ring) {
                if (!h->pcurve || h->edge == nullptr || h->next == nullptr ||
                    h->next->origin == nullptr || h->next->edge == nullptr) {
                    const std::string edge = h->edge != nullptr && h->edge->topoId.isValid()
                                                 ? " (along edge " + h->edge->topoId.tag() + ")"
                                                 : std::string();
                    problem = "a half-edge of its loop has no pcurve" + edge;
                    return false;
                }
                allowances.push_back(topo::joinAllowance(*h, tolerance_));
            }
            topo::LoopPolygon poly;
            if (!topo::unwrapLoop(ring, topo::kLoopSamples, s, periodU, periodV, allowances,
                                  poly)) {
                problem = "a loop of it closes only after winding round its surface";
                return false;
            }
            const double area = topo::signedArea(poly);
            const bool wantPositive = outer != face_.reversed;
            if (!(std::isfinite(area) && area != 0.0 && (area > 0.0) == wantPositive)) {
                problem = std::string(outer ? "its outer loop" : "an inner loop of it") +
                          " runs against its sense (signed area " + num(area) + " in (u, v))";
                return false;
            }
            if (outer) sense_ = area > 0.0 ? 1.0 : -1.0;
            if (!addLoop(ring, poly, problem)) return false;
        }
        return true;
    }

    /// The stretches of one unwrapped loop: each pcurve between its ends, moved by its shift,
    /// and the jump from each one's end to the next one's start where it moves the outer
    /// parameter.
    bool addLoop(const std::vector<const HalfEdge*>& ring, const topo::LoopPolygon& poly,
                 std::string& problem) {
        const std::size_t perEdge = static_cast<std::size_t>(topo::kLoopSamples - 1);
        // The loop's extent, and where its inner integrals start: its least inner parameter, so
        // that a side along that parameter line adds nothing.
        Vec2 lo = poly.front().a;
        Vec2 hi = lo;
        for (const auto& seg : poly) {
            for (const Vec2& p : {seg.a, seg.b}) {
                lo = {std::min(lo.x, p.x), std::min(lo.y, p.y)};
                hi = {std::max(hi.x, p.x), std::max(hi.y, p.y)};
            }
        }
        if (!finite(lo) || !finite(hi)) {
            problem = "a pcurve of it is not finite";
            return false;
        }
        const double reference = alongV_ ? lo.y : lo.x;
        const double outerExtent = alongV_ ? hi.x - lo.x : hi.y - lo.y;
        std::vector<Vec2> starts;
        std::vector<Vec2> ends;
        starts.reserve(ring.size());
        ends.reserve(ring.size());
        for (std::size_t k = 0; k < ring.size(); ++k) {
            const HalfEdge& h = *ring[k];
            const Vec2 shift = poly[k * perEdge].shift;
            starts.push_back(h.pcurve->evaluate(h.pcurveStart) + shift);
            ends.push_back(h.pcurve->evaluate(h.pcurveEnd) + shift);
            if (!finite(starts.back()) || !finite(ends.back()) || !std::isfinite(h.pcurveStart) ||
                !std::isfinite(h.pcurveEnd)) {
                problem = "a pcurve of it is not finite";
                return false;
            }
            Stretch stretch;
            stretch.curve = h.pcurve.get();
            stretch.shift = shift;
            stretch.reference = reference;
            if (!addSpans(stretch, h.pcurveStart, h.pcurveEnd, problem)) return false;
        }
        for (std::size_t k = 0; k < ring.size(); ++k) {
            const Vec2 from = ends[k];
            const Vec2 to = starts[(k + 1) % ring.size()];
            // A jump that moves the outer parameter by rounding adds nothing a face can tell.
            const double moves = alongV_ ? to.x - from.x : to.y - from.y;
            if (!(std::fabs(moves) > 1e-13 * outerExtent)) continue;
            Stretch jump;
            jump.from = from;
            jump.to = to;
            jump.reference = reference;
            if (!addSpans(jump, 0.0, 1.0, problem)) return false;
        }
        return true;
    }

    /// The point of @p s at @p t and its derivative in t.
    static void at(const Stretch& s, double t, Vec2& uv, Vec2& d) {
        if (s.curve != nullptr) {
            uv = s.curve->evaluate(t) + s.shift;
            d = s.curve->derivative(t, 1);
        } else {
            uv = s.from + (s.to - s.from) * t;
            d = s.to - s.from;
        }
    }

    /// Adds @p stretch from @p t0 to @p t1 (either way round) as its first spans: cut at a
    /// B-spline pcurve's knots, and a straight one where it crosses the surface's knots, where
    /// the integrand is not smooth.
    bool addSpans(const Stretch& stretch, double t0, double t1, std::string& problem) {
        stretches_.push_back(stretch);
        const std::size_t index = stretches_.size() - 1;
        if (t0 == t1) return true;
        const double lo = std::min(t0, t1);
        const double hi = std::max(t0, t1);
        std::vector<double> cuts;
        const auto* line = dynamic_cast<const geo::Line2D*>(stretch.curve);
        if (const auto* spline = dynamic_cast<const geo::Nurbs2D*>(stretch.curve)) {
            for (double k : distinct(spline->curve().knots())) {
                if (k > lo && k < hi) cuts.push_back(k);
            }
        } else if ((stretch.curve == nullptr || line != nullptr) && surface_.nurbs() != nullptr) {
            Vec2 a;
            Vec2 b;
            Vec2 d;
            at(stretch, lo, a, d);
            at(stretch, hi, b, d);
            const Breaks& uBreaks = alongV_ ? otherBreaks_ : rule_.breaks;
            const Breaks& vBreaks = alongV_ ? rule_.breaks : otherBreaks_;
            const auto crossings = [&](double from, double to, const Breaks& breaks) {
                std::vector<double> values;
                if (from == to) return true;
                if (!breaks.between(std::min(from, to), std::max(from, to), kMaxStretches,
                                    values)) {
                    return false;
                }
                for (double v : values) cuts.push_back(lo + (hi - lo) * (v - from) / (to - from));
                return true;
            };
            if (!crossings(a.x, b.x, uBreaks) || !crossings(a.y, b.y, vBreaks)) {
                problem = "a stretch of its boundary crosses more than " +
                          std::to_string(kMaxStretches) +
                          " knots of its surface, or lies more than " + num(kMaxPeriods) +
                          " periods round it";
                return false;
            }
        }
        std::sort(cuts.begin(), cuts.end());
        if (pending_.size() + cuts.size() + 1 > kMaxStretches) {
            problem = "its integral did not settle within " + std::to_string(kMaxStretches) +
                      " stretches of its boundary";
            return false;
        }
        double from = lo;
        for (double cut : cuts) {
            if (cut > from && cut < hi) {
                pending_.push_back({index, from, cut, t1 < t0});
                from = cut;
            }
        }
        pending_.push_back({index, from, hi, t1 < t0});
        return true;
    }

    /// The integral from the loop's reference to (u, v) along the inner direction, into @p out.
    bool inner(const Vec2& uv, double reference, Terms& out) {
        out.fill(0.0);
        const double to = alongV_ ? uv.y : uv.x;
        const double across = alongV_ ? uv.x : uv.y;
        if (to == reference) return true;
        const double lo = std::min(reference, to);
        const double hi = std::max(reference, to);
        const double sign = to < reference ? -1.0 : 1.0;
        bool ok = true;
        const auto add = [&](double a, double b, int points) {
            if (!ok) return;
            if (surface_.evaluations >= kMaxEvaluations) {
                why_ = overBudget();
                ok = false;
                return;
            }
            const GaussRule& rule = gaussRule(points);
            const double half = 0.5 * (b - a);
            const double mid = 0.5 * (a + b);
            Terms t{};
            for (std::size_t i = 0; i < rule.nodes.size(); ++i) {
                const double s = mid + half * rule.nodes[i];
                if (!(alongV_ ? surface_.terms(across, s, t) : surface_.terms(s, across, t))) {
                    why_ = notFiniteAt(alongV_ ? Vec2(across, s) : Vec2(s, across));
                    ok = false;
                    return;
                }
                for (std::size_t k = 0; k < kTerms; ++k) {
                    out[k] += sign * half * rule.weights[i] * t[k];
                }
            }
        };
        const auto tooFar = [&] {
            why_ = "a stretch of its boundary lies more than " + num(kMaxPeriods) +
                   " periods round its surface";
            return false;
        };
        switch (rule_.kind) {
            case InnerRule::Kind::Polynomial:
                add(lo, hi, rule_.points);
                break;
            case InnerRule::Kind::Angle: {
                const double length = hi - lo;
                if (!(length <= kMaxPeriods * math::kTwoPi)) return tooFar();
                const auto& spans = angleSpans();
                const int pieces = static_cast<int>(std::ceil(length / spans[kMaxGaussPoints]));
                const double piece = length / pieces;
                int points = 1;
                while (points < kMaxGaussPoints &&
                       spans[static_cast<std::size_t>(points)] < piece) {
                    ++points;
                }
                for (int i = 0; i < pieces && ok; ++i) {
                    add(lo + piece * i, i + 1 == pieces ? hi : lo + piece * (i + 1), points);
                }
                break;
            }
            case InnerRule::Kind::Spans: {
                breaks_.clear();
                if (!rule_.breaks.between(lo, hi, kMaxStretches, breaks_)) return tooFar();
                double from = lo;
                for (double b : breaks_) {
                    add(from, b, rule_.points);
                    from = b;
                }
                add(from, hi, rule_.points);
                break;
            }
        }
        return ok;
    }

    /// The 15-point Gauss–Kronrod estimate of span @p s, with QUADPACK's error estimate.
    bool measure(Span& s) {
        const Stretch& stretch = stretches_[s.stretch];
        const double half = 0.5 * (s.t1 - s.t0);
        const double centre = 0.5 * (s.t0 + s.t1);
        std::array<Terms, 15> f{};
        for (std::size_t j = 0; j < 15; ++j) {
            const double x = j < 8 ? -kKronrodNodes[j] : kKronrodNodes[14 - j];
            const double t = centre + half * x;
            Vec2 uv;
            Vec2 d;
            at(stretch, t, uv, d);
            const double multiplier = alongV_ ? -d.x : d.y;
            if (!finite(uv) || !std::isfinite(multiplier)) {
                why_ = notFiniteAt(uv);
                return false;
            }
            f[j].fill(0.0);
            if (multiplier == 0.0) continue;
            Terms g{};
            if (!inner(uv, stretch.reference, g)) return false;
            for (std::size_t k = 0; k < kTerms; ++k) f[j][k] = g[k] * multiplier;
        }
        // f[j] at -x_j for j < 8 (j = 7 the middle), at +x_(14-j) for j > 7.
        s.worst = 0.0;
        for (std::size_t k = 0; k < kTerms; ++k) {
            const double fc = f[7][k];
            double kronrod = fc * kKronrodWeights[7];
            double gauss = fc * kGaussWeights[3];
            double absolute = std::fabs(kronrod);
            for (std::size_t j = 0; j < 7; ++j) {
                const double a = f[j][k];
                const double b = f[14 - j][k];
                kronrod += kKronrodWeights[j] * (a + b);
                absolute += kKronrodWeights[j] * (std::fabs(a) + std::fabs(b));
                if (j % 2 == 1) gauss += kGaussWeights[j / 2] * (a + b);
            }
            const double mean = 0.5 * kronrod;
            double spread = kKronrodWeights[7] * std::fabs(fc - mean);
            for (std::size_t j = 0; j < 7; ++j) {
                spread += kKronrodWeights[j] *
                          (std::fabs(f[j][k] - mean) + std::fabs(f[14 - j][k] - mean));
            }
            const double width = std::fabs(half);
            double error = std::fabs((kronrod - gauss) * half);
            spread *= width;
            absolute *= width;
            if (spread != 0.0 && error != 0.0) {
                error = spread * std::min(1.0, std::pow(200.0 * error / spread, 1.5));
            }
            error = std::max(error, 50.0 * std::numeric_limits<double>::epsilon() * absolute);
            s.value[k] = kronrod * half;
            s.error[k] = error;
            s.magnitude[k] = absolute;
            s.worst = std::max(s.worst, error);
        }
        if (!std::isfinite(s.worst)) {
            why_ = "its integral is not finite";
            return false;
        }
        return true;
    }

    std::string notFiniteAt(const Vec2& uv) const {
        return "its surface or a pcurve is not finite at (u, v) = (" + num(uv.x) + ", " +
               num(uv.y) + ")";
    }

    static std::string overBudget() {
        return "its integral did not settle within " + std::to_string(kMaxEvaluations) +
               " evaluations of its surface";
    }

    bool stopped() const {
        return cancelled_ != nullptr && cancelled_->load(std::memory_order_relaxed);
    }

    /// The boundary integral, refined where its error is largest until it is within
    /// kRelativeAccuracy of the face's size or a budget is spent.
    void integrate(FaceResult& result) {
        const auto worse = [](const Span& a, const Span& b) { return a.worst < b.worst; };
        Terms value{};
        Terms error{};
        Terms magnitude{};
        const auto take = [&](Span& s) {
            for (std::size_t k = 0; k < kTerms; ++k) {
                value[k] += s.value[k];
                error[k] += s.error[k];
                magnitude[k] += s.magnitude[k];
            }
            spans_.push_back(s);
            std::push_heap(spans_.begin(), spans_.end(), worse);
        };
        const auto failed = [&] { result.problem = why_; };
        for (std::size_t i = 0; i < pending_.size(); ++i) {
            if ((i & 63U) == 63U && stopped()) {
                result.problem = "stopped";
                return;
            }
            const Pending& p = pending_[i];
            Span s;
            s.stretch = p.stretch;
            s.t0 = p.reversed ? p.t1 : p.t0;
            s.t1 = p.reversed ? p.t0 : p.t1;
            if (!measure(s)) return failed();
            take(s);
        }
        const auto settled = [&] {
            const double size = *std::max_element(magnitude.begin(), magnitude.end());
            const double worst = *std::max_element(error.begin(), error.end());
            return worst <= kRelativeAccuracy * size;
        };
        std::size_t rounds = 0;
        while (!settled()) {
            if ((++rounds & 63U) == 0 && stopped()) {
                result.problem = "stopped";
                return;
            }
            const auto unsettled = [&](const std::string& why) {
                const double size = *std::max_element(magnitude.begin(), magnitude.end());
                const double worst = *std::max_element(error.begin(), error.end());
                result.problem = "its integral did not settle " + why + " (estimated error " +
                                 num(worst / size) + " of its size)";
            };
            if (spans_.size() >= kMaxStretches) {
                return unsettled("within " + std::to_string(kMaxStretches) +
                                 " stretches of its boundary");
            }
            std::pop_heap(spans_.begin(), spans_.end(), worse);
            const Span worst = spans_.back();
            if (worst.halvings >= kMaxHalvings) {
                return unsettled("with a stretch of its boundary halved " +
                                 std::to_string(kMaxHalvings) + " times");
            }
            spans_.pop_back();
            for (std::size_t k = 0; k < kTerms; ++k) {
                value[k] -= worst.value[k];
                error[k] -= worst.error[k];
                magnitude[k] -= worst.magnitude[k];
            }
            const double mid = 0.5 * (worst.t0 + worst.t1);
            Span left;
            left.stretch = worst.stretch;
            left.t0 = worst.t0;
            left.t1 = mid;
            left.halvings = worst.halvings + 1;
            Span right = left;
            right.t0 = mid;
            right.t1 = worst.t1;
            if (!measure(left) || !measure(right)) return failed();
            take(left);
            take(right);
        }
        // The sums afresh, free of what taking spans out and in left of rounding.
        result.flux.fill(0.0);
        result.error.fill(0.0);
        for (const Span& s : spans_) {
            for (std::size_t k = 0; k < kTerms; ++k) {
                result.flux[k] += s.value[k];
                result.error[k] += s.error[k];
            }
        }
        // The loops run as the face's sense says (checked), so the line integrals are the
        // region's integrals signed by `sense_`: the volume terms' flux is signed by `reversed`
        // as well, which is the same sign, and the area is the region's.
        result.flux[kArea] *= sense_;
    }

    struct Pending {
        std::size_t stretch;
        double t0;
        double t1;
        bool reversed;  ///< the stretch runs from t1 to t0
    };

    const Face& face_;
    FaceSurface surface_;
    const math::ModelTolerance& tolerance_;
    const std::atomic<bool>* cancelled_;
    InnerRule rule_;
    Breaks otherBreaks_;  ///< a B-spline's knots in the outer direction
    bool alongV_ = false;
    double sense_ = 1.0;
    std::vector<Stretch> stretches_;
    std::vector<Pending> pending_;
    std::vector<Span> spans_;
    std::vector<double> breaks_;
    std::string why_;  ///< why measure() or inner() failed
};

/// The centre of the box round the solid's live vertices: the point moments are taken from.
Vec3 middleOf(const topo::Solid& solid) {
    math::BoundingBox box;
    for (const auto& v : solid.vertices()) {
        if (v.alive && std::isfinite(v.point.x) && std::isfinite(v.point.y) &&
            std::isfinite(v.point.z)) {
            box.expand(v.point);
        }
    }
    return box.isValid() ? box.center() : Vec3{};
}

/// The powers of the size each integral scales by (volume 3, first moments 4, second 5, area
/// 2), and the factors Measures::sums hold them by (finish() divides them out).
constexpr std::array<int, kTerms> kPowers = {3, 4, 4, 4, 5, 5, 5, 5, 5, 5, 2};
constexpr std::array<double, 10> kSumFactors = {6, 24, 24, 24, 60, 60, 60, 120, 120, 120};

ExactMeasure notValid(double density, std::string problem) {
    ExactMeasure out;
    out.properties.density = density;
    out.properties.problem = std::move(problem);
    return out;
}

ExactMeasure measure(const topo::Solid& solid, const Material* material,
                     const std::atomic<bool>* cancelled) {
    const double density = material ? material->density : 1.0;
    const double size = topo::exactExtent(solid);
    if (!(size > 0.0) || !std::isfinite(size)) {
        return notValid(density, "the solid has no size to measure");
    }
    const Vec3 origin = middleOf(solid);
    const math::ModelTolerance tolerance = topo::exactTolerance(solid);

    Measures measures;
    measures.origin = origin;
    double volumeError = 0.0;
    double areaError = 0.0;
    std::set<std::string> plain;
    for (const auto& face : solid.faces()) {
        if (cancelled != nullptr && cancelled->load(std::memory_order_relaxed)) {
            return notValid(density, "stopped");
        }
        if (!face.alive) continue;
        if (!face.exactSurface) {
            // As modelled: the polygon formulas, about the same origin.
            for (const auto& tri : faceTriangles(face)) {
                measures.add(tri[0] - origin, tri[1] - origin, tri[2] - origin);
            }
            plain.insert(face.topoId.isValid() ? logicalFace(face.topoId.tag()) : nameOf(face));
            continue;
        }
        FaceIntegral integral(face, origin, size, tolerance, cancelled);
        const FaceResult r = integral.run();
        if (!r.problem.empty()) return notValid(density, nameOf(face) + ": " + r.problem);
        for (std::size_t k = 0; k < 10; ++k) {
            measures.sums[k] += r.flux[k] * std::pow(size, kPowers[k]) * kSumFactors[k];
        }
        measures.area += r.flux[kArea] * size * size;
        volumeError += r.error[0] * size * size * size;
        areaError += r.error[kArea] * size * size;
    }

    ExactMeasure out;
    out.plainFaces.assign(plain.begin(), plain.end());
    if (!std::isfinite(measures.sums[0])) return notValid(density, "its volume is not finite");
    // Each face is signed by its own sense, so a void subtracts; the solid as a whole may face
    // in (the kernel's box is wound so, and read back from STEP it is exact so), and finish()
    // normalises that sign as it does the polygon formulas'.
    if (measures.sums[0] == 0.0) return notValid(density, "its faces enclose no volume");
    out.properties = finish(measures, density);
    if (!out.properties.valid) {
        out.properties.problem = "its faces enclose no volume";
        return out;
    }
    out.estimatedError =
        std::max(volumeError / out.properties.volume,
                 out.properties.surfaceArea > 0.0 ? areaError / out.properties.surfaceArea : 0.0);
    return out;
}

}  // namespace

ExactMeasure measureExact(const topo::Solid& solid, const Material* material,
                          const std::atomic<bool>* cancelled) {
    try {
        return measure(solid, material, cancelled);
    } catch (const std::exception& e) {
        return notValid(material ? material->density : 1.0,
                        std::string("the solid could not be measured: ") + e.what());
    }
}

}  // namespace hz::model::detail
