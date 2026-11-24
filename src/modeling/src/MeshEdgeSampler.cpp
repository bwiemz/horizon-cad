#include "MeshEdgeSampler.h"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <limits>
#include <string>
#include <utility>

#include "horizon/geometry/curves/Curve2D.h"
#include "horizon/geometry/curves/Curve3.h"
#include "horizon/geometry/surfaces/Surface.h"
#include "horizon/math/Constants.h"
#include "horizon/topology/ExactTolerance.h"
#include "horizon/topology/LoopUnwrap.h"

namespace hz::model::detail {

namespace {

constexpr int kCurveSamples = 33;     // along an edge, to find its tightest bend
constexpr int kSurfaceSamples = 9;    // a side of the (u, v) lattice the grid is spaced from
constexpr int kEdgeRounds = 30;       // halvings of an edge segment to suit its faces
constexpr int kGoldenSteps = 80;      // narrows a bracket by 0.618^80, below 1e-16
constexpr double kMarchSteps = 8.0;   // steps of the pcurve search per edge segment
constexpr double kRowSpacing = 32.0;  // rows across a surface bent one way, in its bent steps
constexpr double kMaxRows = 16.0;     // and at most this many bands between them
// Where a surface's rows differ in radius (a cone, a torus), a (u, v) diagonal between two of them
// strays sideways along the surface from its chord, by |S_uv| du dv / 4 for steps du and dv, and
// the mesher's check counts a stray beside a triangle as much as one off it. A cone's staggered
// rows reach half a step across: |S_uv| du dv is held to kConeStray times the tolerance, so they
// stray 15/16 of it. A torus's rows reach a whole step: kTorusStray, so 3/4 of it, where it adds
// to half the tolerance off the surface (its rows' own sag).
constexpr double kConeStray = 7.5;
constexpr double kTorusStray = 3.0;
// A torus's widest u step, in w, is at most kTall times its rows' spacing: wider, and Delaunay
// joins rows two apart across a gap instead of the rows either side of it (measured on R10 r3 at
// 1e-4 of its size: 27k triangles, 35k at 3, 45k unbounded).
constexpr double kTall = 1.5;
// Where a torus row is saddle-shaped (it curves one way along u and the other along v, on the
// inner side), a cell's diagonal and centroid stray less than its sides, the two curvatures taking
// from each other: its row's curve may sag this much of the tolerance, not half.
constexpr double kSaddleSag = 0.7;

std::string num(double x) {
    char buf[32];
    std::snprintf(buf, sizeof buf, "%.4g", x);
    return buf;
}

/// The angle a chord may turn through on a curve of curvature @p kappa: @p maxAngle, or less
/// where its sag, r (1 - cos(angle / 2)) = 2 r sin^2(angle / 4), would pass @p tolerance. The
/// half-angle form keeps its precision where the curvature is tiny.
double allowedTurn(double kappa, double tolerance, double maxAngle) {
    const double x = 0.5 * tolerance * kappa;
    return x < 1.0 ? std::min(maxAngle, 4.0 * std::asin(std::sqrt(x))) : maxAngle;
}

/// How many equal parameter steps from t0 to t1 keep every chord of @p c within allowedTurn,
/// read from the tightest bend met (as a real: it may be beyond any count that can be built).
double segmentsFor(const geo::Curve3& c, double t0, double t1, double tolerance, double maxAngle) {
    double perUnit = 0.0;  // the most allowed turns per unit parameter
    for (int i = 0; i < kCurveSamples; ++i) {
        const auto d = c.derivatives(t0 + (t1 - t0) * i / (kCurveSamples - 1));
        const double speed2 = d.d1.lengthSquared();
        const double bend = d.d1.cross(d.d2).length();
        if (!(speed2 > 0.0) || !(bend > 0.0)) continue;
        const double kappa = bend / (speed2 * std::sqrt(speed2));
        perUnit = std::max(perUnit, bend / speed2 / allowedTurn(kappa, tolerance, maxAngle));
    }
    return std::max(1.0, std::ceil(perUnit * std::fabs(t1 - t0)));
}

/// The parameters on @p h's pcurve at which @p s comes nearest each of @p pts (in h's
/// direction): pcurveStart and pcurveEnd at the ends, each other one searched for onward from
/// the one before, since a pcurve and its edge's curve agree in shape, not in parameter.
std::vector<double> pcurveParameters(const HalfEdge& h, const geo::Surface& s,
                                     const std::vector<Vec3>& pts) {
    const geo::Curve2D& c = *h.pcurve;
    const double s1 = h.pcurveEnd;
    std::vector<double> out(pts.size(), h.pcurveStart);
    out.back() = s1;
    const double span = s1 - h.pcurveStart;
    if (span == 0.0 || !std::isfinite(span)) return out;
    // The march visits start + span * (k / total) for whole k, not a running sum: a step under
    // half an ulp of a far-offset parameter (1e16 and a few segments) would add nothing. Nor
    // is a step worth less than a few ulps (the grid would repeat one parameter for many k), so
    // total stops where steps do.
    const double ulps = 4.0 * std::numeric_limits<double>::epsilon() *
                        std::max(std::fabs(h.pcurveStart), std::fabs(s1));
    const double total = std::clamp(kMarchSteps * static_cast<double>(pts.size() - 1), 1.0,
                                    std::max(1.0, std::floor(std::fabs(span) / ulps)));
    const auto gridAt = [&](double k) {
        return k >= total ? s1 : h.pcurveStart + span * (k / total);
    };
    for (std::size_t j = 1; j + 1 < pts.size(); ++j) {
        const auto dist = [&](double x) {
            const Vec2 q = c.evaluate(x);
            return (s.evaluate(q.x, q.y) - pts[j]).length();
        };
        // March on while the distance falls: the nearest point is then within a step of the
        // best on either side.
        double before = out[j - 1];
        double best = before;
        double nearest = dist(best);
        double after = s1;
        // On from the first grid point more than half a step past where the last one was found
        // (nearer, a rounding of the distance could end the march before it began).
        double first =
            std::clamp(std::floor((before - h.pcurveStart) / span * total), 0.0, total) + 1.0;
        if (std::fabs(gridAt(first) - before) < 0.5 * std::fabs(span) / total) first += 1.0;
        const auto steps = static_cast<std::int64_t>(total - first);
        for (std::int64_t i = 0; i <= steps; ++i) {
            const double k = first + static_cast<double>(i);
            const double next = gridAt(k);
            const double d = dist(next);
            if (d > nearest) {
                after = next;
                break;
            }
            before = best;
            best = next;
            nearest = d;
        }
        // Golden section between them.
        constexpr double kRatio = 0.6180339887498949;
        const auto at = [&](double f) { return before + (after - before) * f; };
        double lo = 0.0;
        double hi = 1.0;
        double x1 = hi - kRatio * (hi - lo);
        double x2 = lo + kRatio * (hi - lo);
        double f1 = dist(at(x1));
        double f2 = dist(at(x2));
        for (int i = 0; i < kGoldenSteps; ++i) {
            if (f1 <= f2) {
                hi = x2;
                x2 = x1;
                f2 = f1;
                x1 = hi - kRatio * (hi - lo);
                f1 = dist(at(x1));
            } else {
                lo = x1;
                x1 = x2;
                f1 = f2;
                x2 = lo + kRatio * (hi - lo);
                f2 = dist(at(x2));
            }
        }
        out[j] = at(0.5 * (lo + hi));
    }
    return out;
}

}  // namespace

void EdgeSampler::sampleEdges() {
    tol_ = topo::exactTolerance(solid_);
    for (const Face& f : solid_.faces()) {
        if (f.alive && f.exactSurface) planFace(f);
    }
    for (const Edge& e : solid_.edges()) {
        if (!e.alive || !e.halfEdge) continue;
        if (!e.halfEdge->twin) {
            edges_[&e].problem = "an edge of the face has no twin";
            continue;
        }
        sampleEdge(e, edges_[&e]);
    }
}

/// Halves the given segments of each edge, for both its faces, and suits the edge to them
/// again. The faces whose boundaries changed, in no particular order.
std::vector<const Face*> EdgeSampler::halveSegments(const std::vector<Segment>& segments) {
    std::unordered_map<const Edge*, std::vector<std::size_t>> byEdge;
    for (const auto& [e, k] : segments) byEdge[e].push_back(k);
    std::vector<const Face*> changed;
    for (auto& [e, ks] : byEdge) {
        EdgeSamples& es = edges_.at(e);
        if (!es.problem.empty()) continue;
        std::sort(ks.begin(), ks.end());
        ks.erase(std::unique(ks.begin(), ks.end()), ks.end());
        for (auto k = ks.rbegin(); k != ks.rend(); ++k) {
            if (*k + 1 >= es.t.size()) continue;
            const auto at = es.t.begin() + static_cast<std::ptrdiff_t>(*k + 1);
            es.t.insert(at, 0.5 * (es.t[*k] + es.t[*k + 1]));
        }
        suitEdge(*e, es);
        for (const HalfEdge* h : {static_cast<const HalfEdge*>(e->halfEdge),
                                  static_cast<const HalfEdge*>(e->halfEdge->twin)}) {
            if (h->face) changed.push_back(h->face);
        }
    }
    return changed;
}

/// Plans @p f's grid over its outer loop's box in (u, v), the loop laid out as boundaryOf
/// lays it out from the same samples. A face that cannot be laid out gets no plan, and
/// boundaryOf says why.
void EdgeSampler::planFace(const Face& f) {
    const geo::Surface& s = *f.exactSurface;
    std::vector<const HalfEdge*> ring;
    if (!f.outerLoop ||
        !topo::loopHalfEdges(f, *f.outerLoop, 2 * solid_.edges().size() + 1, ring)) {
        return;
    }
    std::vector<double> allowances;
    for (const HalfEdge* h : ring) {
        if (!h->pcurve || !h->edge || !h->next->edge) return;
        allowances.push_back(topo::joinAllowance(*h, tol_));
    }
    topo::LoopPolygon poly;
    if (!topo::unwrapLoop(ring, topo::kLoopSamples, s, topo::uPeriod(s), topo::vPeriod(s),
                          allowances, poly) ||
        poly.empty()) {
        return;
    }
    Plan p{poly.front().a, poly.front().a, {}};
    for (const auto& seg : poly) {
        p.lo = {std::min(p.lo.x, seg.a.x), std::min(p.lo.y, seg.a.y)};
        p.hi = {std::max(p.hi.x, seg.a.x), std::max(p.hi.y, seg.a.y)};
    }
    p.spacing = spacingOf(s, p.lo, p.hi);
    plans_[&f] = p;
}

/// How many segments a half-edge along a planned face's flat direction needs to meet each
/// of its rows; 0 for any other half-edge. Along means its pcurve keeps the bent parameter
/// exactly (a ruling of a cylinder or cone), at its ends and its middle.
double EdgeSampler::rowsAlong(const HalfEdge& h) const {
    if (!h.face || !h.pcurve) return 0.0;
    const auto it = plans_.find(h.face);
    if (it == plans_.end() || it->second.spacing.flat < 0) return 0.0;
    const Plan& p = it->second;
    const bool flatU = p.spacing.flat == 0;
    const auto bent = [&](const Vec2& q) { return flatU ? q.y : q.x; };
    const auto along = [&](const Vec2& q) { return flatU ? q.x : q.y; };
    const Vec2 a = h.pcurve->evaluate(h.pcurveStart);
    const Vec2 m = h.pcurve->evaluate(0.5 * (h.pcurveStart + h.pcurveEnd));
    const Vec2 b = h.pcurve->evaluate(h.pcurveEnd);
    if (bent(a) != bent(m) || bent(m) != bent(b)) return 0.0;
    const double span = along(p.hi) - along(p.lo);
    const double rows = std::ceil(span / (flatU ? p.spacing.du : p.spacing.dv));
    return std::max(1.0, std::round(std::fabs(along(b) - along(a)) / (span / rows)));
}

/// True when @p s's normal turns by more than maxAngle from (u, v) @p a through @p c to
/// @p b. Where the surface has no normal (a pole, an apex) nothing is measured.
bool EdgeSampler::turns(const geo::Surface& s, const Vec2& a, const Vec2& c, const Vec2& b) const {
    const Vec3 na = s.normal(a.x, a.y);
    const Vec3 nc = s.normal(c.x, c.y);
    const Vec3 nb = s.normal(b.x, b.y);
    if (na.lengthSquared() == 0.0 || nc.lengthSquared() == 0.0 || nb.lengthSquared() == 0.0) {
        return false;
    }
    return angleBetween(na, nc) + angleBetween(nc, nb) > opt_.maxAngle;
}

void EdgeSampler::sampleEdge(const Edge& e, EdgeSamples& es) {
    if (!e.exactCurve) {
        es.problem = "an edge has no exact curve";
        return;
    }
    const geo::Curve3& c = *e.exactCurve;
    const HalfEdge* fwd = e.halfEdge;
    const double cap = static_cast<double>(opt_.maxTrianglesPerFace) + 2.0;
    const bool closed = fwd->origin == fwd->twin->origin;
    const double n =
        std::max({segmentsFor(c, e.curveStart, e.curveEnd, sideTolerance(), stepAngle()),
                  closed ? 3.0 : 1.0, rowsAlong(*fwd), rowsAlong(*fwd->twin)});
    if (!(n <= cap)) {
        es.problem = "an edge needs " + num(n) + " segments, more than the triangle cap";
        return;
    }
    const auto count = static_cast<std::size_t>(n);
    for (std::size_t j = 0; j <= count; ++j) {
        es.t.push_back(e.curveStart + (e.curveEnd - e.curveStart) * (static_cast<double>(j) / n));
    }
    es.t.back() = e.curveEnd;
    suitEdge(e, es);
}

/// Halves @p es's segments until each suits both of @p e's faces, then records each
/// half-edge's pcurve parameters at the samples.
void EdgeSampler::suitEdge(const Edge& e, EdgeSamples& es) {
    const geo::Curve3& c = *e.exactCurve;
    const HalfEdge* fwd = e.halfEdge;
    const double cap = static_cast<double>(opt_.maxTrianglesPerFace) + 2.0;
    for (int round = 0;; ++round) {
        es.points.clear();
        for (const double t : es.t) es.points.push_back(c.evaluate(t));
        es.points.front() = fwd->origin->point;
        es.points.back() = fwd->twin->origin->point;
        // A segment is halved where, on either face, its (u, v) midpoint strays from its
        // chord or the surface's normal turns by more than maxAngle along it: a face cannot
        // split its boundary, so the edge must be fine enough for both.
        std::vector<bool> halve(es.t.size() - 1, false);
        bool any = false;
        for (const HalfEdge* h : {fwd, static_cast<const HalfEdge*>(fwd->twin)}) {
            if (!h->alive || !h->face || !h->face->exactSurface || !h->pcurve) continue;
            const bool forward = h == fwd;
            std::vector<Vec3> pts = es.points;
            if (!forward) std::reverse(pts.begin(), pts.end());
            const geo::Surface& s = *h->face->exactSurface;
            std::vector<double> ps = pcurveParameters(*h, s, pts);
            for (std::size_t k = 0; k + 1 < pts.size(); ++k) {
                const Vec2 a = h->pcurve->evaluate(ps[k]);
                const Vec2 b = h->pcurve->evaluate(ps[k + 1]);
                const Vec2 c = h->pcurve->evaluate(0.5 * (ps[k] + ps[k + 1]));
                const Vec2 m = (a + b) * 0.5;
                if (distanceToSegment(s.evaluate(m.x, m.y), pts[k], pts[k + 1]) > sideTolerance() ||
                    turns(s, a, c, b)) {
                    halve[forward ? k : pts.size() - 2 - k] = true;
                    any = true;
                }
            }
            params_[h] = std::move(ps);
        }
        if (!any) return;
        if (round == kEdgeRounds) {
            es.problem = "an edge cannot be sampled within the tolerance on its faces";
            return;
        }
        std::vector<double> t;
        for (std::size_t k = 0; k < es.t.size(); ++k) {
            t.push_back(es.t[k]);
            if (k < halve.size() && halve[k]) t.push_back(0.5 * (es.t[k] + es.t[k + 1]));
        }
        es.t = std::move(t);
        if (static_cast<double>(es.t.size() - 1) > cap) {
            es.problem = "an edge needs more segments than the triangle cap";
            return;
        }
    }
}

/// The u step of a torus's row at @p v over [u0, u1], its rows @p dv apart. Its own radius sets
/// it, not the largest: the row's curve sags no more than sideTolerance between steps (kSaddleSag
/// on the inner side) by its whole curvature, 1 / (R + r cos v), not the surface's along it: on
/// top of a torus the curve lies in the tangent plane, and a row edge's midpoint strays along the
/// surface beside it, which counts as much as off it. It turns no more than half stepAngle: rows of
/// different steps do not line up, and a triangle between them spans up to two. And a diagonal
/// to the next row strays no more than kTorusStray allows. Where the angle sets the steps, that
/// is no coarser than the grid's one step, and the rows stay as they were.
double EdgeSampler::rowStep(const geo::Surface& s, double u0, double u1, double v,
                            double dv) const {
    double step = kInfinity;
    for (int i = 0; i < kSurfaceSamples; ++i) {
        const auto d = s.derivatives(u0 + (u1 - u0) * i / (kSurfaceSamples - 1), v);
        const double lu = d.du.length();
        const Vec3 n = d.du.cross(d.dv);
        const double ln = n.length();
        if (!(lu > 0.0) || !(ln > 0.0)) continue;
        const double kappa = d.du.cross(d.duu).length() / (lu * lu * lu);
        const double twist = d.duv.length();
        if (kappa > 0.0) {
            const bool saddle = n.dot(d.duu) * n.dot(d.dvv) < 0.0;
            const double sag = saddle ? kSaddleSag * opt_.chordTolerance : sideTolerance();
            step = std::min(step, allowedTurn(kappa, sag, 0.5 * stepAngle()) / (kappa * lu));
        }
        if (twist > 0.0) step = std::min(step, kTorusStray * opt_.chordTolerance / (twist * dv));
    }
    return step;
}

/// How finely the surface bends over [lo, hi] in (u, v), from a lattice of samples, for grid
/// sides that sag no more than sideTolerance.
Spacing EdgeSampler::spacingOf(const geo::Surface& s, const Vec2& lo, const Vec2& hi) const {
    const double half = sideTolerance();
    Spacing sp;
    double su = 0.0;
    double sv = 0.0;
    double twist = 0.0;  // the largest |S_uv|
    for (int i = 0; i < kSurfaceSamples; ++i) {
        for (int j = 0; j < kSurfaceSamples; ++j) {
            const auto d = s.derivatives(lo.x + (hi.x - lo.x) * i / (kSurfaceSamples - 1),
                                         lo.y + (hi.y - lo.y) * j / (kSurfaceSamples - 1));
            const double lu = d.du.length();
            const double lv = d.dv.length();
            su = std::max(su, lu);
            sv = std::max(sv, lv);
            twist = std::max(twist, d.duv.length());
            const Vec3 n = d.du.cross(d.dv);
            const double ln = n.length();
            if (!(ln > 0.0)) continue;
            // Turning per unit parameter is |n . S''| / |S'|, curvature that over |S'|.
            const double bu = std::fabs(n.dot(d.duu)) / ln;
            const double bv = std::fabs(n.dot(d.dvv)) / ln;
            if (bu > 0.0) {
                sp.du = std::min(sp.du, allowedTurn(bu / (lu * lu), half, stepAngle()) * lu / bu);
            }
            if (bv > 0.0) {
                sp.dv = std::min(sp.dv, allowedTurn(bv / (lv * lv), half, stepAngle()) * lv / bv);
            }
        }
    }
    if (su > 0.0 && std::isfinite(su)) sp.su = su;
    if (sv > 0.0 && std::isfinite(sv)) sp.sv = sv;
    // A surface bent one way only (a cylinder, a cone) gets rows the other way too,
    // kRowSpacing of its bent steps apart in space, so that its triangles are not needles
    // between two far boundaries. The rows make bands of two triangles a column, as many as its
    // rims alone would make: at most kMaxRows bands, so the grid adds at most kMaxRows times the
    // triangles of the boundary alone, and no more than fit under the triangle cap; no rows
    // where even two bands would not fit. Its straight edges that way are sampled at the rows
    // (rowsAlong; a cone's, between them).
    //
    // A cone's rows are nearer: its diagonals between rows stray sideways (kConeStray), and rows
    // that are staggered, every other one half a step along, halve that. Its rows start half a row
    // in from each end: the boundary's samples do not line up with the first row's, and the
    // diagonals there reach up to a whole step across.
    const auto rows = [&](double bent, double bentScale, double flatScale, double bentSpan,
                          double flatSpan, double nearest) {
        const double columns = std::max(1.0, std::ceil(bentSpan / bent));
        const double bands = std::min(
            kMaxRows, std::floor(static_cast<double>(opt_.maxTrianglesPerFace) / (2.0 * columns)));
        // Half a band short, so that the grid's ceil(span / step) never makes one band more.
        return bands < 2.0 ? kInfinity
                           : std::max(std::min(kRowSpacing * bent * bentScale / flatScale, nearest),
                                      flatSpan / (bands - 0.5));
    };
    if (std::isfinite(sp.du) && !std::isfinite(sp.dv)) {
        sp.staggered = s.kind() == geo::SurfaceKind::Cone && twist > 0.0;
        sp.dv = rows(sp.du, sp.su, sp.sv, hi.x - lo.x, hi.y - lo.y,
                     sp.staggered ? kConeStray * opt_.chordTolerance / (twist * sp.du) : kInfinity);
        if (std::isfinite(sp.dv)) {
            sp.flat = 1;
        } else {
            sp.staggered = false;
        }
        // Square cells in w: with rows far apart in w for their steps, a point that refinement
        // puts between them joins row points many steps along, and those diagonals stray.
        if (sp.staggered) sp.su = sp.sv * sp.dv / sp.du;
    } else if (std::isfinite(sp.dv) && !std::isfinite(sp.du)) {
        sp.du = rows(sp.dv, sp.sv, sp.su, hi.y - lo.y, hi.x - lo.x, kInfinity);
        if (std::isfinite(sp.du)) sp.flat = 0;
    } else if (s.kind() == geo::SurfaceKind::Torus && std::isfinite(sp.du) &&
               std::isfinite(sp.dv)) {
        const double nv = std::ceil((hi.y - lo.y) / sp.dv);
        const double stepV = (hi.y - lo.y) / nv;
        // As many rows as the grid makes; one that is past the cap fails there.
        if (nv - 1.0 <= static_cast<double>(opt_.maxTrianglesPerFace)) {
            const auto rows = static_cast<std::size_t>(nv) - 1;
            double widest = sp.du;
            sp.rowSteps.reserve(rows);
            for (std::size_t j = 1; j <= rows; ++j) {
                const double v = lo.y + stepV * static_cast<double>(j);
                sp.rowSteps.push_back(std::max(sp.du, rowStep(s, lo.x, hi.x, v, stepV)));
                widest = std::max(widest, std::min(sp.rowSteps.back(), hi.x - lo.x));
            }
            sp.su = std::min(sp.su, kTall * sp.sv * stepV / widest);
        }
    }
    return sp;
}

}  // namespace hz::model::detail
