#include "horizon/modeling/TrimmedFaceMesher.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <limits>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <utility>

#include "horizon/geometry/Triangulate2D.h"
#include "horizon/geometry/curves/Curve2D.h"
#include "horizon/geometry/curves/Curve3.h"
#include "horizon/geometry/surfaces/Surface.h"
#include "horizon/math/Constants.h"
#include "horizon/math/ModelTolerance.h"
#include "horizon/math/Vec2.h"
#include "horizon/topology/LoopUnwrap.h"

namespace hz::model {

namespace {

using math::Vec2;
using math::Vec3;
using topo::Edge;
using topo::Face;
using topo::HalfEdge;

constexpr double kInfinity = std::numeric_limits<double>::infinity();
constexpr int kCurveSamples = 33;    // along an edge, to find its tightest bend
constexpr int kSurfaceSamples = 9;   // a side of the (u, v) lattice the grid is spaced from
constexpr int kEdgeRounds = 30;      // halvings of an edge segment to suit its faces
constexpr int kRefineRounds = 64;    // rounds of refinement
constexpr int kGoldenSteps = 80;     // narrows a bracket by 0.618^80, below 1e-16
constexpr double kMarchSteps = 8.0;  // steps of the pcurve search per edge segment
constexpr double kKeepAway = 0.5;    // grid points nearer the boundary (in spacings) go
constexpr std::uint32_t kUnused = std::numeric_limits<std::uint32_t>::max();
constexpr int kFits = -1;     // a triangle that needs no point
constexpr int kCentroid = 3;  // one that needs its centroid; 0 to 2 name an edge to split

std::string num(double x) {
    char buf[32];
    std::snprintf(buf, sizeof buf, "%.4g", x);
    return buf;
}

std::uint64_t edgeKey(std::uint32_t a, std::uint32_t b) {
    return (static_cast<std::uint64_t>(std::min(a, b)) << 32) | std::max(a, b);
}

template <typename V>  // Vec2 or Vec3
double distanceToSegment(const V& p, const V& a, const V& b) {
    const V ab = b - a;
    const double len2 = ab.lengthSquared();
    const double t = len2 > 0.0 ? std::clamp((p - a).dot(ab) / len2, 0.0, 1.0) : 0.0;
    return (p - (a + ab * t)).length();
}

/// The distance from @p p to the triangle abc. Written with cross products: the usual
/// barycentric form multiplies dot products together, and on a long thin triangle (a sliver
/// face's) that loses most of the precision.
double distanceToTriangle(const Vec3& p, const Vec3& a, const Vec3& b, const Vec3& c) {
    const Vec3 n = (b - a).cross(c - a);
    const double ln = n.length();
    if (ln > 0.0 && (b - a).cross(p - a).dot(n) >= 0.0 && (c - b).cross(p - b).dot(n) >= 0.0 &&
        (a - c).cross(p - c).dot(n) >= 0.0) {
        return std::fabs((p - a).dot(n)) / ln;
    }
    return std::min(
        {distanceToSegment(p, a, b), distanceToSegment(p, b, c), distanceToSegment(p, c, a)});
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
    const double step = (s1 - h.pcurveStart) / (kMarchSteps * static_cast<double>(pts.size() - 1));
    if (step == 0.0) return out;
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
        while (best != s1) {
            const double next = (s1 - best) / step <= 1.0 ? s1 : best + step;
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

/// An edge discretised once for both its faces.
struct EdgeSamples {
    std::vector<double> t;     // curve parameters, curveStart to curveEnd
    std::vector<Vec3> points;  // the curve at t; at the ends, the vertices' own points
    std::string problem;
};

/// How the face's (u, v) is scaled for triangulating, and the grid steps its bending allows.
struct Spacing {
    double su = 1.0;  // w = (u su, v sv): near lengths on the surface, for Delaunay to judge
    double sv = 1.0;
    double du = kInfinity;
    double dv = kInfinity;
};

/// A face's loops (outer first) in unwrapped (u, v), with each sample's position.
struct Boundary {
    std::vector<std::vector<Vec2>> uv;
    std::vector<std::vector<Vec3>> xyz;
};

class Mesher {
public:
    Mesher(const topo::Solid& solid, const MeshOptions& options) : solid_(solid), opt_(options) {}

    void sampleEdges() {
        double lo[3] = {kInfinity, kInfinity, kInfinity};
        double hi[3] = {-kInfinity, -kInfinity, -kInfinity};
        for (const Edge& e : solid_.edges()) {
            if (!e.alive || !e.halfEdge || !e.halfEdge->twin) continue;
            sampleEdge(e, edges_[&e]);
            for (const Vec3& p : edges_[&e].points) {
                const double xyz[3] = {p.x, p.y, p.z};
                for (int k = 0; k < 3; ++k) {
                    lo[k] = std::min(lo[k], xyz[k]);
                    hi[k] = std::max(hi[k], xyz[k]);
                }
            }
        }
        const double extent = std::hypot(hi[0] - lo[0], hi[1] - lo[1], hi[2] - lo[2]);
        tol_ = math::ModelTolerance::forExtent(extent);
    }

    FaceMesh meshFace(const Face& f) const {
        FaceMesh out;
        out.face = &f;
        if (!f.exactSurface) {
            out.problem = "the face has no exact surface";
            return out;
        }
        Boundary b;
        out.problem = boundaryOf(f, b);
        if (out.problem.empty()) out.problem = triangulate(f, b, out);
        if (!out.problem.empty()) out = FaceMesh{&f, {}, {}, {}, out.problem};
        return out;
    }

private:
    /// How far one direction may sag: an edge's chords (also as their (u, v) midpoints map onto
    /// each face), a grid cell's sides. A triangle's error combines two directions, and near an
    /// edge its own; with each held to half the tolerance, refinement at a boundary converges
    /// (a triangle on a boundary chord errs by at most 8/9 of the chord's sag there).
    double sideTolerance() const { return 0.5 * opt_.chordTolerance; }

    void sampleEdge(const Edge& e, EdgeSamples& es) {
        if (!e.exactCurve) {
            es.problem = "an edge has no exact curve";
            return;
        }
        const geo::Curve3& c = *e.exactCurve;
        const HalfEdge* fwd = e.halfEdge;
        const double cap = static_cast<double>(opt_.maxTrianglesPerFace) + 2.0;
        const bool closed = fwd->origin == fwd->twin->origin;
        const double n =
            std::max(segmentsFor(c, e.curveStart, e.curveEnd, sideTolerance(), opt_.maxAngle),
                     closed ? 3.0 : 1.0);
        if (!(n <= cap)) {
            es.problem = "an edge needs " + num(n) + " segments, more than the triangle cap";
            return;
        }
        const auto count = static_cast<std::size_t>(n);
        for (std::size_t j = 0; j <= count; ++j) {
            es.t.push_back(e.curveStart +
                           (e.curveEnd - e.curveStart) * (static_cast<double>(j) / n));
        }
        es.t.back() = e.curveEnd;
        for (int round = 0;; ++round) {
            es.points.clear();
            for (const double t : es.t) es.points.push_back(c.evaluate(t));
            es.points.front() = fwd->origin->point;
            es.points.back() = fwd->twin->origin->point;
            // A segment whose (u, v) midpoint on either face strays from its chord is halved:
            // a face cannot split its boundary, so the edge must be fine enough for both.
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
                    const Vec2 m =
                        (h->pcurve->evaluate(ps[k]) + h->pcurve->evaluate(ps[k + 1])) * 0.5;
                    if (distanceToSegment(s.evaluate(m.x, m.y), pts[k], pts[k + 1]) >
                        sideTolerance()) {
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

    /// The face's loops in (u, v), unwrapped and aligned as the validator does it, each sample
    /// at its pcurve parameter; empty, or why the face cannot be laid out.
    std::string boundaryOf(const Face& f, Boundary& b) const {
        const geo::Surface& s = *f.exactSurface;
        const double pu = topo::uPeriod(s);
        const double pv = topo::vPeriod(s);
        std::vector<const topo::Wire*> wires{f.outerLoop};
        wires.insert(wires.end(), f.innerLoops.begin(), f.innerLoops.end());
        topo::LoopPolygon outer;
        for (std::size_t w = 0; w < wires.size(); ++w) {
            std::vector<const HalfEdge*> ring;
            if (!wires[w] ||
                !topo::loopHalfEdges(f, *wires[w], 2 * solid_.edges().size() + 1, ring)) {
                return "a loop of the face does not close";
            }
            std::vector<double> allowances;
            for (const HalfEdge* h : ring) {
                const auto es = edges_.find(h->edge);
                if (es == edges_.end() || !h->pcurve) {
                    return "a half-edge of the face has no pcurve or no edge";
                }
                if (!es->second.problem.empty()) return es->second.problem;
                if (params_.count(h) == 0) return "a half-edge of the face was not sampled";
                allowances.push_back(topo::joinAllowance(*h, tol_));
            }
            topo::LoopPolygon poly;
            if (!topo::unwrapLoop(ring, topo::kLoopSamples, s, pu, pv, allowances, poly)) {
                return "a loop of the face closes only after winding round the surface";
            }
            if (w == 0) {
                outer = poly;
            } else {
                topo::alignLoop(poly, outer, pu, pv);
            }
            std::vector<Vec2> uv;
            std::vector<Vec3> xyz;
            for (std::size_t k = 0; k < ring.size(); ++k) {
                const HalfEdge* h = ring[k];
                const Vec2 shift = poly[k * (topo::kLoopSamples - 1)].shift;
                const std::vector<Vec3>& pts = edges_.at(h->edge).points;
                const std::vector<double>& ps = params_.at(h);
                const std::size_t n = pts.size() - 1;
                for (std::size_t j = 0; j < n; ++j) {  // the last is the next one's first
                    uv.push_back(h->pcurve->evaluate(ps[j]) + shift);
                    xyz.push_back(pts[h == h->edge->halfEdge ? j : n - j]);
                }
            }
            b.uv.push_back(std::move(uv));
            b.xyz.push_back(std::move(xyz));
        }
        return {};
    }

    /// How finely the surface bends over [lo, hi] in (u, v), from a lattice of samples, for grid
    /// sides that sag no more than sideTolerance.
    Spacing spacingOf(const geo::Surface& s, const Vec2& lo, const Vec2& hi) const {
        const double half = sideTolerance();
        Spacing sp;
        double su = 0.0;
        double sv = 0.0;
        for (int i = 0; i < kSurfaceSamples; ++i) {
            for (int j = 0; j < kSurfaceSamples; ++j) {
                const auto d = s.derivatives(lo.x + (hi.x - lo.x) * i / (kSurfaceSamples - 1),
                                             lo.y + (hi.y - lo.y) * j / (kSurfaceSamples - 1));
                const double lu = d.du.length();
                const double lv = d.dv.length();
                su = std::max(su, lu);
                sv = std::max(sv, lv);
                const Vec3 n = d.du.cross(d.dv);
                const double ln = n.length();
                if (!(ln > 0.0)) continue;
                // Turning per unit parameter is |n . S''| / |S'|, curvature that over |S'|.
                const double bu = std::fabs(n.dot(d.duu)) / ln;
                const double bv = std::fabs(n.dot(d.dvv)) / ln;
                if (bu > 0.0) {
                    sp.du =
                        std::min(sp.du, allowedTurn(bu / (lu * lu), half, opt_.maxAngle) * lu / bu);
                }
                if (bv > 0.0) {
                    sp.dv =
                        std::min(sp.dv, allowedTurn(bv / (lv * lv), half, opt_.maxAngle) * lv / bv);
                }
            }
        }
        if (su > 0.0 && std::isfinite(su)) sp.su = su;
        if (sv > 0.0 && std::isfinite(sv)) sp.sv = sv;
        return sp;
    }

    /// Interior points (in w) on a grid of the spacing over [lo, hi], without those within
    /// kKeepAway spacings of a boundary segment, which would make slivers along it.
    std::string gridPoints(const Spacing& sp, const Vec2& lo, const Vec2& hi,
                           const std::vector<std::vector<Vec2>>& loopsW,
                           std::vector<Vec2>& out) const {
        const double nu = std::ceil((hi.x - lo.x) / sp.du);
        const double nv = std::ceil((hi.y - lo.y) / sp.dv);
        if (!(nu > 1.0) || !(nv > 1.0)) return {};
        if (!((nu - 1.0) * (nv - 1.0) <= 0.5 * static_cast<double>(opt_.maxTrianglesPerFace))) {
            return "the face's grid alone needs more triangles than the cap";
        }
        const auto cu = static_cast<std::size_t>(nu) - 1;
        const auto cv = static_cast<std::size_t>(nv) - 1;
        const double stepU = (hi.x - lo.x) / nu;
        const double stepV = (hi.y - lo.y) / nv;
        const auto pointAt = [&](std::size_t i, std::size_t j) {
            return Vec2{(lo.x + stepU * static_cast<double>(i + 1)) * sp.su,
                        (lo.y + stepV * static_cast<double>(j + 1)) * sp.sv};
        };
        const double away = kKeepAway * std::min(stepU * sp.su, stepV * sp.sv);
        std::vector<bool> keep(cu * cv, true);
        // The grid indices whose lines lie within [x0, x1] (in w) or a line beyond, clamped.
        const auto lines = [](double x0, double x1, double from, double step, std::size_t count) {
            const double first = std::max(0.0, std::floor((x0 - from) / step) - 2.0);
            const double last =
                std::min(static_cast<double>(count) - 1.0, std::ceil((x1 - from) / step));
            return std::pair<std::size_t, std::size_t>{
                static_cast<std::size_t>(first),
                last >= first ? static_cast<std::size_t>(last) + 1 : 0};
        };
        for (const auto& loop : loopsW) {
            for (std::size_t k = 0; k < loop.size(); ++k) {
                const Vec2& a = loop[k];
                const Vec2& b = loop[(k + 1) % loop.size()];
                const auto [i0, i1] = lines(std::min(a.x, b.x) - away, std::max(a.x, b.x) + away,
                                            lo.x * sp.su, stepU * sp.su, cu);
                const auto [j0, j1] = lines(std::min(a.y, b.y) - away, std::max(a.y, b.y) + away,
                                            lo.y * sp.sv, stepV * sp.sv, cv);
                for (std::size_t i = i0; i < i1; ++i) {
                    for (std::size_t j = j0; j < j1; ++j) {
                        if (distanceToSegment(pointAt(i, j), a, b) < away) keep[i * cv + j] = false;
                    }
                }
            }
        }
        for (std::size_t i = 0; i < cu; ++i) {
            for (std::size_t j = 0; j < cv; ++j) {
                if (keep[i * cv + j]) out.push_back(pointAt(i, j));
            }
        }
        return {};
    }

    /// kFits when the triangle's (u, v) centroid, and the midpoints of its edges that are not
    /// boundary segments, map within the chord tolerance of it, and it faces within half the
    /// options' angle of the surface's du x dv at its centroid. Otherwise where it needs a point:
    /// an edge k (from vertex k to k + 1) whose midpoint strays, as the centroid would leave that
    /// edge in place, or kCentroid.
    int misfit(const geo::Surface& s, const std::array<Vec2, 3>& uv, const std::array<Vec3, 3>& p,
               const std::array<bool, 3>& onBoundary) const {
        const Vec3 n = (p[1] - p[0]).cross(p[2] - p[0]);
        const double ln = n.length();
        if (!(ln > 0.0)) return kCentroid;
        const auto near = [&](const Vec2& q) {
            return distanceToTriangle(s.evaluate(q.x, q.y), p[0], p[1], p[2]) <=
                   opt_.chordTolerance;
        };
        for (int k = 0; k < 3; ++k) {
            // A boundary segment's midpoint was held to its chord when the edge was sampled.
            const auto a = static_cast<std::size_t>(k);
            if (!onBoundary[a] && !near((uv[a] + uv[(a + 1) % 3]) * 0.5)) return k;
        }
        const Vec2 c = (uv[0] + uv[1] + uv[2]) / 3.0;
        if (!near(c)) return kCentroid;
        const Vec3 sn = s.normal(c.x, c.y);
        if (sn.lengthSquared() == 0.0) return kFits;  // a pole or apex: no direction to keep to
        const double cosine = n.dot(sn) / ln;
        return cosine > 0.0 && cosine >= std::cos(0.5 * opt_.maxAngle) ? kFits : kCentroid;
    }

    /// Triangulates the face's region, refining until every triangle fits, into @p out.
    std::string triangulate(const Face& f, const Boundary& b, FaceMesh& out) const {
        const geo::Surface& s = *f.exactSurface;
        Vec2 lo = b.uv[0][0];
        Vec2 hi = lo;
        for (const Vec2& p : b.uv[0]) {
            lo = {std::min(lo.x, p.x), std::min(lo.y, p.y)};
            hi = {std::max(hi.x, p.x), std::max(hi.y, p.y)};
        }
        const Spacing sp = spacingOf(s, lo, hi);
        std::vector<std::vector<Vec2>> loopsW;
        std::vector<Vec2> uvs;
        std::vector<Vec3> xyzs;
        for (std::size_t l = 0; l < b.uv.size(); ++l) {
            loopsW.emplace_back();
            for (const Vec2& p : b.uv[l]) loopsW.back().push_back({p.x * sp.su, p.y * sp.sv});
            uvs.insert(uvs.end(), b.uv[l].begin(), b.uv[l].end());
            xyzs.insert(xyzs.end(), b.xyz[l].begin(), b.xyz[l].end());
        }
        const std::vector<std::vector<Vec2>> holesW(loopsW.begin() + 1, loopsW.end());
        std::vector<Vec2> interior;
        std::string problem = gridPoints(sp, lo, hi, loopsW, interior);
        if (!problem.empty()) return problem;

        const std::size_t boundaryCount = uvs.size();
        std::size_t kept = 0;
        for (int round = 0;; ++round) {
            const geo::Triangulation2D t = geo::triangulateRegion(loopsW[0], holesW, interior);
            if (!t.problem.empty()) return "the face's (u, v) region: " + t.problem;
            if (t.triangles.size() > opt_.maxTrianglesPerFace) {
                return "the face needs more triangles than the cap of " +
                       std::to_string(opt_.maxTrianglesPerFace);
            }
            if (round > 0 && t.points.size() <= kept) return "refining the face stalled";
            kept = t.points.size();
            std::unordered_set<std::uint64_t> walls;
            std::size_t base = 0;
            for (const auto& loop : loopsW) {
                for (std::size_t i = 0; i < loop.size(); ++i) {
                    walls.insert(edgeKey(t.boundaryVertex[base + i],
                                         t.boundaryVertex[base + (i + 1) % loop.size()]));
                }
                base += loop.size();
            }
            std::vector<Vec2> uv(uvs);
            std::vector<Vec3> xyz(xyzs);
            for (std::size_t i = boundaryCount; i < t.points.size(); ++i) {
                uv.push_back({t.points[i].x / sp.su, t.points[i].y / sp.sv});
                xyz.push_back(s.evaluate(uv.back().x, uv.back().y));
            }
            std::vector<Vec2> added;
            std::unordered_set<std::uint64_t> split;
            for (const auto& tri : t.triangles) {
                const std::array<Vec2, 3> tuv = {uv[tri[0]], uv[tri[1]], uv[tri[2]]};
                const std::array<Vec3, 3> txyz = {xyz[tri[0]], xyz[tri[1]], xyz[tri[2]]};
                std::array<bool, 3> onBoundary{};
                for (std::size_t k = 0; k < 3; ++k) {
                    onBoundary[k] = walls.count(edgeKey(tri[k], tri[(k + 1) % 3])) != 0;
                }
                const int m = misfit(s, tuv, txyz, onBoundary);
                if (m == kCentroid) {
                    added.push_back((t.points[tri[0]] + t.points[tri[1]] + t.points[tri[2]]) / 3.0);
                } else if (m != kFits) {
                    const std::uint32_t a = tri[static_cast<std::size_t>(m)];
                    const std::uint32_t b = tri[static_cast<std::size_t>(m + 1) % 3];
                    if (split.insert(edgeKey(a, b)).second) {
                        added.push_back((t.points[a] + t.points[b]) * 0.5);
                    }
                }
            }
            if (added.empty()) {
                emit(f, t, uv, xyz, out);
                return {};
            }
            if (round == kRefineRounds) return "refining the face did not meet the tolerance";
            interior.assign(t.points.begin() + static_cast<std::ptrdiff_t>(boundaryCount),
                            t.points.end());
            interior.insert(interior.end(), added.begin(), added.end());
        }
    }

    /// The triangles' vertices, compacted, with the surface's outward normals; triangles wound
    /// outward.
    static void emit(const Face& f, const geo::Triangulation2D& t, const std::vector<Vec2>& uv,
                     const std::vector<Vec3>& xyz, FaceMesh& out) {
        const geo::Surface& s = *f.exactSurface;
        const double sign = f.reversed ? -1.0 : 1.0;
        std::vector<std::uint32_t> index(t.points.size(), kUnused);
        for (const auto& tri : t.triangles) {
            std::array<std::uint32_t, 3> v{};
            for (std::size_t k = 0; k < 3; ++k) {
                if (index[tri[k]] == kUnused) {
                    index[tri[k]] = static_cast<std::uint32_t>(out.points.size());
                    out.points.push_back(xyz[tri[k]]);
                    out.normals.push_back(s.normal(uv[tri[k]].x, uv[tri[k]].y) * sign);
                }
                v[k] = index[tri[k]];
            }
            // Counter-clockwise in (u, v) faces along du x dv: outward unless reversed.
            if (f.reversed) std::swap(v[1], v[2]);
            out.triangles.push_back(v);
        }
        // Where the surface has no normal (a pole, an apex), the triangles' own.
        for (std::size_t i = 0; i < out.normals.size(); ++i) {
            if (out.normals[i].lengthSquared() > 0.0) continue;
            Vec3 sum;
            for (const auto& tri : out.triangles) {
                if (tri[0] != i && tri[1] != i && tri[2] != i) continue;
                sum += (out.points[tri[1]] - out.points[tri[0]])
                           .cross(out.points[tri[2]] - out.points[tri[0]]);
            }
            const double len = sum.length();
            if (len > 0.0) out.normals[i] = sum / len;
        }
    }

    const topo::Solid& solid_;
    const MeshOptions& opt_;
    math::ModelTolerance tol_;
    std::unordered_map<const Edge*, EdgeSamples> edges_;
    /// Each sampled half-edge's pcurve parameter at each of its edge's samples, in its own
    /// direction.
    std::unordered_map<const HalfEdge*, std::vector<double>> params_;
};

}  // namespace

std::vector<FaceMesh> TrimmedFaceMesher::meshSolid(const topo::Solid& solid,
                                                   const MeshOptions& options) {
    std::vector<FaceMesh> out;
    std::string invalid;
    if (!(options.chordTolerance > 0.0) || !std::isfinite(options.chordTolerance)) {
        invalid = "the chord tolerance must be positive and finite";
    } else if (!(options.maxAngle > 0.0) || !(options.maxAngle <= math::kPi)) {
        invalid = "the angle must be above 0 and at most pi";
    } else if (options.maxTrianglesPerFace == 0) {
        invalid = "the triangle cap must be above 0";
    }
    Mesher mesher(solid, options);
    if (invalid.empty()) mesher.sampleEdges();
    for (const Face& f : solid.faces()) {
        if (!f.alive) continue;
        if (invalid.empty()) {
            out.push_back(mesher.meshFace(f));
        } else {
            out.push_back(FaceMesh{&f, {}, {}, {}, invalid});
        }
    }
    return out;
}

}  // namespace hz::model
