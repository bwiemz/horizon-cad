#include "horizon/modeling/SeamInsertion.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <exception>
#include <functional>
#include <limits>
#include <memory>
#include <optional>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

#include "horizon/geometry/curves/AnalyticCurves.h"
#include "horizon/geometry/curves/Line2D.h"
#include "horizon/geometry/curves/NurbsCurve.h"
#include "horizon/geometry/surfaces/AnalyticSurfaces.h"
#include "horizon/geometry/surfaces/NurbsSurface.h"
#include "horizon/math/Constants.h"
#include "horizon/topology/EulerOps.h"
#include "horizon/topology/LoopUnwrap.h"

namespace hz::model {

namespace {

using math::Vec2;
using math::Vec3;
using topo::Edge;
using topo::Face;
using topo::HalfEdge;
using topo::Solid;
using topo::Vertex;
using topo::Wire;

/// The chords each pcurve is unwrapped into: the validator's samples.
constexpr int kChords = topo::kLoopSamples - 1;
/// The lines a seam may take, tried in turn: u0, then the middles of the widest gaps.
constexpr std::size_t kLines = 16;
/// Bisection and golden-section steps: enough to reach the last bit of a double.
constexpr int kSteps = 200;

const std::string kNoSeam = "no seam can be laid on it: ";

double coord(const Vec2& p, int dir) {
    return dir == 0 ? p.x : p.y;
}

/// The point of (u, v) where parameter @p dir is @p at and the other is @p along.
Vec2 isoPoint(int dir, double at, double along) {
    return dir == 0 ? Vec2{at, along} : Vec2{along, at};
}

double periodOf(const geo::Surface& s, int dir) {
    return dir == 0 ? topo::uPeriod(s) : topo::vPeriod(s);
}

/// How fast @p s moves along parameter @p dir at (u, v).
double speedAlong(const geo::Surface& s, int dir, const Vec2& uv) {
    const auto d = s.derivatives(uv.x, uv.y);
    return (dir == 0 ? d.du : d.dv).length();
}

Vec3 pointOf(const geo::Surface& s, const HalfEdge& h, double t) {
    const Vec2 uv = h.pcurve->evaluate(t);
    return s.evaluate(uv.x, uv.y);
}

// ---- a face's loops, as the validator reads them ----

/// One loop of a face: its half-edges from its wire's first, and its chain in (u, v), unwrapped
/// as the validator unwraps it (kChords chords for each half-edge, in ring order).
struct Loop {
    Wire* wire = nullptr;
    std::vector<HalfEdge*> ring;
    std::vector<double> allowances;  ///< each half-edge's join to the next, as joinAllowance
    topo::LoopPolygon chain;
    std::array<double, 2> winds{};  ///< whole periods its chain ends from its start, in u and v
};

/// The half-edges of the loop through @p first, following next; empty where the cycle leaves
/// @p face or does not come back within @p limit half-edges.
std::vector<HalfEdge*> ringFrom(const Face& face, HalfEdge* first, std::size_t limit) {
    std::vector<HalfEdge*> ring;
    HalfEdge* h = first;
    while (h != nullptr && h->face == &face && ring.size() <= limit) {
        ring.push_back(h);
        h = h->next;
        if (h == first) return ring;
    }
    return {};
}

/// The whole periods @p chain winds round @p s in u and v: its end's distance from its start,
/// where unwrapLoop finds it a wrap (beyond the last join's allowance), in periods.
std::array<double, 2> windingOf(const topo::LoopPolygon& chain, const geo::Surface& s,
                                double allowance) {
    std::array<double, 2> winds{};
    if (chain.empty()) return winds;
    const Vec2 end = chain.back().b;
    const Vec2 gap = end - chain.front().a;
    const auto d = s.derivatives(end.x, end.y);
    const double speeds[2] = {d.du.length(), d.dv.length()};
    for (int dir = 0; dir < 2; ++dir) {
        const double period = periodOf(s, dir);
        if (period > 0.0 && std::fabs(coord(gap, dir)) * speeds[dir] > allowance) {
            winds[static_cast<std::size_t>(dir)] = std::round(coord(gap, dir) / period);
        }
    }
    return winds;
}

/// @p face's loops, its outer loop first; nullopt where one cannot be read (a cycle that does
/// not close, a half-edge without a pcurve), which the validator reports.
std::optional<std::vector<Loop>> loopsOf(const Solid& solid, const Face& face,
                                         const math::ModelTolerance& tolerance) {
    const geo::Surface& s = *face.exactSurface;
    const std::size_t limit = 2 * solid.edges().size() + 1;
    std::vector<Wire*> wires{face.outerLoop};
    wires.insert(wires.end(), face.innerLoops.begin(), face.innerLoops.end());
    std::vector<Loop> loops;
    for (Wire* w : wires) {
        if (w == nullptr || w->halfEdge == nullptr) return std::nullopt;
        Loop loop;
        loop.wire = w;
        loop.ring = ringFrom(face, w->halfEdge, limit);
        if (loop.ring.empty()) return std::nullopt;
        std::vector<const HalfEdge*> walk;
        for (HalfEdge* h : loop.ring) {
            if (!h->pcurve || h->edge == nullptr || h->twin == nullptr || h->next == nullptr ||
                h->origin == nullptr) {
                return std::nullopt;
            }
            walk.push_back(h);
            loop.allowances.push_back(topo::joinAllowance(*h, tolerance));
        }
        topo::unwrapLoop(walk, topo::kLoopSamples, s, topo::uPeriod(s), topo::vPeriod(s),
                         loop.allowances, loop.chain);
        loop.winds = windingOf(loop.chain, s, loop.allowances.back());
        loops.push_back(std::move(loop));
    }
    return loops;
}

/// Whether some half-edge of @p face has its twin on @p face too: a seam of its own.
bool hasSeam(const Solid& solid, const Face& face) {
    const std::size_t limit = 2 * solid.edges().size() + 1;
    std::vector<Wire*> wires{face.outerLoop};
    wires.insert(wires.end(), face.innerLoops.begin(), face.innerLoops.end());
    for (Wire* w : wires) {
        if (w == nullptr) continue;
        for (const HalfEdge* h : ringFrom(face, w->halfEdge, limit)) {
            if (h->twin != nullptr && h->twin->face == &face) return true;
        }
    }
    return false;
}

// ---- rims read the wrong way round ----

/// Whether @p e is a rim: a closed edge alone in its loop on each of two faces, each on its pcurve.
bool isRim(const Edge& e) {
    const HalfEdge* h = e.halfEdge;
    if (!e.alive || h == nullptr || h->twin == nullptr || !e.exactCurve) return false;
    const HalfEdge* t = h->twin;
    return h->origin != nullptr && h->origin == t->origin && h->next == h && t->next == t &&
           h->face != nullptr && t->face != nullptr && h->face != t->face && h->pcurve && t->pcurve;
}

/// Turns @p e, a rim, the other way round on both its faces: each half-edge runs its pcurve the
/// other way, from the same vertex, and the other one runs with the edge's curve.
void turnRim(Edge& e) {
    HalfEdge* h = e.halfEdge;
    std::swap(h->pcurveStart, h->pcurveEnd);
    std::swap(h->twin->pcurveStart, h->twin->pcurveEnd);
    e.halfEdge = h->twin;
}

/// What a face says of two of its loops: turned either both or neither (@p differ false), or one
/// of them (true). A loop that is not a rim is held as written, its index kNotARim.
struct Tie {
    std::size_t a = 0;
    std::size_t b = 0;
    bool differ = false;
};
constexpr std::size_t kNotARim = std::numeric_limits<std::size_t>::max();

/// The ties @p face's loops make, into @p ties, each loop's rim's index from @p rimOf (kNotARim
/// for a loop that is not a rim): a band's two loops wind opposite ways, and an inner loop of a
/// face whose surface wraps neither way runs against its outer loop. Any other loop but the outer
/// loop of a face that is no band (a band's hole, a hole in a face that wraps, a hole whose area
/// says nothing) is held as written: nothing here says which way it must run. The outer loop of a
/// face that is no band is not held: its face takes its sense from it, or says it runs against it.
void tiesOf(const Solid& solid, const Face& face, const math::ModelTolerance& tolerance,
            const std::function<std::size_t(const Loop&)>& rimOf, std::vector<Tie>& ties) {
    if (!face.alive || !face.exactSurface) return;
    std::optional<std::vector<Loop>> read;
    try {
        read = loopsOf(solid, face, tolerance);
    } catch (const std::exception&) {
        return;  // its loops cannot be read: insertSeams and the validator say why
    }
    if (!read) return;
    const std::vector<Loop>& loops = *read;
    const geo::Surface& s = *face.exactSurface;
    std::vector<bool> tied(loops.size(), false);
    bool band = false;
    if (!(topo::uPeriod(s) > 0.0) && !(topo::vPeriod(s) > 0.0)) {
        const double outside = topo::signedArea(loops.front().chain);
        for (std::size_t k = 1; k < loops.size(); ++k) {
            const double inside = topo::signedArea(loops[k].chain);
            if (!(outside != 0.0) || !(inside != 0.0)) continue;  // NaN too: says nothing
            ties.push_back(
                {rimOf(loops.front()), rimOf(loops[k]), (inside > 0.0) == (outside > 0.0)});
            tied[k] = true;
        }
    } else if (!hasSeam(solid, face)) {
        // A band: two loops once round along one parameter, and none round any other way.
        std::vector<std::size_t> winding;
        int dir = -1;
        bool alone = true;
        for (std::size_t k = 0; k < loops.size(); ++k) {
            const bool u = loops[k].winds[0] != 0.0;
            const bool v = loops[k].winds[1] != 0.0;
            if (!u && !v) continue;
            if ((u && v) || (dir >= 0 && dir != (u ? 0 : 1))) alone = false;
            dir = u ? 0 : 1;
            winding.push_back(k);
        }
        if (alone && winding.size() == 2) {
            const double wa = loops[winding[0]].winds[static_cast<std::size_t>(dir)];
            const double wb = loops[winding[1]].winds[static_cast<std::size_t>(dir)];
            if (std::fabs(wa) == 1.0 && std::fabs(wb) == 1.0) {
                ties.push_back({rimOf(loops[winding[0]]), rimOf(loops[winding[1]]), wa == wb});
                tied[winding[0]] = tied[winding[1]] = true;
                band = true;
            }
        }
    }
    for (std::size_t k = band ? 0 : 1; k < loops.size(); ++k) {
        if (!tied[k]) ties.push_back({kNotARim, rimOf(loops[k]), false});
    }
}

// ---- where a loop crosses a seam's line ----

/// Where a loop's chain meets one of the lines c + j period of parameter dir.
struct Crossing {
    std::size_t index = 0;  ///< the half-edge of the ring it is on
    bool atOrigin = false;  ///< at that half-edge's origin; else inside its pcurve
    double lo = 0.0;        ///< the pcurve parameters it lies between, inside
    double hi = 0.0;
    double level = 0.0;  ///< the chain's value there
};

/// Where @p loop's chain meets the lines @p c + j @p period of parameter @p dir: at a vertex
/// whose value is within its join's allowance of one, otherwise inside a pcurve where it passes
/// a line between two of its samples (or lands on one). Past two, more are not looked for.
std::vector<Crossing> crossingsOf(const Loop& loop, const geo::Surface& s, int dir, double c,
                                  double period) {
    constexpr std::size_t kEnough = 2;
    const std::size_t n = loop.ring.size();
    const auto nearest = [&](double x) { return c + std::round((x - c) / period) * period; };
    std::vector<Crossing> found;
    std::vector<char> onLine(n, 0);
    for (std::size_t k = 0; k < n; ++k) {
        const topo::LoopSegment& first = loop.chain[k * kChords];
        const double x = coord(first.a, dir);
        const double speed = speedAlong(s, dir, first.a - first.shift);
        if (speed > 0.0 && std::fabs(x - nearest(x)) * speed <= loop.allowances[(k + n - 1) % n]) {
            onLine[k] = 1;
            found.push_back({k, true, 0.0, 0.0, nearest(x)});
        }
    }
    for (std::size_t k = 0; k < n && found.size() <= kEnough; ++k) {
        const HalfEdge& h = *loop.ring[k];
        std::array<double, kChords + 1> x{};
        for (int i = 0; i < kChords; ++i) {
            x[static_cast<std::size_t>(i)] = coord(loop.chain[k * kChords + i].a, dir);
        }
        x[kChords] = coord(loop.chain[k * kChords + kChords - 1].b, dir);
        if (onLine[k] != 0) x[0] = nearest(x[0]);
        if (onLine[(k + 1) % n] != 0) x[kChords] = nearest(x[kChords]);
        for (int i = 0; i < kChords; ++i) {
            const double a = x[static_cast<std::size_t>(i)];
            const double b = x[static_cast<std::size_t>(i) + 1];
            const double ta = topo::loopSampleParameter(h, i, topo::kLoopSamples);
            const double tb = topo::loopSampleParameter(h, i + 1, topo::kLoopSamples);
            if (!std::isfinite(a) || !std::isfinite(b) || std::fabs(b - a) > 2.0 * period) {
                return std::vector<Crossing>(kEnough + 1);  // not a chain that tells anything
            }
            if (i > 0 && a == nearest(a)) found.push_back({k, false, ta, ta, a});
            // The lines strictly between the two samples.
            const double from = (std::min(a, b) - c) / period;
            const double to = (std::max(a, b) - c) / period;
            for (double j = std::floor(from) + 1.0; j < to && found.size() <= kEnough; j += 1.0) {
                found.push_back({k, false, ta, tb, c + j * period});
            }
        }
    }
    return found;
}

/// The range of parameter @p dir that @p loop's chain covers.
std::pair<double, double> spanOf(const Loop& loop, int dir) {
    double lo = std::numeric_limits<double>::infinity();
    double hi = -lo;
    for (const topo::LoopSegment& seg : loop.chain) {
        for (const Vec2& p : {seg.a, seg.b}) {
            lo = std::min(lo, coord(p, dir));
            hi = std::max(hi, coord(p, dir));
        }
    }
    return {lo, hi};
}

/// The lines of parameter @p dir a seam joining @p a and @p b may take, in the order tried: the
/// value at @p a's first vertex (u0), then the middles of the gaps between the two loops'
/// vertices and the ends of the other loops' spans, widest first; kLines at most.
std::vector<double> seamLines(const Loop& a, const Loop& b, const std::vector<const Loop*>& others,
                              int dir, double period) {
    std::vector<double> lines{coord(a.chain.front().a, dir)};
    std::vector<double> marks;
    const auto mark = [&](double x) {
        if (std::isfinite(x)) marks.push_back(x - period * std::floor(x / period));
    };
    for (const Loop* l : {&a, &b}) {
        for (std::size_t k = 0; k < l->ring.size(); ++k) mark(coord(l->chain[k * kChords].a, dir));
    }
    for (const Loop* o : others) {
        const auto [lo, hi] = spanOf(*o, dir);
        mark(lo);
        mark(hi);
    }
    std::sort(marks.begin(), marks.end());
    marks.erase(std::unique(marks.begin(), marks.end()), marks.end());
    std::vector<std::pair<double, double>> gaps;  // width, middle
    for (std::size_t i = 0; i < marks.size(); ++i) {
        const double next = i + 1 < marks.size() ? marks[i + 1] : marks.front() + period;
        gaps.emplace_back(next - marks[i], marks[i] + (next - marks[i]) / 2.0);
    }
    std::stable_sort(gaps.begin(), gaps.end(),
                     [](const auto& p, const auto& q) { return p.first > q.first; });
    for (const auto& gap : gaps) {
        if (lines.size() == kLines) break;
        lines.push_back(gap.second);
    }
    return lines;
}

/// Whether the line @p c of parameter @p dir runs clear of @p others: outside each one's span,
/// a millionth of a period clear.
bool clearOf(double c, const std::vector<const Loop*>& others, int dir, double period) {
    const double margin = 1e-6 * period;
    for (const Loop* o : others) {
        const auto [lo, hi] = spanOf(*o, dir);
        if (!(hi - lo < period - 2.0 * margin)) return false;
        // The line's copy at or above the span's low end.
        const double copy = c + std::ceil((lo - margin - c) / period) * period;
        if (copy <= hi + margin) return false;
    }
    return true;
}

// ---- splitting an edge where the seam's line crosses it ----

/// A split of an edge, worked out before anything is changed.
struct Split {
    HalfEdge* h = nullptr;  ///< the face's half-edge it falls on
    Vertex* at = nullptr;   ///< an end vertex it falls on after all: no split
    double sOwn = 0.0;      ///< h's pcurve parameter there
    double sTwin = 0.0;     ///< its twin's
    double t = 0.0;         ///< the edge's curve parameter there
    Vec3 point;             ///< the new vertex
    double vertexTolerance = 0.0;
    std::shared_ptr<geo::NurbsCurve> carrierFirst;   ///< from the edge's start to the split
    std::shared_ptr<geo::NurbsCurve> carrierSecond;  ///< from the split to its end
};

/// The parameter of @p h's pcurve whose point on @p s is nearest @p p: the nearest of 65
/// samples, by golden section between its neighbours, then Gauss-Newton steps that only ever
/// come nearer.
double nearestOnPcurve(const HalfEdge& h, const geo::Surface& s, const Vec3& p) {
    constexpr int kScan = 64;
    const double a = h.pcurveStart;
    const double b = h.pcurveEnd;
    const auto at = [&](int i) { return a + (b - a) * i / kScan; };
    const auto far = [&](double t) { return (pointOf(s, h, t) - p).lengthSquared(); };
    int best = 0;
    double bestFar = std::numeric_limits<double>::infinity();
    for (int i = 0; i <= kScan; ++i) {
        const double d = far(at(i));
        if (d < bestFar) {
            bestFar = d;
            best = i;
        }
    }
    double lo = at(std::max(best - 1, 0));
    double hi = at(std::min(best + 1, kScan));
    const double golden = (std::sqrt(5.0) - 1.0) / 2.0;
    double x1 = hi - golden * (hi - lo);
    double x2 = lo + golden * (hi - lo);
    double f1 = far(x1);
    double f2 = far(x2);
    for (int i = 0; i < kSteps && x1 != x2; ++i) {
        if (f1 < f2) {
            hi = x2;
            x2 = x1;
            f2 = f1;
            x1 = hi - golden * (hi - lo);
            f1 = far(x1);
        } else {
            lo = x1;
            x1 = x2;
            f1 = f2;
            x2 = lo + golden * (hi - lo);
            f2 = far(x2);
        }
    }
    double t = f1 < f2 ? x1 : x2;
    double tFar = std::min(f1, f2);
    const double low = std::min(a, b);
    const double high = std::max(a, b);
    for (int i = 0; i < 8; ++i) {
        const Vec2 uv = h.pcurve->evaluate(t);
        const auto d = s.derivatives(uv.x, uv.y);
        const Vec2 dp = h.pcurve->derivative(t, 1);
        const Vec3 along = d.du * dp.x + d.dv * dp.y;
        const double speed2 = along.lengthSquared();
        if (!(speed2 > 0.0)) break;
        const double next = std::clamp(t - (d.point - p).dot(along) / speed2, low, high);
        const double nextFar = far(next);
        if (!(nextFar < tFar)) break;
        t = next;
        tFar = nextFar;
    }
    return t;
}

/// @p curve split at its parameter nearest @p point into the parts before and after it, or
/// nullopt where that parameter is at an end.
std::optional<std::pair<geo::NurbsCurve, geo::NurbsCurve>> cut(const geo::NurbsCurve& curve,
                                                               const Vec3& point) {
    const double t = curve.closestPoint(point);
    const double domain = curve.tMax() - curve.tMin();
    if (!(t - curve.tMin() > 1e-9 * domain) || !(curve.tMax() - t > 1e-9 * domain)) {
        return std::nullopt;
    }
    return std::make_pair(curve.segment(curve.tMin(), t), curve.segment(t, curve.tMax()));
}

/// The distance from @p p to @p curve.
double distanceTo(const geo::NurbsCurve& curve, const Vec3& p) {
    return (curve.evaluate(curve.closestPoint(p)) - p).length();
}

/// How far apart the ends of @p curve are from @p a and @p b, whichever way round it runs.
double endsFrom(const geo::NurbsCurve& curve, const Vec3& a, const Vec3& b) {
    const Vec3 c0 = curve.evaluate(curve.tMin());
    const Vec3 c1 = curve.evaluate(curve.tMax());
    return std::min(std::max((c0 - a).length(), (c1 - b).length()),
                    std::max((c0 - b).length(), (c1 - a).length()));
}

/// Works out the split of @p h's edge at its pcurve parameter @p s: the edge's curve parameter
/// nearest the pcurve's point there, the vertex on the curve at it, the twin's matching pcurve
/// parameter, and the carrier's two parts. Empty on success; else why.
std::string planSplit(HalfEdge& h, double s, const math::ModelTolerance& tolerance, Split& out) {
    Edge& e = *h.edge;
    const HalfEdge& twin = *h.twin;
    if (!e.exactCurve || !twin.pcurve || twin.face == nullptr || !twin.face->exactSurface) {
        return kNoSeam +
               "an edge its seam crosses has no exact curve, or no pcurve on its other "
               "face";
    }
    const geo::Curve3& curve = *e.exactCurve;
    Vertex* start = e.halfEdge->origin;
    Vertex* end = e.halfEdge->twin->origin;
    const Vec3 own = pointOf(*h.face->exactSurface, h, s);
    const double lo = std::min(e.curveStart, e.curveEnd);
    const double hi = std::max(e.curveStart, e.curveEnd);
    double t = curve.closestParameter(own);
    if (curve.periodic()) {
        const geo::Interval range = curve.range();
        const double period = range.max - range.min;
        t += period * std::ceil((lo - t) / period);
    }
    if (!(t >= lo && t <= hi)) {
        return kNoSeam + "where its seam crosses an edge is not on the edge's curve";
    }
    out = Split{};
    out.h = &h;
    out.point = curve.evaluate(t);
    for (Vertex* v : {start, end}) {
        if ((out.point - v->point).length() <=
            tolerance.effective(std::max(e.tolerance, v->tolerance))) {
            out.at = v;
            return {};
        }
    }
    out.t = t;
    out.sOwn = s;
    out.sTwin = nearestOnPcurve(twin, *twin.face->exactSurface, out.point);
    const double ownGap = (own - out.point).length();
    const double twinGap =
        (pointOf(*twin.face->exactSurface, twin, out.sTwin) - out.point).length();
    if (!(std::max(ownGap, twinGap) <= 2.0 * tolerance.effective(e.tolerance))) {
        return kNoSeam + "where its seam crosses an edge, the edge's pcurves part by " +
               std::to_string(std::max(ownGap, twinGap));
    }
    const double gap = std::max(ownGap, twinGap);
    out.vertexTolerance = gap > tolerance.linear ? gap : 0.0;
    if (e.curve) {
        const auto parts = cut(*e.curve, out.point);
        if (!parts) return kNoSeam + "an edge its seam crosses has a carrier it cannot be split on";
        // The part along the edge's curve from its start to the split is the first.
        const Vec3 middle = curve.evaluate((e.curveStart + t) / 2.0);
        const bool firstFirst =
            distanceTo(parts->first, middle) <= distanceTo(parts->second, middle);
        out.carrierFirst =
            std::make_shared<geo::NurbsCurve>(firstFirst ? parts->first : parts->second);
        out.carrierSecond =
            std::make_shared<geo::NurbsCurve>(firstFirst ? parts->second : parts->first);
        const double near = 2.0 * tolerance.effective(std::max(e.tolerance, out.vertexTolerance));
        if (!(endsFrom(*out.carrierFirst, start->point, out.point) <= near) ||
            !(endsFrom(*out.carrierSecond, out.point, end->point) <= near)) {
            return kNoSeam +
                   "an edge its seam crosses has a carrier that does not end at its "
                   "vertices";
        }
    }
    return {};
}

/// The number @p digits spells, or nullopt where it is not all digits or has more than nine.
std::optional<int> pieceNumber(const std::string& digits) {
    if (digits.empty() || digits.size() > 9) return std::nullopt;
    int k = 0;
    for (const char c : digits) {
        if (c < '0' || c > '9') return std::nullopt;
        k = k * 10 + (c - '0');
    }
    return k;
}

/// The edge a name `<edge>/piece:<k>` is a piece of, or nullopt for a name of no piece.
std::optional<std::string> pieceOf(const std::string& tag) {
    const std::size_t at = tag.rfind("/piece:");
    if (at == std::string::npos || !pieceNumber(tag.substr(at + 7))) return std::nullopt;
    return tag.substr(0, at);
}

/// The names of the two parts a split of @p e leaves, the part from its start first: pieces of
/// the edge it was (`<edge>/piece:<k>`), as the kernel names what an operation cuts in pieces,
/// so a reference to the edge has no exact match left and takes them all as its descendants. A
/// piece cut again keeps its name, and its new part is the edge's next piece. Nameless where @p e
/// is.
std::pair<topo::TopologyID, topo::TopologyID> pieceNames(const Solid& solid, const Edge& e) {
    if (!e.topoId.isValid()) return {};
    const std::optional<std::string> whole = pieceOf(e.topoId.tag());
    if (!whole) {
        return {e.topoId.child("piece", 0), e.topoId.child("piece", 1)};
    }
    const std::string prefix = *whole + "/piece:";
    int next = 1;
    for (const Edge& other : solid.edges()) {
        const std::string& tag = other.topoId.tag();
        if (tag.compare(0, prefix.size(), prefix) != 0) continue;
        if (const auto k = pieceNumber(tag.substr(prefix.size())); k && *k >= next) next = *k + 1;
    }
    return {e.topoId, topo::TopologyID::fromTag(*whole).child("piece", next)};
}

/// Splits the edge of @p plan's half-edge as planned: the edge keeps its start to the new
/// vertex, a new edge runs on from it, and each half-edge's pcurve is split at its parameter.
/// Returns the half-edge of the planned half-edge's loop that leaves the new vertex.
HalfEdge* applySplit(Solid& solid, const Split& plan) {
    HalfEdge* h = plan.h;
    Edge& e = *h->edge;
    HalfEdge* forward = e.halfEdge;  // from the edge's start, at curveStart
    HalfEdge* backward = forward->twin;
    const double sForward = h == forward ? plan.sOwn : plan.sTwin;
    const double sBackward = h == forward ? plan.sTwin : plan.sOwn;

    auto [first, second] = pieceNames(solid, e);

    Vertex* v = solid.allocVertex();
    v->point = plan.point;
    v->tolerance = plan.vertexTolerance;
    if (second.isValid()) v->topoId = second.child("start", 0);

    Edge* rest = solid.allocEdge();
    e.topoId = std::move(first);
    rest->topoId = std::move(second);
    rest->exactCurve = e.exactCurve;
    rest->curveStart = plan.t;
    rest->curveEnd = e.curveEnd;
    rest->tolerance = e.tolerance;
    rest->analyticCurve = e.analyticCurve;
    rest->curve = plan.carrierSecond;
    e.curveEnd = plan.t;
    if (plan.carrierFirst) e.curve = plan.carrierFirst;

    HalfEdge* forwardOn = solid.allocHalfEdge();  // from v to the edge's end, beside forward
    forwardOn->origin = v;
    forwardOn->face = forward->face;
    forwardOn->edge = rest;
    forwardOn->pcurve = forward->pcurve;
    forwardOn->pcurveStart = sForward;
    forwardOn->pcurveEnd = forward->pcurveEnd;
    forward->pcurveEnd = sForward;

    HalfEdge* backwardOn = solid.allocHalfEdge();  // from v back to the edge's start
    backwardOn->origin = v;
    backwardOn->face = backward->face;
    backwardOn->edge = &e;
    backwardOn->pcurve = backward->pcurve;
    backwardOn->pcurveStart = sBackward;
    backwardOn->pcurveEnd = backward->pcurveEnd;
    backward->pcurveEnd = sBackward;
    backward->edge = rest;

    forwardOn->next = forward->next;
    forwardOn->next->prev = forwardOn;
    forward->next = forwardOn;
    forwardOn->prev = forward;
    backwardOn->next = backward->next;
    backwardOn->next->prev = backwardOn;
    backward->next = backwardOn;
    backwardOn->prev = backward;

    forward->twin = backwardOn;
    backwardOn->twin = forward;
    forwardOn->twin = backward;
    backward->twin = forwardOn;
    rest->halfEdge = forwardOn;
    v->halfEdge = forwardOn;
    return h == forward ? forwardOn : backwardOn;
}

/// Where @p loop meets the seam's line at @p x: the half-edge of its loop that will leave the
/// meeting point, found now where it is a vertex, else planned as a split into @p split.
std::string placeCrossing(Loop& loop, const Crossing& x, int dir,
                          const math::ModelTolerance& tolerance, HalfEdge*& leaving,
                          std::optional<Split>& split) {
    HalfEdge* h = loop.ring[x.index];
    leaving = nullptr;
    split.reset();
    if (x.atOrigin) {
        leaving = h;
        return {};
    }
    // The pcurve's parameter where its chain value is the line's: by bisection.
    const double shift = coord(loop.chain[x.index * kChords].shift, dir);
    const auto value = [&](double t) {
        return coord(h->pcurve->evaluate(t), dir) + shift - x.level;
    };
    double lo = x.lo;
    double hi = x.hi;
    if (lo != hi) {
        const bool rising = value(lo) < 0.0;
        for (int i = 0; i < kSteps; ++i) {
            const double mid = lo + (hi - lo) / 2.0;
            if (mid == lo || mid == hi) break;
            if ((value(mid) < 0.0) == rising) {
                lo = mid;
            } else {
                hi = mid;
            }
        }
    }
    Split plan;
    if (std::string why = planSplit(*h, lo, tolerance, plan); !why.empty()) return why;
    if (plan.at != nullptr) {
        leaving = plan.at == h->origin ? h : h->next;
        return {};
    }
    split = plan;
    return {};
}

// ---- the seam ----

/// The B-spline basis of @p degree on @p knots at @p t, for @p count control points: the index
/// of the first function that is not zero there, and the degree + 1 values from it.
std::pair<int, std::vector<double>> basisAt(const std::vector<double>& knots, int degree, int count,
                                            double t) {
    int span = degree;
    for (int i = degree; i < count; ++i) {
        if (knots[static_cast<std::size_t>(i)] <= t) span = i;
    }
    while (span > degree &&
           !(knots[static_cast<std::size_t>(span)] < knots[static_cast<std::size_t>(span) + 1])) {
        --span;
    }
    const auto p = static_cast<std::size_t>(degree);
    const auto k = static_cast<std::size_t>(span);
    std::vector<double> n(p + 1, 0.0);
    std::vector<double> left(p + 1, 0.0);
    std::vector<double> right(p + 1, 0.0);
    n[0] = 1.0;
    for (std::size_t j = 1; j <= p; ++j) {
        left[j] = t - knots[k + 1 - j];
        right[j] = knots[k + j] - t;
        double saved = 0.0;
        for (std::size_t r = 0; r < j; ++r) {
            const double temp = n[r] / (right[r + 1] + left[j - r]);
            n[r] = saved + right[r + 1] * temp;
            saved = left[j - r] * temp;
        }
        n[j] = saved;
    }
    return {span - degree, n};
}

/// The iso-curve of @p s where its parameter @p dir is @p at, exactly: the net's rows (or
/// columns) combined by the basis at @p at, in homogeneous coordinates, on the other
/// direction's knots. Null where a weight comes out not positive.
std::shared_ptr<const geo::Curve3> nurbsIsoCurve(const geo::NurbsSurface& s, int dir, double at) {
    const auto& net = s.controlPoints();
    const auto& weights = s.weights();
    const int count = dir == 0 ? s.controlPointCountU() : s.controlPointCountV();
    const int degree = dir == 0 ? s.degreeU() : s.degreeV();
    const double lo = dir == 0 ? s.uMin() : s.vMin();
    const double hi = dir == 0 ? s.uMax() : s.vMax();
    const double period = hi - lo;
    double t = at;
    if ((dir == 0 ? s.periodicU() : s.periodicV()) && period > 0.0) {
        t = lo + (t - lo) - period * std::floor((t - lo) / period);
    }
    t = std::clamp(t, lo, hi);
    const auto [first, basis] = basisAt(dir == 0 ? s.knotsU() : s.knotsV(), degree, count, t);
    const int across = dir == 0 ? s.controlPointCountV() : s.controlPointCountU();
    std::vector<Vec3> points;
    std::vector<double> w;
    for (int j = 0; j < across; ++j) {
        Vec3 sum;
        double weight = 0.0;
        for (std::size_t r = 0; r < basis.size(); ++r) {
            const auto i = static_cast<std::size_t>(first) + r;
            const auto jj = static_cast<std::size_t>(j);
            const Vec3& p = dir == 0 ? net[i][jj] : net[jj][i];
            const double wi = dir == 0 ? weights[i][jj] : weights[jj][i];
            sum = sum + p * (basis[r] * wi);
            weight += basis[r] * wi;
        }
        if (!(weight > 0.0) || !std::isfinite(weight)) return nullptr;
        points.push_back(sum / weight);
        w.push_back(weight);
    }
    return std::make_shared<geo::NurbsCurve>(std::move(points), std::move(w),
                                             dir == 0 ? s.knotsV() : s.knotsU(),
                                             dir == 0 ? s.degreeV() : s.degreeU());
}

/// The curve of @p s along which its parameter @p dir is @p at, parametrised by the other: a
/// cylinder's or a cone's generator, a sphere's or a torus's meridian, a torus's parallel, a
/// B-spline's exact iso-curve. Null where @p s has none this way.
std::shared_ptr<const geo::Curve3> isoCurve(const geo::Surface& s, int dir, double at) {
    const auto radial = [&](const geo::Frame& f) {
        return f.x * std::cos(at) + f.y * std::sin(at);
    };
    if (const auto* cyl = dynamic_cast<const geo::Cylinder*>(&s); cyl != nullptr && dir == 0) {
        return std::make_shared<geo::LineCurve>(s.evaluate(at, 0.0), cyl->frame().z,
                                                cyl->axialScale());
    }
    if (const auto* cone = dynamic_cast<const geo::Cone*>(&s); cone != nullptr && dir == 0) {
        const Vec3 d = radial(cone->frame()) * std::tan(cone->halfAngle()) + cone->frame().z;
        return std::make_shared<geo::LineCurve>(s.evaluate(at, 0.0), d,
                                                cone->axialScale() * d.length());
    }
    if (const auto* sphere = dynamic_cast<const geo::Sphere*>(&s); sphere != nullptr && dir == 0) {
        geo::Frame f;
        f.origin = sphere->frame().origin;
        f.x = radial(sphere->frame());
        f.y = sphere->frame().z;
        f.z = f.x.cross(f.y);
        return std::make_shared<geo::CircleCurve>(f, sphere->radius());
    }
    if (const auto* torus = dynamic_cast<const geo::Torus*>(&s); torus != nullptr) {
        const geo::Frame& t = torus->frame();
        geo::Frame f;
        if (dir == 0) {
            f.x = radial(t);
            f.origin = t.origin + f.x * torus->majorRadius();
            f.y = t.z;
            f.z = f.x.cross(f.y);
            return std::make_shared<geo::CircleCurve>(f, torus->minorRadius());
        }
        const double rho = torus->majorRadius() + torus->minorRadius() * std::cos(at);
        if (!(rho > 0.0)) return nullptr;
        f.origin = t.origin + t.z * (torus->minorRadius() * std::sin(at));
        f.x = t.x;
        f.y = t.y;
        f.z = t.z;
        return std::make_shared<geo::CircleCurve>(f, rho);
    }
    if (const auto* nurbs = dynamic_cast<const geo::NurbsSurface*>(&s); nurbs != nullptr) {
        return nurbsIsoCurve(*nurbs, dir, at);
    }
    return nullptr;
}

/// A carrier for a seam on @p curve from @p w0 to @p w1, between @p a and @p b: the line between
/// them, the circle's arc as a rational B-spline, or the B-spline's own part.
std::shared_ptr<geo::NurbsCurve> seamCarrier(const geo::Curve3& curve, double w0, double w1,
                                             const Vec3& a, const Vec3& b) {
    if (curve.kind() == geo::CurveKind::Line) {
        return std::make_shared<geo::NurbsCurve>(std::vector<Vec3>{a, b},
                                                 std::vector<double>{1.0, 1.0},
                                                 std::vector<double>{0.0, 0.0, 1.0, 1.0}, 1);
    }
    if (const auto* circle = dynamic_cast<const geo::CircleCurve*>(&curve); circle != nullptr) {
        const geo::NurbsCurve flat = geo::NurbsCurve::makeArc({0.0, 0.0, 0.0}, circle->radius(),
                                                              std::min(w0, w1), std::max(w0, w1));
        const geo::Frame& f = circle->frame();
        std::vector<Vec3> points;
        for (const Vec3& p : flat.controlPoints())
            points.push_back(f.origin + f.x * p.x + f.y * p.y);
        return std::make_shared<geo::NurbsCurve>(std::move(points), flat.weights(), flat.knots(),
                                                 flat.degree());
    }
    if (const auto* nurbs = dynamic_cast<const geo::NurbsCurve*>(&curve); nurbs != nullptr) {
        return std::make_shared<geo::NurbsCurve>(nurbs->segment(w0, w1));
    }
    return nullptr;
}

/// @p curve run backwards over the same domain: its points and weights reversed, its knots
/// mirrored.
geo::NurbsCurve reversedCurve(const geo::NurbsCurve& curve) {
    std::vector<Vec3> points(curve.controlPoints().rbegin(), curve.controlPoints().rend());
    std::vector<double> weights(curve.weights().rbegin(), curve.weights().rend());
    const std::vector<double>& knots = curve.knots();
    std::vector<double> mirrored;
    mirrored.reserve(knots.size());
    for (auto k = knots.rbegin(); k != knots.rend(); ++k) {
        mirrored.push_back(knots.front() + knots.back() - *k);
    }
    return {std::move(points), std::move(weights), std::move(mirrored), curve.degree()};
}

/// Turns the carrier of @p e, a closed edge, to run the way the edge does (its curve from
/// curveStart to curveEnd). A closed carrier's ends cannot say which way round it runs, so the
/// faceted copy samples one its own way: a seamed loop is one rectangle there, whose sides must
/// all run the way the loop does, where a band's rims could each run either way.
void carrierWithItsEdge(Edge& e) {
    if (!e.curve || !e.exactCurve || e.halfEdge == nullptr || e.halfEdge->twin == nullptr ||
        e.halfEdge->origin != e.halfEdge->twin->origin) {
        return;
    }
    const Vec3 along =
        e.exactCurve->derivatives(e.curveStart).d1 * (e.curveEnd >= e.curveStart ? 1.0 : -1.0);
    const Vec3 carried = e.curve->derivative(e.curve->tMin(), 1);
    if (along.dot(carried) < 0.0)
        e.curve = std::make_shared<geo::NurbsCurve>(reversedCurve(*e.curve));
}

/// topo::joinAllowance for each half-edge of @p ring, joining the next in the ring: for a ring
/// not linked yet.
std::vector<double> allowancesOf(const std::vector<const HalfEdge*>& ring,
                                 const math::ModelTolerance& tolerance) {
    std::vector<double> out;
    out.reserve(ring.size());
    for (std::size_t k = 0; k < ring.size(); ++k) {
        out.push_back(topo::joinAllowance(*ring[k], *ring[(k + 1) % ring.size()], tolerance));
    }
    return out;
}

/// Lays the seam of @p face along parameter @p dir, joining @p a and @p b where they cross its
/// line, as @p xa and @p xb find. Empty on success; else why.
std::string laySeam(Solid& solid, Face& face, int dir, double period, Loop& a, const Crossing& xa,
                    Loop& b, const Crossing& xb, const math::ModelTolerance& tolerance) {
    const geo::Surface& s = *face.exactSurface;
    HalfEdge* ha = nullptr;
    HalfEdge* hb = nullptr;
    std::optional<Split> splitA;
    std::optional<Split> splitB;
    // The chain's value at the crossing, in the raw (u, v) of the half-edge it is on.
    const double levelA = xa.level - coord(a.chain[xa.index * kChords].shift, dir);
    if (std::string why = placeCrossing(a, xa, dir, tolerance, ha, splitA); !why.empty()) {
        return why;
    }
    if (std::string why = placeCrossing(b, xb, dir, tolerance, hb, splitB); !why.empty()) {
        return why;
    }
    if (splitA) ha = applySplit(solid, *splitA);
    if (splitB) hb = applySplit(solid, *splitB);
    Vertex* va = ha->origin;
    Vertex* vb = hb->origin;
    if (va == vb || (va->point - vb->point).length() <=
                        tolerance.effective(std::max(va->tolerance, vb->tolerance))) {
        return kNoSeam + "its two loops meet where its seam would run";
    }

    const std::size_t limit = 2 * solid.edges().size() + 1;
    const std::vector<HalfEdge*> ringA = ringFrom(face, ha, limit);
    const std::vector<HalfEdge*> ringB = ringFrom(face, hb, limit);
    if (ringA.empty() || ringB.empty()) return kNoSeam + "a loop it joins does not close";

    // The seam's line, at the crossing's value where it leaves a's start, and a's winding on.
    const double atNear = levelA;
    const double atFar = levelA + a.winds[static_cast<std::size_t>(dir)] * period;
    const int other = 1 - dir;
    const double w0 = coord(ha->pcurve->evaluate(ha->pcurveStart), other);
    double w1 = coord(hb->pcurve->evaluate(hb->pcurveStart), other);
    const auto curve = isoCurve(s, dir, atNear);
    if (curve == nullptr) return kNoSeam + "its surface gives no exact curve along a seam";

    // How far its line passes from the vertices it joins: bounded here, and measured for the
    // seam's tolerance once its pcurves are laid.
    const auto onLine = [&](double w) {
        const Vec2 uv = isoPoint(dir, atNear, w);
        return s.evaluate(uv.x, uv.y);
    };
    const double gapA = (onLine(w0) - va->point).length();
    const double gapB = (onLine(w1) - vb->point).length();
    double near = std::max(va->tolerance, vb->tolerance);
    for (const HalfEdge* h : {ha, ha->prev, hb, hb->prev})
        near = std::max(near, h->edge->tolerance);
    if (!(std::max(gapA, gapB) <= 4.0 * tolerance.effective(near))) {
        return kNoSeam + "its seam's line passes " + std::to_string(std::max(gapA, gapB)) +
               " from a vertex it joins";
    }
    Edge probe;
    probe.tolerance = std::max(gapA, gapB) > tolerance.linear ? std::max(gapA, gapB) : 0.0;

    // Which way round: where the other parameter wraps too, the seam can run either way from
    // a to b, and runs the way that leaves the face on the side its sense says.
    const double otherPeriod = periodOf(s, other);
    std::vector<double> ends{w1};
    if (otherPeriod > 0.0) {
        const double up = w1 + otherPeriod * std::ceil((w0 - w1) / otherPeriod);
        ends = {up == w0 ? up + otherPeriod : up, (up == w0 ? up + otherPeriod : up) - otherPeriod};
    }
    std::optional<double> chosen;
    for (const double end : ends) {
        HalfEdge probeUp;
        probeUp.origin = va;
        probeUp.edge = &probe;
        probeUp.pcurve =
            std::make_shared<geo::Line2D>(isoPoint(dir, atFar, w0), isoPoint(dir, atFar, end));
        probeUp.pcurveEnd = 1.0;
        HalfEdge probeDown;
        probeDown.origin = vb;
        probeDown.edge = &probe;
        probeDown.pcurve =
            std::make_shared<geo::Line2D>(isoPoint(dir, atNear, end), isoPoint(dir, atNear, w0));
        probeDown.pcurveEnd = 1.0;
        std::vector<const HalfEdge*> ring(ringA.begin(), ringA.end());
        ring.push_back(&probeUp);
        ring.insert(ring.end(), ringB.begin(), ringB.end());
        ring.push_back(&probeDown);
        topo::LoopPolygon chain;
        if (!topo::unwrapLoop(ring, topo::kLoopSamples, s, topo::uPeriod(s), topo::vPeriod(s),
                              allowancesOf(ring, tolerance), chain)) {
            continue;
        }
        const double area = topo::signedArea(chain);
        if (!std::isfinite(area) || area == 0.0) continue;
        if (otherPeriod > 0.0 && (area > 0.0) == face.reversed) continue;
        chosen = end;
        break;
    }
    if (!chosen) {
        return kNoSeam + (otherPeriod > 0.0 ? "neither way round does its seam leave the face on "
                                              "the side it faces"
                                            : "its loops do not close with a seam between them");
    }
    w1 = *chosen;

    // A B-spline's iso-curve runs over its knots, not round: the seam's range must lie in them.
    double from = w0;
    double to = w1;
    if (const auto* nurbs = dynamic_cast<const geo::NurbsCurve*>(curve.get()); nurbs != nullptr) {
        const double slop = 1e-9 * (nurbs->tMax() - nurbs->tMin());
        if (otherPeriod > 0.0) {
            const double shift =
                otherPeriod * std::ceil((nurbs->tMin() - slop - std::min(from, to)) / otherPeriod);
            from += shift;
            to += shift;
        }
        if (std::min(from, to) < nurbs->tMin() - slop ||
            std::max(from, to) > nurbs->tMax() + slop) {
            return kNoSeam + "its seam would cross where its surface closes";
        }
        from = std::clamp(from, nurbs->tMin(), nurbs->tMax());
        to = std::clamp(to, nurbs->tMin(), nurbs->tMax());
    }
    // The curve is the surface's own iso-curve, to rounding, all along the seam.
    for (int k = 0; k <= 8; ++k) {
        const double w = from + (to - from) * k / 8.0;
        const Vec2 uv = isoPoint(dir, atNear, w);
        if (!((curve->evaluate(w) - s.evaluate(uv.x, uv.y)).length() <= tolerance.linear)) {
            return kNoSeam + "its surface gives no exact curve along a seam";
        }
    }
    auto carrier = seamCarrier(*curve, from, to, va->point, vb->point);

    // Its sides' pcurves, and its tolerance: how far their ends, sampled where the validator
    // samples them, and its curve's ends lie from the vertices it joins.
    const std::shared_ptr<const geo::Curve2D> upCurve =
        std::make_shared<geo::Line2D>(isoPoint(dir, atFar, from), isoPoint(dir, atFar, to));
    const std::shared_ptr<const geo::Curve2D> downCurve =
        std::make_shared<geo::Line2D>(isoPoint(dir, atNear, to), isoPoint(dir, atNear, from));
    double gap = std::max((curve->evaluate(from) - va->point).length(),
                          (curve->evaluate(to) - vb->point).length());
    const auto endsOf = [&](const std::shared_ptr<const geo::Curve2D>& pcurve, const Vertex& first,
                            const Vertex& last) {
        HalfEdge side;
        side.pcurve = pcurve;
        side.pcurveEnd = 1.0;
        for (const int i : {0, topo::kLoopSamples - 1}) {
            const Vec2 uv =
                pcurve->evaluate(topo::loopSampleParameter(side, i, topo::kLoopSamples));
            const Vertex& v = i == 0 ? first : last;
            gap = std::max(gap, (s.evaluate(uv.x, uv.y) - v.point).length());
        }
    };
    endsOf(upCurve, *va, *vb);
    endsOf(downCurve, *vb, *va);

    // Joined: a from its crossing round to it, up the seam's far side to b, b round, and down
    // its near side back to a's start.
    HalfEdge* up = solid.allocHalfEdge();
    HalfEdge* down = solid.allocHalfEdge();
    Edge* seam = solid.allocEdge();
    seam->topoId = face.topoId.child("seam", dir);
    seam->halfEdge = up;
    seam->exactCurve = curve;
    seam->curveStart = from;
    seam->curveEnd = to;
    seam->tolerance = gap > tolerance.linear ? gap : 0.0;
    seam->curve = std::move(carrier);
    up->origin = va;
    down->origin = vb;
    up->twin = down;
    down->twin = up;
    up->edge = down->edge = seam;
    up->face = down->face = &face;
    up->pcurve = upCurve;
    up->pcurveEnd = 1.0;
    down->pcurve = downCurve;
    down->pcurveEnd = 1.0;
    HalfEdge* beforeA = ringA.back();
    HalfEdge* beforeB = ringB.back();
    beforeA->next = up;
    up->prev = beforeA;
    up->next = hb;
    hb->prev = up;
    beforeB->next = down;
    down->prev = beforeB;
    down->next = ha;
    ha->prev = down;

    for (const std::vector<HalfEdge*>* ring : {&ringA, &ringB}) {
        for (HalfEdge* h : *ring) carrierWithItsEdge(*h->edge);
    }

    Wire* outer = face.outerLoop;
    a.wire->halfEdge = ha;
    face.outerLoop = a.wire;
    auto& inner = face.innerLoops;
    inner.erase(std::remove_if(inner.begin(), inner.end(),
                               [&](const Wire* w) { return w == a.wire || w == b.wire; }),
                inner.end());
    if (outer != nullptr && outer != a.wire) inner.push_back(outer);
    return {};
}

/// Gives @p face, whose @p loops wind round its surface along parameter @p dir only, a seam that
/// way, joining the two that wind.
std::string seamAlong(Solid& solid, Face& face, int dir, std::vector<Loop>& loops,
                      const math::ModelTolerance& tolerance) {
    const geo::Surface& s = *face.exactSurface;
    const double period = periodOf(s, dir);
    std::vector<std::size_t> winding;
    for (std::size_t k = 0; k < loops.size(); ++k) {
        if (loops[k].winds[static_cast<std::size_t>(dir)] != 0.0) winding.push_back(k);
    }
    if (winding.empty()) return {};
    if (winding.size() == 1) {
        return kNoSeam +
               "one of its loops winds round it and no other does, so a seam has "
               "nothing to join it to";
    }
    if (winding.size() > 2) {
        return kNoSeam + std::to_string(winding.size()) +
               " of its loops wind round it, and a seam joins two";
    }
    // a is the outer loop where it winds (u0 is its first vertex's), else the first that does.
    Loop& a = loops[winding[0]];
    Loop& b = loops[winding[1]];
    const double wa = a.winds[static_cast<std::size_t>(dir)];
    const double wb = b.winds[static_cast<std::size_t>(dir)];
    if (std::fabs(wa) != 1.0 || std::fabs(wb) != 1.0) {
        return kNoSeam + "a loop winds round it more than once";
    }
    if (wa == wb) return kNoSeam + "its two loops wind round it the same way";
    std::vector<const Loop*> others;
    for (std::size_t k = 0; k < loops.size(); ++k) {
        if (k != winding[0] && k != winding[1]) others.push_back(&loops[k]);
    }
    for (const double c : seamLines(a, b, others, dir, period)) {
        if (!clearOf(c, others, dir, period)) continue;
        const auto xa = crossingsOf(a, s, dir, c, period);
        const auto xb = crossingsOf(b, s, dir, c, period);
        if (xa.size() != 1 || xb.size() != 1) continue;
        return laySeam(solid, face, dir, period, a, xa.front(), b, xb.front(), tolerance);
    }
    return kNoSeam +
           "every line round it a seam could take crosses one of its loops more than "
           "once, or runs through a hole";
}

/// The u, in @p loop's unwrapped chain, of where it meets a line of v at @p x: at a vertex, its
/// value there; inside a pcurve, the pcurve's at the split @p split plans; at an end vertex the
/// split fell on (@p leaving, from placeCrossing), that vertex's.
double crossingU(const Loop& loop, const Crossing& x, const std::optional<Split>& split,
                 const HalfEdge* leaving) {
    const std::size_t first = x.index * kChords;
    if (split) {
        return loop.ring[x.index]->pcurve->evaluate(split->sOwn).x + loop.chain[first].shift.x;
    }
    if (x.atOrigin || leaving == loop.ring[x.index]) return loop.chain[first].a.x;
    return loop.chain[first + kChords - 1].b.x;
}

/// Cuts @p face, a band whose loops wound round its surface both ways, seamed along u (its loop
/// spans a period in u and more than one in v), in two along the iso-curve v = c that halves its
/// span in v: each piece then spans less than a period each way, as every face's region must.
/// The cut is a new edge between the two (topo::euler::makeEdgeFace), on the surface's own
/// iso-curve, as a seam is; the pieces share the band's surfaces and sense, and are its pieces,
/// `<face>/piece:0` and `/piece:1`; the cut is `<face>/seam:1`. Empty on success; else why.
std::string cutAcross(Solid& solid, Face& face, const math::ModelTolerance& tolerance) {
    const geo::Surface& s = *face.exactSurface;
    const double vPeriod = periodOf(s, 1);
    auto read = loopsOf(solid, face, tolerance);
    if (!read || read->size() != 1) return kNoSeam + "its loop does not close with its seam";
    Loop& loop = read->front();
    const std::pair<double, double> span = spanOf(loop, 1);
    const double c = (span.first + span.second) / 2.0;
    const std::vector<Crossing> xs = crossingsOf(loop, s, 1, c, vPeriod);
    if (xs.size() != 2) {
        return kNoSeam + "the line across it that halves it meets its loop " +
               (xs.size() < 2 ? "less" : "more") + " than twice";
    }
    std::array<HalfEdge*, 2> leaving{};
    std::array<std::optional<Split>, 2> splits;
    std::array<double, 2> at{};
    for (std::size_t k = 0; k < 2; ++k) {
        if (loop.ring[xs[k].index]->twin->face == &face) {
            return kNoSeam + "the line across it that halves it runs along its seam";
        }
        if (std::string why = placeCrossing(loop, xs[k], 1, tolerance, leaving[k], splits[k]);
            !why.empty()) {
            return why;
        }
        at[k] = crossingU(loop, xs[k], splits[k], leaving[k]);
    }
    // Each read through a reference: an element of an array of optionals is not one the
    // unchecked-optional-access check can tie a test to.
    const std::optional<Split>& first = splits[0];
    const std::optional<Split>& second = splits[1];
    if (first && second && first->h->edge == second->h->edge) {
        return kNoSeam + "the line across it that halves it meets one edge twice";
    }
    for (std::size_t k = 0; k < 2; ++k) {
        const std::optional<Split>& split = splits[k];
        if (split) leaving[k] = applySplit(solid, *split);
    }
    Vertex* va = leaving[0]->origin;
    Vertex* vb = leaving[1]->origin;
    if (va == vb || (va->point - vb->point).length() <=
                        tolerance.effective(std::max(va->tolerance, vb->tolerance))) {
        return kNoSeam + "its two loops meet where it would be cut";
    }

    // The cut's curve, the surface's own iso-curve all along it (a B-spline's within its knots).
    const auto curve = isoCurve(s, 1, c);
    if (curve == nullptr) return kNoSeam + "its surface gives no exact curve across it";
    double from = at[0];
    double to = at[1];
    if (const auto* nurbs = dynamic_cast<const geo::NurbsCurve*>(curve.get()); nurbs != nullptr) {
        const double uPeriod = periodOf(s, 0);
        const double slop = 1e-9 * (nurbs->tMax() - nurbs->tMin());
        const double shift =
            uPeriod * std::ceil((nurbs->tMin() - slop - std::min(from, to)) / uPeriod);
        from += shift;
        to += shift;
        if (std::max(from, to) > nurbs->tMax() + slop) {
            return kNoSeam + "the line across it would cross where its surface closes";
        }
        from = std::clamp(from, nurbs->tMin(), nurbs->tMax());
        to = std::clamp(to, nurbs->tMin(), nurbs->tMax());
    }
    for (int k = 0; k <= 8; ++k) {
        const double u = at[0] + (at[1] - at[0]) * k / 8.0;
        const double w = from + (to - from) * k / 8.0;
        if (!((curve->evaluate(w) - s.evaluate(u, c)).length() <= tolerance.linear)) {
            return kNoSeam + "its surface gives no exact curve across it";
        }
    }

    // Its sides' pcurves, and its tolerance: how far their ends, sampled where the validator
    // samples them, and its curve's ends lie from the vertices it joins.
    const std::shared_ptr<const geo::Curve2D> along =
        std::make_shared<geo::Line2D>(Vec2{at[0], c}, Vec2{at[1], c});
    const std::shared_ptr<const geo::Curve2D> back =
        std::make_shared<geo::Line2D>(Vec2{at[1], c}, Vec2{at[0], c});
    double gap = std::max((curve->evaluate(from) - va->point).length(),
                          (curve->evaluate(to) - vb->point).length());
    gap = std::max({gap, (s.evaluate(at[0], c) - va->point).length(),
                    (s.evaluate(at[1], c) - vb->point).length()});
    double near = std::max(va->tolerance, vb->tolerance);
    for (const HalfEdge* h : {leaving[0], leaving[0]->prev, leaving[1], leaving[1]->prev}) {
        near = std::max(near, h->edge->tolerance);
    }
    if (!(gap <= 4.0 * tolerance.effective(near))) {
        return kNoSeam + "the line across it passes " + std::to_string(gap) +
               " from a vertex it joins";
    }

    // Cut: the face keeps the side from b's crossing round to a's, the new face the rest.
    const topo::TopologyID whole = face.topoId;
    const topo::MEFResult made = topo::euler::makeEdgeFace(solid, leaving[0], leaving[1]);
    Edge* cut = made.edge;
    Face* piece = made.newFace;
    if (whole.isValid()) cut->topoId = whole.child("seam", 1);
    cut->exactCurve = curve;
    cut->curveStart = from;
    cut->curveEnd = to;
    cut->tolerance = gap > tolerance.linear ? gap : 0.0;
    cut->curve = seamCarrier(*curve, from, to, va->point, vb->point);
    HalfEdge* forward = cut->halfEdge;  // from a's crossing to b's, on the face
    forward->pcurve = along;
    forward->pcurveStart = 0.0;
    forward->pcurveEnd = 1.0;
    forward->twin->pcurve = back;
    forward->twin->pcurveStart = 0.0;
    forward->twin->pcurveEnd = 1.0;
    piece->exactSurface = face.exactSurface;
    piece->surface = face.surface;
    piece->analyticSurface = face.analyticSurface;
    piece->reversed = face.reversed;
    if (whole.isValid()) {
        face.topoId = whole.child("piece", 0);
        piece->topoId = whole.child("piece", 1);
    }
    return {};
}

/// Gives @p face, a band whose two loops each wind once round its surface both ways and
/// opposite ways (a helical band on a torus), a seam along u, which leaves its region over more
/// than a period in v, then cuts it in two across that (cutAcross).
std::string seamAndCut(Solid& solid, Face& face, std::vector<Loop>& loops,
                       const math::ModelTolerance& tolerance) {
    if (loops.size() != 2) {
        return kNoSeam +
               "a band whose loops wind round it both ways is cut in two only where "
               "it has no hole";
    }
    const std::array<double, 2>& wa = loops[0].winds;
    const std::array<double, 2>& wb = loops[1].winds;
    if (std::fabs(wa[0]) != 1.0 || std::fabs(wa[1]) != 1.0 || wb[0] != -wa[0] || wb[1] != -wa[1]) {
        return kNoSeam + "its two loops do not each wind once round it both ways, opposite ways";
    }
    if (std::string why = seamAlong(solid, face, 0, loops, tolerance); !why.empty()) return why;
    return cutAcross(solid, face, tolerance);
}

}  // namespace

std::string insertSeams(Solid& solid, Face& face, const math::ModelTolerance& tolerance) {
    if (!face.alive || !face.exactSurface) return {};
    const geo::Surface& s = *face.exactSurface;
    if (!(topo::uPeriod(s) > 0.0) && !(topo::vPeriod(s) > 0.0)) return {};
    if (hasSeam(solid, face)) return {};
    try {
        auto loops = loopsOf(solid, face, tolerance);
        if (!loops) return {};
        // The way its loops wind round, if any: each the one way (a band, seamed that way), or
        // each both ways (a helical band on a torus, seamed along u and cut in two along v).
        int dir = -1;
        bool both = false;
        for (const Loop& loop : *loops) {
            const bool u = loop.winds[0] != 0.0;
            const bool v = loop.winds[1] != 0.0;
            if (!u && !v) continue;
            const int way = u && v ? 2 : (u ? 0 : 1);
            if (dir >= 0 && dir != way) {
                return "its loops wind round the surface different ways, which no seam this "
                       "stage lays can open";
            }
            dir = way;
            both = way == 2;
        }
        if (dir < 0) return {};
        if (both) return seamAndCut(solid, face, *loops, tolerance);
        return seamAlong(solid, face, dir, *loops, tolerance);
    } catch (const std::exception& e) {
        return kNoSeam + e.what();
    }
}

std::string insertSeams(Solid& solid, const math::ModelTolerance& tolerance) {
    // By index: a band cut in two adds a face, and a deque's iterators do not outlive that.
    for (std::size_t k = 0; k < solid.faces().size(); ++k) {
        if (std::string why = insertSeams(solid, solid.faces()[k], tolerance); !why.empty()) {
            return why;
        }
    }
    return {};
}

std::vector<Face*> turnBackwardRims(Solid& solid, const math::ModelTolerance& tolerance) {
    // The rims, and what each face's loops say of them.
    std::vector<Edge*> rims;
    std::unordered_map<const HalfEdge*, std::size_t> rimAt;
    for (Edge& e : solid.edges()) {
        if (!isRim(e)) continue;
        rimAt[e.halfEdge] = rimAt[e.halfEdge->twin] = rims.size();
        rims.push_back(&e);
    }
    if (rims.empty()) return {};
    const auto rimOf = [&rimAt](const Loop& loop) {
        if (loop.ring.size() != 1) return kNotARim;
        const auto it = rimAt.find(loop.ring.front());
        return it == rimAt.end() ? kNotARim : it->second;
    };
    std::vector<Tie> ties;
    std::unordered_set<const Face*> seen;
    for (const Edge* e : rims) {
        for (const Face* f : {e->halfEdge->face, e->halfEdge->twin->face}) {
            if (seen.insert(f).second) tiesOf(solid, *f, tolerance, rimOf, ties);
        }
    }

    // Each rim tied to others, and each held to a loop taken as written: whether it is turned.
    struct Link {
        std::size_t to = 0;
        bool differ = false;
    };
    std::vector<std::vector<Link>> links(rims.size());
    std::vector<std::vector<bool>> held(rims.size());  // turned exactly where true
    for (const Tie& t : ties) {
        if (t.a == kNotARim && t.b == kNotARim) continue;  // nothing to turn: insertSeams says
        if (t.a == kNotARim || t.b == kNotARim) {
            held[t.a == kNotARim ? t.b : t.a].push_back(t.differ);
        } else {
            links[t.a].push_back({t.b, t.differ});
            links[t.b].push_back({t.a, t.differ});
        }
    }

    // Each set of rims tied together, turned as its ties say: one way round where a rim is held,
    // else the fewer turns; none where its ties cannot all hold, or as many could turn either way.
    std::vector<int> turn(rims.size(), -1);
    std::vector<Face*> turned;
    std::unordered_set<Face*> listed;
    for (std::size_t first = 0; first < rims.size(); ++first) {
        if (turn[first] >= 0) continue;
        std::vector<std::size_t> group{first};
        turn[first] = 0;
        bool holds = true;
        for (std::size_t i = 0; i < group.size(); ++i) {
            const std::size_t r = group[i];
            for (const Link& l : links[r]) {
                const int want = turn[r] ^ (l.differ ? 1 : 0);
                if (turn[l.to] < 0) {
                    turn[l.to] = want;
                    group.push_back(l.to);
                } else if (turn[l.to] != want) {
                    holds = false;
                }
            }
        }
        // Flipping every rim of the group satisfies the same ties: which of the two is decided by
        // the rims held as written, else by the fewer turns (-1 until one is).
        int flip = -1;
        std::size_t turns = 0;
        for (const std::size_t r : group) {
            turns += static_cast<std::size_t>(turn[r]);
            for (const bool h : held[r]) {
                const int want = turn[r] ^ (h ? 1 : 0);
                if (flip >= 0 && flip != want) holds = false;
                flip = want;
            }
        }
        if (!holds) continue;
        if (flip < 0) {
            if (2 * turns == group.size()) continue;  // as many either way: which cannot be told
            flip = 2 * turns > group.size() ? 1 : 0;
        }
        for (const std::size_t r : group) {
            if ((turn[r] ^ flip) == 0) continue;
            turnRim(*rims[r]);
            for (Face* f : {rims[r]->halfEdge->face, rims[r]->halfEdge->twin->face}) {
                if (listed.insert(f).second) turned.push_back(f);
            }
        }
    }
    return turned;
}

}  // namespace hz::model
