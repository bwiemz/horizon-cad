#include "horizon/fileio/StepFormat.h"

#include <algorithm>
#include <atomic>
#include <cctype>
#include <charconv>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <functional>
#include <iomanip>
#include <limits>
#include <map>
#include <optional>
#include <set>
#include <sstream>
#include <string_view>
#include <system_error>
#include <unordered_map>
#include <unordered_set>
#include <variant>

#include "horizon/fileio/AtomicFile.h"
#include "horizon/fileio/ImportReport.h"
#include "horizon/geometry/curves/NurbsCurve.h"
#include "horizon/geometry/surfaces/NurbsSurface.h"
#include "horizon/math/Constants.h"
#include "horizon/math/Mat4.h"
#include "horizon/math/Vec3.h"
#include "horizon/modeling/Faceting.h"
#include "horizon/modeling/MateGeometry.h"
#include "horizon/modeling/Pattern.h"

namespace hz::io {

using hz::math::Mat4;
using hz::math::Vec3;

namespace {

thread_local std::string g_lastError;

/// lastError() after a read stopped by its cancel flag.
constexpr const char* kCancelled = "cancelled";

constexpr double kMergeTol = 1e-9;

/// Piecewise degree-2 rational arc in an explicit placement frame.
/// Angles are measured from @p xAxis toward @p yAxis around the frame normal.
/// Unlike NurbsCurve::makeArc this takes the frame verbatim, which STEP
/// placements require.
std::shared_ptr<geo::NurbsCurve> makeArcInFrame(const Vec3& center, double radius, double a0,
                                                double a1, const Vec3& xAxis, const Vec3& yAxis) {
    double sweep = a1 - a0;
    while (sweep <= 1e-12) sweep += math::kTwoPi;
    while (sweep > math::kTwoPi + 1e-12) sweep -= math::kTwoPi;

    const int numSegments = std::max(1, static_cast<int>(std::ceil(sweep / math::kHalfPi - 1e-9)));
    const double segSweep = sweep / numSegments;
    const double wMid = std::cos(segSweep / 2.0);

    auto at = [&](double a, double scale) {
        return center + (xAxis * std::cos(a) + yAxis * std::sin(a)) * (radius * scale);
    };

    std::vector<Vec3> cps;
    std::vector<double> weights;
    cps.push_back(at(a0, 1.0));
    weights.push_back(1.0);
    double angle = a0;
    for (int seg = 0; seg < numSegments; ++seg) {
        const double mid = angle + segSweep / 2.0;
        const double end = angle + segSweep;
        cps.push_back(at(mid, 1.0 / wMid));
        weights.push_back(wMid);
        cps.push_back(at(end, 1.0));
        weights.push_back(1.0);
        angle = end;
    }

    std::vector<double> knots{0.0, 0.0, 0.0};
    for (int seg = 1; seg < numSegments; ++seg) {
        const double k = static_cast<double>(seg) / numSegments;
        knots.push_back(k);
        knots.push_back(k);
    }
    knots.insert(knots.end(), {1.0, 1.0, 1.0});

    return std::make_shared<geo::NurbsCurve>(std::move(cps), std::move(weights), std::move(knots),
                                             2);
}

/// Reverse a surface's U direction: rows and weights reversed, U knot vector
/// mirrored. Flips the surface normal while describing identical geometry —
/// used to honour ADVANCED_FACE same_sense = .F. on import (the kernel's
/// convention is surface normal == outward face normal).
std::shared_ptr<geo::NurbsSurface> reverseSurfaceU(const geo::NurbsSurface& s) {
    auto cps = s.controlPoints();
    auto wts = s.weights();
    std::reverse(cps.begin(), cps.end());
    std::reverse(wts.begin(), wts.end());
    const auto& k = s.knotsU();
    const double lo = k.front();
    const double hi = k.back();
    std::vector<double> rk(k.rbegin(), k.rend());
    for (double& v : rk) v = lo + hi - v;
    return std::make_shared<geo::NurbsSurface>(std::move(cps), std::move(wts), std::move(rk),
                                               s.knotsV(), s.degreeU(), s.degreeV());
}

// ===========================================================================
// Writer
// ===========================================================================

/// Parse a Part-21 real. std::from_chars ignores the C locale (strtod read
/// "1.5" as 1 under a comma-decimal locale) and must consume the whole token,
/// so "1-2" is an error rather than a silent 1.
bool parseReal(std::string_view text, double& out) {
    if (!text.empty() && text.front() == '+') text.remove_prefix(1);  // from_chars rejects '+'
    const char* last = text.data() + text.size();
    const auto [ptr, ec] = std::from_chars(text.data(), last, out);
    return ec == std::errc{} && ptr == last;
}

/// Format a real in Part-21 syntax: the mantissa always contains a '.'
/// (ISO 10303-21 clause 6.4.2 — "1E-07" is invalid, "1.E-07" is required).
///
/// std::to_chars rather than printf: it ignores the C locale — which Qt sets
/// from the environment on Unix, so "%g" wrote "1,5" under de_DE — and gives
/// the shortest text that reads back as exactly the same double.
std::string fmtReal(double v) {
    if (!std::isfinite(v)) return "0.";
    char buf[64];
    const auto [end, ec] = std::to_chars(buf, buf + sizeof(buf), v);
    if (ec != std::errc{}) return "0.";
    std::string s(buf, end);
    // Part-21 uses upper-case E for exponents.
    std::replace(s.begin(), s.end(), 'e', 'E');
    if (s.find('.') == std::string::npos) {
        const size_t e = s.find('E');
        if (e == std::string::npos) {
            s += '.';
        } else {
            s.insert(e, ".");
        }
    }
    return s;
}

/// Group a raw knot vector into (unique knots, multiplicities).
void groupKnots(const std::vector<double>& raw, std::vector<double>& knots,
                std::vector<int>& mults) {
    knots.clear();
    mults.clear();
    for (double k : raw) {
        if (!knots.empty() && std::abs(k - knots.back()) < 1e-12) {
            ++mults.back();
        } else {
            knots.push_back(k);
            mults.push_back(1);
        }
    }
}

bool allUnitWeights(const std::vector<double>& w) {
    return std::all_of(w.begin(), w.end(), [](double x) { return std::abs(x - 1.0) < 1e-12; });
}

/// Incremental Part-21 DATA-section builder with monotonically increasing ids.
class StepWriter {
public:
    /// Add an instance; @p rhs is everything right of "#id = ". Returns the id.
    int add(const std::string& rhs) {
        m_body << '#' << m_next << " = " << rhs << ";\n";
        return m_next++;
    }

    int addPoint(const Vec3& p) {
        return add("CARTESIAN_POINT('',(" + fmtReal(p.x) + "," + fmtReal(p.y) + "," + fmtReal(p.z) +
                   "))");
    }

    static std::string refList(const std::vector<int>& ids) {
        std::string s = "(";
        for (size_t i = 0; i < ids.size(); ++i) {
            if (i) s += ',';
            s += '#' + std::to_string(ids[i]);
        }
        return s + ")";
    }

    static std::string realList(const std::vector<double>& vals) {
        std::string s = "(";
        for (size_t i = 0; i < vals.size(); ++i) {
            if (i) s += ',';
            s += fmtReal(vals[i]);
        }
        return s + ")";
    }

    static std::string intList(const std::vector<int>& vals) {
        std::string s = "(";
        for (size_t i = 0; i < vals.size(); ++i) {
            if (i) s += ',';
            s += std::to_string(vals[i]);
        }
        return s + ")";
    }

    std::string body() const { return m_body.str(); }
    int nextId() const { return m_next; }

private:
    std::ostringstream m_body;
    int m_next = 1;
};

int writeCurve(StepWriter& w, const geo::NurbsCurve& c) {
    std::vector<int> cpIds;
    cpIds.reserve(c.controlPoints().size());
    for (const Vec3& p : c.controlPoints()) cpIds.push_back(w.addPoint(p));

    std::vector<double> knots;
    std::vector<int> mults;
    groupKnots(c.knots(), knots, mults);

    const std::string common =
        std::to_string(c.degree()) + "," + StepWriter::refList(cpIds) + ",.UNSPECIFIED.,.F.,.F.";
    const std::string knotPart =
        StepWriter::intList(mults) + "," + StepWriter::realList(knots) + ",.UNSPECIFIED.";

    if (allUnitWeights(c.weights())) {
        return w.add("B_SPLINE_CURVE_WITH_KNOTS(''," + common + "," + knotPart + ")");
    }
    // Rational curves need the Part-21 complex (multi-leaf) instance form.
    return w.add("(BOUNDED_CURVE() B_SPLINE_CURVE(" + common + ") B_SPLINE_CURVE_WITH_KNOTS(" +
                 knotPart + ") CURVE() GEOMETRIC_REPRESENTATION_ITEM() RATIONAL_B_SPLINE_CURVE(" +
                 StepWriter::realList(c.weights()) + ") REPRESENTATION_ITEM(''))");
}

int writeSurface(StepWriter& w, const geo::NurbsSurface& s) {
    // Control net rows: STEP lists control points as a list of lists [u][v].
    std::string net = "(";
    bool rational = false;
    std::string weights = "(";
    for (size_t iu = 0; iu < s.controlPoints().size(); ++iu) {
        if (iu) {
            net += ',';
            weights += ',';
        }
        std::vector<int> row;
        for (const Vec3& p : s.controlPoints()[iu]) row.push_back(w.addPoint(p));
        net += StepWriter::refList(row);
        weights += StepWriter::realList(s.weights()[iu]);
        if (!allUnitWeights(s.weights()[iu])) rational = true;
    }
    net += ")";
    weights += ")";

    std::vector<double> knotsU, knotsV;
    std::vector<int> multsU, multsV;
    groupKnots(s.knotsU(), knotsU, multsU);
    groupKnots(s.knotsV(), knotsV, multsV);

    const std::string common = std::to_string(s.degreeU()) + "," + std::to_string(s.degreeV()) +
                               "," + net + ",.UNSPECIFIED.,.F.,.F.,.F.";
    const std::string knotPart = StepWriter::intList(multsU) + "," + StepWriter::intList(multsV) +
                                 "," + StepWriter::realList(knotsU) + "," +
                                 StepWriter::realList(knotsV) + ",.UNSPECIFIED.";

    if (!rational) {
        return w.add("B_SPLINE_SURFACE_WITH_KNOTS(''," + common + "," + knotPart + ")");
    }
    return w.add("(BOUNDED_SURFACE() B_SPLINE_SURFACE(" + common +
                 ") B_SPLINE_SURFACE_WITH_KNOTS(" + knotPart +
                 ") GEOMETRIC_REPRESENTATION_ITEM() RATIONAL_B_SPLINE_SURFACE(" + weights +
                 ") REPRESENTATION_ITEM('') SURFACE())");
}

// ---------------------------------------------------------------------------
// Curved faces as designed (Phase 151)
// ---------------------------------------------------------------------------

/// A circle: what a rim's chords stand in for.
struct Circle {
    Vec3 center;
    Vec3 normal;  ///< unit
    double radius = 0.0;
};

/// @p curve as a circle, if it is one: three points on it make a circle,
/// and every other point sampled must lie on that one.
std::optional<Circle> circleOf(const geo::NurbsCurve& curve) {
    const double t0 = curve.tMin();
    const double t1 = curve.tMax();
    const Vec3 a = curve.evaluate(t0);
    const Vec3 b = curve.evaluate(t0 + (t1 - t0) / 3.0);
    const Vec3 c = curve.evaluate(t0 + 2.0 * (t1 - t0) / 3.0);
    const Vec3 ab = b - a;
    const Vec3 ac = c - a;
    const Vec3 n = ab.cross(ac);
    const double n2 = n.dot(n);
    const double scale = std::max({ab.length(), ac.length(), 1e-300});
    if (n2 <= 1e-18 * scale * scale * scale * scale) return std::nullopt;  // collinear
    // The circumcentre of a, b, c.
    const Vec3 center =
        a + (n.cross(ab) * ac.dot(ac) + ac.cross(n) * ab.dot(ab)) * (1.0 / (2.0 * n2));
    Circle circle{center, n * (1.0 / std::sqrt(n2)), (a - center).length()};
    const double tolerance = 1e-9 * std::max(1.0, circle.radius);
    for (int i = 0; i <= 16; ++i) {
        const Vec3 p = curve.evaluate(t0 + (t1 - t0) * i / 16.0);
        const Vec3 off = p - circle.center;
        if (std::abs(off.length() - circle.radius) > tolerance ||
            std::abs(off.dot(circle.normal)) > tolerance) {
            return std::nullopt;
        }
    }
    return circle;
}

/// A face's plane, from its outer loop (Newell): its normal (unit, the way
/// the loop runs) and a point on it. None for a degenerate loop.
std::optional<std::pair<Vec3, Vec3>> planeOf(const topo::Face& f) {
    if (f.outerLoop == nullptr || f.outerLoop->halfEdge == nullptr) return std::nullopt;
    Vec3 normal;
    Vec3 centroid;
    int count = 0;
    const topo::HalfEdge* start = f.outerLoop->halfEdge;
    const topo::HalfEdge* he = start;
    do {
        if (he == nullptr || he->origin == nullptr || he->next == nullptr ||
            he->next->origin == nullptr) {
            return std::nullopt;
        }
        const Vec3& p = he->origin->point;
        const Vec3& q = he->next->origin->point;
        normal = normal + Vec3((p.y - q.y) * (p.z + q.z), (p.z - q.z) * (p.x + q.x),
                               (p.x - q.x) * (p.y + q.y));
        centroid = centroid + p;
        ++count;
        he = he->next;
    } while (he != start && count < 1000000);
    const double length = normal.length();
    if (length <= 0.0 || count == 0) return std::nullopt;
    return std::make_pair(normal * (1.0 / length), centroid * (1.0 / count));
}

/// A curved face as designed: the facets that stand in for one analytic
/// surface, joined by their shared edges, and how they are written as one
/// face on it. `why` says why they are not, when they are not.
struct CurvedFace {
    std::vector<const topo::Face*> facets;
    const geo::NurbsSurface* surface = nullptr;
    /// Its outline: the half-edges of its facets whose other side is not
    /// one of them, in loops as its facets run.
    std::vector<std::vector<const topo::HalfEdge*>> loops;
    /// A closed face's (a cylinder's side): the edge between its two
    /// outlines it is cut along, and its half-edge from the first to the
    /// second.
    const topo::HalfEdge* seamUp = nullptr;
    bool sameSense = true;
    std::string why;  ///< empty: written as designed
};

/// The curved faces of @p solid (Phase 151): the facets of each analytic
/// surface, joined over their shared edges, and whether each can be written
/// as one face on its surface. One that cannot keeps its facets, and says
/// why. @p curved receives the edges written on their ideal curves, and
/// @p rims the circles recovered for those whose ideal was lost (a hole's
/// rim, cut by a Boolean: where a cylinder meets a plane square to it).
std::vector<CurvedFace> planCurvedFaces(const topo::Solid& solid,
                                        std::unordered_set<const topo::Edge*>& curved,
                                        std::unordered_map<const topo::Edge*, Circle>& rims) {
    // Facets of one surface, joined over their shared edges.
    std::unordered_map<const topo::Face*, const topo::Face*> parent;
    const std::function<const topo::Face*(const topo::Face*)> root = [&](const topo::Face* f) {
        const topo::Face* r = f;
        while (parent.at(r) != r) r = parent.at(r);
        while (parent.at(f) != r) {
            const topo::Face* next = parent.at(f);
            parent[f] = r;
            f = next;
        }
        return r;
    };
    for (const auto& f : solid.faces()) {
        if (f.analyticSurface) parent[&f] = &f;
    }
    for (const auto& e : solid.edges()) {
        const topo::HalfEdge* h = e.halfEdge;
        if (h == nullptr || h->twin == nullptr || h->face == nullptr || h->twin->face == nullptr) {
            continue;
        }
        const topo::Face* a = h->face;
        const topo::Face* b = h->twin->face;
        if (a == b || !a->analyticSurface || a->analyticSurface != b->analyticSurface) continue;
        const topo::Face* ra = root(a);
        const topo::Face* rb = root(b);
        if (ra != rb) parent[ra] = rb;
    }
    std::unordered_map<const topo::Face*, std::size_t> groupOf;
    std::vector<CurvedFace> faces;
    for (const auto& f : solid.faces()) {  // in the solid's order, so the file is too
        if (!f.analyticSurface) continue;
        const topo::Face* r = root(&f);
        auto it = groupOf.find(r);
        if (it == groupOf.end()) {
            it = groupOf.emplace(r, faces.size()).first;
            faces.emplace_back();
            faces.back().surface = f.analyticSurface.get();
        }
        groupOf[&f] = it->second;
        faces[it->second].facets.push_back(&f);
    }
    const auto groupIndex = [&](const topo::Face* f) -> std::optional<std::size_t> {
        const auto it = groupOf.find(f);
        if (it == groupOf.end()) return std::nullopt;
        return it->second;
    };
    const auto inGroup = [&](const topo::Face* f, std::size_t g) {
        const auto i = groupIndex(f);
        return i && *i == g;
    };
    const auto onSurface = [](const geo::NurbsSurface& surface, const Vec3& p, double scale) {
        const auto [u, v] = surface.closestPoint(p);
        return (surface.evaluate(u, v) - p).length() <= 1e-7 * std::max(1.0, scale);
    };

    for (std::size_t g = 0; g < faces.size(); ++g) {
        CurvedFace& face = faces[g];
        const geo::NurbsSurface& surface = *face.surface;
        // Its outline, and whether any of its corners is inside it: a pole,
        // or an apex, which no one outline goes round.
        std::vector<const topo::HalfEdge*> outline;
        std::unordered_set<const topo::Vertex*> corners;
        std::unordered_set<const topo::Vertex*> onOutline;
        std::size_t halfEdges = 0;
        for (const topo::Face* f : face.facets) {
            std::vector<const topo::Wire*> wires{f->outerLoop};
            wires.insert(wires.end(), f->innerLoops.begin(), f->innerLoops.end());
            for (const topo::Wire* wire : wires) {
                if (wire == nullptr || wire->halfEdge == nullptr) continue;
                const topo::HalfEdge* start = wire->halfEdge;
                const topo::HalfEdge* he = start;
                do {
                    ++halfEdges;
                    corners.insert(he->origin);
                    if (he->twin == nullptr || !inGroup(he->twin->face, g)) {
                        outline.push_back(he);
                        onOutline.insert(he->origin);
                        if (he->next != nullptr) onOutline.insert(he->next->origin);
                    }
                    he = he->next;
                } while (he != nullptr && he != start && halfEdges < 10000000);
            }
        }
        if (outline.empty()) {
            face.why = "it is closed all round (a sphere or a torus)";
            continue;
        }
        if (corners.size() != onOutline.size()) {
            face.why = "it comes to a point inside it (a cone's apex, a pole)";
            continue;
        }
        // The outline in loops: from a half-edge on it to the next, round
        // the vertex it ends at, over the facets' own edges.
        std::unordered_set<const topo::HalfEdge*> left(outline.begin(), outline.end());
        std::size_t nextStart = 0;
        while (!left.empty()) {
            std::vector<const topo::HalfEdge*> loop;
            // The same start each time, whatever the set's order: the first
            // of the outline, in its own order, still left.
            while (left.count(outline[nextStart]) == 0) ++nextStart;
            const topo::HalfEdge* he = outline[nextStart];
            bool closed = false;
            for (std::size_t guard = 0; guard <= halfEdges; ++guard) {
                loop.push_back(he);
                left.erase(he);
                const topo::HalfEdge* next = he->next;
                std::size_t turns = 0;
                while (next != nullptr && next->twin != nullptr && inGroup(next->twin->face, g) &&
                       turns++ <= halfEdges) {
                    next = next->twin->next;
                }
                if (next == nullptr) break;
                if (next == loop.front()) {
                    closed = true;
                    break;
                }
                if (left.count(next) == 0) break;  // not a simple outline
                he = next;
            }
            if (!closed) {
                face.why = "its outline does not close";
                break;
            }
            face.loops.push_back(std::move(loop));
        }
        if (!face.why.empty()) continue;
        if (face.loops.size() > 2) {
            face.why = "it has more than two outlines";
            continue;
        }

        // Every edge of its outline, on it: a circle, a curve of its own
        // span, a straight edge along it (a cylinder's rulings), or a rim
        // where it meets a plane square to it, whose circle a Boolean did
        // not keep.
        const auto frame = model::MateGeometry::frameForFace(*face.facets.front());
        // A rim's circle from its own corners, which lie on it exactly: the
        // circle through three of them, the rest checked against it. (The
        // cylinder's own centre is fitted to its facets, 5e-6 off.)
        bool rimOffCircle = false;  // a rim found, its corners not all on one circle
        const auto rimOf = [&](const std::vector<const topo::HalfEdge*>& loop,
                               const topo::HalfEdge* he) -> std::optional<Circle> {
            if (!frame || frame->kind != model::MateFrameKind::Cylindrical || he->twin == nullptr ||
                groupIndex(he->twin->face)) {
                return std::nullopt;
            }
            const auto plane = planeOf(*he->twin->face);
            if (!plane || std::abs(std::abs(plane->first.dot(frame->direction)) - 1.0) > 1e-9) {
                return std::nullopt;
            }
            // Its corners: of the outline's edges into that plane, whatever
            // faces it is in (a Boolean leaves a plane in triangles).
            std::vector<Vec3> corners;
            for (const topo::HalfEdge* h : loop) {
                if (h->twin == nullptr || h->twin->face == nullptr) continue;
                const auto other = planeOf(*h->twin->face);
                if (!other || other->first.dot(plane->first) < 1.0 - 1e-12 ||
                    std::abs((other->second - plane->second).dot(plane->first)) >
                        1e-9 * std::max(1.0, plane->second.length())) {
                    continue;
                }
                corners.push_back(h->origin->point);
            }
            if (corners.size() < 3) return std::nullopt;
            const Vec3& p = corners[0];
            const Vec3& q = corners[corners.size() / 3];
            const Vec3& r = corners[2 * corners.size() / 3];
            const Vec3 pq = q - p;
            const Vec3 pr = r - p;
            const Vec3 n = pq.cross(pr);
            const double n2 = n.dot(n);
            if (n2 <= 0.0) return std::nullopt;
            const Vec3 centre =
                p + (n.cross(pq) * pr.dot(pr) + pr.cross(n) * pq.dot(pq)) * (1.0 / (2.0 * n2));
            const Circle rim{centre, plane->first, (p - centre).length()};
            const double tol = 1e-9 * std::max(1.0, rim.radius + centre.length());
            // The cylinder's own rim, every corner on it: a Boolean may split
            // a chord in two, at a point inside the circle, and then no one
            // circle goes through its corners.
            bool onCircle =
                std::abs(rim.radius - frame->radius) <= 1e-5 * std::max(1.0, rim.radius);
            for (const Vec3& corner : corners) {
                const Vec3 off = corner - rim.center;
                onCircle = onCircle && std::abs(off.length() - rim.radius) <= tol &&
                           std::abs(off.dot(rim.normal)) <= tol;
            }
            if (!onCircle) {
                rimOffCircle = true;
                return std::nullopt;
            }
            return rim;
        };
        double scale = 0.0;
        for (const auto* he : outline) {
            scale = std::max(scale, he->origin->point.length());
        }
        for (const auto& loop : face.loops) {
            for (const topo::HalfEdge* he : loop) {
                const topo::Edge* e = he->edge;
                if (e == nullptr || e->halfEdge == nullptr || e->halfEdge->twin == nullptr) {
                    face.why = "an edge of its outline is incomplete";
                    break;
                }
                const Vec3& a = e->halfEdge->origin->point;
                const Vec3& b = e->halfEdge->twin->origin->point;
                if (e->analyticCurve) {
                    // A rim's circle is shared by its chords, and a Boolean
                    // gives it to the pieces of a chord it cuts, whose cut
                    // end is inside the circle: this edge's own ends on it.
                    if (const auto circle = circleOf(*e->analyticCurve)) {
                        const double tol = 1e-9 * std::max(1.0, circle->radius);
                        const auto onIt = [&](const Vec3& p) {
                            const Vec3 off = p - circle->center;
                            return std::abs(off.length() - circle->radius) <= tol &&
                                   std::abs(off.dot(circle->normal)) <= tol;
                        };
                        if (onIt(a) && onIt(b)) continue;
                        face.why =
                            "an edge of its outline has an end off its circle (a cut "
                            "chord)";
                        break;
                    }
                    const Vec3 c0 = e->analyticCurve->evaluate(e->analyticCurve->tMin());
                    const Vec3 c1 = e->analyticCurve->evaluate(e->analyticCurve->tMax());
                    const double tol = 1e-9 * std::max(1.0, scale);
                    if ((((c0 - a).length() <= tol && (c1 - b).length() <= tol) ||
                         ((c0 - b).length() <= tol && (c1 - a).length() <= tol))) {
                        continue;  // its own span
                    }
                    face.why = "an edge of its outline is part of a curve other than a circle";
                    break;
                }
                if (const auto rim = rimOf(loop, he)) {
                    rims[e] = *rim;
                    continue;
                }
                if (!onSurface(surface, (a + b) * 0.5, scale)) {
                    face.why = rimOffCircle
                                   ? "its rim's corners are not all on one circle (a Boolean "
                                     "split its chords)"
                                   : "a straight edge of its outline leaves its surface";
                    break;
                }
            }
            if (!face.why.empty()) break;
        }
        if (!face.why.empty()) continue;

        // Two outlines (a cylinder's side): cut along an edge between them
        // that lies on the surface (a ruling, not a triangle's diagonal).
        if (face.loops.size() == 2) {
            std::unordered_set<const topo::Vertex*> first;
            std::unordered_set<const topo::Vertex*> second;
            for (const auto* he : face.loops[0]) first.insert(he->origin);
            for (const auto* he : face.loops[1]) second.insert(he->origin);
            bool joined = false;
            for (const topo::Face* f : face.facets) {
                const topo::HalfEdge* start = f->outerLoop->halfEdge;
                const topo::HalfEdge* he = start;
                do {
                    if (he->twin != nullptr && inGroup(he->twin->face, g) &&
                        first.count(he->origin) != 0 && he->next != nullptr &&
                        second.count(he->next->origin) != 0) {
                        joined = true;
                        const Vec3 mid = (he->origin->point + he->next->origin->point) * 0.5;
                        if (!he->edge->analyticCurve && onSurface(surface, mid, scale)) {
                            face.seamUp = he;
                            break;
                        }
                    }
                    he = he->next;
                } while (he != nullptr && he != start);
                if (face.seamUp != nullptr) break;
            }
            if (face.seamUp == nullptr) {
                face.why = joined ? "no edge joining its two outlines lies on its surface"
                                  : "no edge joins its two outlines";
                continue;
            }
        }

        // Which way it faces: each facet's normal against the surface's.
        int along = 0;
        int against = 0;
        const std::size_t step = std::max<std::size_t>(1, face.facets.size() / 16);
        for (std::size_t i = 0; i < face.facets.size(); i += step) {
            const auto plane = planeOf(*face.facets[i]);
            if (!plane) continue;
            const auto [u, v] = surface.closestPoint(plane->second);
            const double d = surface.normal(u, v).dot(plane->first);
            if (d > 1e-6) ++along;
            if (d < -1e-6) ++against;
        }
        if (along > 0 && against > 0) {
            face.why = "its facets face both ways on its surface";
            continue;
        }
        if (along == 0 && against == 0) {
            face.why = "which way it faces cannot be told";
            continue;
        }
        face.sameSense = along > 0;
    }

    // The edges of the outlines written on their ideal curves; each must be
    // on the face across it too: another written as designed, or a plane
    // the curve lies in (a cylinder's cap). A face kept in facets is
    // bounded by chords, and one written as designed beside it cannot be.
    for (bool changed = true; changed;) {
        changed = false;
        curved.clear();
        for (std::size_t g = 0; g < faces.size(); ++g) {
            CurvedFace& face = faces[g];
            if (!face.why.empty()) continue;
            for (const auto& loop : face.loops) {
                for (const topo::HalfEdge* he : loop) {
                    const topo::Edge* e = he->edge;
                    const auto rim = rims.find(e);
                    if (!e->analyticCurve && rim == rims.end()) continue;
                    const topo::Face* across = he->twin->face;
                    const auto other = groupIndex(across);
                    bool accepted = other && faces[*other].why.empty();
                    if (!accepted && !other) {
                        const auto plane = planeOf(*across);
                        accepted = plane.has_value();
                        for (int i = 0; accepted && i <= 8; ++i) {
                            Vec3 p;
                            if (e->analyticCurve) {
                                p = e->analyticCurve->evaluate(
                                    e->analyticCurve->tMin() +
                                    (e->analyticCurve->tMax() - e->analyticCurve->tMin()) * i /
                                        8.0);
                            } else {
                                // A recovered rim lies in the plane it was
                                // found from: its circle's points, by angle.
                                const Circle& c = rim->second;
                                const Vec3 ref =
                                    std::abs(c.normal.x) < 0.9 ? Vec3(1, 0, 0) : Vec3(0, 1, 0);
                                const Vec3 u =
                                    c.normal.cross(ref) * (1.0 / c.normal.cross(ref).length());
                                const Vec3 w = c.normal.cross(u);
                                const double angle = 2.0 * math::kPi * i / 8.0;
                                p = c.center +
                                    (u * std::cos(angle) + w * std::sin(angle)) * c.radius;
                            }
                            accepted = std::abs((p - plane->second).dot(plane->first)) <=
                                       1e-7 * std::max(1.0, p.length());
                        }
                    }
                    if (!accepted) {
                        face.why = "a face beside it is kept in facets";
                        changed = true;
                        break;
                    }
                    curved.insert(e);
                }
                if (!face.why.empty()) break;
            }
        }
    }
    return faces;
}

/// Emit one solid; returns one MANIFOLD_SOLID_BREP id per shell (Horizon
/// solids may hold several disjoint shells, e.g. Pattern results — STEP
/// expresses those as sibling MANIFOLD_SOLID_BREPs in one representation).
std::vector<int> writeSolid(StepWriter& w, const topo::Solid& solid, int index, bool asDesigned,
                            std::vector<std::string>* faceted) {
    // Its curved faces as designed, where they can be (Phase 151).
    std::unordered_set<const topo::Edge*> curved;
    std::unordered_map<const topo::Edge*, Circle> rims;
    std::vector<CurvedFace> designed;
    if (asDesigned) designed = planCurvedFaces(solid, curved, rims);
    std::unordered_map<const topo::Face*, const CurvedFace*> designedOf;
    std::unordered_set<const topo::Edge*> seams;
    for (const CurvedFace& face : designed) {
        if (!face.why.empty()) {
            if (faceted != nullptr) {
                faceted->push_back("a curved face of " + std::to_string(face.facets.size()) +
                                   " facets: " + face.why);
            }
            continue;
        }
        for (const topo::Face* f : face.facets) designedOf[f] = &face;
        if (face.seamUp != nullptr) seams.insert(face.seamUp->edge);
    }
    // An edge between two facets of one face written as designed is inside
    // it, and not written: but for the seam it is cut along.
    const auto inside = [&](const topo::Edge& e) {
        const topo::HalfEdge* h = e.halfEdge;
        if (h == nullptr || h->twin == nullptr || seams.count(&e) != 0) return false;
        const auto a = designedOf.find(h->face);
        const auto b = designedOf.find(h->twin->face);
        return a != designedOf.end() && b != designedOf.end() && a->second == b->second;
    };

    // Vertices.
    std::unordered_map<const topo::Vertex*, int> vertexIds;
    for (const auto& v : solid.vertices()) {
        vertexIds[&v] = w.add("VERTEX_POINT('',#" + std::to_string(w.addPoint(v.point)) + ")");
    }

    // Edges. Orient the written curve along the recorded half-edge direction.
    std::unordered_map<const topo::Edge*, int> edgeIds;
    for (const auto& e : solid.edges()) {
        if (e.halfEdge == nullptr || e.curve == nullptr || inside(e)) continue;
        const topo::Vertex* start = e.halfEdge->origin;
        const topo::Vertex* end = e.halfEdge->twin->origin;
        const std::string ends =
            "#" + std::to_string(vertexIds.at(start)) + ",#" + std::to_string(vertexIds.at(end));
        if (curved.count(&e) != 0) {
            const auto rim = rims.find(&e);
            const std::optional<Circle> circle =
                rim != rims.end() ? std::optional<Circle>(rim->second) : circleOf(*e.analyticCurve);
            if (circle) {
                // On its circle, the short way from its start to its end: the
                // circle's axis turned so that way runs anticlockwise.
                const Vec3 from = start->point - circle->center;
                const Vec3 to = end->point - circle->center;
                const Vec3 axis = from.cross(to).dot(circle->normal) >= 0.0 ? circle->normal
                                                                            : circle->normal * -1.0;
                const Vec3 ref = from * (1.0 / from.length());
                const int placement = w.add(
                    "AXIS2_PLACEMENT_3D('',#" + std::to_string(w.addPoint(circle->center)) + ",#" +
                    std::to_string(w.add("DIRECTION('',(" + fmtReal(axis.x) + "," +
                                         fmtReal(axis.y) + "," + fmtReal(axis.z) + "))")) +
                    ",#" +
                    std::to_string(w.add("DIRECTION('',(" + fmtReal(ref.x) + "," + fmtReal(ref.y) +
                                         "," + fmtReal(ref.z) + "))")) +
                    ")");
                const int circleId = w.add("CIRCLE('',#" + std::to_string(placement) + "," +
                                           fmtReal(circle->radius) + ")");
                edgeIds[&e] =
                    w.add("EDGE_CURVE(''," + ends + ",#" + std::to_string(circleId) + ",.T.)");
                continue;
            }
            // A curve of the edge's own span.
            const Vec3 c0 = e.analyticCurve->evaluate(e.analyticCurve->tMin());
            const bool senseForward = (c0 - start->point).length() <= (c0 - end->point).length();
            const int curveId = writeCurve(w, *e.analyticCurve);
            edgeIds[&e] = w.add("EDGE_CURVE(''," + ends + ",#" + std::to_string(curveId) + "," +
                                (senseForward ? ".T." : ".F.") + ")");
            continue;
        }
        // same_sense: does the curve run start → end?
        const Vec3 c0 = e.curve->evaluate(e.curve->tMin());
        const bool senseForward = (c0 - start->point).length() <= (c0 - end->point).length();
        const int curveId = writeCurve(w, *e.curve);
        edgeIds[&e] = w.add("EDGE_CURVE(''," + ends + ",#" + std::to_string(curveId) + "," +
                            (senseForward ? ".T." : ".F.") + ")");
    }

    // Faces.
    auto writeFace = [&](const topo::Face& f) -> std::optional<int> {
        if (f.outerLoop == nullptr || f.surface == nullptr) return std::nullopt;

        auto writeLoop = [&](const topo::Wire* wire, bool outer) -> std::optional<int> {
            std::vector<int> oriented;
            const topo::HalfEdge* start = wire->halfEdge;
            const topo::HalfEdge* cur = start;
            do {
                if (cur->edge == nullptr) return std::nullopt;
                auto it = edgeIds.find(cur->edge);
                if (it == edgeIds.end()) return std::nullopt;
                const bool forward = (cur == cur->edge->halfEdge);
                oriented.push_back(w.add("ORIENTED_EDGE('',*,*,#" + std::to_string(it->second) +
                                         "," + (forward ? ".T." : ".F.") + ")"));
                cur = cur->next;
            } while (cur != nullptr && cur != start);
            const int loop = w.add("EDGE_LOOP(''," + StepWriter::refList(oriented) + ")");
            return w.add(std::string(outer ? "FACE_OUTER_BOUND" : "FACE_BOUND") + "('',#" +
                         std::to_string(loop) + ",.T.)");
        };

        std::vector<int> bounds;
        if (auto b = writeLoop(f.outerLoop, true)) bounds.push_back(*b);
        for (const topo::Wire* inner : f.innerLoops) {
            if (auto b = writeLoop(inner, false)) bounds.push_back(*b);
        }
        if (bounds.empty()) return std::nullopt;

        const int surfId = writeSurface(w, *f.surface);
        return w.add("ADVANCED_FACE(''," + StepWriter::refList(bounds) + ",#" +
                     std::to_string(surfId) + ",.T.)");
    };

    // A curved face as designed: one face on its surface, round its outline;
    // a closed one's two outlines joined into one loop along its seam, used
    // once each way, as other systems (and this one's reader) expect.
    auto writeDesigned = [&](const CurvedFace& face) -> std::optional<int> {
        const auto oriented = [&](const topo::HalfEdge* he) -> std::optional<int> {
            const auto it = edgeIds.find(he->edge);
            if (it == edgeIds.end()) return std::nullopt;
            const bool forward = he->origin == he->edge->halfEdge->origin;
            return w.add("ORIENTED_EDGE('',*,*,#" + std::to_string(it->second) + "," +
                         (forward ? ".T." : ".F.") + ")");
        };
        std::vector<const topo::HalfEdge*> path;
        const auto from = [](const std::vector<const topo::HalfEdge*>& loop,
                             const topo::Vertex* v) {
            std::vector<const topo::HalfEdge*> turned;
            const auto at = std::find_if(loop.begin(), loop.end(),
                                         [v](const topo::HalfEdge* he) { return he->origin == v; });
            if (at == loop.end()) return turned;
            turned.insert(turned.end(), at, loop.end());
            turned.insert(turned.end(), loop.begin(), at);
            return turned;
        };
        if (face.seamUp == nullptr) {
            path = face.loops.front();
        } else {
            const auto first = from(face.loops[0], face.seamUp->origin);
            const auto second = from(face.loops[1], face.seamUp->next->origin);
            if (first.empty() || second.empty()) return std::nullopt;
            path = first;
            path.push_back(face.seamUp);
            path.insert(path.end(), second.begin(), second.end());
            path.push_back(face.seamUp->twin);
        }
        std::vector<int> edges;
        for (const topo::HalfEdge* he : path) {
            const auto id = oriented(he);
            if (!id) return std::nullopt;
            edges.push_back(*id);
        }
        const int loop = w.add("EDGE_LOOP(''," + StepWriter::refList(edges) + ")");
        const int bound = w.add("FACE_OUTER_BOUND('',#" + std::to_string(loop) + ",.T.)");
        const int surfId = writeSurface(w, *face.surface);
        return w.add("ADVANCED_FACE('',(#" + std::to_string(bound) + "),#" +
                     std::to_string(surfId) + "," + (face.sameSense ? ".T." : ".F.") + ")");
    };

    auto writeShell = [&](const std::vector<const topo::Face*>& faces,
                          int shellIndex) -> std::optional<int> {
        std::vector<int> faceIds;
        std::unordered_set<const CurvedFace*> written;
        for (const topo::Face* f : faces) {
            if (f == nullptr) continue;
            const auto designedFace = designedOf.find(f);
            if (designedFace != designedOf.end()) {
                // Its facets are the one face: written once, for the first.
                if (!written.insert(designedFace->second).second) continue;
                if (auto id = writeDesigned(*designedFace->second)) faceIds.push_back(*id);
                continue;
            }
            if (auto id = writeFace(*f)) faceIds.push_back(*id);
        }
        if (faceIds.empty()) return std::nullopt;
        const int shell = w.add("CLOSED_SHELL(''," + StepWriter::refList(faceIds) + ")");
        return w.add("MANIFOLD_SOLID_BREP('solid_" + std::to_string(index) + "_" +
                     std::to_string(shellIndex) + "',#" + std::to_string(shell) + ")");
    };

    std::vector<int> msbIds;
    for (const auto& sh : solid.shells()) {
        std::vector<const topo::Face*> faces(sh.faces.begin(), sh.faces.end());
        if (auto msb = writeShell(faces, static_cast<int>(msbIds.size()))) {
            msbIds.push_back(*msb);
        }
    }
    if (msbIds.empty()) {
        // Defensive fallback for faces not registered with any shell.
        std::vector<const topo::Face*> faces;
        for (const auto& f : solid.faces()) faces.push_back(&f);
        if (auto msb = writeShell(faces, 0)) msbIds.push_back(*msb);
    }
    return msbIds;
}

// ===========================================================================
// Parser — Part-21 tokenizer and instance model
// ===========================================================================

struct StepValue;
using StepList = std::vector<StepValue>;

struct StepValue {
    // NOLINTNEXTLINE(readability-enum-initial-value)
    enum Kind { Null, Star, Real, Str, Enum, Ref, List, Typed };
    Kind kind = Null;
    double num = 0.0;
    std::string text;                 // Str payload, Enum name, Typed name
    int ref = 0;                      // Ref payload
    std::shared_ptr<StepList> items;  // List / Typed payload

    bool isRef() const { return kind == Ref; }
    bool isList() const { return kind == List; }
};

/// A parsed instance: one or more (entityType, args) leaves. Simple instances
/// have exactly one leaf; complex instances have several.
struct StepInstance {
    std::vector<std::pair<std::string, StepList>> leaves;

    const StepList* leaf(const std::string& type) const {
        for (const auto& [t, args] : leaves) {
            if (t == type) return &args;
        }
        return nullptr;
    }
    bool hasType(const std::string& type) const { return leaf(type) != nullptr; }
    const std::string& primaryType() const {
        static const std::string empty;
        return leaves.empty() ? empty : leaves.front().first;
    }
};

class StepParser {
public:
    explicit StepParser(const std::string& text) : m_text(text) {}

    /// Parse the DATA section into the instance map. False on hard error,
    /// or once @p cancelled is set.
    bool parse(std::string& error, const std::atomic<bool>* cancelled = nullptr) {
        std::string stripped = stripComments(m_text);
        const size_t dataPos = stripped.find("DATA;");
        if (dataPos == std::string::npos) {
            error = "no DATA section found";
            return false;
        }
        size_t pos = dataPos + 5;
        const size_t endPos = stripped.find("ENDSEC;", dataPos);
        const size_t limit = (endPos == std::string::npos) ? stripped.size() : endPos;

        size_t read = 0;
        while (pos < limit) {
            if (cancelled != nullptr && (++read & 1023U) == 0 &&
                cancelled->load(std::memory_order_relaxed)) {
                error = kCancelled;
                return false;
            }
            // Find next instance start.
            while (pos < limit && stripped[pos] != '#') ++pos;
            if (pos >= limit) break;
            // Split at the terminating ';' outside of strings.
            size_t end = pos;
            bool inString = false;
            while (end < limit) {
                const char c = stripped[end];
                if (c == '\'') inString = !inString;
                if (c == ';' && !inString) break;
                ++end;
            }
            if (end >= limit) break;
            if (!parseInstance(stripped, pos, end, error)) return false;
            pos = end + 1;
        }
        return true;
    }

    const StepInstance* find(int id) const {
        auto it = m_instances.find(id);
        return it == m_instances.end() ? nullptr : &it->second;
    }

    /// All ids whose instance contains a leaf of @p type, in file order.
    std::vector<int> allOfType(const std::string& type) const {
        std::vector<int> out;
        for (int id : m_order) {
            if (m_instances.at(id).hasType(type)) out.push_back(id);
        }
        return out;
    }

private:
    static std::string stripComments(const std::string& in) {
        std::string out;
        out.reserve(in.size());
        size_t i = 0;
        bool inString = false;
        while (i < in.size()) {
            if (!inString && i + 1 < in.size() && in[i] == '/' && in[i + 1] == '*') {
                const size_t close = in.find("*/", i + 2);
                if (close == std::string::npos) break;
                i = close + 2;
                continue;
            }
            if (in[i] == '\'') inString = !inString;
            out += in[i++];
        }
        return out;
    }

    /// Read the digits of an entity number ("#123") at `p`. False when the
    /// number does not fit an int — accumulating it unchecked is signed
    /// overflow, and a wrapped id could alias another entity.
    static bool parseEntityNumber(const std::string& s, size_t& p, size_t end, int& id) {
        id = 0;
        while (p < end && std::isdigit(static_cast<unsigned char>(s[p]))) {
            const int digit = s[p] - '0';
            if (id > (std::numeric_limits<int>::max() - digit) / 10) return false;
            id = id * 10 + digit;
            ++p;
        }
        return true;
    }

    bool parseInstance(const std::string& s, size_t pos, size_t end, std::string& error) {
        // "#id = RHS" — RHS is TYPE(args) or (TYPE1(args) TYPE2(args) ...).
        size_t p = pos + 1;
        int id = 0;
        if (!parseEntityNumber(s, p, end, id)) {
            error = "entity number too large near offset " + std::to_string(pos);
            return false;
        }
        while (p < end &&
               (s[p] == ' ' || s[p] == '=' || s[p] == '\n' || s[p] == '\r' || s[p] == '\t')) {
            ++p;
        }
        if (p >= end || id == 0) {
            error = "malformed instance near offset " + std::to_string(pos);
            return false;
        }

        StepInstance inst;
        if (s[p] == '(') {
            // Complex instance: sequence of leaves inside the outer parens.
            ++p;
            while (p < end) {
                skipWs(s, p, end);
                if (p < end && s[p] == ')') break;
                if (!parseLeaf(s, p, end, inst, error)) return false;
            }
        } else {
            if (!parseLeaf(s, p, end, inst, error)) return false;
        }
        m_instances.emplace(id, std::move(inst));
        m_order.push_back(id);
        return true;
    }

    static void skipWs(const std::string& s, size_t& p, size_t end) {
        while (p < end && std::isspace(static_cast<unsigned char>(s[p]))) ++p;
    }

    bool parseLeaf(const std::string& s, size_t& p, size_t end, StepInstance& inst,
                   std::string& error) {
        skipWs(s, p, end);
        std::string type;
        while (p < end && (std::isalnum(static_cast<unsigned char>(s[p])) || s[p] == '_')) {
            type += s[p++];
        }
        skipWs(s, p, end);
        if (type.empty() || p >= end || s[p] != '(') {
            error = "expected entity leaf";
            return false;
        }
        ++p;  // consume '('
        StepList args;
        if (!parseArgs(s, p, end, args, error)) return false;
        inst.leaves.emplace_back(std::move(type), std::move(args));
        return true;
    }

    /// Parse a comma-separated argument list; consumes the closing ')'.
    bool parseArgs(const std::string& s, size_t& p, size_t end, StepList& out, std::string& error) {
        // parseArgs and parseValue are mutually recursive; adversarial files
        // with thousands of nested '(' would otherwise overflow the stack.
        if (m_depth >= kMaxNesting) {
            error = "argument nesting too deep";
            return false;
        }
        ++m_depth;
        const bool ok = parseArgsInner(s, p, end, out, error);
        --m_depth;
        return ok;
    }

    bool parseArgsInner(const std::string& s, size_t& p, size_t end, StepList& out,
                        std::string& error) {
        while (p < end) {
            skipWs(s, p, end);
            if (p >= end) break;
            if (s[p] == ')') {
                ++p;
                return true;
            }
            if (s[p] == ',') {
                ++p;
                continue;
            }
            StepValue v;
            if (!parseValue(s, p, end, v, error)) return false;
            out.push_back(std::move(v));
        }
        error = "unterminated argument list";
        return false;
    }

    bool parseValue(const std::string& s, size_t& p, size_t end, StepValue& v, std::string& error) {
        skipWs(s, p, end);
        if (p >= end) {
            error = "unexpected end of input";
            return false;
        }
        const char c = s[p];
        if (c == '$') {
            v.kind = StepValue::Null;
            ++p;
            return true;
        }
        if (c == '*') {
            v.kind = StepValue::Star;
            ++p;
            return true;
        }
        if (c == '#') {
            ++p;
            int id = 0;
            if (!parseEntityNumber(s, p, end, id)) {
                error = "entity reference too large near offset " + std::to_string(p);
                return false;
            }
            v.kind = StepValue::Ref;
            v.ref = id;
            return true;
        }
        if (c == '\'') {
            ++p;
            std::string str;
            while (p < end) {
                if (s[p] == '\'') {
                    if (p + 1 < end && s[p + 1] == '\'') {  // escaped quote
                        str += '\'';
                        p += 2;
                        continue;
                    }
                    ++p;
                    break;
                }
                str += s[p++];
            }
            v.kind = StepValue::Str;
            v.text = std::move(str);
            return true;
        }
        if (c == '.') {
            ++p;
            std::string name;
            while (p < end && s[p] != '.') name += s[p++];
            if (p < end) ++p;  // closing '.'
            v.kind = StepValue::Enum;
            v.text = std::move(name);
            return true;
        }
        if (c == '(') {
            ++p;
            v.kind = StepValue::List;
            v.items = std::make_shared<StepList>();
            return parseArgs(s, p, end, *v.items, error);
        }
        if (std::isdigit(static_cast<unsigned char>(c)) || c == '-' || c == '+') {
            size_t q = p;
            while (q < end && (std::isdigit(static_cast<unsigned char>(s[q])) || s[q] == '.' ||
                               s[q] == '-' || s[q] == '+' || s[q] == 'e' || s[q] == 'E')) {
                ++q;
            }
            v.kind = StepValue::Real;
            if (!parseReal(std::string_view(s).substr(p, q - p), v.num)) {
                error = "malformed number near offset " + std::to_string(p);
                return false;
            }
            p = q;
            return true;
        }
        if (std::isalpha(static_cast<unsigned char>(c)) || c == '_') {
            // Typed nested value: NAME(args).
            std::string name;
            while (p < end && (std::isalnum(static_cast<unsigned char>(s[p])) || s[p] == '_')) {
                name += s[p++];
            }
            skipWs(s, p, end);
            if (p < end && s[p] == '(') {
                ++p;
                v.kind = StepValue::Typed;
                v.text = std::move(name);
                v.items = std::make_shared<StepList>();
                return parseArgs(s, p, end, *v.items, error);
            }
            error = "unexpected token '" + name + "'";
            return false;
        }
        error = std::string("unexpected character '") + c + "'";
        return false;
    }

    static constexpr int kMaxNesting = 64;

    const std::string& m_text;
    std::unordered_map<int, StepInstance> m_instances;
    std::vector<int> m_order;
    int m_depth = 0;
};

// ===========================================================================
// Reconstruction — STEP entities → hz::topo::Solid
// ===========================================================================

class SolidBuilder {
public:
    SolidBuilder(const StepParser& parser, int solidIndex,
                 const std::atomic<bool>* cancelled = nullptr, double radiansPerUnit = 1.0)
        : m_parser(parser),
          m_solidIndex(solidIndex),
          m_cancelled(cancelled),
          m_radiansPerUnit(radiansPerUnit) {}

    /// Build one topo::Solid from a group of MANIFOLD_SOLID_BREP instance ids
    /// (one shell per MSB — Horizon multi-shell solids export as siblings in
    /// one shape representation).
    std::unique_ptr<topo::Solid> build(const std::vector<int>& msbIds, std::string& error) {
        m_solid = std::make_unique<topo::Solid>();

        for (int msbId : msbIds) {
            const StepInstance* msb = m_parser.find(msbId);
            const StepList* args = msb ? msb->leaf("MANIFOLD_SOLID_BREP") : nullptr;
            if (args == nullptr || args->size() < 2 || !(*args)[1].isRef()) {
                error = "malformed MANIFOLD_SOLID_BREP #" + std::to_string(msbId);
                return nullptr;
            }
            const StepInstance* shellInst = m_parser.find((*args)[1].ref);
            const StepList* shellArgs = shellInst ? shellInst->leaf("CLOSED_SHELL") : nullptr;
            if (shellArgs == nullptr) {
                shellArgs = shellInst ? shellInst->leaf("OPEN_SHELL") : nullptr;
            }
            if (shellArgs == nullptr || shellArgs->size() < 2 || !(*shellArgs)[1].isList()) {
                error = "malformed shell for MANIFOLD_SOLID_BREP #" + std::to_string(msbId);
                return nullptr;
            }

            topo::Shell* shell = m_solid->allocShell();
            shell->solid = m_solid.get();

            for (const StepValue& faceRef : *(*shellArgs)[1].items) {
                if (m_cancelled != nullptr && m_cancelled->load(std::memory_order_relaxed)) {
                    error = kCancelled;
                    return nullptr;
                }
                if (!faceRef.isRef()) continue;
                if (!buildFace(faceRef.ref, shell, error)) return nullptr;
            }
        }

        if (!linkTwins(error)) return nullptr;

        if (!m_solid->isValid()) {
            error = "imported solid failed validation:\n" + m_solid->validationReport();
            return nullptr;
        }
        return std::move(m_solid);
    }

private:
    // -- Geometry ------------------------------------------------------------

    std::optional<Vec3> readPoint(int id) const {
        const StepInstance* inst = m_parser.find(id);
        const StepList* args = inst ? inst->leaf("CARTESIAN_POINT") : nullptr;
        if (args == nullptr || args->size() < 2 || !(*args)[1].isList() ||
            (*args)[1].items->size() < 3) {
            return std::nullopt;
        }
        const StepList& c = *(*args)[1].items;
        return Vec3{c[0].num, c[1].num, c[2].num};
    }

    std::optional<Vec3> readDirection(int id) const {
        const StepInstance* inst = m_parser.find(id);
        const StepList* args = inst ? inst->leaf("DIRECTION") : nullptr;
        if (args == nullptr || args->size() < 2 || !(*args)[1].isList() ||
            (*args)[1].items->size() < 3) {
            return std::nullopt;
        }
        const StepList& c = *(*args)[1].items;
        return Vec3{c[0].num, c[1].num, c[2].num};
    }

    /// AXIS2_PLACEMENT_3D → (origin, zAxis, xAxis).
    bool readPlacement(int id, Vec3& origin, Vec3& zAxis, Vec3& xAxis) const {
        const StepInstance* inst = m_parser.find(id);
        const StepList* args = inst ? inst->leaf("AXIS2_PLACEMENT_3D") : nullptr;
        if (args == nullptr || args->size() < 3) return false;
        auto o = (*args)[1].isRef() ? readPoint((*args)[1].ref) : std::nullopt;
        if (!o) return false;
        origin = *o;
        zAxis = Vec3{0, 0, 1};
        xAxis = Vec3{1, 0, 0};
        if ((*args)[2].isRef()) {
            if (auto z = readDirection((*args)[2].ref)) zAxis = z->normalized();
        }
        if (args->size() > 3 && (*args)[3].isRef()) {
            if (auto x = readDirection((*args)[3].ref)) xAxis = x->normalized();
        }
        // Re-orthogonalize X against Z; when the reference direction is absent
        // or (near-)parallel to the axis, derive any perpendicular instead.
        xAxis = xAxis - zAxis * xAxis.dot(zAxis);
        if (xAxis.length() < 1e-9) {
            const Vec3 seed = std::abs(zAxis.x) < 0.9 ? Vec3{1, 0, 0} : Vec3{0, 1, 0};
            xAxis = seed - zAxis * seed.dot(zAxis);
        }
        xAxis = xAxis.normalized();
        return true;
    }

    static std::vector<double> expandKnots(const StepList& mults, const StepList& knots) {
        // Multiplicities come straight from the file — bound them before
        // allocating, or a single absurd value drives an OOM.
        std::vector<double> out;
        size_t total = 0;
        for (size_t i = 0; i < mults.size() && i < knots.size(); ++i) {
            const double raw = mults[i].num;
            if (!(raw >= 1.0 && raw <= 1e4)) return {};
            const auto m = static_cast<size_t>(raw);
            total += m;
            if (total > 100000) return {};
            for (size_t j = 0; j < m; ++j) out.push_back(knots[i].num);
        }
        return out;
    }

    std::shared_ptr<geo::NurbsCurve> readBSplineCurve(const StepInstance& inst) const {
        // Attribute layout differs between the simple and complex forms.
        const StepList* simple = inst.leaf("B_SPLINE_CURVE_WITH_KNOTS");
        const StepList* core = inst.leaf("B_SPLINE_CURVE");
        const StepList* rational = inst.leaf("RATIONAL_B_SPLINE_CURVE");

        int degree = 0;
        const StepList* cpList = nullptr;
        const StepList* multsL = nullptr;
        const StepList* knotsL = nullptr;

        if (core != nullptr && simple != nullptr) {
            // Complex instance: B_SPLINE_CURVE(degree, cps, ...) +
            // B_SPLINE_CURVE_WITH_KNOTS(mults, knots, spec).
            if (core->size() < 2 || !(*core)[1].isList()) return nullptr;
            degree = static_cast<int>((*core)[0].num);
            cpList = (*core)[1].items.get();
            if (simple->size() < 2 || !(*simple)[0].isList() || !(*simple)[1].isList()) {
                return nullptr;
            }
            multsL = (*simple)[0].items.get();
            knotsL = (*simple)[1].items.get();
        } else if (simple != nullptr) {
            // Simple: ('', degree, cps, form, closed, selfint, mults, knots, spec).
            if (simple->size() < 8 || !(*simple)[2].isList() || !(*simple)[6].isList() ||
                !(*simple)[7].isList()) {
                return nullptr;
            }
            degree = static_cast<int>((*simple)[1].num);
            cpList = (*simple)[2].items.get();
            multsL = (*simple)[6].items.get();
            knotsL = (*simple)[7].items.get();
        } else {
            return nullptr;
        }

        std::vector<Vec3> cps;
        for (const StepValue& r : *cpList) {
            if (!r.isRef()) return nullptr;
            auto p = readPoint(r.ref);
            if (!p) return nullptr;
            cps.push_back(*p);
        }
        std::vector<double> weights(cps.size(), 1.0);
        if (rational != nullptr && !rational->empty() && (*rational)[0].isList()) {
            const StepList& wl = *(*rational)[0].items;
            for (size_t i = 0; i < wl.size() && i < weights.size(); ++i) weights[i] = wl[i].num;
        }
        std::vector<double> knots = expandKnots(*multsL, *knotsL);
        if (cps.size() < 2 || knots.size() != cps.size() + degree + 1) return nullptr;
        return std::make_shared<geo::NurbsCurve>(std::move(cps), std::move(weights),
                                                 std::move(knots), degree);
    }

    std::shared_ptr<geo::NurbsSurface> readBSplineSurface(const StepInstance& inst) const {
        const StepList* simple = inst.leaf("B_SPLINE_SURFACE_WITH_KNOTS");
        const StepList* core = inst.leaf("B_SPLINE_SURFACE");
        const StepList* rational = inst.leaf("RATIONAL_B_SPLINE_SURFACE");

        int degU = 0;
        int degV = 0;
        const StepList* net = nullptr;
        const StepList* multsU = nullptr;
        const StepList* multsV = nullptr;
        const StepList* knotsU = nullptr;
        const StepList* knotsV = nullptr;

        if (core != nullptr && simple != nullptr) {
            if (core->size() < 3 || !(*core)[2].isList()) return nullptr;
            degU = static_cast<int>((*core)[0].num);
            degV = static_cast<int>((*core)[1].num);
            net = (*core)[2].items.get();
            if (simple->size() < 4 || !(*simple)[0].isList() || !(*simple)[1].isList() ||
                !(*simple)[2].isList() || !(*simple)[3].isList()) {
                return nullptr;
            }
            multsU = (*simple)[0].items.get();
            multsV = (*simple)[1].items.get();
            knotsU = (*simple)[2].items.get();
            knotsV = (*simple)[3].items.get();
        } else if (simple != nullptr) {
            // ('', degU, degV, net, form, uClosed, vClosed, selfint,
            //  multsU, multsV, knotsU, knotsV, spec).
            if (simple->size() < 12 || !(*simple)[3].isList()) return nullptr;
            degU = static_cast<int>((*simple)[1].num);
            degV = static_cast<int>((*simple)[2].num);
            net = (*simple)[3].items.get();
            if (!(*simple)[8].isList() || !(*simple)[9].isList() || !(*simple)[10].isList() ||
                !(*simple)[11].isList()) {
                return nullptr;
            }
            multsU = (*simple)[8].items.get();
            multsV = (*simple)[9].items.get();
            knotsU = (*simple)[10].items.get();
            knotsV = (*simple)[11].items.get();
        } else {
            return nullptr;
        }

        std::vector<std::vector<Vec3>> cps;
        for (const StepValue& row : *net) {
            if (!row.isList()) return nullptr;
            std::vector<Vec3> r;
            for (const StepValue& v : *row.items) {
                if (!v.isRef()) return nullptr;
                auto p = readPoint(v.ref);
                if (!p) return nullptr;
                r.push_back(*p);
            }
            cps.push_back(std::move(r));
        }
        if (cps.empty() || cps.front().empty()) return nullptr;

        std::vector<std::vector<double>> weights(cps.size(),
                                                 std::vector<double>(cps.front().size(), 1.0));
        if (rational != nullptr && !rational->empty() && (*rational)[0].isList()) {
            const StepList& rows = *(*rational)[0].items;
            for (size_t i = 0; i < rows.size() && i < weights.size(); ++i) {
                if (!rows[i].isList()) continue;
                const StepList& wr = *rows[i].items;
                for (size_t j = 0; j < wr.size() && j < weights[i].size(); ++j) {
                    weights[i][j] = wr[j].num;
                }
            }
        }

        std::vector<double> ku = expandKnots(*multsU, *knotsU);
        std::vector<double> kv = expandKnots(*multsV, *knotsV);
        if (ku.size() != cps.size() + degU + 1 || kv.size() != cps.front().size() + degV + 1) {
            return nullptr;
        }
        return std::make_shared<geo::NurbsSurface>(std::move(cps), std::move(weights),
                                                   std::move(ku), std::move(kv), degU, degV);
    }

    /// Curve for an EDGE_CURVE, given the edge's endpoint positions.
    std::shared_ptr<geo::NurbsCurve> readEdgeGeometry(int curveId, const Vec3& start,
                                                      const Vec3& end, int depth = 0) const {
        const StepInstance* inst = m_parser.find(curveId);
        if (inst == nullptr || depth > 4) return nullptr;

        // OCC-style writers (FreeCAD et al.) wrap the 3D geometry: a
        // SURFACE_CURVE / SEAM_CURVE carries the real curve as its curve_3d
        // attribute. Depth-limit the hop to survive self-referential files.
        for (const char* wrapper : {"SURFACE_CURVE", "SEAM_CURVE"}) {
            if (const StepList* sc = inst->leaf(wrapper)) {
                if (sc->size() >= 2 && (*sc)[1].isRef()) {
                    return readEdgeGeometry((*sc)[1].ref, start, end, depth + 1);
                }
                return nullptr;
            }
        }

        if (inst->hasType("B_SPLINE_CURVE_WITH_KNOTS") || inst->hasType("B_SPLINE_CURVE")) {
            return readBSplineCurve(*inst);
        }
        if (inst->hasType("LINE")) {
            // STEP lines are unbounded; the edge vertices bound them exactly.
            return std::make_shared<geo::NurbsCurve>(std::vector<Vec3>{start, end},
                                                     std::vector<double>{1.0, 1.0},
                                                     std::vector<double>{0.0, 0.0, 1.0, 1.0}, 1);
        }
        if (const StepList* circ = inst->leaf("CIRCLE")) {
            if (circ->size() < 3 || !(*circ)[1].isRef()) return nullptr;
            Vec3 center;
            Vec3 zAxis;
            Vec3 xAxis;
            if (!readPlacement((*circ)[1].ref, center, zAxis, xAxis)) return nullptr;
            const double radius = (*circ)[2].num;
            const Vec3 yAxis = zAxis.cross(xAxis);
            if ((start - end).length() < kMergeTol) {
                // Closed seam edge: anchor the reconstructed curve at the
                // recorded seam vertex, not at the placement X-axis.
                const Vec3 d = start - center;
                const double a0 =
                    (d.length() > kMergeTol) ? std::atan2(d.dot(yAxis), d.dot(xAxis)) : 0.0;
                return makeArcInFrame(center, radius, a0, a0 + math::kTwoPi, xAxis, yAxis);
            }
            const Vec3 ds = start - center;
            const Vec3 de = end - center;
            const double a0 = std::atan2(ds.dot(yAxis), ds.dot(xAxis));
            const double a1 = std::atan2(de.dot(yAxis), de.dot(xAxis));
            return makeArcInFrame(center, radius, a0, a1, xAxis, yAxis);
        }
        return nullptr;
    }

    /// Surface for an ADVANCED_FACE; @p loopPoints bounds analytic surfaces.
    std::shared_ptr<geo::NurbsSurface> readFaceGeometry(int surfId,
                                                        const std::vector<Vec3>& loopPoints) const {
        const StepInstance* inst = m_parser.find(surfId);
        if (inst == nullptr) return nullptr;

        if (inst->hasType("B_SPLINE_SURFACE_WITH_KNOTS") || inst->hasType("B_SPLINE_SURFACE")) {
            return readBSplineSurface(*inst);
        }
        if (const StepList* plane = inst->leaf("PLANE")) {
            if (plane->size() < 2 || !(*plane)[1].isRef() || loopPoints.empty()) return nullptr;
            Vec3 origin;
            Vec3 zAxis;
            Vec3 xAxis;
            if (!readPlacement((*plane)[1].ref, origin, zAxis, xAxis)) return nullptr;
            const Vec3 yAxis = zAxis.cross(xAxis);
            double uMin = 1e300;
            double uMax = -1e300;
            double vMin = 1e300;
            double vMax = -1e300;
            for (const Vec3& p : loopPoints) {
                const Vec3 d = p - origin;
                uMin = std::min(uMin, d.dot(xAxis));
                uMax = std::max(uMax, d.dot(xAxis));
                vMin = std::min(vMin, d.dot(yAxis));
                vMax = std::max(vMax, d.dot(yAxis));
            }
            const double uSize = std::max(uMax - uMin, kMergeTol);
            const double vSize = std::max(vMax - vMin, kMergeTol);
            return std::make_shared<geo::NurbsSurface>(geo::NurbsSurface::makePlane(
                origin + xAxis * uMin + yAxis * vMin, xAxis, yAxis, uSize, vSize));
        }
        if (const StepList* cyl = inst->leaf("CYLINDRICAL_SURFACE")) {
            if (cyl->size() < 3 || !(*cyl)[1].isRef() || loopPoints.empty()) return nullptr;
            Vec3 origin;
            Vec3 zAxis;
            Vec3 xAxis;
            if (!readPlacement((*cyl)[1].ref, origin, zAxis, xAxis)) return nullptr;
            const double radius = (*cyl)[2].num;
            double hMin = 1e300;
            double hMax = -1e300;
            for (const Vec3& p : loopPoints) {
                const double h = (p - origin).dot(zAxis);
                hMin = std::min(hMin, h);
                hMax = std::max(hMax, h);
            }
            const double height = std::max(hMax - hMin, kMergeTol);
            return std::make_shared<geo::NurbsSurface>(
                geo::NurbsSurface::makeCylinder(origin + zAxis * hMin, zAxis, radius, height));
        }
        if (const StepList* cone = inst->leaf("CONICAL_SURFACE")) {
            // The radius at the placement, widening along its axis at the
            // semi-angle: the apex is behind it.
            if (cone->size() < 4 || !(*cone)[1].isRef() || loopPoints.empty()) return nullptr;
            Vec3 origin;
            Vec3 zAxis;
            Vec3 xAxis;
            if (!readPlacement((*cone)[1].ref, origin, zAxis, xAxis)) return nullptr;
            const double radius = (*cone)[2].num;
            const double semi = (*cone)[3].num * m_radiansPerUnit;
            if (!(radius >= 0.0) || !(semi > 0.0) || !(semi < math::kPi / 2)) return nullptr;
            const Vec3 apex = origin - zAxis * (radius / std::tan(semi));
            // The nappe the face is on, as far as it reaches.
            double side = 0.0;
            double reach = 0.0;
            for (const Vec3& p : loopPoints) {
                const double h = (p - apex).dot(zAxis);
                side += h;
                reach = std::max(reach, std::abs(h));
            }
            return std::make_shared<geo::NurbsSurface>(geo::NurbsSurface::makeCone(
                apex, side >= 0.0 ? zAxis : zAxis * -1.0, semi, std::max(reach, kMergeTol)));
        }
        if (const StepList* sphere = inst->leaf("SPHERICAL_SURFACE")) {
            if (sphere->size() < 3 || !(*sphere)[1].isRef()) return nullptr;
            Vec3 origin;
            Vec3 zAxis;
            Vec3 xAxis;
            if (!readPlacement((*sphere)[1].ref, origin, zAxis, xAxis)) return nullptr;
            const double radius = (*sphere)[2].num;
            if (!(radius > 0.0)) return nullptr;
            // Turned into the placement's frame, so the file's poles are the
            // surface's; and turned inside out, as NurbsSurface's sphere
            // faces in and a STEP sphere out.
            const auto unit = geo::NurbsSurface::makeSphere(Vec3(), radius);
            const Vec3 yAxis = zAxis.cross(xAxis);
            auto points = unit.controlPoints();
            for (auto& row : points) {
                for (auto& q : row) q = origin + xAxis * q.x + yAxis * q.y + zAxis * q.z;
            }
            const geo::NurbsSurface placed(std::move(points), unit.weights(), unit.knotsU(),
                                           unit.knotsV(), unit.degreeU(), unit.degreeV());
            return reverseSurfaceU(placed);
        }
        if (const StepList* torus = inst->leaf("TOROIDAL_SURFACE")) {
            if (torus->size() < 4 || !(*torus)[1].isRef()) return nullptr;
            Vec3 origin;
            Vec3 zAxis;
            Vec3 xAxis;
            if (!readPlacement((*torus)[1].ref, origin, zAxis, xAxis)) return nullptr;
            const double major = (*torus)[2].num;
            const double minor = (*torus)[3].num;
            if (!(minor > 0.0) || !(major > minor)) return nullptr;
            return std::make_shared<geo::NurbsSurface>(
                geo::NurbsSurface::makeTorus(origin, zAxis, major, minor));
        }
        return nullptr;
    }

    // -- Topology --------------------------------------------------------------

    topo::Vertex* vertexFor(int vertexPointId) {
        auto it = m_vertices.find(vertexPointId);
        if (it != m_vertices.end()) return it->second;
        const StepInstance* inst = m_parser.find(vertexPointId);
        const StepList* args = inst ? inst->leaf("VERTEX_POINT") : nullptr;
        if (args == nullptr || args->size() < 2 || !(*args)[1].isRef()) return nullptr;
        auto p = readPoint((*args)[1].ref);
        if (!p) return nullptr;
        topo::Vertex* v = m_solid->allocVertex();
        v->point = *p;
        v->topoId = topo::TopologyID::make("step", "solid:" + std::to_string(m_solidIndex) +
                                                       "/v:" + std::to_string(vertexPointId));
        m_vertices.emplace(vertexPointId, v);
        return v;
    }

    struct EdgeRecord {
        topo::Edge* edge = nullptr;
        topo::Vertex* start = nullptr;
        topo::Vertex* end = nullptr;
        std::vector<topo::HalfEdge*> uses;  // half-edges referencing this edge
        std::vector<bool> forward;          // per-use: runs start → end?
    };

    EdgeRecord* edgeFor(int edgeCurveId, std::string& error) {
        auto it = m_edges.find(edgeCurveId);
        if (it != m_edges.end()) return &it->second;

        const StepInstance* inst = m_parser.find(edgeCurveId);
        const StepList* args = inst ? inst->leaf("EDGE_CURVE") : nullptr;
        if (args == nullptr || args->size() < 5 || !(*args)[1].isRef() || !(*args)[2].isRef() ||
            !(*args)[3].isRef()) {
            error = "malformed EDGE_CURVE #" + std::to_string(edgeCurveId);
            return nullptr;
        }
        topo::Vertex* start = vertexFor((*args)[1].ref);
        topo::Vertex* end = vertexFor((*args)[2].ref);
        if (start == nullptr || end == nullptr) {
            error = "EDGE_CURVE #" + std::to_string(edgeCurveId) + " has invalid vertices";
            return nullptr;
        }
        const bool sameSense = (*args)[4].kind == StepValue::Enum && (*args)[4].text == "T";
        auto curve = readEdgeGeometry((*args)[3].ref, sameSense ? start->point : end->point,
                                      sameSense ? end->point : start->point);
        if (curve == nullptr) {
            error = "unsupported curve geometry on EDGE_CURVE #" + std::to_string(edgeCurveId);
            return nullptr;
        }

        topo::Edge* e = m_solid->allocEdge();
        e->curve = std::move(curve);
        e->topoId = topo::TopologyID::make(
            "step", "solid:" + std::to_string(m_solidIndex) + "/e:" + std::to_string(edgeCurveId));
        EdgeRecord rec;
        rec.edge = e;
        rec.start = start;
        rec.end = end;
        auto [ins, ok] = m_edges.emplace(edgeCurveId, std::move(rec));
        (void)ok;
        return &ins->second;
    }

    bool buildFace(int faceId, topo::Shell* shell, std::string& error) {
        const StepInstance* inst = m_parser.find(faceId);
        const StepList* args = inst ? inst->leaf("ADVANCED_FACE") : nullptr;
        if (args == nullptr) args = inst ? inst->leaf("FACE_SURFACE") : nullptr;
        if (args == nullptr || args->size() < 3 || !(*args)[1].isList() || !(*args)[2].isRef()) {
            error = "malformed face #" + std::to_string(faceId);
            return false;
        }

        topo::Face* face = m_solid->allocFace();
        face->shell = shell;
        face->topoId = topo::TopologyID::make(
            "step", "solid:" + std::to_string(m_solidIndex) + "/f:" + std::to_string(faceId));
        shell->faces.push_back(face);

        std::vector<Vec3> loopPoints;

        for (const StepValue& boundRef : *(*args)[1].items) {
            if (!boundRef.isRef()) continue;
            const StepInstance* bound = m_parser.find(boundRef.ref);
            const StepList* bArgs = bound ? bound->leaf("FACE_OUTER_BOUND") : nullptr;
            const bool isOuter = bArgs != nullptr;
            if (bArgs == nullptr) bArgs = bound ? bound->leaf("FACE_BOUND") : nullptr;
            if (bArgs == nullptr || bArgs->size() < 3 || !(*bArgs)[1].isRef()) {
                error = "malformed face bound on face #" + std::to_string(faceId);
                return false;
            }
            const bool boundOrientation =
                (*bArgs)[2].kind == StepValue::Enum && (*bArgs)[2].text == "T";

            const StepInstance* loop = m_parser.find((*bArgs)[1].ref);
            const StepList* lArgs = loop ? loop->leaf("EDGE_LOOP") : nullptr;
            if (lArgs == nullptr || lArgs->size() < 2 || !(*lArgs)[1].isList()) {
                error = "malformed EDGE_LOOP on face #" + std::to_string(faceId);
                return false;
            }

            // Collect (edge, forward) pairs in loop order; honour bound orientation.
            std::vector<std::pair<EdgeRecord*, bool>> loopEdges;
            for (const StepValue& oeRef : *(*lArgs)[1].items) {
                if (!oeRef.isRef()) continue;
                const StepInstance* oe = m_parser.find(oeRef.ref);
                const StepList* oeArgs = oe ? oe->leaf("ORIENTED_EDGE") : nullptr;
                if (oeArgs == nullptr || oeArgs->size() < 5 || !(*oeArgs)[3].isRef()) {
                    error = "malformed ORIENTED_EDGE on face #" + std::to_string(faceId);
                    return false;
                }
                EdgeRecord* rec = edgeFor((*oeArgs)[3].ref, error);
                if (rec == nullptr) return false;
                const bool fwd = (*oeArgs)[4].kind == StepValue::Enum && (*oeArgs)[4].text == "T";
                loopEdges.emplace_back(rec, fwd);
            }
            if (!boundOrientation) {
                std::reverse(loopEdges.begin(), loopEdges.end());
                for (auto& [rec, fwd] : loopEdges) fwd = !fwd;
            }
            if (loopEdges.empty()) {
                error = "empty edge loop on face #" + std::to_string(faceId);
                return false;
            }

            // Materialize half-edges for this loop.
            topo::Wire* wire = m_solid->allocWire();
            std::vector<topo::HalfEdge*> hes;
            for (auto& [rec, fwd] : loopEdges) {
                topo::HalfEdge* he = m_solid->allocHalfEdge();
                he->origin = fwd ? rec->start : rec->end;
                he->edge = rec->edge;
                he->face = face;
                rec->uses.push_back(he);
                rec->forward.push_back(fwd);
                hes.push_back(he);
                loopPoints.push_back(he->origin->point);
                // Analytic carrier surfaces are sized from the loop extent;
                // curve control hulls bound curved edges (vertices alone
                // under-span e.g. a circular cap, whose single seam vertex
                // would collapse the patch to a point).
                if (rec->edge->curve != nullptr) {
                    for (const Vec3& p : rec->edge->curve->controlPoints()) {
                        loopPoints.push_back(p);
                    }
                }
            }
            for (size_t i = 0; i < hes.size(); ++i) {
                hes[i]->next = hes[(i + 1) % hes.size()];
                hes[i]->prev = hes[(i + hes.size() - 1) % hes.size()];
                if (hes[i]->origin->halfEdge == nullptr) hes[i]->origin->halfEdge = hes[i];
            }
            wire->halfEdge = hes.front();
            if (isOuter && face->outerLoop == nullptr) {
                face->outerLoop = wire;
            } else {
                face->innerLoops.push_back(wire);
            }
        }

        if (face->outerLoop == nullptr) {
            // Files that use plain FACE_BOUND for the outer loop: promote the
            // first inner loop.
            if (face->innerLoops.empty()) {
                error = "face #" + std::to_string(faceId) + " has no bounds";
                return false;
            }
            face->outerLoop = face->innerLoops.front();
            face->innerLoops.erase(face->innerLoops.begin());
        }

        face->surface = readFaceGeometry((*args)[2].ref, loopPoints);
        if (face->surface == nullptr) {
            error = "unsupported surface geometry on face #" + std::to_string(faceId);
            return false;
        }

        // ADVANCED_FACE same_sense: .F. means the face normal is the reverse
        // of the surface normal.  The kernel has no per-face sense flag (its
        // convention is surface normal == outward face normal), so bake the
        // flip into the surface itself.
        const bool sameSenseFace =
            args->size() < 4 || (*args)[3].kind != StepValue::Enum || (*args)[3].text != "F";
        if (!sameSenseFace) {
            face->surface = reverseSurfaceU(*face->surface);
        }
        return true;
    }

    bool linkTwins(std::string& error) {
        for (auto& [id, rec] : m_edges) {
            if (rec.uses.size() != 2) {
                error = "edge #" + std::to_string(id) + " used by " +
                        std::to_string(rec.uses.size()) + " loops (manifold solids need 2)";
                return false;
            }
            if (rec.forward[0] == rec.forward[1]) {
                error = "edge #" + std::to_string(id) + " has inconsistent loop orientations";
                return false;
            }
            rec.uses[0]->twin = rec.uses[1];
            rec.uses[1]->twin = rec.uses[0];
            // Record direction: the forward half-edge (start → end).
            rec.edge->halfEdge = rec.forward[0] ? rec.uses[0] : rec.uses[1];
        }
        return true;
    }

    const StepParser& m_parser;
    int m_solidIndex;
    const std::atomic<bool>* m_cancelled;
    double m_radiansPerUnit;  ///< of the file's plane angle unit
    std::unique_ptr<topo::Solid> m_solid;
    std::unordered_map<int, topo::Vertex*> m_vertices;
    std::map<int, EdgeRecord> m_edges;
};

// ===========================================================================
// Units
// ===========================================================================

/// Millimetres per unit of the length unit instance `id`: an SI_UNIT of
/// metres with its prefix, or a CONVERSION_BASED_UNIT through its measure
/// ('INCH' is 25.4 of a millimetre unit). 0 when it cannot be read.
double unitMillimetres(const StepParser& parser, int id, std::string* name, int depth = 0) {
    const StepInstance* inst = depth > 8 ? nullptr : parser.find(id);
    if (inst == nullptr) return 0.0;
    if (const StepList* si = inst->leaf("SI_UNIT")) {
        if (si->size() < 2 || (*si)[1].kind != StepValue::Enum || (*si)[1].text != "METRE") {
            return 0.0;
        }
        static const std::map<std::string, double> kPrefixes = {
            {"EXA", 1e18},  {"PETA", 1e15},  {"TERA", 1e12},   {"GIGA", 1e9},
            {"MEGA", 1e6},  {"KILO", 1e3},   {"HECTO", 1e2},   {"DECA", 1e1},
            {"DECI", 1e-1}, {"CENTI", 1e-2}, {"MILLI", 1e-3},  {"MICRO", 1e-6},
            {"NANO", 1e-9}, {"PICO", 1e-12}, {"FEMTO", 1e-15}, {"ATTO", 1e-18}};
        double metres = 1.0;
        std::string prefix;
        if ((*si)[0].kind == StepValue::Enum) {
            const auto it = kPrefixes.find((*si)[0].text);
            if (it == kPrefixes.end()) return 0.0;
            metres = it->second;
            prefix = it->first;
            std::transform(prefix.begin(), prefix.end(), prefix.begin(),
                           [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
        }
        if (name) *name = prefix + "metres";
        return metres * 1000.0;
    }
    if (const StepList* cb = inst->leaf("CONVERSION_BASED_UNIT")) {
        if (cb->size() < 2 || !(*cb)[1].isRef()) return 0.0;
        const StepInstance* measure = parser.find((*cb)[1].ref);
        const StepList* args = measure ? measure->leaf("LENGTH_MEASURE_WITH_UNIT") : nullptr;
        if (args == nullptr) args = measure ? measure->leaf("MEASURE_WITH_UNIT") : nullptr;
        if (args == nullptr || args->size() < 2 || !(*args)[1].isRef()) return 0.0;
        // The value: LENGTH_MEASURE(25.4), or a bare real.
        double value = 0.0;
        const StepValue& v = (*args)[0];
        if (v.kind == StepValue::Real) {
            value = v.num;
        } else if (v.kind == StepValue::Typed && v.items && !v.items->empty() &&
                   v.items->front().kind == StepValue::Real) {
            value = v.items->front().num;
        }
        const double base = unitMillimetres(parser, (*args)[1].ref, nullptr, depth + 1);
        if (!(value > 0.0) || !(base > 0.0) || !std::isfinite(value * base)) return 0.0;
        if (name) {
            std::string unit = (*cb)[0].kind == StepValue::Str ? (*cb)[0].text : "";
            std::transform(unit.begin(), unit.end(), unit.begin(),
                           [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
            if (unit == "inch") unit = "inches";
            if (unit == "foot") unit = "feet";
            *name = unit.empty() ? "a converted unit" : unit;
        }
        return value * base;
    }
    return 0.0;
}

/// Radians per unit of the plane angle unit instance `id`: an SI_UNIT of
/// radians, or a CONVERSION_BASED_UNIT through its measure ('DEGREE' is
/// 0.01745 of a radian). 0 when it cannot be read.
double unitRadians(const StepParser& parser, int id, int depth = 0) {
    const StepInstance* inst = depth > 8 ? nullptr : parser.find(id);
    if (inst == nullptr) return 0.0;
    if (const StepList* si = inst->leaf("SI_UNIT")) {
        const bool radian =
            si->size() >= 2 && (*si)[1].kind == StepValue::Enum && (*si)[1].text == "RADIAN";
        return radian ? 1.0 : 0.0;
    }
    if (const StepList* cb = inst->leaf("CONVERSION_BASED_UNIT")) {
        if (cb->size() < 2 || !(*cb)[1].isRef()) return 0.0;
        const StepInstance* measure = parser.find((*cb)[1].ref);
        const StepList* args = measure ? measure->leaf("PLANE_ANGLE_MEASURE_WITH_UNIT") : nullptr;
        if (args == nullptr) args = measure ? measure->leaf("MEASURE_WITH_UNIT") : nullptr;
        if (args == nullptr || args->size() < 2 || !(*args)[1].isRef()) return 0.0;
        double value = 0.0;
        const StepValue& v = (*args)[0];
        if (v.kind == StepValue::Real) {
            value = v.num;
        } else if (v.kind == StepValue::Typed && v.items && !v.items->empty() &&
                   v.items->front().kind == StepValue::Real) {
            value = v.items->front().num;
        }
        const double base = unitRadians(parser, (*args)[1].ref, depth + 1);
        if (!(value > 0.0) || !(base > 0.0) || !std::isfinite(value * base)) return 0.0;
        return value * base;
    }
    return 0.0;
}

/// Radians per plane angle unit in the representation context `contextId`:
/// 1 when it names none, or one that cannot be read.
double contextRadians(const StepParser& parser, int contextId) {
    const StepInstance* ctx = parser.find(contextId);
    const StepList* units = ctx ? ctx->leaf("GLOBAL_UNIT_ASSIGNED_CONTEXT") : nullptr;
    if (units == nullptr || units->empty() || !(*units)[0].isList()) return 1.0;
    for (const StepValue& u : *(*units)[0].items) {
        if (!u.isRef()) continue;
        const StepInstance* unit = parser.find(u.ref);
        if (unit != nullptr && unit->hasType("PLANE_ANGLE_UNIT")) {
            const double radians = unitRadians(parser, u.ref);
            return radians > 0.0 ? radians : 1.0;
        }
    }
    return 1.0;
}

/// Millimetres per length unit in the representation context `contextId`:
/// the LENGTH_UNIT of its GLOBAL_UNIT_ASSIGNED_CONTEXT. 1 when it names
/// none; 0 when it names one that cannot be read.
double contextMillimetres(const StepParser& parser, int contextId, std::string* name) {
    const StepInstance* ctx = parser.find(contextId);
    const StepList* units = ctx ? ctx->leaf("GLOBAL_UNIT_ASSIGNED_CONTEXT") : nullptr;
    if (units == nullptr || units->empty() || !(*units)[0].isList()) return 1.0;
    for (const StepValue& u : *(*units)[0].items) {
        if (!u.isRef()) continue;
        const StepInstance* unit = parser.find(u.ref);
        if (unit != nullptr && unit->hasType("LENGTH_UNIT")) {
            return unitMillimetres(parser, u.ref, name);
        }
    }
    return 1.0;
}

// ===========================================================================
// Product structure (Phase 153)
// ===========================================================================

/// A Part-21 string's text as UTF-8: its \\ and its \X2\...\X0\ (UTF-16),
/// \X4\...\X0\ (UTF-32), \X\hh and \S\c (ISO 8859-1) escapes undone.
std::string decodeStepText(const std::string& in) {
    std::string out;
    const auto put = [&out](std::uint32_t cp) {
        if (cp < 0x80) {
            out += static_cast<char>(cp);
        } else if (cp < 0x800) {
            out += static_cast<char>(0xC0 | (cp >> 6));
            out += static_cast<char>(0x80 | (cp & 0x3F));
        } else if (cp < 0x10000) {
            out += static_cast<char>(0xE0 | (cp >> 12));
            out += static_cast<char>(0x80 | ((cp >> 6) & 0x3F));
            out += static_cast<char>(0x80 | (cp & 0x3F));
        } else if (cp < 0x110000) {
            out += static_cast<char>(0xF0 | (cp >> 18));
            out += static_cast<char>(0x80 | ((cp >> 12) & 0x3F));
            out += static_cast<char>(0x80 | ((cp >> 6) & 0x3F));
            out += static_cast<char>(0x80 | (cp & 0x3F));
        }
    };
    const auto hex = [&in](std::size_t at, std::size_t digits, std::uint32_t& value) {
        if (at + digits > in.size()) return false;
        value = 0;
        for (std::size_t k = 0; k < digits; ++k) {
            const char c = in[at + k];
            const int d = std::isdigit(static_cast<unsigned char>(c)) != 0 ? c - '0'
                          : (c >= 'A' && c <= 'F')                         ? c - 'A' + 10
                          : (c >= 'a' && c <= 'f')                         ? c - 'a' + 10
                                                                           : -1;
            if (d < 0) return false;
            value = value * 16 + static_cast<std::uint32_t>(d);
        }
        return true;
    };
    std::size_t i = 0;
    while (i < in.size()) {
        if (in[i] != '\\') {
            out += in[i++];
            continue;
        }
        if (in.compare(i, 2, "\\\\") == 0) {
            out += '\\';
            i += 2;
        } else if (in.compare(i, 4, "\\X2\\") == 0 || in.compare(i, 4, "\\X4\\") == 0) {
            const std::size_t digits = in[i + 2] == '2' ? 4 : 8;
            std::size_t at = i + 4;
            std::uint32_t unit = 0;
            std::uint32_t high = 0;  // a UTF-16 high surrogate waiting for its low
            while (at < in.size() && in[at] != '\\' && hex(at, digits, unit)) {
                if (digits == 4 && unit >= 0xD800 && unit < 0xDC00) {
                    high = unit;
                } else if (digits == 4 && unit >= 0xDC00 && unit < 0xE000 && high != 0) {
                    put(0x10000 + ((high - 0xD800) << 10) + (unit - 0xDC00));
                    high = 0;
                } else {
                    put(unit);
                }
                at += digits;
            }
            i = in.compare(at, 4, "\\X0\\") == 0 ? at + 4 : at;
        } else if (in.compare(i, 3, "\\X\\") == 0) {
            std::uint32_t byte = 0;
            if (hex(i + 3, 2, byte)) {
                put(byte);
                i += 5;
            } else {
                out += in[i++];
            }
        } else if (in.compare(i, 3, "\\S\\") == 0 && i + 3 < in.size()) {
            put(static_cast<unsigned char>(in[i + 3]) + 128U);
            i += 4;
        } else if (in.compare(i, 2, "\\P") == 0 && i + 3 < in.size() && in[i + 3] == '\\') {
            i += 4;  // a code page chosen for \S\: taken as ISO 8859-1
        } else {
            out += in[i++];
        }
    }
    return out;
}

/// @p text (UTF-8) as a Part-21 string, quoted: its quote and backslash
/// doubled, printable ASCII as it is, and all else \X2\ or \X4\ escaped.
std::string stepText(const std::string& text) {
    std::string out = "'";
    std::string wide;  // a run of UTF-16 code units, in hex
    const auto flush = [&] {
        if (wide.empty()) return;
        out += "\\X2\\" + wide + "\\X0\\";
        wide.clear();
    };
    const auto hex = [](std::uint32_t v, int digits) {
        std::string h(static_cast<std::size_t>(digits), '0');
        for (int k = digits - 1; k >= 0; --k, v >>= 4) {
            h[static_cast<std::size_t>(k)] = "0123456789ABCDEF"[v & 0xF];
        }
        return h;
    };
    std::size_t i = 0;
    while (i < text.size()) {
        const auto c = static_cast<unsigned char>(text[i]);
        std::uint32_t cp = '?';
        std::size_t length = 1;
        if (c < 0x80) {
            cp = c;
        } else if ((c & 0xE0) == 0xC0) {
            cp = c & 0x1FU;
            length = 2;
        } else if ((c & 0xF0) == 0xE0) {
            cp = c & 0x0FU;
            length = 3;
        } else if ((c & 0xF8) == 0xF0) {
            cp = c & 0x07U;
            length = 4;
        }
        bool valid = c < 0x80 || length > 1;
        for (std::size_t k = 1; valid && k < length; ++k) {
            const auto next =
                i + k < text.size() ? static_cast<unsigned char>(text[i + k]) : std::uint8_t{0};
            valid = (next & 0xC0) == 0x80;
            cp = (cp << 6) | (next & 0x3FU);
        }
        if (!valid) {
            cp = '?';
            length = 1;
        }
        i += length;
        if (cp >= 0x20 && cp < 0x7F) {
            flush();
            if (cp == '\'')
                out += "''";
            else if (cp == '\\')
                out += "\\\\";
            else
                out += static_cast<char>(cp);
        } else if (cp < 0x10000) {
            wide += hex(cp, 4);
        } else {
            flush();
            out += "\\X4\\" + hex(cp, 8) + "\\X0\\";
        }
    }
    flush();
    return out + "'";
}

/// The text of @p value when it is a string; empty when it is not.
std::string textOf(const StepValue& value) {
    return value.kind == StepValue::Str ? decodeStepText(value.text) : std::string();
}

/// Whether @p text is empty or only spaces.
bool blank(const std::string& text) {
    return std::all_of(text.begin(), text.end(),
                       [](char c) { return std::isspace(static_cast<unsigned char>(c)) != 0; });
}

/// The context of representation @p repId: the last attribute of any
/// representation, simple or complex ('name', (items), #context).
int representationContext(const StepParser& parser, int repId) {
    const StepInstance* rep = parser.find(repId);
    if (rep == nullptr) return 0;
    for (const auto& [type, args] : rep->leaves) {
        if (args.size() >= 3 && args[1].isList() && args[2].isRef()) return args[2].ref;
    }
    return 0;
}

/// A relationship between two representations, simple or complex:
/// (name, description, rep_1, rep_2), and its transformation's id (0 for
/// none).
struct Relationship {
    int rep1 = 0;
    int rep2 = 0;
    int transformation = 0;
};

std::optional<Relationship> relationshipOf(const StepInstance& inst) {
    Relationship r;
    const StepList* withTransform = inst.leaf("REPRESENTATION_RELATIONSHIP_WITH_TRANSFORMATION");
    if (withTransform != nullptr && !withTransform->empty()) {
        // Complex: its own one attribute. Simple (as some write it): after
        // the relationship's four.
        const StepValue& t = withTransform->size() >= 5 ? (*withTransform)[4] : (*withTransform)[0];
        if (t.isRef()) r.transformation = t.ref;
    }
    for (const char* type : {"REPRESENTATION_RELATIONSHIP", "SHAPE_REPRESENTATION_RELATIONSHIP",
                             "REPRESENTATION_RELATIONSHIP_WITH_TRANSFORMATION"}) {
        const StepList* args = inst.leaf(type);
        if (args != nullptr && args->size() >= 4 && (*args)[2].isRef() && (*args)[3].isRef()) {
            r.rep1 = (*args)[2].ref;
            r.rep2 = (*args)[3].ref;
            return r;
        }
    }
    return std::nullopt;
}

/// The matrix taking an AXIS2_PLACEMENT_3D's own coordinates into its
/// representation's, its origin in millimetres by @p mm.
std::optional<Mat4> axisMatrix(const StepParser& parser, int id, double mm) {
    const auto triple = [&parser](const StepValue& value, const char* type) -> std::optional<Vec3> {
        const StepInstance* inst = value.isRef() ? parser.find(value.ref) : nullptr;
        const StepList* args = inst ? inst->leaf(type) : nullptr;
        if (args == nullptr || args->size() < 2 || !(*args)[1].isList() ||
            (*args)[1].items->size() < 3) {
            return std::nullopt;
        }
        const StepList& c = *(*args)[1].items;
        const Vec3 v{c[0].num, c[1].num, c[2].num};
        if (!std::isfinite(v.x) || !std::isfinite(v.y) || !std::isfinite(v.z)) return std::nullopt;
        return v;
    };
    const StepInstance* inst = parser.find(id);
    const StepList* args = inst ? inst->leaf("AXIS2_PLACEMENT_3D") : nullptr;
    if (args == nullptr || args->size() < 2) return std::nullopt;
    const auto origin = triple((*args)[1], "CARTESIAN_POINT");
    if (!origin) return std::nullopt;
    Vec3 z{0, 0, 1};
    Vec3 x{1, 0, 0};
    if (args->size() > 2) {
        if (const auto d = triple((*args)[2], "DIRECTION"); d && d->length() > 1e-12) {
            z = d->normalized();
        }
    }
    if (args->size() > 3) {
        if (const auto d = triple((*args)[3], "DIRECTION")) x = *d;
    }
    x = x - z * x.dot(z);
    if (x.length() < 1e-9) {
        const Vec3 seed = std::abs(z.x) < 0.9 ? Vec3{1, 0, 0} : Vec3{0, 1, 0};
        x = seed - z * seed.dot(z);
    }
    x = x.normalized();
    const Vec3 y = z.cross(x);
    Mat4 m = Mat4::identity();
    for (int row = 0; row < 3; ++row) {
        m.at(row, 0) = row == 0 ? x.x : row == 1 ? x.y : x.z;
        m.at(row, 1) = row == 0 ? y.x : row == 1 ? y.y : y.z;
        m.at(row, 2) = row == 0 ? z.x : row == 1 ? z.y : z.z;
    }
    m.at(0, 3) = origin->x * mm;
    m.at(1, 3) = origin->y * mm;
    m.at(2, 3) = origin->z * mm;
    return m;
}

/// The inverse of a rotation and a move: exact, as a general inverse is not.
Mat4 rigidInverse(const Mat4& m) {
    Mat4 inv = Mat4::identity();
    for (int r = 0; r < 3; ++r) {
        for (int c = 0; c < 3; ++c) inv.at(r, c) = m.at(c, r);
    }
    for (int r = 0; r < 3; ++r) {
        inv.at(r, 3) =
            -(inv.at(r, 0) * m.at(0, 3) + inv.at(r, 1) * m.at(1, 3) + inv.at(r, 2) * m.at(2, 3));
    }
    return inv;
}

bool isIdentity(const Mat4& m) {
    for (int r = 0; r < 4; ++r) {
        for (int c = 0; c < 4; ++c) {
            if (m.at(r, c) != (r == c ? 1.0 : 0.0)) return false;
        }
    }
    return true;
}

/// MANIFOLD_SOLID_BREPs that make one solid: the shells in one shape
/// representation, with its context (which holds its units).
struct SolidGroup {
    std::vector<int> msbs;
    int context = 0;
    /// The ADVANCED_BREP_SHAPE_REPRESENTATION holding them; 0 for one
    /// outside any.
    int rep = 0;
};

/// A file's solids, grouped into the shapes they make.
std::vector<SolidGroup> solidGroups(const StepParser& parser) {
    std::vector<SolidGroup> groups;
    std::unordered_set<int> grouped;
    for (int repId : parser.allOfType("ADVANCED_BREP_SHAPE_REPRESENTATION")) {
        const StepInstance* rep = parser.find(repId);
        const StepList* args = rep ? rep->leaf("ADVANCED_BREP_SHAPE_REPRESENTATION") : nullptr;
        if (args == nullptr || args->size() < 2 || !(*args)[1].isList()) continue;
        SolidGroup group;
        group.rep = repId;
        if (args->size() > 2 && (*args)[2].isRef()) group.context = (*args)[2].ref;
        for (const StepValue& item : *(*args)[1].items) {
            if (!item.isRef() || grouped.count(item.ref) != 0) continue;
            const StepInstance* it = parser.find(item.ref);
            if (it != nullptr && it->hasType("MANIFOLD_SOLID_BREP")) {
                group.msbs.push_back(item.ref);
                grouped.insert(item.ref);
            }
        }
        if (!group.msbs.empty()) groups.push_back(std::move(group));
    }
    // MSBs outside any representation (minimal files) import one solid each,
    // in the file's first context with units, if it has one.
    int fileContext = 0;
    for (int id : parser.allOfType("GLOBAL_UNIT_ASSIGNED_CONTEXT")) {
        fileContext = id;
        break;
    }
    for (int id : parser.allOfType("MANIFOLD_SOLID_BREP")) {
        if (grouped.count(id) == 0) groups.push_back({{id}, fileContext, 0});
    }
    return groups;
}

/// A file's product structure: its products with shapes, and every
/// placement of each, found by walking its assemblies from the top.
struct ProductStructure {
    struct Product {
        std::string name;
        std::vector<std::size_t> groups;  ///< its solids: indices into the groups
    };
    std::vector<Product> products;
    /// Each product's placements; StepOccurrence::part indexes products.
    std::vector<StepOccurrence> placed;
    /// Whether any product is placed by an assembly.
    bool structured = false;
    /// What could only be followed in part, and what was left out, said in
    /// a report.
    std::vector<std::string> notes;
    std::vector<std::string> leftOut;
};

/// At most this many placements, and this many assemblies walked into: a
/// file whose assemblies multiply beyond them (a part used ten times in each
/// of ten nested levels) is cut short.
constexpr std::size_t kMaxPlacements = 20000;
constexpr std::size_t kMaxVisits = 1000000;
/// Assemblies nested deeper than this are taken for a loop.
constexpr int kMaxDepth = 64;

ProductStructure readStructure(const StepParser& parser, const std::vector<SolidGroup>& groups) {
    ProductStructure structure;
    std::unordered_map<int, std::size_t> groupOfRep;
    for (std::size_t g = 0; g < groups.size(); ++g) {
        if (groups[g].rep != 0) groupOfRep.emplace(groups[g].rep, g);
    }

    // Representations related without a transformation are one shape
    // (a part's SHAPE_REPRESENTATION and the ADVANCED_BREP one holding its
    // solids); those with one place one in the other.
    std::unordered_map<int, std::vector<int>> joined;
    std::map<int, Relationship> placing;  // by the relationship's id
    std::set<int> relationships;
    for (const char* type : {"REPRESENTATION_RELATIONSHIP", "SHAPE_REPRESENTATION_RELATIONSHIP",
                             "REPRESENTATION_RELATIONSHIP_WITH_TRANSFORMATION"}) {
        for (int id : parser.allOfType(type)) relationships.insert(id);
    }
    for (int id : relationships) {
        const auto r = relationshipOf(*parser.find(id));
        if (!r) continue;
        if (r->transformation != 0) {
            placing.emplace(id, *r);
        } else {
            joined[r->rep1].push_back(r->rep2);
            joined[r->rep2].push_back(r->rep1);
        }
    }
    const auto shapeOf = [&](const std::vector<int>& reps) {
        std::set<int> seen(reps.begin(), reps.end());
        std::vector<int> stack(reps.begin(), reps.end());
        while (!stack.empty()) {
            const int rep = stack.back();
            stack.pop_back();
            const auto found = joined.find(rep);
            if (found == joined.end()) continue;
            for (int other : found->second) {
                if (seen.insert(other).second) stack.push_back(other);
            }
        }
        return seen;
    };

    // What each PRODUCT_DEFINITION_SHAPE defines: a product definition, or
    // an occurrence of one in an assembly.
    std::unordered_map<int, int> definedBy;
    for (int id : parser.allOfType("PRODUCT_DEFINITION_SHAPE")) {
        const StepList* args = parser.find(id)->leaf("PRODUCT_DEFINITION_SHAPE");
        if (args != nullptr && args->size() >= 3 && (*args)[2].isRef()) {
            definedBy.emplace(id, (*args)[2].ref);
        }
    }
    // Each product definition's representations.
    std::unordered_map<int, std::vector<int>> repsOf;
    for (int id : parser.allOfType("SHAPE_DEFINITION_REPRESENTATION")) {
        const StepList* args = parser.find(id)->leaf("SHAPE_DEFINITION_REPRESENTATION");
        if (args == nullptr || args->size() < 2 || !(*args)[0].isRef() || !(*args)[1].isRef()) {
            continue;
        }
        const auto definition = definedBy.find((*args)[0].ref);
        if (definition != definedBy.end()) repsOf[definition->second].push_back((*args)[1].ref);
    }

    // The product definitions, in the file's order, and their names.
    std::set<int> definitions;
    for (const char* type :
         {"PRODUCT_DEFINITION", "PRODUCT_DEFINITION_WITH_ASSOCIATED_DOCUMENTS"}) {
        for (int id : parser.allOfType(type)) definitions.insert(id);
    }
    const auto nameOf = [&parser](int definition) {
        const StepInstance* pd = parser.find(definition);
        const StepList* args = pd ? pd->leaf("PRODUCT_DEFINITION") : nullptr;
        if (args == nullptr && pd != nullptr) {
            args = pd->leaf("PRODUCT_DEFINITION_WITH_ASSOCIATED_DOCUMENTS");
        }
        const StepInstance* formation = args != nullptr && args->size() >= 3 && (*args)[2].isRef()
                                            ? parser.find((*args)[2].ref)
                                            : nullptr;
        const StepList* fArgs =
            formation ? formation->leaf("PRODUCT_DEFINITION_FORMATION") : nullptr;
        if (fArgs == nullptr && formation != nullptr) {
            fArgs = formation->leaf("PRODUCT_DEFINITION_FORMATION_WITH_SPECIFIED_SOURCE");
        }
        const StepInstance* product = fArgs != nullptr && fArgs->size() >= 3 && (*fArgs)[2].isRef()
                                          ? parser.find((*fArgs)[2].ref)
                                          : nullptr;
        const StepList* pArgs = product ? product->leaf("PRODUCT") : nullptr;
        if (pArgs == nullptr || pArgs->size() < 2) return std::string();
        const std::string name = textOf((*pArgs)[1]);
        return blank(name) ? textOf((*pArgs)[0]) : name;
    };

    // Products: the definitions with solids. Which rep of theirs each is,
    // to tell which side of a placement a part is on.
    std::unordered_map<int, std::size_t> productOf;
    std::unordered_map<int, std::set<int>> shapeReps;
    std::vector<bool> inProduct(groups.size(), false);
    for (int definition : definitions) {
        const auto reps = repsOf.find(definition);
        if (reps == repsOf.end()) continue;
        std::set<int> shape = shapeOf(reps->second);
        ProductStructure::Product product;
        for (int rep : shape) {
            const auto group = groupOfRep.find(rep);
            if (group != groupOfRep.end()) product.groups.push_back(group->second);
        }
        std::sort(product.groups.begin(), product.groups.end());
        shapeReps.emplace(definition, std::move(shape));
        if (product.groups.empty()) continue;
        for (std::size_t g : product.groups) inProduct[g] = true;
        product.name = nameOf(definition);
        productOf.emplace(definition, structure.products.size());
        structure.products.push_back(std::move(product));
    }

    // Each use of a product in an assembly, and the relationship placing it.
    struct Use {
        int occurrence = 0;
        int parent = 0;
        int child = 0;
        /// What it calls this use of its part: its name, unless that is
        /// blank or a writer's stock text; then the part's.
        std::string name;
    };
    std::map<int, std::vector<Use>> usesIn;  // by the assembly's definition
    std::set<int> used;
    for (int id : parser.allOfType("NEXT_ASSEMBLY_USAGE_OCCURRENCE")) {
        const StepList* args = parser.find(id)->leaf("NEXT_ASSEMBLY_USAGE_OCCURRENCE");
        if (args == nullptr || args->size() < 5 || !(*args)[3].isRef() || !(*args)[4].isRef()) {
            continue;
        }
        std::string name = textOf((*args)[1]);
        if (blank(name) || name == "Next assembly relationship") name = nameOf((*args)[4].ref);
        usesIn[(*args)[3].ref].push_back({id, (*args)[3].ref, (*args)[4].ref, std::move(name)});
        used.insert((*args)[4].ref);
    }
    std::unordered_map<int, int> placedBy;  // occurrence → its relationship
    for (int id : parser.allOfType("CONTEXT_DEPENDENT_SHAPE_REPRESENTATION")) {
        const StepList* args = parser.find(id)->leaf("CONTEXT_DEPENDENT_SHAPE_REPRESENTATION");
        if (args == nullptr || args->size() < 2 || !(*args)[0].isRef() || !(*args)[1].isRef()) {
            continue;
        }
        const auto occurrence = definedBy.find((*args)[1].ref);
        if (occurrence != definedBy.end()) placedBy.emplace(occurrence->second, (*args)[0].ref);
    }
    std::unordered_map<int, double> mmOfRep;
    const auto mmOf = [&](int rep) {
        const auto found = mmOfRep.find(rep);
        if (found != mmOfRep.end()) return found->second;
        const int context = representationContext(parser, rep);
        double mm = context != 0 ? contextMillimetres(parser, context, nullptr) : 1.0;
        if (!(mm > 0.0)) mm = 1.0;
        mmOfRep.emplace(rep, mm);
        return mm;
    };
    // The use's part's coordinates into its assembly's: the transformation
    // takes rep_1's into rep_2's, the part's into the assembly's as written
    // by most, inverted when the part is rep_2.
    const auto placement = [&](const Use& use) -> std::optional<Mat4> {
        const auto by = placedBy.find(use.occurrence);
        if (by == placedBy.end()) return std::nullopt;
        const auto r = placing.find(by->second);
        if (r == placing.end()) return std::nullopt;
        const StepInstance* idt = parser.find(r->second.transformation);
        const StepList* args = idt ? idt->leaf("ITEM_DEFINED_TRANSFORMATION") : nullptr;
        if (args == nullptr || args->size() < 4 || !(*args)[2].isRef() || !(*args)[3].isRef()) {
            return std::nullopt;
        }
        const auto from = axisMatrix(parser, (*args)[2].ref, mmOf(r->second.rep1));
        const auto to = axisMatrix(parser, (*args)[3].ref, mmOf(r->second.rep2));
        if (!from || !to) return std::nullopt;
        const Mat4 t = *to * rigidInverse(*from);
        const auto part = shapeReps.find(use.child);
        const bool partSecond = part != shapeReps.end() &&
                                part->second.count(r->second.rep2) != 0 &&
                                part->second.count(r->second.rep1) == 0;
        return partSecond ? rigidInverse(t) : t;
    };

    // From each top assembly (a definition no assembly uses) down.
    std::set<int> onPath;
    std::vector<bool> placedProduct(structure.products.size(), false);
    bool cut = false;
    std::size_t visits = 0;
    std::set<int> unplaced;  // uses with no placement that can be read
    std::set<int> loops;     // uses of an assembly within itself
    // Each placement is named by the uses down to it ("Sub:1/Bolt:2"); a
    // top product by its own name.
    std::function<void(int, const Mat4&, int, const std::string&)> walk =
        [&](int definition, const Mat4& world, int depth, const std::string& path) {
            if (structure.placed.size() >= kMaxPlacements || ++visits > kMaxVisits) {
                cut = true;
                return;
            }
            const auto product = productOf.find(definition);
            if (product != productOf.end()) {
                structure.placed.push_back(
                    {product->second, depth > 0 ? path : structure.products[product->second].name,
                     world});
                placedProduct[product->second] = true;
                if (depth > 0) structure.structured = true;
            }
            const auto uses = usesIn.find(definition);
            if (uses == usesIn.end()) return;
            onPath.insert(definition);
            for (const Use& use : uses->second) {
                if (onPath.count(use.child) != 0 || depth >= kMaxDepth) {
                    loops.insert(use.occurrence);
                    continue;
                }
                auto t = placement(use);
                if (!t) {
                    unplaced.insert(use.occurrence);
                    t = Mat4::identity();
                }
                walk(use.child, world * *t, depth + 1,
                     path.empty() ? use.name : path + "/" + use.name);
            }
            onPath.erase(definition);
        };
    for (int definition : definitions) {
        if (used.count(definition) == 0) walk(definition, Mat4::identity(), 0, std::string());
    }
    // Which uses, by their ids: the first few.
    const auto listed = [](const std::set<int>& ids) {
        std::string text;
        std::size_t n = 0;
        for (int id : ids) {
            if (n++ == 5) return text + ", ...";
            text += (text.empty() ? "#" : ", #") + std::to_string(id);
        }
        return text;
    };
    if (!unplaced.empty()) {
        structure.notes.push_back(
            std::to_string(unplaced.size()) +
            (unplaced.size() == 1 ? " use of a part has" : " uses of parts have") +
            " no placement that can be read, and " + (unplaced.size() == 1 ? "is" : "are") +
            " placed where drawn (" + listed(unplaced) + ")");
    }
    if (!loops.empty()) {
        structure.notes.push_back("an assembly that contains itself is followed once (" +
                                  listed(loops) + ")");
    }
    if (cut) {
        structure.leftOut.push_back("the assembly places its parts more times than can be read (" +
                                    std::to_string(structure.placed.size()) +
                                    " placements read): the rest are left out");
    }
    // A product no walk reached (only under a loop, or under a missing
    // assembly), where drawn; and solids of no product, each its own.
    for (std::size_t p = 0; p < structure.products.size(); ++p) {
        if (!placedProduct[p]) {
            structure.placed.push_back({p, structure.products[p].name, Mat4::identity()});
        }
    }
    for (std::size_t g = 0; g < groups.size(); ++g) {
        if (inProduct[g]) continue;
        const StepInstance* msb = parser.find(groups[g].msbs.front());
        const StepList* args = msb ? msb->leaf("MANIFOLD_SOLID_BREP") : nullptr;
        std::string name = args != nullptr && !args->empty() ? textOf((*args)[0]) : std::string();
        if (blank(name)) name = "solid " + std::to_string(g + 1);
        structure.placed.push_back({structure.products.size(), name, Mat4::identity()});
        structure.products.push_back({name, {g}});
    }
    return structure;
}

// ===========================================================================
// Reading and writing, shared
// ===========================================================================

/// Parse @p text into @p parser; false, with g_lastError said, when it
/// cannot be, or once @p cancelled is set.
bool parseText(StepParser& parser, const std::atomic<bool>* cancelled) {
    std::string error;
    bool parsed = false;
    try {
        parsed = parser.parse(error, cancelled);
    } catch (const std::exception& e) {
        error = e.what();
    }
    if (cancelled != nullptr && cancelled->load(std::memory_order_relaxed)) {
        g_lastError = kCancelled;
        return false;
    }
    if (!parsed) g_lastError = "STEP parse error: " + error;
    return parsed;
}

/// A file's solid groups, each built once, in millimetres: null where one
/// could not be. What fell short, for a report.
struct BuiltSolids {
    std::vector<std::unique_ptr<topo::Solid>> solids;
    std::vector<std::string> failures;
    std::vector<std::string> approximated;
    std::map<std::string, int> conversions;
    std::string firstError;
};

BuiltSolids buildSolids(const StepParser& parser, const std::vector<SolidGroup>& groups,
                        bool reporting, const std::atomic<bool>* cancelled) {
    BuiltSolids built;
    built.solids.resize(groups.size());
    // Each solid on its own: one that cannot be rebuilt is reported, and the
    // others still come in. Its index in the file names its topology, so a
    // solid's names do not depend on whether the ones before it were read.
    for (size_t index = 0; index < groups.size(); ++index) {
        if (cancelled != nullptr && cancelled->load(std::memory_order_relaxed)) return built;
        const SolidGroup& group = groups[index];
        const std::string which =
            "solid " + std::to_string(index + 1) + " (#" + std::to_string(group.msbs.front()) + ")";
        SolidBuilder builder(parser, static_cast<int>(index), cancelled,
                             group.context != 0 ? contextRadians(parser, group.context) : 1.0);
        std::unique_ptr<topo::Solid> solid;
        std::string error;
        try {
            solid = builder.build(group.msbs, error);
        } catch (const std::exception& e) {
            // The geometry constructors throw on inputs the reader's own
            // checks let through (a degree-0 B-spline, ragged control rows).
            error = e.what();
        }
        if (solid == nullptr) {
            if (built.firstError.empty()) {
                built.firstError = "failed to reconstruct solid #" +
                                   std::to_string(group.msbs.front()) + ": " + error;
            }
            built.failures.push_back(which + ": " + error);
            continue;
        }

        // Into millimetres. A unit that cannot be read is taken as the
        // millimetre, and said so.
        std::string unit;
        const double mm =
            group.context != 0 ? contextMillimetres(parser, group.context, &unit) : 1.0;
        if (!(mm > 0.0)) {
            ++built.conversions["the length unit could not be read; read as millimetres"];
        } else if (mm != 1.0) {
            solid = model::Pattern::transformed(*solid, math::Mat4::scale(mm));
            std::ostringstream factor;
            factor << std::setprecision(10) << mm;
            ++built.conversions["drawn in " + unit + ", scaled by " + factor.str() +
                                " into millimetres"];
        }
        // Its curved faces go in as facets (ImportedBodyFeature): say where
        // that falls short.
        if (reporting) {
            const auto faceted = model::facetCurved(*solid);
            if (!faceted.solid) {
                built.approximated.push_back(
                    which + ": its curved faces could not be cut into facets (" + faceted.error +
                    "), so its volume and Booleans follow its corners alone");
            } else if (!faceted.outlined.empty()) {
                const size_t n = faceted.outlined.size();
                built.approximated.push_back(
                    which + ": " + std::to_string(n) +
                    (n == 1 ? " curved face is" : " curved faces are") +
                    " not bounded by its surface's own edges, and is one facet, its outline");
            }
        }
        built.solids[index] = std::move(solid);
    }
    return built;
}

/// What @p built and @p structure fell short of, into @p report.
void reportRead(const BuiltSolids& built, const ProductStructure& structure, ImportReport* report) {
    if (report == nullptr) return;
    for (const auto& failure : built.failures) report->skipped.push_back(failure);
    for (const auto& note : structure.leftOut) report->skipped.push_back(note);
    for (const auto& note : structure.notes) report->approximated.push_back(note);
    for (const auto& line : built.approximated) report->approximated.push_back(line);
    for (const auto& [what, count] : built.conversions) {
        const std::string line =
            (count == 1 ? std::string("1 solid ") : std::to_string(count) + " solids ") + what;
        if (what.rfind("the length unit", 0) == 0) {
            report->approximated.push_back(line);
        } else {
            report->converted.push_back(line);
        }
    }
}

/// @p solid's names moved from solid @p from's to solid @p to's: a second
/// placement of a part named as a solid of its own.
void renameSolid(topo::Solid& solid, std::size_t from, std::size_t to) {
    const std::string was = "step/solid:" + std::to_string(from) + "/";
    const std::string now = "step/solid:" + std::to_string(to) + "/";
    const auto rename = [&](topo::TopologyID& id) {
        if (id.tag().rfind(was, 0) == 0) {
            id = topo::TopologyID::fromTag(now + id.tag().substr(was.size()));
        }
    };
    for (auto& v : solid.vertices()) rename(v.topoId);
    for (auto& e : solid.edges()) rename(e.topoId);
    for (auto& f : solid.faces()) rename(f.topoId);
}

/// The entities a file starts with: its units and context, its
/// application. Their ids.
struct Preamble {
    int context = 0;
    int appContext = 0;
    int productContext = 0;
};

Preamble writePreamble(StepWriter& w) {
    Preamble pre;
    // Geometric representation context (SI millimetres) shared by all solids.
    const int lengthUnit = w.add("(LENGTH_UNIT() NAMED_UNIT(*) SI_UNIT(.MILLI.,.METRE.))");
    const int angleUnit = w.add("(NAMED_UNIT(*) PLANE_ANGLE_UNIT() SI_UNIT($,.RADIAN.))");
    const int solidAngleUnit = w.add("(NAMED_UNIT(*) SI_UNIT($,.STERADIAN.) SOLID_ANGLE_UNIT())");
    const int uncertainty =
        w.add("UNCERTAINTY_MEASURE_WITH_UNIT(LENGTH_MEASURE(1.E-7),#" + std::to_string(lengthUnit) +
              ",'distance_accuracy_value','confusion accuracy')");
    pre.context = w.add(
        "(GEOMETRIC_REPRESENTATION_CONTEXT(3) GLOBAL_UNCERTAINTY_ASSIGNED_CONTEXT((#" +
        std::to_string(uncertainty) + ")) GLOBAL_UNIT_ASSIGNED_CONTEXT((#" +
        std::to_string(lengthUnit) + ",#" + std::to_string(angleUnit) + ",#" +
        std::to_string(solidAngleUnit) + ")) REPRESENTATION_CONTEXT('Context #1','3D Context'))");

    pre.appContext = w.add("APPLICATION_CONTEXT('managed model based 3d engineering')");
    w.add(
        "APPLICATION_PROTOCOL_DEFINITION('international standard',"
        "'ap242_managed_model_based_3d_engineering',2020,#" +
        std::to_string(pre.appContext) + ")");

    pre.productContext =
        w.add("PRODUCT_CONTEXT('',#" + std::to_string(pre.appContext) + ",'mechanical')");
    return pre;
}

/// A product and its definition: the definition's id, and its shape's.
struct ProductIds {
    int definition = 0;
    int shape = 0;
};

/// A product, @p quoted its name as Part-21 text, of @p category.
ProductIds writeProduct(StepWriter& w, const Preamble& pre, const std::string& quoted,
                        const char* category) {
    const int product = w.add("PRODUCT(" + quoted + "," + quoted + ",'',(#" +
                              std::to_string(pre.productContext) + "))");
    w.add(std::string("PRODUCT_RELATED_PRODUCT_CATEGORY('") + category + "',$,(#" +
          std::to_string(product) + "))");
    const int formation =
        w.add("PRODUCT_DEFINITION_FORMATION('',$,#" + std::to_string(product) + ")");
    const int pdc = w.add("PRODUCT_DEFINITION_CONTEXT('part definition',#" +
                          std::to_string(pre.appContext) + ",'design')");
    ProductIds ids;
    ids.definition = w.add("PRODUCT_DEFINITION('design',$,#" + std::to_string(formation) + ",#" +
                           std::to_string(pdc) + ")");
    ids.shape = w.add("PRODUCT_DEFINITION_SHAPE('',$,#" + std::to_string(ids.definition) + ")");
    return ids;
}

std::string fileText(const StepWriter& w) {
    std::ostringstream out;
    out << "ISO-10303-21;\n"
        << "HEADER;\n"
        << "FILE_DESCRIPTION(('Horizon CAD B-Rep model'),'2;1');\n"
        << "FILE_NAME('','',('Horizon CAD'),(''),'Horizon STEP writer','Horizon CAD','');\n"
        << "FILE_SCHEMA(('AP242_MANAGED_MODEL_BASED_3D_ENGINEERING_MIM_LF { 1 0 10303 442 3 1 "
           "4 }'));\n"
        << "ENDSEC;\n"
        << "DATA;\n"
        << w.body() << "ENDSEC;\n"
        << "END-ISO-10303-21;\n";
    return out.str();
}

/// Whether @p m is a rotation and a move alone: what an axis placement
/// can say.
bool isRigid(const Mat4& m) {
    constexpr double kTol = 1e-6;
    if (std::abs(m.at(3, 0)) > kTol || std::abs(m.at(3, 1)) > kTol || std::abs(m.at(3, 2)) > kTol ||
        std::abs(m.at(3, 3) - 1.0) > kTol) {
        return false;
    }
    const Vec3 x{m.at(0, 0), m.at(1, 0), m.at(2, 0)};
    const Vec3 y{m.at(0, 1), m.at(1, 1), m.at(2, 1)};
    const Vec3 z{m.at(0, 2), m.at(1, 2), m.at(2, 2)};
    for (double v : {m.at(0, 3), m.at(1, 3), m.at(2, 3)}) {
        if (!std::isfinite(v)) return false;
    }
    return std::abs(x.length() - 1.0) < kTol && std::abs(y.length() - 1.0) < kTol &&
           std::abs(z.length() - 1.0) < kTol && std::abs(x.dot(y)) < kTol &&
           std::abs(y.dot(z)) < kTol && std::abs(z.dot(x)) < kTol && x.cross(y).dot(z) > 0.0;
}

}  // namespace

// ===========================================================================
// Public API
// ===========================================================================

std::string StepFormat::toString(const std::vector<const topo::Solid*>& solids,
                                 const WriteOptions& options, WriteReport* report) {
    StepWriter w;
    if (report != nullptr) {
        report->faceted.clear();
        report->leftOut.clear();
    }
    const Preamble pre = writePreamble(w);

    int index = 0;
    for (const topo::Solid* solid : solids) {
        if (solid == nullptr) continue;
        const std::vector<int> msbs = writeSolid(w, *solid, index, options.asDesigned,
                                                 report != nullptr ? &report->faceted : nullptr);
        if (msbs.empty()) continue;

        const std::string name = "'part_" + std::to_string(index) + "'";
        const ProductIds ids = writeProduct(w, pre, name, "part");
        const int rep = w.add("ADVANCED_BREP_SHAPE_REPRESENTATION(" + name + "," +
                              StepWriter::refList(msbs) + ",#" + std::to_string(pre.context) + ")");
        w.add("SHAPE_DEFINITION_REPRESENTATION(#" + std::to_string(ids.shape) + ",#" +
              std::to_string(rep) + ")");
        ++index;
    }
    return fileText(w);
}

std::string StepFormat::assemblyToString(const std::string& name,
                                         const std::vector<StepWritePart>& parts,
                                         const std::vector<StepOccurrence>& occurrences,
                                         const WriteOptions& options, WriteReport* report) {
    StepWriter w;
    if (report != nullptr) {
        report->faceted.clear();
        report->leftOut.clear();
    }
    const auto leaveOut = [report](const std::string& why) {
        if (report != nullptr) report->leftOut.push_back(why);
    };
    const Preamble pre = writePreamble(w);
    const auto direction = [&w](const Vec3& d) {
        return w.add("DIRECTION('',(" + fmtReal(d.x) + "," + fmtReal(d.y) + "," + fmtReal(d.z) +
                     "))");
    };
    // The frame @p m takes the part's own into: where it is placed.
    const auto axis = [&](const Mat4& m) {
        const int origin = w.addPoint({m.at(0, 3), m.at(1, 3), m.at(2, 3)});
        const int z = direction(Vec3{m.at(0, 2), m.at(1, 2), m.at(2, 2)}.normalized());
        const int x = direction(Vec3{m.at(0, 0), m.at(1, 0), m.at(2, 0)}.normalized());
        return w.add("AXIS2_PLACEMENT_3D(''," + std::string("#") + std::to_string(origin) + ",#" +
                     std::to_string(z) + ",#" + std::to_string(x) + ")");
    };

    // The occurrences that can be placed, and the parts they use.
    std::vector<const StepOccurrence*> placeable;
    std::vector<bool> used(parts.size(), false);
    for (const StepOccurrence& occurrence : occurrences) {
        const std::string what = "\"" + occurrence.name + "\"";
        if (occurrence.part >= parts.size()) {
            leaveOut(what + ": its part is not in the assembly");
        } else if (!isRigid(occurrence.transform)) {
            leaveOut(what + ": its placement is not a rotation and a move, which STEP places by");
        } else {
            placeable.push_back(&occurrence);
            used[occurrence.part] = true;
        }
    }

    // Each part used, once: its solids and its origin, the frame its
    // placements move.
    struct Written {
        int definition = 0;
        int rep = 0;
        int origin = 0;
    };
    std::vector<std::optional<Written>> written(parts.size());
    int index = 0;
    for (std::size_t p = 0; p < parts.size(); ++p) {
        if (!used[p]) continue;
        std::vector<int> items;
        for (const topo::Solid* body : parts[p].bodies) {
            if (body == nullptr) continue;
            const std::vector<int> msbs =
                writeSolid(w, *body, index++, options.asDesigned,
                           report != nullptr ? &report->faceted : nullptr);
            items.insert(items.end(), msbs.begin(), msbs.end());
        }
        if (items.empty()) continue;
        Written part;
        part.origin = axis(Mat4::identity());
        items.push_back(part.origin);
        const std::string quoted = stepText(parts[p].name);
        const ProductIds ids = writeProduct(w, pre, quoted, "part");
        part.definition = ids.definition;
        part.rep = w.add("ADVANCED_BREP_SHAPE_REPRESENTATION(" + quoted + "," +
                         StepWriter::refList(items) + ",#" + std::to_string(pre.context) + ")");
        w.add("SHAPE_DEFINITION_REPRESENTATION(#" + std::to_string(ids.shape) + ",#" +
              std::to_string(part.rep) + ")");
        written[p] = part;
    }

    // The assembly: its origin and a frame for each placement, and a use
    // of each part at it.
    std::vector<int> items{axis(Mat4::identity())};
    struct Placement {
        const StepOccurrence* occurrence = nullptr;
        Written part;
        int frame = 0;
    };
    std::vector<Placement> placements;
    for (const StepOccurrence* occurrence : placeable) {
        const auto& part = written[occurrence->part];
        if (!part) {
            leaveOut("\"" + occurrence->name + "\": its part has no solid to write");
            continue;
        }
        placements.push_back({occurrence, *part, axis(occurrence->transform)});
        items.push_back(placements.back().frame);
    }
    const std::string quoted = stepText(name);
    const ProductIds assembly = writeProduct(w, pre, quoted, "assembly");
    const int rep = w.add("SHAPE_REPRESENTATION(" + quoted + "," + StepWriter::refList(items) +
                          ",#" + std::to_string(pre.context) + ")");
    w.add("SHAPE_DEFINITION_REPRESENTATION(#" + std::to_string(assembly.shape) + ",#" +
          std::to_string(rep) + ")");
    int number = 0;
    for (const auto& [occurrence, part, frame] : placements) {
        const int use =
            w.add("NEXT_ASSEMBLY_USAGE_OCCURRENCE('" + std::to_string(++number) + "'," +
                  stepText(occurrence->name) + ",'',#" + std::to_string(assembly.definition) +
                  ",#" + std::to_string(part.definition) + ",$)");
        const int shape = w.add("PRODUCT_DEFINITION_SHAPE('','',#" + std::to_string(use) + ")");
        const int transformation =
            w.add("ITEM_DEFINED_TRANSFORMATION('','',#" + std::to_string(part.origin) + ",#" +
                  std::to_string(frame) + ")");
        const int relationship =
            w.add("(REPRESENTATION_RELATIONSHIP('','',#" + std::to_string(part.rep) + ",#" +
                  std::to_string(rep) + ") REPRESENTATION_RELATIONSHIP_WITH_TRANSFORMATION(#" +
                  std::to_string(transformation) + ") SHAPE_REPRESENTATION_RELATIONSHIP())");
        w.add("CONTEXT_DEPENDENT_SHAPE_REPRESENTATION(#" + std::to_string(relationship) + ",#" +
              std::to_string(shape) + ")");
    }
    return fileText(w);
}

bool StepFormat::save(const std::string& filePath, const std::vector<const topo::Solid*>& solids,
                      const WriteOptions& options, WriteReport* report) {
    g_lastError.clear();
    if (solids.empty()) {
        g_lastError = "no solids to export";
        return false;
    }
    std::string error;
    if (!writeFileAtomically(pathFromUtf8(filePath), toString(solids, options, report), &error)) {
        g_lastError = error;
        return false;
    }
    return true;
}

bool StepFormat::saveAssembly(const std::string& filePath, const std::string& name,
                              const std::vector<StepWritePart>& parts,
                              const std::vector<StepOccurrence>& occurrences,
                              const WriteOptions& options, WriteReport* report) {
    g_lastError.clear();
    WriteReport own;
    WriteReport& said = report != nullptr ? *report : own;
    const std::string text = assemblyToString(name, parts, occurrences, options, &said);
    if (said.leftOut.size() >= occurrences.size()) {
        g_lastError = "no component to place";
        return false;
    }
    std::string error;
    if (!writeFileAtomically(pathFromUtf8(filePath), text, &error)) {
        g_lastError = error;
        return false;
    }
    return true;
}

std::vector<std::unique_ptr<topo::Solid>> StepFormat::fromString(
    const std::string& text, ImportReport* report, const std::atomic<bool>* cancelled) {
    g_lastError.clear();
    const auto stopped = [cancelled] {
        return cancelled != nullptr && cancelled->load(std::memory_order_relaxed);
    };
    StepParser parser(text);
    if (!parseText(parser, cancelled)) return {};
    const std::vector<SolidGroup> groups = solidGroups(parser);
    if (groups.empty()) {
        g_lastError = "no MANIFOLD_SOLID_BREP found in file";
        return {};
    }
    BuiltSolids built = buildSolids(parser, groups, report != nullptr, cancelled);
    if (stopped()) {
        g_lastError = kCancelled;
        return {};
    }
    const ProductStructure structure = readStructure(parser, groups);

    std::vector<std::unique_ptr<topo::Solid>> out;
    if (!structure.structured) {
        // Parts alone: each solid where it was drawn, as the file has them.
        for (auto& solid : built.solids) {
            if (solid) out.push_back(std::move(solid));
        }
    } else {
        // Each placement of each part's solids. A solid's first keeps its
        // names (as a file of the part alone names it); each after it is
        // named as a solid of its own, numbered after the file's.
        std::vector<std::size_t> uses(groups.size(), 0);
        for (const StepOccurrence& placed : structure.placed) {
            for (std::size_t g : structure.products[placed.part].groups) ++uses[g];
        }
        std::vector<bool> named(groups.size(), false);
        std::size_t next = groups.size();
        std::set<std::size_t> parts;  // the products placed
        for (const StepOccurrence& placed : structure.placed) {
            if (stopped()) {
                g_lastError = kCancelled;
                return {};
            }
            for (std::size_t g : structure.products[placed.part].groups) {
                auto& solid = built.solids[g];
                if (!solid) continue;
                std::unique_ptr<topo::Solid> copy =
                    --uses[g] == 0 && isIdentity(placed.transform)
                        ? std::move(solid)
                        : model::Pattern::transformed(*solid, placed.transform);
                if (named[g]) renameSolid(*copy, g, next++);
                named[g] = true;
                out.push_back(std::move(copy));
                parts.insert(placed.part);
            }
        }
        if (report != nullptr && !out.empty()) {
            report->converted.push_back("an assembly: " + std::to_string(parts.size()) +
                                        (parts.size() == 1 ? " part placed" : " parts placed") +
                                        " where it puts them, as " + std::to_string(out.size()) +
                                        (out.size() == 1 ? " solid" : " solids"));
        }
    }
    if (out.empty()) {
        g_lastError = built.firstError.empty() ? "no solid could be read" : built.firstError;
        return out;
    }
    reportRead(built, structure, report);
    return out;
}

StepAssembly StepFormat::assemblyFromString(const std::string& text, ImportReport* report,
                                            const std::atomic<bool>* cancelled) {
    g_lastError.clear();
    StepAssembly assembly;
    StepParser parser(text);
    if (!parseText(parser, cancelled)) return assembly;
    const std::vector<SolidGroup> groups = solidGroups(parser);
    if (groups.empty()) {
        g_lastError = "no MANIFOLD_SOLID_BREP found in file";
        return assembly;
    }
    BuiltSolids built = buildSolids(parser, groups, report != nullptr, cancelled);
    if (cancelled != nullptr && cancelled->load(std::memory_order_relaxed)) {
        g_lastError = kCancelled;
        return {};
    }
    const ProductStructure structure = readStructure(parser, groups);

    // Each product with a solid that could be read, a part; its solids
    // moved into it, or copied when another product has them too.
    std::vector<std::size_t> uses(groups.size(), 0);
    for (const auto& product : structure.products) {
        for (std::size_t g : product.groups) ++uses[g];
    }
    constexpr std::size_t kNone = std::numeric_limits<std::size_t>::max();
    std::vector<std::size_t> partOf(structure.products.size(), kNone);
    for (std::size_t p = 0; p < structure.products.size(); ++p) {
        const auto& product = structure.products[p];
        StepAssembly::Part part;
        part.name = blank(product.name) ? "part " + std::to_string(p + 1) : product.name;
        for (std::size_t g : product.groups) {
            auto& solid = built.solids[g];
            if (!solid) continue;
            part.bodies.push_back(--uses[g] == 0
                                      ? std::move(solid)
                                      : model::Pattern::transformed(*solid, Mat4::identity()));
        }
        if (part.bodies.empty()) continue;
        partOf[p] = assembly.parts.size();
        assembly.parts.push_back(std::move(part));
    }
    for (const StepOccurrence& placed : structure.placed) {
        if (partOf[placed.part] == kNone) continue;
        StepOccurrence occurrence = placed;
        occurrence.part = partOf[placed.part];
        if (blank(occurrence.name)) occurrence.name = assembly.parts[occurrence.part].name;
        assembly.occurrences.push_back(std::move(occurrence));
    }
    assembly.structured = structure.structured;
    if (assembly.parts.empty()) {
        g_lastError = built.firstError.empty() ? "no solid could be read" : built.firstError;
        return assembly;
    }
    reportRead(built, structure, report);
    return assembly;
}

namespace {

/// The bytes of the file at @p filePath; nullopt, with g_lastError said,
/// when it cannot be opened.
std::optional<std::string> fileBytes(const std::string& filePath) {
    std::ifstream file(pathFromUtf8(filePath), std::ios::binary);
    if (!file) {
        g_lastError = "cannot open file: " + filePath;
        return std::nullopt;
    }
    std::ostringstream ss;
    ss << file.rdbuf();
    return ss.str();
}

}  // namespace

std::vector<std::unique_ptr<topo::Solid>> StepFormat::load(const std::string& filePath,
                                                           ImportReport* report,
                                                           const std::atomic<bool>* cancelled) {
    g_lastError.clear();
    const auto text = fileBytes(filePath);
    return text ? fromString(*text, report, cancelled)
                : std::vector<std::unique_ptr<topo::Solid>>{};
}

StepAssembly StepFormat::loadAssembly(const std::string& filePath, ImportReport* report,
                                      const std::atomic<bool>* cancelled) {
    g_lastError.clear();
    const auto text = fileBytes(filePath);
    return text ? assemblyFromString(*text, report, cancelled) : StepAssembly{};
}

const std::string& StepFormat::lastError() {
    return g_lastError;
}

}  // namespace hz::io
