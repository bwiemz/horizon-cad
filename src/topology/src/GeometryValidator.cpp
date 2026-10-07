#include "horizon/topology/GeometryValidator.h"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <limits>
#include <map>
#include <sstream>
#include <utility>
#include <vector>

#include "horizon/geometry/curves/NurbsCurve.h"
#include "horizon/geometry/surfaces/NurbsSurface.h"
#include "horizon/math/BoundingBox.h"
#include "horizon/topology/Queries.h"
#include "horizon/topology/Solid.h"

namespace hz::topo {

using hz::math::Vec3;

namespace {

/// Ordered vertex positions of a wire, walking the next chain once.
std::vector<Vec3> loopPoints(const Wire* wire) {
    std::vector<Vec3> pts;
    if (wire == nullptr || wire->halfEdge == nullptr) {
        return pts;
    }
    const HalfEdge* start = wire->halfEdge;
    const HalfEdge* cur = start;
    // Bounded walk: a broken chain is the structural validator's business, not
    // ours, but we must not spin forever on one.
    for (size_t guard = 0; guard < 100000; ++guard) {
        if (cur->origin == nullptr) {
            break;
        }
        pts.push_back(cur->origin->point);
        cur = cur->next;
        if (cur == nullptr || cur == start) {
            break;
        }
    }
    return pts;
}

/// A loop's area vector (Newell), from its points taken relative to its
/// first: the same vector, but from small numbers. From the points as they
/// are, a million millimetres out, each term was about 4e12 and its rounding
/// about 1e-3, where the closed-shell check allows about 1e-6; it held only
/// while each edge's term in one face was exactly the negative of its term
/// in the other, which a fused multiply-add (Apple silicon) does not keep,
/// and every far result was taken for an open shell (Phase 169).
Vec3 newell(const std::vector<Vec3>& pts) {
    Vec3 a(0, 0, 0);
    if (pts.empty()) return a;
    const Vec3& origin = pts.front();
    for (size_t i = 0; i < pts.size(); ++i) {
        const Vec3 p = pts[i] - origin;
        const Vec3 q = pts[(i + 1) % pts.size()] - origin;
        a = a + p.cross(q);
    }
    return a * 0.5;
}

/// Largest pairwise coordinate span of a point set — used to keep the
/// planarity tolerance meaningful for models far from the origin.
double extentOf(const std::vector<Vec3>& pts) {
    if (pts.empty()) {
        return 0.0;
    }
    Vec3 lo = pts[0];
    Vec3 hi = pts[0];
    for (const auto& p : pts) {
        lo.x = std::min(lo.x, p.x);
        lo.y = std::min(lo.y, p.y);
        lo.z = std::min(lo.z, p.z);
        hi.x = std::max(hi.x, p.x);
        hi.y = std::max(hi.y, p.y);
        hi.z = std::max(hi.z, p.z);
    }
    return (hi - lo).length();
}

/// Which side of the line through a and b the point c is, with a band of
/// @p eps (a length) either side of it. The cross product is a length times
/// a length; it is compared as c's distance from the line, so the band is
/// the same at every size. Compared as it was, for a part a hundredth of a
/// millimetre across every point fell in the band, and segments that do not
/// meet were taken for collinear ones that overlap (Phase 169).
int orient2d(double ax, double ay, double bx, double by, double cx, double cy, double eps) {
    const double d = (bx - ax) * (cy - ay) - (by - ay) * (cx - ax);
    const double base = std::hypot(bx - ax, by - ay);
    if (base == 0.0) return 0;
    const double distance = d / base;
    if (distance > eps) return 1;
    if (distance < -eps) return -1;
    return 0;
}

bool onSegment2d(double ax, double ay, double bx, double by, double px, double py, double eps) {
    return px >= std::min(ax, bx) - eps && px <= std::max(ax, bx) + eps &&
           py >= std::min(ay, by) - eps && py <= std::max(ay, by) + eps;
}

/// Proper or improper crossing of two 2D segments.
bool segmentsCross(const std::pair<double, double>& a, const std::pair<double, double>& b,
                   const std::pair<double, double>& c, const std::pair<double, double>& d,
                   double eps) {
    const int o1 = orient2d(a.first, a.second, b.first, b.second, c.first, c.second, eps);
    const int o2 = orient2d(a.first, a.second, b.first, b.second, d.first, d.second, eps);
    const int o3 = orient2d(c.first, c.second, d.first, d.second, a.first, a.second, eps);
    const int o4 = orient2d(c.first, c.second, d.first, d.second, b.first, b.second, eps);

    if (o1 != o2 && o3 != o4 && o1 != 0 && o2 != 0 && o3 != 0 && o4 != 0) {
        return true;
    }
    // Collinear overlap.
    if (o1 == 0 && onSegment2d(a.first, a.second, b.first, b.second, c.first, c.second, eps))
        return true;
    if (o2 == 0 && onSegment2d(a.first, a.second, b.first, b.second, d.first, d.second, eps))
        return true;
    if (o3 == 0 && onSegment2d(c.first, c.second, d.first, d.second, a.first, a.second, eps))
        return true;
    if (o4 == 0 && onSegment2d(c.first, c.second, d.first, d.second, b.first, b.second, eps))
        return true;
    return false;
}

/// A planar loop crosses itself when two non-adjacent segments intersect.
/// Adjacent pairs (and the wrap-around pair) share an endpoint by
/// construction and are skipped.
bool loopSelfIntersects(const std::vector<Vec3>& pts, const Vec3& normal, double tol) {
    const size_t n = pts.size();
    if (n < 4) {
        return false;
    }

    // Build an in-plane basis.
    Vec3 ref = std::abs(normal.x) < 0.9 ? Vec3(1, 0, 0) : Vec3(0, 1, 0);
    Vec3 u = normal.cross(ref);
    const double ulen = u.length();
    if (ulen < 1e-12) {
        return false;
    }
    u = u * (1.0 / ulen);
    const Vec3 v = normal.cross(u);

    // From the loop's first point: small numbers, however far out it is.
    std::vector<std::pair<double, double>> p2(n);
    for (size_t i = 0; i < n; ++i) {
        p2[i] = {(pts[i] - pts[0]).dot(u), (pts[i] - pts[0]).dot(v)};
    }

    for (size_t i = 0; i < n; ++i) {
        const size_t i2 = (i + 1) % n;
        for (size_t j = i + 1; j < n; ++j) {
            const size_t j2 = (j + 1) % n;
            if (i == j || i2 == j || j2 == i) {
                continue;  // Shares an endpoint by construction.
            }
            if (segmentsCross(p2[i], p2[i2], p2[j], p2[j2], tol)) {
                return true;
            }
        }
    }
    return false;
}

constexpr double kInf = std::numeric_limits<double>::infinity();

/// An axis-aligned box as the crossing check keeps it: plain numbers, so
/// that comparing two costs no calls (it compares many).
struct Box {
    double lo[3] = {kInf, kInf, kInf};
    double hi[3] = {-kInf, -kInf, -kInf};
};

void expand(Box& box, const Vec3& p) {
    const double c[3] = {p.x, p.y, p.z};
    for (int k = 0; k < 3; ++k) {
        if (c[k] < box.lo[k]) box.lo[k] = c[k];
        if (c[k] > box.hi[k]) box.hi[k] = c[k];
    }
}

void expand(Box& box, const Box& other) {
    for (int k = 0; k < 3; ++k) {
        if (other.lo[k] < box.lo[k]) box.lo[k] = other.lo[k];
        if (other.hi[k] > box.hi[k]) box.hi[k] = other.hi[k];
    }
}

/// Whether two boxes meet (sides touching count).
bool meet(const Box& a, const Box& b) {
    return a.lo[0] <= b.hi[0] && b.lo[0] <= a.hi[0] && a.lo[1] <= b.hi[1] && b.lo[1] <= a.hi[1] &&
           a.lo[2] <= b.hi[2] && b.lo[2] <= a.hi[2];
}

/// A flat face, as the crossing check reads it: its plane, its box, and
/// where its loops and vertices are kept among all the faces' (see
/// FlatFaces).
struct FlatFace {
    const Shell* shell = nullptr;
    Vec3 origin;
    Vec3 normal;
    Vec3 u;
    Vec3 v;
    double tol = 0.0;
    uint32_t firstLoop = 0;  ///< Its loops are FlatFaces::loops[firstLoop, lastLoop), outer first.
    uint32_t lastLoop = 0;
    uint32_t firstVertex = 0;  ///< Its vertices are FlatFaces::vertices[firstVertex, lastVertex),
    uint32_t lastVertex = 0;   ///< sorted by std::less, for a binary search.
    Box box;
};

/// Every flat face of a solid, their loops kept together rather than each
/// in its own allocation: a loop is a run of points in its face's plane, in
/// coordinates taken from a corner of the face (small numbers, however far
/// out the part is).
struct FlatFaces {
    std::vector<FlatFace> faces;
    std::vector<std::pair<double, double>> points;
    std::vector<std::pair<uint32_t, uint32_t>> loops;  ///< Runs of points.
    std::vector<const Vertex*> vertices;
    std::vector<Vec3> scratch;  ///< Reused while reading each face.
    std::vector<std::pair<uint32_t, uint32_t>> scratchLoops;
};

double segmentDistance2d(const std::pair<double, double>& p, const std::pair<double, double>& a,
                         const std::pair<double, double>& b) {
    const double dx = b.first - a.first;
    const double dy = b.second - a.second;
    const double len2 = dx * dx + dy * dy;
    double t = 0.0;
    if (len2 > 0.0) {
        t = std::clamp(((p.first - a.first) * dx + (p.second - a.second) * dy) / len2, 0.0, 1.0);
    }
    return std::hypot(p.first - (a.first + t * dx), p.second - (a.second + t * dy));
}

/// Whether @p p is inside the loop of @p n points at @p loop (by its
/// crossing number), lowering @p distance to its distance from the loop.
bool insideLoop(const std::pair<double, double>* loop, size_t n, const std::pair<double, double>& p,
                double& distance) {
    bool inside = false;
    for (size_t i = 0, j = n - 1; i < n; j = i++) {
        const auto& a = loop[i];
        const auto& b = loop[j];
        distance = std::min(distance, segmentDistance2d(p, a, b));
        if ((a.second > p.second) != (b.second > p.second)) {
            const double x =
                a.first + (p.second - a.second) * (b.first - a.first) / (b.second - a.second);
            if (p.first < x) inside = !inside;
        }
    }
    return inside;
}

/// @p point (on the face's plane) is inside the face, in its outer loop and
/// none of its holes, and further than the face's tolerance from all of its
/// boundary.
bool strictlyInside(const FlatFaces& all, const FlatFace& face, const Vec3& point) {
    const Vec3 d = point - face.origin;
    const std::pair<double, double> p{d.dot(face.u), d.dot(face.v)};
    double distance = std::numeric_limits<double>::infinity();
    for (uint32_t k = face.firstLoop; k < face.lastLoop; ++k) {
        const auto [first, last] = all.loops[k];
        const bool inside = insideLoop(all.points.data() + first, last - first, p, distance);
        // Inside the outer loop, and outside every hole.
        if (inside != (k == face.firstLoop)) return false;
    }
    return distance > face.tol;
}

/// Reads @p face into @p all, as the crossing check reads it, or leaves
/// @p all as it was and returns false when it is not one it reads: a face
/// without area, or one whose loops are not flat (a curved face, whose
/// inside a plane does not describe).
bool readFlatFace(const Face& face, double tol, FlatFaces& all) {
    if (face.outerLoop == nullptr || face.outerLoop->halfEdge == nullptr || face.shell == nullptr) {
        return false;
    }
    // Every loop's corners, one after another in the scratch, outer first.
    std::vector<Vec3>& pts = all.scratch;
    std::vector<std::pair<uint32_t, uint32_t>>& runs = all.scratchLoops;
    pts.clear();
    runs.clear();
    const size_t firstVertex = all.vertices.size();
    const auto rollBack = [&] {
        all.vertices.resize(firstVertex);
        return false;
    };
    const auto readLoop = [&](const Wire* wire) {
        if (wire == nullptr || wire->halfEdge == nullptr) return false;
        const auto first = static_cast<uint32_t>(pts.size());
        const HalfEdge* start = wire->halfEdge;
        const HalfEdge* cur = start;
        // Bounded walk, as loopPoints'.
        for (size_t guard = 0; guard < 100000 && cur != nullptr; ++guard) {
            if (cur->origin == nullptr) break;
            pts.push_back(cur->origin->point);
            all.vertices.push_back(cur->origin);
            cur = cur->next;
            if (cur == start) break;
        }
        runs.emplace_back(first, static_cast<uint32_t>(pts.size()));
        return pts.size() - first >= 3;
    };
    if (!readLoop(face.outerLoop)) return rollBack();
    for (const Wire* wire : face.innerLoops) {
        if (!readLoop(wire)) return rollBack();
    }

    // The outer loop's area and extent, as newell() and extentOf() read them.
    const auto [outerFirst, outerLast] = runs.front();
    const Vec3& corner = pts[outerFirst];
    Vec3 area(0, 0, 0);
    Box extentBox;
    for (uint32_t i = outerFirst; i < outerLast; ++i) {
        const uint32_t next = i + 1 < outerLast ? i + 1 : outerFirst;
        area = area + (pts[i] - corner).cross(pts[next] - corner);
        expand(extentBox, pts[i]);
    }
    area = area * 0.5;
    const double extent = (Vec3(extentBox.hi[0], extentBox.hi[1], extentBox.hi[2]) -
                           Vec3(extentBox.lo[0], extentBox.lo[1], extentBox.lo[2]))
                              .length();
    const double len = area.length();

    FlatFace flat;
    flat.tol = std::max(tol, GeometryValidator::kDefaultTol * extent);  // as planarity's
    // Narrower than the tolerance: nothing passes through it.
    if (!(len > flat.tol * std::max(extent, tol))) return rollBack();
    flat.shell = face.shell;
    flat.origin = corner;
    flat.normal = area * (1.0 / len);
    for (const Vec3& p : pts) {
        if (std::abs(flat.normal.dot(p - flat.origin)) > flat.tol) return rollBack();
    }
    const Vec3 ref = std::abs(flat.normal.x) < 0.9 ? Vec3(1, 0, 0) : Vec3(0, 1, 0);
    flat.u = flat.normal.cross(ref).normalized();
    flat.v = flat.normal.cross(flat.u);

    flat.firstLoop = static_cast<uint32_t>(all.loops.size());
    const auto offset = static_cast<uint32_t>(all.points.size());
    for (const auto& [first, last] : runs) all.loops.emplace_back(offset + first, offset + last);
    flat.lastLoop = static_cast<uint32_t>(all.loops.size());
    for (const Vec3& p : pts) {
        const Vec3 d = p - flat.origin;
        all.points.emplace_back(d.dot(flat.u), d.dot(flat.v));
        expand(flat.box, p);
    }
    flat.firstVertex = static_cast<uint32_t>(firstVertex);
    flat.lastVertex = static_cast<uint32_t>(all.vertices.size());
    std::sort(all.vertices.begin() + static_cast<std::ptrdiff_t>(firstVertex), all.vertices.end(),
              std::less<const Vertex*>());
    all.faces.push_back(flat);
    return true;
}

/// Whether segments ab and cd meet at a point inside both: within the
/// tolerance (as a face's, kDefaultTol of their length at least) of each
/// other there, and further than it from every end. Parallel segments meet
/// nowhere inside both in a sound solid's way of meeting (they would overlap
/// along a line), and are not counted here.
bool segmentsMeetInside(const Vec3& a, const Vec3& b, const Vec3& c, const Vec3& d, double tol) {
    // Closest points of the two lines (Ericson, Real-Time Collision
    // Detection 5.1.9), from a: small numbers however far out they are.
    const Vec3 d1 = b - a;
    const Vec3 d2 = d - c;
    const Vec3 r = a - c;
    const double aa = d1.dot(d1);
    const double ee = d2.dot(d2);
    const double bb = d1.dot(d2);
    const double denom = aa * ee - bb * bb;
    if (!(aa > 0.0) || !(ee > 0.0) || !(denom > 1e-12 * aa * ee)) return false;  // parallel
    const double cc = d1.dot(r);
    const double ff = d2.dot(r);
    const double s = (bb * ff - cc * ee) / denom;
    const double t = (aa * ff - bb * cc) / denom;
    const double la = std::sqrt(aa);
    const double lc = std::sqrt(ee);
    const double eps = std::max(tol, GeometryValidator::kDefaultTol * std::max(la, lc));
    // Inside both, by more than the tolerance from either end.
    if (s * la <= eps || (1.0 - s) * la <= eps || t * lc <= eps || (1.0 - t) * lc <= eps) {
        return false;
    }
    return ((a + d1 * s) - (c + d2 * t)).length() <= eps;
}

/// Boxes that do not change, indexed once (a linear bounding-volume
/// hierarchy): put in order along a Z-order curve through their centres, so
/// that boxes near one another are mostly near in the order, then the order
/// halved over and over, each part's box holding its members'. Building it
/// costs one sort; finding the boxes that meet one costs about log n and
/// those found, with nothing allocated per question (the crossing check asks
/// once per edge).
class BoxTree {
public:
    explicit BoxTree(std::vector<Box> boxes) : m_boxes(std::move(boxes)) {
        const size_t n = m_boxes.size();
        if (n == 0) return;
        Box centres;
        for (const Box& b : m_boxes) {
            expand(centres, Vec3(0.5 * (b.lo[0] + b.hi[0]), 0.5 * (b.lo[1] + b.hi[1]),
                                 0.5 * (b.lo[2] + b.hi[2])));
        }
        std::vector<std::pair<uint64_t, uint32_t>> keyed(n);
        for (size_t i = 0; i < n; ++i) {
            const Box& b = m_boxes[i];
            uint64_t code = 0;
            for (int k = 0; k < 3; ++k) {
                const double span = centres.hi[k] - centres.lo[k];
                double q =
                    span > 0.0 ? (0.5 * (b.lo[k] + b.hi[k]) - centres.lo[k]) / span * kCells : 0.0;
                if (!(q >= 0.0)) q = 0.0;  // NaN too
                if (q > kCells) q = kCells;
                code |= spread(static_cast<uint64_t>(q)) << k;
            }
            keyed[i] = {code, static_cast<uint32_t>(i)};
        }
        std::sort(keyed.begin(), keyed.end());
        m_order.reserve(n);
        for (const auto& entry : keyed) m_order.push_back(entry.second);
        m_nodes.reserve(2 * (n / kLeafSize + 1));
        build(0, static_cast<uint32_t>(n));
    }

    /// Calls @p visit with the index of every box that meets @p box.
    template <typename Visit>
    void forEachMeeting(const Box& box, Visit&& visit) const {
        if (m_nodes.empty()) return;
        // Halving 2^32 boxes down to leaves takes under 32 steps, and the
        // walk keeps at most one more node than the steps it has taken.
        uint32_t stack[64] = {};
        size_t top = 0;
        stack[top++] = 0;
        while (top > 0) {
            const Node& node = m_nodes[stack[--top]];
            if (!meet(node.box, box)) continue;
            if (node.count > 0) {
                for (uint32_t k = node.first; k < node.first + node.count; ++k) {
                    if (meet(m_boxes[m_order[k]], box)) visit(m_order[k]);
                }
            } else {
                stack[top++] = node.second;
                stack[top++] = node.first;
            }
        }
    }

private:
    struct Node {
        Box box;
        uint32_t first = 0;   ///< A leaf's first entry, or a branch's first part.
        uint32_t second = 0;  ///< A branch's second part.
        uint32_t count = 0;   ///< A leaf's entries; none for a branch.
    };
    static constexpr uint32_t kLeafSize = 4;
    static constexpr double kCells = 2097151.0;  ///< 2^21 - 1: three fit a 64-bit code.

    /// The low 21 bits of @p v, two zero bits put after each.
    static uint64_t spread(uint64_t v) {
        v &= 0x1fffffULL;
        v = (v | v << 32U) & 0x1f00000000ffffULL;
        v = (v | v << 16U) & 0x1f0000ff0000ffULL;
        v = (v | v << 8U) & 0x100f00f00f00f00fULL;
        v = (v | v << 4U) & 0x10c30c30c30c30c3ULL;
        v = (v | v << 2U) & 0x1249249249249249ULL;
        return v;
    }

    uint32_t build(uint32_t first, uint32_t last) {
        const auto index = static_cast<uint32_t>(m_nodes.size());
        m_nodes.emplace_back();
        if (last - first <= kLeafSize) {
            Box box;
            for (uint32_t k = first; k < last; ++k) expand(box, m_boxes[m_order[k]]);
            m_nodes[index].box = box;
            m_nodes[index].first = first;
            m_nodes[index].count = last - first;
            return index;
        }
        const uint32_t mid = first + (last - first) / 2;
        const uint32_t left = build(first, mid);
        const uint32_t right = build(mid, last);
        Box box = m_nodes[left].box;
        expand(box, m_nodes[right].box);
        m_nodes[index].box = box;
        m_nodes[index].first = left;
        m_nodes[index].second = right;
        return index;
    }

    std::vector<Box> m_boxes;
    std::vector<uint32_t> m_order;
    std::vector<Node> m_nodes;
};

/// Edges that pass through a flat face of their own shell, away from its
/// boundary, or through another of its edges. Where two faces of one skin
/// cross, an edge of one of them goes through the other, or (crossing
/// exactly edge on edge) through one of its edges.
///
/// An edge that shares a vertex with a face is not tested against it: the
/// line through the edge meets the face's plane at that vertex, so it can
/// meet it nowhere else unless it lies in the plane, and an edge lying in a
/// face's plane touches the face rather than passing through it. An edge
/// whose end lies on the plane touches it too. The faces and edges near each
/// edge are found by their boxes, all in one index, so the cost grows with
/// the edges and what is near each, not with their product.
int countCrossings(const Solid& solid, double tol) {
    FlatFaces flats;
    flats.faces.reserve(solid.faceCount());
    for (const auto& face : solid.faces()) readFlatFace(face, tol, flats);
    if (flats.faces.empty()) return 0;

    struct Segment {
        const Vertex* a;
        const Vertex* b;
        const Shell* shell;
        double eps;  ///< The tolerance it meets another within (segmentsMeetInside's).
    };
    std::vector<Segment> segments;
    segments.reserve(solid.edgeCount());
    // Faces first, then edges (each grown by its tolerance, so that one
    // passing within it of another is found).
    std::vector<Box> boxes;
    boxes.reserve(flats.faces.size() + solid.edgeCount());
    for (const FlatFace& f : flats.faces) boxes.push_back(f.box);
    const auto faceCount = static_cast<uint32_t>(boxes.size());
    for (const auto& edge : solid.edges()) {
        const HalfEdge* he = edge.halfEdge;
        if (he == nullptr || he->twin == nullptr || he->origin == nullptr ||
            he->twin->origin == nullptr || he->face == nullptr) {
            continue;
        }
        const Vec3& a = he->origin->point;
        const Vec3& b = he->twin->origin->point;
        const double eps = std::max(tol, GeometryValidator::kDefaultTol * (b - a).length());
        Box box;
        expand(box, a);
        expand(box, b);
        for (int k = 0; k < 3; ++k) {
            box.lo[k] -= eps;
            box.hi[k] += eps;
        }
        boxes.push_back(box);
        segments.push_back({he->origin, he->twin->origin, he->face->shell, eps});
    }
    std::vector<Box> queries(boxes.begin() + static_cast<std::ptrdiff_t>(faceCount), boxes.end());
    const BoxTree index(std::move(boxes));

    int crossings = 0;
    const auto less = std::less<const Vertex*>();
    for (uint32_t i = 0; i < segments.size(); ++i) {
        const Segment& s = segments[i];
        const Vec3& a = s.a->point;
        const Vec3& b = s.b->point;
        index.forEachMeeting(queries[i], [&](uint32_t k) {
            if (k >= faceCount) {
                // Two edges of one shell that meet inside both: faces that
                // cross exactly edge on edge, which no edge's passing through
                // a face shows (two equal square sections swept across each
                // other cross only so).
                const uint32_t j = k - faceCount;
                if (j <= i) return;  // each pair once
                const Segment& o = segments[j];
                if (o.shell != s.shell || o.a == s.a || o.a == s.b || o.b == s.a || o.b == s.b) {
                    return;
                }
                if (segmentsMeetInside(a, b, o.a->point, o.b->point, tol)) ++crossings;
                return;
            }
            const FlatFace& f = flats.faces[k];
            if (f.shell != s.shell) return;
            const auto first = flats.vertices.begin() + static_cast<std::ptrdiff_t>(f.firstVertex);
            const auto last = flats.vertices.begin() + static_cast<std::ptrdiff_t>(f.lastVertex);
            if (std::binary_search(first, last, s.a, less) ||
                std::binary_search(first, last, s.b, less)) {
                return;
            }
            const double da = f.normal.dot(a - f.origin);
            const double db = f.normal.dot(b - f.origin);
            const bool through = (da > f.tol && db < -f.tol) || (da < -f.tol && db > f.tol);
            if (through && strictlyInside(flats, f, a + (b - a) * (da / (da - db)))) ++crossings;
        });
    }
    return crossings;
}

/// The vertices of a wire, in loop order, walking the next chain once.
std::vector<const Vertex*> loopVertices(const Wire* wire) {
    std::vector<const Vertex*> out;
    if (wire == nullptr || wire->halfEdge == nullptr) return out;
    const HalfEdge* start = wire->halfEdge;
    const HalfEdge* cur = start;
    for (size_t guard = 0; guard < 100000 && cur != nullptr && cur->origin != nullptr; ++guard) {
        out.push_back(cur->origin);
        cur = cur->next;
        if (cur == start) break;
    }
    return out;
}

/// A face's loops in its plane, from @p origin along @p u and @p v.
using Loop2d = std::vector<std::pair<double, double>>;
Loop2d inPlane(const std::vector<const Vertex*>& loop, const Vec3& origin, const Vec3& u,
               const Vec3& v) {
    Loop2d out;
    out.reserve(loop.size());
    for (const Vertex* vertex : loop) {
        const Vec3 d = vertex->point - origin;
        out.emplace_back(d.dot(u), d.dot(v));
    }
    return out;
}

/// Whether the outer loop and the holes of a flat face cross one another: a
/// hole that crosses the outline, or another hole. Segments that share a
/// vertex touch there and are not counted (a hole may touch its outline at a
/// point); each loop's own crossings are loopSelfIntersects'.
bool loopsCross(const std::vector<std::vector<const Vertex*>>& loops,
                const std::vector<Loop2d>& loops2d, double tol) {
    for (size_t a = 0; a < loops.size(); ++a) {
        for (size_t b = a + 1; b < loops.size(); ++b) {
            const size_t na = loops[a].size();
            const size_t nb = loops[b].size();
            for (size_t i = 0; i < na; ++i) {
                const size_t i2 = (i + 1) % na;
                for (size_t j = 0; j < nb; ++j) {
                    const size_t j2 = (j + 1) % nb;
                    if (loops[a][i] == loops[b][j] || loops[a][i] == loops[b][j2] ||
                        loops[a][i2] == loops[b][j] || loops[a][i2] == loops[b][j2]) {
                        continue;
                    }
                    if (segmentsCross(loops2d[a][i], loops2d[a][i2], loops2d[b][j], loops2d[b][j2],
                                      tol)) {
                        return true;
                    }
                }
            }
        }
    }
    return false;
}

/// Whether @p hole lies outside @p outer: a corner of it off the outline's
/// boundary is not inside it (by its crossing number). A hole whose every
/// corner is on the outline says nothing either way, and is not counted.
bool holeOutside(const Loop2d& hole, const Loop2d& outer, double tol) {
    for (const auto& p : hole) {
        bool inside = false;
        double distance = std::numeric_limits<double>::infinity();
        for (size_t i = 0, j = outer.size() - 1; i < outer.size(); j = i++) {
            const auto& a = outer[i];
            const auto& b = outer[j];
            const double dx = b.first - a.first;
            const double dy = b.second - a.second;
            const double len2 = dx * dx + dy * dy;
            const double t =
                len2 > 0.0
                    ? std::clamp(((p.first - a.first) * dx + (p.second - a.second) * dy) / len2,
                                 0.0, 1.0)
                    : 0.0;
            distance = std::min(
                distance, std::hypot(p.first - a.first - t * dx, p.second - a.second - t * dy));
            if ((a.second > p.second) != (b.second > p.second)) {
                const double x = a.first + (p.second - a.second) * dx / (b.second - a.second);
                if (p.first < x) inside = !inside;
            }
        }
        if (distance > tol) return !inside;
    }
    return false;
}

/// Quantized key for the coincident-vertex bucket scan.
struct CellKey {
    int64_t x, y, z;
    bool operator<(const CellKey& o) const {
        if (x != o.x) return x < o.x;
        if (y != o.y) return y < o.y;
        return z < o.z;
    }
};

}  // namespace

Vec3 GeometryValidator::loopAreaVector(const Face& face) {
    return newell(loopPoints(face.outerLoop));
}

bool GeometryValidator::facePlane(const Face& face, Vec3& originOut, Vec3& normalOut, double tol) {
    if (face.surface == nullptr) {
        // No carrier: the loop defines its own plane.
        const auto pts = loopPoints(face.outerLoop);
        if (pts.size() < 3) {
            return false;
        }
        const Vec3 a = newell(pts);
        const double len = a.length();
        if (len < tol * tol) {
            return false;
        }
        originOut = pts[0];
        normalOut = a * (1.0 / len);
        return true;
    }

    // A carrier is planar when its normal field is constant.  Sampling beats
    // inspecting the control net: it is correct for any parameterization and
    // treats degree-elevated or refined planes the same as `makePlane`.
    const geo::NurbsSurface& s = *face.surface;
    const double u0 = s.uMin();
    const double u1 = s.uMax();
    const double v0 = s.vMin();
    const double v1 = s.vMax();

    // A bilinear patch — the carrier of box, extrusion and pattern faces — is
    // planar exactly when its four control points are, which settles it
    // without evaluating the surface: on a 10,000-body pattern the sampling
    // below took most of a validation that every feature now runs.
    const auto& cp = s.controlPoints();
    if (s.degreeU() == 1 && s.degreeV() == 1 && cp.size() == 2 && cp[0].size() == 2 &&
        cp[1].size() == 2) {
        const Vec3 du = cp[1][0] - cp[0][0];
        const Vec3 dv = cp[0][1] - cp[0][0];
        const Vec3 n = du.cross(dv);  // the normal at (u0, v0), as sampled below
        const double len = n.length();
        if (len > 1e-12 * du.length() * dv.length()) {
            const Vec3 unit = n * (1.0 / len);
            const Vec3 diagonal = cp[1][1] - cp[0][0];
            if (std::abs(unit.dot(diagonal)) > 1e-7 * diagonal.length()) {
                return false;  // Twisted: a curved carrier.
            }
            originOut = s.evaluate((u0 + u1) * 0.5, (v0 + v1) * 0.5);
            normalOut = unit;
            return true;
        }
        // A corner with a collapsed side: fall back to sampling.
    }

    Vec3 ref(0, 0, 0);
    bool haveRef = false;
    for (int i = 0; i <= 2; ++i) {
        for (int j = 0; j <= 2; ++j) {
            const double u = u0 + (u1 - u0) * (i * 0.5);
            const double v = v0 + (v1 - v0) * (j * 0.5);
            Vec3 n = s.normal(u, v);
            const double len = n.length();
            if (len < 1e-12) {
                continue;  // Degenerate sample (pole); ignore.
            }
            n = n * (1.0 / len);
            if (!haveRef) {
                ref = n;
                haveRef = true;
                continue;
            }
            if (n.cross(ref).length() > 1e-7) {
                return false;  // Curved carrier.
            }
        }
    }
    if (!haveRef) {
        return false;
    }
    originOut = s.evaluate((u0 + u1) * 0.5, (v0 + v1) * 0.5);
    normalOut = ref;
    return true;
}

GeometryValidator::Issues GeometryValidator::check(const Solid& solid, double tol, Scope scope) {
    const bool everything = scope == Scope::Everything;
    Issues issues;
    const double areaTol = tol * tol;

    // -- Per-half-edge: vertex chain and twin coincidence --------------------
    for (const auto& face : solid.faces()) {
        std::vector<const Wire*> wires;
        if (face.outerLoop != nullptr) {
            wires.push_back(face.outerLoop);
        }
        for (const auto* inner : face.innerLoops) {
            wires.push_back(inner);
        }
        for (const auto* wire : wires) {
            if (wire == nullptr || wire->halfEdge == nullptr) {
                continue;
            }
            const HalfEdge* start = wire->halfEdge;
            const HalfEdge* cur = start;
            for (size_t guard = 0; guard < 100000; ++guard) {
                if (cur->next == nullptr || cur->twin == nullptr) {
                    break;  // Structural defect; Solid::checkManifold reports it.
                }
                if (cur->next->origin != cur->twin->origin) {
                    ++issues.vertexChainErrors;
                }
                cur = cur->next;
                if (cur == start) {
                    break;
                }
            }
        }
    }

    // -- Per-edge: twin endpoints span one segment; non-degenerate -----------
    for (const auto& edge : solid.edges()) {
        const HalfEdge* he = edge.halfEdge;
        if (he == nullptr || he->twin == nullptr || he->origin == nullptr ||
            he->twin->origin == nullptr) {
            continue;
        }
        const Vec3 a = he->origin->point;
        const Vec3 b = he->twin->origin->point;

        if (a.distanceTo(b) < tol) {
            ++issues.degenerateEdges;
        }

        // The half-edge ends where its next half-edge starts; that position
        // must be the twin's origin, and vice versa.
        if (he->next != nullptr && he->next->origin != nullptr &&
            he->next->origin->point.distanceTo(b) > tol) {
            ++issues.twinCoincidenceErrors;
        } else if (he->twin->next != nullptr && he->twin->next->origin != nullptr &&
                   he->twin->next->origin->point.distanceTo(a) > tol) {
            ++issues.twinCoincidenceErrors;
        }

        if (everything && edge.curve != nullptr) {
            const Vec3 c0 = edge.curve->evaluate(edge.curve->tMin());
            const Vec3 c1 = edge.curve->evaluate(edge.curve->tMax());
            const bool forward = c0.distanceTo(a) <= tol && c1.distanceTo(b) <= tol;
            const bool reverse = c0.distanceTo(b) <= tol && c1.distanceTo(a) <= tol;
            if (!forward && !reverse) {
                ++issues.edgeCurveMismatches;
            }
        }
    }

    // -- Per-face: degeneracy, planarity, self-intersection -------------------
    // Each loop of a face, its holes as well as its outline: the holes were
    // never looked at, so one off the face's plane, crossing its outline, or
    // outside it altogether passed every check.
    for (const auto& face : solid.faces()) {
        const auto pts = loopPoints(face.outerLoop);
        if (pts.size() < 3) {
            ++issues.degenerateFaces;
            continue;
        }
        const Vec3 areaVec = newell(pts);
        if (areaVec.length() < areaTol) {
            ++issues.degenerateFaces;
            continue;
        }
        std::vector<std::vector<Vec3>> holes;
        bool degenerate = false;
        for (const auto* inner : face.innerLoops) {
            holes.push_back(loopPoints(inner));
            if (holes.back().size() < 3 || newell(holes.back()).length() < areaTol) {
                degenerate = true;
            }
        }
        if (degenerate) {
            ++issues.degenerateFaces;
            continue;
        }

        Vec3 origin(0, 0, 0);
        Vec3 normal(0, 0, 0);
        if (!facePlane(face, origin, normal, tol)) {
            continue;  // Curved carrier: planarity does not apply.
        }

        // tol, or a ten-millionth of the face (kDefaultTol relative to its
        // size), whichever is the larger. It was tol times the face's size,
        // and the feature gate already scales tol with the part: a face 100 m
        // long in a part that size got a tolerance of 10 mm, and its two long
        // edges 10 mm apart read as one, a boundary crossing itself.
        const double planeTol = std::max(tol, kDefaultTol * extentOf(pts));
        const auto onPlane = [&](const std::vector<Vec3>& loop) {
            return std::all_of(loop.begin(), loop.end(), [&](const Vec3& p) {
                return std::abs(normal.dot(p - origin)) <= planeTol;
            });
        };
        const bool planar = onPlane(pts) && std::all_of(holes.begin(), holes.end(), onPlane);
        if (!planar) {
            ++issues.nonPlanarLoops;
            continue;  // A non-planar loop's 2D projection is meaningless.
        }

        bool crosses = loopSelfIntersects(pts, normal, planeTol);
        for (const auto& hole : holes)
            crosses = crosses || loopSelfIntersects(hole, normal, planeTol);
        if (!holes.empty()) {
            const Vec3 ref = std::abs(normal.x) < 0.9 ? Vec3(1, 0, 0) : Vec3(0, 1, 0);
            const Vec3 u = normal.cross(ref).normalized();
            const Vec3 v = normal.cross(u);
            std::vector<std::vector<const Vertex*>> loops{loopVertices(face.outerLoop)};
            for (const auto* inner : face.innerLoops) loops.push_back(loopVertices(inner));
            std::vector<Loop2d> loops2d;
            loops2d.reserve(loops.size());
            for (const auto& loop : loops) loops2d.push_back(inPlane(loop, pts.front(), u, v));
            crosses = crosses || loopsCross(loops, loops2d, planeTol);
            for (size_t k = 1; k < loops2d.size(); ++k) {
                if (holeOutside(loops2d[k], loops2d.front(), planeTol)) ++issues.strayHoles;
            }
        }
        if (crosses) {
            ++issues.selfIntersectingLoops;
        }
    }

    // -- Per-shell: the area vectors of a closed skin sum to zero ------------
    {
        std::map<const Shell*, Vec3> sums;
        std::map<const Shell*, double> scales;
        for (const auto& face : solid.faces()) {
            Vec3 a = newell(loopPoints(face.outerLoop));
            for (const auto* inner : face.innerLoops) {
                a = a + newell(loopPoints(inner));
            }
            sums[face.shell] = sums[face.shell] + a;
            scales[face.shell] += a.length();
        }
        for (const auto& [shell, sum] : sums) {
            const double scale = std::max(scales[shell], 1.0);
            if (sum.length() > 1e-9 * scale) {
                ++issues.openShells;
            }
        }
    }

    // -- Per-shell: no face passes through another ---------------------------
    issues.crossingFaces = countCrossings(solid, tol);

    // -- Coincident vertices (reported only) ---------------------------------
    if (everything) {
        const double cell = std::max(tol, 1e-12);
        std::map<CellKey, std::vector<const Vertex*>> buckets;
        for (const auto& v : solid.vertices()) {
            const CellKey k{static_cast<int64_t>(std::floor(v.point.x / cell)),
                            static_cast<int64_t>(std::floor(v.point.y / cell)),
                            static_cast<int64_t>(std::floor(v.point.z / cell))};
            // Probe the 27-cell neighbourhood so a pair straddling a cell wall
            // is still found.
            bool matched = false;
            for (int dx = -1; dx <= 1 && !matched; ++dx) {
                for (int dy = -1; dy <= 1 && !matched; ++dy) {
                    for (int dz = -1; dz <= 1 && !matched; ++dz) {
                        auto it = buckets.find({k.x + dx, k.y + dy, k.z + dz});
                        if (it == buckets.end()) {
                            continue;
                        }
                        for (const auto* other : it->second) {
                            if (other->point.distanceTo(v.point) <= tol) {
                                ++issues.coincidentVertices;
                                matched = true;
                                break;
                            }
                        }
                    }
                }
            }
            buckets[k].push_back(&v);
        }
    }

    return issues;
}

bool GeometryValidator::isGeometricallyValid(const Solid& solid, double tol) {
    return check(solid, tol, Scope::FailingOnly).ok();
}

std::string GeometryValidator::report(const Solid& solid, double tol) {
    const Issues i = check(solid, tol);
    std::ostringstream out;

    auto line = [&out](const char* label, int count, const char* detail) {
        if (count > 0) {
            out << label << ": " << count << " (" << detail << ")\n";
        }
    };

    line("Vertex chain errors", i.vertexChainErrors,
         "he->next->origin != he->twin->origin — loop and edge disagree on a vertex");
    line("Twin coincidence errors", i.twinCoincidenceErrors,
         "an edge's two half-edges span different positions");
    line("Degenerate edges", i.degenerateEdges, "zero-length");
    line("Degenerate faces", i.degenerateFaces, "fewer than 3 vertices, or vanishing area");
    line("Non-planar loops", i.nonPlanarLoops, "a vertex lies off its own planar face");
    line("Self-intersecting loops", i.selfIntersectingLoops,
         "boundary segments cross, a hole's or the outline's");
    line("Stray holes", i.strayHoles, "a hole lies outside its face");
    line("Open shells", i.openShells, "face area vectors do not sum to zero");
    line("Crossing faces", i.crossingFaces,
         "an edge passes through a face of its own shell — the skin runs into itself");
    line("Edge curve mismatches", i.edgeCurveMismatches,
         "curve endpoints differ from the edge vertices — reported, not failing");
    line("Coincident vertices", i.coincidentVertices,
         "distinct vertices at the same position — reported, not failing");

    if (i.ok() && i.edgeCurveMismatches == 0 && i.coincidentVertices == 0) {
        out << "Geometry checks OK\n";
    } else if (i.ok()) {
        out << "Geometry checks OK (with advisories above)\n";
    }
    return out.str();
}

}  // namespace hz::topo
