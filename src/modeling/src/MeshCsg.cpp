#include "MeshCsg.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <string>
#include <unordered_map>
#include <utility>

#include "horizon/math/BoundingBox.h"

namespace hz::model {

using hz::math::Vec3;

namespace {

struct Plane {
    Vec3 normal;   // unit
    double w = 0;  // normal · x = w
    bool ok = false;

    static Plane fromPolygon(const std::vector<Vec3>& pts) {
        Plane p;
        // Newell's sum about the first point: about the origin, its terms
        // grow with the distance from it, and a triangle a million out came
        // out tilted enough (2e-10 rad) to shift its plane 2e-4 there.
        Vec3 n = Vec3::Zero;
        const size_t count = pts.size();
        for (size_t i = 0; i < count; ++i) {
            const Vec3 a = pts[i] - pts[0];
            const Vec3 b = pts[(i + 1) % count] - pts[0];
            n.x += (a.y - b.y) * (a.z + b.z);
            n.y += (a.z - b.z) * (a.x + b.x);
            n.z += (a.x - b.x) * (a.y + b.y);
        }
        const double len = n.length();
        if (len < 1e-30) return p;
        p.normal = n / len;
        p.w = p.normal.dot(pts[0]);
        p.ok = true;
        return p;
    }

    void flip() {
        normal = normal * -1.0;
        w = -w;
    }
};

struct Poly {
    CsgPolygon data;
    Plane plane;

    void flip() {
        std::reverse(data.points.begin(), data.points.end());
        plane.flip();
    }
};

enum : int { COPLANAR = 0, FRONT = 1, BACK = 2, SPANNING = 3 };

/// Split `poly` by `plane` into the four output buckets (csg.js algorithm),
/// moved there whole when it does not cross it.
void splitPolygon(const Plane& plane, double eps, Poly&& poly, std::vector<Poly>& coplanarFront,
                  std::vector<Poly>& coplanarBack, std::vector<Poly>& front,
                  std::vector<Poly>& back) {
    const auto& pts = poly.data.points;
    const size_t n = pts.size();

    int polygonType = 0;
    std::vector<int> types(n);
    for (size_t i = 0; i < n; ++i) {
        const double t = plane.normal.dot(pts[i]) - plane.w;
        const int type = (t < -eps) ? BACK : (t > eps) ? FRONT : COPLANAR;
        polygonType |= type;
        types[i] = type;
    }

    switch (polygonType) {
        case COPLANAR:
            (plane.normal.dot(poly.plane.normal) > 0 ? coplanarFront : coplanarBack)
                .push_back(std::move(poly));
            break;
        case FRONT:
            front.push_back(std::move(poly));
            break;
        case BACK:
            back.push_back(std::move(poly));
            break;
        case SPANNING: {
            std::vector<Vec3> f;
            std::vector<Vec3> b;
            f.reserve(n + 1);
            b.reserve(n + 1);
            for (size_t i = 0; i < n; ++i) {
                const size_t j = (i + 1) % n;
                const int ti = types[i];
                const int tj = types[j];
                const Vec3& vi = pts[i];
                const Vec3& vj = pts[j];
                if (ti != BACK) f.push_back(vi);
                if (ti != FRONT) b.push_back(vi);
                if ((ti | tj) == SPANNING) {
                    const double denom = plane.normal.dot(vj - vi);
                    if (std::abs(denom) > 1e-30) {
                        const double t = (plane.w - plane.normal.dot(vi)) / denom;
                        const Vec3 v = vi + (vj - vi) * t;
                        f.push_back(v);
                        b.push_back(v);
                    }
                }
            }
            auto emit = [&poly](std::vector<Vec3>&& loop, std::vector<Poly>& out) {
                if (loop.size() < 3) return;
                Poly piece;
                piece.data.points = std::move(loop);
                piece.data.topoId = poly.data.topoId;
                piece.data.surface = nullptr;  // fragment no longer matches source patch
                piece.data.fromA = poly.data.fromA;
                piece.plane = poly.plane;  // splitting preserves the carrier plane
                out.push_back(std::move(piece));
            };
            emit(std::move(f), front);
            emit(std::move(b), back);
            break;
        }
        default:
            break;
    }
}

/// A BSP tree (csg.js structure, double precision) held as an array of
/// nodes, each traversal a loop over an explicit stack: recursion went as
/// deep as the tree, which for a prismatic solid split by its first
/// polygon's plane is its triangle count, and a large part overflowed the
/// 1 MB stack of a Windows thread. The traversals visit in the recursive
/// order (a node, then its front, then its back), so every result, and the
/// order of the fragments it returns, is as it was.
class Bsp {
public:
    /// @p eps: the on-plane band.
    explicit Bsp(double eps) : m_eps(eps), m_nodes(1) {}

    /// Flip inside and outside: every polygon and plane, front for back.
    void invert() {
        for (Node& node : m_nodes) {
            for (auto& p : node.polygons) p.flip();
            if (node.hasPlane) node.plane.flip();
            std::swap(node.front, node.back);
        }
        m_inverted = !m_inverted;
    }

    /// The box the tree's solid lies in, as built, for clipPolygons to pass
    /// over what lies clear of it. Without one, everything is clipped.
    void setBounds(const math::BoundingBox& bounds) { m_bounds = bounds; }

    /// @p list less what lies inside this tree's solid (behind a leaf's
    /// plane), split where it crosses.
    std::vector<Poly> clipPolygons(std::vector<Poly> list) const {
        std::vector<Poly> out;
        // A polygon clear of the solid's box by more than the on-plane band
        // lies outside the solid, near none of its planes: kept whole, or,
        // when the tree is the solid's outside, dropped whole. The tree
        // would only have cut it into pieces on the same side.
        std::vector<Poly> clear;
        if (m_bounds.isValid()) {
            const Vec3 low = m_bounds.min() - Vec3(m_eps, m_eps, m_eps);
            const Vec3 high = m_bounds.max() + Vec3(m_eps, m_eps, m_eps);
            const auto beyond = [&low, &high](const Poly& poly) {
                const auto& pts = poly.data.points;
                const auto all = [&pts](auto test) {
                    return std::all_of(pts.begin(), pts.end(), test);
                };
                return all([&](const Vec3& p) { return p.x < low.x; }) ||
                       all([&](const Vec3& p) { return p.x > high.x; }) ||
                       all([&](const Vec3& p) { return p.y < low.y; }) ||
                       all([&](const Vec3& p) { return p.y > high.y; }) ||
                       all([&](const Vec3& p) { return p.z < low.z; }) ||
                       all([&](const Vec3& p) { return p.z > high.z; });
            };
            const auto split = std::stable_partition(
                list.begin(), list.end(), [&beyond](const Poly& poly) { return !beyond(poly); });
            if (!m_inverted) {
                clear.assign(std::make_move_iterator(split), std::make_move_iterator(list.end()));
            }
            list.erase(split, list.end());
        }
        std::vector<std::pair<int, std::vector<Poly>>> stack;
        stack.emplace_back(0, std::move(list));
        while (!stack.empty()) {
            auto [index, polys] = std::move(stack.back());
            stack.pop_back();
            const Node& node = m_nodes[static_cast<size_t>(index)];
            if (!node.hasPlane) {
                out.insert(out.end(), std::make_move_iterator(polys.begin()),
                           std::make_move_iterator(polys.end()));
                continue;
            }
            std::vector<Poly> f;
            std::vector<Poly> b;
            for (auto& poly : polys) splitPolygon(node.plane, m_eps, std::move(poly), f, b, f, b);
            // The back after the front: pushed first.
            if (node.back >= 0) stack.emplace_back(node.back, std::move(b));
            // (Behind a leaf's plane is inside the solid: dropped.)
            if (node.front >= 0) {
                stack.emplace_back(node.front, std::move(f));
            } else {
                out.insert(out.end(), std::make_move_iterator(f.begin()),
                           std::make_move_iterator(f.end()));
            }
        }
        out.insert(out.end(), std::make_move_iterator(clear.begin()),
                   std::make_move_iterator(clear.end()));
        return out;
    }

    /// Clip every polygon of this tree to @p bsp's solid.
    void clipTo(const Bsp& bsp) {
        for (Node& node : m_nodes) node.polygons = bsp.clipPolygons(std::move(node.polygons));
    }

    /// Every polygon: a node's, then its front's, then its back's.
    std::vector<Poly> allPolygons() const {
        std::vector<Poly> out;
        std::vector<int> stack{0};
        while (!stack.empty()) {
            const Node& node = m_nodes[static_cast<size_t>(stack.back())];
            stack.pop_back();
            out.insert(out.end(), node.polygons.begin(), node.polygons.end());
            if (node.back >= 0) stack.push_back(node.back);
            if (node.front >= 0) stack.push_back(node.front);
        }
        return out;
    }

    /// Add @p list to the tree, splitting it by the planes it meets and
    /// growing nodes where it reaches a leaf.
    void build(std::vector<Poly> list) {
        // Each list with the size a chain is tried at below it: after one
        // that was not convex, only once a list is half its size.
        struct Pending {
            int index;
            std::vector<Poly> polys;
            size_t tryChainUpTo;
        };
        std::vector<Pending> stack;
        stack.push_back({0, std::move(list), std::numeric_limits<size_t>::max()});
        while (!stack.empty()) {
            auto [index, polys, tryChainUpTo] = std::move(stack.back());
            stack.pop_back();
            if (polys.empty()) continue;
            if (!m_nodes[static_cast<size_t>(index)].hasPlane) {
                // A new node given a convex set: the chain, in one pass
                // (Phase 165). Few polygons are built as ever: the check
                // would cost more than it saves. The tree is the same
                // either way, so where the chain is tried is only a matter
                // of time: a solid not convex fails the check at every
                // node, and tried at each, a prism of 10,000 sides with a
                // wavy outline took minutes to cut.
                if (polys.size() >= kChainAtLeast && polys.size() <= tryChainUpTo) {
                    if (buildChain(index, polys)) continue;
                    tryChainUpTo = polys.size() / 2;
                }
                m_nodes[static_cast<size_t>(index)].plane = polys[0].plane;
                m_nodes[static_cast<size_t>(index)].hasPlane = true;
            }
            std::vector<Poly> frontList;
            std::vector<Poly> backList;
            {
                Node& node = m_nodes[static_cast<size_t>(index)];
                for (auto& poly : polys) {
                    splitPolygon(node.plane, m_eps, std::move(poly), node.polygons, node.polygons,
                                 frontList, backList);
                }
            }
            // The front before the back, as the recursion built them: the
            // back pushed first. (A new node can move the array: index it.)
            if (!backList.empty()) {
                stack.push_back({childOf(index, false), std::move(backList), tryChainUpTo});
            }
            if (!frontList.empty()) {
                stack.push_back({childOf(index, true), std::move(frontList), tryChainUpTo});
            }
        }
    }

private:
    /// Below this many polygons a new node is built as ever.
    static constexpr size_t kChainAtLeast = 32;

    /// Build @p list from the new node @p index as a chain, if it is convex
    /// (Phase 165), and say whether it was. build() takes a new node's plane
    /// from the first polygon, keeps there every polygon in that plane, and
    /// sends the rest on; when nothing is in front of any plane it meets, the
    /// rest all go behind, and the tree is a chain of the distinct planes in
    /// the order they first appear. Each polygon, in order, joins the first
    /// of those planes it lies in, or begins the next: the same chain, made
    /// without splitting every polygon against every plane before it. It is
    /// the chain only if no point is in front of a node's plane; else nothing
    /// is made, and build() builds it as ever.
    bool buildChain(int index, std::vector<Poly>& list) {
        // Not convex, found cheaply: a point in front of the plane of one of
        // a few polygons spread through the list. (Their own planes, which
        // may differ from the planes the chain would take; a set refused
        // here that the full check passes is only built the slower way,
        // into the same tree.)
        constexpr size_t kSamples = 16;
        const size_t stride = std::max<size_t>(1, list.size() / kSamples);
        for (size_t i = 0; i < list.size(); i += stride) {
            const Plane& plane = list[i].plane;
            for (const auto& poly : list) {
                for (const Vec3& p : poly.data.points) {
                    if (plane.normal.dot(p) - plane.w > m_eps) return false;
                }
            }
        }
        // In a plane as splitPolygon judges it: every point within m_eps.
        const auto inPlane = [this](const Plane& plane, const Poly& poly) {
            for (const Vec3& p : poly.data.points) {
                if (std::abs(plane.normal.dot(p) - plane.w) > m_eps) return false;
            }
            return true;
        };
        std::vector<Plane> planes;
        std::vector<size_t> nodeOf(list.size());
        for (size_t i = 0; i < list.size(); ++i) {
            size_t k = 0;
            while (k < planes.size() && !inPlane(planes[k], list[i])) ++k;
            if (k == planes.size()) planes.push_back(list[i].plane);
            nodeOf[i] = k;
        }
        // Nothing in front of any node's plane: each point once.
        std::vector<Vec3> points;
        for (const auto& poly : list) {
            points.insert(points.end(), poly.data.points.begin(), poly.data.points.end());
        }
        std::sort(points.begin(), points.end(), [](const Vec3& a, const Vec3& b) {
            return a.x != b.x ? a.x < b.x : a.y != b.y ? a.y < b.y : a.z < b.z;
        });
        points.erase(std::unique(points.begin(), points.end(),
                                 [](const Vec3& a, const Vec3& b) {
                                     return a.x == b.x && a.y == b.y && a.z == b.z;
                                 }),
                     points.end());
        for (const Plane& plane : planes) {
            for (const Vec3& p : points) {
                if (plane.normal.dot(p) - plane.w > m_eps) return false;
            }
        }
        // The chain: node k's plane, its polygons in order, then behind.
        std::vector<int> nodes;
        nodes.reserve(planes.size());
        for (size_t k = 0; k < planes.size(); ++k) {
            if (k > 0) index = childOf(index, false);
            m_nodes[static_cast<size_t>(index)].plane = planes[k];
            m_nodes[static_cast<size_t>(index)].hasPlane = true;
            nodes.push_back(index);
        }
        for (size_t i = 0; i < list.size(); ++i) {
            m_nodes[static_cast<size_t>(nodes[nodeOf[i]])].polygons.push_back(std::move(list[i]));
        }
        list.clear();
        return true;
    }

    struct Node {
        Plane plane;
        bool hasPlane = false;
        int front = -1;  ///< index into m_nodes, or -1
        int back = -1;
        std::vector<Poly> polygons;
    };

    /// The node's front (or back) child, made if it has none.
    int childOf(int index, bool front) {
        int child = front ? m_nodes[static_cast<size_t>(index)].front
                          : m_nodes[static_cast<size_t>(index)].back;
        if (child < 0) {
            child = static_cast<int>(m_nodes.size());
            m_nodes.emplace_back();
            (front ? m_nodes[static_cast<size_t>(index)].front
                   : m_nodes[static_cast<size_t>(index)].back) = child;
        }
        return child;
    }

    double m_eps;
    std::vector<Node> m_nodes;   ///< the root first
    math::BoundingBox m_bounds;  ///< the solid's, when given (setBounds)
    bool m_inverted = false;     ///< the tree is the solid's outside
};

/// @p polys with the pieces of the faces cut into the most pieces first,
/// each face's pieces still in their order. A node of the tree takes its
/// plane from the first polygon it is given: a face of many pieces (a
/// non-convex cap, triangulated) taken first keeps them all at one node,
/// where taken after the side faces each side's plane cut through every
/// sliver of it that it crossed, into pieces in number the square of the
/// sides, and memory ran out.
std::vector<Poly> largestFacesFirst(std::vector<Poly> polys) {
    std::unordered_map<std::string, size_t> pieces;
    for (const auto& poly : polys) {
        if (poly.data.topoId.isValid()) ++pieces[poly.data.topoId.tag()];
    }
    std::vector<size_t> count(polys.size(), 1);
    for (size_t i = 0; i < polys.size(); ++i) {
        if (polys[i].data.topoId.isValid()) count[i] = pieces[polys[i].data.topoId.tag()];
    }
    std::vector<size_t> order(polys.size());
    for (size_t i = 0; i < order.size(); ++i) order[i] = i;
    std::stable_sort(order.begin(), order.end(),
                     [&count](size_t a, size_t b) { return count[a] > count[b]; });
    std::vector<Poly> out;
    out.reserve(polys.size());
    for (const size_t i : order) out.push_back(std::move(polys[i]));
    return out;
}

std::vector<Poly> toPolys(const std::vector<CsgPolygon>& in) {
    std::vector<Poly> out;
    out.reserve(in.size());
    for (const auto& p : in) {
        Poly poly;
        poly.plane = Plane::fromPolygon(p.points);
        if (!poly.plane.ok) continue;  // degenerate input polygon
        poly.data = p;
        out.push_back(std::move(poly));
    }
    return out;
}

}  // namespace

CsgTolerance CsgTolerance::of(const Vec3& low, const Vec3& high) {
    const double extent = (high - low).length();
    const double magnitude = std::max({std::abs(low.x), std::abs(low.y), std::abs(low.z),
                                       std::abs(high.x), std::abs(high.y), std::abs(high.z)});
    CsgTolerance t;
    t.plane = std::max(1e-8 * extent, 64.0 * std::numeric_limits<double>::epsilon() * magnitude);
    if (!(t.plane > 0.0) || !std::isfinite(t.plane)) t.plane = kCsgPlaneEps;
    return t;
}

std::vector<CsgPolygon> csgExecute(const std::vector<CsgPolygon>& a,
                                   const std::vector<CsgPolygon>& b, BooleanType type,
                                   double planeEps, CsgFragments fragments) {
    Bsp nodeA(planeEps);
    Bsp nodeB(planeEps);
    const bool fewest = fragments == CsgFragments::Fewest;
    nodeA.build(fewest ? largestFacesFirst(toPolys(a)) : toPolys(a));
    nodeB.build(fewest ? largestFacesFirst(toPolys(b)) : toPolys(b));
    if (fewest) {
        const auto bounds = [](const std::vector<CsgPolygon>& polygons) {
            math::BoundingBox box;
            for (const auto& poly : polygons) {
                for (const Vec3& p : poly.points) box.expand(p);
            }
            return box;
        };
        nodeA.setBounds(bounds(a));
        nodeB.setBounds(bounds(b));
    }

    switch (type) {
        case BooleanType::Union:
            nodeA.clipTo(nodeB);
            nodeB.clipTo(nodeA);
            nodeB.invert();
            nodeB.clipTo(nodeA);
            nodeB.invert();
            break;
        case BooleanType::Subtract:
            nodeA.invert();
            nodeA.clipTo(nodeB);
            nodeB.clipTo(nodeA);
            nodeB.invert();
            nodeB.clipTo(nodeA);
            nodeB.invert();
            break;
        case BooleanType::Intersect:
            nodeA.invert();
            nodeB.clipTo(nodeA);
            nodeB.invert();
            nodeA.clipTo(nodeB);
            nodeB.clipTo(nodeA);
            break;
    }

    std::vector<Poly> merged;
    if (fewest) {
        // What is left of each, together. Building B's into A's tree, as
        // csg.js does, only cuts them further along A's planes, which costs
        // as much as the clipping did, for pieces merged back after.
        merged = nodeA.allPolygons();
        std::vector<Poly> fromB = nodeB.allPolygons();
        merged.insert(merged.end(), std::make_move_iterator(fromB.begin()),
                      std::make_move_iterator(fromB.end()));
        if (type != BooleanType::Union) {
            for (auto& poly : merged) poly.flip();
        }
    } else {
        nodeA.build(nodeB.allPolygons());
        if (type != BooleanType::Union) {
            nodeA.invert();
        }
        merged = nodeA.allPolygons();
    }

    std::vector<CsgPolygon> result;
    result.reserve(merged.size());
    for (auto& poly : merged) {
        result.push_back(std::move(poly.data));
    }
    return result;
}

namespace {

/// Whether @p points make a convex polygon the CSG can take whole: flat to
/// well within @p planeEps, every corner turning the same way, going round
/// once, and no edge of no length. Anything less is triangulated.
bool convexAndFlat(const std::vector<Vec3>& points, double planeEps) {
    const size_t n = points.size();
    if (n < 4) return false;  // a triangle is taken whole anyway
    const Plane plane = Plane::fromPolygon(points);
    if (!plane.ok) return false;
    for (const Vec3& p : points) {
        if (!(std::abs(plane.normal.dot(p) - plane.w) <= 0.25 * planeEps)) return false;
    }
    double turning = 0.0;
    for (size_t i = 0; i < n; ++i) {
        const Vec3 e1 = points[(i + 1) % n] - points[i];
        const Vec3 e2 = points[(i + 2) % n] - points[(i + 1) % n];
        if (e1.lengthSquared() == 0.0 || e2.lengthSquared() == 0.0) return false;
        const double sine = plane.normal.dot(e1.cross(e2));
        if (sine < 0.0) return false;  // a reflex corner
        turning += std::atan2(sine, e1.dot(e2));
    }
    // Round once, not twice (a star drawn without lifting the pen).
    constexpr double kTwoPi = 6.283185307179586;
    return std::abs(turning - kTwoPi) < 1e-6;
}

}  // namespace

std::vector<CsgPolygon> csgPieces(const std::vector<BoundaryPolygon>& polygons, bool fromA,
                                  double planeEps) {
    std::vector<CsgPolygon> out;
    out.reserve(polygons.size() * 2);
    for (const auto& poly : polygons) {
        if (poly.holes.empty() && convexAndFlat(poly.points, planeEps)) {
            CsgPolygon p;
            p.points = poly.points;
            p.topoId = poly.topoId;
            p.surface = nullptr;
            p.fromA = fromA;
            out.push_back(std::move(p));
            continue;
        }
        for (const auto& tri : BoundaryMesh::triangulatePolygon(poly.points)) {
            CsgPolygon p;
            p.points.assign(tri.begin(), tri.end());
            p.topoId = poly.topoId;
            p.surface = nullptr;
            p.fromA = fromA;
            out.push_back(std::move(p));
        }
    }
    return out;
}

std::vector<CsgPolygon> csgTriangles(const std::vector<BoundaryPolygon>& polygons, bool fromA) {
    std::vector<CsgPolygon> out;
    out.reserve(polygons.size() * 2);
    for (const auto& poly : polygons) {
        for (const auto& tri : BoundaryMesh::triangulatePolygon(poly.points)) {
            CsgPolygon p;
            p.points.assign(tri.begin(), tri.end());
            p.topoId = poly.topoId;
            p.surface = nullptr;
            p.fromA = fromA;
            out.push_back(std::move(p));
        }
    }
    return out;
}

double csgVolume(const std::vector<CsgPolygon>& polygons) {
    // About a point of its own (see BoundaryMesh::signedVolume).
    Vec3 o;
    for (const auto& poly : polygons) {
        if (!poly.points.empty()) {
            o = poly.points[0];
            break;
        }
    }
    double vol6 = 0.0;
    for (const auto& poly : polygons) {
        const auto& p = poly.points;
        for (size_t i = 1; i + 1 < p.size(); ++i) {
            vol6 += (p[0] - o).dot((p[i] - o).cross(p[i + 1] - o));
        }
    }
    return vol6 / 6.0;
}

}  // namespace hz::model
