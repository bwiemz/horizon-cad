#include "MeshCsg.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <utility>

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
    }

    /// @p list less what lies inside this tree's solid (behind a leaf's
    /// plane), split where it crosses.
    std::vector<Poly> clipPolygons(std::vector<Poly> list) const {
        std::vector<Poly> out;
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
        std::vector<std::pair<int, std::vector<Poly>>> stack;
        stack.emplace_back(0, std::move(list));
        while (!stack.empty()) {
            auto [index, polys] = std::move(stack.back());
            stack.pop_back();
            if (polys.empty()) continue;
            if (!m_nodes[static_cast<size_t>(index)].hasPlane) {
                m_nodes[static_cast<size_t>(index)].plane = splitPlane(polys, m_eps);
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
                stack.emplace_back(childOf(index, false), std::move(backList));
            }
            if (!frontList.empty()) {
                stack.emplace_back(childOf(index, true), std::move(frontList));
            }
        }
    }

private:
    /// The plane to split @p polys by: of up to 16 of their own, evenly
    /// spaced, the one that splits fewest and leaves front and back most
    /// even, judged on up to 256 of them. The first polygon's plane, as
    /// csg.js takes, made a prism's tree a chain as deep as its triangle
    /// count, and every clip walk all of it.
    static Plane splitPlane(const std::vector<Poly>& polys, double eps) {
        constexpr size_t kCandidates = 16;
        constexpr size_t kJudged = 256;
        const size_t n = polys.size();
        if (n <= 2) return polys[0].plane;
        const size_t candidates = std::min(n, kCandidates);
        const size_t judged = std::min(n, kJudged);
        size_t best = 0;
        double bestScore = std::numeric_limits<double>::infinity();
        for (size_t c = 0; c < candidates; ++c) {
            const Plane& plane = polys[c * n / candidates].plane;
            double front = 0.0;
            double back = 0.0;
            double spanning = 0.0;
            for (size_t k = 0; k < judged; ++k) {
                int type = 0;
                for (const Vec3& q : polys[k * n / judged].data.points) {
                    const double t = plane.normal.dot(q) - plane.w;
                    type |= (t < -eps) ? BACK : (t > eps) ? FRONT : COPLANAR;
                }
                front += type == FRONT ? 1.0 : 0.0;
                back += type == BACK ? 1.0 : 0.0;
                spanning += type == SPANNING ? 1.0 : 0.0;
            }
            const double score = 8.0 * spanning + std::abs(front - back);
            if (score < bestScore) {
                bestScore = score;
                best = c * n / candidates;
            }
        }
        return polys[best].plane;
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
    std::vector<Node> m_nodes;  ///< the root first
};

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
                                   double planeEps) {
    Bsp nodeA(planeEps);
    Bsp nodeB(planeEps);
    nodeA.build(toPolys(a));
    nodeB.build(toPolys(b));

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

    nodeA.build(nodeB.allPolygons());
    if (type != BooleanType::Union) {
        nodeA.invert();
    }

    std::vector<Poly> merged = nodeA.allPolygons();

    std::vector<CsgPolygon> result;
    result.reserve(merged.size());
    for (auto& poly : merged) {
        result.push_back(std::move(poly.data));
    }
    return result;
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
