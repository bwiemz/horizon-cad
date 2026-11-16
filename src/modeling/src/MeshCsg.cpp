#include "MeshCsg.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <string>
#include <unordered_map>
#include <utility>

#include "horizon/math/BoundingBox.h"
#include "horizon/math/RTree.h"

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

/// The work of the csgExecute running on this thread, and its limits.
struct Budget {
    std::size_t work = 0;
    std::size_t workLimit = std::numeric_limits<std::size_t>::max();
    std::size_t splits = 0;
    std::size_t splitLimit = std::numeric_limits<std::size_t>::max();
    const std::atomic<bool>* cancelled = nullptr;
};
thread_local Budget t_budget;

/// csgExecute's limits for @p polygons, set for as long as it runs.
class BudgetScope {
public:
    explicit BudgetScope(std::size_t polygons) : m_saved(t_budget) {
        t_budget = Budget{};
        const CsgLimits& set = csgLimitsOnThisThread();
        t_budget.workLimit =
            set.work != 0 ? set.work : std::max(kCsgWorkLimit, kCsgWorkPerPolygon * polygons);
        t_budget.splitLimit = set.splits != 0
                                  ? set.splits
                                  : std::max(kCsgSplitLimit, kCsgSplitsPerPolygon * polygons);
        t_budget.cancelled = set.cancelled;
    }
    ~BudgetScope() { t_budget = m_saved; }
    BudgetScope(const BudgetScope&) = delete;
    BudgetScope& operator=(const BudgetScope&) = delete;

private:
    Budget m_saved;
};

/// @p n more polygon corners tested against a plane, charged to the
/// running csgExecute: past its limit, it stops (CsgTooComplex); asked now
/// and then, every 65,536 corners, a few milliseconds' work, whether what it
/// runs for still wants it (CsgCancelled).
void chargeWork(std::size_t n) {
    const std::size_t before = t_budget.work;
    t_budget.work += n;
    if (t_budget.work > t_budget.workLimit) {
        throw CsgTooComplex("cutting them would take far longer than any part needs");
    }
    if ((before >> 16) != (t_budget.work >> 16) && t_budget.cancelled != nullptr &&
        t_budget.cancelled->load(std::memory_order_relaxed)) {
        throw CsgCancelled();
    }
}

/// Split `poly` by `plane` into the four output buckets (csg.js algorithm),
/// moved there whole when it does not cross it.
void splitPolygon(const Plane& plane, double eps, Poly&& poly, std::vector<Poly>& coplanarFront,
                  std::vector<Poly>& coplanarBack, std::vector<Poly>& front,
                  std::vector<Poly>& back) {
    const auto& pts = poly.data.points;
    const size_t n = pts.size();
    chargeWork(n);

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
            if (++t_budget.splits > t_budget.splitLimit) {
                throw CsgTooComplex("they would be cut into more pieces than memory holds");
            }
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

    /// Where @p point lies: outside this tree's solid (1), inside it (-1),
    /// or on one of the planes it meets on the way down, within the
    /// on-plane band (0), where only clipping can tell.
    int classify(const Vec3& point) const {
        if (m_bounds.isValid()) {
            const Vec3 low = m_bounds.min() - Vec3(m_eps, m_eps, m_eps);
            const Vec3 high = m_bounds.max() + Vec3(m_eps, m_eps, m_eps);
            if (point.x < low.x || point.y < low.y || point.z < low.z || point.x > high.x ||
                point.y > high.y || point.z > high.z) {
                return m_inverted ? -1 : 1;
            }
        }
        int index = 0;
        for (;;) {
            const Node& node = m_nodes[static_cast<size_t>(index)];
            if (!node.hasPlane) return 1;  // an empty tree: nothing is inside
            const double d = node.plane.normal.dot(point) - node.plane.w;
            if (d > m_eps) {
                if (node.front < 0) return 1;
                index = node.front;
            } else if (d < -m_eps) {
                if (node.back < 0) return -1;
                index = node.back;
            } else {
                return 0;
            }
        }
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
        // Its work charged as a split's is: for a convex body of many
        // facets, each of these loops is in the square of their number.
        for (size_t i = 0; i < list.size(); i += stride) {
            const Plane& plane = list[i].plane;
            for (const auto& poly : list) {
                chargeWork(poly.data.points.size());
                for (const Vec3& p : poly.data.points) {
                    if (plane.normal.dot(p) - plane.w > m_eps) return false;
                }
            }
        }
        // In a plane as splitPolygon judges it: every point within m_eps.
        const auto inPlane = [this](const Plane& plane, const Poly& poly) {
            chargeWork(poly.data.points.size());
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
            chargeWork(points.size());
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

namespace {

math::BoundingBox boxOf(const std::vector<Vec3>& points) {
    math::BoundingBox box;
    for (const Vec3& p : points) box.expand(p);
    return box;
}

/// The pieces of @p a and @p b, combined as @p type asks, in as few
/// fragments as the CSG can leave (see CsgFragments::Fewest).
///
/// A piece whose box is clear of every piece of the other body cannot cross
/// that body's surface: it is all inside it or all outside, and one point of
/// it says which. It is kept whole or left out, and only the pieces near the
/// other body are clipped. Clipped, a piece is cut along the whole of every
/// plane of the other body it crosses, however far off the polygon in that
/// plane lies: a part patterned eight times from a cylinder of 4,096 facets
/// took most of two minutes, nearly all in cutting faces the next instance
/// came nowhere near, and its mirror image ran out of memory.
///
/// Each body's tree is still built from all of it, as the solid it is: what
/// is clipped is clipped against the whole of the other, and the cut is
/// where it was.
std::vector<CsgPolygon> csgFewest(const std::vector<CsgPolygon>& a,
                                  const std::vector<CsgPolygon>& b, BooleanType type,
                                  double planeEps) {
    std::vector<Poly> polysA = toPolys(a);
    std::vector<Poly> polysB = toPolys(b);
    Bsp solidA(planeEps);
    Bsp solidB(planeEps);
    solidA.build(largestFacesFirst(polysA));
    solidB.build(largestFacesFirst(polysB));
    math::BoundingBox boundsA;
    math::BoundingBox boundsB;
    for (const auto& poly : a) boundsA.expand(boxOf(poly.points));
    for (const auto& poly : b) boundsB.expand(boxOf(poly.points));
    solidA.setBounds(boundsA);
    solidB.setBounds(boundsB);

    // Near: a box within a few on-plane bands of a box of the other's.
    const double margin = 4.0 * planeEps;
    const auto nearTo = [margin](const std::vector<Poly>& mine, const std::vector<Poly>& theirs) {
        math::RTree<size_t> index;
        for (size_t i = 0; i < theirs.size(); ++i) index.insert(i, boxOf(theirs[i].data.points));
        std::vector<char> near(mine.size(), 0);
        for (size_t i = 0; i < mine.size(); ++i) {
            const math::BoundingBox box = boxOf(mine[i].data.points);
            const Vec3 pad(margin, margin, margin);
            near[i] =
                index.query(math::BoundingBox(box.min() - pad, box.max() + pad)).empty() ? 0 : 1;
        }
        return near;
    };
    const std::vector<char> nearA = nearTo(polysA, polysB);
    const std::vector<char> nearB = nearTo(polysB, polysA);

    // Where a piece clear of the other body lies, by its middle: a convex
    // piece's mean corner is inside it. On a plane of the other's, within
    // the band, it is clipped as one near it is.
    const auto where = [](const Bsp& other, const Poly& poly) {
        Vec3 middle = Vec3::Zero;
        for (const Vec3& p : poly.data.points) middle += p;
        return other.classify(middle / static_cast<double>(poly.data.points.size()));
    };
    // What each keeps of a piece wholly outside (1) or inside (-1) the other.
    const int keepA = type == BooleanType::Intersect ? -1 : 1;
    const int keepB = type == BooleanType::Union ? 1 : -1;
    std::vector<Poly> clipA;
    std::vector<Poly> clipB;
    std::vector<Poly> wholeA;
    std::vector<Poly> wholeB;
    for (size_t i = 0; i < polysA.size(); ++i) {
        const int side = nearA[i] != 0 ? 0 : where(solidB, polysA[i]);
        if (side == 0) {
            clipA.push_back(std::move(polysA[i]));
        } else if (side == keepA) {
            wholeA.push_back(std::move(polysA[i]));
        }
    }
    for (size_t i = 0; i < polysB.size(); ++i) {
        const int side = nearB[i] != 0 ? 0 : where(solidA, polysB[i]);
        if (side == 0) {
            clipB.push_back(std::move(polysB[i]));
        } else if (side == keepB) {
            wholeB.push_back(std::move(polysB[i]));
        }
    }

    // The near pieces, as csgExecute clips them, each against the whole of
    // the other body; a tree turned inside out is turned with its solid.
    Bsp nodeA(planeEps);
    Bsp nodeB(planeEps);
    nodeA.build(largestFacesFirst(std::move(clipA)));
    nodeB.build(largestFacesFirst(std::move(clipB)));
    switch (type) {
        case BooleanType::Union:
            nodeA.clipTo(solidB);
            nodeB.clipTo(solidA);
            nodeB.invert();
            nodeB.clipTo(solidA);
            nodeB.invert();
            break;
        case BooleanType::Subtract:
            nodeA.invert();
            solidA.invert();
            nodeA.clipTo(solidB);
            nodeB.clipTo(solidA);
            nodeB.invert();
            nodeB.clipTo(solidA);
            nodeB.invert();
            break;
        case BooleanType::Intersect:
            nodeA.invert();
            solidA.invert();
            nodeB.clipTo(solidA);
            nodeB.invert();
            solidB.invert();
            nodeA.clipTo(solidB);
            nodeB.clipTo(solidA);
            break;
    }

    // What is left of each, together, turned as csgExecute turns them: the
    // near pieces flipped back for a cut or an intersection, and those kept
    // whole as they were, but B's in a cut, which face into the hole.
    std::vector<CsgPolygon> result;
    const auto take = [&result](std::vector<Poly>&& polys, bool flip) {
        for (auto& poly : polys) {
            if (flip) poly.flip();
            result.push_back(std::move(poly.data));
        }
    };
    const bool flipNear = type != BooleanType::Union;
    take(nodeA.allPolygons(), flipNear);
    take(std::move(wholeA), false);
    take(nodeB.allPolygons(), flipNear);
    take(std::move(wholeB), type == BooleanType::Subtract);
    return result;
}

}  // namespace

CsgLimits& csgLimitsOnThisThread() {
    thread_local CsgLimits limits;
    return limits;
}

std::vector<CsgPolygon> csgExecute(const std::vector<CsgPolygon>& a,
                                   const std::vector<CsgPolygon>& b, BooleanType type,
                                   double planeEps, CsgFragments fragments) {
    const BudgetScope budget(a.size() + b.size());
    if (fragments == CsgFragments::Fewest) return csgFewest(a, b, type, planeEps);
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

/// How a polygon turns at @p cur, coming from @p prev and going on to
/// @p next, about @p normal: left (convex), straight on, or back (reflex).
/// A corner within @p straightEps of the line through its neighbours runs
/// straight on: an outline keeps such a corner where another face meets it
/// corner to corner, and rounding alone turns it a hair either way.
enum class Turn { Convex, Straight, Reflex };
Turn turnAt(const Vec3& prev, const Vec3& cur, const Vec3& next, const Vec3& normal,
            double straightEps) {
    const Vec3 e1 = cur - prev;
    const Vec3 e2 = next - cur;
    const double sine = normal.dot(e1.cross(e2));
    // Its distance from the line through its neighbours, signed: e1 x e2 is
    // twice the triangle's area, and (prev, next) its base.
    const double base = (next - prev).length();
    if (std::abs(sine) <= straightEps * base) {
        // Straight on, or doubling back on itself, a spike.
        return e1.dot(e2) > 0.0 ? Turn::Straight : Turn::Reflex;
    }
    return sine > 0.0 ? Turn::Convex : Turn::Reflex;
}

/// Whether @p points lie in one plane, to well within @p planeEps.
bool flatTo(const std::vector<Vec3>& points, const Plane& plane, double planeEps) {
    for (const Vec3& p : points) {
        if (!(std::abs(plane.normal.dot(p) - plane.w) <= 0.25 * planeEps)) return false;
    }
    return true;
}

/// Whether @p points make a convex polygon the CSG can take whole: flat to
/// well within @p planeEps, every corner turning the same way or running
/// straight on, going round once, and no edge of no length. Anything less
/// is cut into convex pieces.
bool convexAndFlat(const std::vector<Vec3>& points, double planeEps) {
    const size_t n = points.size();
    if (n < 4) return false;  // a triangle is taken whole anyway
    const Plane plane = Plane::fromPolygon(points);
    if (!plane.ok || !flatTo(points, plane, planeEps)) return false;
    const double straightEps = 1e-3 * planeEps;
    double turning = 0.0;
    for (size_t i = 0; i < n; ++i) {
        const Vec3& a = points[i];
        const Vec3& b = points[(i + 1) % n];
        const Vec3& c = points[(i + 2) % n];
        const Vec3 e1 = b - a;
        const Vec3 e2 = c - b;
        if (e1.lengthSquared() == 0.0 || e2.lengthSquared() == 0.0) return false;
        if (turnAt(a, b, c, plane.normal, straightEps) == Turn::Reflex) return false;
        turning += std::atan2(std::max(0.0, plane.normal.dot(e1.cross(e2))), e1.dot(e2));
    }
    // Round once, not twice (a star drawn without lifting the pen).
    constexpr double kTwoPi = 6.283185307179586;
    return std::abs(turning - kTwoPi) < 1e-6;
}

/// @p points, a flat polygon with @p normal, in convex pieces: triangulated,
/// then each side two triangles share taken out wherever the piece either
/// side of it and the one beyond make one convex piece, both its ends still
/// turning left (Hertel and Mehlhorn). A face with a notch cut in it is a
/// few pieces, not a triangle for each of its thousands of corners, and the
/// CSG's tree and its clipping are as much smaller. At most four times as
/// many pieces as the fewest there could be.
std::vector<std::vector<Vec3>> convexPieces(const std::vector<Vec3>& points, const Vec3& normal,
                                            double straightEps) {
    const auto triangles = BoundaryMesh::triangulatePolygonCorners(points);
    // Each triangle a ring of three corners; joined, two rings become one.
    struct Corner {
        size_t vertex;
        size_t next;
        size_t prev;
        bool alive;
    };
    std::vector<Corner> corners;
    corners.reserve(3 * triangles.size());
    // A side of a ring, from the corner at one end to the next: the corner
    // it starts from. The same side of the ring beyond runs the other way.
    const size_t n = points.size();
    const auto key = [n](size_t from, size_t to) { return from * (n + 1) + to; };
    std::unordered_map<size_t, size_t> sideFrom;
    sideFrom.reserve(3 * triangles.size());
    for (const auto& tri : triangles) {
        const size_t first = corners.size();
        for (size_t k = 0; k < 3; ++k) {
            corners.push_back({tri[k], first + (k + 1) % 3, first + (k + 2) % 3, true});
        }
        for (size_t k = 0; k < 3; ++k) sideFrom[key(tri[k], tri[(k + 1) % 3])] = first + k;
    }
    // Which ring each corner is in, as rings join.
    std::vector<size_t> ringOf(triangles.size());
    for (size_t t = 0; t < ringOf.size(); ++t) ringOf[t] = t;
    const auto root = [&ringOf](size_t t) {
        while (ringOf[t] != t) t = ringOf[t] = ringOf[ringOf[t]];
        return t;
    };

    const auto convex = [&](size_t prev, size_t cur, size_t next) {
        return turnAt(points[corners[prev].vertex], points[corners[cur].vertex],
                      points[corners[next].vertex], normal, straightEps) != Turn::Reflex;
    };
    for (size_t t = 0; t < triangles.size(); ++t) {
        for (size_t k = 0; k < 3; ++k) {
            const size_t u = triangles[t][k];
            const size_t w = triangles[t][(k + 1) % 3];
            if (u > w) continue;  // each side shared once
            const auto there = sideFrom.find(key(u, w));
            const auto back = sideFrom.find(key(w, u));
            if (there == sideFrom.end() || back == sideFrom.end()) continue;  // an outer side
            const size_t a = there->second;  // u, in one ring: a -> b
            const size_t c = back->second;   // w, in the other: c -> d
            const size_t b = corners[a].next;
            const size_t d = corners[c].next;
            const size_t ringA = root(a / 3);
            const size_t ringC = root(c / 3);
            if (ringA == ringC) continue;
            // Joined, the ring runs ... a -> (after d) ... (before c) -> b ...
            if (!convex(corners[a].prev, a, corners[d].next) ||
                !convex(corners[c].prev, b, corners[b].next)) {
                continue;
            }
            const size_t afterD = corners[d].next;
            const size_t beforeC = corners[c].prev;
            corners[a].next = afterD;
            corners[afterD].prev = a;
            corners[beforeC].next = b;
            corners[b].prev = beforeC;
            corners[c].alive = false;
            corners[d].alive = false;
            sideFrom.erase(there);
            sideFrom.erase(key(w, u));
            sideFrom[key(u, corners[afterD].vertex)] = a;
            ringOf[ringC] = ringA;
        }
    }

    std::vector<std::vector<Vec3>> pieces;
    std::vector<bool> taken(corners.size(), false);
    for (size_t start = 0; start < corners.size(); ++start) {
        if (!corners[start].alive || taken[start]) continue;
        std::vector<Vec3> piece;
        size_t at = start;
        do {
            taken[at] = true;
            piece.push_back(points[corners[at].vertex]);
            at = corners[at].next;
        } while (at != start && piece.size() <= corners.size());
        pieces.push_back(std::move(piece));
    }
    return pieces;
}

}  // namespace

std::vector<CsgPolygon> csgPieces(const std::vector<BoundaryPolygon>& polygons, bool fromA,
                                  double planeEps) {
    std::vector<CsgPolygon> out;
    out.reserve(polygons.size() * 2);
    const auto add = [&](std::vector<Vec3> points, const BoundaryPolygon& poly) {
        CsgPolygon p;
        p.points = std::move(points);
        p.topoId = poly.topoId;
        p.surface = nullptr;
        p.fromA = fromA;
        out.push_back(std::move(p));
    };
    for (const auto& poly : polygons) {
        if (poly.holes.empty() && convexAndFlat(poly.points, planeEps)) {
            add(poly.points, poly);
            continue;
        }
        // Flat: in as few convex pieces as its shape allows. Not flat: in
        // triangles, each of which is.
        const Plane plane = poly.points.size() > 3 ? Plane::fromPolygon(poly.points) : Plane{};
        if (plane.ok && flatTo(poly.points, plane, planeEps)) {
            for (auto& piece : convexPieces(poly.points, plane.normal, 1e-3 * planeEps)) {
                add(std::move(piece), poly);
            }
            continue;
        }
        for (const auto& tri : BoundaryMesh::triangulatePolygon(poly.points)) {
            add({tri.begin(), tri.end()}, poly);
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
