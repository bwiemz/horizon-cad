#include "horizon/modeling/DrawingProjection.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <memory>
#include <string>

#include "horizon/geometry/curves/NurbsCurve.h"
#include "horizon/geometry/surfaces/NurbsSurface.h"
#include "horizon/modeling/ExactPredicates.h"
#include "horizon/modeling/Naming.h"
#include "horizon/topology/HalfEdge.h"
#include "horizon/topology/Queries.h"
#include "horizon/topology/Solid.h"

namespace hz::model {

using hz::math::Vec2;
using hz::math::Vec3;

namespace {

/// Orthonormal 2D basis on the view plane plus the normalized view direction.
struct Basis {
    Vec3 right;
    Vec3 upn;
    Vec3 dir;  ///< normalized view direction (into the screen)
};

Basis makeBasis(const ViewProjection& view) {
    Vec3 dir = view.dir.normalized();
    Vec3 right = dir.cross(view.up);
    if (right.length() < 1e-12) {
        // `up` is parallel to `dir`; pick any perpendicular reference instead.
        const Vec3 alt = (std::abs(dir.x) < 0.9) ? Vec3(1.0, 0.0, 0.0) : Vec3(0.0, 1.0, 0.0);
        right = dir.cross(alt);
    }
    right = right.normalized();
    const Vec3 upn = right.cross(dir).normalized();
    return {right, upn, dir};
}

Vec2 projectPoint(const Vec3& p, const ViewProjection& view, const Basis& b) {
    const Vec3 rel = p - view.origin;
    return Vec2(rel.dot(b.right), rel.dot(b.upn));
}

/// Möller–Trumbore ray/triangle test; forward hits only. Returns the hit
/// distance in @p t.
bool rayTriangle(const Vec3& origin, const Vec3& dir, const Vec3& v0, const Vec3& v1,
                 const Vec3& v2, double& t) {
    const Vec3 e1 = v1 - v0;
    const Vec3 e2 = v2 - v0;
    const Vec3 h = dir.cross(e2);
    const double a = e1.dot(h);
    if (std::abs(a) < 1e-15) return false;
    const double f = 1.0 / a;
    const Vec3 s = origin - v0;
    const double u = f * s.dot(h);
    if (u < 0.0 || u > 1.0) return false;
    const Vec3 q = s.cross(e1);
    const double v = f * dir.dot(q);
    if (v < 0.0 || u + v > 1.0) return false;
    t = f * e2.dot(q);
    return t > 0.0;
}

/// A bounding-volume tree over the occluding triangles: a ray is tried
/// against the few triangles whose boxes it crosses. Every ray tried every
/// triangle, and a 2,048-facet cylinder took 27 s to draw.
class TriangleTree {
public:
    explicit TriangleTree(const std::vector<Vec3>& mesh) : m_mesh(mesh) {
        const std::size_t count = mesh.size() / 3;
        m_order.resize(count);
        for (std::size_t i = 0; i < count; ++i) m_order[i] = static_cast<std::uint32_t>(i);
        if (count > 0) m_nodes.reserve(2 * count);
        if (count > 0) build(0, count);
    }

    /// Whether the ray from @p origin along @p dir hits a triangle beyond @p minT.
    bool hits(const Vec3& origin, const Vec3& dir, double minT) const {
        if (m_nodes.empty()) return false;
        // No allocation per ray: the tree is at most log2(n / kLeaf) + 1
        // deep, and the stack holds one node per level plus one.
        std::array<std::uint32_t, 128> stack{};
        std::size_t top = 0;
        stack[top++] = 0;
        while (top > 0) {
            const Node& node = m_nodes[stack[--top]];
            if (!crosses(node, origin, dir)) continue;
            if (node.count == 0) {
                stack[top++] = node.left;
                stack[top++] = node.right;
                continue;
            }
            for (std::uint32_t k = 0; k < node.count; ++k) {
                const std::size_t tri = m_order[node.first + k];
                double t = 0.0;
                if (rayTriangle(origin, dir, m_mesh[3 * tri], m_mesh[3 * tri + 1],
                                m_mesh[3 * tri + 2], t) &&
                    t > minT) {
                    return true;
                }
            }
        }
        return false;
    }

private:
    struct Node {
        Vec3 lo;
        Vec3 hi;
        std::uint32_t left = 0;  ///< an inner node's children
        std::uint32_t right = 0;
        std::uint32_t first = 0;  ///< a leaf's triangles: m_order[first, first + count)
        std::uint32_t count = 0;  ///< 0 for an inner node
    };

    Vec3 centroid(std::uint32_t tri) const {
        return (m_mesh[3 * tri] + m_mesh[3 * tri + 1] + m_mesh[3 * tri + 2]) * (1.0 / 3.0);
    }

    /// The node for triangles [begin, end) of m_order; its index.
    std::uint32_t build(std::size_t begin, std::size_t end) {
        Vec3 lo(1e300, 1e300, 1e300);
        Vec3 hi(-1e300, -1e300, -1e300);
        for (std::size_t i = begin; i < end; ++i) {
            for (std::size_t k = 0; k < 3; ++k) {
                const Vec3& p = m_mesh[3 * static_cast<std::size_t>(m_order[i]) + k];
                lo = Vec3(std::min(lo.x, p.x), std::min(lo.y, p.y), std::min(lo.z, p.z));
                hi = Vec3(std::max(hi.x, p.x), std::max(hi.y, p.y), std::max(hi.z, p.z));
            }
        }
        const auto index = static_cast<std::uint32_t>(m_nodes.size());
        m_nodes.push_back(Node{lo, hi});
        constexpr std::size_t kLeaf = 4;
        if (end - begin <= kLeaf) {
            m_nodes[index].first = static_cast<std::uint32_t>(begin);
            m_nodes[index].count = static_cast<std::uint32_t>(end - begin);
            return index;
        }
        // Split at the median centroid along the box's longest side.
        const Vec3 size = hi - lo;
        const int axis = size.x >= size.y && size.x >= size.z ? 0 : (size.y >= size.z ? 1 : 2);
        const auto along = [axis](const Vec3& p) {
            return axis == 0 ? p.x : (axis == 1 ? p.y : p.z);
        };
        const std::size_t mid = begin + (end - begin) / 2;
        std::nth_element(m_order.begin() + static_cast<std::ptrdiff_t>(begin),
                         m_order.begin() + static_cast<std::ptrdiff_t>(mid),
                         m_order.begin() + static_cast<std::ptrdiff_t>(end),
                         [this, &along](std::uint32_t a, std::uint32_t b) {
                             return along(centroid(a)) < along(centroid(b));
                         });
        const std::uint32_t left = build(begin, mid);
        const std::uint32_t right = build(mid, end);
        m_nodes[index].left = left;  // by index: the vector grew meanwhile
        m_nodes[index].right = right;
        return index;
    }

    /// Whether the ray crosses the node's box (slab test).
    static bool crosses(const Node& node, const Vec3& origin, const Vec3& dir) {
        double t0 = 0.0;
        double t1 = 1e300;
        const double o[3] = {origin.x, origin.y, origin.z};
        const double d[3] = {dir.x, dir.y, dir.z};
        const double lo[3] = {node.lo.x, node.lo.y, node.lo.z};
        const double hi[3] = {node.hi.x, node.hi.y, node.hi.z};
        for (int k = 0; k < 3; ++k) {
            if (std::abs(d[k]) < 1e-300) {
                // Parallel to this slab: inside it throughout, or never.
                if (o[k] < lo[k] || o[k] > hi[k]) return false;
                continue;
            }
            double a = (lo[k] - o[k]) / d[k];
            double b = (hi[k] - o[k]) / d[k];
            if (a > b) std::swap(a, b);
            t0 = std::max(t0, a);
            t1 = std::min(t1, b);
            if (t0 > t1) return false;
        }
        return true;
    }

    const std::vector<Vec3>& m_mesh;
    std::vector<std::uint32_t> m_order;
    std::vector<Node> m_nodes;
};

/// The normal of the surface @p face approximates, at @p point: its ideal's
/// when it has one (a facet of a cylinder: the cylinder's, there), else its
/// own. Only its line matters here, not which way it points.
Vec3 idealNormal(const topo::Face* face, const Vec3& point) {
    if (face->analyticSurface) {
        const auto [u, v] = face->analyticSurface->locate(point);
        const auto at = face->analyticSurface->evaluateWithDerivatives(u, v);
        const Vec3 n = at.du.cross(at.dv);
        if (n.length() > 1e-12) return n.normalized();  // none at a pole
    }
    return topo::loopNormal(face);
}

/// Whether @p a and @p b are parts of one logical face: facets of one ideal
/// surface, or names that differ only by a facet or a piece.
bool oneLogicalFace(const topo::Face* a, const topo::Face* b) {
    if (a->analyticSurface && a->analyticSurface == b->analyticSurface) return true;
    const std::string& ta = a->topoId.tag();
    const std::string& tb = b->topoId.tag();
    if (ta.empty() || tb.empty()) return false;
    const std::string la = logicalFace(ta);
    const std::string lb = logicalFace(tb);
    return la == lb && (la != ta || lb != tb);
}

/// What an edge is in this view; `std::nullopt`-like `drawn = false` for a
/// seam inside a face that is not its outline.
struct Classified {
    bool drawn = true;
    ProjectedEdge::Kind kind = ProjectedEdge::Kind::Edge;
};

/// Faces that meet within this angle of each other meet smoothly: a fillet
/// and the face it rounds onto (their ideal normals agree there exactly).
const double kTangentCos = std::cos(1.0 * 3.14159265358979323846 / 180.0);
/// Facets farther apart than this meet sharply whatever they approximate:
/// a fillet's first facet leans half a facet step off the face it meets.
const double kNearlySmoothCos = std::cos(30.0 * 3.14159265358979323846 / 180.0);

Classified classify(const topo::Edge& edge, const Vec3& viewDir) {
    const topo::HalfEdge* he = edge.halfEdge;
    const topo::Face* left = he->face;
    const topo::Face* right = he->twin->face;
    if (left == nullptr || right == nullptr) return {};  // a boundary: drawn
    if (oneLogicalFace(left, right)) {
        // A seam between two facets of one curved face: drawn only where the
        // face turns from the viewer to away from it, as its outline.
        // Facing the viewer is -1, away +1, and edge-on (or degenerate) 0:
        // a facet seen exactly edge-on, as an odd count of facets has in
        // Front, is the outline itself, drawn once, by its seam with the
        // neighbour that faces the viewer. (The product of the two was 0
        // there, and the outline could go missing.)
        const auto side = [&viewDir](const topo::Face* f) {
            const double d = topo::loopNormal(f).dot(viewDir);
            return d > 1e-9 ? 1 : (d < -1e-9 ? -1 : 0);
        };
        const int l = side(left);
        const int r = side(right);
        if (l * r < 0 || (l == 0 && r < 0) || (r == 0 && l < 0)) {
            return {true, ProjectedEdge::Kind::Silhouette};
        }
        return {false, ProjectedEdge::Kind::Edge};
    }
    // Faces whose facets meet at a clear angle meet sharply: only facets
    // within 30 degrees of each other are worth the ideal surfaces' normals
    // (a search on each surface).
    if (std::abs(topo::loopNormal(left).dot(topo::loopNormal(right))) < kNearlySmoothCos) {
        return {true, ProjectedEdge::Kind::Edge};
    }
    const Vec3 mid = (he->origin->point + he->twin->origin->point) * 0.5;
    const double c = std::abs(idealNormal(left, mid).dot(idealNormal(right, mid)));
    return {true, c > kTangentCos ? ProjectedEdge::Kind::Tangent : ProjectedEdge::Kind::Edge};
}

bool same(const Vec3& a, const Vec3& b, double tolerance) {
    return (a - b).length() <= tolerance;
}

/// The span of @p curve from @p a to @p b, if it runs between them: its
/// parameters at each end, and which way (the shorter way round a closed
/// curve, as a full circle every chord of a rim shares is). False for a
/// curve that does not pass through both ends, or whose span strays farther
/// from the chord than the chord is long: it is not this edge's.
bool spanBetween(const geo::NurbsCurve& curve, const Vec3& a, const Vec3& b, double tolerance,
                 double& from, double& along, bool& closed) {
    const double t0 = curve.tMin();
    const double t1 = curve.tMax();
    const double span = t1 - t0;
    closed = same(curve.evaluate(t0), curve.evaluate(t1), tolerance);
    // The ends first: a chord tagged with its own arc, the usual case; or a
    // closed edge, whose one vertex is both ends of its whole curve.
    if (same(curve.evaluate(t0), a, tolerance) && same(curve.evaluate(t1), b, tolerance)) {
        from = t0;
        along = span;
        return true;
    }
    if (same(curve.evaluate(t1), a, tolerance) && same(curve.evaluate(t0), b, tolerance)) {
        from = t1;
        along = -span;
        return true;
    }
    // Else each end found on the curve, to a millionth of the chord: a
    // boolean's or an import's vertices sit that close, not closer.
    const double near = std::max(tolerance, 1e-6 * (b - a).length());
    const double ta = curve.closestPoint(a, 1e-12);
    const double tb = curve.closestPoint(b, 1e-12);
    if (!same(curve.evaluate(ta), a, near) || !same(curve.evaluate(tb), b, near)) {
        return false;
    }
    const auto wrap = [&](double t) {
        if (!closed) return std::clamp(t, t0, t1);
        t = t0 + std::fmod(t - t0, span);
        return t < t0 ? t + span : t;
    };
    const Vec3 middle = (a + b) * 0.5;
    double d = tb - ta;
    if (closed) {
        const double other = d > 0.0 ? d - span : d + span;
        if ((curve.evaluate(wrap(ta + 0.5 * other)) - middle).length() <
            (curve.evaluate(wrap(ta + 0.5 * d)) - middle).length()) {
            d = other;
        }
    }
    if ((curve.evaluate(wrap(ta + 0.5 * d)) - middle).length() > (b - a).length()) return false;
    from = ta;
    along = d;
    return true;
}

/// The points an edge is drawn through, from its start to its end: along the
/// ideal curve it records when that curve runs between its ends (a rim is
/// drawn on its circle, not as a polygon, whether each chord records its own
/// arc or all share the circle); along its curve if it has one; else
/// straight. In pieces about @p pieceLength long, 1 to 64 (at least 4 along
/// a curve that strays from its chord); @p curved says which.
std::vector<Vec3> sampleEdge(const topo::Edge& edge, double pieceLength, double tolerance,
                             bool& curved) {
    const topo::HalfEdge* he = edge.halfEdge;
    const Vec3 a = he->origin->point;
    const Vec3 b = he->twin->origin->point;
    const geo::NurbsCurve* curve = nullptr;
    double from = 0.0;
    double along = 0.0;
    bool closed = false;
    if (edge.analyticCurve &&
        spanBetween(*edge.analyticCurve, a, b, tolerance, from, along, closed)) {
        curve = edge.analyticCurve.get();
    } else if (edge.curve && edge.curve->degree() > 1) {
        curve = edge.curve.get();
        from = curve->tMin();
        along = curve->tMax() - curve->tMin();
        closed = false;
    }
    curved = curve != nullptr;

    const auto at = [&](double s) {  // s in [0, 1], from the edge's start
        if (curve == nullptr) return a + (b - a) * s;
        const double t0 = curve->tMin();
        const double span = curve->tMax() - t0;
        double t = from + along * s;
        if (closed) {
            t = t0 + std::fmod(t - t0, span);
            if (t < t0) t += span;
        }
        return curve->evaluate(std::clamp(t, t0, t0 + span));
    };
    double length = (b - a).length();
    if (curved) {
        length = 0.0;
        Vec3 last = at(0.0);
        for (int i = 1; i <= 8; ++i) {
            const Vec3 next = at(i / 8.0);
            length += (next - last).length();
            last = next;
        }
    }
    // A curve is drawn in at least four pieces where it strays from its chord
    // visibly; a finely faceted rim's chords are already the curve.
    const bool bends = curved && (at(0.5) - (a + b) * 0.5).length() > pieceLength * 0.02;
    const int pieces =
        std::clamp(static_cast<int>(std::ceil(length / pieceLength)), bends ? 4 : 1, 64);
    std::vector<Vec3> points;
    points.reserve(static_cast<std::size_t>(pieces) + 1);
    for (int i = 0; i <= pieces; ++i) points.push_back(at(static_cast<double>(i) / pieces));
    points.front() = a;  // exactly its ends, not the curve's rounding of them
    points.back() = b;
    return points;
}

double meshDiagonal(const std::vector<Vec3>& mesh) {
    if (mesh.empty()) return 1.0;
    Vec3 lo = mesh[0];
    Vec3 hi = mesh[0];
    for (const Vec3& p : mesh) {
        lo = Vec3(std::min(lo.x, p.x), std::min(lo.y, p.y), std::min(lo.z, p.z));
        hi = Vec3(std::max(hi.x, p.x), std::max(hi.y, p.y), std::max(hi.z, p.z));
    }
    const double d = (hi - lo).length();
    return d > 1e-9 ? d : 1.0;
}

}  // namespace

Vec2 DrawingProjection::toView(const ViewProjection& view, const Vec3& p) {
    return projectPoint(p, view, makeBasis(view));
}

std::pair<Vec3, Vec3> DrawingProjection::axes(const ViewProjection& view) {
    const Basis basis = makeBasis(view);
    return {basis.right, basis.upn};
}

std::vector<ProjectedEdge> DrawingProjection::project(const topo::Solid& solid,
                                                      const ViewProjection& view) {
    const Basis basis = makeBasis(view);

    // Build the occluder mesh once, with its tree, for every visibility query.
    const std::vector<Vec3> mesh = ExactPredicates::tessellateSolid(solid);
    const TriangleTree occluders(mesh);
    const double diagonal = meshDiagonal(mesh);
    // Skip self-hits from the edge's own adjacent faces (which sit at t≈0)
    // without missing genuine occluders (at t ~ model scale).
    const double minT = diagonal * 1e-3;
    // Drop runs that collapse to a point in the view (edges parallel to the
    // view direction): they are not drawn as lines.
    const double degenerate = diagonal * 1e-7;
    // An edge is cut into pieces about this long, each classified by its
    // middle; a short edge is one piece.
    const double pieceLength = diagonal * 0.005;
    const Vec3 toViewer = -basis.dir;

    std::vector<ProjectedEdge> out;
    const auto emit = [&](const Vec3& from, const Vec3& to, bool visible,
                          const topo::TopologyID& id, ProjectedEdge::Kind kind) {
        ProjectedEdge pe;
        pe.a = projectPoint(from, view, basis);
        pe.b = projectPoint(to, view, basis);
        const double dx = pe.a.x - pe.b.x;
        const double dy = pe.a.y - pe.b.y;
        if (std::sqrt(dx * dx + dy * dy) < degenerate) return;  // view-parallel: skip
        pe.sourceEdge = id;
        pe.visibility =
            visible ? ProjectedEdge::Visibility::Visible : ProjectedEdge::Visibility::Hidden;
        pe.kind = kind;
        out.push_back(pe);
    };

    for (const auto& edge : solid.edges()) {
        const topo::HalfEdge* he = edge.halfEdge;
        if (he == nullptr || he->origin == nullptr || he->twin == nullptr ||
            he->twin->origin == nullptr) {
            continue;
        }
        const Classified what = classify(edge, basis.dir);
        if (!what.drawn) continue;

        bool curved = false;
        const std::vector<Vec3> samples = sampleEdge(edge, pieceLength, diagonal * 1e-9, curved);
        // Each piece by whether its middle can be seen. A straight edge's run
        // of pieces alike is one segment; a curve's pieces stay pieces, so it
        // is drawn as a curve.
        const std::size_t pieces = samples.size() - 1;
        std::vector<bool> visible(pieces);
        for (std::size_t s = 0; s < pieces; ++s) {
            const Vec3 middle = (samples[s] + samples[s + 1]) * 0.5;
            visible[s] = !occluders.hits(middle, toViewer, minT);
        }
        if (curved) {
            for (std::size_t s = 0; s < pieces; ++s) {
                emit(samples[s], samples[s + 1], visible[s], edge.topoId, what.kind);
            }
            continue;
        }
        std::size_t runStart = 0;
        for (std::size_t s = 1; s <= pieces; ++s) {
            if (s == pieces || visible[s] != visible[runStart]) {
                emit(samples[runStart], samples[s], visible[runStart], edge.topoId, what.kind);
                runStart = s;
            }
        }
    }
    return out;
}

ViewProjection DrawingProjection::standardView(StandardView view) {
    switch (view) {
        case StandardView::Front:  // from the front (-Y), looking along +Y; Z is up
            return {Vec3(0, 0, 0), Vec3(0, 1, 0), Vec3(0, 0, 1)};
        case StandardView::Top:  // look along -Z; Y is up
            return {Vec3(0, 0, 0), Vec3(0, 0, -1), Vec3(0, 1, 0)};
        case StandardView::Right:  // look along -X; Z is up
            return {Vec3(0, 0, 0), Vec3(-1, 0, 0), Vec3(0, 0, 1)};
        case StandardView::Isometric:  // look from (+,+,+) toward the origin
            return {Vec3(0, 0, 0), Vec3(-1, -1, -1), Vec3(0, 0, 1)};
    }
    return {};
}

}  // namespace hz::model
