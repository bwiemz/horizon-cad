#include "horizon/modeling/SolidTessellator.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <functional>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include "horizon/geometry/curves/Curve3.h"
#include "horizon/geometry/curves/NurbsCurve.h"
#include "horizon/geometry/surfaces/NurbsSurface.h"
#include "horizon/modeling/BoundaryMesh.h"
#include "horizon/topology/ExactTolerance.h"
#include "horizon/topology/HalfEdge.h"

namespace hz::model {

using hz::math::Vec3;

namespace {

/// True for the 2×2 degree-1 bounding-rectangle patches produced by
/// NurbsSurface::makePlane (with coplanar corners) — these over-cover the
/// trimmed face, so the loop polygon is the faithful boundary.
bool isPlanarPatch(const geo::NurbsSurface& surface) {
    if (surface.degreeU() != 1 || surface.degreeV() != 1) return false;
    if (surface.controlPointCountU() != 2 || surface.controlPointCountV() != 2) return false;
    const auto& cp = surface.controlPoints();
    const Vec3& p00 = cp[0][0];
    const Vec3 u = cp[1][0] - p00;
    const Vec3 v = cp[0][1] - p00;
    const Vec3 d = cp[1][1] - p00;
    const Vec3 n = u.cross(v);
    const double nLen = n.length();
    if (nLen < 1e-30) return true;  // degenerate patch — treat as planar
    const double scale = std::max({u.length(), v.length(), d.length(), 1e-12});
    return std::abs(d.dot(n / nLen)) < 1e-9 * scale;
}

void appendLoopTriangles(const BoundaryPolygon& poly, geo::MeshData& out) {
    const auto tris = BoundaryMesh::triangulatePolygon(poly.points);
    for (const auto& tri : tris) {
        Vec3 n = (tri[1] - tri[0]).cross(tri[2] - tri[0]);
        const double len = n.length();
        n = (len > 1e-30) ? n / len : Vec3(0, 0, 1);

        const auto base = static_cast<uint32_t>(out.positions.size() / 3);
        for (const Vec3& p : tri) {
            out.positions.push_back(static_cast<float>(p.x));
            out.positions.push_back(static_cast<float>(p.y));
            out.positions.push_back(static_cast<float>(p.z));
            out.normals.push_back(static_cast<float>(n.x));
            out.normals.push_back(static_cast<float>(n.y));
            out.normals.push_back(static_cast<float>(n.z));
        }
        out.indices.push_back(base);
        out.indices.push_back(base + 1);
        out.indices.push_back(base + 2);
    }
}

/// A corner of an exact face's mesh, by its position and normal bit for bit: the mesher's one
/// vertex, which each triangle of the face that meets there repeats.
struct Corner {
    std::array<double, 6> values{};

    bool operator==(const Corner& other) const { return values == other.values; }
};

struct CornerHash {
    std::size_t operator()(const Corner& corner) const {
        std::uint64_t hash = 1469598103934665603ULL;
        for (const double value : corner.values) {
            std::uint64_t bits = 0;
            std::memcpy(&bits, &value, sizeof bits);
            hash ^= bits + 0x9e3779b97f4a7c15ULL + (hash << 6) + (hash >> 2);
        }
        return static_cast<std::size_t>(hash);
    }
};

/// The triangles @p first to @p last of @p polygons, one exact face's as the read point gives
/// them, as one mesh: their corners joined again where the face's mesh shares them, each shaded
/// by the surface's normal there.
void appendExactTriangles(const std::vector<BoundaryPolygon>& polygons, std::size_t first,
                          std::size_t last, geo::MeshData& out) {
    std::unordered_map<Corner, uint32_t, CornerHash> vertexOf;
    vertexOf.reserve(2 * (last - first));
    for (std::size_t i = first; i < last; ++i) {
        const BoundaryPolygon& tri = polygons[i];
        if (tri.points.size() != 3) continue;
        Vec3 flat = (tri.points[1] - tri.points[0]).cross(tri.points[2] - tri.points[0]);
        const double flatLength = flat.length();
        flat = flatLength > 1e-30 ? flat / flatLength : Vec3(0, 0, 1);
        for (std::size_t k = 0; k < 3; ++k) {
            const Vec3& p = tri.points[k];
            Vec3 n = k < tri.normals.size() ? tri.normals[k] : flat;
            const double length = n.length();
            n = length > 1e-30 ? n / length : flat;
            const Corner corner{{p.x, p.y, p.z, n.x, n.y, n.z}};
            const auto next = static_cast<uint32_t>(out.positions.size() / 3);
            const auto [at, added] = vertexOf.emplace(corner, next);
            if (added) {
                out.positions.insert(
                    out.positions.end(),
                    {static_cast<float>(p.x), static_cast<float>(p.y), static_cast<float>(p.z)});
                out.normals.insert(
                    out.normals.end(),
                    {static_cast<float>(n.x), static_cast<float>(n.y), static_cast<float>(n.z)});
            }
            out.indices.push_back(at->second);
        }
    }
}

void appendSurfaceTriangles(const geo::NurbsSurface& surface, double tolerance,
                            geo::MeshData& out) {
    const auto faceMesh = surface.tessellate(tolerance);
    const auto offset = static_cast<uint32_t>(out.positions.size() / 3);
    out.positions.insert(out.positions.end(), faceMesh.positions.begin(), faceMesh.positions.end());
    out.normals.insert(out.normals.end(), faceMesh.normals.begin(), faceMesh.normals.end());
    for (uint32_t idx : faceMesh.indices) {
        out.indices.push_back(idx + offset);
    }
}

/// A face's unit normal from its outer loop (Newell's method), or zero.
Vec3 loopNormal(const topo::Face& face) {
    Vec3 normal;
    if (!face.outerLoop || !face.outerLoop->halfEdge) return normal;
    const topo::HalfEdge* start = face.outerLoop->halfEdge;
    const topo::HalfEdge* he = start;
    size_t guard = 0;
    do {
        if (!he->origin || !he->next || !he->next->origin) return Vec3();
        const Vec3& a = he->origin->point;
        const Vec3& b = he->next->origin->point;
        normal.x += (a.y - b.y) * (a.z + b.z);
        normal.y += (a.z - b.z) * (a.x + b.x);
        normal.z += (a.x - b.x) * (a.y + b.y);
        he = he->next;
    } while (he && he != start && ++guard < 1000000);
    const double len = normal.length();
    return len > 1e-30 ? normal / len : Vec3();
}

/// Whether the edge between faces @p a and @p b is one the part shows: not
/// a seam between two facets of one curved surface (the same ideal surface,
/// or two ideal surfaces meeting at a gentle bend, as a patterned cylinder's
/// facets do), nor a line across one flat face split in two.
bool showsAsEdge(const topo::Face& a, const topo::Face& b,
                 std::unordered_map<const topo::Face*, Vec3>& normals) {
    if (a.analyticSurface && a.analyticSurface == b.analyticSurface) return false;
    const auto normalOf = [&normals](const topo::Face& f) {
        const auto it = normals.find(&f);
        if (it != normals.end()) return it->second;
        return normals.emplace(&f, loopNormal(f)).first->second;
    };
    const double cosine = normalOf(a).dot(normalOf(b));
    if (cosine > 1.0 - 1e-9) return false;  // coplanar pieces
    // Two facets of curved surfaces bending by under 30 degrees: one smooth
    // surface, faceted.
    constexpr double kSmooth = 0.8660254037844387;  // cos 30 degrees
    return !(a.analyticSurface && b.analyticSurface && cosine > kSmooth);
}

/// The angle between two directions; 0 when either is zero.
double turnBetween(const Vec3& a, const Vec3& b) {
    if (a.lengthSquared() <= 0.0 || b.lengthSquared() <= 0.0) return 0.0;
    return std::atan2(a.cross(b).length(), a.dot(b));
}

/// Points along an exact edge's curve, @p curve from @p t0 to @p t1, strictly between its ends:
/// each chord within half @p tolerance of the curve at its middle and turning by at most half the
/// display's angle (TrimmedFaceMesher's sampling of the faces' boundaries), so that the line runs
/// where the faces it parts meet.
void sampleBetween(const geo::Curve3& curve, double t0, double t1, double tolerance,
                   const std::function<void(const Vec3&)>& add) {
    if (curve.kind() == geo::CurveKind::Line) return;  // its ends are all of it
    constexpr int kPieces = 8;  // a closed curve's ends meet: its middle is found first
    constexpr int kDepth = 10;  // at most 8 x 1024 chords
    constexpr double kTurn = 0.5 * SolidTessellator::kDisplayMaxAngle;  // radians
    const double sag = 0.5 * tolerance;
    const std::function<void(double, const Vec3&, double, const Vec3&, int)> split =
        [&](double a, const Vec3& pa, double b, const Vec3& pb, int depth) {
            const double m = 0.5 * (a + b);
            const Vec3 pm = curve.evaluate(m);
            const Vec3 chord = pb - pa;
            const double length2 = chord.lengthSquared();
            const double along =
                length2 > 0.0 ? std::clamp((pm - pa).dot(chord) / length2, 0.0, 1.0) : 0.0;
            const double off = (pm - (pa + chord * along)).length();
            const double turn = turnBetween(curve.derivatives(a).d1, curve.derivatives(b).d1);
            if (depth < kDepth && (off > sag || turn > kTurn)) {
                split(a, pa, m, pm, depth + 1);
                add(pm);
                split(m, pm, b, pb, depth + 1);
            }
        };
    Vec3 last = curve.evaluate(t0);
    for (int i = 1; i <= kPieces; ++i) {
        const double a = t0 + (t1 - t0) * static_cast<double>(i - 1) / kPieces;
        const double b = t0 + (t1 - t0) * static_cast<double>(i) / kPieces;
        const Vec3 pb = curve.evaluate(b);
        split(a, last, b, pb, 0);
        if (i < kPieces) add(pb);
        last = pb;
    }
}

/// The edges the part shows, as polylines: a line for a straight edge,
/// sampled along its curve for a curved one. An edge with its face on both
/// sides (a seam of a periodic face) is never one.
void appendEdges(const topo::Solid& solid, double tolerance, geo::MeshData& out) {
    std::unordered_map<const topo::Face*, Vec3> normals;
    for (const auto& edge : solid.edges()) {
        const topo::HalfEdge* he = edge.halfEdge;
        if (!he || !he->origin || !he->next || !he->next->origin || !he->face) continue;
        if (he->twin && he->twin->face == he->face) continue;
        // An exact edge parts two faces as read: shown. A faceted one, as a
        // facet's edge may be a line across one smooth surface.
        if (!edge.exactCurve && he->twin && he->twin->face &&
            !showsAsEdge(*he->face, *he->twin->face, normals)) {
            continue;
        }
        geo::MeshData::Edge shown;
        shown.tag = edge.topoId.tag();
        const auto add = [&shown](const Vec3& p) {
            shown.points.push_back(static_cast<float>(p.x));
            shown.points.push_back(static_cast<float>(p.y));
            shown.points.push_back(static_cast<float>(p.z));
        };
        if (edge.exactCurve) {
            // From its vertex to its other, along its curve between (curveStart to curveEnd).
            add(he->origin->point);
            if (edge.curveEnd != edge.curveStart) {
                sampleBetween(*edge.exactCurve, edge.curveStart, edge.curveEnd, tolerance, add);
            }
            add(he->next->origin->point);
        } else if (edge.curve && edge.curve->degree() > 1) {
            constexpr int kSteps = 16;
            const double t0 = edge.curve->tMin();
            const double t1 = edge.curve->tMax();
            for (int k = 0; k <= kSteps; ++k) {
                add(edge.curve->evaluate(t0 + (t1 - t0) * static_cast<double>(k) / kSteps));
            }
        } else {
            add(he->origin->point);
            add(he->next->origin->point);
        }
        out.edges.push_back(std::move(shown));
    }
}

}  // namespace

double SolidTessellator::displayTolerance(const topo::Solid& solid) {
    return std::max(kDisplayTolerance, kDisplayRelativeTolerance * topo::exactExtent(solid));
}

geo::MeshData SolidTessellator::tessellate(const topo::Solid& solid) {
    return tessellate(solid, displayTolerance(solid));
}

geo::MeshData SolidTessellator::tessellate(const topo::Solid& solid, double tolerance,
                                           std::string* problem,
                                           const std::atomic<bool>* cancelled) {
    geo::MeshData result;

    // extractFacePolygons keeps face order and normalizes loop orientation
    // globally, so loop-triangle normals face outward. An exact face is meshed
    // there, to the tolerance, and read as its triangles one after another:
    // one face here. Each triangle records the face it came from, by name, so
    // a click on it names the face. A face that cannot be meshed is said, not
    // thrown, and left out.
    std::string unmeshed;
    MeshOptions options;
    options.chordTolerance = tolerance;
    options.maxAngle = kDisplayMaxAngle;
    options.cancelled = cancelled;
    const auto polygons = BoundaryMesh::extractFacePolygons(solid, options, &unmeshed);
    if (problem != nullptr) *problem = std::move(unmeshed);
    for (std::size_t i = 0; i < polygons.size();) {
        const BoundaryPolygon& poly = polygons[i];
        if (poly.exactSurface) {
            std::size_t last = i + 1;
            while (last < polygons.size() && polygons[last].exactSurface == poly.exactSurface &&
                   polygons[last].topoId.tag() == poly.topoId.tag()) {
                ++last;
            }
            appendExactTriangles(polygons, i, last, result);
            i = last;
        } else if (poly.surface && !isPlanarPatch(*poly.surface)) {
            appendSurfaceTriangles(*poly.surface, tolerance, result);
            ++i;
        } else {
            appendLoopTriangles(poly, result);
            ++i;
        }
        const auto face = static_cast<uint32_t>(result.faceTags.size());
        result.faceTags.push_back(poly.topoId.tag());
        result.triangleFaces.resize(result.indices.size() / 3, face);
    }
    appendEdges(solid, tolerance, result);

    return result;
}

}  // namespace hz::model
