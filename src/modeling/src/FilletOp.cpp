#include "horizon/modeling/FilletOp.h"

#include <algorithm>
#include <cassert>
#include <cmath>
#include <map>
#include <optional>
#include <set>

#include "horizon/geometry/curves/NurbsCurve.h"
#include "horizon/geometry/surfaces/NurbsSurface.h"
#include "horizon/math/Constants.h"
#include "horizon/modeling/Naming.h"
#include "horizon/modeling/Pattern.h"
#include "horizon/topology/GeometryValidator.h"
#include "horizon/topology/Queries.h"

namespace hz::model {

using namespace hz::topo;
using hz::math::Vec3;

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------

/// The edges a reference names: the edge itself, or — once an operation has
/// split it into pieces — every piece (its descendants).
static std::vector<const Edge*> findEdges(const Solid& solid, const TopologyID& id) {
    std::vector<const Edge*> chords;
    std::vector<const Edge*> pieces;
    for (const auto& e : solid.edges()) {
        if (e.topoId == id) return {&e};
        // The chords of the curve it names (Stable names), not a pattern
        // copy's, which are its descendants too.
        if (logicalEdge(e.topoId.tag()) == id.tag()) chords.push_back(&e);
        if (e.topoId.isDescendantOf(id)) pieces.push_back(&e);
    }
    return chords.empty() ? pieces : chords;
}

/// Face normal derived from the loop winding (Newell). Unlike the surface
/// carrier's normal — whose sign is not globally consistent — the Newell
/// normal is tied to the traversal order, so "interior lies to the left when
/// walking the loop with this normal up" holds by construction.
static Vec3 newellNormal(const Face* face) {
    auto verts = faceVertices(face);
    Vec3 n(0, 0, 0);
    for (size_t i = 0; i < verts.size(); ++i) {
        const Vec3& a = verts[i]->point;
        const Vec3& b = verts[(i + 1) % verts.size()]->point;
        n.x += (a.y - b.y) * (a.z + b.z);
        n.y += (a.z - b.z) * (a.x + b.x);
        n.z += (a.x - b.x) * (a.y + b.y);
    }
    const double len = n.length();
    return len > 1e-12 ? n * (1.0 / len) : Vec3(0, 0, 1);
}

/// Signed volume of the solid computed from its face loops (divergence
/// theorem over loop fans). Positive exactly when the loops' Newell normals
/// point outward — calibrates convexity tests independently of the kernel's
/// winding convention.
static double signedLoopVolume(const Solid& solid) {
    // About a vertex of the solid's own: about the origin, the rounding grows
    // as the cube of the distance, and far out the sign was chance.
    const Vec3 o = solid.vertices().empty() ? Vec3() : solid.vertices().front().point;
    double vol = 0.0;
    for (const auto& f : solid.faces()) {
        auto verts = faceVertices(&f);
        if (verts.size() < 3) continue;
        const Vec3 a = verts[0]->point - o;
        for (size_t i = 1; i + 1 < verts.size(); ++i) {
            vol += a.dot((verts[i]->point - o).cross(verts[i + 1]->point - o)) / 6.0;
        }
    }
    return vol;
}

/// Make a degree-1 linear NURBS curve between two points.
static std::shared_ptr<geo::NurbsCurve> makeLineCurve(const Vec3& a, const Vec3& b) {
    return std::make_shared<geo::NurbsCurve>(std::vector<Vec3>{a, b}, std::vector<double>{1.0, 1.0},
                                             std::vector<double>{0.0, 0.0, 1.0, 1.0}, 1);
}

/// Assign a line curve to an edge based on half-edge endpoint positions.
static void assignEdgeCurve(Edge* edge) {
    assert(edge->halfEdge != nullptr);
    HalfEdge* he = edge->halfEdge;
    const Vec3& a = he->origin->point;
    const Vec3& b = he->twin->origin->point;
    edge->curve = makeLineCurve(a, b);
}

// ---------------------------------------------------------------------------
// Fillet geometry computation
// ---------------------------------------------------------------------------

/// One cross-section of a fillet: the rolling-ball state at parameter t.
struct FilletStop {
    double t = 0.0;  ///< Normalized position along v1→v2.
    double r = 0.0;  ///< Fillet radius at this stop.
    Vec3 posA;       ///< Tangent point on faceA.
    Vec3 posB;       ///< Tangent point on faceB.
    Vec3 arcCenter;  ///< Rolling-ball center.
};

/// Data describing one edge to be filleted.
struct FilletEdgeInfo {
    const Edge* originalEdge = nullptr;
    Vertex* v1 = nullptr;   ///< First endpoint (t = 0).
    Vertex* v2 = nullptr;   ///< Second endpoint (t = 1).
    Face* faceA = nullptr;  ///< Left face.
    Face* faceB = nullptr;  ///< Right face.
    Vec3 edgeDir;           ///< Unit direction v1→v2.
    double edgeLen = 0.0;

    // Offset directions (inward along each face, perpendicular to edge).
    Vec3 offsetA;  ///< Direction on faceA from the edge inward.
    Vec3 offsetB;  ///< Direction on faceB from the edge inward.

    // The wedge the rolling ball sits in, between offsetA and offsetB: the
    // material for a convex edge, the empty side for a concave one.
    double theta = hz::math::kPi / 2.0;  ///< its angle
    Vec3 inA;                            ///< faceA's normal into the wedge
    double cotHalf = 1.0;                ///< setback per unit radius, cot(theta/2)
    double arcWeight = 0.0;              ///< the blend arc's middle weight, sin(theta/2)
    bool concave = false;                ///< the blend adds material

    /// Where a blend ends on a face it does not run along (the end face at an
    /// unfilleted vertex), its end section lies on that face's plane, not
    /// perpendicular to the edge: this is that plane's normal.
    std::optional<Vec3> endFrontNormal;
    std::optional<Vec3> endBackNormal;

    std::vector<FilletStop> stops;  ///< ≥ 2, increasing t; ends define topology.

    /// A mitered end (Phase 94): the blend's end section lies on the plane
    /// through that end's vertex with this normal instead of perpendicular to
    /// the edge, so it meets the next blend of a chain.
    bool miterFront = false;
    bool miterBack = false;
    Vec3 miterFrontNormal;
    Vec3 miterBackNormal;

    double maxRadius() const {
        double r = 0.0;
        for (const auto& s : stops) r = std::max(r, s.r);
        return r;
    }
    const FilletStop& front() const { return stops.front(); }
    const FilletStop& back() const { return stops.back(); }
};

/// Compute the frame of a fillet edge: adjacent faces, direction, and inward
/// offset directions. Radius-independent. @p outwardSign is +1 when the
/// solid's loop Newell normals point outward, -1 when inward (from
/// signedLoopVolume).
static bool computeFilletFrame(const Edge* edge, double outwardSign, FilletEdgeInfo& info) {
    info.originalEdge = edge;
    info.faceA = leftFace(edge);
    info.faceB = rightFace(edge);
    if (!info.faceA || !info.faceB) {
        return false;
    }

    HalfEdge* he = edge->halfEdge;
    info.v1 = he->origin;
    info.v2 = he->twin->origin;

    Vec3 dir = info.v2->point - info.v1->point;
    info.edgeLen = dir.length();
    if (info.edgeLen < 1e-12) {
        return false;
    }
    info.edgeDir = dir * (1.0 / info.edgeLen);

    // Loop-winding (Newell) normals — sign-consistent with traversal order.
    Vec3 nA = newellNormal(info.faceA);
    Vec3 nB = newellNormal(info.faceB);

    // Inward offset on each face: walking the loop with the Newell normal up,
    // the interior lies to the LEFT — an identity of the winding, so no
    // centroid heuristic (which mis-picks the side on non-convex faces).
    // faceA owns the half-edge that traverses v1→v2; faceB traverses v2→v1.
    Vec3 candidateA = nA.cross(info.edgeDir);
    Vec3 candidateB = nB.cross(info.edgeDir * (-1.0));

    double lenA = candidateA.length();
    double lenB = candidateB.length();
    if (lenA < 1e-12 || lenB < 1e-12) {
        return false;
    }
    info.offsetA = candidateA * (1.0 / lenA);
    info.offsetB = candidateB * (1.0 / lenB);

    // The ball rolls in the wedge between the two in-face directions, at any
    // angle: in the material for a convex edge, on the empty side for a
    // concave one (the blend then adds material). It touches each face a
    // setback r cot(theta/2) from the edge, its centre one radius off faceA
    // into the wedge, and the arc between the touch points spans pi - theta.
    // (It was refused at anything but a convex right angle, where every
    // formula here reduces to the old one.) Faces that nearly continue one
    // another, or meet in a knife edge, leave no wedge to roll in.
    const double cosTheta = std::clamp(info.offsetA.dot(info.offsetB), -1.0, 1.0);
    info.theta = std::acos(cosTheta);
    constexpr double kMinWedge = hz::math::kPi / 180.0;  // one degree
    if (info.theta < kMinWedge || info.theta > hz::math::kPi - kMinWedge) return false;
    const Vec3 across = info.offsetB - info.offsetA * cosTheta;
    if (across.length() < 1e-12) return false;
    info.inA = across * (1.0 / across.length());
    info.cotHalf = 1.0 / std::tan(info.theta / 2.0);
    info.arcWeight = std::sin(info.theta / 2.0);
    // Concave: faceB's in-face direction leaves faceA on its outward side.
    info.concave = info.offsetB.dot(nA * outwardSign) > 0.0;
    return true;
}

/// Materialize a stop: positions of the rolling ball at parameter t.
static FilletStop makeStop(const FilletEdgeInfo& info, double t, double r) {
    FilletStop s;
    s.t = t;
    s.r = r;
    const Vec3 p = info.v1->point + info.edgeDir * (t * info.edgeLen);
    const double setback = r * info.cotHalf;
    s.posA = p + info.offsetA * setback;
    s.posB = p + info.offsetB * setback;
    s.arcCenter = s.posA + info.inA * r;
    return s;
}

/// Ruled NURBS surface between two quarter arcs (the fillet cross-sections at
/// consecutive stops). Equal radii produce an exact cylindrical patch;
/// different radii an exact conical blend. The patch spans exactly the face
/// extent (unlike a full-cylinder carrier).
static std::shared_ptr<geo::NurbsSurface> makeArcLoft(const FilletEdgeInfo& info,
                                                      const FilletStop& s0, const FilletStop& s1) {
    // The arc's middle weight, cos of half the arc (pi - theta): sin(theta/2).
    const double w = info.arcWeight;

    // Arc control points at a stop: from the faceA tangent to the faceB
    // tangent; the tangents there meet on the edge, the middle point.
    auto arcRow = [&](const FilletStop& s) {
        const Vec3 onEdge = info.v1->point + info.edgeDir * (s.t * info.edgeLen);
        return std::vector<Vec3>{s.posA, onEdge, s.posB};
    };

    std::vector<std::vector<Vec3>> cps{arcRow(s0), arcRow(s1)};
    std::vector<std::vector<double>> weights{{1.0, w, 1.0}, {1.0, w, 1.0}};
    std::vector<double> knotsU{0.0, 0.0, 1.0, 1.0};
    std::vector<double> knotsV{0.0, 0.0, 0.0, 1.0, 1.0, 1.0};
    return std::make_shared<geo::NurbsSurface>(std::move(cps), std::move(weights),
                                               std::move(knotsU), std::move(knotsV), 1, 2);
}

/// Points along a stop's blend arc, from the faceA tangent to the faceB
/// tangent, as @p segments chords.
///
/// The blend used to be emitted as a single quad joining the two tangent
/// lines.  It carried the correct rational-quadratic arc surface, but its
/// *loop* was the chord — and since every loop-based path in the kernel
/// evaluates a solid from its face loops, mass properties, Booleans,
/// classification and export all saw a chamfer where the renderer drew a
/// fillet.  A 10mm cube filleted at r lost (1/2)r^2 L instead of
/// r^2(1 - pi/4)L: 2.33 times too much material, at every radius.
///
/// Sampling the arc makes the boundary agree with the surface.  The points
/// are exact on the arc — spherical interpolation between the two tangent
/// radii, not a subdivision of the chord — so the blend inscribes the true
/// fillet and converges to it as the count rises.
static std::vector<Vec3> arcSamples(const FilletStop& s, int segments) {
    std::vector<Vec3> pts;
    pts.reserve(static_cast<size_t>(segments) + 1);

    const Vec3 a = s.posA - s.arcCenter;
    const Vec3 b = s.posB - s.arcCenter;
    const double la = a.length();
    const double lb = b.length();
    if (segments < 2 || la < 1e-12 || lb < 1e-12) {
        pts.push_back(s.posA);
        pts.push_back(s.posB);
        return pts;
    }

    const Vec3 ua = a * (1.0 / la);
    const Vec3 ub = b * (1.0 / lb);
    const double cosTheta = std::clamp(ua.dot(ub), -1.0, 1.0);
    const double theta = std::acos(cosTheta);
    const double sinTheta = std::sin(theta);
    if (sinTheta < 1e-12) {
        pts.push_back(s.posA);
        pts.push_back(s.posB);
        return pts;
    }

    for (int i = 0; i <= segments; ++i) {
        const double t = static_cast<double>(i) / static_cast<double>(segments);
        const Vec3 dir =
            (ua * std::sin((1.0 - t) * theta) + ub * std::sin(t * theta)) * (1.0 / sinTheta);
        pts.push_back(s.arcCenter + dir * s.r);
    }
    // Pin the ends exactly: the corner-blend cycle matches them by position.
    pts.front() = s.posA;
    pts.back() = s.posB;
    return pts;
}

/// The circle an edge's ideal curve is (Phase 164): from three of its points,
/// and a fourth on it; none for a line or another curve.
struct Circle {
    Vec3 centre;
    Vec3 axis;  ///< unit
    double radius = 0.0;
};
static std::optional<Circle> circleOf(const geo::NurbsCurve& curve) {
    const double t0 = curve.tMin();
    const double t1 = curve.tMax();
    const Vec3 a = curve.evaluate(t0);
    const Vec3 b = curve.evaluate(t0 + (t1 - t0) / 3.0);
    const Vec3 c = curve.evaluate(t0 + 2.0 * (t1 - t0) / 3.0);
    const Vec3 ab = b - a;
    const Vec3 ac = c - a;
    const Vec3 normal = ab.cross(ac);
    const double n2 = normal.lengthSquared();
    if (n2 < 1e-24) return std::nullopt;
    Circle out;
    out.centre =
        a + (normal.cross(ab) * ac.lengthSquared() + ac.cross(normal) * ab.lengthSquared()) *
                (1.0 / (2.0 * n2));
    out.radius = (a - out.centre).length();
    out.axis = normal.normalized();
    const Vec3 d = curve.evaluate(t0 + 0.5 * (t1 - t0));
    if (std::abs((d - out.centre).length() - out.radius) > 1e-9 * std::max(1.0, out.radius)) {
        return std::nullopt;
    }
    return out;
}

/// The exact section of a fillet along a rim, at a mitered corner @p v whose
/// miter plane (normal @p miter) holds the rim's axis: the meridian plane
/// there (Phase 164). The ball touches the two faces' lines in that plane,
/// as a revolved fillet's does, not the chord's own prism cut slantwise (an
/// ellipse). Consecutive corners' sections are then turns of one another,
/// so the bands between them are flat, and every corner of every band is on
/// the torus the rim's fillet is. None for an edge that is no rim, or a
/// miter plane that does not hold its axis.
static std::optional<FilletStop> rimSection(const FilletEdgeInfo& fe, const Vertex* v,
                                            const Vec3& miter, double r) {
    if (!fe.originalEdge->analyticCurve) return std::nullopt;
    const auto circle = circleOf(*fe.originalEdge->analyticCurve);
    if (!circle || std::abs(miter.dot(circle->axis)) > 1e-9) return std::nullopt;
    const Vec3 fromCentre = v->point - circle->centre;
    if (std::abs(fromCentre.dot(circle->axis)) > 1e-9 * std::max(1.0, circle->radius) ||
        std::abs(fromCentre.length() - circle->radius) > 1e-9 * std::max(1.0, circle->radius)) {
        return std::nullopt;  // the corner is not on the rim's circle
    }
    // Each face's line in the meridian plane, from the corner into the face.
    const auto lineOf = [&miter](const Face* face, const Vec3& into) -> std::optional<Vec3> {
        Vec3 along = topo::loopNormal(face).cross(miter);
        if (along.length() < 1e-12) return std::nullopt;
        along = along.normalized();
        return along.dot(into) >= 0.0 ? along : along * -1.0;
    };
    const auto ua = lineOf(fe.faceA, fe.offsetA);
    const auto ub = lineOf(fe.faceB, fe.offsetB);
    if (!ua || !ub) return std::nullopt;
    const double cosWedge = std::clamp(ua->dot(*ub), -1.0, 1.0);
    const double sinWedge = std::sqrt(std::max(0.0, 1.0 - cosWedge * cosWedge));
    if (sinWedge < 1e-9) return std::nullopt;
    const double setback = r * (1.0 + cosWedge) / sinWedge;  // r cot(wedge / 2)
    FilletStop stop;
    stop.r = r;
    stop.posA = v->point + *ua * setback;
    stop.posB = v->point + *ub * setback;
    stop.arcCenter = stop.posA + (*ub - *ua * cosWedge).normalized() * r;
    return stop;
}

/// The blend-arc samples of every stop of @p fe, from the faceA tangent to
/// the faceB tangent.  At a mitered end each sample is carried along the edge
/// onto the miter plane: the cut of a constant cross-section prism by the
/// plane bisecting the turn, exactly as a mitered sweep joins two segments.
/// Every band between consecutive samples therefore stays planar.
static std::vector<std::vector<Vec3>> stopSamples(const FilletEdgeInfo& fe, int segments) {
    std::vector<std::vector<Vec3>> out;
    out.reserve(fe.stops.size());
    for (const auto& stop : fe.stops) out.push_back(arcSamples(stop, segments));
    auto project = [&fe](std::vector<Vec3>& samples, const Vec3& through, const Vec3& normal) {
        const double denom = fe.edgeDir.dot(normal);
        for (auto& p : samples) p = p + fe.edgeDir * ((through - p).dot(normal) / denom);
    };
    // A rim's corner: its exact section (Phase 164); another's, the
    // prism's cut by the miter plane.
    if (fe.miterFront) {
        if (const auto exact = rimSection(fe, fe.v1, fe.miterFrontNormal, fe.stops.front().r)) {
            out.front() = arcSamples(*exact, segments);
        } else {
            project(out.front(), fe.v1->point, fe.miterFrontNormal);
        }
    } else if (fe.endFrontNormal && std::abs(fe.edgeDir.dot(*fe.endFrontNormal)) > 1e-9) {
        project(out.front(), fe.v1->point, *fe.endFrontNormal);  // onto the end face
    }
    if (fe.miterBack) {
        if (const auto exact = rimSection(fe, fe.v2, fe.miterBackNormal, fe.stops.back().r)) {
            out.back() = arcSamples(*exact, segments);
        } else {
            project(out.back(), fe.v2->point, fe.miterBackNormal);
        }
    } else if (fe.endBackNormal && std::abs(fe.edgeDir.dot(*fe.endBackNormal)) > 1e-9) {
        project(out.back(), fe.v2->point, *fe.endBackNormal);
    }
    return out;
}

/// The torus a fillet along a rim sweeps (Phase 164): the ball's centre goes
/// round the rim's circle, so the rim's bands are one torus, not a ruled
/// patch each. Its spine is the circle through the exact sections' centres
/// about the rim's axis; its tube, the radius. Null for an edge that is no
/// rim chord mitered at its start, or a radius that varies.
static std::shared_ptr<geo::NurbsSurface> makeRimTorus(const FilletEdgeInfo& info) {
    if (!info.miterFront || info.stops.size() < 2) return nullptr;
    const double r = info.stops.front().r;
    for (const auto& stop : info.stops) {
        if (std::abs(stop.r - r) > 1e-12 * std::max(1.0, r)) return nullptr;
    }
    const auto section = rimSection(info, info.v1, info.miterFrontNormal, r);
    if (!section) return nullptr;
    const auto circle = circleOf(*info.originalEdge->analyticCurve);
    if (!circle) return nullptr;
    const Vec3 ball = section->arcCenter - circle->centre;
    const double height = ball.dot(circle->axis);
    const double spine = (ball - circle->axis * height).length();
    if (!(spine > r)) return nullptr;  // the tube would cross the axis
    return std::make_shared<geo::NurbsSurface>(geo::NurbsSurface::makeTorus(
        circle->centre + circle->axis * height, circle->axis, spine, r));
}

// ---------------------------------------------------------------------------
// Corner blends (Phase 61): three filleted edges meeting at a vertex
// ---------------------------------------------------------------------------

struct CornerBlend {
    const Vertex* vertex = nullptr;
    std::vector<size_t> edgeIndices;  ///< Indices into filletEdges (size 3).
    double radius = 0.0;
    Vec3 sphereCenter;
    std::vector<Vec3> cornerPoints;  ///< The three blended tangent points.
};

/// Direction of edge fe pointing AWAY from vertex v (into the solid).
static Vec3 dirFromVertex(const FilletEdgeInfo& fe, const Vertex* v) {
    return (fe.v1 == v) ? fe.edgeDir : fe.edgeDir * (-1.0);
}

/// Radius of the fillet at the end that touches vertex v.
static double radiusAtVertex(const FilletEdgeInfo& fe, const Vertex* v) {
    return (fe.v1 == v) ? fe.front().r : fe.back().r;
}

/// Trim the fillet's end at vertex v back by @p trim (edge-length units),
/// keeping the end radius constant over the trimmed span.
static void trimAtVertex(FilletEdgeInfo& fe, const Vertex* v, double trim) {
    const double dt = trim / fe.edgeLen;
    if (fe.v1 == v) {
        const double r = fe.front().r;
        const double t0 = dt;
        auto& stops = fe.stops;
        stops.erase(std::remove_if(stops.begin(), stops.end(),
                                   [t0](const FilletStop& s) { return s.t < t0 + 1e-12; }),
                    stops.end());
        stops.insert(stops.begin(), makeStop(fe, t0, r));
    } else {
        const double r = fe.back().r;
        const double t1 = 1.0 - dt;
        auto& stops = fe.stops;
        stops.erase(std::remove_if(stops.begin(), stops.end(),
                                   [t1](const FilletStop& s) { return s.t > t1 - 1e-12; }),
                    stops.end());
        stops.push_back(makeStop(fe, t1, r));
    }
}

/// Validate and build a corner blend for three fillets sharing vertex v.
/// On success the three fillets are trimmed and @p blend is filled.
static bool buildCornerBlend(std::vector<FilletEdgeInfo>& filletEdges,
                             const std::vector<size_t>& indices, const Vertex* v,
                             CornerBlend& blend, std::string& error) {
    // Equal radii at the shared corner.
    const double r = radiusAtVertex(filletEdges[indices[0]], v);
    for (size_t k : indices) {
        if (std::abs(radiusAtVertex(filletEdges[k], v) - r) > 1e-9) {
            error = "Vertex blend requires equal fillet radii at the shared corner";
            return false;
        }
    }
    // The corner's sphere sits one radius along each edge: a convex right-
    // angled corner only.
    for (size_t k : indices) {
        const auto& fe = filletEdges[k];
        if (fe.concave || std::abs(fe.theta - hz::math::kPi / 2.0) > 1e-6) {
            error =
                "A corner blend of three fillets needs square, convex edges; this corner "
                "is oblique or concave (edge " +
                fe.originalEdge->topoId.tag() + ")";
            return false;
        }
    }
    // Each pair of edges must share a face (a genuine solid corner).
    for (size_t i = 0; i < indices.size(); ++i) {
        for (size_t j = i + 1; j < indices.size(); ++j) {
            const auto& a = filletEdges[indices[i]];
            const auto& b = filletEdges[indices[j]];
            const bool share = a.faceA == b.faceA || a.faceA == b.faceB || a.faceB == b.faceA ||
                               a.faceB == b.faceB;
            if (!share) {
                error = "Vertex blend edges must pairwise share a face";
                return false;
            }
        }
    }
    // Trim feasibility: the retracted span must leave room for the rest.
    for (size_t k : indices) {
        const auto& fe = filletEdges[k];
        if (r >= fe.edgeLen * 0.5) {
            error = "Vertex blend radius too large for edge: " + fe.originalEdge->topoId.tag();
            return false;
        }
    }

    blend.vertex = v;
    blend.edgeIndices = indices;
    blend.radius = r;

    // Rolling-ball corner center: one radius along each edge into the solid.
    Vec3 center = v->point;
    for (size_t k : indices) {
        center = center + dirFromVertex(filletEdges[k], v) * r;
    }
    blend.sphereCenter = center;

    // Trim the fillets and collect the (pairwise coincident) tangent points.
    std::vector<Vec3> points;
    for (size_t k : indices) {
        auto& fe = filletEdges[k];
        trimAtVertex(fe, v, r);
        const FilletStop& end = (fe.v1 == v) ? fe.front() : fe.back();
        points.push_back(end.posA);
        points.push_back(end.posB);
    }
    for (const Vec3& p : points) {
        bool found = false;
        for (const Vec3& q : blend.cornerPoints) {
            if (p.distanceTo(q) < 1e-9) {
                found = true;
                break;
            }
        }
        if (!found) blend.cornerPoints.push_back(p);
    }
    if (blend.cornerPoints.size() != 3) {
        error = "Vertex blend corner did not resolve to three tangent points";
        return false;
    }
    return true;
}

// ---------------------------------------------------------------------------
// Mitered chains (Phase 94): two filleted edges meeting at a vertex
// ---------------------------------------------------------------------------

/// Two fillets meeting at a vertex whose third edge is not filleted — the
/// rim of a faceted cylinder, or any polygonal edge loop around a face — are
/// joined by a miter rather than a corner patch.  The two edges share one
/// face (the cap) and their other faces meet along the third edge, which is
/// perpendicular to the cap because both dihedrals are right angles.  Each
/// blend's end section is carried onto the plane that bisects the turn in the
/// cap and contains that third edge; the faceB tangent lines of both blends
/// meet on the third edge one radius from the cap, and the faceA tangent
/// lines meet on the bisector, so the two blends share their end section and
/// no patch is needed.
static bool buildMiter(std::vector<FilletEdgeInfo>& filletEdges, const std::vector<size_t>& indices,
                       const Vertex* v, std::string& error) {
    auto& a = filletEdges[indices[0]];
    auto& b = filletEdges[indices[1]];
    if (std::abs(radiusAtVertex(a, v) - radiusAtVertex(b, v)) > 1e-9) {
        error = "Mitered fillet chain requires equal radii where two edges meet";
        return false;
    }
    // Two blends meet on the miter plane in one section only if they have
    // the same section: the same wedge, the same side of the material.
    if (std::abs(a.theta - b.theta) > 1e-6 || a.concave != b.concave) {
        error = "A chain of fillets across corners of different angles is not supported: " +
                a.originalEdge->topoId.tag() + " and " + b.originalEdge->topoId.tag();
        return false;
    }

    // Exactly one shared face, and a vertex with exactly one other edge,
    // joining the two fillets' other faces.
    const Face* shared = nullptr;
    int sharedCount = 0;
    for (const Face* fa : {a.faceA, a.faceB}) {
        for (const Face* fb : {b.faceA, b.faceB}) {
            if (fa == fb) {
                shared = fa;
                ++sharedCount;
            }
        }
    }
    if (sharedCount != 1) {
        error = "Two filleted edges meeting at a vertex must share exactly one face";
        return false;
    }
    const Face* otherA = (a.faceA == shared) ? a.faceB : a.faceA;
    const Face* otherB = (b.faceA == shared) ? b.faceB : b.faceA;

    int degree = 0;
    const Edge* third = nullptr;
    const HalfEdge* start = v->halfEdge;
    const HalfEdge* he = start;
    do {
        ++degree;
        if (he->edge != a.originalEdge && he->edge != b.originalEdge) third = he->edge;
        he = he->twin->next;
    } while (he != start && degree <= 8);
    if (degree != 3 || third == nullptr) {
        error = "Mitered fillet chain requires a vertex with exactly three edges";
        return false;
    }
    const Face* thirdL = third->halfEdge->face;
    const Face* thirdR = third->halfEdge->twin->face;
    if (!((thirdL == otherA && thirdR == otherB) || (thirdL == otherB && thirdR == otherA))) {
        error = "Mitered fillet chain: the unfilleted edge must join the two side faces";
        return false;
    }

    // Miter plane: normal along the bisector of the turn, arriving along a
    // and leaving along b.
    const Vec3 arriving = dirFromVertex(a, v) * -1.0;
    const Vec3 leaving = dirFromVertex(b, v);
    const Vec3 bisector = arriving + leaving;
    if (bisector.length() < 1e-9) {
        error = "Mitered fillet chain: the edges double back on each other";
        return false;
    }
    const Vec3 normal = bisector.normalized();
    for (auto* fe : {&a, &b}) {
        if (fe->v1 == v) {
            fe->miterFront = true;
            fe->miterFrontNormal = normal;
        } else {
            fe->miterBack = true;
            fe->miterBackNormal = normal;
        }
    }
    return true;
}

// ---------------------------------------------------------------------------
// Core execution over prepared fillet edges
// ---------------------------------------------------------------------------

static FilletResult executeCore(const Solid& inputSolid, std::vector<FilletEdgeInfo>& filletEdges,
                                const std::string& featureID, int arcSegments) {
    FilletResult result;

    // -- Shared-vertex analysis: three edges → corner blend, two → miter --
    std::map<const Vertex*, std::vector<size_t>> vertexEdges;
    for (size_t i = 0; i < filletEdges.size(); ++i) {
        vertexEdges[filletEdges[i].v1].push_back(i);
        vertexEdges[filletEdges[i].v2].push_back(i);
    }
    std::vector<CornerBlend> blends;
    std::set<const Vertex*> miters;
    for (const auto& [v, indices] : vertexEdges) {
        if (indices.size() == 1) continue;
        if (indices.size() == 2) {
            if (!buildMiter(filletEdges, indices, v, result.errorMessage)) {
                return result;
            }
            miters.insert(v);
            continue;
        }
        if (indices.size() == 3) {
            CornerBlend blend;
            if (!buildCornerBlend(filletEdges, indices, v, blend, result.errorMessage)) {
                return result;
            }
            blends.push_back(std::move(blend));
            continue;
        }
        result.errorMessage =
            "Vertex blend not supported: " + std::to_string(indices.size()) +
            " selected edges share a vertex (two form a mitered chain, three a corner blend)";
        return result;
    }

    // An end at a vertex no blend or miter joins lies on the end face there
    // (the face at the vertex the blend does not run along), which need not be
    // square to the edge.
    for (auto& fe : filletEdges) {
        const auto endFaceNormal = [&fe](const Vertex* v) -> std::optional<Vec3> {
            std::optional<Vec3> found;
            const HalfEdge* start = v->halfEdge;
            const HalfEdge* he = start;
            int guard = 0;
            do {
                if (he->face != nullptr && he->face != fe.faceA && he->face != fe.faceB) {
                    if (found) return std::nullopt;  // more than one: not a simple end
                    const Vec3 n = newellNormal(he->face);
                    if (n.length() > 1e-12) found = n * (1.0 / n.length());
                }
                he = he->twin->next;
            } while (he != start && ++guard < 64);
            return found;
        };
        const auto joined = [&](const Vertex* v) {
            return miters.count(v) != 0 ||
                   std::any_of(blends.begin(), blends.end(),
                               [v](const CornerBlend& b) { return b.vertex == v; });
        };
        if (!joined(fe.v1)) fe.endFrontNormal = endFaceNormal(fe.v1);
        if (!joined(fe.v2)) fe.endBackNormal = endFaceNormal(fe.v2);
    }

    // Every blend's arc samples, carried onto its miter planes.  A miter must
    // not fold: each sample has to travel forward along its edge from one
    // end section to the other, or the blend passes through itself (a turn
    // too tight for the radius).
    std::vector<std::vector<std::vector<Vec3>>> samples;
    samples.reserve(filletEdges.size());
    for (const auto& fe : filletEdges) {
        samples.push_back(stopSamples(fe, arcSegments));
        const auto& first = samples.back().front();
        const auto& last = samples.back().back();
        for (size_t k = 0; k < first.size() && k < last.size(); ++k) {
            if ((last[k] - first[k]).dot(fe.edgeDir) <= 1e-9 * fe.edgeLen) {
                result.errorMessage = "Fillet radius too large for the turn at a chained edge: " +
                                      fe.originalEdge->topoId.tag();
                return result;
            }
        }
    }

    // -- Validate radius against face dimensions --
    // Capacity: the blend cannot retract a face further than the face reaches.
    //
    // This used to cap the radius at half the shortest edge of either adjacent
    // face — the same proxy ChamferOp carried, with the same problem.  It
    // holds for a box and collapses once faces are faceted: on a 32-sided
    // cylinder the shortest edge is the facet chord, so any radius over half
    // of that was refused regardless of how much room the face had.  The real
    // bound is how far the face reaches along the offset direction, and the
    // geometric gate on the result is what guarantees correctness past this
    // necessary condition.
    for (const auto& fe : filletEdges) {
        const Vec3 onEdge = fe.v1->point;
        auto reachOf = [&onEdge](const Face* face, const Vec3& dir) {
            double reach = 0.0;
            for (const auto* v : faceVertices(face)) {
                reach = std::max(reach, (v->point - onEdge).dot(dir));
            }
            return reach;
        };
        const double limit = std::min(reachOf(fe.faceA, fe.offsetA), reachOf(fe.faceB, fe.offsetB));
        if (fe.maxRadius() * fe.cotHalf > limit + 1e-9) {  // the setback, not the radius
            result.errorMessage =
                "Fillet radius too large for edge: " + fe.originalEdge->topoId.tag();
            return result;
        }
    }

    // -- Build the new solid --
    //
    // Face loops are emitted as consistently oriented vertex-position rings
    // (walking the original solid's half-edge loops keeps original faces
    // consistent; fillet/blend faces are oriented to oppose them), then the
    // half-edge structure is assembled directly with strict twin linking —
    // no heuristics, and Solid::isValid() gates the result.

    // Map from original edge id to fillet info.
    std::map<uint32_t, size_t> edgeToFillet;  // edge.id -> index in filletEdges
    for (size_t i = 0; i < filletEdges.size(); ++i) {
        edgeToFillet[filletEdges[i].originalEdge->id] = i;
    }

    // Map from unblended fillet endpoints to their fillet: these vertices get
    // chord-cut on the end face (the face at the vertex that is not adjacent
    // to the filleted edge).
    std::map<const Vertex*, size_t> endVertexToFillet;
    for (size_t i = 0; i < filletEdges.size(); ++i) {
        const auto& fe = filletEdges[i];
        auto isBlendedAt = [&](const Vertex* v) {
            return miters.count(v) != 0 ||
                   std::any_of(blends.begin(), blends.end(),
                               [v](const CornerBlend& b) { return b.vertex == v; });
        };
        if (!isBlendedAt(fe.v1)) {
            if (endVertexToFillet.count(fe.v1) != 0) {
                result.errorMessage = "Vertex blend not supported: 2 selected edges share a vertex";
                return result;
            }
            endVertexToFillet[fe.v1] = i;
        }
        if (!isBlendedAt(fe.v2)) {
            if (endVertexToFillet.count(fe.v2) != 0) {
                result.errorMessage = "Vertex blend not supported: 2 selected edges share a vertex";
                return result;
            }
            endVertexToFillet[fe.v2] = i;
        }
    }

    struct NewFaceData {
        std::vector<Vec3> vertices;
        /// Its holes (Phase 164: a revolve's flat ring, a plate's hole), each
        /// rewritten as the outline is.
        std::vector<std::vector<Vec3>> holes;
        TopologyID topoId;
        bool isOriginal = true;
        std::shared_ptr<geo::NurbsSurface> surface;  ///< Prebuilt (blend faces).
        /// The ideal surface a faceted band approximates; see
        /// topo::Face::analyticSurface.  Not the carrier.
        std::shared_ptr<geo::NurbsSurface> analyticSurface;
    };

    std::vector<NewFaceData> newFaces;

    // Process each original face. Its surface carrier travels with it —
    // curved faces (including fillet patches from a previous FilletOp call)
    // must not be re-bound as planes.
    for (const auto& face : inputSolid.faces()) {
        NewFaceData fd;
        fd.topoId = face.topoId;
        fd.isOriginal = true;
        fd.surface = face.surface;
        // Carry the ideal geometry a face records with it: a blend band from
        // an earlier operation is still an approximation of its arc, and
        // dropping that on the next operation would lose the design intent
        // feature recognition depends on.
        fd.analyticSurface = face.analyticSurface;

        // Each loop rewritten the same way: the outline, then each hole.
        const auto rewrite = [&](const HalfEdge* start, std::vector<Vec3>& vertices) -> bool {
            const HalfEdge* cur = start;
            do {
                Edge* edge = cur->edge;
                auto fitIt = edgeToFillet.find(edge->id);

                if (fitIt != edgeToFillet.end()) {
                    const auto& fe = filletEdges[fitIt->second];
                    const auto& feSamples = samples[fitIt->second];
                    const bool forward = (cur->origin == fe.v1);
                    const bool isFaceA = (cur->face == fe.faceA);

                    // Emit the tangent chain for this side, in traversal order:
                    // the first (faceA) or last (faceB) sample of each stop, which
                    // at a mitered end is the tangent point on the miter plane.
                    auto tangent = [isFaceA](const std::vector<Vec3>& arc) {
                        return isFaceA ? arc.front() : arc.back();
                    };
                    if (forward) {
                        for (const auto& arc : feSamples) vertices.push_back(tangent(arc));
                    } else {
                        for (auto it = feSamples.rbegin(); it != feSamples.rend(); ++it) {
                            vertices.push_back(tangent(*it));
                        }
                    }
                } else {
                    const Vertex* v = cur->origin;

                    // A mitered vertex vanishes on every face around it: the cap
                    // and both side faces each receive it through a blend's
                    // tangent chain, which ends on the miter plane.
                    if (miters.count(v) != 0) {
                        cur = cur->next;
                        continue;
                    }

                    // Corner-blend vertices vanish — the adjacent chains meet.
                    bool blended = false;
                    for (const auto& b : blends) {
                        if (v == b.vertex) {
                            blended = true;
                            break;
                        }
                    }
                    if (blended) {
                        cur = cur->next;
                        continue;
                    }

                    // Fillet endpoints never survive: on the fillet's own faces
                    // the tangent chain already replaced them; on the end face
                    // (the third face at the vertex) the corner is chord-cut with
                    // the two tangent points, ordered to match the neighbouring
                    // faces: arrive on the side shared with the previous face,
                    // leave on the side shared with the next.
                    auto endIt = endVertexToFillet.find(v);
                    if (endIt != endVertexToFillet.end()) {
                        const auto& fe = filletEdges[endIt->second];
                        if (cur->face == fe.faceA || cur->face == fe.faceB) {
                            cur = cur->next;  // chain covers this corner
                            continue;
                        }
                        const Face* arrivingFace =
                            cur->prev->twin != nullptr ? cur->prev->twin->face : nullptr;
                        const Face* leavingFace = cur->twin != nullptr ? cur->twin->face : nullptr;
                        if ((arrivingFace != fe.faceA && arrivingFace != fe.faceB) ||
                            (leavingFace != fe.faceA && leavingFace != fe.faceB)) {
                            result.errorMessage =
                                "Unsupported fillet-end configuration at vertex (non box-like "
                                "corner)";
                            return false;
                        }
                        // The end face carries the whole arc, not just its chord:
                        // the blend is faceted across the arc, so anything less
                        // leaves the two boundaries disagreeing and the shell open.
                        // As the blend's own end section, carried onto this face.
                        const auto& feSamples = samples[endIt->second];
                        auto chain = (fe.v1 == v) ? feSamples.front() : feSamples.back();
                        if (arrivingFace == fe.faceB) {
                            std::reverse(chain.begin(), chain.end());
                        }
                        for (const auto& p : chain) {
                            vertices.push_back(p);
                        }
                        cur = cur->next;
                        continue;
                    }

                    vertices.push_back(v->point);
                }
                cur = cur->next;
            } while (cur != start);
            return true;
        };
        if (!rewrite(face.outerLoop->halfEdge, fd.vertices)) return result;
        for (const Wire* inner : face.innerLoops) {
            fd.holes.emplace_back();
            if (!rewrite(inner->halfEdge, fd.holes.back())) return result;
        }

        newFaces.push_back(std::move(fd));
    }

    // Add fillet faces: one ruled patch per stop segment, oriented to oppose
    // the adjacent face chains (faceA traverses stops forward, faceB
    // backward).
    // A rim's chords share one circle, and so their bands one torus: found
    // again by where it is, not by the curve object (a rim's chords may
    // carry copies of one circle).
    std::vector<std::shared_ptr<geo::NurbsSurface>> tori;
    std::map<const geo::NurbsSurface*, Circle> torusAxis;  ///< each torus's rim circle
    const auto sameTorus = [](const geo::NurbsSurface& a, const geo::NurbsSurface& b) {
        const auto& pa = a.controlPoints();
        const auto& pb = b.controlPoints();
        if (pa.size() != pb.size()) return false;
        for (size_t u = 0; u < pa.size(); ++u) {
            if (pa[u].size() != pb[u].size()) return false;
            for (size_t v = 0; v < pa[u].size(); ++v) {
                if ((pa[u][v] - pb[u][v]).length() > 1e-9) return false;
            }
        }
        return true;
    };
    for (size_t i = 0; i < filletEdges.size(); ++i) {
        const auto& fe = filletEdges[i];
        std::shared_ptr<geo::NurbsSurface> torus = makeRimTorus(fe);
        if (torus) {
            const auto known = std::find_if(tori.begin(), tori.end(),
                                            [&](const auto& t) { return sameTorus(*t, *torus); });
            if (known != tori.end()) {
                torus = *known;
            } else {
                tori.push_back(torus);
                if (const auto circle = circleOf(*fe.originalEdge->analyticCurve)) {
                    torusAxis[torus.get()] = *circle;
                }
            }
        }
        for (size_t s = 0; s + 1 < fe.stops.size(); ++s) {
            const auto& lo = samples[i][s];
            const auto& hi = samples[i][s + 1];
            const size_t bands = std::min(lo.size(), hi.size()) - 1;
            // The whole-arc surface stays available as the ideal geometry each
            // band approximates; the bands themselves carry planar carriers,
            // synthesized below, that match their loops.
            auto analytic = torus ? torus : makeArcLoft(fe, fe.stops[s], fe.stops[s + 1]);
            for (size_t j = 0; j < bands; ++j) {
                NewFaceData fd;
                fd.isOriginal = false;
                fd.topoId =
                    TopologyID::make(featureID, "fillet")
                        .child(fe.originalEdge->topoId.tag(), static_cast<int>(s * bands + j));
                fd.vertices.push_back(hi[j]);
                fd.vertices.push_back(lo[j]);
                fd.vertices.push_back(lo[j + 1]);
                fd.vertices.push_back(hi[j + 1]);
                fd.analyticSurface = analytic;
                newFaces.push_back(std::move(fd));
            }
        }
    }

    // Add spherical corner-blend faces. Orientation: the three fillet end
    // arcs, as traversed by their quads, form a directed 3-cycle around the
    // corner; the sphere loop is that cycle reversed.
    for (size_t bi = 0; bi < blends.size(); ++bi) {
        const auto& b = blends[bi];

        auto keyOf = [&](const Vec3& p) -> int {
            for (size_t k = 0; k < b.cornerPoints.size(); ++k) {
                if (p.distanceTo(b.cornerPoints[k]) < 1e-9) return static_cast<int>(k);
            }
            return -1;
        };

        // Directed end arcs at this corner, as the quads traverse them.
        // Quad loop [A_next, A_end, B_end, B_next] contains A_end → B_end at
        // the corner end when the corner is at the front (v1) stop, and
        // B_end → A_end when at the back (v2) stop.
        // Each end arc is now a chain of samples, not one segment, so the
        // corner loop follows those samples too — otherwise the sphere patch
        // would be a flat triangle spanning three arcs that are no longer
        // straight, and its boundary would not match the blends it joins.
        std::map<int, int> arcNext;
        std::map<int, std::vector<Vec3>> arcInterior;
        bool arcsOk = true;
        for (size_t k : b.edgeIndices) {
            const auto& fe = filletEdges[k];
            const bool atFront = (fe.v1 == b.vertex);
            const FilletStop& end = atFront ? fe.front() : fe.back();
            auto chain = arcSamples(end, arcSegments);
            if (!atFront) {
                std::reverse(chain.begin(), chain.end());  // quad traverses B_end → A_end
            }
            const int a = keyOf(chain.front());
            const int c = keyOf(chain.back());
            if (a < 0 || c < 0) {
                arcsOk = false;
                break;
            }
            arcNext[a] = c;
            arcInterior[a] = std::vector<Vec3>(chain.begin() + 1, chain.end() - 1);
        }
        if (!arcsOk || arcNext.size() != 3) {
            result.errorMessage = "Vertex blend arcs do not form a corner cycle";
            return result;
        }

        // Walk the cycle and reverse it for the sphere loop.
        std::vector<int> cycle;
        int at = arcNext.begin()->first;
        for (int step = 0; step < 3; ++step) {
            cycle.push_back(at);
            auto it = arcNext.find(at);
            if (it == arcNext.end()) {
                arcsOk = false;
                break;
            }
            at = it->second;
        }
        if (!arcsOk || at != cycle.front()) {
            result.errorMessage = "Vertex blend arcs do not close into a corner cycle";
            return result;
        }

        NewFaceData fd;
        fd.isOriginal = false;
        fd.topoId = TopologyID::make(featureID, "blend").child("corner", static_cast<int>(bi));
        // Reverse the cycle for the sphere loop, carrying each chain's
        // interior samples with it.
        std::vector<Vec3> forward;
        for (int key : cycle) {
            forward.push_back(b.cornerPoints[static_cast<size_t>(key)]);
            const auto it = arcInterior.find(key);
            if (it != arcInterior.end()) {
                forward.insert(forward.end(), it->second.begin(), it->second.end());
            }
        }
        fd.vertices.assign(forward.rbegin(), forward.rend());
        // The corner is the eighth of the ball facing the vertex: from the
        // ball's centre, back along each of the three edges. Drawing and
        // exporting the whole sphere left seven eighths of it inside the part
        // and in every STL and glTF export. The whole sphere stays as the
        // ideal surface, for mates and dimensions.
        Vec3 axes[3];
        for (size_t k = 0; k < 3; ++k) {
            axes[k] = dirFromVertex(filletEdges[b.edgeIndices[k]], b.vertex) * (-1.0);
        }
        if (axes[0].dot(axes[1].cross(axes[2])) < 0.0) std::swap(axes[0], axes[1]);
        fd.surface = std::make_shared<geo::NurbsSurface>(geo::NurbsSurface::makeSphereOctant(
            b.sphereCenter, b.radius, axes[0], axes[1], axes[2]));
        fd.analyticSurface = std::make_shared<geo::NurbsSurface>(
            geo::NurbsSurface::makeSphere(b.sphereCenter, b.radius));
        newFaces.push_back(std::move(fd));
    }

    // -- Deduplicate positions into indexed loops --
    std::vector<Vec3> uniquePositions;
    std::map<size_t, std::vector<int>> faceVertexIndices;

    auto findOrAddVertex = [&](const Vec3& pos) -> int {
        for (size_t i = 0; i < uniquePositions.size(); ++i) {
            if (pos.distanceTo(uniquePositions[i]) < 1e-9) {
                return static_cast<int>(i);
            }
        }
        uniquePositions.push_back(pos);
        return static_cast<int>(uniquePositions.size() - 1);
    };

    std::map<size_t, std::vector<std::vector<int>>> faceHoleIndices;
    // A loop's points as vertex indices. Corner-blend chains meet at shared
    // tangent points: consecutive duplicates (and the wraparound duplicate)
    // are dropped so no degenerate edges enter the loop.
    const auto indexed = [&findOrAddVertex](const std::vector<Vec3>& loop) {
        std::vector<int> cleaned;
        for (const auto& pos : loop) {
            const int idx = findOrAddVertex(pos);
            if (cleaned.empty() || cleaned.back() != idx) cleaned.push_back(idx);
        }
        while (cleaned.size() > 1 && cleaned.front() == cleaned.back()) cleaned.pop_back();
        return cleaned;
    };
    for (size_t fi = 0; fi < newFaces.size(); ++fi) {
        std::vector<int> cleaned = indexed(newFaces[fi].vertices);
        if (cleaned.size() < 3) {
            result.errorMessage = "Degenerate face loop after fillet";
            return result;
        }
        faceVertexIndices[fi] = std::move(cleaned);
        for (const auto& hole : newFaces[fi].holes) {
            std::vector<int> holeIndices = indexed(hole);
            if (holeIndices.size() < 3) {
                result.errorMessage = "Degenerate face loop after fillet";
                return result;
            }
            faceHoleIndices[fi].push_back(std::move(holeIndices));
        }
    }

    const int numVerts = static_cast<int>(uniquePositions.size());
    if (numVerts < 4 || newFaces.size() < 4) {
        result.errorMessage = "Degenerate solid after fillet (too few vertices or faces)";
        return result;
    }

    // -- Assemble the half-edge structure directly --
    // Each face loop spawns its half-edges; twins link by strict (a, b) /
    // (b, a) pairing. Any unmatched or doubly-used directed pair means the
    // emitted loops are not a closed 2-manifold — fail cleanly.
    auto solid = std::make_unique<Solid>();
    Shell* shell = solid->allocShell();
    shell->solid = solid.get();

    std::vector<Vertex*> verts(static_cast<size_t>(numVerts), nullptr);
    for (int i = 0; i < numVerts; ++i) {
        Vertex* v = solid->allocVertex();
        v->point = uniquePositions[static_cast<size_t>(i)];
        verts[static_cast<size_t>(i)] = v;
    }

    std::map<std::pair<int, int>, HalfEdge*> directedEdges;
    std::vector<Face*> builtFaces;

    for (size_t fi = 0; fi < newFaces.size(); ++fi) {
        const auto& indices = faceVertexIndices[fi];

        Face* face = solid->allocFace();
        face->shell = shell;
        face->topoId = newFaces[fi].topoId;
        shell->faces.push_back(face);
        builtFaces.push_back(face);

        // One wire per loop: the outline, then each hole.
        const auto build = [&](const std::vector<int>& loop) -> Wire* {
            const size_t m = loop.size();
            Wire* wire = solid->allocWire();
            std::vector<HalfEdge*> hes(m, nullptr);
            for (size_t k = 0; k < m; ++k) {
                HalfEdge* he = solid->allocHalfEdge();
                he->origin = verts[static_cast<size_t>(loop[k])];
                he->face = face;
                hes[k] = he;

                const std::pair<int, int> key{loop[k], loop[(k + 1) % m]};
                if (directedEdges.count(key) != 0) return nullptr;
                directedEdges[key] = he;
            }
            for (size_t k = 0; k < m; ++k) {
                hes[k]->next = hes[(k + 1) % m];
                hes[k]->prev = hes[(k + m - 1) % m];
                if (hes[k]->origin->halfEdge == nullptr) {
                    hes[k]->origin->halfEdge = hes[k];
                }
            }
            wire->halfEdge = hes.front();
            return wire;
        };
        face->outerLoop = build(indices);
        bool manifold = face->outerLoop != nullptr;
        for (const auto& hole : faceHoleIndices[fi]) {
            if (!manifold) break;
            Wire* wire = build(hole);
            manifold = wire != nullptr;
            face->innerLoops.push_back(wire);
        }
        if (!manifold) {
            result.errorMessage = "Fillet produced a non-manifold face loop";
            return result;
        }
    }

    // Twin-link and create one Edge per undirected pair.
    for (auto& [key, he] : directedEdges) {
        if (he->twin != nullptr) continue;
        auto opposite = directedEdges.find({key.second, key.first});
        if (opposite == directedEdges.end()) {
            result.errorMessage = "Fillet left an open boundary (unmatched edge " +
                                  std::to_string(key.first) + "-" + std::to_string(key.second) +
                                  ")";
            return result;
        }
        HalfEdge* twin = opposite->second;
        he->twin = twin;
        twin->twin = he;

        Edge* edge = solid->allocEdge();
        edge->halfEdge = he;
        he->edge = edge;
        twin->edge = edge;
    }

    if (!solid->isValid()) {
        result.errorMessage = "Fillet produced invalid topology:\n" + solid->validationReport();
        return result;
    }

    // -- Assign edge TopologyIDs and curves --
    {
        int idx = 0;
        for (auto& e : const_cast<std::deque<Edge>&>(solid->edges())) {
            e.topoId = TopologyID::make(featureID, "edge" + std::to_string(idx));
            ++idx;
            assignEdgeCurve(&e);
        }
    }

    // -- Bind NURBS surfaces (faces map 1:1 to the emitted loops) --
    for (size_t fi = 0; fi < builtFaces.size(); ++fi) {
        Face* f = builtFaces[fi];
        f->analyticSurface = newFaces[fi].analyticSurface;
        if (newFaces[fi].surface != nullptr) {
            f->surface = newFaces[fi].surface;
            continue;
        }

        auto fv = faceVertices(f);
        if (fv.size() < 3) {
            continue;
        }
        // Bind planar NURBS surface computed from the vertex loop.
        Vec3 origin = fv[0]->point;
        Vec3 u = fv[1]->point - fv[0]->point;
        Vec3 v_dir(0, 0, 0);
        for (size_t i = 2; i < fv.size(); ++i) {
            v_dir = fv[i]->point - fv[0]->point;
            Vec3 cross = u.cross(v_dir);
            if (cross.length() > 1e-9) {
                break;
            }
        }
        double uSize = u.length();
        double vSize = v_dir.length();
        if (uSize < 1e-12) {
            uSize = 1.0;
        }
        if (vSize < 1e-12) {
            vSize = 1.0;
        }
        Vec3 uDir = u * (1.0 / uSize);
        Vec3 vDir = v_dir * (1.0 / vSize);

        // Align the carrier normal (u × v) with the loop's outward Newell
        // normal so planar faces keep the kernel's surface-normal-points-out
        // convention regardless of where the loop starts.
        Vec3 newell(0, 0, 0);
        for (size_t i = 0; i < fv.size(); ++i) {
            const Vec3& a = fv[i]->point;
            const Vec3& b = fv[(i + 1) % fv.size()]->point;
            newell.x += (a.y - b.y) * (a.z + b.z);
            newell.y += (a.z - b.z) * (a.x + b.x);
            newell.z += (a.x - b.x) * (a.y + b.y);
        }
        if (uDir.cross(vDir).dot(newell) < 0.0) {
            std::swap(uDir, vDir);
            std::swap(uSize, vSize);
        }

        f->surface = std::make_shared<geo::NurbsSurface>(
            geo::NurbsSurface::makePlane(origin, uDir, vDir, uSize, vSize));
    }

    // A rim's fillet (Phase 164): where its torus meets a face beside it,
    // each edge is a chord of a circle about the rim's axis. Each records
    // its circle, as a revolve's rims do, so an export finds what it was
    // cut from; one circle for all the chords of one tangent line.
    if (!torusAxis.empty()) {
        struct Ring {
            double height;
            double radius;
            std::shared_ptr<geo::NurbsCurve> circle;
        };
        std::map<const geo::NurbsSurface*, std::vector<Ring>> rings;
        for (auto& e : const_cast<std::deque<Edge>&>(solid->edges())) {
            const HalfEdge* h = e.halfEdge;
            if (h == nullptr || h->twin == nullptr) continue;
            const auto* sa = h->face->analyticSurface.get();
            const auto* sb = h->twin->face->analyticSurface.get();
            if (sa == sb) continue;  // within the band, or between two others
            auto found = torusAxis.find(sa);
            if (found == torusAxis.end()) found = torusAxis.find(sb);
            if (found == torusAxis.end()) continue;
            const Circle& rim = found->second;
            const auto place = [&rim](const Vec3& p) {
                const Vec3 d = p - rim.centre;
                const double height = d.dot(rim.axis);
                return std::pair<double, double>{height, (d - rim.axis * height).length()};
            };
            // Plain names, not bindings: clang before 16 cannot capture those.
            const auto first = place(h->origin->point);
            const auto second = place(h->twin->origin->point);
            const double h0 = first.first;
            const double r0 = first.second;
            const double h1 = second.first;
            const double r1 = second.second;
            const double tol = 1e-9 * std::max(1.0, rim.radius);
            if (std::abs(h0 - h1) > tol || std::abs(r0 - r1) > tol || r0 < tol) continue;
            auto& list = rings[found->first];
            auto ring = std::find_if(list.begin(), list.end(), [&](const Ring& g) {
                return std::abs(g.height - h0) <= tol && std::abs(g.radius - r0) <= tol;
            });
            if (ring == list.end()) {
                list.push_back({h0, r0,
                                std::make_shared<geo::NurbsCurve>(geo::NurbsCurve::makeCircle(
                                    rim.centre + rim.axis * h0, r0, rim.axis))});
                ring = list.end() - 1;
            }
            e.analyticCurve = ring->circle;
        }
    }

    // Same output contract as ChamferOp: a fillet that produced structurally
    // sound but geometrically inconsistent loops is refused, not returned.
    // Curved carriers (the blend patches themselves) are exempt from the
    // planarity check by construction, so this gates the planar remainder.
    const auto issues = topo::GeometryValidator::check(*solid);
    if (!issues.ok()) {
        result.errorMessage =
            "Fillet produced invalid geometry:\n" + topo::GeometryValidator::report(*solid);
        return result;
    }

    result.solid = std::move(solid);
    return result;
}

// ---------------------------------------------------------------------------
// Public entry points
// ---------------------------------------------------------------------------

int FilletOp::arcSegmentsForTolerance(double radius, double tolerance) {
    if (!(radius > 0.0) || !(tolerance > 0.0)) {
        return kDefaultArcSegments;
    }
    // A chord spanning angle a on a circle of radius r sags r(1 - cos(a/2)).
    const double ratio = std::max(-1.0, 1.0 - tolerance / radius);
    const double maxChordAngle = 2.0 * std::acos(ratio);
    const double n = (0.5 * math::kPi) / maxChordAngle;
    return std::clamp(static_cast<int>(std::ceil(n - 1e-9)), 1, 1024);
}

/// A part of several bodies (as Pattern::collect makes them): each body with
/// edges to round is rounded alone, and the bodies put back together, names
/// kept. The rebuild puts every face in one shell, which another body's faces
/// cannot join. Nothing for a part of one body.
///
/// @p op is given the body, its edges and the feature's ID to name what it
/// makes by: the first body rounded, @p featureID; the k-th after it,
/// `<featureID>/body:<k>`. Each rebuild numbers its corner blends and new
/// edges from 0, so two bodies rounded under one ID made two faces named
/// `<featureID>/blend/corner:0`.
template <typename Op>
static std::optional<FilletResult> perBody(const Solid& input, const std::vector<TopologyID>& ids,
                                           const std::string& featureID, Op op) {
    auto bodies = Pattern::separate(input);
    if (bodies.size() < 2) return std::nullopt;
    std::vector<bool> found(ids.size(), false);
    std::unique_ptr<Solid> out;
    int rounded = 0;
    for (auto& body : bodies) {
        std::vector<TopologyID> mine;
        for (size_t i = 0; i < ids.size(); ++i) {
            if (!findEdges(*body, ids[i]).empty()) {
                mine.push_back(ids[i]);
                found[i] = true;
            }
        }
        std::unique_ptr<Solid> done;
        if (mine.empty()) {
            done = std::move(body);
        } else {
            const std::string id =
                rounded == 0 ? featureID : featureID + "/body:" + std::to_string(rounded);
            ++rounded;
            FilletResult part = op(*body, mine, id);
            if (!part.solid) return part;
            done = std::move(part.solid);
        }
        out = out ? Pattern::collect(*out, *done) : std::move(done);
    }
    FilletResult result;
    for (size_t i = 0; i < ids.size(); ++i) {
        if (!found[i]) {
            result.errorMessage = "Edge not found: " + ids[i].tag();
            return result;
        }
    }
    result.solid = std::move(out);
    return result;
}

FilletResult FilletOp::execute(const Solid& inputSolid, const std::vector<TopologyID>& edgeIds,
                               double radius, const std::string& featureID, int arcSegments,
                               NamingScheme naming) {
    FilletResult result;

    if (radius <= 0.0) {
        result.errorMessage = "Fillet radius must be positive";
        return result;
    }
    if (edgeIds.empty()) {
        result.errorMessage = "No edges specified for fillet";
        return result;
    }
    if (auto split = perBody(
            inputSolid, edgeIds, featureID,
            [&](const Solid& body, const std::vector<TopologyID>& mine, const std::string& id) {
                return execute(body, mine, radius, id, arcSegments, naming);
            })) {
        return std::move(*split);
    }

    const double outwardSign = signedLoopVolume(inputSolid) >= 0.0 ? 1.0 : -1.0;

    std::vector<FilletEdgeInfo> filletEdges;
    filletEdges.reserve(edgeIds.size());
    std::vector<std::pair<const Edge*, const TopologyID*>> chosen;
    for (const auto& eid : edgeIds) {
        const auto edges = findEdges(inputSolid, eid);
        if (edges.empty()) {
            result.errorMessage = "Edge not found: " + eid.tag();
            return result;
        }
        for (const Edge* edge : edges) {
            // Once each, however many references reach it.
            const bool listed = std::any_of(chosen.begin(), chosen.end(),
                                            [edge](const auto& c) { return c.first == edge; });
            if (!listed) chosen.emplace_back(edge, &eid);
        }
    }
    for (const auto& [edge, idPtr] : chosen) {
        const TopologyID& eid = *idPtr;
        FilletEdgeInfo info;
        if (!computeFilletFrame(edge, outwardSign, info)) {
            result.errorMessage = "Cannot compute fillet geometry for edge: " + eid.tag();
            return result;
        }
        info.stops.push_back(makeStop(info, 0.0, radius));
        info.stops.push_back(makeStop(info, 1.0, radius));
        filletEdges.push_back(std::move(info));
    }

    FilletResult built = executeCore(inputSolid, filletEdges, featureID, arcSegments);
    // The edges it did not touch keep their names; it renamed every edge in
    // storage order, so a second fillet on the part's edges lost them.
    if (built.solid && naming == NamingScheme::Stable) {
        nameBlendFaces(*built.solid, featureID + "/fillet/");
        keepEdgeNames(*built.solid, inputSolid);
    }
    return built;
}

FilletResult FilletOp::executeVariable(const Solid& inputSolid, const TopologyID& edgeId,
                                       const std::vector<RadiusStop>& stops,
                                       const std::string& featureID, int arcSegments) {
    FilletResult result;

    if (stops.size() < 2) {
        result.errorMessage = "Variable fillet needs at least two radius stops";
        return result;
    }
    for (size_t i = 0; i < stops.size(); ++i) {
        if (stops[i].radius <= 0.0) {
            result.errorMessage = "Fillet radius must be positive";
            return result;
        }
        if (i > 0 && stops[i].t <= stops[i - 1].t + 1e-12) {
            result.errorMessage = "Radius stops must have strictly increasing parameters";
            return result;
        }
    }
    if (std::abs(stops.front().t) > 1e-9 || std::abs(stops.back().t - 1.0) > 1e-9) {
        result.errorMessage = "Radius stops must cover t = 0 and t = 1";
        return result;
    }

    const auto edges = findEdges(inputSolid, edgeId);
    if (edges.size() != 1) {
        result.errorMessage = edges.empty()
                                  ? "Edge not found: " + edgeId.tag()
                                  : "A variable fillet needs one edge; " + edgeId.tag() +
                                        " is now in " + std::to_string(edges.size()) + " pieces";
        return result;
    }
    const Edge* edge = edges.front();
    FilletEdgeInfo info;
    const double outwardSign = signedLoopVolume(inputSolid) >= 0.0 ? 1.0 : -1.0;
    if (!computeFilletFrame(edge, outwardSign, info)) {
        result.errorMessage = "Cannot compute fillet geometry for edge: " + edgeId.tag();
        return result;
    }
    for (const auto& s : stops) {
        info.stops.push_back(makeStop(info, s.t, s.radius));
    }

    std::vector<FilletEdgeInfo> filletEdges;
    filletEdges.push_back(std::move(info));
    return executeCore(inputSolid, filletEdges, featureID, arcSegments);
}

}  // namespace hz::model
