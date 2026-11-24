#include <gtest/gtest.h>

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <map>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "horizon/geometry/Frame.h"
#include "horizon/geometry/curves/AnalyticCurves.h"
#include "horizon/geometry/curves/Curve2D.h"
#include "horizon/geometry/curves/Line2D.h"
#include "horizon/geometry/surfaces/AnalyticSurfaces.h"
#include "horizon/geometry/surfaces/NurbsSurface.h"
#include "horizon/geometry/surfaces/Surface.h"
#include "horizon/math/Constants.h"
#include "horizon/math/Mat4.h"
#include "horizon/math/ModelTolerance.h"
#include "horizon/modeling/ExactPrimitives.h"
#include "horizon/modeling/Pattern.h"
#include "horizon/modeling/TrimmedFaceMesher.h"
#include "horizon/topology/ExactValidator.h"
#include "horizon/topology/Solid.h"
#include "horizon/topology/TopologyID.h"

using namespace hz;
using math::Vec2;
using math::Vec3;
using model::FaceMesh;
using model::MeshOptions;
using model::TrimmedFaceMesher;

namespace {

double distanceToSegment(const Vec3& p, const Vec3& a, const Vec3& b) {
    const Vec3 ab = b - a;
    const double len2 = ab.lengthSquared();
    const double t = len2 > 0.0 ? std::clamp((p - a).dot(ab) / len2, 0.0, 1.0) : 0.0;
    return (p - (a + ab * t)).length();
}

/// The distance from @p p to the triangle abc: to its plane when p projects inside it, else to
/// its nearest side. Cross products keep it exact enough on a long thin triangle.
double distanceToTriangle(const Vec3& p, const Vec3& a, const Vec3& b, const Vec3& c) {
    const Vec3 n = (b - a).cross(c - a);
    const bool inside = (b - a).cross(p - a).dot(n) >= 0.0 && (c - b).cross(p - b).dot(n) >= 0.0 &&
                        (a - c).cross(p - c).dot(n) >= 0.0;
    if (n.length() > 0.0 && inside) return std::fabs((p - a).dot(n)) / n.length();
    return std::min(
        {distanceToSegment(p, a, b), distanceToSegment(p, b, c), distanceToSegment(p, c, a)});
}

/// The surface's parameters at @p p, a periodic one moved by whole periods to lie nearest
/// @p near: a triangle at the seam has vertices that invert to both sides of it.
Vec2 uvNear(const geo::Surface& s, const Vec3& p, const Vec2& near) {
    Vec2 uv = s.invert(p);
    if (s.periodicU()) {
        const double period = s.uRange().max - s.uRange().min;
        uv.x += std::round((near.x - uv.x) / period) * period;
    }
    if (s.periodicV()) {
        const double period = s.vRange().max - s.vRange().min;
        uv.y += std::round((near.y - uv.y) / period) * period;
    }
    return uv;
}

using Point = std::array<double, 3>;
using Segment = std::pair<Point, Point>;

Point key(const Vec3& p) {
    return {p.x, p.y, p.z};
}

struct Summary {
    std::map<std::string, double> area;            // by face tag
    std::map<std::string, std::size_t> triangles;  // by face tag
    double volume = 0.0;
    std::size_t boundarySegments = 0;  // across all the meshes, each way counted
};

/// The brief's properties of a solid's meshes: each face meshed; watertight (every boundary
/// segment, as an ordered pair of positions, once each way across all the meshes, which covers
/// shared edges and seams alike; once only on an open sheet, whose edges have no second face);
/// within the chord tolerance (plus @p slack for the inversion the check itself does) at each
/// triangle's (u, v) centroid and edge midpoints; every triangle facing the way the surface's
/// outward normal does at its centroid; normals the surface's, outward.
Summary checkMeshes(const topo::Solid& solid, const std::vector<FaceMesh>& meshes,
                    const MeshOptions& options, double slack, bool closed = true) {
    Summary out;
    std::size_t live = 0;
    for (const auto& f : solid.faces()) live += f.alive ? 1 : 0;
    EXPECT_EQ(meshes.size(), live);
    std::map<Segment, int> boundary;
    for (const FaceMesh& m : meshes) {
        EXPECT_TRUE(m.problem.empty()) << m.problem;
        if (!m.face || !m.face->exactSurface || !m.problem.empty()) continue;
        const std::string tag = m.face->topoId.tag();
        EXPECT_FALSE(m.triangles.empty()) << tag;
        EXPECT_EQ(m.points.size(), m.normals.size()) << tag;
        const geo::Surface& s = *m.face->exactSurface;
        const double sign = m.face->reversed ? -1.0 : 1.0;

        std::map<std::pair<std::uint32_t, std::uint32_t>, int> directed;
        double worst = 0.0;
        for (const auto& t : m.triangles) {
            if (std::any_of(t.begin(), t.end(),
                            [&](std::uint32_t i) { return i >= m.points.size(); })) {
                ADD_FAILURE() << tag << ": an index out of range";
                continue;
            }
            for (int k = 0; k < 3; ++k) ++directed[{t[k], t[(k + 1) % 3]}];
            const Vec3& a = m.points[t[0]];
            const Vec3& b = m.points[t[1]];
            const Vec3& c = m.points[t[2]];
            const Vec2 ua = s.invert(a);
            const Vec2 ub = uvNear(s, b, ua);
            const Vec2 uc = uvNear(s, c, ua);
            const Vec2 centroid = (ua + ub + uc) / 3.0;
            for (const Vec2& q : {centroid, (ua + ub) * 0.5, (ub + uc) * 0.5, (uc + ua) * 0.5}) {
                worst = std::max(worst, distanceToTriangle(s.evaluate(q.x, q.y), a, b, c));
            }
            const Vec3 n = (b - a).cross(c - a);
            EXPECT_GT(n.dot(s.normal(centroid.x, centroid.y) * sign), 0.0)
                << tag << ": an inverted triangle";
            out.area[tag] += 0.5 * n.length();
            out.volume += a.dot(b.cross(c)) / 6.0;
        }
        EXPECT_LE(worst, options.chordTolerance + slack) << tag;
        out.triangles[tag] += m.triangles.size();

        for (const auto& [e, uses] : directed) {
            EXPECT_EQ(uses, 1) << tag << ": an edge used twice the same way";
            if (directed.count({e.second, e.first}) == 0) {
                ++boundary[{key(m.points[e.first]), key(m.points[e.second])}];
            }
        }
        for (std::size_t i = 0; i < m.points.size(); ++i) {
            const Vec2 uv = s.invert(m.points[i]);
            EXPECT_NEAR(m.normals[i].dot(s.normal(uv.x, uv.y) * sign), 1.0, 1e-9) << tag;
        }
    }
    EXPECT_FALSE(boundary.empty());
    out.boundarySegments = boundary.size();
    for (const auto& [seg, uses] : boundary) {
        EXPECT_EQ(uses, 1) << "a boundary segment used twice the same way";
        const auto back = boundary.find({seg.second, seg.first});
        if (closed) {
            EXPECT_TRUE(back != boundary.end() && back->second == 1) << "a segment unmatched";
        } else {
            EXPECT_TRUE(back == boundary.end()) << "a segment of an open sheet matched";
        }
    }
    return out;
}

/// The mesh's whole area. A mesh within a tolerance of its solid's surface encloses the solid's
/// volume give or take that area times the tolerance.
double totalArea(const Summary& s) {
    double sum = 0.0;
    for (const auto& [tag, a] : s.area) sum += a;
    return sum;
}

/// The model tolerance a part of this size is checked to.
double slackFor(double extent) {
    return math::ModelTolerance::forExtent(extent).linear;
}

std::vector<FaceMesh> meshValid(const topo::Solid& solid, const MeshOptions& options,
                                double extent) {
    EXPECT_TRUE(topo::validateExact(solid, math::ModelTolerance::forExtent(extent)).empty());
    return TrimmedFaceMesher::meshSolid(solid, options);
}

/// A torus as one face: the square [0, 2 pi]^2 of its (u, v), closed up by two edges that each
/// have both half-edges in the face, as a cylinder's seam does: the outer equator (along u at
/// v = 0 and v = 2 pi) and a meridian (along v at u = 0 and u = 2 pi), meeting at the one vertex.
std::unique_ptr<topo::Solid> makeTorus(const geo::Frame& frame, double major, double minor) {
    auto solid = std::make_unique<topo::Solid>();
    topo::Shell* shell = solid->allocShell();
    shell->solid = solid.get();
    topo::Vertex* vertex = solid->allocVertex();
    vertex->point = frame.origin + frame.x * (major + minor);
    topo::Face* face = solid->allocFace();
    face->topoId = topo::TopologyID::make("torus", "face");
    face->exactSurface = std::make_shared<geo::Torus>(frame, major, minor);
    face->shell = shell;
    shell->faces.push_back(face);
    // Counter-clockwise in (u, v): along the equator, up the meridian at u = 2 pi, back along
    // the equator at v = 2 pi, down the meridian at u = 0.
    const double T = math::kTwoPi;
    const std::array<Vec2, 4> corners = {{{0, 0}, {T, 0}, {T, T}, {0, T}}};
    std::array<topo::HalfEdge*, 4> h{};
    for (std::size_t i = 0; i < 4; ++i) {
        h[i] = solid->allocHalfEdge();
        h[i]->origin = vertex;
        h[i]->face = face;
        h[i]->pcurve = std::make_shared<geo::Line2D>(corners[i], corners[(i + 1) % 4]);
        h[i]->pcurveStart = 0.0;
        h[i]->pcurveEnd = 1.0;
    }
    for (std::size_t i = 0; i < 4; ++i) {
        h[i]->next = h[(i + 1) % 4];
        h[(i + 1) % 4]->prev = h[i];
    }
    vertex->halfEdge = h[0];
    topo::Wire* wire = solid->allocWire();
    wire->halfEdge = h[0];
    face->outerLoop = wire;
    geo::Frame meridian = frame;  // C(t) = S(0, t)
    meridian.origin = frame.origin + frame.x * major;
    meridian.y = frame.z;
    meridian.z = frame.x.cross(frame.z);
    const std::array<std::shared_ptr<const geo::Curve3>, 2> curves = {
        std::make_shared<geo::CircleCurve>(frame, major + minor),
        std::make_shared<geo::CircleCurve>(meridian, minor)};
    for (std::size_t k = 0; k < 2; ++k) {
        topo::Edge* e = solid->allocEdge();
        e->halfEdge = h[k];
        h[k]->twin = h[k + 2];
        h[k + 2]->twin = h[k];
        h[k]->edge = h[k + 2]->edge = e;
        e->exactCurve = curves[k];
        e->curveStart = 0.0;
        e->curveEnd = T;
    }
    return solid;
}

/// A side of an open face: its pcurve from start to end, and its edge's curve over
/// [curveStart, curveEnd]; without a curve, the edge is the line between the side's ends.
struct Side {
    std::shared_ptr<const geo::Curve2D> pcurve;
    double start = 0.0;
    double end = 1.0;
    std::shared_ptr<const geo::Curve3> curve;
    double curveStart = 0.0;
    double curveEnd = 0.0;
};

/// One open face on @p surface: the first loop its outer one, the rest holes, each running as
/// the face's orientation needs. The edges' other half-edges have no face.
std::unique_ptr<topo::Solid> makeOpenFace(std::shared_ptr<const geo::Surface> surface,
                                          const std::vector<std::vector<Side>>& loops) {
    auto solid = std::make_unique<topo::Solid>();
    topo::Shell* shell = solid->allocShell();
    shell->solid = solid.get();
    topo::Face* face = solid->allocFace();
    face->topoId = topo::TopologyID::make("open", "face");
    face->exactSurface = std::move(surface);
    face->shell = shell;
    shell->faces.push_back(face);
    for (const std::vector<Side>& sides : loops) {
        const std::size_t n = sides.size();
        std::vector<topo::Vertex*> v(n);
        std::vector<topo::HalfEdge*> h(n);
        for (std::size_t i = 0; i < n; ++i) {
            const Vec2 uv = sides[i].pcurve->evaluate(sides[i].start);
            v[i] = solid->allocVertex();
            v[i]->point = face->exactSurface->evaluate(uv.x, uv.y);
            h[i] = solid->allocHalfEdge();
            h[i]->origin = v[i];
            h[i]->face = face;
            h[i]->pcurve = sides[i].pcurve;
            h[i]->pcurveStart = sides[i].start;
            h[i]->pcurveEnd = sides[i].end;
            v[i]->halfEdge = h[i];
        }
        for (std::size_t i = 0; i < n; ++i) {
            h[i]->next = h[(i + 1) % n];
            h[(i + 1) % n]->prev = h[i];
            topo::HalfEdge* other = solid->allocHalfEdge();
            other->origin = v[(i + 1) % n];
            topo::Edge* e = solid->allocEdge();
            e->halfEdge = h[i];
            h[i]->twin = other;
            other->twin = h[i];
            h[i]->edge = other->edge = e;
            if (sides[i].curve) {
                e->exactCurve = sides[i].curve;
                e->curveStart = sides[i].curveStart;
                e->curveEnd = sides[i].curveEnd;
            } else {
                const Vec3 d = v[(i + 1) % n]->point - v[i]->point;
                e->exactCurve = std::make_shared<geo::LineCurve>(v[i]->point, d);
                e->curveEnd = d.length();
            }
        }
        topo::Wire* wire = solid->allocWire();
        wire->halfEdge = h[0];
        if (face->outerLoop) {
            face->innerLoops.push_back(wire);
        } else {
            face->outerLoop = wire;
        }
    }
    return solid;
}

/// The bilinear patch through @p p00, @p p10, @p p01 and @p p11 (at (u, v) = (0, 0), (1, 0),
/// (0, 1) and (1, 1)).
std::shared_ptr<geo::Surface> bilinear(const Vec3& p00, const Vec3& p10, const Vec3& p01,
                                       const Vec3& p11) {
    return std::make_shared<geo::NurbsSurface>(
        std::vector<std::vector<Vec3>>{{p00, p01}, {p10, p11}},
        std::vector<std::vector<double>>{{1, 1}, {1, 1}}, std::vector<double>{0, 0, 1, 1},
        std::vector<double>{0, 0, 1, 1}, 1, 1);
}

std::shared_ptr<geo::Curve2D> line(const Vec2& a, const Vec2& b) {
    return std::make_shared<geo::Line2D>(a, b);
}

/// The open face z = h u v over [0, L]^2 (a hyperbolic paraboloid), bounded by four straight
/// edges. It twists without bending along u or v, so a grid spaced from those curvatures has
/// nothing to go on: only refinement meets the tolerance.
std::unique_ptr<topo::Solid> makeTwistedPatch(double size, double rise) {
    return makeOpenFace(bilinear({0, 0, 0}, {size, 0, 0}, {0, size, 0}, {size, size, rise}),
                        {{{line({0, 0}, {1, 0})},
                          {line({1, 0}, {1, 1})},
                          {line({1, 1}, {0, 1})},
                          {line({0, 1}, {0, 0})}}});
}

/// (t / (2 - t), t): on the flat bilinear patch through (0,0), (2,0), (0,1), (1,1), the course of
/// the straight line from (0,0) to (1,1). It bows in (u, v) though its edge is straight.
class Bowed : public geo::Curve2D {
public:
    Vec2 evaluate(double t) const override { return {t / (2.0 - t), t}; }
    Vec2 derivative(double t, int order) const override {
        const double d = 2.0 - t;
        if (order == 1) return {2.0 / (d * d), 1.0};
        if (order == 2) return {4.0 / (d * d * d), 0.0};
        return {};
    }
    double tMin() const override { return 0.0; }
    double tMax() const override { return 1.0; }
    bool isClosed() const override { return false; }
};

/// A quadrilateral loop on the cylinder of @p frame and radius @p radius, from corner i to i + 1
/// alternately along an arc (v constant) and a line along the axis; corners may lie beyond 2 pi.
/// Side i's pcurve is written @p written[i] (a whole period, or none) away from its corners.
std::vector<Side> quadOnCylinder(const geo::Frame& frame, double radius,
                                 const std::array<Vec2, 4>& corner,
                                 const std::array<double, 4>& written) {
    std::vector<Side> sides;
    for (std::size_t i = 0; i < 4; ++i) {
        const Vec2& a = corner[i];
        const Vec2& b = corner[(i + 1) % 4];
        Side side{line(a + Vec2{written[i], 0}, b + Vec2{written[i], 0})};
        if (a.y == b.y) {  // an arc about the axis at this height
            geo::Frame at = frame;
            at.origin = frame.origin + frame.z * a.y;
            side.curve = std::make_shared<geo::CircleCurve>(at, radius);
            side.curveStart = a.x;
            side.curveEnd = b.x;
        }
        sides.push_back(side);
    }
    return sides;
}

}  // namespace

TEST(TrimmedFaceMesher, AFaceAcrossTheSeamMeshesWhicheverWindowItsPcurvesAreWrittenIn) {
    // Half a cylinder, u in [3 pi/2, 5 pi/2], across the surface's seam at 2 pi, with a hole
    // astride the seam. Its right side is written at 5 pi/2 or at pi/2, and the hole's first side
    // in its own window or a period lower: the loops are laid out as the validator lays them out
    // (unwrapped, the hole brought into the outer loop's window) or the face would not mesh.
    const geo::Frame frame = geo::Frame::fromAxis({1, -2, 3}, {0, 0, 1});
    const double r = 2.0;
    const double a = 1.5 * math::kPi;
    const double b = 2.5 * math::kPi;
    const double u = math::kTwoPi;
    for (const double right : {0.0, -math::kTwoPi}) {
        for (const double first : {0.0, -math::kTwoPi}) {
            const auto face = makeOpenFace(
                std::make_shared<geo::Cylinder>(frame, r),
                {quadOnCylinder(frame, r, {{{a, 0}, {b, 0}, {b, 3}, {a, 3}}}, {0, right, 0, 0}),
                 quadOnCylinder(frame, r,
                                {{{u + 0.2, 1}, {u - 0.2, 1}, {u - 0.2, 2}, {u + 0.2, 2}}},
                                {first, 0, 0, 0})});
            MeshOptions options;
            options.chordTolerance = 1e-3;
            const Summary s =
                checkMeshes(*face, meshValid(*face, options, 10.0), options, slackFor(10.0), false);
            const double area = math::kPi * r * 3.0 - 0.4 * r * 1.0;
            EXPECT_NEAR(s.area.at("open/face"), area, 1e-3 * area) << right << " " << first;
        }
    }
}

TEST(TrimmedFaceMesher, ATwistedPatchIsRefinedToTheTolerance) {
    const auto patch = makeTwistedPatch(10.0, 5.0);
    for (const double tol : {0.05, 0.01, 1e-3}) {
        MeshOptions options;
        options.chordTolerance = tol;
        const auto meshes = meshValid(*patch, options, 20.0);
        checkMeshes(*patch, meshes, options, slackFor(20.0), false);
        ASSERT_EQ(meshes.size(), 1u);
        EXPECT_GT(meshes[0].triangles.size(), 2u);  // the two a square would take are not enough
    }
    // Refinement that would need more triangles than the cap says so instead.
    MeshOptions options;
    options.chordTolerance = 1e-3;
    options.maxTrianglesPerFace = 50;
    const auto meshes = TrimmedFaceMesher::meshSolid(*patch, options);
    ASSERT_EQ(meshes.size(), 1u);
    EXPECT_FALSE(meshes[0].problem.empty());
    EXPECT_TRUE(meshes[0].triangles.empty());
}

TEST(TrimmedFaceMesher, AnEdgeWhosePcurveBowsIsSampledFiner) {
    // A triangle of a flat trapezoid, its third side the straight diagonal from (1,1,0) back to
    // the origin, which bows in (u, v): its (u, v) chord, mapped onto the face, strays from the
    // edge's chord, so the edge is halved until neither face's view of it does.
    const auto face = makeOpenFace(
        bilinear({0, 0, 0}, {2, 0, 0}, {0, 1, 0}, {1, 1, 0}),
        {{{line({0, 0}, {1, 0})}, {line({1, 0}, {1, 1})}, {std::make_shared<Bowed>(), 1.0, 0.0}}});
    MeshOptions options;
    options.chordTolerance = 0.01;
    const auto meshes = meshValid(*face, options, 3.0);
    const Summary s = checkMeshes(*face, meshes, options, slackFor(3.0), false);
    EXPECT_GT(s.boundarySegments, 3u);
    EXPECT_NEAR(s.area.at("open/face"), 1.0, 1e-12);
}

TEST(TrimmedFaceMesher, ATorusMeshesWatertightAcrossBothSeams) {
    const double major = 10.0;
    const double minor = 3.0;
    for (const Vec3& axis : {Vec3{0, 0, 1}, Vec3{1, -2, 0.5}}) {
        const auto torus = makeTorus(geo::Frame::fromAxis({4, -2, 7}, axis), major, minor);
        for (const double tol : {0.01, 0.1}) {
            MeshOptions options;
            options.chordTolerance = tol;
            const Summary s =
                checkMeshes(*torus, meshValid(*torus, options, 30.0), options, slackFor(30.0));
            const double area = 4.0 * math::kPi * math::kPi * major * minor;
            const double volume = 2.0 * math::kPi * math::kPi * major * minor * minor;
            EXPECT_NEAR(s.area.at("torus/face"), area, 0.01 * area);
            // Every point of the mesh is within the tolerance of the surface.
            EXPECT_NEAR(s.volume, volume, area * tol);
        }
    }
}

TEST(TrimmedFaceMesher, ACylinderMeshesWatertightToTheTolerance) {
    const double r = 5.0;
    const double h = 10.0;
    for (const Vec3& axis : {Vec3{0, 0, 1}, Vec3{1, 1, 1}, Vec3{-2, 1, 0.5}}) {
        const auto cyl = model::exact::makeCylinder(geo::Frame::fromAxis({1, 2, 3}, axis), r, h);
        ASSERT_NE(cyl, nullptr);
        for (const double tol : {1e-3 * r, 0.01, 0.2}) {
            MeshOptions options;
            options.chordTolerance = tol;
            const Summary s =
                checkMeshes(*cyl, meshValid(*cyl, options, 20.0), options, slackFor(20.0));
            if (tol == 1e-3 * r) {
                EXPECT_NEAR(s.area.at("cylinder/side"), 2.0 * math::kPi * r * h,
                            1e-3 * 2.0 * math::kPi * r * h);
            }
            EXPECT_NEAR(s.volume, math::kPi * r * r * h, totalArea(s) * tol);
        }
    }
}

TEST(TrimmedFaceMesher, APlateWithAHoleMeshesWatertight) {
    const auto plate = model::exact::makePlateWithHole({0, 0, 0}, {40, 30, 10}, {15, 12}, 5.0);
    ASSERT_NE(plate, nullptr);
    const double volume = 40.0 * 30.0 * 10.0 - math::kPi * 25.0 * 10.0;
    for (const double tol : {5e-3, 0.01, 0.5}) {
        MeshOptions options;
        options.chordTolerance = tol;
        const Summary s =
            checkMeshes(*plate, meshValid(*plate, options, 60.0), options, slackFor(60.0));
        EXPECT_NEAR(s.volume, volume, totalArea(s) * tol);
    }
    // Mirrored and turned copies: their walls and caps change handedness or orientation.
    for (const math::Mat4& m : {math::Mat4::scale({-1, 1, 1}),
                                math::Mat4::translation({3, -4, 5}) * math::Mat4::rotationZ(0.7)}) {
        const auto copy = model::Pattern::transformed(*plate, m);
        const MeshOptions options;
        const Summary s =
            checkMeshes(*copy, meshValid(*copy, options, 60.0), options, slackFor(60.0));
        EXPECT_NEAR(s.volume, volume, totalArea(s) * options.chordTolerance);
    }
}

TEST(TrimmedFaceMesher, ASliverFaceHasNoInvertedTriangles) {
    // A 100 x 0.01 x 50 block: its top and bottom are 100 x 0.01 slivers. With a hole of radius
    // 0.004 through them, 0.001 is left on either side of it.
    const auto block = model::exact::makeBlock({0, 0, 0}, {100, 0.01, 50});
    const auto plate =
        model::exact::makePlateWithHole({0, 0, 0}, {100, 0.01, 50}, {50, 0.005}, 0.004);
    ASSERT_NE(block, nullptr);
    ASSERT_NE(plate, nullptr);
    for (const double tol : {1e-5, 0.01}) {
        MeshOptions options;
        options.chordTolerance = tol;
        const Summary b =
            checkMeshes(*block, meshValid(*block, options, 120.0), options, slackFor(120.0));
        EXPECT_NEAR(b.volume, 100.0 * 0.01 * 50.0, 1e-9);
        const Summary p =
            checkMeshes(*plate, meshValid(*plate, options, 120.0), options, slackFor(120.0));
        const double hole = math::kPi * 0.004 * 0.004 * 50.0;
        EXPECT_NEAR(p.volume, 100.0 * 0.01 * 50.0 - hole, 0.2 * hole);
    }
}

TEST(TrimmedFaceMesher, TinyAndHugePartsMeshAlike) {
    std::vector<std::map<std::string, std::size_t>> counts;
    for (const double scale : {1.0, 1e-3, 1e4}) {
        const auto block = model::exact::makeBlock(Vec3{-1, 2, 3} * scale, Vec3{1, 5, 7} * scale);
        const auto cyl = model::exact::makeCylinder(
            geo::Frame::fromAxis(Vec3{1, 2, 3} * scale, {1, 1, 1}), 5.0 * scale, 10.0 * scale);
        const auto plate = model::exact::makePlateWithHole(Vec3{0, 0, 0}, Vec3{40, 30, 10} * scale,
                                                           Vec2{15, 12} * scale, 5.0 * scale);
        MeshOptions options;
        options.chordTolerance = 1e-3 * 5.0 * scale;
        const double slack = slackFor(60.0 * scale);
        std::map<std::string, std::size_t> c;
        const double unit = scale * scale * scale;

        const Summary sb =
            checkMeshes(*block, meshValid(*block, options, 60.0 * scale), options, slack);
        EXPECT_NEAR(sb.volume, 2.0 * 3.0 * 4.0 * unit, 1e-9 * unit);
        const Summary sc =
            checkMeshes(*cyl, meshValid(*cyl, options, 60.0 * scale), options, slack);
        EXPECT_NEAR(sc.area.at("cylinder/side"), 2.0 * math::kPi * 50.0 * scale * scale,
                    1e-3 * 2.0 * math::kPi * 50.0 * scale * scale);
        const Summary sp =
            checkMeshes(*plate, meshValid(*plate, options, 60.0 * scale), options, slack);
        const double plateVolume = (40.0 * 30.0 * 10.0 - math::kPi * 250.0) * unit;
        EXPECT_NEAR(sp.volume, plateVolume, 1e-3 * plateVolume);
        for (const Summary* s : {&sb, &sc, &sp}) {
            for (const auto& [tag, n] : s->triangles) c[tag] += n;
        }
        counts.push_back(c);
    }
    EXPECT_EQ(counts[1], counts[0]);
    EXPECT_EQ(counts[2], counts[0]);
}

TEST(TrimmedFaceMesher, AFaceWithoutExactGeometrySaysSoAndGetsNoTriangles) {
    auto cyl = model::exact::makeCylinder(geo::Frame::fromAxis({0, 0, 0}, {0, 0, 1}), 5.0, 10.0);
    for (auto& f : cyl->faces()) {
        if (f.topoId.tag() == "cylinder/side") f.exactSurface.reset();
    }
    const auto meshes = TrimmedFaceMesher::meshSolid(*cyl, MeshOptions{});
    ASSERT_EQ(meshes.size(), 3u);
    for (const FaceMesh& m : meshes) {
        if (m.face->topoId.tag() == "cylinder/side") {
            EXPECT_FALSE(m.problem.empty());
            EXPECT_TRUE(m.triangles.empty());
            EXPECT_TRUE(m.points.empty());
        } else {
            EXPECT_TRUE(m.problem.empty()) << m.problem;
            EXPECT_FALSE(m.triangles.empty());
        }
    }
}

TEST(TrimmedFaceMesher, AFaceThatNeedsMoreTrianglesThanTheCapSaysSo) {
    const auto cyl =
        model::exact::makeCylinder(geo::Frame::fromAxis({0, 0, 0}, {0, 0, 1}), 5.0, 10.0);
    MeshOptions options;
    options.chordTolerance = 5e-3;  // 100 segments round each rim
    options.maxTrianglesPerFace = 150;
    const auto meshes = TrimmedFaceMesher::meshSolid(*cyl, options);
    ASSERT_EQ(meshes.size(), 3u);
    for (const FaceMesh& m : meshes) {
        if (m.face->topoId.tag() == "cylinder/side") {  // two triangles a segment: over the cap
            EXPECT_FALSE(m.problem.empty());
            EXPECT_TRUE(m.triangles.empty());
        } else {  // two fewer triangles than its rim has points: under it
            EXPECT_TRUE(m.problem.empty()) << m.problem;
            EXPECT_LE(m.triangles.size(), options.maxTrianglesPerFace);
        }
    }
    // An edge that needs more segments than any face may have triangles fails both its faces.
    options.chordTolerance = 1e-7;
    for (const FaceMesh& m : TrimmedFaceMesher::meshSolid(*cyl, options)) {
        EXPECT_FALSE(m.problem.empty()) << m.face->topoId.tag();
        EXPECT_TRUE(m.triangles.empty());
    }
}

TEST(TrimmedFaceMesher, OptionsThatCannotBeMetAreReported) {
    const auto block = model::exact::makeBlock({0, 0, 0}, {1, 1, 1});
    for (const double tol : {0.0, -1.0, std::nan("")}) {
        MeshOptions options;
        options.chordTolerance = tol;
        const auto meshes = TrimmedFaceMesher::meshSolid(*block, options);
        EXPECT_EQ(meshes.size(), 6u);
        for (const FaceMesh& m : meshes) {
            EXPECT_FALSE(m.problem.empty());
            EXPECT_TRUE(m.triangles.empty());
        }
    }
}

TEST(TrimmedFaceMesher, ThePrimitivesMeshQuickly) {
    const auto block = model::exact::makeBlock({0, 0, 0}, {2, 3, 4});
    const auto cyl =
        model::exact::makeCylinder(geo::Frame::fromAxis({0, 0, 0}, {0, 0, 1}), 5.0, 10.0);
    const auto plate = model::exact::makePlateWithHole({0, 0, 0}, {40, 30, 10}, {15, 12}, 5.0);
    for (const auto* s : {block.get(), cyl.get(), plate.get()}) {
        const auto start = std::chrono::steady_clock::now();
        const auto meshes = TrimmedFaceMesher::meshSolid(*s, MeshOptions{});
        const std::chrono::duration<double, std::milli> took =
            std::chrono::steady_clock::now() - start;
        std::size_t triangles = 0;
        for (const FaceMesh& m : meshes) triangles += m.triangles.size();
        std::printf("[ timing   ] %zu faces, %zu triangles in %.2f ms\n", meshes.size(), triangles,
                    took.count());
        EXPECT_GT(triangles, 0u);
        EXPECT_LT(took.count(), 5000.0);  // generous: sanitizer builds run slowly
    }
}
