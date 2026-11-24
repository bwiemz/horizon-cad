#include <gtest/gtest.h>

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <functional>
#include <map>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "MeshEdgeSampler.h"  // the tolerance the mesher lays loops out with
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
#include "horizon/modeling/BoundaryMesh.h"
#include "horizon/modeling/ExactPrimitives.h"
#include "horizon/modeling/Pattern.h"
#include "horizon/modeling/TrimmedFaceMesher.h"
#include "horizon/topology/ExactTolerance.h"
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

/// The parameters of a point of a surface: the surface's own inversion unless a test knows
/// better.
using Inverse = std::function<Vec2(const geo::Surface&, const Vec3&)>;

/// @p uv moved by whole periods of @p s to lie nearest @p near: a triangle at the seam has
/// vertices that invert to both sides of it.
Vec2 uvNear(const geo::Surface& s, Vec2 uv, const Vec2& near) {
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
                    const MeshOptions& options, double slack, bool closed = true,
                    const Inverse& inverse = {}) {
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
        std::vector<Vec2> uvs;
        for (const Vec3& q : m.points) uvs.push_back(inverse ? inverse(s, q) : s.invert(q));

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
            const Vec2 ua = uvs[t[0]];
            const Vec2 ub = uvNear(s, uvs[t[1]], ua);
            const Vec2 uc = uvNear(s, uvs[t[2]], ua);
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
            EXPECT_LT((s.evaluate(uvs[i].x, uvs[i].y) - m.points[i]).length(), slack) << tag;
            EXPECT_NEAR(m.normals[i].dot(s.normal(uvs[i].x, uvs[i].y) * sign), 1.0, 1e-9) << tag;
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

/// The largest angle between the normals of two triangles that share an edge of a face's mesh.
double worstNeighbourAngle(const std::vector<FaceMesh>& meshes) {
    double worst = 0.0;
    for (const FaceMesh& m : meshes) {
        std::map<std::pair<std::uint32_t, std::uint32_t>, Vec3> first;
        for (const auto& t : m.triangles) {
            const Vec3 n = (m.points[t[1]] - m.points[t[0]]).cross(m.points[t[2]] - m.points[t[0]]);
            for (int k = 0; k < 3; ++k) {
                const auto key = std::minmax(t[k], t[(k + 1) % 3]);
                const auto [it, fresh] = first.emplace(key, n);
                if (!fresh) {
                    worst = std::max(worst,
                                     std::atan2(it->second.cross(n).length(), it->second.dot(n)));
                }
            }
        }
    }
    return worst;
}

/// The worst triangle of the meshes: its longest side over its height onto that side.
double worstAspect(const std::vector<FaceMesh>& meshes) {
    double worst = 0.0;
    for (const FaceMesh& m : meshes) {
        for (const auto& t : m.triangles) {
            const Vec3& a = m.points[t[0]];
            const Vec3& b = m.points[t[1]];
            const Vec3& c = m.points[t[2]];
            const double longest = std::max({(b - a).length(), (c - b).length(), (a - c).length()});
            const double twiceArea = (b - a).cross(c - a).length();
            worst = std::max(worst, longest * longest / twiceArea);
        }
    }
    return worst;
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
/// [curveStart, curveEnd]; without a curve, the edge is the line between the side's ends. Its
/// start vertex sits @c offset away from the surface's point.
struct Side {
    std::shared_ptr<const geo::Curve2D> pcurve{};
    double start = 0.0;
    double end = 1.0;
    std::shared_ptr<const geo::Curve3> curve{};
    double curveStart = 0.0;
    double curveEnd = 0.0;
    Vec3 offset{};
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
            v[i]->point = face->exactSurface->evaluate(uv.x, uv.y) + sides[i].offset;
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

/// From a to b as t^p runs from 0 to 1: a pcurve whose parameter is not its edge's.
class Warped : public geo::Curve2D {
public:
    Warped(const Vec2& a, const Vec2& b, double power) : a_(a), b_(b), p_(power) {}
    Vec2 evaluate(double t) const override { return a_ + (b_ - a_) * std::pow(t, p_); }
    Vec2 derivative(double t, int order) const override {
        if (order == 1) return (b_ - a_) * (p_ * std::pow(t, p_ - 1.0));
        if (order == 2) return (b_ - a_) * (p_ * (p_ - 1.0) * std::pow(t, p_ - 2.0));
        return {};
    }
    double tMin() const override { return 0.0; }
    double tMax() const override { return 1.0; }
    bool isClosed() const override { return false; }

private:
    Vec2 a_;
    Vec2 b_;
    double p_;
};

/// Three quarters of a cylinder, u in [0, 3 pi/2], v in [0, h], each side's pcurve warped by
/// t^power; the top and left sides run their pcurves backwards, from 1 down to 0.
std::unique_ptr<topo::Solid> makeWarpedBand(const geo::Frame& frame, double r, double h,
                                            double power) {
    const double end = 1.5 * math::kPi;
    const std::array<Vec2, 4> corner = {{{0, 0}, {end, 0}, {end, h}, {0, h}}};
    std::vector<Side> sides;
    for (std::size_t i = 0; i < 4; ++i) {
        const Vec2& a = corner[i];
        const Vec2& b = corner[(i + 1) % 4];
        const bool backwards = i >= 2;
        Side side{backwards ? std::make_shared<Warped>(b, a, power)
                            : std::make_shared<Warped>(a, b, power),
                  backwards ? 1.0 : 0.0, backwards ? 0.0 : 1.0};
        if (a.y == b.y) {  // an arc about the axis at this height
            geo::Frame at = frame;
            at.origin = frame.origin + frame.z * a.y;
            side.curve = std::make_shared<geo::CircleCurve>(at, r);
            side.curveStart = a.x;
            side.curveEnd = b.x;
        }
        sides.push_back(side);
    }
    return makeOpenFace(std::make_shared<geo::Cylinder>(frame, r), {sides});
}

/// A cubic B-spline over [0, 1] that is zero but for one basis function, on @p knots: the five
/// knots it spans, between 0 and 1 (clamped at both ends).
std::vector<double> clusteredKnots(const std::array<double, 5>& inner) {
    std::vector<double> knots{0, 0, 0, 0};
    knots.insert(knots.end(), inner.begin(), inner.end());
    knots.insert(knots.end(), {1, 1, 1, 1});
    return knots;
}

/// The flat square [0, L]^2 with one narrow feature where the fifth basis function of each
/// direction is non-zero: its control point raised by @p rise and pulled back along x by
/// @p pull (enough pull folds the surface over itself). Elsewhere x = L u, y = L v, z = 0.
std::shared_ptr<geo::Surface> featuredSquare(double size, const std::array<double, 5>& innerU,
                                             const std::array<double, 5>& innerV, double rise,
                                             double pull) {
    const std::vector<double> ku = clusteredKnots(innerU);
    const std::vector<double> kv = clusteredKnots(innerV);
    // Control points at the Greville abscissae reproduce x = L u and y = L v exactly.
    const auto greville = [](const std::vector<double>& k, std::size_t i) {
        return (k[i + 1] + k[i + 2] + k[i + 3]) / 3.0;
    };
    std::vector<std::vector<Vec3>> points(9, std::vector<Vec3>(9));
    for (std::size_t i = 0; i < 9; ++i) {
        for (std::size_t j = 0; j < 9; ++j) {
            points[i][j] = {size * greville(ku, i), size * greville(kv, j), 0.0};
        }
    }
    points[4][4] = points[4][4] + Vec3{-pull, 0.0, rise};
    return std::make_shared<geo::NurbsSurface>(
        points, std::vector<std::vector<double>>(9, std::vector<double>(9, 1.0)), ku, kv, 3, 3);
}

/// The parameters of a point of featuredSquare(size, ..., pull): v is y / L exactly, and u
/// lies within pull / L of x / L, where a fine search along it finds the point. The surface's
/// own inversion starts from a 16 x 16 grid and can miss a feature that narrow.
Inverse featureInverse(double size, double pull) {
    return [size, pull](const geo::Surface& s, const Vec3& p) {
        const double v = p.y / size;
        const auto dist = [&](double u) { return (s.evaluate(u, v) - p).length(); };
        const double reach = pull / size + 0.01;
        const double lo = std::max(0.0, p.x / size - reach);
        const double hi = std::min(1.0, p.x / size + reach);
        constexpr int kSteps = 400;
        double best = lo;
        for (int i = 0; i <= kSteps; ++i) {
            const double u = lo + (hi - lo) * i / kSteps;
            if (dist(u) < dist(best)) best = u;
        }
        double a = std::max(lo, best - (hi - lo) / kSteps);
        double b = std::min(hi, best + (hi - lo) / kSteps);
        for (int i = 0; i < 100; ++i) {  // golden section
            const double x1 = b - 0.618033988749895 * (b - a);
            const double x2 = a + 0.618033988749895 * (b - a);
            if (dist(x1) <= dist(x2)) {
                b = x2;
            } else {
                a = x1;
            }
        }
        return Vec2{0.5 * (a + b), v};
    };
}

/// The triangle (0,0), (1,0), (0,1) of @p surface's (u, v) as one open face of straight edges.
std::unique_ptr<topo::Solid> makeTriangleFace(std::shared_ptr<const geo::Surface> surface) {
    return makeOpenFace(std::move(surface),
                        {{{line({0, 0}, {1, 0})}, {line({1, 0}, {0, 1})}, {line({0, 1}, {0, 0})}}});
}

/// The image on @p surface of the straight (u, v) segment from a to b as t runs from 0 to 1:
/// the exact curve of an edge whose pcurve is that segment.
class OnSurface : public geo::Curve3 {
public:
    OnSurface(std::shared_ptr<const geo::Surface> surface, const Vec2& a, const Vec2& b)
        : s_(std::move(surface)), a_(a), b_(b) {}
    geo::CurveKind kind() const override { return geo::CurveKind::Nurbs; }
    Vec3 evaluate(double t) const override {
        const Vec2 q = a_ + (b_ - a_) * t;
        return s_->evaluate(q.x, q.y);
    }
    geo::CurveDerivatives derivatives(double t) const override {
        const Vec2 q = a_ + (b_ - a_) * t;
        const Vec2 d = b_ - a_;
        const auto sd = s_->derivatives(q.x, q.y);
        return {sd.point, sd.du * d.x + sd.dv * d.y,
                sd.duu * (d.x * d.x) + sd.duv * (2.0 * d.x * d.y) + sd.dvv * (d.y * d.y)};
    }
    geo::Interval range() const override { return {0.0, 1.0}; }
    bool periodic() const override { return false; }
    double closestParameter(const Vec3& p) const override {
        const auto dist = [&](double t) { return (evaluate(t) - p).length(); };
        constexpr int kSteps = 1000;
        double best = 0.0;
        for (int i = 1; i <= kSteps; ++i) {
            if (dist(static_cast<double>(i) / kSteps) < dist(best)) best = i / double(kSteps);
        }
        double lo = std::max(0.0, best - 1.0 / kSteps);
        double hi = std::min(1.0, best + 1.0 / kSteps);
        for (int i = 0; i < 100; ++i) {
            const double x1 = hi - 0.618033988749895 * (hi - lo);
            const double x2 = lo + 0.618033988749895 * (hi - lo);
            if (dist(x1) <= dist(x2)) {
                hi = x2;
            } else {
                lo = x1;
            }
        }
        return 0.5 * (lo + hi);
    }
    std::unique_ptr<geo::Curve3> transformed(const math::Mat4& /*m*/) const override {
        return nullptr;
    }

private:
    std::shared_ptr<const geo::Surface> s_;
    Vec2 a_;
    Vec2 b_;
};

/// z = rise u g(v) over x = L u, y = L v, with g a cubic B-spline bump from 0 at v = 0 up to 1
/// at v = 1/2 and back to 0 at v = 1. Along u = 0 the surface is the straight line x = 0, z = 0,
/// but its normal there leans over by atan(rise g / L) and back as v runs across the bump.
std::shared_ptr<geo::Surface> leaningSurface(double size, double rise) {
    const std::vector<double> kv{0, 0, 0, 0, 0.25, 0.5, 0.75, 1, 1, 1, 1};
    std::vector<std::vector<Vec3>> points(2, std::vector<Vec3>(7));
    for (std::size_t i = 0; i < 2; ++i) {
        for (std::size_t j = 0; j < 7; ++j) {
            const double greville = (kv[j + 1] + kv[j + 2] + kv[j + 3]) / 3.0;
            const double bump = j == 3 ? 1.5 : 0.0;  // 1.5 times the basis' peak of 2/3
            points[i][j] = {size * static_cast<double>(i), size * greville,
                            rise * static_cast<double>(i) * bump};
        }
    }
    return std::make_shared<geo::NurbsSurface>(
        points, std::vector<std::vector<double>>(2, std::vector<double>(7, 1.0)),
        std::vector<double>{0, 0, 1, 1}, kv, 1, 3);
}

}  // namespace

TEST(TrimmedFaceMesher, FilletLikeToriMeshWithinTheAngleAndTheCap) {
    // Thin tori at tolerances where the angle, not the chord, sets the steps: refinement beside
    // their boundary once ran away to the triangle cap (the first six cases) until the edges
    // themselves were halved and the steps took half the angle. The last two always meshed.
    const geo::Frame frame = geo::Frame::fromAxis({4, -2, 7}, {1, -2, 0.5});
    const struct {
        double major;
        double minor;
        double tol;
    } cases[] = {{10, 1, 0.08}, {10, 1, 0.3}, {10, 1, 0.5},  {10, 3, 0.3},
                 {50, 1, 0.3},  {50, 1, 0.5}, {10, 1, 0.02}, {50, 1, 0.05}};
    for (const auto& c : cases) {
        const auto torus = makeTorus(frame, c.major, c.minor);
        MeshOptions options;
        options.chordTolerance = c.tol;
        const auto meshes = meshValid(*torus, options, 2.0 * (c.major + c.minor));
        checkMeshes(*torus, meshes, options, slackFor(2.0 * (c.major + c.minor)));
        EXPECT_LE(worstNeighbourAngle(meshes), options.maxAngle + 1e-12);
        ASSERT_EQ(meshes.size(), 1u);
        // At the whole angle, steps left a fifth of the triangles turned and refining them
        // cascaded: R50 r1 at 0.3 took 44k. Half the angle keeps every case under 7.5k.
        EXPECT_LT(meshes[0].triangles.size(), 10000u)
            << "R" << c.major << " r" << c.minor << " at " << c.tol;
    }
}

TEST(TrimmedFaceMesher, ACurvedBoundaryIsHalvedWhereTheTrianglesBesideItTurn) {
    // A patch of a thin torus, u in [0.2, 1.4] and v in [0.5, 1.5], bounded by the images of
    // straight (u, v) segments. Its edges bend away from the u and v directions the grid is
    // spaced along, so triangles beside them turn by more than the angle; the faces ask for
    // those boundary segments to be halved, a few rounds over, and get no further without.
    const auto torus =
        std::make_shared<geo::Torus>(geo::Frame::fromAxis({4, -2, 7}, {0, 0, 1}), 10.0, 1.0);
    const std::array<Vec2, 4> corner = {{{0.2, 0.5}, {1.4, 0.5}, {1.4, 1.5}, {0.2, 1.5}}};
    std::vector<Side> sides;
    for (std::size_t i = 0; i < 4; ++i) {
        const Vec2& a = corner[i];
        const Vec2& b = corner[(i + 1) % 4];
        sides.push_back({line(a, b), 0.0, 1.0, std::make_shared<OnSurface>(torus, a, b), 0.0, 1.0});
    }
    const auto patch = makeOpenFace(torus, {sides});
    MeshOptions options;
    options.chordTolerance = 0.1;
    options.maxAngle = 0.2;
    const auto meshes = meshValid(*patch, options, 30.0);
    checkMeshes(*patch, meshes, options, slackFor(30.0), false);
    EXPECT_LE(worstNeighbourAngle(meshes), options.maxAngle + 1e-12);
}

TEST(TrimmedFaceMesher, APartWhoseVerticesCoincideTakesItsToleranceFromItsSize) {
    // A torus as one face has a single vertex: the box of its vertices is a point, and
    // forExtent(0) is 1e-7 whatever the part's size. The tolerance sets joinAllowance, which
    // decides whether a jump near a pole is a wrap, so the mesher must take the part's size
    // from its edges too, as the read point and the validator's callers do (Ruling 24).
    const double major = 10.0;
    const double minor = 1.0;
    const auto torus = makeTorus(geo::Frame::fromAxis({0, 0, 0}, {0, 0, 1}), major, minor);
    // The box 2 (R + r) by 2 (R + r) by 2 r: the equator's and the meridian's extremes are
    // among their samples.
    const double size = std::sqrt(8.0 * (major + minor) * (major + minor) + 4.0 * minor * minor);
    const double want = math::ModelTolerance::forExtent(size).linear;
    const MeshOptions options;
    model::detail::EdgeSampler sampler(*torus, options);
    sampler.sampleEdges();
    EXPECT_NEAR(sampler.tolerance().linear, want, want * 1e-12);
    EXPECT_GT(sampler.tolerance().linear, 10.0 * math::ModelTolerance::forExtent(0.0).linear);
    // One size for all: the mesher's tolerance is the solid's, and the read point meshes at
    // 1e-4 of the size that tolerance comes from.
    EXPECT_EQ(sampler.tolerance().linear, topo::exactTolerance(*torus).linear);
    EXPECT_NEAR(topo::exactExtent(*torus), size, size * 1e-12);
    std::string problem;
    const auto polys = model::BoundaryMesh::extractFacePolygons(*torus);
    EXPECT_EQ(polys.size(), model::BoundaryMesh::extractFacePolygons(
                                *torus, 1e-4 * topo::exactExtent(*torus), &problem)
                                .size());
    EXPECT_TRUE(problem.empty()) << problem;
}

TEST(TrimmedFaceMesher, ARhombusOnASaddleHasItsBoundaryHalved) {
    // A small rhombus on z = 100 u v over 10 x 10, with a loose chord tolerance: the angle sets
    // the steps. Its sides run diagonally across the saddle's bend, so triangles beside them turn
    // by more than the angle, and a turned triangle's longest side is the boundary chord: the
    // face asks for those segments to be halved (for both faces of the edge; here the face has
    // no other) and is meshed again from the finer boundary (Ruling 20).
    const auto surface = bilinear({0, 0, 0}, {10, 0, 0}, {0, 10, 0}, {10, 10, 100});
    const std::array<Vec2, 4> corner = {{{0.5, 0.43}, {0.7, 0.5}, {0.5, 0.57}, {0.3, 0.5}}};
    std::vector<Side> sides;
    for (std::size_t i = 0; i < 4; ++i) {
        const Vec2& a = corner[i];
        const Vec2& b = corner[(i + 1) % 4];
        sides.push_back(
            {line(a, b), 0.0, 1.0, std::make_shared<OnSurface>(surface, a, b), 0.0, 1.0});
    }
    const auto face = makeOpenFace(surface, {sides});
    MeshOptions options;
    options.maxAngle = 0.1;
    options.chordTolerance = 1.0;
    options.maxTrianglesPerFace = 2000;

    // The edges' samples before any face asks for more.
    model::detail::EdgeSampler first(*face, options);
    first.sampleEdges();
    std::size_t firstSegments = 0;
    for (const auto& e : face->edges()) {
        const auto* samples = first.samples(&e);
        ASSERT_NE(samples, nullptr);
        ASSERT_TRUE(samples->problem.empty()) << samples->problem;
        firstSegments += samples->points.size() - 1;
    }

    const double size = topo::exactExtent(*face);
    const auto meshes = meshValid(*face, options, size);
    ASSERT_EQ(meshes.size(), 1u);
    ASSERT_TRUE(meshes[0].problem.empty()) << meshes[0].problem;
    checkMeshes(*face, meshes, options, slackFor(size), false);
    EXPECT_LE(worstNeighbourAngle(meshes), options.maxAngle + 1e-12);
    // Every segment of an open face's mesh boundary is a segment of an edge's samples: more of
    // them than the edges were first sampled into is a round of halving.
    std::map<std::pair<std::uint32_t, std::uint32_t>, int> directed;
    for (const auto& t : meshes[0].triangles) {
        for (int k = 0; k < 3; ++k) ++directed[{t[k], t[(k + 1) % 3]}];
    }
    std::size_t boundarySegments = 0;
    for (const auto& [e, uses] : directed) {
        if (directed.count({e.second, e.first}) == 0) ++boundarySegments;
    }
    EXPECT_GT(boundarySegments, firstSegments)
        << "no round of halving; " << meshes[0].triangles.size() << " triangles";
}

TEST(TrimmedFaceMesher, ACylinderUnderASmallTriangleCapStillMeshes) {
    // Rows across the side are bounded by the cap: with as many as their spacing wants, the
    // grid alone would pass it and the face would fail.
    const auto cyl =
        model::exact::makeCylinder(geo::Frame::fromAxis({0, 0, 0}, {0, 0, 1}), 5.0, 10.0);
    const struct {
        std::size_t cap;
        double tol;
    } cases[] = {{10000, 5e-5}, {5000, 5e-5}, {2000, 5e-4}, {2000, 5e-5}};
    for (const auto& c : cases) {
        MeshOptions options;
        options.chordTolerance = c.tol;
        options.maxTrianglesPerFace = c.cap;
        const auto meshes = meshValid(*cyl, options, 20.0);
        checkMeshes(*cyl, meshes, options, slackFor(20.0));
        for (const FaceMesh& m : meshes) {
            EXPECT_LE(m.triangles.size(), c.cap) << m.face->topoId.tag() << " cap " << c.cap;
        }
    }
}

TEST(TrimmedFaceMesher, AnEdgeWhoseNormalLeansOverAndBackIsSplit) {
    // The strip u in [0, 0.05] of leaningSurface: its side along u = 0 is a straight line whose
    // ends face the same way, but whose middle leans by 0.46. Only measuring the turn through
    // the middle splits it; the triangles beside it turn by less than the angle of 0.6 against
    // one another, so the faces would never ask.
    const double size = 10.0;
    const double width = 0.05;
    const auto surface = leaningSurface(size, 5.0);
    const auto face = makeOpenFace(
        surface, {{{line({0, 0}, {width, 0})},
                   {line({width, 0}, {width, 1}), 0.0, 1.0,
                    std::make_shared<OnSurface>(surface, Vec2{width, 0}, Vec2{width, 1}), 0.0, 1.0},
                   {line({width, 1}, {0, 1})},
                   {line({0, 1}, {0, 0})}}});
    MeshOptions options;
    options.chordTolerance = 0.01;
    options.maxAngle = 0.6;
    const auto meshes = meshValid(*face, options, 20.0);
    const Inverse exact = [size](const geo::Surface&, const Vec3& p) {
        return Vec2{p.x / size, p.y / size};
    };
    checkMeshes(*face, meshes, options, slackFor(20.0), false, exact);
    ASSERT_EQ(meshes.size(), 1u);
    const FaceMesh& m = meshes[0];
    std::map<std::pair<std::uint32_t, std::uint32_t>, int> directed;
    for (const auto& t : m.triangles) {
        for (int k = 0; k < 3; ++k) ++directed[{t[k], t[(k + 1) % 3]}];
    }
    std::size_t along = 0;
    for (const auto& [e, uses] : directed) {
        const Vec3& a = m.points[e.first];
        const Vec3& b = m.points[e.second];
        if (directed.count({e.second, e.first}) != 0 || a.x != 0.0 || b.x != 0.0) continue;
        ++along;
        const Vec3 na = surface->normal(0.0, a.y / size);
        const Vec3 nm = surface->normal(0.0, 0.5 * (a.y + b.y) / size);
        const Vec3 nb = surface->normal(0.0, b.y / size);
        const double turn = std::atan2(na.cross(nm).length(), na.dot(nm)) +
                            std::atan2(nm.cross(nb).length(), nm.dot(nb));
        EXPECT_LE(turn, options.maxAngle + 1e-9) << "segment from y " << a.y << " to " << b.y;
    }
    EXPECT_GT(along, 1u);
}

TEST(TrimmedFaceMesher, PcurvesThatDoNotShareTheirEdgesParametersStillMesh) {
    // Each boundary sample's (u, v) must be found on its pcurve by search: t^p is not the
    // edge's parameter, and the top and left sides run their pcurves backwards.
    const geo::Frame frame = geo::Frame::fromAxis({1, 2, 3}, {0.3, 1, 0.2});
    for (const double power : {0.5, 2.0, 3.0}) {
        const auto band = makeWarpedBand(frame, 5.0, 10.0, power);
        for (const double tol : {1e-3, 0.05}) {
            MeshOptions options;
            options.chordTolerance = tol;
            const Summary s =
                checkMeshes(*band, meshValid(*band, options, 30.0), options, slackFor(30.0), false);
            const double area = 1.5 * math::kPi * 5.0 * 10.0;
            EXPECT_NEAR(s.area.at("open/face"), area, 2e-3 * area) << power << " " << tol;
        }
    }
}

TEST(TrimmedFaceMesher, NeighbouringTrianglesTurnByNoMoreThanTheAngle) {
    // The twisted patch's straight edges carry no sag at all, yet its normal turns by nearly
    // half a radian along them: the angle alone must split them.
    const auto patch = makeTwistedPatch(10.0, 5.0);
    for (const double angle : {0.35, 0.1, 0.05}) {
        for (const double tol : {0.01, 1e-3}) {
            MeshOptions options;
            options.chordTolerance = tol;
            options.maxAngle = angle;
            const auto meshes = meshValid(*patch, options, 20.0);
            checkMeshes(*patch, meshes, options, slackFor(20.0), false);
            EXPECT_LE(worstNeighbourAngle(meshes), angle + 1e-12) << angle << " " << tol;
        }
    }
    const auto torus = makeTorus(geo::Frame::fromAxis({4, -2, 7}, {1, -2, 0.5}), 10.0, 3.0);
    for (const double angle : {0.35, 0.1}) {
        MeshOptions options;
        options.chordTolerance = 0.05;
        options.maxAngle = angle;
        const auto meshes = meshValid(*torus, options, 30.0);
        checkMeshes(*torus, meshes, options, slackFor(30.0));
        EXPECT_LE(worstNeighbourAngle(meshes), angle + 1e-12) << angle;
    }
}

TEST(TrimmedFaceMesher, AFeatureOnlyATrianglesCentroidSeesIsFound) {
    // A bump 0.4 across at the centroid of a triangle 10 a side: its sides and their midpoints
    // are flat, so only the centroid shows the face is not.
    const double third = 1.0 / 3.0;
    const std::array<double, 5> around = {third - 0.02, third - 0.01, third, third + 0.01,
                                          third + 0.02};
    const auto face = makeTriangleFace(featuredSquare(10.0, around, around, 0.1, 0.0));
    MeshOptions options;
    options.chordTolerance = 0.01;
    const auto meshes = meshValid(*face, options, 20.0);
    checkMeshes(*face, meshes, options, slackFor(20.0), false, featureInverse(10.0, 0.0));
    ASSERT_EQ(meshes.size(), 1u);
    EXPECT_GT(meshes[0].triangles.size(), 1u);
}

TEST(TrimmedFaceMesher, NoTriangleFacesAgainstTheSurface) {
    // A fold at the centroid of a triangle 10 a side: there the surface turns back over itself
    // and faces down, while a triangle through the three corners faces up. With a tolerance too
    // loose to see the fold and no limit on the angle, only the triangles' orientation does.
    const double third = 1.0 / 3.0;
    const std::array<double, 5> rising = {third - 0.04 / 3.0, third - 0.01 / 3.0,
                                          third + 0.02 / 3.0, third + 0.05 / 3.0,
                                          third + 0.08 / 3.0};
    const std::array<double, 5> around = {third - 0.02, third - 0.01, third, third + 0.01,
                                          third + 0.02};
    const auto face = makeTriangleFace(featuredSquare(10.0, rising, around, 0.1, 0.5));
    MeshOptions options;
    options.chordTolerance = 1.0;
    options.maxAngle = math::kPi;
    const auto meshes = TrimmedFaceMesher::meshSolid(*face, options);
    checkMeshes(*face, meshes, options, slackFor(20.0), false, featureInverse(10.0, 0.5));
}

TEST(TrimmedFaceMesher, ACylindersSideAtATightToleranceHasNoNeedles) {
    // Bent one way only, the side gets rows across it (32 bent steps apart) as well as the
    // columns its bending needs, and its seam is sampled at the rows: two rows of 994 points
    // each would make triangles 10 long and 0.03 wide.
    const double r = 5.0;
    const auto cyl =
        model::exact::makeCylinder(geo::Frame::fromAxis({1, 2, 3}, {1, 1, 1}), r, 10.0);
    MeshOptions options;
    options.chordTolerance = 1e-5 * r;
    const auto start = std::chrono::steady_clock::now();
    const auto meshes = meshValid(*cyl, options, 20.0);
    const std::chrono::duration<double, std::milli> took = std::chrono::steady_clock::now() - start;
    std::size_t triangles = 0;
    for (const FaceMesh& m : meshes) triangles += m.triangles.size();
    std::printf("[ timing   ] cylinder at 1e-5 r: %zu triangles in %.0f ms\n", triangles,
                took.count());
    checkMeshes(*cyl, meshes, options, slackFor(20.0));
    // The side: its caps are planes, triangulated from their rims alone.
    for (const FaceMesh& m : meshes) {
        if (m.face->topoId.tag() == "cylinder/side") {
            EXPECT_LE(worstAspect({m}), 40.0);
        }
    }
}

TEST(TrimmedFaceMesher, BoundaryPointsThatMeetInUvAtDifferentPositionsSaySo) {
    // A loop pinched at (1, 1) by two vertices, one lifted 1e-3 off the plane: the triangles
    // there could keep only one of the two positions, so the face says so instead.
    const auto face =
        makeOpenFace(std::make_shared<geo::Plane>(geo::Frame::fromAxis({0, 0, 0}, {0, 0, 1})),
                     {{{line({0, 0}, {2, 0})},
                       {line({2, 0}, {1, 1})},
                       {line({1, 1}, {2, 2})},
                       {line({2, 2}, {0, 2})},
                       {line({0, 2}, {1, 1})},
                       {line({1, 1}, {0, 0}), 0.0, 1.0, nullptr, 0.0, 0.0, Vec3{0, 0, 1e-3}}}});
    const auto meshes = TrimmedFaceMesher::meshSolid(*face, MeshOptions{});
    ASSERT_EQ(meshes.size(), 1u);
    EXPECT_FALSE(meshes[0].problem.empty());
    EXPECT_TRUE(meshes[0].triangles.empty());
}

TEST(TrimmedFaceMesher, AnEdgeWithNoTwinSaysSo) {
    // A copy of an open sheet keeps no half-edges without a face: its edges have no twins.
    const auto copy = model::Pattern::transformed(*makeTwistedPatch(10.0, 5.0),
                                                  math::Mat4::translation({1, 2, 3}));
    const auto meshes = TrimmedFaceMesher::meshSolid(*copy, MeshOptions{});
    ASSERT_EQ(meshes.size(), 1u);
    EXPECT_NE(meshes[0].problem.find("no twin"), std::string::npos) << meshes[0].problem;
    EXPECT_TRUE(meshes[0].triangles.empty());
}

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
    // Scales by powers of two (about 1e-3 and 1e4) scale every input exactly, so each decision
    // is taken alike and the counts must be equal; at 1e-3 a decision near a threshold could
    // flip with a platform's libm.
    std::vector<std::map<std::string, std::size_t>> counts;
    for (const double scale : {1.0, std::ldexp(1.0, -10), std::ldexp(1.0, 13)}) {
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
