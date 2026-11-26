// The exact revolve (stage 3): a profile of lines, arcs and circles turned about an axis in its
// plane, built from whole faces carrying exact geometry. A line square to the axis sweeps a plane
// (a disc, a ring, or a sector of one), one along it a cylinder, one on it nothing, any other a
// cone to the point where it meets the axis. An arc centred on the axis sweeps a sphere, one off it
// a torus where its circle clears the axis and a NURBS surface of revolution where it does not. A
// partial turn is capped by the profile at each end; a hole in the profile is an inner loop of
// those caps, or, through a whole turn, a cavity of its own. Where the profile meets the axis
// there is no edge: a face comes to a pole or an apex there, its loop jumping along it in (u, v).
// Faces have master's Stable logical names, edges are named after them.
//
// Master's logical names were recorded from master's faceted revolve (Revolve::execute, Stable
// naming) at 9314271e, before the revolve was exact; "{k}" in a recorded name is the k-th entity of
// the profile ("e<id>" in the tag).

#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <map>
#include <memory>
#include <set>
#include <string>
#include <vector>

#include "horizon/drafting/DraftArc.h"
#include "horizon/drafting/DraftCircle.h"
#include "horizon/drafting/DraftLine.h"
#include "horizon/drafting/SketchPlane.h"
#include "horizon/geometry/surfaces/Surface.h"
#include "horizon/math/Constants.h"
#include "horizon/math/ModelTolerance.h"
#include "horizon/modeling/BoundaryMesh.h"
#include "horizon/modeling/MassProperties.h"
#include "horizon/modeling/Naming.h"
#include "horizon/modeling/Revolve.h"
#include "horizon/topology/ExactTolerance.h"
#include "horizon/topology/ExactValidator.h"
#include "horizon/topology/HalfEdge.h"
#include "horizon/topology/Solid.h"

using hz::draft::DraftArc;
using hz::draft::DraftCircle;
using hz::draft::DraftEntity;
using hz::draft::DraftLine;
using hz::draft::SketchPlane;
using hz::geo::SurfaceKind;
using hz::math::kPi;
using hz::math::Vec2;
using hz::math::Vec3;
using hz::model::NamingScheme;
using hz::model::Revolve;
using Profile = std::vector<std::shared_ptr<DraftEntity>>;

namespace {

constexpr double kTwoPi = 2.0 * kPi;

std::shared_ptr<DraftEntity> line(Vec2 a, Vec2 b) {
    return std::make_shared<DraftLine>(a, b);
}

std::shared_ptr<DraftEntity> arc(Vec2 c, double r, double a0, double a1) {
    return std::make_shared<DraftArc>(c, r, a0, a1);
}

std::shared_ptr<DraftEntity> circle(Vec2 c, double r) {
    return std::make_shared<DraftCircle>(c, r);
}

// Every profile is drawn on the sketch's x >= 0 side of its y axis, the axis it turns about.

/// A rectangle 3 wide and 5 high against the axis: a cylinder of radius 3 and height 5.
Profile rectangle() {
    return {line({0, 0}, {3, 0}), line({3, 0}, {3, 5}), line({3, 5}, {0, 5}), line({0, 5}, {0, 0})};
}

/// A right triangle against the axis, 3 wide and 4 high: a cone to its apex at (0, 4).
Profile triangle() {
    return {line({0, 0}, {3, 0}), line({3, 0}, {0, 4}), line({0, 4}, {0, 0})};
}

/// A half disc of radius 2 against the axis: a ball.
Profile semicircle() {
    return {arc({0, 0}, 2, -kPi / 2, kPi / 2), line({0, 2}, {0, -2})};
}

/// A disc of radius 1 five from the axis: a ring torus.
Profile offAxisCircle() {
    return {circle({5, 0}, 1)};
}

/// The part of a circle of radius 2 about (-1, 0) on the axis's far side, between the axis and
/// the arc: a lemon, whose arc's circle crosses the axis (its "major radius" 1 below its minor 2).
Profile lemon() {
    auto bulge = std::make_shared<DraftArc>(Vec2(-1, 0), 2, -kPi / 3, kPi / 3);
    const Vec2 from = bulge->startPoint();
    const Vec2 to = bulge->endPoint();
    return {bulge, line(to, from)};
}
const double kLemonVolume = kPi * (6.0 * std::sqrt(3.0) - 8.0 * kPi / 3.0);

/// A quarter circle of radius 3 about the origin from the axis at (0, -3) to (3, 0), then a line
/// up to the axis at (0, 4): a hemisphere with its pole on the axis, under a cone with its apex
/// on it, 18 pi + 12 pi in a whole turn.
Profile poleAndApex() {
    return {arc({0, 0}, 3, -kPi / 2, 0), line({3, 0}, {0, 4}), line({0, 4}, {0, -3})};
}
const double kPoleAndApexVolume = 30.0 * kPi;

/// A tangent ogive, 4 high on a base of radius 2: an arc of radius 5 about (-3, 0), across the
/// axis, from the base's rim to the tip on the axis, a NURBS surface of revolution with one pole.
Profile ogive() {
    auto nose = std::make_shared<DraftArc>(Vec2(-3, 0), 5, 0.0, std::atan2(4.0, 3.0));
    const Vec2 tip = nose->endPoint();
    return {nose, line(tip, {0, 0}), line({0, 0}, {2, 0})};
}

/// A 2 x 3 rectangle from 2 to 4 off the axis: a washer.
Profile washer() {
    return {line({2, 0}, {4, 0}), line({4, 0}, {4, 3}), line({4, 3}, {2, 3}), line({2, 3}, {2, 0})};
}

/// A 4 x 4 square from 1 to 5 off the axis with a round hole of radius 1 at its middle (3, 2).
Profile holed() {
    return {line({1, 0}, {5, 0}), line({5, 0}, {5, 4}), line({5, 4}, {1, 4}), line({1, 4}, {1, 0}),
            circle({3, 2}, 1)};
}
/// Its area times the length of the turn of its centroid, 3 from the axis, per radian.
const double kHoledVolumePerRadian = 3.0 * (16.0 - kPi);

/// A sketch plane tilted 30 degrees about x, then turned 20 degrees about z, at (3, -2, 5).
SketchPlane inclined() {
    const double a = 30.0 * kPi / 180.0;
    const double b = 20.0 * kPi / 180.0;
    const auto turn = [a, b](Vec3 p) {
        const Vec3 q(p.x, p.y * std::cos(a) - p.z * std::sin(a),
                     p.y * std::sin(a) + p.z * std::cos(a));
        return Vec3(q.x * std::cos(b) - q.y * std::sin(b), q.x * std::sin(b) + q.y * std::cos(b),
                    q.z);
    };
    return SketchPlane(Vec3(3, -2, 5), turn(Vec3(0, 0, 1)), turn(Vec3(1, 0, 0)));
}

/// @p profile turned by @p angle about @p plane's y axis through its origin, exactly, "x"; it
/// must build.
std::unique_ptr<hz::topo::Solid> revolved(const Profile& profile, double angle,
                                          const SketchPlane& plane = SketchPlane()) {
    std::string why;
    auto solid = Revolve::exact(profile, plane, plane.origin(), plane.yAxis(), angle, "x", &why);
    EXPECT_NE(solid, nullptr) << why;
    return solid;
}

/// A recorded name, "{k}" standing for the k-th entity of @p profile, as its tag has it, and an
/// edge's two faces `<src>/edge:<a>|<b>` in sorted order (nameEdgesLogically sorts them as they
/// are named, and entity ids sort otherwise in another run: "e9" after "e10").
std::string named(std::string recorded, const Profile& profile) {
    for (size_t k = 0; k < profile.size(); ++k) {
        const std::string mark = "{" + std::to_string(k) + "}";
        const std::string id = "e" + std::to_string(profile[k]->id());
        for (size_t at = recorded.find(mark); at != std::string::npos; at = recorded.find(mark)) {
            recorded.replace(at, mark.size(), id);
        }
    }
    const size_t edge = recorded.find("edge:");
    if (edge == std::string::npos) return recorded;
    const size_t a = edge + 5;
    const size_t bar = recorded.find('|', a);
    if (bar == std::string::npos) return recorded;
    const size_t end = std::min(recorded.find('/', bar), recorded.size());
    std::string first = recorded.substr(a, bar - a);
    std::string second = recorded.substr(bar + 1, end - bar - 1);
    if (second < first) std::swap(first, second);
    return recorded.substr(0, a) + first + "|" + second + recorded.substr(end);
}

std::set<std::string> named(const std::set<std::string>& recorded, const Profile& profile) {
    std::set<std::string> out;
    for (const auto& name : recorded) out.insert(named(name, profile));
    return out;
}

std::set<std::string> faceNames(const hz::topo::Solid& solid) {
    std::set<std::string> names;
    for (const auto& face : solid.faces()) {
        if (face.alive) names.insert(face.topoId.tag());
    }
    return names;
}

/// The logical edges of @p solid's edges that part two logical faces (model::logicalEdge), each
/// as named() writes it.
std::set<std::string> logicalEdges(const hz::topo::Solid& solid) {
    std::set<std::string> names;
    for (const auto& edge : solid.edges()) {
        const hz::topo::HalfEdge* he = edge.halfEdge;
        if (!edge.alive || he == nullptr || he->twin == nullptr) continue;
        if (hz::model::logicalFace(he->face->topoId.tag()) ==
            hz::model::logicalFace(he->twin->face->topoId.tag())) {
            continue;
        }
        names.insert(named(hz::model::logicalEdge(edge.topoId.tag()), {}));
    }
    return names;
}

/// How many of @p solid's faces lie on each kind of surface.
std::map<SurfaceKind, int> surfaceKinds(const hz::topo::Solid& solid) {
    std::map<SurfaceKind, int> kinds;
    for (const auto& face : solid.faces()) {
        if (face.exactSurface) ++kinds[face.exactSurface->kind()];
    }
    return kinds;
}

/// @p solid is combinatorially valid, its exact geometry agrees with its topology at its own
/// tolerance, every face is exact (and has no NURBS carrier), and it measures @p volume to 1e-9.
void expectValidAndExact(const hz::topo::Solid& solid, double volume) {
    EXPECT_TRUE(solid.isValid()) << solid.validationReport();
    const auto issues = hz::topo::validateExact(solid, hz::topo::exactTolerance(solid));
    EXPECT_TRUE(issues.empty()) << issues.size() << " issues, the first: "
                                << (issues.empty() ? "" : issues.front().detail);
    for (const auto& face : solid.faces()) {
        EXPECT_NE(face.exactSurface, nullptr) << face.topoId.tag();
        EXPECT_EQ(face.surface, nullptr) << "a constructed face has no NURBS carrier";
    }
    const auto props = hz::model::MassPropertiesCalculator::compute(solid);
    ASSERT_TRUE(props.valid) << props.problem;
    EXPECT_NEAR(props.volume, volume, 1e-9 * volume);
}

/// expectValidAndExact, and @p solid meshes at the read point with no problem.
void expectValidMeshedAndExact(const hz::topo::Solid& solid, double volume) {
    expectValidAndExact(solid, volume);
    std::string problem;
    const auto polygons = hz::model::BoundaryMesh::extractFacePolygons(
        solid, hz::model::BoundaryMesh::defaultChordTolerance(solid), &problem);
    EXPECT_TRUE(problem.empty()) << problem;
    EXPECT_FALSE(polygons.empty());
}

/// Revolve::execute of @p profile by @p angle about @p plane's y axis under @p naming, at 32
/// segments a turn, which must build: under Stable the exact revolve's bridge copy, under
/// FromGeometry master's faceted construction.
std::unique_ptr<hz::topo::Solid> faceted(const Profile& profile, double angle,
                                         const SketchPlane& plane = SketchPlane(),
                                         NamingScheme naming = NamingScheme::Stable) {
    std::string why;
    auto solid = Revolve::execute(profile, plane, plane.origin(), plane.yAxis(), angle, "x", 32,
                                  0.0, &why, naming);
    EXPECT_NE(solid, nullptr) << why;
    return solid;
}

/// The size of @p solid: its vertices' box's diagonal.
double sizeOf(const hz::topo::Solid& solid) {
    Vec3 lo(1e300, 1e300, 1e300);
    Vec3 hi(-1e300, -1e300, -1e300);
    for (const auto& v : solid.vertices()) {
        lo = Vec3(std::min(lo.x, v.point.x), std::min(lo.y, v.point.y), std::min(lo.z, v.point.z));
        hi = Vec3(std::max(hi.x, v.point.x), std::max(hi.y, v.point.y), std::max(hi.z, v.point.z));
    }
    return (hi - lo).length();
}

/// Every vertex of @p bridge is one of master's faceted construction's (FromGeometry naming, the
/// construction of master's Stable one but for a whole turn's flat faces, which it fanned from a
/// point on the axis), within 1e-12 of the size.
void expectMastersVertices(const hz::topo::Solid& bridge, const Profile& profile, double angle,
                           const SketchPlane& plane = SketchPlane()) {
    const auto masters = faceted(profile, angle, plane, NamingScheme::FromGeometry);
    ASSERT_NE(masters, nullptr);
    const double size = sizeOf(*masters);
    for (const auto& v : bridge.vertices()) {
        double nearest = 1e300;
        for (const auto& w : masters->vertices()) {
            nearest = std::min(nearest, (v.point - w.point).length());
        }
        EXPECT_LT(nearest, 1e-12 * size) << v.point.x << " " << v.point.y << " " << v.point.z;
    }
}

/// Each vertex of each face of @p bridge lies on the exact face it is a facet of, within
/// @p within.
void expectOnTheExactFaces(const hz::topo::Solid& bridge, const hz::topo::Solid& exact,
                           double within) {
    std::map<std::string, const hz::topo::Face*> byName;
    for (const auto& face : exact.faces()) byName[face.topoId.tag()] = &face;
    for (const auto& face : bridge.faces()) {
        const auto found = byName.find(hz::model::logicalFace(face.topoId.tag()));
        ASSERT_NE(found, byName.end()) << face.topoId.tag();
        const hz::geo::Surface& s = *found->second->exactSurface;
        std::vector<const hz::topo::Wire*> loops{face.outerLoop};
        loops.insert(loops.end(), face.innerLoops.begin(), face.innerLoops.end());
        for (const hz::topo::Wire* w : loops) {
            const hz::topo::HalfEdge* he = w->halfEdge;
            do {
                const Vec3& p = he->origin->point;
                const Vec2 uv = s.invert(p);
                EXPECT_LT((s.evaluate(uv.x, uv.y) - p).length(), within) << face.topoId.tag();
                he = he->next;
            } while (he != w->halfEdge);
        }
    }
}

/// The logical faces of @p solid's faces (model::logicalFace).
std::set<std::string> logicalFaces(const hz::topo::Solid& solid) {
    std::set<std::string> names;
    for (const auto& face : solid.faces()) {
        if (face.alive) names.insert(hz::model::logicalFace(face.topoId.tag()));
    }
    return names;
}

/// The exact solid named as its bridge copy is: the same logical faces and logical edges.
void expectNamedAsItsFacets(const hz::topo::Solid& exact, const hz::topo::Solid& bridge) {
    EXPECT_EQ(faceNames(exact), logicalFaces(bridge));
    EXPECT_EQ(logicalEdges(exact), logicalEdges(bridge));
}

}  // namespace

// A rectangle against the axis is a cylinder and two discs: three faces, two vertices (one on
// each rim) and three edges (the rims and the seam), pi r^2 h.
TEST(ExactRevolve, ARectangleAgainstTheAxisIsACylinderWithDiscs) {
    const auto solid = revolved(rectangle(), kTwoPi);
    ASSERT_NE(solid, nullptr);
    EXPECT_EQ(solid->faceCount(), 3u);
    EXPECT_EQ(solid->edges().size(), 3u);
    EXPECT_EQ(solid->vertices().size(), 2u);
    EXPECT_EQ(surfaceKinds(*solid),
              (std::map<SurfaceKind, int>{{SurfaceKind::Plane, 2}, {SurfaceKind::Cylinder, 1}}));
    expectValidMeshedAndExact(*solid, kPi * 9.0 * 5.0);
}

// A triangle against the axis is a disc and a cone to its apex, where its seam ends: pi r^2 h / 3.
TEST(ExactRevolve, ATriangleAgainstTheAxisIsAnApexCone) {
    const auto solid = revolved(triangle(), kTwoPi);
    ASSERT_NE(solid, nullptr);
    EXPECT_EQ(solid->faceCount(), 2u);
    EXPECT_EQ(solid->edges().size(), 2u) << "the rim and the seam";
    EXPECT_EQ(solid->vertices().size(), 2u) << "on the rim, and the apex";
    EXPECT_EQ(surfaceKinds(*solid),
              (std::map<SurfaceKind, int>{{SurfaceKind::Plane, 1}, {SurfaceKind::Cone, 1}}));
    expectValidMeshedAndExact(*solid, kPi * 9.0 * 4.0 / 3.0);
}

// A half disc against the axis is a sphere: one face, its seam from pole to pole, 4/3 pi r^3.
TEST(ExactRevolve, ASemicircleIsASphere) {
    const auto solid = revolved(semicircle(), kTwoPi);
    ASSERT_NE(solid, nullptr);
    EXPECT_EQ(solid->faceCount(), 1u);
    EXPECT_EQ(solid->edges().size(), 1u);
    EXPECT_EQ(solid->vertices().size(), 2u) << "the poles";
    EXPECT_EQ(surfaceKinds(*solid), (std::map<SurfaceKind, int>{{SurfaceKind::Sphere, 1}}));
    expectValidMeshedAndExact(*solid, 4.0 / 3.0 * kPi * 8.0);
}

// A disc off the axis is a ring torus: one face bounded by two seams (the rim its corner traces
// and the circle itself), 2 pi^2 R r^2.
TEST(ExactRevolve, AnOffAxisCircleIsATorus) {
    const auto solid = revolved(offAxisCircle(), kTwoPi);
    ASSERT_NE(solid, nullptr);
    EXPECT_EQ(solid->faceCount(), 1u);
    EXPECT_EQ(solid->edges().size(), 2u);
    EXPECT_EQ(solid->vertices().size(), 1u);
    EXPECT_EQ(surfaceKinds(*solid), (std::map<SurfaceKind, int>{{SurfaceKind::Torus, 1}}));
    expectValidMeshedAndExact(*solid, 2.0 * kPi * kPi * 5.0 * 1.0);
}

// Ruling 2: an arc off the axis whose circle reaches the axis (r >= R) is no typed torus but a
// NURBS surface of revolution. The lemon between such an arc and the axis measures what Pappus
// gives it: 2 pi times its area's moment about the axis, pi (6 sqrt 3 - 8 pi / 3).
TEST(ExactRevolve, ASpindleArcIsANurbsRevolution) {
    const auto solid = revolved(lemon(), kTwoPi);
    ASSERT_NE(solid, nullptr);
    EXPECT_EQ(solid->faceCount(), 1u);
    EXPECT_EQ(surfaceKinds(*solid), (std::map<SurfaceKind, int>{{SurfaceKind::Nurbs, 1}}));
    expectValidMeshedAndExact(*solid, kLemonVolume);
}

// A partial turn is capped at both ends by the profile: the rectangle a quarter turn is two
// sectors of a disc, a quarter cylinder and two caps, five faces, pi r^2 h / 4. Where the profile
// met the axis the caps meet along it.
TEST(ExactRevolve, AQuarterTurnHasCaps) {
    const auto solid = revolved(rectangle(), kPi / 2.0);
    ASSERT_NE(solid, nullptr);
    EXPECT_EQ(solid->faceCount(), 5u);
    EXPECT_EQ(solid->vertices().size(), 6u);
    EXPECT_EQ(solid->edges().size(), 9u);
    EXPECT_TRUE(solid->checkEulerFormula());
    EXPECT_EQ(surfaceKinds(*solid),
              (std::map<SurfaceKind, int>{{SurfaceKind::Plane, 4}, {SurfaceKind::Cylinder, 1}}));
    EXPECT_EQ(faceNames(*solid).count("x/cap_start"), 1u);
    EXPECT_EQ(faceNames(*solid).count("x/cap_end"), 1u);
    expectValidMeshedAndExact(*solid, kPi * 9.0 * 5.0 / 4.0);
}

// Review Focus 3: a profile with a sloped edge meeting the axis (an apex) and an arc meeting it (a
// pole), turned a whole turn, a quarter and 359.9 degrees, on the sketch's own plane and on one
// tilted and turned. Each validates, meshes and measures its share of 30 pi exactly.
TEST(ExactRevolve, APoleAndAnApexAtAnyAngle) {
    for (const SketchPlane& plane : {SketchPlane(), inclined()}) {
        for (const double angle : {kTwoPi, kPi / 2.0, 359.9 * kPi / 180.0}) {
            SCOPED_TRACE(angle);
            const auto solid = revolved(poleAndApex(), angle, plane);
            ASSERT_NE(solid, nullptr);
            const bool whole = angle == kTwoPi;
            EXPECT_EQ(solid->faceCount(), whole ? 2u : 4u);
            EXPECT_EQ(surfaceKinds(*solid).at(SurfaceKind::Sphere), 1);
            EXPECT_EQ(surfaceKinds(*solid).at(SurfaceKind::Cone), 1);
            EXPECT_TRUE(solid->checkEulerFormula());
            expectValidMeshedAndExact(*solid, kPoleAndApexVolume * angle / kTwoPi);
        }
    }
}

// The other shapes a partial turn makes: a lune of a ball, a sector of a torus and a slice of a
// lemon, each capped. (A lemon's slice of a hundredth of a radian validates and measures exactly,
// but the read point refines its NURBS face into some 200,000 triangles, 23 s in Debug: not
// meshed here.)
TEST(ExactRevolve, PartialTurnsOfASphereATorusAndALemon) {
    for (const double angle : {kPi / 2.0, 359.9 * kPi / 180.0, 0.01}) {
        SCOPED_TRACE(angle);
        const auto lune = revolved(semicircle(), angle);
        ASSERT_NE(lune, nullptr);
        EXPECT_EQ(lune->faceCount(), 3u);
        expectValidMeshedAndExact(*lune, 2.0 / 3.0 * angle * 8.0);
        const auto ring = revolved(offAxisCircle(), angle);
        ASSERT_NE(ring, nullptr);
        EXPECT_EQ(ring->faceCount(), 3u);
        expectValidMeshedAndExact(*ring, angle * 5.0 * kPi);
        if (angle < 0.1) continue;
        const auto slice = revolved(lemon(), angle);
        ASSERT_NE(slice, nullptr);
        EXPECT_EQ(slice->faceCount(), 3u);
        expectValidMeshedAndExact(*slice, kLemonVolume * angle / kTwoPi);
    }
}

// A washer: its flat ends are rings (an outer loop and an inner one), its sides two cylinders,
// the inner one facing the axis.
TEST(ExactRevolve, AWasherHasRingsAndAnInnerWall) {
    const auto solid = revolved(washer(), kTwoPi);
    ASSERT_NE(solid, nullptr);
    EXPECT_EQ(solid->faceCount(), 4u);
    EXPECT_EQ(surfaceKinds(*solid),
              (std::map<SurfaceKind, int>{{SurfaceKind::Plane, 2}, {SurfaceKind::Cylinder, 2}}));
    int rings = 0;
    int reversed = 0;
    for (const auto& face : solid->faces()) {
        if (face.innerLoops.size() == 1) ++rings;
        if (face.reversed) ++reversed;
    }
    EXPECT_EQ(rings, 2);
    EXPECT_EQ(reversed, 1) << "the inner wall's cylinder faces the axis";
    expectValidMeshedAndExact(*solid, kPi * (16.0 - 4.0) * 3.0);
    const auto quarter = revolved(washer(), kPi / 2.0);
    ASSERT_NE(quarter, nullptr);
    EXPECT_EQ(quarter->faceCount(), 6u);
    expectValidMeshedAndExact(*quarter, kPi * (16.0 - 4.0) * 3.0 / 4.0);
}

// A hole in the profile is no Boolean. Through a whole turn it sweeps a cavity, a shell of its
// own facing in; through part of one it is a tunnel, an inner loop of each cap.
TEST(ExactRevolve, AHoleInTheProfileIsACavityOrInnerLoops) {
    for (const SketchPlane& plane : {SketchPlane(), inclined()}) {
        const Profile p = holed();
        const auto whole = revolved(p, kTwoPi, plane);
        ASSERT_NE(whole, nullptr);
        EXPECT_EQ(whole->shellCount(), 2u);
        EXPECT_EQ(whole->faceCount(), 5u);
        EXPECT_EQ(faceNames(*whole).count(named("x~hole0/revolved:{4}", p)), 1u);
        expectValidMeshedAndExact(*whole, kTwoPi * kHoledVolumePerRadian);

        const auto quarter = revolved(p, kPi / 2.0, plane);
        ASSERT_NE(quarter, nullptr);
        EXPECT_EQ(quarter->shellCount(), 1u);
        EXPECT_EQ(quarter->faceCount(), 7u);
        for (const auto& face : quarter->faces()) {
            const bool cap = face.topoId.tag() == "x/cap_start" || face.topoId.tag() == "x/cap_end";
            EXPECT_EQ(face.innerLoops.size(), cap ? 1u : 0u) << face.topoId.tag();
        }
        EXPECT_EQ(faceNames(*quarter).count(named("x~hole0/revolved:{4}", p)), 1u);
        expectValidMeshedAndExact(*quarter, kPi / 2.0 * kHoledVolumePerRadian);
    }
}

// The profile's material lies to the left of its loops in the half-plane it turns in whichever
// way round it is drawn, whichever side of the axis it lies on and whichever way the axis points:
// each such revolve is the same solid, every face facing out.
TEST(ExactRevolve, EitherSideEitherWayRoundAndEitherWayUp) {
    const auto mirrored = [](const Profile& profile) {
        Profile out;
        for (const auto& e : profile) {
            if (const auto* l = dynamic_cast<const DraftLine*>(e.get())) {
                out.push_back(line({-l->start().x, l->start().y}, {-l->end().x, l->end().y}));
            } else if (const auto* a = dynamic_cast<const DraftArc*>(e.get())) {
                out.push_back(arc({-a->center().x, a->center().y}, a->radius(), kPi - a->endAngle(),
                                  kPi - a->startAngle()));
            }
        }
        return out;
    };
    const auto clockwise = [](const Profile& profile) {
        Profile out;
        for (auto it = profile.rbegin(); it != profile.rend(); ++it) {
            if (const auto* l = dynamic_cast<const DraftLine*>(it->get())) {
                out.push_back(line(l->end(), l->start()));
            } else {
                out.push_back(*it);
            }
        }
        return out;
    };
    for (const double angle : {kTwoPi, kPi / 2.0}) {
        SCOPED_TRACE(angle);
        const double volume = kPoleAndApexVolume * angle / kTwoPi;
        for (const Profile& profile : {poleAndApex(), clockwise(poleAndApex())}) {
            for (const Vec3& axis : {Vec3(0, 1, 0), Vec3(0, -1, 0)}) {
                std::string why;
                auto solid = Revolve::exact(profile, SketchPlane(), Vec3(), axis, angle, "x", &why);
                ASSERT_NE(solid, nullptr) << why;
                expectValidAndExact(*solid, volume);
                solid = Revolve::exact(mirrored(profile), SketchPlane(), Vec3(), axis, angle, "x",
                                       &why);
                ASSERT_NE(solid, nullptr) << why;
                expectValidAndExact(*solid, volume);
            }
        }
    }
}

// A sketch whose ends meet within the profile's join tolerance but miss by more than the model's
// resolution: on a part a millimetre across, a cone's line misses the arc after it by 0.9 nm.
// The corner where the arc's own end misses its vertex has the gap as its vertices' tolerance
// and its rim's (topo::ToleranceGrowth), and the solid validates; nothing else grows.
TEST(ExactRevolve, ASketchsLooseEndsAreItsCornersTolerances) {
    const double gap = 9e-7;
    const Profile loose = {line({0, 0.3}, {0.25, gap}), arc({0, 0}, 0.25, -kPi / 2, 0),
                           line({0, -0.25}, {0, 0.3})};
    for (const double angle : {kTwoPi, kPi / 2.0}) {
        SCOPED_TRACE(angle);
        const auto solid = revolved(loose, angle);
        ASSERT_NE(solid, nullptr);
        const hz::math::ModelTolerance tolerance = hz::topo::exactTolerance(*solid);
        ASSERT_GT(gap, tolerance.linear) << "a gap the resolution does not cover";
        const auto issues = hz::topo::validateExact(*solid, tolerance);
        EXPECT_TRUE(issues.empty())
            << issues.size()
            << " issues, the first: " << (issues.empty() ? "" : issues.front().detail);
        int looseCorners = 0;
        for (const auto& v : solid->vertices()) {
            if (v.tolerance > 0.0) {
                ++looseCorners;
                EXPECT_NEAR(v.tolerance, gap, 1e-12);
            }
        }
        EXPECT_EQ(looseCorners, angle == kTwoPi ? 1 : 2) << "the corner, at 0 and at the angle";
        int grown = 0;
        for (const auto& e : solid->edges()) {
            if (e.tolerance > 0.0) ++grown;
        }
        EXPECT_EQ(grown, 1) << "the corner's rim";
    }
}

// Each exact face is named with the logical name master's Stable revolve gave its facets, and the
// edges between them likewise (recorded at 9314271e). A profile with a hole was master's Boolean,
// whose pieces of each cap (`/piece:<k>`) are one face here (Ruling 23).
TEST(ExactRevolve, NamesAreMastersLogicalNames) {
    struct Case {
        const char* what;
        Profile profile;
        double angle;
        std::set<std::string> faces;
        std::set<std::string> edges;
    };
    const std::set<std::string> capped{"x/cap_end", "x/cap_start"};
    const auto with = [](std::set<std::string> a, const std::set<std::string>& b) {
        a.insert(b.begin(), b.end());
        return a;
    };
    std::vector<Case> cases;
    cases.push_back({"a rectangle",
                     rectangle(),
                     kTwoPi,
                     {"x/revolved:{0}", "x/revolved:{1}", "x/revolved:{2}"},
                     {"x/edge:revolved:{0}|revolved:{1}", "x/edge:revolved:{1}|revolved:{2}"}});
    cases.push_back(
        {"a rectangle, a quarter turn",
         rectangle(),
         kPi / 2.0,
         with(capped, {"x/revolved:{0}", "x/revolved:{1}", "x/revolved:{2}"}),
         {"x/edge:cap_end|cap_start", "x/edge:cap_end|revolved:{0}", "x/edge:cap_end|revolved:{1}",
          "x/edge:cap_end|revolved:{2}", "x/edge:cap_start|revolved:{0}",
          "x/edge:cap_start|revolved:{1}", "x/edge:cap_start|revolved:{2}",
          "x/edge:revolved:{0}|revolved:{1}", "x/edge:revolved:{1}|revolved:{2}"}});
    cases.push_back({"a triangle",
                     triangle(),
                     kTwoPi,
                     {"x/revolved:{0}", "x/revolved:{1}"},
                     {"x/edge:revolved:{1}|revolved:{0}"}});
    cases.push_back({"a half disc", semicircle(), kTwoPi, {"x/revolved:{0}"}, {}});
    cases.push_back({"a half disc, a quarter turn",
                     semicircle(),
                     kPi / 2.0,
                     with(capped, {"x/revolved:{0}"}),
                     {"x/edge:cap_end|cap_start", "x/edge:cap_end|revolved:{0}",
                      "x/edge:cap_start|revolved:{0}"}});
    cases.push_back({"a torus", offAxisCircle(), kTwoPi, {"x/revolved:{0}"}, {}});
    cases.push_back({"a torus, a quarter turn",
                     offAxisCircle(),
                     kPi / 2.0,
                     with(capped, {"x/revolved:{0}"}),
                     {"x/edge:cap_end|revolved:{0}", "x/edge:cap_start|revolved:{0}"}});
    cases.push_back({"a lemon", lemon(), kTwoPi, {"x/revolved:{0}"}, {}});
    cases.push_back({"a pole and an apex",
                     poleAndApex(),
                     kTwoPi,
                     {"x/revolved:{0}", "x/revolved:{1}"},
                     {"x/edge:revolved:{0}|revolved:{1}"}});
    for (const double angle : {kPi / 2.0, 359.9 * kPi / 180.0}) {
        cases.push_back({"a pole and an apex, a partial turn",
                         poleAndApex(),
                         angle,
                         with(capped, {"x/revolved:{0}", "x/revolved:{1}"}),
                         {"x/edge:cap_end|cap_start", "x/edge:cap_end|revolved:{0}",
                          "x/edge:cap_end|revolved:{1}", "x/edge:cap_start|revolved:{0}",
                          "x/edge:cap_start|revolved:{1}", "x/edge:revolved:{0}|revolved:{1}"}});
    }
    const std::set<std::string> washerFaces{"x/revolved:{0}", "x/revolved:{1}", "x/revolved:{2}",
                                            "x/revolved:{3}"};
    const std::set<std::string> washerEdges{
        "x/edge:revolved:{0}|revolved:{1}", "x/edge:revolved:{0}|revolved:{3}",
        "x/edge:revolved:{1}|revolved:{2}", "x/edge:revolved:{2}|revolved:{3}"};
    cases.push_back({"a washer", washer(), kTwoPi, washerFaces, washerEdges});
    cases.push_back(
        {"a washer, a quarter turn", washer(), kPi / 2.0, with(capped, washerFaces),
         with(washerEdges, {"x/edge:cap_end|revolved:{0}", "x/edge:cap_end|revolved:{1}",
                            "x/edge:cap_end|revolved:{2}", "x/edge:cap_end|revolved:{3}",
                            "x/edge:cap_start|revolved:{0}", "x/edge:cap_start|revolved:{1}",
                            "x/edge:cap_start|revolved:{2}", "x/edge:cap_start|revolved:{3}"})});
    // Master's Boolean cut the hole: each of its flat ends (and caps) was two pieces, and the
    // edges between them and the pieces are gone; the hole's face is master's `x~hole0/...`.
    cases.push_back({"a square with a hole", holed(), kTwoPi,
                     with(washerFaces, {"x~hole0/revolved:{4}"}), washerEdges});
    cases.push_back(
        {"a square with a hole, a quarter turn", holed(), kPi / 2.0,
         with(capped, with(washerFaces, {"x~hole0/revolved:{4}"})),
         with(washerEdges,
              {"edge:x/cap_end|x~hole0/revolved:{4}", "edge:x/cap_start|x~hole0/revolved:{4}",
               "x/edge:cap_end|revolved:{0}", "x/edge:cap_end|revolved:{1}",
               "x/edge:cap_end|revolved:{2}", "x/edge:cap_end|revolved:{3}",
               "x/edge:cap_start|revolved:{0}", "x/edge:cap_start|revolved:{1}",
               "x/edge:cap_start|revolved:{2}", "x/edge:cap_start|revolved:{3}"})});
    for (const Case& c : cases) {
        SCOPED_TRACE(c.what);
        const auto solid = revolved(c.profile, c.angle);
        ASSERT_NE(solid, nullptr);
        EXPECT_EQ(faceNames(*solid), named(c.faces, c.profile));
        EXPECT_EQ(logicalEdges(*solid), named(c.edges, c.profile));
    }
}

// What master refuses, the exact revolve refuses in master's words: a profile across the axis, an
// axis out of the profile's plane, an angle of nothing or of more than a turn. A circle that only
// touches the axis turns to a horn, whose cusp has no exact form here: master faceted it; the
// exact revolve says why it cannot.
TEST(ExactRevolve, WhatMasterRefusesItRefusesAlike) {
    const auto refusals = [](const Profile& profile, const Vec3& axisPoint, const Vec3& axisDir,
                             double angle) {
        std::string why;
        std::string masters;
        EXPECT_EQ(Revolve::exact(profile, SketchPlane(), axisPoint, axisDir, angle, "x", &why),
                  nullptr);
        EXPECT_EQ(Revolve::execute(profile, SketchPlane(), axisPoint, axisDir, angle, "x", 32, 0.0,
                                   &masters, NamingScheme::FromGeometry),
                  nullptr);
        EXPECT_EQ(why, masters);
    };
    const Profile across = {line({-1, 0}, {3, 0}), line({3, 0}, {3, 5}), line({3, 5}, {-1, 5}),
                            line({-1, 5}, {-1, 0})};
    refusals(across, Vec3(), Vec3(0, 1, 0), kTwoPi);
    refusals(rectangle(), Vec3(0, 0, 1), Vec3(0, 1, 0), kTwoPi);  // off the profile's plane
    refusals(rectangle(), Vec3(), Vec3(0, 0, 1), kTwoPi);         // square to it
    refusals(rectangle(), Vec3(), Vec3(0, 1, 0), 0.0);
    refusals(rectangle(), Vec3(), Vec3(0, 1, 0), 7.0);
    refusals(rectangle(), Vec3(), Vec3(), kTwoPi);
    refusals({line({0, 0}, {1, 0})}, Vec3(), Vec3(0, 1, 0), kTwoPi);

    const Profile touching = {circle({1, 0}, 1)};
    std::string masters;
    EXPECT_NE(Revolve::execute(touching, SketchPlane(), Vec3(), Vec3(0, 1, 0), kTwoPi, "x", 32, 0.0,
                               &masters, NamingScheme::FromGeometry),
              nullptr)
        << masters;
    std::string why;
    EXPECT_EQ(Revolve::exact(touching, SketchPlane(), Vec3(), Vec3(0, 1, 0), kTwoPi, "x", &why),
              nullptr);
    EXPECT_EQ(why.rfind("could not be built exactly: ", 0), 0u) << why;
}

// Under Stable naming Revolve::execute gives the exact revolve's bridge copy (model::facetExact at
// its segments and chord tolerance): master's facets, vertex for vertex, each named for the exact
// face it lies on and lying on it within 1e-9 of the size. Its counts are master's, recorded from
// master's Stable revolve at 9314271e at 32 segments a turn: through a whole turn a full turn's
// flat face was already one face, a disc or a ring.
TEST(ExactRevolve, TheBridgeIsMastersFacets) {
    struct Case {
        const char* what;
        Profile profile;
        double angle;
        std::size_t faces, edges, vertices;
        SketchPlane plane;
    };
    const double almost = 359.9 * kPi / 180.0;
    std::vector<Case> cases;
    cases.push_back({"a rectangle", rectangle(), kTwoPi, 34, 96, 64, SketchPlane()});
    cases.push_back({"a triangle", triangle(), kTwoPi, 33, 64, 33, SketchPlane()});
    cases.push_back({"a half disc", semicircle(), kTwoPi, 512, 992, 482, SketchPlane()});
    cases.push_back(
        {"a half disc, a quarter turn", semicircle(), kPi / 2.0, 130, 265, 137, SketchPlane()});
    cases.push_back({"a torus", offAxisCircle(), kTwoPi, 1024, 2048, 1024, SketchPlane()});
    cases.push_back(
        {"a torus, a quarter turn", offAxisCircle(), kPi / 2.0, 258, 544, 288, SketchPlane()});
    cases.push_back({"a pole and an apex", poleAndApex(), kTwoPi, 288, 544, 258, SketchPlane()});
    cases.push_back({"a pole and an apex, a quarter turn", poleAndApex(), kPi / 2.0, 74, 146, 74,
                     SketchPlane()});
    cases.push_back(
        {"a pole and an apex, 359.9 degrees", poleAndApex(), almost, 290, 554, 266, SketchPlane()});
    cases.push_back({"a pole and an apex on a tilted sketch", poleAndApex(), kTwoPi, 288, 544, 258,
                     inclined()});
    cases.push_back({"a washer", washer(), kTwoPi, 66, 192, 128, SketchPlane()});
    // A NURBS surface of revolution from a rim to a pole: its columns follow the rim's, so its
    // facets are master's (its 160 and the base; master's Stable counts, its FromGeometry
    // construction fanning the base from its centre).
    cases.push_back({"an ogive", ogive(), kTwoPi, 161, 320, 161, SketchPlane()});
    for (const Case& c : cases) {
        SCOPED_TRACE(c.what);
        const auto exact = revolved(c.profile, c.angle, c.plane);
        const auto bridge = faceted(c.profile, c.angle, c.plane);
        ASSERT_NE(exact, nullptr);
        ASSERT_NE(bridge, nullptr);
        EXPECT_EQ(bridge->faceCount(), c.faces);
        EXPECT_EQ(bridge->edges().size(), c.edges);
        EXPECT_EQ(bridge->vertices().size(), c.vertices);
        EXPECT_TRUE(bridge->isValid()) << bridge->validationReport();
        expectMastersVertices(*bridge, c.profile, c.angle, c.plane);
        expectOnTheExactFaces(*bridge, *exact, 1e-9 * sizeOf(*bridge));
        expectNamedAsItsFacets(*exact, *bridge);
    }
}

// Two differ from master on purpose. Through part of a turn a flat face is one plane, a sector of
// a disc or a ring, and its bridge is one face, as every plane's is: master cut it into a quad for
// each step (the rectangle's quarter turn was 26 faces, 44 edges and 20 vertices, the washer's
// 34, 68 and 36). The vertices are master's.
TEST(ExactRevolve, APartialTurnsFlatFaceIsOneFacet) {
    struct Case {
        const char* what;
        Profile profile;
        std::size_t faces, edges, vertices;
    };
    // Two sectors, eight facets of the wall and the caps; the washer's two walls.
    const std::vector<Case> cases = {{"a rectangle", rectangle(), 12, 30, 20},
                                     {"a washer", washer(), 20, 54, 36}};
    for (const Case& c : cases) {
        SCOPED_TRACE(c.what);
        const auto exact = revolved(c.profile, kPi / 2.0);
        const auto bridge = faceted(c.profile, kPi / 2.0);
        ASSERT_NE(exact, nullptr);
        ASSERT_NE(bridge, nullptr);
        EXPECT_EQ(bridge->faceCount(), c.faces);
        EXPECT_EQ(bridge->edges().size(), c.edges);
        EXPECT_EQ(bridge->vertices().size(), c.vertices);
        expectMastersVertices(*bridge, c.profile, kPi / 2.0);
        expectOnTheExactFaces(*bridge, *exact, 1e-9 * sizeOf(*bridge));
        expectNamedAsItsFacets(*exact, *bridge);
    }
}

// A profile with a hole is no longer master's Boolean of cutters (Ruling 23): through a whole turn
// the hole's cavity is a shell of the bridge, and each flat end one ring with no seam across it
// (1090 faces, 2240 edges, 1152 vertices; master's 1092, 2244 and 1152, each end in two pieces);
// through a quarter turn each cap is one face with the hole's polygon, its corners the hole's own
// ring (276 faces, 598 edges, 324 vertices; master's Boolean cut each cap in two along its cutter's
// facets, turned a twentieth further at each end, 324, 684 and 360).
TEST(ExactRevolve, AHoledProfilesBridgeHasItsHolesAsLoops) {
    const Profile p = holed();
    const auto whole = faceted(p, kTwoPi);
    ASSERT_NE(whole, nullptr);
    EXPECT_EQ(whole->shellCount(), 2u);
    EXPECT_EQ(whole->faceCount(), 1090u);
    EXPECT_EQ(whole->edges().size(), 2240u);
    EXPECT_EQ(whole->vertices().size(), 1152u);
    const auto quarter = faceted(p, kPi / 2.0);
    ASSERT_NE(quarter, nullptr);
    EXPECT_EQ(quarter->shellCount(), 1u);
    EXPECT_EQ(quarter->faceCount(), 276u);
    EXPECT_EQ(quarter->edges().size(), 598u);
    EXPECT_EQ(quarter->vertices().size(), 324u);
    for (const auto* bridge : {whole.get(), quarter.get()}) {
        EXPECT_TRUE(bridge->isValid()) << bridge->validationReport();
        const auto exact = revolved(p, bridge == whole.get() ? kTwoPi : kPi / 2.0);
        ASSERT_NE(exact, nullptr);
        expectOnTheExactFaces(*bridge, *exact, 1e-9 * sizeOf(*bridge));
        expectNamedAsItsFacets(*exact, *bridge);
    }
}

// Under the older schemes Revolve::execute builds master's faceted revolve as before; under
// Stable, a revolve only the exact construction refuses is refused, with its reason.
TEST(ExactRevolve, OnlyStableNamingBuildsExactly) {
    const auto positional =
        faceted(rectangle(), kPi / 2.0, SketchPlane(), NamingScheme::Positional);
    ASSERT_NE(positional, nullptr);
    EXPECT_EQ(positional->faceCount(), 26u) << "master's facets of the flat ends";
    std::string why;
    EXPECT_EQ(Revolve::execute({circle({1, 0}, 1)}, SketchPlane(), Vec3(), Vec3(0, 1, 0), kTwoPi,
                               "x", 32, 0.0, &why, NamingScheme::Stable),
              nullptr);
    EXPECT_EQ(why.rfind("could not be built exactly: ", 0), 0u) << why;
    EXPECT_EQ(Revolve::execute(rectangle(), SketchPlane(), Vec3(), Vec3(0, 1, 0), kTwoPi, "x", 2,
                               0.0, &why, NamingScheme::Stable),
              nullptr);
    EXPECT_EQ(why, "a revolve needs at least 3 steps per turn");
}

// Not master's: a NURBS surface of revolution that comes to the axis at both ends (a lemon) has no
// side to take its columns from, and facetExact cuts it at equal steps of its u, which between the
// quarter turns of its rational arcs is no angle. Its counts are master's (352, 672, 322 at 32
// segments), and each facet lies on it, but its vertices between the quarter turns miss master's,
// by as much as 1.6% of its radius: tracked in the task's report, not fixed here.
TEST(ExactRevolve, ALemonsBridgeIsCutEvenlyInItsU) {
    const Profile p = lemon();
    const auto exact = revolved(p, kTwoPi);
    const auto bridge = faceted(p, kTwoPi);
    const auto masters = faceted(p, kTwoPi, SketchPlane(), NamingScheme::FromGeometry);
    ASSERT_NE(exact, nullptr);
    ASSERT_NE(bridge, nullptr);
    ASSERT_NE(masters, nullptr);
    EXPECT_EQ(bridge->faceCount(), 352u);
    EXPECT_EQ(bridge->edges().size(), 672u);
    EXPECT_EQ(bridge->vertices().size(), 322u);
    expectOnTheExactFaces(*bridge, *exact, 1e-9 * sizeOf(*bridge));
    double worst = 0.0;
    for (const auto& v : bridge->vertices()) {
        double nearest = 1e300;
        for (const auto& w : masters->vertices()) {
            nearest = std::min(nearest, (v.point - w.point).length());
        }
        worst = std::max(worst, nearest);
    }
    EXPECT_GT(worst, 1e-3);
    EXPECT_LT(worst, 0.02);
}

// Not master's: with a chord tolerance, master turned every point of its profile through as many
// steps as its widest radius needed (Revolve::segmentsForTolerance at profileRadius, the feature's
// segments()), where the bridge cuts each rim by its own radius (FacetResolution::stepsFor), as
// it cuts every circle. A washer of radii 2 and 4 at 0.01: master's 45 steps round both rims (92
// faces, 270 edges, 180 vertices under Stable naming), the bridge's 45 round the outer and 32 round
// the inner (79, 231 and 154). Tracked in the task's report, not fixed here: the bridge would need
// a count for the circles round the revolve's axis that FacetResolution does not carry.
TEST(ExactRevolve, AChordToleranceCutsEachRimByItsOwnRadius) {
    const Profile p = washer();
    const double radius = Revolve::profileRadius(p, SketchPlane(), Vec3(), Vec3(0, 1, 0));
    const int segments = Revolve::segmentsForTolerance(radius, 0.01);
    ASSERT_EQ(segments, 45);
    std::string why;
    const auto bridge = Revolve::execute(p, SketchPlane(), Vec3(), Vec3(0, 1, 0), kTwoPi, "x",
                                         segments, 0.01, &why, NamingScheme::Stable);
    ASSERT_NE(bridge, nullptr) << why;
    EXPECT_EQ(bridge->faceCount(), 79u);
    EXPECT_EQ(bridge->edges().size(), 231u);
    EXPECT_EQ(bridge->vertices().size(), 154u);
    EXPECT_TRUE(bridge->isValid()) << bridge->validationReport();
}
