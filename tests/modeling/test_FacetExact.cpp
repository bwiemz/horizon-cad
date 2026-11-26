// The bridge from exact faces (stage 3): facetExact cuts a solid of exact faces, which carry no
// NURBS carrier, into the planar facets every operation not yet converted reads, at a feature's
// own resolution, so that a part built exactly has master's faceted counts, and the names on its
// facets and chords are its exact faces' and edges'. A curved face with no surface at all is
// refused, by facetCurved too, never outlined by its corners; and a body of exact shells with a
// void is told apart by its shells' exact signed volumes.

#include <gtest/gtest.h>

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <functional>
#include <map>
#include <memory>
#include <numbers>
#include <set>
#include <string>
#include <utility>
#include <vector>

#include "../TimeLimits.h"
#include "StepTestText.h"
#include "horizon/fileio/StepFormat.h"
#include "horizon/geometry/Frame.h"
#include "horizon/geometry/curves/Curve3.h"
#include "horizon/geometry/surfaces/NurbsSurface.h"
#include "horizon/geometry/surfaces/Surface.h"
#include "horizon/math/Mat4.h"
#include "horizon/math/Units.h"
#include "horizon/math/Vec2.h"
#include "horizon/math/Vec3.h"
#include "horizon/modeling/ExactPrimitives.h"
#include "horizon/modeling/FacetResolution.h"
#include "horizon/modeling/Faceting.h"
#include "horizon/modeling/MassProperties.h"
#include "horizon/modeling/Naming.h"
#include "horizon/modeling/Pattern.h"
#include "horizon/modeling/PrimitiveFactory.h"
#include "horizon/topology/HalfEdge.h"
#include "horizon/topology/Queries.h"
#include "horizon/topology/Solid.h"

namespace {

namespace geo = hz::geo;
namespace topo = hz::topo;
namespace exact = hz::model::exact;
using hz::math::Mat4;
using hz::math::Vec3;
using hz::model::facetCurved;
using hz::model::facetExact;
using hz::model::FacetResolution;
using hz::model::logicalEdge;
using hz::model::logicalFace;
using hz::model::MassPropertiesCalculator;
using hz::model::nameEdgesLogically;
using hz::model::Pattern;

constexpr double kPi = std::numbers::pi;

/// The two faces' tags an edge of @p e parts.
std::pair<std::string, std::string> facesOf(const topo::Edge& e) {
    return {e.halfEdge->face->topoId.tag(), e.halfEdge->twin->face->topoId.tag()};
}

/// Every half-edge of @p solid, by its faces' loops.
std::vector<topo::HalfEdge*> halfEdgesOf(topo::Solid& solid) {
    std::vector<topo::HalfEdge*> out;
    for (auto& f : solid.faces()) {
        std::vector<topo::Wire*> loops{f.outerLoop};
        loops.insert(loops.end(), f.innerLoops.begin(), f.innerLoops.end());
        for (topo::Wire* w : loops) {
            topo::HalfEdge* h = w->halfEdge;
            do {
                out.push_back(h);
                h = h->next;
            } while (h != w->halfEdge);
        }
    }
    return out;
}

/// @p solid turned inside out: every loop run the other way, every face's sense with it, so its
/// faces face into what it encloses, as a void's do.
void turnInsideOut(topo::Solid& solid) {
    const std::vector<topo::HalfEdge*> hes = halfEdgesOf(solid);
    // Each vertex leaves by its outgoing half-edge's twin, which will leave it.
    for (auto& v : solid.vertices()) {
        if (v.halfEdge != nullptr && v.halfEdge->twin != nullptr) v.halfEdge = v.halfEdge->twin;
    }
    std::vector<topo::Vertex*> ends;
    ends.reserve(hes.size());
    for (topo::HalfEdge* h : hes) ends.push_back(h->next->origin);
    for (std::size_t i = 0; i < hes.size(); ++i) {
        topo::HalfEdge* h = hes[i];
        h->origin = ends[i];
        std::swap(h->next, h->prev);
        std::swap(h->pcurveStart, h->pcurveEnd);
    }
    // Each edge's curve now runs from its half-edge's new origin, the old end.
    for (auto& e : solid.edges()) std::swap(e.curveStart, e.curveEnd);
    for (auto& f : solid.faces()) f.reversed = !f.reversed;
}

/// Whether some point of @p points lies within @p tol of @p p.
bool hasPointNear(const std::vector<Vec3>& points, const Vec3& p, double tol) {
    for (const Vec3& q : points) {
        if ((q - p).length() <= tol) return true;
    }
    return false;
}

// Master's faceted cylinder at 32 segments, built exactly and faceted at {32, 0}: the same prism,
// vertex for vertex. The exact cylinder's seam lies along its frame's x, where master's ring
// starts, its rims are cut at equal angles from there, and its side is one row of 32 quads.
TEST(FacetExact, ACylinderAtThirtyTwoIsMastersPrism) {
    const auto cylinder = exact::makeCylinder(geo::Frame{}, 5.0, 10.0);
    ASSERT_NE(cylinder, nullptr);
    const auto bridge = facetExact(*cylinder, FacetResolution{32, 0.0});
    ASSERT_NE(bridge.solid, nullptr) << bridge.error;
    const auto master = hz::model::PrimitiveFactory::makeCylinder(5.0, 10.0, 32);
    ASSERT_NE(master, nullptr);

    EXPECT_EQ(master->faceCount(), 34u);
    EXPECT_EQ(master->edgeCount(), 96u);
    EXPECT_EQ(master->vertexCount(), 64u);
    EXPECT_EQ(bridge.solid->faceCount(), master->faceCount());
    EXPECT_EQ(bridge.solid->edgeCount(), master->edgeCount());
    EXPECT_EQ(bridge.solid->vertexCount(), master->vertexCount());
    EXPECT_TRUE(bridge.solid->isValid()) << bridge.solid->validationReport();

    std::vector<Vec3> ours;
    for (const auto& v : bridge.solid->vertices()) ours.push_back(v.point);
    std::vector<Vec3> theirs;
    for (const auto& v : master->vertices()) theirs.push_back(v.point);
    for (const Vec3& p : theirs) {
        EXPECT_TRUE(hasPointNear(ours, p, 1e-12))
            << "master's vertex " << p.x << ", " << p.y << ", " << p.z;
    }
    for (const Vec3& p : ours) {
        EXPECT_TRUE(hasPointNear(theirs, p, 1e-12))
            << "a vertex of its own " << p.x << ", " << p.y << ", " << p.z;
    }

    const double masterVolume = MassPropertiesCalculator::compute(*master).volume;
    const double volume = MassPropertiesCalculator::compute(*bridge.solid).volume;
    EXPECT_NEAR(volume, masterVolume, 1e-12 * masterVolume);
}

// A sphere's poles and a cone's apex are where its grid's rows close to a point: faceted at 32,
// each is master's, vertex for vertex: the sphere's 16 rows at equal latitudes (its seam, a half
// circle, in stepsFor(r, pi) chords) and 32 columns (its equator's, which is no edge), the cone's
// one row of triangles to its apex.
TEST(FacetExact, APoleAndAnApexAreMastersToo) {
    const struct {
        const char* what;
        std::unique_ptr<topo::Solid> exact;
        std::unique_ptr<topo::Solid> master;
    } cases[] = {
        {"a sphere", exact::makeSphere(geo::Frame{}, 5.0),
         hz::model::PrimitiveFactory::makeSphere(5.0, 32)},
        {"a cone", exact::makeCone(geo::Frame{}, 5.0, 0.0, 7.0),
         hz::model::PrimitiveFactory::makeCone(5.0, 0.0, 7.0, 32)},
    };
    for (const auto& c : cases) {
        SCOPED_TRACE(c.what);
        ASSERT_NE(c.exact, nullptr);
        ASSERT_NE(c.master, nullptr);
        const auto bridge = facetExact(*c.exact, FacetResolution{32, 0.0});
        ASSERT_NE(bridge.solid, nullptr) << bridge.error;
        EXPECT_EQ(bridge.solid->faceCount(), c.master->faceCount());
        EXPECT_EQ(bridge.solid->edgeCount(), c.master->edgeCount());
        EXPECT_EQ(bridge.solid->vertexCount(), c.master->vertexCount());
        std::vector<Vec3> ours;
        for (const auto& v : bridge.solid->vertices()) ours.push_back(v.point);
        for (const auto& v : c.master->vertices()) {
            EXPECT_TRUE(hasPointNear(ours, v.point, 1e-12))
                << "master's vertex " << v.point.x << ", " << v.point.y << ", " << v.point.z;
        }
        const double masterVolume = MassPropertiesCalculator::compute(*c.master).volume;
        EXPECT_NEAR(MassPropertiesCalculator::compute(*bridge.solid).volume, masterVolume,
                    1e-12 * masterVolume);
    }
}

// Master's torus is cut round its tube into half its segments (makeTorus's tubeSegments), which
// stepsFor of the tube's circle is not: at {32, 0, 16} the exact torus's bridge is master's torus,
// its rings round the tube from the outer equator (v = 0) as master's start, vertex for vertex.
TEST(FacetExact, ATorusAtThirtyTwoAndSixteenIsMastersTorus) {
    const auto torus = exact::makeTorus(geo::Frame{}, 5.0, 1.0);
    ASSERT_NE(torus, nullptr);
    const auto bridge = facetExact(*torus, FacetResolution{32, 0.0, 16});
    ASSERT_NE(bridge.solid, nullptr) << bridge.error;
    const auto master = hz::model::PrimitiveFactory::makeTorus(5.0, 1.0, 32);
    ASSERT_NE(master, nullptr);
    EXPECT_EQ(master->faceCount(), 32u * 16u);
    EXPECT_EQ(bridge.solid->faceCount(), master->faceCount());
    EXPECT_EQ(bridge.solid->edgeCount(), master->edgeCount());
    EXPECT_EQ(bridge.solid->vertexCount(), master->vertexCount());
    std::vector<Vec3> ours;
    for (const auto& v : bridge.solid->vertices()) ours.push_back(v.point);
    std::vector<Vec3> theirs;
    for (const auto& v : master->vertices()) theirs.push_back(v.point);
    for (const Vec3& p : theirs) {
        EXPECT_TRUE(hasPointNear(ours, p, 1e-12))
            << "master's vertex " << p.x << ", " << p.y << ", " << p.z;
    }
    for (const Vec3& p : ours) {
        EXPECT_TRUE(hasPointNear(theirs, p, 1e-12))
            << "a vertex of its own " << p.x << ", " << p.y << ", " << p.z;
    }
}

// Master's sphere is cut pole to pole into half its segments, rounded down (makeSphere's stacks):
// at 33 segments, 16 rows, where stepsFor(r, pi) would give 17. At {33, 0, 16} the exact sphere's
// bridge is master's sphere, vertex for vertex: 33 columns from its seam, 16 rows at equal
// latitudes, a fan of triangles at each pole.
TEST(FacetExact, ASphereAtThirtyThreeIsMastersSphere) {
    const auto sphere = exact::makeSphere(geo::Frame{}, 3.0);
    ASSERT_NE(sphere, nullptr);
    const auto bridge = facetExact(*sphere, FacetResolution{33, 0.0, 16});
    ASSERT_NE(bridge.solid, nullptr) << bridge.error;
    const auto master = hz::model::PrimitiveFactory::makeSphere(3.0, 33, 16);
    ASSERT_NE(master, nullptr);
    EXPECT_EQ(master->vertexCount(), 33u * 15u + 2u);
    EXPECT_EQ(bridge.solid->faceCount(), master->faceCount());
    EXPECT_EQ(bridge.solid->edgeCount(), master->edgeCount());
    EXPECT_EQ(bridge.solid->vertexCount(), master->vertexCount());
    std::vector<Vec3> ours;
    for (const auto& v : bridge.solid->vertices()) ours.push_back(v.point);
    std::vector<Vec3> theirs;
    for (const auto& v : master->vertices()) theirs.push_back(v.point);
    for (const Vec3& p : theirs) {
        EXPECT_TRUE(hasPointNear(ours, p, 1e-12))
            << "master's vertex " << p.x << ", " << p.y << ", " << p.z;
    }
    for (const Vec3& p : ours) {
        EXPECT_TRUE(hasPointNear(theirs, p, 1e-12))
            << "a vertex of its own " << p.x << ", " << p.y << ", " << p.z;
    }
}

// Each facet is named for its exact face (`<face>/facet:<k>`, or the face's own tag where it is
// one facet) and each chord for its exact edge (`<edge>/chord:<k>`, or the edge's tag where it is
// one chord): logicalFace and logicalEdge take them back to the exact solid's names, a seam's
// included. Every other edge of the bridge lies inside one face. Named again as a Stable feature
// names them (nameEdgesLogically on both), the chords between two faces keep their logical names;
// a seam's chords, as every line between two facets of one face, are its face's seams.
TEST(FacetExact, FacetsNameTheirExactFace) {
    const geo::Frame frame = geo::Frame::fromAxis({1, 2, 3}, {0.3, -0.4, 1.0}, {1, 1, 0});
    const struct {
        const char* what;
        std::function<std::unique_ptr<topo::Solid>()> make;
    } cases[] = {
        {"a box", [&] { return exact::makeBox(frame, 4.0, 5.0, 6.0); }},
        {"a cylinder", [&] { return exact::makeCylinder(frame, 5.0, 10.0); }},
        {"a frustum", [&] { return exact::makeCone(frame, 5.0, 2.0, 7.0); }},
        {"a cone to its apex", [&] { return exact::makeCone(frame, 0.0, 4.0, 6.0); }},
        {"a sphere", [&] { return exact::makeSphere(frame, 5.0); }},
        {"a torus", [&] { return exact::makeTorus(frame, 6.0, 2.0); }},
        {"a sphere's sector", [&] { return exact::makeSphereSector(frame, 5.0, 2.0); }},
        {"a plate with a hole",
         [] {
             return exact::makePlateWithHole({0, 0, 0}, {20, 10, 4}, {8, 5}, 3.0);
         }},
    };
    for (const auto& c : cases) {
        SCOPED_TRACE(c.what);
        auto solid = c.make();
        ASSERT_NE(solid, nullptr);
        nameEdgesLogically(*solid);
        std::set<std::string> exactFaces;
        for (const auto& f : solid->faces()) exactFaces.insert(f.topoId.tag());
        std::set<std::string> exactEdges;
        std::set<std::string> exactBetween;  // the edges between two faces, not a face's seams
        for (const auto& e : solid->edges()) {
            exactEdges.insert(e.topoId.tag());
            const auto [a, b] = facesOf(e);
            if (a != b) exactBetween.insert(e.topoId.tag());
        }
        ASSERT_EQ(exactEdges.size(), solid->edges().size()) << "each exact edge named once";

        auto bridge = facetExact(*solid, FacetResolution{32, 0.0});
        ASSERT_NE(bridge.solid, nullptr) << bridge.error;
        EXPECT_TRUE(bridge.solid->isValid()) << bridge.solid->validationReport();
        EXPECT_TRUE(bridge.outlined.empty());

        std::set<std::string> faces;
        for (const auto& f : bridge.solid->faces()) faces.insert(logicalFace(f.topoId.tag()));
        EXPECT_EQ(faces, exactFaces);

        std::set<std::string> chords;
        for (const auto& e : bridge.solid->edges()) {
            const std::string edge = logicalEdge(e.topoId.tag());
            if (exactEdges.count(edge) != 0) {
                chords.insert(edge);
                continue;
            }
            const auto [a, b] = facesOf(e);
            EXPECT_EQ(logicalFace(a), logicalFace(b))
                << e.topoId.tag() << " is no chord of an edge, so lies inside one face";
        }
        EXPECT_EQ(chords, exactEdges);

        nameEdgesLogically(*bridge.solid);
        std::set<std::string> between;
        for (const auto& e : bridge.solid->edges()) {
            const auto [a, b] = facesOf(e);
            if (logicalFace(a) != logicalFace(b)) between.insert(logicalEdge(e.topoId.tag()));
        }
        EXPECT_EQ(between, exactBetween);
    }
}

// Ruling 18: a converted feature's bridge copy is named by nameBridgeEdges, not
// nameEdgesLogically. Each chord lying on an exact edge keeps facetExact's name for it, so
// logicalEdge takes it to that edge, a seam's chords included; every other edge of the bridge (a
// line inside one face, between two facets of it) is named a seam of its face,
// `<face>/seam:<n>`, numbered past every name the exact solid uses, so none is taken for an exact
// edge it does not lie on. Whether an edge lies on an exact edge is read from the geometry: both
// its ends on that edge's curve, within its range.
TEST(FacetExact, BridgeEdgesAreNamedForTheExactEdgesTheyLieOn) {
    const geo::Frame frame = geo::Frame::fromAxis({1, 2, 3}, {0.3, -0.4, 1.0}, {1, 1, 0});
    const struct {
        const char* what;
        std::function<std::unique_ptr<topo::Solid>()> make;
        int minor;
    } cases[] = {
        {"a cylinder (its seam a line)", [&] { return exact::makeCylinder(frame, 5.0, 10.0); }, 0},
        {"a sphere (its seam a meridian)", [&] { return exact::makeSphere(frame, 5.0); }, 16},
        {"a torus (two seams, both circles)", [&] { return exact::makeTorus(frame, 6.0, 2.0); },
         16},
        {"a cone to its apex", [&] { return exact::makeCone(frame, 0.0, 4.0, 6.0); }, 0},
    };
    for (const auto& c : cases) {
        SCOPED_TRACE(c.what);
        auto solid = c.make();
        ASSERT_NE(solid, nullptr);
        hz::model::scopeToFeature(*solid, "primitive_7");
        nameEdgesLogically(*solid);
        std::set<std::string> exactTags;
        for (const auto& e : solid->edges()) exactTags.insert(e.topoId.tag());

        auto bridge = facetExact(*solid, FacetResolution{32, 0.0, c.minor});
        ASSERT_NE(bridge.solid, nullptr) << bridge.error;
        hz::model::nameBridgeEdges(*bridge.solid, *solid);

        // The exact edge a point lies on, by its curve within its range.
        const auto onEdge = [](const topo::Edge& e, const Vec3& p) {
            const double t = e.exactCurve->closestParameter(p);
            if ((e.exactCurve->evaluate(t) - p).length() > 1e-9) return false;
            const double lo = std::min(e.curveStart, e.curveEnd) - 1e-9;
            const double hi = std::max(e.curveStart, e.curveEnd) + 1e-9;
            if (!e.exactCurve->periodic()) return t >= lo && t <= hi;
            const geo::Interval range = e.exactCurve->range();
            const double period = range.max - range.min;
            double s = t;
            while (s > hi) s -= period;
            while (s < lo) s += period;
            return s <= hi;
        };
        std::map<std::string, int> chordsOf;
        std::set<std::string> seams;
        for (const auto& e : bridge.solid->edges()) {
            const Vec3& a = e.halfEdge->origin->point;
            const Vec3& b = e.halfEdge->twin->origin->point;
            const topo::Edge* lying = nullptr;
            for (const auto& x : solid->edges()) {
                if (x.exactCurve && onEdge(x, a) && onEdge(x, b)) lying = &x;
            }
            const std::string tag = e.topoId.tag();
            if (lying != nullptr) {
                EXPECT_EQ(logicalEdge(tag), lying->topoId.tag()) << tag;
                ++chordsOf[lying->topoId.tag()];
                continue;
            }
            EXPECT_EQ(exactTags.count(logicalEdge(tag)), 0u)
                << tag << " lies on no exact edge, but is named for one";
            const auto [fa, fb] = facesOf(e);
            EXPECT_EQ(logicalFace(fa), logicalFace(fb)) << tag << " lies inside one face";
            EXPECT_EQ(tag.rfind(logicalFace(fa) + "/seam:", 0), 0u) << tag;
            EXPECT_TRUE(seams.insert(tag).second) << tag << " names two lines";
        }
        for (const auto& tag : exactTags) {
            EXPECT_GE(chordsOf[tag], 1) << tag << " has no chord on it";
        }
    }
}

// Under a chord tolerance each circle is cut as finely as its own radius needs, as master's
// profiles are: a hole of radius 2 and one of radius 20, each through its own plate, each rim in
// FacetResolution::stepsFor(r, 2 pi) chords, and the hole's wall in as many columns.
TEST(FacetExact, AChordToleranceGivesEachArcItsOwnCount) {
    const FacetResolution resolution{32, 0.01};
    ASSERT_NE(resolution.stepsFor(2.0, 2.0 * kPi), resolution.stepsFor(20.0, 2.0 * kPi));
    ASSERT_NE(resolution.stepsFor(20.0, 2.0 * kPi), 32);
    for (const double r : {2.0, 20.0}) {
        SCOPED_TRACE(r);
        auto plate = exact::makePlateWithHole({-30, -30, 0}, {30, 30, 5}, {1, -2}, r);
        ASSERT_NE(plate, nullptr);
        nameEdgesLogically(*plate);
        const auto bridge = facetExact(*plate, resolution);
        ASSERT_NE(bridge.solid, nullptr) << bridge.error;
        int rims = 0;
        for (const auto& e : plate->edges()) {
            if (e.exactCurve == nullptr || e.exactCurve->kind() != geo::CurveKind::Circle) continue;
            ++rims;
            int chords = 0;
            for (const auto& c : bridge.solid->edges()) {
                if (logicalEdge(c.topoId.tag()) == e.topoId.tag()) ++chords;
            }
            EXPECT_EQ(chords, resolution.stepsFor(r, 2.0 * kPi)) << e.topoId.tag();
        }
        EXPECT_EQ(rims, 2);
        int wall = 0;
        for (const auto& f : bridge.solid->faces()) {
            if (logicalFace(f.topoId.tag()) == "hole/wall") ++wall;
        }
        EXPECT_EQ(wall, resolution.stepsFor(r, 2.0 * kPi));
    }
}

// A curved face with neither a NURBS carrier nor an exact surface has nothing to be cut on: its
// corners (two, on a cylinder's side) outline nothing like it. Both bridges refuse it, naming it.
TEST(FacetExact, ACurvedFaceWithNoSurfaceIsRefused) {
    auto cylinder = exact::makeCylinder(geo::Frame{}, 5.0, 10.0);
    ASSERT_NE(cylinder, nullptr);
    int lost = 0;
    for (auto& f : cylinder->faces()) {
        if (f.topoId.tag() != "cylinder/side") continue;
        f.exactSurface.reset();
        ++lost;
    }
    ASSERT_EQ(lost, 1);
    const auto fromExact = facetExact(*cylinder, FacetResolution{});
    EXPECT_EQ(fromExact.solid, nullptr);
    EXPECT_EQ(fromExact.error, "face cylinder/side has no surface to facet");
    const auto fromCarriers = facetCurved(*cylinder);
    EXPECT_EQ(fromCarriers.solid, nullptr);
    EXPECT_EQ(fromCarriers.error, fromExact.error);
}

// An exact block with an exact void inside it, joined by Pattern::append: one body of two shells,
// the outer first, whose volume is the block's less the void's to 1e-9. The void's shell is told
// from a body by its exact signed volume: a block's corners span it, but a cylinder's (two, one on
// each rim) enclose nothing.
TEST(FacetExact, AnExactBodyWithAVoidIsMeasured) {
    const auto outer = exact::makeBlock({0, 0, 0}, {10, 10, 10});
    ASSERT_NE(outer, nullptr);
    const struct {
        const char* what;
        std::function<std::unique_ptr<topo::Solid>()> make;
        double volume;
    } voids[] = {
        {"a block",
         [] {
             return exact::makeBlock({3, 3, 3}, {7, 7, 7});
         },
         64.0},
        {"a cylinder",
         [] {
             return exact::makeCylinder(geo::Frame::fromAxis({5, 5, 2}, {0, 0, 1}), 2.0, 6.0);
         },
         kPi * 4.0 * 6.0},
    };
    for (const auto& v : voids) {
        SCOPED_TRACE(v.what);
        auto inner = v.make();
        ASSERT_NE(inner, nullptr);
        turnInsideOut(*inner);
        topo::Solid both;
        Pattern::append(both, *outer, Mat4::identity());
        Pattern::append(both, *inner, Mat4::identity());
        ASSERT_EQ(both.shells().size(), 2u);

        const auto bodies = Pattern::bodyShells(both);
        ASSERT_EQ(bodies.size(), 1u) << "one body, with its void";
        ASSERT_EQ(bodies[0].size(), 2u);
        EXPECT_EQ(bodies[0][0], &both.shells()[0]) << "the outer shell first";
        EXPECT_EQ(bodies[0][1], &both.shells()[1]);

        const auto props = MassPropertiesCalculator::compute(both);
        ASSERT_TRUE(props.valid) << props.problem;
        const double volume = 1000.0 - v.volume;
        EXPECT_NEAR(props.volume, volume, 1e-9 * volume);
    }
}

// A part built exactly beside an import (stage 3): the import's faces carry NURBS carriers, the
// constructed body's do not, so facetCurved refuses the whole solid and measureShells is left with
// corners. The import is a block with a ball out of it, its void one face bounded by its seam,
// whose corners (the ball's two poles) span nothing: measured so, the void was taken for a body
// of its own. Every shell whose faces are all exact is measured on them where any shell lacks
// carriers, so the void stays with its block and the sphere is a body of its own.
TEST(FacetExact, AnImportsVoidBesideABodyBuiltExactlyStaysWithItsBlock) {
    hz::io::StepReadOptions inMillimetres;
    inMillimetres.unknownLengthUnit = hz::math::LengthUnit::Millimetre;
    auto read = hz::io::StepFormat::fromString(hz::step_text::blockWithABall(), nullptr, nullptr,
                                               inMillimetres);
    ASSERT_EQ(read.size(), 1u) << hz::io::StepFormat::lastError();
    const auto sphere = exact::makeSphere(geo::Frame::fromAxis({10, 2, 2}, {0, 0, 1}), 1.5);
    ASSERT_NE(sphere, nullptr);
    topo::Solid both;
    Pattern::append(both, *read.front(), Mat4::identity());
    Pattern::append(both, *sphere, Mat4::identity());
    ASSERT_EQ(both.shells().size(), 3u);

    const auto bodies = Pattern::bodyShells(both);
    ASSERT_EQ(bodies.size(), 2u) << "the block with its void, and the sphere";
    std::multiset<std::size_t> sizes;
    for (const auto& body : bodies) sizes.insert(body.size());
    EXPECT_EQ(sizes, (std::multiset<std::size_t>{1u, 2u}));
    for (const auto& body : bodies) {
        if (body.size() != 2u) continue;
        EXPECT_EQ(body[0], &both.shells()[0]) << "the block's outer shell first";
        EXPECT_EQ(body[1], &both.shells()[1]) << "then its void";
    }
}

}  // namespace

namespace {

/// A resolution of @p segments per turn and @p chordTolerance turning @p aroundAxis steps round
/// the z axis through the origin (Ruling 24), or none where it is 0.
FacetResolution roundZ(int segments, double chordTolerance, int aroundAxis) {
    FacetResolution resolution;
    resolution.segmentsPerTurn = segments;
    resolution.chordTolerance = chordTolerance;
    resolution.aroundAxis = aroundAxis;
    resolution.axisPoint = Vec3(0, 0, 0);
    resolution.axisDirection = Vec3(0, 0, 2);  // any length
    return resolution;
}

/// How far each of @p solid's vertices off the z axis is from the nearest of @p n equal angles
/// round it from the x axis, in radians: the most.
double offTheAngles(const topo::Solid& solid, int n) {
    double worst = 0.0;
    for (const auto& v : solid.vertices()) {
        if (std::hypot(v.point.x, v.point.y) < 1e-9) continue;
        const double step = 2.0 * kPi / static_cast<double>(n);
        const double a = std::atan2(v.point.y, v.point.x) / step;
        worst = std::max(worst, std::abs(a - std::round(a)) * step);
    }
    return worst;
}

}  // namespace

// Ruling 24: about a revolve's axis (FacetResolution::aroundAxis) every circle round it and every
// face turned about it are cut into the revolve's one count, whatever the chord tolerance would
// give each radius, and a curve along the profile keeps stepsFor. A cylinder of radius 1 at a
// chord tolerance of 0.01 (23 steps by its own radius) is cut 45 times round; a ring torus's
// equator 20 times round while its tube keeps 8.
TEST(FacetExact, RoundARevolvesAxisEveryCircleAndFaceTakesItsCount) {
    const auto cylinder = exact::makeCylinder(geo::Frame{}, 1.0, 3.0);
    ASSERT_NE(cylinder, nullptr);
    const auto own = facetExact(*cylinder, FacetResolution{45, 0.01});
    ASSERT_NE(own.solid, nullptr) << own.error;
    EXPECT_EQ(own.solid->vertexCount(), 2u * 23u) << "each rim by its own radius";
    const auto round = facetExact(*cylinder, roundZ(45, 0.01, 45));
    ASSERT_NE(round.solid, nullptr) << round.error;
    EXPECT_EQ(round.solid->vertexCount(), 2u * 45u);
    EXPECT_EQ(round.solid->faceCount(), 45u + 2u);
    EXPECT_LT(offTheAngles(*round.solid, 45), 1e-12);

    const auto torus = exact::makeTorus(geo::Frame{}, 5.0, 1.0);
    ASSERT_NE(torus, nullptr);
    const auto bridge = facetExact(*torus, roundZ(8, 0.0, 20));
    ASSERT_NE(bridge.solid, nullptr) << bridge.error;
    EXPECT_EQ(bridge.solid->faceCount(), 20u * 8u);
    EXPECT_LT(offTheAngles(*bridge.solid, 20), 1e-12);
}

// A NURBS surface of revolution's u is no angle between its rational arcs' knots, and a face of
// one with a pole at each end (an apple) has no rim to take its columns from: cut evenly in u its
// points miss the turned profile's. About the axis its points inside are its first meridian turned
// to equal angles: every vertex of the apple's bridge lies at a twelfth of a turn, and on the face.
TEST(FacetExact, AnAppleIsCutAtEqualAnglesRoundItsAxis) {
    const auto apple = exact::makeTorus(geo::Frame{}, 1.0, 2.0);
    ASSERT_NE(apple, nullptr);
    const auto even = facetExact(*apple, FacetResolution{12, 0.0});
    ASSERT_NE(even.solid, nullptr) << even.error;
    EXPECT_GT(offTheAngles(*even.solid, 12), 1e-3) << "cut evenly in its u";
    const auto round = facetExact(*apple, roundZ(12, 0.0, 12));
    ASSERT_NE(round.solid, nullptr) << round.error;
    EXPECT_EQ(round.solid->faceCount(), even.solid->faceCount());
    EXPECT_LT(offTheAngles(*round.solid, 12), 1e-12);
    const auto& surface = *apple->faces().front().exactSurface;
    for (const auto& v : round.solid->vertices()) {
        const auto uv = surface.invert(v.point);
        EXPECT_LT((surface.evaluate(uv.x, uv.y) - v.point).length(), 1e-9);
    }
}

namespace {

/// The one solid @p text holds, read as File > Import > STEP reads it, in millimetres.
std::unique_ptr<topo::Solid> readOne(const std::string& text) {
    hz::io::StepReadOptions inMillimetres;
    inMillimetres.unknownLengthUnit = hz::math::LengthUnit::Millimetre;
    auto read = hz::io::StepFormat::fromString(text, nullptr, nullptr, inMillimetres);
    EXPECT_EQ(read.size(), 1u) << hz::io::StepFormat::lastError();
    return read.empty() ? nullptr : std::move(read.front());
}

/// Each face of @p solid as its name and its outer loop's corners, in order from its wire's
/// start; sorted, so two solids sewn from the same facets compare equal.
std::vector<std::pair<std::string, std::vector<std::array<double, 3>>>> facetsOf(
    const topo::Solid& solid) {
    std::vector<std::pair<std::string, std::vector<std::array<double, 3>>>> out;
    for (const auto& f : solid.faces()) {
        std::vector<std::array<double, 3>> corners;
        const topo::HalfEdge* h = f.outerLoop->halfEdge;
        do {
            corners.push_back({h->origin->point.x, h->origin->point.y, h->origin->point.z});
            h = h->next;
        } while (h != f.outerLoop->halfEdge);
        out.emplace_back(f.topoId.tag(), std::move(corners));
    }
    std::sort(out.begin(), out.end());
    return out;
}

/// The farthest any point of @p solid's edges and faces strays inside the cylinder of radius
/// @p radius about the z axis through @p centre, of those whose corners are all on it (within
/// @p on): an edge's middle, the centre of a face along it (not across it, as a cap is). The sag
/// of its chords and facets.
double sagInside(const topo::Solid& solid, const Vec3& centre, double radius, double on) {
    const auto off = [&](const Vec3& p) {
        return radius - std::hypot(p.x - centre.x, p.y - centre.y);
    };
    double worst = 0.0;
    for (const auto& e : solid.edges()) {
        const Vec3 a = e.halfEdge->origin->point;
        const Vec3 b = e.halfEdge->twin->origin->point;
        if (std::abs(off(a)) > on || std::abs(off(b)) > on) continue;
        worst = std::max(worst, off((a + b) * 0.5));
    }
    for (const auto& f : solid.faces()) {
        Vec3 sum;
        int n = 0;
        bool onIt = true;
        const topo::HalfEdge* h = f.outerLoop->halfEdge;
        do {
            onIt = onIt && std::abs(off(h->origin->point)) <= on;
            sum = sum + h->origin->point;
            ++n;
            h = h->next;
        } while (h != f.outerLoop->halfEdge);
        const bool along = std::abs(topo::loopNormal(&f).normalized().z) < 0.5;
        if (onIt && along) worst = std::max(worst, off(sum * (1.0 / n)));
    }
    return worst;
}

}  // namespace

// Ruling 17: a face with a NURBS carrier is an import's, and the bridge cuts it as the import's
// own bridge (facetCurved) does: at 32 a turn, the same facets, corner for corner, a curved face
// that is no rectangle of its surface (a band between two rims, as Solid Edge and Alibre write a
// cylinder's side) and a flat face included. Where it was refused, a part made of an import and
// a primitive had no copy to draw.
TEST(FacetExact, AFaceWithACarrierIsCutAsTheImportsBridgeCutsIt) {
    const struct {
        const char* what;
        std::string text;
    } imports[] = {
        {"OpenCASCADE's cylinder", hz::step_text::externalStyleCylinder()},
        {"a band between two rims", hz::step_text::bandCylinder()},
        {"a cylinder cut on a slant", hz::step_text::slantCylinder(1.0, 2.0, 0.3)},
        {"Alibre's tube", hz::step_text::alibreTube()},
        {"a block with a ball out of it", hz::step_text::blockWithABall()},
        {"a block", hz::step_text::block(false)},
    };
    for (const auto& i : imports) {
        SCOPED_TRACE(i.what);
        const auto solid = readOne(i.text);
        ASSERT_NE(solid, nullptr);
        const auto imported = facetCurved(*solid);
        ASSERT_NE(imported.solid, nullptr) << imported.error;
        const auto bridge = facetExact(*solid, FacetResolution{32, 0.0});
        ASSERT_NE(bridge.solid, nullptr) << bridge.error;
        EXPECT_FALSE(bridge.copied) << "an import's faces are sewn as its bridge sews them";
        EXPECT_EQ(facetsOf(*bridge.solid), facetsOf(*imported.solid));
        EXPECT_EQ(bridge.outlined, imported.outlined);
        EXPECT_TRUE(bridge.solid->isValid()) << bridge.solid->validationReport();
    }
}

// With a chord tolerance an import's faces are cut finer, to keep within it, as a drawing's copy
// needs (0.01 mm on paper): the rims, and the lines and triangles across the side, each with its
// corners on the cylinder, sag no more than it, on a band between two rims and on a side cut
// within its trim (a cylinder cut on a slant). At 32 a turn alone they sag four times as much.
// Each direction of the side is stepped by its own curvature (Ruling 30): round the cylinder as
// finely as the tolerance asks, along it not at all, every point on a rim.
TEST(FacetExact, AnImportsFacesAreCutWithinAChordTolerance) {
    const struct {
        const char* what;
        std::string text;
        double lean;  ///< of the top: it is at z = 2 + x tan(lean)
    } imports[] = {
        {"a band between two rims", hz::step_text::bandCylinder(), 0.0},
        {"a cylinder cut on a slant", hz::step_text::slantCylinder(1.0, 2.0, 0.3), 0.3},
    };
    constexpr double kTolerance = 1e-3;
    for (const auto& i : imports) {
        SCOPED_TRACE(i.what);
        const auto solid = readOne(i.text);
        ASSERT_NE(solid, nullptr);
        const auto coarse = facetExact(*solid, FacetResolution{32, 0.0});
        ASSERT_NE(coarse.solid, nullptr) << coarse.error;
        EXPECT_GT(sagInside(*coarse.solid, {}, 1.0, 1e-9), 4.0 * kTolerance);
        const auto fine = facetExact(*solid, FacetResolution{32, kTolerance});
        ASSERT_NE(fine.solid, nullptr) << fine.error;
        EXPECT_TRUE(fine.solid->isValid()) << fine.solid->validationReport();
        EXPECT_TRUE(fine.outlined.empty());
        EXPECT_LE(sagInside(*fine.solid, {}, 1.0, 1e-9), kTolerance);
        std::size_t onTheBottom = 0;
        for (const auto& v : fine.solid->vertices()) {
            const Vec3& p = v.point;
            const bool bottom = std::abs(p.z) < 1e-9;
            const bool top = std::abs(p.z - (2.0 + p.x * std::tan(i.lean))) < 1e-9;
            EXPECT_TRUE(bottom || top)
                << "a point inside, at " << p.x << ", " << p.y << ", " << p.z;
            if (bottom) ++onTheBottom;
        }
        EXPECT_EQ(onTheBottom, 71u) << "pi / acos(1 - 0.001) = 70.2: 71 chords round, alike";
        const double volume = MassPropertiesCalculator::compute(*fine.solid).volume;
        EXPECT_NEAR(volume, 2.0 * kPi, 2.0 * kPi * 2.0 * kTolerance) << "within its sag";
    }
}

// A part of an import and a primitive, as its exact part gathers them (Ruling 17): the import's
// faces are cut as its bridge cuts them and the primitive's from their exact fields, each as it is
// cut alone, in one copy.
TEST(FacetExact, AnImportBesideABodyBuiltExactlyIsCutEachItsOwnWay) {
    const auto band = readOne(hz::step_text::bandCylinder());
    ASSERT_NE(band, nullptr);
    const auto cylinder = exact::makeCylinder(geo::Frame::fromAxis({5, 0, 0}, {0, 0, 1}), 1.0, 2.0);
    ASSERT_NE(cylinder, nullptr);
    nameEdgesLogically(*cylinder);
    topo::Solid both;
    Pattern::append(both, *band, Mat4::identity());
    Pattern::append(both, *cylinder, Mat4::identity());

    const auto bridge = facetExact(both, FacetResolution{32, 0.0});
    ASSERT_NE(bridge.solid, nullptr) << bridge.error;
    EXPECT_TRUE(bridge.solid->isValid()) << bridge.solid->validationReport();
    const auto imported = facetCurved(*band);
    ASSERT_NE(imported.solid, nullptr) << imported.error;
    const auto built = facetExact(*cylinder, FacetResolution{32, 0.0});
    ASSERT_NE(built.solid, nullptr) << built.error;
    auto expected = facetsOf(*imported.solid);
    const auto own = facetsOf(*built.solid);
    expected.insert(expected.end(), own.begin(), own.end());
    std::sort(expected.begin(), expected.end());
    EXPECT_EQ(facetsOf(*bridge.solid), expected);
}

// Ruling 32: an import is untrusted, and a tolerance may ask for any number of steps. A cylinder's
// trimmed side cut to a nanometre asks for some 70,000 steps round it: it is cut at its grid's
// cap, 400, its rims into 400 chords each and inside it no more steps either way, in a moment.
// (Its rims cut into 4096 chords each, the most a curve is, the cut by ears of its outline took
// 21 s in Debug.)
TEST(FacetExact, AFaceAskingForMoreThanItsGridsCapIsCutWithinIt) {
    const auto slant = readOne(hz::step_text::slantCylinder(1.0, 2.0, 0.3));
    ASSERT_NE(slant, nullptr);
    const auto started = std::chrono::steady_clock::now();
    const auto fine = facetExact(*slant, FacetResolution{32, 1e-9});
    const double seconds =
        std::chrono::duration<double>(std::chrono::steady_clock::now() - started).count();
    ASSERT_NE(fine.solid, nullptr) << fine.error;
    EXPECT_TRUE(fine.solid->isValid()) << fine.solid->validationReport();
    EXPECT_TRUE(fine.outlined.empty());
    // Two rims of 400 chords each, and no more inside than the cap's square.
    std::size_t onTheBottom = 0;
    for (const auto& v : fine.solid->vertices()) {
        if (std::abs(v.point.z) < 1e-9) ++onTheBottom;
    }
    EXPECT_EQ(onTheBottom, 400u);
    EXPECT_LE(fine.solid->vertexCount(), 2u * 400u + 400u * 400u);
    std::printf("[   INFO   ] cut to a nanometre in %.3f s, %zu vertices\n", seconds,
                fine.solid->vertexCount());
#if HZ_TIME_LIMITS
#ifdef NDEBUG
    EXPECT_LT(seconds, 1.0);
#else
    EXPECT_LT(seconds, 10.0 * HZ_UNOPTIMIZED_SLOWDOWN) << "unoptimized: a generous bound";
#endif
#endif
}

// Ruling 32: a surface whose knots are finite but span no finite range (from -1e308 to 1e308) has
// nothing to measure a cut by. Its face is refused, saying so, by both bridges; never a crash, nor
// a grid without end.
TEST(FacetExact, AFaceWithNoFiniteParameterRangeIsRefused) {
    auto slant = readOne(hz::step_text::slantCylinder(1.0, 2.0, 0.3));
    ASSERT_NE(slant, nullptr);
    std::string side;
    for (auto& face : slant->faces()) {
        if (!face.exactSurface || face.exactSurface->kind() != geo::SurfaceKind::Cylinder) continue;
        constexpr double kFar = 1e308;
        face.surface = std::make_shared<geo::NurbsSurface>(
            std::vector<std::vector<Vec3>>{{{1, 0, 0}, {1, 0, 2}}, {{0, 1, 0}, {0, 1.5, 2}}},
            std::vector<std::vector<double>>{{1, 1}, {1, 1}},
            std::vector<double>{-kFar, -kFar, kFar, kFar}, std::vector<double>{0, 0, 1, 1}, 1, 1);
        side = face.topoId.tag();
    }
    ASSERT_FALSE(side.empty());
    for (const double tolerance : {0.0, 1e-3}) {
        SCOPED_TRACE(tolerance);
        const auto bridge = facetExact(*slant, FacetResolution{32, tolerance});
        EXPECT_EQ(bridge.solid, nullptr);
        EXPECT_EQ(bridge.error, "face " + side + "'s surface has no finite parameter range");
    }
    const auto imported = facetCurved(*slant);
    EXPECT_EQ(imported.solid, nullptr);
    EXPECT_EQ(imported.error, "face " + side + "'s surface has no finite parameter range");
}

// Ruling 32: a face cut from pole to pole (a STEP ball's skin, a void in a block, one face
// bounded by its seam there and back) takes its columns from its own turning. Under a tolerance
// they went up to the most a curve is (4096), here 497 round a ball of radius 1 at 2e-5; finer,
// columns times rows passed the cap's square and the face could not be cut at all, nor a drawing
// of it opened. They stop at the cap, 400, and the face is cut there, valid and watertight with
// its seam.
TEST(FacetExact, AFaceFromPoleToPoleIsCutAtItsGridsCapUnderAFineTolerance) {
    const auto block = readOne(hz::step_text::blockWithABall());
    ASSERT_NE(block, nullptr);
    // The ball's radius is 1: a turn asks for pi / acos(1 - 2e-5) = 497 chords.
    const auto started = std::chrono::steady_clock::now();
    const auto fine = facetExact(*block, FacetResolution{32, 2e-5});
    const double seconds =
        std::chrono::duration<double>(std::chrono::steady_clock::now() - started).count();
    ASSERT_NE(fine.solid, nullptr) << fine.error;
    EXPECT_TRUE(fine.solid->isValid()) << fine.solid->validationReport();
    EXPECT_TRUE(fine.outlined.empty());
    // Round the ball's ring nearest its equator (z = 2): its columns, the cap's.
    double nearest = 1.0;
    for (const auto& v : fine.solid->vertices()) {
        if (std::hypot(v.point.x - 2.0, v.point.y - 2.0) > 0.5) {
            nearest = std::min(nearest, std::abs(v.point.z - 2.0));
        }
    }
    std::size_t onTheRing = 0;
    for (const auto& v : fine.solid->vertices()) {
        if (std::hypot(v.point.x - 2.0, v.point.y - 2.0) > 0.5 &&
            std::abs(std::abs(v.point.z - 2.0) - nearest) < 1e-9 && v.point.z >= 2.0 - 1e-9) {
            ++onTheRing;
        }
    }
    EXPECT_EQ(onTheRing, 400u);
    std::printf("[   INFO   ] cut in %.3f s, %zu facets\n", seconds, fine.solid->faceCount());
#if HZ_TIME_LIMITS
#ifdef NDEBUG
    EXPECT_LT(seconds, 2.0);
#else
    EXPECT_LT(seconds, 20.0 * HZ_UNOPTIMIZED_SLOWDOWN) << "unoptimized: a generous bound";
#endif
#endif
}
