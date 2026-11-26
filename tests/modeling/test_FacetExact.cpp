// The bridge from exact faces (stage 3): facetExact cuts a solid of exact faces, which carry no
// NURBS carrier, into the planar facets every operation not yet converted reads, at a feature's
// own resolution, so that a part built exactly has master's faceted counts, and the names on its
// facets and chords are its exact faces' and edges'. A curved face with no surface at all is
// refused, by facetCurved too, never outlined by its corners; and a body of exact shells with a
// void is told apart by its shells' exact signed volumes.

#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <functional>
#include <map>
#include <memory>
#include <numbers>
#include <set>
#include <string>
#include <utility>
#include <vector>

#include "horizon/geometry/Frame.h"
#include "horizon/geometry/curves/Curve3.h"
#include "horizon/math/Mat4.h"
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

}  // namespace
