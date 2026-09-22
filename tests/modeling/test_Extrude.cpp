#include <gtest/gtest.h>

#include <cmath>
#include <memory>
#include <set>
#include <string>
#include <vector>

#include "horizon/drafting/DraftArc.h"
#include "horizon/drafting/DraftCircle.h"
#include "horizon/drafting/DraftLine.h"
#include "horizon/drafting/SketchPlane.h"
#include "horizon/geometry/curves/NurbsCurve.h"
#include "horizon/geometry/surfaces/NurbsSurface.h"
#include "horizon/math/Constants.h"
#include "horizon/modeling/Extrude.h"
#include "horizon/modeling/MassProperties.h"
#include "horizon/topology/GeometryValidator.h"
#include "horizon/topology/Solid.h"
#include "horizon/topology/TopologyID.h"

using namespace hz::model;
using namespace hz::topo;
using namespace hz::draft;
using hz::math::kPi;
using hz::math::Vec2;
using hz::math::Vec3;

namespace {
double volumeOf(const Solid& solid) {
    return MassPropertiesCalculator::compute(solid).volume;
}
}  // namespace

// Helper: build a rectangle profile on XY plane.
static std::vector<std::shared_ptr<DraftEntity>> makeRectProfile(double w, double h) {
    std::vector<std::shared_ptr<DraftEntity>> profile;
    profile.push_back(std::make_shared<DraftLine>(Vec2(0, 0), Vec2(w, 0)));
    profile.push_back(std::make_shared<DraftLine>(Vec2(w, 0), Vec2(w, h)));
    profile.push_back(std::make_shared<DraftLine>(Vec2(w, h), Vec2(0, h)));
    profile.push_back(std::make_shared<DraftLine>(Vec2(0, h), Vec2(0, 0)));
    return profile;
}

// Helper: build a circle profile.
static std::vector<std::shared_ptr<DraftEntity>> makeCircleProfile(double r) {
    std::vector<std::shared_ptr<DraftEntity>> profile;
    profile.push_back(std::make_shared<DraftCircle>(Vec2(0, 0), r));
    return profile;
}

// Helper: build a triangle profile.
static std::vector<std::shared_ptr<DraftEntity>> makeTriangleProfile() {
    std::vector<std::shared_ptr<DraftEntity>> profile;
    profile.push_back(std::make_shared<DraftLine>(Vec2(0, 0), Vec2(3, 0)));
    profile.push_back(std::make_shared<DraftLine>(Vec2(3, 0), Vec2(1.5, 2.6)));
    profile.push_back(std::make_shared<DraftLine>(Vec2(1.5, 2.6), Vec2(0, 0)));
    return profile;
}

// ---------------------------------------------------------------------------
// ExtrudeRectangleProduces6FaceBox
// ---------------------------------------------------------------------------

TEST(ExtrudeTest, ExtrudeRectangleProduces6FaceBox) {
    auto profile = makeRectProfile(2.0, 3.0);
    SketchPlane plane;  // XY at origin
    Vec3 dir(0, 0, 1);
    double dist = 4.0;

    auto solid = Extrude::execute(profile, plane, dir, dist, "extrude_rect");
    ASSERT_NE(solid, nullptr);

    EXPECT_EQ(solid->vertexCount(), 8u);
    EXPECT_EQ(solid->edgeCount(), 12u);
    EXPECT_EQ(solid->faceCount(), 6u);
    EXPECT_EQ(solid->shellCount(), 1u);
    EXPECT_TRUE(solid->checkEulerFormula());
    EXPECT_TRUE(solid->checkManifold());
    EXPECT_TRUE(solid->isValid()) << solid->validationReport();
}

// ---------------------------------------------------------------------------
// ExtrudeCircleProducesCylinder
// ---------------------------------------------------------------------------

TEST(ExtrudeTest, ExtrudeCircleProducesCylinder) {
    auto profile = makeCircleProfile(5.0);
    SketchPlane plane;
    Vec3 dir(0, 0, 1);
    double dist = 10.0;

    auto solid = Extrude::execute(profile, plane, dir, dist, "extrude_cyl");
    ASSERT_NE(solid, nullptr);

    // A faceted cylinder: an N-sided prism.  It used to be box topology —
    // four points on the circle, a square prism with a cylinder surface
    // pasted on — so its volume read 500 against 785.
    const size_t N = Extrude::kDefaultSegments;
    EXPECT_EQ(solid->vertexCount(), 2 * N);
    EXPECT_EQ(solid->edgeCount(), 3 * N);
    EXPECT_EQ(solid->faceCount(), N + 2);
    EXPECT_TRUE(solid->checkEulerFormula());
    EXPECT_TRUE(solid->checkManifold());
    EXPECT_TRUE(solid->isValid()) << solid->validationReport();
    EXPECT_TRUE(GeometryValidator::isGeometricallyValid(*solid))
        << GeometryValidator::report(*solid);

    // The inscribed N-gon prism, exactly.
    const double exact = 0.5 * N * 25.0 * std::sin(2.0 * kPi / N) * dist;
    EXPECT_NEAR(volumeOf(*solid), exact, 1e-9);
}

// ---------------------------------------------------------------------------
// Faceted profile arcs (Phase 91)
// ---------------------------------------------------------------------------

TEST(ExtrudeTest, CircleVolumeConvergesFromBelow) {
    const double exact = kPi * 25.0 * 10.0;
    double previousError = exact;
    for (int segments : {8, 16, 32, 64, 128}) {
        auto solid = Extrude::execute(makeCircleProfile(5.0), SketchPlane(), Vec3(0, 0, 1), 10.0,
                                      "c", segments);
        ASSERT_NE(solid, nullptr);
        const double error = exact - volumeOf(*solid);
        EXPECT_GT(error, 0.0) << "an inscribed prism can only undershoot";
        EXPECT_LT(error, previousError * 0.3) << "segments = " << segments;
        previousError = error;
    }
    EXPECT_LT(previousError / exact, 5e-4);
}

TEST(ExtrudeTest, ArcsAreFollowedNotChorded) {
    // A slot: two lines joined by two half-circle arcs, the second arc drawn
    // against the chain so its samples are walked backwards.
    std::vector<std::shared_ptr<DraftEntity>> slot = {
        std::make_shared<DraftLine>(Vec2(0, -1), Vec2(4, -1)),
        std::make_shared<DraftArc>(Vec2(4, 0), 1.0, -kPi / 2, kPi / 2),
        std::make_shared<DraftLine>(Vec2(4, 1), Vec2(0, 1)),
        std::make_shared<DraftArc>(Vec2(0, 0), 1.0, kPi / 2, 3 * kPi / 2),
    };
    auto solid = Extrude::execute(slot, SketchPlane(), Vec3(0, 0, 1), 10.0, "slot", 64);
    ASSERT_NE(solid, nullptr);
    EXPECT_TRUE(GeometryValidator::isGeometricallyValid(*solid))
        << GeometryValidator::report(*solid);
    // Each half circle is 32 chords of a 64-gon: the rectangle plus one
    // inscribed 64-gon.  Chording each arc straight across gave the bare 80.
    const double exact = (8.0 + 0.5 * 64 * std::sin(2.0 * kPi / 64)) * 10.0;
    EXPECT_NEAR(volumeOf(*solid), exact, 1e-9);
    EXPECT_EQ(solid->faceCount(), static_cast<size_t>(2 + 2 + 2 * 32));
}

TEST(ExtrudeTest, ArcFacetsRecordTheirCylinderAndCircles) {
    auto solid = Extrude::execute(makeCircleProfile(5.0), SketchPlane(), Vec3(0, 0, 1), 10.0, "c");
    ASSERT_NE(solid, nullptr);
    int lateral = 0;
    for (const auto& f : solid->faces()) {
        if (f.topoId.tag().find("lateral") == std::string::npos) {
            EXPECT_EQ(f.analyticSurface, nullptr) << "caps are exact";
            continue;
        }
        ++lateral;
        ASSERT_NE(f.analyticSurface, nullptr);
        // Any point of the ideal lies on the radius-5 cylinder about Z.
        const Vec3 p = f.analyticSurface->evaluate(0.3, 0.7);
        EXPECT_NEAR(std::hypot(p.x, p.y), 5.0, 1e-9);
    }
    EXPECT_EQ(lateral, Extrude::kDefaultSegments);

    int rims = 0;
    for (const auto& e : solid->edges()) {
        const Vec3 a = e.halfEdge->origin->point;
        const Vec3 b = e.halfEdge->twin->origin->point;
        if (std::abs(a.z - b.z) > 1e-12) {
            EXPECT_EQ(e.analyticCurve, nullptr) << "a lateral edge is a real line";
            continue;
        }
        ++rims;
        ASSERT_NE(e.analyticCurve, nullptr);
        const Vec3 c = e.analyticCurve->evaluate(0.25);
        EXPECT_NEAR(std::hypot(c.x, c.y), 5.0, 1e-9);
        EXPECT_NEAR(c.z, a.z, 1e-9);
    }
    EXPECT_EQ(rims, 2 * Extrude::kDefaultSegments);
}

TEST(ExtrudeTest, ChordToleranceFacetsEachArcAtItsOwnRadius) {
    // A 1mm and a 50mm circle at the same budget: the large one needs more
    // chords, and neither sags past the budget.
    for (double r : {1.0, 50.0}) {
        auto solid = Extrude::execute(makeCircleProfile(r), SketchPlane(), Vec3(0, 0, 1), 1.0, "t",
                                      Extrude::kDefaultSegments, 0.01);
        ASSERT_NE(solid, nullptr);
        const int n = static_cast<int>(solid->faceCount()) - 2;
        EXPECT_LE(r * (1.0 - std::cos(kPi / n)), 0.01 * 1.0000001) << "r = " << r;
        EXPECT_GT(r * (1.0 - std::cos(kPi / (n - 1))), 0.01) << "r = " << r;
    }
}

// ---------------------------------------------------------------------------
// ExtrudeHasTopologyIDs
// ---------------------------------------------------------------------------

TEST(ExtrudeTest, ExtrudeHasTopologyIDs) {
    auto profile = makeRectProfile(1.0, 1.0);
    SketchPlane plane;
    Vec3 dir(0, 0, 1);

    auto solid = Extrude::execute(profile, plane, dir, 1.0, "ext1");
    ASSERT_NE(solid, nullptr);

    // All faces should have valid TopologyIDs.
    std::set<std::string> faceTags;
    for (const auto& f : solid->faces()) {
        EXPECT_TRUE(f.topoId.isValid()) << "Face " << f.id << " has no TopologyID";
        faceTags.insert(f.topoId.tag());
    }
    // Should have cap_bottom, cap_top, and 4 laterals.
    EXPECT_TRUE(faceTags.count("ext1/cap_bottom"));
    EXPECT_TRUE(faceTags.count("ext1/cap_top"));
    EXPECT_TRUE(faceTags.count("ext1/lateral_0"));
    EXPECT_TRUE(faceTags.count("ext1/lateral_1"));

    // All edges should have valid TopologyIDs.
    for (const auto& e : solid->edges()) {
        EXPECT_TRUE(e.topoId.isValid()) << "Edge " << e.id << " has no TopologyID";
    }
}

// ---------------------------------------------------------------------------
// ExtrudeHasNURBSGeometry
// ---------------------------------------------------------------------------

TEST(ExtrudeTest, ExtrudeHasNURBSGeometry) {
    auto profile = makeRectProfile(2.0, 2.0);
    SketchPlane plane;
    Vec3 dir(0, 0, 1);

    auto solid = Extrude::execute(profile, plane, dir, 3.0, "ext_geom");
    ASSERT_NE(solid, nullptr);

    // All faces should have NURBS surfaces.
    for (const auto& f : solid->faces()) {
        EXPECT_NE(f.surface, nullptr)
            << "Face " << f.id << " (" << f.topoId.tag() << ") has no NURBS surface";
    }

    // All edges should have NURBS curves.
    for (const auto& e : solid->edges()) {
        EXPECT_NE(e.curve, nullptr) << "Edge " << e.id << " has no NURBS curve";
    }
}

// ---------------------------------------------------------------------------
// InvalidProfileReturnsNull
// ---------------------------------------------------------------------------

TEST(ExtrudeTest, InvalidProfileReturnsNull) {
    // Open chain (2 lines, not closed).
    std::vector<std::shared_ptr<DraftEntity>> profile;
    profile.push_back(std::make_shared<DraftLine>(Vec2(0, 0), Vec2(1, 0)));
    profile.push_back(std::make_shared<DraftLine>(Vec2(1, 0), Vec2(1, 1)));

    SketchPlane plane;
    Vec3 dir(0, 0, 1);

    auto solid = Extrude::execute(profile, plane, dir, 1.0, "bad");
    EXPECT_EQ(solid, nullptr);
}

// ---------------------------------------------------------------------------
// ExtrudeTriangleProduces5FacePrism
// ---------------------------------------------------------------------------

TEST(ExtrudeTest, ExtrudeTriangleProduces5FacePrism) {
    auto profile = makeTriangleProfile();
    SketchPlane plane;
    Vec3 dir(0, 0, 1);
    double dist = 5.0;

    auto solid = Extrude::execute(profile, plane, dir, dist, "extrude_tri");
    ASSERT_NE(solid, nullptr);

    // Triangle prism: 6V, 9E, 5F (N=3: 2*3=6V, 3*3=9E, 3+2=5F).
    EXPECT_EQ(solid->vertexCount(), 6u);
    EXPECT_EQ(solid->edgeCount(), 9u);
    EXPECT_EQ(solid->faceCount(), 5u);
    EXPECT_EQ(solid->shellCount(), 1u);
    EXPECT_TRUE(solid->checkEulerFormula());
    EXPECT_TRUE(solid->checkManifold());
    EXPECT_TRUE(solid->isValid()) << solid->validationReport();
}

// ---------------------------------------------------------------------------
// ExtrudeCircleHasTopologyIDs
// ---------------------------------------------------------------------------

TEST(ExtrudeTest, ExtrudeCircleHasTopologyIDs) {
    auto profile = makeCircleProfile(3.0);
    SketchPlane plane;
    Vec3 dir(0, 0, 1);

    auto solid = Extrude::execute(profile, plane, dir, 2.0, "cyl1");
    ASSERT_NE(solid, nullptr);

    std::set<std::string> faceTags;
    for (const auto& f : solid->faces()) {
        EXPECT_TRUE(f.topoId.isValid());
        faceTags.insert(f.topoId.tag());
    }
    EXPECT_TRUE(faceTags.count("cyl1/cap_bottom"));
    EXPECT_TRUE(faceTags.count("cyl1/cap_top"));
}

// ---------------------------------------------------------------------------
// EmptyProfileReturnsNull
// ---------------------------------------------------------------------------

TEST(ExtrudeTest, EmptyProfileReturnsNull) {
    std::vector<std::shared_ptr<DraftEntity>> profile;
    SketchPlane plane;
    Vec3 dir(0, 0, 1);

    auto solid = Extrude::execute(profile, plane, dir, 1.0, "empty");
    EXPECT_EQ(solid, nullptr);
}
