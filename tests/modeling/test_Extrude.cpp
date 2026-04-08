#include "horizon/modeling/Extrude.h"

#include "horizon/drafting/DraftCircle.h"
#include "horizon/drafting/DraftLine.h"
#include "horizon/drafting/SketchPlane.h"
#include "horizon/topology/Solid.h"
#include "horizon/topology/TopologyID.h"

#include <gtest/gtest.h>
#include <memory>
#include <set>
#include <string>
#include <vector>

using namespace hz::model;
using namespace hz::topo;
using namespace hz::draft;
using hz::math::Vec2;
using hz::math::Vec3;

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

    // Cylinder uses box topology: 8V, 12E, 6F.
    EXPECT_EQ(solid->vertexCount(), 8u);
    EXPECT_EQ(solid->edgeCount(), 12u);
    EXPECT_EQ(solid->faceCount(), 6u);
    EXPECT_TRUE(solid->checkEulerFormula());
    EXPECT_TRUE(solid->checkManifold());
    EXPECT_TRUE(solid->isValid()) << solid->validationReport();
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
        EXPECT_NE(f.surface, nullptr) << "Face " << f.id << " (" << f.topoId.tag()
                                      << ") has no NURBS surface";
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
