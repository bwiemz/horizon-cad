#include <gtest/gtest.h>

#include "horizon/document/Sketch.h"
#include "horizon/constraint/ConstraintSystem.h"
#include "horizon/drafting/DraftLine.h"
#include "horizon/drafting/DraftCircle.h"
#include "horizon/drafting/SketchPlane.h"
#include "horizon/math/Vec2.h"
#include "horizon/math/Vec3.h"
#include "horizon/math/BoundingBox.h"

using namespace hz::doc;
using namespace hz::draft;
using namespace hz::math;

TEST(SketchTest, DefaultSketchIsOnXYPlane) {
    Sketch sketch;
    const SketchPlane& plane = sketch.plane();
    EXPECT_TRUE(plane.normal().isApproxEqual(Vec3::UnitZ));
    EXPECT_TRUE(plane.origin().isApproxEqual(Vec3::Zero));
    EXPECT_TRUE(plane.xAxis().isApproxEqual(Vec3::UnitX));
    EXPECT_TRUE(plane.yAxis().isApproxEqual(Vec3::UnitY));
}

TEST(SketchTest, AddAndRetrieveEntity) {
    Sketch sketch;
    auto line = std::make_shared<DraftLine>(Vec2(0.0, 0.0), Vec2(10.0, 5.0));
    const uint64_t lineId = line->id();

    sketch.addEntity(line);

    ASSERT_EQ(sketch.entities().size(), 1u);
    EXPECT_EQ(sketch.entities()[0]->id(), lineId);
}

TEST(SketchTest, RemoveEntity) {
    Sketch sketch;
    auto line = std::make_shared<DraftLine>(Vec2(0.0, 0.0), Vec2(10.0, 5.0));
    const uint64_t lineId = line->id();
    sketch.addEntity(line);

    sketch.removeEntity(lineId);

    EXPECT_TRUE(sketch.entities().empty());
}

TEST(SketchTest, SketchOwnsConstraintSystem) {
    Sketch sketch;
    // ConstraintSystem is accessible and initially empty.
    EXPECT_TRUE(sketch.constraintSystem().empty());

    // Const accessor also works.
    const Sketch& cSketch = sketch;
    EXPECT_TRUE(cSketch.constraintSystem().empty());
}

TEST(SketchTest, ClearRemovesEverything) {
    Sketch sketch;
    sketch.addEntity(std::make_shared<DraftLine>(Vec2(0.0, 0.0), Vec2(5.0, 5.0)));
    sketch.addEntity(std::make_shared<DraftLine>(Vec2(1.0, 1.0), Vec2(8.0, 2.0)));

    sketch.clear();

    EXPECT_TRUE(sketch.entities().empty());
    EXPECT_TRUE(sketch.constraintSystem().empty());

    // Spatial index should be empty too — verify via query.
    BoundingBox huge(Vec3(-1000, -1000, 0), Vec3(1000, 1000, 0));
    EXPECT_TRUE(sketch.spatialIndex().query(huge).empty());
}

TEST(SketchTest, CustomPlane) {
    Vec3 origin(10.0, 20.0, 30.0);
    Vec3 normal = Vec3::UnitY;
    Vec3 xAxis  = Vec3::UnitX;
    SketchPlane plane(origin, normal, xAxis);

    Sketch sketch(plane);

    EXPECT_TRUE(sketch.plane().origin().isApproxEqual(origin));
    EXPECT_TRUE(sketch.plane().normal().isApproxEqual(normal));
    EXPECT_TRUE(sketch.plane().xAxis().isApproxEqual(xAxis));
}

TEST(SketchTest, EntitiesStoreLocalCoords) {
    // Sketch on the XZ plane (normal = Y).
    // A line with local coords (5,3)-(2,7) is stored as-is in local space.
    Vec3 normal = Vec3::UnitY;
    Vec3 xAxis  = Vec3::UnitX;
    SketchPlane plane(Vec3::Zero, normal, xAxis);
    Sketch sketch(plane);

    Vec2 localStart(5.0, 3.0);
    Vec2 localEnd(2.0, 7.0);
    auto line = std::make_shared<DraftLine>(localStart, localEnd);
    sketch.addEntity(line);

    const auto& stored = sketch.entities()[0];
    auto* storedLine = dynamic_cast<DraftLine*>(stored.get());
    ASSERT_NE(storedLine, nullptr);

    EXPECT_NEAR(storedLine->start().x, 5.0, 1e-10);
    EXPECT_NEAR(storedLine->start().y, 3.0, 1e-10);
    EXPECT_NEAR(storedLine->end().x, 2.0, 1e-10);
    EXPECT_NEAR(storedLine->end().y, 7.0, 1e-10);
}

TEST(SketchTest, SpatialIndexWorksWithinSketch) {
    Sketch sketch;
    auto line = std::make_shared<DraftLine>(Vec2(0.0, 0.0), Vec2(10.0, 10.0));
    const uint64_t lineId = line->id();
    sketch.addEntity(line);

    // Query that covers the line.
    BoundingBox searchBox(Vec3(-1, -1, 0), Vec3(11, 11, 0));
    auto hits = sketch.spatialIndex().query(searchBox);
    ASSERT_EQ(hits.size(), 1u);
    EXPECT_EQ(hits[0], lineId);

    // Query that misses the line.
    BoundingBox missBox(Vec3(50, 50, 0), Vec3(60, 60, 0));
    EXPECT_TRUE(sketch.spatialIndex().query(missBox).empty());
}

TEST(SketchTest, NameAndId) {
    Sketch sketch;

    // Default name is empty.
    EXPECT_TRUE(sketch.name().empty());

    // Assign a name and id.
    sketch.setName("MySketch");
    sketch.setId(42u);

    EXPECT_EQ(sketch.name(), "MySketch");
    EXPECT_EQ(sketch.id(), 42u);
}

TEST(SketchTest, RebuildSpatialIndex) {
    Sketch sketch;
    auto line = std::make_shared<DraftLine>(Vec2(0.0, 0.0), Vec2(5.0, 5.0));
    sketch.addEntity(line);

    // Clear the index manually, then rebuild.
    sketch.spatialIndex().clear();
    BoundingBox searchBox(Vec3(-1, -1, 0), Vec3(6, 6, 0));
    EXPECT_TRUE(sketch.spatialIndex().query(searchBox).empty());

    sketch.rebuildSpatialIndex();
    auto hits = sketch.spatialIndex().query(searchBox);
    ASSERT_EQ(hits.size(), 1u);
    EXPECT_EQ(hits[0], line->id());
}

TEST(SketchTest, SetPlane) {
    Sketch sketch;  // Default XY plane.

    // Switch to a custom plane.
    Vec3 newOrigin(1.0, 2.0, 3.0);
    SketchPlane newPlane(newOrigin, Vec3::UnitZ, Vec3::UnitX);
    sketch.setPlane(newPlane);

    EXPECT_TRUE(sketch.plane().origin().isApproxEqual(newOrigin));
}

TEST(SketchTest, MultipleEntitiesAndSpatialQuery) {
    Sketch sketch;

    auto line1 = std::make_shared<DraftLine>(Vec2(0.0, 0.0), Vec2(5.0, 0.0));
    auto line2 = std::make_shared<DraftLine>(Vec2(10.0, 10.0), Vec2(15.0, 10.0));
    sketch.addEntity(line1);
    sketch.addEntity(line2);

    EXPECT_EQ(sketch.entities().size(), 2u);

    // Query only around line1.
    BoundingBox box1(Vec3(-1, -1, 0), Vec3(6, 1, 0));
    auto hits = sketch.spatialIndex().query(box1);
    ASSERT_EQ(hits.size(), 1u);
    EXPECT_EQ(hits[0], line1->id());
}
