#include <gtest/gtest.h>
#include "horizon/drafting/SpatialIndex.h"
#include "horizon/drafting/DraftLine.h"
#include "horizon/drafting/DraftCircle.h"
#include "horizon/drafting/DraftDocument.h"
#include "horizon/math/BoundingBox.h"

using namespace hz::draft;
using namespace hz::math;

TEST(SpatialIndexTest, EmptyIndexQueryReturnsNothing) {
    SpatialIndex index;
    BoundingBox searchBox(Vec3(-100, -100, 0), Vec3(100, 100, 0));
    auto results = index.query(searchBox);
    EXPECT_TRUE(results.empty());
}

TEST(SpatialIndexTest, InsertAndQueryEntity) {
    SpatialIndex index;
    auto line = std::make_shared<DraftLine>(Vec2(0, 0), Vec2(10, 10));
    index.insert(line);

    // Query a box that overlaps the line
    BoundingBox searchBox(Vec3(-1, -1, 0), Vec3(5, 5, 0));
    auto results = index.query(searchBox);
    ASSERT_EQ(results.size(), 1u);
    EXPECT_EQ(results[0], line->id());
}

TEST(SpatialIndexTest, InsertAndQueryMiss) {
    SpatialIndex index;
    auto line = std::make_shared<DraftLine>(Vec2(0, 0), Vec2(10, 10));
    index.insert(line);

    // Query a box that does NOT overlap the line
    BoundingBox searchBox(Vec3(50, 50, 0), Vec3(60, 60, 0));
    auto results = index.query(searchBox);
    EXPECT_TRUE(results.empty());
}

TEST(SpatialIndexTest, RemoveEntity) {
    SpatialIndex index;
    auto line = std::make_shared<DraftLine>(Vec2(0, 0), Vec2(10, 10));
    index.insert(line);
    index.remove(line->id());

    BoundingBox searchBox(Vec3(-1, -1, 0), Vec3(15, 15, 0));
    auto results = index.query(searchBox);
    EXPECT_TRUE(results.empty());
}

TEST(SpatialIndexTest, UpdateEntityPosition) {
    SpatialIndex index;
    auto line = std::make_shared<DraftLine>(Vec2(0, 0), Vec2(10, 10));
    index.insert(line);

    // Move the line far away and update the index
    line->setStart(Vec2(100, 100));
    line->setEnd(Vec2(110, 110));
    index.update(line);

    // Old position should miss
    BoundingBox oldBox(Vec3(-1, -1, 0), Vec3(15, 15, 0));
    EXPECT_TRUE(index.query(oldBox).empty());

    // New position should hit
    BoundingBox newBox(Vec3(99, 99, 0), Vec3(111, 111, 0));
    auto results = index.query(newBox);
    ASSERT_EQ(results.size(), 1u);
    EXPECT_EQ(results[0], line->id());
}

TEST(SpatialIndexTest, RebuildFromEntities) {
    SpatialIndex index;
    auto line = std::make_shared<DraftLine>(Vec2(0, 0), Vec2(10, 10));
    auto circle = std::make_shared<DraftCircle>(Vec2(50, 50), 5.0);

    std::vector<std::shared_ptr<DraftEntity>> entities{line, circle};
    index.rebuild(entities);

    // Query the whole world
    BoundingBox bigBox(Vec3(-100, -100, 0), Vec3(200, 200, 0));
    auto results = index.query(bigBox);
    EXPECT_EQ(results.size(), 2u);

    // Query only near the line
    BoundingBox lineBox(Vec3(-1, -1, 0), Vec3(11, 11, 0));
    results = index.query(lineBox);
    ASSERT_EQ(results.size(), 1u);
    EXPECT_EQ(results[0], line->id());

    // Query only near the circle
    BoundingBox circleBox(Vec3(44, 44, 0), Vec3(56, 56, 0));
    results = index.query(circleBox);
    ASSERT_EQ(results.size(), 1u);
    EXPECT_EQ(results[0], circle->id());
}

// --- DraftDocument integration tests ---

TEST(DraftDocumentSpatialTest, AddEntityUpdatesSpatialIndex) {
    DraftDocument doc;
    auto line = std::make_shared<DraftLine>(Vec2(0, 0), Vec2(10, 10));
    doc.addEntity(line);

    BoundingBox searchBox(Vec3(-1, -1, 0), Vec3(5, 5, 0));
    auto results = doc.spatialIndex().query(searchBox);
    ASSERT_EQ(results.size(), 1u);
    EXPECT_EQ(results[0], line->id());
}

TEST(DraftDocumentSpatialTest, RemoveEntityUpdatesSpatialIndex) {
    DraftDocument doc;
    auto line = std::make_shared<DraftLine>(Vec2(0, 0), Vec2(10, 10));
    uint64_t lineId = line->id();
    doc.addEntity(line);
    doc.removeEntity(lineId);

    BoundingBox searchBox(Vec3(-1, -1, 0), Vec3(15, 15, 0));
    auto results = doc.spatialIndex().query(searchBox);
    EXPECT_TRUE(results.empty());
}

TEST(DraftDocumentSpatialTest, ClearEmptiesSpatialIndex) {
    DraftDocument doc;
    auto line = std::make_shared<DraftLine>(Vec2(0, 0), Vec2(10, 10));
    auto circle = std::make_shared<DraftCircle>(Vec2(50, 50), 5.0);
    doc.addEntity(line);
    doc.addEntity(circle);
    doc.clear();

    BoundingBox searchBox(Vec3(-100, -100, 0), Vec3(200, 200, 0));
    auto results = doc.spatialIndex().query(searchBox);
    EXPECT_TRUE(results.empty());
}
