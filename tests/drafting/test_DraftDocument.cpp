#include <gtest/gtest.h>

#include <chrono>
#include <memory>
#include <vector>

#include "horizon/drafting/DraftCircle.h"
#include "horizon/drafting/DraftDocument.h"
#include "horizon/drafting/DraftLine.h"
#include "horizon/math/BoundingBox.h"

#if defined(__SANITIZE_THREAD__)
#define HZ_UNDER_TSAN 1
#elif defined(__has_feature)
#if __has_feature(thread_sanitizer)
#define HZ_UNDER_TSAN 1
#endif
#endif

using hz::draft::DraftCircle;
using hz::draft::DraftDocument;
using hz::draft::DraftEntity;
using hz::draft::DraftLine;
using hz::math::BoundingBox;
using hz::math::Vec2;
using hz::math::Vec3;

namespace {

std::shared_ptr<DraftLine> lineAt(double x, double y) {
    return std::make_shared<DraftLine>(Vec2(x, y), Vec2(x + 1, y));
}

std::vector<uint64_t> ids(const DraftDocument& d) {
    std::vector<uint64_t> out;
    for (const auto& e : d.entities()) out.push_back(e->id());
    return out;
}

BoundingBox around(double x, double y, double r = 0.1) {
    return BoundingBox(Vec3(x - r, y - r, -1), Vec3(x + r, y + r, 1));
}

}  // namespace

TEST(DraftDocumentTest, FindsEntitiesById) {
    DraftDocument d;
    auto a = lineAt(0, 0);
    auto b = lineAt(5, 0);
    d.addEntity(a);
    d.addEntity(b);
    EXPECT_EQ(d.findEntity(a->id()), a.get());
    EXPECT_EQ(d.sharedEntity(b->id()), b);
    EXPECT_EQ(d.findEntity(987654321), nullptr);

    d.removeEntity(a->id());
    EXPECT_EQ(d.findEntity(a->id()), nullptr);
    d.clear();
    EXPECT_EQ(d.findEntity(b->id()), nullptr);
}

// Removing an entity says where it stood, and inserting it there again
// restores the drawing order: undo used to put it at the end.
TEST(DraftDocumentTest, RemoveAndInsertKeepTheDrawingOrder) {
    DraftDocument d;
    std::vector<std::shared_ptr<DraftLine>> lines;
    for (int i = 0; i < 5; ++i) {
        lines.push_back(lineAt(3.0 * i, 0));
        d.addEntity(lines.back());
    }
    const auto before = ids(d);
    const size_t at = d.removeEntity(lines[2]->id());
    EXPECT_EQ(at, 2u);
    EXPECT_EQ(d.removeEntity(lines[2]->id()), DraftDocument::npos) << "already gone";
    d.insertEntity(at, lines[2]);
    EXPECT_EQ(ids(d), before);
    EXPECT_FALSE(d.spatialIndex().query(around(6.5, 0)).empty()) << "indexed again";

    d.insertEntity(1000, lineAt(99, 99));  // past the end appends
    EXPECT_EQ(d.entities().size(), 6u);
}

TEST(DraftDocumentTest, BatchRemovalRestoresEveryEntityInPlace) {
    DraftDocument d;
    std::vector<uint64_t> all;
    for (int i = 0; i < 10; ++i) {
        auto line = lineAt(2.0 * i, 0);
        all.push_back(line->id());
        d.addEntity(line);
    }
    const auto removed = d.removeEntities({all[0], all[3], all[4], all[9], 424242});
    ASSERT_EQ(removed.size(), 4u) << "an unknown id is skipped";
    EXPECT_EQ(removed[0].position, 0u);
    EXPECT_EQ(removed[3].position, 9u);
    EXPECT_EQ(d.entities().size(), 6u);
    EXPECT_EQ(d.findEntity(all[3]), nullptr);
    EXPECT_TRUE(d.spatialIndex().query(around(6.5, 0)).empty());

    d.restoreEntities(removed);
    EXPECT_EQ(ids(d), all);
    EXPECT_EQ(d.findEntity(all[3])->id(), all[3]);
    EXPECT_FALSE(d.spatialIndex().query(around(6.5, 0)).empty());
}

// A replacement takes the entity's place in the order and in both indexes.
TEST(DraftDocumentTest, ReplaceKeepsThePlaceAndTheIndexes) {
    DraftDocument d;
    auto a = lineAt(0, 0);
    auto b = lineAt(5, 0);
    auto c = lineAt(10, 0);
    d.addEntity(a);
    d.addEntity(b);
    d.addEntity(c);
    auto moved = std::make_shared<DraftLine>(Vec2(50, 50), Vec2(51, 50));
    moved->setId(b->id());
    ASSERT_TRUE(d.replaceEntity(b->id(), moved));
    EXPECT_EQ(d.entities()[1], moved);
    EXPECT_EQ(d.findEntity(b->id()), moved.get());
    EXPECT_TRUE(d.spatialIndex().query(around(5.5, 0)).empty()) << "not at the old place";
    EXPECT_FALSE(d.spatialIndex().query(around(50.5, 50)).empty()) << "at the new one";
    EXPECT_FALSE(d.replaceEntity(777777, moved));
}

// An entity changed in place is found where it is after updateEntityBounds.
TEST(DraftDocumentTest, BoundsUpdateMovesTheEntityInTheIndex) {
    DraftDocument d;
    auto circle = std::make_shared<DraftCircle>(Vec2(0, 0), 1.0);
    d.addEntity(circle);
    circle->translate(Vec2(100, 0));
    d.updateEntityBounds(circle->id());
    EXPECT_TRUE(d.spatialIndex().query(around(0, 1)).empty());
    EXPECT_FALSE(d.spatialIndex().query(around(100, 1)).empty());
    // Removal still finds it, although it moved after it was indexed.
    circle->translate(Vec2(0, 300));
    d.removeEntity(circle->id());
    EXPECT_EQ(d.spatialIndex().size(), 0u);
}

// Removing entities one at a time used to rebuild the whole spatial index
// each time: undoing a 20,000-entity import took minutes.
TEST(DraftDocumentTest, RemovingManyEntitiesOneByOneIsFast) {
#ifdef HZ_UNDER_TSAN
    GTEST_SKIP()
        << "a time limit means nothing under ThreadSanitizer, which runs code 5-15x slower";
#endif
    DraftDocument d;
    std::vector<uint64_t> added;
    for (int i = 0; i < 20000; ++i) {
        auto line = lineAt(static_cast<double>(i % 200) * 2.0, static_cast<double>(i / 200) * 2.0);
        added.push_back(line->id());
        d.addEntity(line);
    }
    const auto start = std::chrono::steady_clock::now();
    for (auto it = added.rbegin(); it != added.rend(); ++it) d.removeEntity(*it);  // as undo does
    const double ms =
        std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - start).count();
    EXPECT_TRUE(d.entities().empty());
    EXPECT_EQ(d.spatialIndex().size(), 0u);
#ifdef NDEBUG
    EXPECT_LT(ms, 500.0);
#else
    EXPECT_LT(ms, 10000.0);  // Debug and sanitizer builds; was minutes
#endif
}
