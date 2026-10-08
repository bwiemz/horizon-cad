// Snapping reports what it landed on (Phase 110): a midpoint, a centre, a
// quadrant or where two entities cross, not "endpoint" for all of them. An
// object snap in reach beats the grid, and a filter keeps entities out.

#include <gtest/gtest.h>

#include <memory>

#include "horizon/drafting/BlockDefinition.h"
#include "horizon/drafting/DraftArc.h"
#include "horizon/drafting/DraftBlockRef.h"
#include "horizon/drafting/DraftCircle.h"
#include "horizon/drafting/DraftDocument.h"
#include "horizon/drafting/DraftLine.h"
#include "horizon/drafting/SnapEngine.h"
#include "horizon/math/Constants.h"

using hz::draft::DraftDocument;
using hz::draft::SnapEngine;
using hz::draft::SnapResult;
using hz::draft::SnapType;
using hz::math::Vec2;

namespace {

SnapEngine engine(double tolerance = 0.5) {
    SnapEngine e;
    e.setSnapTolerance(tolerance);
    e.setGridSpacing(1.0);
    return e;
}

bool at(const SnapResult& r, SnapType type, const Vec2& p) {
    return r.type == type && (r.point - p).length() < 1e-9;
}

}  // namespace

TEST(SnappingTest, EachSnapSaysWhatItLandedOn) {
    DraftDocument doc;
    doc.addEntity(std::make_shared<hz::draft::DraftLine>(Vec2(0, 0), Vec2(10, 0)));
    doc.addEntity(std::make_shared<hz::draft::DraftCircle>(Vec2(20, 0), 3.0));
    doc.addEntity(
        std::make_shared<hz::draft::DraftArc>(Vec2(40, 0), 2.0, 0.0, hz::math::kPi / 2.0));
    const SnapEngine snapper = engine();
    const auto snapAt = [&](double x, double y) {
        return snapper.snap(Vec2(x, y), doc.spatialIndex(), doc.entities());
    };
    EXPECT_TRUE(at(snapAt(0.1, 0.1), SnapType::Endpoint, Vec2(0, 0)));
    EXPECT_TRUE(at(snapAt(5.1, 0.1), SnapType::Midpoint, Vec2(5, 0)))
        << "a line's midpoint used to be no snap point at all";
    EXPECT_TRUE(at(snapAt(20.1, 0.1), SnapType::Center, Vec2(20, 0)));
    EXPECT_TRUE(at(snapAt(23.1, 0.1), SnapType::Quadrant, Vec2(23, 0)));
    EXPECT_TRUE(at(snapAt(40.1, 0.1), SnapType::Center, Vec2(40, 0)));
    EXPECT_TRUE(at(snapAt(42.1, 0.1), SnapType::Endpoint, Vec2(42, 0)));
}

TEST(SnappingTest, WhereTwoEntitiesCrossIsASnap) {
    DraftDocument doc;
    doc.addEntity(std::make_shared<hz::draft::DraftLine>(Vec2(0, 0), Vec2(10, 10)));
    doc.addEntity(std::make_shared<hz::draft::DraftLine>(Vec2(0, 6), Vec2(12, 0)));
    doc.addEntity(std::make_shared<hz::draft::DraftCircle>(Vec2(30, 0), 5.0));
    doc.addEntity(std::make_shared<hz::draft::DraftLine>(Vec2(20, 3), Vec2(40, 3)));
    const SnapEngine snapper = engine();
    EXPECT_TRUE(at(snapper.snap(Vec2(4.1, 3.9), doc.spatialIndex(), doc.entities()),
                   SnapType::Intersection, Vec2(4, 4)));
    // A line across a circle: (30 ± 4, 3).
    EXPECT_TRUE(at(snapper.snap(Vec2(34.1, 3.1), doc.spatialIndex(), doc.entities()),
                   SnapType::Intersection, Vec2(34, 3)));
}

TEST(SnappingTest, AnObjectSnapInReachBeatsANearerGridPoint) {
    DraftDocument doc;
    doc.addEntity(std::make_shared<hz::draft::DraftLine>(Vec2(0.3, 0.3), Vec2(5.3, 0.3)));
    // The grid point (0, 0) is nearer the cursor than the endpoint, but the
    // endpoint is what is being drawn to.
    EXPECT_TRUE(at(engine().snap(Vec2(0.1, 0.1), doc.spatialIndex(), doc.entities()),
                   SnapType::Endpoint, Vec2(0.3, 0.3)));
}

TEST(SnappingTest, AFilterKeepsEntitiesOut) {
    DraftDocument doc;
    auto line = std::make_shared<hz::draft::DraftLine>(Vec2(0.3, 0.3), Vec2(5.3, 0.3));
    line->setLayer("Hidden");
    doc.addEntity(line);
    const auto onlyLayer0 = [](const hz::draft::DraftEntity& e) { return e.layer() == "0"; };
    const SnapResult result =
        engine().snap(Vec2(0.1, 0.1), doc.spatialIndex(), doc.entities(), onlyLayer0);
    EXPECT_TRUE(at(result, SnapType::Grid, Vec2(0, 0))) << "only the grid is left";
    EXPECT_TRUE(
        at(engine().snap(Vec2(0.1, 0.1), doc.entities(), onlyLayer0), SnapType::Grid, Vec2(0, 0)));
}

TEST(SnappingTest, ABlocksContentSnapsWithItsKinds) {
    auto def = std::make_shared<hz::draft::BlockDefinition>();
    def->name = "Bolt";
    def->entities.push_back(std::make_shared<hz::draft::DraftCircle>(Vec2(0, 0), 1.0));
    DraftDocument doc;
    doc.addEntity(std::make_shared<hz::draft::DraftBlockRef>(def, Vec2(10, 0), 0.0, 2.0));
    const SnapEngine snapper = engine();
    EXPECT_TRUE(at(snapper.snap(Vec2(12.1, 0.1), doc.spatialIndex(), doc.entities()),
                   SnapType::Quadrant, Vec2(12, 0)));
}

// A point to snap to may lie outside what the entity covers: the centre of
// an arc short of a quarter, the insertion point of a block drawn away from
// it. The snap through the drawing's index found only entities whose box
// reached the cursor, so these went unseen; picking and box selection still
// go by what the entity covers.
TEST(SnappingTest, ASnapPointOutsideTheEntityIsFound) {
    auto def = std::make_shared<hz::draft::BlockDefinition>();
    def->name = "Offset";
    def->entities.push_back(std::make_shared<hz::draft::DraftCircle>(Vec2(10, 10), 1.0));
    DraftDocument doc;
    doc.addEntity(std::make_shared<hz::draft::DraftArc>(Vec2(40, 0), 2.0, hz::math::kPi / 6.0,
                                                        hz::math::kPi / 3.0));
    doc.addEntity(std::make_shared<hz::draft::DraftBlockRef>(def, Vec2(60, 0), 0.0, 1.0));
    const SnapEngine snapper = engine();

    EXPECT_TRUE(at(snapper.snap(Vec2(40.1, 0.1), doc.entities()), SnapType::Center, Vec2(40, 0)))
        << "the list sees it";
    EXPECT_TRUE(at(snapper.snap(Vec2(40.1, 0.1), doc.spatialIndex(), doc.entities()),
                   SnapType::Center, Vec2(40, 0)))
        << "and so does the index";
    EXPECT_TRUE(at(snapper.snap(Vec2(40.1, 0.1), doc), SnapType::Center, Vec2(40, 0)));
    EXPECT_TRUE(at(snapper.snap(Vec2(60.1, 0.1), doc), SnapType::Endpoint, Vec2(60, 0)))
        << "the block's insertion point";

    const hz::math::BoundingBox aroundCentre(hz::math::Vec3(39.5, -0.5, -1),
                                             hz::math::Vec3(40.5, 0.5, 1));
    EXPECT_TRUE(doc.spatialIndex().query(aroundCentre).empty())
        << "a box over the centre does not cross the arc";

    // Moved, it is found where it went, and not where it was.
    auto* arc = dynamic_cast<hz::draft::DraftArc*>(doc.entities()[0].get());
    arc->translate(Vec2(0, 20));
    doc.updateEntityBounds(arc->id());
    EXPECT_TRUE(at(snapper.snap(Vec2(40.1, 20.1), doc), SnapType::Center, Vec2(40, 20)));
    EXPECT_FALSE(at(snapper.snap(Vec2(40.1, 0.1), doc), SnapType::Center, Vec2(40, 0)));
    doc.removeEntity(arc->id());
    EXPECT_FALSE(at(snapper.snap(Vec2(40.1, 20.1), doc), SnapType::Center, Vec2(40, 20)));
}

// Object snaps and the grid snap can each be switched off (F3 and F9): the
// cursor is then taken where it is, not pulled to a point it was not aimed at.
TEST(SnappingTest, ObjectAndGridSnapsCanBeSwitchedOff) {
    DraftDocument doc;
    doc.addEntity(std::make_shared<hz::draft::DraftLine>(Vec2(0, 0), Vec2(10, 0)));
    SnapEngine snapper = engine();
    const auto snapAt = [&](double x, double y) { return snapper.snap(Vec2(x, y), doc); };

    snapper.setObjectSnapEnabled(false);
    EXPECT_TRUE(at(snapAt(0.1, 0.1), SnapType::Grid, Vec2(0, 0))) << "the grid still";
    snapper.setGridSnapEnabled(false);
    EXPECT_TRUE(at(snapAt(0.1, 0.1), SnapType::None, Vec2(0.1, 0.1))) << "nothing";
    snapper.setObjectSnapEnabled(true);
    EXPECT_TRUE(at(snapAt(0.1, 0.1), SnapType::Endpoint, Vec2(0, 0)));
    EXPECT_TRUE(at(snapAt(3.3, 2.2), SnapType::None, Vec2(3.3, 2.2))) << "no grid point either";
}
