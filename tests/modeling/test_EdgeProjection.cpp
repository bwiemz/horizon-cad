// Phase 157: an edge of a part drawn straight onto a sketch plane.

#include <gtest/gtest.h>

#include <cmath>
#include <memory>
#include <numbers>
#include <string>
#include <vector>

#include "horizon/drafting/DraftArc.h"
#include "horizon/drafting/DraftCircle.h"
#include "horizon/drafting/DraftLine.h"
#include "horizon/drafting/DraftPolyline.h"
#include "horizon/drafting/SketchPlane.h"
#include "horizon/modeling/EdgeProjection.h"
#include "horizon/modeling/Extrude.h"
#include "horizon/modeling/Naming.h"
#include "horizon/modeling/PrimitiveFactory.h"

using hz::draft::SketchPlane;
using hz::math::Vec2;
using hz::math::Vec3;
using hz::model::projectEdge;

namespace {

/// The logical name of the first edge of @p solid both of whose ends pass
/// @p test, and whether it has an ideal curve if @p round.
template <typename Test>
std::string edgeWhere(const hz::topo::Solid& solid, Test test, bool round = false) {
    for (const auto& e : solid.edges()) {
        const auto* he = e.halfEdge;
        if (he == nullptr || he->origin == nullptr || he->next == nullptr ||
            he->next->origin == nullptr || !e.topoId.isValid()) {
            continue;
        }
        if (round != (e.analyticCurve != nullptr)) continue;
        if (test(he->origin->point) && test(he->next->origin->point)) {
            return hz::model::wholeEdgeName(e.topoId.tag());
        }
    }
    return {};
}

void expectNear(const Vec2& a, const Vec2& b, const char* what) {
    EXPECT_NEAR(a.x, b.x, 1e-9) << what;
    EXPECT_NEAR(a.y, b.y, 1e-9) << what;
}

/// A D: half a disc of radius 5 on the XY plane, its arc above the x axis,
/// extruded 10 up, named as features name it.
std::unique_ptr<hz::topo::Solid> dShape() {
    std::vector<std::shared_ptr<hz::draft::DraftEntity>> profile{
        std::make_shared<hz::draft::DraftArc>(Vec2(0, 0), 5.0, 0.0, std::numbers::pi),
        std::make_shared<hz::draft::DraftLine>(Vec2(-5, 0), Vec2(5, 0))};
    return hz::model::Extrude::execute(profile, SketchPlane(), Vec3::UnitZ, 10.0, "extrude_9",
                                       hz::model::Extrude::kDefaultSegments, 0.0, nullptr,
                                       hz::model::NamingScheme::Stable);
}

/// A disc of radius 5 about the z axis, 12 high, named as features name it:
/// each rim one curve in chords.
std::unique_ptr<hz::topo::Solid> disc() {
    std::vector<std::shared_ptr<hz::draft::DraftEntity>> profile{
        std::make_shared<hz::draft::DraftCircle>(Vec2(0, 0), 5.0)};
    return hz::model::Extrude::execute(profile, SketchPlane(), Vec3::UnitZ, 12.0, "extrude_8",
                                       hz::model::Extrude::kDefaultSegments, 0.0, nullptr,
                                       hz::model::NamingScheme::Stable);
}

}  // namespace

TEST(EdgeProjectionTest, AStraightEdgeIsALine) {
    const auto box = hz::model::PrimitiveFactory::makeBox(10, 20, 30);
    // The top's edge along x at y = 20.
    const std::string edge =
        edgeWhere(*box, [](const Vec3& p) { return p.z == 30.0 && p.y == 20.0; });
    ASSERT_FALSE(edge.empty());
    const auto entity = projectEdge(*box, edge, SketchPlane());
    const auto* line = dynamic_cast<const hz::draft::DraftLine*>(entity.get());
    ASSERT_NE(line, nullptr);
    EXPECT_NEAR(line->start().y, 20.0, 1e-12);
    EXPECT_NEAR(line->end().y, 20.0, 1e-12);
    EXPECT_NEAR(std::abs(line->end().x - line->start().x), 10.0, 1e-12);
    EXPECT_TRUE(entity->sourceEdge().empty()) << "its caller names the edge";
}

TEST(EdgeProjectionTest, NothingIsDrawnForAnEdgeGoneOrSeenEndOn) {
    const auto box = hz::model::PrimitiveFactory::makeBox(10, 20, 30);
    std::string why;
    EXPECT_EQ(projectEdge(*box, "box/nosuch", SketchPlane(), &why), nullptr);
    EXPECT_EQ(why, "is not there");
    // An upright edge, seen from above.
    const std::string upright =
        edgeWhere(*box, [](const Vec3& p) { return p.x == 0.0 && p.y == 0.0; });
    ASSERT_FALSE(upright.empty());
    EXPECT_EQ(projectEdge(*box, upright, SketchPlane(), &why), nullptr);
    EXPECT_EQ(why, "is seen end on");
}

TEST(EdgeProjectionTest, ARimIsACircleSeenSquareOnAndALineSeenEdgeOn) {
    const auto cylinder = disc();
    ASSERT_NE(cylinder, nullptr);
    const std::string rim = edgeWhere(
        *cylinder, [](const Vec3& p) { return std::abs(p.z - 12.0) < 1e-12; }, true);
    ASSERT_FALSE(rim.empty());

    const auto above =
        projectEdge(*cylinder, rim, SketchPlane(Vec3(1, 2, 40), Vec3::UnitZ, Vec3::UnitX));
    const auto* circle = dynamic_cast<const hz::draft::DraftCircle*>(above.get());
    ASSERT_NE(circle, nullptr) << "the whole rim, round";
    expectNear(circle->center(), Vec2(-1, -2), "about the axis, in the plane's own terms");
    EXPECT_NEAR(circle->radius(), 5.0, 1e-9);

    // From the front (the XZ plane): the rim is a line across the top.
    const auto front =
        projectEdge(*cylinder, rim, SketchPlane(Vec3::Zero, Vec3(0, -1, 0), Vec3::UnitX));
    const auto* line = dynamic_cast<const hz::draft::DraftLine*>(front.get());
    ASSERT_NE(line, nullptr) << "seen edge on";
    EXPECT_NEAR(std::abs(line->end().x - line->start().x), 10.0, 1e-9);
    EXPECT_NEAR(line->start().y, 12.0, 1e-9);

    // At a slant: an ellipse, drawn through the chords' ends.
    const double c = std::cos(std::numbers::pi / 6.0);
    const double s = std::sin(std::numbers::pi / 6.0);
    const auto slant =
        projectEdge(*cylinder, rim, SketchPlane(Vec3::Zero, Vec3(0, -s, c), Vec3::UnitX));
    const auto* polyline = dynamic_cast<const hz::draft::DraftPolyline*>(slant.get());
    ASSERT_NE(polyline, nullptr);
    EXPECT_TRUE(polyline->closed());
}

TEST(EdgeProjectionTest, AnArcRunsCounterclockwiseFromItsStart) {
    const auto d = dShape();
    ASSERT_NE(d, nullptr);
    const std::string top = edgeWhere(
        *d, [](const Vec3& p) { return std::abs(p.z - 10.0) < 1e-12; }, true);
    ASSERT_FALSE(top.empty());
    const auto entity = projectEdge(*d, top, SketchPlane());
    const auto* arc = dynamic_cast<const hz::draft::DraftArc*>(entity.get());
    ASSERT_NE(arc, nullptr) << top;
    expectNear(arc->center(), Vec2(0, 0), "centre");
    EXPECT_NEAR(arc->radius(), 5.0, 1e-9);
    expectNear(arc->startPoint(), Vec2(5, 0), "from the right");
    expectNear(arc->endPoint(), Vec2(-5, 0), "over the top to the left");

    // Seen from below (the plane facing down, so its y axis is the world's
    // -y), the arc is below the plane's x axis, and still counterclockwise
    // from its start: its middle is where the arc is.
    const auto below = projectEdge(*d, top, SketchPlane(Vec3::Zero, Vec3(0, 0, -1), Vec3::UnitX));
    const auto* flipped = dynamic_cast<const hz::draft::DraftArc*>(below.get());
    ASSERT_NE(flipped, nullptr);
    double sweep = flipped->endAngle() - flipped->startAngle();
    if (sweep <= 0.0) sweep += 2.0 * std::numbers::pi;
    EXPECT_NEAR(sweep, std::numbers::pi, 1e-9) << "half a turn, not the other half";
    const double middle = flipped->startAngle() + sweep / 2.0;
    EXPECT_NEAR(flipped->center().y + flipped->radius() * std::sin(middle), -5.0, 1e-9);
}
