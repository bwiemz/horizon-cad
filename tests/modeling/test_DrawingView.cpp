#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <vector>

#include "horizon/math/Mat4.h"
#include "horizon/math/Vec2.h"
#include "horizon/math/Vec3.h"
#include "horizon/modeling/BooleanOp.h"
#include "horizon/modeling/DrawingProjection.h"
#include "horizon/modeling/DrawingView.h"
#include "horizon/modeling/FilletOp.h"
#include "horizon/modeling/Pattern.h"
#include "horizon/modeling/PrimitiveFactory.h"
#include "horizon/topology/Solid.h"

using hz::math::Vec2;
using hz::math::Vec3;
using hz::model::Drawing;
using hz::model::DrawingGenerator;
using hz::model::DrawingView;
using hz::model::PrimitiveFactory;
using hz::model::StandardView;
using hz::model::ViewProjection;

namespace {

// Placed axis-aligned bounds of a view: [placement, placement + size].
struct Box2 {
    double x0, y0, x1, y1;
};

Box2 placedBox(const DrawingView& v) {
    return {v.placement.x, v.placement.y, v.placement.x + v.width(), v.placement.y + v.height()};
}

bool overlaps(const Box2& a, const Box2& b) {
    return a.x0 < b.x1 && b.x0 < a.x1 && a.y0 < b.y1 && b.y0 < a.y1;
}

}  // namespace

TEST(DrawingViewTest, MakeViewComputesBounds) {
    auto box = PrimitiveFactory::makeBox(4.0, 3.0, 2.0);
    DrawingView v = DrawingGenerator::makeView(*box, StandardView::Front);
    ASSERT_FALSE(v.edges.empty());
    // Front view (look -Y) shows the XZ face: width = X extent, height = Z extent.
    EXPECT_NEAR(v.width(), 4.0, 1e-6);
    EXPECT_NEAR(v.height(), 2.0, 1e-6);
}

TEST(DrawingViewTest, StandardViewsProducesFourNonEmptyViews) {
    auto box = PrimitiveFactory::makeBox(2.0, 2.0, 2.0);
    Drawing d = DrawingGenerator::standardViews(*box);
    ASSERT_EQ(d.views.size(), 4u);

    EXPECT_EQ(d.views[0].kind, StandardView::Front);
    EXPECT_EQ(d.views[1].kind, StandardView::Top);
    EXPECT_EQ(d.views[2].kind, StandardView::Right);
    EXPECT_EQ(d.views[3].kind, StandardView::Isometric);

    for (const auto& v : d.views) {
        EXPECT_FALSE(v.edges.empty());
        EXPECT_GT(v.width(), 0.0);
        EXPECT_GT(v.height(), 0.0);
    }
}

TEST(DrawingViewTest, StandardViewsAreLaidOutWithoutOverlap) {
    auto box = PrimitiveFactory::makeBox(3.0, 2.0, 4.0);
    Drawing d = DrawingGenerator::standardViews(*box, 10.0);
    ASSERT_EQ(d.views.size(), 4u);

    // Every pair of placed views must be disjoint on the sheet.
    for (size_t i = 0; i < d.views.size(); ++i) {
        for (size_t j = i + 1; j < d.views.size(); ++j) {
            EXPECT_FALSE(overlaps(placedBox(d.views[i]), placedBox(d.views[j])))
                << "views " << i << " and " << j << " overlap";
        }
    }
}

TEST(DrawingViewTest, MakeViewFromArbitraryProjection) {
    auto box = PrimitiveFactory::makeBox(4.0, 3.0, 2.0);
    ViewProjection proj;
    proj.dir = Vec3(-1.0, -1.0, -1.0);  // an isometric-ish direction
    proj.up = Vec3(0.0, 0.0, 1.0);

    DrawingView v = DrawingGenerator::makeView(*box, proj);
    EXPECT_FALSE(v.edges.empty());
    EXPECT_GT(v.width(), 0.0);
    EXPECT_GT(v.height(), 0.0);
    // The view records the camera it was projected through.
    EXPECT_DOUBLE_EQ(v.projection.dir.x, -1.0);
    EXPECT_DOUBLE_EQ(v.projection.dir.z, -1.0);
}

TEST(DrawingViewTest, AuxiliaryViewMatchesEquivalentStandardView) {
    // Looking square at the +X face (outward normal +X) means viewing along -X,
    // which is exactly the Right standard view — so the two agree edge-for-edge.
    auto box = PrimitiveFactory::makeBox(4.0, 3.0, 2.0);

    DrawingView aux = DrawingGenerator::auxiliaryView(*box, Vec3(1.0, 0.0, 0.0));
    DrawingView right = DrawingGenerator::makeView(*box, StandardView::Right);

    EXPECT_DOUBLE_EQ(aux.projection.dir.x, -1.0);  // opposite the outward normal
    EXPECT_EQ(aux.edges.size(), right.edges.size());
    EXPECT_NEAR(aux.width(), right.width(), 1e-9);
    EXPECT_NEAR(aux.height(), right.height(), 1e-9);
}

TEST(DrawingViewTest, DetailViewCropsToCircleAndEnlarges) {
    auto box = PrimitiveFactory::makeBox(4.0, 3.0, 2.0);
    DrawingView front = DrawingGenerator::makeView(*box, StandardView::Front);
    ASSERT_FALSE(front.edges.empty());

    // Detail around a corner of the front view.
    const Vec2 corner = front.boundsMin;
    const double radius = 1.0;
    const double scale = 3.0;
    DrawingView detail = DrawingGenerator::detailView(front, corner, radius, scale);

    ASSERT_FALSE(detail.edges.empty());  // the corner has geometry within the circle
    // Cropped geometry lies within `radius` of the corner, then is enlarged
    // `scale`x about it — so every kept point is within radius*scale of the
    // corner.
    const double maxDist = radius * scale + 1e-6;
    for (const auto& e : detail.edges) {
        for (const Vec2& p : {e.a, e.b}) {
            const double dx = p.x - corner.x;
            const double dy = p.y - corner.y;
            EXPECT_LE(std::sqrt(dx * dx + dy * dy), maxDist);
        }
    }
}

TEST(DrawingViewTest, DetailViewEmptyWhenCircleMissesGeometry) {
    auto box = PrimitiveFactory::makeBox(4.0, 3.0, 2.0);
    DrawingView front = DrawingGenerator::makeView(*box, StandardView::Front);
    DrawingView detail = DrawingGenerator::detailView(front, Vec2(1000.0, 1000.0), 1.0, 2.0);
    EXPECT_TRUE(detail.edges.empty());
}

// ---------------------------------------------------------------------------
// A drawing sheet (Phase 148): views laid out on a sheet at a standard scale.
// ---------------------------------------------------------------------------

namespace {

struct SheetBox {
    double x0, y0, x1, y1;
};

SheetBox onSheet(const hz::model::DrawingView& v) {
    const Vec2 lo = v.toSheet(v.boundsMin);
    return {lo.x, lo.y, lo.x + v.sheetWidth(), lo.y + v.sheetHeight()};
}

bool overlap(const SheetBox& a, const SheetBox& b) {
    return a.x0 < b.x1 - 1e-9 && b.x0 < a.x1 - 1e-9 && a.y0 < b.y1 - 1e-9 && b.y0 < a.y1 - 1e-9;
}

}  // namespace

// Four views on an A3 sheet at the largest standard scale they fit at: inside
// the border, above the title block, apart from each other, with Top over
// Front and Right beside it, aligned as third-angle projection has them.
TEST(DrawingViewTest, TheSheetLayoutFitsAtAStandardScale) {
    auto box = PrimitiveFactory::makeBox(100.0, 50.0, 20.0);
    hz::model::Sheet sheet;  // A3 landscape, 10 mm margin
    hz::model::TitleBlock tb;
    double scale = 0.0;
    const auto d = DrawingGenerator::sheetLayout(*box, sheet, tb, 10.0, &scale);
    ASSERT_EQ(d.views.size(), 4u);
    EXPECT_DOUBLE_EQ(scale, 1.0) << "2:1 is too wide for A3; 1:1 fits";
    for (const auto& v : d.views) {
        EXPECT_DOUBLE_EQ(v.scale, scale);
        EXPECT_FALSE(v.showTangentEdges) << "a drawing leaves tangent edges out";
        const SheetBox b = onSheet(v);
        EXPECT_GE(b.x0, sheet.margin);
        EXPECT_LE(b.x1, sheet.widthMm() - sheet.margin);
        EXPECT_GE(b.y0, sheet.margin + tb.height) << "above the title block";
        EXPECT_LE(b.y1, sheet.heightMm() - sheet.margin);
    }
    for (size_t i = 0; i < d.views.size(); ++i) {
        for (size_t j = i + 1; j < d.views.size(); ++j) {
            EXPECT_FALSE(overlap(onSheet(d.views[i]), onSheet(d.views[j]))) << i << " and " << j;
        }
    }
    const auto& front = d.views[0];
    const auto& top = d.views[1];
    const auto& right = d.views[2];
    EXPECT_NEAR(front.toSheet(front.boundsMin).x, top.toSheet(top.boundsMin).x, 1e-9)
        << "Top above Front, x aligned";
    EXPECT_NEAR(front.toSheet(front.boundsMin).y, right.toSheet(right.boundsMin).y, 1e-9)
        << "Right beside Front, z aligned";
}

// A large part is drawn smaller, a small one larger, at standard scales.
TEST(DrawingViewTest, TheSheetScaleFollowsThePartsSize) {
    hz::model::Sheet sheet;
    hz::model::TitleBlock tb;
    double scale = 0.0;
    auto big = PrimitiveFactory::makeBox(2000.0, 1000.0, 400.0);
    DrawingGenerator::sheetLayout(*big, sheet, tb, 10.0, &scale);
    EXPECT_LE(scale, 0.1);
    auto small = PrimitiveFactory::makeBox(5.0, 3.0, 2.0);
    DrawingGenerator::sheetLayout(*small, sheet, tb, 10.0, &scale);
    EXPECT_DOUBLE_EQ(scale, 10.0);
    EXPECT_EQ(DrawingGenerator::scaleName(0.5), "1:2");
    EXPECT_EQ(DrawingGenerator::scaleName(0.01), "1:100");
    EXPECT_EQ(DrawingGenerator::scaleName(1.0), "1:1");
    EXPECT_EQ(DrawingGenerator::scaleName(5.0), "5:1");
}

// A scale the user chose is used as chosen, even where the views do not fit:
// the sheet says so by what it shows, not by choosing again.
TEST(DrawingViewTest, AChosenSheetScaleIsKept) {
    hz::model::Sheet sheet;
    hz::model::TitleBlock tb;
    auto box = PrimitiveFactory::makeBox(100.0, 50.0, 20.0);
    double scale = 0.0;
    const auto d = DrawingGenerator::sheetLayout(*box, sheet, tb, 10.0, &scale, 5.0);
    EXPECT_DOUBLE_EQ(scale, 5.0);
    ASSERT_EQ(d.views.size(), 4u);
    for (const auto& v : d.views) EXPECT_DOUBLE_EQ(v.scale, 5.0);
    DrawingGenerator::sheetLayout(*box, sheet, tb, 10.0, &scale, 0.0);
    EXPECT_DOUBLE_EQ(scale, 1.0) << "none chosen: the largest that fits";
}

// A view added to a sheet goes where there is room: inside the border, clear
// of the title block, and the gap from every view and caption. One too large
// for any room goes beside the sheet, and says so.
TEST(DrawingViewTest, AnAddedViewIsPlacedWhereThereIsRoom) {
    auto box = PrimitiveFactory::makeBox(100.0, 50.0, 20.0);
    hz::model::Sheet sheet;  // A3 landscape
    hz::model::TitleBlock tb;
    hz::model::Drawing d = DrawingGenerator::sheetLayout(*box, sheet, tb);
    hz::model::DrawingView added;
    added.label = "A";
    added.boundsMax = {30.0, 30.0};
    const double gap = 10.0;
    bool fits = false;
    added.placement = DrawingGenerator::freePlacement(d, added, sheet, tb, gap, &fits);
    ASSERT_TRUE(fits);
    const auto [low, high] = added.sheetFootprint();
    EXPECT_NEAR(high.y - low.y, 30.0 + hz::model::DrawingView::kCaptionRoom, 1e-9)
        << "its caption's room is below it";
    EXPECT_GE(low.x, sheet.margin + gap - 1e-9);
    EXPECT_GE(low.y, sheet.margin + gap - 1e-9);
    EXPECT_LE(high.x, sheet.widthMm() - sheet.margin - gap + 1e-9);
    EXPECT_LE(high.y, sheet.heightMm() - sheet.margin - gap + 1e-9);
    const double tbLeft = sheet.widthMm() - sheet.margin - tb.width;
    const double tbTop = sheet.margin + tb.height;
    EXPECT_TRUE(high.x <= tbLeft - gap + 1e-9 || low.y >= tbTop + gap - 1e-9)
        << "clear of the title block";
    for (const auto& v : d.views) {
        const auto [vl, vh] = v.sheetFootprint();
        const bool apart = high.x <= vl.x - gap + 1e-9 || vh.x + gap <= low.x + 1e-9 ||
                           high.y <= vl.y - gap + 1e-9 || vh.y + gap <= low.y + 1e-9;
        EXPECT_TRUE(apart);
    }

    // Placed, it is taken: the next goes elsewhere.
    d.views.push_back(added);
    hz::model::DrawingView next = added;
    next.placement = DrawingGenerator::freePlacement(d, next, sheet, tb, gap, &fits);
    EXPECT_TRUE(fits);
    EXPECT_FALSE(next.placement.x == added.placement.x && next.placement.y == added.placement.y);

    hz::model::DrawingView huge;
    huge.boundsMax = {1000.0, 1000.0};
    huge.placement = DrawingGenerator::freePlacement(d, huge, sheet, tb, gap, &fits);
    EXPECT_FALSE(fits);
    EXPECT_GT(huge.placement.x, sheet.widthMm()) << "beside the sheet";
}

namespace {

/// The lengths of @p view's centre lines, shortest first.
std::vector<double> centreLineLengths(const DrawingView& view) {
    std::vector<double> lengths;
    for (const auto& [a, b] : view.centreLines) lengths.push_back((b - a).length());
    std::sort(lengths.begin(), lengths.end());
    return lengths;
}

}  // namespace

// A cylinder seen end-on has a cross on its centre, its arms to the
// outline; seen side-on, its axis end to end; at an angle, none.
TEST(DrawingViewTest, ACylinderHasCentreLines) {
    auto cyl = PrimitiveFactory::makeCylinder(10.0, 30.0, 32);  // about Z, from z = 0
    const DrawingView top = DrawingGenerator::makeView(*cyl, StandardView::Top);
    ASSERT_EQ(top.centreLines.size(), 2u) << "a cross";
    for (const double length : centreLineLengths(top)) EXPECT_NEAR(length, 20.0, 1e-9);
    const Vec2 centre = hz::model::DrawingProjection::toView(top.projection, Vec3(0, 0, 0));
    for (const auto& [a, b] : top.centreLines) {
        EXPECT_NEAR((a.x + b.x) / 2.0, centre.x, 1e-9);
        EXPECT_NEAR((a.y + b.y) / 2.0, centre.y, 1e-9);
    }

    const DrawingView front = DrawingGenerator::makeView(*cyl, StandardView::Front);
    ASSERT_EQ(front.centreLines.size(), 1u) << "its axis";
    EXPECT_NEAR(centreLineLengths(front)[0], 30.0, 1e-9);

    const DrawingView iso = DrawingGenerator::makeView(*cyl, StandardView::Isometric);
    EXPECT_TRUE(iso.centreLines.empty()) << "seen at an angle: an ellipse, not marked";

    // A detail keeps what of them is inside its circle.
    const DrawingView detail = DrawingGenerator::detailView(top, centre, 4.0, 1.0);
    ASSERT_EQ(detail.centreLines.size(), 2u);
    for (const double length : centreLineLengths(detail)) EXPECT_NEAR(length, 8.0, 1e-9);
}

// A hole through a block is marked; the block's straight faces are not.
TEST(DrawingViewTest, AHoleThroughABlockHasACentreMark) {
    auto block = PrimitiveFactory::makeBox(40.0, 40.0, 10.0);
    auto pin = PrimitiveFactory::makeCylinder(5.0, 30.0, 32);
    pin = hz::model::Pattern::transformed(*pin, hz::math::Mat4::translation(Vec3(20, 20, -10)));
    auto holed = hz::model::BooleanOp::execute(*block, *pin, hz::model::BooleanType::Subtract);
    ASSERT_NE(holed, nullptr);
    const DrawingView top = DrawingGenerator::makeView(*holed, StandardView::Top);
    ASSERT_EQ(top.centreLines.size(), 2u);
    for (const double length : centreLineLengths(top)) EXPECT_NEAR(length, 10.0, 1e-9);
    const DrawingView front = DrawingGenerator::makeView(*holed, StandardView::Front);
    ASSERT_EQ(front.centreLines.size(), 1u);
    // Its axis is fitted to the hole's facets: good to 1e-5, not exact.
    EXPECT_NEAR(centreLineLengths(front)[0], 10.0, 1e-4) << "through the block, not past it";
}

// A fillet goes a quarter of the way round: not a hole, and not marked, in
// any view.
TEST(DrawingViewTest, AFilletHasNoCentreLines) {
    auto box = PrimitiveFactory::makeBox(10, 10, 10);
    auto rounded = hz::model::FilletOp::execute(*box, {box->edges().front().topoId}, 2.0, "f");
    ASSERT_NE(rounded.solid, nullptr) << rounded.errorMessage;
    for (const StandardView v : {StandardView::Front, StandardView::Top, StandardView::Right}) {
        EXPECT_TRUE(DrawingGenerator::makeView(*rounded.solid, v).centreLines.empty());
    }
}
