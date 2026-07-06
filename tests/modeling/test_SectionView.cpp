#include <gtest/gtest.h>

#include <cmath>

#include "horizon/math/Vec3.h"
#include "horizon/modeling/DrawingDimension.h"
#include "horizon/modeling/Pattern.h"
#include "horizon/modeling/PrimitiveFactory.h"
#include "horizon/modeling/SectionView.h"
#include "horizon/topology/Solid.h"

using hz::math::Vec2;
using hz::math::Vec3;
using hz::model::DrawingDimensioner;
using hz::model::DrawingView;
using hz::model::Pattern;
using hz::model::PrimitiveFactory;
using hz::model::RadialDimension;
using hz::model::SectionGenerator;

namespace {

double loopArea(const std::vector<Vec2>& loop) {
    double a = 0.0;
    for (size_t i = 0; i < loop.size(); ++i) {
        const Vec2& p = loop[i];
        const Vec2& q = loop[(i + 1) % loop.size()];
        a += p.x * q.y - q.x * p.y;
    }
    return std::abs(a) * 0.5;
}

}  // namespace

// ---------------------------------------------------------------------------
// Section views (Phase 61)
// ---------------------------------------------------------------------------

TEST(SectionViewTest, MidBoxSectionYieldsRectangularProfile) {
    auto box = PrimitiveFactory::makeBox(10.0, 6.0, 4.0);
    ASSERT_NE(box, nullptr);

    // Cut at x = 5 looking along -X: the profile is the 6×4 cross-section.
    DrawingView view =
        SectionGenerator::sectionView(*box, Vec3(5.0, 0.0, 0.0), Vec3(1.0, 0.0, 0.0));

    ASSERT_EQ(view.sectionLoops.size(), 1u);
    EXPECT_NEAR(loopArea(view.sectionLoops[0]), 6.0 * 4.0, 1e-6);

    // Hatching exists and stays inside the profile bounds.
    EXPECT_FALSE(view.sectionHatch.empty());
    for (const auto& seg : view.sectionHatch) {
        for (const Vec2& p : {seg.first, seg.second}) {
            EXPECT_GE(p.x, view.boundsMin.x - 1e-6);
            EXPECT_LE(p.x, view.boundsMax.x + 1e-6);
            EXPECT_GE(p.y, view.boundsMin.y - 1e-6);
            EXPECT_LE(p.y, view.boundsMax.y + 1e-6);
        }
    }

    // Retained outline: the half behind the plane projects visible-only.
    EXPECT_FALSE(view.edges.empty());
    for (const auto& e : view.edges) {
        EXPECT_EQ(e.visibility, hz::model::ProjectedEdge::Visibility::Visible);
    }
}

TEST(SectionViewTest, PlaneMissingTheSolidYieldsNoProfile) {
    auto box = PrimitiveFactory::makeBox(10.0, 6.0, 4.0);
    ASSERT_NE(box, nullptr);

    DrawingView view =
        SectionGenerator::sectionView(*box, Vec3(50.0, 0.0, 0.0), Vec3(1.0, 0.0, 0.0));
    EXPECT_TRUE(view.sectionLoops.empty());
    EXPECT_TRUE(view.sectionHatch.empty());
}

TEST(SectionViewTest, MultiShellSolidYieldsOneLoopPerShell) {
    auto box = PrimitiveFactory::makeBox(4.0, 4.0, 4.0);
    ASSERT_NE(box, nullptr);
    auto pattern = Pattern::linear(*box, Vec3(0.0, 0.0, 1.0), 8.0, 2);
    ASSERT_NE(pattern, nullptr);
    ASSERT_EQ(pattern->shellCount(), 2u);

    // A vertical plane cuts both stacked boxes: two separate profiles.
    DrawingView view =
        SectionGenerator::sectionView(*pattern, Vec3(2.0, 0.0, 0.0), Vec3(1.0, 0.0, 0.0));
    ASSERT_EQ(view.sectionLoops.size(), 2u);
    EXPECT_NEAR(loopArea(view.sectionLoops[0]), 16.0, 1e-6);
    EXPECT_NEAR(loopArea(view.sectionLoops[1]), 16.0, 1e-6);
}

TEST(SectionViewTest, AngledSectionProfileAreaMatchesAnalytic) {
    auto box = PrimitiveFactory::makeBox(10.0, 10.0, 10.0);
    ASSERT_NE(box, nullptr);

    // Cut through the body diagonal plane x = y: the profile is the diagonal
    // rectangle 10√2 × 10.
    const Vec3 n = Vec3(1.0, -1.0, 0.0).normalized();
    DrawingView view = SectionGenerator::sectionView(*box, Vec3(5.0, 5.0, 0.0), n);

    ASSERT_EQ(view.sectionLoops.size(), 1u);
    EXPECT_NEAR(loopArea(view.sectionLoops[0]), 10.0 * std::sqrt(2.0) * 10.0, 1e-6);
}

// ---------------------------------------------------------------------------
// Radial dimensions (Phase 61)
// ---------------------------------------------------------------------------

TEST(RadialDimensionTest, MeasuresCylinderRimRadius) {
    auto cyl = PrimitiveFactory::makeCylinder(5.0, 10.0);
    ASSERT_NE(cyl, nullptr);

    // Find a circular edge (a rim) and measure it.
    bool measured = false;
    for (const auto& e : cyl->edges()) {
        double r = 0.0;
        if (DrawingDimensioner::measureRadius(*cyl, e.topoId, r)) {
            EXPECT_NEAR(r, 5.0, 1e-6);
            measured = true;

            RadialDimension dim;
            ASSERT_TRUE(DrawingDimensioner::dimensionRadius(*cyl, e.topoId, true, dim));
            EXPECT_NEAR(dim.value, 5.0, 1e-6);
            EXPECT_TRUE(dim.diameter);
            EXPECT_TRUE(dim.edge == e.topoId);
            break;
        }
    }
    EXPECT_TRUE(measured) << "no circular edge found on the cylinder";
}

TEST(RadialDimensionTest, StraightEdgeIsNotCircular) {
    auto box = PrimitiveFactory::makeBox(10.0, 10.0, 10.0);
    ASSERT_NE(box, nullptr);

    double r = 0.0;
    EXPECT_FALSE(DrawingDimensioner::measureRadius(*box, box->edges().front().topoId, r));
}

TEST(RadialDimensionTest, MissingEdgeFails) {
    auto box = PrimitiveFactory::makeBox(10.0, 10.0, 10.0);
    double r = 0.0;
    EXPECT_FALSE(
        DrawingDimensioner::measureRadius(*box, hz::topo::TopologyID::make("no", "such"), r));
}
