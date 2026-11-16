// Profiles of many points (Phase 168): an outline traced from a scan, a DXF
// contour, a spline flattened finely. Each step from the sketch to the cut
// solid took time in the square of the point count, or worse, and memory with
// it: a profile of 50,000 points took minutes to extrude, and a prism of
// 30,000 sides could not be cut at all before memory ran out.

#include <gtest/gtest.h>

#include <chrono>
#include <cmath>
#include <memory>
#include <numbers>
#include <string>
#include <vector>

#include "../TimeLimits.h"
#include "horizon/drafting/DraftPolyline.h"
#include "horizon/drafting/SketchPlane.h"
#include "horizon/modeling/BooleanOp.h"
#include "horizon/modeling/BoundaryMesh.h"
#include "horizon/modeling/Extrude.h"
#include "horizon/modeling/MassProperties.h"
#include "horizon/modeling/PrimitiveFactory.h"
#include "horizon/topology/Solid.h"

using hz::math::Vec2;
using hz::math::Vec3;
using hz::model::BooleanOp;
using hz::model::BooleanType;
using hz::model::BoundaryMesh;
using hz::model::Extrude;
using hz::model::MassPropertiesCalculator;
using hz::model::NamingScheme;
using hz::model::PrimitiveFactory;

namespace {

/// A wavy ring of @p n points: bent one way and the other, as a traced
/// outline is, so it is not convex and its curvature passes through nothing.
std::vector<Vec2> wavyOutline(int n) {
    std::vector<Vec2> points;
    points.reserve(static_cast<size_t>(n));
    for (int i = 0; i < n; ++i) {
        const double t = 2.0 * std::numbers::pi * i / n;
        const double r = 10.0 + 0.5 * std::sin(7.0 * t);
        points.emplace_back(r * std::cos(t), r * std::sin(t));
    }
    return points;
}

double areaOf(const std::vector<Vec2>& loop) {
    double twice = 0.0;
    for (size_t i = 0; i < loop.size(); ++i) {
        const Vec2& a = loop[i];
        const Vec2& b = loop[(i + 1) % loop.size()];
        twice += a.x * b.y - b.x * a.y;
    }
    return 0.5 * twice;
}

std::unique_ptr<hz::topo::Solid> prism(const std::vector<Vec2>& outline, double height) {
    const auto profile = std::make_shared<hz::draft::DraftPolyline>(outline, true);
    const hz::draft::SketchPlane plane;
    std::string why;
    auto solid = Extrude::execute({profile}, plane, plane.normal(), height, "extrude_1", 32, 0.0,
                                  &why, NamingScheme::Stable);
    EXPECT_NE(solid, nullptr) << why;
    return solid;
}

double secondsSince(std::chrono::steady_clock::time_point start) {
    return std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count();
}

}  // namespace

// 50,000 points extruded: each step of building the prism looked for the
// half-edge it needed by walking the whole face, and the profile's checks
// compared every segment with every other. 221 s in a build with some
// optimisation; well under one now.
TEST(LargeProfileTest, FiftyThousandPointsExtrudeQuickly) {
    const auto outline = wavyOutline(50'000);
    const auto start = std::chrono::steady_clock::now();
    const auto solid = prism(outline, 5.0);
    const double seconds = secondsSince(start);
    ASSERT_NE(solid, nullptr);
    EXPECT_TRUE(solid->isValid()) << solid->validationReport();
    EXPECT_EQ(solid->faceCount(), 50'002u);
    EXPECT_NEAR(MassPropertiesCalculator::compute(*solid).volume, 5.0 * areaOf(outline), 1e-6);
#if HZ_TIME_LIMITS
#ifdef NDEBUG
    EXPECT_LT(seconds, 3.0);
#else
    EXPECT_LT(seconds, 30.0) << "unoptimized: a generous bound";
#endif
#else
    (void)seconds;
#endif
}

// A face of 30,000 corners is triangulated as it was, corner for corner, but
// each ear is tested against the points near it, not all of them: seconds,
// where it was minutes.
TEST(LargeProfileTest, ALargeFaceIsTriangulatedWhole) {
    const auto outline = wavyOutline(30'000);
    std::vector<Vec3> face;
    for (const Vec2& p : outline) face.emplace_back(p.x, p.y, 2.0);
    const auto start = std::chrono::steady_clock::now();
    const auto triangles = BoundaryMesh::triangulatePolygon(face);
    const double seconds = secondsSince(start);
    ASSERT_EQ(triangles.size(), face.size() - 2);
    double area = 0.0;
    for (const auto& t : triangles) {
        const double z = (t[1] - t[0]).cross(t[2] - t[0]).z;
        EXPECT_GT(z, 0.0) << "every triangle turns the face's way";
        area += 0.5 * z;
    }
    EXPECT_NEAR(area, areaOf(outline), 1e-9 * areaOf(outline));
#if HZ_TIME_LIMITS
#ifdef NDEBUG
    EXPECT_LT(seconds, 5.0);
#else
    EXPECT_LT(seconds, 60.0) << "unoptimized: a generous bound";
#endif
#else
    (void)seconds;
#endif
}

// A prism of 10,000 sides cut by a box. Each new node of the cut's tree
// checked whether what it was given was convex, at a cost in the square of
// its size, and the side faces' planes cut through every sliver of the
// triangulated cap: 291 s in a build with some optimisation, about two now.
TEST(LargeProfileTest, APrismOfTenThousandSidesIsCut) {
    const auto outline = wavyOutline(10'000);
    const auto solid = prism(outline, 5.0);
    ASSERT_NE(solid, nullptr);
    const auto box = PrimitiveFactory::makeBox(4.0, 4.0, 20.0);
    const auto start = std::chrono::steady_clock::now();
    std::string why;
    const auto cut =
        BooleanOp::execute(*solid, *box, BooleanType::Subtract, &why, NamingScheme::Stable);
    const double seconds = secondsSince(start);
    ASSERT_NE(cut, nullptr) << why;
    EXPECT_TRUE(cut->isValid()) << cut->validationReport();
    EXPECT_NEAR(MassPropertiesCalculator::compute(*cut).volume, 5.0 * areaOf(outline) - 80.0, 1e-6);
#if HZ_TIME_LIMITS
#ifdef NDEBUG
    EXPECT_LT(seconds, 6.0);
#else
    EXPECT_LT(seconds, 60.0) << "unoptimized: a generous bound";
#endif
#else
    (void)seconds;
#endif
}

// Finer still: 25,000 sides. Its facets turn so little near where the
// outline bends from one way to the other that the merged cap's outline ran
// straight through their corners, within the tolerance, and dropped them:
// the side faces met the cap in the middle of its edges, and the result did
// not sew. A corner another face keeps is kept.
TEST(LargeProfileTest, AFinelyFacetedPrismsCapMeetsItsSidesCornerToCorner) {
    const auto outline = wavyOutline(25'000);
    const auto solid = prism(outline, 5.0);
    ASSERT_NE(solid, nullptr);
    const auto box = PrimitiveFactory::makeBox(4.0, 4.0, 20.0);
    std::string why;
    const auto cut =
        BooleanOp::execute(*solid, *box, BooleanType::Subtract, &why, NamingScheme::Stable);
    ASSERT_NE(cut, nullptr) << why;
    EXPECT_TRUE(cut->isValid()) << cut->validationReport();
    EXPECT_TRUE(cut->checkManifold());
    EXPECT_NEAR(MassPropertiesCalculator::compute(*cut).volume, 5.0 * areaOf(outline) - 80.0, 1e-6);
}
