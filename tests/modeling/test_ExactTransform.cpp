// Exact solids moved by any affine matrix (exact kernel, stage 3). A similarity moves each exact
// surface and curve as it is; any other matrix (a non-uniform scale, a shear) moves the NURBS each
// one is, refits the pcurves and grows the tolerances as the STEP reader does, or, where it
// cannot, leaves the copy without exact geometry and says why. A mirror reverses every loop, and
// a loop through a cone's apex then wraps round the cone at the apex.

#include <gtest/gtest.h>

#include <cmath>
#include <memory>
#include <numbers>
#include <string>
#include <vector>

#include "StepTestText.h"
#include "horizon/fileio/StepFormat.h"
#include "horizon/geometry/Frame.h"
#include "horizon/math/Mat4.h"
#include "horizon/math/Vec3.h"
#include "horizon/modeling/ExactPrimitives.h"
#include "horizon/modeling/MassProperties.h"
#include "horizon/modeling/Pattern.h"
#include "horizon/topology/ExactTolerance.h"
#include "horizon/topology/ExactValidator.h"
#include "horizon/topology/HalfEdge.h"
#include "horizon/topology/Solid.h"

namespace {

namespace geo = hz::geo;
namespace topo = hz::topo;
using hz::math::Mat4;
using hz::math::Vec3;
using hz::model::MassProperties;
using hz::model::MassPropertiesCalculator;
using hz::model::Pattern;

constexpr double kPi = std::numbers::pi;

/// The one solid @p text holds, read in millimetres (these files name no unit).
std::unique_ptr<topo::Solid> readOne(const std::string& text) {
    hz::io::StepReadOptions options;
    options.unknownLengthUnit = hz::math::LengthUnit::Millimetre;
    auto solids = hz::io::StepFormat::fromString(text, nullptr, nullptr, options);
    EXPECT_EQ(solids.size(), 1u) << hz::io::StepFormat::lastError();
    if (solids.size() != 1) return nullptr;
    return std::move(solids.front());
}

/// validateExact's first issue, or empty.
std::string firstIssue(const topo::Solid& solid) {
    const auto issues = topo::validateExact(solid, topo::exactTolerance(solid));
    return issues.empty() ? std::string() : issues.front().detail;
}

/// A shear: x moves by half of y.
Mat4 shear() {
    Mat4 m = Mat4::identity();
    m.m[0][1] = 0.5;
    return m;
}

}  // namespace

// An exact cylinder stretched to twice its width along x is an elliptic cylinder: still exact,
// valid, and its volume is the cylinder's times the matrix's determinant, to 1e-9. So with a
// shear (determinant 1), and with the stretch mirrored too.
TEST(ExactTransform, ANonUniformScaleKeepsTheSolidExact) {
    const double r = 5.0;
    const double h = 10.0;
    const auto cyl =
        hz::model::exact::makeCylinder(geo::Frame::fromAxis({1, 2, 3}, {0, 0, 1}), r, h);
    ASSERT_NE(cyl, nullptr);
    const double volume = kPi * r * r * h;
    const struct {
        const char* what;
        Mat4 m;
        double det;
    } cases[] = {{"scale (2, 1, 1)", Mat4::scale({2, 1, 1}), 2.0},
                 {"a shear", shear(), 1.0},
                 {"scale (-2, 1, 1)", Mat4::scale({-2, 1, 1}), 2.0}};
    for (const auto& c : cases) {
        SCOPED_TRACE(c.what);
        std::string problem;
        const auto copy = Pattern::transformed(*cyl, c.m, &problem);
        ASSERT_NE(copy, nullptr);
        EXPECT_TRUE(problem.empty()) << problem;
        EXPECT_TRUE(topo::hasExactFaces(*copy));
        EXPECT_EQ(firstIssue(*copy), "");
        const MassProperties props = MassPropertiesCalculator::compute(*copy);
        ASSERT_TRUE(props.valid) << props.problem;
        EXPECT_NEAR(props.volume, c.det * volume, 1e-9 * c.det * volume);
        // The overload that does not ask why makes the same copy.
        const auto same = Pattern::transformed(*cyl, c.m);
        EXPECT_TRUE(topo::hasExactFaces(*same));
    }
}

// A block sheared and stretched stays exact too: its planes are parallelograms.
TEST(ExactTransform, ANonUniformScaleKeepsABlockExact) {
    const auto block = hz::model::exact::makeBlock({0, 0, 0}, {10, 20, 30});
    ASSERT_NE(block, nullptr);
    for (const Mat4& m : {Mat4::scale({2, 1, 1}), shear()}) {
        std::string problem;
        const auto copy = Pattern::transformed(*block, m, &problem);
        EXPECT_TRUE(problem.empty()) << problem;
        EXPECT_EQ(firstIssue(*copy), "");
        const MassProperties props = MassPropertiesCalculator::compute(*copy);
        ASSERT_TRUE(props.valid) << props.problem;
        EXPECT_NEAR(props.volume, 6000.0 * m.determinant3(), 1e-9 * 12000.0);
    }
}

// Stretched, as read from files: a ball (its one face bounded by a seam from pole to pole and back)
// is an ellipsoid, a ring torus an elliptic one, a cone to its apex an elliptic cone, and a block
// with a ball-shaped void keeps its void. Each copy is exact and measures as the original times
// the determinant, to 1e-9: its refitted pcurves keep the periods the original loops had, which
// only the joins at a pole or apex cannot say.
TEST(ExactTransform, StretchedPolesAndApexesStayExact) {
    const struct {
        const char* what;
        std::string text;
    } cases[] = {{"the ball", hz::step_text::sphere()},
                 {"the torus", hz::step_text::torus()},
                 {"the apex cone", hz::step_text::apexCone()},
                 {"the block with a ball-shaped void", hz::step_text::blockWithABall()}};
    for (const auto& c : cases) {
        SCOPED_TRACE(c.what);
        const auto solid = readOne(c.text);
        ASSERT_NE(solid, nullptr);
        const MassProperties original = MassPropertiesCalculator::compute(*solid);
        ASSERT_TRUE(original.valid) << original.problem;
        for (const Mat4& m :
             {Mat4::scale({2, 1, 1}), Mat4::scale({1, 1, 3}), Mat4::scale({-2, 1, 1})}) {
            SCOPED_TRACE(m.determinant3());
            std::string problem;
            const auto copy = Pattern::transformed(*solid, m, &problem);
            EXPECT_TRUE(problem.empty()) << problem;
            ASSERT_TRUE(topo::hasExactFaces(*copy));
            EXPECT_EQ(firstIssue(*copy), "");
            const MassProperties props = MassPropertiesCalculator::compute(*copy);
            ASSERT_TRUE(props.valid) << props.problem;
            const double volume = original.volume * std::abs(m.determinant3());
            EXPECT_NEAR(props.volume, volume, 1e-9 * volume);
        }
    }
}

// Each tolerance scales by the most the matrix stretches a length (its largest singular value),
// as a similarity's scale them by its scale: a shear of a half stretches by 1.2808.
TEST(ExactTransform, TolerancesScaleByTheMostTheMatrixStretches) {
    auto block = hz::model::exact::makeBlock({0, 0, 0}, {10, 20, 30});
    ASSERT_NE(block, nullptr);
    for (auto& e : block->edges()) e.tolerance = 0.25;
    for (auto& v : block->vertices()) v.tolerance = 0.5;
    const double stretch = std::sqrt((2.25 + std::sqrt(2.25 * 2.25 - 4.0)) / 2.0);
    std::string problem;
    const auto copy = Pattern::transformed(*block, shear(), &problem);
    ASSERT_TRUE(problem.empty()) << problem;
    for (const auto& e : copy->edges()) EXPECT_NEAR(e.tolerance, 0.25 * stretch, 1e-12);
    for (const auto& v : copy->vertices()) EXPECT_NEAR(v.tolerance, 0.5 * stretch, 1e-12);
}

// A matrix that flattens the solid leaves no exact geometry on the copy, and says why.
TEST(ExactTransform, ACopyThatCannotStayExactSaysWhy) {
    const auto cyl =
        hz::model::exact::makeCylinder(geo::Frame::fromAxis({0, 0, 0}, {0, 0, 1}), 5.0, 10.0);
    std::string problem;
    const auto flat = Pattern::transformed(*cyl, Mat4::scale({1, 1, 0}), &problem);
    ASSERT_NE(flat, nullptr);
    EXPECT_NE(problem.find("flattens"), std::string::npos) << problem;
    EXPECT_FALSE(topo::hasExactFaces(*flat));
    for (const auto& e : flat->edges()) {
        EXPECT_EQ(e.exactCurve, nullptr);
        EXPECT_EQ(e.tolerance, 0.0);
        for (const topo::HalfEdge* he : {e.halfEdge, e.halfEdge->twin}) {
            EXPECT_EQ(he->pcurve, nullptr);
        }
    }
}

// A cone mirrored: each loop is reversed, and a loop through the apex that wrapped round the cone
// at a rim now wraps round it at the apex. The frustum (no apex), the cone whose seam runs up to
// its apex, and Unigraphics' cone with a VERTEX_LOOP at its apex: each mirrored copy validates,
// and measures as the original.
TEST(ExactTransform, AMirroredConeIsValid) {
    const struct {
        const char* what;
        std::string text;
    } cases[] = {{"the frustum", hz::step_text::frustum()},
                 {"the cone", hz::step_text::cone()},
                 {"the apex cone", hz::step_text::apexCone()}};
    for (const auto& c : cases) {
        SCOPED_TRACE(c.what);
        const auto cone = readOne(c.text);
        ASSERT_NE(cone, nullptr);
        ASSERT_EQ(firstIssue(*cone), "");
        const auto mirrored = Pattern::transformed(*cone, Mat4::scale({-1, 1, 1}));
        ASSERT_NE(mirrored, nullptr);
        ASSERT_TRUE(topo::hasExactFaces(*mirrored));
        EXPECT_EQ(firstIssue(*mirrored), "");
        const MassProperties before = MassPropertiesCalculator::compute(*cone);
        const MassProperties after = MassPropertiesCalculator::compute(*mirrored);
        ASSERT_TRUE(before.valid) << before.problem;
        ASSERT_TRUE(after.valid) << after.problem;
        EXPECT_NEAR(after.volume, before.volume, 1e-9 * before.volume);
    }
}
