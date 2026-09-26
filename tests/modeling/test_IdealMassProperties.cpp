// The ideal mass properties (Phase 141): a part measured on the surfaces its
// facets approximate, against the closed forms. As modelled, the facets, a
// 32-sided cylinder reads 0.6 % light; ideally it is the cylinder.

#include <gtest/gtest.h>

#include <atomic>
#include <chrono>
#include <cmath>
#include <deque>
#include <memory>
#include <numbers>
#include <thread>
#include <vector>

#include "horizon/drafting/DraftLine.h"
#include "horizon/drafting/SketchPlane.h"
#include "horizon/modeling/FilletOp.h"
#include "horizon/modeling/MassProperties.h"
#include "horizon/modeling/PrimitiveFactory.h"
#include "horizon/modeling/Revolve.h"
#include "horizon/topology/Solid.h"

using namespace hz::model;
using hz::math::Vec2;
using hz::math::Vec3;

namespace {

constexpr double kPi = std::numbers::pi;

/// Relative to @p expected.
void expectNear(double actual, double expected, double relative, const char* what) {
    EXPECT_NEAR(actual, expected, relative * std::abs(expected)) << what;
}

struct Closed {
    double volume;
    double area;
    Vec3 centre;
    double izz;  ///< about the axis through the centre, unit density
};

void expectIdeal(const hz::topo::Solid& solid, const Closed& closed, const char* what) {
    const auto ideal = MassPropertiesCalculator::computeIdeal(solid);
    ASSERT_TRUE(ideal.properties.valid) << what;
    EXPECT_TRUE(ideal.exact) << what;
    expectNear(ideal.properties.volume, closed.volume, 1e-9, what);
    expectNear(ideal.properties.surfaceArea, closed.area, 1e-9, what);
    EXPECT_LT((ideal.properties.centerOfMass - closed.centre).length(), 1e-9) << what;
    expectNear(ideal.properties.inertia.at(2, 2), closed.izz, 1e-9, what);

    // As modelled, the facets: unchanged, and short of the ideal.
    const auto modelled = MassPropertiesCalculator::compute(solid);
    EXPECT_GT(std::abs(modelled.volume - closed.volume), 1e-5 * closed.volume) << what;
}

}  // namespace

TEST(IdealMassPropertiesTest, ACylinderIsACylinder) {
    const double r = 2.0, h = 5.0;
    auto solid = PrimitiveFactory::makeCylinder(r, h);
    ASSERT_NE(solid, nullptr);
    const double v = kPi * r * r * h;
    expectIdeal(*solid, {v, 2 * kPi * r * h + 2 * kPi * r * r, Vec3(0, 0, h / 2), v * r * r / 2},
                "cylinder");
}

TEST(IdealMassPropertiesTest, AConeIsACone) {
    const double r = 3.0, h = 4.0;
    auto solid = PrimitiveFactory::makeCone(r, 0.0, h);
    ASSERT_NE(solid, nullptr);
    const double v = kPi * r * r * h / 3;
    expectIdeal(*solid,
                {v, kPi * r * r + kPi * r * std::hypot(r, h), Vec3(0, 0, h / 4), 0.3 * v * r * r},
                "cone");
}

TEST(IdealMassPropertiesTest, ASphereIsASphere) {
    const double r = 1.5;
    auto solid = PrimitiveFactory::makeSphere(r, 16);  // the ideal does not need more facets
    ASSERT_NE(solid, nullptr);
    const double v = 4.0 / 3.0 * kPi * r * r * r;
    expectIdeal(*solid, {v, 4 * kPi * r * r, Vec3(), 0.4 * v * r * r}, "sphere");
}

TEST(IdealMassPropertiesTest, ATorusIsATorus) {
    const double big = 4.0, small = 1.0;
    auto solid = PrimitiveFactory::makeTorus(big, small, 16);
    ASSERT_NE(solid, nullptr);
    const double v = 2 * kPi * kPi * big * small * small;
    expectIdeal(*solid,
                {v, 4 * kPi * kPi * big * small, Vec3(), v * (big * big + 0.75 * small * small)},
                "torus");
}

// A ring from a rectangle revolved: two cylinders and two annuli.
TEST(IdealMassPropertiesTest, ARevolveIsItsSurfacesOfRevolution) {
    const double inner = 2.0, outer = 3.0, h = 1.5;
    std::vector<std::shared_ptr<hz::draft::DraftEntity>> profile{
        std::make_shared<hz::draft::DraftLine>(Vec2(inner, 0), Vec2(outer, 0)),
        std::make_shared<hz::draft::DraftLine>(Vec2(outer, 0), Vec2(outer, h)),
        std::make_shared<hz::draft::DraftLine>(Vec2(outer, h), Vec2(inner, h)),
        std::make_shared<hz::draft::DraftLine>(Vec2(inner, h), Vec2(inner, 0))};
    // The sketch's y is the axis.
    auto solid = Revolve::execute(profile, hz::draft::SketchPlane(), Vec3(0, 0, 0), Vec3(0, 1, 0),
                                  2 * kPi, "ring");
    ASSERT_NE(solid, nullptr);
    const double v = kPi * (outer * outer - inner * inner) * h;
    const double area = 2 * kPi * (outer + inner) * h + 2 * kPi * (outer * outer - inner * inner);
    // About the y axis: the ring's (r_o^2 + r_i^2) / 2.
    const auto ideal = MassPropertiesCalculator::computeIdeal(*solid);
    ASSERT_TRUE(ideal.properties.valid);
    EXPECT_TRUE(ideal.exact);
    expectNear(ideal.properties.volume, v, 1e-9, "volume");
    expectNear(ideal.properties.surfaceArea, area, 1e-9, "area");
    expectNear(ideal.properties.inertia.at(1, 1), v * (outer * outer + inner * inner) / 2, 1e-9,
               "inertia about the axis");
}

// A box with one edge filleted: the quarter cylinder, exactly.
TEST(IdealMassPropertiesTest, AFilletIsItsQuarterCylinder) {
    const double a = 10.0, b = 6.0, c = 4.0, r = 1.5;
    auto box = PrimitiveFactory::makeBox(a, b, c);
    ASSERT_NE(box, nullptr);
    // The edge along x at y = b, z = c.
    hz::topo::TopologyID edge;
    for (const auto& e : box->edges()) {
        const Vec3& p = e.halfEdge->origin->point;
        const Vec3& q = e.halfEdge->twin->origin->point;
        if (std::abs(p.y - b) < 1e-9 && std::abs(q.y - b) < 1e-9 && std::abs(p.z - c) < 1e-9 &&
            std::abs(q.z - c) < 1e-9) {
            edge = e.topoId;
        }
    }
    ASSERT_TRUE(edge.isValid());
    auto rounded = FilletOp::execute(*box, {edge}, r, "f", 8);
    ASSERT_NE(rounded.solid, nullptr) << rounded.errorMessage;
    const double corner = (1 - kPi / 4) * r * r;
    const auto ideal = MassPropertiesCalculator::computeIdeal(*rounded.solid);
    ASSERT_TRUE(ideal.properties.valid);
    EXPECT_TRUE(ideal.exact);
    expectNear(ideal.properties.volume, a * b * c - corner * a, 1e-9, "volume");
    expectNear(ideal.properties.surfaceArea,
               2 * (a * b + b * c + c * a) - 2 * r * a + kPi / 2 * r * a - 2 * corner, 1e-9,
               "area");
}

// A part with nothing curved is measured as modelled, exactly.
TEST(IdealMassPropertiesTest, APartWithNothingCurvedIsAsModelled) {
    auto box = PrimitiveFactory::makeBox(3, 4, 5);
    const auto ideal = MassPropertiesCalculator::computeIdeal(*box);
    EXPECT_TRUE(ideal.exact);
    EXPECT_EQ(ideal.properties.volume, MassPropertiesCalculator::compute(*box).volume);
    EXPECT_DOUBLE_EQ(ideal.properties.volume, 60.0);
}

// A part whose curved faces have no ideals says so: it is measured as
// modelled there, and its faces are named. (A mesh imported, say.)
TEST(IdealMassPropertiesTest, FacetsWithoutTheirIdealAreNamed) {
    auto solid = PrimitiveFactory::makeCylinder(2.0, 5.0);
    ASSERT_NE(solid, nullptr);
    for (auto& face : const_cast<std::deque<hz::topo::Face>&>(solid->faces())) {
        face.analyticSurface = nullptr;
    }
    for (auto& edge : const_cast<std::deque<hz::topo::Edge>&>(solid->edges())) {
        edge.analyticCurve = nullptr;
    }
    const auto ideal = MassPropertiesCalculator::computeIdeal(*solid);
    EXPECT_FALSE(ideal.exact);
    ASSERT_FALSE(ideal.withoutIdeal.empty());
    EXPECT_EQ(ideal.withoutIdeal.front().rfind("cylinder/", 0), 0u) << ideal.withoutIdeal.front();
    EXPECT_EQ(ideal.properties.volume, MassPropertiesCalculator::compute(*solid).volume);
}

// A rim rounded (Phase 164): its bands are one torus, tangent to the side's
// cylinder and the top's plane, so the ideals meet and the rounded cylinder
// is measured as designed. (Rounded chord by chord, each chord's fillet
// ideally straight, the ideals parted, and the measure was not exact.)
TEST(IdealMassPropertiesTest, ARoundedRimIsMeasuredAsDesigned) {
    const double r = 5.0, h = 4.0;
    auto solid = PrimitiveFactory::makeCylinder(r, h, 16);
    ASSERT_NE(solid, nullptr);
    std::vector<hz::topo::TopologyID> rim;
    for (const auto& e : solid->edges()) {
        const Vec3& p = e.halfEdge->origin->point;
        const Vec3& q = e.halfEdge->twin->origin->point;
        if (std::abs(p.z - h) < 1e-9 && std::abs(q.z - h) < 1e-9) rim.push_back(e.topoId);
    }
    ASSERT_EQ(rim.size(), 16u);
    auto rounded = FilletOp::execute(*solid, rim, 0.5, "f", 4);
    ASSERT_NE(rounded.solid, nullptr) << rounded.errorMessage;
    const auto ideal = MassPropertiesCalculator::computeIdeal(*rounded.solid);
    ASSERT_TRUE(ideal.properties.valid);
    EXPECT_EQ(ideal.partedEdges, 0) << "the torus meets the cylinder and the plane";
    // The rounded cylinder: less the rim's section, (1 - pi/4) f^2, turned
    // about the axis at its centroid, (10 - 3 pi) / (3 (4 - pi)) f in from
    // the rim (Pappus).
    const double f = 0.5;
    const double in = (10.0 - 3.0 * kPi) / (3.0 * (4.0 - kPi)) * f;
    const double rounded_ = kPi * r * r * h - 2 * kPi * (r - in) * (1 - kPi / 4) * f * f;
    const double modelled = MassPropertiesCalculator::compute(*rounded.solid).volume;
    EXPECT_NEAR(ideal.properties.volume, rounded_, 1e-3 * rounded_);
    EXPECT_LT(std::abs(ideal.properties.volume - rounded_), std::abs(modelled - rounded_))
        << "nearer the design than the facets";
}

// A measurement cancelled stops within a moment and says it is not valid:
// the window waits for it when it closes (Phase 141 review). It looked
// only between faces, so a large face at a fine refinement ran on.
TEST(IdealMassPropertiesTest, ACancelledMeasurementStopsPromptly) {
    auto solid = PrimitiveFactory::makeTorus(4.0, 1.0, 64);
    ASSERT_NE(solid, nullptr);
    std::atomic<bool> cancelled{false};
    std::chrono::steady_clock::time_point stoppedAt;
    hz::model::IdealMassProperties ideal;
    std::thread worker([&] {
        // A tolerance it cannot reach: it would refine to its limit.
        ideal = MassPropertiesCalculator::computeIdeal(*solid, nullptr, 0.0, &cancelled);
        stoppedAt = std::chrono::steady_clock::now();
    });
    std::this_thread::sleep_for(std::chrono::milliseconds(200));
    const auto cancelledAt = std::chrono::steady_clock::now();
    cancelled = true;
    worker.join();
    EXPECT_FALSE(ideal.properties.valid);
    const double ms = std::chrono::duration<double, std::milli>(stoppedAt - cancelledAt).count();
    EXPECT_LT(ms, 2000.0) << "stopped " << ms << " ms after it was cancelled";
}
