#include <gtest/gtest.h>

#include <cmath>
#include <memory>
#include <vector>

#include "horizon/drafting/DraftLine.h"
#include "horizon/drafting/SketchPlane.h"
#include "horizon/math/Constants.h"
#include "horizon/modeling/MassProperties.h"
#include "horizon/modeling/Revolve.h"
#include "horizon/topology/GeometryValidator.h"
#include "horizon/topology/Solid.h"
#include "horizon/topology/TopologyID.h"

using namespace hz::model;
using namespace hz::topo;
using namespace hz::draft;
using hz::math::kPi;
using hz::math::Vec2;
using hz::math::Vec3;

static constexpr double kTwoPi = 2.0 * kPi;

/// A closed profile through the given sketch-plane points.
static std::vector<std::shared_ptr<DraftEntity>> makeProfile(const std::vector<Vec2>& pts) {
    std::vector<std::shared_ptr<DraftEntity>> profile;
    for (size_t i = 0; i < pts.size(); ++i) {
        profile.push_back(std::make_shared<DraftLine>(pts[i], pts[(i + 1) % pts.size()]));
    }
    return profile;
}

/// A rectangle offset from the Y axis, the standard revolve test profile.
static std::vector<std::shared_ptr<DraftEntity>> makeOffsetRectProfile(double xMin, double xMax,
                                                                       double yMin, double yMax) {
    return makeProfile({{xMin, yMin}, {xMax, yMin}, {xMax, yMax}, {xMin, yMax}});
}

/// Pappus: revolving an area A whose centroid sits at radius R sweeps 2*pi*R*A.
static double pappusVolume(double xMin, double xMax, double yMin, double yMax, double angle) {
    const double area = (xMax - xMin) * (yMax - yMin);
    const double centroidRadius = 0.5 * (xMin + xMax);
    return angle * centroidRadius * area;
}

// ---------------------------------------------------------------------------
// Topology
// ---------------------------------------------------------------------------

TEST(RevolveTest, FullRevolutionIsAManifoldGenusOneShell) {
    auto solid = Revolve::execute(makeOffsetRectProfile(5.0, 10.0, 0.0, 5.0), SketchPlane(),
                                  Vec3::Zero, Vec3::UnitY, kTwoPi, "revolve_1", 32);
    ASSERT_NE(solid, nullptr);

    // 32 angular steps of a 4-vertex profile: one ring of 4 vertices per step,
    // one band of 4 quads per step, and 8 edges per step (4 along the profile,
    // 4 spanning to the next ring).
    EXPECT_EQ(solid->vertexCount(), 128u);
    EXPECT_EQ(solid->edgeCount(), 256u);
    EXPECT_EQ(solid->faceCount(), 128u);
    EXPECT_EQ(solid->shellCount(), 1u);
    EXPECT_TRUE(solid->checkManifold());

    // A rectangle clear of the axis sweeps a torus: genus 1, so its Euler
    // characteristic is 0, not 2.  Solid::checkEulerFormula() carries no genus
    // term and therefore rejects it — the same caveat as makeTorus.
    EXPECT_FALSE(solid->checkEulerFormula())
        << "the genus-0 Euler check is expected to reject a torus";
}

TEST(RevolveTest, PartialRevolutionIsCappedAndGenusZero) {
    auto solid = Revolve::execute(makeOffsetRectProfile(5.0, 10.0, 0.0, 5.0), SketchPlane(),
                                  Vec3::Zero, Vec3::UnitY, kPi * 0.5, "revolve_quarter", 32);
    ASSERT_NE(solid, nullptr);

    // Capping both ends closes the hole, so the quarter turn is genus 0.
    EXPECT_TRUE(solid->checkManifold());
    EXPECT_TRUE(solid->checkEulerFormula());
    EXPECT_TRUE(solid->isValid()) << solid->validationReport();
    EXPECT_TRUE(GeometryValidator::isGeometricallyValid(*solid))
        << GeometryValidator::report(*solid);
}

TEST(RevolveTest, PartialRevolutionKeepsTheAngularResolutionOfAFullTurn) {
    // A quarter turn at 32 steps per full turn is 8 steps, so 9 rings.
    auto solid = Revolve::execute(makeOffsetRectProfile(5.0, 10.0, 0.0, 5.0), SketchPlane(),
                                  Vec3::Zero, Vec3::UnitY, kPi * 0.5, "revolve_res", 32);
    ASSERT_NE(solid, nullptr);
    EXPECT_EQ(solid->vertexCount(), 9u * 4u);
}

TEST(RevolveTest, BandsAreExactlyPlanar) {
    // Rotating two points about a common axis leaves all four corners of the
    // band on one plane, so no band needs an approximate carrier.  A profile
    // edge oblique to the axis is the interesting case: its band changes both
    // radius and height.
    auto solid = Revolve::execute(makeProfile({{4, 0}, {9, 0}, {6, 6}, {3, 4}}), SketchPlane(),
                                  Vec3::Zero, Vec3::UnitY, kPi * 0.75, "revolve_oblique", 24);
    ASSERT_NE(solid, nullptr);

    const auto issues = GeometryValidator::check(*solid);
    EXPECT_EQ(issues.nonPlanarLoops, 0) << GeometryValidator::report(*solid);
    EXPECT_EQ(issues.selfIntersectingLoops, 0) << GeometryValidator::report(*solid);
    EXPECT_TRUE(issues.ok()) << GeometryValidator::report(*solid);
}

// ---------------------------------------------------------------------------
// Volume — the property the old box topology got wrong
// ---------------------------------------------------------------------------

TEST(RevolveTest, FullRevolutionVolumeConvergesToPappus) {
    const double exact = pappusVolume(5.0, 10.0, 0.0, 5.0, kTwoPi);

    double previousError = 0.0;
    for (int segments : {16, 32, 64, 128}) {
        auto solid = Revolve::execute(makeOffsetRectProfile(5.0, 10.0, 0.0, 5.0), SketchPlane(),
                                      Vec3::Zero, Vec3::UnitY, kTwoPi, "revolve_vol", segments);
        ASSERT_NE(solid, nullptr) << "segments = " << segments;

        const double volume = MassPropertiesCalculator::compute(*solid).volume;
        const double error = std::abs(volume - exact) / exact;
        EXPECT_LT(volume, exact) << "an inscribed polygon can only undershoot";

        if (previousError > 0.0) {
            // Chord error falls as 1/n^2: doubling the steps must quarter it.
            EXPECT_LT(error, previousError * 0.30) << "segments = " << segments;
        }
        previousError = error;
    }
    EXPECT_LT(previousError, 0.001) << "128 steps should be within a tenth of a percent";
}

TEST(RevolveTest, PartialRevolutionVolumeIsTheMatchingFraction) {
    for (double fraction : {0.25, 0.5, 0.75}) {
        const double angle = kTwoPi * fraction;
        auto solid = Revolve::execute(makeOffsetRectProfile(5.0, 10.0, 0.0, 5.0), SketchPlane(),
                                      Vec3::Zero, Vec3::UnitY, angle, "revolve_part", 64);
        ASSERT_NE(solid, nullptr) << "fraction = " << fraction;

        const double exact = pappusVolume(5.0, 10.0, 0.0, 5.0, angle);
        const double volume = MassPropertiesCalculator::compute(*solid).volume;
        EXPECT_NEAR(volume, exact, exact * 0.002) << "fraction = " << fraction;
    }
}

TEST(RevolveTest, ProfileTouchingTheAxisSweepsACone) {
    // A right triangle with one vertex on the axis: radius 5, height 10.
    // Vertices on the axis do not move, so their bands collapse to triangles.
    const double exact = kPi * 25.0 * 10.0 / 3.0;
    auto solid = Revolve::execute(makeProfile({{0, 0}, {5, 0}, {0, 10}}), SketchPlane(), Vec3::Zero,
                                  Vec3::UnitY, kTwoPi, "revolve_cone", 128);
    ASSERT_NE(solid, nullptr);

    EXPECT_TRUE(solid->checkManifold());
    // Touching the axis closes the hole, so the cone is genus 0.
    EXPECT_TRUE(solid->checkEulerFormula());
    EXPECT_NEAR(MassPropertiesCalculator::compute(*solid).volume, exact, exact * 0.001);
}

// ---------------------------------------------------------------------------
// Geometry bindings
// ---------------------------------------------------------------------------

TEST(RevolveTest, EveryFaceAndEdgeCarriesGeometryAndAnID) {
    auto solid = Revolve::execute(makeOffsetRectProfile(5.0, 10.0, 0.0, 5.0), SketchPlane(),
                                  Vec3::Zero, Vec3::UnitY, kPi, "revolve_geom", 16);
    ASSERT_NE(solid, nullptr);

    for (const auto& face : solid->faces()) {
        EXPECT_TRUE(face.topoId.isValid()) << "Face " << face.id << " has no TopologyID";
        EXPECT_NE(face.surface, nullptr) << "Face " << face.id << " has no carrier surface";
    }
    for (const auto& edge : solid->edges()) {
        EXPECT_TRUE(edge.topoId.isValid()) << "Edge " << edge.id << " has no TopologyID";
        EXPECT_NE(edge.curve, nullptr) << "Edge " << edge.id << " has no NURBS curve";
    }
}

TEST(RevolveTest, CurvedBandsRecordTheSurfaceOfRevolutionTheyApproximate) {
    // The rectangle's two axis-parallel edges sweep cylinders; its two
    // axis-perpendicular edges sweep flat annuli, whose planar carrier is
    // already exact and so needs no ideal recorded.
    auto solid = Revolve::execute(makeOffsetRectProfile(5.0, 10.0, 0.0, 5.0), SketchPlane(),
                                  Vec3::Zero, Vec3::UnitY, kTwoPi, "revolve_ideal", 16);
    ASSERT_NE(solid, nullptr);

    int curved = 0;
    int flat = 0;
    for (const auto& face : solid->faces()) {
        if (face.analyticSurface != nullptr) {
            ++curved;
        } else {
            ++flat;
        }
    }
    // Profile edges 1 and 3 run parallel to the Y axis; 0 and 2 run across it.
    EXPECT_EQ(curved, 32) << "the two cylindrical bands, 16 steps each";
    EXPECT_EQ(flat, 32) << "the two annular bands are exactly planar";
}

TEST(RevolveTest, RimEdgesRecordTheCircleTheyApproximate) {
    auto solid = Revolve::execute(makeOffsetRectProfile(5.0, 10.0, 0.0, 5.0), SketchPlane(),
                                  Vec3::Zero, Vec3::UnitY, kTwoPi, "revolve_rims", 16);
    ASSERT_NE(solid, nullptr);

    int rims = 0;
    for (const auto& edge : solid->edges()) {
        if (edge.analyticCurve == nullptr) {
            continue;
        }
        ++rims;
        // The chord's ends must sit on the circle the edge claims to follow.
        const Vec3 a = edge.halfEdge->origin->point;
        const Vec3 b = edge.halfEdge->twin->origin->point;
        EXPECT_NEAR(std::hypot(a.x, a.z), std::hypot(b.x, b.z), 1e-9);
    }
    // Four profile vertices, each tracing a circle sampled at 16 steps.
    EXPECT_EQ(rims, 64);
}

// ---------------------------------------------------------------------------
// Rejections
// ---------------------------------------------------------------------------

TEST(RevolveTest, RejectsAProfileCrossingTheAxis) {
    // Sweeping a profile that straddles the axis drags it through itself.
    auto solid = Revolve::execute(makeOffsetRectProfile(-5.0, 5.0, 0.0, 5.0), SketchPlane(),
                                  Vec3::Zero, Vec3::UnitY, kTwoPi, "revolve_straddle");
    EXPECT_EQ(solid, nullptr);
}

TEST(RevolveTest, RejectsAnAxisThatIsNotInTheProfilePlane) {
    // The axis of revolution has to lie in the profile's own plane, or the
    // profile does not lie in a half-plane bounded by it.  Revolving an XY
    // sketch about Z — the sketch normal — sweeps the profile through itself
    // within its own plane and produces no solid at all.
    auto solid = Revolve::execute(makeOffsetRectProfile(3.0, 7.0, -2.0, 2.0), SketchPlane(),
                                  Vec3::Zero, Vec3::UnitZ, kTwoPi, "revolve_z");
    EXPECT_EQ(solid, nullptr);

    // Parallel to the plane but offset out of it is refused for the same
    // reason: the radial directions are no longer collinear.
    auto offset = Revolve::execute(makeOffsetRectProfile(5.0, 10.0, 0.0, 5.0), SketchPlane(),
                                   Vec3(0.0, 0.0, 3.0), Vec3::UnitY, kTwoPi, "revolve_offset");
    EXPECT_EQ(offset, nullptr);
}

TEST(RevolveTest, RejectsAnglesOutsideAFullTurn) {
    auto profile = makeOffsetRectProfile(5.0, 10.0, 0.0, 5.0);
    SketchPlane plane;
    EXPECT_EQ(Revolve::execute(profile, plane, Vec3::Zero, Vec3::UnitY, 0.0, "a"), nullptr);
    EXPECT_EQ(Revolve::execute(profile, plane, Vec3::Zero, Vec3::UnitY, -kPi, "b"), nullptr);
    EXPECT_EQ(Revolve::execute(profile, plane, Vec3::Zero, Vec3::UnitY, kTwoPi * 1.1, "c"),
              nullptr);
    EXPECT_EQ(Revolve::execute(profile, plane, Vec3::Zero, Vec3::UnitY, kTwoPi, "d", 2), nullptr)
        << "fewer than three steps cannot bound a volume";
}

TEST(RevolveTest, InvalidProfileReturnsNull) {
    std::vector<std::shared_ptr<DraftEntity>> profile;
    profile.push_back(std::make_shared<DraftLine>(Vec2(0, 0), Vec2(10, 0)));  // open chain

    auto solid =
        Revolve::execute(profile, SketchPlane(), Vec3::Zero, Vec3::UnitY, kTwoPi, "revolve_bad");
    EXPECT_EQ(solid, nullptr);
}

TEST(RevolveTest, EmptyProfileReturnsNull) {
    auto solid =
        Revolve::execute({}, SketchPlane(), Vec3::Zero, Vec3::UnitY, kTwoPi, "revolve_empty");
    EXPECT_EQ(solid, nullptr);
}

// ---------------------------------------------------------------------------
// Tolerance-driven resolution
// ---------------------------------------------------------------------------

TEST(RevolveTest, SegmentsForToleranceTightensAsToleranceFalls) {
    int previous = 0;
    for (double tolerance : {1.0, 0.1, 0.01, 0.001}) {
        const int segments = Revolve::segmentsForTolerance(10.0, tolerance);
        EXPECT_GE(segments, 3);
        EXPECT_GT(segments, previous);
        previous = segments;

        // The chord sagitta of the chosen step must actually meet the ask.
        const double sagitta = 10.0 * (1.0 - std::cos(kPi / static_cast<double>(segments)));
        EXPECT_LE(sagitta, tolerance * 1.0000001) << "tolerance = " << tolerance;
    }
}
