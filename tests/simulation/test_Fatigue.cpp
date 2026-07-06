#include <gtest/gtest.h>

#include <cmath>
#include <limits>

#include "horizon/simulation/Fatigue.h"

using hz::sim::cyclesToFailure;
using hz::sim::fatigueSafetyFactor;
using hz::sim::goodmanEquivalent;
using hz::sim::SNCurve;
using hz::sim::soderbergEquivalent;

namespace {
constexpr double kInf = std::numeric_limits<double>::infinity();
}

// fromTwoPoints fits S = A*N^b through both given points exactly, and the
// finite-life inverse cyclesToFailure recovers each point's cycle count.
TEST(FatigueTest, BasquinRoundTripsThroughItsPoints) {
    const double Su = 500.0e6;               // 500 MPa ultimate
    const double s1 = 0.9 * Su, n1 = 1.0e3;  // 1e3-cycle strength
    const double s2 = 0.5 * Su, n2 = 1.0e6;  // endurance point
    const SNCurve sn = SNCurve::fromTwoPoints(s1, n1, s2, n2, /*Se=*/0.5 * Su, Su);
    ASSERT_TRUE(sn.isValid());

    // Both anchor points lie on the curve: N(s) == n.
    EXPECT_NEAR(cyclesToFailure(sn, s1), n1, 1e-6 * n1);
    // s2 equals the endurance limit here, so it is infinite-life by definition;
    // probe just above the knee instead to exercise the finite-life branch.
    const double sMid = 0.7 * Su;
    const double nMid = std::pow(sMid / sn.basquinCoefficient, 1.0 / sn.basquinExponent);
    EXPECT_NEAR(cyclesToFailure(sn, sMid), nMid, 1e-6 * nMid);

    // Higher stress amplitude -> fewer cycles (monotonic).
    EXPECT_LT(cyclesToFailure(sn, 0.8 * Su), cyclesToFailure(sn, 0.6 * Su));
}

// Endurance and 1-cycle boundaries behave as documented.
TEST(FatigueTest, LifeBoundaries) {
    const double Su = 500.0e6;
    const SNCurve sn = SNCurve::steel(Su);
    ASSERT_TRUE(sn.isValid());

    // At/below the endurance limit -> infinite life.
    EXPECT_EQ(cyclesToFailure(sn, 0.5 * Su), kInf);
    EXPECT_EQ(cyclesToFailure(sn, 0.4 * Su), kInf);
    // Non-positive amplitude -> infinite life.
    EXPECT_EQ(cyclesToFailure(sn, 0.0), kInf);
    // At/above the 1-cycle strength A -> clamps to 1 cycle.
    EXPECT_EQ(cyclesToFailure(sn, sn.basquinCoefficient), 1.0);
    EXPECT_EQ(cyclesToFailure(sn, 2.0 * sn.basquinCoefficient), 1.0);
    // Invalid curve -> 0.
    EXPECT_EQ(cyclesToFailure(SNCurve{}, 100.0e6), 0.0);
}

// steel() places the knee at Se = 0.5*Su and 1e6 cycles.
TEST(FatigueTest, SteelPresetKnee) {
    const double Su = 400.0e6;
    const SNCurve sn = SNCurve::steel(Su);
    ASSERT_TRUE(sn.isValid());
    EXPECT_NEAR(sn.enduranceLimit, 0.5 * Su, 1e-6 * Su);
    EXPECT_NEAR(sn.ultimateStrength, Su, 1e-6 * Su);
    // Just above the endurance limit, life is finite and large (near 1e6).
    const double life = cyclesToFailure(sn, 0.5 * Su + 1.0e3);
    EXPECT_GT(life, 1.0e5);
    EXPECT_LT(life, 1.0e6);
}

// Goodman/Soderberg reduce to the raw amplitude at zero mean, inflate the
// equivalent amplitude as the mean rises, and diverge at the intercept.
TEST(FatigueTest, MeanStressCorrections) {
    const double Su = 500.0e6, Sy = 350.0e6;
    const double Sa = 100.0e6;

    // Zero mean: no correction.
    EXPECT_NEAR(goodmanEquivalent(Sa, 0.0, Su), Sa, 1e-6 * Sa);
    EXPECT_NEAR(soderbergEquivalent(Sa, 0.0, Sy), Sa, 1e-6 * Sa);

    // Nonzero mean inflates the equivalent amplitude; Soderberg (vs yield) is
    // more conservative than Goodman (vs ultimate) for the same mean.
    const double Sm = 150.0e6;
    const double g = goodmanEquivalent(Sa, Sm, Su);
    const double s = soderbergEquivalent(Sa, Sm, Sy);
    EXPECT_NEAR(g, Sa / (1.0 - Sm / Su), 1e-6 * g);
    EXPECT_NEAR(s, Sa / (1.0 - Sm / Sy), 1e-6 * s);
    EXPECT_GT(s, g);
    EXPECT_GT(g, Sa);

    // Mean at/above the intercept: diverges (no usable alternating capacity).
    EXPECT_EQ(goodmanEquivalent(Sa, Su, Su), kInf);
    EXPECT_EQ(soderbergEquivalent(Sa, Sy, Sy), kInf);
    // Non-positive strength guard.
    EXPECT_EQ(goodmanEquivalent(Sa, Sm, 0.0), 0.0);
}

// The safety factor is Se / Sar(Goodman); SF == 1 at the endurance boundary and
// falls below 1 when the corrected amplitude exceeds the endurance limit.
TEST(FatigueTest, SafetyFactor) {
    const double Su = 500.0e6;
    const SNCurve sn = SNCurve::steel(Su);  // Se = 250 MPa
    ASSERT_TRUE(sn.isValid());

    // Fully reversed at exactly the endurance limit -> SF = 1.
    EXPECT_NEAR(fatigueSafetyFactor(sn, sn.enduranceLimit, 0.0), 1.0, 1e-9);
    // Half the endurance limit, zero mean -> SF = 2.
    EXPECT_NEAR(fatigueSafetyFactor(sn, 0.5 * sn.enduranceLimit, 0.0), 2.0, 1e-9);
    // Overloaded -> SF < 1.
    EXPECT_LT(fatigueSafetyFactor(sn, 1.5 * sn.enduranceLimit, 0.0), 1.0);
    // A tensile mean stress lowers the safety factor at fixed amplitude.
    const double sfNoMean = fatigueSafetyFactor(sn, 0.5 * sn.enduranceLimit, 0.0);
    const double sfMean = fatigueSafetyFactor(sn, 0.5 * sn.enduranceLimit, 100.0e6);
    EXPECT_LT(sfMean, sfNoMean);

    // Zero load -> infinite safety; invalid curve -> 0.
    EXPECT_EQ(fatigueSafetyFactor(sn, 0.0, 0.0), kInf);
    EXPECT_EQ(fatigueSafetyFactor(SNCurve{}, 100.0e6, 0.0), 0.0);
}
