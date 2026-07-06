#pragma once

namespace hz::sim {

/// A stress-life (S-N) material model for high-cycle fatigue.
///
/// The finite-life region follows Basquin's power law S = A * N^b (equivalently
/// N = (S / A)^(1/b) with b < 0), and at or below the endurance limit Se the
/// material is treated as having infinite life. All stresses share the caller's
/// stress unit (e.g. Pa); N is in cycles.
struct SNCurve {
    double basquinCoefficient = 0.0;  ///< A: extrapolated stress at N = 1 cycle
    double basquinExponent = 0.0;     ///< b: log-log slope (must be < 0)
    double enduranceLimit = 0.0;      ///< Se: stress below which life is infinite
    double ultimateStrength = 0.0;    ///< Su: for mean-stress corrections

    /// Usable when A > 0, b < 0, 0 < Se <= A, and Su > 0.
    bool isValid() const;

    /// Build an S-N curve from two finite-life points (stress s1 at n1 cycles,
    /// s2 at n2) plus the endurance limit and ultimate strength. Requires
    /// positive, distinct points with s1 != s2 and n1 != n2.
    static SNCurve fromTwoPoints(double s1, double n1, double s2, double n2, double enduranceLimit,
                                 double ultimateStrength);

    /// A textbook high-cycle steel curve from the ultimate strength Su:
    /// S = 0.9*Su at 1e3 cycles, knee at Se = 0.5*Su at 1e6 cycles.
    static SNCurve steel(double ultimateStrength);
};

/// Cycles to failure for a fully-reversed stress amplitude (zero mean).
/// Returns +infinity at or below the endurance limit (and for a non-positive
/// amplitude); clamps to 1 cycle for an amplitude at or above the 1-cycle
/// strength A. Returns 0 for an invalid curve.
double cyclesToFailure(const SNCurve& sn, double stressAmplitude);

/// Goodman mean-stress correction: the equivalent fully-reversed amplitude of a
/// load with amplitude Sa and mean Sm, Sar = Sa / (1 - Sm/Su). Returns +infinity
/// as the mean approaches the ultimate strength, 0 for a non-positive Su.
double goodmanEquivalent(double amplitude, double mean, double ultimateStrength);

/// Soderberg mean-stress correction against the yield strength Sy,
/// Sar = Sa / (1 - Sm/Sy). More conservative than Goodman.
double soderbergEquivalent(double amplitude, double mean, double yieldStrength);

/// Fatigue safety factor against infinite life: Se / Sar, where Sar is the
/// Goodman-corrected equivalent amplitude of (amplitude, mean). A value >= 1
/// means the endurance limit is not exceeded (infinite life). Returns 0 for an
/// invalid curve and +infinity for a non-positive equivalent amplitude.
double fatigueSafetyFactor(const SNCurve& sn, double amplitude, double mean);

}  // namespace hz::sim
