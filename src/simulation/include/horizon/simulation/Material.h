#pragma once

namespace hz::sim {

/// An isotropic linear-elastic material for finite-element analysis.
///
/// Units are the caller's responsibility but must be consistent (e.g. SI: E in
/// pascals, density in kg/m^3, lengths in metres). The Poisson ratio must lie in
/// [0, 0.5) — the elasticity matrix is singular at exactly 0.5 (incompressible).
struct ElasticMaterial {
    double youngsModulus = 0.0;  ///< E — stiffness
    double poissonRatio = 0.0;   ///< nu — lateral contraction, 0 <= nu < 0.5
    double density = 0.0;        ///< rho — mass per unit volume

    /// A material is usable for analysis when E > 0 and 0 <= nu < 0.5.
    bool isValid() const {
        return youngsModulus > 0.0 && poissonRatio >= 0.0 && poissonRatio < 0.5;
    }
};

/// Common material presets (SI units: E in Pa, density in kg/m^3).
namespace materials {

/// Structural steel: E = 200 GPa, nu = 0.30, rho = 7850 kg/m^3.
ElasticMaterial steel();

/// Aluminium 6061: E = 69 GPa, nu = 0.33, rho = 2700 kg/m^3.
ElasticMaterial aluminum();

/// Titanium Ti-6Al-4V: E = 114 GPa, nu = 0.34, rho = 4430 kg/m^3.
ElasticMaterial titanium();

}  // namespace materials

}  // namespace hz::sim
