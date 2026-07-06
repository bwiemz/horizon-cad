#include "horizon/simulation/Fatigue.h"

#include <cmath>
#include <limits>

namespace hz::sim {

namespace {
constexpr double kInf = std::numeric_limits<double>::infinity();
}

bool SNCurve::isValid() const {
    return basquinCoefficient > 0.0 && basquinExponent < 0.0 && enduranceLimit > 0.0 &&
           enduranceLimit <= basquinCoefficient && ultimateStrength > 0.0;
}

SNCurve SNCurve::fromTwoPoints(double s1, double n1, double s2, double n2, double enduranceLimit,
                               double ultimateStrength) {
    SNCurve sn;
    if (s1 <= 0.0 || s2 <= 0.0 || n1 <= 0.0 || n2 <= 0.0 || s1 == s2 || n1 == n2) return sn;

    // S = A * N^b  =>  b = ln(s1/s2) / ln(n1/n2),  A = s1 / n1^b.
    const double b = std::log(s1 / s2) / std::log(n1 / n2);
    sn.basquinExponent = b;
    sn.basquinCoefficient = s1 / std::pow(n1, b);
    sn.enduranceLimit = enduranceLimit;
    sn.ultimateStrength = ultimateStrength;
    return sn;
}

SNCurve SNCurve::steel(double ultimateStrength) {
    if (ultimateStrength <= 0.0) return SNCurve{};
    const double se = 0.5 * ultimateStrength;
    return fromTwoPoints(0.9 * ultimateStrength, 1.0e3, se, 1.0e6, se, ultimateStrength);
}

double cyclesToFailure(const SNCurve& sn, double stressAmplitude) {
    if (!sn.isValid()) return 0.0;
    if (stressAmplitude <= 0.0 || stressAmplitude <= sn.enduranceLimit) return kInf;
    if (stressAmplitude >= sn.basquinCoefficient) return 1.0;  // at/above 1-cycle strength
    // N = (S / A)^(1/b).
    return std::pow(stressAmplitude / sn.basquinCoefficient, 1.0 / sn.basquinExponent);
}

double goodmanEquivalent(double amplitude, double mean, double ultimateStrength) {
    if (ultimateStrength <= 0.0) return 0.0;
    const double denom = 1.0 - mean / ultimateStrength;
    if (denom <= 0.0) return kInf;  // mean at/above ultimate: no usable amplitude
    return amplitude / denom;
}

double soderbergEquivalent(double amplitude, double mean, double yieldStrength) {
    if (yieldStrength <= 0.0) return 0.0;
    const double denom = 1.0 - mean / yieldStrength;
    if (denom <= 0.0) return kInf;
    return amplitude / denom;
}

double fatigueSafetyFactor(const SNCurve& sn, double amplitude, double mean) {
    if (!sn.isValid()) return 0.0;
    const double sar = goodmanEquivalent(amplitude, mean, sn.ultimateStrength);
    if (sar <= 0.0) return kInf;  // no effective alternating stress
    if (sar == kInf) return 0.0;  // mean stress alone exhausts the material
    return sn.enduranceLimit / sar;
}

}  // namespace hz::sim
