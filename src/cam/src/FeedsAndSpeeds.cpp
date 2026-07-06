#include "horizon/cam/FeedsAndSpeeds.h"

namespace hz::cam {

namespace {
constexpr double kPi = 3.14159265358979323846;
}

double spindleRpm(double surfaceSpeedMPerMin, double diameterMm) {
    if (diameterMm <= 0.0) return 0.0;
    // v (m/min) = pi * D(m) * N; D in mm -> D/1000 m.
    return 1000.0 * surfaceSpeedMPerMin / (kPi * diameterMm);
}

double feedRate(double rpm, int flutes, double chipLoadMm) {
    if (rpm <= 0.0 || flutes <= 0 || chipLoadMm <= 0.0) return 0.0;
    return rpm * flutes * chipLoadMm;
}

}  // namespace hz::cam
