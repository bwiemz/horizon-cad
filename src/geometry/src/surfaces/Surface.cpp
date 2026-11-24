#include "horizon/geometry/surfaces/Surface.h"

namespace hz::geo {

math::Vec3 Surface::normal(double u, double v) const {
    const SurfaceDerivatives d = derivatives(u, v);
    const math::Vec3 n = d.du.cross(d.dv);
    const double len = n.length();
    if (len == 0.0) {
        return {0.0, 0.0, 0.0};
    }
    return n / len;
}

}  // namespace hz::geo
