#include "horizon/modeling/MassProperties.h"

#include <array>
#include <cmath>
#include <vector>

#include "horizon/topology/HalfEdge.h"

namespace hz::model {

using hz::math::Mat3;
using hz::math::Vec3;

namespace {

using Triangle = std::array<Vec3, 3>;

// Triangulate the solid's boundary directly from its B-Rep face loops (fan from
// each outer loop's first vertex). This is exact for planar-faced solids and,
// unlike the render tessellation, has no overlapping/inconsistently-wound
// triangles. Curved faces are approximated by their loop polygon (the follow-up
// for smooth-surface accuracy is per-face NURBS integration). Inner loops
// (holes) are not yet subtracted.
std::vector<Triangle> boundaryTriangles(const topo::Solid& solid) {
    std::vector<Triangle> tris;
    for (const auto& face : solid.faces()) {
        const topo::Wire* w = face.outerLoop;
        if (!w || !w->halfEdge) continue;

        std::vector<Vec3> loop;
        const topo::HalfEdge* start = w->halfEdge;
        const topo::HalfEdge* cur = start;
        do {
            if (cur && cur->origin) loop.push_back(cur->origin->point);
            cur = cur ? cur->next : nullptr;
        } while (cur && cur != start && loop.size() < 100000);

        for (size_t i = 1; i + 1 < loop.size(); ++i)
            tris.push_back({loop[0], loop[i], loop[i + 1]});
    }
    return tris;
}

// Face loops are not guaranteed to be wound consistently outward; orient every
// triangle away from an interior reference (the vertex average, inside any
// convex/star-shaped solid) so the divergence integrals share a sign.
void orientOutward(std::vector<Triangle>& tris) {
    Vec3 ref;
    for (const Triangle& t : tris) ref = ref + t[0] + t[1] + t[2];
    if (tris.empty()) return;
    ref = ref / static_cast<double>(tris.size() * 3);
    for (Triangle& t : tris) {
        const Vec3 ng = (t[1] - t[0]).cross(t[2] - t[0]);
        const Vec3 tc = (t[0] + t[1] + t[2]) / 3.0;
        if ((tc - ref).dot(ng) < 0.0) std::swap(t[1], t[2]);
    }
}

// Eberly, "Polyhedral Mass Properties (Revisited)": the 10 volume integrals
// {1, x, y, z, x², y², z², xy, yz, zx} over the meshed solid.
void subexpressions(double w0, double w1, double w2, double& f1, double& f2, double& f3, double& g0,
                    double& g1, double& g2) {
    const double t0 = w0 + w1;
    f1 = t0 + w2;
    const double t1 = w0 * w0;
    const double t2 = t1 + w1 * t0;
    f2 = t2 + w2 * f1;
    f3 = w0 * t1 + w1 * t2 + w2 * f2;
    g0 = f2 + w0 * (f1 + w0);
    g1 = f2 + w1 * (f1 + w1);
    g2 = f2 + w2 * (f1 + w2);
}

}  // namespace

MassProperties MassPropertiesCalculator::compute(const topo::Solid& solid,
                                                 const Material* material) {
    MassProperties props;
    props.density = material ? material->density : 1.0;

    std::vector<Triangle> tris = boundaryTriangles(solid);
    if (tris.size() < 4) return props;  // need a closed volume
    orientOutward(tris);

    constexpr double kOneDiv6 = 1.0 / 6.0;
    constexpr double kOneDiv24 = 1.0 / 24.0;
    constexpr double kOneDiv60 = 1.0 / 60.0;
    constexpr double kOneDiv120 = 1.0 / 120.0;
    const std::array<double, 10> mult = {kOneDiv6,  kOneDiv24, kOneDiv24,  kOneDiv24,  kOneDiv60,
                                         kOneDiv60, kOneDiv60, kOneDiv120, kOneDiv120, kOneDiv120};
    std::array<double, 10> intg = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
    double area = 0.0;

    for (const Triangle& t : tris) {
        const Vec3& p0 = t[0];
        const Vec3& p1 = t[1];
        const Vec3& p2 = t[2];

        area += 0.5 * (p1 - p0).cross(p2 - p0).length();

        const double a1 = p1.x - p0.x, b1 = p1.y - p0.y, c1 = p1.z - p0.z;
        const double a2 = p2.x - p0.x, b2 = p2.y - p0.y, c2 = p2.z - p0.z;
        const double d0 = b1 * c2 - b2 * c1;
        const double d1 = a2 * c1 - a1 * c2;
        const double d2 = a1 * b2 - a2 * b1;

        double f1x, f2x, f3x, g0x, g1x, g2x;
        double f1y, f2y, f3y, g0y, g1y, g2y;
        double f1z, f2z, f3z, g0z, g1z, g2z;
        subexpressions(p0.x, p1.x, p2.x, f1x, f2x, f3x, g0x, g1x, g2x);
        subexpressions(p0.y, p1.y, p2.y, f1y, f2y, f3y, g0y, g1y, g2y);
        subexpressions(p0.z, p1.z, p2.z, f1z, f2z, f3z, g0z, g1z, g2z);

        intg[0] += d0 * f1x;
        intg[1] += d0 * f2x;
        intg[2] += d1 * f2y;
        intg[3] += d2 * f2z;
        intg[4] += d0 * f3x;
        intg[5] += d1 * f3y;
        intg[6] += d2 * f3z;
        intg[7] += d0 * (p0.y * g0x + p1.y * g1x + p2.y * g2x);
        intg[8] += d1 * (p0.z * g0y + p1.z * g1y + p2.z * g2y);
        intg[9] += d2 * (p0.x * g0z + p1.x * g1z + p2.x * g2z);
    }

    if (intg[0] < 0.0) {  // normalize inward/outward sign to positive volume
        for (double& v : intg) v = -v;
    }
    for (size_t k = 0; k < 10; ++k) intg[k] *= mult[k];

    props.surfaceArea = area;
    props.volume = intg[0];
    if (props.volume <= 0.0) return props;  // degenerate

    const Vec3 cm(intg[1] / props.volume, intg[2] / props.volume, intg[3] / props.volume);
    props.centerOfMass = cm;

    const double rho = props.density;
    const double ixx = rho * (intg[5] + intg[6] - props.volume * (cm.y * cm.y + cm.z * cm.z));
    const double iyy = rho * (intg[4] + intg[6] - props.volume * (cm.z * cm.z + cm.x * cm.x));
    const double izz = rho * (intg[4] + intg[5] - props.volume * (cm.x * cm.x + cm.y * cm.y));
    const double ixy = -rho * (intg[7] - props.volume * cm.x * cm.y);
    const double iyz = -rho * (intg[8] - props.volume * cm.y * cm.z);
    const double izx = -rho * (intg[9] - props.volume * cm.z * cm.x);

    props.inertia.at(0, 0) = ixx;
    props.inertia.at(1, 1) = iyy;
    props.inertia.at(2, 2) = izz;
    props.inertia.at(0, 1) = props.inertia.at(1, 0) = ixy;
    props.inertia.at(1, 2) = props.inertia.at(2, 1) = iyz;
    props.inertia.at(0, 2) = props.inertia.at(2, 0) = izx;

    props.mass = rho * props.volume;
    props.valid = true;
    return props;
}

}  // namespace hz::model
