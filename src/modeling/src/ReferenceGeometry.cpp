#include "horizon/modeling/ReferenceGeometry.h"

#include <cmath>

#include "horizon/math/Quaternion.h"

namespace hz::model::refgeo {

using hz::math::Quaternion;
using hz::math::Vec3;

namespace {

constexpr double kEps = 1e-12;

// Return a unit vector perpendicular to n (choice is arbitrary but stable).
Vec3 anyPerpendicular(const Vec3& n) {
    Vec3 ref = (std::abs(n.x) < 0.9) ? Vec3::UnitX : Vec3::UnitY;
    Vec3 x = ref - n * ref.dot(n);
    return x.normalized();
}

// Orthogonalize a desired in-plane direction against the normal.
Vec3 inPlaneAxis(const Vec3& desired, const Vec3& normal) {
    Vec3 x = desired - normal * desired.dot(normal);
    if (x.length() < 1e-9) return anyPerpendicular(normal);
    return x.normalized();
}

}  // namespace

// --- Datum planes -----------------------------------------------------------

DatumPlane planeOffset(const DatumPlane& base, double offset) {
    Vec3 n = base.normal.normalized();
    return DatumPlane{base.origin + n * offset, n, base.xAxis.normalized()};
}

std::optional<DatumPlane> planeThroughPoints(const Vec3& p0, const Vec3& p1, const Vec3& p2) {
    Vec3 u = p1 - p0;
    Vec3 v = p2 - p0;
    Vec3 n = u.cross(v);
    if (n.length() < kEps) return std::nullopt;  // collinear or coincident
    Vec3 normal = n.normalized();
    return DatumPlane{p0, normal, inPlaneAxis(u, normal)};
}

DatumPlane planeAtAngle(const DatumPlane& base, const Vec3& hingeOrigin, const Vec3& hingeDir,
                        double angleRad) {
    Quaternion q = Quaternion::fromAxisAngle(hingeDir.normalized(), angleRad);
    Vec3 normal = q.rotate(base.normal.normalized()).normalized();
    Vec3 xAxis = inPlaneAxis(q.rotate(base.xAxis.normalized()), normal);
    return DatumPlane{hingeOrigin, normal, xAxis};
}

DatumPlane planeMidplane(const DatumPlane& a, const DatumPlane& b) {
    Vec3 na = a.normal.normalized();
    Vec3 nb = b.normal.normalized();
    if (na.dot(nb) < 0.0) nb = -nb;  // align orientation before averaging
    Vec3 n = na + nb;
    n = (n.length() < kEps) ? na : n.normalized();
    return DatumPlane{(a.origin + b.origin) * 0.5, n, inPlaneAxis(a.xAxis, n)};
}

// --- Datum axes -------------------------------------------------------------

std::optional<DatumAxis> axisThroughPoints(const Vec3& p0, const Vec3& p1) {
    Vec3 d = p1 - p0;
    if (d.length() < kEps) return std::nullopt;
    return DatumAxis{p0, d.normalized()};
}

std::optional<DatumAxis> axisPlaneIntersection(const DatumPlane& a, const DatumPlane& b) {
    Vec3 n1 = a.normal.normalized();
    Vec3 n2 = b.normal.normalized();
    Vec3 dir = n1.cross(n2);
    double len = dir.length();
    if (len < 1e-9) return std::nullopt;  // parallel planes
    // A point on the intersection line (standard two-plane formula).
    double d1 = n1.dot(a.origin);
    double d2 = n2.dot(b.origin);
    Vec3 p = (n2.cross(dir) * d1 + dir.cross(n1) * d2) / dir.dot(dir);
    return DatumAxis{p, dir / len};
}

DatumAxis axisFromDirection(const Vec3& base, const Vec3& dir) {
    return DatumAxis{base, dir.normalized()};
}

// --- Datum points -----------------------------------------------------------

DatumPoint pointAt(const Vec3& position) {
    return DatumPoint{position};
}

std::optional<DatumPoint> pointCentroid(const std::vector<Vec3>& points) {
    if (points.empty()) return std::nullopt;
    Vec3 c = Vec3::Zero;
    for (const auto& p : points) c = c + p;
    return DatumPoint{c / static_cast<double>(points.size())};
}

std::optional<DatumPoint> pointLineIntersection(const DatumAxis& a, const DatumAxis& b) {
    Vec3 u = a.direction.normalized();
    Vec3 v = b.direction.normalized();
    Vec3 w0 = a.origin - b.origin;
    double bb = u.dot(v);
    double denom = 1.0 - bb * bb;           // a·a = c·c = 1 for unit directions
    if (denom < kEps) return std::nullopt;  // parallel lines
    double d = u.dot(w0);
    double e = v.dot(w0);
    double s = (bb * e - d) / denom;
    double t = (e - bb * d) / denom;
    Vec3 pa = a.origin + u * s;
    Vec3 pb = b.origin + v * t;
    return DatumPoint{(pa + pb) * 0.5};
}

}  // namespace hz::model::refgeo
