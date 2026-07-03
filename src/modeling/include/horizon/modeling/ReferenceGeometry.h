#pragma once

#include <optional>
#include <vector>

#include "horizon/drafting/SketchPlane.h"
#include "horizon/math/Vec3.h"

namespace hz::model {

/// A construction plane: an origin and an orthonormal frame
/// (normal, xAxis, yAxis = normal × xAxis). Consumed by sketches and features
/// but does not contribute to the solid body.
struct DatumPlane {
    math::Vec3 origin;
    math::Vec3 normal;  ///< unit
    math::Vec3 xAxis;   ///< unit, in-plane reference direction

    math::Vec3 yAxis() const { return normal.cross(xAxis); }

    /// The equivalent sketch plane (a Sketch can be built on it).
    draft::SketchPlane toSketchPlane() const { return draft::SketchPlane(origin, normal, xAxis); }
};

/// A construction axis: an infinite line through @c origin along @c direction.
struct DatumAxis {
    math::Vec3 origin;
    math::Vec3 direction;  ///< unit
};

/// A construction point.
struct DatumPoint {
    math::Vec3 position;
};

/// Construction-geometry constructions.
///
/// Every method captures resolved coordinates (Era-2 scope) rather than a live
/// reference to a model face/edge/vertex. Fallible constructions (degenerate
/// inputs) return `std::nullopt`.
namespace refgeo {

// --- Datum planes -----------------------------------------------------------

/// Plane parallel to @p base, shifted @p offset along its normal.
DatumPlane planeOffset(const DatumPlane& base, double offset);

/// Plane through three points: @p p0 is the origin, xAxis points toward @p p1,
/// normal = (p1-p0) × (p2-p0). Returns nullopt if the points are collinear.
std::optional<DatumPlane> planeThroughPoints(const math::Vec3& p0, const math::Vec3& p1,
                                             const math::Vec3& p2);

/// Plane containing the hinge line (@p hingeOrigin, @p hingeDir), rotated
/// @p angleRad from @p base about that hinge. The datum's origin is the hinge
/// origin so the plane contains the hinge line.
DatumPlane planeAtAngle(const DatumPlane& base, const math::Vec3& hingeOrigin,
                        const math::Vec3& hingeDir, double angleRad);

/// Midplane between two (near-parallel) planes: origins are averaged and the
/// orientation-aligned normals are averaged.
DatumPlane planeMidplane(const DatumPlane& a, const DatumPlane& b);

// --- Datum axes -------------------------------------------------------------

/// Axis through two distinct points. Returns nullopt if the points coincide.
std::optional<DatumAxis> axisThroughPoints(const math::Vec3& p0, const math::Vec3& p1);

/// Axis along the intersection line of two planes. Returns nullopt if the
/// planes are parallel.
std::optional<DatumAxis> axisPlaneIntersection(const DatumPlane& a, const DatumPlane& b);

/// Axis captured directly from a cylinder's base point and direction.
DatumAxis axisFromDirection(const math::Vec3& base, const math::Vec3& dir);

// --- Datum points -----------------------------------------------------------

/// Point at a given position (e.g. a vertex).
DatumPoint pointAt(const math::Vec3& position);

/// Centroid of a set of points (e.g. a face's boundary vertices).
/// Returns nullopt if the set is empty.
std::optional<DatumPoint> pointCentroid(const std::vector<math::Vec3>& points);

/// Midpoint of the shortest segment between two lines (edge "intersection").
/// Returns nullopt if the lines are parallel.
std::optional<DatumPoint> pointLineIntersection(const DatumAxis& a, const DatumAxis& b);

}  // namespace refgeo

}  // namespace hz::model
