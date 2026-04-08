#pragma once

#include "horizon/math/Vec3.h"

#include <cstdint>
#include <vector>

namespace hz::topo {
class Solid;
}  // namespace hz::topo

namespace hz::model {

/// A single intersection point between two faces.
struct SSIPoint {
    math::Vec3 point;
    uint32_t faceIdA = 0;
    uint32_t faceIdB = 0;
};

/// An ordered chain of intersection points forming an intersection curve
/// between a pair of faces.
struct SSICurve {
    std::vector<SSIPoint> points;
    uint32_t faceIdA = 0;
    uint32_t faceIdB = 0;
};

/// Result of intersecting two solids: zero or more intersection curves.
struct SSIResult {
    std::vector<SSICurve> curves;
};

/// Computes the intersection curves between two B-Rep solids by tessellating
/// each face and performing triangle-triangle intersection tests.
///
/// A bounding-box R-tree pre-filter is used to skip face pairs that cannot
/// overlap.  The current implementation is tessellation-based and works
/// well for planar faces (box-box).
class SurfaceSurfaceIntersection {
public:
    static SSIResult compute(const topo::Solid& solidA, const topo::Solid& solidB,
                             double tolerance = 1e-6);
};

}  // namespace hz::model
