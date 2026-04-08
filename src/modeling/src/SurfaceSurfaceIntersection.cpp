#include "horizon/modeling/SurfaceSurfaceIntersection.h"

#include "horizon/geometry/surfaces/NurbsSurface.h"
#include "horizon/math/BoundingBox.h"
#include "horizon/math/RTree.h"
#include "horizon/topology/Solid.h"

#include <cmath>
#include <unordered_map>

namespace hz::model {

namespace {

/// Compute bounding box from tessellation positions.
math::BoundingBox computeFaceBBox(const geo::TessellationResult& tess) {
    math::BoundingBox bbox;
    for (size_t i = 0; i + 2 < tess.positions.size(); i += 3) {
        bbox.expand(math::Vec3(tess.positions[i], tess.positions[i + 1], tess.positions[i + 2]));
    }
    return bbox;
}

/// Extract a Vec3 vertex from a flat position array.
inline math::Vec3 getVertex(const std::vector<float>& pos, uint32_t idx) {
    return {pos[idx * 3], pos[idx * 3 + 1], pos[idx * 3 + 2]};
}

/// Compute the intersection of a line segment (p0->p1) with a plane defined by
/// (planePoint, planeNormal).  Returns true if there is an intersection in [0,1]
/// and sets `out` to the intersection point.
bool segmentPlaneIntersect(const math::Vec3& p0, const math::Vec3& p1,
                           const math::Vec3& planePoint, const math::Vec3& planeNormal,
                           math::Vec3& out) {
    math::Vec3 dir = p1 - p0;
    double denom = dir.dot(planeNormal);
    if (std::abs(denom) < 1e-15) return false;  // Parallel.
    double t = (planePoint - p0).dot(planeNormal) / denom;
    if (t < -1e-10 || t > 1.0 + 1e-10) return false;
    out = p0 + dir * t;
    return true;
}

/// Test whether a point lies inside a triangle (v0, v1, v2) using barycentric coords.
/// The triangle must be non-degenerate.
bool pointInTriangle(const math::Vec3& p, const math::Vec3& v0, const math::Vec3& v1,
                     const math::Vec3& v2, double tol) {
    math::Vec3 e0 = v1 - v0;
    math::Vec3 e1 = v2 - v0;
    math::Vec3 ep = p - v0;

    double d00 = e0.dot(e0);
    double d01 = e0.dot(e1);
    double d11 = e1.dot(e1);
    double dp0 = ep.dot(e0);
    double dp1 = ep.dot(e1);

    double denom = d00 * d11 - d01 * d01;
    if (std::abs(denom) < 1e-30) return false;

    double u = (d11 * dp0 - d01 * dp1) / denom;
    double v = (d00 * dp1 - d01 * dp0) / denom;

    return u >= -tol && v >= -tol && (u + v) <= 1.0 + tol;
}

/// Compute the intersection segment between two triangles.
/// Returns 0, 1, or 2 intersection points in `out`.
int triangleTriangleIntersect(const math::Vec3& a0, const math::Vec3& a1, const math::Vec3& a2,
                              const math::Vec3& b0, const math::Vec3& b1, const math::Vec3& b2,
                              math::Vec3 out[2]) {
    int count = 0;

    // Compute plane of triangle B.
    math::Vec3 bnorm = (b1 - b0).cross(b2 - b0);
    double bnormLen = bnorm.length();
    if (bnormLen < 1e-15) return 0;
    bnorm = bnorm / bnormLen;

    // Test edges of A against plane of B.
    const math::Vec3* aVerts[3] = {&a0, &a1, &a2};
    for (int i = 0; i < 3 && count < 2; ++i) {
        math::Vec3 pt;
        if (segmentPlaneIntersect(*aVerts[i], *aVerts[(i + 1) % 3], b0, bnorm, pt)) {
            if (pointInTriangle(pt, b0, b1, b2, 1e-8)) {
                // Check for duplicate with existing point.
                bool dup = false;
                for (int k = 0; k < count; ++k) {
                    if (pt.distanceTo(out[k]) < 1e-10) {
                        dup = true;
                        break;
                    }
                }
                if (!dup) out[count++] = pt;
            }
        }
    }

    // Compute plane of triangle A.
    math::Vec3 anorm = (a1 - a0).cross(a2 - a0);
    double anormLen = anorm.length();
    if (anormLen < 1e-15) return count;
    anorm = anorm / anormLen;

    // Test edges of B against plane of A.
    const math::Vec3* bVerts[3] = {&b0, &b1, &b2};
    for (int i = 0; i < 3 && count < 2; ++i) {
        math::Vec3 pt;
        if (segmentPlaneIntersect(*bVerts[i], *bVerts[(i + 1) % 3], a0, anorm, pt)) {
            if (pointInTriangle(pt, a0, a1, a2, 1e-8)) {
                bool dup = false;
                for (int k = 0; k < count; ++k) {
                    if (pt.distanceTo(out[k]) < 1e-10) {
                        dup = true;
                        break;
                    }
                }
                if (!dup) out[count++] = pt;
            }
        }
    }

    return count;
}

/// Key for face-pair deduplication.
struct FacePairKey {
    uint32_t a, b;
    bool operator==(const FacePairKey& o) const { return a == o.a && b == o.b; }
};

struct FacePairHash {
    size_t operator()(const FacePairKey& k) const {
        return std::hash<uint64_t>()(static_cast<uint64_t>(k.a) << 32 | k.b);
    }
};

}  // namespace

SSIResult SurfaceSurfaceIntersection::compute(const topo::Solid& solidA,
                                              const topo::Solid& solidB, double tolerance) {
    SSIResult result;

    // Use the tolerance as the tessellation chord-height tolerance.
    // Clamp to a reasonable minimum so we don't produce enormous meshes.
    double tessTol = std::max(tolerance, 0.01);

    // Tessellate all faces of both solids and compute bounding boxes.
    struct FaceTess {
        uint32_t faceId = 0;
        geo::TessellationResult tess;
        math::BoundingBox bbox;
    };

    std::vector<FaceTess> tessA;
    for (const auto& face : solidA.faces()) {
        if (!face.surface) continue;
        FaceTess ft;
        ft.faceId = face.id;
        ft.tess = face.surface->tessellate(tessTol);
        ft.bbox = computeFaceBBox(ft.tess);
        tessA.push_back(std::move(ft));
    }

    std::vector<FaceTess> tessB;
    math::RTree<size_t> treeB;
    for (size_t i = 0; i < solidB.faces().size(); ++i) {
        const auto& face = solidB.faces()[i];
        if (!face.surface) continue;
        FaceTess ft;
        ft.faceId = face.id;
        ft.tess = face.surface->tessellate(tessTol);
        ft.bbox = computeFaceBBox(ft.tess);
        treeB.insert(tessB.size(), ft.bbox);
        tessB.push_back(std::move(ft));
    }

    // For each face pair, collect intersection points and build curves.
    std::unordered_map<FacePairKey, std::vector<math::Vec3>, FacePairHash> pairPoints;

    for (const auto& fA : tessA) {
        auto candidates = treeB.query(fA.bbox);
        for (size_t bIdx : candidates) {
            const auto& fB = tessB[bIdx];

            const auto& posA = fA.tess.positions;
            const auto& idxA = fA.tess.indices;
            const auto& posB = fB.tess.positions;
            const auto& idxB = fB.tess.indices;

            FacePairKey key{fA.faceId, fB.faceId};

            // Test all triangle pairs.
            for (size_t ia = 0; ia + 2 < idxA.size(); ia += 3) {
                math::Vec3 a0 = getVertex(posA, idxA[ia]);
                math::Vec3 a1 = getVertex(posA, idxA[ia + 1]);
                math::Vec3 a2 = getVertex(posA, idxA[ia + 2]);

                for (size_t ib = 0; ib + 2 < idxB.size(); ib += 3) {
                    math::Vec3 b0 = getVertex(posB, idxB[ib]);
                    math::Vec3 b1 = getVertex(posB, idxB[ib + 1]);
                    math::Vec3 b2 = getVertex(posB, idxB[ib + 2]);

                    math::Vec3 pts[2];
                    int n = triangleTriangleIntersect(a0, a1, a2, b0, b1, b2, pts);
                    for (int k = 0; k < n; ++k) {
                        pairPoints[key].push_back(pts[k]);
                    }
                }
            }
        }
    }

    // Convert collected points to SSICurves.
    for (auto& [key, pts] : pairPoints) {
        if (pts.empty()) continue;

        // Deduplicate points.
        std::vector<math::Vec3> unique;
        unique.reserve(pts.size());
        for (const auto& p : pts) {
            bool dup = false;
            for (const auto& u : unique) {
                if (p.distanceTo(u) < tolerance) {
                    dup = true;
                    break;
                }
            }
            if (!dup) unique.push_back(p);
        }

        if (unique.empty()) continue;

        SSICurve curve;
        curve.faceIdA = key.a;
        curve.faceIdB = key.b;
        for (const auto& p : unique) {
            SSIPoint sp;
            sp.point = p;
            sp.faceIdA = key.a;
            sp.faceIdB = key.b;
            curve.points.push_back(sp);
        }
        result.curves.push_back(std::move(curve));
    }

    return result;
}

}  // namespace hz::model
