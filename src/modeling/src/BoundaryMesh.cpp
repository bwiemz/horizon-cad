#include "horizon/modeling/BoundaryMesh.h"

#include <algorithm>
#include <cmath>
#include <cstddef>

#include "horizon/topology/Queries.h"
#include "horizon/topology/Solid.h"

namespace hz::model {

using hz::math::Vec3;

namespace {

/// Newell's method: robust normal (unnormalized) for a possibly non-convex,
/// possibly slightly non-planar loop.
Vec3 newellNormal(const std::vector<Vec3>& pts) {
    Vec3 n = Vec3::Zero;
    const size_t count = pts.size();
    for (size_t i = 0; i < count; ++i) {
        const Vec3& a = pts[i];
        const Vec3& b = pts[(i + 1) % count];
        n.x += (a.y - b.y) * (a.z + b.z);
        n.y += (a.z - b.z) * (a.x + b.x);
        n.z += (a.x - b.x) * (a.y + b.y);
    }
    return n;
}

struct Basis2D {
    Vec3 origin;
    Vec3 u;
    Vec3 v;
    bool valid = false;
};

/// Build an in-plane orthonormal basis for a polygon from its Newell normal.
Basis2D planeBasis(const std::vector<Vec3>& pts) {
    Basis2D basis;
    Vec3 n = newellNormal(pts);
    const double nLen = n.length();
    if (nLen < 1e-30) return basis;
    n = n / nLen;

    // First sufficiently long edge defines U.
    for (size_t i = 0; i < pts.size(); ++i) {
        Vec3 edge = pts[(i + 1) % pts.size()] - pts[i];
        edge = edge - n * edge.dot(n);  // project into plane
        if (edge.length() > 1e-12) {
            basis.origin = pts[0];
            basis.u = edge.normalized();
            basis.v = n.cross(basis.u);
            basis.valid = true;
            return basis;
        }
    }
    return basis;
}

struct P2 {
    double x, y;
};

inline double cross2(const P2& o, const P2& a, const P2& b) {
    return (a.x - o.x) * (b.y - o.y) - (a.y - o.y) * (b.x - o.x);
}

/// Point-in-triangle for the ear test (strictly inside within tolerance).
bool pointInTri2(const P2& p, const P2& a, const P2& b, const P2& c, double eps) {
    const double d1 = cross2(a, b, p);
    const double d2 = cross2(b, c, p);
    const double d3 = cross2(c, a, p);
    const bool hasNeg = (d1 < -eps) || (d2 < -eps) || (d3 < -eps);
    const bool hasPos = (d1 > eps) || (d2 > eps) || (d3 > eps);
    return !(hasNeg && hasPos);
}

}  // namespace

std::vector<std::array<Vec3, 3>> BoundaryMesh::triangulatePolygon(const std::vector<Vec3>& points) {
    std::vector<std::array<Vec3, 3>> tris;
    const size_t n = points.size();
    if (n < 3) return tris;
    if (n == 3) {
        tris.push_back({points[0], points[1], points[2]});
        return tris;
    }

    Basis2D basis = planeBasis(points);
    if (!basis.valid) {
        // Degenerate normal — emit a fan and let downstream drop zero-area triangles.
        for (size_t i = 1; i + 1 < n; ++i) tris.push_back({points[0], points[i], points[i + 1]});
        return tris;
    }

    // Project to 2D.
    std::vector<P2> pts2(n);
    double scale = 0.0;
    for (size_t i = 0; i < n; ++i) {
        const Vec3 d = points[i] - basis.origin;
        pts2[i] = {d.dot(basis.u), d.dot(basis.v)};
        scale = std::max(scale, std::max(std::abs(pts2[i].x), std::abs(pts2[i].y)));
    }
    const double areaEps = std::max(1e-30, scale * scale * 1e-14);

    // Work on an index list so output triangles use the original 3D points.
    std::vector<size_t> idx(n);
    for (size_t i = 0; i < n; ++i) idx[i] = i;

    // The projected polygon is CCW by construction (basis derived from the
    // loop's own Newell normal), but guard against numerical sign flips.
    double area2 = 0.0;
    for (size_t i = 0; i < n; ++i) {
        const P2& a = pts2[i];
        const P2& b = pts2[(i + 1) % n];
        area2 += a.x * b.y - b.x * a.y;
    }
    const bool reversed = area2 < 0.0;
    if (reversed) {
        for (size_t i = 0; i < n / 2; ++i) std::swap(idx[i], idx[n - 1 - i]);
    }

    auto emit = [&](size_t a, size_t b, size_t c) {
        // Preserve the caller's original winding.
        if (reversed) {
            tris.push_back({points[c], points[b], points[a]});
        } else {
            tris.push_back({points[a], points[b], points[c]});
        }
    };

    size_t guard = 0;
    const size_t maxIterations = n * n + 16;
    while (idx.size() > 3 && guard++ < maxIterations) {
        bool clipped = false;
        const size_t m = idx.size();
        for (size_t i = 0; i < m; ++i) {
            const size_t iPrev = idx[(i + m - 1) % m];
            const size_t iCur = idx[i];
            const size_t iNext = idx[(i + 1) % m];
            const P2& a = pts2[iPrev];
            const P2& b = pts2[iCur];
            const P2& c = pts2[iNext];

            const double convexity = cross2(a, b, c);
            if (convexity < areaEps) continue;  // reflex or degenerate corner

            bool containsOther = false;
            for (size_t j = 0; j < m; ++j) {
                const size_t iOther = idx[j];
                if (iOther == iPrev || iOther == iCur || iOther == iNext) continue;
                if (pointInTri2(pts2[iOther], a, b, c, areaEps)) {
                    containsOther = true;
                    break;
                }
            }
            if (containsOther) continue;

            emit(iPrev, iCur, iNext);
            idx.erase(idx.begin() + static_cast<std::ptrdiff_t>(i));
            clipped = true;
            break;
        }
        if (!clipped) {
            // No ear found (collinear runs or numerically flat region) —
            // fall back to a fan over the remainder.
            for (size_t i = 1; i + 1 < idx.size(); ++i) emit(idx[0], idx[i], idx[i + 1]);
            return tris;
        }
    }
    if (idx.size() == 3) emit(idx[0], idx[1], idx[2]);
    return tris;
}

double BoundaryMesh::signedVolume(const std::vector<BoundaryPolygon>& polygons) {
    double vol6 = 0.0;
    for (const auto& poly : polygons) {
        const auto& p = poly.points;
        for (size_t i = 1; i + 1 < p.size(); ++i) {
            vol6 += p[0].dot(p[i].cross(p[i + 1]));
        }
    }
    return vol6 / 6.0;
}

std::vector<BoundaryPolygon> BoundaryMesh::extractFacePolygons(const topo::Solid& solid) {
    std::vector<BoundaryPolygon> polygons;
    polygons.reserve(solid.faceCount());

    for (const auto& face : solid.faces()) {
        auto verts = topo::faceVertices(&face);
        if (verts.size() < 3) continue;

        BoundaryPolygon poly;
        poly.points.reserve(verts.size());
        for (const auto* v : verts) poly.points.push_back(v->point);
        poly.topoId = face.topoId;
        poly.surface = face.surface;
        polygons.push_back(std::move(poly));
    }

    // Manifold twin-linking makes all loops consistently wound up to one
    // global inward/outward sign (see MassProperties) — normalize to outward.
    if (signedVolume(polygons) < 0.0) {
        for (auto& poly : polygons) {
            std::reverse(poly.points.begin(), poly.points.end());
        }
    }

    return polygons;
}

}  // namespace hz::model
