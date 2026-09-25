#include "horizon/modeling/BoundaryMesh.h"

#include <algorithm>
#include <cmath>
#include <cstddef>

#include "horizon/geometry/curves/NurbsCurve.h"
#include "horizon/topology/HalfEdge.h"
#include "horizon/topology/Queries.h"
#include "horizon/topology/Solid.h"

namespace hz::model {

using hz::math::Vec3;

namespace {

/// Newell's method: robust normal (unnormalized) for a possibly non-convex,
/// possibly slightly non-planar loop; summed about the first point, so its
/// error does not grow with the loop's distance from the origin.
Vec3 newellNormal(const std::vector<Vec3>& pts) {
    Vec3 n = Vec3::Zero;
    const size_t count = pts.size();
    for (size_t i = 0; i < count; ++i) {
        const Vec3 a = pts[i] - pts[0];
        const Vec3 b = pts[(i + 1) % count] - pts[0];
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

bool samePoint(const P2& a, const P2& b) {
    return a.x == b.x && a.y == b.y;
}

/// Whether segments ab and cd cross at a point inside both. Segments that
/// only touch at an end, or run along each other, do not.
bool segmentsCross(const P2& a, const P2& b, const P2& c, const P2& d) {
    const double d1 = cross2(c, d, a);
    const double d2 = cross2(c, d, b);
    const double d3 = cross2(a, b, c);
    const double d4 = cross2(a, b, d);
    return ((d1 > 0.0 && d2 < 0.0) || (d1 < 0.0 && d2 > 0.0)) &&
           ((d3 > 0.0 && d4 < 0.0) || (d3 < 0.0 && d4 > 0.0));
}

/// Twice the signed area of a 2D loop: positive when counterclockwise.
double area2Of(const std::vector<P2>& loop) {
    double area = 0.0;
    for (size_t i = 0; i < loop.size(); ++i) {
        const P2& a = loop[i];
        const P2& b = loop[(i + 1) % loop.size()];
        area += a.x * b.y - b.x * a.y;
    }
    return area;
}

}  // namespace

std::vector<Vec3> BoundaryMesh::loopPoints(const topo::Wire& wire, bool alongCurves) {
    std::vector<Vec3> pts;
    const topo::HalfEdge* start = wire.halfEdge;
    const topo::HalfEdge* cur = start;
    if (cur == nullptr) return pts;
    do {
        if (cur->origin == nullptr) return {};
        pts.push_back(cur->origin->point);
        const topo::Edge* edge = cur->edge;
        const geo::NurbsCurve* curve = edge ? edge->curve.get() : nullptr;
        if (alongCurves && curve != nullptr && curve->degree() > 1) {
            // Run along the curve the way the loop does: from the end nearer
            // this half-edge's origin. A closed edge has both ends there, and
            // its direction is settled by the caller's winding.
            const Vec3& from = cur->origin->point;
            const bool reversed = (curve->evaluate(curve->tMax()) - from).length() <
                                  (curve->evaluate(curve->tMin()) - from).length();
            constexpr int kSamples = 16;
            for (int k = 1; k < kSamples; ++k) {
                const double f = static_cast<double>(k) / kSamples;
                const double t = reversed ? curve->tMax() + (curve->tMin() - curve->tMax()) * f
                                          : curve->tMin() + (curve->tMax() - curve->tMin()) * f;
                pts.push_back(curve->evaluate(t));
            }
        }
        cur = cur->next;
    } while (cur != nullptr && cur != start && pts.size() < 100000);
    return pts;
}

std::vector<Vec3> BoundaryMesh::keyholePolygon(const std::vector<Vec3>& outer,
                                               std::vector<std::vector<Vec3>> holes) {
    const Basis2D basis = planeBasis(outer);
    if (!basis.valid) return outer;
    const auto project = [&basis](const Vec3& p) {
        const Vec3 d = p - basis.origin;
        return P2{d.dot(basis.u), d.dot(basis.v)};
    };

    // The polygon as it grows, in 3D and projected. The outer loop is
    // counterclockwise in its own basis; each hole is wound the other way.
    std::vector<Vec3> poly = outer;
    std::vector<P2> poly2;
    poly2.reserve(poly.size());
    for (const Vec3& p : poly) poly2.push_back(project(p));

    struct Hole {
        std::vector<Vec3> pts;
        std::vector<P2> pts2;
    };
    std::vector<Hole> pending;
    for (auto& h : holes) {
        if (h.size() < 3) continue;
        Hole hole{std::move(h), {}};
        for (const Vec3& p : hole.pts) hole.pts2.push_back(project(p));
        if (area2Of(hole.pts2) > 0.0) {
            std::reverse(hole.pts.begin(), hole.pts.end());
            std::reverse(hole.pts2.begin(), hole.pts2.end());
        }
        pending.push_back(std::move(hole));
    }
    // Rightmost holes first, as in ear-cut hole elimination: a bridge to a
    // hole further right never has to cross one still waiting.
    const auto maxX = [](const Hole& h) {
        double m = h.pts2.front().x;
        for (const P2& p : h.pts2) m = std::max(m, p.x);
        return m;
    };
    std::sort(pending.begin(), pending.end(),
              [&maxX](const Hole& a, const Hole& b) { return maxX(a) > maxX(b); });

    // Whether the bridge a-b crosses any edge of the polygon or of a hole.
    const auto blocked = [&](const P2& a, const P2& b, size_t from) {
        const auto crossesLoop = [&](const std::vector<P2>& loop) {
            for (size_t i = 0; i < loop.size(); ++i) {
                if (segmentsCross(a, b, loop[i], loop[(i + 1) % loop.size()])) return true;
            }
            return false;
        };
        if (crossesLoop(poly2)) return true;
        for (size_t k = from; k < pending.size(); ++k) {
            if (crossesLoop(pending[k].pts2)) return true;
        }
        return false;
    };

    for (size_t k = 0; k < pending.size(); ++k) {
        const Hole& hole = pending[k];
        // The hole's rightmost point, bridged to the nearest point of the
        // polygon the bridge reaches without crossing anything.
        size_t h = 0;
        for (size_t i = 1; i < hole.pts2.size(); ++i) {
            if (hole.pts2[i].x > hole.pts2[h].x) h = i;
        }
        const P2& hp = hole.pts2[h];
        size_t best = poly2.size();
        double bestDist = 0.0;
        for (size_t i = 0; i < poly2.size(); ++i) {
            const double dx = poly2[i].x - hp.x;
            const double dy = poly2[i].y - hp.y;
            const double dist = dx * dx + dy * dy;
            if (best != poly2.size() && dist >= bestDist) continue;
            if (blocked(hp, poly2[i], k)) continue;
            // The bridge must leave the polygon's vertex into its inside:
            // between the edges that meet there.
            const size_t n = poly2.size();
            const P2& prev = poly2[(i + n - 1) % n];
            const P2& next = poly2[(i + 1) % n];
            const bool convex = cross2(prev, poly2[i], next) >= 0.0;
            const bool leftOfPrev = cross2(prev, poly2[i], hp) > 0.0;
            const bool leftOfNext = cross2(poly2[i], next, hp) > 0.0;
            if (convex ? !(leftOfPrev && leftOfNext) : !(leftOfPrev || leftOfNext)) continue;
            best = i;
            bestDist = dist;
        }
        if (best == poly2.size()) continue;  // unreachable: left out

        // ..., p, h, h+1, ..., h-1, h, p, ...
        std::vector<Vec3> spliced;
        std::vector<P2> spliced2;
        spliced.reserve(poly.size() + hole.pts.size() + 2);
        spliced2.reserve(poly.size() + hole.pts.size() + 2);
        for (size_t i = 0; i <= best; ++i) {
            spliced.push_back(poly[i]);
            spliced2.push_back(poly2[i]);
        }
        for (size_t j = 0; j <= hole.pts.size(); ++j) {
            const size_t at = (h + j) % hole.pts.size();
            spliced.push_back(hole.pts[at]);
            spliced2.push_back(hole.pts2[at]);
        }
        spliced.push_back(poly[best]);
        spliced2.push_back(poly2[best]);
        for (size_t i = best + 1; i < poly.size(); ++i) {
            spliced.push_back(poly[i]);
            spliced2.push_back(poly2[i]);
        }
        poly = std::move(spliced);
        poly2 = std::move(spliced2);
    }
    return poly;
}

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
                // A point at one of the ear's corners — a keyhole polygon
                // visits each bridge end twice — does not block it.
                const P2& o = pts2[iOther];
                if (samePoint(o, a) || samePoint(o, b) || samePoint(o, c)) continue;
                if (pointInTri2(o, a, b, c, areaEps)) {
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
    // About a point of the solid's own, not the origin: a triple product's
    // rounding grows as the cube of the distance, and a million out it was
    // larger than a box's volume, so the box was sometimes found inside out.
    Vec3 o;
    for (const auto& poly : polygons) {
        if (!poly.points.empty()) {
            o = poly.points[0];
            break;
        }
    }
    double vol6 = 0.0;
    for (const auto& poly : polygons) {
        const auto& p = poly.points;
        for (size_t i = 1; i + 1 < p.size(); ++i) {
            vol6 += (p[0] - o).dot((p[i] - o).cross(p[i + 1] - o));
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
        if (!face.innerLoops.empty()) {
            std::vector<std::vector<Vec3>> holes;
            for (const topo::Wire* inner : face.innerLoops) {
                if (inner) holes.push_back(loopPoints(*inner, true));
            }
            poly.points = keyholePolygon(poly.points, std::move(holes));
        }
        poly.topoId = face.topoId;
        poly.surface = face.surface;
        poly.analyticSurface = face.analyticSurface;
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
