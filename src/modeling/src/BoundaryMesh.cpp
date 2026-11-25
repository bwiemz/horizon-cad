#include "horizon/modeling/BoundaryMesh.h"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <vector>

#include "horizon/geometry/curves/NurbsCurve.h"
#include "horizon/modeling/TrimmedFaceMesher.h"
#include "horizon/topology/ExactTolerance.h"
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

std::vector<std::array<size_t, 3>> BoundaryMesh::triangulatePolygonCorners(
    const std::vector<Vec3>& points) {
    std::vector<std::array<size_t, 3>> tris;
    const size_t n = points.size();
    if (n < 3) return tris;
    if (n == 3) {
        tris.push_back({0, 1, 2});
        return tris;
    }

    Basis2D basis = planeBasis(points);
    if (!basis.valid) {
        // Degenerate normal — emit a fan and let downstream drop zero-area triangles.
        for (size_t i = 1; i + 1 < n; ++i) tris.push_back({0, i, i + 1});
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

    // The order the loop is clipped in: its own, or reversed so it runs
    // counterclockwise. Output triangles use the original 3D points.
    std::vector<size_t> order(n);
    for (size_t i = 0; i < n; ++i) order[i] = i;

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
        for (size_t i = 0; i < n / 2; ++i) std::swap(order[i], order[n - 1 - i]);
    }

    auto emit = [&](size_t a, size_t b, size_t c) {
        // Preserve the caller's original winding.
        if (reversed) {
            tris.push_back({c, b, a});
        } else {
            tris.push_back({a, b, c});
        }
    };

    // The loop still to clip, as a ring of positions in `order`, walked from
    // the first left: the order the clipping takes ears in is as when it
    // erased each from a list, and so are the triangles. (Erasing moved the
    // rest of the list each time.)
    std::vector<size_t> nextOf(n);
    std::vector<size_t> prevOf(n);
    for (size_t k = 0; k < n; ++k) {
        nextOf[k] = (k + 1) % n;
        prevOf[k] = (k + n - 1) % n;
    }
    size_t first = 0;
    size_t left = n;

    // The points by where they are, on a grid of about two to a cell: an
    // ear is tested against the points that could lie in it, not all of
    // them. Which those are follows from the test (pointInTri2): with the
    // ear's corners at barycentric weights, it holds where every weight is
    // above -eps / (twice the area), or, for an ear almost flat, every
    // weight below eps / (twice the area); outside both triangles so
    // bounded no point passes. A loop of N points was clipped in time N
    // squared at best, and one of 30,000 took seconds.
    double lowX = pts2[0].x, lowY = pts2[0].y, highX = pts2[0].x, highY = pts2[0].y;
    for (const P2& p : pts2) {
        lowX = std::min(lowX, p.x);
        lowY = std::min(lowY, p.y);
        highX = std::max(highX, p.x);
        highY = std::max(highY, p.y);
    }
    const size_t side = std::max<size_t>(
        1, static_cast<size_t>(std::ceil(std::sqrt(static_cast<double>(n) / 2.0))));
    const double spanX = std::max(highX - lowX, 1e-300);
    const double spanY = std::max(highY - lowY, 1e-300);
    const auto cellOf = [&](double v, double low, double span) {
        const double c = std::floor((v - low) / span * static_cast<double>(side));
        if (!(c > 0.0)) return size_t{0};  // below, or not a number
        return std::min(static_cast<size_t>(c), side - 1);
    };
    std::vector<std::vector<size_t>> cells(side * side);
    for (size_t i = 0; i < n; ++i) {
        cells[cellOf(pts2[i].y, lowY, spanY) * side + cellOf(pts2[i].x, lowX, spanX)].push_back(i);
    }
    std::vector<bool> gone(n, false);  // by point: clipped off the loop
    // Whether any point still on the loop, other than the ear's own corners
    // (or a point where one of them is, as a keyhole repeats its bridge's
    // ends), lies in the ear abc, whose doubled area is `twice`.
    const auto blocked = [&](size_t iPrev, size_t iCur, size_t iNext, double twice) {
        const P2& a = pts2[iPrev];
        const P2& b = pts2[iCur];
        const P2& c = pts2[iNext];
        double boxLowX = std::numeric_limits<double>::infinity();
        double boxLowY = boxLowX;
        double boxHighX = -boxLowX;
        double boxHighY = -boxLowX;
        const auto include = [&](double x, double y) {
            boxLowX = std::min(boxLowX, x);
            boxLowY = std::min(boxLowY, y);
            boxHighX = std::max(boxHighX, x);
            boxHighY = std::max(boxHighY, y);
        };
        const double t = areaEps / twice;
        const P2* corner[3] = {&a, &b, &c};
        for (int k = 0; k < 3; ++k) {
            const P2& v = *corner[k];
            const P2& u = *corner[(k + 1) % 3];
            const P2& w = *corner[(k + 2) % 3];
            // Weight 1 + 2t here and -t at the others; and, almost flat,
            // 1 - 2t here and t at the others.
            include(v.x * (1 + 2 * t) - t * (u.x + w.x), v.y * (1 + 2 * t) - t * (u.y + w.y));
            if (3.0 * t >= 1.0) {
                include(v.x * (1 - 2 * t) + t * (u.x + w.x), v.y * (1 - 2 * t) + t * (u.y + w.y));
            }
        }
        // And more, for rounding in the test: the box need only hold every
        // point that could pass, and a few more cost little.
        const double slack = 1e-6 * scale + 1e-300;
        const size_t x0 = cellOf(boxLowX - slack, lowX, spanX);
        const size_t x1 = cellOf(boxHighX + slack, lowX, spanX);
        const size_t y0 = cellOf(boxLowY - slack, lowY, spanY);
        const size_t y1 = cellOf(boxHighY + slack, lowY, spanY);
        const bool everywhere = !std::isfinite(boxLowX + boxLowY + boxHighX + boxHighY);
        for (size_t cy = everywhere ? 0 : y0; cy <= (everywhere ? side - 1 : y1); ++cy) {
            for (size_t cx = everywhere ? 0 : x0; cx <= (everywhere ? side - 1 : x1); ++cx) {
                for (const size_t iOther : cells[cy * side + cx]) {
                    if (gone[iOther]) continue;
                    if (iOther == iPrev || iOther == iCur || iOther == iNext) continue;
                    const P2& o = pts2[iOther];
                    if (samePoint(o, a) || samePoint(o, b) || samePoint(o, c)) continue;
                    if (pointInTri2(o, a, b, c, areaEps)) return true;
                }
            }
        }
        return false;
    };

    size_t guard = 0;
    const size_t maxIterations = n * n + 16;
    while (left > 3 && guard++ < maxIterations) {
        bool clipped = false;
        size_t k = first;
        for (size_t step = 0; step < left; ++step, k = nextOf[k]) {
            const size_t iPrev = order[prevOf[k]];
            const size_t iCur = order[k];
            const size_t iNext = order[nextOf[k]];
            const P2& a = pts2[iPrev];
            const P2& b = pts2[iCur];
            const P2& c = pts2[iNext];

            const double convexity = cross2(a, b, c);
            if (convexity < areaEps) continue;  // reflex or degenerate corner

            // A point at one of the ear's corners — a keyhole polygon
            // visits each bridge end twice — does not block it.
            if (blocked(iPrev, iCur, iNext, convexity)) continue;

            emit(iPrev, iCur, iNext);
            gone[iCur] = true;
            nextOf[prevOf[k]] = nextOf[k];
            prevOf[nextOf[k]] = prevOf[k];
            if (k == first) first = nextOf[k];
            --left;
            clipped = true;
            break;
        }
        if (!clipped) {
            // No ear found (collinear runs or numerically flat region) —
            // fall back to a fan over the remainder.
            const size_t fanFrom = order[first];
            size_t at = nextOf[first];
            for (size_t i = 1; i + 1 < left; ++i, at = nextOf[at]) {
                emit(fanFrom, order[at], order[nextOf[at]]);
            }
            return tris;
        }
    }
    if (left == 3) emit(order[first], order[nextOf[first]], order[nextOf[nextOf[first]]]);
    return tris;
}

std::vector<std::array<Vec3, 3>> BoundaryMesh::triangulatePolygon(const std::vector<Vec3>& points) {
    std::vector<std::array<Vec3, 3>> tris;
    const auto corners = triangulatePolygonCorners(points);
    tris.reserve(corners.size());
    for (const auto& [a, b, c] : corners) tris.push_back({points[a], points[b], points[c]});
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

double BoundaryMesh::defaultChordTolerance(const topo::Solid& solid) {
    if (!topo::hasExactFaces(solid)) return 0.0;
    // 1e-4 of the part's size, the size its model tolerance comes from.
    const double size = topo::exactExtent(solid);
    return size > 0.0 ? 1e-4 * size : 1e-4;
}

std::vector<BoundaryPolygon> BoundaryMesh::extractFacePolygons(const topo::Solid& solid) {
    return extractFacePolygons(solid, defaultChordTolerance(solid));
}

std::vector<BoundaryPolygon> BoundaryMesh::extractFacePolygons(const topo::Solid& solid,
                                                               double chordTolerance,
                                                               std::string* problem,
                                                               const std::atomic<bool>* cancelled) {
    if (problem != nullptr) problem->clear();
    std::vector<BoundaryPolygon> polygons;
    polygons.reserve(solid.faceCount());

    // The exact faces, meshed in one pass (they share their edges' points): by face.
    std::unordered_map<const topo::Face*, FaceMesh> meshes;
    std::string firstProblem;
    if (topo::hasExactFaces(solid)) {
        MeshOptions options;
        options.chordTolerance = chordTolerance;
        options.cancelled = cancelled;
        for (auto& mesh : TrimmedFaceMesher::meshSolid(solid, options)) {
            if (mesh.face == nullptr || !mesh.face->exactSurface) continue;
            if (!mesh.problem.empty() && firstProblem.empty()) {
                firstProblem = mesh.face->topoId.tag() + ": " + mesh.problem;
            }
            meshes.emplace(mesh.face, std::move(mesh));
        }
    }
    if (!firstProblem.empty()) {
        if (problem == nullptr) throw std::runtime_error(firstProblem);
        *problem = firstProblem;
    }

    bool anyFaceted = false;
    for (const auto& face : solid.faces()) {
        if (const auto it = meshes.find(&face); it != meshes.end()) {
            const FaceMesh& mesh = it->second;
            const bool shaded = mesh.normals.size() == mesh.points.size();
            for (const auto& tri : mesh.triangles) {
                BoundaryPolygon poly;
                poly.points = {mesh.points[tri[0]], mesh.points[tri[1]], mesh.points[tri[2]]};
                if (shaded) {
                    poly.normals = {mesh.normals[tri[0]], mesh.normals[tri[1]],
                                    mesh.normals[tri[2]]};
                }
                poly.topoId = face.topoId;
                poly.exactSurface = face.exactSurface;
                polygons.push_back(std::move(poly));
            }
            continue;
        }
        auto verts = topo::faceVertices(&face);
        if (verts.size() < 3) continue;

        BoundaryPolygon poly;
        poly.points.reserve(verts.size());
        for (const auto* v : verts) poly.points.push_back(v->point);
        if (!face.innerLoops.empty()) {
            std::vector<std::vector<Vec3>> holes;
            poly.outer = poly.points;
            for (const topo::Wire* inner : face.innerLoops) {
                if (!inner) continue;
                holes.push_back(loopPoints(*inner, true));
                poly.holes.push_back(loopPoints(*inner, false));
            }
            poly.points = keyholePolygon(poly.points, std::move(holes));
        }
        poly.topoId = face.topoId;
        poly.surface = face.surface;
        poly.analyticSurface = face.analyticSurface;
        polygons.push_back(std::move(poly));
        anyFaceted = true;
    }

    // Manifold twin-linking makes all loops consistently wound up to one
    // global inward/outward sign (see MassProperties) — normalize to outward.
    // A mesh of an exact face is wound outward already and is never flipped;
    // a solid mixing the two keeps its loops as they are (they are wound
    // outward as exact solids are: material to the left of each half-edge).
    if (meshes.empty() && anyFaceted && signedVolume(polygons) < 0.0) {
        for (auto& poly : polygons) {
            std::reverse(poly.points.begin(), poly.points.end());
            std::reverse(poly.outer.begin(), poly.outer.end());
            for (auto& hole : poly.holes) std::reverse(hole.begin(), hole.end());
        }
    }

    return polygons;
}

}  // namespace hz::model
