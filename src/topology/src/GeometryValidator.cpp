#include "horizon/topology/GeometryValidator.h"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <map>
#include <sstream>
#include <vector>

#include "horizon/geometry/curves/NurbsCurve.h"
#include "horizon/geometry/surfaces/NurbsSurface.h"
#include "horizon/topology/Queries.h"
#include "horizon/topology/Solid.h"

namespace hz::topo {

using hz::math::Vec3;

namespace {

/// Ordered vertex positions of a wire, walking the next chain once.
std::vector<Vec3> loopPoints(const Wire* wire) {
    std::vector<Vec3> pts;
    if (wire == nullptr || wire->halfEdge == nullptr) {
        return pts;
    }
    const HalfEdge* start = wire->halfEdge;
    const HalfEdge* cur = start;
    // Bounded walk: a broken chain is the structural validator's business, not
    // ours, but we must not spin forever on one.
    for (size_t guard = 0; guard < 100000; ++guard) {
        if (cur->origin == nullptr) {
            break;
        }
        pts.push_back(cur->origin->point);
        cur = cur->next;
        if (cur == nullptr || cur == start) {
            break;
        }
    }
    return pts;
}

Vec3 newell(const std::vector<Vec3>& pts) {
    Vec3 a(0, 0, 0);
    for (size_t i = 0; i < pts.size(); ++i) {
        const Vec3& p = pts[i];
        const Vec3& q = pts[(i + 1) % pts.size()];
        a = a + p.cross(q);
    }
    return a * 0.5;
}

/// Largest pairwise coordinate span of a point set — used to keep the
/// planarity tolerance meaningful for models far from the origin.
double extentOf(const std::vector<Vec3>& pts) {
    if (pts.empty()) {
        return 0.0;
    }
    Vec3 lo = pts[0];
    Vec3 hi = pts[0];
    for (const auto& p : pts) {
        lo.x = std::min(lo.x, p.x);
        lo.y = std::min(lo.y, p.y);
        lo.z = std::min(lo.z, p.z);
        hi.x = std::max(hi.x, p.x);
        hi.y = std::max(hi.y, p.y);
        hi.z = std::max(hi.z, p.z);
    }
    return (hi - lo).length();
}

/// 2D orientation sign with a tolerance band.
int orient2d(double ax, double ay, double bx, double by, double cx, double cy, double eps) {
    const double d = (bx - ax) * (cy - ay) - (by - ay) * (cx - ax);
    if (d > eps) return 1;
    if (d < -eps) return -1;
    return 0;
}

bool onSegment2d(double ax, double ay, double bx, double by, double px, double py, double eps) {
    return px >= std::min(ax, bx) - eps && px <= std::max(ax, bx) + eps &&
           py >= std::min(ay, by) - eps && py <= std::max(ay, by) + eps;
}

/// Proper or improper crossing of two 2D segments.
bool segmentsCross(const std::pair<double, double>& a, const std::pair<double, double>& b,
                   const std::pair<double, double>& c, const std::pair<double, double>& d,
                   double eps) {
    const int o1 = orient2d(a.first, a.second, b.first, b.second, c.first, c.second, eps);
    const int o2 = orient2d(a.first, a.second, b.first, b.second, d.first, d.second, eps);
    const int o3 = orient2d(c.first, c.second, d.first, d.second, a.first, a.second, eps);
    const int o4 = orient2d(c.first, c.second, d.first, d.second, b.first, b.second, eps);

    if (o1 != o2 && o3 != o4 && o1 != 0 && o2 != 0 && o3 != 0 && o4 != 0) {
        return true;
    }
    // Collinear overlap.
    if (o1 == 0 && onSegment2d(a.first, a.second, b.first, b.second, c.first, c.second, eps))
        return true;
    if (o2 == 0 && onSegment2d(a.first, a.second, b.first, b.second, d.first, d.second, eps))
        return true;
    if (o3 == 0 && onSegment2d(c.first, c.second, d.first, d.second, a.first, a.second, eps))
        return true;
    if (o4 == 0 && onSegment2d(c.first, c.second, d.first, d.second, b.first, b.second, eps))
        return true;
    return false;
}

/// A planar loop crosses itself when two non-adjacent segments intersect.
/// Adjacent pairs (and the wrap-around pair) share an endpoint by
/// construction and are skipped.
bool loopSelfIntersects(const std::vector<Vec3>& pts, const Vec3& normal, double tol) {
    const size_t n = pts.size();
    if (n < 4) {
        return false;
    }

    // Build an in-plane basis.
    Vec3 ref = std::abs(normal.x) < 0.9 ? Vec3(1, 0, 0) : Vec3(0, 1, 0);
    Vec3 u = normal.cross(ref);
    const double ulen = u.length();
    if (ulen < 1e-12) {
        return false;
    }
    u = u * (1.0 / ulen);
    const Vec3 v = normal.cross(u);

    std::vector<std::pair<double, double>> p2(n);
    for (size_t i = 0; i < n; ++i) {
        p2[i] = {pts[i].dot(u), pts[i].dot(v)};
    }

    for (size_t i = 0; i < n; ++i) {
        const size_t i2 = (i + 1) % n;
        for (size_t j = i + 1; j < n; ++j) {
            const size_t j2 = (j + 1) % n;
            if (i == j || i2 == j || j2 == i) {
                continue;  // Shares an endpoint by construction.
            }
            if (segmentsCross(p2[i], p2[i2], p2[j], p2[j2], tol)) {
                return true;
            }
        }
    }
    return false;
}

/// Quantized key for the coincident-vertex bucket scan.
struct CellKey {
    int64_t x, y, z;
    bool operator<(const CellKey& o) const {
        if (x != o.x) return x < o.x;
        if (y != o.y) return y < o.y;
        return z < o.z;
    }
};

}  // namespace

Vec3 GeometryValidator::loopAreaVector(const Face& face) {
    return newell(loopPoints(face.outerLoop));
}

bool GeometryValidator::facePlane(const Face& face, Vec3& originOut, Vec3& normalOut, double tol) {
    if (face.surface == nullptr) {
        // No carrier: the loop defines its own plane.
        const auto pts = loopPoints(face.outerLoop);
        if (pts.size() < 3) {
            return false;
        }
        const Vec3 a = newell(pts);
        const double len = a.length();
        if (len < tol * tol) {
            return false;
        }
        originOut = pts[0];
        normalOut = a * (1.0 / len);
        return true;
    }

    // A carrier is planar when its normal field is constant.  Sampling beats
    // inspecting the control net: it is correct for any parameterization and
    // treats degree-elevated or refined planes the same as `makePlane`.
    const geo::NurbsSurface& s = *face.surface;
    const double u0 = s.uMin();
    const double u1 = s.uMax();
    const double v0 = s.vMin();
    const double v1 = s.vMax();

    // A bilinear patch — the carrier of box, extrusion and pattern faces — is
    // planar exactly when its four control points are, which settles it
    // without evaluating the surface: on a 10,000-body pattern the sampling
    // below took most of a validation that every feature now runs.
    const auto& cp = s.controlPoints();
    if (s.degreeU() == 1 && s.degreeV() == 1 && cp.size() == 2 && cp[0].size() == 2 &&
        cp[1].size() == 2) {
        const Vec3 du = cp[1][0] - cp[0][0];
        const Vec3 dv = cp[0][1] - cp[0][0];
        const Vec3 n = du.cross(dv);  // the normal at (u0, v0), as sampled below
        const double len = n.length();
        if (len > 1e-12 * du.length() * dv.length()) {
            const Vec3 unit = n * (1.0 / len);
            const Vec3 diagonal = cp[1][1] - cp[0][0];
            if (std::abs(unit.dot(diagonal)) > 1e-7 * diagonal.length()) {
                return false;  // Twisted: a curved carrier.
            }
            originOut = s.evaluate((u0 + u1) * 0.5, (v0 + v1) * 0.5);
            normalOut = unit;
            return true;
        }
        // A corner with a collapsed side: fall back to sampling.
    }

    Vec3 ref(0, 0, 0);
    bool haveRef = false;
    for (int i = 0; i <= 2; ++i) {
        for (int j = 0; j <= 2; ++j) {
            const double u = u0 + (u1 - u0) * (i * 0.5);
            const double v = v0 + (v1 - v0) * (j * 0.5);
            Vec3 n = s.normal(u, v);
            const double len = n.length();
            if (len < 1e-12) {
                continue;  // Degenerate sample (pole); ignore.
            }
            n = n * (1.0 / len);
            if (!haveRef) {
                ref = n;
                haveRef = true;
                continue;
            }
            if (n.cross(ref).length() > 1e-7) {
                return false;  // Curved carrier.
            }
        }
    }
    if (!haveRef) {
        return false;
    }
    originOut = s.evaluate((u0 + u1) * 0.5, (v0 + v1) * 0.5);
    normalOut = ref;
    return true;
}

GeometryValidator::Issues GeometryValidator::check(const Solid& solid, double tol, Scope scope) {
    const bool everything = scope == Scope::Everything;
    Issues issues;
    const double areaTol = tol * tol;

    // -- Per-half-edge: vertex chain and twin coincidence --------------------
    for (const auto& face : solid.faces()) {
        std::vector<const Wire*> wires;
        if (face.outerLoop != nullptr) {
            wires.push_back(face.outerLoop);
        }
        for (const auto* inner : face.innerLoops) {
            wires.push_back(inner);
        }
        for (const auto* wire : wires) {
            if (wire == nullptr || wire->halfEdge == nullptr) {
                continue;
            }
            const HalfEdge* start = wire->halfEdge;
            const HalfEdge* cur = start;
            for (size_t guard = 0; guard < 100000; ++guard) {
                if (cur->next == nullptr || cur->twin == nullptr) {
                    break;  // Structural defect; Solid::checkManifold reports it.
                }
                if (cur->next->origin != cur->twin->origin) {
                    ++issues.vertexChainErrors;
                }
                cur = cur->next;
                if (cur == start) {
                    break;
                }
            }
        }
    }

    // -- Per-edge: twin endpoints span one segment; non-degenerate -----------
    for (const auto& edge : solid.edges()) {
        const HalfEdge* he = edge.halfEdge;
        if (he == nullptr || he->twin == nullptr || he->origin == nullptr ||
            he->twin->origin == nullptr) {
            continue;
        }
        const Vec3 a = he->origin->point;
        const Vec3 b = he->twin->origin->point;

        if (a.distanceTo(b) < tol) {
            ++issues.degenerateEdges;
        }

        // The half-edge ends where its next half-edge starts; that position
        // must be the twin's origin, and vice versa.
        if (he->next != nullptr && he->next->origin != nullptr &&
            he->next->origin->point.distanceTo(b) > tol) {
            ++issues.twinCoincidenceErrors;
        } else if (he->twin->next != nullptr && he->twin->next->origin != nullptr &&
                   he->twin->next->origin->point.distanceTo(a) > tol) {
            ++issues.twinCoincidenceErrors;
        }

        if (everything && edge.curve != nullptr) {
            const Vec3 c0 = edge.curve->evaluate(edge.curve->tMin());
            const Vec3 c1 = edge.curve->evaluate(edge.curve->tMax());
            const bool forward = c0.distanceTo(a) <= tol && c1.distanceTo(b) <= tol;
            const bool reverse = c0.distanceTo(b) <= tol && c1.distanceTo(a) <= tol;
            if (!forward && !reverse) {
                ++issues.edgeCurveMismatches;
            }
        }
    }

    // -- Per-face: degeneracy, planarity, self-intersection -------------------
    for (const auto& face : solid.faces()) {
        const auto pts = loopPoints(face.outerLoop);
        if (pts.size() < 3) {
            ++issues.degenerateFaces;
            continue;
        }
        const Vec3 areaVec = newell(pts);
        if (areaVec.length() < areaTol) {
            ++issues.degenerateFaces;
            continue;
        }

        Vec3 origin(0, 0, 0);
        Vec3 normal(0, 0, 0);
        if (!facePlane(face, origin, normal, tol)) {
            continue;  // Curved carrier: planarity does not apply.
        }

        const double planeTol = tol * std::max(1.0, extentOf(pts));
        bool planar = true;
        for (const auto& p : pts) {
            if (std::abs(normal.dot(p - origin)) > planeTol) {
                planar = false;
                break;
            }
        }
        if (!planar) {
            ++issues.nonPlanarLoops;
            continue;  // A non-planar loop's 2D projection is meaningless.
        }

        if (loopSelfIntersects(pts, normal, planeTol)) {
            ++issues.selfIntersectingLoops;
        }
    }

    // -- Per-shell: the area vectors of a closed skin sum to zero ------------
    {
        std::map<const Shell*, Vec3> sums;
        std::map<const Shell*, double> scales;
        for (const auto& face : solid.faces()) {
            Vec3 a = newell(loopPoints(face.outerLoop));
            for (const auto* inner : face.innerLoops) {
                a = a + newell(loopPoints(inner));
            }
            sums[face.shell] = sums[face.shell] + a;
            scales[face.shell] += a.length();
        }
        for (const auto& [shell, sum] : sums) {
            const double scale = std::max(scales[shell], 1.0);
            if (sum.length() > 1e-9 * scale) {
                ++issues.openShells;
            }
        }
    }

    // -- Coincident vertices (reported only) ---------------------------------
    if (everything) {
        const double cell = std::max(tol, 1e-12);
        std::map<CellKey, std::vector<const Vertex*>> buckets;
        for (const auto& v : solid.vertices()) {
            const CellKey k{static_cast<int64_t>(std::floor(v.point.x / cell)),
                            static_cast<int64_t>(std::floor(v.point.y / cell)),
                            static_cast<int64_t>(std::floor(v.point.z / cell))};
            // Probe the 27-cell neighbourhood so a pair straddling a cell wall
            // is still found.
            bool matched = false;
            for (int dx = -1; dx <= 1 && !matched; ++dx) {
                for (int dy = -1; dy <= 1 && !matched; ++dy) {
                    for (int dz = -1; dz <= 1 && !matched; ++dz) {
                        auto it = buckets.find({k.x + dx, k.y + dy, k.z + dz});
                        if (it == buckets.end()) {
                            continue;
                        }
                        for (const auto* other : it->second) {
                            if (other->point.distanceTo(v.point) <= tol) {
                                ++issues.coincidentVertices;
                                matched = true;
                                break;
                            }
                        }
                    }
                }
            }
            buckets[k].push_back(&v);
        }
    }

    return issues;
}

bool GeometryValidator::isGeometricallyValid(const Solid& solid, double tol) {
    return check(solid, tol, Scope::FailingOnly).ok();
}

std::string GeometryValidator::report(const Solid& solid, double tol) {
    const Issues i = check(solid, tol);
    std::ostringstream out;

    auto line = [&out](const char* label, int count, const char* detail) {
        if (count > 0) {
            out << label << ": " << count << " (" << detail << ")\n";
        }
    };

    line("Vertex chain errors", i.vertexChainErrors,
         "he->next->origin != he->twin->origin — loop and edge disagree on a vertex");
    line("Twin coincidence errors", i.twinCoincidenceErrors,
         "an edge's two half-edges span different positions");
    line("Degenerate edges", i.degenerateEdges, "zero-length");
    line("Degenerate faces", i.degenerateFaces, "fewer than 3 vertices, or vanishing area");
    line("Non-planar loops", i.nonPlanarLoops, "a vertex lies off its own planar face");
    line("Self-intersecting loops", i.selfIntersectingLoops, "boundary segments cross");
    line("Open shells", i.openShells, "face area vectors do not sum to zero");
    line("Edge curve mismatches", i.edgeCurveMismatches,
         "curve endpoints differ from the edge vertices — reported, not failing");
    line("Coincident vertices", i.coincidentVertices,
         "distinct vertices at the same position — reported, not failing");

    if (i.ok() && i.edgeCurveMismatches == 0 && i.coincidentVertices == 0) {
        out << "Geometry checks OK\n";
    } else if (i.ok()) {
        out << "Geometry checks OK (with advisories above)\n";
    }
    return out.str();
}

}  // namespace hz::topo
