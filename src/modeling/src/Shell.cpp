#include "horizon/modeling/Shell.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <string>

#include "RingStack.h"
#include "horizon/topology/Queries.h"

namespace hz::model {

using hz::math::Vec3;
using namespace hz::topo;

namespace {

constexpr const char* kOnlyPrisms =
    "shell supports only a plain prism for now (two caps and straight sides); this part has "
    "other features, holes or tapered sides, which would be lost";

Vec3 polyCentroid(const std::vector<Vec3>& poly) {
    Vec3 c = Vec3::Zero;
    for (const auto& p : poly) c += p;
    return c * (1.0 / static_cast<double>(poly.size()));
}

Vec3 newellNormal(const std::vector<Vec3>& poly) {
    Vec3 n = Vec3::Zero;
    const size_t M = poly.size();
    for (size_t i = 0; i < M; ++i) {
        const Vec3& a = poly[i];
        const Vec3& b = poly[(i + 1) % M];
        n.x += (a.y - b.y) * (a.z + b.z);
        n.y += (a.z - b.z) * (a.x + b.x);
        n.z += (a.x - b.x) * (a.y + b.y);
    }
    return n;
}

std::vector<Vec3> facePolygon(const Face& face) {
    std::vector<Vec3> poly;
    for (const auto* v : faceVertices(&face)) poly.push_back(v->point);
    return poly;
}

// Mitered inward offset of a planar polygon by @p t. The polygon must run
// counter-clockwise around @p axis, its outward normal: then the inside of
// each edge is on its left, axis x edge. (Picking "inward" as the side
// facing the centroid turned edges of non-convex profiles outward.)
std::vector<Vec3> offsetPolygonInward(const std::vector<Vec3>& poly, const Vec3& axis, double t) {
    const size_t N = poly.size();

    std::vector<Vec3> edgeInward(N);
    for (size_t i = 0; i < N; ++i) {
        const Vec3 e = poly[(i + 1) % N] - poly[i];
        const Vec3 nin = axis.cross(e);
        edgeInward[i] = nin.length() < 1e-12 ? Vec3::Zero : nin.normalized();
    }

    std::vector<Vec3> inner(N);
    for (size_t i = 0; i < N; ++i) {
        const Vec3& nPrev = edgeInward[(i + N - 1) % N];
        const Vec3& nCur = edgeInward[i];
        double denom = 1.0 + nPrev.dot(nCur);
        if (std::abs(denom) < 1e-9) {
            inner[i] = poly[i];
            continue;
        }
        inner[i] = poly[i] + (nPrev + nCur) * (t / denom);
    }
    return inner;
}

// Why @p inner, the inward offset of @p outer (both counter-clockwise around
// @p axis), cannot be a cavity profile, or "" if it can: every edge must keep
// its direction (an edge the offset turns back on itself has collapsed), and
// the polygon must be simple and lie inside @p outer.
std::string innerProfileProblem(const std::vector<Vec3>& outer, const std::vector<Vec3>& inner,
                                const Vec3& axis) {
    const size_t N = outer.size();
    Vec3 u = axis.cross(std::abs(axis.x) < 0.9 ? Vec3(1, 0, 0) : Vec3(0, 1, 0)).normalized();
    Vec3 v = axis.cross(u);
    struct P2 {
        double x, y;
    };
    const auto flat = [&](const std::vector<Vec3>& poly) {
        std::vector<P2> out;
        out.reserve(poly.size());
        for (const Vec3& q : poly) out.push_back({q.dot(u), q.dot(v)});
        return out;
    };
    const std::vector<P2> a = flat(outer);
    const std::vector<P2> b = flat(inner);

    double scale = 0.0;
    for (size_t i = 0; i < N; ++i) {
        scale = std::max(scale, std::hypot(a[(i + 1) % N].x - a[i].x, a[(i + 1) % N].y - a[i].y));
    }
    const double eps = 1e-9 * std::max(scale, 1.0);

    for (size_t i = 0; i < N; ++i) {
        const size_t j = (i + 1) % N;
        const double dx = b[j].x - b[i].x, dy = b[j].y - b[i].y;
        const double ox = a[j].x - a[i].x, oy = a[j].y - a[i].y;
        if (dx * ox + dy * oy <= eps * scale) {
            return "the wall is too thick for this profile: an edge of the cavity collapses";
        }
    }
    const auto cross = [](const P2& o, const P2& p, const P2& q) {
        return (p.x - o.x) * (q.y - o.y) - (p.y - o.y) * (q.x - o.x);
    };
    for (size_t i = 0; i < N; ++i) {
        for (size_t k = i + 1; k < N; ++k) {
            if (k == i + 1 || (i == 0 && k == N - 1)) continue;  // neighbours share a corner
            const P2 &p1 = b[i], &p2 = b[(i + 1) % N], &q1 = b[k], &q2 = b[(k + 1) % N];
            const double d1 = cross(q1, q2, p1), d2 = cross(q1, q2, p2);
            const double d3 = cross(p1, p2, q1), d4 = cross(p1, p2, q2);
            if (((d1 > eps && d2 < -eps) || (d1 < -eps && d2 > eps)) &&
                ((d3 > eps && d4 < -eps) || (d3 < -eps && d4 > eps))) {
                return "the wall is too thick for this profile: the cavity crosses itself";
            }
        }
    }
    // Every cavity corner inside the outer profile (crossing-number test).
    for (const P2& q : b) {
        bool inside = false;
        for (size_t i = 0, j = N - 1; i < N; j = i++) {
            if ((a[i].y > q.y) != (a[j].y > q.y) &&
                q.x < (a[j].x - a[i].x) * (q.y - a[i].y) / (a[j].y - a[i].y) + a[i].x) {
                inside = !inside;
            }
        }
        if (!inside) return "the wall is too thick for this profile: the cavity leaves the part";
    }
    return {};
}

// Reorder @p src so each entry aligns (horizontally, perpendicular to axis)
// with the corresponding entry in @p ref. Used to match the base cap corners
// to the top cap corners of a prism.
std::vector<Vec3> alignByHorizontal(const std::vector<Vec3>& src, const std::vector<Vec3>& ref,
                                    const Vec3& axis) {
    std::vector<Vec3> out;
    out.reserve(ref.size());
    std::vector<bool> used(src.size(), false);
    for (const auto& r : ref) {
        Vec3 rPerp = r - axis * r.dot(axis);
        double best = std::numeric_limits<double>::max();
        size_t bestIdx = 0;
        for (size_t i = 0; i < src.size(); ++i) {
            if (used[i]) continue;
            Vec3 sPerp = src[i] - axis * src[i].dot(axis);
            double d = (sPerp - rPerp).lengthSquared();
            if (d < best) {
                best = d;
                bestIdx = i;
            }
        }
        used[bestIdx] = true;
        out.push_back(src[bestIdx]);
    }
    return out;
}

void bindCupGeometry(topo::Solid& solid, const ringstack::RingStackBuild& build,
                     const std::vector<std::vector<Vec3>>& rings, const Vec3& axis) {
    ringstack::assignEdgeCurves(solid);
    if (build.bottomFace)
        build.bottomFace->surface = ringstack::makeCapSurface(rings.front(), -axis);
    if (build.topFace) build.topFace->surface = ringstack::makeCapSurface(rings.back(), axis);
    const size_t N = rings.front().size();
    for (size_t L = 0; L < build.lateralFaces.size(); ++L) {
        const auto& lower = rings[L];
        const auto& upper = rings[L + 1];
        for (size_t i = 0; i < N; ++i) {
            size_t j = (i + 1) % N;
            if (build.lateralFaces[L][i]) {
                build.lateralFaces[L][i]->surface =
                    ringstack::makeBilinearPatch(lower[i], lower[j], upper[i], upper[j]);
            }
        }
    }
}

void assignCupTopologyIds(topo::Solid& solid, const ringstack::RingStackBuild& build) {
    if (build.bottomFace) build.bottomFace->topoId = TopologyID::make("shell", "outer_bottom");
    if (build.topFace) build.topFace->topoId = TopologyID::make("shell", "cavity_floor");
    const char* levelName[3] = {"outer_wall", "rim", "inner_wall"};
    for (size_t L = 0; L < build.lateralFaces.size(); ++L) {
        const char* nm = L < 3 ? levelName[L] : "wall";
        for (size_t i = 0; i < build.lateralFaces[L].size(); ++i) {
            if (build.lateralFaces[L][i]) {
                build.lateralFaces[L][i]->topoId =
                    TopologyID::make("shell", std::string(nm) + "_" + std::to_string(i));
            }
        }
    }
    int idx = 0;
    for (auto& e : const_cast<std::deque<Edge>&>(solid.edges())) {
        e.topoId = TopologyID::make("shell", "edge" + std::to_string(idx));
        ++idx;
    }
}

}  // namespace

ShellResult Shell::execute(std::unique_ptr<topo::Solid> solid, double thickness,
                           const std::vector<topo::TopologyID>& removedFaceIds) {
    ShellResult result;
    if (!solid) {
        result.message = "null solid";
        return result;
    }
    if (thickness <= 0.0) {
        result.message = "thickness must be positive";
        return result;
    }
    if (removedFaceIds.empty()) {
        result.message = "shell requires at least one face to remove (closed hollow deferred)";
        return result;
    }
    // Only the first face would be opened; the others would be ignored.
    if (removedFaceIds.size() > 1) {
        result.message = "opening more than one face is not supported yet";
        return result;
    }
    // The cup is rebuilt from two caps of one body; anything else in the
    // solid would be dropped without a word. A part holds several bodies
    // after a New-body feature or a spaced pattern.
    if (solid->shells().size() > 1) {
        result.message = "the part has several bodies; shelling one of them is not supported yet";
        return result;
    }

    // Solid centroid for outward normal orientation.
    Vec3 solidCentroid = Vec3::Zero;
    size_t vc = 0;
    for (const auto& v : solid->vertices()) {
        solidCentroid += v.point;
        ++vc;
    }
    if (vc == 0) {
        result.message = "empty solid";
        return result;
    }
    solidCentroid = solidCentroid * (1.0 / static_cast<double>(vc));

    // Locate the removed cap → shell axis.
    // The face itself, or — if an operation split it — its first piece.
    const Face* removedFace = nullptr;
    for (const auto& id : removedFaceIds) {
        const Face* piece = nullptr;
        for (const auto& face : solid->faces()) {
            if (face.topoId == id) {
                removedFace = &face;
                break;
            }
            if (piece == nullptr && face.topoId.isDescendantOf(id)) piece = &face;
        }
        if (!removedFace) removedFace = piece;
        if (removedFace) break;
    }
    if (!removedFace) {
        result.message = "removed face not found";
        return result;
    }

    std::vector<Vec3> topPoly = facePolygon(*removedFace);
    if (topPoly.size() < 3) {
        result.message = "removed face is not a polygon";
        return result;
    }
    Vec3 topCentroid = polyCentroid(topPoly);
    Vec3 axis = newellNormal(topPoly).normalized();
    if (axis.dot(topCentroid - solidCentroid) < 0.0) axis = -axis;
    // Normalize winding so the top polygon is CCW around +axis.
    if (newellNormal(topPoly).dot(axis) < 0.0) std::reverse(topPoly.begin(), topPoly.end());

    // Opposite cap (base).
    const Face* baseFace = nullptr;
    double bestDot = -0.5;
    for (const auto& face : solid->faces()) {
        if (&face == removedFace) continue;
        std::vector<Vec3> poly = facePolygon(face);
        if (poly.size() != topPoly.size()) continue;
        Vec3 n = newellNormal(poly).normalized();
        Vec3 c = polyCentroid(poly);
        if (n.dot(c - solidCentroid) < 0.0) n = -n;
        double d = n.dot(-axis);
        if (d > bestDot) {
            bestDot = d;
            baseFace = &face;
        }
    }
    if (!baseFace || bestDot < 0.9) {
        result.message = kOnlyPrisms;
        return result;
    }

    std::vector<Vec3> basePoly = facePolygon(*baseFace);
    Vec3 baseCentroid = polyCentroid(basePoly);
    const double height = (topCentroid - baseCentroid).dot(axis);
    if (height <= 1e-9) {
        result.message = "degenerate prism height";
        return result;
    }

    // The cup is rebuilt from the two caps alone, with vertical walls. That is
    // the part only when the part is a right prism: the two caps, one side
    // face per edge and nothing else, the base straight below the top.
    // Anything else (a hole, a boss, a split face, a taper) would be dropped
    // or wrong without a word, so it is refused.
    const size_t N = topPoly.size();
    bool prism = solid->faces().size() == N + 2 && solid->vertices().size() == 2 * N;
    for (const auto& face : solid->faces()) {
        if (!prism) break;
        if (!face.innerLoops.empty()) prism = false;
        if (&face != removedFace && &face != baseFace && faceVertices(&face).size() != 4) {
            prism = false;
        }
    }
    std::vector<Vec3> outerTop = topPoly;
    std::vector<Vec3> outerBase = alignByHorizontal(basePoly, outerTop, axis);
    if (prism) {
        double size = height;
        for (size_t i = 0; i < N; ++i)
            size = std::max(size, (topPoly[(i + 1) % N] - topPoly[i]).length());
        const double tolerance = 1e-6 * std::max(1.0, size);
        for (size_t i = 0; i < N && prism; ++i) {
            prism = (outerBase[i] - (outerTop[i] - axis * height)).length() <= tolerance;
        }
    }
    if (!prism) {
        result.message = kOnlyPrisms;
        return result;
    }
    if (thickness >= height) {
        result.message = "wall thickness exceeds part height";
        return result;
    }

    // Build the four cup rings, all index-aligned and CCW around +axis:
    //   outer base → outer top → inner top (rim) → inner base (cavity floor).
    std::vector<Vec3> innerTop = offsetPolygonInward(outerTop, axis, thickness);
    const std::string problem = innerProfileProblem(outerTop, innerTop, axis);
    if (!problem.empty()) {
        result.message = problem;
        return result;
    }
    std::vector<Vec3> innerBase(innerTop.size());
    for (size_t i = 0; i < innerTop.size(); ++i) {
        innerBase[i] = innerTop[i] - axis * (height - thickness);
    }

    std::vector<std::vector<Vec3>> rings = {outerBase, outerTop, innerTop, innerBase};

    auto cup = std::make_unique<topo::Solid>();
    ringstack::RingStackBuild build = ringstack::build(*cup, rings);
    if (!build.bottomFace || !build.topFace) {
        result.message = "failed to build shelled solid";
        return result;
    }
    bindCupGeometry(*cup, build, rings, axis);
    assignCupTopologyIds(*cup, build);

    if (!cup->checkManifold()) {
        result.message = "shelled solid is not manifold";
        return result;
    }

    result.solid = std::move(cup);
    result.ok = true;
    return result;
}

}  // namespace hz::model
