#include "horizon/modeling/Shell.h"

#include <algorithm>
#include <cmath>
#include <limits>

#include "RingStack.h"
#include "horizon/topology/Queries.h"

namespace hz::model {

using hz::math::Vec3;
using namespace hz::topo;

namespace {

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

// Minimum distance from the polygon centroid to any edge line (inradius).
double polygonInradius(const std::vector<Vec3>& poly, const Vec3& centroid) {
    double minDist = std::numeric_limits<double>::max();
    const size_t N = poly.size();
    for (size_t i = 0; i < N; ++i) {
        const Vec3& a = poly[i];
        const Vec3& b = poly[(i + 1) % N];
        Vec3 e = b - a;
        double len = e.length();
        if (len < 1e-12) continue;
        Vec3 ap = centroid - a;
        double t = std::clamp(ap.dot(e) / (len * len), 0.0, 1.0);
        Vec3 closest = a + e * t;
        minDist = std::min(minDist, (centroid - closest).length());
    }
    return minDist;
}

// Mitered inward offset of a planar polygon by @p t. @p axis is the polygon's
// outward normal. Preserves winding.
std::vector<Vec3> offsetPolygonInward(const std::vector<Vec3>& poly, const Vec3& axis, double t) {
    const size_t N = poly.size();
    Vec3 centroid = polyCentroid(poly);

    std::vector<Vec3> edgeInward(N);
    for (size_t i = 0; i < N; ++i) {
        Vec3 e = poly[(i + 1) % N] - poly[i];
        Vec3 nin = axis.cross(e);
        if (nin.length() < 1e-12) {
            edgeInward[i] = Vec3::Zero;
            continue;
        }
        nin = nin.normalized();
        Vec3 mid = (poly[i] + poly[(i + 1) % N]) * 0.5;
        if (nin.dot(centroid - mid) < 0.0) nin = -nin;
        edgeInward[i] = nin;
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
    const Face* removedFace = nullptr;
    for (const auto& face : solid->faces()) {
        for (const auto& id : removedFaceIds) {
            if (face.topoId == id) {
                removedFace = &face;
                break;
            }
        }
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
        result.message = "no matching opposing cap found (unsupported solid for shell)";
        return result;
    }

    std::vector<Vec3> basePoly = facePolygon(*baseFace);
    Vec3 baseCentroid = polyCentroid(basePoly);
    const double height = (topCentroid - baseCentroid).dot(axis);
    if (height <= 1e-9) {
        result.message = "degenerate prism height";
        return result;
    }

    // Self-intersection guard.
    const double inradius = polygonInradius(topPoly, topCentroid);
    if (thickness >= inradius) {
        result.message = "wall thickness (" + std::to_string(thickness) +
                         ") meets or exceeds profile inradius (" + std::to_string(inradius) + ")";
        return result;
    }
    if (thickness >= height) {
        result.message = "wall thickness exceeds part height";
        return result;
    }

    // Build the four cup rings, all index-aligned and CCW around +axis:
    //   outer base → outer top → inner top (rim) → inner base (cavity floor).
    std::vector<Vec3> outerTop = topPoly;
    std::vector<Vec3> outerBase = alignByHorizontal(basePoly, outerTop, axis);
    std::vector<Vec3> innerTop = offsetPolygonInward(outerTop, axis, thickness);
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
