#include "horizon/modeling/Loft.h"

#include <algorithm>
#include <cmath>
#include <limits>

#include "RingStack.h"
#include "horizon/modeling/ProfileValidator.h"
#include "horizon/topology/TopologyID.h"

namespace hz::model {

using hz::math::Vec2;
using hz::math::Vec3;
using namespace hz::topo;

namespace {

Vec3 centroid(const std::vector<Vec3>& ring) {
    Vec3 c = Vec3::Zero;
    for (const auto& p : ring) c = c + p;
    return c * (1.0 / static_cast<double>(ring.size()));
}

// Newell's method: area-weighted normal of a (possibly non-planar) ring.
Vec3 newellNormal(const std::vector<Vec3>& ring) {
    Vec3 n = Vec3::Zero;
    const size_t M = ring.size();
    for (size_t i = 0; i < M; ++i) {
        const Vec3& a = ring[i];
        const Vec3& b = ring[(i + 1) % M];
        n.x += (a.y - b.y) * (a.z + b.z);
        n.y += (a.z - b.z) * (a.x + b.x);
        n.z += (a.x - b.x) * (a.y + b.y);
    }
    return n;
}

// Rotate @p ring so vertex @p offset becomes index 0.
std::vector<Vec3> rotated(const std::vector<Vec3>& ring, size_t offset) {
    std::vector<Vec3> out;
    out.reserve(ring.size());
    for (size_t i = 0; i < ring.size(); ++i) {
        out.push_back(ring[(i + offset) % ring.size()]);
    }
    return out;
}

// Best start-index offset aligning @p ring to @p reference (minimizes the sum
// of squared vertex distances over all rotations).
size_t bestAlignmentOffset(const std::vector<Vec3>& ring, const std::vector<Vec3>& reference) {
    const size_t N = ring.size();
    size_t best = 0;
    double bestCost = std::numeric_limits<double>::max();
    for (size_t offset = 0; offset < N; ++offset) {
        double cost = 0.0;
        for (size_t i = 0; i < N; ++i) {
            const Vec3 d = ring[(i + offset) % N] - reference[i];
            cost += d.dot(d);
        }
        if (cost < bestCost) {
            bestCost = cost;
            best = offset;
        }
    }
    return best;
}

}  // namespace

std::unique_ptr<topo::Solid> Loft::execute(const std::vector<LoftSection>& sections,
                                           const std::string& featureID) {
    if (sections.size() < 2) return nullptr;

    // -----------------------------------------------------------------------
    // 1. Validate and extract each section as a 3D ring.
    // -----------------------------------------------------------------------
    std::vector<std::vector<Vec3>> rings;
    rings.reserve(sections.size());
    size_t N = 0;
    for (const auto& section : sections) {
        auto validation = ProfileValidator::validate(section.profile);
        if (!validation.isClosed) return nullptr;

        std::vector<Vec2> verts2D =
            ringstack::extractProfileVertices(validation.orderedEdges, 1e-6);
        if (verts2D.size() < 3) return nullptr;
        if (N == 0) {
            N = verts2D.size();
        } else if (verts2D.size() != N) {
            // Era-2 scope: sections must share a vertex count.
            return nullptr;
        }

        std::vector<Vec3> ring;
        ring.reserve(N);
        for (const auto& v : verts2D) ring.push_back(section.plane.localToWorld(v));
        rings.push_back(std::move(ring));
    }

    // -----------------------------------------------------------------------
    // 2. Consistent winding + start-index alignment.
    // -----------------------------------------------------------------------
    const Vec3 loftAxis = (centroid(rings.back()) - centroid(rings.front())).normalized();

    for (auto& ring : rings) {
        // Wind so the ring normal points along the loft axis (outward at top).
        if (newellNormal(ring).dot(loftAxis) < 0.0) {
            std::reverse(ring.begin(), ring.end());
        }
    }
    for (size_t L = 1; L < rings.size(); ++L) {
        size_t offset = bestAlignmentOffset(rings[L], rings[L - 1]);
        if (offset != 0) rings[L] = rotated(rings[L], offset);
    }

    // -----------------------------------------------------------------------
    // 3. Build ring-stack topology.
    // -----------------------------------------------------------------------
    auto solid = std::make_unique<topo::Solid>();
    ringstack::RingStackBuild build = ringstack::build(*solid, rings);
    if (build.bottomFace == nullptr || build.topFace == nullptr) return nullptr;

    // -----------------------------------------------------------------------
    // 4. TopologyIDs.
    // -----------------------------------------------------------------------
    build.bottomFace->topoId = TopologyID::make(featureID, "cap_bottom");
    build.topFace->topoId = TopologyID::make(featureID, "cap_top");
    for (size_t L = 0; L < build.lateralFaces.size(); ++L) {
        for (size_t i = 0; i < build.lateralFaces[L].size(); ++i) {
            build.lateralFaces[L][i]->topoId = TopologyID::make(
                featureID, "lateral_" + std::to_string(L) + "_" + std::to_string(i));
        }
    }
    {
        int idx = 0;
        for (auto& e : const_cast<std::deque<Edge>&>(solid->edges())) {
            e.topoId = TopologyID::make(featureID, "edge" + std::to_string(idx));
            ++idx;
        }
    }

    // -----------------------------------------------------------------------
    // 5. Geometry: edge curves, bilinear lateral patches, planar caps.
    // -----------------------------------------------------------------------
    ringstack::assignEdgeCurves(*solid);

    build.bottomFace->surface =
        ringstack::makeCapSurface(rings.front(), sections.front().plane.normal() * -1.0);
    build.topFace->surface =
        ringstack::makeCapSurface(rings.back(), sections.back().plane.normal());

    for (size_t L = 0; L < build.lateralFaces.size(); ++L) {
        const auto& lower = rings[L];
        const auto& upper = rings[L + 1];
        for (size_t i = 0; i < N; ++i) {
            size_t j = (i + 1) % N;
            build.lateralFaces[L][i]->surface =
                ringstack::makeBilinearPatch(lower[i], lower[j], upper[i], upper[j]);
        }
    }

    return solid;
}

}  // namespace hz::model
