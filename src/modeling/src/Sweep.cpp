#include "horizon/modeling/Sweep.h"

#include <algorithm>
#include <cmath>

#include "RingStack.h"
#include "horizon/modeling/ProfileValidator.h"
#include "horizon/topology/TopologyID.h"

namespace hz::model {

using hz::math::Vec2;
using hz::math::Vec3;
using namespace hz::topo;

namespace {

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

}  // namespace

std::unique_ptr<topo::Solid> Sweep::execute(
    const std::vector<std::shared_ptr<draft::DraftEntity>>& profile,
    const draft::SketchPlane& plane, const std::vector<math::Vec3>& pathPoints,
    const std::string& featureID) {
    // -----------------------------------------------------------------------
    // 1. Validate the path (>= 2 distinct points, no zero-length segments).
    // -----------------------------------------------------------------------
    if (pathPoints.size() < 2) return nullptr;
    std::vector<Vec3> path;
    path.push_back(pathPoints[0]);
    for (size_t i = 1; i < pathPoints.size(); ++i) {
        if ((pathPoints[i] - path.back()).length() < 1e-9) continue;  // collapse duplicates
        path.push_back(pathPoints[i]);
    }
    if (path.size() < 2) return nullptr;

    // -----------------------------------------------------------------------
    // 2. Validate and extract the profile as a 3D ring.
    // -----------------------------------------------------------------------
    auto validation = ProfileValidator::validate(profile);
    if (!validation.isClosed) return nullptr;

    std::vector<Vec2> verts2D = ringstack::extractProfileVertices(validation.orderedEdges, 1e-6);
    const size_t N = verts2D.size();
    if (N < 3) return nullptr;

    std::vector<Vec3> profileRing;
    profileRing.reserve(N);
    for (const auto& v : verts2D) profileRing.push_back(plane.localToWorld(v));

    // Wind so the profile normal points along the initial sweep direction, so
    // the leading cap faces outward.
    const Vec3 sweepDir = (path[1] - path[0]).normalized();
    if (newellNormal(profileRing).dot(sweepDir) < 0.0) {
        std::reverse(profileRing.begin(), profileRing.end());
    }

    // -----------------------------------------------------------------------
    // 3. Translation transport: one ring per path point.
    // -----------------------------------------------------------------------
    std::vector<std::vector<Vec3>> rings;
    rings.reserve(path.size());
    for (const auto& p : path) {
        const Vec3 delta = p - path.front();
        std::vector<Vec3> ring;
        ring.reserve(N);
        for (const auto& base : profileRing) ring.push_back(base + delta);
        rings.push_back(std::move(ring));
    }

    // -----------------------------------------------------------------------
    // 4. Build ring-stack topology.
    // -----------------------------------------------------------------------
    auto solid = std::make_unique<topo::Solid>();
    ringstack::RingStackBuild build = ringstack::build(*solid, rings);
    if (build.bottomFace == nullptr || build.topFace == nullptr) return nullptr;

    // -----------------------------------------------------------------------
    // 5. TopologyIDs.
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
    // 6. Geometry.
    // -----------------------------------------------------------------------
    ringstack::assignEdgeCurves(*solid);

    build.bottomFace->surface = ringstack::makeCapSurface(rings.front(), sweepDir * -1.0);
    const Vec3 endDir = (path.back() - path[path.size() - 2]).normalized();
    build.topFace->surface = ringstack::makeCapSurface(rings.back(), endDir);

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
