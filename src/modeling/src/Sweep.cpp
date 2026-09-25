#include "horizon/modeling/Sweep.h"

#include <algorithm>
#include <cmath>

#include "RingStack.h"
#include "horizon/modeling/ProfileValidator.h"
#include "horizon/topology/GeometryValidator.h"
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

// Rotate v by the smallest rotation taking unit vector a onto unit vector b
// (about a x b).  Requires a . b > -1.
Vec3 minimalRotation(const Vec3& a, const Vec3& b, const Vec3& v) {
    const Vec3 k = a.cross(b);
    const double c = a.dot(b);
    return v + k.cross(v) + k.cross(k.cross(v)) * (1.0 / (1.0 + c));
}

// Directions closer to parallel than this are treated as parallel.
constexpr double kParallelTol = 1e-9;

}  // namespace

std::unique_ptr<topo::Solid> Sweep::execute(
    const std::vector<std::shared_ptr<draft::DraftEntity>>& profile,
    const draft::SketchPlane& plane, const std::vector<math::Vec3>& pathPoints,
    const std::string& featureID, int profileSegments, double chordTolerance, std::string* reason) {
    const auto fail = [reason](std::string why) -> std::unique_ptr<topo::Solid> {
        if (reason) *reason = std::move(why);
        return nullptr;
    };
    // -----------------------------------------------------------------------
    // 1. Validate the path (>= 2 distinct points, no zero-length segments).
    // -----------------------------------------------------------------------
    if (pathPoints.size() < 2) return fail("the path has no length");
    std::vector<Vec3> path;
    path.push_back(pathPoints[0]);
    for (size_t i = 1; i < pathPoints.size(); ++i) {
        if ((pathPoints[i] - path.back()).length() < 1e-9) continue;  // collapse duplicates
        path.push_back(pathPoints[i]);
    }
    if (path.size() < 2) return fail("the path has no length");

    // -----------------------------------------------------------------------
    // 2. Validate and extract the profile as a 3D ring.
    // -----------------------------------------------------------------------
    auto validation = ProfileValidator::validate(profile);
    if (!validation.isClosed) return fail("the profile: " + validation.errorMessage);

    if (profileSegments < 3) return fail("arcs need at least 3 segments per turn");
    const std::vector<Vec2> verts2D =
        ringstack::sampleProfile(validation.orderedEdges, 1e-6,
                                 ringstack::ProfileResolution{profileSegments, chordTolerance})
            .vertices;
    const size_t N = verts2D.size();
    if (N < 3) return fail("the profile has fewer than three distinct points");

    std::vector<Vec3> profileRing;
    profileRing.reserve(N);
    for (const auto& v : verts2D) profileRing.push_back(plane.localToWorld(v));

    // Wind so the profile normal points along the initial sweep direction, so
    // the leading cap faces outward.
    const Vec3 sweepDir = (path[1] - path[0]).normalized();
    Vec3 profileNormal = newellNormal(profileRing);
    if (profileNormal.length() < 1e-12) return fail("the profile encloses no area");
    profileNormal = profileNormal.normalized();
    if (profileNormal.dot(sweepDir) < 0.0) {
        std::reverse(profileRing.begin(), profileRing.end());
        profileNormal = profileNormal * -1.0;
    }

    // -----------------------------------------------------------------------
    // 3. Mitered transport.
    //
    // Each path segment k sweeps a prism along its direction d_k, cut at both
    // ends by a plane: at an interior path point the miter plane (normal
    // d_{k-1} + d_k, bisecting the turn), at the far end the profile's own
    // plane carried along by the same turns.  Ring k+1 is ring k projected
    // along d_k onto the next cutting plane, so each lateral band is exactly
    // planar (its four corners lie on two lines parallel to d_k) and the
    // cross-section perpendicular to each segment is the profile rotated by
    // the minimal rotation between consecutive directions — the discrete
    // rotation-minimizing frame.  With the profile's centroid on the path the
    // volume is exactly area × path length.
    // -----------------------------------------------------------------------
    const size_t S = path.size() - 1;
    std::vector<Vec3> dirs;
    dirs.reserve(S);
    for (size_t k = 0; k < S; ++k) dirs.push_back((path[k + 1] - path[k]).normalized());

    // A profile containing the sweep direction sweeps no volume.
    if (std::abs(profileNormal.dot(sweepDir)) < kParallelTol) {
        return fail("the path starts along the profile's plane, so it sweeps no volume");
    }

    double scale = 0.0;
    for (const auto& p : profileRing) scale = std::max(scale, (p - path.front()).length());
    for (const auto& p : path) scale = std::max(scale, (p - path.front()).length());
    const double lengthTol = 1e-9 * std::max(scale, 1.0);

    std::vector<std::vector<Vec3>> rings;
    rings.reserve(S + 1);
    rings.push_back(profileRing);
    Vec3 endNormal = profileNormal;
    for (size_t k = 0; k < S; ++k) {
        Vec3 cutNormal;
        if (k + 1 < S) {
            const Vec3 bisector = dirs[k] + dirs[k + 1];
            // A path that doubles back on itself has no miter plane.
            if (bisector.length() < kParallelTol) return fail("the path doubles back on itself");
            cutNormal = bisector.normalized();
            endNormal = minimalRotation(dirs[k], dirs[k + 1], endNormal);
        } else {
            cutNormal = endNormal;
        }

        const double denom = dirs[k].dot(cutNormal);
        if (denom < kParallelTol) return fail("the path turns too sharply for the profile");

        std::vector<Vec3> next;
        next.reserve(N);
        for (const auto& p : rings[k]) {
            const double t = (path[k + 1] - p).dot(cutNormal) / denom;
            // Every point must advance along the segment; one that does not
            // means the profile reaches past the inside of the turn and the
            // band folds through itself.
            if (t <= lengthTol) {
                return fail(
                    "the profile reaches past the inside of a turn in the path: it would fold "
                    "through itself");
            }
            next.push_back(p + dirs[k] * t);
        }
        rings.push_back(std::move(next));
    }

    // -----------------------------------------------------------------------
    // 4. Build ring-stack topology.
    // -----------------------------------------------------------------------
    auto solid = std::make_unique<topo::Solid>();
    ringstack::RingStackBuild build = ringstack::build(*solid, rings);
    if (build.bottomFace == nullptr || build.topFace == nullptr) {
        return fail("the swept shape could not be built");
    }

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

    build.bottomFace->surface = ringstack::makeCapSurface(rings.front(), profileNormal * -1.0);
    build.topFace->surface = ringstack::makeCapSurface(rings.back(), endNormal);

    for (size_t L = 0; L < build.lateralFaces.size(); ++L) {
        const auto& lower = rings[L];
        const auto& upper = rings[L + 1];
        for (size_t i = 0; i < N; ++i) {
            size_t j = (i + 1) % N;
            build.lateralFaces[L][i]->surface =
                ringstack::makeBilinearPatch(lower[i], lower[j], upper[i], upper[j]);
        }
    }

    // A path that crosses itself, or turns tightly enough elsewhere, can still
    // sew into a shell whose loops are not what the rings describe.  Refuse it
    // rather than return it.
    if (!GeometryValidator::check(*solid).ok()) {
        return fail("the path crosses itself or turns too tightly: the shape runs into itself");
    }

    return solid;
}

}  // namespace hz::model
