#include "horizon/modeling/InterferenceChecker.h"

#include <algorithm>
#include <cmath>
#include <set>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "MeshCsg.h"
#include "horizon/math/RTree.h"
#include "horizon/modeling/BooleanOp.h"
#include "horizon/modeling/BoundaryMesh.h"
#include "horizon/modeling/MassProperties.h"

namespace hz::model {

using hz::math::BoundingBox;
using hz::math::Vec3;

namespace {

/// Volume below which an intersection is treated as touching, not
/// interfering, per cube of the parts' extent (Phase 142: it was 1e-9
/// absolute, which is most of a hundredth-of-a-millimetre part).
constexpr double kMinInterferenceFraction = 1e-12;

BoundingBox overlapBox(const BoundingBox& a, const BoundingBox& b) {
    const Vec3 mn(std::max(a.min().x, b.min().x), std::max(a.min().y, b.min().y),
                  std::max(a.min().z, b.min().z));
    const Vec3 mx(std::min(a.max().x, b.max().x), std::min(a.max().y, b.max().y),
                  std::min(a.max().z, b.max().z));
    return BoundingBox(mn, mx);
}

}  // namespace

BoundingBox InterferenceChecker::solidBounds(const topo::Solid& solid) {
    BoundingBox box;
    for (const auto& vtx : solid.vertices()) box.expand(vtx.point);
    return box;
}

bool InterferenceChecker::solidsInterfere(const topo::Solid& a, const topo::Solid& b,
                                          std::string* problem) {
    if (problem != nullptr) problem->clear();
    const BoundingBox ba = solidBounds(a);
    const BoundingBox bb = solidBounds(b);
    if (!ba.intersects(bb)) return false;

    // Interference = the Boolean intersection encloses real volume.  This
    // reuses the CSG face splitting and classification; the previous
    // edge-crosses-face heuristic missed exactly-grazing configurations
    // (crossing points landing on triangulation diagonals of symmetric
    // geometry), and surface touching must not count as interference.
    // Exact faces are read as their meshes; one that cannot be meshed leaves
    // the pair to be looked at, with why, as one too fine to work out is.
    std::vector<BoundaryPolygon> polysA;
    std::vector<BoundaryPolygon> polysB;
    try {
        polysA = BoundaryMesh::extractFacePolygons(a);
        polysB = BoundaryMesh::extractFacePolygons(b);
    } catch (const std::runtime_error& e) {
        if (problem != nullptr) *problem = e.what();
        return true;
    }
    if (polysA.empty() || polysB.empty()) return false;

    // The tolerances of the parts' size, as BooleanOp takes them.
    BoundingBox both = ba;
    both.expand(bb);
    const CsgTolerance tol = CsgTolerance::of(both.min(), both.max());
    // Only the volume counts, not how the faces are cut up: convex faces go
    // in whole (csgPieces), and parts of many facets are checked in time.
    std::vector<CsgPolygon> fragments;
    try {
        fragments =
            csgExecute(csgPieces(polysA, true, tol.plane), csgPieces(polysB, false, tol.plane),
                       BooleanType::Intersect, tol.plane, CsgFragments::Fewest);
    } catch (const CsgCancelled&) {
        return false;  // the check stops before the next pair
    } catch (const CsgTooComplex&) {
        // Too finely faceted, through and through, to work out in time:
        // their boxes overlap, and they are reported, to be looked at, not
        // passed as clear.
        return true;
    }
    if (fragments.empty()) return false;
    const double extent = (both.max() - both.min()).length();
    return std::abs(csgVolume(fragments)) > kMinInterferenceFraction * extent * extent * extent;
}

std::vector<InterferencePair> InterferenceChecker::check(
    const std::vector<const topo::Solid*>& solids, const std::atomic<bool>* cancelled) {
    std::vector<InterferencePair> pairs;
    // A pair of many facets stops as soon as the check is cancelled.
    const BooleanCancellation stop(cancelled);

    // Broad phase: index every valid solid's AABB in an R*-tree.
    std::vector<BoundingBox> bounds(solids.size());
    math::RTree<size_t> tree;
    for (size_t i = 0; i < solids.size(); ++i) {
        if (!solids[i]) continue;
        bounds[i] = solidBounds(*solids[i]);
        if (bounds[i].isValid()) tree.insert(i, bounds[i]);
    }

    std::set<std::pair<size_t, size_t>> tested;
    for (size_t i = 0; i < solids.size(); ++i) {
        if (!solids[i] || !bounds[i].isValid()) continue;
        for (size_t j : tree.query(bounds[i])) {
            if (cancelled != nullptr && cancelled->load(std::memory_order_relaxed)) return pairs;
            if (j == i || !solids[j]) continue;
            const auto key = std::minmax(i, j);
            if (!tested.insert({key.first, key.second}).second) continue;  // dedup
            // Narrow phase.
            std::string problem;
            if (solidsInterfere(*solids[key.first], *solids[key.second], &problem)) {
                InterferencePair pair;
                pair.indexA = key.first;
                pair.indexB = key.second;
                pair.overlapBounds = overlapBox(bounds[key.first], bounds[key.second]);
                // Not measured where it could not be worked out: the Boolean would fail alike.
                auto shared = problem.empty()
                                  ? BooleanOp::execute(*solids[key.first], *solids[key.second],
                                                       BooleanType::Intersect)
                                  : nullptr;
                if (shared) {
                    pair.volume = MassPropertiesCalculator::compute(*shared).volume;
                    pair.volumeResolved = true;
                }
                pair.problem = std::move(problem);
                pairs.push_back(std::move(pair));
            }
        }
    }
    return pairs;
}

}  // namespace hz::model
