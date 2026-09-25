#include "horizon/modeling/InterferenceChecker.h"

#include <algorithm>
#include <cmath>
#include <set>
#include <utility>

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

bool InterferenceChecker::solidsInterfere(const topo::Solid& a, const topo::Solid& b) {
    const BoundingBox ba = solidBounds(a);
    const BoundingBox bb = solidBounds(b);
    if (!ba.intersects(bb)) return false;

    // Interference = the Boolean intersection encloses real volume.  This
    // reuses the CSG face splitting and classification; the previous
    // edge-crosses-face heuristic missed exactly-grazing configurations
    // (crossing points landing on triangulation diagonals of symmetric
    // geometry), and surface touching must not count as interference.
    const auto polysA = BoundaryMesh::extractFacePolygons(a);
    const auto polysB = BoundaryMesh::extractFacePolygons(b);
    if (polysA.empty() || polysB.empty()) return false;

    // The tolerances of the parts' size, as BooleanOp takes them.
    BoundingBox both = ba;
    both.expand(bb);
    const CsgTolerance tol = CsgTolerance::of(both.min(), both.max());
    const auto fragments = csgExecute(csgTriangles(polysA, true), csgTriangles(polysB, false),
                                      BooleanType::Intersect, tol.plane);
    if (fragments.empty()) return false;
    const double extent = (both.max() - both.min()).length();
    return std::abs(csgVolume(fragments)) > kMinInterferenceFraction * extent * extent * extent;
}

std::vector<InterferencePair> InterferenceChecker::check(
    const std::vector<const topo::Solid*>& solids, const std::atomic<bool>* cancelled) {
    std::vector<InterferencePair> pairs;

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
            if (solidsInterfere(*solids[key.first], *solids[key.second])) {
                InterferencePair pair;
                pair.indexA = key.first;
                pair.indexB = key.second;
                pair.overlapBounds = overlapBox(bounds[key.first], bounds[key.second]);
                auto shared = BooleanOp::execute(*solids[key.first], *solids[key.second],
                                                 BooleanType::Intersect);
                if (shared) {
                    pair.volume = MassPropertiesCalculator::compute(*shared).volume;
                    pair.volumeResolved = true;
                }
                pairs.push_back(pair);
            }
        }
    }
    return pairs;
}

}  // namespace hz::model
