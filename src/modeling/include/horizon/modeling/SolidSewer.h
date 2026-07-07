#pragma once

#include <memory>
#include <vector>

#include "horizon/math/Vec3.h"
#include "horizon/topology/TopologyID.h"

namespace hz::topo {
class Solid;
}  // namespace hz::topo

namespace hz::geo {
class NurbsSurface;
}  // namespace hz::geo

namespace hz::model {

/// Rebuilds a half-edge B-Rep Solid from a soup of boundary polygons.
///
/// Pipeline: weld coincident vertices → drop degenerate faces → eliminate
/// T-junctions (insert vertices that lie on another face's edge, so shared
/// boundaries have matching vertex chains) → construct half-edges with twin
/// pairing → group faces into shells by connectivity → synthesize planar
/// bounding-rectangle surface patches (the codebase convention for planar
/// faces, see Extrude) and linear edge curves.
///
/// A watertight, T-junction-free input yields a solid that passes
/// Solid::checkManifold().  checkEulerFormula() additionally holds for
/// genus-0 shells; the topology module's Euler check has no genus term, so
/// through-hole (genus ≥ 1) solids report as Euler-invalid even when the
/// mesh is perfectly manifold.
///
/// Assumes 2-manifold contact: twin pairing greedily matches each directed
/// half-edge with the first available oppositely-directed one on the same
/// undirected edge.  Where exactly two faces meet an edge (the normal case)
/// this is unambiguous; at a non-manifold edge shared by four+ half-edges
/// (two lobes touching along an edge) the pairing is arbitrary and may not
/// reflect the true radial order.  Callers who can produce such contacts
/// (not the current Boolean pipeline) should split them first.
class SolidSewer {
public:
    struct InputFace {
        std::vector<math::Vec3> points;              ///< Planar loop, outward wound.
        topo::TopologyID topoId;                     ///< Assigned to the created face.
        std::shared_ptr<geo::NurbsSurface> surface;  ///< Used if set; else synthesized.
    };

    /// Sew the faces into a Solid.  Returns nullptr when no non-degenerate
    /// face survives.  @p weldTol is the vertex coincidence distance.
    static std::unique_ptr<topo::Solid> sew(const std::vector<InputFace>& faces,
                                            double weldTol = 1e-7);
};

}  // namespace hz::model
