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
/// Solid::checkManifold(), and checkEulerFormula() for any genus.
///
/// Where exactly two faces meet an edge (the normal case) twin pairing is
/// unambiguous. Where more do — two bodies touching along an edge, as a Union
/// of boxes that meet only there makes — each face is paired with the one met
/// first turning about the edge into the material behind it, its own body's;
/// and a vertex where shells meet is given to each shell as a copy of its
/// own. Bodies touching along an edge or at a point come out as separate
/// shells, each a solid, sharing nothing.
class SolidSewer {
public:
    /// Distance below which two input points are the same vertex.
    ///
    /// Callers that build loops by clipping or intersecting must treat this
    /// as the definition of vertex identity: emitting two points closer than
    /// this leaves a loop that claims more vertices than it sews to, and the
    /// collapsed segment reads downstream as a degenerate or self-intersecting
    /// boundary.  Deduplicate at this tolerance, not a tighter one.
    static constexpr double kDefaultWeldTol = 1e-7;

    struct InputFace {
        std::vector<math::Vec3> points;  ///< Planar loop, outward wound.
        /// Inner loops, wound against @p points: the face's holes, each made
        /// an inner wire (topo::Face::innerLoops).
        std::vector<std::vector<math::Vec3>> holes;
        topo::TopologyID topoId;                     ///< Assigned to the created face.
        std::shared_ptr<geo::NurbsSurface> surface;  ///< Used if set; else synthesized.
        /// Copied to the created face's topo::Face::analyticSurface.
        std::shared_ptr<geo::NurbsSurface> analyticSurface;
    };

    /// Sew the faces into a Solid.  Returns nullptr when no non-degenerate
    /// face survives.  @p weldTol is the vertex coincidence distance.
    static std::unique_ptr<topo::Solid> sew(const std::vector<InputFace>& faces,
                                            double weldTol = kDefaultWeldTol);
};

}  // namespace hz::model
