#pragma once

#include "horizon/topology/Solid.h"

#include <memory>

namespace hz::model {

enum class BooleanType { Union, Subtract, Intersect };

/// Mesh-free Boolean operations on B-Rep solids.
///
/// Uses face-level classification: each face's centroid is tested against the
/// opposing solid via ray-casting (ExactPredicates::classifyPoint).  Faces are
/// selected or discarded according to the Boolean type, then reassembled into
/// a new Solid using Euler operators.
///
/// Limitations (Phase 36):
/// - Faces that partially intersect the other solid are kept whole (no face
///   splitting yet — that requires SSI curve trimming from a future phase).
/// - The result may not be perfectly watertight at the intersection boundary.
class BooleanOp {
public:
    /// Execute a Boolean operation on two solids.
    /// @return A new solid containing the selected faces, or nullptr on failure.
    static std::unique_ptr<topo::Solid> execute(const topo::Solid& solidA,
                                                 const topo::Solid& solidB,
                                                 BooleanType type);
};

}  // namespace hz::model
