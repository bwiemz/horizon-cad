#pragma once

#include <memory>
#include <string>
#include <vector>

#include "horizon/math/Vec2.h"
#include "horizon/modeling/SheetMetal.h"
#include "horizon/topology/Solid.h"

namespace hz::model {

/// Folded 3D flange bodies from the Phase-62 sheet-metal strip model
/// (Phase 61b). A strip of flat segments joined by bends becomes a solid
/// plate: the folded side profile is swept (extruded) along the bend axis
/// by the part width, with bends approximated by arc polylines.
class SheetMetalSolid {
public:
    /// Closed cross-section polygon of the folded strip, counter-clockwise:
    /// bottom boundary walked forward, then the top boundary (offset by the
    /// material thickness) walked back. Bend angles are signed: positive
    /// bends fold toward the material's top side, negative away; |angle|
    /// must stay below pi. Each bend becomes @p segmentsPerBend chords.
    /// Segment lengths are measured between bend tangent lines, matching
    /// the Phase-62 flat-pattern convention. Empty on invalid input.
    static std::vector<math::Vec2> crossSection(const SheetMetalStrip& strip,
                                                const SheetMetalParams& params,
                                                int segmentsPerBend = 8);

    /// Base/edge flange body: the folded cross-section extruded to
    /// @p width along the bend axis. nullptr on invalid input.
    static std::unique_ptr<topo::Solid> fold(const SheetMetalStrip& strip,
                                             const SheetMetalParams& params, double width,
                                             const std::string& featureID, int segmentsPerBend = 8);

    /// Flat-pattern outline of the same body: a developedLength() x width
    /// rectangle (counter-clockwise, origin corner first). Empty on
    /// invalid input.
    static std::vector<math::Vec2> flatPattern(const SheetMetalStrip& strip,
                                               const SheetMetalParams& params, double width);
};

}  // namespace hz::model
