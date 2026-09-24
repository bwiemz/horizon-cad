#pragma once

#include <memory>
#include <string>
#include <vector>

#include "horizon/drafting/DraftEntity.h"
#include "horizon/drafting/SketchPlane.h"
#include "horizon/topology/Solid.h"

namespace hz::model {

/// One cross-section of a loft: a closed 2D profile on a sketch plane.
struct LoftSection {
    std::vector<std::shared_ptr<draft::DraftEntity>> profile;
    draft::SketchPlane plane;
};

/// Creates a 3D solid by lofting between two or more closed profile sections.
///
/// Era-2 scope (matches the roadmap): all sections must have the same vertex
/// count (automatic vertex matching is deferred). Sections are ruled together;
/// winding is aligned and the start index of each section is rotated to
/// minimize twist relative to the previous one.
///
/// A band between two sections whose corners are coplanar is one flat quad
/// carrying its patch.  A level with any non-planar band (a twist, or
/// sections that are not similar) is faceted: cut along its rulings into
/// `twistSegments` strips (rounded up to even), each non-planar strip split
/// into two triangles on alternating diagonals, every facet recording the
/// bilinear patch it approximates on `topo::Face::analyticSurface`.  A
/// non-planar quad has no well-defined enclosed volume, so without this the
/// display and the loop-based paths (mass properties, Booleans) disagreed —
/// by 20% on a 0.6 rad twist.  Alternating diagonals make the faceted volume
/// equal the ruled solid's exactly; the strip count only sets how closely the
/// facets follow the curved patches.
class Loft {
public:
    /// Strips per non-planar level (rounded up to even).
    static constexpr int kDefaultTwistSegments = 8;

    /// Loft through the given sections in order.
    ///
    /// @param sections   Ordered cross-sections (>= 2), each a closed loop.
    /// @param featureID  Base name for TopologyID generation (e.g. "loft_1").
    /// @param twistSegments  Strips per non-planar level (>= 1, rounded up to even).
    /// @return The lofted solid, or nullptr if the input is invalid.
    static std::unique_ptr<topo::Solid> execute(const std::vector<LoftSection>& sections,
                                                const std::string& featureID,
                                                int twistSegments = kDefaultTwistSegments,
                                                std::string* reason = nullptr);
};

}  // namespace hz::model
