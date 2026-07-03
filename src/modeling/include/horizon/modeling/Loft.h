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
/// count (automatic vertex matching is deferred). Sections are ruled together
/// with bilinear NURBS patches; winding is aligned and the start index of
/// each section is rotated to minimize twist relative to the previous one.
class Loft {
public:
    /// Loft through the given sections in order.
    ///
    /// @param sections   Ordered cross-sections (>= 2), each a closed loop.
    /// @param featureID  Base name for TopologyID generation (e.g. "loft_1").
    /// @return The lofted solid, or nullptr if the input is invalid.
    static std::unique_ptr<topo::Solid> execute(const std::vector<LoftSection>& sections,
                                                const std::string& featureID);
};

}  // namespace hz::model
