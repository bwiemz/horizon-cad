#pragma once

// The part's exact counterpart (stage 2) as builds make it and reuse it. Private to the document
// module: FeatureTree's build follows its features with an ExactFollower, and the Document holds
// what it makes for its accessors and for the next build.

#include <memory>
#include <string>
#include <vector>

#include "horizon/document/FeatureTree.h"
#include "horizon/geometry/MeshData.h"
#include "horizon/topology/Solid.h"

namespace hz::doc {

/// A part's exact counterpart as one build made it, never changed after: shared by the document
/// that shows it and by the later builds that reuse it.
struct ExactCounterpart {
    /// The imported bodies as read, exact, gathered as the part gathers their facets
    /// (ImportedBodyFeature::exactBody); null when they could not be had, as `note` says.
    std::shared_ptr<const topo::Solid> solid;
    /// Its mesh, through the read point at model::SolidTessellator::kDisplayTolerance, as the
    /// display draws it (model::SolidTessellator::tessellate): each face's triangles, wound
    /// outward, their corners shared and shaded by the surface's normals; one faceTags entry a
    /// face, by its name; its edges but its seams, along their curves. Set with `solid`, null
    /// with it.
    std::shared_ptr<const geo::MeshData> mesh;
    /// Why there is no `solid`: "shown from facets: <why>" ("the exact form faces inward (it
    /// encloses <volume>)" among them), "exact form not checked: <why>" or "exact form disagrees
    /// with its facets: <exact> vs <faceted>". Empty with a solid.
    std::string note;
    /// What it was made from: each import that made the part, in order, by its feature, its
    /// naming and its body's definition. A build whose imports say the same reuses it. Empty for
    /// one that failed for something that may not happen again (an exception): never reused.
    std::string madeFrom;
};

/// The counterpart as a build follows its features: the imports that have made the part, while
/// only imports have (each an ImportedBodyFeature made a new body, or the first body whatever its
/// operation, which no Boolean combines).
class ExactFollower {
public:
    /// Follow @p feature, just applied to the part; @p first when there was no part before it.
    /// Any other feature, or an import a Boolean combines, ends the counterpart: none, by design.
    void follow(const Feature& feature, bool first) noexcept;

    /// Whether the part so far is imports alone, and so has a counterpart, or a note why not.
    bool following() const { return !m_imports.empty() || !m_note.empty(); }

    /// The counterpart of the part the followed imports made, @p faceted its faceted copy: @p
    /// previous when it was made from the same imports, so that nothing is meshed again;
    /// otherwise made now. Made now, it is a step of the build's own on @p control (a meshing
    /// that takes seconds is not shown as done), and it stops between faces when the build is
    /// cancelled: null then, with @p cancelled set. Null too when the part has none by design.
    /// A counterpart that cannot be had is one with its note, kept for reuse as one that can.
    /// Never throws.
    std::shared_ptr<const ExactCounterpart> make(
        const topo::Solid& faceted, const std::shared_ptr<const ExactCounterpart>& previous,
        BuildControl* control, bool& cancelled) const noexcept;

private:
    std::vector<const ImportedBodyFeature*> m_imports;
    std::string m_note;  ///< why the imports could not be followed (no memory for them)
    bool m_open = true;
};

}  // namespace hz::doc
