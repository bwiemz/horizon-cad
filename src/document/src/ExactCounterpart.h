#pragma once

// The part's exact counterpart (stage 2; the part built exactly, stage 3) as builds make it and
// reuse it. Private to the document module: FeatureTree's build follows its features with an
// ExactFollower, and the Document holds what it makes for its accessors and for the next build.

#include <memory>
#include <string>

#include "horizon/document/FeatureTree.h"
#include "horizon/geometry/MeshData.h"
#include "horizon/topology/Solid.h"

namespace hz::doc {

/// A part's exact counterpart as one build made it, never changed after: shared by the document
/// that shows it and by the later builds that reuse it.
struct ExactCounterpart {
    /// The part built exactly: each body's exact form (FeatureOutput::exact: an import's body as
    /// read, ImportedBodyFeature::exactBody; a primitive's exact primitive), gathered as the part
    /// gathers their facets; null when it could not be had, as `note` says.
    std::shared_ptr<const topo::Solid> solid;
    /// Its mesh, through the read point at its display tolerance
    /// (model::SolidTessellator::displayTolerance), as the display draws it
    /// (model::SolidTessellator::tessellate): each face's triangles, wound
    /// outward, their corners shared and shaded by the surface's normals; one faceTags entry a
    /// face, by its name; its edges but its seams, along their curves. Set with `solid`, null
    /// with it.
    std::shared_ptr<const geo::MeshData> mesh;
    /// Why there is no `solid`: "shown from facets: <why>" ("the exact form faces inward (it
    /// encloses <volume>)" among them), "exact form not checked: <why>" or "exact form disagrees
    /// with its facets: <exact> vs <faceted>". Empty with a solid.
    std::string note;
    /// What it was made from: the exact part's definition (its fingerprint: its counts, and each
    /// face's surface and each edge's curve, bit for bit) and its faces' and edges' names; for one
    /// an import not exact left without a solid, its note. A build whose part says the same reuses
    /// it. Empty for one that failed for something that may not happen again (an exception):
    /// never reused.
    std::string madeFrom;
};

/// The counterpart as a build follows its features: the part's exact solid, while every body of
/// it has an exact form (each feature made a new body, or the first body whatever its operation,
/// which no Boolean combines, and built it exactly: FeatureOutput::exact).
class ExactFollower {
public:
    /// @p keep: whether the build keeps the part's exact solid (BuildOptions::exactCounterpart).
    /// A follower that does not keeps no body, and says only whether the part would have one
    /// (following()).
    explicit ExactFollower(bool keep = true) : m_keep(keep) {}

    /// Whether the build keeps an exact body a feature makes a new body of now
    /// (BuildContext::keepExact): it keeps the part's exact solid, and the part is still exact.
    bool accepting() const { return m_keep && m_open && m_note.empty(); }

    /// Follow @p feature, just applied to the part; @p first when there was no part before it;
    /// @p exactTool the exact body it built (FeatureOutput::exact), null where it built none. Each
    /// is joined to the part's exact solid (Pattern::append) as the part collects its facets. Any
    /// other operation (a Boolean, a transform), or a feature with no exact body, ends the
    /// counterpart: none, by design. An import whose body has no exact form leaves a note why
    /// (its import's, or that it has none), until some other feature ends it.
    void follow(const Feature& feature, bool first,
                std::unique_ptr<topo::Solid> exactTool) noexcept;

    /// Whether the part so far has an exact solid, and so a counterpart, or a note why not; for a
    /// follower that keeps none, whether it would have.
    bool following() const { return m_part != nullptr || !m_note.empty() || m_would; }

    /// The counterpart of the part followed, @p faceted its faceted copy: @p previous when it was
    /// made from the same exact part (madeFrom), so that nothing is meshed again; otherwise made
    /// now, from the part's exact solid, which it takes: called once, when the features are
    /// built. Made now, it is a step of the build's own on @p control (a meshing
    /// that takes seconds is not shown as done), and it stops between faces when the build is
    /// cancelled: null then, with @p cancelled set. Null too when the part has none by design.
    /// A counterpart that cannot be had is one with its note, kept for reuse as one that can;
    /// one whose making threw, whatever it threw, says so in its note and is not kept for reuse.
    /// Never throws.
    std::shared_ptr<const ExactCounterpart> make(
        const topo::Solid& faceted, const std::shared_ptr<const ExactCounterpart>& previous,
        BuildControl* control, bool& cancelled) noexcept;

private:
    std::unique_ptr<topo::Solid> m_part;  ///< the exact bodies followed, joined
    std::string m_note;       ///< why the part has none: an import not exact, or no memory for it
    bool m_noteKept = false;  ///< the note is an import's, and its counterpart kept for reuse
    bool m_open = true;       ///< false once the part is no longer bodies built exactly
    bool m_keep = true;       ///< whether the part's exact solid is kept (else only m_would)
    bool m_would = false;     ///< kept nothing, but the part so far would have one
};

}  // namespace hz::doc
