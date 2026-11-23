#pragma once

#include <cstdint>
#include <map>
#include <optional>
#include <string>
#include <vector>

#include "horizon/document/AssemblyDocument.h"
#include "horizon/math/Mat4.h"
#include "horizon/modeling/AssemblySolver.h"

namespace hz::doc {

class DatumFeature;
class DocumentManager;

/// What placing an assembly by its mates came to (placeByMates).
struct MatePlacement {
    /// The mates hold: each component is where they put it.
    bool placed = false;
    /// The geometry every mate names was found, and they were solved (held
    /// or not); false when a mate names geometry that is not there.
    bool gathered = false;
    /// Why not, when not: geometry a mate names that cannot be found, or the
    /// solver's word ("did not converge").
    std::string why;
    /// The solve's own result, when there was one (none without mates).
    model::AssemblySolveResult result;
};

/// Place @p assembly by its mates: each component's part read whole from
/// @p documents first (a mate's faces are found there), the mates gathered
/// and solved, and, when they hold, each component moved where they put it
/// and the patterns' instances after their seeds. The Assembly menu's
/// commands and the AI assistant's tools place an assembly this one way. An
/// assembly with no mates is placed as it is.
MatePlacement placeByMates(AssemblyDocument& assembly, DocumentManager& documents);

/// The frame a mate takes of @p datum (Phase 160): a plane's origin and
/// normal, an axis's origin and direction, a point's position, in its part.
/// For a component that is @p mirrored, mirrored as its faces and edges are
/// (ComponentInstance::ownMirror()).
model::MateFrame datumFrame(const DatumFeature& datum, bool mirrored);

/// An assembly's mates gathered for its solver (Phase 158): each component
/// where it is, and each mate's frames found once in its components' parts.
/// Gathered once, it is solved as often as a drag moves, and the faces are
/// not looked for again.
class AssemblyMates {
public:
    /// @p assembly's components and mates, the frames found in each
    /// component's resolved part: the caller resolves them first. Nullopt,
    /// and why in @p why, when a mate names geometry that cannot be found
    /// ("mate 3 references geometry that could not be resolved").
    static std::optional<AssemblyMates> gather(const AssemblyDocument& assembly,
                                               std::string* why = nullptr);

    /// A component held where a drag puts it: grounded there.
    struct Hold {
        uint64_t component = 0;
        math::Mat4 at = math::Mat4::identity();
    };
    struct Options {
        std::optional<Hold> hold;
        /// The solver's rank analysis (redundant mates, freedom left): the
        /// dense part of the solve, which a drag, solving at every move,
        /// leaves out.
        bool diagnostics = true;
    };

    /// The mates solved, from the components where they are now (see
    /// place()). A held component starts, and stays, where it is held; what
    /// grounds the assembly without a hold (a Fixed mate, else its first
    /// component) grounds it still.
    model::AssemblySolveResult solve(const Options& options) const;
    model::AssemblySolveResult solve() const { return solve(Options{}); }

    /// Start the next solve from @p transforms (by component id): where the
    /// last one left them, so a drag moves on from there.
    void place(const std::map<uint64_t, math::Mat4>& transforms);

    const std::vector<model::SolverComponent>& components() const { return m_components; }
    const std::vector<model::SolverMate>& mates() const { return m_mates; }

private:
    std::vector<model::SolverComponent> m_components;
    std::vector<model::SolverMate> m_mates;
};

}  // namespace hz::doc
