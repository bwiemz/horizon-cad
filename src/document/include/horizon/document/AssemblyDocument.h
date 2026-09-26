#pragma once

#include <atomic>
#include <cstdint>
#include <functional>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "horizon/geometry/MeshData.h"
#include "horizon/math/Mat4.h"
#include "horizon/math/Units.h"
#include "horizon/modeling/MateGeometry.h"
#include "horizon/topology/TopologyID.h"

namespace hz::doc {

class Document;
class AssemblyDocument;

/// How much of a referenced part is held in memory.
enum class ComponentState {
    Lightweight,  ///< Cached tessellation + transform only.
    Resolved,     ///< Full part document (feature tree) in memory.
};

/// One placed occurrence of a part inside an assembly.
///
/// The part itself lives in its own `.hzpart` file; the instance stores the
/// reference (path relative to the assembly file, or absolute), a placement
/// transform, and per-instance display state. Depending on `state`, either
/// only a cached tessellated mesh is loaded (Lightweight) or the complete
/// part document is available for editing (Resolved).
struct ComponentInstance {
    uint64_t id = 0;
    std::string name;
    std::string partPath;
    math::Mat4 transform = math::Mat4::identity();
    bool suppressed = false;

    ComponentState state = ComponentState::Lightweight;
    /// Display mesh: shared by every instance of the part (DocumentManager).
    std::shared_ptr<const geo::MeshData> cachedMesh;
    std::shared_ptr<Document> resolvedPart;  ///< Non-null when Resolved.
    /// Non-null for a component that is an assembly (Phase 159): that
    /// assembly, read for its components alone, each resolved as this one
    /// is. Rigid here: placed by this component's transform as it was saved.
    std::shared_ptr<AssemblyDocument> resolvedAssembly;
    /// A Resolved subassembly's components gathered into one solid
    /// (AssemblyDocument::drawingSolid, "c<id>/" before each name): what its
    /// parent's mates, interference check and exports take it as.
    std::shared_ptr<const topo::Solid> assemblySolid;

    /// Whether its file is an assembly (Phase 159): by its extension.
    bool isAssembly() const;
    /// The solid it places: its part's, or a subassembly's gathered one;
    /// null when neither is resolved.
    const topo::Solid* solid() const;
};

/// Geometric mate constraint types between component faces (Phase 42).
/// Defined in the modeling layer next to the solver that consumes them.
using MateType = model::MateType;

/// What one side of a mate names on its component (Phase 160).
enum class ReferenceKind {
    Face,   ///< a face, by its (logical) name
    Edge,   ///< an edge, by its whole name (model::wholeEdgeName)
    Datum,  ///< a datum plane, axis or point of the part, by its feature id
};

/// One side of a mate: a face (or, Phase 160, an edge or a datum) on a
/// placed component, referenced by name so the mate survives part rebuilds
/// (genealogy resolution).
struct MateReference {
    uint64_t componentId = 0;
    topo::TopologyID faceId;  ///< the name of what it refers to, of its kind
    ReferenceKind kind = ReferenceKind::Face;
};

/// A mate constraint between two component faces.
struct Mate {
    uint64_t id = 0;
    MateType type = MateType::Coincident;
    MateReference a;
    MateReference b;     ///< Unused for Fixed.
    double value = 0.0;  ///< Distance (length) or Angle (radians).
    /// Limits (Phase 160), for a Distance or an Angle mate: what it measures
    /// kept between them, free within them (its value then not held).
    std::optional<double> minimum;
    std::optional<double> maximum;
};

/// Two placed components that share material.
struct ComponentInterference {
    uint64_t componentA = 0;
    uint64_t componentB = 0;
    double volume = 0.0;          ///< Shared material, in model units cubed.
    bool volumeResolved = false;  ///< False if the overlap could not be measured.
};

/// Result of an assembly interference check.
struct InterferenceReport {
    std::vector<ComponentInterference> pairs;
    /// Unsuppressed components that could not take part because their part
    /// is not resolved (or has no solid): the check says nothing about them.
    std::vector<uint64_t> unchecked;
};

/// One step of an exploded view (Phase 161): components moved together along
/// a direction.
struct ExplodeStep {
    std::vector<uint64_t> components;
    math::Vec3 direction = math::Vec3::UnitZ;  ///< unit
    double distance = 0.0;
};

/// A named exploded view (Phase 161): its steps, in order. Shown, each
/// component is drawn moved by every step it is in; it is never placed there:
/// its mates, its file and its drawings keep where it is.
struct ExplodedView {
    uint64_t id = 0;
    std::string name;
    std::vector<ExplodeStep> steps;
};

/// What an edit to an assembly can change: the components placed, the mates
/// between them, and its exploded views. Undo puts one of these back.
struct AssemblyState {
    std::vector<ComponentInstance> components;
    std::vector<Mate> mates;
    std::vector<ExplodedView> views;
};

/// Assembly document: component instances plus the mates that position them.
///
/// This class provides the structural container plus file-path and dirty
/// bookkeeping matching the conventions of `Document`.
class AssemblyDocument {
public:
    AssemblyDocument() = default;

    // --- Components ---

    /// Add a component instance. Assigns a unique id if none is set.
    /// Returns the instance id.
    uint64_t addComponent(ComponentInstance instance);

    /// Remove a component by id, and the mates that refer to it. Returns
    /// true if found.
    bool removeComponent(uint64_t id);

    /// Find a component by id (nullptr if absent).
    ComponentInstance* component(uint64_t id);
    const ComponentInstance* component(uint64_t id) const;

    const std::vector<ComponentInstance>& components() const { return m_components; }
    std::vector<ComponentInstance>& components() { return m_components; }

    // --- Mates ---

    /// Add a mate. Assigns a unique id if none is set. Returns the mate id.
    uint64_t addMate(Mate mate);

    /// Remove a mate by id. Returns true if found.
    bool removeMate(uint64_t id);

    /// Find a mate by id (nullptr if absent).
    Mate* mate(uint64_t id);
    const Mate* mate(uint64_t id) const;

    const std::vector<Mate>& mates() const { return m_mates; }
    std::vector<Mate>& mates() { return m_mates; }

    // --- Exploded views (Phase 161) ---
    /// Add @p view (an id given if it has none); returns its id.
    uint64_t addExplodedView(ExplodedView view);
    /// Remove view @p id; the view shown is then none, if it was. Returns
    /// true if it was there.
    bool removeExplodedView(uint64_t id);
    ExplodedView* explodedView(uint64_t id);
    const ExplodedView* explodedView(uint64_t id) const;
    const std::vector<ExplodedView>& explodedViews() const { return m_views; }
    /// The view shown, 0 for none: how the assembly is drawn. Not an edit to
    /// undo, but kept with it.
    uint64_t shownView() const { return m_shownView; }
    /// Show view @p id (0: none, the components where they are). Returns
    /// false for an id that is no view.
    bool setShownView(uint64_t id);
    /// Where @p comp is drawn: its placement, moved by each step of the
    /// shown view it is in.
    math::Mat4 displayTransform(const ComponentInstance& comp) const;

    /// Remove all components and mates and reset bookkeeping.
    void clear();

    /// The components, mates and exploded views as they are now.
    AssemblyState snapshot() const { return {m_components, m_mates, m_views}; }

    /// Put back a snapshot: its components, placements and mates. A
    /// component still present keeps the geometry loaded for it now (mesh,
    /// resolved part), which may be newer than the snapshot's. Leaves the
    /// dirty flag alone: an undo stack that restores snapshots tracks
    /// modification itself.
    void restore(AssemblyState state);

    // --- Analysis ---

    /// Which placed components share material, and how much.  Each resolved,
    /// unsuppressed component's solid is placed by its transform and the set
    /// is run through model::InterferenceChecker.  Components that only touch
    /// (mated faces, tangent cylinders) do not interfere.  Components whose
    /// part is not resolved are listed in @c unchecked rather than silently
    /// passed — resolve them first for a complete answer.
    InterferenceReport findInterference() const;

    /// What an interference check measures: each unsuppressed, resolved
    /// component's solid, placed. Copies — so the measuring can run on a
    /// worker thread while the assembly is edited.
    struct InterferenceInput {
        std::vector<std::unique_ptr<topo::Solid>> placed;
        std::vector<uint64_t> ids;        ///< the component of each placed solid
        std::vector<uint64_t> unchecked;  ///< components with no solid to check
        /// Faces in all the placed solids: how long the measuring will take.
        std::size_t faceCount() const;
    };
    InterferenceInput interferenceInput() const;

    /// The unsuppressed components as one solid, for a drawing (Phase 150):
    /// each part placed by its component's transform, its names prefixed
    /// with the component's ("c7/"), so two instances of one part are told
    /// apart, and the bodies gathered, not merged. @p partOf gives a
    /// component's part solid; one it gives none for is left out and listed
    /// in @p missing. Null when no component has a solid.
    std::unique_ptr<topo::Solid> drawingSolid(
        const std::function<const topo::Solid*(const ComponentInstance&)>& partOf,
        std::vector<uint64_t>* missing = nullptr) const;
    /// The prefix drawingSolid() gives the names of component @p id.
    static std::string namePrefix(uint64_t id);
    /// The unsuppressed components as one mesh (Phase 159): each one's mesh
    /// placed by its transform, its faces and edges named as drawingSolid()
    /// names them. What a parent assembly shows of this one, as one of its
    /// components. Null when no component has a mesh.
    std::shared_ptr<geo::MeshData> drawingMesh() const;
    /// Measure @p input. Once @p cancelled is set, measuring stops between
    /// pairs, and the report holds only the pairs measured by then.
    static InterferenceReport measureInterference(const InterferenceInput& input,
                                                  const std::atomic<bool>* cancelled = nullptr);

    // --- Dirty tracking ---

    bool isDirty() const { return m_dirty; }
    void setDirty(bool dirty) { m_dirty = dirty; }

    // --- File path ---

    const std::string& filePath() const { return m_filePath; }
    void setFilePath(const std::string& path) { m_filePath = path; }

    // --- Unit (Phase 154) ---

    /// The unit its lengths are shown and typed in, saved with it.
    math::LengthUnit lengthUnit() const { return m_lengthUnit; }
    void setLengthUnit(math::LengthUnit unit) { m_lengthUnit = unit; }

private:
    std::vector<ComponentInstance> m_components;
    std::vector<Mate> m_mates;
    uint64_t m_nextComponentId = 1;
    uint64_t m_nextMateId = 1;
    std::vector<ExplodedView> m_views;
    uint64_t m_nextViewId = 1;
    uint64_t m_shownView = 0;
    bool m_dirty = false;
    std::string m_filePath;
    math::LengthUnit m_lengthUnit = math::LengthUnit::Millimetre;
};

}  // namespace hz::doc
