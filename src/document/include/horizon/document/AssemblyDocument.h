#pragma once

#include <atomic>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "horizon/geometry/MeshData.h"
#include "horizon/math/Mat4.h"
#include "horizon/modeling/MateGeometry.h"
#include "horizon/topology/TopologyID.h"

namespace hz::doc {

class Document;

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
};

/// Geometric mate constraint types between component faces (Phase 42).
/// Defined in the modeling layer next to the solver that consumes them.
using MateType = model::MateType;

/// One side of a mate: a face on a placed component, referenced by
/// TopologyID so the mate survives part rebuilds (genealogy resolution).
struct MateReference {
    uint64_t componentId = 0;
    topo::TopologyID faceId;
};

/// A mate constraint between two component faces.
struct Mate {
    uint64_t id = 0;
    MateType type = MateType::Coincident;
    MateReference a;
    MateReference b;     ///< Unused for Fixed.
    double value = 0.0;  ///< Distance (length) or Angle (radians).
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

/// What an edit to an assembly can change: the components placed and the mates
/// between them. Undo puts one of these back.
struct AssemblyState {
    std::vector<ComponentInstance> components;
    std::vector<Mate> mates;
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

    /// Remove all components and mates and reset bookkeeping.
    void clear();

    /// The components and mates as they are now.
    AssemblyState snapshot() const { return {m_components, m_mates}; }

    /// Put back a snapshot. Leaves the dirty flag alone: an undo stack that
    /// restores snapshots tracks modification itself.
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

private:
    std::vector<ComponentInstance> m_components;
    std::vector<Mate> m_mates;
    uint64_t m_nextComponentId = 1;
    uint64_t m_nextMateId = 1;
    bool m_dirty = false;
    std::string m_filePath;
};

}  // namespace hz::doc
