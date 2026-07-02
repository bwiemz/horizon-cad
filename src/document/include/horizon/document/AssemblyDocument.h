#pragma once

#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "horizon/math/Mat4.h"
#include "horizon/render/SceneGraph.h"

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
    std::shared_ptr<render::MeshData> cachedMesh;  ///< Lightweight display mesh.
    std::shared_ptr<Document> resolvedPart;        ///< Non-null when Resolved.
};

/// Assembly document: an ordered collection of component instances.
///
/// Mates (assembly constraints) are added in Phase 42; this class provides
/// the structural container plus file-path and dirty bookkeeping matching
/// the conventions of `Document`.
class AssemblyDocument {
public:
    AssemblyDocument() = default;

    // --- Components ---

    /// Add a component instance. Assigns a unique id if none is set.
    /// Returns the instance id.
    uint64_t addComponent(ComponentInstance instance);

    /// Remove a component by id. Returns true if found.
    bool removeComponent(uint64_t id);

    /// Find a component by id (nullptr if absent).
    ComponentInstance* component(uint64_t id);
    const ComponentInstance* component(uint64_t id) const;

    const std::vector<ComponentInstance>& components() const { return m_components; }
    std::vector<ComponentInstance>& components() { return m_components; }

    /// Remove all components and reset bookkeeping.
    void clear();

    // --- Dirty tracking ---

    bool isDirty() const { return m_dirty; }
    void setDirty(bool dirty) { m_dirty = dirty; }

    // --- File path ---

    const std::string& filePath() const { return m_filePath; }
    void setFilePath(const std::string& path) { m_filePath = path; }

private:
    std::vector<ComponentInstance> m_components;
    uint64_t m_nextComponentId = 1;
    bool m_dirty = false;
    std::string m_filePath;
};

}  // namespace hz::doc
