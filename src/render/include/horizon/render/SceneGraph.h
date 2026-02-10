#pragma once

#include "horizon/math/Mat4.h"
#include "horizon/math/Vec3.h"

#include <cstdint>
#include <memory>
#include <string>
#include <vector>

namespace hz::render {

/// Per-node material description (simple Phong parameters).
struct Material {
    math::Vec3 color{0.7, 0.7, 0.7};
    float ambient = 0.15f;
    float specular = 0.5f;
    float shininess = 32.0f;
};

/// Mesh data stored on a scene node (CPU-side, indices + positions + normals).
struct MeshData {
    std::vector<float> positions;  // 3 floats per vertex
    std::vector<float> normals;    // 3 floats per vertex
    std::vector<uint32_t> indices; // triangle list
};

/// A single node in the scene graph.
class SceneNode {
public:
    explicit SceneNode(std::string name = "Node");
    ~SceneNode();

    // Name
    const std::string& name() const { return m_name; }
    void setName(const std::string& name) { m_name = name; }

    // Transform (local)
    const math::Mat4& localTransform() const { return m_localTransform; }
    void setLocalTransform(const math::Mat4& t) { m_localTransform = t; }

    // World transform (computed by traversal)
    math::Mat4 worldTransform() const;

    // Visibility
    bool isVisible() const { return m_visible; }
    void setVisible(bool v) { m_visible = v; }

    // Unique ID for selection
    uint32_t id() const { return m_id; }

    // Mesh data (optional)
    bool hasMesh() const { return m_mesh != nullptr; }
    const MeshData& mesh() const { return *m_mesh; }
    void setMesh(std::unique_ptr<MeshData> mesh) { m_mesh = std::move(mesh); }

    // Material
    const Material& material() const { return m_material; }
    void setMaterial(const Material& mat) { m_material = mat; }

    // Children
    void addChild(std::shared_ptr<SceneNode> child);
    void removeChild(const std::shared_ptr<SceneNode>& child);
    const std::vector<std::shared_ptr<SceneNode>>& children() const { return m_children; }

    // Parent (weak reference)
    SceneNode* parent() const { return m_parent; }

private:
    std::string m_name;
    math::Mat4 m_localTransform;
    bool m_visible = true;
    uint32_t m_id;

    std::unique_ptr<MeshData> m_mesh;
    Material m_material;

    SceneNode* m_parent = nullptr;
    std::vector<std::shared_ptr<SceneNode>> m_children;

    static uint32_t s_nextId;
};

/// Root container for the scene graph.
class SceneGraph {
public:
    SceneGraph();
    ~SceneGraph();

    /// Add a top-level node.
    void addNode(std::shared_ptr<SceneNode> node);

    /// Remove a top-level node.
    void removeNode(const std::shared_ptr<SceneNode>& node);

    /// Clear all nodes.
    void clear();

    /// Get all top-level nodes.
    const std::vector<std::shared_ptr<SceneNode>>& nodes() const { return m_nodes; }

    /// Collect all visible nodes with meshes (flattened list for rendering).
    std::vector<SceneNode*> collectVisibleMeshNodes() const;

    /// Find a node by ID (searches recursively).
    SceneNode* findNodeById(uint32_t id) const;

private:
    std::vector<std::shared_ptr<SceneNode>> m_nodes;

    void collectVisibleHelper(SceneNode* node,
                              std::vector<SceneNode*>& out) const;
    SceneNode* findByIdHelper(SceneNode* node, uint32_t id) const;
};

}  // namespace hz::render
