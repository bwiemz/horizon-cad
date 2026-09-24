#include "horizon/render/SceneGraph.h"

#include <algorithm>

namespace hz::render {

// Static ID counter
math::IdCounter<uint32_t> SceneNode::s_nextId{1};

// --- SceneNode ---

SceneNode::SceneNode(std::string name)
    : m_name(std::move(name)), m_localTransform(math::Mat4::identity()), m_id(s_nextId.next()) {}

SceneNode::~SceneNode() = default;

math::Mat4 SceneNode::worldTransform() const {
    if (m_parent) {
        return m_parent->worldTransform() * m_localTransform;
    }
    return m_localTransform;
}

void SceneNode::addChild(std::shared_ptr<SceneNode> child) {
    if (!child) return;
    child->m_parent = this;
    m_children.push_back(std::move(child));
}

void SceneNode::removeChild(const std::shared_ptr<SceneNode>& child) {
    if (!child) return;
    auto it = std::find(m_children.begin(), m_children.end(), child);
    if (it != m_children.end()) {
        (*it)->m_parent = nullptr;
        m_children.erase(it);
    }
}

// --- SceneGraph ---

SceneGraph::SceneGraph() = default;
SceneGraph::~SceneGraph() = default;

void SceneGraph::addNode(std::shared_ptr<SceneNode> node) {
    if (node) {
        m_nodes.push_back(std::move(node));
    }
}

void SceneGraph::removeNode(const std::shared_ptr<SceneNode>& node) {
    auto it = std::find(m_nodes.begin(), m_nodes.end(), node);
    if (it != m_nodes.end()) {
        m_nodes.erase(it);
    }
}

void SceneGraph::clear() {
    m_nodes.clear();
}

std::vector<SceneNode*> SceneGraph::collectVisibleMeshNodes() const {
    std::vector<SceneNode*> result;
    for (const auto& node : m_nodes) {
        collectVisibleHelper(node.get(), result);
    }
    return result;
}

void SceneGraph::collectVisibleHelper(SceneNode* node, std::vector<SceneNode*>& out) const {
    if (!node || !node->isVisible()) return;

    if (node->hasMesh()) {
        out.push_back(node);
    }

    for (const auto& child : node->children()) {
        collectVisibleHelper(child.get(), out);
    }
}

SceneNode* SceneGraph::findNodeById(uint32_t id) const {
    for (const auto& node : m_nodes) {
        SceneNode* found = findByIdHelper(node.get(), id);
        if (found) return found;
    }
    return nullptr;
}

SceneNode* SceneGraph::findByIdHelper(SceneNode* node, uint32_t id) const {
    if (!node) return nullptr;
    if (node->id() == id) return node;

    for (const auto& child : node->children()) {
        SceneNode* found = findByIdHelper(child.get(), id);
        if (found) return found;
    }
    return nullptr;
}

std::unordered_set<uint32_t> SceneGraph::nodeIds() const {
    std::unordered_set<uint32_t> ids;
    std::vector<const SceneNode*> pending;
    pending.reserve(m_nodes.size());
    for (const auto& node : m_nodes) pending.push_back(node.get());
    while (!pending.empty()) {
        const SceneNode* node = pending.back();
        pending.pop_back();
        if (node == nullptr) continue;
        ids.insert(node->id());
        for (const auto& child : node->children()) pending.push_back(child.get());
    }
    return ids;
}

}  // namespace hz::render
