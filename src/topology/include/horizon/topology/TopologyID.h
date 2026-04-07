#pragma once

#include <optional>
#include <string>
#include <vector>

namespace hz::topo {

/// Persistent topological identity that survives modeling operations.
///
/// TopologyIDs form a genealogy tree: each entity produced by a modeling
/// operation records its parent's tag plus the operation name and index.
/// This is the foundation for solving the Topological Naming Problem —
/// downstream features can relocate their referenced topology even after
/// the model is rebuilt, as long as the genealogy prefix matches.
class TopologyID {
public:
    TopologyID() = default;

    /// Create a root-level ID from a source feature name and a role within it.
    /// Example: make("box", "top") → tag "box/top"
    static TopologyID make(const std::string& source, const std::string& role);

    /// Derive a child ID from this one via an operation.
    /// Example: parent.child("split", 0) → tag "parent_tag/split:0"
    [[nodiscard]] TopologyID child(const std::string& operation, int index) const;

    /// True if this ID descends from @p ancestor (ancestor's tag is a strict
    /// prefix of this tag, followed by '/').
    [[nodiscard]] bool isDescendantOf(const TopologyID& ancestor) const;

    /// The full genealogy string.
    [[nodiscard]] const std::string& tag() const;

    /// True if the tag is non-empty (i.e. this was constructed via make/child).
    [[nodiscard]] bool isValid() const;

    bool operator==(const TopologyID& other) const;
    bool operator!=(const TopologyID& other) const;
    bool operator<(const TopologyID& other) const;

    /// Resolve @p target against a set of @p candidates.
    /// Returns exact match first; if none, returns the first descendant; else nullopt.
    static std::optional<TopologyID> resolve(const TopologyID& target,
                                             const std::vector<TopologyID>& candidates);

private:
    std::string m_tag;
    explicit TopologyID(std::string tag);
};

}  // namespace hz::topo
