#include "horizon/topology/TopologyID.h"

#include <utility>

namespace hz::topo {

TopologyID::TopologyID(std::string tag) : m_tag(std::move(tag)) {}

TopologyID TopologyID::make(const std::string& source, const std::string& role) {
    return TopologyID(source + "/" + role);
}

TopologyID TopologyID::child(const std::string& operation, int index) const {
    return TopologyID(m_tag + "/" + operation + ":" + std::to_string(index));
}

bool TopologyID::isDescendantOf(const TopologyID& ancestor) const {
    if (m_tag.size() <= ancestor.m_tag.size()) {
        return false;
    }
    return m_tag.compare(0, ancestor.m_tag.size(), ancestor.m_tag) == 0 &&
           m_tag[ancestor.m_tag.size()] == '/';
}

const std::string& TopologyID::tag() const {
    return m_tag;
}

bool TopologyID::isValid() const {
    return !m_tag.empty();
}

bool TopologyID::operator==(const TopologyID& other) const {
    return m_tag == other.m_tag;
}

bool TopologyID::operator!=(const TopologyID& other) const {
    return m_tag != other.m_tag;
}

bool TopologyID::operator<(const TopologyID& other) const {
    return m_tag < other.m_tag;
}

std::optional<TopologyID> TopologyID::resolve(const TopologyID& target,
                                              const std::vector<TopologyID>& candidates) {
    // Exact match first.
    for (const auto& c : candidates) {
        if (c == target) {
            return c;
        }
    }
    // Fallback: first descendant.
    for (const auto& c : candidates) {
        if (c.isDescendantOf(target)) {
            return c;
        }
    }
    return std::nullopt;
}

}  // namespace hz::topo
