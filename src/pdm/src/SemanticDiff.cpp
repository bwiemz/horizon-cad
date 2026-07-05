#include "horizon/pdm/SemanticDiff.h"

#include <algorithm>
#include <nlohmann/json.hpp>
#include <set>

namespace hz::pdm {

namespace {

using nlohmann::json;

/// Compact string form of a JSON value for display (scalars unquoted-ish,
/// containers dumped).
std::string valueString(const json& v) {
    if (v.is_string()) return v.get<std::string>();
    return v.dump();
}

void diffNode(const json& before, const json& after, const std::string& path,
              std::vector<JsonChange>& out) {
    if (before == after) return;

    // Both objects: recurse over the union of keys in a stable (sorted) order.
    if (before.is_object() && after.is_object()) {
        std::set<std::string> keys;
        for (auto it = before.begin(); it != before.end(); ++it) keys.insert(it.key());
        for (auto it = after.begin(); it != after.end(); ++it) keys.insert(it.key());
        for (const std::string& key : keys) {
            const std::string childPath = path + "/" + key;
            const bool inBefore = before.contains(key);
            const bool inAfter = after.contains(key);
            if (inBefore && inAfter) {
                diffNode(before[key], after[key], childPath, out);
            } else if (inAfter) {
                out.push_back({ChangeKind::Added, childPath, "", valueString(after[key])});
            } else {
                out.push_back({ChangeKind::Removed, childPath, valueString(before[key]), ""});
            }
        }
        return;
    }

    // Both arrays: compare element-wise by index.
    if (before.is_array() && after.is_array()) {
        const std::size_t n = std::max(before.size(), after.size());
        for (std::size_t i = 0; i < n; ++i) {
            const std::string childPath = path + "/" + std::to_string(i);
            const bool inBefore = i < before.size();
            const bool inAfter = i < after.size();
            if (inBefore && inAfter) {
                diffNode(before[i], after[i], childPath, out);
            } else if (inAfter) {
                out.push_back({ChangeKind::Added, childPath, "", valueString(after[i])});
            } else {
                out.push_back({ChangeKind::Removed, childPath, valueString(before[i]), ""});
            }
        }
        return;
    }

    // Scalars, or a type mismatch between the two revisions.
    out.push_back({ChangeKind::Modified, path, valueString(before), valueString(after)});
}

}  // namespace

std::string changeKindName(ChangeKind kind) {
    switch (kind) {
        case ChangeKind::Added:
            return "added";
        case ChangeKind::Removed:
            return "removed";
        case ChangeKind::Modified:
            return "modified";
    }
    return "modified";
}

std::vector<JsonChange> diffJson(const std::string& before, const std::string& after) {
    json b, a;
    try {
        b = json::parse(before);
        a = json::parse(after);
    } catch (const json::exception&) {
        // Not JSON — fall back to a whole-document change if they differ.
        if (before == after) return {};
        return {{ChangeKind::Modified, "", before, after}};
    }

    std::vector<JsonChange> changes;
    diffNode(b, a, "", changes);
    return changes;
}

std::string formatChanges(const std::vector<JsonChange>& changes) {
    std::string out;
    for (const JsonChange& c : changes) {
        const std::string loc = c.path.empty() ? "/" : c.path;
        switch (c.kind) {
            case ChangeKind::Added:
                out += "+ " + loc + ": " + c.after;
                break;
            case ChangeKind::Removed:
                out += "- " + loc + ": " + c.before;
                break;
            case ChangeKind::Modified:
                out += "~ " + loc + ": " + c.before + " -> " + c.after;
                break;
        }
        out += "\n";
    }
    return out;
}

}  // namespace hz::pdm
