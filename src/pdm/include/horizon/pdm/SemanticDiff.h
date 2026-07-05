#pragma once

#include <string>
#include <vector>

namespace hz::pdm {

/// The nature of a single change between two document revisions.
enum class ChangeKind {
    Added,     ///< a value present only in the newer revision
    Removed,   ///< a value present only in the older revision
    Modified,  ///< a value that exists in both but differs
};

/// One structured difference between two revisions, located by a JSON-pointer
/// path (e.g. "/features/0/radius").
struct JsonChange {
    ChangeKind kind = ChangeKind::Modified;
    std::string path;    ///< JSON-pointer path to the changed value
    std::string before;  ///< value in the older revision ("" for Added)
    std::string after;   ///< value in the newer revision ("" for Removed)
};

/// The short name of a change kind ("added", "removed", "modified").
std::string changeKindName(ChangeKind kind);

/// Compute the structural (semantic) diff between two JSON documents given as
/// strings — the feature tree + parameter table serialized as JSON. Objects are
/// compared key-by-key and arrays element-by-element (positionally); scalars and
/// type mismatches yield a Modified entry with both values. Changes are returned
/// in a stable, depth-first order.
///
/// If either input is not valid JSON, returns a single Modified change at the
/// root path "" with the raw strings, so callers still get a usable result.
std::vector<JsonChange> diffJson(const std::string& before, const std::string& after);

/// A human-readable one-line-per-change rendering, e.g.
/// "~ /features/0/radius: 5.0 -> 6.0" (modified), "+ /params/x: 3" (added),
/// "- /features/2: {...}" (removed).
std::string formatChanges(const std::vector<JsonChange>& changes);

}  // namespace hz::pdm
