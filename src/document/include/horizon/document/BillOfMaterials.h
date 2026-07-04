#pragma once

#include <string>
#include <vector>

namespace hz::doc {

class AssemblyDocument;

/// One row of a bill of materials: a unique part and how many times it occurs.
struct BomLine {
    int item = 0;          ///< 1-based line number, in order of first appearance
    std::string partName;  ///< display name (part file stem, or instance name)
    std::string partPath;  ///< the part's file reference — the identity key
    int quantity = 0;      ///< number of (unsuppressed) occurrences in the assembly
};

/// A bill of materials: the unique parts of an assembly with quantities.
struct BillOfMaterials {
    std::vector<BomLine> lines;

    /// Total unsuppressed component count (sum of quantities).
    int totalQuantity() const;
};

/// Rolls up an assembly's component instances into a bill of materials.
class BomGenerator {
public:
    /// Traverse @p assembly's components, grouping by part reference (partPath),
    /// counting occurrences. Suppressed components are excluded. Lines are
    /// numbered 1..N in order of first appearance; two instances of the same part
    /// collapse to one line with quantity 2.
    static BillOfMaterials generate(const AssemblyDocument& assembly);
};

}  // namespace hz::doc
