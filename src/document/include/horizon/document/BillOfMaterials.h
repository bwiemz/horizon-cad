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
    /// How deep it is (Phase 159): 0 for the assembly's own components, 1
    /// for a subassembly's, and so on; only an indented BOM has any but 0.
    int level = 0;
    /// Its number with its assemblies' before it ("2.1"): the item alone at
    /// level 0.
    std::string index;
    bool assembly = false;  ///< a subassembly's line
};

/// Which bill of materials (Phase 159).
enum class BomKind {
    TopLevel,   ///< the assembly's own components: a subassembly one line
    Indented,   ///< and under each subassembly its own lines, numbered 2.1, 2.2...
    PartsOnly,  ///< every part at any depth, quantities multiplied through
};

/// A bill of materials: the unique parts of an assembly with quantities.
struct BillOfMaterials {
    std::vector<BomLine> lines;

    /// Total unsuppressed component count (sum of quantities): the
    /// assembly's own components, or, parts only, every part.
    int totalQuantity() const;
};

/// Rolls up an assembly's component instances into a bill of materials.
class BomGenerator {
public:
    /// Traverse @p assembly's components, grouping by part file (partPath,
    /// a relative one taken from the assembly's folder, made canonical: one
    /// file spelled two ways is one line), counting occurrences. Suppressed components are
    /// excluded. Lines are numbered 1..N in order of first appearance; two instances of the same
    /// part collapse to one line with quantity 2.
    ///
    /// A subassembly's own components come from its resolved assembly
    /// (ComponentInstance::resolvedAssembly): an indented BOM lists them
    /// under it, their quantities each one of it holds; parts only lists the
    /// parts at every depth instead, their quantities multiplied through.
    /// One not resolved is a line of its own.
    static BillOfMaterials generate(const AssemblyDocument& assembly,
                                    BomKind kind = BomKind::TopLevel);
};

}  // namespace hz::doc
