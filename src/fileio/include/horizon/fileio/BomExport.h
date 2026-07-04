#pragma once

#include <string>

namespace hz::doc {
struct BillOfMaterials;
}  // namespace hz::doc

namespace hz::io {

/// Exports a bill of materials to a spreadsheet-friendly CSV file.
///
/// Columns: Item, Part, Quantity, Path. Fields containing commas, quotes, or
/// newlines are quoted per RFC 4180 so the file round-trips through any
/// spreadsheet tool.
class BomExport {
public:
    /// Write @p bom to @p path as CSV (header row + one row per line). Returns
    /// false on I/O failure.
    static bool toCsv(const std::string& path, const doc::BillOfMaterials& bom);
};

}  // namespace hz::io
