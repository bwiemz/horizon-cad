#include "horizon/fileio/BomExport.h"

#include <algorithm>
#include <sstream>
#include <string>

#include "horizon/document/BillOfMaterials.h"
#include "horizon/fileio/AtomicFile.h"

namespace hz::io {

namespace {

/// Quote a field per RFC 4180 when it contains a comma, quote, or newline
/// (doubling embedded quotes); otherwise return it unchanged.
std::string csvField(const std::string& value) {
    const bool needsQuote = value.find_first_of(",\"\r\n") != std::string::npos;
    if (!needsQuote) return value;

    std::string out = "\"";
    for (char ch : value) {
        if (ch == '"') out += '"';  // escape by doubling
        out += ch;
    }
    out += '"';
    return out;
}

}  // namespace

bool BomExport::toCsv(const std::string& path, const doc::BillOfMaterials& bom) {
    std::ostringstream out;
    // An indented BOM (Phase 159) says each line's level too, and its item
    // is its number under its assemblies ("2.1").
    const bool levels = std::any_of(bom.lines.begin(), bom.lines.end(),
                                    [](const doc::BomLine& line) { return line.level > 0; });
    out << (levels ? "Item,Level,Part,Quantity,Path\r\n" : "Item,Part,Quantity,Path\r\n");
    for (const doc::BomLine& line : bom.lines) {
        out << csvField(line.index.empty() ? std::to_string(line.item) : line.index) << ',';
        if (levels) out << line.level << ',';
        out << csvField(line.partName) << ',' << line.quantity << ',' << csvField(line.partPath)
            << "\r\n";
    }

    return writeFileAtomically(pathFromUtf8(path), out.str());
}

}  // namespace hz::io
