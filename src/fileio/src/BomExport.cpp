#include "horizon/fileio/BomExport.h"

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
    out << "Item,Part,Quantity,Path\r\n";
    for (const doc::BomLine& line : bom.lines) {
        out << line.item << ',' << csvField(line.partName) << ',' << line.quantity << ','
            << csvField(line.partPath) << "\r\n";
    }

    return writeFileAtomically(pathFromUtf8(path), out.str());
}

}  // namespace hz::io
