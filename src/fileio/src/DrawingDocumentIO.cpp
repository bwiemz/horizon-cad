#include "horizon/fileio/DrawingDocumentIO.h"

#include <fstream>
#include <nlohmann/json.hpp>

#include "horizon/document/Document.h"
#include "horizon/fileio/NativeFormat.h"
#include "horizon/topology/Solid.h"

namespace hz::io {

using json = nlohmann::json;

bool DrawingDocumentIO::save(const std::string& path, const DrawingDocumentSpec& spec) {
    json root;
    root["format"] = "hzdwg";
    root["version"] = 1;
    root["part"] = spec.partPath;
    root["gap"] = spec.gap;

    std::ofstream out(path);
    if (!out) return false;
    out << root.dump(2);
    return static_cast<bool>(out);
}

bool DrawingDocumentIO::load(const std::string& path, DrawingDocumentSpec& outSpec,
                             model::Drawing& outDrawing) {
    std::ifstream in(path);
    if (!in) return false;

    json root;
    try {
        in >> root;
    } catch (const json::exception&) {
        return false;
    }

    outSpec.partPath = root.value("part", std::string());
    outSpec.gap = root.value("gap", 10.0);
    if (outSpec.partPath.empty()) return false;

    // Open and rebuild the referenced part, then regenerate the drawing from it
    // so the views reflect the current model.
    doc::Document part;
    if (!NativeFormat::load(outSpec.partPath, part)) return false;
    if (!part.rebuildModel()) return false;
    const topo::Solid* solid = part.solid();
    if (solid == nullptr) return false;

    outDrawing = model::DrawingGenerator::standardViews(*solid, outSpec.gap);
    return true;
}

}  // namespace hz::io
