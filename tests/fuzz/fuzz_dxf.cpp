// Fuzz target: the DXF reader. Any input may be rejected; none may crash,
// hang or throw.

#include <cstddef>
#include <cstdint>
#include <string>

#include "horizon/document/Document.h"
#include "horizon/fileio/DxfFormat.h"

extern "C" int LLVMFuzzerTestOneInput(const uint8_t* data, size_t size) {
    const std::string text(reinterpret_cast<const char*>(data), size);
    hz::doc::Document doc;
    std::string error;
    (void)hz::io::DxfFormat::loadFromString(text, doc, &error);
    return 0;
}
