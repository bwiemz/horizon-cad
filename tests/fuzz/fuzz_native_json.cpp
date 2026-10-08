// Fuzz target: the native document and assembly readers (.hcad/.hzpart/.hzasm),
// and the fast path that reads only a part's tessellation cache. Any input may
// be rejected; none may crash, hang or throw.

#include <cstddef>
#include <cstdint>
#include <string>

#include "FuzzTempDir.h"
#include "horizon/document/AssemblyDocument.h"
#include "horizon/document/Document.h"
#include "horizon/fileio/NativeFormat.h"

extern "C" int LLVMFuzzerTestOneInput(const uint8_t* data, size_t size) {
    const std::string text(reinterpret_cast<const char*>(data), size);
    {
        hz::doc::Document doc;
        std::string error;
        (void)hz::io::NativeFormat::documentFromJson(text, doc, &error);
    }
    {
        hz::doc::AssemblyDocument asmDoc;
        std::string error;
        (void)hz::io::NativeFormat::assemblyFromJson(text, asmDoc, "", &error);
    }
    {
        // It reads only from a file.
        static const hz::fuzz::TempDir dir("hz_fuzz_native");
        (void)hz::io::NativeFormat::loadPartMesh(dir.write("input.hzpart", data, size).string());
    }
    return 0;
}
