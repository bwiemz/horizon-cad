// Fuzz target: the binary (FlatBuffers) part and assembly reader, including
// its fast path that reads only the tessellation cache. Any input may be
// rejected; none may crash, hang or throw.

#include <cstddef>
#include <cstdint>

#include "FuzzTempDir.h"
#include "horizon/document/AssemblyDocument.h"
#include "horizon/document/Document.h"
#include "horizon/fileio/BinaryFormat.h"

extern "C" int LLVMFuzzerTestOneInput(const uint8_t* data, size_t size) {
    static const hz::fuzz::TempDir dir("hz_fuzz_binary");
    const std::string path = dir.write("input.hzpart", data, size).string();
    (void)hz::io::BinaryFormat::isBinaryFile(path);
    (void)hz::io::BinaryFormat::loadPartMesh(path);
    {
        hz::doc::Document doc;
        (void)hz::io::BinaryFormat::load(path, doc);
    }
    {
        hz::doc::AssemblyDocument assembly;
        (void)hz::io::BinaryFormat::loadAssembly(path, assembly);
    }
    return 0;
}
