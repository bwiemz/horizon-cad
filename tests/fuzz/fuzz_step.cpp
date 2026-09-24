// Fuzz target: the STEP (ISO 10303-21) reader. Any input may be rejected;
// none may crash, hang or throw.

#include <cstddef>
#include <cstdint>
#include <string>

#include "horizon/fileio/StepFormat.h"

extern "C" int LLVMFuzzerTestOneInput(const uint8_t* data, size_t size) {
    const std::string text(reinterpret_cast<const char*>(data), size);
    (void)hz::io::StepFormat::fromString(text);
    return 0;
}
