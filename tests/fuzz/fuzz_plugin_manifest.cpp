// Fuzz target: a plugin's plugin.json, which comes with a downloaded plugin.
// Any manifest may be refused; none may crash, hang or throw, and none may
// name an entry script outside its plugin's directory (the registry's own
// checks, exercised here with a directory that has an entry to find).

#include <cstddef>
#include <cstdint>
#include <string>

#include "FuzzTempDir.h"
#include "horizon/plugin/PluginRegistry.h"

extern "C" int LLVMFuzzerTestOneInput(const uint8_t* data, size_t size) {
    static const hz::fuzz::TempDir dir("hz_fuzz_plugin");
    static const bool made = [] {
        static const uint8_t script[] = "print('hello')\n";
        dir.write("main.py", script, sizeof script - 1);
        return true;
    }();
    (void)made;
    const std::string text(reinterpret_cast<const char*>(data), size);
    std::string error;
    (void)hz::plugin::PluginRegistry::parseManifest(text, dir.path(), &error);
    (void)hz::plugin::PluginRegistry::parseSemver(text);
    return 0;
}
