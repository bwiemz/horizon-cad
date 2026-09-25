#include "horizon/fileio/StepAssemblyFiles.h"

#include <algorithm>
#include <cctype>
#include <filesystem>
#include <memory>
#include <set>
#include <string_view>
#include <system_error>
#include <utility>

#include "horizon/document/AssemblyDocument.h"
#include "horizon/document/Document.h"
#include "horizon/document/FeatureTree.h"
#include "horizon/fileio/AtomicFile.h"
#include "horizon/fileio/NativeFormat.h"

namespace hz::io {

namespace {

std::string utf8Of(const std::filesystem::path& path) {
    const std::u8string text = path.u8string();
    return {text.begin(), text.end()};
}

/// @p name as a file's name on any system: the characters some cannot take
/// replaced, no space or dot at either end, never empty, and never a name
/// Windows keeps for a device.
std::string fileNameOf(const std::string& name) {
    std::string out;
    out.reserve(name.size());
    for (const char c : name) {
        const bool control = static_cast<unsigned char>(c) < 0x20;
        out +=
            control || std::string_view("<>:\"/\\|?*").find(c) != std::string_view::npos ? '_' : c;
    }
    const auto edge = [](char c) { return c == ' ' || c == '.'; };
    while (!out.empty() && edge(out.back())) out.pop_back();
    const auto first = std::find_if_not(out.begin(), out.end(), edge);
    out.erase(out.begin(), first);
    if (out.empty()) return "part";
    std::string upper = out.substr(0, out.find('.'));
    for (char& c : upper) c = static_cast<char>(std::toupper(static_cast<unsigned char>(c)));
    static const std::set<std::string> kDevices{"CON", "PRN", "AUX", "NUL"};
    const bool numbered = upper.size() == 4 &&
                          (upper.rfind("COM", 0) == 0 || upper.rfind("LPT", 0) == 0) &&
                          upper[3] >= '1' && upper[3] <= '9';
    if (kDevices.count(upper) != 0 || numbered) out += '_';
    return out;
}

std::string lowered(std::string text) {
    for (char& c : text) c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
    return text;
}

}  // namespace

bool saveStepAssembly(StepAssembly& read, const std::string& assemblyPath,
                      const std::string& partsDir, const std::string& source,
                      StepAssemblyFiles* written, std::string* error) {
    namespace fs = std::filesystem;
    const auto fail = [error](std::string why) {
        if (error != nullptr) *error = std::move(why);
        return false;
    };
    StepAssemblyFiles files;
    const auto keep = [&] {
        if (written != nullptr) *written = files;
    };
    const fs::path dir = pathFromUtf8(partsDir);
    std::error_code ec;
    fs::create_directories(dir, ec);
    if (ec) return fail("the folder for its parts could not be made: " + ec.message());

    // Each part a part file of its own, its bodies imported.
    std::set<std::string> taken;  // names given here, as a case-blind system sees them
    for (StepAssembly::Part& part : read.parts) {
        doc::Document document;
        document.setType(doc::DocumentType::Part);
        for (auto& body : part.bodies) {
            if (!body) continue;
            document.featureTree().addFeature(std::make_unique<doc::ImportedBodyFeature>(
                std::shared_ptr<const topo::Solid>(std::move(body)), source));
        }
        part.bodies.clear();
        document.rebuildModel();

        const std::string stem = fileNameOf(part.name);
        fs::path file;
        for (int n = 1;; ++n) {
            const std::string name = n == 1 ? stem : stem + " " + std::to_string(n);
            file = dir / pathFromUtf8(name + ".hzpart");
            if (taken.count(lowered(name)) == 0 && !fs::exists(file, ec)) {
                taken.insert(lowered(name));
                break;
            }
        }
        std::string why;
        if (!NativeFormat::save(utf8Of(file), document, &why)) {
            keep();
            return fail("its part \"" + part.name + "\" could not be written: " + why);
        }
        files.parts.push_back(utf8Of(fs::absolute(file, ec)));
    }

    // The assembly, placing them.
    doc::AssemblyDocument assembly;
    for (const StepOccurrence& occurrence : read.occurrences) {
        if (occurrence.part >= files.parts.size()) continue;
        doc::ComponentInstance component;
        component.name = occurrence.name;
        component.partPath = files.parts[occurrence.part];
        component.transform = occurrence.transform;
        assembly.addComponent(std::move(component));
    }
    std::string why;
    if (!NativeFormat::saveAssembly(assemblyPath, assembly, &why)) {
        keep();
        return fail("its assembly could not be written: " + why);
    }
    files.assembly = assemblyPath;
    keep();
    return true;
}

}  // namespace hz::io
