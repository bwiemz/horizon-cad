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

void removeStepAssemblyFiles(const StepAssemblyFiles& files) {
    namespace fs = std::filesystem;
    std::error_code ec;
    for (const std::string& part : files.parts) fs::remove(pathFromUtf8(part), ec);
    if (!files.assembly.empty()) fs::remove(pathFromUtf8(files.assembly), ec);
    // fs::remove takes a folder only when it is empty: one that holds
    // anything else is kept.
    if (files.madePartsDir && !files.partsDir.empty()) fs::remove(pathFromUtf8(files.partsDir), ec);
}

bool saveStepAssembly(StepAssembly& read, const std::string& assemblyPath,
                      const std::string& partsDir, const std::string& source,
                      StepAssemblyFiles* written, std::string* error,
                      const std::atomic<bool>* cancelled, const StepAssemblyProgress& progress) {
    namespace fs = std::filesystem;
    StepAssemblyFiles files;
    files.partsDir = partsDir;
    // What does not finish leaves nothing: what it wrote goes again.
    const auto fail = [&](std::string why) {
        removeStepAssemblyFiles(files);
        if (error != nullptr) *error = std::move(why);
        return false;
    };
    const auto stopped = [cancelled] {
        return cancelled != nullptr && cancelled->load(std::memory_order_relaxed);
    };
    const fs::path dir = pathFromUtf8(partsDir);
    std::error_code ec;
    files.madePartsDir = !fs::exists(dir, ec);
    fs::create_directories(dir, ec);
    if (ec) {
        files.madePartsDir = false;  // not made: nothing of it to take away
        return fail("the folder for its parts could not be made: " + ec.message());
    }

    // Each part a part file of its own, its bodies imported.
    std::set<std::string> taken;  // names given here, as a case-blind system sees them
    for (StepAssembly::Part& part : read.parts) {
        if (stopped()) return fail("cancelled");
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
            return fail("its part \"" + part.name + "\" could not be written: " + why);
        }
        files.parts.push_back(utf8Of(fs::absolute(file, ec)));
        if (progress) progress(files.parts.size(), read.parts.size());
    }
    if (stopped()) return fail("cancelled");

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
        return fail("its assembly could not be written: " + why);
    }
    files.assembly = assemblyPath;
    if (written != nullptr) *written = files;
    return true;
}

}  // namespace hz::io
