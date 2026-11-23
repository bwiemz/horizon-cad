#include "horizon/mcp/McpSession.h"

#include <algorithm>
#include <cctype>
#include <filesystem>
#include <nlohmann/json.hpp>
#include <optional>
#include <utility>
#include <vector>

#include "horizon/ai/CadTools.h"
#include "horizon/ai/SystemPrompt.h"
#include "horizon/ai/ViewImage.h"
#include "horizon/document/AssemblyDocument.h"
#include "horizon/document/AssemblyMates.h"
#include "horizon/document/Document.h"
#include "horizon/document/DocumentManager.h"
#include "horizon/document/FeatureTree.h"
#include "horizon/document/UndoStack.h"
#include "horizon/fileio/AssemblyStepExport.h"
#include "horizon/fileio/DxfFormat.h"
#include "horizon/fileio/GltfExport.h"
#include "horizon/fileio/NativeFormat.h"
#include "horizon/fileio/StepFormat.h"
#include "horizon/fileio/StlExport.h"

namespace hz::mcp {

namespace {

using nlohmann::json;
namespace fs = std::filesystem;

std::string extensionOf(const std::string& path) {
    std::string extension = fs::path(path).extension().string();
    std::transform(extension.begin(), extension.end(), extension.begin(),
                   [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
    return extension;
}

std::string pathArgument(const json& in, const char* key) {
    if (!in.contains(key) || !in[key].is_string() || in[key].get<std::string>().empty()) {
        throw std::invalid_argument(std::string("\"") + key + "\" must be a file path");
    }
    return in[key].get<std::string>();
}

/// Read @p path into a new document; null, and why in @p error, when it
/// cannot be.
std::unique_ptr<doc::Document> readDocument(const std::string& path, std::string* error) {
    auto document = std::make_unique<doc::Document>();
    const std::string extension = extensionOf(path);
    bool read = false;
    if (extension == ".hzpart" || extension == ".hcad") {
        read = io::NativeFormat::load(path, *document, error);
    } else if (extension == ".dxf") {
        read = io::DxfFormat::load(path, *document, error);
    } else {
        if (error) *error = "it is not a Horizon CAD part (.hzpart), drawing (.hcad) or DXF file";
        return nullptr;
    }
    if (!read) return nullptr;
    document->setFilePath(path);
    document->undoStack().setClean();
    document->rebuildModel();
    return document;
}

/// Whether any component of @p now is placed other than in @p before.
bool placementsDiffer(const doc::AssemblyState& before, const doc::AssemblyDocument& now) {
    for (const auto& was : before.components) {
        const doc::ComponentInstance* comp = now.component(was.id);
        if (comp == nullptr) continue;
        for (int r = 0; r < 4; ++r) {
            for (int c = 0; c < 4; ++c) {
                if (was.transform.at(r, c) != comp->transform.at(r, c)) return true;
            }
        }
    }
    return false;
}

/// Read the assembly at @p path: its components shown as their parts'
/// files hold them, and placed by its mates as the window places one it
/// opens (moved by them, it has changes to save). Null, and why in @p error,
/// when it cannot be read.
std::shared_ptr<doc::AssemblyDocument> readAssembly(const std::string& path,
                                                    doc::DocumentManager& documents,
                                                    std::string* error) {
    auto assembly = std::make_shared<doc::AssemblyDocument>();
    if (!io::NativeFormat::loadAssembly(path, *assembly, error)) return nullptr;
    assembly->setFilePath(path);
    const std::string dir = fs::path(path).parent_path().string();
    // Not inside itself, at any depth.
    const std::vector<std::string> within{doc::DocumentManager::canonicalPath(path)};
    for (auto& comp : assembly->components()) {
        documents.resolveComponent(comp, doc::ComponentState::Lightweight, dir, nullptr, within);
    }
    const doc::AssemblyState read = assembly->snapshot();
    doc::placeByMates(*assembly, documents);
    assembly->setDirty(placementsDiffer(read, *assembly));
    return assembly;
}

json summary(const doc::Document& document, const doc::AssemblyDocument* assembly,
             const std::string& path) {
    if (assembly != nullptr) {
        json out = {{"type", "assembly"},
                    {"components", assembly->components().size()},
                    {"mates", assembly->mates().size()},
                    {"modified", assembly->isDirty() || document.isDirty()}};
        out["path"] = path.empty() ? json(nullptr) : json(path);
        if (assembly->readInPart()) {
            out["left_out"] = assembly->leftOut();
            out["read_in_part"] =
                "the file holds things this version could not read: save to a new file, or "
                "saving over it loses them";
        }
        return out;
    }
    json out = {{"type", document.type() == doc::DocumentType::Part ? "part" : "drawing"},
                {"features", document.featureTree().featureCount()},
                {"drawing_entities", document.draftDocument().entities().size()},
                {"modified", document.isDirty()}};
    out["path"] = path.empty() ? json(nullptr) : json(path);
    if (document.readInPart()) {
        // Saved over its file, these would be lost.
        out["left_out"] = document.leftOut();
        out["read_in_part"] =
            "the file holds things this version could not read: save to a new file, or saving "
            "over it loses them";
    }
    if (document.failedFeatureIndex() >= 0) {
        out["build_error"] = {{"feature_index", document.failedFeatureIndex()},
                              {"message", document.lastBuildMessage()}};
    }
    return out;
}

ai::ToolSpec spec(std::string name, std::string description, json properties,
                  std::vector<std::string> required = {}) {
    json schema = {{"type", "object"}, {"properties", std::move(properties)}};
    if (!required.empty()) schema["required"] = std::move(required);
    return {std::move(name), std::move(description), std::move(schema)};
}

}  // namespace

McpSession::McpSession(bool part, const std::filesystem::path& root)
    : m_documents(std::make_unique<doc::DocumentManager>()),
      m_document(std::make_unique<doc::Document>()) {
    // An assembly's parts are read as the window reads them.
    m_documents->setPartLoader([](const std::string& path, doc::Document& document) {
        return io::NativeFormat::load(path, document);
    });
    m_documents->setMeshLoader(
        [](const std::string& path) { return io::NativeFormat::loadPartMesh(path); });
    m_documents->setAssemblyLoader([](const std::string& path, doc::AssemblyDocument& assembly) {
        return io::NativeFormat::loadAssembly(path, assembly);
    });
    std::error_code ec;
    m_rootAsGiven = fs::absolute(root, ec).lexically_normal();
    m_root = fs::weakly_canonical(m_rootAsGiven, ec);
    if (ec || m_root.empty()) m_root = m_rootAsGiven;
    m_document->setType(part ? doc::DocumentType::Part : doc::DocumentType::Drawing);
    bind();
}

namespace {

/// Whether @p path is @p root or inside it, component by component.
bool within(const fs::path& root, const fs::path& path) {
    const auto [rootEnd, pathAt] =
        std::mismatch(root.begin(), root.end(), path.begin(), path.end());
    (void)pathAt;
    return rootEnd == root.end();
}

}  // namespace

std::optional<std::filesystem::path> McpSession::resolve(const std::string& given,
                                                         std::string* why) const {
    const auto refuse = [why](std::string reason) -> std::optional<fs::path> {
        if (why) *why = std::move(reason);
        return std::nullopt;
    };
    if (given.empty()) return refuse("the path is empty");
    fs::path path(given);
    // Another drive, or a network share (\\host\share, \\?\, \\.\): refused
    // before anything touches it, so no share is ever contacted.
    if (path.has_root_name() && path.root_name() != m_root.root_name() &&
        path.root_name() != m_rootAsGiven.root_name()) {
        return refuse("it is on another drive or a network share");
    }
    if (path.is_relative()) path = m_root / path;
    const fs::path lexical = path.lexically_normal();
    const std::string outside = "it is outside the folder this server may use, " + m_root.string() +
                                " (horizon-mcp --root sets it)";
    // First by its words alone, then on the disk, where links are followed.
    if (!within(m_root, lexical) && !within(m_rootAsGiven, lexical)) return refuse(outside);
    std::error_code ec;
    fs::path resolved = fs::weakly_canonical(lexical, ec);  // not const: it is returned
    if (ec) return refuse("the path cannot be resolved: " + ec.message());
    if (!within(m_root, resolved)) return refuse(outside);
    // A link left as the last part (one to nothing, which the resolving
    // above cannot follow) would be written through, out of the folder.
    if (fs::is_symlink(fs::symlink_status(resolved, ec))) {
        return refuse("it is a symbolic link");
    }
    return resolved;
}

McpSession::~McpSession() = default;

bool McpSession::open(const std::string& path, std::string* error) {
    std::shared_ptr<doc::AssemblyDocument> assembly;
    std::unique_ptr<doc::Document> document;
    if (extensionOf(path) == ".hzasm") {
        assembly = readAssembly(path, *m_documents, error);
        if (!assembly) return false;
        document = std::make_unique<doc::Document>();  // holds its undo steps
    } else {
        document = readDocument(path, error);
        if (!document) return false;
    }
    // The tools that refer to the old document go before it does.
    m_tools.clear();
    m_document = std::move(document);
    m_assembly = std::move(assembly);
    m_path = path;
    bind();
    return true;
}

void McpSession::applyPending() {
    if (!m_pending) return;
    // The tools that refer to the old document go before it does.
    m_tools.clear();
    m_document = std::move(m_pending);
    m_assembly = std::move(m_pendingAssembly);
    m_path = std::move(m_pendingPath);
    m_pendingPath.clear();
    bind();
}

void McpSession::bind() {
    m_tools.clear();
    if (m_assembly) {
        ai::AssemblyToolHooks hooks;
        // Unsaved, it takes parts from the server's folder; saved, from its
        // own (in it, when this server saved it).
        hooks.unsavedFolder = m_root.string();
        ai::registerAssemblyTools(m_tools, *m_assembly, *m_document, *m_documents,
                                  std::move(hooks));
        ai::registerViewTool(m_tools, *m_assembly);
    } else {
        ai::registerCadTools(m_tools, *m_document);
        ai::registerViewTool(m_tools, *m_document);
    }
    registerDocumentTools();
}

std::optional<std::string> McpSession::overwriteRefused(const std::string& path,
                                                        const nlohmann::json& in) const {
    std::error_code ec;
    if (!fs::exists(path, ec)) return std::nullopt;
    if (!m_path.empty() && fs::equivalent(path, m_path, ec)) return std::nullopt;  // its own file
    if (in.contains("overwrite") && in["overwrite"].is_boolean() && in["overwrite"].get<bool>()) {
        return std::nullopt;
    }
    return path + " already exists: pass overwrite true to replace it (ask the user first).";
}

void McpSession::registerDocumentTools() {
    m_tools.add(
        spec("get_session_info",
             "The document this server works on: part, drawing or assembly, its file (null "
             "until saved), whether it has unsaved changes, any build error, and the folder "
             "files may be opened, saved and exported in.",
             json::object()),
        ai::ToolAccess::Read, [this](const json&) {
            json out = summary(*m_document, m_assembly.get(), m_path);
            out["folder"] = m_root.string();
            return ai::ToolResult::ok(out);
        });

    m_tools.add(spec("new_document",
                     "Start a new, empty part (default), drawing or assembly, in place of the "
                     "current document. Unsaved changes to it are lost: save first. An unsaved "
                     "assembly takes its parts from the server's folder.",
                     {{"type", {{"type", "string"}, {"enum", {"part", "drawing", "assembly"}}}}}),
                ai::ToolAccess::Modify, [this](const json& in) {
                    const std::string type = in.contains("type") && in["type"].is_string()
                                                 ? in["type"].get<std::string>()
                                                 : "part";
                    if (type != "part" && type != "drawing" && type != "assembly") {
                        throw std::invalid_argument("\"type\" must be part, drawing or assembly");
                    }
                    m_pending = std::make_unique<doc::Document>();
                    m_pending->setType(type == "drawing" ? doc::DocumentType::Drawing
                                                         : doc::DocumentType::Part);
                    m_pendingAssembly =
                        type == "assembly" ? std::make_shared<doc::AssemblyDocument>() : nullptr;
                    m_pendingPath.clear();
                    return ai::ToolResult::ok(json{{"new", type}});
                });

    m_tools.add(spec("open_document",
                     "Open a Horizon CAD part (.hzpart), drawing (.hcad), assembly (.hzasm) or a "
                     "DXF file, in place of the current document. Unsaved changes to it are "
                     "lost: save first.",
                     {{"path",
                       {{"type", "string"},
                        {"description",
                         "the file's path, in the server's folder (relative to it, or "
                         "absolute inside it)"}}}},
                     {"path"}),
                ai::ToolAccess::External, [this](const json& in) {
                    std::string why;
                    const auto resolved = resolve(pathArgument(in, "path"), &why);
                    if (!resolved) return ai::ToolResult::error("Cannot open that file: " + why);
                    const std::string path = resolved->string();
                    std::string error;
                    if (extensionOf(path) == ".hzasm") {
                        // What it places is read too, and goes to the model:
                        // kept to the folder, checked from the files before
                        // any part is read.
                        if (!m_documents->placesOnly(path, [this](const std::string& file) {
                                return resolve(file).has_value();
                            })) {
                            return ai::ToolResult::error(
                                "Cannot open that assembly: it places parts from outside the "
                                "server's folder, or one of its subassemblies cannot be read.");
                        }
                        auto assembly = readAssembly(path, *m_documents, &error);
                        if (!assembly) {
                            return ai::ToolResult::error("Could not open " + path + ": " + error);
                        }
                        auto backing = std::make_unique<doc::Document>();
                        json out = summary(*backing, assembly.get(), path);
                        m_pending = std::move(backing);
                        m_pendingAssembly = std::move(assembly);
                        m_pendingPath = path;
                        return ai::ToolResult::ok(out);
                    }
                    auto document = readDocument(path, &error);
                    if (!document) {
                        return ai::ToolResult::error("Could not open " + path + ": " + error);
                    }
                    json out = summary(*document, nullptr, path);
                    m_pending = std::move(document);
                    m_pendingAssembly.reset();
                    m_pendingPath = path;
                    return ai::ToolResult::ok(out);
                });

    m_tools.add(
        spec("save_document",
             "Save the document as a Horizon CAD file: a part as .hzpart, a drawing as .hcad. "
             "With no path, to the file it was opened from or last saved to.",
             {{"path",
               {{"type", "string"},
                {"description", "where to save (.hzpart or .hcad), in the server's folder"}}},
              {"overwrite",
               {{"type", "boolean"}, {"description", "replace a file already there"}}}}),
        ai::ToolAccess::External, [this](const json& in) {
            // Its own file, which the user chose: kept to wherever it is.
            std::string path = m_path;
            const bool asked = in.contains("overwrite") && in["overwrite"].is_boolean() &&
                               in["overwrite"].get<bool>();
            if (in.contains("path") && in["path"].is_string()) {
                std::string why;
                const auto resolved = resolve(in["path"].get<std::string>(), &why);
                if (!resolved) return ai::ToolResult::error("Cannot save there: " + why);
                path = resolved->string();
                if (const auto refused = overwriteRefused(path, in)) {
                    return ai::ToolResult::error(*refused);
                }
            }
            if (path.empty()) {
                return ai::ToolResult::error("The document has no file yet: give a path.");
            }
            // A file read in part keeps what was left out of it; saving over
            // it would lose that, so it is not done unasked (as the window
            // asks before it replaces such a file).
            std::error_code same;
            const bool readInPart =
                m_assembly ? m_assembly->readInPart() : m_document->readInPart();
            const std::size_t leftOut =
                m_assembly ? m_assembly->leftOut().size() : m_document->leftOut().size();
            if (readInPart && !m_path.empty() && !asked &&
                (path == m_path || fs::equivalent(path, m_path, same))) {
                return ai::ToolResult::error(
                    "This document was read from " + m_path +
                    " in part: saving over it would lose what this version could not read (" +
                    std::to_string(leftOut) +
                    " items; see get_session_info). Save to a new path, or pass overwrite true "
                    "after asking the user.");
            }
            const std::string extension = extensionOf(path);
            if (m_assembly) {
                if (extension != ".hzasm") {
                    return ai::ToolResult::error(
                        "Save an assembly as .hzasm; for STEP, STL or glTF use export_document.");
                }
                std::string error;
                if (!io::NativeFormat::saveAssembly(path, *m_assembly, &error)) {
                    return ai::ToolResult::error("Could not save " + path + ": " + error);
                }
                m_assembly->setFilePath(path);
                m_assembly->setDirty(false);
                m_assembly->setLeftOut({});   // the file now holds all it has
                m_document->setDirty(false);  // its undo stack's saved state
                m_path = path;
                return ai::ToolResult::ok(json{{"saved", path}});
            }
            if (extension != ".hzpart" && extension != ".hcad") {
                return ai::ToolResult::error(
                    "Save as .hzpart (a part) or .hcad (a drawing); for other formats use "
                    "export_document.");
            }
            // The file's kind is the document's: set for the save, and put
            // back when the save fails.
            const doc::DocumentType before = m_document->type();
            m_document->setType(extension == ".hzpart" ? doc::DocumentType::Part
                                                       : doc::DocumentType::Drawing);
            std::string error;
            if (!io::NativeFormat::save(path, *m_document, &error)) {
                m_document->setType(before);
                return ai::ToolResult::error("Could not save " + path + ": " + error);
            }
            m_document->setFilePath(path);
            m_document->undoStack().setClean();
            m_document->setLeftOut({});  // the file now holds all it has
            m_path = path;
            return ai::ToolResult::ok(json{{"saved", path}});
        });

    m_tools.add(
        spec(
            "export_document",
            "Export the part's solid as STEP (.step, .stp), STL (.stl) or glTF binary (.glb), or "
            "the 2D drawing as DXF (.dxf), by the path's extension.",
            {{"path",
              {{"type", "string"}, {"description", "the file to write, in the server's folder"}}},
             {"overwrite", {{"type", "boolean"}, {"description", "replace a file already there"}}}},
            {"path"}),
        ai::ToolAccess::External, [this](const json& in) {
            std::string why;
            const auto resolved = resolve(pathArgument(in, "path"), &why);
            if (!resolved) return ai::ToolResult::error("Cannot export there: " + why);
            const std::string path = resolved->string();
            if (const auto refused = overwriteRefused(path, in)) {
                return ai::ToolResult::error(*refused);
            }
            const std::string extension = extensionOf(path);
            if (m_assembly) return exportAssembly(path, extension);
            std::string error;
            bool written = false;
            if (extension == ".dxf") {
                written = io::DxfFormat::save(path, *m_document, &error);
            } else {
                const topo::Solid* solid = m_document->solid();
                if (solid == nullptr) {
                    return ai::ToolResult::error("The part has no solid to export.");
                }
                if (extension == ".step" || extension == ".stp") {
                    written = io::StepFormat::save(path, {solid});
                    if (!written) error = io::StepFormat::lastError();
                } else if (extension == ".stl") {
                    written = io::StlExport::save(path, *solid, &error);
                } else if (extension == ".glb") {
                    written = io::GltfExport::saveSolid(path, *solid, render::Material{});
                    if (!written)
                        error = "the solid has no triangles, or the file cannot be written";
                } else {
                    return ai::ToolResult::error(
                        "Export to .step, .stp, .stl, .glb or .dxf (given: " + extension + ").");
                }
            }
            if (!written) return ai::ToolResult::error("Could not export " + path + ": " + error);
            return ai::ToolResult::ok(json{{"exported", path}});
        });
}

ai::ToolResult McpSession::exportAssembly(const std::string& path, const std::string& extension) {
    if (extension == ".step" || extension == ".stp") {
        // Each part once, and each component a use of it where it is placed,
        // as the window's STEP export writes it.
        const io::AssemblyStepExport gathered = io::gatherAssemblyStep(*m_assembly, *m_documents);
        if (gathered.occurrences.empty()) {
            return ai::ToolResult::error(
                "None of the assembly's parts can be read: there is "
                "nothing to export.");
        }
        const std::string title = m_path.empty() ? "Assembly" : fs::path(m_path).stem().string();
        if (!io::StepFormat::saveAssembly(path, title, gathered.parts, gathered.occurrences)) {
            return ai::ToolResult::error("Could not export " + path + ": " +
                                         io::StepFormat::lastError());
        }
        json out = {{"exported", path},
                    {"parts", gathered.parts.size()},
                    {"components", gathered.occurrences.size()}};
        if (!gathered.unread.empty()) out["left_out_unreadable"] = gathered.unread;
        return ai::ToolResult::ok(out);
    }
    if (extension != ".stl" && extension != ".glb") {
        return ai::ToolResult::error(
            "Export an assembly to .step, .stp, .stl or .glb (given: " + extension + ").");
    }
    // One mesh of every component, placed: their parts read whole.
    const std::string dir = m_assembly->filePath().empty()
                                ? std::string()
                                : fs::path(m_assembly->filePath()).parent_path().string();
    for (auto& comp : m_assembly->components()) {
        if (!comp.suppressed && comp.solid() == nullptr) {
            m_documents->resolveComponent(comp, doc::ComponentState::Resolved, dir);
        }
    }
    std::vector<uint64_t> missing;
    const auto solid = m_assembly->drawingSolid(
        [](const doc::ComponentInstance& comp) { return comp.ownSolid(); }, &missing);
    if (!solid) {
        return ai::ToolResult::error(
            "None of the assembly's parts can be read: there is "
            "nothing to export.");
    }
    std::string error;
    bool written = false;
    if (extension == ".stl") {
        written = io::StlExport::save(path, *solid, &error);
    } else {
        written = io::GltfExport::saveSolid(path, *solid, render::Material{});
        if (!written) error = "the assembly has no triangles, or the file cannot be written";
    }
    if (!written) return ai::ToolResult::error("Could not export " + path + ": " + error);
    json out = {{"exported", path}};
    if (!missing.empty()) out["components_left_out_unreadable"] = missing.size();
    return ai::ToolResult::ok(out);
}

std::string serverInstructions() {
    return ai::defaultSystemPrompt() +
           "\n\nThrough this server you work on one document at a time. get_session_info says "
           "which; new_document and open_document replace it (save first: replacing it loses "
           "unsaved changes). Nothing is saved until you call save_document; export_document "
           "writes STEP, STL, glTF or DXF. Files are opened, saved and exported only within the "
           "server's folder (get_session_info names it); a file already there is replaced only "
           "with overwrite true, after asking the user."
           "\n\nTo assemble parts: make and save each part (.hzpart) in the folder, then "
           "new_document with type assembly, insert_component each part file, and mate them; "
           "save it as .hzasm (beside its parts) and export_document to STEP for other CAD "
           "systems.";
}

}  // namespace hz::mcp
