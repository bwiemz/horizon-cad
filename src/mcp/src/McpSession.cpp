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
#include "horizon/document/Document.h"
#include "horizon/document/FeatureTree.h"
#include "horizon/document/UndoStack.h"
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

json summary(const doc::Document& document, const std::string& path) {
    json out = {{"type", document.type() == doc::DocumentType::Part ? "part" : "drawing"},
                {"features", document.featureTree().featureCount()},
                {"drawing_entities", document.draftDocument().entities().size()},
                {"modified", document.isDirty()}};
    out["path"] = path.empty() ? json(nullptr) : json(path);
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
    : m_document(std::make_unique<doc::Document>()) {
    std::error_code ec;
    m_root = fs::weakly_canonical(fs::absolute(root, ec), ec);
    if (ec || m_root.empty()) m_root = fs::absolute(root);
    m_document->setType(part ? doc::DocumentType::Part : doc::DocumentType::Drawing);
    bind();
}

std::optional<std::filesystem::path> McpSession::resolve(const std::string& given,
                                                         std::string* why) const {
    if (given.empty()) {
        if (why) *why = "the path is empty";
        return std::nullopt;
    }
    fs::path path(given);
    if (path.is_relative()) path = m_root / path;
    // Symbolic links and ".." resolved, so neither leads out of the root.
    std::error_code ec;
    const fs::path resolved = fs::weakly_canonical(path, ec);
    if (ec) {
        if (why) *why = "the path cannot be resolved: " + ec.message();
        return std::nullopt;
    }
    const auto [rootEnd, pathAt] =
        std::mismatch(m_root.begin(), m_root.end(), resolved.begin(), resolved.end());
    if (rootEnd != m_root.end()) {
        if (why) {
            *why = "it is outside the folder this server may use, " + m_root.string() +
                   " (horizon-mcp --root sets it)";
        }
        return std::nullopt;
    }
    return resolved;
}

McpSession::~McpSession() = default;

bool McpSession::open(const std::string& path, std::string* error) {
    auto document = readDocument(path, error);
    if (!document) return false;
    m_document = std::move(document);
    m_path = path;
    bind();
    return true;
}

void McpSession::applyPending() {
    if (!m_pending) return;
    // The tools that refer to the old document go before it does.
    m_tools.clear();
    m_document = std::move(m_pending);
    m_path = std::move(m_pendingPath);
    m_pendingPath.clear();
    bind();
}

void McpSession::bind() {
    m_tools.clear();
    ai::registerCadTools(m_tools, *m_document);
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
             "The document this server works on: part or drawing, its file (null until saved), "
             "whether it has unsaved changes, any build error, and the folder files may be "
             "opened, saved and exported in.",
             json::object()),
        ai::ToolAccess::Read, [this](const json&) {
            json out = summary(*m_document, m_path);
            out["folder"] = m_root.string();
            return ai::ToolResult::ok(out);
        });

    m_tools.add(spec("new_document",
                     "Start a new, empty part (default) or drawing, in place of the current "
                     "document. Unsaved changes to it are lost: save first.",
                     {{"type", {{"type", "string"}, {"enum", {"part", "drawing"}}}}}),
                ai::ToolAccess::Modify, [this](const json& in) {
                    const std::string type = in.contains("type") && in["type"].is_string()
                                                 ? in["type"].get<std::string>()
                                                 : "part";
                    if (type != "part" && type != "drawing") {
                        throw std::invalid_argument("\"type\" must be part or drawing");
                    }
                    m_pending = std::make_unique<doc::Document>();
                    m_pending->setType(type == "part" ? doc::DocumentType::Part
                                                      : doc::DocumentType::Drawing);
                    m_pendingPath.clear();
                    return ai::ToolResult::ok(json{{"new", type}});
                });

    m_tools.add(spec("open_document",
                     "Open a Horizon CAD part (.hzpart), drawing (.hcad) or a DXF file, in place "
                     "of the current document. Unsaved changes to it are lost: save first.",
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
                    auto document = readDocument(path, &error);
                    if (!document) {
                        return ai::ToolResult::error("Could not open " + path + ": " + error);
                    }
                    json out = summary(*document, path);
                    m_pending = std::move(document);
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
            const std::string extension = extensionOf(path);
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

std::string serverInstructions() {
    return ai::defaultSystemPrompt() +
           "\n\nThrough this server you work on one document at a time. get_session_info says "
           "which; new_document and open_document replace it (save first: replacing it loses "
           "unsaved changes). Nothing is saved until you call save_document; export_document "
           "writes STEP, STL, glTF or DXF. Files are opened, saved and exported only within the "
           "server's folder (get_session_info names it); a file already there is replaced only "
           "with overwrite true, after asking the user.";
}

}  // namespace hz::mcp
