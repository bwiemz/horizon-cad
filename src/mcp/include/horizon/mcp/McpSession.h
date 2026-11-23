#pragma once

#include <filesystem>
#include <memory>
#include <nlohmann/json.hpp>
#include <optional>
#include <string>

#include "horizon/ai/ToolRegistry.h"

namespace hz::doc {
class AssemblyDocument;
class Document;
class DocumentManager;
}  // namespace hz::doc

namespace hz::mcp {

/// The document an MCP client works on, a part, a drawing or an assembly,
/// and the tools it is given: the CAD tools (hz::ai::registerCadTools), or
/// for an assembly the assembly tools (hz::ai::registerAssemblyTools), and
/// new_document, open_document, save_document, export_document and
/// get_session_info for its files.
///
/// Opening or starting a document replaces the one held, after the tool call
/// that asked for it returns (applyPending()): a tool's handler cannot be
/// replaced while it runs.
class McpSession {
public:
    /// Start with an empty part (or drawing). Files are opened, saved and
    /// exported only within @p root (a model chooses their paths): a path
    /// is taken relative to it, and one that leads out of it, through ".."
    /// or a symbolic link, is refused.
    explicit McpSession(bool part = true,
                        const std::filesystem::path& root = std::filesystem::current_path());
    ~McpSession();

    McpSession(const McpSession&) = delete;
    McpSession& operator=(const McpSession&) = delete;

    /// The folder files may be used in.
    const std::filesystem::path& root() const { return m_root; }
    /// @p given within the root, resolved; nullopt, and why, when it is not.
    std::optional<std::filesystem::path> resolve(const std::string& given,
                                                 std::string* why = nullptr) const;

    /// Open @p path (.hzpart, .hcad, .dxf or .hzasm) as the document now. (The
    /// user's own choice, on the command line: not held to the root.) False, and
    /// why in @p error, when it cannot be read; the document is unchanged.
    bool open(const std::string& path, std::string* error = nullptr);

    ai::ToolRegistry& tools() { return m_tools; }
    doc::Document& document() { return *m_document; }
    /// The assembly worked on, when the document is one (document() then
    /// holds its undo steps); else null.
    doc::AssemblyDocument* assembly() { return m_assembly.get(); }
    const std::string& path() const { return m_path; }

    /// Put in place a document a tool asked for. Call after each message.
    void applyPending();

private:
    void bind();
    void registerDocumentTools();
    /// Export the assembly to @p path: STEP with its parts and placements,
    /// or one mesh of every component (STL, glTF).
    ai::ToolResult exportAssembly(const std::string& path, const std::string& extension);
    /// Why writing @p path is refused: it exists, is not the document's own
    /// file, and @p in does not say overwrite. Nullopt when it may be written.
    std::optional<std::string> overwriteRefused(const std::string& path,
                                                const nlohmann::json& in) const;

    std::filesystem::path m_root;         ///< resolved
    std::filesystem::path m_rootAsGiven;  ///< as given, absolute, before links are followed

    /// Reads an assembly's parts. Before the documents: they hold parts it
    /// knows of, and go first.
    std::unique_ptr<doc::DocumentManager> m_documents;
    std::unique_ptr<doc::Document> m_document;
    std::shared_ptr<doc::AssemblyDocument> m_assembly;  ///< when the document is one
    std::string m_path;
    std::unique_ptr<doc::Document> m_pending;  ///< to be the document next
    std::shared_ptr<doc::AssemblyDocument> m_pendingAssembly;
    std::string m_pendingPath;
    ai::ToolRegistry m_tools;  ///< last: its tools refer to the rest
};

/// What the MCP server tells a client's model about Horizon CAD.
std::string serverInstructions();

}  // namespace hz::mcp
