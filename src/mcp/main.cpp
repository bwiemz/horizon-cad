// horizon-mcp: Horizon CAD's tools served over the Model Context Protocol on
// standard input and output, for MCP clients (Claude Desktop, Claude Code,
// Cursor, and others) to build and change parts, assemblies and drawings with.
//
//   horizon-mcp [--root DIR] [--open FILE] [--drawing]
//
// Standard output carries the protocol only; anything else goes to standard
// error.

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <filesystem>
#include <iostream>
#include <string>

#include "horizon/Version.h"
#include "horizon/ai/McpServer.h"
#include "horizon/mcp/McpSession.h"

#ifdef _WIN32
#include <fcntl.h>
#include <io.h>
#endif

namespace {

/// The user's home folder, or empty.
std::filesystem::path homeFolder() {
#ifdef _WIN32
    const char* home = std::getenv("USERPROFILE");
#else
    const char* home = std::getenv("HOME");
#endif
    return home != nullptr && *home != '\0' ? std::filesystem::path(home) : std::filesystem::path();
}

void usage(std::ostream& out) {
    out << "Usage: horizon-mcp [--root DIR] [--open FILE] [--drawing]\n\n"
           "Serves Horizon CAD's CAD tools over the Model Context Protocol (stdio).\n"
           "  --root DIR    the folder files may be opened, saved and exported in\n"
           "                (default: Documents/Horizon CAD MCP in the home folder)\n"
           "  --open FILE   start with FILE (.hzpart, .hcad, .hzasm or .dxf)\n"
           "  --drawing     start with an empty 2D drawing (default: an empty part)\n"
           "  --version     print the version\n"
           "  --help        print this\n";
}

}  // namespace

int main(int argc, char** argv) {
#ifdef _WIN32
    // Lines end in \n alone: no \r added on the way out, none expected in.
    _setmode(_fileno(stdout), _O_BINARY);
    _setmode(_fileno(stdin), _O_BINARY);
#endif
    std::string open;
    std::string root;
    bool part = true;
    for (int i = 1; i < argc; ++i) {
        const std::string arg = argv[i];
        if (arg == "--help" || arg == "-h") {
            usage(std::cout);
            return 0;
        }
        if (arg == "--version") {
            std::cout << "horizon-mcp " << hz::version::kString << "\n";
            return 0;
        }
        if (arg == "--drawing") {
            part = false;
        } else if (arg == "--open" && i + 1 < argc) {
            open = argv[++i];
        } else if (arg == "--root" && i + 1 < argc) {
            root = argv[++i];
        } else {
            std::cerr << "horizon-mcp: unknown argument " << arg << "\n";
            usage(std::cerr);
            return 2;
        }
    }

    // The folder a model may use files in. Not the folder it happens to be
    // started in (an MCP client may start it in / or the home folder): one
    // of its own, unless --root names another.
    namespace fs = std::filesystem;
    std::error_code ec;
    const fs::path home = homeFolder();
    fs::path folder = root.empty() ? fs::path() : fs::path(root);
    if (folder.empty()) {
        if (home.empty()) {
            std::cerr << "horizon-mcp: no home folder: give the folder to use with --root\n";
            return 2;
        }
        folder = home / "Documents" / "Horizon CAD MCP";
        fs::create_directories(folder, ec);
    }
    if (!fs::is_directory(folder, ec)) {
        std::cerr << "horizon-mcp: " << folder.string() << " is not a folder\n";
        return 2;
    }
    const fs::path resolved = fs::weakly_canonical(fs::absolute(folder, ec), ec);
    if (resolved == resolved.root_path() || (!home.empty() && fs::equivalent(resolved, home, ec))) {
        std::cerr << "horizon-mcp: " << resolved.string()
                  << " is too wide a folder to let a model write in: give a project folder "
                     "with --root\n";
        return 2;
    }
    hz::mcp::McpSession session(part, folder);
    if (!open.empty()) {
        std::string error;
        if (!session.open(open, &error)) {
            std::cerr << "horizon-mcp: could not open " << open << ": " << error << "\n";
            return 1;
        }
    }
    hz::ai::McpServer server(session.tools(), "horizon-cad", hz::version::kString);
    server.setInstructions(hz::mcp::serverInstructions());
    // A document a tool opened or started takes over once its call is done:
    // before the next message, or the next call in a batch.
    server.setAfterEach([&session] { session.applyPending(); });
    std::cerr << "horizon-mcp " << hz::version::kString << ": serving "
              << (open.empty() ? std::string(part ? "a new part" : "a new drawing") : open)
              << " on stdio, files in " << session.root().string() << "\n";
    server.serve(std::cin, std::cout);
    return 0;
}
