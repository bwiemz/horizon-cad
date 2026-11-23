// horizon-mcp: Horizon CAD's tools served over the Model Context Protocol on
// standard input and output, for MCP clients (Claude Desktop, Claude Code,
// Cursor, and others) to build and change parts and drawings with.
//
//   horizon-mcp [--open FILE] [--drawing]
//
// Standard output carries the protocol only; anything else goes to standard
// error.

#include <cstdio>
#include <cstring>
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

void usage(std::ostream& out) {
    out << "Usage: horizon-mcp [--open FILE] [--drawing]\n\n"
           "Serves Horizon CAD's CAD tools over the Model Context Protocol (stdio).\n"
           "  --open FILE   start with FILE (.hzpart, .hcad or .dxf)\n"
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
        } else {
            std::cerr << "horizon-mcp: unknown argument " << arg << "\n";
            usage(std::cerr);
            return 2;
        }
    }

    hz::mcp::McpSession session(part);
    if (!open.empty()) {
        std::string error;
        if (!session.open(open, &error)) {
            std::cerr << "horizon-mcp: could not open " << open << ": " << error << "\n";
            return 1;
        }
    }
    hz::ai::McpServer server(session.tools(), "horizon-cad", hz::version::kString);
    server.setInstructions(hz::mcp::serverInstructions());
    std::cerr << "horizon-mcp " << hz::version::kString << ": serving "
              << (open.empty() ? std::string(part ? "a new part" : "a new drawing") : open)
              << " on stdio\n";

    std::string line;
    while (std::getline(std::cin, line)) {
        if (const auto reply = server.handleLine(line)) {
            std::cout << *reply << '\n';
            std::cout.flush();
        }
        // A document a tool opened or started takes over now that its call
        // is done.
        session.applyPending();
    }
    return 0;
}
