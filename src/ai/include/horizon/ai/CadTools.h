#pragma once

#include <functional>

#include "horizon/ai/ToolRegistry.h"

namespace hz::doc {
class Document;
}

namespace hz::ai {

/// What the host (the window, the MCP server) is told when a tool changes the
/// document, so it can show the change.
struct CadToolHooks {
    /// The part changed: its features, sketches, variables or solid. The
    /// model has already been rebuilt.
    std::function<void()> modelChanged;
    /// The 2D drawing or its layers changed.
    std::function<void()> drawingChanged;
};

/// Add the CAD tools, acting on @p document, to @p registry: queries of the
/// part and the drawing, sketches and features, variables, and 2D drafting.
/// Lengths are millimetres and angles degrees. Every change is one step on
/// the document's undo stack; a feature that fails to build is withdrawn and
/// its reason returned. @p document must outlive the registry's use.
void registerCadTools(ToolRegistry& registry, doc::Document& document, CadToolHooks hooks = {});

}  // namespace hz::ai
