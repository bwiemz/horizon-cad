#pragma once

#include <functional>

#include "horizon/ai/ToolRegistry.h"

namespace hz::doc {
class AssemblyDocument;
class Document;
class DocumentManager;
}  // namespace hz::doc

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

/// What the host is told when an assembly tool changes the assembly.
struct AssemblyToolHooks {
    /// Its components, their placements, its mates or its patterns changed
    /// (and the mates were solved): the view is drawn again.
    std::function<void()> assemblyChanged;
};

/// Add the assembly tools, acting on @p assembly, to @p registry: what it
/// holds, inserting, placing, mating and patterning its components, the
/// interference check and the bill of materials. Every change is one step on
/// @p backing's undo stack (the assembly tab's document), made as the
/// Assembly menu makes it: its mates are solved after it, and a change they
/// cannot hold is not made. @p documents reads the components' parts. All
/// three must outlive the registry's use.
void registerAssemblyTools(ToolRegistry& registry, doc::AssemblyDocument& assembly,
                           doc::Document& backing, doc::DocumentManager& documents,
                           AssemblyToolHooks hooks = {});

}  // namespace hz::ai
