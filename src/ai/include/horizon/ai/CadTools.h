#pragma once

#include <functional>
#include <string>
#include <utility>
#include <vector>

#include "horizon/ai/ToolRegistry.h"

namespace hz::doc {
class AssemblyDocument;
class Document;
class DocumentManager;
}  // namespace hz::doc

namespace hz::ai {

/// A drawing sheet of a part or an assembly, as make_drawing asks the host for
/// one: the standard views (front, top, right, isometric) laid out over the
/// title block, as the menu's New Drawing from Part makes them.
struct DrawingRequest {
    std::string source;  ///< the part's or assembly's file, saved as it is shown
    /// a0, a1, a2, a3, a4, ansi_a (Letter), ansi_b (Tabloid), ansi_c, ansi_d.
    std::string paper = "a3";
    bool portrait = false;
    double scale = 0.0;  ///< sheet mm per model mm; 0: the largest standard one that fits
    // The title block; an empty title is the file's name.
    std::string title;
    std::string partNumber;
    std::string revision;
    std::string drawnBy;
    std::string date;
    std::string material;
    std::string company;
};

/// What the host made of a DrawingRequest.
struct DrawingMade {
    std::string error;               ///< why there is no sheet; empty when there is
    std::string tab;                 ///< the title of the sheet's tab
    std::string scale;               ///< as the title block states it: "1:2"
    std::vector<std::string> views;  ///< "front", "top", "right", "isometric"
};

/// Makes the sheet, in a tab of its own that is not made current (a tab
/// switch would stop the request), unsaved: the user looks at it and saves
/// it.
using MakeDrawing = std::function<DrawingMade(const DrawingRequest&)>;

/// What the host (the window, the MCP server) is told when a tool changes the
/// document, so it can show the change.
struct CadToolHooks {
    /// The part changed: its features, sketches, variables or solid. The
    /// model has already been rebuilt.
    std::function<void()> modelChanged;
    /// The 2D drawing or its layers changed.
    std::function<void()> drawingChanged;
    /// Make a drawing sheet of the part (make_drawing); unset: no such tool.
    MakeDrawing makeDrawing;
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
    /// A change is about to be made: a component being dragged in the view
    /// is put back first, or the drag's release would record over it.
    std::function<void()> beforeChange;
    /// The folder an unsaved assembly takes its parts from (horizon-mcp: its
    /// root). Empty: none, so it must be saved beside them first. A saved
    /// one takes them from its own folder.
    std::string unsavedFolder;
    /// Make a drawing sheet of the assembly (make_drawing); unset: no such
    /// tool.
    MakeDrawing makeDrawing;
};

/// A change to a drawing sheet, named rather than clicked: what the sheet
/// tools ask the host for. Each is the Drawing menu command's change, one
/// undo step.
struct SheetChange {
    enum class Kind {
        TitleBlock,    ///< fields, paper, orientation
        Scale,         ///< scale
        MoveView,      ///< view, by (or to)
        RemoveView,    ///< view
        ViewSwitches,  ///< view, hidden, tangent, centreLines
        AddDimension,  ///< view, edge
        RemoveDimension,
    };
    Kind kind = Kind::TitleBlock;
    /// The title block fields given, by the tools' names (title,
    /// part_number, revision, drawn_by, date, material, company,
    /// sheet_number); the others stay as they are.
    std::vector<std::pair<std::string, std::string>> fields;
    std::string paper;        ///< as make_drawing names it ("a3"); "" keeps it
    std::string orientation;  ///< "landscape", "portrait"; "" keeps it
    double scale = 0.0;       ///< sheet mm per model mm; 0: the largest that fits
    int view = -1;            ///< as get_sheet_info numbers them
    double x = 0.0;           ///< MoveView: by (sheet mm), or to (its lower-left corner)
    double y = 0.0;
    bool to = false;
    int hidden = -1;  ///< ViewSwitches: 1 shown, 0 left out, -1 as it is
    int tangent = -1;
    int centreLines = -1;
    std::string edge;  ///< as list_view_edges names it
};

/// What came of a SheetChange.
struct SheetChanged {
    std::string error;  ///< why it was not made; empty when it was
    std::string note;   ///< what the model should know of it ("a view had no room")
    bool step = false;  ///< it added an undo step (none for a change that changed nothing)
};

/// The host's side of the sheet tools, for the sheet one tab shows.
struct SheetToolHooks {
    /// The sheet as the model reads it: paper, scale, title block, views.
    std::function<nlohmann::json()> info;
    /// The model edges view @p view shows, named for dimensions; null, with
    /// why in @p error, when there is no such view.
    std::function<nlohmann::json(int view, std::string* error)> edges;
    std::function<SheetChanged(const SheetChange&)> change;
};

/// Add the drawing sheet tools to @p registry: what the sheet shows, its
/// title block, paper and scale, its views (moved, removed, what they show)
/// and their dimensions. Every change is one undo step, as the Drawing
/// menu's are. Nothing is added unless all three hooks are set.
void registerSheetTools(ToolRegistry& registry, SheetToolHooks hooks);

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
