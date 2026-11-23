#pragma once

// The window's side of the AI assistant's drawing sheet tools: what a sheet
// shows, read from the drawing workbench, and its changes made through it.

#include <memory>
#include <optional>
#include <string>

#include "horizon/ai/CadTools.h"
#include "horizon/modeling/Sheet.h"

namespace hz::doc {
class Document;
}

namespace hz::ui {

class DrawingWorkbench;

/// The sheet tools' hooks for the sheet @p sheet, which @p drawings draws.
/// They do nothing once either is gone.
ai::SheetToolHooks sheetToolHooks(DrawingWorkbench& drawings, std::weak_ptr<doc::Document> sheet);

/// A paper size as the assistant's tools name it ("a3", "ansi_a"), and back.
std::optional<model::PaperSize> paperSizeNamed(const std::string& name);
std::string paperNameOf(model::PaperSize size);

}  // namespace hz::ui
