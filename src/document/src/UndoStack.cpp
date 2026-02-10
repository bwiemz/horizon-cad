#include "horizon/document/UndoStack.h"

namespace hz::doc {

UndoStack::UndoStack() = default;
UndoStack::~UndoStack() = default;

void UndoStack::push(std::unique_ptr<Command> cmd) {
    cmd->execute();
    m_undoStack.push_back(std::move(cmd));
    m_redoStack.clear();
}

void UndoStack::undo() {
    if (m_undoStack.empty()) return;
    auto cmd = std::move(m_undoStack.back());
    m_undoStack.pop_back();
    cmd->undo();
    m_redoStack.push_back(std::move(cmd));
}

void UndoStack::redo() {
    if (m_redoStack.empty()) return;
    auto cmd = std::move(m_redoStack.back());
    m_redoStack.pop_back();
    cmd->execute();
    m_undoStack.push_back(std::move(cmd));
}

bool UndoStack::canUndo() const { return !m_undoStack.empty(); }
bool UndoStack::canRedo() const { return !m_redoStack.empty(); }

void UndoStack::clear() {
    m_undoStack.clear();
    m_redoStack.clear();
}

}  // namespace hz::doc
