#include "horizon/document/UndoStack.h"

namespace hz::doc {

UndoStack::UndoStack() = default;
UndoStack::~UndoStack() = default;

void UndoStack::push(std::unique_ptr<Command> cmd) {
    cmd->execute();
    // A clean state deeper than the current depth lives in the redo history,
    // which this push discards.
    if (m_cleanIndex != kCleanUnreachable && m_cleanIndex > m_undoStack.size()) {
        m_cleanIndex = kCleanUnreachable;
    }
    m_undoStack.push_back(std::move(cmd));
    m_redoStack.clear();
    notifyChanged();
}

void UndoStack::undo() {
    if (m_undoStack.empty()) return;
    auto cmd = std::move(m_undoStack.back());
    m_undoStack.pop_back();
    cmd->undo();
    m_redoStack.push_back(std::move(cmd));
    notifyChanged();
}

void UndoStack::redo() {
    if (m_redoStack.empty()) return;
    auto cmd = std::move(m_redoStack.back());
    m_redoStack.pop_back();
    cmd->execute();
    m_undoStack.push_back(std::move(cmd));
    notifyChanged();
}

bool UndoStack::canUndo() const {
    return !m_undoStack.empty();
}
bool UndoStack::canRedo() const {
    return !m_redoStack.empty();
}

void UndoStack::clear() {
    m_undoStack.clear();
    m_redoStack.clear();
    m_cleanIndex = 0;
    notifyChanged();
}

void UndoStack::setClean() {
    m_cleanIndex = m_undoStack.size();
    if (m_onChange) m_onChange();
}

bool UndoStack::isClean() const {
    return m_cleanIndex == m_undoStack.size();
}

void UndoStack::setChangeCallback(std::function<void()> callback) {
    m_onChange = std::move(callback);
}

void UndoStack::notifyChanged() {
    ++m_revision;
    if (m_onChange) m_onChange();
}

}  // namespace hz::doc
