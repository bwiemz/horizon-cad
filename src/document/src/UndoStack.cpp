#include "horizon/document/UndoStack.h"

#include <cstddef>

namespace hz::doc {

UndoStack::UndoStack() = default;
UndoStack::~UndoStack() = default;

void UndoStack::Step::execute() const {
    command->execute();
    if (followUp) followUp->execute();
}

void UndoStack::Step::undo() const {
    if (followUp) followUp->undo();
    command->undo();
}

void UndoStack::push(std::unique_ptr<Command> cmd) {
    cmd->execute();
    std::unique_ptr<Command> followUp = m_followUp ? m_followUp() : nullptr;
    if (followUp) followUp->execute();
    // A clean state deeper than the current depth lives in the redo history,
    // which this push discards.
    if (m_cleanIndex != kCleanUnreachable && m_cleanIndex > m_undoStack.size()) {
        m_cleanIndex = kCleanUnreachable;
    }
    m_undoStack.push_back({std::move(cmd), std::move(followUp)});
    m_redoStack.clear();
    trim();
    notifyChanged();
}

void UndoStack::setLimit(std::size_t steps) {
    m_limit = steps;
    trim();
}

void UndoStack::trim() {
    if (m_limit == 0 || m_undoStack.size() <= m_limit) return;
    const std::size_t dropped = m_undoStack.size() - m_limit;
    m_undoStack.erase(m_undoStack.begin(),
                      m_undoStack.begin() + static_cast<std::ptrdiff_t>(dropped));
    // The saved state, as deep as it was less what went: one among what went
    // cannot be undone back to.
    if (m_cleanIndex != kCleanUnreachable) {
        m_cleanIndex = m_cleanIndex >= dropped ? m_cleanIndex - dropped : kCleanUnreachable;
    }
}

void UndoStack::undo() {
    if (m_undoStack.empty()) return;
    Step step = std::move(m_undoStack.back());
    m_undoStack.pop_back();
    step.undo();
    m_redoStack.push_back(std::move(step));
    notifyChanged();
}

void UndoStack::redo() {
    if (m_redoStack.empty()) return;
    Step step = std::move(m_redoStack.back());
    m_redoStack.pop_back();
    step.execute();
    m_undoStack.push_back(std::move(step));
    notifyChanged();
}

bool UndoStack::withdraw(const Command* command) {
    if (m_undoStack.empty() || m_undoStack.back().command.get() != command) return false;
    const Step step = std::move(m_undoStack.back());
    m_undoStack.pop_back();
    step.undo();
    // Saved with the command in it: that state is gone for good.
    if (m_cleanIndex != kCleanUnreachable && m_cleanIndex > m_undoStack.size()) {
        m_cleanIndex = kCleanUnreachable;
    }
    notifyChanged();
    return true;
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

void UndoStack::setFollowUp(std::function<std::unique_ptr<Command>()> followUp) {
    m_followUp = std::move(followUp);
}

void UndoStack::notifyChanged() {
    ++m_revision;
    if (m_onChange) m_onChange();
}

}  // namespace hz::doc
