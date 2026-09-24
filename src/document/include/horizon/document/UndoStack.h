#pragma once

#include <cstddef>
#include <cstdint>
#include <functional>
#include <memory>
#include <string>
#include <vector>

namespace hz::doc {

/// Abstract base class for undoable commands.
class Command {
public:
    virtual ~Command() = default;
    virtual void execute() = 0;
    virtual void undo() = 0;
    virtual std::string description() const = 0;
};

/// Manages a stack of undoable commands.
///
/// The stack also remembers which state was last saved (the *clean* state), so
/// the document's modified flag follows undo and redo: undoing back to the
/// saved state clears it, and redoing away from it sets it again.
class UndoStack {
public:
    UndoStack();
    ~UndoStack();

    /// Execute a command and push it onto the undo stack. Clears the redo stack.
    void push(std::unique_ptr<Command> cmd);

    void undo();
    void redo();
    /// Undo @p command and forget it, when it is the newest step: a step
    /// refused once it was taken (a feature whose build failed), which
    /// leaves the history as it was before it, with nothing to redo. A saved
    /// state that had the command in it can no longer be reached. Returns
    /// false, changing nothing, for any other command.
    bool withdraw(const Command* command);

    bool canUndo() const;
    bool canRedo() const;

    /// Drop all history. The empty stack is the clean state.
    void clear();

    /// Record the current state as the saved one.
    void setClean();

    /// True while the stack is at the state last recorded by setClean(), or
    /// at the initial empty state if setClean() has not been called. Once a
    /// push discards the redo history that held the clean state, it can never
    /// be reached again and this stays false until the next setClean().
    bool isClean() const;

    /// Increases on every push, undo, redo and clear. Observers compare it
    /// with a value they stored to learn whether anything changed since.
    std::uint64_t revision() const { return m_revision; }

    /// Called after every push, undo, redo, clear and setClean.
    void setChangeCallback(std::function<void()> callback);

private:
    void notifyChanged();

    static constexpr std::size_t kCleanUnreachable = static_cast<std::size_t>(-1);

    std::vector<std::unique_ptr<Command>> m_undoStack;
    std::vector<std::unique_ptr<Command>> m_redoStack;
    /// Undo depth of the clean state, or kCleanUnreachable.
    std::size_t m_cleanIndex = 0;
    std::uint64_t m_revision = 0;
    std::function<void()> m_onChange;
};

}  // namespace hz::doc
