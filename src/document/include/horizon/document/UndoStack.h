#pragma once

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
class UndoStack {
public:
    UndoStack();
    ~UndoStack();

    /// Execute a command and push it onto the undo stack. Clears the redo stack.
    void push(std::unique_ptr<Command> cmd);

    void undo();
    void redo();

    bool canUndo() const;
    bool canRedo() const;

    void clear();

private:
    std::vector<std::unique_ptr<Command>> m_undoStack;
    std::vector<std::unique_ptr<Command>> m_redoStack;
};

}  // namespace hz::doc
