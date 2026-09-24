#include <gtest/gtest.h>

#include <memory>

#include "horizon/document/Commands.h"
#include "horizon/document/Document.h"
#include "horizon/document/UndoStack.h"
#include "horizon/drafting/DraftLine.h"

using hz::doc::AddEntityCommand;
using hz::doc::Command;
using hz::doc::Document;
using hz::doc::UndoStack;
using hz::draft::DraftLine;
using hz::math::Vec2;

namespace {

/// Adds `delta` to a counter; undo subtracts it.
class AddCommand : public Command {
public:
    AddCommand(int& target, int delta) : m_target(target), m_delta(delta) {}
    void execute() override { m_target += m_delta; }
    void undo() override { m_target -= m_delta; }
    std::string description() const override { return "Add"; }

private:
    int& m_target;
    int m_delta;
};

std::unique_ptr<Command> add(int& target, int delta) {
    return std::make_unique<AddCommand>(target, delta);
}

}  // namespace

// ---------------------------------------------------------------------------
// Clean state
// ---------------------------------------------------------------------------

TEST(UndoStackTest, FreshStackIsClean) {
    UndoStack stack;
    EXPECT_TRUE(stack.isClean());
}

TEST(UndoStackTest, PushLeavesTheCleanState) {
    int value = 0;
    UndoStack stack;
    stack.push(add(value, 1));
    EXPECT_FALSE(stack.isClean());
}

TEST(UndoStackTest, UndoBackToTheSavedStateIsClean) {
    int value = 0;
    UndoStack stack;
    stack.push(add(value, 1));
    stack.setClean();  // saved with one command applied
    stack.push(add(value, 2));
    EXPECT_FALSE(stack.isClean());

    stack.undo();
    EXPECT_TRUE(stack.isClean());
    EXPECT_EQ(value, 1);

    stack.undo();  // before the save
    EXPECT_FALSE(stack.isClean());

    stack.redo();
    EXPECT_TRUE(stack.isClean());
    stack.redo();
    EXPECT_FALSE(stack.isClean());
}

TEST(UndoStackTest, BranchingAwayFromTheSavedStateMakesItUnreachable) {
    int value = 0;
    UndoStack stack;
    stack.push(add(value, 1));
    stack.push(add(value, 2));
    stack.setClean();  // saved at depth 2

    stack.undo();               // depth 1; the saved state is in the redo history
    stack.push(add(value, 5));  // discards it

    // Back at depth 2, but with different content than what was saved.
    EXPECT_FALSE(stack.isClean());
    stack.undo();
    EXPECT_FALSE(stack.isClean());
    stack.undo();
    EXPECT_FALSE(stack.isClean());
}

TEST(UndoStackTest, PushAfterUndoingPastAShallowerCleanStateKeepsItReachable) {
    int value = 0;
    UndoStack stack;
    stack.push(add(value, 1));
    stack.setClean();  // depth 1
    stack.push(add(value, 2));
    stack.push(add(value, 3));
    stack.undo();               // depth 2; clean state is below, not in redo
    stack.push(add(value, 4));  // depth 3

    stack.undo();
    stack.undo();
    EXPECT_TRUE(stack.isClean());
    EXPECT_EQ(value, 1);
}

TEST(UndoStackTest, SetCleanAfterBranchingRestoresTracking) {
    int value = 0;
    UndoStack stack;
    stack.push(add(value, 1));
    stack.setClean();
    stack.undo();
    stack.push(add(value, 2));
    EXPECT_FALSE(stack.isClean());

    stack.setClean();
    EXPECT_TRUE(stack.isClean());
}

TEST(UndoStackTest, ClearMakesTheEmptyStackClean) {
    int value = 0;
    UndoStack stack;
    stack.push(add(value, 1));
    stack.clear();
    EXPECT_TRUE(stack.isClean());
    EXPECT_FALSE(stack.canUndo());
    EXPECT_FALSE(stack.canRedo());
}

// ---------------------------------------------------------------------------
// Revision and change notification
// ---------------------------------------------------------------------------

TEST(UndoStackTest, RevisionAdvancesOnEveryChange) {
    int value = 0;
    UndoStack stack;
    const auto r0 = stack.revision();
    stack.push(add(value, 1));
    const auto r1 = stack.revision();
    stack.undo();
    const auto r2 = stack.revision();
    stack.redo();
    const auto r3 = stack.revision();
    EXPECT_LT(r0, r1);
    EXPECT_LT(r1, r2);  // undo is a change even though it restores content
    EXPECT_LT(r2, r3);

    stack.setClean();  // saving changes nothing in the document
    EXPECT_EQ(stack.revision(), r3);

    stack.undo();  // nothing happens past the ends...
    stack.undo();
    const auto r4 = stack.revision();
    stack.undo();
    EXPECT_EQ(stack.revision(), r4);  // ...and a no-op does not count
}

TEST(UndoStackTest, CallbackFiresForEveryChange) {
    int value = 0;
    int calls = 0;
    UndoStack stack;
    stack.setChangeCallback([&] { ++calls; });
    stack.push(add(value, 1));
    stack.undo();
    stack.redo();
    stack.setClean();
    stack.clear();
    EXPECT_EQ(calls, 5);

    stack.undo();  // empty: nothing to report
    EXPECT_EQ(calls, 5);
}

// ---------------------------------------------------------------------------
// Document modified state
// ---------------------------------------------------------------------------

TEST(DocumentDirtyTest, CommandsMarkTheDocumentModified) {
    Document doc;
    EXPECT_FALSE(doc.isDirty());

    auto line = std::make_shared<DraftLine>(Vec2(0, 0), Vec2(1, 0));
    doc.undoStack().push(std::make_unique<AddEntityCommand>(doc.draftDocument(), line));
    EXPECT_TRUE(doc.isDirty());

    doc.undoStack().undo();
    EXPECT_FALSE(doc.isDirty()) << "undoing back to the unmodified state is not a change";
}

TEST(DocumentDirtyTest, SavingMarksTheCurrentStateClean) {
    Document doc;
    auto a = std::make_shared<DraftLine>(Vec2(0, 0), Vec2(1, 0));
    auto b = std::make_shared<DraftLine>(Vec2(0, 1), Vec2(1, 1));
    doc.undoStack().push(std::make_unique<AddEntityCommand>(doc.draftDocument(), a));
    doc.setDirty(false);  // what a successful save does
    EXPECT_FALSE(doc.isDirty());

    doc.undoStack().push(std::make_unique<AddEntityCommand>(doc.draftDocument(), b));
    EXPECT_TRUE(doc.isDirty());
    doc.undoStack().undo();
    EXPECT_FALSE(doc.isDirty());
    doc.undoStack().undo();  // past the save
    EXPECT_TRUE(doc.isDirty());
}

TEST(DocumentDirtyTest, ChangesOutsideTheUndoStackStayMarkedUntilSaved) {
    Document doc;
    doc.setDirty(true);  // e.g. a feature added without a command
    EXPECT_TRUE(doc.isDirty());

    auto line = std::make_shared<DraftLine>(Vec2(0, 0), Vec2(1, 0));
    doc.undoStack().push(std::make_unique<AddEntityCommand>(doc.draftDocument(), line));
    doc.undoStack().undo();
    EXPECT_TRUE(doc.isDirty()) << "undo cannot revert a change it never recorded";

    doc.setDirty(false);
    EXPECT_FALSE(doc.isDirty());
}

TEST(DocumentDirtyTest, ClearResetsToAnUnmodifiedDocument) {
    Document doc;
    auto line = std::make_shared<DraftLine>(Vec2(0, 0), Vec2(1, 0));
    doc.undoStack().push(std::make_unique<AddEntityCommand>(doc.draftDocument(), line));
    doc.setDirty(true);
    doc.clear();
    EXPECT_FALSE(doc.isDirty());
}

TEST(DocumentDirtyTest, ChangeCallbackCoversCommandsAndExplicitMarks) {
    Document doc;
    int calls = 0;
    doc.setChangeCallback([&] { ++calls; });

    auto line = std::make_shared<DraftLine>(Vec2(0, 0), Vec2(1, 0));
    doc.undoStack().push(std::make_unique<AddEntityCommand>(doc.draftDocument(), line));
    EXPECT_EQ(calls, 1);
    doc.setDirty(true);  // a change the undo stack never saw
    EXPECT_EQ(calls, 2);
    doc.setDirty(false);  // saved
    EXPECT_EQ(calls, 3);

    doc.setChangeCallback(nullptr);
    doc.undoStack().undo();
    doc.setDirty(true);
    EXPECT_EQ(calls, 3) << "a cleared callback is not called";
}
