// The viewport does per-frame work only when something changed (Phase 113):
// the constraint analysis behind the DOF colours ran the solver on every
// frame, and a frame is drawn on every mouse move.

#include <gtest/gtest.h>

#include <memory>

#include "horizon/document/Commands.h"
#include "horizon/document/Document.h"
#include "horizon/document/UndoStack.h"
#include "horizon/drafting/DraftLine.h"
#include "horizon/ui/ViewportRenderer.h"

TEST(RenderEfficiencyTest, TheConstraintAnalysisRunsWhenTheDocumentChangesNotEveryFrame) {
    hz::doc::Document doc;
    hz::ui::ViewportRenderer renderer;

    for (int frame = 0; frame < 5; ++frame) renderer.recomputeDOF(&doc);
    EXPECT_EQ(renderer.dofComputations(), 1u) << "five frames, one analysis";

    auto line = std::make_shared<hz::draft::DraftLine>(hz::math::Vec2(0, 0), hz::math::Vec2(1, 0));
    doc.undoStack().push(std::make_unique<hz::doc::AddEntityCommand>(doc.draftDocument(), line));
    renderer.recomputeDOF(&doc);
    renderer.recomputeDOF(&doc);
    EXPECT_EQ(renderer.dofComputations(), 2u) << "an edit, one more";

    doc.undoStack().undo();
    renderer.recomputeDOF(&doc);
    EXPECT_EQ(renderer.dofComputations(), 3u) << "an undo is a change too";

    hz::doc::Document other;
    renderer.recomputeDOF(&other);
    EXPECT_EQ(renderer.dofComputations(), 4u) << "and so is another document";

    renderer.invalidateDOF();
    renderer.recomputeDOF(&other);
    EXPECT_EQ(renderer.dofComputations(), 5u) << "a document set anew is analysed anew";
}
