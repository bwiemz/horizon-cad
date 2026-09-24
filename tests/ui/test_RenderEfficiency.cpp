// The viewport does per-frame work only when something changed (Phase 113):
// the constraint analysis behind the DOF colours ran the solver on every
// frame, and a frame is drawn on every mouse move. And the 2D view scales
// (Phase 136): what it draws is built once, kept while the drawing does not
// change, batched by pen and drawn only where it is in view, and the text
// overlay is painted only when what it shows changes.

#include <gtest/gtest.h>

#include <chrono>
#include <cstdint>
#include <memory>
#include <set>
#include <utility>
#include <vector>

#include "../TimeLimits.h"
#include "horizon/document/Commands.h"
#include "horizon/document/Document.h"
#include "horizon/document/Sketch.h"
#include "horizon/document/UndoStack.h"
#include "horizon/drafting/DraftArc.h"
#include "horizon/drafting/DraftCircle.h"
#include "horizon/drafting/DraftLine.h"
#include "horizon/drafting/DraftText.h"
#include "horizon/drafting/Layer.h"
#include "horizon/math/Constants.h"
#include "horizon/render/Camera.h"
#include "horizon/render/SelectionManager.h"
#include "horizon/ui/DrawingCache.h"
#include "horizon/ui/ViewportRenderer.h"

using hz::math::Vec2;
using hz::math::Vec3;
using hz::ui::DrawingCache;

namespace {

std::shared_ptr<hz::draft::DraftLine> lineFrom(double x0, double y0, double x1, double y1) {
    return std::make_shared<hz::draft::DraftLine>(Vec2(x0, y0), Vec2(x1, y1));
}

void add(hz::doc::Document& doc, std::shared_ptr<hz::draft::DraftEntity> entity) {
    doc.undoStack().push(
        std::make_unique<hz::doc::AddEntityCommand>(doc.draftDocument(), std::move(entity)));
}

/// Looking straight down at the square @p size across from (@p x0, @p y0).
hz::render::Camera lookingAt(double x0, double y0, double size) {
    hz::render::Camera camera;
    camera.setOrthographic(size, size, -1000.0, 1000.0);
    const Vec3 middle(x0 + size / 2.0, y0 + size / 2.0, 0.0);
    camera.lookAt(middle + Vec3(0, 0, 100), middle, Vec3(0, 1, 0));
    return camera;
}

size_t vertexCount(const DrawingCache& cache) {
    size_t n = 0;
    for (const auto& batch : cache.batches()) n += batch.vertices.size() / 4;
    return n;
}

/// The vertices a frame through @p camera draws, and where they are.
std::pair<size_t, std::set<std::pair<float, float>>> drawnThrough(
    const DrawingCache& cache, const hz::render::Camera& camera) {
    const auto visible = cache.visibleChunks(camera.viewProjectionMatrix());
    size_t count = 0;
    std::set<std::pair<float, float>> points;
    for (const auto& batch : cache.batches()) {
        for (const auto& range : DrawingCache::visibleRanges(batch, visible)) {
            count += range.count;
            for (std::uint32_t v = range.first; v < range.first + range.count; ++v) {
                points.insert({batch.vertices[v * 4], batch.vertices[v * 4 + 1]});
            }
        }
    }
    return {count, points};
}

}  // namespace

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

// What the view draws is built once and kept while nothing it is built from
// changes. It was built again, every vertex of every entity, on every frame.
TEST(RenderEfficiencyTest, TheDrawingIsBuiltWhenItChangesNotEveryFrame) {
    hz::doc::Document doc;
    hz::render::SelectionManager selection;
    const hz::cstr::DOFAnalysis dof;
    std::uint64_t dofRevision = 0;
    DrawingCache cache;
    auto line = lineFrom(0, 0, 10, 0);
    add(doc, line);
    for (int frame = 0; frame < 5; ++frame) cache.update(doc, selection, dof, dofRevision);
    EXPECT_EQ(cache.builds(), 1u) << "five frames, one build";

    // Each change to what is drawn, one build more, and only one.
    const auto rebuilt = [&](const char* what) {
        const auto before = cache.builds();
        cache.update(doc, selection, dof, dofRevision);
        cache.update(doc, selection, dof, dofRevision);
        EXPECT_EQ(cache.builds(), before + 1) << what;
    };
    line->setEnd(Vec2(12, 0));
    doc.draftDocument().updateEntityBounds(line->id());
    rebuilt("a line moved in place, as a tool drags it");
    add(doc, lineFrom(0, 5, 10, 5));
    rebuilt("an entity added");
    doc.undoStack().undo();
    rebuilt("and undone");
    selection.select(line->id());
    rebuilt("a line selected: drawn in the selection's colour");
    selection.select(line->id());
    cache.update(doc, selection, dof, dofRevision);
    EXPECT_EQ(cache.builds(), 5u) << "selected again: no change";
    doc.layerManager().getLayer("0")->color = 0xFF00FF00;
    rebuilt("a layer's colour, changed through its pointer");
    auto style = doc.draftDocument().dimensionStyle();
    style.textHeight = 5.0;
    doc.draftDocument().setDimensionStyle(style);
    rebuilt("the dimension style");
    ++dofRevision;
    rebuilt("the constraint analysis behind the DOF colours");
    auto sketch = std::make_shared<hz::doc::Sketch>();
    doc.addSketch(sketch);
    doc.editSketch(sketch);
    rebuilt("a sketch edited: another drawing shown");

    hz::doc::Document other;
    const auto before = cache.builds();
    cache.update(other, selection, dof, dofRevision);
    EXPECT_EQ(cache.builds(), before + 1) << "another document";
}

// Circles and arcs go into the batch of their pen, with the lines: a draw for
// each pen, where every circle and arc was a draw of its own.
TEST(RenderEfficiencyTest, CirclesAndArcsGoInTheBatchOfTheirPen) {
    hz::doc::Document doc;
    hz::render::SelectionManager selection;
    const hz::cstr::DOFAnalysis dof;
    add(doc, lineFrom(0, 0, 10, 0));
    auto circle = std::make_shared<hz::draft::DraftCircle>(Vec2(20, 0), 5.0);
    add(doc, circle);
    add(doc, std::make_shared<hz::draft::DraftArc>(Vec2(40, 0), 5.0, 0.0, hz::math::kPi / 2));
    auto red = lineFrom(0, 10, 10, 10);
    red->setColor(0xFFFF0000);
    add(doc, red);
    add(doc, std::make_shared<hz::draft::DraftText>(Vec2(0, 20), "a note"));

    DrawingCache cache;
    cache.build(doc, selection, dof);
    ASSERT_EQ(cache.batches().size(), 2u) << "two pens, two batches";
    // The line, the circle's 64 segments and the quarter arc's 16.
    EXPECT_EQ(cache.batches()[0].vertices.size() / 4, 2u + 128u + 32u);
    EXPECT_EQ(cache.batches()[1].pen.color, 0xFFFF0000u);
    EXPECT_EQ(cache.batches()[1].vertices.size() / 4, 2u);
    ASSERT_EQ(cache.texts().size(), 1u);
    EXPECT_EQ(cache.texts()[0].text, "a note");

    // What is selected is drawn in its colour; a hidden layer is not drawn.
    selection.select(circle->id());
    cache.build(doc, selection, dof);
    ASSERT_EQ(cache.batches().size(), 3u);
    size_t orange = 0;
    for (const auto& batch : cache.batches()) {
        if (batch.pen.color == 0xFFFF9900) orange = batch.vertices.size() / 4;
    }
    EXPECT_EQ(orange, 128u) << "the circle, selected";
    doc.layerManager().getLayer("0")->visible = false;
    cache.build(doc, selection, dof);
    EXPECT_TRUE(cache.batches().empty());
    EXPECT_TRUE(cache.texts().empty());
}

// A frame draws only the chunks in view: of a drawing a hundred views
// across, a view of one corner draws a small part of it, and all that is in
// that corner. Seen whole, all of it; looking away, none.
TEST(RenderEfficiencyTest, OnlyTheChunksInViewAreDrawn) {
    hz::doc::Document doc;
    hz::render::SelectionManager selection;
    const hz::cstr::DOFAnalysis dof;
    for (int i = 0; i < 100; ++i) {
        for (int j = 0; j < 100; ++j) {
            doc.draftDocument().addEntity(
                lineFrom(i * 100.0, j * 100.0, i * 100.0 + 50, j * 100.0));
        }
    }
    DrawingCache cache;
    cache.build(doc, selection, dof);
    const size_t total = vertexCount(cache);
    ASSERT_EQ(total, 20000u);
    EXPECT_GT(cache.chunkBounds().size(), 1u);

    const auto [corner, points] = drawnThrough(cache, lookingAt(0, 0, 350));
    EXPECT_LT(corner, total / 20) << "a small part";
    for (int i = 0; i < 4; ++i) {
        for (int j = 0; j < 4; ++j) {
            EXPECT_TRUE(points.count({static_cast<float>(i * 100), static_cast<float>(j * 100)}))
                << "the line at " << i * 100 << ", " << j * 100 << " is in view";
        }
    }
    EXPECT_EQ(drawnThrough(cache, lookingAt(-100, -100, 10200)).first, total) << "all of it";

    // In perspective too: from above, and looking away.
    hz::render::Camera above;
    above.setPerspective(45.0, 1.0, 0.1, 10000.0);
    above.lookAt(Vec3(175, 175, 400), Vec3(175, 175, 0), Vec3(0, 1, 0));
    const auto [seen, abovePoints] = drawnThrough(cache, above);
    EXPECT_LT(seen, total / 20);
    EXPECT_TRUE(abovePoints.count({200.0f, 200.0f}));
    hz::render::Camera away = above;
    away.lookAt(Vec3(175, 175, 400), Vec3(175, 175, 800), Vec3(0, 1, 0));
    EXPECT_EQ(drawnThrough(cache, away).first, 0u) << "all of it behind";
}

// The text overlay is painted only when what it shows changes. It was
// painted, and sent to the GPU whole, on every frame, and a frame is drawn on
// every move of the cursor.
TEST(RenderEfficiencyTest, TheTextOverlayIsPaintedWhenWhatItShowsChanges) {
    hz::doc::Document doc;
    hz::render::SelectionManager selection;
    hz::ui::ViewportRenderer renderer;
    add(doc, std::make_shared<hz::draft::DraftText>(Vec2(10, 10), "a note"));
    auto camera = lookingAt(0, 0, 100);
    int width = 800;
    const auto frame = [&] {
        renderer.recomputeDOF(&doc);
        renderer.prepareEntities(doc, selection);
        renderer.prepareTextOverlay(camera, &doc, selection, width, 600, 0.125);
    };
    for (int i = 0; i < 5; ++i) frame();
    EXPECT_EQ(renderer.overlayPaints(), 1u) << "five frames, the cursor moving: one paint";

    camera = lookingAt(10, 0, 100);
    frame();
    frame();
    EXPECT_EQ(renderer.overlayPaints(), 2u) << "the view panned";
    add(doc, std::make_shared<hz::draft::DraftText>(Vec2(20, 10), "another"));
    frame();
    EXPECT_EQ(renderer.overlayPaints(), 3u) << "a text added";
    width = 1024;
    frame();
    EXPECT_EQ(renderer.overlayPaints(), 4u) << "the view resized";
    selection.select(doc.draftDocument().entities()[0]->id());
    frame();
    EXPECT_EQ(renderer.overlayPaints(), 5u) << "the selection, which annotations follow";
}

// A frame of a large drawing that has not changed costs next to nothing: it
// was built again, every vertex, on every frame. Built once, a frame only
// checks it still stands and picks the chunks in view.
TEST(RenderEfficiencyTest, AFrameOfALargeUnchangedDrawingIsCheap) {
    HZ_SKIP_WITHOUT_TIME_LIMITS();
    hz::doc::Document doc;
    hz::render::SelectionManager selection;
    const hz::cstr::DOFAnalysis dof;
    auto& drawing = doc.draftDocument();
    for (int i = 0; i < 100000; ++i) {
        const double x = (i % 400) * 25.0;
        const double y = (i / 400) * 25.0;
        drawing.addEntity(lineFrom(x, y, x + 20.0, y + 5.0));
        if (i % 5 == 0)
            drawing.addEntity(std::make_shared<hz::draft::DraftCircle>(Vec2(x, y), 4.0));
        if (i % 5 == 1) {
            drawing.addEntity(std::make_shared<hz::draft::DraftArc>(Vec2(x, y), 4.0, 0.0, 2.0));
        }
    }
    DrawingCache cache;
    cache.update(doc, selection, dof, 0);
    const auto camera = lookingAt(0, 0, 500);

    constexpr int kFrames = 100;
    size_t drawn = 0;
    const auto start = std::chrono::steady_clock::now();
    for (int frame = 0; frame < kFrames; ++frame) {
        cache.update(doc, selection, dof, 0);
        const auto visible = cache.visibleChunks(camera.viewProjectionMatrix());
        for (const auto& batch : cache.batches()) {
            for (const auto& range : DrawingCache::visibleRanges(batch, visible)) {
                drawn += range.count;
            }
        }
    }
    const double ms =
        std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - start)
            .count() /
        kFrames;
    EXPECT_EQ(cache.builds(), 1u);
    EXPECT_LT(drawn / kFrames, vertexCount(cache) / 50) << "a corner of it in view";
#ifdef NDEBUG
    EXPECT_LT(ms, 1.0) << "a frame's work, in ms";
#else
    EXPECT_LT(ms, 20.0) << "a frame's work, in ms";
#endif
}
