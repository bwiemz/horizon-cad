// Tools used one after another in any order (Phase 168): a seeded walk
// through the drafting tools, clicking where the drawing has something and
// where it has nothing, pressing Escape and Enter, undoing, redoing,
// deleting and copying between, as a user does in a long session. After
// every step the drawing holds only finite geometry; at the end it saves
// and loads back whole, and undoing everything leaves the drawing it began
// with.

#include <gtest/gtest.h>

#include <QAction>
#include <QCoreApplication>
#include <QKeySequence>
#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <memory>
#include <random>
#include <string>
#include <vector>

#include "../PortableRandom.h"
#include "UiTestSupport.h"
#include "horizon/document/Document.h"
#include "horizon/document/UndoStack.h"
#include "horizon/drafting/DraftArc.h"
#include "horizon/drafting/DraftCircle.h"
#include "horizon/drafting/DraftEllipse.h"
#include "horizon/drafting/DraftLine.h"
#include "horizon/drafting/DraftPolyline.h"
#include "horizon/drafting/DraftText.h"
#include "horizon/fileio/ImportReport.h"
#include "horizon/fileio/NativeFormat.h"
#include "horizon/ui/MainWindow.h"
#include "horizon/ui/ViewportWidget.h"

using hz::math::Vec2;
using hz::test::ModalCloser;
using hz::test::ToolDriver;
using hz::ui::MainWindow;

namespace {

/// The window's drafting tools: the ribbon's checkable tool actions.
std::vector<QAction*> draftingTools(MainWindow& w) {
    std::vector<QAction*> out;
    for (QAction* a : w.findChildren<QAction*>()) {
        if (a->objectName().startsWith(QStringLiteral("tool_")) && a->isEnabled()) {
            out.push_back(a);
        }
    }
    return out;
}

/// The action that answers @p keys, if one does.
QAction* actionFor(MainWindow& w, const QKeySequence& keys) {
    for (QAction* a : w.findChildren<QAction*>()) {
        if (a->shortcut() == keys && a->isEnabled()) return a;
    }
    return nullptr;
}

bool finite(const Vec2& p) {
    return std::isfinite(p.x) && std::isfinite(p.y);
}

/// Why @p entity holds a value that is not a finite number, if it does.
std::string notFinite(const hz::draft::DraftEntity& entity) {
    const auto box = entity.boundingBox();
    if (box.isValid()) {
        for (const auto& v : {box.min(), box.max()}) {
            if (!std::isfinite(v.x) || !std::isfinite(v.y) || !std::isfinite(v.z)) {
                return "its bounds";
            }
        }
    }
    if (const auto* line = dynamic_cast<const hz::draft::DraftLine*>(&entity)) {
        if (!finite(line->start()) || !finite(line->end())) return "a line's ends";
    } else if (const auto* circle = dynamic_cast<const hz::draft::DraftCircle*>(&entity)) {
        if (!finite(circle->center()) || !std::isfinite(circle->radius())) return "a circle";
    } else if (const auto* arc = dynamic_cast<const hz::draft::DraftArc*>(&entity)) {
        if (!finite(arc->center()) || !std::isfinite(arc->radius()) ||
            !std::isfinite(arc->startAngle()) || !std::isfinite(arc->endAngle())) {
            return "an arc";
        }
    } else if (const auto* poly = dynamic_cast<const hz::draft::DraftPolyline*>(&entity)) {
        for (const Vec2& p : poly->points()) {
            if (!finite(p)) return "a polyline's points";
        }
    } else if (const auto* text = dynamic_cast<const hz::draft::DraftText*>(&entity)) {
        if (!finite(text->position()) || !std::isfinite(text->textHeight()) ||
            !std::isfinite(text->rotation())) {
            return "a text";
        }
    } else if (const auto* ellipse = dynamic_cast<const hz::draft::DraftEllipse*>(&entity)) {
        if (!finite(ellipse->center()) || !std::isfinite(ellipse->semiMajor()) ||
            !std::isfinite(ellipse->semiMinor())) {
            return "an ellipse";
        }
    }
    return {};
}

/// A point to click at: on something the drawing has (an end of an entity,
/// its middle), near the origin, or far off, and now and then at a size
/// much larger or smaller than the rest.
Vec2 somewhere(std::mt19937& rng, const hz::draft::DraftDocument& drawing) {
    hz::test::Uniform unit(0.0, 1.0);
    hz::test::Uniform spread(-50.0, 50.0);
    const double pick = unit(rng);
    const auto& entities = drawing.entities();
    if (pick < 0.5 && !entities.empty()) {
        const size_t index =
            static_cast<size_t>(unit(rng) * static_cast<double>(entities.size())) % entities.size();
        const auto box = entities[index]->boundingBox();
        if (box.isValid()) {
            const double t = unit(rng);
            const auto& a = box.min();
            const auto& b = box.max();
            if (std::isfinite(a.x + a.y + b.x + b.y)) {
                return {a.x + (b.x - a.x) * t, a.y + (b.y - a.y) * (unit(rng) < 0.5 ? t : 1 - t)};
            }
        }
    }
    if (pick < 0.9) return {spread(rng), spread(rng)};
    if (pick < 0.95) return {spread(rng) * 1e-6, spread(rng) * 1e-6};
    return {spread(rng) * 1e5, spread(rng) * 1e5};
}

/// One seeded walk of @p steps through the tools, checked as it goes.
void walk(unsigned seed, int steps) {
    SCOPED_TRACE("seed " + std::to_string(seed));
    ModalCloser closer;  // every dialog a step opens is dismissed
    MainWindow w;
    ToolDriver drive(w);
    auto* doc = w.activeDocument();
    ASSERT_NE(doc, nullptr);
    // Something to work on: a line, a circle, an arc, a closed polyline and
    // a text, as a drawing opened has.
    {
        auto& drawing = doc->draftDocument();
        drawing.addEntity(std::make_shared<hz::draft::DraftLine>(Vec2(-20, -10), Vec2(20, -10)));
        drawing.addEntity(std::make_shared<hz::draft::DraftCircle>(Vec2(10, 10), 6.0));
        drawing.addEntity(std::make_shared<hz::draft::DraftArc>(Vec2(-10, 10), 5.0, 0.3, 2.8));
        drawing.addEntity(std::make_shared<hz::draft::DraftPolyline>(
            std::vector<Vec2>{{-30, -30}, {-15, -30}, {-15, -18}, {-30, -18}}, true));
        drawing.addEntity(std::make_shared<hz::draft::DraftText>(Vec2(0, 25), "Note", 2.5));
    }
    const size_t startEntities = doc->draftDocument().entities().size();
    const std::vector<QAction*> tools = draftingTools(w);
    ASSERT_GT(tools.size(), 10u) << "the drafting tools are on the ribbon";

    std::mt19937 rng(seed);
    hz::test::Uniform unit(0.0, 1.0);
    std::string last;
    for (int step = 0; step < steps; ++step) {
        const auto& drawing = doc->draftDocument();
        const double r = unit(rng);
        if (r < 0.18) {
            QAction* tool =
                tools[static_cast<size_t>(unit(rng) * static_cast<double>(tools.size())) %
                      tools.size()];
            last = "tool " + tool->objectName().toStdString();
            tool->trigger();
            // A few clicks with it, as a tool is used, before anything else.
            const int clicks = 1 + static_cast<int>(unit(rng) * 4.0);
            for (int c = 0; c < clicks; ++c) {
                const Vec2 at = somewhere(rng, doc->draftDocument());
                drive.move(at);
                drive.click(at);
            }
        } else if (r < 0.62) {
            const Vec2 at = somewhere(rng, drawing);
            last = "click " + std::to_string(at.x) + "," + std::to_string(at.y);
            drive.move(at);
            drive.click(at);
        } else if (r < 0.70) {
            const Vec2 from = somewhere(rng, drawing);
            const Vec2 to = somewhere(rng, drawing);
            last = "drag";
            drive.drag(from, to);
        } else if (r < 0.76) {
            last = "move";
            drive.move(somewhere(rng, drawing));
        } else if (r < 0.81) {
            last = "Enter";
            drive.key(Qt::Key_Return);
        } else if (r < 0.85) {
            last = "Escape";
            drive.key(Qt::Key_Escape);
        } else if (r < 0.89) {
            last = "Delete";
            drive.key(Qt::Key_Delete);
        } else if (r < 0.95) {
            last = "undo";
            if (QAction* undo = actionFor(w, QKeySequence::Undo)) undo->trigger();
        } else if (r < 0.98) {
            last = "redo";
            if (QAction* redo = actionFor(w, QKeySequence::Redo)) redo->trigger();
        } else {
            last = "copy and paste";
            if (QAction* copy = actionFor(w, QKeySequence::Copy)) copy->trigger();
            if (QAction* paste = actionFor(w, QKeySequence::Paste)) paste->trigger();
        }
        QCoreApplication::processEvents();
        ASSERT_EQ(w.activeDocument(), doc) << "step " << step << ": " << last;
        for (const auto& entity : doc->draftDocument().entities()) {
            ASSERT_NE(entity, nullptr) << "step " << step << ": " << last;
            const std::string bad = notFinite(*entity);
            ASSERT_TRUE(bad.empty()) << "step " << step << " (" << last << ") left " << bad
                                     << " that is not a finite number";
        }
    }
    drive.key(Qt::Key_Escape);
    drive.key(Qt::Key_Escape);
    QCoreApplication::processEvents();

    // Saved and loaded back: every entity, none left out.
    const auto path = std::filesystem::temp_directory_path() /
                      ("hz_tool_sequence_" + std::to_string(seed) + ".hcad");
    std::string error;
    ASSERT_TRUE(hz::io::NativeFormat::save(path.string(), *doc, &error)) << error;
    hz::doc::Document back;
    hz::io::ImportReport report;
    ASSERT_TRUE(hz::io::NativeFormat::load(path.string(), back, &error, &report)) << error;
    std::filesystem::remove(path);
    EXPECT_EQ(back.draftDocument().entities().size(), doc->draftDocument().entities().size());
    EXPECT_TRUE(report.skipped.empty()) << report.skipped.front();

    // Undone to the start, and redone to the end.
    const size_t endEntities = doc->draftDocument().entities().size();
    int undone = 0;
    while (doc->undoStack().canUndo() && undone < 100'000) {
        doc->undoStack().undo();
        ++undone;
    }
    EXPECT_EQ(doc->draftDocument().entities().size(), startEntities) << "undone to the start";
    while (doc->undoStack().canRedo() && undone-- > 0) doc->undoStack().redo();
    EXPECT_EQ(doc->draftDocument().entities().size(), endEntities) << "redone to the end";
}

}  // namespace

// Several walks, each from its own seed: a failure names the seed and the
// step, and the walk is the same each time it runs. HZ_TOOL_SEQUENCE_SEEDS
// and HZ_TOOL_SEQUENCE_STEPS take it further, for a long run by hand.
TEST(ToolSequenceTest, AnyOrderOfToolsLeavesASoundDrawing) {
    const auto setting = [](const char* name, unsigned fallback) {
        const char* value = std::getenv(name);
        const long parsed = value != nullptr ? std::strtol(value, nullptr, 10) : 0;
        return parsed > 0 ? static_cast<unsigned>(parsed) : fallback;
    };
    const unsigned seeds = setting("HZ_TOOL_SEQUENCE_SEEDS", 20);
    const int steps = static_cast<int>(setting("HZ_TOOL_SEQUENCE_STEPS", 500));
    for (unsigned seed = 1; seed <= seeds; ++seed) {
        walk(seed, steps);
        if (HasFatalFailure()) return;
    }
}
