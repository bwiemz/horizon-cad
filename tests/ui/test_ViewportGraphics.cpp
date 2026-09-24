// A viewport that cannot draw says so instead of staying silently blank.

#include <gtest/gtest.h>

#include <QCoreApplication>
#include <QElapsedTimer>
#include <QGuiApplication>
#include <QImage>
#include <QMessageBox>
#include <memory>

#include "UiTestSupport.h"
#include "horizon/document/Commands.h"
#include "horizon/document/Document.h"
#include "horizon/document/UndoStack.h"
#include "horizon/drafting/DraftLine.h"
#include "horizon/drafting/DraftText.h"
#include "horizon/render/Camera.h"
#include "horizon/ui/ViewportWidget.h"

using hz::test::DialogResponder;

TEST(ViewportGraphicsTest, AViewportThatCannotDrawTellsTheUser) {
    hz::doc::Document doc;
    hz::ui::ViewportWidget viewport;
    viewport.setDocument(&doc);
    viewport.resize(320, 240);

    DialogResponder responder(QMessageBox::Ok, QStringLiteral("Graphics Problem"), 5000);
    viewport.show();
    responder.waitForDialog(4000);

    if (viewport.isValid() && viewport.graphicsProblem().isEmpty()) {
        // A platform with working OpenGL (e.g. run under a real display).
        EXPECT_FALSE(responder.seen()) << "a working viewport is not reported";
    } else {
        // Qt's offscreen platform has no OpenGL: the viewport can never paint.
        EXPECT_TRUE(responder.seen()) << "a blank viewport must be explained";
        EXPECT_FALSE(viewport.graphicsProblem().isEmpty());
    }
}

namespace {

/// Whether the frame has a pixel for which @p is holds within three of @p at
/// (logical pixels).
template <typename Is>
bool near(const QImage& frame, const QPointF& at, Is is) {
    const double ratio = frame.devicePixelRatio();
    const int cx = qRound(at.x() * ratio);
    const int cy = qRound(at.y() * ratio);
    const int reach = qRound(3 * ratio);
    for (int y = cy - reach; y <= cy + reach; ++y) {
        for (int x = cx - reach; x <= cx + reach; ++x) {
            if (x < 0 || y < 0 || x >= frame.width() || y >= frame.height()) continue;
            if (is(frame.pixelColor(x, y))) return true;
        }
    }
    return false;
}

/// Magenta, over the dark background: the grid's axes are red and green.
bool magenta(const QColor& c) {
    return c.red() - c.green() > 50 && c.blue() - c.green() > 50;
}

/// A text's pixels, or a light line's.
bool light(const QColor& c) {
    return c.red() > 170 && c.green() > 170 && c.blue() > 170;
}

}  // namespace

// The 2D view on a real OpenGL (Phase 136): what the drawing cache holds is
// drawn where it is, over the grid, follows the view as it pans, and is
// drawn anew when the drawing changes, a line dragged in place included; a
// text is drawn where it is too. It needs a display: run with
// QT_QPA_PLATFORM=xcb (or wayland, windows). Qt's offscreen platform, which
// the tests use by default, has no OpenGL.
TEST(ViewportGraphicsTest, TheDrawingIsDrawnWhereItIsAndFollowsChanges) {
    if (QGuiApplication::platformName() == QStringLiteral("offscreen")) {
        GTEST_SKIP() << "no OpenGL on the offscreen platform: set QT_QPA_PLATFORM to run it";
    }
    using hz::math::Vec2;
    hz::doc::Document doc;
    auto line = std::make_shared<hz::draft::DraftLine>(Vec2(-100, 10), Vec2(100, 10));
    line->setColor(0xFFFF00FF);
    doc.undoStack().push(std::make_unique<hz::doc::AddEntityCommand>(doc.draftDocument(), line));
    auto text = std::make_shared<hz::draft::DraftText>(Vec2(-40, 40), "HHHHHHHH", 8.0);
    text->setAlignment(hz::draft::TextAlignment::Left);
    doc.undoStack().push(std::make_unique<hz::doc::AddEntityCommand>(doc.draftDocument(), text));
    // Far off, many: chunks the view does not draw until it looks there.
    for (int i = 0; i < 5000; ++i) {
        doc.draftDocument().addEntity(std::make_shared<hz::draft::DraftLine>(
            Vec2(5000 + (i % 100) * 10.0, (i / 100) * 10.0),
            Vec2(5005 + (i % 100) * 10.0, (i / 100) * 10.0)));
    }

    hz::ui::ViewportWidget viewport;
    viewport.setDocument(&doc);
    viewport.resize(400, 300);
    viewport.show();
    QElapsedTimer waited;
    waited.start();
    while (!viewport.isValid() && waited.elapsed() < 5000) QCoreApplication::processEvents();
    const auto look = [&viewport](double x, double y) {
        viewport.camera().lookAt(hz::math::Vec3(x, y, 150), hz::math::Vec3(x, y, 0),
                                 hz::math::Vec3(0, 1, 0));
    };
    look(0, 0);
    QImage frame = viewport.grabFramebuffer();
    if (!viewport.isValid() || !viewport.graphicsProblem().isEmpty()) {
        GTEST_SKIP() << "no OpenGL here: " << viewport.graphicsProblem().toStdString();
    }
    const auto at = [&viewport](double x, double y) { return viewport.worldToScreen(Vec2(x, y)); };
    EXPECT_TRUE(near(frame, at(50, 10), magenta)) << "on the line, over the grid";
    EXPECT_FALSE(near(frame, at(50, 25), magenta)) << "off it";
    EXPECT_TRUE(near(frame, at(-30, 42), light)) << "the text, where it is";
    EXPECT_FALSE(near(frame, at(-30, -42), light)) << "not mirrored across the view";

    look(0, 20);  // panned: the same line, lower on the screen
    frame = viewport.grabFramebuffer();
    EXPECT_TRUE(near(frame, at(50, 10), magenta)) << "panned";

    // Dragged in place, as a tool drags it.
    line->setStart(Vec2(-100, 30));
    line->setEnd(Vec2(100, 30));
    doc.draftDocument().updateEntityBounds(line->id());
    frame = viewport.grabFramebuffer();
    EXPECT_TRUE(near(frame, at(50, 30), magenta)) << "moved";
    EXPECT_FALSE(near(frame, at(50, 10), magenta)) << "not where it was";

    look(5052, 50);  // where the many are
    frame = viewport.grabFramebuffer();
    EXPECT_TRUE(near(frame, at(5052, 50), light)) << "a chunk not drawn before, now in view";
    viewport.hide();
}
