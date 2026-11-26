// A viewport that cannot draw says so instead of staying silently blank.

#include <gtest/gtest.h>

#include <QAction>
#include <QCoreApplication>
#include <QElapsedTimer>
#include <QGuiApplication>
#include <QImage>
#include <QMessageBox>
#include <QTemporaryDir>
#include <cmath>
#include <memory>
#include <optional>
#include <string>
#include <utility>

#include "UiTestSupport.h"
#include "horizon/document/Commands.h"
#include "horizon/document/Document.h"
#include "horizon/document/FeatureTree.h"
#include "horizon/document/UndoStack.h"
#include "horizon/drafting/DraftLine.h"
#include "horizon/drafting/DraftText.h"
#include "horizon/fileio/NativeFormat.h"
#include "horizon/math/Constants.h"
#include "horizon/render/Camera.h"
#include "horizon/ui/MainWindow.h"
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
/// Over 150, not more: a one-pixel line that lies on the boundary between two
/// rows of pixels is drawn across both at part strength by some rasterizers
/// (Mesa's llvmpipe gives 166 each), where others fill one row. The grid
/// stays under 100.
bool light(const QColor& c) {
    return c.red() > 150 && c.green() > 150 && c.blue() > 150;
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

// A part's solid on a real OpenGL (Phase 138: meshes are shared, and the GPU
// keeps one buffer for each mesh, not each node): the box is drawn where it
// is, and again after the scene is built anew around the same mesh (a
// sketch opened and closed), which reuses what the GPU has.
TEST(ViewportGraphicsTest, TheSolidIsDrawnAndDrawnAgainFromTheSameMesh) {
    if (QGuiApplication::platformName() == QStringLiteral("offscreen")) {
        GTEST_SKIP() << "no OpenGL on the offscreen platform: set QT_QPA_PLATFORM to run it";
    }
    hz::ui::MainWindow w;
    w.resize(640, 480);
    w.show();
    {
        hz::test::FormFiller filler(QStringLiteral("Box"),
                                    hz::test::FormAnswers()
                                        .number(QStringLiteral("size0"), 10.0)
                                        .number(QStringLiteral("size1"), 10.0)
                                        .number(QStringLiteral("size2"), 10.0));
        w.findChild<QAction*>(QStringLiteral("action_box"))->trigger();
        ASSERT_TRUE(filler.seen());
    }
    auto& viewport = *w.findChild<hz::ui::ViewportWidget*>();
    QElapsedTimer waited;
    waited.start();
    while (!viewport.isValid() && waited.elapsed() < 5000) QCoreApplication::processEvents();
    viewport.camera().lookAt(hz::math::Vec3(40, -35, 45), hz::math::Vec3(5, 5, 5),
                             hz::math::Vec3(0, 0, 1));
    QImage frame = viewport.grabFramebuffer();
    if (!viewport.isValid() || !viewport.graphicsProblem().isEmpty()) {
        GTEST_SKIP() << "no OpenGL here";
    }
    // The solid is shaded blue-grey; the background and grid are near grey.
    const auto shaded = [](const QColor& c) { return c.blue() > 90 && c.blue() > c.red() + 15; };
    const QPointF middle = viewport.projectToScreen(hz::math::Vec3(5, 5, 5));
    EXPECT_TRUE(near(frame, middle, shaded)) << "the box, where it is";
    EXPECT_FALSE(near(frame, QPointF(8, viewport.height() / 2.0), shaded)) << "not everywhere";

    w.findChild<QAction*>(QStringLiteral("action_sketch_xy"))->trigger();
    w.findChild<QAction*>(QStringLiteral("action_sketch_finish"))->trigger();
    viewport.camera().lookAt(hz::math::Vec3(40, -35, 45), hz::math::Vec3(5, 5, 5),
                             hz::math::Vec3(0, 0, 1));
    frame = viewport.grabFramebuffer();
    EXPECT_TRUE(near(frame, viewport.projectToScreen(hz::math::Vec3(5, 5, 5)), shaded))
        << "drawn again from the same mesh";
}

namespace {

using hz::math::Vec3;

/// A cylinder primitive of radius 10 and height 20 about +z from the origin, cut in 6 round its
/// axis (its corners at 0, 60, ... degrees): Stable, so exact, drawn round, and its faceted copy a
/// hexagonal prism. With @p box, a box of that size standing at that corner beside it, a body of
/// its own. Opened in @p w, shown and built; its side's name in @p side.
void openCoarseCylinder(hz::ui::MainWindow& w, const QTemporaryDir& dir, std::string& side,
                        const std::optional<std::pair<Vec3, Vec3>>& box = std::nullopt) {
    hz::doc::Document coarse;
    coarse.setType(hz::doc::DocumentType::Part);
    auto made = hz::doc::PrimitiveFeature::makeCylinder(10.0, 20.0);
    ASSERT_TRUE(made->setParameter("segments", 6.0));
    side = made->featureID() + "/side";
    coarse.featureTree().addFeature(std::move(made));
    if (box) {
        auto beside = hz::doc::PrimitiveFeature::makeBox(box->first.x, box->first.y, box->first.z);
        beside->setOperation(hz::doc::BodyOperation::NewBody);
        ASSERT_TRUE(beside->setVector("basePoint", box->second));
        coarse.featureTree().addFeature(std::move(beside));
    }
    const QString path = dir.filePath(QStringLiteral("coarse.hzpart"));
    std::string error;
    ASSERT_TRUE(hz::io::NativeFormat::save(path.toStdString(), coarse, &error)) << error;
    w.resize(1400, 900);
    w.show();
    ASSERT_TRUE(w.openPath(path));
    QElapsedTimer waited;
    waited.start();
    while (w.backgroundWorkRunning() && waited.elapsed() < 30'000) {
        QCoreApplication::processEvents(QEventLoop::AllEvents, 20);
    }
    ASSERT_NE(w.activeDocument()->exactSolid(), nullptr) << w.activeDocument()->exactNote();
    auto& viewport = *w.findChild<hz::ui::ViewportWidget*>();
    waited.restart();
    while (!viewport.isValid() && waited.elapsed() < 5000) QCoreApplication::processEvents();
}

/// The colour drawn at @p world in @p frame.
QColor colourAt(const QImage& frame, const hz::ui::ViewportWidget& viewport, const Vec3& world) {
    const QPointF at = viewport.projectToScreen(world);
    const double ratio = frame.devicePixelRatio();
    return frame.pixelColor(qRound(at.x() * ratio), qRound(at.y() * ratio));
}

/// The hover's colour (0.35, 0.75, 1.0) at 0.3 over the part's blue-grey: bluer and lighter.
bool highlighted(const QColor& before, const QColor& after) {
    return after.blue() + after.green() > before.blue() + before.green() + 20;
}

std::string colours(const QColor& before, const QColor& after) {
    return "unhovered " + before.name().toStdString() + ", hovered " + after.name().toStdString();
}

}  // namespace

// Owner amendment A1 (Task 10) on a real OpenGL: an exact part's face is highlighted from its
// faceted copy, whose flat facets lie inside the round side drawn (by r (1 - cos 30 degrees) for
// a cylinder cut in 6), and so behind it; they are drawn where the face drawn is seen (a stencil
// of it, Ruling 39), whatever their depth, and are seen. Looked at square on one facet's middle,
// where it lies deepest, the hovered side is lighter and bluer there than it is unhovered.
TEST(ViewportGraphicsTest, AnExactPartsFacetHighlightIsSeenOverItsRoundSide) {
    if (QGuiApplication::platformName() == QStringLiteral("offscreen")) {
        GTEST_SKIP() << "no OpenGL on the offscreen platform: set QT_QPA_PLATFORM to run it";
    }
    QTemporaryDir dir;
    ASSERT_TRUE(dir.isValid());
    hz::ui::MainWindow w;
    std::string side;
    openCoarseCylinder(w, dir, side);
    if (HasFatalFailure()) return;
    auto& viewport = *w.findChild<hz::ui::ViewportWidget*>();
    // Square on the facet between the corners at 0 and 60 degrees: its middle, at 30 degrees, lies
    // r cos 30 from the axis, the round side drawn in front of it at r.
    const double at = hz::math::kPi / 6.0;
    const Vec3 middle(8.66 * std::cos(at), 8.66 * std::sin(at), 10.0);
    viewport.camera().lookAt(Vec3(60 * std::cos(at), 60 * std::sin(at), 10.0), middle,
                             Vec3(0, 0, 1));
    const QImage plain = viewport.grabFramebuffer();
    if (!viewport.isValid() || !viewport.graphicsProblem().isEmpty()) {
        GTEST_SKIP() << "no OpenGL here";
    }
    viewport.setModelHover(hz::ui::ViewportWidget::ModelPick{0, side, false});
    const QImage hovered = viewport.grabFramebuffer();
    const QColor before = colourAt(plain, viewport, middle);
    const QColor after = colourAt(hovered, viewport, middle);
    EXPECT_TRUE(highlighted(before, after)) << colours(before, after);
}

// Ruling 39: what stands in front of the face hides its highlight there, however near it stands. A
// thin box 0.3 to 1 in front of the round side, across the middle of the facet facing +y: seen
// from +y, the box is drawn as it is, unhovered or hovered; the facet beside it is highlighted.
TEST(ViewportGraphicsTest, AnExactPartsFacetHighlightIsHiddenBehindWhatStandsInFront) {
    if (QGuiApplication::platformName() == QStringLiteral("offscreen")) {
        GTEST_SKIP() << "no OpenGL on the offscreen platform: set QT_QPA_PLATFORM to run it";
    }
    QTemporaryDir dir;
    ASSERT_TRUE(dir.isValid());
    hz::ui::MainWindow w;
    std::string side;
    openCoarseCylinder(w, dir, side, std::pair{Vec3(3.0, 0.7, 3.0), Vec3(-1.5, 10.3, 8.5)});
    if (HasFatalFailure()) return;
    auto& viewport = *w.findChild<hz::ui::ViewportWidget*>();
    // The facet between the corners at 60 and 120 degrees: in the plane y = r cos 30.
    const Vec3 middle(0.0, 8.66, 10.0);
    viewport.camera().lookAt(Vec3(0.0, 60.0, 10.0), middle, Vec3(0, 0, 1));
    const QImage plain = viewport.grabFramebuffer();
    if (!viewport.isValid() || !viewport.graphicsProblem().isEmpty()) {
        GTEST_SKIP() << "no OpenGL here";
    }
    viewport.setModelHover(hz::ui::ViewportWidget::ModelPick{0, side, false});
    const QImage hovered = viewport.grabFramebuffer();
    const QColor onTheBox = colourAt(plain, viewport, middle);
    const QColor onTheBoxHovered = colourAt(hovered, viewport, middle);
    EXPECT_EQ(onTheBox, onTheBoxHovered)
        << "the box in front: " << colours(onTheBox, onTheBoxHovered);
    const Vec3 beside(-3.5, 8.66, 15.0);
    const QColor before = colourAt(plain, viewport, beside);
    const QColor after = colourAt(hovered, viewport, beside);
    EXPECT_TRUE(highlighted(before, after)) << "the facet beside it: " << colours(before, after);
}

// Ruling 39: a facet seen almost edge on is highlighted wherever the round side over it is seen,
// however far in front of it the side stands along the line of sight. Seen from 110 degrees round
// the axis, the facet between the corners at 0 and 60 degrees, at a third of its way, lies some 5
// behind the round side along the line of sight, and is highlighted.
TEST(ViewportGraphicsTest, AnExactPartsFacetHighlightIsSeenAtAGrazingAngle) {
    if (QGuiApplication::platformName() == QStringLiteral("offscreen")) {
        GTEST_SKIP() << "no OpenGL on the offscreen platform: set QT_QPA_PLATFORM to run it";
    }
    QTemporaryDir dir;
    ASSERT_TRUE(dir.isValid());
    hz::ui::MainWindow w;
    std::string side;
    openCoarseCylinder(w, dir, side);
    if (HasFatalFailure()) return;
    auto& viewport = *w.findChild<hz::ui::ViewportWidget*>();
    const double from = 110.0 * hz::math::kPi / 180.0;
    viewport.camera().lookAt(Vec3(60 * std::cos(from), 60 * std::sin(from), 10.0), Vec3(0, 0, 10.0),
                             Vec3(0, 0, 1));
    const QImage plain = viewport.grabFramebuffer();
    if (!viewport.isValid() || !viewport.graphicsProblem().isEmpty()) {
        GTEST_SKIP() << "no OpenGL here";
    }
    viewport.setModelHover(hz::ui::ViewportWidget::ModelPick{0, side, false});
    const QImage hovered = viewport.grabFramebuffer();
    const Vec3 a(10.0, 0.0, 10.0);
    const Vec3 b(5.0, 8.66, 10.0);
    const Vec3 onTheFacet = a + (b - a) * 0.35;
    const QColor before = colourAt(plain, viewport, onTheFacet);
    const QColor after = colourAt(hovered, viewport, onTheFacet);
    EXPECT_TRUE(highlighted(before, after)) << colours(before, after);
}
