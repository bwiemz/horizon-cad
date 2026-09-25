// Finding and seeing the part (Phase 135): the modelling commands in the Model
// menu (and so the command palette), Fit All that frames solids, a projection
// that survives a resize, display modes, a section plane and mass properties.

#include <gtest/gtest.h>

#include <QAction>
#include <QMenu>
#include <QMenuBar>
#include <QStatusBar>
#include <cmath>
#include <set>
#include <string>
#include <utility>

#include "UiTestSupport.h"
#include "horizon/document/Document.h"
#include "horizon/math/Vec4.h"
#include "horizon/render/Camera.h"
#include "horizon/render/DisplayMode.h"
#include "horizon/ui/MainWindow.h"
#include "horizon/ui/ViewportWidget.h"

using hz::math::Vec3;
using hz::test::DialogResponder;
using hz::test::FormAnswers;
using hz::test::FormFiller;
using hz::ui::MainWindow;

namespace {

void trigger(MainWindow& w, const char* name) {
    auto* action = w.findChild<QAction*>(QString::fromLatin1(name));
    ASSERT_NE(action, nullptr) << name;
    action->trigger();
}

void run(MainWindow& w, const char* command, const QString& title, FormAnswers answers) {
    FormFiller filler(title, std::move(answers));
    trigger(w, command);
    EXPECT_TRUE(filler.seen()) << "no \"" << title.toStdString() << "\" form";
}

void box(MainWindow& w) {
    run(w, "action_box", QStringLiteral("Box"),
        FormAnswers()
            .number(QStringLiteral("size0"), 10.0)
            .number(QStringLiteral("size1"), 10.0)
            .number(QStringLiteral("size2"), 10.0));
}

hz::ui::ViewportWidget& viewportOf(MainWindow& w) {
    return *w.findChild<hz::ui::ViewportWidget*>();
}

/// Whether every corner of the 10 mm box from the origin is in the view.
bool boxInView(const hz::render::Camera& camera) {
    for (int c = 0; c < 8; ++c) {
        const hz::math::Vec4 clip =
            camera.viewProjectionMatrix() *
            hz::math::Vec4((c & 1) ? 10.0 : 0.0, (c & 2) ? 10.0 : 0.0, (c & 4) ? 10.0 : 0.0, 1.0);
        if (clip.w <= 0.0 || std::abs(clip.x) > clip.w || std::abs(clip.y) > clip.w) return false;
    }
    return true;
}

/// Every action in a menu, submenus included, by object name.
void collect(const QMenu* menu, std::set<std::string>& names) {
    for (QAction* action : menu->actions()) {
        if (!action->objectName().isEmpty()) names.insert(action->objectName().toStdString());
        if (action->menu()) collect(action->menu(), names);
    }
}

}  // namespace

// Every modelling command is in the Model menu, which the Ctrl+K palette reads:
// they were on the ribbon only, out of the palette's reach.
TEST(SeeingTest, EveryModellingCommandIsInTheModelMenu) {
    MainWindow w;
    const QMenu* model = nullptr;
    for (QAction* top : w.menuBar()->actions()) {
        if (top->menu() && top->text().remove('&') == QStringLiteral("Model")) model = top->menu();
    }
    ASSERT_NE(model, nullptr);
    std::set<std::string> names;
    collect(model, names);
    for (const char* command : {"action_box",
                                "action_cylinder",
                                "action_sphere",
                                "action_cone",
                                "action_torus",
                                "action_extrude",
                                "action_revolve",
                                "action_boolean-union",
                                "action_boolean-subtract",
                                "action_boolean-intersect",
                                "action_fillet-3d",
                                "action_chamfer-3d",
                                "action_shell",
                                "action_draft",
                                "action_pattern-linear",
                                "action_pattern-circular",
                                "action_loft",
                                "action_sweep",
                                "action_datum_plane",
                                "action_sketch_xy",
                                "action_mass_properties"}) {
        EXPECT_TRUE(names.count(command)) << command << " is not in the Model menu";
    }
    // The common ones have a shortcut, and each command is one action (the
    // menu's is the ribbon's).
    EXPECT_FALSE(w.findChild<QAction*>(QStringLiteral("action_extrude"))->shortcut().isEmpty());
    EXPECT_EQ(w.findChildren<QAction*>(QStringLiteral("action_extrude")).size(), 1);
}

// Fit All frames the solid: it looked only at the drawing, so a part with no
// drawing was not framed at all.
TEST(SeeingTest, FitAllFramesTheSolid) {
    MainWindow w;
    box(w);
    auto& camera = viewportOf(w).camera();
    camera.lookAt(Vec3(500, 500, 500), Vec3(300, 300, 300), Vec3(0, 0, 1));
    trigger(w, "action_fit-all");
    EXPECT_NEAR((camera.target() - Vec3(5, 5, 5)).length(), 0.0, 1e-6) << "at the box's middle";
    EXPECT_LT((camera.eye() - camera.target()).length(), 200.0) << "and close enough to see it";
}

// Fit All frames the part in a window wider than tall, and narrower, in
// either projection; orthographic keeps the window's shape, so a circle stays
// round. It sized an orthographic view by the window's shape before the last
// resize, and a perspective one by its height alone.
TEST(SeeingTest, FitAllFramesThePartInAWideOrANarrowView) {
    MainWindow w;
    box(w);
    auto& view = viewportOf(w);
    const std::pair<int, int> shapes[] = {{1200, 400}, {400, 1200}};
    for (const bool ortho : {false, true}) {
        view.setOrthographic(ortho);
        for (const auto& shape : shapes) {
            view.applyProjection(shape.first, shape.second);  // a resize
            trigger(w, "action_fit-all");
            const auto& camera = view.camera();
            EXPECT_TRUE(boxInView(camera)) << (ortho ? "orthographic " : "perspective ")
                                           << shape.first << " x " << shape.second;
            if (ortho) {
                EXPECT_NEAR(camera.orthoWidth() / camera.orthoHeight(),
                            static_cast<double>(shape.first) / shape.second, 1e-9)
                    << "the window's shape";
            }
        }
    }
}

// Orthographic stays orthographic across a resize, which put every view back
// into perspective.
TEST(SeeingTest, OrthographicSurvivesAResize) {
    MainWindow w;
    auto& view = viewportOf(w);
    trigger(w, "action_view_ortho");
    EXPECT_TRUE(view.isOrthographic());
    EXPECT_EQ(view.camera().projectionType(), hz::render::ProjectionType::Orthographic);
    const double tall = view.camera().orthoHeight();
    view.applyProjection(1200, 600);  // what a resize does
    EXPECT_EQ(view.camera().projectionType(), hz::render::ProjectionType::Orthographic);
    EXPECT_NEAR(view.camera().orthoHeight(), tall, 1e-9) << "as tall, only wider";
    trigger(w, "action_view_ortho");
    EXPECT_EQ(view.camera().projectionType(), hz::render::ProjectionType::Perspective);
}

// The display mode and a section plane are chosen from the View menu.
TEST(SeeingTest, DisplayModesAndASectionPlane) {
    MainWindow w;
    auto& view = viewportOf(w);
    EXPECT_EQ(view.displayMode(), hz::render::DisplayMode::ShadedWithEdges);
    trigger(w, "action_display_wireframe");
    EXPECT_EQ(view.displayMode(), hz::render::DisplayMode::Wireframe);
    trigger(w, "action_display_shaded");
    EXPECT_EQ(view.displayMode(), hz::render::DisplayMode::Shaded);

    run(w, "action_section", QStringLiteral("Section Plane"),
        FormAnswers()
            .choose(QStringLiteral("axis"), QStringLiteral("Z"))
            .number(QStringLiteral("offset"), 4.0));
    ASSERT_TRUE(view.sectionPlane().has_value());
    // What is kept: below z = 4, so a point at z = 2 is kept and one at 6 cut.
    const auto& plane = *view.sectionPlane();
    const auto kept = [&plane](const Vec3& p) {
        return plane.x * p.x + plane.y * p.y + plane.z * p.z + plane.w >= 0.0;
    };
    EXPECT_TRUE(kept(Vec3(0, 0, 2)));
    EXPECT_FALSE(kept(Vec3(0, 0, 6)));
    trigger(w, "action_section_off");
    EXPECT_FALSE(view.sectionPlane().has_value());
}

// Mass properties of a 10 mm steel cube: 1000 mm3, 7.85 g, centred at 5, 5, 5.
TEST(SeeingTest, TheMassPropertiesOfABox) {
    MainWindow w;
    box(w);
    DialogResponder answer(QMessageBox::Ok, QStringLiteral("Mass Properties"));
    FormFiller filler(QStringLiteral("Mass Properties"),
                      FormAnswers().choose(QStringLiteral("material"), QStringLiteral("Steel")));
    trigger(w, "action_mass_properties");
    ASSERT_TRUE(filler.seen());
    ASSERT_TRUE(answer.seen());
    EXPECT_TRUE(answer.text().contains(QStringLiteral("Volume: 1000 ")))
        << answer.text().toStdString();
    EXPECT_TRUE(answer.text().contains(QStringLiteral("Mass: 7.85 g")))
        << answer.text().toStdString();
    EXPECT_TRUE(answer.text().contains(QStringLiteral("(5, 5, 5)"))) << answer.text().toStdString();
}
