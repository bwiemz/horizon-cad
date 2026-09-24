// 2D drawing behaves the same at any zoom and leaves hidden and locked layers
// alone (Phase 110): snaps and picks reach a fixed distance on screen, and
// Trim cuts only at what is drawn and keeps the line type and group of what
// it cuts.

#include <gtest/gtest.h>

#include <QMouseEvent>
#include <cmath>
#include <memory>

#include "horizon/document/Document.h"
#include "horizon/drafting/DraftLine.h"
#include "horizon/drafting/Layer.h"
#include "horizon/ui/TrimTool.h"
#include "horizon/ui/ViewportWidget.h"

using hz::draft::DraftLine;
using hz::draft::SnapType;
using hz::math::Vec2;

namespace {

/// A viewport on `doc`, zoomed until one screen pixel spans at most
/// `worldPerPixel` (in), or at least it (out).
void zoomTo(hz::ui::ViewportWidget& viewport, double worldPerPixel, bool in) {
    for (int i = 0; i < 200; ++i) {
        const double now = viewport.pixelToWorldScale();
        if (in ? now <= worldPerPixel : now >= worldPerPixel) return;
        viewport.camera().zoom(in ? 0.8 : 1.25);  // a factor below 1 moves the eye closer
    }
}

bool click(hz::ui::Tool& tool, const Vec2& world) {
    QMouseEvent press(QEvent::MouseButtonPress, QPointF(0, 0), QPointF(0, 0), Qt::LeftButton,
                      Qt::LeftButton, Qt::NoModifier);
    return tool.mousePressEvent(&press, world);
}

struct Drawing {
    hz::doc::Document doc;
    hz::ui::ViewportWidget viewport;
    Drawing() {
        viewport.setDocument(&doc);
        viewport.resize(800, 600);
    }
    std::shared_ptr<DraftLine> line(const Vec2& a, const Vec2& b, const std::string& layer = "0") {
        auto l = std::make_shared<DraftLine>(a, b);
        l->setLayer(layer);
        doc.draftDocument().addEntity(l);
        return l;
    }
    void addLayer(const std::string& name, bool visible, bool locked) {
        hz::draft::LayerProperties props;
        props.name = name;
        props.visible = visible;
        props.locked = locked;
        doc.layerManager().addLayer(props);
    }
};

}  // namespace

TEST(Drafting2DTest, ASnapReachesTheSameDistanceOnScreenAtAnyZoom) {
    Drawing d;
    d.line(Vec2(0.3, 0.3), Vec2(100.3, 0.3));
    for (const bool in : {true, false}) {
        zoomTo(d.viewport, in ? 1e-4 : 1.0, in);
        const double reach = d.viewport.snapPixels() * d.viewport.pixelToWorldScale();
        const Vec2 end(0.3, 0.3);
        EXPECT_EQ(d.viewport.snap(end + Vec2(0, 0.5 * reach)).type, SnapType::Endpoint)
            << "half the reach away, zoomed " << (in ? "in" : "out");
        EXPECT_NE(d.viewport.snap(end + Vec2(0, 1.5 * reach)).type, SnapType::Endpoint)
            << "one and a half times the reach away, zoomed " << (in ? "in" : "out")
            << ": the old fixed 0.5 reached ten thousand pixels when zoomed in";
    }
}

TEST(Drafting2DTest, HiddenAndLockedLayersAreNotSnappedTo) {
    Drawing d;
    d.addLayer("Hidden", false, false);
    d.addLayer("Locked", true, true);
    d.line(Vec2(0.3, 0.3), Vec2(10.3, 0.3), "Hidden");
    d.line(Vec2(0.3, 5.3), Vec2(10.3, 5.3), "Locked");
    d.line(Vec2(0.3, 9.3), Vec2(10.3, 9.3));
    zoomTo(d.viewport, 0.01, true);
    EXPECT_NE(d.viewport.snap(Vec2(0.3, 0.3)).type, SnapType::Endpoint);
    EXPECT_NE(d.viewport.snap(Vec2(0.3, 5.3)).type, SnapType::Endpoint);
    EXPECT_EQ(d.viewport.snap(Vec2(0.3, 9.3)).type, SnapType::Endpoint) << "layer 0 is drawn";
}

TEST(Drafting2DTest, TrimCutsOnlyAtWhatIsDrawnAndKeepsLineTypeAndGroup) {
    Drawing d;
    d.addLayer("Construction", false, false);
    auto target = d.line(Vec2(0, 0), Vec2(10, 0));
    target->setLineType(2);  // dashed
    target->setGroupId(7);
    d.line(Vec2(5, -5), Vec2(5, 5), "Construction");
    zoomTo(d.viewport, 0.01, true);

    hz::ui::TrimTool trim;
    trim.activate(&d.viewport);
    click(trim, Vec2(8, 0));
    EXPECT_EQ(d.doc.draftDocument().entities().size(), 2u) << "a hidden line is not a cutting edge";

    d.doc.layerManager().getLayer("Construction")->visible = true;
    click(trim, Vec2(8, 0));
    const auto& entities = d.doc.draftDocument().entities();
    std::shared_ptr<DraftLine> piece;
    for (const auto& e : entities) {
        auto l = std::dynamic_pointer_cast<DraftLine>(e);
        if (l && l->layer() == "0") piece = l;
    }
    ASSERT_NE(piece, nullptr);
    EXPECT_TRUE((piece->end() - Vec2(5, 0)).length() < 1e-9 ||
                (piece->start() - Vec2(5, 0)).length() < 1e-9)
        << "cut at the edge";
    EXPECT_EQ(piece->lineType(), 2) << "still dashed";
    EXPECT_EQ(piece->groupId(), 7u) << "still in its group";
}

TEST(Drafting2DTest, APickReachesTheSameDistanceOnScreenAtAnyZoom) {
    Drawing d;
    d.line(Vec2(0, 0), Vec2(10, 0));
    d.line(Vec2(5, -5), Vec2(5, 5));
    // Zoomed far in, a click 1.5 pick-widths from the line misses it. The old
    // world floor of 0.15 reached thousands of pixels here and trimmed it.
    zoomTo(d.viewport, 5e-5, true);
    const double reach = d.viewport.pickTolerance();
    hz::ui::TrimTool trim;
    trim.activate(&d.viewport);
    EXPECT_FALSE(click(trim, Vec2(8, 1.5 * reach)));
    EXPECT_EQ(d.doc.draftDocument().entities().size(), 2u);
    EXPECT_TRUE(click(trim, Vec2(8, 0.5 * reach)));
    EXPECT_EQ(d.doc.draftDocument().entities().size(), 2u) << "the clicked part is cut away";
}

TEST(Drafting2DTest, AViewportWithNoSizeStillReachesSomething) {
    // Mid-layout a viewport can be 0 x 0, where the projection divides by
    // zero. A pick used to have a world floor to hide that; now the last good
    // scale stands in.
    Drawing d;
    const double sized = d.viewport.pixelToWorldScale();
    d.viewport.resize(0, 0);
    const double reach = d.viewport.pickTolerance();
    EXPECT_TRUE(std::isfinite(reach));
    EXPECT_GT(reach, 0.0);
    EXPECT_DOUBLE_EQ(d.viewport.pixelToWorldScale(), sized);
}
