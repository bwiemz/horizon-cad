#include <gtest/gtest.h>

#include <cmath>
#include <memory>
#include <string>
#include <vector>

#include "horizon/drafting/DraftCircle.h"
#include "horizon/drafting/DraftEntity.h"
#include "horizon/drafting/DraftLine.h"
#include "horizon/drafting/DraftText.h"
#include "horizon/fileio/GeometricToleranceRenderer.h"
#include "horizon/math/Constants.h"
#include "horizon/modeling/DrawingView.h"
#include "horizon/modeling/GeometricTolerance.h"
#include "horizon/modeling/Naming.h"
#include "horizon/modeling/PrimitiveFactory.h"
#include "horizon/topology/Solid.h"
#include "horizon/topology/TopologyID.h"

using hz::draft::DraftCircle;
using hz::draft::DraftEntity;
using hz::draft::DraftLine;
using hz::draft::DraftText;
using hz::io::GeometricToleranceRenderer;
using hz::math::Vec2;
using hz::model::DrawingGenerator;
using hz::model::DrawingView;
using hz::model::FeatureControlFrame;
using hz::model::Gdt;
using hz::model::GeometricCharacteristic;
using hz::model::MaterialCondition;
using hz::model::PrimitiveFactory;
using hz::model::StandardView;
using hz::topo::TopologyID;

namespace {

using Entities = std::vector<std::shared_ptr<DraftEntity>>;

constexpr double kCell = GeometricToleranceRenderer::kFrameHeight;
constexpr double kH = GeometricToleranceRenderer::kTextHeight;

std::string stated(double mm) {
    return Gdt::formatValue(mm);
}

Entities render(const DrawingView& view, const GeometricToleranceRenderer::Hangers& hangers = {}) {
    return GeometricToleranceRenderer::render(view, &stated, hangers);
}

std::vector<const DraftText*> textsOf(const Entities& entities) {
    std::vector<const DraftText*> texts;
    for (const auto& e : entities) {
        if (const auto* t = dynamic_cast<const DraftText*>(e.get())) texts.push_back(t);
    }
    return texts;
}

const DraftText* textSaying(const Entities& entities, const std::string& said) {
    for (const DraftText* t : textsOf(entities)) {
        if (t->text() == said) return t;
    }
    return nullptr;
}

bool near(const Vec2& a, const Vec2& b, double within = 1e-6) {
    return a.distanceTo(b) <= within;
}

/// A line of @p entities that ends at @p at.
bool lineEndsAt(const Entities& entities, const Vec2& at) {
    for (const auto& e : entities) {
        if (const auto* l = dynamic_cast<const DraftLine*>(e.get())) {
            if (near(l->start(), at) || near(l->end(), at)) return true;
        }
    }
    return false;
}

/// The longest straight edge the front view of a 40 x 30 x 20 box shows,
/// and its midpoint on the sheet.
struct Seen {
    DrawingView view;
    TopologyID edge;
    Vec2 middle;
};

Seen boxFront() {
    static const auto box = PrimitiveFactory::makeBox(40.0, 30.0, 20.0);
    Seen seen{DrawingGenerator::makeView(*box, StandardView::Front), {}, {}};
    double longest = 0.0;
    for (const auto& e : seen.view.edges) {
        const double length = e.a.distanceTo(e.b);
        if (e.visibility == hz::model::ProjectedEdge::Visibility::Visible && length > longest) {
            longest = length;
            seen.edge = e.sourceEdge;
            seen.middle = seen.view.toSheet((e.a + e.b) * 0.5);
        }
    }
    return seen;
}

FeatureControlFrame frameOf(GeometricCharacteristic c, const TopologyID& edge,
                            std::vector<std::string> datums = {"A"}) {
    FeatureControlFrame f;
    f.characteristic = c;
    f.tolerance = 0.05;
    f.datumRefs = std::move(datums);
    f.feature = edge;
    return f;
}

/// The view's sheet rectangle, as [low, high].
std::pair<Vec2, Vec2> rectOf(const DrawingView& view) {
    return {view.toSheet(view.boundsMin), view.toSheet(view.boundsMax)};
}

}  // namespace

// A frame is a row of boxes on a leader from the middle of its edge, outside
// the view: its tolerance and its datum written in their cells.
TEST(GeometricToleranceRendererTest, FrameIsBoxesOnALeaderFromItsEdge) {
    Seen seen = boxFront();
    ASSERT_TRUE(seen.edge.isValid());
    seen.view.tolerances.push_back(
        frameOf(GeometricCharacteristic::Perpendicularity, seen.edge, {"A"}));

    const Entities drawn = render(seen.view);
    const DraftText* tolerance = textSaying(drawn, "0.05");
    const DraftText* datum = textSaying(drawn, "A");
    ASSERT_NE(tolerance, nullptr);
    ASSERT_NE(datum, nullptr);
    EXPECT_EQ(textsOf(drawn).size(), 2u);  // its symbol is drawn, not written
    EXPECT_TRUE(lineEndsAt(drawn, seen.middle));

    // Both cells' text is out of the view, at one height, the datum after.
    const auto [low, high] = rectOf(seen.view);
    for (const DraftText* t : {tolerance, datum}) {
        const Vec2& p = t->position();
        EXPECT_FALSE(p.x > low.x && p.x < high.x && p.y > low.y && p.y < high.y) << t->text();
    }
    EXPECT_DOUBLE_EQ(tolerance->position().y, datum->position().y);
    EXPECT_GT(datum->position().x, tolerance->position().x);
}

// A frame or datum on an edge the view does not show draws nothing.
TEST(GeometricToleranceRendererTest, NothingForAnEdgeNotShown) {
    Seen seen = boxFront();
    const TopologyID missing = TopologyID::make("box", "edge999");
    seen.view.tolerances.push_back(frameOf(GeometricCharacteristic::Flatness, missing, {}));
    seen.view.datums.push_back({"A", missing});
    EXPECT_TRUE(render(seen.view).empty());
}

// Every characteristic's symbol is drawn in its cell, in lines, circles or
// arcs: none is written as a character a font or DXF reader may not have.
TEST(GeometricToleranceRendererTest, EverySymbolIsDrawnInItsCell) {
    for (int c = 0; c <= static_cast<int>(GeometricCharacteristic::Profile); ++c) {
        const auto characteristic = static_cast<GeometricCharacteristic>(c);
        Seen seen = boxFront();
        seen.view.tolerances.push_back(frameOf(characteristic, seen.edge, {}));
        const Entities drawn = render(seen.view);
        const DraftText* tolerance = textSaying(drawn, "0.05");
        ASSERT_NE(tolerance, nullptr) << Gdt::name(characteristic);
        // The symbol's cell: before the tolerance's, as high as the frame.
        const double left = tolerance->position().x - 1.0 - kCell;
        const double bottom = tolerance->position().y - (kCell - kH) / 2.0;
        int inside = 0;
        for (const auto& e : drawn) {
            const auto box = e->boundingBox();
            if (box.min().x > left + 0.1 && box.max().x < left + kCell - 0.1 &&
                box.min().y > bottom + 0.1 && box.max().y < bottom + kCell - 0.1) {
                EXPECT_EQ(dynamic_cast<const DraftText*>(e.get()), nullptr);
                ++inside;
            }
        }
        EXPECT_GE(inside, 1) << Gdt::name(characteristic);
    }
}

// A cylindrical zone writes ⌀ before the tolerance; MMC is an M in a circle
// after it.
TEST(GeometricToleranceRendererTest, DiameterZoneAndCircledModifier) {
    Seen seen = boxFront();
    FeatureControlFrame f = frameOf(GeometricCharacteristic::Position, seen.edge, {"A", "B"});
    f.tolerance = 0.2;
    f.diameterZone = true;
    f.modifier = MaterialCondition::MMC;
    seen.view.tolerances.push_back(f);
    const Entities drawn = render(seen.view);
    ASSERT_NE(textSaying(drawn,
                         "\xE2\x8C\x80"
                         "0.2"),
              nullptr);
    const DraftText* m = textSaying(drawn, "M");
    ASSERT_NE(m, nullptr);
    bool circled = false;
    for (const auto& e : drawn) {
        if (const auto* c = dynamic_cast<const DraftCircle*>(e.get())) {
            circled = circled || std::abs(c->center().x - m->position().x) < 1e-9;
        }
    }
    EXPECT_TRUE(circled);
    EXPECT_NE(textSaying(drawn, "B"), nullptr);
}

// Two frames on one edge share its leader, the second under the first.
TEST(GeometricToleranceRendererTest, FramesOnOneEdgeStack) {
    Seen seen = boxFront();
    FeatureControlFrame first = frameOf(GeometricCharacteristic::Flatness, seen.edge, {});
    FeatureControlFrame second = frameOf(GeometricCharacteristic::Parallelism, seen.edge, {"A"});
    second.tolerance = 0.1;
    seen.view.tolerances = {first, second};
    const Entities drawn = render(seen.view);
    const DraftText* top = textSaying(drawn, "0.05");
    const DraftText* under = textSaying(drawn, "0.1");
    ASSERT_NE(top, nullptr);
    ASSERT_NE(under, nullptr);
    EXPECT_NEAR(top->position().y - under->position().y, kCell, 1e-9);
}

// A datum feature symbol: a triangle on the edge, and its letter boxed at
// the end of a leader out of the view.
TEST(GeometricToleranceRendererTest, DatumIsATriangleOnItsEdgeAndABoxedLetter) {
    Seen seen = boxFront();
    seen.view.datums.push_back({"A", seen.edge});
    const Entities drawn = render(seen.view);
    const DraftText* letter = textSaying(drawn, "A");
    ASSERT_NE(letter, nullptr);
    EXPECT_EQ(letter->alignment(), hz::draft::TextAlignment::Center);
    // The triangle's base: a line about the edge's middle.
    bool based = false;
    for (const auto& e : drawn) {
        if (const auto* l = dynamic_cast<const DraftLine*>(e.get())) {
            based = based || near((l->start() + l->end()) * 0.5, seen.middle);
        }
    }
    EXPECT_TRUE(based);
    const auto [low, high] = rectOf(seen.view);
    const Vec2& p = letter->position();
    EXPECT_FALSE(p.x > low.x && p.x < high.x && p.y > low.y && p.y < high.y);
}

// On a hole seen end-on, frames leave the circle up and to the left, unless
// the hole is called out: then they hang under its callout, the hole's
// datum under them.
TEST(GeometricToleranceRendererTest, HoleFramesLeaveTheCircleOrHangUnderItsCallout) {
    // A hole's rim seen end-on, as a Hole feature names it: two pieces of a
    // circle of radius 5, each in chords.
    DrawingView top;
    top.boundsMin = {-10.0, -10.0};
    top.boundsMax = {10.0, 10.0};
    top.placement = {40.0, 40.0};
    constexpr int kChords = 24;
    for (int i = 0; i < kChords; ++i) {
        const double a0 = hz::math::kTwoPi * i / kChords;
        const double a1 = hz::math::kTwoPi * (i + 1) / kChords;
        hz::model::ProjectedEdge e;
        e.a = {5.0 * std::cos(a0), 5.0 * std::sin(a0)};
        e.b = {5.0 * std::cos(a1), 5.0 * std::sin(a1)};
        e.sourceEdge = TopologyID::fromTag(
            "edge:hole_1/wall|block/top/piece:" + std::to_string(i < kChords / 2 ? 0 : 1) +
            "/chord:" + std::to_string(i));
        top.edges.push_back(e);
    }
    const TopologyID rim = top.edges.back().sourceEdge;
    const double r = 5.0;
    top.tolerances.push_back(frameOf(GeometricCharacteristic::Position, rim, {"B"}));

    // On its own leader, from the circle at 135°.
    const Entities leader = render(top);
    ASSERT_NE(textSaying(leader, "0.05"), nullptr);
    const Vec2 centre = top.toSheet({0.0, 0.0});
    const Vec2 leaves{centre.x - r * std::sqrt(0.5), centre.y + r * std::sqrt(0.5)};
    for (const auto& e : top.edges)
        std::printf("DBG edge %s %f %f %f %f\n", e.sourceEdge.tag().c_str(), e.a.x, e.a.y, e.b.x,
                    e.b.y);

    // Under a callout: the frame's top-left where the callout leaves room.
    top.datums.push_back({"A", rim});
    const std::string key = hz::model::withoutPiece(hz::model::logicalEdge(rim.tag()));
    const Vec2 room{100.0, 50.0};
    const Entities hung = render(top, {{key, room}});
    const DraftText* tolerance = textSaying(hung, "0.05");
    ASSERT_NE(tolerance, nullptr);
    EXPECT_NEAR(tolerance->position().x, room.x + kCell + 1.0, 1e-9);
    EXPECT_NEAR(tolerance->position().y, room.y - kCell + (kCell - kH) / 2.0, 1e-9);
    EXPECT_FALSE(lineEndsAt(hung, leaves));
    const DraftText* datum = textSaying(hung, "A");
    ASSERT_NE(datum, nullptr);
    EXPECT_NEAR(datum->position().x, room.x + kCell / 2.0, 1e-9);
    EXPECT_LT(datum->position().y, room.y - kCell);
}
