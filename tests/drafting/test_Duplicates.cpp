// Copies of entities, found to be removed (Edit > Delete Duplicates, as
// AutoCAD's OVERKILL): an entity drawn twice, or pasted on itself, which a
// drawing from elsewhere often holds, and which breaks the closed outline
// a sketch's profile needs.

#include <gtest/gtest.h>

#include <chrono>
#include <cmath>
#include <memory>
#include <numbers>
#include <vector>

#include "horizon/drafting/DraftArc.h"
#include "horizon/drafting/DraftCircle.h"
#include "horizon/drafting/DraftEllipse.h"
#include "horizon/drafting/DraftLine.h"
#include "horizon/drafting/DraftPolyline.h"
#include "horizon/drafting/DraftRectangle.h"
#include "horizon/drafting/DraftText.h"
#include "horizon/drafting/Duplicates.h"

using hz::draft::DraftEntity;
using hz::draft::duplicateEntities;
using hz::math::Vec2;

namespace {

template <typename T, typename... Args>
std::shared_ptr<DraftEntity> make(Args&&... args) {
    return std::make_shared<T>(std::forward<Args>(args)...);
}

}  // namespace

// Each kind found again however it was drawn, the first kept: a line drawn
// from its other end, an open polyline backward, an arc whose angles are a
// turn apart, an ellipse turned half round, a text over itself.
TEST(DuplicatesTest, EachKindIsFoundHoweverItWasDrawn) {
    using namespace hz::draft;
    const double pi = std::numbers::pi;
    std::vector<std::shared_ptr<DraftEntity>> all = {
        make<DraftLine>(Vec2(0, 0), Vec2(10, 5)),
        make<DraftLine>(Vec2(10, 5), Vec2(0, 0)),  // the other way: a copy
        make<DraftCircle>(Vec2(3, 3), 2.0),
        make<DraftCircle>(Vec2(3, 3), 2.0),  // a copy
        make<DraftCircle>(Vec2(3, 3), 2.5),  // another size: not
        make<DraftArc>(Vec2(0, 0), 4.0, 0.5, 2.0),
        make<DraftArc>(Vec2(0, 0), 4.0, 0.5 + 2 * pi, 2.0 + 2 * pi),  // the same arc
        make<DraftArc>(Vec2(0, 0), 4.0, 2.0, 0.5),                    // the rest of the circle: not
        make<DraftPolyline>(std::vector<Vec2>{{0, 0}, {1, 0}, {1, 1}}, false),
        make<DraftPolyline>(std::vector<Vec2>{{1, 1}, {1, 0}, {0, 0}}, false),  // backward
        make<DraftPolyline>(std::vector<Vec2>{{0, 0}, {1, 0}, {1, 1}}, true),   // closed: not
        make<DraftRectangle>(Vec2(0, 0), Vec2(4, 2)),
        make<DraftRectangle>(Vec2(0, 0), Vec2(4, 2)),  // a copy
        make<DraftEllipse>(Vec2(5, 5), 3.0, 1.0, 0.3),
        make<DraftEllipse>(Vec2(5, 5), 3.0, 1.0, 0.3 + pi),  // half a turn: the same
        make<DraftText>(Vec2(1, 1), "Note", 2.5),
        make<DraftText>(Vec2(1, 1), "Note", 2.5),   // a copy
        make<DraftText>(Vec2(1, 1), "Other", 2.5),  // other words: not
    };
    const auto copies = duplicateEntities(all, 1e-9);
    const std::vector<uint64_t> expected = {all[1]->id(), all[3]->id(),  all[6]->id(),
                                            all[9]->id(), all[12]->id(), all[14]->id(),
                                            all[16]->id()};
    EXPECT_EQ(copies, expected);
}

// The same shape on another layer, in another colour or line type, is not a
// copy: taking it out would change the drawing.
TEST(DuplicatesTest, ACopyIsTheSameInEveryProperty) {
    auto a = make<hz::draft::DraftLine>(Vec2(0, 0), Vec2(1, 0));
    auto onOther = make<hz::draft::DraftLine>(Vec2(0, 0), Vec2(1, 0));
    onOther->setLayer("Other");
    auto red = make<hz::draft::DraftLine>(Vec2(0, 0), Vec2(1, 0));
    red->setColor(0xFFFF0000u);
    auto dashed = make<hz::draft::DraftLine>(Vec2(0, 0), Vec2(1, 0));
    dashed->setLineType(2);
    auto moved = make<hz::draft::DraftLine>(Vec2(0, 0), Vec2(1, 1e-6));
    EXPECT_TRUE(duplicateEntities({a, onOther, red, dashed, moved}, 1e-9).empty());
    // Within the tolerance, it is.
    auto near = make<hz::draft::DraftLine>(Vec2(0, 0), Vec2(1, 1e-12));
    EXPECT_EQ(duplicateEntities({a, near}, 1e-9), std::vector<uint64_t>{near->id()});
}

// A drawing of 100,000 lines, every other one drawn twice: found in time
// in the count, not its square.
TEST(DuplicatesTest, ManyAreFoundQuickly) {
    std::vector<std::shared_ptr<DraftEntity>> all;
    std::vector<uint64_t> expected;
    for (int i = 0; i < 50'000; ++i) {
        all.push_back(make<hz::draft::DraftLine>(Vec2(i, 0), Vec2(i, 1)));
        all.push_back(make<hz::draft::DraftLine>(Vec2(i, i % 2 == 0 ? 1 : 2), Vec2(i, 0)));
        if (i % 2 == 0) expected.push_back(all.back()->id());
    }
    const auto start = std::chrono::steady_clock::now();
    EXPECT_EQ(duplicateEntities(all, 1e-9), expected);
    const double seconds =
        std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count();
    EXPECT_LT(seconds, 30.0) << "a generous bound for any build";
}
